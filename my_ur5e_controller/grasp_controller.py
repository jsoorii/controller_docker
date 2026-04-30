"""
grasp_controller.py
-------------------
Notion 제어기 구조 명세(2024-05-28) + MIT Biomimetics Reflexive Control 기반
파지 제어기(GraspController) 구현.

ref: https://biomimetics.mit.edu/research/reflexive-control

계층 구조:
    High-level Planner  (< 1 Hz)   : GraspPlanner
        파지 타입 선택, 물체 포즈, reflex 활성화 여부 지정
    Low-level Reflex Layer (300+ Hz): ReflexCoordinator
        센서 기반 빠른 반응 — 계획 최적화보다 실행 가능성 우선
        AntiSlipReflex  : 슬립 감지 → 수직력 증가
        ReGraspReflex   : 비대칭 접촉 → antipodal 재정렬 (< 150 ms)

RT 태스크 (최고 우선순위, rt7):
    controller_run(state, command)
    GraspController
        - FourBarLinkage  : 4절 링크 메커니즘 (FK / IK)
        - FingerKinematics: 손가락 FK, 6×3 자코비안
        - ReflexCoordinator: reflex 우선순위 관리

기본 태스크 (보통 우선순위):
    - joint / current / vel / force 명령 수신
    - joint state / 접촉 센서 데이터 수신
    - 응답 패킷 송신

제어 상태머신:
    WAIT_STABLE → INIT_CONTROL → CHECK_MOTOR → INIT_POSITION → RUN_CONTROL

처리 흐름 (RUN_CONTROL 루프):
    1. joint state 읽기
    2. 손가락 FK 계산
    3. kinematics / 자코비안 계산
    4. ReflexCoordinator.update() — 우선순위 reflex 선택
    5. 토크 명령 출력

타이밍:
    제어루프 2ms (500 Hz), 모터 응답 폴링 (miss 카운트)

저장 목록:
    MotorState   : q, qdot, current
    MotorCommand : q, qdot, current
    FingerState  : p, R, J (손가락당)
    FingerCommand: p_target, force_target, q_target
    ContactState : force_3axis(3,), contact_pos_2d(2,), in_contact
    ReflexState  : IDLE / TRIGGERED / EXECUTING / RESOLVED
"""

import abc
import threading
import time
from dataclasses import dataclass, field
from enum import Enum
from typing import List, Optional

import numpy as np


# ---------------------------------------------------------------------------
# 상태머신 열거형
# ---------------------------------------------------------------------------

class GraspState(Enum):
    WAIT_STABLE   = 0   # 안정화 대기
    INIT_CONTROL  = 1   # SRAM 메모리 초기화
    CHECK_MOTOR   = 2   # 모터 초기 연결 확인
    INIT_POSITION = 3   # 물리 홈 포지션 탐색 → 초기위치 저장
    RUN_CONTROL   = 4   # 상태읽기 → 실측값 업데이트 → 제어계산 → 전류명령 송신


class ReflexState(Enum):
    """개별 Reflex 상태머신."""
    IDLE       = 0   # 비활성
    TRIGGERED  = 1   # 조건 감지, 다음 스텝부터 실행
    EXECUTING  = 2   # 실행 중
    RESOLVED   = 3   # 완료 → 다음 스텝에서 IDLE 전이


# ---------------------------------------------------------------------------
# 데이터 구조체
# ---------------------------------------------------------------------------

@dataclass
class MotorState:
    """모터 상태: q(rad), qdot(rad/s), current(A)."""
    q:       np.ndarray = field(default_factory=lambda: np.zeros(3))
    qdot:    np.ndarray = field(default_factory=lambda: np.zeros(3))
    current: np.ndarray = field(default_factory=lambda: np.zeros(3))


@dataclass
class MotorCommand:
    """모터 명령: q(rad), qdot(rad/s), current(A)."""
    q:       np.ndarray = field(default_factory=lambda: np.zeros(3))
    qdot:    np.ndarray = field(default_factory=lambda: np.zeros(3))
    current: np.ndarray = field(default_factory=lambda: np.zeros(3))


@dataclass
class FingerState:
    """손가락 FK 결과.

    p : 끝점 위치  (3,)
    R : 회전행렬  (3, 3)
    J : 6×3 자코비안  (6, 3) — 상단 3행: 선속도, 하단 3행: 각속도
    """
    p: np.ndarray = field(default_factory=lambda: np.zeros(3))
    R: np.ndarray = field(default_factory=lambda: np.eye(3))
    J: np.ndarray = field(default_factory=lambda: np.zeros((6, 3)))


@dataclass
class FingerCommand:
    """손가락 목표 명령."""
    p_target:     np.ndarray = field(default_factory=lambda: np.zeros(3))
    force_target: float = 0.0
    q_target:     np.ndarray = field(default_factory=lambda: np.zeros(3))


@dataclass
class ContactState:
    """손가락 끝 접촉 센서 상태.

    MIT Reflexive Control의 bimodal force sensor 출력을 모델링.

    force_3axis : 센서 로컬 프레임에서 [fx, fy, fz] (N).
                  fz = 법선력(normal force), fx/fy = 전단력(shear force)
    contact_pos : 접촉 위치 (2D, 센서 면 상의 [u, v] — 단위화, -1~1).
    in_contact  : 접촉 여부 (|fz| > contact_threshold)
    """
    force_3axis: np.ndarray = field(default_factory=lambda: np.zeros(3))
    contact_pos: np.ndarray = field(default_factory=lambda: np.zeros(2))
    in_contact:  bool = False


# ---------------------------------------------------------------------------
# Reflex 추상 기반 클래스
# ---------------------------------------------------------------------------

class Reflex(abc.ABC):
    """MIT Reflexive Control 단일 Reflex 기반 클래스.

    각 Reflex는 독립적인 상태머신을 가진다:
        IDLE → TRIGGERED → EXECUTING → RESOLVED → IDLE

    Parameters
    ----------
    priority  : 낮을수록 높은 우선순위 (0 = 최고).
    timeout_s : EXECUTING 상태 최대 허용 시간(s). 초과 시 강제 RESOLVED.
    """

    def __init__(self, priority: int = 10, timeout_s: float = 0.5):
        self.priority   = priority
        self.timeout_s  = timeout_s
        self.state      = ReflexState.IDLE
        self._exec_t0: float = 0.0

    # --- 공개 API (ReflexCoordinator 호출) ---

    def update(self,
               finger_states: list,
               contact_states: list,
               motor_states: list,
               dt: float) -> Optional[dict]:
        """한 스텝 업데이트. 토크 override를 반환하거나 None.

        Returns
        -------
        override : dict {"torque": ndarray (n_fingers, n_joints)} or None
        """
        if self.state == ReflexState.IDLE:
            if self.check(finger_states, contact_states, motor_states):
                self.state  = ReflexState.TRIGGERED
                self._exec_t0 = time.perf_counter()

        elif self.state == ReflexState.TRIGGERED:
            self.state = ReflexState.EXECUTING

        elif self.state == ReflexState.EXECUTING:
            # 타임아웃 감시
            if time.perf_counter() - self._exec_t0 > self.timeout_s:
                self.state = ReflexState.RESOLVED
                return None
            result = self.execute(finger_states, contact_states, motor_states, dt)
            if result is None:
                self.state = ReflexState.RESOLVED
            return result

        elif self.state == ReflexState.RESOLVED:
            self.state = ReflexState.IDLE

        return None

    def is_active(self) -> bool:
        return self.state == ReflexState.EXECUTING

    # --- 하위 클래스에서 구현 ---

    @abc.abstractmethod
    def check(self, finger_states: list, contact_states: list,
              motor_states: list) -> bool:
        """Reflex 발동 조건 확인. True면 TRIGGERED."""
        ...

    @abc.abstractmethod
    def execute(self, finger_states: list, contact_states: list,
                motor_states: list, dt: float) -> Optional[dict]:
        """Reflex 실행 한 스텝.
        완료 시 None 반환 → RESOLVED 전이.
        진행 중이면 {"torque": ndarray} 반환.
        """
        ...


# ---------------------------------------------------------------------------
# Anti-Slip Reflex
# ---------------------------------------------------------------------------

class AntiSlipReflex(Reflex):
    """슬립 감지 → 수직력(법선력) 증가 Reflex.

    슬립 조건 (Coulomb 마찰 근사):
        slip_ratio = |F_tangential| / (mu * F_normal) > slip_threshold

    실행:
        수직력이 부족한 손가락의 q_target을 Δq_step씩 닫는 방향으로 이동.
        모든 손가락이 slip_threshold * safety_factor 이하가 되면 완료.

    Parameters
    ----------
    mu              : 마찰 계수 (기본 0.5 — 고무/플라스틱)
    slip_threshold  : 슬립 발동 임계 비율 (기본 0.85)
    safety_factor   : 목표 마찰 여유 비율 (기본 0.60)
    f_normal_min    : 접촉 인식 최소 법선력 (N, 기본 0.1)
    dq_step         : 한 스텝 관절 닫기 증분 (rad, 기본 0.01)
    """

    def __init__(self, n_fingers: int = 3, n_joints: int = 3,
                 mu: float = 0.5, slip_threshold: float = 0.85,
                 safety_factor: float = 0.60, f_normal_min: float = 0.1,
                 dq_step: float = 0.01, **kwargs):
        super().__init__(priority=0, timeout_s=0.3, **kwargs)
        self.n_fingers      = n_fingers
        self.n_joints       = n_joints
        self.mu             = mu
        self.slip_threshold = slip_threshold
        self.safety_factor  = safety_factor
        self.f_normal_min   = f_normal_min
        self.dq_step        = dq_step

        # 실행 중 수정할 q_target 내부 버퍼
        self._q_target      = np.zeros((n_fingers, n_joints))
        self._exec_started  = False   # execute() 첫 진입 여부

    def _slip_ratio(self, contact: ContactState) -> float:
        """슬립 비율 계산. 접촉 없으면 0 반환."""
        fn = contact.force_3axis[2]          # 법선력
        if fn < self.f_normal_min:
            return 0.0
        ft = np.linalg.norm(contact.force_3axis[:2])  # 전단력 크기
        return ft / (self.mu * fn + 1e-9)

    def check(self, finger_states, contact_states, motor_states) -> bool:
        self._exec_started = False   # IDLE에서 호출 → 다음 실행을 위해 리셋
        for cs in contact_states:
            if cs.in_contact and self._slip_ratio(cs) > self.slip_threshold:
                return True
        return False

    def execute(self, finger_states, contact_states, motor_states,
                dt: float) -> Optional[dict]:
        # execute() 첫 진입 시 현재 q를 버퍼로 복사 (state는 이미 EXECUTING)
        if not self._exec_started:
            for fi in range(self.n_fingers):
                self._q_target[fi] = motor_states[fi].q.copy()
            self._exec_started = True

        n_joints = self.n_joints
        torque   = np.zeros((self.n_fingers, n_joints))
        all_safe = True

        for fi, (cs, ms) in enumerate(zip(contact_states, motor_states)):
            if not cs.in_contact:
                continue
            ratio = self._slip_ratio(cs)
            if ratio > self.safety_factor:
                all_safe = False
                # 관절을 닫는 방향으로 증분 (관절 0번: 주 굽힘 관절)
                self._q_target[fi, 0] += self.dq_step

            # PD 토크로 q_target 추종
            err  = self._q_target[fi] - ms.q
            derr = -ms.qdot
            torque[fi] = 200.0 * err + 20.0 * derr

        if all_safe:
            return None  # 완료 → RESOLVED
        return {"torque": torque}


# ---------------------------------------------------------------------------
# Re-Grasp Reflex
# ---------------------------------------------------------------------------

class ReGraspReflex(Reflex):
    """비대칭 접촉 감지 → antipodal 재정렬 Reflex (< 150 ms 목표).

    MIT 논문의 re-grasping reflex:
        - 각 손가락 쌍의 contact_pos를 비교해 대칭도(symmetry score) 계산
        - 비대칭 (score < sym_threshold) → 손가락 위치 조정

    대칭도 계산:
        손가락 쌍 (fi, fj)의 접촉 위치를 로봇 베이스 프레임으로 변환 후
        연결 벡터와 법선 방향 사이 정렬 오차를 측정.

        단순화 버전 (2D sensor 기준):
            sym_score = 1 - |contact_pos_fi[0] + contact_pos_fj[0]| / 2
            (u축 기준 대칭: 두 손가락의 u 값 합이 0에 가까울수록 대칭)

    실행:
        비대칭 손가락 쌍에 대해 더 작은 u를 가진 손가락을 +Δq, 큰 쪽을 -Δq.
        sym_score > sym_threshold가 되거나 timeout이면 완료.

    Parameters
    ----------
    sym_threshold  : 대칭 판정 임계값 (기본 0.85, 0~1)
    dq_step        : 한 스텝 관절 이동 증분 (rad, 기본 0.005)
    finger_pairs   : 비교할 손가락 쌍 인덱스 리스트 [(0,1), (0,2), ...]
    """

    def __init__(self, n_fingers: int = 3, n_joints: int = 3,
                 sym_threshold: float = 0.85, dq_step: float = 0.005,
                 finger_pairs: list = None, **kwargs):
        super().__init__(priority=1, timeout_s=0.15, **kwargs)  # 150 ms 타임아웃
        self.n_fingers     = n_fingers
        self.n_joints      = n_joints
        self.sym_threshold = sym_threshold
        self.dq_step       = dq_step
        self.finger_pairs  = finger_pairs or self._default_pairs(n_fingers)
        self._q_target     = np.zeros((n_fingers, n_joints))
        self._exec_started = False   # execute() 첫 진입 여부

    @staticmethod
    def _default_pairs(n: int) -> list:
        """인접 손가락 쌍 생성: [(0,1), (1,2), ...]"""
        return [(i, i + 1) for i in range(n - 1)]

    def _sym_score(self, ci: ContactState, cj: ContactState) -> float:
        """두 손가락 접촉 위치의 대칭도 (0~1, 1=완벽 대칭)."""
        if not (ci.in_contact and cj.in_contact):
            return 1.0  # 접촉 없으면 대칭 판정 스킵
        u_sum = abs(ci.contact_pos[0] + cj.contact_pos[0])
        return 1.0 - min(u_sum / 2.0, 1.0)

    def check(self, finger_states, contact_states, motor_states) -> bool:
        self._exec_started = False   # IDLE에서 호출 → 다음 실행을 위해 리셋
        for fi, fj in self.finger_pairs:
            if self._sym_score(contact_states[fi], contact_states[fj]) < self.sym_threshold:
                return True
        return False

    def execute(self, finger_states, contact_states, motor_states,
                dt: float) -> Optional[dict]:
        # execute() 첫 진입 시 현재 q를 버퍼로 복사 (state는 이미 EXECUTING)
        if not self._exec_started:
            for fi in range(self.n_fingers):
                self._q_target[fi] = motor_states[fi].q.copy()
            self._exec_started = True

        torque   = np.zeros((self.n_fingers, self.n_joints))
        all_sym  = True

        for fi, fj in self.finger_pairs:
            score = self._sym_score(contact_states[fi], contact_states[fj])
            if score < self.sym_threshold:
                all_sym = False
                ci, cj = contact_states[fi], contact_states[fj]
                # u > 0이면 접촉이 한쪽으로 쏠림 → 반대 방향으로 조정
                if ci.contact_pos[0] > cj.contact_pos[0]:
                    self._q_target[fi, 0] += self.dq_step
                    self._q_target[fj, 0] -= self.dq_step
                else:
                    self._q_target[fi, 0] -= self.dq_step
                    self._q_target[fj, 0] += self.dq_step

        for fi, ms in enumerate(motor_states):
            err  = self._q_target[fi] - ms.q
            derr = -ms.qdot
            torque[fi] = 200.0 * err + 20.0 * derr

        if all_sym:
            return None  # 완료
        return {"torque": torque}


# ---------------------------------------------------------------------------
# Reflex Coordinator (Low-level Reflex Layer, 300+ Hz)
# ---------------------------------------------------------------------------

class ReflexCoordinator:
    """우선순위 기반 Reflex 관리자.

    RT 루프에서 매 스텝 호출되며, 활성화된 reflex 중 가장 높은 우선순위
    (priority 값이 가장 낮은) 것의 토크 override를 반환한다.

    동작:
        1. 모든 reflex.update() 호출 (상태머신 전진)
        2. EXECUTING 상태인 reflex 중 priority 최소값 선택
        3. 선택된 reflex의 토크 override 반환
        4. 활성 reflex 없으면 None 반환 → 기본 PD 토크 사용

    Parameters
    ----------
    reflexes : Reflex 인스턴스 리스트. priority 기준 자동 정렬.
    """

    def __init__(self, reflexes: List[Reflex] = None):
        self._reflexes: List[Reflex] = sorted(
            reflexes or [], key=lambda r: r.priority
        )

    def add(self, reflex: Reflex):
        self._reflexes.append(reflex)
        self._reflexes.sort(key=lambda r: r.priority)

    def update(self, finger_states: list, contact_states: list,
               motor_states: list, dt: float) -> Optional[dict]:
        """모든 reflex 업데이트 후 최우선 override 반환."""
        overrides = []
        for reflex in self._reflexes:
            result = reflex.update(finger_states, contact_states, motor_states, dt)
            if result is not None:
                overrides.append((reflex.priority, result))

        if not overrides:
            return None
        # 가장 낮은 priority 값(= 가장 높은 우선순위) 선택
        overrides.sort(key=lambda x: x[0])
        return overrides[0][1]

    def active_reflexes(self) -> List[str]:
        """현재 EXECUTING 상태인 reflex 이름 목록."""
        return [type(r).__name__ for r in self._reflexes if r.is_active()]


# ---------------------------------------------------------------------------
# High-level Grasp Planner (< 1 Hz)
# ---------------------------------------------------------------------------

class GraspPlanner:
    """High-level 파지 계획자 (MIT Reflexive Control 상위 레이어).

    역할:
        - 파지 타입(POWER / PINCH / LATERAL) 선택
        - 물체 포즈 기반 초기 q_target 생성
        - Reflex 활성화/비활성화 지령
        - GraspController에 명령 주입

    GraspController와 분리된 스레드 (또는 저주기 태스크)에서 실행.
    RT 루프 간섭 없이 _cmd_buf에만 접근.

    Parameters
    ----------
    controller  : GraspController 인스턴스
    update_hz   : 계획 주기 (기본 1 Hz)
    """

    class GraspType(Enum):
        POWER   = "power"    # 전체 손가락 감싸기
        PINCH   = "pinch"    # 엄지-검지 집기
        LATERAL = "lateral"  # 측면 집기

    def __init__(self, controller, update_hz: float = 1.0):
        self._ctrl      = controller
        self.dt         = 1.0 / update_hz
        self._running   = False
        self._grasp_type = GraspPlanner.GraspType.POWER

        # 파지 타입별 q_target 프리셋 (손가락 3개 × 관절 3개, 예시값)
        self._presets: dict = {
            GraspPlanner.GraspType.POWER:   np.deg2rad([[60, 40, 30]] * 3),
            GraspPlanner.GraspType.PINCH:   np.deg2rad([[0, 60, 45], [0, 60, 45], [0, 0, 0]]),
            GraspPlanner.GraspType.LATERAL: np.deg2rad([[30, 20, 15]] * 3),
        }

    def set_grasp_type(self, gtype: "GraspPlanner.GraspType"):
        self._grasp_type = gtype

    def set_preset(self, gtype: "GraspPlanner.GraspType", q_deg: np.ndarray):
        """파지 타입별 q_target 프리셋을 설정 (단위: degree → 내부 radian 변환)."""
        self._presets[gtype] = np.deg2rad(q_deg)

    def plan_once(self):
        """한 번 계획 실행: 현재 grasp_type에 맞는 q_target을 controller에 주입."""
        q_target = self._presets[self._grasp_type]
        for fi in range(self._ctrl.n_fingers):
            self._ctrl.write_command(fi, q=q_target[fi])

    def start(self):
        """블로킹 계획 루프 (별도 스레드에서 실행 권장)."""
        self._running = True
        while self._running:
            t0 = time.perf_counter()
            self.plan_once()
            elapsed = time.perf_counter() - t0
            sleep_t = self.dt - elapsed
            if sleep_t > 0.0:
                time.sleep(sleep_t)

    def stop(self):
        self._running = False

    def start_thread(self) -> threading.Thread:
        t = threading.Thread(target=self.start, daemon=True)
        t.start()
        return t


# ---------------------------------------------------------------------------
# 4절 링크 메커니즘 (Freudenstein 방정식)
# ---------------------------------------------------------------------------

class FourBarLinkage:
    """크랭크-록커 4절 링크 메커니즘.

    링크 길이 (미터):
        L1 : 고정 링크 (지지 프레임)
        L2 : 입력 링크 (크랭크, 모터 구동)
        L3 : 커플러 링크
        L4 : 출력 링크 (피동, 손가락 근위부와 연결)

    부호 규약: 모든 각도 라디안, CCW 양(+).
    """

    def __init__(self, L1: float = 0.040, L2: float = 0.015,
                 L3: float = 0.035, L4: float = 0.025):
        self.L1 = L1
        self.L2 = L2
        self.L3 = L3
        self.L4 = L4

    # --- FK: 모터 각도 θ2 → 출력 링크 각도 θ4 ----------------------------

    def fk(self, theta2: float) -> float:
        """Freudenstein 방정식으로 출력 각도 θ4를 구한다.

        K1·cos(θ4) − K2·cos(θ2) + K3 = cos(θ2 − θ4)

        반고정 절반(t = tan(θ4/2)) 치환 후 이차방정식 풀이.
        """
        L1, L2, L3, L4 = self.L1, self.L2, self.L3, self.L4
        K1 = L1 / L4
        K2 = L1 / L2
        K3 = (L2**2 - L3**2 + L4**2 + L1**2) / (2.0 * L2 * L4)

        c2 = np.cos(theta2)
        s2 = np.sin(theta2)

        A =  c2 - K1 - K2 * c2 + K3
        B = -2.0 * s2
        C =  K1 - (K2 + 1.0) * c2 + K3

        disc = B * B - 4.0 * A * C
        if disc < 0.0:
            disc = 0.0

        denom = 2.0 * A
        if abs(denom) < 1e-12:
            return 0.0

        t = (-B - np.sqrt(disc)) / denom
        return 2.0 * np.arctan(t)

    # --- 속도비: dθ4/dθ2 (수치 미분) -------------------------------------

    def velocity_ratio(self, theta2: float, eps: float = 1e-5) -> float:
        """출력/입력 속도비 dθ4/dθ2."""
        return (self.fk(theta2 + eps) - self.fk(theta2 - eps)) / (2.0 * eps)

    # --- IK: 목표 θ4 → 입력 각도 θ2 (이분법) ----------------------------

    def ik(self, theta4_target: float,
           search_lo: float = -np.pi, search_hi: float = np.pi,
           tol: float = 1e-6, max_iter: int = 50) -> float:
        """이분법(bisection)으로 역기구학을 푼다. scipy 불필요."""
        def residual(t2: float) -> float:
            return self.fk(t2) - theta4_target

        # 부호 변환 구간을 N=200 샘플로 탐색
        N = 200
        angles = np.linspace(search_lo, search_hi, N)
        a, b = None, None
        for i in range(N - 1):
            fa, fb = residual(angles[i]), residual(angles[i + 1])
            if fa * fb <= 0.0:
                a, b = angles[i], angles[i + 1]
                break

        if a is None:
            return 0.0  # 해 없음 — 기본값 반환

        for _ in range(max_iter):
            mid = 0.5 * (a + b)
            if (b - a) < tol:
                break
            if residual(a) * residual(mid) <= 0.0:
                b = mid
            else:
                a = mid

        return 0.5 * (a + b)


# ---------------------------------------------------------------------------
# 손가락 기구학 (평면 3R 체인)
# ---------------------------------------------------------------------------

class FingerKinematics:
    """평면 3R 손가락 기구학.

    손가락이 로컬 xy-평면(z축이 법선) 내에서 움직인다고 가정한다.
    각 관절은 z축을 회전축으로 사용.

    파라미터:
        link_lengths : [l1, l2, l3] — 각 링크 길이(m)
        base_R       : 손가락 기저부의 로봇 기준 좌표계 상 회전행렬 (3×3)
        base_p       : 손가락 기저부의 로봇 기준 좌표계 상 위치 (3,)
    """

    def __init__(self, link_lengths: list = None,
                 base_R: np.ndarray = None,
                 base_p: np.ndarray = None):
        self.link_lengths = np.asarray(link_lengths or [0.04, 0.03, 0.025])
        self.base_R = base_R if base_R is not None else np.eye(3)
        self.base_p = base_p if base_p is not None else np.zeros(3)

    # --- FK: q(3,) → FingerState ----------------------------------------

    def fk(self, q: np.ndarray) -> FingerState:
        """손가락 FK 계산.

        Returns
        -------
        FingerState
            p : 끝점 위치  (3,)  [로봇 기준 좌표계]
            R : 회전행렬  (3, 3)
            J : 6×3 자코비안 (선속도 3행 + 각속도 3행)
        """
        l = self.link_lengths
        q1, q2, q3 = float(q[0]), float(q[1]), float(q[2])

        # 누적 각도
        a1 = q1
        a2 = q1 + q2
        a3 = q1 + q2 + q3

        # 로컬 좌표계(손가락 평면)에서 관절 위치
        p0 = np.zeros(3)                                       # 기저부
        p1 = p0 + np.array([l[0] * np.cos(a1),
                             l[0] * np.sin(a1), 0.0])          # 1번 관절 후
        p2 = p1 + np.array([l[1] * np.cos(a2),
                             l[1] * np.sin(a2), 0.0])          # 2번 관절 후
        p3 = p2 + np.array([l[2] * np.cos(a3),
                             l[2] * np.sin(a3), 0.0])          # 끝점

        # 끝점 회전행렬 (z 회전 a3)
        R_local = np.array([
            [ np.cos(a3), -np.sin(a3), 0.0],
            [ np.sin(a3),  np.cos(a3), 0.0],
            [        0.0,         0.0, 1.0],
        ])

        # 로봇 기준으로 변환
        p_world = self.base_R @ p3 + self.base_p
        R_world = self.base_R @ R_local

        # --- 자코비안 계산 (6×3) ---
        # 회전축 z (기저부 좌표계에서 고정)
        z_axis = self.base_R @ np.array([0.0, 0.0, 1.0])

        # 각 관절에서 끝점까지 벡터 (로봇 기준)
        p_e = p_world
        p_j = [
            self.base_p,
            self.base_R @ p1 + self.base_p,
            self.base_R @ p2 + self.base_p,
        ]

        J = np.zeros((6, 3))
        for i in range(3):
            r = p_e - p_j[i]                    # 관절 → 끝점 벡터
            J[:3, i] = np.cross(z_axis, r)      # 선속도 기여
            J[3:, i] = z_axis                   # 각속도 기여 (모두 동일 축)

        state = FingerState()
        state.p = p_world
        state.R = R_world
        state.J = J
        return state

    # --- 자코비안 전치 토크: F → τ (힘 → 관절 토크) ---------------------

    def force_to_torque(self, q: np.ndarray, force_world: np.ndarray) -> np.ndarray:
        """끝점에 가해지는 힘 force_world(6,) → 관절 토크 τ(3,).

        Parameters
        ----------
        force_world : (6,) — [fx, fy, fz, mx, my, mz]
        """
        fk_state = self.fk(q)
        return fk_state.J.T @ force_world  # (3, 6) @ (6,) = (3,)


# ---------------------------------------------------------------------------
# 파지 제어기 메인 클래스
# ---------------------------------------------------------------------------

class GraspController:
    """파지 제어기 (Notion 명세 구현).

    Parameters
    ----------
    n_fingers    : 손가락 수
    n_joints     : 손가락당 관절 수 (기본 3)
    motor_iface  : 모터 인터페이스 객체 (선택). None이면 시뮬레이션 모드.
                   구현 필요 메서드: read_state() -> (q, qdot, current)
                                    write_command(q, qdot, current)
    loop_hz      : 제어루프 주기 (기본 500 Hz = 2ms)
    poll_max     : 모터 폴링 최대 카운트 (초과 시 miss 처리)
    """

    LOOP_HZ  = 500           # 제어루프 주기 (Hz) → 2ms
    POLL_MAX = 20            # 폴링 최대 카운트

    def __init__(self, n_fingers: int = 3, n_joints: int = 3,
                 motor_iface=None, loop_hz: int = LOOP_HZ,
                 poll_max: int = POLL_MAX,
                 reflexes: List[Reflex] = None):
        """
        Parameters
        ----------
        reflexes : Reflex 인스턴스 리스트. None이면 기본 reflex 2개 자동 생성.
                   빈 리스트([])를 넘기면 reflex 없이 순수 PD 제어.
        """

        self.n_fingers = n_fingers
        self.n_joints  = n_joints
        self.loop_hz   = loop_hz
        self.dt        = 1.0 / loop_hz
        self.poll_max  = poll_max

        self.state     = GraspState.WAIT_STABLE
        self._lock     = threading.Lock()
        self._running  = False
        self.miss_count = 0

        # 모터 인터페이스 (None = 시뮬 모드)
        self._motor_iface = motor_iface

        # ---------- 저장 목록 ----------
        self.motor_state   = [MotorState(
            q=np.zeros(n_joints), qdot=np.zeros(n_joints), current=np.zeros(n_joints)
        ) for _ in range(n_fingers)]

        self.motor_cmd     = [MotorCommand(
            q=np.zeros(n_joints), qdot=np.zeros(n_joints), current=np.zeros(n_joints)
        ) for _ in range(n_fingers)]

        self.finger_state  = [FingerState() for _ in range(n_fingers)]
        self.finger_cmd    = [FingerCommand() for _ in range(n_fingers)]

        # 접촉 센서 상태 (손가락당)
        self.contact_state = [ContactState() for _ in range(n_fingers)]

        # ---------- 기구학 객체 ----------
        self._linkage  = [FourBarLinkage() for _ in range(n_fingers)]
        self._fk_model = [FingerKinematics() for _ in range(n_fingers)]

        # ---------- 홈 포지션 ----------
        self._home_q = np.zeros((n_fingers, n_joints))

        # ---------- PD 게인 ----------
        self.kp = np.full(n_joints, 200.0)
        self.kd = np.full(n_joints, 20.0)

        # ---------- Reflex Layer (Low-level, 300+ Hz) ----------
        if reflexes is None:
            reflexes = [
                AntiSlipReflex(n_fingers=n_fingers, n_joints=n_joints),
                ReGraspReflex(n_fingers=n_fingers, n_joints=n_joints),
            ]
        self._reflex_coord = ReflexCoordinator(reflexes)

        # ---------- 외부 명령 버퍼 ----------
        self._cmd_buf: dict = {
            "q_target":     np.zeros((n_fingers, n_joints)),
            "vel_target":   np.zeros((n_fingers, n_joints)),
            "cur_target":   np.zeros((n_fingers, n_joints)),
            "force_target": np.zeros(n_fingers),
        }
        # read: joint pos/vel/current (외부에서 읽어 가는 용도)
        self._state_buf: dict = {
            "q":              np.zeros((n_fingers, n_joints)),
            "qdot":           np.zeros((n_fingers, n_joints)),
            "current":        np.zeros((n_fingers, n_joints)),
            "active_reflexes": [],
        }

    # -----------------------------------------------------------------------
    # 공개 통신 API (기본 태스크)
    # -----------------------------------------------------------------------

    def write_command(self, finger_idx: int,
                      q: np.ndarray = None,
                      qdot: np.ndarray = None,
                      current: np.ndarray = None,
                      force: float = None):
        """joint pos / vel / current / 목표힘 명령 수신 (write)."""
        with self._lock:
            if q       is not None:
                self._cmd_buf["q_target"][finger_idx]   = np.asarray(q)
            if qdot    is not None:
                self._cmd_buf["vel_target"][finger_idx] = np.asarray(qdot)
            if current is not None:
                self._cmd_buf["cur_target"][finger_idx] = np.asarray(current)
            if force   is not None:
                self._cmd_buf["force_target"][finger_idx] = float(force)

    def read_state(self, finger_idx: int) -> dict:
        """joint pos / vel / current 상태 읽기 (read) → 응답 패킷."""
        with self._lock:
            return {
                "q":       self._state_buf["q"][finger_idx].copy(),
                "qdot":    self._state_buf["qdot"][finger_idx].copy(),
                "current": self._state_buf["current"][finger_idx].copy(),
                "finger_state": {
                    "p": self.finger_state[finger_idx].p.copy(),
                    "R": self.finger_state[finger_idx].R.copy(),
                },
            }

    def update_contact(self, finger_idx: int,
                       force_3axis: np.ndarray,
                       contact_pos: np.ndarray,
                       contact_threshold: float = 0.1):
        """접촉 센서 데이터 업데이트 (기본 태스크에서 호출).

        Parameters
        ----------
        force_3axis       : [fx, fy, fz] (N), fz = 법선력
        contact_pos       : [u, v] 센서 면 접촉 위치 (-1~1)
        contact_threshold : 접촉 판정 최소 법선력 (N)
        """
        with self._lock:
            cs = self.contact_state[finger_idx]
            cs.force_3axis = np.asarray(force_3axis, dtype=float)
            cs.contact_pos = np.asarray(contact_pos, dtype=float)
            cs.in_contact  = float(force_3axis[2]) >= contact_threshold

    def read_all_states(self) -> dict:
        """모든 손가락 상태 읽기 (active reflex 정보 포함)."""
        with self._lock:
            return {
                "q":              self._state_buf["q"].copy(),
                "qdot":           self._state_buf["qdot"].copy(),
                "current":        self._state_buf["current"].copy(),
                "grasp_state":    self.state.name,
                "miss_count":     self.miss_count,
                "active_reflexes": list(self._state_buf["active_reflexes"]),
            }

    # -----------------------------------------------------------------------
    # RT 제어 태스크 (최고 우선순위)
    # -----------------------------------------------------------------------

    def controller_run(self, state: GraspState, command: dict) -> np.ndarray:
        """RT 제어 함수 (rt7 태스크 진입점).

        Parameters
        ----------
        state   : 현재 제어 상태 (GraspState)
        command : 명령 딕셔너리 {"q_target", "vel_target", "force_target"}

        Returns
        -------
        torque : (n_fingers, n_joints) 토크 명령
        """
        torque = np.zeros((self.n_fingers, self.n_joints))

        if state != GraspState.RUN_CONTROL:
            return torque

        for fi in range(self.n_fingers):
            q    = self.motor_state[fi].q
            qdot = self.motor_state[fi].qdot

            q_tgt = command.get("q_target",     self._home_q)[fi]
            f_tgt = command.get("force_target", np.zeros(self.n_fingers))[fi]

            # FK 기반 위치 오차 없을 때 → PD 관절 토크
            err  = q_tgt - q
            derr = -qdot          # 목표속도 0 기준

            tau_pd = self.kp * err + self.kd * derr

            # 목표힘이 있을 때 → 자코비안 전치로 토크 추가
            if abs(f_tgt) > 1e-6:
                force_vec = np.array([f_tgt, 0.0, 0.0, 0.0, 0.0, 0.0])
                tau_force = self.finger_state[fi].J.T @ force_vec
                torque[fi] = tau_pd + tau_force
            else:
                torque[fi] = tau_pd

        return torque

    # -----------------------------------------------------------------------
    # 처리 흐름 (RUN_CONTROL 내부)
    # -----------------------------------------------------------------------

    def _run_control_step(self, command: dict):
        """RUN_CONTROL 한 스텝: 읽기 → FK → Reflex → 토크 출력."""

        # 1. joint state 읽기
        self._step_read_joint_state()

        # 2 & 3. 손가락 FK / 자코비안 계산
        for fi in range(self.n_fingers):
            q_motor = self.motor_state[fi].q
            q_proximal = self._linkage[fi].fk(q_motor[0])
            q_finger   = np.array([q_proximal, q_motor[1], q_motor[2]])
            self.finger_state[fi] = self._fk_model[fi].fk(q_finger)

        # 4. Reflex Layer 업데이트 (Low-level, 300+ Hz)
        #    접촉 센서 스냅샷을 뮤텍스 밖에서 복사해 RT 루프에 전달
        with self._lock:
            contact_snap = [
                ContactState(
                    force_3axis=cs.force_3axis.copy(),
                    contact_pos=cs.contact_pos.copy(),
                    in_contact=cs.in_contact,
                ) for cs in self.contact_state
            ]

        reflex_override = self._reflex_coord.update(
            self.finger_state, contact_snap, self.motor_state, self.dt
        )

        # 5. 토크 결정: reflex override 우선, 없으면 기본 PD
        if reflex_override is not None:
            torque = reflex_override["torque"]
        else:
            torque = self.controller_run(GraspState.RUN_CONTROL, command)

        # 모터 명령에 저장
        for fi in range(self.n_fingers):
            self.motor_cmd[fi].current = torque[fi]

        # 모터 인터페이스가 있으면 전송
        if self._motor_iface is not None:
            self._step_write_motor_command()

        # 상태 버퍼 업데이트 (외부 read용)
        with self._lock:
            for fi in range(self.n_fingers):
                self._state_buf["q"][fi]       = self.motor_state[fi].q.copy()
                self._state_buf["qdot"][fi]    = self.motor_state[fi].qdot.copy()
                self._state_buf["current"][fi] = self.motor_state[fi].current.copy()
            self._state_buf["active_reflexes"] = self._reflex_coord.active_reflexes()

    # -----------------------------------------------------------------------
    # 상태머신 단계별 처리
    # -----------------------------------------------------------------------

    def _step_wait_stable(self):
        """WAIT_STABLE: 안정화 대기 (일정 시간 후 INIT_CONTROL로 전이)."""
        time.sleep(0.1)
        self.state = GraspState.INIT_CONTROL

    def _step_init_control(self):
        """INIT_CONTROL: 메모리 초기화."""
        for fi in range(self.n_fingers):
            self.motor_state[fi] = MotorState(
                q=np.zeros(self.n_joints),
                qdot=np.zeros(self.n_joints),
                current=np.zeros(self.n_joints),
            )
            self.motor_cmd[fi]   = MotorCommand(
                q=np.zeros(self.n_joints),
                qdot=np.zeros(self.n_joints),
                current=np.zeros(self.n_joints),
            )
            self.finger_state[fi] = FingerState()
            self.finger_cmd[fi]   = FingerCommand()
        self.miss_count = 0
        self.state = GraspState.CHECK_MOTOR

    def _step_check_motor(self):
        """CHECK_MOTOR: 모터 연결 확인. 실제 구현에서는 ping/heartbeat 사용."""
        if self._motor_iface is not None:
            ok = getattr(self._motor_iface, "check_connection", lambda: True)()
            if not ok:
                print("[GraspController] 모터 연결 실패 — 재확인 대기")
                time.sleep(0.5)
                return
        self.state = GraspState.INIT_POSITION

    def _step_init_position(self):
        """INIT_POSITION: 물리 홈 포지션 탐색 후 초기위치 저장."""
        # 실제 구현: 폴링으로 end-stop 감지 후 엔코더 영점화
        for fi in range(self.n_fingers):
            self._home_q[fi] = np.zeros(self.n_joints)

        with self._lock:
            self._cmd_buf["q_target"][:] = self._home_q.copy()

        print("[GraspController] 홈 포지션 초기화 완료")
        self.state = GraspState.RUN_CONTROL

    # -----------------------------------------------------------------------
    # 하드웨어 인터페이스 헬퍼
    # -----------------------------------------------------------------------

    def _step_read_joint_state(self):
        """모터 드라이버에서 joint pos / vel / current 읽기 (폴링)."""
        if self._motor_iface is None:
            # 시뮬 모드: 명령값을 상태로 사용 (가상 응답)
            for fi in range(self.n_fingers):
                self.motor_state[fi].q    += (
                    self.motor_cmd[fi].q - self.motor_state[fi].q
                ) * 0.1
                self.motor_state[fi].qdot = self.motor_cmd[fi].qdot.copy()
            return

        poll = 0
        while poll < self.poll_max:
            try:
                for fi in range(self.n_fingers):
                    q, qdot, cur = self._motor_iface.read_state(fi)
                    self.motor_state[fi].q       = np.asarray(q)
                    self.motor_state[fi].qdot    = np.asarray(qdot)
                    self.motor_state[fi].current = np.asarray(cur)
                return
            except Exception:
                poll += 1
                time.sleep(self.dt * 0.1)

        # 최대 카운트 초과 → miss 처리
        self.miss_count += 1

    def _step_write_motor_command(self):
        """모터 드라이버로 전류 명령 전송."""
        if self._motor_iface is None:
            return
        for fi in range(self.n_fingers):
            try:
                self._motor_iface.write_command(
                    fi,
                    self.motor_cmd[fi].q,
                    self.motor_cmd[fi].qdot,
                    self.motor_cmd[fi].current,
                )
            except Exception:
                self.miss_count += 1

    # -----------------------------------------------------------------------
    # 제어루프 실행 / 중지
    # -----------------------------------------------------------------------

    def start(self):
        """블로킹 제어루프 시작 (스레드에서 호출 권장)."""
        self._running = True
        while self._running:
            t0 = time.perf_counter()

            # 상태머신 디스패치
            if self.state == GraspState.WAIT_STABLE:
                self._step_wait_stable()

            elif self.state == GraspState.INIT_CONTROL:
                self._step_init_control()

            elif self.state == GraspState.CHECK_MOTOR:
                self._step_check_motor()

            elif self.state == GraspState.INIT_POSITION:
                self._step_init_position()

            elif self.state == GraspState.RUN_CONTROL:
                with self._lock:
                    cmd = {k: v.copy() for k, v in self._cmd_buf.items()}
                self._run_control_step(cmd)

            # 2ms 주기 유지
            elapsed = time.perf_counter() - t0
            sleep_t = self.dt - elapsed
            if sleep_t > 0.0:
                time.sleep(sleep_t)

    def stop(self):
        """제어루프 중지."""
        self._running = False

    def start_thread(self) -> threading.Thread:
        """백그라운드 스레드로 제어루프를 시작하고 스레드 객체를 반환한다."""
        t = threading.Thread(target=self.start, daemon=True)
        t.start()
        return t
