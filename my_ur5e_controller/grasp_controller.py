"""
grasp_controller.py
-------------------
Notion 제어기 구조 명세(2024-05-28) 기반 파지 제어기(GraspController) 구현.

RT 태스크 (최고 우선순위, rt7):
    controller_run(state, command)
    GraspController
        - FourBarLinkage  : 4절 링크 메커니즘 (FK / IK)
        - FingerKinematics: 손가락 FK, 6×3 자코비안

기본 태스크 (보통 우선순위):
    - joint / current / vel 명령 수신
    - joint state / 센서 데이터 수신
    - 응답 패킷 송신

제어 상태머신:
    WAIT_STABLE → INIT_CONTROL → CHECK_MOTOR → INIT_POSITION → RUN_CONTROL

처리 흐름 (RUN_CONTROL 루프):
    1. joint state 읽기
    2. 손가락 FK 계산
    3. kinematics / 자코비안 계산
    4. 토크 명령 출력

타이밍:
    제어루프 2ms (500 Hz), 모터 응답 폴링 (miss 카운트)

저장 목록:
    MotorState  : q, qdot, current
    MotorCommand: q, qdot, current
    FingerState : p, R, J (손가락당)
    FingerCommand: p_target, force_target, q_target
"""

import threading
import time
from dataclasses import dataclass, field
from enum import Enum

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
                 poll_max: int = POLL_MAX):

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
        # 손가락당 독립된 상태/명령 저장
        self.motor_state   = [MotorState(
            q=np.zeros(n_joints), qdot=np.zeros(n_joints), current=np.zeros(n_joints)
        ) for _ in range(n_fingers)]

        self.motor_cmd     = [MotorCommand(
            q=np.zeros(n_joints), qdot=np.zeros(n_joints), current=np.zeros(n_joints)
        ) for _ in range(n_fingers)]

        self.finger_state  = [FingerState() for _ in range(n_fingers)]
        self.finger_cmd    = [FingerCommand() for _ in range(n_fingers)]

        # ---------- 기구학 객체 ----------
        # 각 손가락마다 4절 링크 + 손가락 FK 생성
        # (실제 로봇에서는 링크 파라미터를 캘리브레이션으로 얻음)
        self._linkage  = [FourBarLinkage() for _ in range(n_fingers)]
        self._fk_model = [FingerKinematics() for _ in range(n_fingers)]

        # ---------- 홈 포지션 ----------
        self._home_q = np.zeros((n_fingers, n_joints))

        # ---------- PD 게인 ----------
        self.kp = np.full(n_joints, 200.0)
        self.kd = np.full(n_joints, 20.0)

        # ---------- 외부 명령 버퍼 ----------
        # write: joint pos/vel/current, 목표힘, 모션 정의
        self._cmd_buf: dict = {
            "q_target":     np.zeros((n_fingers, n_joints)),
            "vel_target":   np.zeros((n_fingers, n_joints)),
            "cur_target":   np.zeros((n_fingers, n_joints)),
            "force_target": np.zeros(n_fingers),
        }
        # read: joint pos/vel/current (외부에서 읽어 가는 용도)
        self._state_buf: dict = {
            "q":       np.zeros((n_fingers, n_joints)),
            "qdot":    np.zeros((n_fingers, n_joints)),
            "current": np.zeros((n_fingers, n_joints)),
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

    def read_all_states(self) -> dict:
        """모든 손가락 상태 읽기."""
        with self._lock:
            return {
                "q":       self._state_buf["q"].copy(),
                "qdot":    self._state_buf["qdot"].copy(),
                "current": self._state_buf["current"].copy(),
                "grasp_state": self.state.name,
                "miss_count":  self.miss_count,
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
        """RUN_CONTROL 한 스텝: 읽기 → FK → 자코비안 → 토크 출력."""

        # 1. joint state 읽기
        self._step_read_joint_state()

        # 2 & 3. 손가락 FK / 자코비안 계산
        for fi in range(self.n_fingers):
            q_motor = self.motor_state[fi].q

            # 4절 링크 FK: 모터 각도 → 근위부 각도
            q_proximal = self._linkage[fi].fk(q_motor[0])
            # 나머지 관절은 직접 연결
            q_finger = np.array([q_proximal, q_motor[1], q_motor[2]])

            # 손가락 FK + 자코비안
            self.finger_state[fi] = self._fk_model[fi].fk(q_finger)

        # 4. 토크 명령 출력
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
