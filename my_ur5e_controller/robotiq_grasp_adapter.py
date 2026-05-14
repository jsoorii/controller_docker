"""
robotiq_grasp_adapter.py
─────────────────────────
Robotiq 2F-85 + UR5e (MuJoCo 시뮬레이션) 용
GraspController 반사 제어 통합 어댑터.

GraspController의 AntiSlipReflex 로직을 재사용하여
MuJoCo 접촉 데이터 기반 Anti-Slip 그리퍼 제어를 구현한다.

설계 결정:
    n_fingers = 2, n_joints = 1  (Robotiq 2F-85: 단일 액추에이터 병렬 그리퍼)
    AntiSlipReflex 만 사용       (ReGraspReflex: 독립 핑거 제어 불가하므로 제외)
    접촉 데이터: mj_contactForce() API로 패드 geom에서 직접 읽기
    출력 매핑:  AntiSlipReflex._q_target (0~1) → gripper ctrl (0~255)
              더 닫히는 방향만 허용 (물체 낙하 방지 안전 정책)

GraspController 계층 구조와 차이점:
    원본                         이 어댑터
    ─────────────────────────── ─────────────────────────────────
    motor_iface.read_state()    MuJoCo data.ctrl 에서 직접 읽기
    motor_iface.write_command() ur_controller._gripper_ctrl 갱신
    FingerKinematics (FK)       미사용 (1-DOF 그리퍼 불필요)
    FourBarLinkage              미사용
    ReGraspReflex               미사용
    AntiSlipReflex              재사용 (쿨롱 마찰 + PD 토크 → q_target)

사용법:
    adapter = RobotiqGraspAdapter(model, data, ur_controller)
    adapter.start_thread()          # 백그라운드 100 Hz 루프
    # 또는
    adapter.step()                  # 외부 루프에서 직접 호출
"""

import json
import os
import sys
import threading
import time

import mujoco
import numpy as np

# grasp_controller.py 경로 추가 (같은 디렉토리)
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from grasp_controller import (
    AntiSlipReflex,
    ReGraspReflex,
    ReflexCoordinator,
    ReflexState,
    MotorState,
    FingerState,
    ContactState,
)

# ReGraspReflex DOF 매핑: q[0]=그리퍼 비율, q[1]=EE Y 위치
# 비대칭 파지 감지 시 그리퍼 + 팔 lateral 동시 조정
_ARM_LATERAL_SCALE = 0.04   # q_target 차이 1 → Y_delta(m), 최대 ±4cm
_CONTACT_POS_SCALE = 10.0   # 월드 lateral 편차(m) → contact_pos 스케일


class RobotiqGraspAdapter:
    """Robotiq 2F-85 Anti-Slip Reflex 통합 관리자.

    GraspController의 AntiSlipReflex 로직을 그대로 사용하되,
    MuJoCo 접촉 데이터와 Robotiq 1-DOF 액추에이터에 맞게 인터페이스를 조정한다.

    Parameters
    ----------
    model          : MuJoCo model
    data           : MuJoCo data
    ur_controller  : UR5eRTController 인스턴스
    hz             : 반사 제어 루프 주기 (기본 100 Hz)
    mu             : 마찰 계수 (기본 0.5 — 고무/금속)
    slip_threshold : 슬립 발동 임계 비율 (기본 0.85)
    safety_factor  : 목표 슬립 비율 완료 조건 (기본 0.60)
    f_normal_min   : 접촉 인식 최소 법선력 N (기본 0.5 — 그리퍼 하중 고려)
    dq_step        : 한 스텝 정규화 닫기 증분 (기본 0.02 = 2%)
    """

    def __init__(self, model, data, ur_controller,
                 hz: float = 100.0,
                 mu: float = 0.5,
                 slip_threshold: float = 0.85,
                 safety_factor: float = 0.60,
                 f_normal_min: float = 0.5,
                 dq_step: float = 0.02):
        self._model = model
        self._data  = data
        self._ur    = ur_controller
        self.dt     = 1.0 / hz

        # 그리퍼 액추에이터 범위
        self._act_id     = ur_controller._gripper_act_id
        self._ctrl_open  = float(ur_controller._gripper_ctrl_open)
        self._ctrl_close = float(ur_controller._gripper_ctrl_close)

        if self._act_id < 0:
            print("[RobotiqGraspAdapter] 경고: 그리퍼 액추에이터 없음 — 비활성 모드")

        # 핑거 패드 geom ID 탐색
        self._pad_ids: list = self._find_pad_geoms()
        print(f"[RobotiqGraspAdapter] 좌 핑거 패드 geom ID: {self._pad_ids[0]}")
        print(f"[RobotiqGraspAdapter] 우 핑거 패드 geom ID: {self._pad_ids[1]}")

        # ── GraspController 반사 레이어 ──────────────────────────────────────
        # Robotiq 2F-85: n_fingers=2, n_joints=1
        # AntiSlipReflex만 사용 (ReGraspReflex 제외 — 독립 핑거 제어 불가)
        self._anti_slip = AntiSlipReflex(
            n_fingers=2,
            n_joints=1,
            mu=mu,
            slip_threshold=slip_threshold,
            safety_factor=safety_factor,
            f_normal_min=f_normal_min,
            dq_step=dq_step,
        )
        # AntiSlipReflex는 timeout_s=0.3 고정값으로 super().__init__()을 호출하므로
        # 생성자 인자로 timeout_s를 넘기면 **kwargs 경유로 중복 전달 → TypeError.
        # 대신 인스턴스 속성을 직접 덮어써서 타임아웃을 조정한다.
        self._anti_slip.timeout_s = 0.5
        self._coordinator = ReflexCoordinator([self._anti_slip])

        # ── ReGraspReflex ─────────────────────────────────────────────────────
        # n_joints=2: q[0]=그리퍼 비율, q[1]=EE Y 위치
        self._regrasp = ReGraspReflex(
            n_fingers=2, n_joints=2,
            sym_threshold=0.85, dq_step=0.005,
        )
        self._regrasp_coordinator = ReflexCoordinator([self._regrasp])
        self._regrasp_finger_states = [FingerState(), FingerState()]

        self._running = False
        self._lock = threading.Lock()

        # ── 무게 추정 ──────────────────────────────────────────────────────────
        # 원리: 정적 파지 시 Σ(F_contact_i · ẑ_world) = 물체 무게 W
        #       m ≈ Σ(수직 지지력) / g
        self._weight_buf: list = []           # 이동 평균 버퍼
        self._WEIGHT_WINDOW: int = 50         # 0.5 s @ 100 Hz
        self._no_contact_count: int = 0       # 접촉 소실 카운터
        self._NO_CONTACT_RESET: int = 50      # 0.5 s 이상 미접촉 시 버퍼 리셋

        # 상태 파일 (웹 대시보드용)
        self._state_write_counter: int = 0
        self._STATE_PATH = "/tmp/grasp_state.json"

        print(f"[RobotiqGraspAdapter] 초기화 완료 "
              f"(μ={mu}, slip_thr={slip_threshold}, dq_step={dq_step})")

    # ------------------------------------------------------------------
    # MuJoCo 데이터 읽기
    # ------------------------------------------------------------------

    def _find_pad_geoms(self) -> list:
        """Robotiq 2F-85 좌/우 핑거 패드 geom ID를 이름으로 탐색한다.

        mujoco_menagerie Robotiq 2F-85(v3/v4) geom 명명 패턴:
            v4: gripper/left_pad  / gripper/right_pad
            v3: gripper/finger_1_pad_collision / gripper/finger_2_pad_collision
        gripper_config.attach()에서 "gripper/" prefix가 붙는다.
        """
        LEFT_KEYS  = ["left",  "finger_1", "pad_l", "padl"]
        RIGHT_KEYS = ["right", "finger_2", "pad_r", "padr"]

        pads: list = [[], []]
        for i in range(self._model.ngeom):
            raw  = mujoco.mj_id2name(self._model, mujoco.mjtObj.mjOBJ_GEOM, i)
            name = (raw or "").lower()
            if "gripper/" not in name:
                continue
            local = name.split("gripper/", 1)[1]
            if any(k in local for k in LEFT_KEYS):
                pads[0].append(i)
            elif any(k in local for k in RIGHT_KEYS):
                pads[1].append(i)

        # 탐색 실패: 모든 gripper geom을 양 핑거에 동시 등록 (fallback)
        if not pads[0] and not pads[1]:
            all_gripper = [
                i for i in range(self._model.ngeom)
                if "gripper/" in (
                    mujoco.mj_id2name(self._model, mujoco.mjtObj.mjOBJ_GEOM, i) or ""
                ).lower()
            ]
            pads[0] = pads[1] = all_gripper
            if all_gripper:
                print("[RobotiqGraspAdapter] 경고: 패드 이름 탐색 실패 "
                      f"→ 전체 gripper geom {len(all_gripper)}개 사용")
            else:
                print("[RobotiqGraspAdapter] 경고: gripper geom 없음 "
                      "— 접촉 감지 비활성")
        return pads

    def _read_contact_state(self, fi: int) -> ContactState:
        """핑거 fi의 MuJoCo 접촉 상태를 읽는다.

        MuJoCo 접촉 프레임 규약:
            force6[0] = 법선력  (contact normal 방향)
            force6[1] = 1차 전단력
            force6[2] = 2차 전단력

        GraspController ContactState 규약:
            force_3axis[2] = 법선력 (fz)
            force_3axis[0], [1] = 전단력 (fx, fy)
        """
        cs = ContactState()
        pad_set = set(self._pad_ids[fi])

        for j in range(self._data.ncon):
            c = self._data.contact[j]
            if int(c.geom1) not in pad_set and int(c.geom2) not in pad_set:
                continue

            force6 = np.zeros(6)
            mujoco.mj_contactForce(self._model, self._data, j, force6)

            # [MuJoCo normal, tan1, tan2] → [fx, fy, fz] 재매핑
            cs.force_3axis = np.array([force6[1], force6[2], force6[0]])
            cs.in_contact  = True
            # contact_pos: Robotiq 2F-85는 단일 접촉점이므로 단순화
            cs.contact_pos = np.array([0.0, 0.0])
            break  # 패드 당 첫 번째 유효 접촉만 사용

        return cs

    def _read_contact_state_regrasp(self, fi: int) -> ContactState:
        """ReGraspReflex용 접촉 상태. contact_pos[0]을 EE 기준 Y 편차로 계산."""
        cs = ContactState()
        pad_set = set(self._pad_ids[fi])
        for j in range(self._data.ncon):
            c = self._data.contact[j]
            if int(c.geom1) not in pad_set and int(c.geom2) not in pad_set:
                continue
            force6 = np.zeros(6)
            mujoco.mj_contactForce(self._model, self._data, j, force6)
            cs.force_3axis = np.array([force6[1], force6[2], force6[0]])
            cs.in_contact = True
            # EE 기준 접촉점의 Y 편차: 대칭 파지 시 finger0=-δ, finger1=+δ → 합=0
            ee_pos, _ = self._ur.get_ee_pose()
            lateral = float(c.pos[1] - ee_pos[1]) * _CONTACT_POS_SCALE
            cs.contact_pos = np.array([lateral, 0.0])
            break
        return cs

    def _read_motor_states_regrasp(self) -> list:
        """ReGraspReflex용 2-DOF 모터 상태: q[0]=그리퍼 비율, q[1]=EE Y."""
        span = (self._ctrl_close - self._ctrl_open) + 1e-9
        ctrl = float(self._data.ctrl[self._act_id]) if self._act_id >= 0 else self._ctrl_open
        ratio = float(np.clip((ctrl - self._ctrl_open) / span, 0.0, 1.0))
        ee_pos, _ = self._ur.get_ee_pose()
        ee_y = float(ee_pos[1])
        return [
            MotorState(q=np.array([ratio, ee_y]), qdot=np.zeros(2), current=np.zeros(2))
            for _ in range(2)
        ]

    def _apply_regrasp_output(self) -> None:
        """ReGraspReflex._q_target → 그리퍼 비율 + UR5e arm Y lateral 적용."""
        qt = self._regrasp._q_target   # shape (2, 2)
        # DOF 0: 그리퍼 (평균)
        grip = float(np.clip((qt[0, 0] + qt[1, 0]) / 2, 0.0, 1.0))
        self._ur.set_gripper(grip)
        # DOF 0 차이 → Y lateral 환산
        # qt[0, 1]: 리플렉스 시작 시 캡처된 EE Y (execute() 첫 진입에서 고정)
        # ee_pos[1]을 기준으로 쓰면 팔이 이동할수록 delta가 중복 적용되어 오버슈트 발생
        lateral_delta = (qt[0, 0] - qt[1, 0]) * _ARM_LATERAL_SCALE
        new_y = float(np.clip(qt[0, 1] - lateral_delta, -0.15, 0.15))
        target = self._ur.task_cmd["pos_target"].copy()
        target[1] = new_y
        self._ur.task_cmd["pos_target"] = target
        self._ur.task_cmd["use_task_space"] = True

    def _read_motor_states(self) -> list:
        """현재 그리퍼 상태를 MotorState 목록으로 반환.

        q[0] = 정규화 폐쇄량 (0.0 = 완전 열림, 1.0 = 완전 닫힘)
        양 핑거가 동일한 액추에이터에 연결되어 있으므로 같은 값을 반환한다.
        """
        span  = (self._ctrl_close - self._ctrl_open) + 1e-9
        if self._act_id >= 0:
            ctrl = float(self._data.ctrl[self._act_id])
        else:
            ctrl = self._ctrl_open
        ratio = float(np.clip((ctrl - self._ctrl_open) / span, 0.0, 1.0))

        return [
            MotorState(
                q=np.array([ratio]),
                qdot=np.zeros(1),
                current=np.zeros(1),
            )
            for _ in range(2)
        ]

    # ------------------------------------------------------------------
    # 무게 추정
    # ------------------------------------------------------------------

    def _estimate_weight_raw(self) -> tuple:
        """월드 프레임 접촉력의 수직 지지 성분 합을 반환한다.

        MuJoCo 접촉 프레임 변환:
            force_world = contact.frame.reshape(3,3).T @ force6[:3]
            weight_support_per_finger = max(0, force_world[2])   (+z = 위쪽)

        정적 파지 가정 하에:
            Σ weight_support ≈ m × g  →  m ≈ Σ / g

        Returns
        -------
        total_N   : 수직 지지력 합 (N)
        n_contact : 접촉 중인 핑거 수
        mu_list   : 핑거별 실효 마찰 계수 [μ_eff = |F_tan| / F_normal]
        F_normal_list : 핑거별 법선력 (N)
        """
        total_N = 0.0
        n_contact = 0
        mu_list: list = []
        fn_list: list = []

        for fi in range(2):
            pad_set = set(self._pad_ids[fi])
            for j in range(self._data.ncon):
                c = self._data.contact[j]
                if int(c.geom1) not in pad_set and int(c.geom2) not in pad_set:
                    continue

                force6 = np.zeros(6)
                mujoco.mj_contactForce(self._model, self._data, j, force6)

                # 월드 프레임으로 변환
                frame = c.frame.reshape(3, 3)
                force_world = frame.T @ force6[:3]

                # 수직 지지력 (중력 반대 방향)
                upward = float(force_world[2])
                if upward > 0.0:
                    total_N += upward
                    n_contact += 1

                # 실효 마찰 계수 μ_eff = |F_tangential| / F_normal
                f_normal = abs(float(force6[0]))
                f_tan    = float(np.sqrt(force6[1]**2 + force6[2]**2))
                mu_eff   = f_tan / (f_normal + 1e-9)
                mu_list.append(round(mu_eff, 3))
                fn_list.append(round(f_normal, 3))

                break  # 핑거당 첫 번째 유효 접촉만 사용

        return total_N, n_contact, mu_list, fn_list

    def _write_grasp_state(self):
        """반사 제어 + 무게 추정 상태를 JSON 파일로 원자적 저장."""
        mass_mean = float(np.mean(self._weight_buf)) if self._weight_buf else 0.0
        mass_std  = float(np.std(self._weight_buf))  if len(self._weight_buf) > 1 else 0.0
        n_samples = len(self._weight_buf)

        # 마지막 접촉 데이터
        _, n_contact, mu_list, fn_list = self._estimate_weight_raw()

        ctrl = float(self._ur._gripper_ctrl) if self._act_id >= 0 else 0.0
        span = (self._ctrl_close - self._ctrl_open) + 1e-9
        ratio = float(np.clip((ctrl - self._ctrl_open) / span, 0.0, 1.0))

        antislip_on = bool(getattr(self._ur, "antislip_enabled", True))
        regrasp_on  = bool(getattr(self._ur, "regrasp_enabled", False))
        confidence = (
            "high"   if n_samples >= 30 and n_contact >= 1 else
            "medium" if n_samples >= 10 and n_contact >= 1 else
            "low"
        )

        state = {
            "antislip": {
                "enabled":     antislip_on,
                "state":       self._anti_slip.state.name,
                "ctrl":        round(ctrl, 1),
                "close_ratio": round(ratio, 3),
            },
            "regrasp": {
                "enabled": regrasp_on,
                "state":   self._regrasp.state.name,
            },
            "weight": {
                "mass_kg":    round(mass_mean, 4),
                "weight_N":   round(mass_mean * 9.81, 3),
                "std_kg":     round(mass_std, 4),
                "n_samples":  n_samples,
                "n_contact":  n_contact,
                "confidence": confidence,
            },
            "friction": {
                "mu_eff":   mu_list,
                "F_normal": fn_list,
            },
            "ts": time.time(),
        }
        try:
            tmp = self._STATE_PATH + ".tmp"
            with open(tmp, "w") as f:
                json.dump(state, f)
            os.replace(tmp, self._STATE_PATH)
        except Exception:
            pass

    # ------------------------------------------------------------------
    # 반사 출력 적용
    # ------------------------------------------------------------------

    def _apply_q_target(self):
        """AntiSlipReflex._q_target → gripper ctrl 변환 및 적용.

        두 핑거가 단일 액추에이터로 연결되므로 더 닫힌 쪽 q_target을 사용한다.
        안전 정책: 현재보다 더 닫히는 방향만 허용 (물체 낙하 방지).
        """
        if not self._anti_slip.is_active():
            return

        # _q_target[fi, 0]: 정규화 폐쇄량 (0=열림, 1=닫힘)
        q_tgt = float(max(
            self._anti_slip._q_target[0, 0],
            self._anti_slip._q_target[1, 0],
        ))
        q_tgt = np.clip(q_tgt, 0.0, 1.0)

        ctrl_new = self._ctrl_open + q_tgt * (self._ctrl_close - self._ctrl_open)

        with self._lock:
            ctrl_cur = float(self._ur._gripper_ctrl)
            # 더 닫히는 방향만 허용 (열기는 상위 명령에 맡김)
            if ctrl_new > ctrl_cur:
                self._ur._gripper_ctrl = ctrl_new

    # ------------------------------------------------------------------
    # 제어 루프
    # ------------------------------------------------------------------

    def step(self):
        """반사 제어 한 스텝.

        mj_step() 이후 호출해야 최신 접촉 데이터를 읽을 수 있다.
        별도 스레드에서 호출하면 약간의 지연이 있지만 Anti-Slip 용도로는 충분하다.
        """
        if self._act_id < 0:
            return

        # ── Anti-Slip Reflex ──────────────────────────────────────────────────
        if self._ur.antislip_enabled:
            motor_states   = self._read_motor_states()
            contact_states = [self._read_contact_state(fi) for fi in range(2)]
            finger_states  = [FingerState() for _ in range(2)]
            self._coordinator.update(finger_states, contact_states, motor_states, self.dt)
            self._apply_q_target()
        else:
            if self._anti_slip.state != ReflexState.IDLE:
                self._anti_slip.state = ReflexState.IDLE

        # ── ReGrasp Reflex ────────────────────────────────────────────────────
        if self._ur.regrasp_enabled:
            rg_motor   = self._read_motor_states_regrasp()
            rg_contact = [self._read_contact_state_regrasp(fi) for fi in range(2)]
            result = self._regrasp_coordinator.update(
                self._regrasp_finger_states, rg_contact, rg_motor, self.dt
            )
            if result is not None:
                self._apply_regrasp_output()
        else:
            if self._regrasp.state != ReflexState.IDLE:
                self._regrasp.state = ReflexState.IDLE

        # ── 무게 추정 업데이트 ────────────────────────────────────────────────
        raw_N, n_contact, _, _ = self._estimate_weight_raw()
        if n_contact > 0:
            self._no_contact_count = 0
            self._weight_buf.append(raw_N / 9.81)
            if len(self._weight_buf) > self._WEIGHT_WINDOW:
                self._weight_buf.pop(0)
        else:
            self._no_contact_count += 1
            if self._no_contact_count >= self._NO_CONTACT_RESET:
                self._weight_buf.clear()
                self._no_contact_count = 0

        # ── 상태 파일 주기적 저장 (10 Hz) ────────────────────────────────────
        self._state_write_counter += 1
        if self._state_write_counter >= 10:
            self._state_write_counter = 0
            self._write_grasp_state()

        # ── 활성 반사 + 무게 로그 ─────────────────────────────────────────────
        active = self._coordinator.active_reflexes()
        if active or (self._weight_buf and n_contact > 0):
            ctrl = float(self._ur._gripper_ctrl)
            ratio = (ctrl - self._ctrl_open) / (self._ctrl_close - self._ctrl_open + 1e-9)
            mass  = float(np.mean(self._weight_buf)) if self._weight_buf else 0.0
            print(
                f"\r[Grasp] ctrl={ctrl:.0f} ({ratio*100:.1f}%)"
                f" | 무게≈{mass*1000:.0f} g"
                f" | reflex={active or '없음'}",
                end="",
            )

    def start(self):
        """블로킹 반사 제어 루프 (별도 스레드에서 실행 권장)."""
        self._running = True
        print("[RobotiqGraspAdapter] Anti-Slip 반사 제어 루프 시작")
        while self._running:
            t0 = time.perf_counter()
            self.step()
            elapsed = time.perf_counter() - t0
            sleep_t = self.dt - elapsed
            if sleep_t > 0.0:
                time.sleep(sleep_t)

    def stop(self):
        """반사 제어 루프 중지."""
        self._running = False

    def start_thread(self) -> threading.Thread:
        """백그라운드 스레드로 반사 제어 루프를 시작하고 스레드 객체를 반환한다."""
        t = threading.Thread(target=self.start, daemon=True)
        t.start()
        return t

    # ------------------------------------------------------------------
    # 상태 조회 (웹 대시보드 등 외부 참조용)
    # ------------------------------------------------------------------

    def get_status(self) -> dict:
        """현재 Anti-Slip 반사 상태를 딕셔너리로 반환한다."""
        ctrl = float(self._ur._gripper_ctrl) if self._act_id >= 0 else 0.0
        span = (self._ctrl_close - self._ctrl_open) + 1e-9
        ratio = float(np.clip((ctrl - self._ctrl_open) / span, 0.0, 1.0))
        return {
            "active_reflexes": self._coordinator.active_reflexes(),
            "gripper_ctrl":    ctrl,
            "close_ratio":     round(ratio, 3),
            "antislip_state":  self._anti_slip.state.name,
            "q_target": [
                round(float(self._anti_slip._q_target[0, 0]), 3),
                round(float(self._anti_slip._q_target[1, 0]), 3),
            ],
        }
