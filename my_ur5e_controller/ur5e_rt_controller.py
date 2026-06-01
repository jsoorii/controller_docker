import json
import numpy as np
import os
import time
import threading
import mujoco
import mink
from enum import Enum

class ControlState(Enum):
    WAIT_STABLE       = 0
    INIT_CONTROL      = 1
    CHECK_MOTOR       = 2
    INIT_POSITION     = 3
    RUN_CONTROL       = 4
    COLLISION_STOP    = 5
    COLLISION_ESCAPE  = 6


def _quat_to_rotmat(x, y, z, w) -> np.ndarray:
    """Quaternion (x, y, z, w) → 3×3 rotation matrix."""
    return np.array([
        [1 - 2*(y*y + z*z),  2*(x*y - w*z),      2*(x*z + w*y)],
        [2*(x*y + w*z),      1 - 2*(x*x + z*z),   2*(y*z - w*x)],
        [2*(x*z - w*y),      2*(y*z + w*x),       1 - 2*(x*x + y*y)],
    ])


class UR5eRTController:
    def __init__(self, model, data, ee_body_name="wrist_3_link",
                 ee_max_vel: float = 1.0,    # EE 최대 선속도 (m/s)
                 ee_max_acc: float = 5.0,    # EE 최대 가속도 (m/s²)
                 ee_max_jerk: float = 50.0,  # EE 최대 저크   (m/s³)
                 gripper_cfg=None):  # GripperConfig | None
        self.model = model
        self.data = data
        self.state = ControlState.WAIT_STABLE
        self.regrasp_enabled = False

        # Resolve end-effector body ID
        body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, ee_body_name)
        if body_id == -1:
            body_id = model.nbody - 1
            print(f"[Controller] Warning: '{ee_body_name}' not found, using body id={body_id}")
        self.ee_body_id = body_id

        # Joint-space state & command
        self.motor_state = {"q": np.zeros(6), "dq": np.zeros(6)}
        self.motor_cmd   = {"q_target": np.zeros(6)}

        # Task-space command
        self.task_cmd = {
            "pos_target":    np.zeros(3),
            "rot_target":    np.eye(3),
            "use_task_space": False,
        }

        # EE 운동 한계
        self.ee_max_vel  = ee_max_vel
        self.ee_max_acc  = ee_max_acc
        self.ee_max_jerk = ee_max_jerk

        # mink IK setup (100 Hz — 5 sim steps마다 1회 QP 풀기)
        _gripper_dof_ids   = list(range(6, model.nv))
        self._mink_cfg     = mink.Configuration(model, data.qpos.copy())
        self._mink_ee_task = mink.FrameTask(
            ee_body_name, "body",
            position_cost=1.0,
            orientation_cost=0.5,
        )
        self._mink_damping = mink.DampingTask(model, cost=1e-3)
        self._mink_tasks   = [self._mink_ee_task, self._mink_damping]
        if _gripper_dof_ids:
            self._mink_tasks.append(mink.DofFreezingTask(model, _gripper_dof_ids))

        # 바닥-팔 충돌 회피: 바닥(world body) 지옴 vs 팔 링크 지옴
        _floor_geom_ids = [
            gi for gi in range(model.ngeom)
            if (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY,
                                  int(model.geom_bodyid[gi])) or "") == "world"
        ]
        _arm_body_keywords = ("shoulder", "upper_arm", "forearm", "wrist")
        _arm_geom_ids = [
            gi for gi in range(model.ngeom)
            if any(kw in (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY,
                                             int(model.geom_bodyid[gi])) or "")
                   for kw in _arm_body_keywords)
        ]
        _col_limit = None
        if _floor_geom_ids and _arm_geom_ids:
            _col_limit = mink.CollisionAvoidanceLimit(
                model,
                [(_floor_geom_ids, _arm_geom_ids)],
                minimum_distance_from_collisions=0.015,   # 1.5cm 최소 거리
                collision_detection_distance=0.08,        # 8cm 이내서 활성화
            )
        self._mink_limits  = [mink.VelocityLimit(model)]
        if _col_limit is not None:
            self._mink_limits.append(_col_limit)
        self._mink_skip    = 0   # IK 스킵 카운터 (매 5스텝마다 1회 실행)

        # 오프라인 pre-solve 상태 (home → target IK)
        self._presolve_q:       np.ndarray | None = None   # 수렴된 관절각 (완료 시)
        self._presolve_pos_tgt: np.ndarray        = np.zeros(3)
        self._presolve_pending: bool              = False  # 백그라운드 실행 중

        # home 키프레임 q (pre-solve 초기 자세)
        _home_key = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "home")
        if _home_key >= 0:
            self._home_qpos = model.key_qpos[_home_key][:model.nq].copy()
        else:
            self._home_qpos = np.zeros(model.nq)
            self._home_qpos[:6] = [-1.5708, -1.5708, 1.5708, -1.5708, -1.5708, 0.0]

        # PD gains
        self.kp = 500.0
        self.kd = 50.0

        self.loop_hz = 500
        self.dt = 1.0 / self.loop_hz
        self.miss_count = 0

        # Trajectory state
        self._traj_lock   = threading.Lock()
        self._traj_points: list = []
        self._traj_start: float = 0.0

        # EE target motion profile state (quintic interpolation)
        self._ik_cmd_start_time: float = 0.0
        self._ik_cmd_duration: float   = 0.0
        self._ik_prof_pos_start: np.ndarray = np.zeros(3)
        self._ik_prof_pos_end:   np.ndarray = np.zeros(3)
        self._ik_prof_rot_start: np.ndarray = np.eye(3)
        self._ik_prof_rot_end:   np.ndarray = np.eye(3)
        # 정규화 초기/종료 속도 (0 = 표준 quintic)
        self._ik_v0_norm: float = 0.0  # 출발 속도 (정규화)
        self._ik_v1_norm: float = 0.0  # 도착 속도 (정규화)

        # State file writer
        self._state_write_counter: int = 0
        self._STATE_PATH = "/tmp/robot_state.json"

        # Soft joint limits (속도 감속 / 반발 토크)
        self.soft_limit_enabled: bool  = True
        self.soft_limit_margin:  float = 0.05   # 범위 대비 비율 (0.05 = 5%)
        self.soft_limit_kp:      float = 200.0  # 반발 강성 (N·m/rad)
        self.soft_limit_kd:      float = 20.0   # 속도 감쇠 (N·m·s/rad)
        self._soft_lo: np.ndarray = np.zeros(6)
        self._soft_hi: np.ndarray = np.zeros(6)
        self._update_soft_limits()

        # Collision detection state
        self.collision_detected: bool = False
        self.collision_info: dict = {}
        self._freeze_q: np.ndarray = np.zeros(6)
        self._robot_body_ids: set = self._build_robot_body_ids()
        self._gripper_body_ids: set = self._build_gripper_body_ids()

        # Self-collision / object-collision avoidance (repulsion torque)
        self.self_collision_enabled: bool  = True
        self.self_collision_kp:      float = 500.0   # 반발 강성 (N/m)
        self.self_collision_kd:      float = 50.0    # 속도 감쇠 (N·s/m)
        self._self_col_margin:       float = 0.02    # 근접 감지 거리 (m)
        self._set_robot_geom_margins()

        # 충돌 직전 안전 위치 버퍼 (50스텝 = 0.1s @ 500Hz)
        self._safe_q_buf_size: int = 50
        self._safe_q_buffer: list = []
        self._escape_target_q: np.ndarray = np.zeros(6)

        # Gripper: GripperConfig 객체로 설정, 없으면 비활성
        if gripper_cfg is not None:
            _act_name = f"gripper/{gripper_cfg.actuator}"
            self._gripper_act_id:     int   = mujoco.mj_name2id(
                model, mujoco.mjtObj.mjOBJ_ACTUATOR, _act_name)
            self._gripper_ctrl_open:  float = float(gripper_cfg.ctrl_open)
            self._gripper_ctrl_close: float = float(gripper_cfg.ctrl_close)
        else:
            self._gripper_act_id     = -1
            self._gripper_ctrl_open  = 0.0
            self._gripper_ctrl_close = 1.0
        self._gripper_ctrl: float = self._gripper_ctrl_open  # 시작 시 열림

        # ROS2 state
        self.ros_cmd_active: bool = False   # True after any external ROS2 command
        self._ros_ee_duration: float = 0.0  # pending duration from /ur5e/cmd/ee_duration
        self._ros_node = None

        # Anti-Slip Reflex 활성화 플래그 (RobotiqGraspAdapter에서 참조)
        self.antislip_enabled: bool = True

    # ------------------------------------------------------------------
    # Motion profile helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _rotmat_to_euler_zyx(R: np.ndarray):
        """Rotation matrix → ZYX Euler angles (rad): roll, pitch, yaw."""
        pitch = -np.arcsin(np.clip(R[2, 0], -1.0, 1.0))
        roll  = np.arctan2(R[2, 1], R[2, 2])
        yaw   = np.arctan2(R[1, 0], R[0, 0])
        return roll, pitch, yaw

    _JOINT_SENSOR_NAMES = [
        'torque_shoulder_pan', 'torque_shoulder_lift', 'torque_elbow',
        'torque_wrist_1',      'torque_wrist_2',       'torque_wrist_3',
    ]

    def _read_joint_torques(self) -> list:
        """XML actuatorfrc 센서에서 관절 토크를 읽는다. 센서 미등록 시 actuator_force 사용."""
        tau = []
        for i, sname in enumerate(self._JOINT_SENSOR_NAMES):
            sid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SENSOR, sname)
            if sid >= 0:
                addr = self.model.sensor_adr[sid]
                tau.append(float(self.data.sensordata[addr]))
            else:
                tau.append(float(self.data.actuator_force[i]))
        return tau

    def _read_contacts(self) -> list:
        """모든 접촉의 위치·법선·월드계 힘벡터·힘 크기를 반환한다."""
        contacts = []
        for i in range(self.data.ncon):
            c = self.data.contact[i]
            force6 = np.zeros(6)
            mujoco.mj_contactForce(self.model, self.data, i, force6)
            frame = c.frame.reshape(3, 3)
            force_world = frame.T @ force6[:3]
            contacts.append({
                'geom1':       int(c.geom1),
                'geom2':       int(c.geom2),
                'dist':        float(c.dist),
                'pos':         c.pos.tolist(),
                'normal':      c.frame[:3].tolist(),
                'force_world': force_world.tolist(),
                'force_mag':   float(np.linalg.norm(force_world)),
            })
        return contacts

    def _write_state(self):
        """현재 로봇 상태를 JSON 파일로 저장 (atomic write)."""
        try:
            ee_pos, ee_rot = self.get_ee_pose()
            roll, pitch, yaw = self._rotmat_to_euler_zyx(ee_rot)
            state = {
                "q":            self.motor_state["q"].tolist(),
                "dq":           self.motor_state["dq"].tolist(),
                "ddq":          self.data.qacc[:6].tolist(),
                "tau":          self._read_joint_torques(),
                "ee_pos":       ee_pos.tolist(),
                "ee_euler_deg": [float(np.degrees(roll)),
                                 float(np.degrees(pitch)),
                                 float(np.degrees(yaw))],
                "ts":           time.time(),
                "contacts":     self._read_contacts(),
                "collision": {
                    "detected": self.collision_detected,
                    "info":     self.collision_info,
                },
                "gripper": {
                    "ctrl": self._gripper_ctrl,
                    "open_ratio": round(1.0 - (
                        (self._gripper_ctrl - self._gripper_ctrl_open) /
                        (self._gripper_ctrl_close - self._gripper_ctrl_open + 1e-9)
                    ), 3),
                },
                "joint_limits": self.model.jnt_range[:6].tolist(),
                "soft_limits": {
                    "enabled": self.soft_limit_enabled,
                    "margin":  self.soft_limit_margin,
                    "kp":      self.soft_limit_kp,
                    "kd":      self.soft_limit_kd,
                },
            }
            tmp = self._STATE_PATH + ".tmp"
            with open(tmp, "w") as f:
                json.dump(state, f)
            os.replace(tmp, self._STATE_PATH)
        except Exception:
            pass

    @staticmethod
    def _smooth_alpha(alpha: float) -> float:
        """Quintic polynomial: s(t) = 10t³ - 15t⁴ + 6t⁵
        시작/끝에서 속도·가속도 모두 0 → 부드러운 가감속."""
        a = np.clip(alpha, 0.0, 1.0)
        return a ** 3 * (10.0 - 15.0 * a + 6.0 * a * a)

    @staticmethod
    def _quintic_velocity(t_norm: float) -> float:
        """Quintic 프로파일의 순간 속도 (정규화, [0,1]에서 적분=1).
        v(t) = 30t²(1-t)², 최대값 1.875 at t=0.5."""
        t = np.clip(t_norm, 0.0, 1.0)
        return 30.0 * t * t * (1.0 - t) * (1.0 - t)

    @staticmethod
    def _smooth_alpha_blend(tau: float, v0: float, v1: float) -> float:
        """초기/종료 속도를 지정할 수 있는 일반 quintic.
        경계 조건: f(0)=0, f(1)=1, f'(0)=v0, f'(1)=v1, f''(0)=0, f''(1)=0.
        v0=v1=0 이면 표준 quintic(10t³-15t⁴+6t⁵)과 동일.
        속도는 정규화 값: v_norm = v_actual(m/s) * duration(s) / dist(m)."""
        t = np.clip(tau, 0.0, 1.0)
        c1 = v0
        c3 = 10.0 - 6.0 * v0 - 4.0 * v1
        c4 = -15.0 + 8.0 * v0 + 7.0 * v1
        c5 = 6.0 - 3.0 * v0 - 3.0 * v1
        return c1 * t + c3 * t**3 + c4 * t**4 + c5 * t**5

    def _compute_min_duration(self, dist: float,
                              v0_ms: float, v1_ms: float,
                              ensure_decel: bool = False) -> float:
        """속도·가속도·저크 한계를 모두 만족하는 최소 duration (s) 을 이진탐색으로 계산.

        핵심 관계 (정규화 도함수 → 실제 물리량):
          vel_actual  = vel_norm  × dist / T
          acc_actual  = acc_norm  × dist / T²
          jerk_actual = jerk_norm × dist / T³

        T 를 키우면 실제 물리량이 단조 감소하므로 이진탐색이 성립한다.

        ensure_decel=True: 프로파일이 처음부터 감속 모드로 진입하도록 강제.
          f'''(0) = 6·c3 ≤ 0  ↔  v0_norm ≥ (10 - 4·v1_norm) / 6
          풀면: T ≥ 10·dist / (6·v0_ms + 4·v1_ms)
        """
        if dist < 1e-4:
            return 0.1

        # 표준 quintic (v0=v1=0) 기반 해석적 하한
        #   max vel_norm  = 1.875,  max acc_norm  ≈ 5.774,  max jerk_norm = 60
        T_lo = max(
            1.875 * dist / self.ee_max_vel,
            (5.774 * dist / self.ee_max_acc) ** 0.5,
            (60.0  * dist / self.ee_max_jerk) ** (1.0 / 3.0),
        )

        # 블렌딩 즉시 감속 조건: v0_norm ≥ (10-4·v1_norm)/6
        # → 프로파일 초기 저크 ≤ 0, 처음부터 감속 (속도 급상승 방지)
        if ensure_decel and v0_ms > 1e-3:
            denom = 6.0 * v0_ms + 4.0 * max(v1_ms, 0.0)
            T_lo = max(T_lo, 10.0 * dist / denom)

        taus = np.linspace(0.0, 1.0, 201)

        def _violates(T: float) -> bool:
            v0_n = np.clip(v0_ms * T / dist, 0.0, 2.0)
            v1_n = np.clip(v1_ms * T / dist, 0.0, 2.0)
            c1 = v0_n
            c3 = 10.0 - 6.0 * v0_n - 4.0 * v1_n
            c4 = -15.0 + 8.0 * v0_n + 7.0 * v1_n
            c5 = 6.0 - 3.0 * v0_n - 3.0 * v1_n
            vel_n  = np.abs(c1 + 3*c3*taus**2 + 4*c4*taus**3 + 5*c5*taus**4)
            acc_n  = np.abs(6*c3*taus + 12*c4*taus**2 + 20*c5*taus**3)
            jerk_n = np.abs(6*c3 + 24*c4*taus + 60*c5*taus**2)
            return (
                np.max(vel_n)  * dist / T      > self.ee_max_vel  * 1.001 or
                np.max(acc_n)  * dist / T**2   > self.ee_max_acc  * 1.001 or
                np.max(jerk_n) * dist / T**3   > self.ee_max_jerk * 1.001
            )

        if not _violates(T_lo):
            return T_lo

        # 상한 탐색: 두 배씩 키워서 한계 만족하는 T 찾기
        T_hi = T_lo * 2.0
        while _violates(T_hi):
            T_hi *= 2.0

        # 이진탐색 (25회 → 오차 < T_lo / 2^25 ≈ 무시 가능)
        for _ in range(25):
            T_mid = (T_lo + T_hi) * 0.5
            if _violates(T_mid):
                T_lo = T_mid
            else:
                T_hi = T_mid

        return T_hi

    # ------------------------------------------------------------------
    # Trajectory interface
    # ------------------------------------------------------------------

    def set_trajectory(self, points: list):
        """Load waypoints [(time_from_start_sec, q[6]), ...] and start executing."""
        with self._traj_lock:
            self._traj_points = sorted(points, key=lambda p: p[0])
            self._traj_start  = time.perf_counter()

    @property
    def traj_active(self) -> bool:
        with self._traj_lock:
            return bool(self._traj_points)

    def _traj_step(self):
        with self._traj_lock:
            if not self._traj_points:
                return

            t = time.perf_counter() - self._traj_start

            if t <= self._traj_points[0][0]:
                self.motor_cmd["q_target"] = self._traj_points[0][1].copy()
                return

            if t >= self._traj_points[-1][0]:
                self.motor_cmd["q_target"] = self._traj_points[-1][1].copy()
                self._traj_points = []
                return

            for i in range(len(self._traj_points) - 1):
                t0, q0 = self._traj_points[i]
                t1, q1 = self._traj_points[i + 1]
                if t0 <= t < t1:
                    alpha = self._smooth_alpha((t - t0) / (t1 - t0))
                    self.motor_cmd["q_target"] = q0 + alpha * (q1 - q0)
                    return

    # ------------------------------------------------------------------
    # 오프라인 Pre-solve IK (home 자세에서 출발, 로컬 최솟값 탈출용)
    # ------------------------------------------------------------------

    def _presolve_ik_from_home(self, pos_target: np.ndarray, rot_target: np.ndarray):
        """scipy L-BFGS-B로 홈→목표 IK 오프라인 풀기. 백그라운드 스레드 전용."""
        from scipy.optimize import minimize as _sp_minimize

        cfg = mink.Configuration(self.model, self._home_qpos.copy())
        ee_id = self.ee_body_id
        jnt_lo = self.model.jnt_range[:6, 0]
        jnt_hi = self.model.jnt_range[:6, 1]
        bounds = list(zip(jnt_lo.tolist(), jnt_hi.tolist()))

        def _cost(q_arm):
            q = self._home_qpos.copy()
            q[:6] = q_arm
            cfg.update(q)
            return float(np.linalg.norm(cfg.data.xpos[ee_id] - pos_target) ** 2)

        best_err = 9999.0
        best_q: np.ndarray | None = None

        # 1차: home 자세에서 직접 시작
        res = _sp_minimize(_cost, self._home_qpos[:6].copy(),
                           method="L-BFGS-B", bounds=bounds,
                           options={"maxiter": 500, "ftol": 1e-14})
        q_try = self._home_qpos.copy(); q_try[:6] = res.x; cfg.update(q_try)
        err = float(np.linalg.norm(cfg.data.xpos[ee_id] - pos_target))
        if err < best_err:
            best_err = err; best_q = res.x.copy()

        # 2차: 랜덤 초기화 멀티-재시작 (수렴 실패 시)
        if best_err > 0.02:
            rng = np.random.default_rng(0)
            for _ in range(20):
                q0 = rng.uniform(jnt_lo, jnt_hi)
                res = _sp_minimize(_cost, q0, method="L-BFGS-B", bounds=bounds,
                                   options={"maxiter": 300, "ftol": 1e-12})
                q_try = self._home_qpos.copy(); q_try[:6] = res.x; cfg.update(q_try)
                err = float(np.linalg.norm(cfg.data.xpos[ee_id] - pos_target))
                if err < best_err:
                    best_err = err; best_q = res.x.copy()
                if best_err < 0.02:
                    break

        self._presolve_pending = False

        if best_q is not None and best_err < 0.05:
            # 관절각을 home 기준 가장 가까운 등가각으로 정규화
            diff = best_q - self._home_qpos[:6]
            n = np.round(diff / (2.0 * np.pi))
            normalized = best_q - n * (2.0 * np.pi)
            self._presolve_q       = normalized.copy()
            self._presolve_pos_tgt = pos_target.copy()
            print(f"\n[IK-PreSolve] 수렴  err={best_err:.4f}m"
                  f"  q={np.round(np.degrees(normalized), 1).tolist()}")
        else:
            print(f"\n[IK-PreSolve] 수렴 실패  best_err={best_err:.4f}m")

    def _start_presolve_thread(self, pos_target: np.ndarray, rot_target: np.ndarray):
        """백그라운드에서 pre-solve 실행."""
        self._presolve_q       = None
        self._presolve_pending = True
        self._presolve_pos_tgt = pos_target.copy()
        threading.Thread(
            target=self._presolve_ik_from_home,
            args=(pos_target.copy(), rot_target.copy()),
            daemon=True,
        ).start()

    # ------------------------------------------------------------------
    # Task-space helpers
    # ------------------------------------------------------------------

    def get_ee_pose(self):
        pos = self.data.xpos[self.ee_body_id].copy()
        rot = self.data.xmat[self.ee_body_id].reshape(3, 3).copy()
        return pos, rot

    @staticmethod
    def _rot_error(R_des, R_cur):
        R_err = R_des @ R_cur.T
        return np.array([
            R_err[2, 1] - R_err[1, 2],
            R_err[0, 2] - R_err[2, 0],
            R_err[1, 0] - R_err[0, 1],
        ]) * 0.5

    def _ik_step(self):
        pos_cur, rot_cur = self.get_ee_pose()

        # 목표 위치 결정 (quintic profile 진행 중이면 중간 값, 아니면 static target)
        if self._ik_cmd_duration > 0.0:
            elapsed = time.perf_counter() - self._ik_cmd_start_time
            t_norm  = np.clip(elapsed / self._ik_cmd_duration, 0.0, 1.0)
            if self._ik_v0_norm != 0.0 or self._ik_v1_norm != 0.0:
                alpha = self._smooth_alpha_blend(t_norm, self._ik_v0_norm, self._ik_v1_norm)
            else:
                alpha = self._smooth_alpha(t_norm)
            pos_ref = self._ik_prof_pos_start + alpha * (self._ik_prof_pos_end - self._ik_prof_pos_start)
            rot_ref = self._ik_prof_rot_start + alpha * (self._ik_prof_rot_end - self._ik_prof_rot_start)
            U, _, Vt = np.linalg.svd(rot_ref)
            rot_ref = U @ Vt
            if t_norm >= 1.0:
                self._ik_cmd_duration = 0.0
                self._ik_v0_norm = self._ik_v1_norm = 0.0
        else:
            pos_ref = self.task_cmd["pos_target"]
            rot_ref = self.task_cmd["rot_target"]

        dist_to_target = float(np.linalg.norm(pos_cur - pos_ref))

        # ── Case 1: pre-solve 완료 → 현재 EE가 타겟에서 멀면 joint 명령 유지 ──
        presolve_q = self._presolve_q
        dist_to_presolve = float(np.linalg.norm(self._presolve_pos_tgt - pos_cur))
        if presolve_q is not None and dist_to_presolve > 0.02:
            self.motor_cmd["q_target"] = presolve_q.copy()
            return

        # ── Case 2: pre-solve 진행 중 → 현재 자세 유지 (velocity IK 비활성) ──
        if self._presolve_pending and dist_to_presolve > 0.10:
            # motor_cmd["q_target"] 그대로 유지 (arm hold)
            return

        # ── Case 3: velocity IK (근거리 fine-tuning 또는 pre-solve 실패 폴백) ─
        self._mink_skip = (self._mink_skip + 1) % 5
        if self._mink_skip != 0:
            return

        _ik_dt = self.dt * 5
        self._mink_cfg.update(self.data.qpos.copy())

        if dist_to_target > 0.10:
            self._mink_ee_task.orientation_cost = 0.1
        elif dist_to_target > 0.05:
            self._mink_ee_task.orientation_cost = 0.3
        else:
            self._mink_ee_task.orientation_cost = 0.5

        target_se3 = mink.SE3.from_rotation_and_translation(
            mink.SO3.from_matrix(rot_ref), pos_ref
        )
        self._mink_ee_task.set_target(target_se3)
        vel = mink.solve_ik(self._mink_cfg, self._mink_tasks, dt=_ik_dt,
                            solver="daqp", limits=self._mink_limits, safety_break=False)
        q_new = self._mink_cfg.integrate(vel, _ik_dt)
        self.motor_cmd["q_target"] = q_new[:6]

    # ------------------------------------------------------------------
    # ROS2 subscription
    # ------------------------------------------------------------------

    def start_ros_node(self, node_name: str = "ur5e_rt_controller"):
        """ROS2 노드를 초기화하고 백그라운드 스레드에서 spin."""
        import rclpy
        from std_msgs.msg import Bool, Float64
        from geometry_msgs.msg import PoseStamped
        from trajectory_msgs.msg import JointTrajectory

        try:
            rclpy.init()
        except RuntimeError:
            pass  # already initialized

        self._ros_node = rclpy.create_node(node_name)

        self._ros_node.create_subscription(
            JointTrajectory, "/ur5e/cmd/joint_trajectory",
            self._cb_joint_traj, 10
        )
        self._ros_node.create_subscription(
            PoseStamped, "/ur5e/cmd/ee_target",
            self._cb_ee_target, 10
        )
        self._ros_node.create_subscription(
            Float64, "/ur5e/cmd/ee_duration",
            self._cb_ee_duration, 10
        )
        self._ros_node.create_subscription(
            Bool, "/ur5e/cmd/mode",
            self._cb_mode, 10
        )
        self._ros_node.create_subscription(
            Bool, "/ur5e/cmd/collision_reset",
            self._cb_collision_reset, 10
        )

        from std_msgs.msg import String as StringMsg
        self._ros_node.create_subscription(
            StringMsg, "/ur5e/cmd/soft_limits",
            self._cb_soft_limits, 10
        )
        self._ros_node.create_subscription(
            Float64, "/ur5e/cmd/gripper",
            self._cb_gripper, 10
        )
        self._ros_node.create_subscription(
            Bool, "/ur5e/cmd/antislip",
            self._cb_antislip, 10
        )
        self._ros_node.create_subscription(
            Bool, "/ur5e/cmd/regrasp",
            self._cb_regrasp, 10
        )

        from rclpy.executors import SingleThreadedExecutor
        _executor = SingleThreadedExecutor()
        _executor.add_node(self._ros_node)
        t = threading.Thread(target=_executor.spin, daemon=True)
        t.start()
        print("[Controller] ROS2 노드 시작 완료")

    def _cb_joint_traj(self, msg):
        """trajectory_msgs/JointTrajectory → set_trajectory()"""
        points = []
        for pt in msg.points:
            t = pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9
            q = np.array(pt.positions[:6], dtype=float)
            points.append((t, q))

        if not points:
            return

        # 단일 웨이포인트면 현재 위치에서 보간
        if len(points) == 1:
            t_end, q_end = points[0]
            q_start = self.motor_state["q"].copy()
            points = [(0.0, q_start), (t_end, q_end)]

        self.set_trajectory(points)
        self.task_cmd["use_task_space"] = False  # joint-space 우선
        self.ros_cmd_active = True
        print(f"\n[ROS2] 궤적 수신: {len(points)}개 웨이포인트")

    def _cb_ee_target(self, msg):
        """geometry_msgs/PoseStamped → task_cmd 갱신
        header.stamp에 duration이 인코딩된 경우(sec+nanosec > 0) quintic 프로파일 활성화.
        그 외에는 _ros_ee_duration fallback 사용.
        """
        pos = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
        ])
        R = _quat_to_rotmat(
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w,
        )

        # duration 추출: header.stamp에 인코딩된 값 우선, fallback으로 _ros_ee_duration
        stamp_duration = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        duration = stamp_duration if stamp_duration > 0.0 else self._ros_ee_duration
        self._ros_ee_duration = 0.0  # 한 번 소비 후 초기화

        # vel_start / vel_end 추출: header.frame_id = "base|vel_start|vel_end" 형식
        vel_start_ms = 0.0
        vel_end_ms   = 0.0
        try:
            parts = msg.header.frame_id.split('|')
            if len(parts) >= 2:
                vel_start_ms = float(parts[1])
            if len(parts) >= 3:
                vel_end_ms   = float(parts[2])
        except (ValueError, IndexError):
            pass

        # quintic 보간 프로파일: 현재 위치를 시작점으로 저장
        cur_pos, cur_rot = self.get_ee_pose()
        dist = np.linalg.norm(pos - cur_pos)

        # ── 연속 블렌딩: 이동 중에 새 명령이 오면 현재 속도를 초기조건으로 자동 적용 ──
        is_blend = False
        if self._ik_cmd_duration > 0.0 and dist > 0.001:
            elapsed_now = time.perf_counter() - self._ik_cmd_start_time
            t_now = np.clip(elapsed_now / self._ik_cmd_duration, 0.0, 1.0)

            # 현재 프로파일의 정규화 속도 미분 f'(τ)
            v0_n, v1_n = self._ik_v0_norm, self._ik_v1_norm
            c1 = v0_n
            c3 = 10.0 - 6.0*v0_n - 4.0*v1_n
            c4 = -15.0 + 8.0*v0_n + 7.0*v1_n
            c5 = 6.0 - 3.0*v0_n - 3.0*v1_n
            dalpha = c1 + 3*c3*t_now**2 + 4*c4*t_now**3 + 5*c5*t_now**4

            # 실제 EE 속도 벡터 (m/s) = f'(τ) × Δpos / duration
            cur_vel = dalpha * (self._ik_prof_pos_end - self._ik_prof_pos_start) / self._ik_cmd_duration

            # 새 이동 방향으로 투영 → 그 성분을 출발 속도로 사용 (역방향은 0)
            new_dir = (pos - cur_pos) / dist
            vel_start_ms = max(0.0, float(np.dot(cur_vel, new_dir)))
            is_blend = True  # 즉시 감속 조건 활성화

        # 한계를 만족하는 최소 duration 계산 후, 사용자 요청값과 비교해 큰 쪽 사용
        # is_blend=True 이면 ensure_decel 조건 추가 → 프로파일 초기 가속 방지
        T_min = self._compute_min_duration(dist, vel_start_ms, vel_end_ms,
                                           ensure_decel=is_blend)
        if duration <= 0.0:
            duration = T_min
        elif duration < T_min:
            print(f"\n[Controller] duration 연장: {duration:.2f}s → {T_min:.2f}s "
                  f"(vel/acc/jerk 한계 적용)")
            duration = T_min
        if dist > 0.001:
            # 정규화 속도: v_norm = v(m/s) * duration(s) / dist(m), 범위 [0, 2]
            v0 = float(np.clip(vel_start_ms * duration / dist, 0.0, 2.0))
            v1 = float(np.clip(vel_end_ms   * duration / dist, 0.0, 2.0))
            self._ik_v0_norm = v0
            self._ik_v1_norm = v1
            self._ik_prof_pos_start = cur_pos.copy()
            self._ik_prof_pos_end   = pos.copy()
            self._ik_prof_rot_start = cur_rot.copy()
            self._ik_prof_rot_end   = R.copy()
            self._ik_cmd_start_time = time.perf_counter()  # 반드시 duration 앞에
            self._ik_cmd_duration   = duration  # 가드: 마지막에 설정
        else:
            self._ik_cmd_duration = 0.0
            self._ik_v0_norm = 0.0
            self._ik_v1_norm = 0.0

        self.task_cmd["pos_target"] = pos
        self.task_cmd["rot_target"] = R
        self.task_cmd["use_task_space"] = True
        self.ros_cmd_active = True
        print(f"\n[ROS2] EE 타겟 수신: pos={np.round(pos, 3)}, duration={self._ik_cmd_duration:.1f}s"
              f", v0={self._ik_v0_norm:.2f}, v1={self._ik_v1_norm:.2f}")

        # home 자세에서 pre-solve 시작 (dist > 0.1m 인 원거리 타겟만)
        if dist > 0.10:
            self._start_presolve_thread(pos, R)

    def _cb_ee_duration(self, msg):
        """std_msgs/Float64 → 다음 EE 명령의 이동 시간(초) 저장"""
        self._ros_ee_duration = float(msg.data)

    def _cb_mode(self, msg):
        """std_msgs/Bool → task-space / joint-space 모드 전환"""
        self.task_cmd["use_task_space"] = bool(msg.data)
        self.ros_cmd_active = bool(msg.data)
        mode_str = "task-space" if msg.data else "joint-space"
        print(f"\n[ROS2] 모드 변경: {mode_str}")

    def _cb_collision_reset(self, msg):
        """std_msgs/Bool → 충돌 정지 해제 (data=true 일 때만)"""
        if msg.data and self.collision_detected:
            self.reset_collision()

    def _cb_soft_limits(self, msg):
        """std_msgs/String → 소프트 리밋 설정 갱신 (JSON 페이로드).
        예: '{"enabled":true,"margin":0.05,"kp":200.0,"kd":20.0}'
        모든 필드는 선택적이며, 지정한 필드만 갱신된다."""
        try:
            cfg = json.loads(msg.data)
        except Exception:
            print(f"\n[Controller] soft_limits 파싱 실패: {msg.data!r}")
            return
        changed = []
        if "enabled" in cfg:
            self.soft_limit_enabled = bool(cfg["enabled"])
            changed.append(f"enabled={self.soft_limit_enabled}")
        if "margin" in cfg:
            self.soft_limit_margin = float(np.clip(cfg["margin"], 0.0, 0.4))
            changed.append(f"margin={self.soft_limit_margin:.3f}")
            self._update_soft_limits()
        if "kp" in cfg:
            self.soft_limit_kp = float(np.clip(cfg["kp"], 0.0, 2000.0))
            changed.append(f"kp={self.soft_limit_kp:.1f}")
        if "kd" in cfg:
            self.soft_limit_kd = float(np.clip(cfg["kd"], 0.0, 200.0))
            changed.append(f"kd={self.soft_limit_kd:.1f}")
        if changed:
            print(f"\n[Controller] 소프트 리밋 갱신: {', '.join(changed)}")

    def _cb_gripper(self, msg):
        """std_msgs/Float64 → 그리퍼 개폐 (0.0=열림, 1.0=닫힘)"""
        self.set_gripper(msg.data)
        print(f"\n[ROS2] 그리퍼: {msg.data:.2f} (ctrl={self._gripper_ctrl:.0f})")

    def _cb_antislip(self, msg):
        """std_msgs/Bool → Anti-Slip Reflex 활성화/비활성화"""
        self.antislip_enabled = bool(msg.data)
        state = "ON" if self.antislip_enabled else "OFF"
        print(f"\n[ROS2] Anti-Slip Reflex: {state}")

    def _cb_regrasp(self, msg):
        """std_msgs/Bool → ReGrasp Reflex 활성화/비활성화"""
        self.regrasp_enabled = bool(msg.data)
        state = "ON" if self.regrasp_enabled else "OFF"
        print(f"\n[ROS2] ReGrasp Reflex: {state}")

    # ------------------------------------------------------------------
    # Soft joint limits
    # ------------------------------------------------------------------

    def _update_soft_limits(self):
        """margin 변경 시 소프트 리밋 경계값을 재계산한다."""
        jnt_range = self.model.jnt_range[:6]
        span = jnt_range[:, 1] - jnt_range[:, 0]
        margin = span * np.clip(self.soft_limit_margin, 0.0, 0.4)
        self._soft_lo = jnt_range[:, 0] + margin
        self._soft_hi = jnt_range[:, 1] - margin

    def _soft_limit_torque(self, q: np.ndarray, dq: np.ndarray) -> np.ndarray:
        """소프트 리밋 영역(hard limit 쪽 margin) 진입 시 반발 토크를 반환한다.

        토크 구성:
          반발 강성:  -kp × (q - boundary)          — 경계 밖으로 밀어내는 힘
          속도 감쇠:  -kd × dq  (한계 방향 속도만)  — 접근 속도 감속

        q_target 도 소프트 한계 내로 클램프해 PD 컨트롤러가
        한계 바깥을 목표로 삼지 않도록 막는다.
        """
        tau = np.zeros(6)
        for i in range(6):
            if q[i] > self._soft_hi[i]:
                penetration = q[i] - self._soft_hi[i]
                tau[i] = -self.soft_limit_kp * penetration
                if dq[i] > 0:          # 한계 방향으로 움직일 때만 감쇠
                    tau[i] -= self.soft_limit_kd * dq[i]
            elif q[i] < self._soft_lo[i]:
                penetration = q[i] - self._soft_lo[i]
                tau[i] = -self.soft_limit_kp * penetration
                if dq[i] < 0:          # 한계 방향으로 움직일 때만 감쇠
                    tau[i] -= self.soft_limit_kd * dq[i]
        return tau

    # ------------------------------------------------------------------
    # Collision detection
    # ------------------------------------------------------------------

    def _build_robot_body_ids(self) -> set:
        """EE에서 루트까지 부모 링크를 역추적해 로봇 링크 body ID 집합을 반환한다.
        부산물로 self._robot_base_body_id (체인 최상단 링크 id)를 저장한다."""
        ids = set()
        bid = self.ee_body_id
        last = bid
        while bid > 0:
            ids.add(bid)
            last = bid
            bid = int(self.model.body_parentid[bid])
        self._robot_base_body_id: int = last  # world(0)에 직접 연결된 베이스 링크
        return ids

    def _build_gripper_body_ids(self) -> set:
        """gripper/ 프리픽스를 가진 body ID 집합을 반환한다.
        그리퍼는 물체와 정상 접촉하므로 외부 충돌 반발 대상에서 제외한다."""
        ids = set()
        for bid in range(self.model.nbody):
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, bid) or ""
            if "gripper/" in name:
                ids.add(bid)
        return ids

    def _check_collision(self) -> bool:
        """mj_step() 이후 활성 접촉을 검사해 충돌을 감지한다.

        판정 기준:
          - 접촉하는 두 geom 중 하나 이상이 로봇 링크에 속함
          - contact.dist < 0 (실제 침투 발생, 단순 근접 접촉 제외)

        충돌 유형:
          - self_collision  : 두 geom 모두 로봇 링크
          - object_collision: 로봇 링크 ↔ 외부 물체(바닥, 동적 오브젝트 등)
        """
        for i in range(self.data.ncon):
            c = self.data.contact[i]
            if c.dist >= 0.0:
                continue
            b1 = int(self.model.geom_bodyid[c.geom1])
            b2 = int(self.model.geom_bodyid[c.geom2])
            # 베이스링크↔바닥(world body=0) 접촉만 무시 (정상 마운트 접촉)
            base = self._robot_base_body_id
            if (b1 == 0 and b2 == base) or (b2 == 0 and b1 == base):
                continue
            in_robot1 = b1 in self._robot_body_ids
            in_robot2 = b2 in self._robot_body_ids
            if not (in_robot1 or in_robot2):
                continue
            n1 = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, b1) or f"body{b1}"
            n2 = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, b2) or f"body{b2}"
            self.collision_info = {
                "type": "self_collision" if (in_robot1 and in_robot2) else "object_collision",
                "body1": n1,
                "body2": n2,
                "dist": round(float(c.dist), 5),
                "pos":  [round(float(v), 4) for v in c.pos],
            }
            return True
        return False

    def _set_robot_geom_margins(self):
        """로봇 링크 geom의 contact margin을 설정해 충돌 전 근접 접촉을 감지한다."""
        for i in range(self.model.ngeom):
            if self.model.geom_bodyid[i] in self._robot_body_ids:
                self.model.geom_margin[i] = self._self_col_margin

    def _repulsion_torque(self) -> np.ndarray:
        """로봇 arm 링크의 근접 접촉에서 반발 토크를 계산한다.

        dist < margin 인 접촉에 대해:
          - 반발 강성: kp × (margin - dist)
          - 속도 감쇠: kd × 접근속도  (접근할 때만)

        처리 유형:
          - self-collision  : arm ↔ arm  (인접 링크 제외)
          - object-collision: arm ↔ 외부 물체 (그리퍼·베이스↔바닥 제외)
        """
        tau = np.zeros(6)
        nv  = self.model.nv

        for i in range(self.data.ncon):
            c  = self.data.contact[i]
            if c.dist >= self._self_col_margin:
                continue

            b1   = int(self.model.geom_bodyid[c.geom1])
            b2   = int(self.model.geom_bodyid[c.geom2])
            arm1 = b1 in self._robot_body_ids
            arm2 = b2 in self._robot_body_ids

            if not arm1 and not arm2:
                continue  # 로봇 arm 무관 접촉

            dist   = float(c.dist)
            normal = c.frame[:3].copy()  # geom2 → geom1 방향 단위벡터
            f_mag  = self.self_collision_kp * (self._self_col_margin - dist)

            if arm1 and arm2:
                # ── Self-collision: arm ↔ arm ─────────────────────────────
                p1 = int(self.model.body_parentid[b1])
                p2 = int(self.model.body_parentid[b2])
                if p1 == b2 or p2 == b1:
                    continue  # 인접 링크(관절 연결부) 제외

                jacp1 = np.zeros((3, nv))
                jacp2 = np.zeros((3, nv))
                mujoco.mj_jac(self.model, self.data, jacp1, None, c.pos, b1)
                mujoco.mj_jac(self.model, self.data, jacp2, None, c.pos, b2)
                J_rel   = (jacp1 - jacp2)[:, :6]
                v_close = -float(normal @ J_rel @ self.motor_state["dq"])
                if v_close > 0.0:
                    f_mag += self.self_collision_kd * v_close

                F    = f_mag * normal
                tau += jacp1[:, :6].T @ F
                tau -= jacp2[:, :6].T @ F

            else:
                # ── Object-collision: arm ↔ 외부 물체 ────────────────────
                if arm1:
                    arm_body, arm_normal, ext_body = b1, normal, b2
                else:
                    arm_body, arm_normal, ext_body = b2, -normal, b1

                # 그리퍼 body ↔ arm 접촉은 정상 근접이므로 제외
                if ext_body in self._gripper_body_ids:
                    continue
                # 베이스 링크 ↔ world(바닥) 마운트 접촉 제외
                if ext_body == 0 and arm_body == self._robot_base_body_id:
                    continue

                jacp_arm = np.zeros((3, nv))
                jacp_ext = np.zeros((3, nv))
                mujoco.mj_jac(self.model, self.data, jacp_arm, None, c.pos, arm_body)
                if ext_body > 0:  # 동적 외부 물체: jacobian 계산
                    mujoco.mj_jac(self.model, self.data, jacp_ext, None, c.pos, ext_body)

                J_rel   = (jacp_arm - jacp_ext)[:, :6]
                v_close = -float(arm_normal @ J_rel @ self.motor_state["dq"])
                if v_close > 0.0:
                    f_mag += self.self_collision_kd * v_close

                tau += jacp_arm[:, :6].T @ (f_mag * arm_normal)

        return np.clip(tau, -500.0, 500.0)

    def set_gripper(self, value: float):
        """그리퍼 개폐 명령. value: 0.0=완전 열림, 1.0=완전 닫힘"""
        t = float(np.clip(value, 0.0, 1.0))
        self._gripper_ctrl = self._gripper_ctrl_open + t * (
            self._gripper_ctrl_close - self._gripper_ctrl_open
        )

    def reset_collision(self):
        """충돌 정지 해제: 충돌 직전 안전 위치로 후퇴 후 정상 제어로 복귀한다."""
        # 버퍼에 안전 위치가 있으면 가장 오래된 것(충돌 ~0.1s 전)으로 후퇴
        if self._safe_q_buffer:
            self._escape_target_q = self._safe_q_buffer[0].copy()
        else:
            self._escape_target_q = self._freeze_q.copy()

        # 모션 명령 초기화
        self._ik_cmd_duration = 0.0
        self._ik_v0_norm = 0.0
        self._ik_v1_norm = 0.0
        with self._traj_lock:
            self._traj_points = []
        self.task_cmd["use_task_space"] = False
        self.ros_cmd_active = False
        self.state = ControlState.COLLISION_ESCAPE
        print("\n[Controller] 충돌 정지 해제 → COLLISION_ESCAPE"
              f"  target_q={np.round(np.degrees(self._escape_target_q), 1)}")

    # ------------------------------------------------------------------
    # Real-time control loop
    # ------------------------------------------------------------------

    def controller_run(self):
        self.motor_state["q"]  = self.data.qpos[:6].copy()
        self.motor_state["dq"] = self.data.qvel[:6].copy()

        # 우선순위: trajectory > task-space IK > joint hold
        if self.traj_active:
            self._traj_step()
        elif self.task_cmd["use_task_space"]:
            self._ik_step()

        q   = self.motor_state["q"]
        dq  = self.motor_state["dq"]

        # 소프트 리밋: q_target을 경계 내로 클램프
        q_des = self.motor_cmd["q_target"].copy()

        # UR5e menagerie는 위치 액추에이터 (kp=2000, kd=400 내장).
        # ctrl = q_des(목표 위치)로 직접 전달하면 내장 PD가 적절한 힘을 계산한다.
        # 이전 방식(토크 계산→ctrl 쓰기)은 위치 액추에이터에서 의도치 않은 double-cascade
        # 효과를 만들어 수렴을 방해했다.
        # 중력 feed-forward: kp*(ctrl-q_actual)=qfrc_bias → q_actual=ctrl-bias/kp
        # feed-forward를 더하면 q_actual≈q_des 달성 → pos_gain 크기에 무관하게 중력 극복
        self.data.ctrl[:6] = q_des + self.data.qfrc_bias[:6] / 2000.0

        # 그리퍼 제어 (actuator index 6, 범위 0~255)
        if self._gripper_act_id >= 0:
            self.data.ctrl[self._gripper_act_id] = self._gripper_ctrl

    def start(self):
        print(f"\n[Controller] 시작 - 현재 상태: {self.state.name}")

        while True:
            t_start = time.perf_counter()

            if self.state == ControlState.WAIT_STABLE:
                time.sleep(1.0)
                self.state = ControlState.INIT_CONTROL

            elif self.state == ControlState.INIT_CONTROL:
                self.motor_cmd["q_target"] = self.data.qpos[:6].copy()
                self.state = ControlState.CHECK_MOTOR

            elif self.state == ControlState.CHECK_MOTOR:
                self.state = ControlState.INIT_POSITION

            elif self.state == ControlState.INIT_POSITION:
                pos0, rot0 = self.get_ee_pose()
                self.task_cmd["pos_target"] = pos0.copy()
                self.task_cmd["rot_target"] = rot0.copy()
                self.state = ControlState.RUN_CONTROL

            elif self.state == ControlState.RUN_CONTROL:
                # 안전 위치 버퍼 갱신 (충돌 감지 전 위치 기록)
                self._safe_q_buffer.append(self.data.qpos[:6].copy())
                if len(self._safe_q_buffer) > self._safe_q_buf_size:
                    self._safe_q_buffer.pop(0)

                self.controller_run()
                mujoco.mj_step(self.model, self.data)
                if self._check_collision():
                    self.collision_detected = True
                    self._freeze_q = self.motor_state["q"].copy()
                    info = self.collision_info
                    print(f"\n[⚠️ 충돌 감지] {info['body1']} ↔ {info['body2']}"
                          f"  dist={info['dist']}  pos={info['pos']}")
                    self.state = ControlState.COLLISION_STOP

            elif self.state == ControlState.COLLISION_STOP:
                self.data.ctrl[:6] = self._freeze_q
                mujoco.mj_step(self.model, self.data)

            elif self.state == ControlState.COLLISION_ESCAPE:
                self.data.ctrl[:6] = self._escape_target_q
                mujoco.mj_step(self.model, self.data)
                # 충돌이 해제되면 정상 제어로 복귀
                if not self._check_collision():
                    self.collision_detected = False
                    self.collision_info = {}
                    self.motor_cmd["q_target"] = self._escape_target_q.copy()
                    self._safe_q_buffer.clear()
                    self.state = ControlState.RUN_CONTROL
                    print("\n[Controller] 충돌 해제 완료 → RUN_CONTROL")

            # 10 Hz로 상태 파일 갱신 (모든 상태에서 실행)
            self._state_write_counter += 1
            if self._state_write_counter >= 50:
                self._state_write_counter = 0
                self._write_state()

            t_elapsed = time.perf_counter() - t_start
            t_sleep = self.dt - t_elapsed
            if t_sleep > 0:
                time.sleep(t_sleep)
            else:
                self.miss_count += 1
