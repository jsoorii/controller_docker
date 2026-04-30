import json
import numpy as np
import os
import time
import threading
import mujoco
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

        # IK parameters (damped least squares)
        self._ik_pos_gain  = 5.0
        self._ik_rot_gain  = 2.0
        self._ik_lambda_sq = 1e-4
        self._ik_max_dq    = 0.1    # max joint delta per step (rad)

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

        # quintic 보간: 시작~끝 위치를 부드럽게 연결한 중간 목표를 IK에 전달
        if self._ik_cmd_duration > 0.0:
            elapsed = time.perf_counter() - self._ik_cmd_start_time
            t_norm  = np.clip(elapsed / self._ik_cmd_duration, 0.0, 1.0)

            # 출발/도착 속도가 있으면 blend 프로파일, 없으면 표준 quintic
            if self._ik_v0_norm != 0.0 or self._ik_v1_norm != 0.0:
                alpha = self._smooth_alpha_blend(t_norm, self._ik_v0_norm, self._ik_v1_norm)
            else:
                alpha = self._smooth_alpha(t_norm)

            pos_ref = self._ik_prof_pos_start + alpha * (self._ik_prof_pos_end - self._ik_prof_pos_start)
            # 회전 보간: slerp 근사 (작은 각도에서 충분)
            rot_ref = self._ik_prof_rot_start + alpha * (self._ik_prof_rot_end - self._ik_prof_rot_start)
            # 정규화
            U, _, Vt = np.linalg.svd(rot_ref)
            rot_ref = U @ Vt

            if t_norm >= 1.0:
                self._ik_cmd_duration = 0.0  # 프로파일 종료
                self._ik_v0_norm = 0.0
                self._ik_v1_norm = 0.0
        else:
            pos_ref = self.task_cmd["pos_target"]
            rot_ref = self.task_cmd["rot_target"]

        pos_err = pos_ref - pos_cur
        dx_pos  = self._ik_pos_gain * pos_err

        rot_err = self._rot_error(rot_ref, rot_cur)
        dx = np.concatenate([dx_pos, self._ik_rot_gain * rot_err])

        nv = self.model.nv
        jacp = np.zeros((3, nv))
        jacr = np.zeros((3, nv))
        mujoco.mj_jacBody(self.model, self.data, jacp, jacr, self.ee_body_id)
        J = np.vstack([jacp[:, :6], jacr[:, :6]])

        A = J @ J.T + self._ik_lambda_sq * np.eye(6)
        dq = J.T @ np.linalg.solve(A, dx)
        dq = np.clip(dq, -self._ik_max_dq, self._ik_max_dq)
        self.motor_cmd["q_target"] = self.motor_state["q"] + dq

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

        # 소프트 리밋: q_target을 경계 내로 클램프 → PD가 한계 바깥을 목표로 삼지 않음
        q_des = self.motor_cmd["q_target"].copy()
        if self.soft_limit_enabled:
            q_des = np.clip(q_des, self._soft_lo, self._soft_hi)

        tau_out = (
            self.kp * (q_des - q)
            + self.kd * (0.0 - dq)
            + self.data.qfrc_bias[:6]   # 중력 + 코리올리 + 원심력 보상
        )

        if self.soft_limit_enabled:
            tau_out += self._soft_limit_torque(q, dq)

        self.data.ctrl[:6] = tau_out

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
                # 감지된 위치에서 관절을 고정해 추가 운동 방지
                q_cur  = self.data.qpos[:6]
                dq_cur = self.data.qvel[:6]
                self.data.ctrl[:6] = (
                    self.kp * (self._freeze_q - q_cur)
                    + self.kd * (0.0 - dq_cur)
                    + self.data.qfrc_bias[:6]
                )
                mujoco.mj_step(self.model, self.data)

            elif self.state == ControlState.COLLISION_ESCAPE:
                # 충돌 직전 안전 위치로 PD 제어하며 후퇴
                q_cur  = self.data.qpos[:6]
                dq_cur = self.data.qvel[:6]
                self.data.ctrl[:6] = (
                    self.kp * (self._escape_target_q - q_cur)
                    + self.kd * (0.0 - dq_cur)
                    + self.data.qfrc_bias[:6]
                )
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
