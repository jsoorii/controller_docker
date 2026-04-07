import numpy as np
import time
import mujoco
from enum import Enum

class ControlState(Enum):
    WAIT_STABLE   = 0
    INIT_CONTROL  = 1
    CHECK_MOTOR   = 2
    INIT_POSITION = 3
    RUN_CONTROL   = 4

class UR5eRTController:
    def __init__(self, model, data, ee_body_name="wrist_3_link"):
        self.model = model
        self.data = data
        self.state = ControlState.WAIT_STABLE

        # Resolve end-effector body ID
        body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, ee_body_name)
        if body_id == -1:
            # Fall back to last body in the chain
            body_id = model.nbody - 1
            print(f"[Controller] Warning: '{ee_body_name}' not found, using body id={body_id}")
        self.ee_body_id = body_id

        # Joint-space state & command (SRAM)
        self.motor_state = {"q": np.zeros(6), "dq": np.zeros(6)}
        self.motor_cmd   = {"q_target": np.zeros(6)}

        # Task-space command
        self.task_cmd = {
            "pos_target": np.zeros(3),   # Desired end-effector position [x, y, z]
            "rot_target": np.eye(3),     # Desired end-effector rotation matrix (3x3)
            "use_task_space": False,     # Switch: True = task-space, False = joint-space
        }

        # IK parameters (damped least squares)
        self._ik_pos_gain  = 5.0
        self._ik_rot_gain  = 2.0
        self._ik_lambda_sq = 1e-4   # Damping factor
        self._ik_max_dq    = 0.1    # Max joint delta per step (rad)

        # PD gains for joint-space control
        self.kp = 500.0
        self.kd = 50.0

        self.loop_hz = 500  # 2 ms 주기
        self.dt = 1.0 / self.loop_hz
        self.miss_count = 0

    # ------------------------------------------------------------------
    # Task-space helpers
    # ------------------------------------------------------------------

    def get_ee_pose(self):
        """Return current EE position and rotation matrix."""
        pos = self.data.xpos[self.ee_body_id].copy()
        rot = self.data.xmat[self.ee_body_id].reshape(3, 3).copy()
        return pos, rot

    @staticmethod
    def _rot_error(R_des, R_cur):
        """Orientation error as axis-angle vector (3-vector)."""
        R_err = R_des @ R_cur.T
        # skew-symmetric part -> axis * sin(angle)
        err = np.array([
            R_err[2, 1] - R_err[1, 2],
            R_err[0, 2] - R_err[2, 0],
            R_err[1, 0] - R_err[0, 1],
        ]) * 0.5
        return err

    def _ik_step(self):
        """
        One IK iteration using the body Jacobian (damped least squares).
        Updates motor_cmd['q_target'] in-place.
        """
        pos_cur, rot_cur = self.get_ee_pose()

        # Task-space error (6-vector: position + orientation)
        pos_err = self.task_cmd["pos_target"] - pos_cur
        rot_err = self._rot_error(self.task_cmd["rot_target"], rot_cur)
        dx = np.concatenate([
            self._ik_pos_gain * pos_err,
            self._ik_rot_gain * rot_err,
        ])

        # Compute full body Jacobian (position + rotation)
        nv = self.model.nv
        jacp = np.zeros((3, nv))
        jacr = np.zeros((3, nv))
        mujoco.mj_jacBody(self.model, self.data, jacp, jacr, self.ee_body_id)
        J = np.vstack([jacp[:, :6], jacr[:, :6]])  # 6 x 6 (first 6 DOF)

        # Damped least squares: dq = J^T (J J^T + lam^2 I)^-1 dx
        A = J @ J.T + self._ik_lambda_sq * np.eye(6)
        dq = J.T @ np.linalg.solve(A, dx)

        # Clamp and apply
        dq = np.clip(dq, -self._ik_max_dq, self._ik_max_dq)
        self.motor_cmd["q_target"] = self.motor_state["q"] + dq

    # ------------------------------------------------------------------
    # Real-time control loop
    # ------------------------------------------------------------------

    def controller_run(self):
        """[RT] 최우선 제어 루프"""
        # 1. 현재 상태 읽기
        self.motor_state["q"]  = self.data.qpos[:6].copy()
        self.motor_state["dq"] = self.data.qvel[:6].copy()

        # 2. Task-space IK (optional)
        if self.task_cmd["use_task_space"]:
            self._ik_step()

        # 3. PD 제어 (joint-space)
        q_des = self.motor_cmd["q_target"]
        tau_out = (
            self.kp * (q_des - self.motor_state["q"])
            + self.kd * (0.0 - self.motor_state["dq"])
        )

        # 4. 명령 송신
        self.data.ctrl[:6] = tau_out

    def start(self):
        print(f"\n[Controller] 시작 - 현재 상태: {self.state.name}")

        while True:
            t_start = time.perf_counter()

            # --- State Machine ---
            if self.state == ControlState.WAIT_STABLE:
                time.sleep(1.0)
                self.state = ControlState.INIT_CONTROL

            elif self.state == ControlState.INIT_CONTROL:
                self.motor_cmd["q_target"] = self.data.qpos[:6].copy()
                self.state = ControlState.CHECK_MOTOR

            elif self.state == ControlState.CHECK_MOTOR:
                self.state = ControlState.INIT_POSITION

            elif self.state == ControlState.INIT_POSITION:
                # Capture initial EE pose as default task target
                pos0, rot0 = self.get_ee_pose()
                self.task_cmd["pos_target"] = pos0.copy()
                self.task_cmd["rot_target"] = rot0.copy()
                self.state = ControlState.RUN_CONTROL

            elif self.state == ControlState.RUN_CONTROL:
                self.controller_run()
                mujoco.mj_step(self.model, self.data)

            # --- 2ms 주기 유지 ---
            t_elapsed = time.perf_counter() - t_start
            t_sleep = self.dt - t_elapsed
            if t_sleep > 0:
                time.sleep(t_sleep)
            else:
                self.miss_count += 1
