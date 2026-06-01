"""
ur5e_controller.py — mink 기반 task-space EE 제어기
- mink.solve_ik()로 desired EE pos/rot → joint velocities 계산
- 바닥-팔 충돌 회피: mink.CollisionAvoidanceLimit
- 하드 충돌 감지: contact dist < 0 → COLLISION_STOP
"""
import json
import os
import threading
import time

import mujoco
import mink
import numpy as np
from enum import Enum


class ControlState(Enum):
    IDLE            = 0
    RUN_CONTROL     = 1
    COLLISION_STOP  = 2


def _quat_to_rotmat(x, y, z, w) -> np.ndarray:
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-w*z),   2*(x*z+w*y)],
        [2*(x*y+w*z),   1-2*(x*x+z*z), 2*(y*z-w*x)],
        [2*(x*z-w*y),   2*(y*z+w*x),   1-2*(x*x+y*y)],
    ])


def rotmat_axis_angle(axis, angle_deg: float) -> np.ndarray:
    """Rodrigues 공식: 임의 축 + 각도(도) → 3×3 회전행렬.

    axis: np.ndarray(3,) — 회전축 벡터 (자동 정규화)
    """
    a = np.radians(angle_deg)
    k = np.asarray(axis, dtype=float)
    k = k / np.linalg.norm(k)
    K = np.array([[    0, -k[2],  k[1]],
                  [ k[2],     0, -k[0]],
                  [-k[1],  k[0],     0]])
    return np.eye(3) + np.sin(a) * K + (1.0 - np.cos(a)) * (K @ K)


class UR5eController:
    def __init__(self, model, data,
                 ee_body_name: str = "wrist_3_link",
                 gripper_cfg=None):
        self.model = model
        self.data  = data
        self.state = ControlState.IDLE
        self.dt    = 1.0 / 500          # 500 Hz 시뮬레이션
        self.miss_count = 0

        # ── EE body ───────────────────────────────────────────────────
        ee_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, ee_body_name)
        self.ee_body_id   = ee_id if ee_id >= 0 else model.nbody - 1
        self.ee_body_name = ee_body_name

        # ── mink configuration ────────────────────────────────────────
        gripper_dofs = list(range(6, model.nv))
        self._cfg = mink.Configuration(model, data.qpos.copy())

        self._ee_task = mink.FrameTask(
            ee_body_name, "body",
            position_cost=1.0,
            orientation_cost=0.5,
        )
        self._tasks = [self._ee_task, mink.DampingTask(model, cost=1e-3)]
        if gripper_dofs:
            self._tasks.append(mink.DofFreezingTask(model, gripper_dofs))

        # ── 충돌 회피 limits: 바닥 ↔ 팔 링크 ─────────────────────────
        floor_geoms = [
            gi for gi in range(model.ngeom)
            if (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY,
                                  int(model.geom_bodyid[gi])) or "") == "world"
        ]
        arm_kw = ("shoulder", "upper_arm", "forearm", "wrist")
        arm_geoms = [
            gi for gi in range(model.ngeom)
            if any(kw in (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY,
                                            int(model.geom_bodyid[gi])) or "")
                   for kw in arm_kw)
        ]
        self._limits = [mink.VelocityLimit(model)]
        if floor_geoms and arm_geoms:
            self._limits.append(mink.CollisionAvoidanceLimit(
                model, [(floor_geoms, arm_geoms)],
                minimum_distance_from_collisions=0.015,
                collision_detection_distance=0.08,
            ))

        # IK는 5 sim-step마다 1회 (100 Hz)
        self._ik_dt   = self.dt * 5
        self._ik_skip = 0

        # ── 현재 joint target (위치 액추에이터 입력) ──────────────────
        self._q_target = data.qpos[:6].copy()

        # ── task-space target ──────────────────────────────────────────
        self._target_pos: np.ndarray | None = None
        self._target_rot: np.ndarray | None = None
        self._target_lock = threading.Lock()

        # main_test.py 모니터링용 노출 dict (읽기 전용으로 사용)
        self.task_cmd = {
            "pos_target": np.zeros(3),
            "rot_target": np.eye(3),
        }

        # ── 그리퍼 ────────────────────────────────────────────────────
        if gripper_cfg is not None:
            act = f"gripper/{gripper_cfg.actuator}"
            self._gripper_act_id    = mujoco.mj_name2id(
                model, mujoco.mjtObj.mjOBJ_ACTUATOR, act)
            self._gripper_ctrl_open  = float(gripper_cfg.ctrl_open)
            self._gripper_ctrl_close = float(gripper_cfg.ctrl_close)
        else:
            self._gripper_act_id     = -1
            self._gripper_ctrl_open  = 0.0
            self._gripper_ctrl_close = 1.0
        self._gripper_ctrl = self._gripper_ctrl_open

        # RobotiqGraspAdapter 참조용 플래그
        self.antislip_enabled  = True
        self.regrasp_enabled   = False

        # ── 충돌 감지 ─────────────────────────────────────────────────
        self._robot_body_ids, self._gripper_body_ids, self._base_body_id = \
            self._build_robot_body_ids()
        self._all_robot_ids = self._robot_body_ids | self._gripper_body_ids
        self.collision_detected = False
        self.collision_info: dict = {}
        self._freeze_q = np.zeros(6)

        # ── state file ────────────────────────────────────────────────
        self._state_path    = "/tmp/robot_state.json"
        self._state_counter = 0

        self._ros_node = None

    # ── body ID 헬퍼 ─────────────────────────────────────────────────

    def _build_robot_body_ids(self):
        # 팔 체인: wrist_3 → base (운동학 체인 역방향 탐색)
        arm_ids = set()
        bid = self.ee_body_id
        last = bid
        while bid > 0:
            arm_ids.add(bid)
            last = bid
            bid = int(self.model.body_parentid[bid])

        # 그리퍼 body: 이름 패턴으로 식별
        gripper_kw = ("gripper", "robotiq", "finger", "pad",
                      "knuckle", "coupler", "driver", "follower", "silicone")
        gripper_ids = set()
        for i in range(self.model.nbody):
            name = (mujoco.mj_id2name(
                self.model, mujoco.mjtObj.mjOBJ_BODY, i) or "").lower()
            if any(kw in name for kw in gripper_kw):
                gripper_ids.add(i)

        return arm_ids, gripper_ids, last

    # ── 외부 인터페이스 ───────────────────────────────────────────────

    def set_ee_target(self, pos: np.ndarray, rot: np.ndarray | None = None):
        """task-space EE 목표 설정. rot=None이면 현재 방향 유지."""
        with self._target_lock:
            self._target_pos = np.asarray(pos, dtype=float)
            self._target_rot = rot
        self.task_cmd["pos_target"] = np.asarray(pos, dtype=float)
        if rot is not None:
            self.task_cmd["rot_target"] = rot

    def rotate_ee_target(self,
                         axis,
                         angle_deg: float,
                         frame: str = "ee") -> None:
        """현재 EE 타겟 방향에 추가 회전을 적용한다.

        axis:
          'x' | 'y' | 'z'  → EE 로컬 또는 월드 축 선택 (frame 인수 참조)
          np.ndarray(3,)    → 임의 벡터 (frame 인수 참조)
        angle_deg: 회전 각도 (도, 오른손 법칙)
        frame:
          'ee'    — axis를 현재 EE 로컬 프레임으로 해석 (열벡터 기준)
          'world' — axis를 월드 프레임으로 해석
        """
        # 현재 타겟 pos / rot 스냅샷
        with self._target_lock:
            pos = self._target_pos
            rot = self._target_rot

        # 아직 타겟이 없으면 현재 EE 상태를 기준으로 삼는다
        cur_pos, cur_rot = self.get_ee_pose()
        if pos is None:
            pos = cur_pos
        if rot is None:
            rot = cur_rot

        # 축 벡터 결정
        _axis_map = {'x': 0, 'y': 1, 'z': 2}
        if isinstance(axis, str):
            idx = _axis_map[axis.lower()]
            axis_world = rot[:, idx] if frame == "ee" else np.eye(3)[idx]
        else:
            v = np.asarray(axis, dtype=float)
            axis_world = rot @ v if frame == "ee" else v

        R_delta = rotmat_axis_angle(axis_world, angle_deg)
        R_new   = R_delta @ rot
        self.set_ee_target(pos, R_new)

    def set_gripper(self, value: float):
        """0.0=완전 열림, 1.0=완전 닫힘"""
        t = float(np.clip(value, 0.0, 1.0))
        self._gripper_ctrl = (self._gripper_ctrl_open
                              + t * (self._gripper_ctrl_close - self._gripper_ctrl_open))

    def get_ee_pose(self):
        pos = self.data.xpos[self.ee_body_id].copy()
        rot = self.data.xmat[self.ee_body_id].reshape(3, 3).copy()
        return pos, rot

    def reset_collision(self):
        self.collision_detected = False
        self.collision_info = {}
        self._q_target = self.data.qpos[:6].copy()
        self.state = ControlState.RUN_CONTROL
        print("\n[Controller] 충돌 리셋 → RUN_CONTROL")

    # ── mink IK ───────────────────────────────────────────────────────

    def _ik_step(self):
        self._ik_skip = (self._ik_skip + 1) % 5
        if self._ik_skip != 0:
            return

        with self._target_lock:
            pos = self._target_pos
            rot = self._target_rot

        if pos is None:
            return

        if rot is None:
            _, rot = self.get_ee_pose()

        self._cfg.update(self.data.qpos.copy())
        self._ee_task.set_target(
            mink.SE3.from_rotation_and_translation(mink.SO3.from_matrix(rot), pos)
        )
        vel = mink.solve_ik(
            self._cfg, self._tasks,
            dt=self._ik_dt, solver="daqp",
            limits=self._limits, safety_break=False,
        )
        q_new = self._cfg.integrate(vel, self._ik_dt)
        self._q_target = q_new[:6]

    # ── 충돌 감지 ─────────────────────────────────────────────────────

    def _check_collision(self) -> bool:
        for i in range(self.data.ncon):
            c = self.data.contact[i]
            if c.dist >= 0.0:
                continue
            b1 = int(self.model.geom_bodyid[c.geom1])
            b2 = int(self.model.geom_bodyid[c.geom2])
            # 베이스↔바닥 마운트 접촉 무시
            if ((b1 == 0 and b2 == self._base_body_id) or
                    (b2 == 0 and b1 == self._base_body_id)):
                continue
            # 로봇 내부 접촉(팔↔그리퍼) 무시 — 자연스러운 마운트 접촉
            if b1 in self._all_robot_ids and b2 in self._all_robot_ids:
                continue
            if b1 in self._robot_body_ids or b2 in self._robot_body_ids:
                n1 = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, b1) or f"body{b1}"
                n2 = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, b2) or f"body{b2}"
                self.collision_info = {
                    "body1": n1, "body2": n2,
                    "dist": round(float(c.dist), 5),
                    "pos": [round(float(v), 4) for v in c.pos],
                }
                return True
        return False

    # ── 제어 한 스텝 ───────────────────────────────────────────────────

    def _ctrl_step(self):
        self._ik_step()
        # 위치 액추에이터 + 중력 feed-forward
        self.data.ctrl[:6] = self._q_target + self.data.qfrc_bias[:6] / 2000.0
        if self._gripper_act_id >= 0:
            self.data.ctrl[self._gripper_act_id] = self._gripper_ctrl
        mujoco.mj_step(self.model, self.data)

    # ── state 파일 ────────────────────────────────────────────────────

    def _write_state(self):
        try:
            pos, _ = self.get_ee_pose()
            state = {
                "q":        self.data.qpos[:6].tolist(),
                "dq":       self.data.qvel[:6].tolist(),
                "ee_pos":   pos.tolist(),
                "ts":       time.time(),
                "collision": {
                    "detected": self.collision_detected,
                    "info":     self.collision_info,
                },
                "gripper":  {"ctrl": self._gripper_ctrl},
            }
            tmp = self._state_path + ".tmp"
            with open(tmp, "w") as f:
                json.dump(state, f)
            os.replace(tmp, self._state_path)
        except Exception:
            pass

    # ── ROS2 ─────────────────────────────────────────────────────────

    def start_ros_node(self, node_name: str = "ur5e_controller"):
        import rclpy
        from geometry_msgs.msg import PoseStamped
        from std_msgs.msg import Bool, Float64

        try:
            rclpy.init()
        except RuntimeError:
            pass

        node = rclpy.create_node(node_name)
        self._ros_node = node

        node.create_subscription(
            PoseStamped, "/ur5e/cmd/ee_target", self._cb_ee_target, 10)
        node.create_subscription(
            Float64, "/ur5e/cmd/gripper",
            lambda m: self.set_gripper(float(m.data)), 10)
        node.create_subscription(
            Bool, "/ur5e/cmd/collision_reset",
            lambda m: self.reset_collision() if m.data else None, 10)
        node.create_subscription(
            Bool, "/ur5e/cmd/antislip",
            lambda m: setattr(self, "antislip_enabled", bool(m.data)), 10)

        from rclpy.executors import SingleThreadedExecutor
        ex = SingleThreadedExecutor()
        ex.add_node(node)
        threading.Thread(target=ex.spin, daemon=True).start()
        print("[Controller] ROS2 노드 시작 완료")

    def _cb_ee_target(self, msg):
        pos = np.array([msg.pose.position.x,
                        msg.pose.position.y,
                        msg.pose.position.z])
        R = _quat_to_rotmat(msg.pose.orientation.x,
                            msg.pose.orientation.y,
                            msg.pose.orientation.z,
                            msg.pose.orientation.w)
        self.set_ee_target(pos, R)
        print(f"\n[ROS2] EE 타겟: {np.round(pos, 3)}")

    # ── 메인 루프 ─────────────────────────────────────────────────────

    def start(self):
        print(f"\n[Controller] 시작 - 현재 상태: {self.state.name}")

        # 시작 시 현재 joint 위치를 target으로 초기화
        self._q_target = self.data.qpos[:6].copy()
        self.task_cmd["pos_target"] = self.get_ee_pose()[0].copy()
        self.state = ControlState.RUN_CONTROL

        while True:
            t0 = time.perf_counter()

            if self.state == ControlState.RUN_CONTROL:
                self._ctrl_step()
                if self._check_collision():
                    self.collision_detected = True
                    self._freeze_q = self.data.qpos[:6].copy()
                    info = self.collision_info
                    print(f"\n[충돌 감지] {info['body1']} ↔ {info['body2']}"
                          f"  dist={info['dist']}")
                    self.state = ControlState.COLLISION_STOP

            elif self.state == ControlState.COLLISION_STOP:
                self.data.ctrl[:6] = self._freeze_q
                if self._gripper_act_id >= 0:
                    self.data.ctrl[self._gripper_act_id] = self._gripper_ctrl
                mujoco.mj_step(self.model, self.data)

            # 10 Hz로 state 파일 갱신
            self._state_counter += 1
            if self._state_counter >= 50:
                self._state_counter = 0
                self._write_state()

            elapsed = time.perf_counter() - t0
            sleep = self.dt - elapsed
            if sleep > 0:
                time.sleep(sleep)
            else:
                self.miss_count += 1
