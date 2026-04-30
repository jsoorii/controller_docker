"""
object_approach_node.py
-----------------------
MuJoCo 씬 내 특정 물체를 향해 UR5e EE를 단계적으로 접근시키는 ROS2 노드.

동작 흐름:
    1. /ur5e/approach/start (String) 토픽으로 물체 이름 수신
    2. MuJoCo data.xpos 에서 물체 월드 위치를 읽음
    3. HOVER → DESCEND 순서로 PoseStamped 를 /ur5e/cmd/ee_target 에 발행
       → 컨트롤러의 기존 _cb_ee_target 이 그대로 처리

토픽:
    Subscribe  /ur5e/approach/start   std_msgs/String    물체 이름
    Subscribe  /mujoco/query_objects  std_msgs/String    온디맨드 씬 쿼리 (빈 문자열=전체, 이름=필터)
    Publish    /ur5e/cmd/ee_target    geometry_msgs/PoseStamped  EE 목표
    Publish    /mujoco/scene_objects  std_msgs/String    씬 내 모든 물체 위치 (JSON)

접근 단계:
    IDLE    : 대기
    HOVER   : 물체 바로 위 hover_height(m) 위치로 이동
    DESCEND : 물체 바로 위 grasp_offset(m) 위치로 하강
    DONE    : 접근 완료 (그리퍼 닫기는 상위 레이어 담당)

사용 예시 (ros2 topic pub):
    ros2 topic pub /ur5e/approach/start std_msgs/String "data: 'box'"
    ros2 topic pub /mujoco/query_objects std_msgs/String "data: ''"       # 전체 조회
    ros2 topic pub /mujoco/query_objects std_msgs/String "data: 'box'"    # 이름 필터
"""

import json
import threading
import time
from enum import Enum

import mujoco
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String


# ---------------------------------------------------------------------------
# 로봇팔 / 씬 구조물 body 판별 패턴 (소문자 부분 일치)
# ---------------------------------------------------------------------------

_ROBOT_BODY_PATTERNS = (
    "world", "floor", "ground", "table",
    "base_link", "shoulder", "upper_arm", "forearm",
    "wrist", "hand", "link", "robotiq", "finger",
    "pad", "knuckle", "coupler", "driver", "follower", "silicone",
)


def _is_robot_or_structure(name: str) -> bool:
    n = name.lower()
    return any(pat in n for pat in _ROBOT_BODY_PATTERNS)


# ---------------------------------------------------------------------------
# 유틸리티: 회전행렬 → 쿼터니언 (x, y, z, w)
# ---------------------------------------------------------------------------

def _rotmat_to_quat(R: np.ndarray):
    """3×3 회전행렬 → 쿼터니언 (x, y, z, w)."""
    trace = R[0, 0] + R[1, 1] + R[2, 2]
    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (R[2, 1] - R[1, 2]) * s
        y = (R[0, 2] - R[2, 0]) * s
        z = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return x, y, z, w


# ---------------------------------------------------------------------------
# 접근 단계 열거형
# ---------------------------------------------------------------------------

class ApproachStage(Enum):
    IDLE    = 0   # 대기
    HOVER   = 1   # 물체 위 hover_height 위치로 이동
    DESCEND = 2   # 물체 위 grasp_offset 위치로 하강
    DONE    = 3   # 접근 완료


# ---------------------------------------------------------------------------
# ObjectApproachNode
# ---------------------------------------------------------------------------

class ObjectApproachNode:
    """MuJoCo 물체 위치를 읽어 EE 접근 명령을 ROS2 로 발행하는 노드.

    Parameters
    ----------
    model        : MuJoCo MjModel
    data         : MuJoCo MjData  (RT 루프와 공유 — 읽기 전용 사용)
    hover_height : 물체 위 정지 높이 (m, 기본 0.15)
    grasp_offset : 파지 준비 높이   (m, 기본 0.02)
    pos_tol      : 단계 전환 위치 오차 허용값 (m, 기본 0.01)
    publish_hz   : 목표 발행 주기 (Hz, 기본 10)
    node_name    : ROS2 노드 이름
    """

    # EE 가 위에서 아래를 향하는 기본 회전행렬
    # EE z-axis → −world Z (수직 하강 파지 방향)
    _ROT_DOWN = np.array([
        [ 1.0,  0.0,  0.0],
        [ 0.0, -1.0,  0.0],
        [ 0.0,  0.0, -1.0],
    ])

    def __init__(self, model, data,
                 hover_height: float = 0.15,
                 grasp_offset: float = 0.02,
                 pos_tol: float = 0.01,
                 publish_hz: float = 10.0,
                 node_name: str = "object_approach_node"):
        self.model        = model
        self.data         = data
        self.hover_height = hover_height
        self.grasp_offset = grasp_offset
        self.pos_tol      = pos_tol
        self.dt           = 1.0 / publish_hz

        # ROS2 초기화
        try:
            rclpy.init()
        except RuntimeError:
            pass  # 이미 초기화된 경우

        self._node = rclpy.create_node(node_name)
        self._pub  = self._node.create_publisher(PoseStamped, "/ur5e/cmd/ee_target", 10)
        self._pub_scene = self._node.create_publisher(String, "/mujoco/scene_objects", 10)
        self._node.create_subscription(String, "/ur5e/approach/start",
                                       self._cb_start, 10)
        self._node.create_subscription(String, "/mujoco/query_objects",
                                       self._cb_query_objects, 10)

        # 상태
        self._lock           = threading.Lock()
        self._stage          = ApproachStage.IDLE
        self._target_body_id = -1
        self._target_name    = ""
        self._running        = False

        # ROS2 spin 스레드 — 전용 executor 사용 (rclpy.spin 공유 executor 충돌 방지)
        from rclpy.executors import SingleThreadedExecutor
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)
        self._spin_thread = threading.Thread(
            target=self._executor.spin, daemon=True)
        self._spin_thread.start()
        print("[ApproachNode] ROS2 노드 시작 완료")

    # ------------------------------------------------------------------
    # 씬 물체 탐색
    # ------------------------------------------------------------------

    def _find_target_body(self) -> tuple[int, str]:
        """로봇팔·바닥 제외, 첫 번째 물체 body_id와 이름을 반환. 없으면 (-1, '')."""
        for i in range(1, self.model.nbody):   # 0 = world
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, i) or ""
            if not _is_robot_or_structure(name):
                return i, name
        return -1, ""

    # ------------------------------------------------------------------
    # 물체 위치 발행
    # ------------------------------------------------------------------

    def publish_scene_objects(self) -> bool:
        """로봇팔·바닥 제외 물체의 월드 위치를 /mujoco/scene_objects (JSON String)으로 발행.

        반환값: 발행 성공 여부.
        """
        body_id, name = self._find_target_body()
        if body_id == -1:
            print("[ApproachNode] 대상 물체를 찾을 수 없습니다.")
            return False

        pos  = self.data.xpos[body_id].copy()
        xmat = self.data.xmat[body_id].reshape(3, 3).copy()
        qx, qy, qz, qw = _rotmat_to_quat(xmat)

        payload = {
            "name": name,
            "position":    {"x": round(float(pos[0]), 5),
                            "y": round(float(pos[1]), 5),
                            "z": round(float(pos[2]), 5)},
            "orientation": {"x": round(float(qx), 5),
                            "y": round(float(qy), 5),
                            "z": round(float(qz), 5),
                            "w": round(float(qw), 5)},
        }
        out = String()
        out.data = json.dumps(payload)
        self._pub_scene.publish(out)
        print(f"[ApproachNode] 물체 위치 발행: {name} @ "
              f"({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f})")
        return True

    # ------------------------------------------------------------------
    # ROS2 콜백
    # ------------------------------------------------------------------

    def _cb_query_objects(self, _msg: String):
        """/mujoco/query_objects 수신 → 즉시 씬 물체 위치 발행."""
        self.publish_scene_objects()

    def _cb_start(self, msg: String):
        """'/ur5e/approach/start' 수신 → 물체 이름으로 접근 시작."""
        name = msg.data.strip()
        body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, name)
        if body_id == -1:
            print(f"[ApproachNode] 물체 '{name}' 를 모델에서 찾을 수 없습니다.")
            return
        with self._lock:
            self._target_body_id = body_id
            self._target_name    = name
            self._stage          = ApproachStage.HOVER
        print(f"[ApproachNode] '{name}' (body_id={body_id}) 접근 시작 → HOVER")

    # ------------------------------------------------------------------
    # 내부 헬퍼
    # ------------------------------------------------------------------

    def _get_object_pos(self) -> np.ndarray:
        """현재 물체 월드 위치 반환."""
        return self.data.xpos[self._target_body_id].copy()

    def _make_pose_msg(self, pos: np.ndarray, rot: np.ndarray) -> PoseStamped:
        """위치 + 회전행렬 → PoseStamped (duration=0: 컨트롤러 자동 계산)."""
        msg = PoseStamped()
        msg.header.frame_id = "base"
        msg.pose.position.x = float(pos[0])
        msg.pose.position.y = float(pos[1])
        msg.pose.position.z = float(pos[2])
        qx, qy, qz, qw = _rotmat_to_quat(rot)
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        return msg

    def _ee_pos(self) -> np.ndarray:
        """MuJoCo data 에서 EE 월드 위치를 직접 읽는다."""
        # ur5e_rt_controller 와 같은 방식: data.xpos[ee_body_id]
        # 여기서는 wrist_3_link body 이름으로 찾음
        body_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, "wrist_3_link")
        if body_id == -1:
            body_id = self.model.nbody - 1
        return self.data.xpos[body_id].copy()

    # ------------------------------------------------------------------
    # 접근 루프 한 스텝
    # ------------------------------------------------------------------

    def _step(self):
        with self._lock:
            stage   = self._stage
            body_id = self._target_body_id
            name    = self._target_name

        if stage == ApproachStage.IDLE or body_id == -1:
            return

        obj_pos = self._get_object_pos()
        ee_pos  = self._ee_pos()

        if stage == ApproachStage.HOVER:
            target = obj_pos + np.array([0.0, 0.0, self.hover_height])
            err    = np.linalg.norm(ee_pos - target)

            # 아직 목표에 충분히 가깝지 않으면 계속 발행
            if err > self.pos_tol:
                self._pub.publish(self._make_pose_msg(target, self._ROT_DOWN))
            else:
                print(f"[ApproachNode] HOVER 완료 (err={err:.4f}m) → DESCEND")
                with self._lock:
                    self._stage = ApproachStage.DESCEND

        elif stage == ApproachStage.DESCEND:
            target = obj_pos + np.array([0.0, 0.0, self.grasp_offset])
            err    = np.linalg.norm(ee_pos - target)

            if err > self.pos_tol:
                self._pub.publish(self._make_pose_msg(target, self._ROT_DOWN))
            else:
                print(f"[ApproachNode] DESCEND 완료 (err={err:.4f}m) → DONE")
                with self._lock:
                    self._stage = ApproachStage.DONE

        elif stage == ApproachStage.DONE:
            print(f"[ApproachNode] '{name}' 접근 완료. 그리퍼 닫기 준비.")
            # 더 이상 발행하지 않음 — 상위 레이어에서 그리퍼 명령 전송

    # ------------------------------------------------------------------
    # 실행 / 중지
    # ------------------------------------------------------------------

    def _loop(self):
        while self._running:
            t0 = time.perf_counter()
            self._step()
            elapsed = time.perf_counter() - t0
            sleep_t = self.dt - elapsed
            if sleep_t > 0.0:
                time.sleep(sleep_t)

    def start_thread(self) -> threading.Thread:
        """백그라운드 스레드로 접근 루프를 시작한다."""
        self._running = True
        t = threading.Thread(target=self._loop, daemon=True)
        t.start()
        return t

    def stop(self):
        self._running = False
        with self._lock:
            self._stage = ApproachStage.IDLE

    @property
    def stage(self) -> ApproachStage:
        with self._lock:
            return self._stage
