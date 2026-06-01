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
    IDLE        : 대기
    PRE_APPROACH: 물체에서 30cm 이격된 중간 웨이포인트로 이동 (task space)
    HOVER       : 물체에서 10cm 이격, 10cm 높이로 이동 (task space)
    DESCEND     : 물체에서 grasp_offset(m) 이격, 10cm 높이로 접근 (task space)
    DONE        : 접근 완료 (그리퍼 닫기는 상위 레이어 담당)

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

from ur5e_controller import rotmat_axis_angle as _rotmat_axis_angle


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
    IDLE         = 0   # 대기
    PRE_APPROACH = 1   # task space: 30cm 이격 중간 웨이포인트
    HOVER        = 2   # EE IK: 물체 위 hover_height 위치로 이동
    DESCEND      = 3   # EE IK: 물체 위 grasp_offset 위치로 하강
    DONE         = 4   # 접근 완료


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
    grasp_offset : 물체 표면 기준 DESCEND 이격 거리 (m, 기본 0.05)
    pos_tol      : 단계 전환 위치 오차 허용값 (m, 기본 0.01)
    rot_tol      : 단계 전환 회전 오차 허용값 (rad, 기본 0.10 ≈ 5.7°)
    publish_hz   : 목표 발행 주기 (Hz, 기본 10)
    node_name    : ROS2 노드 이름
    """

    # TCP 오프셋 초기 계산용 기준 회전행렬 (접근 시 실제 사용은 _approach_rot)
    # Col1(wrist Y→world): [0,0,-1] = 도구 수직 하강
    _ROT_DOWN = np.array([
        [1.0,  0.0,  0.0],
        [0.0,  0.0,  1.0],
        [0.0, -1.0,  0.0],
    ])

    def __init__(self, model, data,
                 hover_height: float = 0.15,
                 grasp_offset: float = -0.02,
                 pos_tol: float = 0.01,
                 rot_tol: float = 0.10,
                 publish_hz: float = 10.0,
                 node_name: str = "object_approach_node"):
        self.model        = model
        self.data         = data
        self.hover_height = hover_height
        self.grasp_offset = grasp_offset
        self.pos_tol      = pos_tol
        self.rot_tol      = rot_tol
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
        self._node.create_subscription(String, "/ur5e/approach/descend",
                                       self._cb_descend, 10)

        # 상태
        self._lock               = threading.Lock()
        self._stage              = ApproachStage.IDLE
        self._target_body_id     = -1
        self._target_name        = ""
        self._y_noise            = 0.0   # 접근 시작마다 샘플링되는 Y 노이즈 (m)
        self._cmd_sent           = False  # 단계 진입 시 1회만 발행 플래그
        self._last_pub_time      = 0.0   # 마지막 발행 시간 (재발행 주기 제어)
        self._running            = False
        self._descend_requested  = False  # HOVER 완료 후 DESCEND 진행 수동 트리거
        self._hover_wait_logged  = False  # HOVER 대기 로그 중복 방지
        # 접근마다 물체 방향에 맞춰 동적 계산되는 회전행렬 (초기값 = _ROT_DOWN)
        self._approach_rot   = self._ROT_DOWN.copy()

        # wrist_3_link → 그리퍼 손끝: tool-z 오프셋 + 접근 포즈에서의 XY lateral 보정
        self._tcp_z_offset   = 0.0
        self._tcp_xy_correct = np.zeros(2)  # [dx, dy]: 타겟에서 빼야 할 wrist→site 오프셋
        self._compute_tcp_offsets()
        self._print_frame_debug()

        print(f"[ApproachNode] TCP z-offset (손끝~wrist): {self._tcp_z_offset*100:.1f}cm")
        print(f"[ApproachNode] TCP XY 보정 (world): dx={self._tcp_xy_correct[0]*100:.1f}cm, "
              f"dy={self._tcp_xy_correct[1]*100:.1f}cm")

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
    # 접근 회전행렬 동적 계산
    # ------------------------------------------------------------------

    def _make_approach_rot(self, obj_pos: np.ndarray) -> np.ndarray:
        """물체 방향 기반 수평 측면 접근 회전행렬.

        - wrist Y (J6 축) → 접근방향에서 90° 회전한 방향 (XY 평면 내 수직)
        - wrist X → world -Z (수직 하강)
        - wrist Z → 실제 접근방향 (base→obj, XY 수평)

        J6 회전축  = approach_rot[:, 1] (wrist Y) = normalize(base→obj) = 물체 방향
        wrist Z    = approach_rot[:, 2] = approach_dir의 90° CW 수평 방향
        """
        base_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
        base_xy = self.data.xpos[base_id][:2].copy() if base_id >= 0 else np.zeros(2)
        rel_xy = np.array([float(obj_pos[0]) - base_xy[0],
                           float(obj_pos[1]) - base_xy[1], 0.])
        norm = np.linalg.norm(rel_xy)
        approach_dir = rel_xy / norm if norm > 0.01 else np.array([1., 0., 0.])

        # wrist Y (J6 축) = approach_dir = normalize(base→obj) (물체를 향하는 방향)
        ax, ay = approach_dir[0], approach_dir[1]
        wrist_y = np.array([ax, ay, 0.])

        # wrist X = downward
        wrist_x = np.array([0., 0., -1.])

        # wrist Z = cross(wrist_x, wrist_y)
        wrist_z = np.cross(wrist_x, wrist_y)
        wrist_z /= np.linalg.norm(wrist_z)

        return np.column_stack([wrist_x, wrist_y, wrist_z])

    def _safe_z_rot_deg(self) -> float:
        """J6(wrist_3) 조인트 한계를 넘지 않는 방향으로 EE z축 90° 회전 부호를 반환.

        +90.0 또는 -90.0 반환.
        """
        j6_jnt_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_JOINT, "wrist_3")
        if j6_jnt_id < 0:
            return +90.0   # 조인트를 찾지 못하면 기본값

        q_addr  = int(self.model.jnt_qposadr[j6_jnt_id])
        lo, hi  = float(self.model.jnt_range[j6_jnt_id][0]), \
                  float(self.model.jnt_range[j6_jnt_id][1])
        cur     = float(self.data.qpos[q_addr])
        half_pi = np.pi / 2.0

        plus_val  = cur + half_pi
        minus_val = cur - half_pi
        plus_ok   = lo <= plus_val  <= hi
        minus_ok  = lo <= minus_val <= hi

        if plus_ok and minus_ok:
            # 둘 다 범위 내 → J6 중심(0)에 더 가까운 쪽 선택
            return +90.0 if abs(plus_val) <= abs(minus_val) else -90.0
        elif plus_ok:
            return +90.0
        elif minus_ok:
            return -90.0
        else:
            # 둘 다 초과 가능성 → 한계에서 덜 벗어나는 쪽
            over_plus  = max(0.0, plus_val  - hi) + max(0.0, lo - plus_val)
            over_minus = max(0.0, minus_val - hi) + max(0.0, lo - minus_val)
            return +90.0 if over_plus <= over_minus else -90.0

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

    def _cb_descend(self, _msg: String):
        """/ur5e/approach/descend 수신 → HOVER 완료 후 DESCEND로 진행."""
        with self._lock:
            if self._stage == ApproachStage.HOVER:
                self._descend_requested = True
                self._hover_wait_logged = False
                print("[ApproachNode] DESCEND 요청 수신 → HOVER 수렴 후 진행")

    def _cb_start(self, msg: String):
        """'/ur5e/approach/start' 수신 → 물체 이름으로 접근 시작."""
        name = msg.data.strip()
        body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, name)
        if body_id == -1:
            print(f"[ApproachNode] 물체 '{name}' 를 모델에서 찾을 수 없습니다.")
            return
        # Y 노이즈 샘플링: σ=0.025m → ±2σ ≈ ±5cm, [-5cm, 5cm] 클리핑
        y_noise = float(np.clip(np.random.normal(0.0, 0.003), -0.003, 0.003))
        # 물체 방향 기반 접근 회전행렬 계산 (IK 달성 가능, 6번축 수평 보장)
        obj_pos = self.data.xpos[body_id].copy()
        approach_rot = self._make_approach_rot(obj_pos)
        # J6(wrist_y = col1) 축으로 90° 추가 회전 — approach_dir(col1)은 유지됨
        rot_deg  = self._safe_z_rot_deg()
        wrist_y  = approach_rot[:, 1]   # J6 회전축 = wrist_y = approach direction
        approach_rot = _rotmat_axis_angle(wrist_y, rot_deg) @ approach_rot
        with self._lock:
            self._target_body_id    = body_id
            self._target_name       = name
            self._y_noise           = y_noise
            self._approach_rot      = approach_rot
            self._stage             = ApproachStage.PRE_APPROACH
            self._cmd_sent          = False
            self._descend_requested = False
            self._hover_wait_logged = False
        print(f"[ApproachNode] '{name}' (body_id={body_id}) 접근 시작 → PRE_APPROACH  Y노이즈={y_noise*100:+.1f}cm")
        print(f"[ApproachNode]   obj_xy=[{obj_pos[0]:.3f}, {obj_pos[1]:.3f}]  "
              f"EE-z 회전={rot_deg:+.0f}°  "
              f"approach_rot col1(J6축=물체방향)={np.round(approach_rot[:, 1], 3).tolist()}")

    # ------------------------------------------------------------------
    # 내부 헬퍼
    # ------------------------------------------------------------------

    def _print_frame_debug(self):
        """베이스 기준 월드↔로봇 좌표계 매핑을 콘솔에 출력."""
        print("[Frame] ========= 좌표계 디버그 =========")

        base_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
        wrist_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "wrist_3_link")
        site_id  = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, "attachment_site")

        def _ax(R, i):
            return np.round(R[:, i], 3).tolist()

        if base_id >= 0:
            bR  = self.data.xmat[base_id].reshape(3, 3)
            bp  = np.round(self.data.xpos[base_id], 3)
            print(f"[Frame] base_link  pos(world)={bp.tolist()}")
            print(f"[Frame]            X→world={_ax(bR,0)}  Y→world={_ax(bR,1)}  Z→world={_ax(bR,2)}")

        wrist_pos = None
        wrist_R   = None
        if wrist_id >= 0:
            wrist_R   = self.data.xmat[wrist_id].reshape(3, 3)
            wrist_pos = np.round(self.data.xpos[wrist_id], 3)
            print(f"[Frame] wrist_3_link pos(world)={wrist_pos.tolist()}")
            print(f"[Frame]              X→world={_ax(wrist_R,0)}  Y→world={_ax(wrist_R,1)}  Z→world={_ax(wrist_R,2)}")

        if site_id >= 0 and wrist_pos is not None:
            sp     = self.data.site_xpos[site_id]
            sR     = self.data.site_xmat[site_id].reshape(3, 3)
            offset_world = sp - self.data.xpos[wrist_id]
            offset_local = wrist_R.T @ offset_world
            offset_approach = self._ROT_DOWN @ offset_local
            print(f"[Frame] attachment_site pos(world)={np.round(sp,3).tolist()}")
            print(f"[Frame]   → wrist offset (world):   {np.round(offset_world,3).tolist()}")
            print(f"[Frame]   → wrist offset (wrist로컬): {np.round(offset_local,3).tolist()}")
            print(f"[Frame]   → approach포즈 offset(world): {np.round(offset_approach,3).tolist()}")
            print(f"[Frame]   site Z-axis(approach dir, world): {np.round(sR[:,2],3).tolist()}")

        print(f"[Frame] _ROT_DOWN  X→world={self._ROT_DOWN[:,0].tolist()}  "
              f"Y→world={self._ROT_DOWN[:,1].tolist()}  Z→world={self._ROT_DOWN[:,2].tolist()}")
        tool_dir = (self._ROT_DOWN @ np.array([0.,1.,0.])).tolist()
        print(f"[Frame]   ※ approach 시 도구 방향(wrist Y→world) = {tool_dir}  (수직 하강이면 [0,0,-1])")
        print("[Frame] =====================================")

    def _compute_tcp_offsets(self):
        """wrist_3_link → 그리퍼 손끝의 tool-z 거리(self._tcp_z_offset)와
        접근 포즈(_ROT_DOWN)에서의 XY lateral 보정(self._tcp_xy_correct)을 계산한다.

        lateral 보정: 접근 포즈에서 wrist_3_link origin과 attachment_site 간의
        X,Y 오프셋 → 이만큼 타겟을 조정해야 그리퍼 중심이 물체 위에 온다.
        """
        wrist_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, "wrist_3_link")
        if wrist_id == -1:
            return

        wrist_pos = self.data.xpos[wrist_id].copy()
        wrist_R   = self.data.xmat[wrist_id].reshape(3, 3)

        # ── attachment_site 기반 tool-z 방향 & lateral 보정 ──────────────────
        site_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_SITE, "attachment_site")
        if site_id >= 0:
            tool_z    = self.data.site_xmat[site_id].reshape(3, 3)[:, 2]
            site_world = self.data.site_xpos[site_id].copy()
            # attachment_site의 wrist body 프레임 내 로컬 위치
            site_local = wrist_R.T @ (site_world - wrist_pos)
            # 접근 포즈(_ROT_DOWN)에서 world 프레임 오프셋
            site_in_approach = self._ROT_DOWN @ site_local
            # X, Y 성분 = 타겟에서 빼야 할 lateral 보정
            self._tcp_xy_correct = site_in_approach[:2].copy()
        else:
            # fallback: gripper 질량중심 방향
            gpos_list = [self.data.geom_xpos[gi]
                         for gi in range(self.model.ngeom)
                         if "gripper/" in (mujoco.mj_id2name(
                             self.model, mujoco.mjtObj.mjOBJ_BODY,
                             int(self.model.geom_bodyid[gi])) or "")]
            if not gpos_list:
                return
            v = np.mean(gpos_list, axis=0) - wrist_pos
            norm = np.linalg.norm(v)
            tool_z = v / norm if norm > 1e-6 else np.array([0., 0., -1.])

        # ── pad geom의 tool_z 방향 최대 투영 = tcp_z_offset ─────────────────
        max_proj = 0.0
        for gi in range(self.model.ngeom):
            gname = (mujoco.mj_id2name(
                self.model, mujoco.mjtObj.mjOBJ_GEOM, gi) or "").lower()
            if "gripper/" not in gname:
                continue
            if "pad" not in gname.split("gripper/", 1)[1]:
                continue
            proj = float(np.dot(self.data.geom_xpos[gi] - wrist_pos, tool_z))
            if proj > max_proj:
                max_proj = proj
        self._tcp_z_offset = max_proj

    def _get_object_pos(self) -> np.ndarray:
        """현재 물체 월드 위치 반환."""
        return self.data.xpos[self._target_body_id].copy()

    def _get_object_top_z(self, body_id: int) -> float:
        """물체 body의 geom 크기를 읽어 월드 기준 상단 z를 반환.
        geom 타입별로 절반 높이를 더해 EE가 물체 위에 위치하도록 한다."""
        center_z = float(self.data.xpos[body_id][2])
        half_h   = 0.0
        for i in range(self.model.ngeom):
            if self.model.geom_bodyid[i] != body_id:
                continue
            t = self.model.geom_type[i]
            s = self.model.geom_size[i]
            if t == mujoco.mjtGeom.mjGEOM_CYLINDER:
                half_h = max(half_h, float(s[1]))  # size[1] = 절반높이
            elif t == mujoco.mjtGeom.mjGEOM_BOX:
                half_h = max(half_h, float(s[2]))  # size[2] = z 절반크기
            elif t == mujoco.mjtGeom.mjGEOM_SPHERE:
                half_h = max(half_h, float(s[0]))  # size[0] = 반지름
            elif t == mujoco.mjtGeom.mjGEOM_CAPSULE:
                half_h = max(half_h, float(s[0]) + float(s[1]))
        return center_z + half_h

    def _get_object_bottom_z(self, body_id: int) -> float:
        """물체 body의 바닥(바닥 접촉면) z 좌표를 반환."""
        center_z = float(self.data.xpos[body_id][2])
        half_h = 0.0
        for i in range(self.model.ngeom):
            if self.model.geom_bodyid[i] != body_id:
                continue
            t = self.model.geom_type[i]
            s = self.model.geom_size[i]
            if t == mujoco.mjtGeom.mjGEOM_CYLINDER:
                half_h = max(half_h, float(s[1]))
            elif t == mujoco.mjtGeom.mjGEOM_BOX:
                half_h = max(half_h, float(s[2]))
            elif t == mujoco.mjtGeom.mjGEOM_SPHERE:
                half_h = max(half_h, float(s[0]))
            elif t == mujoco.mjtGeom.mjGEOM_CAPSULE:
                half_h = max(half_h, float(s[0]) + float(s[1]))
        return center_z - half_h

    def _get_object_side_radius(self, body_id: int) -> float:
        """물체 body의 수평 접근 기준 측면 반지름을 반환.
        cylinder→size[0], box→max(size[0],size[1]), sphere/capsule→size[0]"""
        max_r = 0.0
        for i in range(self.model.ngeom):
            if self.model.geom_bodyid[i] != body_id:
                continue
            t = self.model.geom_type[i]
            s = self.model.geom_size[i]
            if t == mujoco.mjtGeom.mjGEOM_CYLINDER:
                max_r = max(max_r, float(s[0]))
            elif t == mujoco.mjtGeom.mjGEOM_BOX:
                max_r = max(max_r, float(s[0]), float(s[1]))
            elif t in (mujoco.mjtGeom.mjGEOM_SPHERE, mujoco.mjtGeom.mjGEOM_CAPSULE):
                max_r = max(max_r, float(s[0]))
        return max_r

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

    def _write_state(self, stage: ApproachStage, name: str, err: float = 0.0):
        """접근 단계를 /tmp/approach_state.json으로 저장 (대시보드 폴링용)."""
        import os
        state = {"stage": stage.name, "target": name, "err": round(err, 4)}
        tmp = "/tmp/approach_state.json.tmp"
        with open(tmp, "w") as f:
            json.dump(state, f)
        os.replace(tmp, "/tmp/approach_state.json")

    def _step(self):
        with self._lock:
            stage        = self._stage
            body_id      = self._target_body_id
            name         = self._target_name
            y_noise      = self._y_noise
            cmd_sent     = self._cmd_sent
            approach_rot = self._approach_rot

        if stage == ApproachStage.IDLE or body_id == -1:
            return

        obj_pos = self._get_object_pos()
        ee_pos  = self._ee_pos()

        if stage == ApproachStage.PRE_APPROACH:
            # HOVER와 같은 XY, 더 높은 Z에서 시작하는 중간 웨이포인트.
            # 측면 접근 방향으로 10cm 이격 후 상방 30cm — dead zone 회피.
            approach_dir = approach_rot[:, 1]
            target = np.array([
                obj_pos[0] - approach_dir[0] * (0.10 + self._tcp_z_offset),
                obj_pos[1] + y_noise - approach_dir[1] * (0.10 + self._tcp_z_offset),
                obj_pos[2] + 0.30,
            ])
            err = np.linalg.norm(ee_pos - target)
            now = time.time()

            # 최초 발행 OR 2초마다 재발행 (ROS2 메시지 손실 대비)
            needs_pub = (not cmd_sent) or (err > self.pos_tol and now - self._last_pub_time > 2.0)
            if needs_pub:
                self._pub.publish(self._make_pose_msg(target, approach_rot))
                self._last_pub_time = now
                with self._lock:
                    self._cmd_sent = True
                if not cmd_sent:
                    print(f"[ApproachNode] PRE_APPROACH 명령 발행 (task space):")
                    print(f"  wrist_3_link 타겟: {np.round(target, 4).tolist()}")
                    print(f"  현재 wrist_3 위치: {np.round(ee_pos, 4).tolist()}")

            if cmd_sent and err < self.pos_tol:
                print(f"[ApproachNode] PRE_APPROACH 완료 (err={err:.4f}m) → HOVER")
                with self._lock:
                    self._stage    = ApproachStage.HOVER
                    self._cmd_sent = False

            self._write_state(self._stage, name, err)
            return

        if stage == ApproachStage.HOVER:
            # J6 축(wrist_y) = approach_dir = 물체 방향 → wrist는 그 반대로 배치
            approach_dir = approach_rot[:, 1]   # col1 = wrist_y = 베이스→물체 방향
            target = np.array([
                obj_pos[0] - approach_dir[0] * (0.10 + self._tcp_z_offset),
                obj_pos[1] + y_noise - approach_dir[1] * (0.10 + self._tcp_z_offset),
                float(obj_pos[2]),   # 수평 접근: 물체 중심 높이로 접근
            ])
            err    = np.linalg.norm(ee_pos - target)

            if err > self.pos_tol:
                # 단계 진입 시 1회만 발행 — 재발행 시 ensure_decel로 인한 속도 저하 방지
                if not cmd_sent:
                    self._pub.publish(self._make_pose_msg(target, approach_rot))
                    with self._lock:
                        self._cmd_sent = True
                    pad_target = target + approach_dir * self._tcp_z_offset
                    print(f"[ApproachNode] HOVER 명령 발행:")
                    print(f"  wrist_3_link 타겟:  {np.round(target, 4).tolist()}")
                    print(f"  그리퍼 패드 타겟:   {np.round(pad_target, 4).tolist()}")
                    print(f"  현재 wrist_3 위치:  {np.round(ee_pos, 4).tolist()}")
            else:
                # 위치 수렴 → 회전도 수렴했는지 확인
                wrist_id = mujoco.mj_name2id(
                    self.model, mujoco.mjtObj.mjOBJ_BODY, "wrist_3_link")
                rot_converged = True
                rot_err_mag   = 0.0
                wR            = None
                if wrist_id >= 0:
                    wR = self.data.xmat[wrist_id].reshape(3, 3)
                    R_err = approach_rot @ wR.T
                    rot_err_vec = np.array([
                        R_err[2, 1] - R_err[1, 2],
                        R_err[0, 2] - R_err[2, 0],
                        R_err[1, 0] - R_err[0, 1],
                    ]) * 0.5
                    rot_err_mag  = float(np.linalg.norm(rot_err_vec))
                    rot_converged = rot_err_mag < self.rot_tol

                if rot_converged:
                    with self._lock:
                        descend_ok = self._descend_requested
                    if not descend_ok:
                        if not self._hover_wait_logged:
                            q_cur = self.data.qpos[:6].copy()
                            print(f"[ApproachNode] HOVER 수렴 완료 (pos_err={err:.4f}m, rot_err={rot_err_mag:.3f}rad)")
                            if wR is not None:
                                print(f"[ApproachNode]   J6축→world(실제): {np.round(wR[:, 1], 3).tolist()}")
                            print(f"[ApproachNode]   ▶ '접근' 버튼을 눌러 DESCEND로 진행하세요")
                            self._hover_wait_logged = True
                    else:
                        q_cur = self.data.qpos[:6].copy()
                        print(f"[ApproachNode] HOVER 완료 → DESCEND")
                        print(f"[ApproachNode]   관절각(deg): {np.round(np.degrees(q_cur), 1).tolist()}")
                        with self._lock:
                            self._stage             = ApproachStage.DESCEND
                            self._cmd_sent          = False
                            self._descend_requested = False
                            self._hover_wait_logged = False
                # else: 회전 수렴 대기 (IK 계속 실행 중)
            self._write_state(self._stage, name, err)

        elif stage == ApproachStage.DESCEND:
            approach_dir = approach_rot[:, 1]   # col1 = wrist_y = 베이스→물체 방향
            side_r    = self._get_object_side_radius(body_id)
            # 패드 타겟 = 물체 중심에서 (반지름 + grasp_offset) 이격
            pad_standoff  = side_r + self.grasp_offset
            wrist_standoff = pad_standoff + self._tcp_z_offset
            target = np.array([
                obj_pos[0] - approach_dir[0] * wrist_standoff,
                obj_pos[1] + y_noise - approach_dir[1] * wrist_standoff,
                float(obj_pos[2]),   # 수평 접근: 물체 중심 높이로 접근
            ])
            err    = np.linalg.norm(ee_pos - target)

            if err > self.pos_tol:
                if not cmd_sent:
                    self._pub.publish(self._make_pose_msg(target, approach_rot))
                    with self._lock:
                        self._cmd_sent = True
                    pad_target = target + approach_dir * self._tcp_z_offset
                    print(f"[ApproachNode] DESCEND 명령 발행 (side_r={side_r*100:.1f}cm, clearance={self.grasp_offset*100:.1f}cm):")
                    print(f"  wrist_3_link 타겟:  {np.round(target, 4).tolist()}")
                    print(f"  그리퍼 패드 타겟:   {np.round(pad_target, 4).tolist()}")
                    print(f"  현재 wrist_3 위치:  {np.round(ee_pos, 4).tolist()}")
            else:
                print(f"[ApproachNode] DESCEND 완료 (err={err:.4f}m) → DONE")
                with self._lock:
                    self._stage    = ApproachStage.DONE
                    self._cmd_sent = False
            self._write_state(self._stage, name, err)

        elif stage == ApproachStage.DONE:
            self._write_state(stage, name, 0.0)

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
