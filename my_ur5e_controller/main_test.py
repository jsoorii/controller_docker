import mujoco
import meshcat
import meshcat.geometry as g
import numpy as np
import time
import threading
import sys

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64MultiArray

from ur5e_rt_controller import UR5eRTController, ControlState


# ---------------------------------------------------------------------------
# ROS2 Node
# ---------------------------------------------------------------------------

class UR5eROS2Node(Node):
    def __init__(self, controller: UR5eRTController):
        super().__init__("ur5e_rt_node")
        self.controller = controller

        # --- Subscribers ---
        self.create_subscription(
            PoseStamped,
            "/ur5e/cmd/ee_target",
            self._cb_ee_target,
            10,
        )
        self.create_subscription(
            Float64MultiArray,
            "/ur5e/cmd/joint_target",
            self._cb_joint_target,
            10,
        )
        self.create_subscription(
            Bool,
            "/ur5e/cmd/mode",
            self._cb_mode,
            10,
        )

        # --- Publishers ---
        self._pub_ee   = self.create_publisher(PoseStamped, "/ur5e/state/ee_pose", 10)
        self._pub_joint = self.create_publisher(JointState, "/ur5e/state/joint", 10)

        # Publish state at 50 Hz
        self.create_timer(0.02, self._publish_state)

        self.get_logger().info(
            "UR5e ROS2 node ready.\n"
            "  Sub: /ur5e/cmd/ee_target (PoseStamped)\n"
            "  Sub: /ur5e/cmd/joint_target (Float64MultiArray, 6 values)\n"
            "  Sub: /ur5e/cmd/mode (Bool: true=task-space)\n"
            "  Pub: /ur5e/state/ee_pose (PoseStamped)\n"
            "  Pub: /ur5e/state/joint (JointState)"
        )

    # --- Callbacks ---

    def _cb_ee_target(self, msg: PoseStamped):
        p = msg.pose.position
        q = msg.pose.orientation  # quaternion x y z w
        self.controller.task_cmd["pos_target"] = np.array([p.x, p.y, p.z])
        # Convert quaternion to rotation matrix
        self.controller.task_cmd["rot_target"] = _quat_to_rot(q.x, q.y, q.z, q.w)
        self.controller.task_cmd["use_task_space"] = True

    def _cb_joint_target(self, msg: Float64MultiArray):
        if len(msg.data) != 6:
            self.get_logger().warn(f"joint_target: expected 6 values, got {len(msg.data)}")
            return
        self.controller.motor_cmd["q_target"] = np.array(msg.data)
        self.controller.task_cmd["use_task_space"] = False

    def _cb_mode(self, msg: Bool):
        self.controller.task_cmd["use_task_space"] = msg.data
        self.get_logger().info(
            f"Mode -> {'task-space' if msg.data else 'joint-space'}"
        )

    # --- State publisher ---

    def _publish_state(self):
        now = self.get_clock().now().to_msg()

        # EE pose
        ee_pos, ee_rot = self.controller.get_ee_pose()
        ps = PoseStamped()
        ps.header.stamp = now
        ps.header.frame_id = "world"
        ps.pose.position.x = float(ee_pos[0])
        ps.pose.position.y = float(ee_pos[1])
        ps.pose.position.z = float(ee_pos[2])
        qx, qy, qz, qw = _rot_to_quat(ee_rot)
        ps.pose.orientation.x = qx
        ps.pose.orientation.y = qy
        ps.pose.orientation.z = qz
        ps.pose.orientation.w = qw
        self._pub_ee.publish(ps)

        # Joint state
        js = JointState()
        js.header.stamp = now
        js.name = [f"joint_{i+1}" for i in range(6)]
        js.position = self.controller.motor_state["q"].tolist()
        js.velocity = self.controller.motor_state["dq"].tolist()
        self._pub_joint.publish(js)


# ---------------------------------------------------------------------------
# Quaternion helpers
# ---------------------------------------------------------------------------

def _quat_to_rot(x, y, z, w) -> np.ndarray:
    """Quaternion (x,y,z,w) -> 3x3 rotation matrix."""
    n = x*x + y*y + z*z + w*w
    if n < 1e-10:
        return np.eye(3)
    s = 2.0 / n
    return np.array([
        [1 - s*(y*y + z*z),   s*(x*y - z*w),     s*(x*z + y*w)],
        [s*(x*y + z*w),       1 - s*(x*x + z*z),  s*(y*z - x*w)],
        [s*(x*z - y*w),       s*(y*z + x*w),      1 - s*(x*x + y*y)],
    ])


def _rot_to_quat(R: np.ndarray):
    """3x3 rotation matrix -> quaternion (x,y,z,w)."""
    tr = R[0,0] + R[1,1] + R[2,2]
    if tr > 0:
        s = 0.5 / np.sqrt(tr + 1.0)
        w = 0.25 / s
        x = (R[2,1] - R[1,2]) * s
        y = (R[0,2] - R[2,0]) * s
        z = (R[1,0] - R[0,1]) * s
    elif R[0,0] > R[1,1] and R[0,0] > R[2,2]:
        s = 2.0 * np.sqrt(1.0 + R[0,0] - R[1,1] - R[2,2])
        w = (R[2,1] - R[1,2]) / s
        x = 0.25 * s
        y = (R[0,1] + R[1,0]) / s
        z = (R[0,2] + R[2,0]) / s
    elif R[1,1] > R[2,2]:
        s = 2.0 * np.sqrt(1.0 + R[1,1] - R[0,0] - R[2,2])
        w = (R[0,2] - R[2,0]) / s
        x = (R[0,1] + R[1,0]) / s
        y = 0.25 * s
        z = (R[1,2] + R[2,1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + R[2,2] - R[0,0] - R[1,1])
        w = (R[1,0] - R[0,1]) / s
        x = (R[0,2] + R[2,0]) / s
        y = (R[1,2] + R[2,1]) / s
        z = 0.25 * s
    return float(x), float(y), float(z), float(w)


# ---------------------------------------------------------------------------
# Meshcat helpers (unchanged)
# ---------------------------------------------------------------------------

def setup_meshcat_robot(model, vis):
    print("📦 로봇 메쉬 등록 중...")
    for i in range(model.ngeom):
        geom_name = f"geom_{i}"
        geom_type = model.geom_type[i]
        if geom_type == mujoco.mjtGeom.mjGEOM_MESH:
            mesh_id = model.geom_dataid[i]
            vert_adr = model.mesh_vertadr[mesh_id]
            vert_num = model.mesh_vertnum[mesh_id]
            face_adr = model.mesh_faceadr[mesh_id]
            face_num = model.mesh_facenum[mesh_id]
            vertices = model.mesh_vert[vert_adr : vert_adr + vert_num]
            faces = model.mesh_face[face_adr : face_adr + face_num]
            geom = g.TriangularMeshGeometry(vertices, faces)
            rgba = model.geom_rgba[i]
            material = g.MeshLambertMaterial(
                color=int('%02X%02X%02X' % tuple((rgba[:3]*255).astype(int)), 16),
                opacity=float(rgba[3])
            )
            vis["robot"][geom_name].set_object(geom, material)
        elif geom_type == mujoco.mjtGeom.mjGEOM_PLANE:
            vis["env"][geom_name].set_object(g.Box([10, 10, 0.01]))


def update_visualizer(vis, model, data):
    for i in range(model.ngeom):
        geom_name = f"geom_{i}"
        pos = data.geom_xpos[i]
        rot = data.geom_xmat[i].reshape(3, 3)
        T = np.eye(4)
        T[:3, :3] = rot
        T[:3, 3] = pos
        if model.geom_type[i] == mujoco.mjtGeom.mjGEOM_MESH:
            vis["robot"][geom_name].set_transform(T)
        elif model.geom_type[i] == mujoco.mjtGeom.mjGEOM_PLANE:
            vis["env"][geom_name].set_transform(T)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    # --- MuJoCo model ---
    model_path = "/ros2_ws/src/mujoco_menagerie/universal_robots_ur5e/ur5e.xml"
    try:
        model = mujoco.MjModel.from_xml_path(model_path)
        data = mujoco.MjData(model)
        print("✅ MuJoCo 모델 로드 완료")
    except Exception as e:
        print(f"❌ 모델 로드 실패: {e}")
        return

    # --- Meshcat ---
    vis = meshcat.Visualizer()
    print(f"\n🌐 Meshcat 접속 주소: {vis.url()}")
    vis.delete()
    setup_meshcat_robot(model, vis)

    # --- Controller ---
    controller = UR5eRTController(model, data)
    ctrl_thread = threading.Thread(target=controller.start, daemon=True)
    ctrl_thread.start()

    # --- ROS2 ---
    rclpy.init()
    node = UR5eROS2Node(controller)
    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    # --- Main loop (visualization + monitor) ---
    print("\n🚀 제어 및 시각화 루프 가동 중... (Ctrl-C to stop)")
    try:
        while True:
            update_visualizer(vis, model, data)

            if np.any(np.abs(data.qvel) > 50):
                print(f"\n⚠️ 물리 불안정 감지! qvel: {data.qvel}")

            ee_pos, _ = controller.get_ee_pose()
            tgt = controller.task_cmd["pos_target"]
            err = np.linalg.norm(tgt - ee_pos)
            mode = "TASK" if controller.task_cmd["use_task_space"] else "JOINT"
            print(
                f"\r[{controller.state.name}][{mode}] "
                f"EE: [{ee_pos[0]:.3f}, {ee_pos[1]:.3f}, {ee_pos[2]:.3f}] "
                f"err: {err:.4f}  Missed: {controller.miss_count}",
                end="",
                flush=True,
            )

            time.sleep(0.02)

    except KeyboardInterrupt:
        print("\n\n👋 종료합니다.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
