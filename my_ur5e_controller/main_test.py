import mujoco
import meshcat
import meshcat.geometry as g
import numpy as np
import time
import threading
import sys
import os

import gripper_config
import scene_config
from ur5e_rt_controller import UR5eRTController, ControlState
from robotiq_grasp_adapter import RobotiqGraspAdapter
from object_approach_node import ObjectApproachNode

UR5E_SCENE = "/ros2_ws/src/mujoco_menagerie/universal_robots_ur5e/scene.xml"


def load_scene_model():
    """MjSpec으로 UR5e scene + 그리퍼를 합쳐 로드한다."""
    spec = mujoco.MjSpec.from_file(UR5E_SCENE)

    # ── 그리퍼: gripper_config.ACTIVE 기준으로 부착 ────────────────────────
    gripper_config.attach(spec)

    # ── 환경 오브젝트: scene_config.OBJECTS 기준으로 추가 ──────────────────
    scene_config.add_objects(spec)

    # ── 관절 토크 센서 (arm 6축) ───────────────────────────────────────────
    for act_name in ('shoulder_pan', 'shoulder_lift', 'elbow',
                     'wrist_1', 'wrist_2', 'wrist_3'):
        s          = spec.add_sensor()
        s.name     = f'torque_{act_name}'
        s.type     = mujoco.mjtSensor.mjSENS_ACTUATORFRC
        s.objtype  = mujoco.mjtObj.mjOBJ_ACTUATOR
        s.objname  = act_name

    return spec.compile()

# geom의 실효 RGBA 반환: alpha==0이면 material 색상을 우선 사용
def get_geom_rgba(model, i):
    rgba = model.geom_rgba[i].copy()
    # MuJoCo 관례: alpha==0은 "material 색상 사용"을 의미
    if rgba[3] == 0:
        mat_id = model.geom_matid[i]
        if mat_id >= 0:
            rgba = model.mat_rgba[mat_id].copy()
    # 여전히 alpha==0이면 기본 회색으로 대체 (완전 투명 방지)
    if rgba[3] == 0:
        rgba = np.array([0.5, 0.5, 0.5, 1.0])
    return rgba

# 1. Meshcat에 로봇 메쉬 등록 (이게 실행되어야 화면에 보임)
def setup_meshcat_robot(model, data, vis):
    print("📦 로봇 메쉬 등록 중...")
    for i in range(model.ngeom):
        geom_name = f"geom_{i}"
        geom_type = model.geom_type[i]

        pos = data.geom_xpos[i]
        rot = data.geom_xmat[i].reshape(3, 3)
        T = np.eye(4)
        T[:3, :3] = rot
        T[:3, 3] = pos

        # 메쉬 타입(Type 7)인 경우 처리
        if geom_type == mujoco.mjtGeom.mjGEOM_MESH:
            mesh_id = model.geom_dataid[i]
            vert_adr = model.mesh_vertadr[mesh_id]
            vert_num = model.mesh_vertnum[mesh_id]
            face_adr = model.mesh_faceadr[mesh_id]
            face_num = model.mesh_facenum[mesh_id]

            vertices = model.mesh_vert[vert_adr : vert_adr + vert_num]
            faces = model.mesh_face[face_adr : face_adr + face_num]

            geom = g.TriangularMeshGeometry(vertices, faces)

            rgba = get_geom_rgba(model, i)
            material = g.MeshLambertMaterial(
                color=int('%02X%02X%02X' % tuple((rgba[:3]*255).astype(int)), 16),
                opacity=float(rgba[3])
            )

            vis["robot"][geom_name].set_object(geom, material)
            vis["robot"][geom_name].set_transform(T)

        # 기본 도형(Plane, Box 등): 정적이므로 초기 한 번만 transform 설정
        elif geom_type == mujoco.mjtGeom.mjGEOM_PLANE:
            vis["env"][geom_name].set_object(g.Box([10, 10, 0.01]))
            vis["env"][geom_name].set_transform(T)

        elif geom_type == mujoco.mjtGeom.mjGEOM_BOX:
            size = model.geom_size[i]  # half-sizes
            rgba = get_geom_rgba(model, i)
            material = g.MeshLambertMaterial(
                color=int('%02X%02X%02X' % tuple((rgba[:3]*255).astype(int)), 16),
                opacity=float(rgba[3])
            )
            vis["env"][geom_name].set_object(g.Box(size * 2), material)
            vis["env"][geom_name].set_transform(T)

# 2. 매 프레임마다 로봇(mesh) geom 위치 업데이트 — env geom은 정적이라 제외
def update_visualizer(vis, model, data):
    for i in range(model.ngeom):
        if model.geom_type[i] != mujoco.mjtGeom.mjGEOM_MESH:
            continue
        geom_name = f"geom_{i}"
        pos = data.geom_xpos[i]
        rot = data.geom_xmat[i].reshape(3, 3)
        T = np.eye(4)
        T[:3, :3] = rot
        T[:3, 3] = pos
        vis["robot"][geom_name].set_transform(T)

def main():
    # --- 모델 로드 ---
    try:
        model = load_scene_model()
        data = mujoco.MjData(model)
        print("✅ MuJoCo 모델 로드 완료")
    except Exception as e:
        print(f"❌ 모델 로드 실패: {e}")
        return

    # --- Meshcat 초기화 ---
    vis = meshcat.Visualizer()
    print(f"\n🌐 Meshcat 접속 주소: {vis.url()}")
    vis.delete()  # 기존 잔상 제거

    # 현재 MeshCat 포트를 파일로 기록 (대시보드 포트 탐지용)
    try:
        import re as _re
        _m = _re.search(r":(\d+)/", vis.url())
        if _m:
            with open("/tmp/meshcat_port.txt", "w") as _f:
                _f.write(_m.group(1))
    except Exception:
        pass

    # 초기 geom_xpos / geom_xmat 계산 (이 없으면 초기 위치가 전부 0)
    mujoco.mj_forward(model, data)

    # 로봇 메쉬 등록 수행
    setup_meshcat_robot(model, data, vis)

    # --- 제어기 객체 생성 및 스레드 실행 ---
    controller = UR5eRTController(model, data, gripper_cfg=gripper_config.ACTIVE)
    controller.start_ros_node()
    ctrl_thread = threading.Thread(target=controller.start, daemon=True)
    ctrl_thread.start()

    # --- Anti-Slip 반사 제어 (GraspController 기반) ---
    grasp_adapter = RobotiqGraspAdapter(model, data, controller)
    grasp_adapter.start_thread()

    # --- 물체 접근 노드 ---
    approach_node = ObjectApproachNode(model, data)
    approach_node.start_thread()

    # --- 메인 루프 (시각화 및 모니터링) ---
    print("\n🚀 제어 및 시각화 루프 가동 중...")
    vis_frame = 0
    try:
        while True:
            # 1. 시각화 업데이트 (10Hz: 5프레임마다 1회 — ZMQ 과부하 방지)
            vis_frame += 1
            if vis_frame % 5 == 0:
                update_visualizer(vis, model, data)

            # 폭발 감지 로그
            if np.any(np.abs(data.qvel) > 50):
                print(f"\n⚠️ 물리 불안정 감지! qvel: {data.qvel}")

            # 2. 상태 모니터링 출력
            ee_pos, _ = controller.get_ee_pose()
            tgt = controller.task_cmd["pos_target"]
            err = np.linalg.norm(tgt - ee_pos)

            if controller.collision_detected:
                info = controller.collision_info
                print(
                    f"\r[⚠️ 충돌 정지] {info.get('body1','?')} ↔ {info.get('body2','?')}"
                    f" | dist={info.get('dist',0):.5f}"
                    f" | Missed: {controller.miss_count}",
                    end=""
                )
            else:
                print(
                    f"\r[상태]: {controller.state.name} | Missed: {controller.miss_count}"
                    f" | EE: [{ee_pos[0]:.3f}, {ee_pos[1]:.3f}, {ee_pos[2]:.3f}]"
                    f" | err: {err:.4f}",
                    end=""
                )

            time.sleep(0.02)

    except KeyboardInterrupt:
        print("\n\n👋 시뮬레이션을 안전하게 종료합니다.")

if __name__ == "__main__":
    main()