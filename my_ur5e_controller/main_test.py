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
from ur5e_controller import UR5eController
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

        # 지면(Plane)만 정적으로 env에 등록
        elif geom_type == mujoco.mjtGeom.mjGEOM_PLANE:
            vis["env"][geom_name].set_object(g.Box([10, 10, 0.01]))
            vis["env"][geom_name].set_transform(T)

        # BOX는 robot에 등록: 그리퍼 패드(left_pad1/2, right_pad1/2)가 BOX 타입이므로
        # env에 두면 손가락이 움직여도 패드가 초기 위치에 고정되어 유령처럼 보임
        elif geom_type == mujoco.mjtGeom.mjGEOM_BOX:
            size = model.geom_size[i]  # half-sizes
            rgba = get_geom_rgba(model, i)
            material = g.MeshLambertMaterial(
                color=int('%02X%02X%02X' % tuple((rgba[:3]*255).astype(int)), 16),
                opacity=float(rgba[3])
            )
            vis["robot"][geom_name].set_object(g.Box(size * 2), material)
            vis["robot"][geom_name].set_transform(T)

        # Meshcat Cylinder은 Y축 정렬, MuJoCo는 Z축 정렬 → Rx(90°) 보정 필요
        elif geom_type == mujoco.mjtGeom.mjGEOM_CYLINDER:
            size = model.geom_size[i]  # [radius, half_height, 0]
            radius, half_h = float(size[0]), float(size[1])
            rgba = get_geom_rgba(model, i)
            material = g.MeshLambertMaterial(
                color=int('%02X%02X%02X' % tuple((rgba[:3]*255).astype(int)), 16),
                opacity=float(rgba[3])
            )
            R_fix = np.array([[1,0,0],[0,0,-1],[0,1,0]], dtype=float)
            T_cyl = T.copy()
            T_cyl[:3, :3] = rot @ R_fix
            vis["robot"][geom_name].set_object(g.Cylinder(half_h * 2, radius), material)
            vis["robot"][geom_name].set_transform(T_cyl)

        elif geom_type == mujoco.mjtGeom.mjGEOM_SPHERE:
            size = model.geom_size[i]
            rgba = get_geom_rgba(model, i)
            material = g.MeshLambertMaterial(
                color=int('%02X%02X%02X' % tuple((rgba[:3]*255).astype(int)), 16),
                opacity=float(rgba[3])
            )
            vis["robot"][geom_name].set_object(g.Sphere(float(size[0])), material)
            vis["robot"][geom_name].set_transform(T)

# 2. 매 프레임마다 robot geom 위치 업데이트 — PLANE(env)만 제외
_R_FIX_CYL = np.array([[1,0,0],[0,0,-1],[0,1,0]], dtype=float)

def update_visualizer(vis, model, data):
    for i in range(model.ngeom):
        geom_type = model.geom_type[i]
        if geom_type == mujoco.mjtGeom.mjGEOM_PLANE:
            continue
        geom_name = f"geom_{i}"
        pos = data.geom_xpos[i]
        rot = data.geom_xmat[i].reshape(3, 3)
        T = np.eye(4)
        T[:3, 3] = pos
        if geom_type == mujoco.mjtGeom.mjGEOM_CYLINDER:
            T[:3, :3] = rot @ _R_FIX_CYL
        else:
            T[:3, :3] = rot
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

    # UR5e "home" 키프레임으로 초기화 (J1=-90°, J2=-90°, J3=90°, J4=-90°, J5=-90°, J6=0°)
    # 기본 qpos=0은 팔이 완전히 접힌 퇴화 자세라 IK 수렴이 불가능하다.
    home_key = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "home")
    if home_key >= 0:
        mujoco.mj_resetDataKeyframe(model, data, home_key)
        print(f"✅ 'home' 키프레임으로 초기화 완료 (key_id={home_key})")
    else:
        # fallback: 수동으로 home 자세 설정
        data.qpos[:6] = np.array([-1.5708, -1.5708, 1.5708, -1.5708, -1.5708, 0.0])
        print("⚠️  'home' 키프레임 없음 → 수동 초기화")

    # 초기 geom_xpos / geom_xmat 계산 (이 없으면 초기 위치가 전부 0)
    mujoco.mj_forward(model, data)

    # 로봇 메쉬 등록 수행
    setup_meshcat_robot(model, data, vis)

    # --- 제어기 객체 생성 및 스레드 실행 ---
    controller = UR5eController(model, data,
                               gripper_cfg=gripper_config.ACTIVE)
    controller.start_ros_node()
    ctrl_thread = threading.Thread(target=controller.start, daemon=True)
    ctrl_thread.start()

    # --- Anti-Slip 반사 제어 (GraspController 기반) ---
    grasp_adapter = RobotiqGraspAdapter(model, data, controller)
    grasp_adapter.start_thread()

    # --- 물체 접근 노드 ---
    approach_node = ObjectApproachNode(model, data, hover_height=0.05)
    approach_node.start_thread()

    # --- 메인 루프 (시각화 및 모니터링) ---
    print("\n🚀 제어 및 시각화 루프 가동 중...")
    vis_frame = 0
    try:
        while True:
            # 1. 시각화 업데이트 (10Hz: 5프레임마다 1회 — ZMQ 과부하 방지)
            vis_frame += 1
            if vis_frame % 20 == 0:
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
                    f" | EE(wrist): [{ee_pos[0]:.3f}, {ee_pos[1]:.3f}, {ee_pos[2]:.3f}]"
                    f" | TGT: [{tgt[0]:.3f}, {tgt[1]:.3f}, {tgt[2]:.3f}]"
                    f" | err: {err:.4f}",
                    end=""
                )

            time.sleep(0.02)

    except KeyboardInterrupt:
        print("\n\n👋 시뮬레이션을 안전하게 종료합니다.")

if __name__ == "__main__":
    main()