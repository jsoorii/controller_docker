import mujoco
import meshcat
import meshcat.geometry as g
import numpy as np
import time
import threading
import sys

# 우리가 만든 RT 제어기 클래스 임포트
from ur5e_rt_controller import UR5eRTController, ControlState

# 1. Meshcat에 로봇 메쉬 등록 (이게 실행되어야 화면에 보임)
def setup_meshcat_robot(model, vis):
    print("📦 로봇 메쉬 등록 중...")
    for i in range(model.ngeom):
        geom_name = f"geom_{i}"
        geom_type = model.geom_type[i]
        
        # 메쉬 타입(Type 7)인 경우 처리
        if geom_type == mujoco.mjtGeom.mjGEOM_MESH:
            mesh_id = model.geom_dataid[i]
            # MuJoCo 메쉬 데이터를 추출
            vert_adr = model.mesh_vertadr[mesh_id]
            vert_num = model.mesh_vertnum[mesh_id]
            face_adr = model.mesh_faceadr[mesh_id]
            face_num = model.mesh_facenum[mesh_id]
            
            vertices = model.mesh_vert[vert_adr : vert_adr + vert_num]
            faces = model.mesh_face[face_adr : face_adr + face_num]
            
            # Meshcat용 Geometry 생성 (Vertices와 Faces 전달)
            # vertices.T는 3xN 형태여야 함
            geom = g.TriangularMeshGeometry(vertices, faces)
            
            # 색상 설정 (RGBA)
            rgba = model.geom_rgba[i]
            material = g.MeshLambertMaterial(
                color=int('%02X%02X%02X' % tuple((rgba[:3]*255).astype(int)), 16),
                opacity=float(rgba[3])
            )
            
            vis["robot"][geom_name].set_object(geom, material)
        
        # 기본 도형(Plane, Box 등)인 경우 간단히 처리
        elif geom_type == mujoco.mjtGeom.mjGEOM_PLANE:
            vis["env"][geom_name].set_object(g.Box([10, 10, 0.01]))

# 2. 매 프레임마다 로봇 위치 업데이트
def update_visualizer(vis, model, data):
    for i in range(model.ngeom):
        geom_name = f"geom_{i}"
        pos = data.geom_xpos[i]
        rot = data.geom_xmat[i].reshape(3, 3)
        
        T = np.eye(4)
        T[:3, :3] = rot
        T[:3, 3] = pos
        
        # 등록된 타입에 따라 경로 분기
        if model.geom_type[i] == mujoco.mjtGeom.mjGEOM_MESH:
            vis["robot"][geom_name].set_transform(T)
        elif model.geom_type[i] == mujoco.mjtGeom.mjGEOM_PLANE:
            vis["env"][geom_name].set_transform(T)

def main():
    # --- 모델 로드 ---
    model_path = "/ros2_ws/src/mujoco_menagerie/universal_robots_ur5e/ur5e.xml"
    try:
        model = mujoco.MjModel.from_xml_path(model_path)
        data = mujoco.MjData(model)
        print("✅ MuJoCo 모델 로드 완료")
    except Exception as e:
        print(f"❌ 모델 로드 실패: {e}")
        return

    # --- Meshcat 초기화 ---
    vis = meshcat.Visualizer()
    print(f"\n🌐 Meshcat 접속 주소: {vis.url()}")
    vis.delete() # 기존 잔상 제거
    
    # 로봇 메쉬 등록 수행
    setup_meshcat_robot(model, vis)

    # --- 제어기 객체 생성 및 스레드 실행 ---
    controller = UR5eRTController(model, data)
    ctrl_thread = threading.Thread(target=controller.start, daemon=True)
    ctrl_thread.start()

    # --- 메인 루프 (시각화 및 모니터링) ---
    print("\n🚀 제어 및 시각화 루프 가동 중...")
    start_time = time.time()
    
    try:
        while True:
            # 1. 시각화 업데이트
            update_visualizer(vis, model, data)
            
            # 폭발 감지 로그
            if np.any(np.abs(data.qvel) > 50):
                print(f"\n⚠️ 물리 불안정 감지! qvel: {data.qvel}")
            
            # 2. 상태 모니터링 출력
            print(f"\r[상태]: {controller.state.name} | Missed: {controller.miss_count} | Joint[1]: {data.qpos[1]:.2f}", end="")
            
            # 3. 테스트 동작 (사인파)
            if controller.state == ControlState.RUN_CONTROL:
                elapsed = time.time() - start_time
                controller.motor_cmd["q_target"][1] = -1.57 + 0.4 * np.sin(elapsed * 1.5)
            
            # [중요] 1초에서 0.02초로 변경
            time.sleep(0.02)

    except KeyboardInterrupt:
        print("\n\n👋 시뮬레이션을 안전하게 종료합니다.")

if __name__ == "__main__":
    main()