import mujoco
import meshcat
import meshcat.geometry as g
import numpy as np
import time
import os

# 1. 모델 및 메쉬 경로 설정
# menagerie 구조상 xml 파일 위치를 기준으로 메쉬 폴더가 결정됩니다.
model_path = "/ros2_ws/src/mujoco_menagerie/universal_robots_ur5e/ur5e.xml"
model = mujoco.MjModel.from_xml_path(model_path)
data = mujoco.MjData(model)

# 2. Meshcat 초기화
vis = meshcat.Visualizer()
print(f"\n🚀 Meshcat 접속 주소: {vis.url()}\n")
vis.delete()

# 3. MuJoCo 메쉬를 Meshcat으로 복사하는 함수
def setup_ur5e_meshes(model, vis):
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

setup_ur5e_meshes(model, vis)

# 4. 시뮬레이션 루프
print("실제 메쉬로 시뮬레이션 중... 브라우저를 확인하세요.")
try:
    while True:
        mujoco.mj_step(model, data)
        
        for i in range(model.ngeom):
            geom_name = f"geom_{i}"
            # 위치 및 회전 행렬 적용
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
        
        # 동작 테스트 (Shoulder 구동)
        data.ctrl[1] = np.sin(time.time() * 1.5) * 0.8
        
        time.sleep(model.opt.timestep)

except KeyboardInterrupt:
    print("종료합니다.")