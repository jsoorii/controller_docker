import mujoco
import meshcat
import meshcat.geometry as g
import numpy as np
import time
import sys

# 1. 모델 경로 설정 (경로가 정확한지 확인하세요)
model_path = "/ros2_ws/src/mujoco_menagerie/universal_robots_ur5e/ur5e.xml"

try:
    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)
    print("MuJoCo 모델 로드 성공!")
except Exception as e:
    print(f"모델 로드 실패: {e}")
    sys.exit(1)

# 2. Meshcat 비주얼라이저 초기화 (자동 서버 시작)
# zmq_url을 생략하면 내부적으로 서버를 생성합니다.
vis = meshcat.Visualizer() 
print("-" * 50)
print(f"🚀 Meshcat 접속 주소: {vis.url()}")
print("-" * 50)

# 기존 잔상 제거
vis.delete()

# 3. MuJoCo Geoms(기하 도형)를 Meshcat 객체로 등록
# UR5e의 각 링크 위치에 5cm 크기의 박스를 생성합니다.
def setup_meshcat_robot(model, vis):
    for i in range(model.ngeom):
        geom_name = f"link_{i}"
        # 색상을 구분하기 위해 간단한 RGB 값 부여
        color = [0.1, 0.5, 0.9] if i % 2 == 0 else [0.8, 0.2, 0.2]
        vis[geom_name].set_object(
            g.Box([0.06, 0.06, 0.06]), 
            g.MeshLambertMaterial(color=int('%02x%02x%02x' % tuple(int(c*255) for c in color), 16))
        )

setup_meshcat_robot(model, vis)

# 4. 시뮬레이션 루프
print("시뮬레이션을 시작합니다. Ctrl+C를 누르면 종료됩니다.")
try:
    while True:
        # 물리 엔진 스텝 진행
        mujoco.mj_step(model, data)
        
        # 5. 각 기하 도형의 위치 및 회전 업데이트
        for i in range(model.ngeom):
            pos = data.geom_xpos[i]
            rot = data.geom_xmat[i].reshape(3, 3)
            
            # 4x4 변환 행렬 구성
            T = np.eye(4)
            T[:3, :3] = rot
            T[:3, 3] = pos
            
            # Meshcat 노드에 적용
            vis[f"link_{i}"].set_transform(T)
        
        # 테스트: 두 번째 관절(Shoulder Lift)을 흔들어 로봇이 살아있는지 확인
        data.ctrl[1] = np.sin(time.time() * 2.0) * 0.7
        
        # 실제 시뮬레이션 시간 속도 맞추기
        time.sleep(model.opt.timestep)

except KeyboardInterrupt:
    print("\n시뮬레이션을 종료합니다.")