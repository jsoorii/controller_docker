"""
현재 로봇 관절 상태를 렌더링해서 출력

사용법:
  python3 render_snapshot.py           → 단일 PNG (base64)
  python3 render_snapshot.py --frames  → 24방향 JPEG 배열 (JSON)
"""
import base64
import json
import re
import subprocess
import sys
from io import BytesIO

import mujoco
from PIL import Image

MODEL_PATH = "/ros2_ws/src/mujoco_menagerie/universal_robots_ur5e/ur5e.xml"

model = mujoco.MjModel.from_xml_path(MODEL_PATH)
data = mujoco.MjData(model)

# 현재 관절 상태 읽기
try:
    result = subprocess.run(
        ["bash", "-c",
         "source /opt/ros/humble/setup.bash && "
         "timeout 3 ros2 topic echo --once /ur5e/state/joint 2>/dev/null"],
        capture_output=True, text=True, timeout=5
    )
    out = result.stdout
    if "position:" in out:
        section = out.split("position:")[1].split("velocity:")[0]
        positions = re.findall(r"[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?", section)
        qpos = [float(p) for p in positions[:model.njnt]]
        for i, q in enumerate(qpos):
            data.qpos[i] = q
except Exception:
    pass

mujoco.mj_forward(model, data)

N_AZ = 24                                          # 방위각 단계 (15°씩 × 24 = 360°)
N_EL = 5                                           # 앙각 단계
ELEVATIONS = [-55.0, -40.0, -25.0, -10.0, 5.0]   # 아래→위 순서
LOOKAT = [0.0, 0.0, 0.5]
DISTANCE = 2.0


def make_cam(azimuth: float, elevation: float) -> mujoco.MjvCamera:
    cam = mujoco.MjvCamera()
    cam.lookat[:] = LOOKAT
    cam.distance = DISTANCE
    cam.azimuth = azimuth
    cam.elevation = elevation
    return cam


def render_frame(renderer: mujoco.Renderer, azimuth: float, elevation: float,
                 jpeg: bool = False) -> str:
    renderer.update_scene(data, camera=make_cam(azimuth, elevation))
    pixels = renderer.render()
    img = Image.fromarray(pixels)
    buf = BytesIO()
    if jpeg:
        img.save(buf, format="JPEG", quality=72)
    else:
        img.save(buf, format="PNG", optimize=True)
    return base64.b64encode(buf.getvalue()).decode()


if "--frames" in sys.argv:
    # 3D 터닝테이블 모드: elevation × azimuth 격자 (N_EL × N_AZ = 120프레임)
    # frames[el_idx * N_AZ + az_idx] 순서로 저장
    renderer = mujoco.Renderer(model, height=360, width=360)
    frames = [
        render_frame(renderer, 360.0 / N_AZ * az_i, ELEVATIONS[el_i], jpeg=True)
        for el_i in range(N_EL)
        for az_i in range(N_AZ)
    ]
    print(json.dumps({"frames": frames, "n_az": N_AZ, "n_el": N_EL}), end="")
else:
    # 단일 PNG 모드 (기존 /api/robot_image 호환)
    renderer = mujoco.Renderer(model, height=480, width=640)
    print(render_frame(renderer, azimuth=135, elevation=-25), end="")
