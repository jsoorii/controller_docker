"""
sim_viewer_combined.py
──────────────────────
UR5e + Robotiq 2F-85 결합 ReGraspReflex 시뮬레이션 + MJPEG HTTP 서버.

DOF 매핑 (n_fingers=2, n_joints=2):
  q[0] = Robotiq 그리퍼 close ratio  (0=열림, 1=닫힘)
  q[1] = UR5e EE Y 위치             (meters, 현재값 트래킹)

ReGraspReflex 출력 해석:
  (q_target[0,0] + q_target[1,0]) / 2   → 그리퍼 close ratio
  (q_target[0,0] - q_target[1,0]) * K   → 팔 Y lateral 이동량

실행 (Docker 내):
  python3 sim_viewer_combined.py [--port 7101]

접속:
  대시보드 /api/sim/combined_stream
"""

import argparse
import os
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import cv2
import mujoco
import numpy as np

import gripper_config
import scene_config
from ur5e_rt_controller import UR5eRTController, ControlState
from grasp_controller import (
    ContactState, FingerState, MotorState,
    ReflexCoordinator, ReflexState, ReGraspReflex,
)

# ── 씬 로드 ───────────────────────────────────────────────────────────────────
UR5E_SCENE = "/ros2_ws/src/mujoco_menagerie/universal_robots_ur5e/scene.xml"


def _load_scene():
    spec = mujoco.MjSpec.from_file(UR5E_SCENE)
    gripper_config.attach(spec)
    scene_config.add_objects(spec)
    return spec.compile()


# ── 공유 프레임 버퍼 ──────────────────────────────────────────────────────────
_frame_lock   = threading.Lock()
_current_jpeg: bytes | None = None


def _put_frame(rgb: np.ndarray, phase: str, state_name: str,
               grip_pct: float, arm_y: float) -> None:
    global _current_jpeg
    img = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)

    overlay = img.copy()
    cv2.rectangle(overlay, (0, 0), (img.shape[1], 64), (20, 20, 20), -1)
    cv2.addWeighted(overlay, 0.78, img, 0.22, 0, img)

    cv2.putText(img, phase, (12, 26),
                cv2.FONT_HERSHEY_SIMPLEX, 0.58, (180, 255, 140), 2, cv2.LINE_AA)

    state_colors = {
        "IDLE":      (160, 220, 160),
        "TRIGGERED": (80,  160, 255),
        "EXECUTING": (80,   80, 255),
        "RESOLVED":  (80,  255, 200),
    }
    sc = state_colors.get(state_name, (200, 200, 200))
    cv2.putText(img, f"Reflex: {state_name}", (12, 46),
                cv2.FONT_HERSHEY_SIMPLEX, 0.40, sc, 1, cv2.LINE_AA)

    info = f"Grip {grip_pct:5.1f}%  ArmY {arm_y:+.3f}m"
    cv2.putText(img, info, (img.shape[1] - 260, 46),
                cv2.FONT_HERSHEY_SIMPLEX, 0.38, (200, 200, 200), 1, cv2.LINE_AA)

    ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 82])
    if ok:
        with _frame_lock:
            _current_jpeg = buf.tobytes()


# ── MJPEG HTTP 핸들러 ──────────────────────────────────────────────────────────
class MJPEGHandler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args): pass

    def do_GET(self):
        self.send_response(200)
        self.send_header("Content-Type",
                         "multipart/x-mixed-replace; boundary=frame")
        self.end_headers()
        try:
            while True:
                with _frame_lock:
                    frame = _current_jpeg
                if frame:
                    self.wfile.write(
                        b"--frame\r\nContent-Type: image/jpeg\r\n\r\n"
                        + frame + b"\r\n"
                    )
                    self.wfile.flush()
                time.sleep(1 / 30)
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass


def _start_http_server(port: int) -> None:
    server = ThreadingHTTPServer(("0.0.0.0", port), MJPEGHandler)
    t = threading.Thread(target=server.serve_forever, daemon=True)
    t.start()


# ── 시뮬레이션 상수 ────────────────────────────────────────────────────────────
# UR5e 기준 그리퍼 목표 위치 (씬 좌표)
PREGRASP_POS = np.array([0.30,  0.00, 0.22])   # 물체 위 대기
GRASP_POS    = np.array([0.30,  0.00, 0.10])    # 파지 높이
GRASP_ROT    = np.array([[ 1,  0,  0],           # EE 하향 (Z=-arm front)
                          [ 0, -1,  0],
                          [ 0,  0, -1]], dtype=float)

# ReGraspReflex 비대칭 / 대칭 합성 접촉 위치
ASYM_POS = [[+0.60, 0.0], [-0.10, 0.0]]
SYM_POS  = [[+0.05, 0.0], [-0.05, 0.0]]

# 팔 lateral 이동 스케일: q_target 차이 1 → Y_delta (m)
ARM_LATERAL_SCALE = 0.04   # 최대 ±4cm 이동


def _synth_contact(pos_list, force_n=1.5):
    return [
        ContactState(
            force_3axis=np.array([0.0, 0.0, force_n]),
            contact_pos=np.array(pos_list[fi]),
            in_contact=True,
        )
        for fi in range(2)
    ]


def _no_contact():
    return [
        ContactState(force_3axis=np.zeros(3),
                     contact_pos=np.zeros(2), in_contact=False)
        for _ in range(2)
    ]


def _motor_states(controller: UR5eRTController) -> list:
    """현재 그리퍼 + EE Y 위치를 MotorState[2] 로 반환."""
    span = (controller._gripper_ctrl_close - controller._gripper_ctrl_open) + 1e-9
    grip = float(np.clip(
        (controller._gripper_ctrl - controller._gripper_ctrl_open) / span, 0.0, 1.0
    ))
    ee_pos, _ = controller.get_ee_pose()
    ee_y = float(ee_pos[1])
    return [
        MotorState(q=np.array([grip, ee_y]), qdot=np.zeros(2), current=np.zeros(2)),
        MotorState(q=np.array([grip, ee_y]), qdot=np.zeros(2), current=np.zeros(2)),
    ]


def _apply_reflex_output(controller: UR5eRTController,
                          reflex: ReGraspReflex,
                          base_arm_y: float) -> float:
    """
    ReGraspReflex _q_target → 그리퍼 + 팔 lateral 명령 적용.
    Returns: 갱신된 base_arm_y.
    """
    qt = reflex._q_target          # shape (2, 2)

    # DOF 0: 그리퍼 (평균)
    grip_ratio = float(np.clip((qt[0, 0] + qt[1, 0]) / 2, 0.0, 1.0))
    controller.set_gripper(grip_ratio)

    # DOF 1: 팔 Y lateral (차이 → 이동량)
    # q_target[0,0] > q_target[1,0]  → f0(좌) 더 닫힘 → 팔을 -Y(좌) 로 이동
    lateral_delta = (qt[0, 0] - qt[1, 0]) * ARM_LATERAL_SCALE
    new_y = base_arm_y - lateral_delta   # 부호: f0 더 닫힘 → 팔이 -Y 이동
    new_y = float(np.clip(new_y, -0.10, 0.10))

    cur_target = controller.task_cmd["pos_target"].copy()
    cur_target[1] = new_y
    controller.task_cmd["pos_target"] = cur_target
    return new_y


# ── 시뮬레이션 루프 ────────────────────────────────────────────────────────────
def run_sim_loop(model, data, controller: UR5eRTController,
                 renderer: mujoco.Renderer, camera: mujoco.MjvCamera) -> None:

    RENDER_EVERY = 4   # ~125Hz → ~30fps
    FINGER_STATES = [FingerState(), FingerState()]
    DT = model.opt.timestep

    def ctrl_step():
        controller.controller_run()
        mujoco.mj_step(model, data)

    def render(phase, state_name, controller):
        span = (controller._gripper_ctrl_close - controller._gripper_ctrl_open) + 1e-9
        grip_pct = float(np.clip(
            (controller._gripper_ctrl - controller._gripper_ctrl_open) / span, 0, 1
        )) * 100
        ee_pos, _ = controller.get_ee_pose()
        renderer.update_scene(data, camera)
        _put_frame(renderer.render(), phase, state_name, grip_pct, float(ee_pos[1]))

    step_count = 0

    while True:
        # ── 리셋 ────────────────────────────────────────────────────────────
        mujoco.mj_resetData(model, data)
        controller.motor_cmd["q_target"] = data.qpos[:6].copy()
        controller.task_cmd["use_task_space"] = True
        controller.task_cmd["pos_target"] = PREGRASP_POS.copy()
        controller.task_cmd["rot_target"] = GRASP_ROT.copy()
        controller.set_gripper(0.0)   # 열림
        controller.state = ControlState.RUN_CONTROL
        controller._gripper_ctrl = controller._gripper_ctrl_open

        # ── Phase 0: 대기 위치로 이동 (2 s) ──────────────────────────────
        for s in range(1000):
            ctrl_step()
            step_count += 1
            if step_count % RENDER_EVERY == 0:
                render("Phase 0  Pre-grasp", "IDLE", controller)

        # ── Phase 1: 파지 위치로 하강 + 그리퍼 닫기 (2 s) ────────────────
        controller.task_cmd["pos_target"] = GRASP_POS.copy()
        for s in range(500):
            ctrl_step()
            step_count += 1
            if step_count % RENDER_EVERY == 0:
                render("Phase 1  Descend", "IDLE", controller)

        controller.set_gripper(0.6)  # 60% 닫기
        for s in range(500):
            ctrl_step()
            step_count += 1
            if step_count % RENDER_EVERY == 0:
                render("Phase 1  Close gripper", "IDLE", controller)

        # ── Phase 2: 비대칭 접촉 → ReGraspReflex 발동 ───────────────────
        reflex = ReGraspReflex(n_fingers=2, n_joints=2,
                               sym_threshold=0.85, dq_step=0.005)
        coord  = ReflexCoordinator([reflex])
        asym_c = _synth_contact(ASYM_POS)
        sym_c  = _synth_contact(SYM_POS)

        # 팔 Y 기준점: 현재 EE Y 위치
        base_arm_y = float(controller.get_ee_pose()[0][1])

        for s in range(500):
            contact = asym_c if s < 200 else sym_c
            ms      = _motor_states(controller)
            result  = coord.update(FINGER_STATES, contact, ms, DT)

            if result is not None:
                base_arm_y = _apply_reflex_output(controller, reflex, base_arm_y)

            ctrl_step()
            step_count += 1
            if step_count % RENDER_EVERY == 0:
                render("Phase 2  ReGrasp Reflex", reflex.state.name, controller)

            if reflex.state == ReflexState.IDLE and s > 210:
                break

        # ── Phase 3: 해소 후 대기 ─────────────────────────────────────────
        for s in range(300):
            ctrl_step()
            step_count += 1
            if step_count % RENDER_EVERY == 0:
                render("Resolved — resetting…", "IDLE", controller)


# ── 메인 ──────────────────────────────────────────────────────────────────────
def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port",   type=int, default=7101)
    parser.add_argument("--width",  type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    args = parser.parse_args()

    model = _load_scene()
    data  = mujoco.MjData(model)
    controller  = UR5eRTController(model, data, gripper_cfg=gripper_config.ACTIVE)

    renderer = mujoco.Renderer(model, height=args.height, width=args.width)

    # 씬 전체가 보이는 카메라 (측면 + 약간 위에서)
    camera = mujoco.MjvCamera()
    camera.type      = mujoco.mjtCamera.mjCAMERA_FREE
    camera.lookat[:] = [0.30, 0.0, 0.15]
    camera.distance  = 1.0
    camera.elevation = -20.0
    camera.azimuth   = 160.0

    _start_http_server(args.port)
    print(f"[sim_combined] MJPEG server: http://0.0.0.0:{args.port}/", flush=True)
    run_sim_loop(model, data, controller, renderer, camera)


if __name__ == "__main__":
    main()
