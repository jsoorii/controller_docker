"""
sim_viewer_server.py
--------------------
2-finger gripper MuJoCo 시뮬레이션 + MJPEG HTTP 서버 (기본 포트 7100).

실행 (Docker 내):
  python3 sim_viewer_server.py [--port 7100] [--width 640] [--height 480]

접속:
  http://localhost:7100/  → MJPEG 스트림
  대시보드:  /api/sim/gripper_stream  (FastAPI 프록시)
"""

import argparse
import os
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

sys.path.insert(0, os.path.dirname(__file__))

import cv2
import mujoco
import numpy as np

from grasp_controller import (
    ContactState,
    FingerState,
    MotorState,
    ReflexCoordinator,
    ReflexState,
    ReGraspReflex,
)

# ── 모델 로드 ──────────────────────────────────────────────────────────────────
_HERE = os.path.dirname(os.path.abspath(__file__))
MODEL_PATH = os.path.join(_HERE, "two_finger_gripper.xml")
model = mujoco.MjModel.from_xml_path(MODEL_PATH)
data  = mujoco.MjData(model)
DT    = model.opt.timestep

# ── 관절 주소 ──────────────────────────────────────────────────────────────────
def _jid(name):
    return mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)

_joints = [_jid(n) for n in ["j0_prox", "j0_dist", "j1_prox", "j1_dist"]]
QA = [model.jnt_qposadr[j] for j in _joints]
VA = [model.jnt_dofadr[j]  for j in _joints]

CLOSE_ANGLE  = 0.18
ASYM_POS     = [[+0.60, 0.0], [-0.10, 0.0]]
SYM_POS      = [[+0.05, 0.0], [-0.05, 0.0]]
FINGER_STATES = [FingerState(), FingerState()]

# ── 공유 프레임 버퍼 ───────────────────────────────────────────────────────────
_frame_lock   = threading.Lock()
_current_jpeg: bytes | None = None


def _put_frame(rgb: np.ndarray, phase: str, state_name: str) -> None:
    global _current_jpeg
    img = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)

    # 상단 정보 바
    overlay = img.copy()
    cv2.rectangle(overlay, (0, 0), (img.shape[1], 56), (25, 25, 25), -1)
    cv2.addWeighted(overlay, 0.78, img, 0.22, 0, img)

    cv2.putText(img, phase, (12, 26),
                cv2.FONT_HERSHEY_SIMPLEX, 0.60, (180, 255, 140), 2, cv2.LINE_AA)

    state_colors = {
        "IDLE":      (160, 220, 160),
        "TRIGGERED": (80,  160, 255),
        "EXECUTING": (80,   80, 255),
        "RESOLVED":  (80,  255, 200),
    }
    sc = state_colors.get(state_name, (200, 200, 200))
    cv2.putText(img, f"Reflex: {state_name}", (12, 48),
                cv2.FONT_HERSHEY_SIMPLEX, 0.42, sc, 1, cv2.LINE_AA)

    ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 82])
    if ok:
        with _frame_lock:
            _current_jpeg = buf.tobytes()


# ── MJPEG HTTP 핸들러 ──────────────────────────────────────────────────────────
class MJPEGHandler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):
        pass  # 로그 무음

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
                        + frame
                        + b"\r\n"
                    )
                    self.wfile.flush()
                time.sleep(1 / 30)
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass


def _start_http_server(port: int) -> None:
    server = ThreadingHTTPServer(("0.0.0.0", port), MJPEGHandler)
    t = threading.Thread(target=server.serve_forever, daemon=True)
    t.start()


# ── 시뮬레이션 유틸 ────────────────────────────────────────────────────────────
def _get_q():
    return np.array([data.qpos[a] for a in QA])

def _get_qdot():
    return np.array([data.qvel[a] for a in VA])

def _motor_states():
    q, v = _get_q(), _get_qdot()
    return [
        MotorState(q=q[0:2].copy(), qdot=v[0:2].copy(), current=np.zeros(2)),
        MotorState(q=q[2:4].copy(), qdot=v[2:4].copy(), current=np.zeros(2)),
    ]

def _synth_contact(pos_list, force_n=1.5):
    return [
        ContactState(
            force_3axis=np.array([0.0, 0.0, force_n]),
            contact_pos=np.array(pos_list[fi]),
            in_contact=True,
        )
        for fi in range(2)
    ]

NO_CONTACT = [
    ContactState(force_3axis=np.zeros(3), contact_pos=np.zeros(2), in_contact=False)
    for _ in range(2)
]


# ── 시뮬레이션 루프 ────────────────────────────────────────────────────────────
def run_sim_loop(renderer: mujoco.Renderer, camera: mujoco.MjvCamera) -> None:
    RENDER_EVERY = 6   # 0.002s × 6 = 0.012s → ~83Hz sim, ~14fps 렌더
    reflex_state_name = "IDLE"

    while True:
        # ── Phase 0: 오픈 ────────────────────────────────────────────────────
        mujoco.mj_resetData(model, data)
        reflex = ReGraspReflex(n_fingers=2, n_joints=2, sym_threshold=0.85, dq_step=0.005)
        coord  = ReflexCoordinator([reflex])

        for s in range(80):
            coord.update(FINGER_STATES, NO_CONTACT, _motor_states(), DT)
            mujoco.mj_step(model, data)
            if s % RENDER_EVERY == 0:
                renderer.update_scene(data, camera)
                _put_frame(renderer.render(), "Phase 0  Open", reflex.state.name)

        # ── Phase 1: 물체에 닫기 ─────────────────────────────────────────────
        data.ctrl[:] = [CLOSE_ANGLE] * 4
        for s in range(700):
            mujoco.mj_step(model, data)
            if s % RENDER_EVERY == 0:
                renderer.update_scene(data, camera)
                _put_frame(renderer.render(), "Phase 1  Close → Contact", reflex.state.name)

        # ── Phase 2: 비대칭 접촉 → ReGraspReflex 발동 ───────────────────────
        reflex = ReGraspReflex(n_fingers=2, n_joints=2, sym_threshold=0.85, dq_step=0.005)
        coord  = ReflexCoordinator([reflex])
        asym_c = _synth_contact(ASYM_POS)
        sym_c  = _synth_contact(SYM_POS)
        data.ctrl[:] = [CLOSE_ANGLE] * 4

        for s in range(400):
            contact = asym_c if s < 150 else sym_c
            result  = coord.update(FINGER_STATES, contact, _motor_states(), DT)
            if result is not None:
                qt = reflex._q_target
                data.ctrl[:] = [qt[0, 0], qt[0, 1], qt[1, 0], qt[1, 1]]
            else:
                data.ctrl[:] = [CLOSE_ANGLE] * 4
            mujoco.mj_step(model, data)
            if s % RENDER_EVERY == 0:
                renderer.update_scene(data, camera)
                _put_frame(renderer.render(), "Phase 2  ReGrasp Reflex", reflex.state.name)
            if reflex.state == ReflexState.IDLE and s > 160:
                break

        # ── 리셋 대기 ────────────────────────────────────────────────────────
        for s in range(100):
            mujoco.mj_step(model, data)
            if s % RENDER_EVERY == 0:
                renderer.update_scene(data, camera)
                _put_frame(renderer.render(), "Resetting…", "IDLE")


# ── 메인 ──────────────────────────────────────────────────────────────────────
def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port",   type=int, default=7100)
    parser.add_argument("--width",  type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    args = parser.parse_args()

    renderer = mujoco.Renderer(model, height=args.height, width=args.width)

    camera = mujoco.MjvCamera()
    camera.type      = mujoco.mjtCamera.mjCAMERA_FREE
    camera.lookat[:] = [0.0, 0.0, 0.075]
    camera.distance  = 0.22
    camera.elevation = -18.0
    camera.azimuth   = 30.0

    _start_http_server(args.port)
    print(f"[sim_viewer] MJPEG server: http://0.0.0.0:{args.port}/", flush=True)
    run_sim_loop(renderer, camera)


if __name__ == "__main__":
    main()
