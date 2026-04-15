"""
Docker 웹 대시보드 - ur5e_mujoco_ros2 컨테이너 제어
"""
import asyncio
import json
import os
import secrets
import subprocess
from pathlib import Path
from typing import AsyncGenerator

import httpx
import websockets as ws_lib

import docker
from dotenv import load_dotenv
from fastapi import Depends, FastAPI, HTTPException, Request, WebSocket, WebSocketDisconnect
from fastapi.middleware.gzip import GZipMiddleware
from fastapi.responses import HTMLResponse, Response, StreamingResponse
from fastapi.security import HTTPBasic, HTTPBasicCredentials
from fastapi.staticfiles import StaticFiles
from sse_starlette.sse import EventSourceResponse

import re
import socket
import time

load_dotenv(Path(__file__).parent / ".env")

# ── 핸드 UI 설정 로드 ─────────────────────────────────────────────────────────
import sys as _sys
_sys.path.insert(0, str(Path(__file__).parent))
from hand_ui_config import ACTIVE as _HAND_CONFIG

COMPOSE_DIR = Path(__file__).parent.parent
CONTAINER_NAME = "ur5e_mujoco_ros2"

# ── MeshCat 포트 동적 탐지 ─────────────────────────────────────────────────────
_CONTAINER_LOG_PATHS = ["/tmp/bg.log", "/tmp/main_test.log"]

def _find_meshcat_port() -> int:
    """컨테이너 내부 로그에서 MeshCat URL을 읽어 포트를 반환. 실패 시 스캔."""
    try:
        container = docker.from_env().containers.get(CONTAINER_NAME)
        for log_path in _CONTAINER_LOG_PATHS:
            try:
                _, raw = container.exec_run(f"cat {log_path}", demux=False)
                text = raw.decode(errors="ignore") if raw else ""
                matches = re.findall(r"http://127\.0\.0\.1:(\d+)", text)
                if matches:
                    port = int(matches[-1])
                    with socket.create_connection(("localhost", port), timeout=1):
                        return port
            except Exception:
                continue
    except Exception:
        pass
    # 로그에서 못 찾으면 7000~7010 중 응답하는 포트를 높은 번호부터 탐색 (최신 인스턴스 우선)
    for port in range(7010, 6999, -1):
        try:
            with socket.create_connection(("localhost", port), timeout=0.3):
                return port
        except Exception:
            continue
    return 7000

_meshcat_port_cache: tuple[int, float] = (0, 0.0)
_MESHCAT_PORT_TTL = 5.0  # seconds

def get_meshcat_port() -> int:
    global _meshcat_port_cache
    port, ts = _meshcat_port_cache
    if port and (time.monotonic() - ts) < _MESHCAT_PORT_TTL:
        return port
    port = _find_meshcat_port()
    _meshcat_port_cache = (port, time.monotonic())
    return port

def invalidate_meshcat_port_cache() -> None:
    global _meshcat_port_cache
    _meshcat_port_cache = (0, 0.0)

# ── 인증 설정 (.env 파일에서 로드) ────────────────────────────────────────────
AUTH_USERNAME = os.getenv("AUTH_USERNAME", "heroi")
AUTH_PASSWORD = os.getenv("AUTH_PASSWORD", "")

security = HTTPBasic()
client = docker.from_env()


def require_auth(credentials: HTTPBasicCredentials = Depends(security)):
    ok_user = secrets.compare_digest(credentials.username, AUTH_USERNAME)
    ok_pass = secrets.compare_digest(credentials.password, AUTH_PASSWORD)
    if not (ok_user and ok_pass):
        raise HTTPException(
            status_code=401,
            detail="Unauthorized",
            headers={"WWW-Authenticate": 'Basic realm="HeroiDockerDashboard"'},
        )


app = FastAPI(title="Heroi Docker Dashboard")
app.add_middleware(GZipMiddleware, minimum_size=1000)


def get_container():
    try:
        return client.containers.get(CONTAINER_NAME)
    except docker.errors.NotFound:
        return None


# ── API ──────────────────────────────────────────────────────────────────────


@app.get("/api/status")
def status(_: None = Depends(require_auth)):
    container = get_container()
    if container is None:
        return {"status": "not_found", "name": CONTAINER_NAME}
    container.reload()
    return {
        "status": container.status,
        "name": container.name,
        "id": container.short_id,
        "image": container.image.tags[0] if container.image.tags else "unknown",
    }


async def _rebuild_generator() -> AsyncGenerator[str, None]:
    for step, cmd in [
        ("docker compose down", ["docker", "compose", "down"]),
        ("docker compose up -d --build", ["docker", "compose", "up", "-d", "--build"]),
    ]:
        yield f"▶ {step}\n"
        proc = await asyncio.create_subprocess_exec(
            *cmd,
            cwd=COMPOSE_DIR,
            stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.STDOUT,
        )
        async for line in proc.stdout:
            yield line.decode(errors="replace")
        await proc.wait()
        if proc.returncode != 0:
            yield f"\n✗ 실패 (exit {proc.returncode})\n"
            return
        yield f"✓ 완료\n\n"
    yield "✅ 재빌드 및 재시작 완료\n"


@app.post("/api/rebuild")
async def rebuild(_: None = Depends(require_auth)):
    return StreamingResponse(_rebuild_generator(), media_type="text/plain")


@app.post("/api/start")
def start(_: None = Depends(require_auth)):
    container = get_container()
    if container and container.status == "running":
        return {"ok": True, "message": "Already running"}
    try:
        result = subprocess.run(
            ["docker", "compose", "up", "-d", "--build"],
            cwd=COMPOSE_DIR,
            capture_output=True,
            text=True,
            timeout=300,
        )
        if result.returncode != 0:
            raise HTTPException(status_code=500, detail=result.stderr)
        return {"ok": True, "message": "Started"}
    except subprocess.TimeoutExpired:
        raise HTTPException(status_code=504, detail="Build timed out")


@app.post("/api/stop")
def stop(_: None = Depends(require_auth)):
    container = get_container()
    if container is None:
        raise HTTPException(status_code=404, detail="Container not found")
    container.stop(timeout=10)
    return {"ok": True, "message": "Stopped"}


@app.post("/api/restart")
def restart(_: None = Depends(require_auth)):
    container = get_container()
    if container is None:
        raise HTTPException(status_code=404, detail="Container not found")
    container.restart(timeout=10)
    return {"ok": True, "message": "Restarted"}


@app.post("/api/exec")
async def exec_command(body: dict, _: None = Depends(require_auth)):
    cmd = body.get("cmd", "").strip()
    if not cmd:
        raise HTTPException(status_code=400, detail="cmd is required")
    container = get_container()
    if container is None or container.status != "running":
        raise HTTPException(status_code=409, detail="Container not running")
    exit_code, output = container.exec_run(
        ["/bin/bash", "-c", f"source /opt/ros/humble/setup.bash && {cmd}"],
        stderr=True,
    )
    return {"exit_code": exit_code, "output": output.decode(errors="replace")}


@app.post("/api/exec_bg")
async def exec_bg(body: dict, _: None = Depends(require_auth)):
    cmd = body.get("cmd", "").strip()
    if not cmd:
        raise HTTPException(status_code=400, detail="cmd is required")
    container = get_container()
    if container is None or container.status != "running":
        raise HTTPException(status_code=409, detail="Container not running")
    container.exec_run(
        ["/bin/bash", "-c", f"source /opt/ros/humble/setup.bash && nohup {cmd} > /tmp/bg.log 2>&1"],
        detach=True,
    )
    return {"ok": True, "message": f"백그라운드 실행 시작: {cmd}", "log": "/tmp/bg.log"}


async def _stream_generator(cmd: str) -> AsyncGenerator[str, None]:
    container = get_container()
    if container is None:
        yield "Container not found\n"
        return
    proc = await asyncio.create_subprocess_exec(
        "docker", "exec", container.id,
        "/bin/bash", "-c", f"source /opt/ros/humble/setup.bash && {cmd}",
        stdout=asyncio.subprocess.PIPE,
        stderr=asyncio.subprocess.STDOUT,
    )
    try:
        async for line in proc.stdout:
            yield line.decode(errors="replace")
    finally:
        proc.terminate()
        await proc.wait()


@app.post("/api/exec_stream")
async def exec_stream(body: dict, _: None = Depends(require_auth)):
    cmd = body.get("cmd", "").strip()
    if not cmd:
        raise HTTPException(status_code=400, detail="cmd is required")
    container = get_container()
    if container is None or container.status != "running":
        raise HTTPException(status_code=409, detail="Container not running")
    return StreamingResponse(_stream_generator(cmd), media_type="text/plain")


async def _log_generator(lines: int, request: Request) -> AsyncGenerator[str, None]:
    container = get_container()
    if container is None:
        yield "Container not found\n"
        return
    proc = await asyncio.create_subprocess_exec(
        "docker", "exec", container.id,
        "tail", f"-n{lines}", "-f", "/tmp/bg.log",
        stdout=asyncio.subprocess.PIPE,
        stderr=asyncio.subprocess.STDOUT,
    )
    try:
        while True:
            if await request.is_disconnected():
                break
            try:
                chunk = await asyncio.wait_for(proc.stdout.read(4096), timeout=1.0)
            except asyncio.TimeoutError:
                continue
            if not chunk:
                break
            yield chunk.decode(errors="replace")
    finally:
        try:
            proc.terminate()
        except Exception:
            pass
        await proc.wait()


@app.get("/api/logs")
async def logs(request: Request, lines: int = 200, _: None = Depends(require_auth)):
    return StreamingResponse(
        _log_generator(lines, request),
        media_type="text/plain",
    )


# ── ROS2 Publish ─────────────────────────────────────────────────────────────

@app.post("/api/pub/mode")
async def pub_mode(body: dict, _: None = Depends(require_auth)):
    task_space = body.get("task_space", True)
    cmd = (
        f"source /opt/ros/humble/setup.bash && "
        f"ros2 topic pub --times 3 --wait-matching-subscriptions 0 /ur5e/cmd/mode "
        f"std_msgs/msg/Bool '{{data: {'true' if task_space else 'false'}}}'"
    )
    container = get_container()
    if container is None or container.status != "running":
        raise HTTPException(status_code=409, detail="Container not running")
    exit_code, output = container.exec_run(["/bin/bash", "-c", cmd], stderr=True)
    return {"ok": exit_code == 0, "output": output.decode(errors="replace")}


@app.post("/api/pub/ee")
async def pub_ee(body: dict, _: None = Depends(require_auth)):
    pos = body.get("position", {})
    ori = body.get("orientation", {})
    duration  = float(body.get("duration",  0.0))
    vel_start = float(body.get("vel_start", 0.0))  # 출발 속도 (m/s)
    vel_end   = float(body.get("vel_end",   0.0))  # 도착 속도 (m/s)
    x, y, z   = pos.get("x", 0.0), pos.get("y", 0.0), pos.get("z", 0.0)
    qx, qy, qz, qw = ori.get("x", 0.0), ori.get("y", 0.0), ori.get("z", 0.0), ori.get("w", 1.0)
    # duration → header.stamp, vel_start/vel_end → header.frame_id 에 인코딩
    # (단일 메시지로 원자적 전달 → race condition 없음)
    dur_sec  = int(duration)
    dur_nsec = int(round((duration - dur_sec) * 1_000_000_000))
    frame_id = f"base|{vel_start:.4f}|{vel_end:.4f}"
    pose_cmd = (
        f"ros2 topic pub --times 1 --wait-matching-subscriptions 0 /ur5e/cmd/ee_target "
        f"geometry_msgs/msg/PoseStamped "
        f"'{{header: {{stamp: {{sec: {dur_sec}, nanosec: {dur_nsec}}}, frame_id: \"{frame_id}\"}}, "
        f"pose: {{position: {{x: {x:.6f}, y: {y:.6f}, z: {z:.6f}}}, "
        f"orientation: {{x: {qx:.6f}, y: {qy:.6f}, z: {qz:.6f}, w: {qw:.6f}}}}}}}'"
    )
    cmd = f"source /opt/ros/humble/setup.bash && {pose_cmd}"
    container = get_container()
    if container is None or container.status != "running":
        raise HTTPException(status_code=409, detail="Container not running")
    exit_code, output = container.exec_run(["/bin/bash", "-c", cmd], stderr=True)
    return {"ok": exit_code == 0, "output": output.decode(errors="replace")}


@app.post("/api/pub/joint")
async def pub_joint(body: dict, _: None = Depends(require_auth)):
    joints = body.get("joints", [])
    duration = float(body.get("duration", 2.0))
    if len(joints) != 6:
        raise HTTPException(status_code=400, detail="joints must have 6 values")
    if duration <= 0:
        raise HTTPException(status_code=400, detail="duration must be > 0")
    pos_str = "[" + ", ".join(f"{v:.6f}" for v in joints) + "]"
    joint_names = ("[shoulder_pan_joint, shoulder_lift_joint, elbow_joint,"
                   " wrist_1_joint, wrist_2_joint, wrist_3_joint]")
    dur_sec  = int(duration)
    dur_nsec = int((duration - dur_sec) * 1_000_000_000)
    cmd = (
        f"source /opt/ros/humble/setup.bash && "
        f"ros2 topic pub --times 3 --rate 100 --wait-matching-subscriptions 0 /ur5e/cmd/joint_trajectory "
        f"trajectory_msgs/msg/JointTrajectory "
        f"'{{joint_names: {joint_names}, "
        f"points: [{{positions: {pos_str}, "
        f"time_from_start: {{sec: {dur_sec}, nanosec: {dur_nsec}}}}}]}}'"
    )
    container = get_container()
    if container is None or container.status != "running":
        raise HTTPException(status_code=409, detail="Container not running")
    exit_code, output = container.exec_run(["/bin/bash", "-c", cmd], stderr=True)
    return {"ok": exit_code == 0, "output": output.decode(errors="replace")}


@app.get("/api/sim/status")
async def sim_status(_: None = Depends(require_auth)):
    """컨테이너 내 main_test.py 프로세스 목록을 반환한다."""
    container = get_container()
    if container is None or container.status != "running":
        raise HTTPException(status_code=409, detail="Container not running")
    exit_code, output = container.exec_run(
        ["/bin/bash", "-c", "ps -eo pid,etime,cmd --no-headers | awk '$3==\"python3\" && $4==\"main_test.py\"'"],
        stderr=False,
    )
    lines = output.decode(errors="replace").strip().splitlines()
    procs = []
    for line in lines:
        parts = line.split(None, 2)
        if len(parts) == 3:
            procs.append({"pid": parts[0], "elapsed": parts[1], "cmd": parts[2]})
    return {"count": len(procs), "processes": procs}


@app.post("/api/sim/kill")
async def sim_kill(_: None = Depends(require_auth)):
    """컨테이너 내 main_test.py 프로세스를 모두 종료한다."""
    container = get_container()
    if container is None or container.status != "running":
        raise HTTPException(status_code=409, detail="Container not running")
    exit_code, output = container.exec_run(
        ["/bin/bash", "-c", "pkill -9 -f 'python3 main_test.py'; echo done"],
        stderr=True,
    )
    return {"ok": True, "output": output.decode(errors="replace").strip()}


@app.post("/api/pub/soft_limits")
async def pub_soft_limits(body: dict, _: None = Depends(require_auth)):
    """소프트 리밋 설정을 /ur5e/cmd/soft_limits 토픽으로 발행한다.
    body: {"enabled": bool, "margin": float, "kp": float, "kd": float} — 모두 선택적."""
    import json as _json
    payload = _json.dumps({k: v for k, v in body.items() if k in ("enabled", "margin", "kp", "kd")})
    escaped_payload = payload.replace('"', '\\"')
    cmd = (
        f"source /opt/ros/humble/setup.bash && "
        f"ros2 topic pub --times 3 --wait-matching-subscriptions 0 "
        f"/ur5e/cmd/soft_limits std_msgs/msg/String "
        f"'{{data: \"{escaped_payload}\"}}' "
    )
    container = get_container()
    if container is None or container.status != "running":
        raise HTTPException(status_code=409, detail="Container not running")
    exit_code, output = container.exec_run(["/bin/bash", "-c", cmd], stderr=True)
    return {"ok": exit_code == 0, "output": output.decode(errors="replace")}


@app.get("/api/hand_config")
def hand_config(_: None = Depends(require_auth)):
    """현재 핸드 UI 설정(hand_ui_config.py의 ACTIVE)을 JSON으로 반환한다."""
    return _HAND_CONFIG


@app.post("/api/pub/hand")
async def pub_hand(body: dict, _: None = Depends(require_auth)):
    """핸드 설정에 정의된 ROS topic으로 값을 발행한다.
    body: {topic, msg_type, msg_field, value}"""
    topic    = body.get("topic", "").strip()
    msg_type = body.get("msg_type", "").strip()
    field    = body.get("msg_field", "data")
    value    = body.get("value", 0.0)

    if not topic or not msg_type:
        raise HTTPException(status_code=400, detail="topic and msg_type are required")

    cmd = (
        f"source /opt/ros/humble/setup.bash && "
        f"ros2 topic pub --times 3 --wait-matching-subscriptions 0 "
        f"{topic} {msg_type} '{{  {field}: {float(value)}  }}'"
    )
    container = get_container()
    if container is None or container.status != "running":
        raise HTTPException(status_code=409, detail="Container not running")
    exit_code, output = container.exec_run(["/bin/bash", "-c", cmd], stderr=True)
    return {"ok": exit_code == 0, "output": output.decode(errors="replace")}


@app.post("/api/sim/collision_reset")
async def sim_collision_reset(_: None = Depends(require_auth)):
    """충돌 정지 상태를 해제한다 (/ur5e/cmd/collision_reset 토픽 발행)."""
    container = get_container()
    if container is None or container.status != "running":
        raise HTTPException(status_code=409, detail="Container not running")
    cmd = (
        "source /opt/ros/humble/setup.bash && "
        "ros2 topic pub --times 3 --wait-matching-subscriptions 0 "
        "/ur5e/cmd/collision_reset std_msgs/msg/Bool '{data: true}'"
    )
    exit_code, output = container.exec_run(["/bin/bash", "-c", cmd], stderr=True)
    return {"ok": exit_code == 0, "output": output.decode(errors="replace")}


@app.post("/api/sim/restart")
async def sim_restart(_: None = Depends(require_auth)):
    """컨테이너 내 main_test.py를 모두 종료하고 백그라운드에서 재시작한다."""
    container = get_container()
    if container is None or container.status != "running":
        raise HTTPException(status_code=409, detail="Container not running")
    container.exec_run(
        ["/bin/bash", "-c",
         "pkill -9 -f 'python3 main_test.py' 2>/dev/null; "
         "sleep 1; "
         "source /opt/ros/humble/setup.bash && "
         "cd /ros2_ws/src/my_ur5e_controller/ && "
         "exec python3 main_test.py > /tmp/bg.log 2>&1"],
        detach=True,
    )
    return {"ok": True, "message": "main_test.py 재시작 완료 (로그: /tmp/bg.log)"}


@app.get("/api/robot_state")
async def robot_state(_: None = Depends(require_auth)):
    """컨테이너 내 /tmp/robot_state.json을 읽어 반환한다."""
    container = get_container()
    if container is None or container.status != "running":
        raise HTTPException(status_code=409, detail="Container not running")
    exit_code, output = container.exec_run(["cat", "/tmp/robot_state.json"], stderr=False)
    if exit_code != 0 or not output:
        raise HTTPException(status_code=503, detail="State not available yet")
    return json.loads(output)


# ── Meshcat 프록시 ────────────────────────────────────────────────────────────

@app.get("/meshcat")
@app.get("/meshcat/")
async def meshcat_index(_: None = Depends(require_auth)):
    port = get_meshcat_port()
    async with httpx.AsyncClient() as client:
        r = await client.get(f"http://localhost:{port}/static/")
    # main.min.js를 절대경로로 재작성 → meshcat_static() 핸들러가 WebSocket URL도 재작성
    html = r.content.replace(
        b'src="main.min.js"',
        b'src="/meshcat/main.min.js"'
    )
    return Response(content=html, media_type="text/html")


@app.get("/meshcat/{path:path}")
async def meshcat_static(path: str, _: None = Depends(require_auth)):
    port = get_meshcat_port()
    async with httpx.AsyncClient() as client:
        r = await client.get(f"http://localhost:{port}/static/{path}")
    content = r.content
    content_type = r.headers.get("content-type", "application/octet-stream")
    if "javascript" in content_type:
        content = content.replace(
            b"`ws://${location.host}`",
            b"`${location.protocol==='https:'?'wss':'ws'}://${location.host}/meshcat-ws`"
        )
    headers = {"Cache-Control": "public, max-age=3600"}
    return Response(content=content, media_type=content_type, headers=headers)


@app.websocket("/meshcat-ws")
async def meshcat_ws_proxy(websocket: WebSocket):
    await websocket.accept()
    port = get_meshcat_port()
    try:
        try:
            ws_conn = await ws_lib.connect(f"ws://localhost:{port}/")
        except Exception:
            # 연결 실패 → 캐시 무효화 후 새 포트로 재시도
            invalidate_meshcat_port_cache()
            port = get_meshcat_port()
            ws_conn = await ws_lib.connect(f"ws://localhost:{port}/")
    except Exception:
        try:
            await websocket.close()
        except Exception:
            pass
        return
    try:
        async with ws_conn as meshcat:
            async def fwd_to_client():
                async for msg in meshcat:
                    if isinstance(msg, bytes):
                        await websocket.send_bytes(msg)
                    else:
                        await websocket.send_text(msg)

            async def fwd_to_meshcat():
                try:
                    while True:
                        data = await websocket.receive()
                        if data.get("type") == "websocket.disconnect":
                            break
                        if data.get("bytes"):
                            await meshcat.send(data["bytes"])
                        elif data.get("text"):
                            await meshcat.send(data["text"])
                except Exception:
                    pass

            t1 = asyncio.create_task(fwd_to_client())
            t2 = asyncio.create_task(fwd_to_meshcat())
            try:
                done, pending = await asyncio.wait(
                    [t1, t2], return_when=asyncio.FIRST_COMPLETED
                )
                for task in pending:
                    task.cancel()
                    try:
                        await task
                    except asyncio.CancelledError:
                        pass
            except Exception:
                t1.cancel()
                t2.cancel()
    except Exception:
        pass
    finally:
        try:
            await websocket.close()
        except Exception:
            pass


# ── Frontend ─────────────────────────────────────────────────────────────────

STATIC = Path(__file__).parent / "static"
app.mount("/static", StaticFiles(directory=STATIC), name="static")


@app.get("/", response_class=HTMLResponse)
def index():
    return (STATIC / "index.html").read_text()


if __name__ == "__main__":
    import uvicorn
    uvicorn.run("main:app", host="0.0.0.0", port=8765, reload=False)
