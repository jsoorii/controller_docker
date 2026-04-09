"""
Docker 웹 대시보드 - ur5e_mujoco_ros2 컨테이너 제어
"""
import asyncio
import json
import os
import re
import secrets
import subprocess
from contextlib import asynccontextmanager
from pathlib import Path
from typing import AsyncGenerator

import httpx
import websockets as ws_lib

import docker
from dotenv import load_dotenv
from fastapi import Depends, FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.middleware.gzip import GZipMiddleware
from fastapi.responses import HTMLResponse, Response, StreamingResponse
from fastapi.security import HTTPBasic, HTTPBasicCredentials
from fastapi.staticfiles import StaticFiles
from sse_starlette.sse import EventSourceResponse

load_dotenv(Path(__file__).parent / ".env")

COMPOSE_DIR = Path(__file__).parent.parent
CONTAINER_NAME = "ur5e_mujoco_ros2"

# ── 인증 설정 (.env 파일에서 로드) ────────────────────────────────────────────
AUTH_USERNAME = os.getenv("AUTH_USERNAME", "heroi")
AUTH_PASSWORD = os.getenv("AUTH_PASSWORD", "")

# ── Meshcat 터널 URL ──────────────────────────────────────────────────────────
meshcat_url: str = ""
_meshcat_proc: asyncio.subprocess.Process | None = None


async def _drain(proc: asyncio.subprocess.Process) -> None:
    """stdout을 계속 소비해 버퍼 블로킹을 방지한다."""
    async for _ in proc.stdout:
        pass


async def _start_meshcat_tunnel() -> None:
    global meshcat_url, _meshcat_proc
    # 이전 인스턴스가 남긴 좀비 프로세스 제거
    pkill = await asyncio.create_subprocess_exec(
        "pkill", "-f", "cloudflared tunnel --url http://localhost:7000",
        stdout=asyncio.subprocess.DEVNULL, stderr=asyncio.subprocess.DEVNULL,
    )
    await pkill.wait()
    await asyncio.sleep(0.3)

    _meshcat_proc = await asyncio.create_subprocess_exec(
        "cloudflared", "tunnel", "--url", "http://localhost:7000",
        stdout=asyncio.subprocess.PIPE,
        stderr=asyncio.subprocess.STDOUT,
    )
    async for line in _meshcat_proc.stdout:
        text = line.decode(errors="replace")
        m = re.search(r'https://[a-z0-9\-]+\.trycloudflare\.com', text)
        if m:
            meshcat_url = m.group(0)
            print(f"[Meshcat Tunnel] {meshcat_url}", flush=True)
            asyncio.create_task(_drain(_meshcat_proc))
            break


@asynccontextmanager
async def lifespan(app: FastAPI):
    asyncio.create_task(_start_meshcat_tunnel())
    yield
    if _meshcat_proc:
        _meshcat_proc.terminate()
        await _meshcat_proc.wait()


app = FastAPI(title="Heroi Docker Dashboard", lifespan=lifespan)
app.add_middleware(GZipMiddleware, minimum_size=1000)
security = HTTPBasic()
client = docker.from_env()


def require_auth(credentials: HTTPBasicCredentials = Depends(security)):
    ok_user = secrets.compare_digest(credentials.username, AUTH_USERNAME)
    ok_pass = secrets.compare_digest(credentials.password, AUTH_PASSWORD)
    if not (ok_user and ok_pass):
        raise HTTPException(
            status_code=401,
            detail="Unauthorized",
            headers={"WWW-Authenticate": "Basic"},
        )


def get_container():
    try:
        return client.containers.get(CONTAINER_NAME)
    except docker.errors.NotFound:
        return None


# ── API ──────────────────────────────────────────────────────────────────────

@app.get("/api/meshcat_url")
def get_meshcat_url(_: None = Depends(require_auth)):
    return {"url": meshcat_url}


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


async def _log_generator(lines: int) -> AsyncGenerator[str, None]:
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
            chunk = await proc.stdout.read(4096)
            if not chunk:
                break
            yield chunk.decode(errors="replace")
    finally:
        proc.terminate()
        await proc.wait()


@app.get("/api/logs")
async def logs(lines: int = 200, _: None = Depends(require_auth)):
    return StreamingResponse(
        _log_generator(lines),
        media_type="text/plain",
    )


# ── ROS2 Publish ─────────────────────────────────────────────────────────────

@app.post("/api/pub/joint")
async def pub_joint(body: dict, _: None = Depends(require_auth)):
    joints = body.get("joints", [])
    if len(joints) != 6:
        raise HTTPException(status_code=400, detail="joints must have 6 values")
    data_str = "[" + ", ".join(f"{v:.6f}" for v in joints) + "]"
    cmd = (
        f"source /opt/ros/humble/setup.bash && "
        f"ros2 topic pub --once /ur5e/cmd/joint_target "
        f"std_msgs/msg/Float64MultiArray '{{data: {data_str}}}'"
    )
    container = get_container()
    if container is None or container.status != "running":
        raise HTTPException(status_code=409, detail="Container not running")
    exit_code, output = container.exec_run(
        ["/bin/bash", "-c", cmd], stderr=True
    )
    return {"ok": exit_code == 0, "output": output.decode(errors="replace")}


@app.get("/api/robot_image")
async def robot_image(_: None = Depends(require_auth)):
    container = get_container()
    if container is None or container.status != "running":
        raise HTTPException(status_code=409, detail="Container not running")
    script = "/ros2_ws/src/my_ur5e_controller/web_dashboard/render_snapshot.py"
    loop = asyncio.get_event_loop()
    exit_code, output = await loop.run_in_executor(
        None,
        lambda: container.exec_run(["python3", script], stderr=False)
    )
    if exit_code != 0:
        raise HTTPException(status_code=500, detail="Rendering failed")
    import base64 as _b64
    img_bytes = _b64.b64decode(output.strip())
    return Response(content=img_bytes, media_type="image/png")


@app.get("/api/robot_3d")
async def robot_3d(_: None = Depends(require_auth)):
    container = get_container()
    if container is None or container.status != "running":
        raise HTTPException(status_code=409, detail="Container not running")
    script = "/ros2_ws/src/my_ur5e_controller/web_dashboard/render_snapshot.py"
    loop = asyncio.get_event_loop()
    exit_code, output = await loop.run_in_executor(
        None,
        lambda: container.exec_run(["python3", script, "--frames"], stderr=False)
    )
    if exit_code != 0:
        raise HTTPException(status_code=500, detail="Rendering failed")
    return json.loads(output.strip())


# ── Meshcat 프록시 ────────────────────────────────────────────────────────────

@app.get("/meshcat")
@app.get("/meshcat/")
async def meshcat_index(_: None = Depends(require_auth)):
    async with httpx.AsyncClient() as client:
        r = await client.get("http://localhost:7000/static/")
    # WebSocket URL을 /meshcat-ws로 재작성
    html = r.content.replace(
        b"ws://${location.host}",
        b"(location.protocol==='https:'?'wss':'ws')+'://'+location.host+'/meshcat-ws'"
    )
    return Response(content=html, media_type="text/html")


@app.get("/meshcat/{path:path}")
async def meshcat_static(path: str, _: None = Depends(require_auth)):
    async with httpx.AsyncClient() as client:
        r = await client.get(f"http://localhost:7000/static/{path}")
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
    try:
        async with ws_lib.connect("ws://localhost:7000/") as meshcat:
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

            await asyncio.gather(fwd_to_client(), fwd_to_meshcat())
    except Exception:
        pass


# ── Frontend ─────────────────────────────────────────────────────────────────

STATIC = Path(__file__).parent / "static"
app.mount("/static", StaticFiles(directory=STATIC), name="static")


@app.get("/", response_class=HTMLResponse)
def index(_: None = Depends(require_auth)):
    return (STATIC / "index.html").read_text()


if __name__ == "__main__":
    import uvicorn
    uvicorn.run("main:app", host="0.0.0.0", port=8765, reload=False)
