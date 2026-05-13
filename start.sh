#!/bin/bash
# Heroi 전체 스택 시작 스크립트 (macOS)
# 사용법: ./start.sh [--no-tunnel | stop]

set -e
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
DASHBOARD_LOG="/tmp/heroi-dashboard.log"
PORT=8765

# ── stop 명령 ─────────────────────────────────────────────────────────────────
if [ "$1" = "stop" ]; then
    echo "[1/2] 대시보드 종료 중..."
    pkill -f "python3 main.py" 2>/dev/null && echo "    ✓ 대시보드 종료" || echo "    - 실행 중인 대시보드 없음"
    echo "[2/2] 컨테이너 종료 중..."
    docker compose -f "$SCRIPT_DIR/docker-compose.yaml" down
    echo "    ✓ 종료 완료"
    exit 0
fi

# ── Docker 데몬 확인 ──────────────────────────────────────────────────────────
echo "[1/4] Docker 상태 확인 중..."
if ! docker info > /dev/null 2>&1; then
    echo "    ✗ Docker Desktop이 실행 중이 아닙니다. Docker Desktop을 먼저 실행해 주세요."
    exit 1
fi
echo "    ✓ Docker 실행 중"

# ── 컨테이너 시작 ─────────────────────────────────────────────────────────────
echo "[2/4] 컨테이너 시작 중..."
docker compose -f "$SCRIPT_DIR/docker-compose.yaml" up -d 2>&1 | sed 's/^/    /'
if ! docker ps --filter name=ur5e_mujoco_ros2 --filter status=running --format '{{.Names}}' | grep -q ur5e_mujoco_ros2; then
    echo "    ✗ 컨테이너 시작 실패"
    exit 1
fi
echo "    ✓ 컨테이너 실행 중"

# ── Python 의존성 설치 ────────────────────────────────────────────────────────
echo "[3/4] Python 패키지 확인 중..."
pip3 install -q -r "$SCRIPT_DIR/web_dashboard/requirements.txt"
echo "    ✓ 패키지 준비 완료"

# ── 대시보드 시작 ─────────────────────────────────────────────────────────────
echo "[4/4] 대시보드 서버 시작 중..."
pkill -f "python3 main.py" 2>/dev/null || true

# 포트 해제 대기 (최대 5초)
for i in $(seq 1 10); do
    if ! lsof -i TCP:"$PORT" -sTCP:LISTEN -P -n 2>/dev/null | grep -q LISTEN; then
        break
    fi
    sleep 0.5
done

cd "$SCRIPT_DIR/web_dashboard"
nohup python3 main.py > "$DASHBOARD_LOG" 2>&1 &
DASHBOARD_PID=$!

# 서버 기동 확인 (최대 10초)
for i in $(seq 1 20); do
    if grep -q "Uvicorn running" "$DASHBOARD_LOG" 2>/dev/null; then
        LOCAL_IP=$(ipconfig getifaddr en0 2>/dev/null || ipconfig getifaddr en1 2>/dev/null || hostname)
        echo "    ✓ 서버 실행 완료 (PID: $DASHBOARD_PID)"
        echo ""
        echo "  대시보드:  http://$LOCAL_IP:$PORT"
        echo "  Meshcat:   http://$LOCAL_IP:8000"
        echo ""
        echo "로그: tail -f $DASHBOARD_LOG"
        echo "종료: ./start.sh stop"
        exit 0
    fi
    if ! kill -0 $DASHBOARD_PID 2>/dev/null; then
        echo "    ✗ 서버 시작 실패. 로그 확인: $DASHBOARD_LOG"
        exit 1
    fi
    sleep 0.5
done

echo "    ✗ 서버 기동 시간 초과. 로그 확인: $DASHBOARD_LOG"
exit 1
