#!/bin/bash
# Heroi 전체 스택 시작 스크립트 (macOS)
# 사용법: ./start.sh [--no-tunnel | stop]

set -e
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
DASHBOARD_LOG="/tmp/heroi-dashboard.log"
SIM_LOG="/tmp/heroi-sim.log"
PID_FILE="/tmp/heroi-dashboard.pid"
PORT=8765

# 대시보드 프로세스 정리 함수
_kill_dashboard() {
    # PID 파일로 정확한 프로세스 종료
    if [ -f "$PID_FILE" ]; then
        OLD_PID=$(cat "$PID_FILE")
        if kill -0 "$OLD_PID" 2>/dev/null; then
            kill "$OLD_PID" 2>/dev/null
        fi
        rm -f "$PID_FILE"
    fi
    # 혹시 남아 있는 프로세스 정리 (PID 파일 없을 때 대비)
    pkill -f "web_dashboard/main.py" 2>/dev/null || true
    pkill -f "uvicorn main:app" 2>/dev/null || true
    # 포트에 물려 있는 프로세스 즉시 강제 종료
    lsof -ti TCP:"$PORT" -sTCP:LISTEN 2>/dev/null | xargs kill -9 2>/dev/null || true
    # 포트가 해제될 때까지 대기 (최대 3초)
    for i in $(seq 1 6); do
        if ! lsof -i TCP:"$PORT" -sTCP:LISTEN -P -n 2>/dev/null | grep -q LISTEN; then
            return 0
        fi
        sleep 0.5
    done
}

# 시뮬레이션 프로세스 정리 함수 (SIGTERM 무반응 → SIGKILL)
_kill_sim() {
    docker exec ur5e_mujoco_ros2 pkill -f "main_test.py" 2>/dev/null || true
    # SIGTERM 후 최대 3초 대기
    for i in $(seq 1 6); do
        if ! docker exec ur5e_mujoco_ros2 pgrep -f "main_test.py" > /dev/null 2>&1; then
            return 0
        fi
        sleep 0.5
    done
    # 여전히 살아 있으면 SIGKILL
    docker exec ur5e_mujoco_ros2 pkill -9 -f "main_test.py" 2>/dev/null || true
    sleep 0.5
}

# ── stop 명령 ─────────────────────────────────────────────────────────────────
if [ "$1" = "stop" ]; then
    echo "[1/3] 대시보드 종료 중..."
    _kill_dashboard && echo "    ✓ 대시보드 종료" || echo "    - 실행 중인 대시보드 없음"
    echo "[2/3] 시뮬레이션 종료 중..."
    _kill_sim && echo "    ✓ 시뮬레이션 종료" || echo "    - 실행 중인 시뮬레이션 없음"
    echo "[3/3] 컨테이너 종료 중..."
    docker compose -f "$SCRIPT_DIR/docker-compose.yaml" down
    echo "    ✓ 종료 완료"
    exit 0
fi

# ── Docker 데몬 확인 ──────────────────────────────────────────────────────────
echo "[1/5] Docker 상태 확인 중..."
if ! docker info > /dev/null 2>&1; then
    echo "    ✗ Docker Desktop이 실행 중이 아닙니다. Docker Desktop을 먼저 실행해 주세요."
    exit 1
fi
echo "    ✓ Docker 실행 중"

# ── 컨테이너 시작 ─────────────────────────────────────────────────────────────
echo "[2/5] 컨테이너 시작 중..."
docker compose -f "$SCRIPT_DIR/docker-compose.yaml" up -d 2>&1 | sed 's/^/    /'
if ! docker ps --filter name=ur5e_mujoco_ros2 --filter status=running --format '{{.Names}}' | grep -q ur5e_mujoco_ros2; then
    echo "    ✗ 컨테이너 시작 실패"
    exit 1
fi
echo "    ✓ 컨테이너 실행 중"

# ── 컨테이너 내 mink 설치 확인 ────────────────────────────────────────────────
echo "[3/6] 컨테이너 내 mink 확인 및 설치..."
docker exec ur5e_mujoco_ros2 python3 -c "import mink" 2>/dev/null || \
    docker exec ur5e_mujoco_ros2 pip3 install -q "mink[daqp]"
echo "    ✓ mink 준비 완료"

# ── 시뮬레이션 시작 ───────────────────────────────────────────────────────────
echo "[4/6] 시뮬레이션(main_test.py) 시작 중..."
_kill_sim
docker exec -d ur5e_mujoco_ros2 bash -c \
    "source /opt/ros/humble/setup.bash && \
     source /ros2_ws/install/setup.bash && \
     cd /ros2_ws/src/my_ur5e_controller && \
     find . -name '*.pyc' -delete && \
     python3 main_test.py > /tmp/heroi-sim.log 2>&1"
echo "    ✓ 시뮬레이션 백그라운드 시작"
echo "      (로그 확인: docker exec ur5e_mujoco_ros2 tail -f /tmp/heroi-sim.log)"

# ── Python 의존성 설치 ────────────────────────────────────────────────────────
echo "[5/6] Python 패키지 확인 중..."
pip3 install -q -r "$SCRIPT_DIR/web_dashboard/requirements.txt" 2>/dev/null || \
    python3 -m pip install -q -r "$SCRIPT_DIR/web_dashboard/requirements.txt"
echo "    ✓ 패키지 준비 완료"

# ── 대시보드 시작 ─────────────────────────────────────────────────────────────
echo "[6/6] 대시보드 서버 시작 중..."
_kill_dashboard

cd "$SCRIPT_DIR/web_dashboard"
nohup python3 main.py > "$DASHBOARD_LOG" 2>&1 &
DASHBOARD_PID=$!
echo $DASHBOARD_PID > "$PID_FILE"

# 서버 기동 확인 (최대 10초)
for i in $(seq 1 20); do
    if grep -q "Uvicorn running" "$DASHBOARD_LOG" 2>/dev/null; then
        LOCAL_IP=$(ipconfig getifaddr en0 2>/dev/null || ipconfig getifaddr en1 2>/dev/null || hostname)
        echo "    ✓ 서버 실행 완료 (PID: $DASHBOARD_PID)"
        echo ""
        echo "  대시보드:  http://$LOCAL_IP:$PORT"
        echo "  Meshcat:   http://$LOCAL_IP:$PORT/meshcat"
        echo ""
        echo "로그: tail -f $DASHBOARD_LOG"
        echo "      docker exec ur5e_mujoco_ros2 tail -f /tmp/heroi-sim.log"
        echo "종료: ./start.sh stop"
        exit 0
    fi
    # 시작 실패 감지: 에러 메시지 확인
    if grep -q "address already in use\|Error\|error" "$DASHBOARD_LOG" 2>/dev/null; then
        echo "    ✗ 서버 시작 실패:"
        grep -E "address already in use|Error|error" "$DASHBOARD_LOG" | head -3 | sed 's/^/      /'
        echo "    로그 전체: $DASHBOARD_LOG"
        exit 1
    fi
    if ! kill -0 $DASHBOARD_PID 2>/dev/null; then
        echo "    ✗ 서버 프로세스가 종료됨. 로그:"
        tail -5 "$DASHBOARD_LOG" | sed 's/^/      /'
        exit 1
    fi
    sleep 0.5
done

echo "    ✗ 서버 기동 시간 초과. 로그 확인: $DASHBOARD_LOG"
exit 1
