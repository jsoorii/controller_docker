#!/bin/bash
# Heroi Dashboard 런치 스크립트
# 사용법: ./launch.sh [--no-tunnel | stop]

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
DASHBOARD_LOG="/tmp/heroi-dashboard.log"
TUNNEL_LOG="/tmp/heroi-tunnel.log"
MESHCAT_TUNNEL_LOG="/tmp/heroi-meshcat-tunnel.log"
PORT=8765

# ── stop 명령 ─────────────────────────────────────────────────────────────────
if [ "$1" = "stop" ]; then
    echo "모든 Heroi 프로세스 종료 중..."
    pkill -f "web_dashboard/main.py" 2>/dev/null
    pkill -f "cloudflared tunnel --url http://localhost:$PORT" 2>/dev/null
    echo "    ✓ 종료 완료 (시뮬레이션은 컨테이너 내에서 별도 종료 필요)"
    exit 0
fi

# ── 기존 프로세스 정리 ────────────────────────────────────────────────────────
echo "[1/4] 기존 프로세스 정리 중..."
pkill -f "web_dashboard/main.py" 2>/dev/null
pkill -f "cloudflared tunnel --url http://localhost:$PORT" 2>/dev/null
sleep 0.5

# ── 대시보드 서버 시작 ────────────────────────────────────────────────────────
echo "[2/4] 대시보드 서버 시작 중..."
cd "$SCRIPT_DIR"
nohup python3 main.py > "$DASHBOARD_LOG" 2>&1 &
DASHBOARD_PID=$!

# 서버 정상 기동 확인 (최대 5초)
for i in $(seq 1 10); do
    if grep -q "Uvicorn running" "$DASHBOARD_LOG" 2>/dev/null; then
        LOCAL_IP=$(hostname -I | awk '{print $1}')
        echo "    ✓ 서버 실행 완료 (PID: $DASHBOARD_PID)"
        echo "    로컬 주소:         http://$LOCAL_IP:$PORT"
        echo "    Meshcat 로컬 주소: http://$LOCAL_IP:7000"
        break
    fi
    if ! kill -0 $DASHBOARD_PID 2>/dev/null; then
        echo "    ✗ 서버 시작 실패. 로그 확인: $DASHBOARD_LOG"
        exit 1
    fi
    sleep 0.5
done

# ── Cloudflare 터널 시작 ──────────────────────────────────────────────────────
if [ "$1" = "--no-tunnel" ]; then
    echo "[3/4] 터널 생략 (--no-tunnel)"
else
    echo "[3/4] Cloudflare 터널 시작 중..."
    nohup cloudflared tunnel --url http://localhost:$PORT > "$TUNNEL_LOG" 2>&1 &
    TUNNEL_PID=$!

    # 대시보드 터널 URL 대기 (최대 15초)
    for i in $(seq 1 30); do
        URL=$(grep -o "https://[a-z0-9-]*\.trycloudflare\.com" "$TUNNEL_LOG" 2>/dev/null | head -1)
        if [ -n "$URL" ]; then
            echo "    ✓ 터널 연결 완료 (PID: $TUNNEL_PID)"
            echo "    외부 주소 (대시보드): $URL"
            break
        fi
        if ! kill -0 $TUNNEL_PID 2>/dev/null; then
            echo "    ✗ 터널 시작 실패. 로그 확인: $TUNNEL_LOG"
            break
        fi
        sleep 0.5
    done

    # Meshcat은 대시보드 프록시(/meshcat)로 접속 — 별도 터널 불필요
    echo "[4/4] Meshcat 접속 주소:"
    echo "    외부 주소 (Meshcat):  $URL/meshcat"
fi

echo ""
echo "로그 확인:"
echo "  대시보드:      tail -f $DASHBOARD_LOG"
echo "  대시보드 터널: tail -f $TUNNEL_LOG"
echo ""
echo "종료:"
echo "  ./launch.sh stop"
