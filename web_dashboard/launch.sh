#!/bin/bash
# Heroi Dashboard 런치 스크립트
# 사용법: ./launch.sh [--no-tunnel | stop]

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
DASHBOARD_LOG="/tmp/heroi-dashboard.log"
TUNNEL_LOG="/tmp/heroi-tunnel.log"
MESHCAT_TUNNEL_LOG="/tmp/heroi-meshcat-tunnel.log"
PORT=8765

# macOS / Linux 공통 유틸 함수
_local_ip() {
    if [ "$(uname)" = "Darwin" ]; then
        ipconfig getifaddr en0 2>/dev/null \
            || ipconfig getifaddr en1 2>/dev/null \
            || hostname
    else
        hostname -I | awk '{print $1}'
    fi
}
_port_in_use() {
    if [ "$(uname)" = "Darwin" ]; then
        lsof -i TCP:"$1" -sTCP:LISTEN -P -n 2>/dev/null | grep -q LISTEN
    else
        ss -tlnp 2>/dev/null | grep -q ":$1 "
    fi
}

# ── stop 명령 ─────────────────────────────────────────────────────────────────
_stop_all() {
    pkill -f "python3 main.py" 2>/dev/null
    sudo tailscale funnel reset 2>/dev/null
}

if [ "$1" = "stop" ]; then
    echo "모든 Heroi 프로세스 종료 중..."
    _stop_all
    echo "    ✓ 종료 완료 (시뮬레이션은 컨테이너 내에서 별도 종료 필요)"
    exit 0
fi

# ── 기존 프로세스 정리 ────────────────────────────────────────────────────────
echo "[1/4] 기존 프로세스 정리 중..."
_stop_all

# 포트가 완전히 해제될 때까지 대기 (최대 10초)
for i in $(seq 1 20); do
    if ! _port_in_use "$PORT"; then
        break
    fi
    sleep 0.5
done

# ── 대시보드 서버 시작 ────────────────────────────────────────────────────────
echo "[2/4] 대시보드 서버 시작 중..."
cd "$SCRIPT_DIR"
nohup python3 main.py > "$DASHBOARD_LOG" 2>&1 &
DASHBOARD_PID=$!

# 서버 정상 기동 확인 (최대 5초)
for i in $(seq 1 10); do
    if grep -q "Uvicorn running" "$DASHBOARD_LOG" 2>/dev/null; then
        LOCAL_IP=$(_local_ip)
        echo "    ✓ 서버 실행 완료 (PID: $DASHBOARD_PID)"
        echo "    로컬 주소:         http://$LOCAL_IP:$PORT"
        echo "    Meshcat 로컬 주소: http://$LOCAL_IP:8000"
        break
    fi
    if ! kill -0 $DASHBOARD_PID 2>/dev/null; then
        echo "    ✗ 서버 시작 실패. 로그 확인: $DASHBOARD_LOG"
        exit 1
    fi
    sleep 0.5
done

# ── Tailscale Funnel 시작 ─────────────────────────────────────────────────────
if [ "$1" = "--no-tunnel" ]; then
    echo "[3/4] 터널 생략 (--no-tunnel)"
else
    echo "[3/4] Tailscale Funnel 시작 중..."
    sudo tailscale funnel reset 2>/dev/null
    sudo tailscale funnel --bg "$PORT" 2>/dev/null

    # URL 추출 (machine-name.tail-xxxx.ts.net)
    URL=$(tailscale funnel status 2>/dev/null | grep -o "https://[^ ]*" | head -1)
    if [ -z "$URL" ]; then
        # fallback: tailscale status에서 DNSName 파싱
        DNS=$(tailscale status --json 2>/dev/null | python3 -c \
            "import json,sys; d=json.load(sys.stdin); print(d.get('Self',{}).get('DNSName','').rstrip('.'))" 2>/dev/null)
        [ -n "$DNS" ] && URL="https://$DNS"
    fi

    if [ -n "$URL" ]; then
        echo "    ✓ 터널 연결 완료"
        echo "    외부 주소 (대시보드): $URL"
    else
        echo "    ✗ URL 확인 실패. 'tailscale funnel status' 로 직접 확인하세요."
    fi

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
