# Heroi Docker Dashboard

`ur5e_mujoco_ros2` 컨테이너를 외부 웹 브라우저에서 제어하는 웹 대시보드.
FastAPI 백엔드 + Docker SDK로 컨테이너를 관리하며, Cloudflare Tunnel로 외부 접속을 지원합니다.

---

## 파일 구조

```
web_dashboard/
├── main.py           # FastAPI 백엔드 (컨테이너 제어 API + 인증)
├── launch.sh         # 서버 + 터널 한번에 실행하는 런치 스크립트
├── requirements.txt  # Python 패키지 목록
├── .env              # 인증 정보 (git 제외)
├── README.md         # 이 파일
└── static/
    └── index.html    # 웹 대시보드 프론트엔드
```

---

## 기능

- **컨테이너 상태 확인** — 5초마다 자동 갱신
- **시작 / 중지 / 재시작** — 버튼 한 번으로 제어
- **컨테이너 내 명령 실행** — 브라우저에서 직접 명령어 입력
- **실시간 로그 스트리밍** — fetch 스트리밍 방식, 중지/지우기 가능
- **3D 로봇 뷰어** — MuJoCo 렌더링, 마우스 드래그로 2축 회전
- **커스텀 로그인** — 페이지 로드 시 오버레이 로그인, sessionStorage 캐시
- **HTTP Basic Auth** — 모든 API 엔드포인트 서버 측 인증 보호

---

## 빠른 시작 (권장)
pkill -f main_test.py
`launch.sh` 하나로 서버와 Cloudflare 터널을 한번에 실행합니다.

```bash
cd /media/jsoori/claw_ws/heroi/web_dashboard
./launch.sh
```

실행 시 출력 예시:
```
[1/3] 기존 프로세스 정리 중...
[2/3] 대시보드 서버 시작 중...
    ✓ 서버 실행 완료 (PID: 12345)
    로컬 주소: http://192.168.x.x:8765
[3/3] Cloudflare 터널 시작 중...
    ✓ 터널 연결 완료 (PID: 12346)
    외부 주소: https://xxxx-xxxx.trycloudflare.com
```

터널 없이 로컬만 실행:
```bash
./launch.sh --no-tunnel

한번에 종료
./launch.sh stop
```

---

## 수동 실행

### 1. 패키지 설치

```bash
pip install -r requirements.txt
```

### 2. 인증 정보 설정

`.env` 파일을 생성합니다 (git에 포함되지 않음):

```env
AUTH_USERNAME=heroi
AUTH_PASSWORD=ros2mujoco!
```

### 3. 서버 실행

포그라운드:
```bash
python3 main.py
```

백그라운드:
```bash
nohup python3 main.py > /tmp/heroi-dashboard.log 2>&1 &
```

### 4. Cloudflare 터널 (외부 접속)

```bash
nohup cloudflared tunnel --url http://localhost:8765 > /tmp/heroi-tunnel.log 2>&1 &

# 터널 URL 확인
grep -o "https://[^ ]*trycloudflare.com" /tmp/heroi-tunnel.log
```

---

## 접속 주소 확인

```bash
# 로컬 주소
hostname -I | awk '{print "http://" $1 ":8765"}'

# 외부 주소 (터널)
grep -o "https://[^ ]*trycloudflare.com" /tmp/heroi-tunnel.log
```

---

## 관리 명령어

```bash
# 로그 실시간 확인
tail -f /tmp/heroi-dashboard.log
tail -f /tmp/heroi-tunnel.log

# 프로세스 확인
pgrep -a -f "python3 main.py"
pgrep -a -f "cloudflared tunnel"

# 종료
pkill -f "python3 main.py"
pkill -f "cloudflared tunnel --url http://localhost:8765"
```

---

## API 엔드포인트

모든 엔드포인트는 HTTP Basic Auth 필요 (`GET /` 제외).

| 메서드 | 경로 | 설명 |
|--------|------|------|
| GET | `/api/status` | 컨테이너 상태 조회 |
| POST | `/api/start` | 컨테이너 시작 (`docker compose up -d --build`) |
| POST | `/api/stop` | 컨테이너 중지 |
| POST | `/api/restart` | 컨테이너 재시작 |
| POST | `/api/exec` | 컨테이너 내 명령 실행 (`body: { "cmd": "..." }`) |
| POST | `/api/exec_bg` | 백그라운드 명령 실행 |
| POST | `/api/exec_stream` | 명령 실행 + 스트리밍 출력 |
| GET | `/api/logs` | 실시간 로그 스트리밍 (`?lines=200`) |
| POST | `/api/pub/joint` | ROS2 joint 명령 퍼블리시 |
| GET | `/api/robot_image` | MuJoCo 로봇 이미지 렌더링 |
| GET | `/api/robot_3d` | MuJoCo 3D 터닝테이블 프레임 |
| GET | `/meshcat` | Meshcat 3D 뷰어 프록시 |
