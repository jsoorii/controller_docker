# Heroi Docker Dashboard

`ur5e_mujoco_ros2` 컨테이너를 외부 웹 브라우저에서 시작·중지·재시작하고 실시간 로그를 확인할 수 있는 웹 대시보드입니다.
FastAPI 백엔드가 Docker SDK로 컨테이너를 제어하며, fetch 스트리밍을 통해 로그를 실시간으로 출력합니다.
`python3 main.py` 로 서버를 실행한 뒤 `http://<호스트IP>:8765` 에 접속하면 사용할 수 있습니다.

---

## 파일 구조

```
web_dashboard/
├── main.py          # FastAPI 백엔드 (컨테이너 제어 API + 인증)
├── requirements.txt # Python 패키지 목록
├── .env             # 인증 정보 (git 제외)
├── README.md        # 이 파일
└── static/
    └── index.html   # 웹 대시보드 프론트엔드
```

---

## 기능

- **컨테이너 상태 확인** — 5초마다 자동 갱신
- **시작 / 중지 / 재시작** — 버튼 한 번으로 제어
- **컨테이너 내 명령 실행** — 브라우저에서 직접 명령어 입력
- **실시간 로그 스트리밍** — fetch 스트리밍 방식, 중지/지우기 가능
- **JS 로그인 프롬프트** — 페이지 로드 시 아이디/비밀번호 입력, sessionStorage에 캐시
- **HTTP Basic Auth** — 모든 API 엔드포인트 인증 보호

---

## 실행 방법

```bash
# 1. 패키지 설치
pip install -r requirements.txt

# 2. .env 파일 확인 (없으면 생성)
# AUTH_USERNAME=heroi
# AUTH_PASSWORD=ros2mujoco!

# 3. 서버 실행 (포트 8765)
python3 main.py

# 4. 외부 접속용 터널 (선택)
cloudflared tunnel --url http://localhost:8765
```

---

## 인증 정보

`.env` 파일에서 관리합니다 (git에 포함되지 않음).

```env
AUTH_USERNAME=heroi
AUTH_PASSWORD=ros2mujoco!
```

변경하려면 `.env` 파일을 수정하고 서버를 재시작하세요.

---

## API 엔드포인트

| 메서드 | 경로 | 설명 |
|--------|------|------|
| GET | `/api/status` | 컨테이너 상태 조회 |
| POST | `/api/start` | 컨테이너 시작 (`docker compose up -d --build`) |
| POST | `/api/stop` | 컨테이너 중지 |
| POST | `/api/restart` | 컨테이너 재시작 |
| POST | `/api/exec` | 컨테이너 내 명령 실행 (`body: { "cmd": "..." }`) |
| GET | `/api/logs` | 실시간 로그 스트리밍 (`?lines=200`) |

모든 엔드포인트는 HTTP Basic Auth 필요.

---

## 백엔드 주요 구조 (main.py)

```python
load_dotenv(".env")                        # 인증 정보 환경변수 로드
AUTH_USERNAME = os.getenv("AUTH_USERNAME") # .env에서 읽기
AUTH_PASSWORD = os.getenv("AUTH_PASSWORD")

def require_auth(...)                      # 모든 API에 적용되는 인증 의존성
def get_container()                        # Docker SDK로 컨테이너 조회

GET  /api/status   → container.reload() 후 상태 반환
POST /api/start    → subprocess로 docker compose up 실행
POST /api/stop     → container.stop()
POST /api/restart  → container.restart()
POST /api/exec     → container.exec_run(["/bin/bash", "-c", cmd])
GET  /api/logs     → StreamingResponse(container.logs(stream=True, follow=True))
```

---

## 프론트엔드 주요 구조 (static/index.html)

```javascript
// 인증: 페이지 로드 시 prompt로 입력받아 sessionStorage에 캐시
function getAuth()         // sessionStorage에서 Basic Auth 헤더 반환
async function apiFetch()  // Authorization 헤더 포함한 fetch 래퍼

// 상태: 5초 폴링
fetchStatus()              // GET /api/status

// 제어 버튼
action('start|stop|restart') // POST /api/{cmd}

// 명령 실행
execCmd()                  // POST /api/exec, Enter 키도 지원

// 로그 스트리밍: EventSource 대신 fetch + ReadableStream 사용
//   (EventSource는 커스텀 헤더 불가 → 인증 불가)
startLog()                 // fetch('/api/logs') → reader.read() 루프
stopLog()                  // AbortController.abort()
```

---

## 외부 접속

Cloudflare Tunnel로 외부 노출 가능.
터널 URL은 서버 재시작 시 변경됨.

---

## 변경 이력

| 날짜 | 내용 |
|------|------|
| 2026-04-08 | 인증 정보를 `main.py` 하드코딩에서 `.env` 파일로 분리 (`python-dotenv` 추가) |
| 2026-04-08 | JS fetch 인증 문제 수정 — `apiFetch()` 래퍼로 모든 API 요청에 Authorization 헤더 포함 |
| 2026-04-08 | 로그 스트리밍을 `EventSource` → `fetch + ReadableStream`으로 교체 (헤더 인증 지원) |
