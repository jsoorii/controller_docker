# Heroi — UR5e MuJoCo Simulation & Web Dashboard

UR5e 로봇팔을 MuJoCo로 시뮬레이션하고, 웹 브라우저에서 원격으로 제어·모니터링하는 풀스택 프로젝트.

---

## 시스템 아키텍처

```
[Mac / 원격 브라우저]
        │  HTTPS (Cloudflare Tunnel)
        ▼
[Web Dashboard  — FastAPI, port 8765]
        │  Docker SDK  │  ROS2 topic pub  │  MuJoCo render
        ▼              ▼                  ▼
[Docker Container: ur5e_mujoco_ros2]
  ├─ MuJoCo Physics Engine (UR5e XML)
  ├─ PD Controller thread  (500 Hz)
  ├─ Meshcat Visualizer    (port 7000)
  └─ ROS2 Humble middleware
```

---

## 디렉토리 구조

```
heroi/
├── Dockerfile                        # ROS2 Humble + MuJoCo 빌드 이미지
├── docker-compose.yaml               # 컨테이너 오케스트레이션 (port 7000)
├── ros_entrypoint.sh                 # ROS2 환경 소싱 스크립트
├── CHANGE.md                         # 변경 이력 (날짜 · 설명 · 롤백 커밋)
├── SUMMARY.md                        # 기술 상세 문서
├── my_ur5e_controller/
│   ├── ur5e_rt_controller.py         # 실시간 PD 컨트롤러 (상태 머신)
│   ├── view_ur5e.py                  # Meshcat 메쉬 시각화 단독 스크립트
│   └── main_test.py                  # 메인 진입점 (컨트롤러 + 시각화)
└── web_dashboard/
    ├── main.py                       # FastAPI 백엔드
    ├── render_snapshot.py            # MuJoCo 오프스크린 렌더러 (이미지 / 3D 프레임)
    ├── launch.sh                     # 서버 + Cloudflare 터널 한번에 실행
    ├── requirements.txt
    ├── .env                          # 인증 정보 (git 제외)
    └── static/index.html             # 웹 대시보드 프론트엔드
```

---

## 기술 스택

| 레이어 | 기술 |
|---|---|
| 물리 시뮬레이션 | MuJoCo (DeepMind) |
| 3D 시각화 | Meshcat (ZMQ + Three.js) |
| 미들웨어 | ROS2 Humble |
| 컨테이너 | Docker + Docker Compose |
| 백엔드 | FastAPI + Docker SDK (Python) |
| 터널 | Cloudflare Tunnel (cloudflared) |
| 인증 | HTTP Basic Auth + HTML 로그인 오버레이 |

---

## 현재 구현된 기능

### 시뮬레이션 (Docker 컨테이너 내)
- **MuJoCo 물리 엔진** — UR5e XML 로드, 중력·충돌·마찰 연산
- **PD 컨트롤러 (500 Hz)** — 5단계 상태 머신, Kp=500 / Kd=50
  ```
  WAIT_STABLE → INIT_CONTROL → CHECK_MOTOR → INIT_POSITION → RUN_CONTROL
  ```
- **Task-space IK** — Damped Least Squares로 EE 위치/자세 추종 (6×6 Jacobian)
- **제어 모드 전환** — Joint-space ↔ Task-space 실시간 전환
- **Meshcat 웹 시각화 (port 7000)** — TriangularMeshGeometry로 실제 UR5e 외형 렌더링
- **멀티스레드** — 500 Hz 제어 스레드 / 50 Hz 시각화 메인 스레드 분리

### ROS2 인터페이스
| 방향 | 토픽 | 타입 | 설명 |
|---|---|---|---|
| Subscribe | `/ur5e/cmd/joint_target` | `Float64MultiArray` | 6개 관절 목표값 (rad) |
| Subscribe | `/ur5e/cmd/ee_target` | `PoseStamped` | EE 목표 위치·자세 (task-space) |
| Subscribe | `/ur5e/cmd/mode` | `Bool` | true=task-space / false=joint-space |
| Publish | `/ur5e/state/joint` | `JointState` | 관절 위치·속도 (50 Hz) |
| Publish | `/ur5e/state/ee_pose` | `PoseStamped` | EE 현재 위치·자세 (50 Hz) |

### 웹 대시보드 (port 8765)
- **컨테이너 제어** — 시작 / 중지 / 재시작 버튼 (5초마다 상태 자동 갱신)
- **실시간 로그 스트리밍** — fetch + ReadableStream 방식, 중지·지우기 가능
- **브라우저 명령 실행** — 컨테이너 내 임의 명령 직접 입력
- **3D 로봇 뷰어** — MuJoCo 오프스크린 렌더, 마우스 드래그 2축 회전
  - 방위각 12단계 × 고도각 5단계 = 60프레임 사전 렌더
  - 가로 드래그 = 방위각, 세로 드래그 = 고도각
- **ROS2 Joint 명령** — `/api/pub/joint` 엔드포인트로 관절 목표값 퍼블리시
- **인증** — 커스텀 HTML 로그인 오버레이 + sessionStorage 캐시, API는 HTTP Basic Auth 보호

### 외부 접속
- **Cloudflare Tunnel** — 고정 IP 없이 외부 HTTPS URL 자동 생성

---

## 빠른 시작

### 1. 시뮬레이션 컨테이너 실행

```bash
# 이미지 빌드 및 컨테이너 시작
docker compose build
docker compose up -d

# 컨테이너 내에서 시뮬레이션 실행
docker exec -it ur5e_mujoco_ros2 python3 \
  /ros2_ws/src/my_ur5e_controller/main_test.py

# 브라우저에서 확인 (동일 머신 또는 SSH 터널)
# http://localhost:7000
```

### 2. 웹 대시보드 실행

```bash
cd web_dashboard

# .env 파일 생성 (최초 1회)
echo "AUTH_USERNAME=heroi" > .env
echo "AUTH_PASSWORD=<비밀번호>" >> .env

# launch.sh 로 서버 + 터널 한번에 실행
./launch.sh
```
cd src/my_ur5e_controller/my_ur5e_controller/ && nohup python3 main_test.py > /tmp/heroi-dashboard.log 2>&1 &

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

터널 없이 로컬만:
```bash
./launch.sh --no-tunnel
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
| POST | `/api/exec` | 컨테이너 내 명령 실행 |
| POST | `/api/exec_bg` | 백그라운드 명령 실행 |
| POST | `/api/exec_stream` | 명령 실행 + 스트리밍 출력 |
| GET | `/api/logs` | 실시간 로그 스트리밍 (`?lines=200`) |
| POST | `/api/pub/joint` | ROS2 joint 명령 퍼블리시 |
| GET | `/api/robot_image` | MuJoCo 로봇 이미지 렌더링 |
| GET | `/api/robot_3d` | MuJoCo 3D 터닝테이블 프레임 (60장) |
| GET | `/meshcat` | Meshcat 3D 뷰어 프록시 |

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

# 외부 터널 URL 확인
grep -o "https://[^ ]*trycloudflare.com" /tmp/heroi-tunnel.log
```

---

## TODO(긴급!)
1. docker 내에서 main_test.py가 여러개 실행되고 있을 경우 확인하는 프로세스 필요.
2. 웹 ui에서 main_test.py 모두 종료하고, 백그라운드에서 실행시키는 명령어버튼(cd src/my_ur5e_controller/my_ur5e_controller/ && nohup python3 main_test.py > /tmp/heroi-dashboard.log 2>&1 &) 필요.
3. 현재 도커 내부에서 src/my_ur5e...폴더안에 web관련 코드들이 있는데, 계층 정리 필요.
4. 웹과 컨트롤러 README가 따로 작성되어 있음, 하나로 통합 필요함.

#TODO
3. **Robot State Publisher / TF 트리** — URDF 연동, `robot_state_publisher`로 TF 프레임 구성
4. **JointTrajectory 인터페이스** — 현재 `Float64MultiArray` → `trajectory_msgs/JointTrajectory`로 전환, 궤적 보간 지원
5. **대시보드 실시간 조인트 슬라이더** — 웹 UI에서 직접 각 조인트 목표값 입력
6. **경로 계획 연동** — MoveIt2와 연결하여 충돌 회피 경로 계획