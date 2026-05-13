# Heroi — UR5e MuJoCo Simulation & Web Dashboard (macOS)

UR5e 로봇팔을 MuJoCo로 시뮬레이션하고, 웹 브라우저에서 원격으로 제어·모니터링하는 풀스택 프로젝트.

> **이 브랜치(`macos`)는 macOS 네이티브 실행용입니다.**
> Linux 버전은 `main` 브랜치를 사용하세요.

---

## 시스템 아키텍처

```
[브라우저]
     │  HTTP (localhost 또는 Tailscale Funnel)
     ▼
[Web Dashboard  — FastAPI, port 8765]  ← 호스트(Mac)에서 실행
     │  Docker SDK  │  ROS2 topic pub
     ▼              ▼
[Docker Container: ur5e_mujoco_ros2]
  ├─ MuJoCo Physics Engine
  │   ├─ UR5e + 장애물 씬 (scene.xml)
  │   └─ 센서: 관절 토크 / 접촉 정보
  ├─ PD Controller thread  (500 Hz)
  ├─ Meshcat Visualizer    (container: 7000 → host: 8000)
  └─ ROS2 Humble middleware
```

---

## 디렉토리 구조

```
heroi/
├── Dockerfile                        # ROS2 Humble + MuJoCo 빌드 이미지
├── docker-compose.yaml               # 컨테이너 오케스트레이션 (포트 매핑)
├── ros_entrypoint.sh                 # ROS2 환경 소싱 스크립트
├── start.sh                          # 전체 스택 한번에 시작/종료 (macOS용)
├── CHANGE.md                         # 변경 이력
├── docs/
│   └── controller_theory.md          # 제어 루프 이론 문서
├── my_ur5e_controller/               # 컨테이너 내 /ros2_ws/src/my_ur5e_controller/
│   ├── ur5e_rt_controller.py         # 실시간 PD 컨트롤러 (상태 머신)
│   ├── main_test.py                  # 메인 진입점 (씬 로드 + 컨트롤러 + 시각화)
│   └── ur5e_minimal.urdf             # TF 트리용 경량 URDF
└── web_dashboard/                    # 호스트(Mac)에서 실행
    ├── main.py                       # FastAPI 백엔드
    ├── launch.sh                     # 대시보드 서버 + Tailscale Funnel 실행
    ├── requirements.txt
    └── static/index.html             # 웹 대시보드 프론트엔드
```

---

## 기술 스택

| 레이어 | 기술 |
|---|---|
| 물리 시뮬레이션 | MuJoCo (DeepMind) |
| 3D 시각화 | Meshcat (ZMQ + Three.js) |
| 미들웨어 | ROS2 Humble |
| 컨테이너 | Docker Desktop (macOS) |
| 백엔드 | FastAPI + Docker SDK (Python) |
| 원격 접속 | Tailscale Funnel (선택) |
| 인증 | 없음 (로컬 전용) |

---

## 포트 구성

| 서비스 | 호스트 포트 | 컨테이너 포트 |
|--------|------------|--------------|
| 웹 대시보드 | 8765 | — (호스트에서 직접 실행) |
| Meshcat 3D 뷰어 | 8000–8010 | 7000–7010 |
| Gripper viewer (MJPEG) | 8100 | 7100 |
| Combined viewer (MJPEG) | 8101 | 7101 |

> macOS의 AirPlay Receiver가 포트 7000을 점유하므로 호스트 포트를 8000번대로 매핑합니다.

---

## 환경 요구사항

| 항목 | 요구사항 |
|------|---------|
| OS | macOS 12 (Monterey) 이상 |
| Docker Desktop | macOS 12: v4.28.0 / macOS 14+: 최신 버전 |
| Python | 3.10 이상 |
| 아키텍처 | Intel (x86_64) |

**Docker Desktop 설치 (macOS 12 Monterey)**
```bash
# Homebrew 사용
brew install --cask docker  # macOS 14+

# macOS 12 전용 (AirPlay가 7000번 사용 중인 경우 포트 이미 조정됨)
curl -Lo /tmp/Docker.dmg "https://desktop.docker.com/mac/main/amd64/139021/Docker.dmg"
open /tmp/Docker.dmg
```

---

## 빠른 시작

### 1. 전체 스택 한번에 실행 (권장)

```bash
cd controller_docker
./start.sh
```

실행 시 출력:
```
[1/4] Docker 상태 확인 중...
    ✓ Docker 실행 중
[2/4] 컨테이너 시작 중...
    ✓ 컨테이너 실행 중
[3/4] Python 패키지 확인 중...
    ✓ 패키지 준비 완료
[4/4] 대시보드 서버 시작 중...
    ✓ 서버 실행 완료 (PID: 12345)

  대시보드:  http://192.168.x.x:8765
  Meshcat:   http://192.168.x.x:8000
```

### 2. 종료

```bash
./start.sh stop
```

### 3. 수동 실행

```bash
# 컨테이너 빌드 & 시작
docker compose up -d --build

# 대시보드 서버 (별도 터미널)
cd web_dashboard
pip3 install -r requirements.txt
./launch.sh --no-tunnel   # 로컬 전용
./launch.sh               # Tailscale Funnel 포함 (원격 접속)
```

---

## 현재 구현된 기능

### 시뮬레이션 (Docker 컨테이너 내)

- **MuJoCo 씬 구성** — `load_scene_model()`이 mujoco_menagerie scene.xml을 파싱해 장애물을 동적으로 추가
- **PD 컨트롤러 (500 Hz)** — 5단계 상태 머신, Kp=500 / Kd=50
  ```
  WAIT_STABLE → INIT_CONTROL → CHECK_MOTOR → INIT_POSITION → RUN_CONTROL
  ```
- **Task-space IK** — Damped Least Squares로 EE 위치/자세 추종 (6×6 Jacobian)
- **제어 모드 전환** — Joint-space ↔ Task-space 실시간 전환
- **Meshcat 웹 시각화 (host: 8000)** — TriangularMeshGeometry로 실제 UR5e 외형 렌더링
- **멀티스레드** — 500 Hz 제어 스레드 / 시각화 메인 스레드 분리

### 안전 기능

- **충돌 감지** — 매 스텝 비인접 링크 접촉 검사 → `COLLISION_STOP` 상태 전환, 관절 고정
- **충돌 정지 해제** — `/ur5e/cmd/collision_reset` ROS2 토픽 or 대시보드 버튼
- **소프트 관절 한계** — 관절 범위 끝 N% 이내 진입 시 반발 토크 + 속도 감쇠

### 모션 프로파일 (EE Task-space)

모든 EE 명령은 Quintic polynomial 프로파일로 실행됩니다.

```
s(τ) = 10τ³ - 15τ⁴ + 6τ⁵   (τ = t / duration)
```

### ROS2 인터페이스

| 방향 | 토픽 | 타입 | 설명 |
|---|---|---|---|
| Subscribe | `/ur5e/cmd/joint_trajectory` | `JointTrajectory` | 관절 궤적 명령 |
| Subscribe | `/ur5e/cmd/ee_target` | `PoseStamped` | EE 목표 위치·자세 |
| Subscribe | `/ur5e/cmd/mode` | `Bool` | true=task-space / false=joint-space |
| Subscribe | `/ur5e/cmd/soft_limits` | `String` | 소프트 한계 파라미터 JSON |
| Subscribe | `/ur5e/cmd/collision_reset` | `Bool` | 충돌 정지 해제 |
| Publish | `/ur5e/state/joint` | `JointState` | 관절 위치·속도 (10 Hz) |
| Publish | `/ur5e/state/ee_pose` | `PoseStamped` | EE 현재 위치·자세 (10 Hz) |

### 웹 대시보드 (port 8765)

- **컨테이너 제어** — 시작 / 중지 / 재시작 / 재빌드
- **시뮬레이션 프로세스 모니터** — main_test.py 실행 개수 확인, 종료·재시작
- **Joint-space 제어 탭** — 6관절 목표값 + 소요 시간, 현재값 불러오기
- **Task-space 제어 탭** — EE 위치(xyz) + 오일러 각, 쿼터니언 실시간 프리뷰
- **소프트 관절 한계 설정** — 활성화 토글, margin 슬라이더, kp/kd 입력
- **관절 상태 테이블** — 현재값(deg) + 범위 내 위치 바 시각화
- **충돌 감지 배너** — 충돌 발생 시 상단 고정 빨간 배너
- **Meshcat 뷰어 프록시** — `/meshcat` 경로로 3D 시각화 접근
- **스트리밍 명령** — 컨테이너 내 임의 명령 실시간 출력
- **실시간 로그** — `/tmp/bg.log` 스트리밍

---

## API 엔드포인트

인증 없이 모든 엔드포인트에 접근 가능합니다.

| 메서드 | 경로 | 설명 |
|--------|------|------|
| GET | `/api/status` | 컨테이너 상태 조회 |
| POST | `/api/start` | 컨테이너 시작 |
| POST | `/api/stop` | 컨테이너 중지 |
| POST | `/api/restart` | 컨테이너 재시작 |
| POST | `/api/rebuild` | 재빌드 후 재시작 (스트리밍 출력) |
| POST | `/api/exec` | 컨테이너 내 명령 실행 |
| POST | `/api/exec_bg` | 백그라운드 명령 실행 |
| POST | `/api/exec_stream` | 명령 실행 + 스트리밍 출력 |
| GET | `/api/logs` | 실시간 로그 스트리밍 (`?lines=200`) |
| GET | `/api/robot_state` | 로봇 현재 상태 JSON |
| POST | `/api/pub/joint` | ROS2 joint 궤적 명령 발행 |
| POST | `/api/pub/ee` | ROS2 EE 명령 발행 |
| POST | `/api/pub/mode` | ROS2 제어 모드 전환 |
| POST | `/api/pub/soft_limits` | 소프트 관절 한계 파라미터 전송 |
| GET | `/api/sim/status` | main_test.py 프로세스 목록 |
| POST | `/api/sim/kill` | main_test.py 전체 종료 |
| POST | `/api/sim/restart` | main_test.py 종료 후 재시작 |
| POST | `/api/sim/collision_reset` | 충돌 정지 해제 |
| GET | `/meshcat` | Meshcat 3D 뷰어 프록시 |

---

## 관리 명령어

```bash
# 로그 실시간 확인
tail -f /tmp/heroi-dashboard.log

# 프로세스 확인
pgrep -a -f "web_dashboard/main.py"

# 컨테이너 상태
docker ps --filter name=ur5e_mujoco_ros2

# 종료
./start.sh stop
```

---

## main 브랜치와의 차이점

| 항목 | main (Linux) | macos |
|------|-------------|-------|
| 네트워크 | `network_mode: host` | 명시적 포트 매핑 |
| Meshcat 호스트 포트 | 7000 | 8000 (AirPlay 충돌 회피) |
| 인증 | HTTP Basic Auth | 없음 |
| 실행 스크립트 | `launch.sh` | `start.sh` (컨테이너+대시보드 통합) |
| 터널 | Cloudflare Tunnel | Tailscale Funnel |
| X11 | `DISPLAY=$DISPLAY` | 불필요 (osmesa headless) |
