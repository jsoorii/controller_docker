# Heroi — UR5e MuJoCo Simulation & Web Dashboard

UR5e 로봇팔을 MuJoCo로 시뮬레이션하고, 웹 브라우저에서 원격으로 제어·모니터링하는 풀스택 프로젝트.

---

## 시스템 아키텍처

```
[Mac / 원격 브라우저]
        │  HTTPS (Cloudflare Tunnel)
        ▼
[Web Dashboard  — FastAPI, port 8765]
        │  Docker SDK  │  ROS2 topic pub
        ▼              ▼
[Docker Container: ur5e_mujoco_ros2]
  ├─ MuJoCo Physics Engine
  │   ├─ UR5e + 장애물 씬 (scene.xml)
  │   └─ 센서: 관절 토크 / 접촉 정보
  ├─ PD Controller thread  (500 Hz)
  ├─ Meshcat Visualizer    (port 7000)
  └─ ROS2 Humble middleware
```

---

## 디렉토리 구조

```
heroi/
├── Dockerfile                        # ROS2 Humble + MuJoCo 빌드 이미지
├── docker-compose.yaml               # 컨테이너 오케스트레이션
├── ros_entrypoint.sh                 # ROS2 환경 소싱 스크립트
├── CHANGE.md                         # 변경 이력 (날짜 · 설명 · 롤백 커밋)
├── docs/
│   └── controller_theory.md          # 제어 루프 이론 문서
├── my_ur5e_controller/               # 컨테이너 내 /ros2_ws/src/my_ur5e_controller/
│   ├── ur5e_rt_controller.py         # 실시간 PD 컨트롤러 (상태 머신)
│   ├── main_test.py                  # 메인 진입점 (씬 로드 + 컨트롤러 + 시각화)
│   ├── view_ur5e.py                  # Meshcat 메쉬 시각화 단독 스크립트
│   └── ur5e_minimal.urdf             # TF 트리용 경량 URDF
└── web_dashboard/                    # 컨테이너 내 /ros2_ws/src/web_dashboard/
    ├── main.py                       # FastAPI 백엔드
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

- **MuJoCo 씬 구성** — `load_scene_model()`이 mujoco_menagerie scene.xml을 파싱해 장애물을 동적으로 추가
  - 정육면체 30cm × 30cm × 30cm (로봇 베이스 x축 방향 30cm 위치)
- **PD 컨트롤러 (500 Hz)** — 5단계 상태 머신, Kp=500 / Kd=50
  ```
  WAIT_STABLE → INIT_CONTROL → CHECK_MOTOR → INIT_POSITION → RUN_CONTROL
  ```
- **Task-space IK** — Damped Least Squares로 EE 위치/자세 추종 (6×6 Jacobian)
- **제어 모드 전환** — Joint-space ↔ Task-space 실시간 전환
- **Meshcat 웹 시각화 (port 7000)** — TriangularMeshGeometry로 실제 UR5e 외형 렌더링 (Box geom 포함)
- **멀티스레드** — 500 Hz 제어 스레드 / 시각화 메인 스레드 분리

### 센서 시스템

매 `mj_step()` 직후 실시간으로 읽을 수 있으며, 50주기마다 `/tmp/robot_state.json`에 기록됩니다.

| 항목 | 소스 | 설명 |
|------|------|------|
| `q` | `data.qpos[:6]` | 관절 위치 (rad) |
| `dq` | `data.qvel[:6]` | 관절 속도 (rad/s) |
| `ddq` | `data.qacc[:6]` | 관절 가속도 (rad/s²) |
| `tau` | XML `actuatorfrc` 센서 | 관절 토크 (Nm), 6축 |
| `contacts` | `data.contact` + `mj_contactForce` | 접촉 목록 |
| `ee_pos` | `data.xpos[ee_body]` | EE 위치 (m) |
| `ee_euler_deg` | EE 회전행렬 → ZYX 오일러 | EE 자세 (deg) |

**contacts 항목 구조:**
```json
{
  "geom1": 3,  "geom2": 12,
  "dist": -0.002,
  "pos":         [x, y, z],
  "normal":      [nx, ny, nz],
  "force_world": [fx, fy, fz],
  "force_mag":   12.3
}
```

### 안전 기능

- **충돌 감지** — 매 스텝 비인접 링크 접촉 (`dist < 0`) 검사 → `COLLISION_STOP` 상태 전환, 관절 고정
- **충돌 정지 해제** — `/ur5e/cmd/collision_reset` ROS2 토픽 or 대시보드 버튼으로 초기 위치 복귀
- **소프트 관절 한계** — 관절 범위 끝 N% 이내 진입 시 반발 토크 + 속도 감쇠 (kp/kd/margin 런타임 변경 가능)

### 모션 프로파일 (EE Task-space)

모든 EE 명령은 Quintic polynomial 프로파일로 실행됩니다.

**기본 프로파일** (`vel_start=0, vel_end=0`):
```
s(τ) = 10τ³ - 15τ⁴ + 6τ⁵   (τ = t / duration)
```

**일반 프로파일** (`vel_start > 0` 또는 `vel_end > 0`):
```
s(τ) = v0·τ + (10-6v0-4v1)·τ³ + (-15+8v0+7v1)·τ⁴ + (6-3v0-3v1)·τ⁵
```

**운동 한계 자동 적용** (`UR5eRTController` 생성자 인자):

| 파라미터 | 기본값 | 설명 |
|---|---|---|
| `ee_max_vel` | 1.0 m/s | EE 최대 선속도 |
| `ee_max_acc` | 5.0 m/s² | EE 최대 가속도 |
| `ee_max_jerk` | 50.0 m/s³ | EE 최대 저크 |

사용자가 요청한 duration이 너무 짧으면 이진탐색으로 최소 duration을 계산해 자동 연장.

**연속 블렌딩** (이동 중 새 명령 수신 시):
1. 현재 프로파일 속도 `f'(τ) × Δpos / T` 계산
2. 새 이동 방향으로 투영 → `vel_start` 자동 설정
3. `T ≥ 10·dist / (6·v0 + 4·v1)` 조건으로 즉시 감속 모드 보장 (속도 급상승 방지)

### ROS2 인터페이스

| 방향 | 토픽 | 타입 | 설명 |
|---|---|---|---|
| Subscribe | `/ur5e/cmd/joint_trajectory` | `JointTrajectory` | 관절 궤적 명령 (waypoint + duration) |
| Subscribe | `/ur5e/cmd/ee_target` | `PoseStamped` | EE 목표 위치·자세 (task-space) |
| Subscribe | `/ur5e/cmd/mode` | `Bool` | true=task-space / false=joint-space |
| Subscribe | `/ur5e/cmd/soft_limits` | `String` | 소프트 한계 파라미터 JSON |
| Subscribe | `/ur5e/cmd/collision_reset` | `Bool` | 충돌 정지 해제 |
| Publish | `/ur5e/state/joint` | `JointState` | 관절 위치·속도 (10 Hz) |
| Publish | `/ur5e/state/ee_pose` | `PoseStamped` | EE 현재 위치·자세 (10 Hz) |

**EE 명령 인코딩:**
- `header.stamp` → 소요 시간 (sec + nanosec)
- `header.frame_id` → `"base|vel_start|vel_end"` 형식으로 출발·도착 속도 (m/s) 전달

### 웹 대시보드 (port 8765)

- **컨테이너 제어** — 시작 / 중지 / 재시작 / 재빌드 버튼
- **시뮬레이션 프로세스 모니터** — main_test.py 실행 개수 확인, 중복 시 경고 배너, 종료·재시작 버튼
- **Joint-space 제어 탭** — 6관절 목표값 + 소요 시간 입력, 현재값 불러오기
- **Task-space 제어 탭** — EE 위치(xyz) + 오일러 각 입력, 쿼터니언 실시간 프리뷰, 출발·도착 속도 지정
- **소프트 관절 한계 설정** — 활성화 토글, margin 슬라이더, kp/kd 입력
- **관절 상태 테이블** — 현재값(deg) + 범위 내 위치 시각화 바 (근접도에 따라 색상 변화)
- **충돌 감지 배너** — 충돌 발생 시 상단 고정 빨간 배너, 정지 해제 버튼
- **Meshcat 뷰어 프록시** — `/meshcat` 경로로 터널 없이 외부에서 3D 시각화 접근 (wss:// 자동 재작성)
- **스트리밍 명령** — 컨테이너 내 임의 명령 실시간 출력
- **실시간 로그** — `/tmp/bg.log` 스트리밍
- **인증** — 커스텀 HTML 로그인 오버레이 + sessionStorage 캐시

### 외부 접속
- **Cloudflare Tunnel** — 고정 IP 없이 외부 HTTPS URL 자동 생성, 자동 재연결 루프

---

## 빠른 시작

### 1. 시뮬레이션 컨테이너 실행

```bash
docker compose build
docker compose up -d
```

### 2. 웹 대시보드 실행 (권장)

```bash
cd web_dashboard

# .env 파일 생성 (최초 1회)
echo "AUTH_USERNAME=heroi" > .env
echo "AUTH_PASSWORD=<비밀번호>" >> .env

./launch.sh           # 서버 + Cloudflare 터널
./launch.sh --no-tunnel  # 로컬만
./launch.sh stop      # 전체 종료
```

실행 시 출력:
```
[1/4] 기존 프로세스 정리 중...
[2/4] 대시보드 서버 시작 중...
    ✓ 서버 실행 완료 (PID: 12345)
    로컬 주소:         http://192.168.x.x:8765
    Meshcat 로컬 주소: http://192.168.x.x:7000
[3/4] Cloudflare 터널 시작 중...
    ✓ 터널 연결 완료
    외부 주소 (대시보드): https://xxxx.trycloudflare.com
[4/4] Meshcat 접속 주소:
    외부 주소 (Meshcat): https://xxxx.trycloudflare.com/meshcat
```

---

## API 엔드포인트

모든 엔드포인트는 HTTP Basic Auth 필요 (`GET /` 제외).

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
| POST | `/api/pub/joint` | ROS2 joint 궤적 명령 퍼블리시 |
| POST | `/api/pub/ee` | ROS2 EE 명령 퍼블리시 |
| POST | `/api/pub/mode` | ROS2 제어 모드 전환 |
| POST | `/api/pub/soft_limits` | 소프트 관절 한계 파라미터 전송 |
| GET | `/api/sim/status` | main_test.py 프로세스 목록 |
| POST | `/api/sim/kill` | main_test.py 전체 종료 |
| POST | `/api/sim/restart` | main_test.py 종료 후 백그라운드 재시작 |
| POST | `/api/sim/collision_reset` | 충돌 정지 해제 |
| GET | `/meshcat` | Meshcat 3D 뷰어 프록시 |

---

## 관리 명령어

```bash
# 로그 실시간 확인
tail -f /tmp/heroi-dashboard.log
tail -f /tmp/heroi-tunnel.log

# 프로세스 확인
pgrep -a -f "web_dashboard/main.py"

# 종료
./launch.sh stop

# 외부 터널 URL 확인
grep -o "https://[^ ]*trycloudflare.com" /tmp/heroi-tunnel.log | tail -1
```

---

## TODO

### 완료
- ✅ MuJoCo 씬 구성 — 장애물(정육면체) 동적 추가, load_scene_model()로 Python에서 XML 조작
- ✅ 센서 시스템 — 관절 토크(actuatorfrc XML 센서) / 접촉 위치·법선·힘벡터 / 관절 가속도
- ✅ 충돌 감지 — 비인접 링크 contact 검사, COLLISION_STOP 상태, 대시보드 배너 + 해제 버튼
- ✅ 소프트 관절 한계 — margin 기반 반발 토크 + 속도 감쇠, 런타임 파라미터 변경
- ✅ 관절 한계 시각화 — 대시보드 테이블에 현재값·범위 바·색상 표시
- ✅ JointTrajectory 인터페이스 — quintic 보간, duration 파라미터
- ✅ Task-space IK — Damped Least Squares EE 위치/자세 추종
- ✅ 모션 프로파일 — Quintic polynomial, vel_start/vel_end, 운동 한계 자동 적용
- ✅ 연속 블렌딩 — 이동 중 새 EE 명령 수신 시 현재 속도 자동 인계
- ✅ Meshcat 대시보드 프록시 — `/meshcat` 경로, wss:// 자동 재작성
- ✅ 시뮬 프로세스 모니터 — 중복 실행 감지, 종료·재시작
- ✅ 도커 내부 계층 정리 — my_ur5e_controller / web_dashboard 분리 마운트
- ✅ Robot State Publisher / TF 트리 — ur5e_minimal.urdf

### 진행 예정
1. **힘 제어 (Force Control)** — 접촉력 피드백을 이용한 임피던스 제어
2. **씬 오브젝트 확장** — 다수 장애물, 목표 지점 마커 추가
3. **궤적 기록 / 재생** — 웹 UI에서 수동 조작 궤적을 JSON으로 저장 후 반복 재생
4. **실제 UR5e 하드웨어 연결** — RTDE 인터페이스로 실물 로봇 명령 전달
