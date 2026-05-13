# heroi 프로젝트 개요

> 이 파일 하나로 프로젝트 전체 구조와 흐름을 파악할 수 있도록 작성했습니다.

---

## 2026-04-28 ~ 04-29 세션 작업 요약

이번 세션에서 완료한 작업 목록:

### ReGraspReflex 통합 (Option A — main_test.py 직접 통합)
- **`robotiq_grasp_adapter.py`**
  - `step()` 재설계: AntiSlip / ReGraspReflex를 독립적으로 실행 (한쪽 비활성 시 다른 쪽은 계속 동작)
  - `_read_contact_state_regrasp(fi)`: EE 기준 접촉점 Y 편차로 `contact_pos[0]` 계산 (`_CONTACT_POS_SCALE=10`)
  - `_read_motor_states_regrasp()`: 2-DOF 모터 상태 (`q[0]`=그리퍼 비율, `q[1]`=EE Y)
  - `_apply_regrasp_output()`: `q_target` 평균→그리퍼, 차이×`_ARM_LATERAL_SCALE(0.04)`→팔 Y lateral
  - `_write_grasp_state()`: `regrasp.enabled` / `regrasp.state` 필드 추가
- **`ur5e_rt_controller.py`**
  - `regrasp_enabled` 플래그 추가
  - `/ur5e/cmd/regrasp` (Bool) 구독 + `_cb_regrasp()` 콜백

### 웹 UI 정비
- **MJPEG 시뮬 카드 제거**: 그리퍼/combined 시뮬 카드 삭제, `/meshcat` 경로로 3D 씬 접속 방식으로 전환
- **ReGrasp Reflex 버튼**: 핸드 제어 카드 Anti-Slip 아래에 추가, `/api/pub/regrasp` 호출
- **충돌 정지 해제 버튼**: 기본 `disabled`, collision 감지 시에만 활성화
- **버그 수정**: `regraspToggle()` `Content-Type: application/json` 헤더 누락(FastAPI 422) 수정

### MeshCat stale zmqserver 문제 수정
- **원인**: `pkill -9`로 main_test.py 종료 시 자식 zmqserver 프로세스가 살아남아 쌓임 (최대 5개 확인). 대시보드가 오래된 정적 서버에 연결 → 로봇이 MeshCat에서 움직이지 않음
- **수정**:
  - `main_test.py`: 시작 시 `/tmp/meshcat_port.txt`에 현재 포트 기록
  - `main.py` `_find_meshcat_port()`: 포트 파일 우선 읽기 → WS 탐색 폴백
  - `main.py` `sim/restart`: zmqserver 전체 종료 + 포트 파일 삭제 + 포트 캐시 무효화
- **버그 수정**: `sim_restart` 백그라운드 명령 끝 `&` 누락(bash 무한 대기) 수정

### PROJECT.md 업데이트
- 시스템 구성도, 핵심 파일 섹션, API 표, 임시 파일 경로, 향후 과제 전면 갱신

---

## 프로젝트 목적

UR5e 로봇팔 + Robotiq 2F-85 그리퍼를 MuJoCo 물리 시뮬레이션에서 구동하고,
웹 대시보드(FastAPI)와 ROS2 토픽으로 원격 제어·모니터링하는 시스템.
최종 목표는 Orca Hand(17-DOF 인간형 핸드)를 로봇팔 끝에 부착해 파지 제어를 완성하는 것.

---

## 시스템 구성

> **macOS 브랜치** — Docker Desktop + 명시적 포트 매핑, 인증 없음

```
[브라우저]
     │  HTTP (localhost 또는 Tailscale Funnel)
     ▼
[web_dashboard/main.py]  ← FastAPI 서버, 호스트(Mac)에서 실행
     │  Docker SDK + REST + WebSocket
     │  WS /meshcat-ws  → zmqserver 프록시 (host:8000-8010)
     ▼
[Docker 컨테이너: ur5e_mujoco_ros2]
     │  포트 매핑: 8000-8010 → 7000-7010 (Meshcat)
     │             8100 → 7100 (Gripper viewer)
     │             8101 → 7101 (Combined viewer)
     │  ROS2 (Humble) 토픽
     ▼
[my_ur5e_controller/main_test.py]  ← 시뮬레이션 엔트리포인트
     ├── UR5eRTController        (500 Hz 제어 루프)
     │     └── ROS2 구독: ee_target, joint_trajectory, gripper,
     │                    collision_reset, antislip, regrasp
     ├── RobotiqGraspAdapter     (100 Hz 반사 제어)
     │     ├── AntiSlipReflex    (슬립 감지 → 그리퍼 닫기)
     │     └── ReGraspReflex     (비대칭 감지 → 그리퍼 + 팔 Y lateral)
     └── ObjectApproachNode      (10 Hz 물체 접근 상태머신)
```

---

## 폴더 구조

```
heroi/
├── PROJECT.md                   ← 이 파일 (프로젝트 개요)
├── CHANGE.md                    ← 변경 이력
├── CLAUDE.md                    ← Claude Code 규칙 (kr_ prefix 제외 등)
├── Dockerfile                   ← ROS2 Humble + MuJoCo + Python 환경
├── docker-compose.yaml          ← 컨테이너 설정 (포트 매핑, 볼륨 마운트)
├── start.sh                     ← 전체 스택 한번에 시작/종료 (macOS 전용)
├── cloudflared-linux-amd64.deb  ← Cloudflare Tunnel 설치파일 (미사용, gitignore 예정)
│
├── my_ur5e_controller/          ← 시뮬레이션 코어 (컨테이너 내 /ros2_ws/src/my_ur5e_controller)
│   ├── main_test.py             ← 엔트리포인트: 씬 로드 → 스레드 시작 → 시각화 루프
│   ├── ur5e_rt_controller.py    ← UR5e RT 제어기 (PD + IK + 궤적)
│   ├── grasp_controller.py      ← 파지 제어기 (반사 제어 계층 구조)
│   ├── robotiq_grasp_adapter.py ← Robotiq 2F-85 ↔ GraspController 브리지
│   ├── object_approach_node.py  ← 물체 접근 ROS2 노드 (HOVER → DESCEND)
│   ├── scene_config.py          ← 씬 오브젝트 설정 (Box/Sphere/Cylinder)
│   ├── gripper_config.py        ← 그리퍼 선택 (ACTIVE 한 줄만 바꾸면 교체)
│   └── ur5e_minimal.urdf        ← 최소 URDF (시각화용)
│
├── web_dashboard/               ← 웹 대시보드 (호스트에서 실행)
│   ├── main.py                  ← FastAPI 서버
│   ├── hand_ui_config.py        ← 핸드/그리퍼 UI 설정 (ACTIVE 한 줄로 교체)
│   ├── launch.sh                ← 서버 시작 + Tailscale Funnel 연결
│   └── static/index.html        ← 대시보드 프론트엔드
│
├── orca_hand/                   ← Orca Hand 하드웨어 자료
│   ├── bambu/                   ← 3MF 프린트 파일 (Bambu Lab)
│   ├── orcahand_description/    ← git submodule: URDF/MJCF 모델
│   ├── stl/                     ← STL 메쉬 파일
│   └── bom.csv                  ← 부품 목록 (Bill of Materials)
│
└── docs/                        ← 설계 노트
    ├── controller_theory.md     ← 제어 이론 정리
    ├── dynamics_bias_torque.md  ← 중력/코리올리 보상 이론
    ├── grasp_controller_flow.md ← GraspController 처리 흐름
    ├── project_structure.md     ← Notion 설계 구조 (참고용 원안)
    └── reflexive_control.md     ← MIT 반사 제어 이론 정리
```

---

## 핵심 파일 상세

### `my_ur5e_controller/main_test.py`
시뮬레이션 진입점. MjSpec으로 UR5e + 그리퍼 + 씬 오브젝트를 합쳐 로드하고,
제어기·반사 제어·물체 접근 스레드를 시작한 뒤 Meshcat 시각화 루프를 돌린다.

```
load_scene_model()            ← gripper_config + scene_config 병합
Meshcat 초기화 후 /tmp/meshcat_port.txt에 포트 기록  ← 대시보드 포트 탐지용
UR5eRTController.start()      ← 500Hz 스레드
RobotiqGraspAdapter.start()   ← 100Hz 스레드
ObjectApproachNode.start()    ← 10Hz 스레드
메인 루프: update_visualizer() + 상태 출력
```

---

### `my_ur5e_controller/ur5e_rt_controller.py`
500Hz RT 제어기. 상태머신 기반으로 동작한다.

**상태머신**: `WAIT_STABLE → INIT_CONTROL → CHECK_MOTOR → INIT_POSITION → RUN_CONTROL → COLLISION_STOP → COLLISION_ESCAPE`

**제어 우선순위 (RUN_CONTROL)**: 궤적 > Task-space IK > 관절 홀드

**핵심 기능**:
- Damped Least Squares IK (6-DOF)
- Quintic 모션 프로파일 (속도·가속도·저크 한계 자동 계산)
- 연속 블렌딩 (이동 중 새 명령 수신 시 현재 속도를 초기조건으로 부드럽게 연결)
- Soft Joint Limits (침투 시 반발 토크)
- 충돌 감지 → 직전 안전 위치로 후퇴
- 상태를 `/tmp/robot_state.json`에 10Hz로 원자적 저장

**ROS2 구독 토픽**:
| 토픽 | 타입 | 기능 |
|------|------|------|
| `/ur5e/cmd/ee_target` | PoseStamped | EE 위치·자세 명령 |
| `/ur5e/cmd/joint_trajectory` | JointTrajectory | 관절 궤적 |
| `/ur5e/cmd/gripper` | Float64 | 그리퍼 개폐 (0~1) |
| `/ur5e/cmd/mode` | Bool | task-space/joint-space 전환 |
| `/ur5e/cmd/soft_limits` | String(JSON) | 소프트 리밋 파라미터 |
| `/ur5e/cmd/collision_reset` | Bool | 충돌 정지 해제 |
| `/ur5e/cmd/antislip` | Bool | Anti-Slip Reflex ON/OFF |
| `/ur5e/cmd/regrasp` | Bool | ReGrasp Reflex ON/OFF |

---

### `my_ur5e_controller/grasp_controller.py`
MIT Reflexive Control 기반 파지 제어기. 실제 하드웨어(Orca Hand) 제어를 목표로 설계됨.

**계층 구조**:
```
GraspPlanner        (< 1 Hz)   파지 타입 선택, q_target 주입
  └── GraspController (500 Hz) 상태머신 RT 루프
        ├── FourBarLinkage     4절 링크 FK/IK (Freudenstein 방정식)
        ├── FingerKinematics   평면 3R FK, 6×3 자코비안
        └── ReflexCoordinator  우선순위 기반 반사 관리자
              ├── AntiSlipReflex   (priority=0) 슬립 → 수직력 증가
              └── ReGraspReflex    (priority=1) 비대칭 → antipodal 재정렬
```

**반사 상태머신**: `IDLE → TRIGGERED → EXECUTING → RESOLVED → IDLE`

현재 MuJoCo 시뮬레이션에서는 `GraspController` 직접 사용 대신,
`RobotiqGraspAdapter`를 통해 AntiSlipReflex와 ReGraspReflex를 적용한다.

---

### `my_ur5e_controller/robotiq_grasp_adapter.py`
Robotiq 2F-85(1-DOF 병렬 그리퍼)를 GraspController 반사 레이어에 연결하는 어댑터.

**AntiSlipReflex** (항상 독립 실행):
- MuJoCo `mj_contactForce()`로 패드 접촉력 직접 읽기
- 슬립 감지 시 그리퍼를 더 닫는 방향으로 제어
- **안전 정책**: 더 닫히는 방향만 허용 (물체 낙하 방지)

**ReGraspReflex** (`regrasp_enabled=True`일 때만 실행):
- n_joints=2 DOF 매핑: `q[0]` = 그리퍼 비율, `q[1]` = EE Y 위치
- 비대칭 파지 감지(sym_score < threshold) 시 그리퍼 + 팔 lateral Y 동시 조정
- `_ARM_LATERAL_SCALE = 0.04` (q_target 차이 1 → 최대 ±4cm Y 이동)
- `_CONTACT_POS_SCALE = 10.0` (월드 좌표 lateral 편차 amplify)

**공통**:
- 무게 추정: 수직 지지력 합 / g (이동 평균 50 샘플)
- 상태를 `/tmp/grasp_state.json`에 10Hz로 저장 (antislip + regrasp + weight + friction)

---

### `my_ur5e_controller/object_approach_node.py`
물체 이름을 받아 EE를 단계적으로 접근시키는 ROS2 노드.

**접근 단계**: `IDLE → HOVER (물체 위 15cm) → DESCEND (물체 위 2cm) → DONE`

| 토픽 | 방향 | 내용 |
|------|------|------|
| `/ur5e/approach/start` | 구독 | 물체 이름 수신 |
| `/mujoco/query_objects` | 구독 | 씬 물체 위치 조회 트리거 |
| `/ur5e/cmd/ee_target` | 발행 | EE 목표 위치 |
| `/mujoco/scene_objects` | 발행 | 씬 내 물체 위치 JSON |

---

### `web_dashboard/main.py`
FastAPI 기반 웹 대시보드. 호스트에서 실행되며 Docker SDK로 컨테이너를 제어한다.

**주요 API**:
| 엔드포인트 | 기능 |
|------------|------|
| `GET /api/status` | 컨테이너 상태 |
| `POST /api/start/stop/restart/rebuild` | 컨테이너 제어 |
| `POST /api/exec`, `/api/exec_bg` | 컨테이너 내 명령 실행 |
| `GET /api/robot_state` | `/tmp/robot_state.json` 반환 |
| `GET /api/grasp_state` | `/tmp/grasp_state.json` 반환 |
| `POST /api/pub/ee` | EE 이동 명령 발행 |
| `POST /api/pub/joint` | 관절 궤적 발행 |
| `POST /api/pub/hand` | 핸드/그리퍼 제어 발행 |
| `POST /api/pub/regrasp` | ReGrasp Reflex ON/OFF 발행 |
| `POST /api/sim/restart` | main_test.py 재시작 (zmqserver도 함께 정리) |
| `POST /api/sim/collision_reset` | 충돌 정지 해제 |
| `GET /meshcat` | Meshcat 3D 시각화 (프록시, `/tmp/meshcat_port.txt` 기반 포트 탐지) |
| `WS /meshcat-ws` | Meshcat WebSocket 프록시 |

인증: 없음 (macOS 로컬 전용 브랜치)

**Meshcat 포트 탐지 전략** (stale zmqserver 문제 방지):
1. 컨테이너 내 `/tmp/meshcat_port.txt` 파일 우선 읽기 (main_test.py가 시작 시 기록)
2. 파일 없으면 WS 씬 데이터 수신 여부로 8010→8000 탐색 (호스트 포트 기준, 폴백)
3. `sim/restart` 시 기존 zmqserver 전체 종료 + 포트 파일 삭제 + 포트 캐시 무효화

**알려진 버그 수정 이력**:
- `sim/restart` 백그라운드 명령 끝에 `&` 누락 → bash가 프로세스 종료까지 무한 대기, 재시작 실패 및 다중 인스턴스 race condition 발생 → `&` 추가로 수정

---

### `web_dashboard/static/index.html`
웹 대시보드 프론트엔드. 주요 카드 구성:

| 카드 | 주요 기능 |
|------|-----------|
| 컨테이너 제어 | 시작/중지/재시작/재빌드 |
| 시뮬레이션 제어 | main_test.py 재시작, 충돌 정지 해제 (collision 감지 시에만 활성화) |
| 로봇 제어 | EE 위치 명령, 관절 명령, task/joint space 전환 |
| 핸드 제어 | 그리퍼 개폐, Anti-Slip ON/OFF, **ReGrasp Reflex 실행/중지** |
| 상태 모니터 | EE 위치·자세, 관절각, 무게 추정, 반사 상태 폴링 |
| 로그 스트림 | bg.log tail |

> `/meshcat` 경로로 직접 접속하면 3D 씬 뷰어를 볼 수 있다.

**알려진 버그 수정 이력**:
- `regraspToggle()` fetch에 `Content-Type: application/json` 헤더 누락 → FastAPI가 422 반환, ROS2 토픽 미발행 → 헤더 추가로 수정

---

### `web_dashboard/hand_ui_config.py`
웹 UI의 핸드 제어 카드 설정. `ACTIVE` 변수 한 줄만 바꾸면 UI가 바뀐다.

현재 `ACTIVE = ROBOTIQ_2F85`. 추후 `ORCA_HAND`로 교체 예정.

---

### `my_ur5e_controller/gripper_config.py`
그리퍼 선택 설정. `ACTIVE` 변수 한 줄만 바꾸면 그리퍼가 교체된다.

현재 `ACTIVE = ROBOTIQ_2F85_V4`. 지원 프리셋: `ROBOTIQ_2F85_V4`, `ROBOTIQ_2F85`, `UMI_GRIPPER`

---

### `my_ur5e_controller/scene_config.py`
씬 오브젝트 설정. `OBJECTS` 리스트에 `Box/Sphere/Cylinder`를 추가하면 씬에 배치된다.

---

## 실행 방법

### 1. 전체 스택 한번에 시작 (권장)
```bash
cd controller_docker
./start.sh           # 컨테이너 + 대시보드 동시 시작
./start.sh stop      # 전체 종료
```

### 2. 수동 실행
```bash
# 컨테이너 빌드 & 시작
docker compose up -d --build

# 웹 대시보드 시작 (별도 터미널)
cd web_dashboard
pip3 install -r requirements.txt
./launch.sh --no-tunnel  # 로컬 전용 (포트 8765)
./launch.sh              # Tailscale Funnel 포함
./launch.sh stop         # 종료
```

### 3. 직접 ROS2 명령 (컨테이너 내)
```bash
# EE 이동
ros2 topic pub --times 1 /ur5e/cmd/ee_target geometry_msgs/msg/PoseStamped \
  '{header: {stamp: {sec: 2}}, pose: {position: {x: 0.3, y: 0.0, z: 0.5}, orientation: {w: 1.0}}}'

# 그리퍼 닫기
ros2 topic pub --times 3 /ur5e/cmd/gripper std_msgs/msg/Float64 '{data: 1.0}'

# 물체 접근
ros2 topic pub --times 1 /ur5e/approach/start std_msgs/msg/String '{data: "box"}'
```

---

## 임시 파일 경로 (컨테이너 내)

| 경로 | 내용 | 갱신 주기 |
|------|------|-----------|
| `/tmp/robot_state.json` | 관절각·EE 위치·충돌·그리퍼 상태 | 10 Hz |
| `/tmp/grasp_state.json` | AntiSlip·ReGrasp 반사 상태·무게 추정·마찰 | 10 Hz |
| `/tmp/meshcat_port.txt` | 현재 MeshCat zmqserver 포트 번호 | 시작 시 1회 |
| `/tmp/bg.log` | main_test.py 실행 로그 | 실시간 |

---

## 설계 패턴 & 규칙

- **설정 교체**: `gripper_config.py`, `scene_config.py`, `hand_ui_config.py` 각각 `ACTIVE` 또는 `OBJECTS` 한 줄만 수정
- **파일명 규칙**: `kr_` prefix 파일은 모든 작업에서 완전 무시 (`CLAUDE.md` 규칙)
- **상태 공유**: 컨테이너 ↔ 대시보드 간 상태는 `/tmp/*.json` 원자적 파일로 교환
- **스레드 안전**: `threading.Lock()` 으로 RT 루프와 ROS2 콜백 간 경쟁 방지
- **변경 이력**: 코드 수정 시 `CHANGE.md`에 날짜 + 설명 + 이전 커밋 해시 추가

---

## 향후 과제

- [ ] Orca Hand 17-DOF 실제 하드웨어 연동 (`GraspController` → 실제 모터 드라이버)
- [ ] `hand_ui_config.py`의 `ORCA_HAND` 토픽 정의 완성
- [ ] `orca_hand/orcahand_description` MJCF를 UR5e 씬에 부착
- [ ] ReGraspReflex 비대칭 파지 검증 (현재 scene_config Box 오브젝트는 크기가 작아 비대칭 접촉 발생 어려움 → 오브젝트 교체 필요)
- [ ] 충돌 후 자동 재시작 또는 오브젝트 재배치 로직 추가
