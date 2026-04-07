제시해주신 프로젝트 요약본을 요청하신 대로 동일한 포맷의 한국어 버전으로 번역 및 정리해 드립니다.

---

# 프로젝트 요약: UR5e MuJoCo-Meshcat 시뮬레이션

## 개요

본 프로젝트는 다음과 같은 기술들을 결합한 **UR5e 협동 로봇 시뮬레이터**입니다.
- **MuJoCo** (DeepMind의 물리 엔진): 정확한 로봇 동역학 시뮬레이션
- **Meshcat** (Three.js/ZMQ 기반 웹 시각화): 브라우저 상의 3D 렌더링
- **ROS2 Humble** (Docker 기반): 기본 미들웨어 프레임워크

최종 목표는 실제 하드웨어 없이도 UR5e 6축 로봇 팔을 시뮬레이션하고, 웹 브라우저에서 이를 시각화하며, 향후 완전한 ROS2 컨트롤러 통합을 위한 기반을 마련하는 것입니다.

---

## 디렉토리 구조

```
heroi/
├── Dockerfile                        # ROS2 Humble + MuJoCo 컨테이너 정의
├── docker-compose.yaml               # 컨테이너 오케스트레이션 (7000번 포트, 호스트 네트워크)
├── ros_entrypoint.sh                 # 컨테이너 시작 시 ROS2 환경 설정
├── .gitignore                        # __pycache__, .venv, build/, log/ 등 제외
├── README.md                         # 한국어 프로젝트 상태 보고서
└── my_ur5e_controller/
    ├── ur5e_rt_controller.py         # 실시간 PD 컨트롤러 (상태 머신 기반)
    ├── view_ur5e.py                  # 단독 메쉬 시각화 스크립트
    └── main_test.py                  # 메인 실행 엔트리 포인트 (컨트롤러 + 시각화)
```

> **컨테이너 전용** (`docker build` 시 클론됨):
> `/ros2_ws/src/mujoco_menagerie/universal_robots_ur5e/` — UR5e URDF, XML 및 `.obj` 메쉬 파일

---

## 기술 스택

| 레이어 | 기술 |
|---|---|
| 언어 | Python 3.10+ |
| 물리 엔진 | MuJoCo (DeepMind) |
| 시각화 | Meshcat (ZMQ + Three.js) |
| 미들웨어 | ROS2 Humble |
| 컨테이너 | Docker + Docker Compose |
| 수학 라이브러리 | NumPy, transforms3d |

---

## 주요 구성 요소

### 1. `ur5e_rt_controller.py` — 실시간 컨트롤러

500 Hz(2ms 타임스텝)로 구동되는 **5단계 상태 머신**:

```
WAIT_STABLE → INIT_CONTROL → CHECK_MOTOR → INIT_POSITION → RUN_CONTROL
```

- **PD 제어** 구현: $\tau = K_p \times (q_{desired} - q_{actual}) + K_d \times (0 - \dot{q}_{actual})$
  - $K_p = 500.0, K_d = 50.0$
- 타이밍 누락 감지 및 로그 기록 (실시간 성능 모니터링)
- 상태 시퀀싱을 통한 초기 구동 시 급격한 조인트 저크(Jerk) 방지
- 공유 MuJoCo `model`/`data` 객체 활용 (설계상 스레드 안전)

### 2. `view_ur5e.py` — 메쉬 시각화

로봇 로딩 및 렌더링을 위한 단독 스크립트:

- MuJoCo Menagerie에서 UR5e URDF/XML 로드
- MuJoCo 내부 배열에서 메쉬 기하 구조(정점 + 면) 추출
- RGBA 재질과 함께 Meshcat에 `TriangularMeshGeometry` 객체 등록
- 시뮬레이션 스텝별 트랜스폼(`xpos`, `xmat`) 업데이트
- 테스트 포함: 숄더 조인트의 정현파(Sinusoidal) 발진 구현

### 3. `main_test.py` — 메인 애플리케이션

제어와 시각화를 통합 관리:

- MuJoCo 모델 로드 및 Meshcat 비주얼라이저 생성 (7000번 포트 자동 시작)
- **데몬 스레드**를 통해 500 Hz 주기로 컨트롤러 실행
- 메인 스레드는 약 50 Hz(20ms 대기) 주기로 시각화 루프 실행
- `RUN_CONTROL` 단계에서 컨트롤러에 정현파 조인트 목표값 전송
- 불안정성 모니터링 (속도 > 50 rad/s 발생 시 안전 정지 트리거)

**스레딩 모델:**
```
메인 스레드 (50 Hz)           컨트롤러 스레드 (500 Hz)
  ├─ Meshcat 업데이트           ├─ MuJoCo 상태 읽기
  ├─ 상태 머신 로그 출력         ├─ PD 토크 계산
  └─ 조인트 명령 전송           └─ MuJoCo에 적용
```

---

## Docker 인프라

### `Dockerfile`
- 베이스 이미지: `osrf/ros:humble-desktop`
- 설치 항목: `mujoco`, `meshcat`, `numpy`, `transforms3d`, 헤드리스 OpenGL 라이브러리
- 빌드 시점에 `mujoco_menagerie` (DeepMind 로봇 모델 저장소) 클론

### `docker-compose.yaml`
- 컨테이너명: `ur5e_mujoco_ros2`
- 볼륨 마운트: 프로젝트 루트 → 컨테이너 내부 `/ros2_ws/src/my_ur5e_controller`
- 호스트 네트워크 모드: SSH 터널링 등을 통한 브라우저 접속 용이성 확보
- `MUJOCO_GL=osmesa`: 헤드리스(디스플레이 서버 없음) 환경 렌더링 설정

### `ros_entrypoint.sh`
- `/opt/ros/humble/setup.bash` 소싱
- 빌드된 워크스페이스가 있을 경우 조건부 소싱
- 전달받은 명령을 ROS2 환경 내에서 실행

---

## 설계 결정 사항

| 결정 사항 | 근거 |
|---|---|
| 상태 머신 기반 컨트롤러 | 급작스러운 움직임 방지 및 안전한 초기화 가능 |
| 별도의 500 Hz 제어 스레드 | 실시간 제어와 상대적으로 느린 시각화 주기 분리 |
| MuJoCo Menagerie 활용 | DeepMind가 공식 관리하는 신뢰도 높은 URDF/메쉬 모델 사용 |
| Rviz 대신 Meshcat 선택 | 웹 기반으로 디스플레이 서버가 없는 원격 환경에서도 시각화 가능 |
| Docker 호스트 네트워크 모드 | 맥(Mac) 등 외부 브라우저에서 Meshcat 포트 접근 편의성 |
| PD 제어 (I항 제외) | 데모 수준에서 충분한 성능; 적분항 누적으로 인한 복잡성 회피 |

---

## 현재 상태

**구현 완료:**
- UR5e의 MuJoCo 물리 시뮬레이션
- 실시간 Meshcat 웹 시각화 (7000번 포트)
- 상태 머신을 적용한 PD 조인트 위치 제어
- 스레드 분리형 제어 + 시각화 아키텍처
- Docker 기반 재현 가능한 환경 구축

**향후 과제 (Next Milestones):**
1. **Joint State Publisher** — MuJoCo 조인트 상태를 ROS2 `sensor_msgs/JointState`로 발행
2. **Robot State Publisher** — ROS2 내비게이션/플래닝을 위한 URDF 기반 TF 트리 구축
3. **ROS2 Controller** — 하드코딩된 입력 대신 ROS2 토픽을 통한 `JointTrajectory` 명령 수신
4. **멀티 로봇/모델 확장** — ROS2 패키지 구조를 활용한 타 로봇 모델 확장

---

## 빠른 시작 가이드

```bash
# 컨테이너 빌드 및 실행
docker compose build
docker compose up -d

# 메인 시뮬레이션 실행 (컨테이너 내부)
docker exec -it ur5e_mujoco_ros2 bash
python3 /ros2_ws/src/my_ur5e_controller/main_test.py

# 브라우저 확인 (로컬 또는 SSH 터널링 이용)
# http://localhost:7000
```