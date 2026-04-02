# UR5e MuJoCo Simulation via Meshcat (ROS2 Humble)

## 1. 시스템 아키텍처
본 환경은 원격 리눅스 서버에서 Docker 컨테이너로 실행되며, 웹 브라우저를 통해 시뮬레이션 시각화 결과를 확인합니다.

* **Compute Node (Remote Linux):** Docker, ROS2 Humble, MuJoCo, Meshcat Server
* **Client Node (Mac):** VSCode (Remote SSH), Web Browser (Chrome)

## 2. 제어 흐름 (Control Flow)
1.  **Input:** 사용자가 ROS2 Topic(`joint_trajectory_controller` 등)으로 목표 값을 발행합니다.
2.  **Controller:** ROS2 Controller가 목표 값과 현재 상태를 비교하여 제어 입력을 계산합니다.
3.  **Hardware Interface (MuJoCo Bridge):** 계산된 토크/위치 값을 MuJoCo API를 통해 가상 로봇에 전달합니다.
4.  **Simulation:** MuJoCo 엔진이 물리 연산을 수행하고 로봇의 상태를 업데이트합니다.
5.  **Visualization:** MuJoCo의 상태 데이터를 Meshcat 서버로 전송하여 브라우저에서 렌더링합니다.

현재까지 구축된 환경의 **폴더 구조**를 포함하여, 지금까지의 작업 내용과 수정 사항을 정리한 최종 리포트(`PROJECT_STATE.md`)입니다. 

이 구조를 유지하면 나중에 ROS2 패키지로 확장하거나 다른 로봇 모델을 추가할 때 매우 관리가 용이해집니다.

---

# 📝 UR5e MuJoCo-Meshcat 프로젝트 상태 보고서

## 1. 프로젝트 디렉토리 구조 (Current Directory Tree)

현재 호스트 리눅스 서버(`~/heroi/`)와 Docker 컨테이너 내부가 `volumes`로 연결된 구조입니다.

```text
/home/jsoori/heroi/                  # 프로젝트 루트 (호스트)
├── Dockerfile                       # ROS2 Humble + MuJoCo 빌드 정의
├── docker-compose.yaml              # 컨테이너 및 네트워크(Port 7000) 설정
├── ros_entrypoint.sh                # 컨테이너 시작 시 ROS2 환경 로드 스크립트
└── my_ur5e_controller/              # 메인 작업 폴더 (컨테이너의 /ros2_ws/src/에 마운트)
    ├── view_ur5e.py                 # [최종] 메쉬 기반 시각화 실행 코드
    └── (기타 테스트 스크립트)
        
# 컨테이너 내부 전용 (빌드 시 생성됨)
/ros2_ws/src/
└── mujoco_menagerie/               # DeepMind의 로봇 모델 저장소 (Git Clone됨)
    └── universal_robots_ur5e/      # UR5e의 .xml 및 .obj 메쉬 파일 위치
```

---

## 2. 작업 히스토리 및 트러블슈팅 요약

| 단계 | 발생한 문제 | 원인 분석 | 해결 방법 |
| :--- | :--- | :--- | :--- |
| **환경 구축** | `ros_entrypoint.sh` Not Found | 빌드 컨텍스트에 파일 부재 | 호스트에 파일 생성 후 `docker compose build` |
| **권한 설정** | `EACCES: permission denied` | Docker(root) 생성 폴더 소유권 문제 | `sudo chown -R $USER:$USER` 실행 |
| **시각화 연결** | 터미널 멈춤 (Hang) | Meshcat 서버 미가동 대기 | `vis = meshcat.Visualizer()`로 자동 서버 실행 |
| **도형 렌더링** | `AttributeError: ... Axes` | 라이브러리에 `Axes` 속성 없음 | `g.Box`로 대체하여 위치 우선 확인 |
| **메쉬 로딩** | `AssertionError: Nx3 array` | MuJoCo와 Meshcat의 행렬 차원 불일치 | `vertices.T` 제거 (Nx3 데이터 그대로 전달) |

---

## 3. 핵심 시스템 개략도 및 제어 흐름

### **[시스템 아키텍처]**
1.  **MuJoCo Engine**: `ur5e.xml`을 읽어 물리 법칙(중력, 마찰, 충돌) 적용 및 로봇 상태 계산.
2.  **Python Bridge (`view_ur5e.py`)**: 
    * MuJoCo에서 각 조인트의 전역 위치(`xpos`)와 회전(`xmat`) 추출.
    * Meshcat 비주얼라이저로 4x4 변환 행렬 전송.
3.  **Meshcat Server**: ZMQ 통신을 통해 받은 데이터를 7000번 포트로 웹 스트리밍.
4.  **User Browser**: Mac Chrome에서 실시간 로봇 움직임 모니터링.



---

## 4. 최종 실행 코드의 주요 특징
* **자동 서버 시작**: `vis = meshcat.Visualizer()`를 통해 별도의 서버 구동 없이 즉시 실행 가능.
* **실제 메쉬 반영**: 단순 상자가 아닌 `TriangularMeshGeometry`를 사용하여 UR5e의 실제 외형 렌더링.
* **실시간 동기화**: `while` 루프 내에서 MuJoCo의 물리 스텝과 시각화 업데이트 속도를 일치시킴.

---

## 5. 다음 단계 (Next Milestone)
현재는 단순히 시각화만 수행하는 단계입니다. 다음 단계는 **ROS2와의 완전한 통합**입니다.

1.  **Joint State Publisher**: MuJoCo의 관절 상태를 ROS2 메시지로 발행.
2.  **Robot State Publisher**: URDF와 연동하여 TF(Transform) 트리 구성.
3.  **Controller**: ROS2 Topic 명령을 받아 MuJoCo 로봇의 모터를 구동.

위 구조대로 잘 정리되셨나요? 이제 이 환경 위에서 로봇을 움직이는 **ROS2 노드** 작성을 시작해볼 준비가 되셨다면 알려주세요!