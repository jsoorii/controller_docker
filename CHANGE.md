# Change Log

---

## 2026-05-24 PRE_APPROACH J6 90° 회전 + 충돌 오탐 수정 + mink 의존성 영구 등록

### 변경 파일
- **수정**: `my_ur5e_controller/ur5e_controller.py`
- **수정**: `my_ur5e_controller/object_approach_node.py`
- **수정**: `Dockerfile`
- **수정**: `start.sh`

### ur5e_controller.py 변경

#### 1. `rotmat_axis_angle(axis, angle_deg)` — 모듈 레벨 헬퍼 추가
Rodrigues 공식으로 임의 축+각도 → 3×3 회전행렬 반환.

#### 2. `rotate_ee_target(axis, angle_deg, frame)` — 메서드 추가
현재 EE 타겟 방향에 추가 회전을 적용한다.
```python
# 사용 예시
controller.rotate_ee_target('z', 90.0)          # EE 로컬 z축으로 90°
controller.rotate_ee_target('z', 45.0, 'world') # 월드 Z축으로 45°
controller.rotate_ee_target(np.array([0,0,1]), -30.0)
```

#### 3. `_build_robot_body_ids()` 리팩터링 + 충돌 오탐 수정
```
변경 전: arm 체인만 추적 → forearm_link ↔ gripper/right_coupler(dist=-0.00071m) 오탐 → 즉시 COLLISION_STOP
변경 후: arm_ids + gripper_ids = all_robot_ids, 두 body 모두 내부이면 무시
```
- `_gripper_body_ids`: 이름 패턴(`gripper`, `coupler`, `finger`, `pad`, `knuckle` 등)으로 자동 식별
- `_check_collision()`: `both in _all_robot_ids → continue` 필터 추가

### object_approach_node.py 변경

#### `_safe_z_rot_deg()` — 신규 메서드
MuJoCo 모델에서 `wrist_3` 조인트 range를 직접 읽어 +90°/−90° 중 J6 한계에 여유 있는 방향 반환.
둘 다 가능하면 중심(0)에 가까운 방향 우선.

#### `_cb_start()` 수정 — PRE_APPROACH 90° 회전 적용
```python
# approach_rot 계산 후
rot_deg      = self._safe_z_rot_deg()
wrist_y      = approach_rot[:, 1]   # J6 회전축 = col1(approach_dir)
approach_rot = _rotmat_axis_angle(wrist_y, rot_deg) @ approach_rot
# col1(approach_dir)이 그대로 유지되어 위치 타겟 계산 정상
```

> **주의**: col2(wrist_z)로 회전하면 col1(approach_dir)이 [0,0,1](수직)으로 바뀌어 위치 타겟 계산이 깨짐.
> J6 회전축 = col1(wrist_y) 기준으로 회전해야 approach_dir이 유지됨.

### Dockerfile / start.sh
- `Dockerfile`: `pip3 install "mink[daqp]"` 추가 — 이미지 빌드 시 자동 설치
- `start.sh`: 시뮬레이션 시작 전 `python3 -c "import mink"` 확인 후 없으면 자동 설치 (컨테이너 재시작 대응)

### 최종 검증 결과
```
EE-z 회전=+90°  approach_rot col1=[-0.928, 0.371, 0.0]
PRE_APPROACH(err=0.009m) → HOVER(err=0.007m) → DESCEND(err=0.009m) → DONE ✓
COLLISION_STOP 없음, forearm↔gripper 오탐 제거 확인
```

---

## 2026-05-24 컨트롤러 교체: ur5e_rt_controller → ur5e_controller

### 변경 파일
- **신규**: `my_ur5e_controller/ur5e_controller.py`
- **수정**: `my_ur5e_controller/main_test.py`
- **보존** (삭제 안 함): `my_ur5e_controller/ur5e_rt_controller.py`

### main_test.py 변경 내역

| 위치 | 변경 전 | 변경 후 |
|------|---------|---------|
| import | `from ur5e_rt_controller import UR5eRTController, ControlState` | `from ur5e_controller import UR5eController` |
| 생성자 | `UR5eRTController(model, data, ee_max_vel=0.25, gripper_cfg=...)` | `UR5eController(model, data, gripper_cfg=...)` |

`ControlState`는 main_test.py에서 직접 참조하지 않아 import 제거.

### 복구 방법
```python
# main_test.py 2줄만 되돌리면 구 컨트롤러로 즉시 복구
from ur5e_rt_controller import UR5eRTController, ControlState
controller = UR5eRTController(model, data, ee_max_vel=0.25, gripper_cfg=gripper_config.ACTIVE)
```

### 신규 컨트롤러에서 제거된 기능 (ur5e_rt_controller.py에 코드 보존됨)

#### 1. Scipy Pre-solve IK
`_presolve_ik_from_home()`, `_start_presolve_thread()`

원거리 목표(>10cm)에 대해 scipy L-BFGS-B로 home 출발 오프라인 IK를 풀어
PD 제어로 gross motion 수행. `_ik_step()` 내 Case 1/2/3 분기 로직 포함.

#### 2. Quintic Trajectory Profile
`_compute_min_duration()`, `_smooth_alpha()`, `_smooth_alpha_blend()`

vel/acc/jerk 한계를 만족하는 최소 이동 시간을 이진탐색으로 계산 + quintic polynomial 보간.
연속 블렌딩(이동 중 새 명령 시 현재 속도 이어받기) 포함.

#### 3. Soft Joint Limits
`_soft_limit_torque()`, `_update_soft_limits()`

관절 한계 내측 margin 진입 시 반발 강성(kp) + 속도 감쇠(kd) 토크 인가.
ROS2 `/ur5e/cmd/soft_limits` 런타임 파라미터 조정 가능.

#### 4. Self/Object Collision Repulsion Torque
`_repulsion_torque()`, `_set_robot_geom_margins()`

팔↔팔·팔↔외부 물체 근접(< 2cm) 시 Jacobian 기반 반발 토크를 ctrl에 합산.
(mink CollisionAvoidanceLimit과 별개의 추가 레이어)

#### 5. COLLISION_ESCAPE 상태 + 안전 위치 버퍼
`reset_collision()` + `_safe_q_buffer`

충돌 감지 직전 50스텝(0.1s) 관절 위치 버퍼 유지.
충돌 리셋 시 버퍼 내 가장 오래된 안전 위치로 후퇴 후 정상 제어 복귀.

#### 6. Joint Trajectory 수신
`set_trajectory()`, `_traj_step()`, `_cb_joint_traj()`

`/ur5e/cmd/joint_trajectory` 토픽으로 웨이포인트 기반 관절 궤적 실행.

#### 7. 복잡한 초기화 상태머신
WAIT_STABLE → INIT_CONTROL → CHECK_MOTOR → INIT_POSITION 단계별 초기화 시퀀스.

---

| Date | Description | Rollback Commit |
|------|-------------|-----------------|
| 2026-05-14 | [macos] Meshcat ghost finger 수정: Robotiq 2F-85 V4 그리퍼 패드(left_pad1/2, right_pad1/2)가 BOX 타입 geom으로 vis["env"]에 등록돼 초기 위치에 고정되는 현상 수정 → BOX 타입도 vis["robot"]으로 이동, update_visualizer가 PLANE 타입만 제외하고 모든 geom을 매 프레임 갱신하도록 변경. | `b2aa1cf` |
| 2026-05-14 | [macos] ReGrasp lateral arm 오버슈트 버그 수정: _apply_regrasp_output()에서 매 스텝마다 ee_pos[1](현재 위치, 변화)을 기준으로 이동량을 누적해 오버슈트 발생 → qt[0,1](리플렉스 시작 시 캡처한 EE Y 고정값)을 기준으로 절대 목표 좌표를 계산하도록 수정. | `b2aa1cf` |
| 2026-05-14 | [macos] object_approach_node에 _write_state() 추가: 접근 상태(stage, target, err)를 /tmp/approach_state.json에 원자적으로 기록해 대시보드 폴링 지원. | `b2aa1cf` |
| 2026-05-14 | [macos] main.py에 /api/pub/approach, /api/approach_state 엔드포인트 추가: approach는 컨테이너 내 ros2 topic pub으로 /ur5e/approach/start 발행, approach_state는 컨테이너 내 /tmp/approach_state.json을 읽어 현재 접근 단계 반환. | `b2aa1cf` |
| 2026-05-14 | [macos] 대시보드 "파지 & ReGrasp" 버튼 추가: 클릭 시 순서대로 (1) cylinder 접근 시작 → (2) /api/approach_state 폴링 DONE 대기 (최대 30s) → (3) ReGrasp 활성화 → (4) 그리퍼 완전 닫기 자동 수행. | `b2aa1cf` |
| 2026-05-14 | [macos] 씬 물체 Box → Cylinder 변경: scene_config.py OBJECTS를 반지름 10cm / 높이 20cm 빨간 원통(name="cylinder", pos=[0.3,0,0.1])으로 교체. geom.size 3원소 요구사항 대응([r, h/2, 0.0]) 및 object_approach_node/index.html 내 물체 이름 "box" → "cylinder" 동기화. | `b2aa1cf` |
| 2026-05-14 | [macos] 접근 목표 Y축 노이즈 추가: _cb_start에서 매 접근마다 N(0, 0.025m) 정규분포 ±5cm 클리핑 노이즈 샘플링, HOVER/DESCEND 목표 y 좌표에 반영해 그립 대칭성 편차 실험 가능. | `b2aa1cf` |
| 2026-05-14 | [macos] _get_object_top_z() 헬퍼 추가: 물체 geom 타입별(CYLINDER/BOX/SPHERE/CAPSULE) 상단 z 좌표를 계산해 HOVER/DESCEND 목표 z를 물체 중심이 아닌 물체 상단 기준으로 산출 (원통 center_z=0.1m, top_z=0.2m 오류 수정). | `b2aa1cf` |
| 2026-05-13 | [macos] README.md / PROJECT.md macOS 브랜치 기준으로 전면 재작성: 포트 변경, 인증 제거, start.sh 추가, main↔macos 차이점 표 추가. | `a402c30` |
| 2026-05-13 | [macos] 로그인 인증 기능 전면 제거: main.py에서 HTTPBasic/secrets/require_auth 제거, index.html 로그인 오버레이 CSS·HTML·JS 제거, apiFetch()를 인증 없는 단순 fetch 래퍼로 교체. | `a402c30` |
| 2026-05-13 | [macos] start.sh 추가: Docker 상태 확인 → 컨테이너 시작 → pip 패키지 확인 → 대시보드 백그라운드 실행을 한 명령으로 처리. `./start.sh stop`으로 전체 종료. | `23aca55` |
| 2026-05-13 | [macos] AirPlay Receiver 포트 충돌 해결: 호스트 포트 7000-7010 → 8000-8010 (Meshcat), 7100→8100 (Gripper viewer), 7101→8101 (Combined viewer). main.py 포트 탐색 범위 및 launch.sh 안내 주소 동기화. | `01533c1` |
| 2026-05-13 | [macos] macOS 네이티브 실행 지원: docker-compose에서 network_mode:host 제거 → 명시적 포트 매핑, DISPLAY 환경변수 제거(osmesa headless 유지). launch.sh에서 hostname -I → ipconfig getifaddr, ss -tlnp → lsof로 교체. | `1a00fd2` |
| 2026-04-30 | sim_restart 재시작 안 되는 버그 수정: exec_run(detach=True)+& 조합에서 bash 종료 시 Docker exec 세션이 백그라운드 프로세스를 함께 종료하는 문제 → 1단계(kill+cleanup, detach=False) / 2단계(시작, detach=True, & 없음) 분리로 수정. | `363131c` |
| 2026-04-29 | regrasp 버그 2건 수정: regraspToggle()에 Content-Type 헤더 추가(누락으로 FastAPI 422 반환→토픽 미발행), sim_restart 명령 끝에 & 추가(없으면 bash가 무한 대기→재시작 실패·다중 인스턴스 race condition). | `363131c` |
| 2026-04-29 | MeshCat stale zmqserver 문제 수정: main_test.py가 시작 시 /tmp/meshcat_port.txt에 포트 기록, main.py _find_meshcat_port()가 해당 파일 우선 읽기, sim_restart 시 zmqserver 전체 종료+포트 파일 삭제 추가. | `363131c` |
| 2026-04-29 | ReGraspReflex 완전 통합: robotiq_grasp_adapter.py step()을 AntiSlip/ReGrasp 독립 실행 구조로 재설계, _write_grasp_state()에 regrasp 상태 추가, main.py에 /api/pub/regrasp 엔드포인트 추가, index.html regraspToggle()을 /api/pub/regrasp 호출로 교체. | `363131c` |
| 2026-04-28 | 충돌정지 해제 버튼 비활성화 수정: 버튼에 disabled 기본값 추가, 상태 폴링 시 d.collision.detected 여부에 따라 동적 활성화/비활성화. | `363131c` |
| 2026-04-28 | 핸드 제어 카드에 ReGrasp Reflex 토글 버튼 추가: Anti-Slip 토글 아래에 "▶ ReGrasp Reflex 실행" 버튼 삽입, 클릭 시 /api/sim/combined_start(stop) 호출, 실행 중/중지됨 뱃지 표시. | `363131c` |
| 2026-04-28 | MeshCat 포트 탐지 수정: _find_meshcat_port()를 실제 WS 씬 데이터 수신 여부로 포트 선택하도록 교체(로그 파싱 방식 제거), 씬 데이터가 있는 포트(7003)를 우선 반환. 캐시 TTL 5s→30s. | `363131c` |
| 2026-04-28 | 웹 UI 시뮬 카드 제거 및 MeshCat 뷰어 카드 추가: 그리퍼/combined MJPEG 스트림 카드 삭제, 메모 카드 아래에 "MeshCat 씬 뷰어" 카드(iframe, /meshcat/ 프록시) 추가, 클릭 시 펼침/접힘 토글. | `363131c` |
| 2026-04-28 | UR5e + Robotiq 결합 ReGraspReflex 시뮬레이션 추가: sim_viewer_combined.py 신규 생성(UR5e+Robotiq 풀씬 로드, n_joints=2 DOF 매핑: q[0]=그리퍼비율/q[1]=EE Y, 팔 lateral+그리퍼 동시 제어), main.py에 combined_start/stop/stream 엔드포인트(포트 7101), index.html에 UR5e+Robotiq ReGraspReflex 카드 추가(4000ms 지연 후 스트림 표시). | `363131c` |
| 2026-04-28 | 그리퍼 시뮬레이션 웹 시각화 추가: sim_viewer_server.py 신규 생성(MuJoCo 오프스크린 렌더링+MJPEG HTTP 서버, 포트 7100), main.py에 gripper_start/stop/stream 엔드포인트 추가(asyncio 프로세스 관리), index.html에 그리퍼 시뮬레이션 카드 추가(▶ 시작/■ 중지, 라이브 스트림). | `363131c` |
| 2026-04-28 | ReGraspReflex MuJoCo 검증 완료: grasp_controller.py의 _exec_started 버그 수정(execute() 내 TRIGGERED 체크 dead code → 첫 진입 시 q 초기화 플래그로 교체), two_finger_gripper.xml 신규 생성(2지 각 2-DOF, 힌지 축 방향 수정), test_regrasp_reflex.py 신규 생성(4-Phase 검증, ALL PASS). | `363131c` |
| 2026-04-27 | 프로젝트 정리: cloudflared-linux-amd64.deb.1/.deb.2(동일 MD5 중복) 삭제, view_ur5e.py(초기 프로토타입, main_test.py에 흡수) 삭제, .gitignore에 *.deb 추가, PROJECT.md(프로젝트 전체 개요) 신규 생성. | `363131c` |
| 2026-04-16 | 시뮬레이션 탭에 에러 로그 표시 추가: /api/sim/errors 엔드포인트(bg.log 상태줄 제거 후 에러 키워드 강조), 시뮬레이션 카드에 에러 로그 패널 + 접힘 상태에서도 보이는 ⚠ 에러 배지, 5초 폴링 및 재시작 시 자동 갱신. | `363131c` |
| 2026-04-16 | ROS2 spin executor 충돌 수정: ur5e_rt_controller, object_approach_node 모두 rclpy.spin(node) → SingleThreadedExecutor 전용 인스턴스로 교체. 기본 executor 공유로 발생하던 ValueError: generator already executing 해결. | `363131c` |
| 2026-04-16 | scene_config.py box 크기 변경: 15cm → 3cm 정육면체 (half-size 0.15 → 0.015), z 위치도 0.15 → 0.015로 바닥 밀착 보정 | `363131c` |
| 2026-04-16 | 씬 물체 위치 발행 기능 추가: object_approach_node에 _find_target_body(로봇·바닥 제외 단일 물체 탐색), publish_scene_objects(/mujoco/scene_objects JSON 발행), /mujoco/query_objects 구독 콜백 구현. main.py에 /api/pub/object_position 엔드포인트, 웹 UI 핸드 카드에 "📍 물체 위치 발행" 버튼 추가. | `363131c` |
| 2026-04-16 | 웹 UI 소프트 관절 한계 카드를 노트 아래로 이동 및 기본 접힘 처리 | `363131c` |
| 2026-04-16 | 웹 UI 카드 순서 변경: 컨테이너/시뮬레이션/명령/스트리밍 카드를 맨 아래로 이동 및 기본 접힘 처리 | `363131c` |
| 2026-04-16 | launch.sh Tailscale Funnel --bg 플래그 추가로 백그라운드 실행 수정 | `363131c` |
| 2026-04-16 | 물체 접근 알고리즘 추가: object_approach_node.py 생성 (MuJoCo data.xpos로 물체 위치 읽기 → /ur5e/cmd/ee_target PoseStamped 발행, IDLE→HOVER→DESCEND→DONE 상태머신). main_test.py에 ObjectApproachNode 연동. | `363131c` |
| 2026-04-16 | main_test.py 시작 크래시 수정: AntiSlipReflex 생성자에 timeout_s=0.5 전달 시 **kwargs 경유로 super().__init__에 중복 전달 → TypeError. timeout_s 인자 제거 후 self._anti_slip.timeout_s = 0.5 로 직접 설정. | `363131c` |
| 2026-04-16 | 반사 기반 무게 추정 구현: robotiq_grasp_adapter에 _estimate_weight_raw() 추가 (접촉력 월드 프레임 수직 성분 합/g), 이동 평균 버퍼, μ_eff 추정, /tmp/grasp_state.json 10Hz 기록. main.py에 /api/grasp_state 추가. 웹UI 핸드 카드에 무게 표시 섹션 + 신뢰도 배지 + 0.5Hz 폴링. | `363131c` |
| 2026-04-16 | Anti-Slip Reflex On/Off 토글 추가: 웹UI 그리퍼 카드에 toggle 컨트롤 타입 구현, pub_hand API bool 지원 추가, ur5e_rt_controller에 antislip_enabled 플래그 + /ur5e/cmd/antislip ROS subscriber 추가, robotiq_grasp_adapter에 enabled 체크 및 IDLE 리셋 추가. | `363131c` |
| 2026-04-16 | Robotiq 2F-85 + UR5e용 GraspController 통합: robotiq_grasp_adapter.py 생성 (AntiSlipReflex 재사용, n_fingers=2 n_joints=1, MuJoCo 접촉 데이터 매핑), main_test.py에 RobotiqGraspAdapter 연결 (100Hz 백그라운드 스레드). | `363131c` |
| 2026-04-16 | Added docs/reflexive_control.md: 계층 구조·설계 철학, ContactState, Reflex 상태머신, AntiSlip/ReGrasp 수식, ReflexCoordinator 우선순위 중재, FourBarLinkage·FingerKinematics 기구학, GraspPlanner 이론 전반 문서화. | `363131c` |
| 2026-04-16 | MIT Reflexive Control 계층 추가: ContactState, ReflexState, Reflex 추상 클래스, AntiSlipReflex, ReGraspReflex, ReflexCoordinator, GraspPlanner을 grasp_controller.py에 통합. | `363131c` |
| 2026-04-15 | Created orca_hand/ folder with README.md, bom.csv, ORCA_v1.step (40MB), 5 STL ZIPs, 9 Bambu 3MF files collected from orcahand.com. | `8ce53ae` |
| 2026-04-15 | Cloned orcahand_description repo into orca_hand/; updated README with DOF detail, actuator ranges, v1/v2 version comparison. | `8ce53ae` |
| 2026-04-15 | 웹 UI에 핸드 제어 카드 추가. hand_ui_config.py로 핸드별 설정 외부화, /api/hand_config + /api/pub/hand 엔드포인트 추가. | `09d39f9` |
| 2026-04-15 | 핸드 제어 카드에 전송 버튼 추가. 슬라이더는 값 표시만, 전송 버튼으로 명시적 전송. 프리셋은 값 설정 후 즉시 전송. | `e2f21b8` |
| 2026-04-07 | Added ROS2 pub/sub interface to main_test.py for real-time UR5e command sending. | `92001ff` |
| 2026-04-08 | Moved AUTH_USERNAME/AUTH_PASSWORD from hardcoded values to .env file in web_dashboard. | `3a494c1` |
| 2026-04-08 | Fixed JS auth for API calls: added apiFetch() wrapper and replaced EventSource with fetch+ReadableStream for log streaming. | `3a494c1` |
| 2026-04-09 | Fixed cloudflared zombie process accumulation and added auto-start dashboard tunnel (port 8765) in app lifespan alongside meshcat tunnel. | `3a494c1` |
| 2026-04-09 | Fixed render_snapshot.py camera: added MjvCamera with lookat/distance/azimuth/elevation to prevent robot being clipped. | `3a494c1` |
| 2026-04-09 | Separated dashboard tunnel from app lifecycle: moved cloudflared(8765) to heroi-tunnel.service, dashboard to heroi-dashboard.service; removed reload=True. | `3a494c1` |
| 2026-04-09 | Added 3D turntable robot viewer: render_snapshot.py --frames mode (24 JPEG frames), /api/robot_3d endpoint, drag-to-rotate UI in dashboard. | `3a494c1` |
| 2026-04-09 | Extended 3D viewer to 2-axis rotation: azimuth×elevation grid (24×5=120 frames), horizontal drag=azimuth, vertical drag=elevation. | `3a494c1` |
| 2026-04-08 | Updated web_dashboard README with full API docs, code structure, and change history. | `3a494c1` |
| 2026-04-09 | Reduced 3D viewer frames from 120 to 60 by halving N_AZ (24→12, 30° steps); adjusted drag sensitivity /12→/24 to maintain same angular speed. | `5cba1fc` |
| 2026-04-13 | Fixed double login: removed HTTP Basic auth from GET / so browser native dialog no longer appears; custom HTML overlay is now the sole login UI; API endpoints retain server-side auth. | `5cba1fc` |
| 2026-04-13 | Added background run commands (nohup, log, pgrep, pkill) to web_dashboard README. | `5cba1fc` |
| 2026-04-13 | Added launch.sh: single script to start dashboard server + Cloudflare tunnel with status output. | `5cba1fc` |
| 2026-04-13 | Rewrote web_dashboard README: added launch.sh usage, address check commands, full API table, reorganized sections. | `5cba1fc` |
| 2026-04-13 | Rewrote root README.md: reflects full current state including web dashboard, 3D viewer, Cloudflare tunnel, and API table. | `5cba1fc` |
| 2026-04-13 | Updated README: added ROS2 interface table (joint/ee_target/mode sub, joint/ee_pose pub), task-space IK; corrected Next Milestones to exclude already-implemented features. | `5cba1fc` |
| 2026-04-13 | Updated launch.sh: added `stop` subcommand to kill all processes at once; added Meshcat local/tunnel URL display on startup; fixed Meshcat tunnel pkill scope. | `5cba1fc` |
| 2026-04-13 | Improved launch.sh Meshcat external URL display: separated into step 4/4 with 20s timeout, using sed instead of double-grep for reliability. | `5cba1fc` |
| 2026-04-13 | Moved Meshcat tunnel management from main.py to launch.sh; removed _start_meshcat_tunnel, lifespan, and /api/meshcat_url from main.py; launch.sh now directly starts cloudflared for port 7000 with dedicated log /tmp/heroi-meshcat-tunnel.log. | `5cba1fc` |
| 2026-04-13 | Removed separate Meshcat Cloudflare tunnel; Meshcat now accessed via dashboard proxy at /meshcat (wss:// rewriting already handled); launch.sh [4/4] now prints dashboard_url/meshcat directly. | `5cba1fc` |
| 2026-04-13 | Fixed Meshcat white screen: rewrote src="main.min.js" to absolute path /meshcat/main.min.js in meshcat_index() so browser loads JS through meshcat_static() handler which rewrites ws:// to wss://. | `5cba1fc` |
| 2026-04-13 | Fixed launch.sh pkill pattern: changed "python3 main.py" to "web_dashboard/main.py" to match actual process command line when launched via absolute path. | `5cba1fc` |
| 2026-04-13 | Clarified launch.sh stop message: simulation (main_test.py) runs inside Docker container and must be stopped separately via dashboard exec. | `5cba1fc` |
| 2026-04-13 | Fixed joint command reliability: changed `ros2 topic pub --once` to `--times 3` in /api/pub/joint to prevent message loss before subscriber discovery. | `5cba1fc` |
| 2026-04-13 | Added sim process monitor UI: duplicate warning banner + red border when count > 1, "종료 후 재시작" button; added /api/sim/restart backend endpoint. | `3fccc24` |
| 2026-04-13 | Refactored Docker volume mounts: split single `.:/ros2_ws/src/my_ur5e_controller` into two mounts (my_ur5e_controller, web_dashboard) so both sit at /ros2_ws/src/ level; updated render_snapshot.py paths and sim/restart path in main.py. | `3fccc24` |
| 2026-04-13 | Merged web_dashboard/README.md into root README.md (manual startup, .env setup, address check, sim API endpoints added); deleted web_dashboard/README.md. | `3fccc24` |
| 2026-04-13 | Added rebuild button: /api/rebuild streaming endpoint (docker compose down → up --build), rebuild output panel in UI with real-time log stream. | `3fccc24` |
| 2026-04-13 | Fixed sim kill: changed pkill to pkill -9 (SIGKILL) in sim/kill and sim/restart — main_test.py ignores SIGTERM so processes were not dying. | `3fccc24` |
| 2026-04-13 | Fixed sim status count: bash wrapper process (nohup bash -c '... python3 main_test.py') was also matched by grep; switched to awk field match ($3=="python3" && $4=="main_test.py") to count only actual python process. | `3fccc24` |
| 2026-04-13 | Added task-space control UI: mode toggle (joint/task), EE position (x,y,z), euler angle inputs with live quaternion preview, /api/pub/mode and /api/pub/ee endpoints. | `3fccc24` |
| 2026-04-13 | Merged joint-space and task-space control into single tabbed card; tab switch auto-publishes ROS2 mode change via switchCtrlTab(). | `3fccc24` |
| 2026-04-13 | Added cloudflared auto-restart loop in launch.sh: tunnel wrapped in `while true; do ... sleep 3; done` so it recovers automatically on disconnect. | `3fccc24` |
| 2026-04-13 | Added robot_state_publisher TF tree: created ur5e_minimal.urdf (kinematic-only, DH params), launched RSP subprocess in main_test.py, fixed joint names (joint_1..6 → shoulder_pan_joint etc.), added /joint_states publisher. | `3fccc24` |
| 2026-04-13 | Fixed broken joint command after JointTrajectory migration: updated /api/pub/joint to send trajectory_msgs/JointTrajectory with duration param; added duration input to joint-space UI tab. | `3fccc24` |
| 2026-04-13 | Added EE motion duration: /api/pub/ee accepts duration param, sends /ur5e/cmd/ee_duration before pose; controller computes max_ee_speed=dist/duration and applies step-clamping in _ik_step(); added duration input to task-space UI tab. | `3fccc24` |
| 2026-04-13 | Switched joint command interface from Float64MultiArray(/ur5e/cmd/joint_target) to JointTrajectory(/ur5e/cmd/joint_trajectory); added LERP trajectory interpolation in ur5e_rt_controller.py (set_trajectory, _traj_step); trajectory takes priority over task-space and joint-hold modes. | `3fccc24` |
| 2026-04-13 | Fixed Meshcat robot invisible: added mj_forward(model, data) after MjData creation so geom_xpos/geom_xmat are valid before first update_visualizer call (zero rotation matrix = degenerate Three.js transform = scale 0). | `3fccc24` |
| 2026-04-14 | Fixed Meshcat robot not visible after sim restart: reduced port cache TTL 30s→5s; on WebSocket connect failure invalidate cache and retry with freshly detected port. | `3fccc24` |
| 2026-04-14 | Removed duration=0 fast-path in _cb_ee_target: duration<=0 now auto-computes from distance (max 0.5 m/s, min 0.5s) so EE always moves with quintic profile regardless of input. | `3fccc24` |
| 2026-04-14 | Added vel_start/vel_end to EE command: encoded in header.frame_id ("base|v0|v1"), parsed in _cb_ee_target, normalized to v_norm=v*duration/dist; added _smooth_alpha_blend(tau,v0,v1) general quintic; updated /api/pub/ee and dashboard UI. | `3fccc24` |
| 2026-04-14 | Added kinematic limits to UR5eRTController (ee_max_vel, ee_max_acc, ee_max_jerk constructor params); _compute_min_duration() uses bisection over 201-point numerical profile to find minimum duration satisfying all constraints; _cb_ee_target enforces T >= T_min with log on extension. | `3fccc24` |
| 2026-04-14 | Implemented continuous blending in _cb_ee_target: when a new EE command arrives mid-motion, current profile velocity is computed (f'(τ)×Δpos/T), projected onto new direction, and used as vel_start for the new profile — ensures velocity-continuous handoff. | `3fccc24` |
| 2026-04-15 | Added horizontal scroll wrapper to joint state table for mobile; added always-visible collision reset button to robot control card header. | `8ce53ae` |
| 2026-04-14 | Fixed velocity spike at blend point: added ensure_decel flag to _compute_min_duration; when blending, enforces T ≥ 10·dist/(6·v0+4·v1) so v0_norm ≥ 5/3, guaranteeing f'''(0)≤0 (immediate deceleration, no initial speedup). | `3fccc24` |
| 2026-04-14 | Added self-collision detection: COLLISION_STOP state added to ControlState; after mj_step() checks data.contact for robot link pairs (both body_id>0) with dist<0; on detection freezes joints and prints collision body names; reset_collision() clears state and returns to INIT_POSITION. | `3fccc24` |
| 2026-04-14 | Added collision reset button: collision state written to robot_state.json; /ur5e/cmd/collision_reset ROS2 topic triggers reset_collision(); /api/sim/collision_reset backend endpoint; sticky red banner with body names and dist shown in dashboard when collision detected; "충돌 정지 해제" button calls the endpoint. | `3fccc24` |
| 2026-04-14 | Added joint limit visualization: jnt_range[:6] written to robot_state.json; dashboard joint table replaced with 4-column layout (name, current deg, visual bar, limit range); bar color green/yellow/red based on proximity to limits (<15%/<5% margin). | `3fccc24` |
| 2026-04-15 | Added "현재값 불러오기" buttons to joint-space and task-space control tabs; _lastRobotState cached in fetchRobotState(); loadJointFromState() fills sliders/inputs from q; loadEEFromState() fills ee-x/y/z and euler angles from ee_pos/ee_euler_deg and calls updateQuat(). | `3fccc24` |
| 2026-04-15 | Added soft joint limits: _update_soft_limits() computes soft_lo/hi from jnt_range×margin; _soft_limit_torque() adds repulsion (-kp×penetration) and velocity damping (-kd×dq toward limit); q_target clamped to soft zone before PD; /ur5e/cmd/soft_limits String topic updates params at runtime; /api/pub/soft_limits endpoint; dashboard settings card with enable toggle, margin slider, kp/kd inputs; UI synced from robot_state on first poll. | `3fccc24` |
| 2026-04-15 | Added docs/controller_theory.md: full theoretical documentation of the control loop organized by execution order — state machine, command priority, joint-space trajectory quintic interpolation, task-space DLS IK, motion profiles (standard/blend quintic, bisection, continuous blending, ensure-decel), PD+dynamics compensation, soft joint limits, collision detection. | `3fccc24` |
| 2026-04-15 | Removed robot 3D viewer: deleted /api/robot_image and /api/robot_3d endpoints from main.py; removed 로봇 3D 뷰어 card and all associated JS (fetch3D, show3DFrame, setup3DDrag) from index.html. | `3fccc24` |
| 2026-04-15 | Added scene.xml: wraps ur5e.xml via include and adds a static 30cm cube at x=0.3m from robot base; main_test.py now loads scene.xml instead of ur5e.xml directly; setup_meshcat_robot handles mjGEOM_BOX type; update_visualizer tracks box transform each frame. | `3fccc24` |
| 2026-04-15 | Fixed intermittent "서버 시작 실패" in launch.sh: replaced fixed sleep 0.5 with port-release polling (ss -tlnp, max 10s) so new uvicorn never starts while old process still holds port 8765. | `3fccc24` |
| 2026-04-15 | Fixed SyntaxError in main.py pub_soft_limits: f-string backslash in payload.replace() not allowed in Python<3.12; extracted escaped_payload variable before f-string. | `3fccc24` |
| 2026-04-15 | Fixed stale tunnel URL in launch.sh: clear TUNNEL_LOG before starting new cloudflared so grep never returns a previous session's URL; also changed head -1 to tail -1 to always get the latest URL. | `3fccc24` |
| 2026-04-15 | Fixed sim restart not running main_test.py: merged two exec_run calls into one detach=True call; replaced nohup bash -c wrapper with exec python3 so Docker exec instance IS the Python process (avoids orphan kill when outer bash exits). | `3fccc24` |
| 2026-04-15 | Added grasp_controller.py: GraspController with 4-bar linkage FK/IK (Freudenstein), planar 3R finger kinematics (FK + 6×3 Jacobian), state machine (WAIT_STABLE→RUN_CONTROL), 2ms RT control loop, read/write communication API. | `3fccc24` |
| 2026-04-15 | Fixed scene model loading: removed scene.xml (include from different dir broke meshdir='assets' resolution); added load_scene_model() in main_test.py that parses mujoco_menagerie/scene.xml, injects box body, writes temp XML alongside ur5e.xml so mesh paths resolve correctly, then cleans up. | `3fccc24` |
| 2026-04-15 | Added sensors: 6x actuatorfrc XML sensors injected in load_scene_model(); _read_joint_torques() reads sensordata by name with actuator_force fallback; _read_contacts() returns pos/normal/force_world/force_mag per contact; ddq(qacc[:6]) and tau/contacts added to robot_state.json. | `3fccc24` |
| 2026-04-15 | Updated README.md: reflected current architecture (sensor system, scene objects, safety features, updated API table, updated TODO). | `3fccc24` |
| 2026-04-15 | Added docs/grasp_controller_flow.md: grasp controller control flow diagram including state machine transitions, RUN_CONTROL loop steps, kinematics pipeline, threading/data flow, and timing summary. | `74cd735` |
| 2026-04-15 | Added docs/project_structure.md: project directory structure from Notion 제어기 설계 (2024-05-26) — comm, control/core/kinematics/linkage/math/util, core, drivers layers. | `74cd735` |
| 2026-04-15 | Extended collision detection to include robot-object collisions: added _build_robot_body_ids() to cache robot kinematic chain; _check_collision() now reports both self_collision (robot↔robot) and object_collision (robot↔external) types. | `74cd735` |
| 2026-04-15 | Fixed false-positive collision: _build_robot_body_ids() now stores _robot_base_body_id; _check_collision() skips only base_link↔floor(world body=0) contacts, not all world-body contacts. | `74cd735` |
| 2026-04-15 | Fixed collision state not written to file: moved _write_state() counter from controller_run() to main start() loop so state file updates in all states including COLLISION_STOP. | `74cd735` |
| 2026-04-15 | Added COLLISION_ESCAPE state: on collision reset, retreats to joint config from 0.1s before collision (50-step safe_q_buffer); PD-controls to that position until collision clears, then resumes RUN_CONTROL. | `74cd735` |
| 2026-04-15 | Merged 컨테이너 상태+도커 제어 cards into one; added toggleCard() collapse/expand to all cards (컨테이너, 시뮬레이션, 명령 실행, 로봇 제어, 소프트 관절 한계, 스트리밍, 로봇 현재 상태) with ▲/▼ chevron. | `8ce53ae` |
| 2026-04-15 | Reordered dashboard cards: 스트리밍 명령 moved below 명령 실행; 소프트 관절 한계 moved above 로봇 제어. | `8ce53ae` |
| 2026-04-15 | Attached Robotiq 2F-85 v4 gripper to UR5e: load_scene_model() rewritten with MjSpec.attach(site='attachment_site'); added _gripper_act_id, set_gripper(), _cb_gripper(/ur5e/cmd/gripper Float64 0~1), gripper state to robot_state.json. | `8ce53ae` |
| 2026-04-15 | Extracted gripper_config.py: GripperConfig dataclass + presets (ROBOTIQ_2F85_V4, ROBOTIQ_2F85, UMI_GRIPPER) + attach(); gripper swap = change ACTIVE in gripper_config.py only; main_test.py and ur5e_rt_controller.py untouched. | `8ce53ae` |
| 2026-04-15 | Extracted scene_config.py: Box/Sphere/Cylinder dataclasses + OBJECTS list + add_objects(); scene objects = edit OBJECTS in scene_config.py only; main_test.py untouched. | `8ce53ae` |
| 2026-04-16 | Fixed sim/restart: replaced `exec python3` with `nohup python3` so SIGHUP from Docker exec detach doesn't kill main_test.py. | `363131c` |
