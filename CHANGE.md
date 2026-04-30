# Change Log

| Date | Description | Rollback Commit |
|------|-------------|-----------------|
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
