# TODO

---

## [x] J6 축 90° 방향 수렴 정확도 개선 — **2026-05-20 완료 (허용 기준 달성)**

### 최종 파라미터 (ur5e_rt_controller.py)
```python
self._ik_rot_gain  = 5.0   # 2.0 → 5.0
self._ik_lambda_sq = 0.1
```

### 최종 수렴 결과
```
J6축 목표: [ 0.707, -0.707,  0.000]
J6축 실제: [ 0.667, -0.741, -0.080]  ← Z 성분 -0.080 (기준 ±0.08 이내)
rot_err:   0.096 rad  (rot_tol=0.10 통과)
관절각(deg): [20.7, -67.9, 135.0, -79.5, -21.8, -78.8]
```

### 시도한 파라미터 조합 요약
| 변경사항 | rot_err | J6_Z |
|---|---|---|
| rot_gain=2 (원래값) | 0.146 rad | -0.108 |
| rot_gain=5 ✓ | 0.087~0.096 rad | -0.072~-0.080 |
| rot_gain=10 | 0.097 rad | -0.081 |
| lambda=0.05 | 0.097 rad | -0.072 |
| lambda=0.01 | 수렴 실패 | — |

### 분석: 왜 J6_Z = 0 을 달성할 수 없는가
Jacobian DLS IK 는 위치(pos_gain=100)와 회전(rot_gain=5)을 동시에 최소화.
이 자세에서 J4/J5 조정이 필요한 회전 방향이 위치 자코비안과 커플링되어
위치 수렴 후 남는 회전 오차가 J6_Z ≈ -0.07 ~ -0.08 에서 안정 최솟값에 수렴.
정확한 J6_Z = 0 을 위해선 Analytical IK 또는 Task-Priority IK가 필요.

### 실용적 판단
4.6° 기울기는 실린더 파지에 충분히 작으므로 **허용 기준 ±0.08 이내**로 완료 처리.

---

## [x] 그리퍼 방향 변경: J6 축 수직 → 수평 (XY 평면 평행) — **2026-05-19 완료**

### 배경
재파지(regrasping) 실험을 위해 6번 관절(wrist_3) 회전축을 수직(Z 방향)에서
수평(XY 평면 내)으로 변경. 기존에는 위에서 수직 하강하는 접근이었으나,
측면에서 수평 접근하는 방식으로 전환.

### 변경 파일: `my_ur5e_controller/object_approach_node.py`

#### 1. `_make_approach_rot` — 접근 회전행렬 변경

```python
# 변경 전: wrist Y = [0,0,-1] (J6 축 = 수직, 위에서 하강)
tool_down = np.array([0., 0., -1.])
return np.column_stack([wrist_x, tool_down, wrist_z])

# 변경 후: wrist Y = base→obj 수평 방향 (J6 축 = XY 평면 내)
wrist_y = rel_xy / norm          # 물체 방향 수평 단위벡터
wrist_x = np.array([0., 0., -1.])   # wrist X = 하강 방향
wrist_z = np.cross(wrist_x, wrist_y)
return np.column_stack([wrist_x, wrist_y, wrist_z])
```

#### 2. HOVER / DESCEND 타겟 — 수직 상공 → 수평 측면으로 변경

```python
# 변경 전 (위에서 하강):
target = [obj_x - cx, obj_y - cy, top_z + hover_height + tcp_z_offset]

# 변경 후 (측면 수평 접근):
approach_dir = approach_rot[:, 1]          # wrist Y = 접근 방향
side_radius  = _get_object_side_radius()   # 물체 반지름
standoff     = side_radius + hover_height + tcp_z_offset
target = [obj_x - approach_dir[0]*standoff,
          obj_y + y_noise - approach_dir[1]*standoff,
          obj_z]   # 물체 중심 높이 유지
```

#### 3. `_get_object_side_radius` — 신규 헬퍼 메서드

```python
# 물체 geom 타입별 측면 반지름 반환 (cylinder→s[0], box→max(s[0],s[1]))
```

#### 4. PRE_APPROACH `q_pre` — wrist_2 초기 자세 변경

```python
# 변경 전: wrist_2 = -π/2  (도구 수직 하강 초기 자세)
# 변경 후: wrist_2 = 0.0   (J6 축 수평 초기 자세)
```

### 테스트 결과 (2026-05-19)

```
cylinder (body_id=19) 접근:
  PRE_APPROACH: J1 -90° → 29.8°
  HOVER 완료:   pos_err=0.0007m, rot_err=0.124rad (<0.15) ✓
    관절각(deg): [29.0, -63.7, 125.1, -63.6, 67.6, -87.3]
    J6축→world 실제: [-0.622, -0.782, 0.036]   ← z≈0, XY 평면 ✓
    J6축→world 목표: [-0.707, -0.707,  0.0  ]
  DESCEND → DONE: err=0.0095m ✓
```

---

## [x] HOVER IK 수렴 문제 해결 — **2026-05-19 완료**

### 최종 해결책 (적용된 파라미터)

```python
# ur5e_rt_controller.py
self._ik_pos_gain  = 100.0
self._ik_rot_gain  = 2.0    # 위치+회전 동시 수렴
self._ik_lambda_sq = 0.1    # 10배 올려 ill-conditioned 방지
self._ik_max_dq    = 0.1
self._q_null_gain_per_joint = [0, 0, 0, 0, 0, 0]   # null-space 제거

# 중력 feed-forward (ctrl에 추가):
self.data.ctrl[:6] = q_des + self.data.qfrc_bias[:6] / 2000.0

# dq 정규화 (clip 대신 방향 보존):
max_abs = np.max(np.abs(dq))
if max_abs > self._ik_max_dq:
    dq = dq * (self._ik_max_dq / max_abs)
```

```python
# main_test.py - hover_height 줄임 (workspace 범위 내)
approach_node = ObjectApproachNode(model, data, hover_height=0.05)
```

### 수렴 결과

PRE_APPROACH(T+6s) → HOVER err=0.030(T+9s) → HOVER err=0.002(T+12s) → DESCEND(T+18s) → **DONE(T+21s)** ✓

### 근본 원인 요약

1. **작업공간 초과**: hover_height=0.15 시 wrist 목표 0.831m > UR5e 최대 도달 0.817m → hover_height=0.05로 변경
2. **null-space 충돌**: W[1]=5가 IK 위치 기울기와 정확히 상쇄 → W[1]=0으로 제거
3. **중력 평형**: pos_gain≤50 시 dq=gravity_offset → ctrl에 qfrc_bias/2000 FF 추가
4. **ill-conditioned**: lambda=1e-2로 dq_raw 폭발 → lambda=0.1로 증가
5. **clip→normalization**: element-wise clip 시 방향 왜곡 → 전체 벡터 정규화

---

## [x] IndexError 수정 — robotiq_grasp_adapter.py 레이스 컨디션

### 완료 내용
`_read_contact_state`, `_read_contact_state_regrasp`, `_estimate_weight_raw` 3개 메서드에서
`data.ncon` / `data.contact` 동시 접근으로 인한 IndexError 수정.

**수정 패턴:**
```python
ncon = int(self._data.ncon)
contacts = self._data.contact
n = min(ncon, len(contacts))
for j in range(n):
    c = contacts[j]
    ...
```

---

## [x] 외부 물체 충돌 회피 확장 — ur5e_rt_controller.py

### 완료 내용
`_repulsion_torque()`에서 self-collision 외에 **로봇팔 ↔ 외부 물체** 충돌도 방지하도록 확장.
- 그리퍼 body (`gripper/` prefix) 와의 충돌은 제외 (그리퍼가 물체를 잡아야 하므로)
- 바닥-베이스 간 충돌은 제외
- `_build_gripper_body_ids()` 메서드 추가

### 파라미터 튜닝 필요 (미완료)
- [ ] 접근 동작 중 진동 여부 확인 → 진동 시 kp 낮추기 (500 → 200)
- [ ] 좁은 공간 이동 시 반발 토크가 충분히 강한지 확인 → 부족하면 margin 늘리기 (0.02 → 0.03~0.05)

---

## [x] PRE_APPROACH Dead Zone 수정 + Pre-solve IK 교체 — **2026-05-24 완료**

### 원인
PRE_APPROACH 타겟 계산식 `obj_pos - approach_dir × (0.30 + tcp_z_offset)` = [0.01, 0.01, 0.60]이
UR5e workspace 내부 dead zone(최소 수평도달거리 ~0.13m 이내)에 위치해 IK 수렴 불가.

### 수정 내용

**`object_approach_node.py` — PRE_APPROACH 스탠드오프 단축**
```python
# 변경 전: (0.30 + tcp_z_offset) → 타겟 [0.01, 0.01, 0.60] (dead zone, 도달 불가)
# 변경 후: (0.10 + tcp_z_offset) → 타겟 [-0.175, 0.089, 0.60] (도달 가능, hor_dist=0.189m)
target = np.array([
    obj_pos[0] - approach_dir[0] * (0.10 + self._tcp_z_offset),
    obj_pos[1] + y_noise - approach_dir[1] * (0.10 + self._tcp_z_offset),
    obj_pos[2] + 0.30,
])
```

**`ur5e_rt_controller.py` — velocity IK pre-solve → scipy L-BFGS-B로 교체**
- 기존: mink velocity IK 300회 반복 → 로컬 미니멈에 빠져 항상 실패 (`dist ≈ 0.14m 유지`)
- 신규: scipy L-BFGS-B (home 출발 → 필요 시 20회 random restart)
- home 기준으로 관절각 정규화(shortest path 보장)

**`_ik_step()` Case 1/2 — pos_ref(trajectory waypoint) 비교 → pos_cur 비교로 수정**
```python
# Case 1: 현재 EE가 pre-solve 타겟에서 5cm 초과 → joint 명령 적용
dist_to_presolve = np.linalg.norm(self._presolve_pos_tgt - pos_cur)
if presolve_q is not None and dist_to_presolve > 0.05:
    self.motor_cmd["q_target"] = presolve_q.copy(); return
# Case 2: pre-solve 진행 중 → hold
if self._presolve_pending and dist_to_presolve > 0.10: return
```

### 검증된 동작 (2026-05-24 실행 결과)
```
PRE_APPROACH 명령 발행 → pos=[-0.175, 0.089, 0.60]
[IK-PreSolve] 수렴 (scipy, max_change ≈ 51°)
PRE_APPROACH 완료 → HOVER 명령 발행 → pos=[-0.175, 0.089, 0.30]
[IK-PreSolve] 수렴 (HOVER용)
```

---

## [x] HOVER → DESCEND → DONE 완료 검증 — **2026-05-24 완료**

### 최종 실행 결과
```
PRE_APPROACH → HOVER (err=0.0090m) → DESCEND (err=0.0085m) → DONE ✓
충돌 감지 없음, RUN_CONTROL 상태 유지
```

### 완료 조건
- 신규 `ur5e_controller.py` (mink 기반) 전 단계 수렴 확인
- `forearm_link ↔ gripper/right_coupler` 오탐 버그 수정으로 COLLISION_STOP 해제
- PRE_APPROACH 90° 회전 적용 후에도 전 단계 정상 동작

---

## [x] PRE_APPROACH EE 90° 회전 + 충돌 오탐 수정 — **2026-05-24 완료**

### 작업 내용

#### 1. `ur5e_controller.py` — 회전 유틸리티 추가
- 모듈 레벨 `rotmat_axis_angle(axis, angle_deg)` — Rodrigues 공식
- `UR5eController.rotate_ee_target(axis, angle_deg, frame)` — EE 타겟에 추가 회전 적용
  - `axis`: `'x'`/`'y'`/`'z'` (문자열) 또는 `np.ndarray(3,)`, `frame`: `'ee'`|`'world'`

#### 2. `object_approach_node.py` — PRE_APPROACH J6 회전 적용
- `_safe_z_rot_deg()` 메서드: `wrist_3` 조인트 range를 모델에서 직접 읽어 ±90° 방향 결정
- `_cb_start()` 수정: `approach_rot` 계산 직후 J6축(`col1`, wrist_y)으로 90° 추가 회전 적용
  - `col1`(approach_dir) 기준으로 회전 → approach_dir 유지, 위치 타겟 계산 정상
- 로그에 `EE-z 회전=+90°` 또는 `-90°` 방향 출력

#### 3. `ur5e_controller.py` — 충돌 감지 오탐 수정
- `_build_robot_body_ids()` 리팩터링: arm 체인 + 그리퍼 body를 분리 식별
  - `_robot_body_ids`: arm chain (wrist_3 → base)
  - `_gripper_body_ids`: 이름 패턴 (`gripper`, `coupler`, `finger`, `pad` 등) 매칭
  - `_all_robot_ids = arm_ids | gripper_ids`
- `_check_collision()` 수정: 두 body 모두 `_all_robot_ids`에 속하면 내부 접촉으로 무시
  - `forearm_link ↔ gripper/right_coupler` (dist=-0.00071m) 오탐 제거

#### 4. `Dockerfile` + `start.sh` — mink 의존성 영구 등록
- Dockerfile `pip3 install`에 `"mink[daqp]"` 추가
- `start.sh`에 시뮬레이션 시작 전 mink 자동 설치/확인 단계 추가 (컨테이너 재시작 대응)

### 최종 검증 (2026-05-24)
```
EE-z 회전=+90°  approach_rot col1(J6축)=[-0.928, 0.371, 0.0]
PRE_APPROACH → HOVER → DESCEND → DONE ✓  (충돌 없음)
```

---

## [ ] TCP z-offset + XY 보정 검증

### 완료 내용
- `_compute_tcp_z_offset()` → `_compute_tcp_offsets()`로 교체
- TCP z-offset: pad geom을 `attachment_site` z-axis로 투영 → **25.0cm** (물리적으로 정확)
- TCP XY 보정: attachment_site ↔ wrist_3_link origin 간 offset 계산 → `dx=0.0cm, dy=-10.0cm`
- HOVER / DESCEND 타겟에 XY 보정 적용 중

### 현재 이슈: 좌표계 불일치
XY 보정 적용 후 팔이 Y 방향으로 이동했지만, 사용자 관점에서는 X축 방향 오차로 인식됨.
→ 월드 좌표계와 로봇 베이스 좌표계 간 매핑이 명확하지 않아 보정 방향이 틀릴 수 있음.

### 체크리스트
- [ ] 베이스 기준 월드↔로봇 좌표계 매핑 출력 및 정렬
  - `data.xmat[base_link_id]` 출력으로 world X/Y/Z가 로봇 어느 축인지 확인
  - `attachment_site` xmat, xpos 출력으로 실제 tool 방향 확인
- [ ] XY 보정 방향 재검증 후 수식 수정
- [ ] 시각 확인: HOVER 시 손끝이 물체 상단 기준 hover_height(15cm) 위에 정지
- [ ] 시각 확인: DESCEND 후 손끝이 물체 상단 기준 grasp_offset(2cm) 위에 정지

---

## [ ] Self-Collision Avoidance 파라미터 튜닝

### 배경
반발 토크 방식(Method A)이 `ur5e_rt_controller.py`에 구현됨.
- `self_collision_kp = 500` N/m
- `self_collision_kd = 50` N·s/m
- `_self_col_margin = 0.02` m (2cm 감지 거리)

### 체크리스트
- [ ] 접근 동작 중 진동 여부 확인 → 진동 시 kp 낮추기 (200 시도)
- [ ] 좁은 공간 이동 시 반발 토크가 충분히 강한지 확인 → 부족하면 margin 늘리기 (0.03~0.05)
- [ ] COLLISION_STOP 없이 스스로 회피하는지 테스트

---

## [ ] 그리퍼 제어 지연 개선 — 경량 HTTP pub 서버 추가

### 배경
슬라이더 → 그리퍼 동작까지 400ms~1초 지연 발생.
원인: `/api/pub/hand` 호출 시마다 Docker exec + `ros2 topic pub` 새 프로세스 기동 비용.

### 구현 계획 (3개 파일, docker-compose 변경 없음)

`network_mode: host`이므로 포트 노출 설정 불필요.

---

#### 1. 신규: `my_ur5e_controller/ros_pub_server.py`

`PUB_CONFIG` 리스트에 항목 추가/제거만으로 엔드포인트 자동 등록.

```python
PORT = 7200

PUB_CONFIG = [
    # (name,       topic,                    ros_pkg,        msg_class, field)
    ("gripper",  "/ur5e/cmd/gripper",  "std_msgs.msg", "Float64", "data"),
    ("antislip", "/ur5e/cmd/antislip", "std_msgs.msg", "Bool",    "data"),
    # ("regrasp",  "/ur5e/cmd/regrasp",  "std_msgs.msg", "Bool",    "data"),
]

def start(node) -> HTTPServer:
    # PUB_CONFIG 기반으로 publisher 초기화 후 포트 7200에서 HTTP 서버 백그라운드 기동
    # POST /pub/<name>  body: {"value": <float|bool>}
```

---

#### 2. 수정: `my_ur5e_controller/main_test.py`

`controller.start_ros_node()` 호출 직후에 추가:

```python
import ros_pub_server
ros_pub_server.start(controller._ros_node)
```

---

#### 3. 수정: `web_dashboard/main.py`

`_FAST_PUB_TOPIC_MAP`에 항목 추가/제거로 빠른 경로 대상 토픽 관리.
실패 시 자동으로 기존 `ros2 topic pub` 경로로 폴백.

```python
_FAST_PUB_PORT = 7200

# ros_pub_server.py 의 PUB_CONFIG와 동기화
_FAST_PUB_TOPIC_MAP: dict[str, str] = {
    "/ur5e/cmd/gripper":  "gripper",
    "/ur5e/cmd/antislip": "antislip",
    # "/ur5e/cmd/regrasp":  "regrasp",
}

async def _try_fast_pub(name: str, value) -> bool:
    # http://localhost:7200/pub/<name> 호출, 실패 시 False 반환

# pub_hand() 내부:
#   fast_name = _FAST_PUB_TOPIC_MAP.get(topic)
#   if fast_name and await _try_fast_pub(fast_name, typed_value):
#       return {"ok": True, "output": ""}
#   # 폴백: 기존 ros2 topic pub 로직
```

---

### 추가/제거 방법 요약

빠른 경로에 토픽 추가:
1. `ros_pub_server.py` `PUB_CONFIG`에 튜플 1줄 추가
2. `main.py` `_FAST_PUB_TOPIC_MAP`에 항목 1줄 추가

빠른 경로에서 토픽 제거:
1. 위 두 파일에서 해당 줄 삭제 (또는 주석 처리)
