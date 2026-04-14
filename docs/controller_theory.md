# UR5e RT Controller — 제어 이론 정리

`ur5e_rt_controller.py` 에 구현된 알고리즘을 **제어 루프 실행 순서**에 따라 설명한다.

---

## 목차

1. [시스템 아키텍처](#1-시스템-아키텍처)
2. [상태 머신](#2-상태-머신)
3. [500 Hz 루프 전체 흐름](#3-500-hz-루프-전체-흐름)
4. [명령 우선순위](#4-명령-우선순위)
5. [관절 공간 궤적 추종](#5-관절-공간-궤적-추종)
6. [태스크 공간 IK 제어](#6-태스크-공간-ik-제어)
7. [모션 프로파일 (Quintic Polynomial)](#7-모션-프로파일-quintic-polynomial)
8. [PD 제어 + 동역학 보상](#8-pd-제어--동역학-보상)
9. [소프트 관절 한계](#9-소프트-관절-한계)
10. [충돌 감지](#10-충돌-감지)

---

## 1. 시스템 아키텍처

```
외부 명령 (ROS2 토픽)
  /ur5e/cmd/joint_trajectory  ─────┐
  /ur5e/cmd/ee_target         ─────┤
  /ur5e/cmd/mode              ─────┤
  /ur5e/cmd/soft_limits       ─────┤──▶  UR5eRTController
  /ur5e/cmd/collision_reset   ─────┘         │
                                             │ 500 Hz
                                             ▼
                                    ┌─────────────────┐
                                    │  제어 루프       │
                                    │  (ctrl_thread)   │
                                    └────────┬────────┘
                                             │ data.ctrl[:6]
                                             ▼
                                    MuJoCo mj_step()
                                    (물리 시뮬레이션)
```

**스레드 구성**

| 스레드 | 주기 | 역할 |
|--------|------|------|
| `ctrl_thread` | 500 Hz (2 ms) | 제어 계산 + `mj_step()` |
| `ros_spin_thread` | 이벤트 기반 | ROS2 콜백 처리 |
| 메인 스레드 | 50 Hz (20 ms) | Meshcat 시각화 + 상태 모니터링 |

---

## 2. 상태 머신

초기화 시퀀스를 보장하기 위한 6단계 상태 머신.

```
WAIT_STABLE ──(1초 대기)──▶ INIT_CONTROL
                                 │
                        q_target ← 현재 qpos
                                 │
                                 ▼
                           CHECK_MOTOR ──▶ INIT_POSITION
                                               │
                                    pos_target ← 현재 EE 위치
                                               │
                                               ▼
                                         RUN_CONTROL ◀──────────┐
                                               │                │
                                    충돌 감지 시│                │reset_collision()
                                               ▼                │
                                        COLLISION_STOP ─────────┘
```

| 상태 | 동작 |
|------|------|
| `WAIT_STABLE` | 물리 엔진 안정화 대기 (1초) |
| `INIT_CONTROL` | 현재 qpos를 q_target으로 설정 (정지 유지) |
| `CHECK_MOTOR` | 모터 점검 (현재는 통과) |
| `INIT_POSITION` | 현재 EE 위치를 task_cmd 초기값으로 설정 |
| `RUN_CONTROL` | 정상 제어 루프 실행 |
| `COLLISION_STOP` | 충돌 감지 후 관절 동결, 외부 해제 대기 |

---

## 3. 500 Hz 루프 전체 흐름

`RUN_CONTROL` 상태에서 매 2 ms 마다 실행되는 전체 흐름:

```
┌─────────────────────────────────────────────────────────────┐
│                    controller_run()                          │
│                                                             │
│  1. 상태 읽기                                               │
│     q  ← data.qpos[:6]                                      │
│     dq ← data.qvel[:6]                                      │
│                                                             │
│  2. 목표 관절각 결정 (우선순위 순)                           │
│     if traj_active  → _traj_step()      (trajectory)        │
│     elif task_space → _ik_step()        (IK)                │
│     else            → q_target 유지     (hold)              │
│                                                             │
│  3. q_target 클램프                                          │
│     if soft_limit_enabled:                                  │
│         q_des ← clip(q_target, soft_lo, soft_hi)           │
│                                                             │
│  4. PD 토크 계산                                            │
│     τ_pd = Kp·(q_des - q) + Kd·(0 - dq)                   │
│                                                             │
│  5. 동역학 보상                                              │
│     τ_bias = data.qfrc_bias[:6]   (중력+코리올리+원심력)    │
│                                                             │
│  6. 소프트 리밋 반발 토크                                    │
│     if soft_limit_enabled:                                  │
│         τ_soft ← _soft_limit_torque(q, dq)                 │
│                                                             │
│  7. 최종 토크 출력                                           │
│     data.ctrl[:6] = τ_pd + τ_bias + τ_soft                 │
│                                                             │
└─────────────────────────────────────────────────────────────┘
         │
         ▼
    mj_step()          ← 물리 적분 (1 step = 2 ms)
         │
         ▼
    _check_collision()  ← 접촉 감지
         │
    충돌 시 → COLLISION_STOP
```

---

## 4. 명령 우선순위

세 가지 제어 소스가 존재하며, 높은 순서가 낮은 순서를 덮어쓴다.

```
우선순위 1 (최고): Trajectory       ← /ur5e/cmd/joint_trajectory
우선순위 2:        Task-space IK    ← /ur5e/cmd/ee_target
우선순위 3 (최저): Joint hold       ← 현재 q_target 유지
```

Trajectory가 끝나면 자동으로 hold 상태로 복귀.  
`/ur5e/cmd/mode` 토픽이 `false` 이면 task-space 비활성화.

---

## 5. 관절 공간 궤적 추종

### 5.1 웨이포인트 보간

`JointTrajectory` 메시지로 `[(t₀, q₀), (t₁, q₁), …, (tₙ, qₙ)]` 형태의 시간-관절각 쌍을 수신.  
단일 웨이포인트가 입력되면 현재 위치를 `t=0` 에 자동 삽입한다.

```
q(t):
     q₁ ─────●
             / ← quintic 보간
    q₀ ─●──/
         t₀  t₁
```

인접 두 웨이포인트 사이에서 **Quintic polynomial** (→ 7절 참조)로 보간:

```
α = smooth_alpha((t - t₀) / (t₁ - t₀))
q_target = q₀ + α · (q₁ - q₀)
```

---

## 6. 태스크 공간 IK 제어

### 6.1 전체 흐름

```
EE 목표 (pos_ref, rot_ref)          ← 7절 quintic 프로파일에서 계산된 중간 목표
         │
         ▼
    오차 계산
    e_pos = pos_ref - pos_cur        (3×1 위치 오차)
    e_rot = rot_error(R_ref, R_cur)  (3×1 회전 오차)
         │
         ▼
    작업 공간 속도 명령
    dx = [Kp_pos·e_pos ; Kp_rot·e_rot]   (6×1)
         │
         ▼
    Jacobian 계산 (mujoco mj_jacBody)
    J ∈ ℝ^{6×6}
         │
         ▼
    Damped Least Squares IK
    dq = Jᵀ (J Jᵀ + λ²I)⁻¹ dx
         │
         ▼
    q_target ← q_cur + clip(dq, -Δq_max, Δq_max)
```

### 6.2 Jacobian

MuJoCo의 `mj_jacBody()`로 EE body의 선속도·각속도 Jacobian을 계산:

```
J = [Jv]  ∈ ℝ^{6×6}
    [Jω]

Jv ∈ ℝ^{3×6}  : 선속도 (translational) Jacobian
Jω ∈ ℝ^{3×6}  : 각속도 (rotational) Jacobian
```

### 6.3 Damped Least Squares (DLS) IK

순수 Jacobian 역행렬(pseudoinverse)은 특이점(singularity) 근방에서 발산하므로,  
**Damped Least Squares** 방법으로 안정화한다.

최적화 문제:

```
최소화:  ‖J·dq - dx‖² + λ²‖dq‖²
```

해:

```
dq = Jᵀ (J Jᵀ + λ²I)⁻¹ · dx
```

| 파라미터 | 값 | 의미 |
|----------|-----|------|
| `λ²` | `1e-4` | 감쇠 계수 — 클수록 특이점 안전, 작을수록 정확도 향상 |
| `Kp_pos` | `5.0` | 위치 비례 게인 |
| `Kp_rot` | `2.0` | 자세 비례 게인 |
| `Δq_max` | `0.1 rad` | 한 스텝 최대 관절 이동량 |

`λ² = 0` 이면 Moore-Penrose pseudoinverse와 동일.

### 6.4 회전 오차

두 회전 행렬 `R_des`, `R_cur` 사이의 오차를 **SO(3) 상의 축-각도**로 계산:

```
R_err = R_des · R_curᵀ

e_rot = 0.5 · [ R_err[2,1] - R_err[1,2] ]   ← (SO3 → 축-각도 근사)
               [ R_err[0,2] - R_err[2,0] ]
               [ R_err[1,0] - R_err[0,1] ]
```

이는 `R_err = I + [e_rot]×` 선형화 가정 하에서 성립하며,  
오차가 작을 때 (‖e_rot‖ ≪ 1) 정확도가 높다.

### 6.5 회전 보간 (SLERP 근사)

IK 프로파일 실행 중 중간 목표 자세는 선형 보간 후 SVD 정규화로 계산:

```
R_ref = R_start + α · (R_end - R_start)    ← 선형 보간
U, _, Vᵀ = SVD(R_ref)
R_ref = U · Vᵀ                              ← SO(3) 투영 (정규화)
```

정확한 SLERP는 쿼터니언 지수 사상이 필요하지만,  
작은 각도 변화 구간에서는 이 SVD 재정규화 방법이 충분히 정확하다.

---

## 7. 모션 프로파일 (Quintic Polynomial)

EE 명령이 수신될 때, 직접 목표로 점프하지 않고 시간 `T` 에 걸쳐  
**5차 다항식 프로파일**로 부드럽게 이동시킨다.

### 7.1 표준 Quintic (출발·도착 속도 = 0)

정규화 시간 `τ = t / T ∈ [0, 1]` 에 대해:

```
s(τ) = 10τ³ - 15τ⁴ + 6τ⁵

경계 조건:
  s(0) = 0,    s(1) = 1       ← 시작·끝 위치
  s'(0) = 0,   s'(1) = 0      ← 시작·끝 속도 = 0
  s''(0) = 0,  s''(1) = 0     ← 시작·끝 가속도 = 0
```

순간 속도 (정규화):

```
s'(τ) = 30τ²(1-τ)²     최대값 1.875 at τ = 0.5
```

실제 EE 위치:

```
p(t) = p_start + s(τ) · (p_end - p_start)
```

### 7.2 일반 Quintic (경계 속도 지정)

출발 속도 `v₀ ≠ 0` 또는 도착 속도 `v₁ ≠ 0` 인 경우 (연속 블렌딩):

```
s(τ) = c₁τ + c₃τ³ + c₄τ⁴ + c₅τ⁵

계수:
  c₁ = v₀
  c₃ = 10 - 6v₀ - 4v₁
  c₄ = -15 + 8v₀ + 7v₁
  c₅ = 6 - 3v₀ - 3v₁

경계 조건:
  s(0) = 0,    s(1) = 1
  s'(0) = v₀,  s'(1) = v₁    ← 정규화 속도
  s''(0) = 0,  s''(1) = 0
```

속도 정규화:

```
v_norm = v_actual (m/s) × T (s) / dist (m)
```

`v₀ = v₁ = 0` 이면 표준 quintic과 동일.

### 7.3 최소 duration 이진탐색

사용자가 요청한 `T` 가 물리 한계를 위반하면 자동으로 연장한다.

정규화 도함수와 실제 물리량 간 관계:

```
vel_actual  = s'(τ)    × dist / T
acc_actual  = s''(τ)   × dist / T²
jerk_actual = s'''(τ)  × dist / T³
```

`T` 를 키우면 세 물리량이 단조 감소 → 이진탐색이 성립.

**해석적 하한 (표준 quintic 기준):**

```
T_lo = max(
    1.875 × dist / v_max,            ← 속도 한계
    √(5.774 × dist / a_max),         ← 가속도 한계
    ∛(60.0  × dist / j_max)          ← 저크 한계
)
```

| 파라미터 | 기본값 |
|----------|--------|
| `v_max` | 1.0 m/s |
| `a_max` | 5.0 m/s² |
| `j_max` | 50.0 m/s³ |

이진탐색 25회 수행 → 오차 `< T_lo / 2²⁵ ≈ 0` (무시 가능).

### 7.4 연속 블렌딩

이동 도중 새 EE 명령이 수신될 때 속도 불연속 없이 전환하는 메커니즘.

```
① 현재 프로파일의 정규화 속도 미분 계산
   f'(τ_now) = c₁ + 3c₃τ² + 4c₄τ³ + 5c₅τ⁴

② 실제 EE 속도 벡터 (m/s)
   v_cur = f'(τ_now) × ΔP_old / T_old

③ 새 이동 방향으로 투영
   v₀_new = max(0, v_cur · n̂_new)     n̂_new = (p_new - p_cur) / dist_new

④ v₀_new 를 새 프로파일의 출발 속도로 설정
```

역방향 성분은 0으로 클램프하여 역주행 방지.

### 7.5 즉시 감속 조건 (ensure_decel)

블렌딩 시 새 프로파일의 **초기 가속(속도 증가)** 을 방지하는 조건.

프로파일 초기 저크:

```
s'''(0) = 6c₃ = 6(10 - 6v₀ - 4v₁)
```

`s'''(0) ≤ 0` (즉시 감속) 조건:

```
v₀_norm ≥ (10 - 4v₁_norm) / 6

정규화 속도를 대입하면:

T ≥ 10 × dist / (6 × v₀_ms + 4 × v₁_ms)
```

`_compute_min_duration()` 의 `ensure_decel=True` 플래그가 이 하한을 추가한다.

---

## 8. PD 제어 + 동역학 보상

### 8.1 PD 제어

```
τ_pd = Kp · (q_des - q) + Kd · (0 - dq)
```

| 파라미터 | 값 | 단위 |
|----------|-----|------|
| `Kp` | 500 | N·m/rad |
| `Kd` | 50  | N·m·s/rad |

`dq` 의 목표값을 0으로 설정 → **속도 제동** 역할.

### 8.2 동역학 보상 (Computed Torque 부분)

로봇 팔 운동 방정식:

```
M(q)·q̈ + C(q,q̇)·q̇ + g(q) = τ
```

중력·코리올리·원심력 항을 사전 보상하여 PD 게인 부담을 줄인다:

```
τ_bias = qfrc_bias[:6]   ← MuJoCo가 매 step 자동 계산
       = C(q,q̇)·q̇ + g(q)
```

MuJoCo 내부적으로 **Recursive Newton-Euler Algorithm (RNEA)** 로 O(n) 계산.

### 8.3 최종 토크

```
τ_out = τ_pd + τ_bias + τ_soft

      = Kp·(q_des - q) + Kd·(0 - dq)    ← PD 오차 제어
      + C(q,q̇)·q̇ + g(q)                ← 동역학 보상
      + τ_soft                            ← 소프트 리밋 (9절)
```

`τ_bias` 보상 없이 PD만 쓰면 중력이 정상상태 오차를 만든다:

```
보상 없음:  Kp·e_ss = g(q)  →  e_ss = g(q) / Kp  (0이 아님)
보상 있음:  PD 는 오차만 제어,  e_ss ≈ 0
```

---

## 9. 소프트 관절 한계

하드 리밋(물리적 스토퍼) 도달 전 미리 속도를 감속시키는 층.

### 9.1 소프트 한계 경계 계산

```
span_i = q_hi_i - q_lo_i                 ← 관절 i 의 전체 범위
margin_i = span_i × margin_ratio          ← margin_ratio ∈ [0, 0.4]

soft_lo_i = q_lo_i + margin_i
soft_hi_i = q_hi_i - margin_i
```

기본값 `margin_ratio = 0.05` (양쪽 5%씩):

```
shoulder_pan (±360°, range=720°) → soft zone 시작: ±342°
elbow        (±180°, range=360°) → soft zone 시작: ±171°
```

### 9.2 q_target 클램프

PD 컨트롤러가 소프트 한계 바깥을 목표로 삼지 않도록 선제 차단:

```
q_des = clip(q_target, soft_lo, soft_hi)
```

IK 또는 trajectory가 한계 밖의 목표를 계산하더라도 이 단계에서 억제된다.

### 9.3 반발 토크

소프트 한계 내부에 진입했을 때 추가 토크 적용:

```
상한 초과 시 (q > soft_hi):
  τ_soft = -Kp_soft × (q - soft_hi)              ← 반발 강성
           -Kd_soft × dq  (dq > 0 인 경우만)      ← 접근 방향 감쇠

하한 미달 시 (q < soft_lo):
  τ_soft = -Kp_soft × (q - soft_lo)
           -Kd_soft × dq  (dq < 0 인 경우만)
```

`dq` 감쇠는 **한계 방향으로 움직이는 경우에만** 적용한다.  
반대 방향(한계에서 멀어지는 방향)의 감쇠는 없어 복귀 운동을 방해하지 않는다.

| 파라미터 | 기본값 | 단위 |
|----------|--------|------|
| `Kp_soft` | 200 | N·m/rad |
| `Kd_soft` | 20  | N·m·s/rad |

### 9.4 전체 소프트 리밋 동작

```
hard limit:  |────────────────────────────|
soft limit:  |──●────────────────────●──|
                 ↑ soft_lo            ↑ soft_hi

관절이 soft_hi 진입 시:
  q_des 클램프 → PD 목표가 soft_hi 로 고정
  τ_soft 반발  → 추가 밀어내는 힘 + 감속
```

---

## 10. 충돌 감지

### 10.1 판정 기준

`mj_step()` 직후 MuJoCo의 `data.contact` 배열을 순회:

```
for i in range(data.ncon):
    contact = data.contact[i]
    body1 = model.geom_bodyid[contact.geom1]
    body2 = model.geom_bodyid[contact.geom2]

    if body1 > 0 and body2 > 0 and contact.dist < 0:
        → 자기 충돌 감지
```

조건 해석:

| 조건 | 의미 |
|------|------|
| `body1 > 0` | world body(바닥)가 아닌 로봇 링크 |
| `body2 > 0` | world body(바닥)가 아닌 로봇 링크 |
| `dist < 0`  | 실제 침투 발생 (단순 근접은 제외) |

mujoco_menagerie UR5e XML에는 인접 링크 간 contact exclusion이 미리 설정되어  
보고되는 접촉은 **비인접 링크 간 자기 충돌**만 해당한다.

### 10.2 충돌 후 동작

```
충돌 감지
    │
    ▼
state → COLLISION_STOP
_freeze_q ← motor_state["q"]   ← 감지 시점의 관절각 저장

COLLISION_STOP 루프:
    q_cur  ← data.qpos[:6]
    dq_cur ← data.qvel[:6]
    data.ctrl[:6] = Kp·(_freeze_q - q_cur)    ← 동결 위치 유지
                  + Kd·(0 - dq_cur)
                  + qfrc_bias[:6]
    mj_step()                                  ← 시뮬레이션은 계속 실행
```

관절을 동결하면서도 `mj_step()`은 계속 호출하여  
물리 시뮬레이션이 멈추지 않고 안정 상태를 유지한다.

### 10.3 충돌 해제

```
/ur5e/cmd/collision_reset  (std_msgs/Bool, data=true) 수신
    │
    ▼
reset_collision():
    collision_detected ← False
    진행 중 trajectory / IK 명령 초기화
    state → INIT_POSITION    ← 현재 EE 위치를 목표로 재설정
```

---

## 파라미터 요약

| 그룹 | 파라미터 | 기본값 | 단위 |
|------|----------|--------|------|
| **PD** | `Kp` | 500 | N·m/rad |
| | `Kd` | 50 | N·m·s/rad |
| **IK** | `Kp_pos` | 5.0 | 1/s |
| | `Kp_rot` | 2.0 | 1/s |
| | `λ²` | 1e-4 | — |
| | `Δq_max` | 0.1 | rad/step |
| **EE 한계** | `v_max` | 1.0 | m/s |
| | `a_max` | 5.0 | m/s² |
| | `j_max` | 50.0 | m/s³ |
| **소프트 리밋** | `margin` | 0.05 | — |
| | `Kp_soft` | 200 | N·m/rad |
| | `Kd_soft` | 20 | N·m·s/rad |
| **루프** | `loop_hz` | 500 | Hz |
