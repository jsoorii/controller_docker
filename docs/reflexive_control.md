# Grasp Controller — Reflexive Control 이론 정리

`grasp_controller.py` 에 구현된 MIT Biomimetics 기반 Reflexive Control 아키텍처를 설명한다.

참고: [MIT Biomimetics Lab — Reflexive Control](https://biomimetics.mit.edu/research/reflexive-control)

---

## 목차

1. [시스템 아키텍처](#1-시스템-아키텍처)
2. [계층 구조와 설계 철학](#2-계층-구조와-설계-철학)
3. [상태머신](#3-상태머신)
4. [500 Hz 루프 전체 흐름](#4-500-hz-루프-전체-흐름)
5. [접촉 센서 모델 (ContactState)](#5-접촉-센서-모델-contactstate)
6. [Reflex 상태머신](#6-reflex-상태머신)
7. [Anti-Slip Reflex](#7-anti-slip-reflex)
8. [Re-Grasp Reflex](#8-re-grasp-reflex)
9. [ReflexCoordinator — 우선순위 중재](#9-reflexcoordinator--우선순위-중재)
10. [기본 PD 제어 (Reflex 없을 때)](#10-기본-pd-제어-reflex-없을-때)
11. [4절 링크 기구학 (FourBarLinkage)](#11-4절-링크-기구학-fourbarlinkage)
12. [손가락 기구학 (FingerKinematics)](#12-손가락-기구학-fingerkinematics)
13. [High-level Planner (GraspPlanner)](#13-high-level-planner-graspplanner)
14. [파라미터 요약](#14-파라미터-요약)

---

## 1. 시스템 아키텍처

```
┌─────────────────────────────────────────────────────────────────────┐
│                     GraspController                                  │
│                                                                      │
│  ┌─────────────────────────┐    ┌────────────────────────────────┐  │
│  │  High-level Planner     │    │  Low-level Reflex Layer        │  │
│  │  GraspPlanner           │    │  ReflexCoordinator             │  │
│  │  < 1 Hz                 │    │  300+ Hz (RT 루프 내)          │  │
│  │                         │    │                                │  │
│  │  - GraspType 선택        │    │  ┌──────────────────────────┐  │  │
│  │  - q_target 프리셋 주입  │───►│  │ AntiSlipReflex [P=0]     │  │  │
│  │  - Reflex 활성화 지령    │    │  │ ReGraspReflex  [P=1]     │  │  │
│  └─────────────────────────┘    │  └──────────────┬───────────┘  │  │
│                                 │                 │ torque override│  │
│                                 └─────────────────┼───────────────┘  │
│                                                   │                  │
│                                    None ◄─── override? ───► dict    │
│                                     │                         │      │
│                               기본 PD 토크            Reflex 토크    │
│                                     │                         │      │
│                                     └──────────┬──────────────┘      │
│                                                ▼                      │
│                                      motor_cmd[fi].current            │
└─────────────────────────────────────────────────────────────────────┘
```

**스레드 구성**

| 스레드 | 주기 | 역할 |
|--------|------|------|
| RT 제어 스레드 | 500 Hz (2 ms) | 상태 읽기 → FK → Reflex → 토크 출력 |
| Planner 스레드 | ≤ 1 Hz | 파지 타입 결정 → q_target 주입 |
| 외부 태스크 | 이벤트 기반 | write_command / update_contact / read_state |

---

## 2. 계층 구조와 설계 철학

### 전통적 파지 제어의 한계

전통적 접근은 비전 기반 전역 계획(global planning)으로 최적 파지 자세를 계산한다.

```
vision → grasp planning → execute
(100ms~1s)   (느림)      (개루프)
```

문제점:
- 계획 중 물체가 움직이거나 미끄러지면 대응 불가
- 센서 노이즈·모델 오차에 취약
- 계획 시간이 반응 속도를 제한

### MIT Reflexive Control 접근

계획 최적화보다 **실행 가능성(feasibility)** 을 우선하는 계층 분리 구조.

```
High-level planner  (< 1 Hz)   : 파지 타입 결정, 대략적 목표 제공
Low-level reflexes  (300+ Hz)  : 접촉 센서 기반 즉각 반응, 계획에 독립적
```

핵심 원칙:
1. **반응 우선**: Reflex가 계획 명령을 override — 물리 현실이 계획보다 중요
2. **계층 독립**: Planner가 느려도 Reflex가 안전 유지
3. **고대역폭 감지**: 저지연 접촉·힘 센서로 300+ Hz 반응 가능

성과 (MIT 원논문):
- Re-grasp reflex: **150 ms** 이내 antipodal 재정렬
- Anti-slip reflex: 슬립 발생 전 선제적 수직력 증가
- 기준 대비 파지 성공 볼륨 **55% 향상**, 자율 물체 제거 **90%+** 성공

---

## 3. 상태머신

`GraspState` — 시스템 초기화 시퀀스를 보장하는 5단계 상태머신.

```
              시작
               │
               ▼
        ┌─────────────┐
        │ WAIT_STABLE │  0.1 s 대기 (하드웨어 안정화)
        └──────┬──────┘
               │
               ▼
        ┌─────────────┐
        │INIT_CONTROL │  MotorState / FingerState 메모리 초기화
        │             │  miss_count = 0
        └──────┬──────┘
               │
               ▼
        ┌─────────────┐
        │ CHECK_MOTOR │◄──────────────┐
        │             │               │ 연결 실패 → 0.5 s 재시도
        └──────┬──────┘───────────────┘
               │ 연결 확인
               ▼
        ┌─────────────┐
        │INIT_POSITION│  _home_q 영점화, q_target ← home_q
        └──────┬──────┘
               │
               ▼
        ┌─────────────┐
        │ RUN_CONTROL │◄── 2 ms 주기 반복
        └─────────────┘
```

---

## 4. 500 Hz 루프 전체 흐름

`RUN_CONTROL` 상태에서 매 2 ms 실행:

```
──────────────────────── 500 Hz (2 ms) ────────────────────────────

  _run_control_step(command)

  Step 1 │ joint state 읽기
         │   [실하드웨어] motor_iface.read_state()  폴링 재시도 (max 20회)
         │   [시뮬 모드]  cmd → state 10% 추종 (가상 응답)
         │
         ▼
  Step 2 │ 4절 링크 FK (손가락 × n_fingers)
         │   FourBarLinkage.fk(q_motor[0]) → q_proximal
         │   q_finger = [q_proximal, q_motor[1], q_motor[2]]
         │
         ▼
  Step 3 │ 손가락 FK + 자코비안 (손가락 × n_fingers)
         │   FingerKinematics.fk(q_finger) → FingerState{p, R, J}
         │
         ▼
  Step 4 │ Reflex Layer 업데이트
         │   contact_state 스냅샷 (뮤텍스 복사)
         │   ReflexCoordinator.update(finger_states, contact_snap, motor_states, dt)
         │       → override: {"torque": ndarray} or None
         │
         ▼
  Step 5 │ 토크 결정
         │   if override: torque = override["torque"]
         │   else:        torque = controller_run(PD + 자코비안 힘)
         │
         ▼
  Step 6 │ 명령 전송
         │   motor_cmd[fi].current = torque[fi]
         │   motor_iface.write_command(...)
         │
         ▼
  Step 7 │ 상태 버퍼 업데이트 (외부 read용, 뮤텍스 보호)
         │   _state_buf["q/qdot/current"] ← motor_state
         │   _state_buf["active_reflexes"] ← coordinator.active_reflexes()
```

---

## 5. 접촉 센서 모델 (ContactState)

MIT 원논문의 **bimodal force sensor** 출력을 모델링한다.

```
ContactState (손가락당)
  force_3axis : [fx, fy, fz]  (N, 센서 로컬 프레임)
                fz = 법선력 (normal force)
                fx, fy = 전단력 (shear / tangential force)
  contact_pos : [u, v]  (-1 ~ 1, 센서 면 상 접촉 위치)
  in_contact  : bool  (fz ≥ contact_threshold)
```

**센서 프레임 규약**

```
        z (법선, 물체를 향함)
        ↑
        │
        ──── x (전단, u 방향)
       /
      y (전단, v 방향)
```

법선력 `fz` 는 파지력(grasp force)에 해당하고,  
전단력 `(fx, fy)` 는 슬립의 원인이 되는 힘이다.

**업데이트 인터페이스**

```python
controller.update_contact(
    finger_idx,
    force_3axis = [fx, fy, fz],   # N
    contact_pos = [u, v],          # -1 ~ 1
    contact_threshold = 0.1        # N, 접촉 판정 임계값
)
```

RT 루프에서 뮤텍스를 통해 스냅샷을 복사하므로 외부 태스크에서 임의 시점에 호출 가능.

---

## 6. Reflex 상태머신

각 `Reflex` 인스턴스는 독립적인 4단계 상태머신을 가진다.

```
         ┌─────────────────────────────────────┐
         │                                     │
         ▼                                     │
      ┌──────┐   check() = True   ┌───────────┐│
      │ IDLE │──────────────────► │ TRIGGERED ││
      └──────┘                    └─────┬─────┘│
         ▲                             │       │
         │ (다음 스텝)                  │       │
         │                             ▼       │
    ┌──────────┐◄─── execute()=None ┌──────────┐│
    │ RESOLVED │     or timeout     │EXECUTING ││
    └──────────┘                    └──────────┘│
                                                │
         check() = False 또는 timeout 초과 ─────┘
```

| 상태 | 진입 조건 | 동작 |
|------|-----------|------|
| `IDLE` | 초기, 또는 RESOLVED 다음 스텝 | check() 감시 |
| `TRIGGERED` | check() = True | 다음 스텝 EXECUTING으로 전이 |
| `EXECUTING` | TRIGGERED 다음 스텝 | execute() 호출, 토크 override 반환 |
| `RESOLVED` | execute() = None 또는 timeout | IDLE 전이 대기 |

`timeout_s` 초과 시 자동으로 RESOLVED → 무한 실행 방지.

---

## 7. Anti-Slip Reflex

### 7.1 물리 배경 — 쿨롱 마찰 모델

파지 중 손가락 끝과 물체 표면 사이 접촉에 쿨롱 마찰 조건:

```
|F_tangential| ≤ μ · F_normal
```

이 조건이 위반될 때 슬립이 발생한다.  
슬립까지의 여유를 나타내는 **슬립 비율(slip ratio)**:

```
slip_ratio = |F_tangential| / (μ · F_normal + ε)
           = √(fx² + fy²) / (μ · fz + ε)
```

`slip_ratio ≥ 1.0` 이면 슬립 발생.  
`slip_threshold = 0.85` 로 슬립 **직전에** 선제적으로 개입한다.

### 7.2 Reflex 발동 조건

```python
def check(contact_states) -> bool:
    for cs in contact_states:
        if cs.in_contact and slip_ratio(cs) > slip_threshold:
            return True
    return False
```

### 7.3 실행 로직

목표: 슬립 비율을 `safety_factor (0.60)` 이하로 낮추는 것.

전략: 슬립 위험 손가락의 주 굽힘 관절(`q[0]`)을 `Δq_step`씩 닫아  
파지력(법선력 `fz`)을 점진적으로 증가시킨다.

```
각 스텝:
    for fi in range(n_fingers):
        if in_contact[fi] and slip_ratio[fi] > safety_factor:
            q_target[fi, 0] += Δq_step    ← 관절 닫기 (파지력 증가)
        
        τ[fi] = Kp · (q_target[fi] - q[fi]) + Kd · (-qdot[fi])

완료 조건:
    모든 in_contact 손가락의 slip_ratio ≤ safety_factor
```

### 7.4 슬립 비율 변화 분석

관절을 닫으면 `fz` 증가:

```
Δfz ≈ k_contact · Δδ    (k_contact: 접촉 강성, Δδ: 침입 깊이 변화)
```

`Δq_step = 0.01 rad` 일 때 법선력 변화:

```
Δδ ≈ l_link · Δq = l_link · 0.01    (l_link: 끝 링크 길이 ≈ 25 mm)
Δδ ≈ 0.25 mm
```

실제 `k_contact` 는 물체 재질에 따라 달라지므로, 점진적 증분 방식이 안전하다.

### 7.5 파라미터

| 파라미터 | 기본값 | 의미 |
|----------|--------|------|
| `mu` | 0.5 | 마찰 계수 (고무/플라스틱 기준) |
| `slip_threshold` | 0.85 | Reflex 발동 슬립 비율 |
| `safety_factor` | 0.60 | 목표 슬립 비율 (완료 조건) |
| `f_normal_min` | 0.1 N | 접촉 인식 최소 법선력 |
| `dq_step` | 0.01 rad | 한 스텝 관절 닫기 증분 |
| `timeout_s` | 0.3 s | 최대 실행 시간 |
| `priority` | 0 | 최고 우선순위 |

---

## 8. Re-Grasp Reflex

### 8.1 물리 배경 — Antipodal 파지 조건

안정적인 파지를 위해 접촉력 합이 물체 무게 중심을 통과해야 한다.  
이를 만족하는 가장 단순한 기하학적 조건이 **antipodal contact**:

```
두 손가락의 접촉 법선이 반대 방향이고 동일 선상에 있을 때
최소 파지력으로 최대 파지 안정성 달성
```

비대칭 접촉의 예:

```
   ○ 물체
  ╱│╲
 ╱ │ ╲
F₁  C  F₂     ← 대칭: F₁, F₂가 물체 중심 C를 지남 → 안정
   
   ○ 물체
  ╱│
 ╱ │  
F₁  C  F₂     ← 비대칭: 합력이 C에서 어긋남 → 불안정
        (F₂가 한쪽으로 치우침)
```

### 8.2 대칭도 측정 (symmetry score)

접촉 위치 센서의 `u` 좌표(가로 방향)를 사용한 단순화된 대칭도:

```
sym_score(fi, fj) = 1 - |u_fi + u_fj| / 2

해석:
  u_fi = -u_fj  →  |합| = 0  →  sym_score = 1.0  (완벽 대칭)
  u_fi = u_fj   →  |합| = 2  →  sym_score = 0.0  (최대 비대칭)
```

두 손가락이 하나라도 접촉이 없으면 `sym_score = 1.0` (판단 보류).

### 8.3 Reflex 발동 조건

```python
def check(contact_states) -> bool:
    for (fi, fj) in finger_pairs:
        if sym_score(contact_states[fi], contact_states[fj]) < sym_threshold:
            return True
    return False
```

`finger_pairs` 기본값: 인접 손가락 쌍 `[(0,1), (1,2), ...]`

### 8.4 실행 로직 — 방향 조정

비대칭 손가락 쌍에서 `u` 값이 큰 쪽(접촉이 한쪽으로 치우친 손가락)을  
반대 방향으로 조정하여 대칭 상태로 수렴시킨다.

```
u_fi > u_fj 이면:
    fi 를 +Δq 방향으로 (더 안쪽)
    fj 를 -Δq 방향으로 (더 바깥쪽)

u_fi ≤ u_fj 이면:
    반대로 조정

각 스텝 PD 토크로 q_target 추종:
    τ[fi] = Kp · (q_target[fi] - q[fi]) + Kd · (-qdot[fi])

완료 조건:
    모든 손가락 쌍의 sym_score ≥ sym_threshold
```

### 8.5 150 ms 타임아웃 설계 근거

MIT 원논문의 re-grasping 완료 시간 목표가 **150 ms** 이내.  
`timeout_s = 0.15` 로 설정하여 이 기준을 직접 적용.

500 Hz 루프 기준 최대 스텝 수:

```
max_steps = 0.15 s × 500 Hz = 75 스텝
```

각 스텝 `Δq = 0.005 rad` → 최대 조정량:

```
Δq_total = 75 × 0.005 = 0.375 rad ≈ 21.5°
```

### 8.6 파라미터

| 파라미터 | 기본값 | 의미 |
|----------|--------|------|
| `sym_threshold` | 0.85 | Reflex 발동 대칭도 임계값 |
| `dq_step` | 0.005 rad | 한 스텝 관절 조정 증분 |
| `finger_pairs` | 인접 쌍 | 대칭도 비교 손가락 쌍 |
| `timeout_s` | 0.15 s | MIT 논문 기준 150 ms |
| `priority` | 1 | AntiSlip보다 낮은 우선순위 |

---

## 9. ReflexCoordinator — 우선순위 중재

### 9.1 역할

여러 Reflex가 동시에 활성화될 수 있으므로, 우선순위에 따라 단일 토크 override를 선택한다.

```
EXECUTING 상태인 Reflex 목록 (예):
    AntiSlipReflex  [P=0]  → torque_A
    ReGraspReflex   [P=1]  → torque_B

중재 결과:
    priority 0이 선택 → torque_A 반환
```

### 9.2 알고리즘

```python
def update(finger_states, contact_states, motor_states, dt):
    overrides = []
    for reflex in sorted_reflexes:            # priority 오름차순 정렬
        result = reflex.update(...)            # 상태머신 전진 + 토크 계산
        if result is not None:
            overrides.append((reflex.priority, result))
    
    if not overrides:
        return None                            # 기본 PD 토크 사용
    
    overrides.sort(key=lambda x: x[0])
    return overrides[0][1]                     # 최소 priority 값 선택
```

**모든 Reflex의 `update()`는 항상 호출된다** — 낮은 우선순위 Reflex도 상태머신이 계속 진행되므로, 높은 우선순위 Reflex가 끝난 직후 이어서 실행 가능.

### 9.3 우선순위 설계 근거

```
AntiSlipReflex [P=0] > ReGraspReflex [P=1]
```

이유:
- 슬립은 즉각 물체 낙하로 이어지므로 최우선 처리
- 재파지는 슬립이 없는 상태에서 수행해야 효과적
- 두 Reflex가 동시 활성화되면 먼저 슬립을 해결하고, 이후 대칭 조정

---

## 10. 기본 PD 제어 (Reflex 없을 때)

Reflex override가 없을 때 `controller_run()` 이 실행된다.

### 10.1 PD + 자코비안 힘 제어

```
τ_pd = Kp · (q_target - q) + Kd · (-qdot)

force_target > 0 이면:
    force_vec = [f_target, 0, 0, 0, 0, 0]    ← x방향 힘
    τ_force   = J^T · force_vec              ← 자코비안 전치
    τ_out = τ_pd + τ_force

else:
    τ_out = τ_pd
```

### 10.2 자코비안 전치 힘 제어 (Jacobian Transpose Force Control)

끝점에 힘 `F ∈ ℝ⁶` (선속도 3 + 각속도 3)를 가하기 위한 관절 토크:

```
τ = J^T · F
```

유도: 가상 작업(virtual work)의 원리

```
δW = F^T · δx = F^T · J · δq = (J^T · F)^T · δq = τ^T · δq

→ τ = J^T · F
```

`J ∈ ℝ^{6×3}` (6: 선속도+각속도, 3: 관절 수) 이므로 `J^T ∈ ℝ^{3×6}`.

장점: 역행렬 불필요, 특이점 근방에서도 안정.  
단점: 위치 정밀 제어 불가 (힘 방향 제어에만 적합).

### 10.3 게인

| 파라미터 | 기본값 | 단위 |
|----------|--------|------|
| `Kp` | 200 | N·m/rad |
| `Kd` | 20 | N·m·s/rad |

---

## 11. 4절 링크 기구학 (FourBarLinkage)

### 11.1 구조

```
        L3 (커플러)
   O2 ──────────── O3
   │                │
L2 │ (크랭크, 입력) │ L4 (출력, 손가락 근위부 연결)
   │                │
   O1 ──────────── O4
        L1 (지지 프레임, 고정)
```

| 링크 | 기본 길이 | 역할 |
|------|-----------|------|
| L1 | 40 mm | 고정 링크 (그라운드) |
| L2 | 15 mm | 입력 링크 (모터 구동) |
| L3 | 35 mm | 커플러 링크 |
| L4 | 25 mm | 출력 링크 (손가락 근위부) |

### 11.2 Freudenstein 방정식 (FK)

4절 링크의 기하학적 구속 조건으로부터 유도:

```
K₁·cos(θ₄) - K₂·cos(θ₂) + K₃ = cos(θ₂ - θ₄)

K₁ = L1/L4
K₂ = L1/L2
K₃ = (L2² - L3² + L4² + L1²) / (2·L2·L4)
```

반각 치환 `t = tan(θ₄/2)` 로 이차방정식으로 변환:

```
A·t² + B·t + C = 0

A = cos(θ₂) - K₁ - K₂·cos(θ₂) + K₃
B = -2·sin(θ₂)
C = K₁ - (K₂+1)·cos(θ₂) + K₃

t = (-B - √(B²-4AC)) / (2A)    ← 크랭크-록커 분기 선택
θ₄ = 2·arctan(t)
```

### 11.3 IK (이분법)

목표 출력각 `θ₄_target` → 입력각 `θ₂` 역산.

```
잔차 함수: r(θ₂) = fk(θ₂) - θ₄_target

① N=200 샘플로 부호 변환 구간 [a, b] 탐색
② 이분법 50회 반복: tol = 1e-6 rad ≈ 0.00006°
③ 해 없으면 0 반환 (기구학 한계 외)
```

### 11.4 속도비

```
dθ₄/dθ₂ = [fk(θ₂+ε) - fk(θ₂-ε)] / (2ε)    (ε = 1e-5)
```

관절 속도 변환에 사용:

```
qdot_proximal = velocity_ratio(q_motor[0]) · qdot_motor[0]
```

---

## 12. 손가락 기구학 (FingerKinematics)

### 12.1 평면 3R 체인

손가락 관절 3개(근위/중위/원위)가 로컬 xy-평면 내에서 회전한다고 가정.

```
   base ── l₁ ── J₁ ── l₂ ── J₂ ── l₃ ── 끝점
           q₁           q₂           q₃
```

링크 길이 기본값:

| 링크 | 기본 길이 | 대응 관절 |
|------|-----------|-----------|
| l₁ | 40 mm | 근위지 (proximal) |
| l₂ | 30 mm | 중위지 (middle) |
| l₃ | 25 mm | 원위지 (distal) |

### 12.2 FK — 순기구학

누적 각도:

```
a₁ = q₁
a₂ = q₁ + q₂
a₃ = q₁ + q₂ + q₃
```

로컬 프레임 관절 위치:

```
p₀ = [0, 0, 0]
p₁ = p₀ + [l₁·cos(a₁), l₁·sin(a₁), 0]
p₂ = p₁ + [l₂·cos(a₂), l₂·sin(a₂), 0]
p₃ = p₂ + [l₃·cos(a₃), l₃·sin(a₃), 0]    ← 끝점
```

로봇 기준 좌표 변환:

```
p_world = base_R · p₃ + base_p
R_world = base_R · R_local(a₃)
```

### 12.3 기하학적 자코비안 (Geometric Jacobian)

각 관절 `i` 의 z축 회전에 의한 끝점 속도 기여:

```
z_axis = base_R · [0, 0, 1]^T    ← 모든 관절이 동일한 z축 회전

J[:3, i] = z_axis × (p_end - p_joint_i)    ← 선속도 기여 (cross product)
J[3:, i] = z_axis                           ← 각속도 기여 (모두 동일)
```

결과: `J ∈ ℝ^{6×3}`

```
선속도 (3×3):  J_v = [z×r₀ | z×r₁ | z×r₂]
각속도 (3×3):  J_ω = [z    | z    | z   ]
```

`rᵢ = p_end - p_jointᵢ` (관절 i → 끝점 벡터)

### 12.4 자코비안 전치 힘 제어 (force_to_torque)

```
τ (3,) = J^T (3×6) · F_world (6,)

F_world = [fx, fy, fz, mx, my, mz]    ← 힘(N) + 모멘트(N·m)
```

---

## 13. High-level Planner (GraspPlanner)

### 13.1 역할과 동작 주기

```
GraspPlanner (≤ 1 Hz)

┌──────────────────────────────────────────────┐
│  1. _grasp_type 확인                          │
│  2. 해당 타입의 q_target 프리셋 선택          │
│  3. controller.write_command(fi, q=q_target) │
│     (n_fingers 회 반복)                      │
└──────────────────────────────────────────────┘
```

RT 루프(500 Hz)의 `_cmd_buf["q_target"]` 만 갱신 — 직접 토크 계산 없음.

### 13.2 파지 타입

| GraspType | 설명 | 예시 용도 |
|-----------|------|-----------|
| `POWER` | 전체 손가락 감싸기 | 원통형 물체, 병 |
| `PINCH` | 엄지-검지 집기 | 작은 물체, 정밀 조작 |
| `LATERAL` | 측면 집기 | 열쇠, 카드 |

### 13.3 프리셋 예시 (3손가락 기준)

```python
POWER   : [[60°, 40°, 30°], [60°, 40°, 30°], [60°, 40°, 30°]]
PINCH   : [[ 0°, 60°, 45°], [ 0°, 60°, 45°], [ 0°,  0°,  0°]]
LATERAL : [[30°, 20°, 15°], [30°, 20°, 15°], [30°, 20°, 15°]]
```

실제 로봇에서는 캘리브레이션으로 측정한 값으로 교체.

### 13.4 계층 분리 이유

Planner가 느린 이유:
- 비전/물체 인식 파이프라인이 느림 (수백 ms ~ 수 s)
- 최적 파지 자세 계산 비용
- 잦은 q_target 변경은 Reflex 실행을 방해

Planner를 저주기로 유지하면:
- RT 루프 간섭 없음
- Reflex가 플래너 명령을 덮어써도 안전 보장
- 시스템 응답성 유지

---

## 14. 파라미터 요약

### 기본 제어

| 그룹 | 파라미터 | 기본값 | 단위 |
|------|----------|--------|------|
| **PD** | `Kp` | 200 | N·m/rad |
| | `Kd` | 20 | N·m·s/rad |
| **루프** | `loop_hz` | 500 | Hz |
| | `poll_max` | 20 | 회 |

### Anti-Slip Reflex

| 파라미터 | 기본값 | 단위 |
|----------|--------|------|
| `mu` | 0.5 | — |
| `slip_threshold` | 0.85 | — |
| `safety_factor` | 0.60 | — |
| `f_normal_min` | 0.1 | N |
| `dq_step` | 0.01 | rad |
| `timeout_s` | 0.3 | s |

### Re-Grasp Reflex

| 파라미터 | 기본값 | 단위 |
|----------|--------|------|
| `sym_threshold` | 0.85 | — |
| `dq_step` | 0.005 | rad |
| `timeout_s` | 0.15 | s |

### 4절 링크

| 링크 | 기본 길이 | 단위 |
|------|-----------|------|
| L1 (고정) | 40 | mm |
| L2 (입력) | 15 | mm |
| L3 (커플러) | 35 | mm |
| L4 (출력) | 25 | mm |

### 손가락 기구학

| 링크 | 기본 길이 | 단위 |
|------|-----------|------|
| l₁ (근위지) | 40 | mm |
| l₂ (중위지) | 30 | mm |
| l₃ (원위지) | 25 | mm |
