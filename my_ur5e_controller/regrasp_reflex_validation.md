# ReGraspReflex MuJoCo 검증

## 개요

`grasp_controller.py`의 `ReGraspReflex`가 실제 물리 시뮬레이션에서 올바르게 동작하는지
MuJoCo 기반 검증 스크립트로 확인한 작업 기록.

---

## 구성 파일

| 파일 | 역할 |
|------|------|
| `two_finger_gripper.xml` | 검증용 MuJoCo 모델 (2지 각 2-DOF) |
| `test_regrasp_reflex.py` | 4-Phase 검증 스크립트 |
| `grasp_controller.py` | 수정된 ReGraspReflex / AntiSlipReflex |

---

## 실행 방법

### Docker 컨테이너 내부에서 실행

```bash
docker exec ur5e_mujoco_ros2 bash -c \
  "cd /ros2_ws/src/my_ur5e_controller && python3 test_regrasp_reflex.py"
```

### 호스트에서 직접 실행 (MuJoCo 설치된 경우)

```bash
cd my_ur5e_controller
python3 test_regrasp_reflex.py
```

### 기대 출력

```
==========================================================
  ReGraspReflex 검증  —  two_finger_gripper.xml
==========================================================

Phase 0  오픈 상태 (접촉 없음) → IDLE 유지 ✓ PASS
Phase 1  그리퍼 닫기 (위치 제어 700 스텝 = 1.4 s)  ✓ PASS
Phase 2  비대칭 → TRIGGERED→EXECUTING→RESOLVED→IDLE  ✓ PASS
Phase 3  대칭 접촉 → reflex 비발동 (IDLE 유지)  ✓ PASS

  ✓  ALL PASS — ReGraspReflex 검증 완료
```

---

## MuJoCo 모델: two_finger_gripper.xml

### 구조

```
world
└── palm (고정, pos="0 0 0.12")
    ├── f0_prox_body (x=-0.035, 왼쪽 손가락)
    │   ├── j0_prox  axis="0 -1 0"  range=[-0.05, 1.57]
    │   └── f0_dist_body
    │       ├── j0_dist  axis="0 -1 0"  range=[-0.05, 1.57]
    │       └── site: tip0
    └── f1_prox_body (x=+0.035, 오른쪽 손가락)
        ├── j1_prox  axis="0 1 0"  range=[-0.05, 1.57]
        └── f1_dist_body
            ├── j1_dist  axis="0 1 0"  range=[-0.05, 1.57]
            └── site: tip1

object (고정, freejoint 없음, pos="0 0 0.034")
└── cylinder: radius=0.013, half-height=0.025
```

### 힌지 축 방향 설계 근거

MuJoCo 힌지 관절은 오른손 법칙을 따른다.
링크 방향 벡터 `(0, 0, -0.05)` (팜에서 아래로 뻗음)에 대해:

- **Finger 0 (x=-0.035, 왼쪽)**: `axis="0 -1 0"` → 양각도 증가 시 팁이 **+x** 방향(중심 쪽)으로 이동 → 파지 방향
- **Finger 1 (x=+0.035, 오른쪽)**: `axis="0 1 0"` → 양각도 증가 시 팁이 **-x** 방향(중심 쪽)으로 이동 → 파지 방향

두 손가락 모두 양의 관절 각도를 키우면 닫힌다.

### 시뮬레이터 설정

```xml
<option timestep="0.002" gravity="0 0 0" integrator="Euler"/>
```

- `gravity="0 0 0"`: 중력 비활성화 (정적 접촉력만 분석)
- `integrator="Euler"`: freejoint 없는 고정 객체에서 Euler가 안정적 (RK4는 불필요한 복잡성)
- 액추에이터: `position` 타입, `kp=30`, `forcerange="-8 8 N"`
- 센서: `touch` 타입 (법선력 스칼라 합, N 단위)

---

## 검증 4단계 설명

### Phase 0 — 오픈 상태 (IDLE 유지)

- 그리퍼 완전 오픈, `ContactState.in_contact=False` 주입
- 60 스텝 동안 `ReflexCoordinator.update()` 호출
- **검증**: `reflex.state == IDLE` 유지 여부

### Phase 1 — 그리퍼 닫기 (물리 접촉 확인)

- 위치 제어로 모든 관절 `0.18 rad` 목표
- 700 스텝(1.4초) 시뮬레이션
- **검증**: `touch0 > 0.01 N` AND `touch1 > 0.01 N`

물체 중심 기준 팁 최종 x 위치 계산:

```
x_tip0 ≈ -0.035 + 0.09 × sin(0.18) ≈ -0.035 + 0.016 = -0.019
x_tip1 ≈ +0.035 - 0.09 × sin(0.18) ≈ +0.035 - 0.016 = +0.019
```

실린더 반경 0.013 + 캡슐 반경 0.007 = 0.020 > 0.019 → 접촉 발생.

### Phase 2 — 비대칭 접촉 → 상태머신 전이

합성 접촉 데이터 주입 (MuJoCo 물리 접촉과 무관하게 직접 생성):

```python
ASYM_POS = [[+0.60, 0.0], [-0.10, 0.0]]  # sym_score = 0.75 < 0.85 → TRIGGERED
SYM_POS  = [[+0.05, 0.0], [-0.05, 0.0]]  # sym_score = 1.00 ≥ 0.85 → 해소
```

시퀀스:
1. 0~149 스텝: 비대칭 접촉 → TRIGGERED → EXECUTING
2. 150 스텝~: 대칭 접촉으로 전환 → `execute()` 반환 `None` → RESOLVED → IDLE

**검증 항목**:
- 상태머신 완전 전이: `IDLE → TRIGGERED → EXECUTING → RESOLVED → IDLE`
- q_target 조정 방향: f0 Δprox > 0 (더 닫힘), f1 Δprox < 0 (더 열림)

대칭도 수식:
```
sym_score = 1 - |u_fi + u_fj| / 2
           = 1 - |0.60 + (-0.10)| / 2
           = 1 - 0.25 = 0.75
```

### Phase 3 — 대칭 접촉 (IDLE 유지)

- `SYM_POS` 접촉만 주입 (sym_score = 1.0)
- 250 스텝 동안 IDLE 이탈 여부 확인
- **검증**: reflex가 한 번도 발동되지 않음

---

## 버그 수정: `grasp_controller.py`

### 문제

`AntiSlipReflex.execute()`와 `ReGraspReflex.execute()` 내부에 다음 코드가 있었다:

```python
def execute(self, ...):
    if self.state == ReflexState.TRIGGERED:   # ← 절대 True가 될 수 없음
        for fi in range(self.n_fingers):
            self._q_target[fi] = motor_states[fi].q.copy()
```

`execute()`는 `ReflexCoordinator.update()`의 `EXECUTING` 분기에서만 호출된다.
호출 시점의 `self.state`는 이미 `EXECUTING`이므로 위 조건은 dead code였다.
결과적으로 `_q_target`이 초기값(0 행렬) 그대로 남아, 관절이 0 rad로 리셋되는 버그가 발생했다.

### 수정

`_exec_started` 플래그를 도입해 `execute()` 첫 진입 시 현재 관절 각도를 복사한다:

```python
# __init__
self._exec_started = False

# check() — IDLE에서 호출, 다음 execute() 실행 전 초기화
def check(self, ...):
    self._exec_started = False
    ...

# execute() — 첫 진입 시에만 q 스냅샷
def execute(self, ...):
    if not self._exec_started:
        for fi in range(self.n_fingers):
            self._q_target[fi] = motor_states[fi].q.copy()
        self._exec_started = True
    ...
```

동일 패턴이 `AntiSlipReflex`와 `ReGraspReflex` 양쪽에 적용됨.

---

## 검증 결과

```
Phase 0  오픈 IDLE 유지           ✓ PASS
Phase 1  물체 접촉 확인            ✓ PASS  (touch0=15.09 N, touch1=15.09 N)
Phase 2  비대칭 상태머신 전이      ✓ PASS  (IDLE→TRIGGERED@0→EXECUTING@1→RESOLVED@150→IDLE@151)
Phase 3  대칭 비발동               ✓ PASS  (sym_score=1.000)

ALL PASS
```
