"""
test_regrasp_reflex.py
-----------------------
2-finger 2-DOF gripper (two_finger_gripper.xml) + ReGraspReflex 검증 스크립트.

검증 흐름:
  Phase 0 — 오픈 상태: 접촉 없음, IDLE 유지 확인
  Phase 1 — 닫기  : 위치 제어로 그리퍼를 물체에 닫음, 양쪽 접촉 확인
  Phase 2 — 비대칭: 합성 비대칭 contact_pos 주입 → TRIGGERED → EXECUTING,
             이후 대칭 contact_pos 전환 → RESOLVED → IDLE 전이 확인
             q_target 조정 방향 검증 (비대칭 쪽 손가락이 더 닫힘)
  Phase 3 — 대칭  : 대칭 contact_pos만 주입, IDLE 유지 확인

실행:
  cd my_ur5e_controller && python3 test_regrasp_reflex.py
"""

import os
import sys
sys.path.insert(0, os.path.dirname(__file__))

import mujoco
import numpy as np

from grasp_controller import (
    ContactState,
    FingerState,
    MotorState,
    ReflexCoordinator,
    ReflexState,
    ReGraspReflex,
)

# ── 모델 로드 ──────────────────────────────────────────────────────────────
_HERE      = os.path.dirname(__file__)
MODEL_PATH = os.path.join(_HERE, "two_finger_gripper.xml")
model = mujoco.MjModel.from_xml_path(MODEL_PATH)
data  = mujoco.MjData(model)
DT    = model.opt.timestep  # 0.002 s

# ── 인덱스 헬퍼 ───────────────────────────────────────────────────────────
def _jid(name):
    return mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)

_joints = [_jid(n) for n in ["j0_prox", "j0_dist", "j1_prox", "j1_dist"]]
QA = [model.jnt_qposadr[j] for j in _joints]   # qpos 주소
VA = [model.jnt_dofadr[j]  for j in _joints]   # qvel 주소

TOUCH0, TOUCH1 = 0, 1   # 센서 인덱스 (XML 선언 순서)


# ── 유틸리티 ──────────────────────────────────────────────────────────────

def get_q():
    return np.array([data.qpos[a] for a in QA])

def get_qdot():
    return np.array([data.qvel[a] for a in VA])

def make_motor_states():
    q, v = get_q(), get_qdot()
    return [
        MotorState(q=q[0:2].copy(), qdot=v[0:2].copy(), current=np.zeros(2)),
        MotorState(q=q[2:4].copy(), qdot=v[2:4].copy(), current=np.zeros(2)),
    ]

def synthetic_contact(pos_list, force_n=1.5):
    """
    합성 ContactState 생성.
    pos_list : [[u0, v0], [u1, v1]]  (-1~1)
    """
    return [
        ContactState(
            force_3axis=np.array([0.0, 0.0, force_n]),
            contact_pos=np.array(pos_list[fi]),
            in_contact=True,
        )
        for fi in range(2)
    ]

def set_ctrl(targets):
    data.ctrl[:] = targets

def step(n=1):
    for _ in range(n):
        mujoco.mj_step(model, data)

def touch_force():
    return data.sensordata[TOUCH0], data.sensordata[TOUCH1]

FINGER_STATES = [FingerState(), FingerState()]   # FK 없이 사용

# 닫기 목표 각도 (rad) — 팁이 물체 표면에 닿는 수준
CLOSE_ANGLE = 0.18


# ═══════════════════════════════════════════════════════════════════════════
# Phase 0  오픈 상태 → IDLE 유지
# ═══════════════════════════════════════════════════════════════════════════
def phase0():
    print("━" * 58)
    print("Phase 0  오픈 상태 (접촉 없음) → IDLE 유지")
    print("━" * 58)

    mujoco.mj_resetData(model, data)

    reflex = ReGraspReflex(n_fingers=2, n_joints=2, sym_threshold=0.85)
    coord  = ReflexCoordinator([reflex])

    no_contact = [
        ContactState(force_3axis=np.zeros(3), contact_pos=np.zeros(2), in_contact=False)
        for _ in range(2)
    ]

    for _ in range(60):
        coord.update(FINGER_STATES, no_contact, make_motor_states(), DT)
        step()

    ok = (reflex.state == ReflexState.IDLE)
    print(f"  reflex 상태: {reflex.state.name}  {'✓ PASS' if ok else '✗ FAIL'}\n")
    return ok


# ═══════════════════════════════════════════════════════════════════════════
# Phase 1  그리퍼 닫기 → 물체 접촉 확인
# ═══════════════════════════════════════════════════════════════════════════
def phase1():
    print("━" * 58)
    print("Phase 1  그리퍼 닫기 (위치 제어 700 스텝 = 1.4 s)")
    print("━" * 58)

    mujoco.mj_resetData(model, data)
    set_ctrl([CLOSE_ANGLE, CLOSE_ANGLE, CLOSE_ANGLE, CLOSE_ANGLE])

    for _ in range(700):
        step()

    q       = get_q()
    t0, t1  = touch_force()
    in_cont = (t0 > 0.01) and (t1 > 0.01)

    print(f"  j0=[{q[0]:.3f}, {q[1]:.3f}] rad   j1=[{q[2]:.3f}, {q[3]:.3f}] rad")
    print(f"  touch0={t0:.3f} N   touch1={t1:.3f} N")
    print(f"  양쪽 접촉: {'✓ PASS' if in_cont else '✗ FAIL (접촉력 부족)'}\n")
    return in_cont


# ═══════════════════════════════════════════════════════════════════════════
# Phase 2  비대칭 접촉 → 상태머신 전이 + q_target 조정 방향 검증
# ═══════════════════════════════════════════════════════════════════════════
#
# f0: u=+0.60, f1: u=-0.10 → sym_score = 1 - |0.60+(-0.10)|/2 = 0.75 < 0.85 → TRIGGERED
# 이후 대칭 접촉(f0: +0.05, f1: -0.05)으로 전환 → execute() 반환 None → RESOLVED → IDLE
#
ASYM_POS = [[+0.60, 0.0], [-0.10, 0.0]]
SYM_POS  = [[+0.05, 0.0], [-0.05, 0.0]]

def phase2():
    print("━" * 58)
    print("Phase 2  비대칭 → TRIGGERED→EXECUTING, 대칭 전환 → RESOLVED→IDLE")
    print("━" * 58)

    # Phase 1 이어서: 손가락 이미 닫혀 있음
    set_ctrl([CLOSE_ANGLE, CLOSE_ANGLE, CLOSE_ANGLE, CLOSE_ANGLE])

    reflex = ReGraspReflex(n_fingers=2, n_joints=2, sym_threshold=0.85, dq_step=0.005)
    coord  = ReflexCoordinator([reflex])

    asym_contact = synthetic_contact(ASYM_POS)
    sym_contact  = synthetic_contact(SYM_POS)

    state_log      = [(ReflexState.IDLE, -1)]   # 초기 상태 기록
    prev_state     = ReflexState.IDLE
    q_exec_first   = None    # execute() 진입 직후 _q_target
    q_exec_last    = None    # execute() 마지막 _q_target

    # 비대칭 기간: 최대 150 스텝, 이후 대칭으로 전환
    ASYM_STEPS = 150
    MAX_STEPS  = 400

    for s in range(MAX_STEPS):
        ms      = make_motor_states()
        contact = asym_contact if s < ASYM_STEPS else sym_contact
        result  = coord.update(FINGER_STATES, contact, ms, DT)

        # 상태 변화 기록
        if reflex.state != prev_state:
            state_log.append((reflex.state, s))
            prev_state = reflex.state

        # execute() override 반영
        if result is not None:
            qt = reflex._q_target.copy()
            if q_exec_first is None:
                q_exec_first = qt
            q_exec_last = qt
            set_ctrl([qt[0, 0], qt[0, 1], qt[1, 0], qt[1, 1]])
        else:
            set_ctrl([CLOSE_ANGLE, CLOSE_ANGLE, CLOSE_ANGLE, CLOSE_ANGLE])

        step()

        # RESOLVED → IDLE 복귀 완료 시 종료
        if reflex.state == ReflexState.IDLE and len(state_log) >= 3:
            break

    print("  상태 전이:")
    for st, idx in state_log:
        print(f"    step {idx:4d}  →  {st.name}")

    # q_target 변화량 검증
    dir_ok = False
    if q_exec_first is not None and q_exec_last is not None:
        delta = q_exec_last - q_exec_first
        print(f"\n  q_target Δ (execute 진입→종료):")
        print(f"    finger0  Δprox={delta[0,0]:+.4f}  Δdist={delta[0,1]:+.4f}")
        print(f"    finger1  Δprox={delta[1,0]:+.4f}  Δdist={delta[1,1]:+.4f}")
        # ASYM: f0.u > f1.u → f0 더 닫힘(Δ>0), f1 열림(Δ<0)
        dir_ok = (delta[0, 0] > 0) and (delta[1, 0] < 0)
        print(f"  조정 방향 (f0 닫힘↑, f1 열림↓): {'✓ PASS' if dir_ok else '✗ FAIL'}")

    seen     = {st.name for st, _ in state_log}
    required = {"IDLE", "TRIGGERED", "EXECUTING", "RESOLVED"}
    sm_ok    = required.issubset(seen)
    missing  = required - seen
    print(f"\n  상태머신 완전 전이: {'✓ PASS' if sm_ok else '✗ FAIL'}")
    if missing:
        print(f"  누락: {missing}")

    ok = sm_ok and dir_ok
    print(f"  Phase 2 종합: {'✓ PASS' if ok else '✗ FAIL'}\n")
    return ok


# ═══════════════════════════════════════════════════════════════════════════
# Phase 3  대칭 접촉 → reflex 비발동
# ═══════════════════════════════════════════════════════════════════════════
def phase3():
    print("━" * 58)
    print("Phase 3  대칭 접촉 → reflex 비발동 (IDLE 유지)")
    print("━" * 58)

    set_ctrl([CLOSE_ANGLE, CLOSE_ANGLE, CLOSE_ANGLE, CLOSE_ANGLE])

    reflex = ReGraspReflex(n_fingers=2, n_joints=2, sym_threshold=0.85)
    coord  = ReflexCoordinator([reflex])

    sym_contact = synthetic_contact(SYM_POS)
    triggered   = False

    for _ in range(250):
        coord.update(FINGER_STATES, sym_contact, make_motor_states(), DT)
        step()
        if reflex.state != ReflexState.IDLE:
            triggered = True
            break

    score = 1.0 - abs(SYM_POS[0][0] + SYM_POS[1][0]) / 2.0
    print(f"  sym_score = {score:.3f}  (threshold=0.85)")
    ok = not triggered
    print(f"  발동 여부: {'✗ FAIL (발동됨)' if triggered else '✓ PASS (비발동)'}\n")
    return ok


# ═══════════════════════════════════════════════════════════════════════════
# 메인
# ═══════════════════════════════════════════════════════════════════════════
def main():
    print()
    print("=" * 58)
    print("  ReGraspReflex 검증  —  two_finger_gripper.xml")
    print("=" * 58)
    print()

    r0 = phase0()
    r1 = phase1()
    r2 = phase2()
    r3 = phase3()

    print("=" * 58)
    print("  최종 결과")
    print("=" * 58)
    rows = [
        ("Phase 0  오픈 IDLE 유지",       r0),
        ("Phase 1  물체 접촉 확인",        r1),
        ("Phase 2  비대칭 상태머신 전이",  r2),
        ("Phase 3  대칭 비발동",           r3),
    ]
    all_ok = True
    for name, ok in rows:
        mark = "✓" if ok else "✗"
        print(f"  {mark}  {name}")
        if not ok:
            all_ok = False

    print()
    if all_ok:
        print("  ✓  ALL PASS — ReGraspReflex 검증 완료")
    else:
        print("  ✗  SOME FAILED")
    print()


if __name__ == "__main__":
    main()
