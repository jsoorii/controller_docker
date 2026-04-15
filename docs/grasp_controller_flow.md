# Grasp Controller 제어 흐름도

## 1. 시스템 개요

```
┌──────────────────────────────────────────────────────────────────┐
│                        GraspController                           │
│                                                                  │
│  ┌─────────────────────┐      ┌──────────────────────────────┐  │
│  │   기본 태스크 (보통)  │      │   RT 태스크 (최고 우선순위)   │  │
│  │                     │      │   rt7 / 500 Hz (2 ms)        │  │
│  │ write_command()     │      │                              │  │
│  │ read_state()        │◄────►│   controller_run()           │  │
│  │ read_all_states()   │      │   _run_control_step()        │  │
│  └─────────────────────┘      └──────────────────────────────┘  │
└──────────────────────────────────────────────────────────────────┘
```

---

## 2. 상태머신 전이도

```
                    ┌─────────────┐
         시작        │             │
     ───────────►   │ WAIT_STABLE │  0.1 s 대기
                    │             │
                    └──────┬──────┘
                           │ 안정화 완료
                           ▼
                    ┌─────────────┐
                    │             │
                    │INIT_CONTROL │  MotorState / FingerState 메모리 초기화
                    │             │  miss_count = 0
                    └──────┬──────┘
                           │ 초기화 완료
                           ▼
                    ┌─────────────┐
                    │             │  모터 ping / heartbeat
                    │CHECK_MOTOR  │◄──────────┐
                    │             │           │ 연결 실패 → 0.5 s 후 재시도
                    └──────┬──────┘───────────┘
                           │ 연결 확인
                           ▼
                    ┌─────────────┐
                    │             │  end-stop 폴링 → 엔코더 영점화
                    │INIT_POSITION│  _home_q 저장
                    │             │  q_target ← home_q
                    └──────┬──────┘
                           │ 홈 완료
                           ▼
                    ┌─────────────┐
                    │             │  ← 2 ms 주기 반복 루프 ─────────┐
                    │ RUN_CONTROL │                                   │
                    │             │──────────────────────────────────┘
                    └─────────────┘
```

---

## 3. RUN_CONTROL 루프 상세 흐름

```
─────────────────────── 500 Hz 주기 (2 ms) ───────────────────────

  ┌─────────────────────────────────────────────────────────────┐
  │  _run_control_step(command)                                 │
  │                                                             │
  │  Step 1 │ joint state 읽기                                  │
  │         │   _step_read_joint_state()                        │
  │         │                                                   │
  │         │   [실하드웨어]          [시뮬 모드]                │
  │         │   motor_iface.read_state()   cmd → state (10%)   │
  │         │   폴링 재시도 (max 20회)                           │
  │         │   초과 시 miss_count++                             │
  │         │                                                   │
  │         ▼                                                   │
  │  Step 2 │ 4절 링크 FK   (손가락 × 3)                        │
  │         │   FourBarLinkage.fk(q_motor[0])                  │
  │         │     Freudenstein 방정식 풀이                       │
  │         │     → q_proximal (출력 링크 각도)                  │
  │         │                                                   │
  │         ▼                                                   │
  │  Step 3 │ 손가락 FK + 자코비안   (손가락 × 3)               │
  │         │   FingerKinematics.fk(q_finger)                  │
  │         │     평면 3R 체인 → 끝점 위치 p, 회전 R             │
  │         │     6×3 자코비안 J 계산                            │
  │         │       J[:3, i] = z × (p_e − p_joint_i)  선속도   │
  │         │       J[3:, i] = z                       각속도   │
  │         │                                                   │
  │         ▼                                                   │
  │  Step 4 │ 토크 명령 계산   controller_run()                 │
  │         │                                                   │
  │         │   ┌─────────────────────────────────────────┐     │
  │         │   │  PD 관절 토크                            │     │
  │         │   │    err  = q_target − q                  │     │
  │         │   │    derr = −qdot                         │     │
  │         │   │    τ_pd = Kp·err + Kd·derr             │     │
  │         │   │    Kp = 200, Kd = 20 (관절별)           │     │
  │         │   └──────────────┬──────────────────────────┘     │
  │         │                  │                                 │
  │         │          force_target > 0?                         │
  │         │         Yes ▼           No ▼                       │
  │         │   ┌───────────────┐  ┌────────────────────┐        │
  │         │   │ 자코비안 토크  │  │  τ = τ_pd          │        │
  │         │   │ F=[f,0,0,...] │  └────────────────────┘        │
  │         │   │ τ_f = Jᵀ · F │                                 │
  │         │   │ τ = τ_pd+τ_f │                                 │
  │         │   └───────────────┘                                │
  │         │                                                   │
  │         ▼                                                   │
  │  Step 5 │ 명령 전송                                          │
  │         │   motor_cmd[fi].current = torque[fi]              │
  │         │   _step_write_motor_command()                     │
  │         │     motor_iface.write_command(fi, q, qdot, cur)   │
  │         │                                                   │
  │         ▼                                                   │
  │  Step 6 │ 상태 버퍼 업데이트 (외부 read용)                   │
  │         │   _state_buf["q/qdot/current"] ← motor_state      │
  │                                                             │
  └─────────────────────────────────────────────────────────────┘
```

---

## 4. 기구학 계산 흐름

```
모터 각도 (q_motor)
       │
       ▼
┌────────────────────┐
│   FourBarLinkage   │   Freudenstein 방정식
│                    │
│  FK:  θ2 → θ4     │   K1·cos(θ4) − K2·cos(θ2) + K3 = cos(θ2−θ4)
│  IK:  θ4 → θ2     │   반각 치환 → 이차방정식 → θ4
│  vel: dθ4/dθ2     │   IK: 이분법 (200 샘플 탐색, 50회 반복)
│                    │
│  L1=40mm L2=15mm  │
│  L3=35mm L4=25mm  │
└────────┬───────────┘
         │  q_proximal (근위부 각도)
         │  q_finger = [q_proximal, q_motor[1], q_motor[2]]
         ▼
┌────────────────────┐
│  FingerKinematics  │   평면 3R 체인
│                    │
│  FK: q(3) →       │   p0 → p1 → p2 → p3 (끝점)
│    FingerState     │   누적 각도: a1, a1+a2, a1+a2+a3
│      p : 끝점 위치 │
│      R : 회전행렬  │   base_R, base_p 로 로봇 좌표 변환
│      J : 6×3 자코  │
│                    │   링크 길이: l1=40mm l2=30mm l3=25mm
│  force_to_torque:  │
│    τ = Jᵀ · F     │
└────────────────────┘
```

---

## 5. 스레딩 구조 및 데이터 흐름

```
외부 (ROS / Web UI 등)
        │
        │  write_command(finger_idx, q, qdot, current, force)
        │                                    ┌─── 뮤텍스 보호 (_lock)
        ▼                                    │
  ┌─────────────────┐                        │
  │   _cmd_buf      │◄───────────────────────┘
  │  q_target       │
  │  vel_target     │
  │  cur_target     │
  │  force_target   │
  └────────┬────────┘
           │  RUN_CONTROL 진입 시 복사 (snapshot)
           ▼
  ┌─────────────────┐    _run_control_step()    ┌───────────────┐
  │  RT 제어루프     │ ─────────────────────────►│  motor_iface  │
  │  (500 Hz)       │◄─────────────────────────  │  read/write   │
  └────────┬────────┘                            └───────────────┘
           │
           │  상태 버퍼 업데이트 (_lock)
           ▼
  ┌─────────────────┐
  │   _state_buf    │
  │  q / qdot /     │
  │  current        │
  └────────┬────────┘
           │  read_state(finger_idx)
           ▼
        외부 (응답 패킷 송신)
```

---

## 6. 데이터 구조 요약

| 구조체 | 필드 | 크기 | 설명 |
|--------|------|------|------|
| `MotorState` | q, qdot, current | (3,) each | 모터 측정값 |
| `MotorCommand` | q, qdot, current | (3,) each | 모터 명령값 |
| `FingerState` | p, R, J | (3,), (3×3), (6×3) | FK 결과 |
| `FingerCommand` | p_target, force_target, q_target | (3,), float, (3,) | 손가락 목표 |

---

## 7. 타이밍 요약

| 항목 | 값 |
|------|-----|
| 제어루프 주기 | 2 ms (500 Hz) |
| 모터 폴링 재시도 최대 | 20회 |
| 1회 재시도 대기 | dt × 0.1 = 0.2 ms |
| WAIT_STABLE 대기 | 100 ms |
| CHECK_MOTOR 실패 재시도 | 500 ms |
