import numpy as np
import time
import mujoco
from enum import Enum

class ControlState(Enum):
    WAIT_STABLE   = 0
    INIT_CONTROL  = 1
    CHECK_MOTOR   = 2
    INIT_POSITION = 3
    RUN_CONTROL   = 4

class UR5eRTController:
    def __init__(self, model, data):
        self.model = model
        self.data = data
        self.state = ControlState.WAIT_STABLE
        
        # 데이터 저장 (SRAM 역할)
        self.motor_state = {"q": np.zeros(6), "dq": np.zeros(6)}
        self.motor_cmd   = {"q_target": np.zeros(6)}
        
        self.loop_hz = 500  # 2ms 주기
        self.dt = 1.0 / self.loop_hz
        self.miss_count = 0

    def controller_run(self):
        """[RT-7] 최우선 제어 루프"""
        # 1. 현재 상태 읽기
        self.motor_state["q"] = self.data.qpos[:6].copy()
        self.motor_state["dq"] = self.data.qvel[:6].copy()

        # 2. PD 제어 계산 (간단한 위치 제어 예시)
        # MuJoCo XML의 actuator가 position 타입이면 직접 입력, 
        # motor(torque) 타입이면 아래처럼 직접 계산해서 넣습니다.
        q_des = self.motor_cmd["q_target"]
        kp = 500.0
        kd = 50.0
        
        # 토크 계산: tau = kp*(error) + kd*(dot_error)
        tau_out = kp * (q_des - self.motor_state["q"]) + kd * (0.0 - self.motor_state["dq"])
        
        # 3. 명령 송신
        self.data.ctrl[:6] = tau_out

    def start(self):
        print(f"\n[Controller] 시작 - 현재 상태: {self.state.name}")
        
        while True:
            t_start = time.perf_counter()

            # --- State Machine ---
            if self.state == ControlState.WAIT_STABLE:
                time.sleep(1.0)
                self.state = ControlState.INIT_CONTROL

            elif self.state == ControlState.INIT_CONTROL:
                # 현재 위치를 목표 위치로 초기화하여 갑작스러운 움직임 방지
                self.motor_cmd["q_target"] = self.data.qpos[:6].copy()
                self.state = ControlState.CHECK_MOTOR

            elif self.state == ControlState.CHECK_MOTOR:
                self.state = ControlState.INIT_POSITION

            elif self.state == ControlState.INIT_POSITION:
                self.state = ControlState.RUN_CONTROL

            elif self.state == ControlState.RUN_CONTROL:
                # 1. 제어 계산
                self.controller_run()
                # 2. 물리 엔진 스텝 전진 (이게 있어야 Joint 값이 변함!)
                mujoco.mj_step(self.model, self.data)

            # --- 2ms 주기 유지 ---
            t_elapsed = time.perf_counter() - t_start
            t_sleep = self.dt - t_elapsed
            if t_sleep > 0:
                time.sleep(t_sleep)
            else:
                self.miss_count += 1