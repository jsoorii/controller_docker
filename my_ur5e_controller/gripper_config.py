"""
그리퍼 설정 모듈
────────────────
그리퍼를 교체할 때는 이 파일의 ACTIVE 변수 하나만 바꾸면 됩니다.
main_test.py와 ur5e_rt_controller.py는 수정하지 않아도 됩니다.

사용법:
    ACTIVE = ROBOTIQ_2F85_V4   # ← 여기만 바꾸면 교체 완료
"""

from dataclasses import dataclass
import mujoco


@dataclass
class GripperConfig:
    xml:        str    # MuJoCo MJCF XML 절대 경로
    actuator:   str    # 'gripper/' prefix 제외한 actuator 이름
    ctrl_open:  float  # 완전 열림 ctrl 값
    ctrl_close: float  # 완전 닫힘 ctrl 값


# ── 사용 가능한 그리퍼 프리셋 ───────────────────────────────────────────────

ROBOTIQ_2F85_V4 = GripperConfig(
    xml       = "/ros2_ws/src/mujoco_menagerie/robotiq_2f85_v4/2f85.xml",
    actuator  = "fingers_actuator",
    ctrl_open = 0.0,
    ctrl_close= 255.0,
)

ROBOTIQ_2F85 = GripperConfig(
    xml       = "/ros2_ws/src/mujoco_menagerie/robotiq_2f85/2f85.xml",
    actuator  = "fingers_actuator",
    ctrl_open = 0.0,
    ctrl_close= 255.0,
)

UMI_GRIPPER = GripperConfig(
    xml       = "/ros2_ws/src/mujoco_menagerie/umi_gripper/umi_gripper.xml",
    actuator  = "fingers_actuator",
    ctrl_open = 0.0,
    ctrl_close= 0.05,
)


# ── 현재 사용할 그리퍼: 여기 한 줄만 바꾸면 됩니다 ─────────────────────────

ACTIVE: GripperConfig | None = ROBOTIQ_2F85_V4


# ── 내부 함수 ────────────────────────────────────────────────────────────────

def attach(spec: mujoco.MjSpec, cfg: GripperConfig | None = None) -> None:
    """MjSpec에 그리퍼를 부착한다. cfg 생략 시 ACTIVE를 사용한다."""
    cfg = cfg if cfg is not None else ACTIVE
    if cfg is None:
        return
    gripper_spec = mujoco.MjSpec.from_file(cfg.xml)
    spec.attach(gripper_spec, prefix="gripper/", site="attachment_site")
