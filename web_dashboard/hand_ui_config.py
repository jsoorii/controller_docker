"""
핸드/그리퍼 웹 UI 설정 모듈
─────────────────────────────
핸드를 교체할 때는 ACTIVE 한 줄만 바꾸면 됩니다.
main.py와 index.html은 수정하지 않아도 됩니다.

control type:
  - "slider" : 슬라이더 + 값 표시, oninput 시 ROS topic publish (debounce 150ms)

preset: 버튼 클릭 시 해당 group의 모든 control 값을 설정하고 즉시 전송
"""

# ── Robotiq 2F-85 (v3/v4) ────────────────────────────────────────────────────

ROBOTIQ_2F85 = {
    "name": "Robotiq 2F-85",
    "enabled": True,
    "groups": [
        {
            "label": "그리퍼",
            "controls": [
                {
                    "id":       "gripper",
                    "label":    "개폐 비율",
                    "unit":     "0 = 열림  /  1 = 닫힘",
                    "type":     "slider",
                    "min":      0.0,
                    "max":      1.0,
                    "step":     0.01,
                    "default":  0.0,
                    "topic":    "/ur5e/cmd/gripper",
                    "msg_type": "std_msgs/msg/Float64",
                    "msg_field":"data",
                },
            ],
            "presets": [
                {"label": "열기",  "values": {"gripper": 0.00}},
                {"label": "반개",  "values": {"gripper": 0.50}},
                {"label": "닫기",  "values": {"gripper": 1.00}},
            ],
        },
    ],
}


# ── Orca Hand (17-DOF) ───────────────────────────────────────────────────────
# 아직 ROS 토픽 미정 — 추후 채워 넣기

ORCA_HAND = {
    "name": "Orca Hand",
    "enabled": False,   # ← 활성화 전까지 False 유지
    "groups": [
        {
            "label": "손목",
            "controls": [
                {
                    "id": "wrist", "label": "손목", "unit": "rad",
                    "type": "slider", "min": -1.13, "max": 0.61, "step": 0.01, "default": 0.0,
                    "topic": "/hand/cmd/joint", "msg_type": "std_msgs/msg/Float64", "msg_field": "data",
                },
            ],
            "presets": [],
        },
        {
            "label": "엄지",
            "controls": [
                {"id": "t_cmc", "label": "CMC",  "unit": "rad", "type": "slider", "min": -0.79, "max": 0.58, "step": 0.01, "default": 0.0, "topic": "/hand/cmd/t_cmc",  "msg_type": "std_msgs/msg/Float64", "msg_field": "data"},
                {"id": "t_abd", "label": "ABD",  "unit": "rad", "type": "slider", "min": -0.31, "max": 0.96, "step": 0.01, "default": 0.0, "topic": "/hand/cmd/t_abd",  "msg_type": "std_msgs/msg/Float64", "msg_field": "data"},
                {"id": "t_mcp", "label": "MCP",  "unit": "rad", "type": "slider", "min": -0.44, "max": 1.75, "step": 0.01, "default": 0.0, "topic": "/hand/cmd/t_mcp",  "msg_type": "std_msgs/msg/Float64", "msg_field": "data"},
                {"id": "t_pip", "label": "PIP",  "unit": "rad", "type": "slider", "min": -0.26, "max": 1.87, "step": 0.01, "default": 0.0, "topic": "/hand/cmd/t_pip",  "msg_type": "std_msgs/msg/Float64", "msg_field": "data"},
            ],
            "presets": [],
        },
    ],
}


# ── 핸드 없음 ────────────────────────────────────────────────────────────────

NONE = {
    "name": "없음",
    "enabled": False,
    "groups": [],
}


# ── 현재 사용 중인 핸드: 여기만 바꾸면 됩니다 ─────────────────────────────

ACTIVE = ROBOTIQ_2F85
