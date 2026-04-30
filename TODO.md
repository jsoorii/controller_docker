# TODO

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
