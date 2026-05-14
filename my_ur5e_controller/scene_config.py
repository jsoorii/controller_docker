"""
씬 오브젝트 설정 모듈
──────────────────────
환경 물체를 추가/수정할 때는 이 파일만 편집하면 됩니다.
main_test.py는 수정하지 않아도 됩니다.

사용법:
    OBJECTS 리스트에 원하는 오브젝트를 추가하면 됩니다.
"""

from dataclasses import dataclass, field
import mujoco


# ── 오브젝트 정의 ─────────────────────────────────────────────────────────────

@dataclass
class Box:
    name: str
    pos:  list          # [x, y, z] 미터
    size: list          # [x, y, z] half-size (실제 크기의 절반)
    rgba: list = field(default_factory=lambda: [0.8, 0.35, 0.1, 1.0])

@dataclass
class Sphere:
    name: str
    pos:  list          # [x, y, z] 미터
    size: float         # 반지름 (미터)
    rgba: list = field(default_factory=lambda: [0.2, 0.6, 0.9, 1.0])

@dataclass
class Cylinder:
    name: str
    pos:  list          # [x, y, z] 미터
    size: list          # [반지름, 절반높이]
    rgba: list = field(default_factory=lambda: [0.4, 0.8, 0.4, 1.0])


# ── 씬에 배치할 오브젝트 목록: 여기만 편집하면 됩니다 ────────────────────────

OBJECTS = [
    Cylinder(
        name = "cylinder",
        pos  = [0.3, 0, 0.1],      # 중심 z = 절반높이 → 바닥 위에 세움
        size = [0.1, 0.1],          # [반지름 10cm, 절반높이 10cm] → 직경 20cm / 높이 20cm
        rgba = [0.8, 0.15, 0.1, 1.0],
    ),
    # 추가 예시:
    # Box(name="box", pos=[0.3, 0, 0.015], size=[0.015, 0.015, 0.015]),
    # Sphere(name="ball", pos=[0.5, 0.2, 0.05], size=0.05),
]


# ── 내부 함수 ────────────────────────────────────────────────────────────────

def _add_object(spec: mujoco.MjSpec, obj) -> None:
    body      = spec.worldbody.add_body()
    body.name = obj.name
    body.pos  = obj.pos
    geom      = body.add_geom()
    geom.name = f"{obj.name}_geom"
    geom.rgba = obj.rgba

    if isinstance(obj, Box):
        geom.type = mujoco.mjtGeom.mjGEOM_BOX
        geom.size = obj.size
    elif isinstance(obj, Sphere):
        geom.type    = mujoco.mjtGeom.mjGEOM_SPHERE
        geom.size[0] = obj.size
    elif isinstance(obj, Cylinder):
        geom.type = mujoco.mjtGeom.mjGEOM_CYLINDER
        geom.size = [obj.size[0], obj.size[1], 0.0]  # MjSpec은 항상 3원소 요구


def add_objects(spec: mujoco.MjSpec) -> None:
    """OBJECTS 목록의 모든 오브젝트를 MjSpec에 추가한다."""
    for obj in OBJECTS:
        _add_object(spec, obj)
