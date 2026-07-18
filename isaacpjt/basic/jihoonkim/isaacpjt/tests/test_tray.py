# -*- coding: utf-8 -*-
"""트레이 **지오메트리** — GPU 없이 돈다.

적재량 추적·만재 판정은 여기 없다. `tray_manager_node` (ROS2, 개인 PC) 파트다.
Isaac 은 칸 좌표를 내주고 시키는 대로 놓을 뿐이다 (CLAUDE.md 5.6).
"""
import pytest
from pxr import Usd, UsdGeom, UsdPhysics

from pjt_config.settings import TrayConfig
from scene.tray import Tray


@pytest.fixture
def stage():
    s = Usd.Stage.CreateInMemory()
    UsdGeom.Xform.Define(s, "/World")
    return s


def make(stage, **kw):
    t = Tray(TrayConfig(**kw))
    t.spawn(stage, "/World/Tray", (0.0, 0.0, 0.8), log=lambda *_: None)
    return t


def test_칸이_capacity_만큼_생긴다(stage):
    t = make(stage)
    assert len(t.cells) == 6
    for c in t.cells:
        assert stage.GetPrimAtPath(c["path"]).IsValid()


def test_칸마다_좌표가_다르다(stage):
    """겹치면 과실을 같은 자리에 쌓는다."""
    t = make(stage)
    pos = [c["position"] for c in t.cells]
    assert len(set(pos)) == len(pos)


def test_칸_간격이_과실_지름보다_크다(stage):
    """68.7mm 과실이 들어가야 한다. 좁으면 낀다."""
    t = make(stage)
    xs = sorted({c["position"][0] for c in t.cells})
    gap = min(b - a for a, b in zip(xs, xs[1:]))
    assert gap > 0.0687, f"칸 간격 {gap*1000:.1f}mm < 과실 지름 68.7mm"


def test_칸_인덱스가_0부터_연속이다(stage):
    """ROS2 가 '3번 칸에 놓아라' 를 보내면 그대로 찾을 수 있어야 한다."""
    t = make(stage)
    assert [c["index"] for c in t.cells] == list(range(6))


def test_트레이가_콜라이더를_갖는다(stage):
    """과실이 통과하면 담기지 않는다."""
    t = make(stage)
    root = stage.GetPrimAtPath(t.root)
    n = sum(1 for p in Usd.PrimRange(root) if p.HasAPI(UsdPhysics.CollisionAPI))
    assert n > 0


def test_트레이는_강체가_아니다(stage):
    """AMR 포크가 들기 전까진 static. RigidBody 면 바닥으로 떨어진다."""
    t = make(stage)
    root = stage.GetPrimAtPath(t.root)
    for p in Usd.PrimRange(root):
        assert not p.HasAPI(UsdPhysics.RigidBodyAPI)


def test_Isaac_쪽에_상태_로직이_없다():
    """CLAUDE.md 5.6 — 결정과 상태는 ROS2. 여기 되살아나면 3PC 구조가 무너진다."""
    for banned in ("count", "is_full", "put", "next_free_cell", "clear"):
        assert not hasattr(Tray, banned), (
            f"Tray.{banned} 는 tray_manager_node(ROS2) 파트다")
