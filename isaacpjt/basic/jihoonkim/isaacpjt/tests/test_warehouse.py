# -*- coding: utf-8 -*-
"""창고 슬롯 **지오메트리** — GPU 없이 돈다.

슬롯 할당·하역 기록은 여기 없다. warehouse_manager_node (ROS2) 파트 (CLAUDE.md 5.6).

1:1 매핑이 깨지면 v3 10장이 걱정한 "창고 적재 위치 결정 로직 복잡도" 가 되살아난다.
여기서 못박는다.
"""
import pytest
from pxr import Usd, UsdGeom, UsdPhysics

from pjt_config.settings import WarehouseConfig
from scene.warehouse import Warehouse


@pytest.fixture
def stage():
    s = Usd.Stage.CreateInMemory()
    UsdGeom.Xform.Define(s, "/World")
    return s


def make(stage, sectors=6, **kw):
    cfg = WarehouseConfig(level_height=0.5, slot_pitch=0.7, **kw)
    w = Warehouse(cfg, sector_count=sectors)
    w.spawn(stage, "/World/Warehouse", (5.0, 0.0, 0.0), log=lambda *_: None)
    return w


def test_슬롯이_섹터와_1대1이다(stage):
    w = make(stage)
    assert len(w.slots) == 6
    assert {s["sector"] for s in w.slots} == set(range(6))


def test_슬롯_수와_섹터_수가_다르면_거부한다():
    """매핑이 깨지면 배치가 탐색 문제가 된다 (v3 10장)."""
    cfg = WarehouseConfig(level_height=0.5, slot_pitch=0.7)   # 3x2 = 6슬롯
    with pytest.raises(ValueError, match="1:1"):
        Warehouse(cfg, sector_count=4)


def test_슬롯마다_섹터_번호가_붙는다(stage):
    """매핑 규칙은 ROS2(warehouse_manager_node)가 쓴다. Isaac 은 라벨만 붙인다."""
    w = make(stage)
    assert sorted(s["sector"] for s in w.slots) == list(range(6))


def test_Isaac_쪽에_할당_로직이_없다():
    """CLAUDE.md 5.6 — 슬롯 할당은 warehouse_manager_node(ROS2) 파트."""
    for banned in ("slot_for_sector", "allocate", "assign"):
        assert not hasattr(Warehouse, banned), (
            f"Warehouse.{banned} 는 warehouse_manager_node(ROS2) 파트다")


def test_2단으로_나뉜다(stage):
    """1단으로 깔면 포크 승강이 필요 없어져 지게차를 고른 이유가 무너진다."""
    w = make(stage)
    levels = {s["level"] for s in w.slots}
    assert levels == {0, 1}
    assert sum(1 for s in w.slots if s["level"] == 1) == 3


def test_2단_슬롯이_실제로_더_높다(stage):
    w = make(stage)
    lo = [s["position"][2] for s in w.slots if s["level"] == 0]
    hi = [s["position"][2] for s in w.slots if s["level"] == 1]
    assert min(hi) > max(lo)


def test_슬롯_좌표가_전부_다르다(stage):
    w = make(stage)
    pos = [s["position"] for s in w.slots]
    assert len(set(pos)) == len(pos)


def test_치수가_미정이면_멈춘다(stage):
    """임의로 채우면 근거 없는 값([4])을 조용히 늘리는 것이다.

    기본값은 2026-07-18 GPU 실측(포크 승강 한계) 이후 채워졌다.
    None 가드 자체는 여전히 살아 있어야 한다 — 명시적으로 None 을 넣어 검증.
    """
    w = Warehouse(WarehouseConfig(level_height=None, slot_pitch=None),
                  sector_count=6)
    with pytest.raises(ValueError, match="level_height"):
        w.spawn(stage, "/World/W2", log=lambda *_: None)


def test_포크_삽입_가이드가_있다(stage):
    """v3 10장이 '포크 삽입 정렬 오차'를 새 리스크로 지목했다."""
    w = make(stage)
    slot0 = stage.GetPrimAtPath(w.slots[0]["path"])
    guides = [p for p in Usd.PrimRange(slot0) if "Guide" in p.GetName()]
    assert len(guides) == 2
    for g in guides:
        assert g.HasAPI(UsdPhysics.CollisionAPI)


def test_슬롯이_콜라이더를_갖는다(stage):
    """트레이가 통과하면 얹히지 않는다."""
    w = make(stage)
    for s in w.slots:
        assert stage.GetPrimAtPath(s["path"]).HasAPI(UsdPhysics.CollisionAPI)
