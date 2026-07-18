# -*- coding: utf-8 -*-
"""씬 USD 구조 검증 — GPU 없이 도는 안전망. 배경은 conftest.py 참고."""
import pytest
from pxr import Usd, UsdGeom, UsdPhysics

from scene.greenhouse import Greenhouse


# ---------------------------------------------------------------
# 과실 물리 — 질량을 상수로 박으면 안 된다
# ---------------------------------------------------------------
def test_과실은_질량이_아니라_밀도로_설정된다(plants, stage, cfg):
    """질량 상수 금지 회귀 테스트.

    메시 부피가 변형별로 6배 차이나므로(4.56 ~ 27.46 cm^3) 질량을 상수로 박으면
    작은 과실의 밀도가 5.6 g/cm^3 (알루미늄 2배) 이 된다. 질량은 PhysX 가
    콜라이더 부피 x 밀도로 과실마다 계산해야 한다.
    """
    for f in plants.fruits:
        api = UsdPhysics.MassAPI(stage.GetPrimAtPath(f["path"]))
        assert api.GetDensityAttr().Get() == cfg.physics.fruit_density
        # HasAttribute 는 스키마 정의만 있어도 True -> authored 여부로 봐야 한다
        assert not api.GetMassAttr().HasAuthoredValue(), (
            "physics:mass 가 박혀 있으면 밀도가 무시된다: " + f["path"])


def test_과실마다_콜라이더가_붙는다(plants, stage, cfg):
    """콜라이더가 없으면 PhysX 가 부피를 몰라 밀도로 질량을 못 구한다."""
    for f in plants.fruits:
        prim = stage.GetPrimAtPath(f["path"])
        meshes = [p for p in Usd.PrimRange(prim) if p.IsA(UsdGeom.Mesh)]
        assert meshes, "참조가 안 붙었다 (에셋 USD 의 defaultPrim 확인)"
        assert any(p.HasAPI(UsdPhysics.CollisionAPI) for p in meshes), (
            "콜라이더 없음: " + f["path"])


def test_과실_콜라이더는_트라이앵글메시가_아니다(plants, stage, cfg):
    """트라이앵글 메시 충돌은 성능상 금지. convexHull 로 근사한다."""
    for f in plants.fruits:
        for p in Usd.PrimRange(stage.GetPrimAtPath(f["path"])):
            if p.HasAPI(UsdPhysics.MeshCollisionAPI):
                approx = UsdPhysics.MeshCollisionAPI(p).GetApproximationAttr().Get()
                assert approx == cfg.physics.fruit_approximation
                assert approx != "none", "트라이앵글 메시 충돌"


def test_매달린_과실은_kinematic이다(plants, stage):
    """kinematic 이라야 물리 솔버가 매 스텝 안 풀고, Play/Stop 에도 제자리."""
    for f in plants.fruits:
        rb = UsdPhysics.RigidBodyAPI(stage.GetPrimAtPath(f["path"]))
        assert rb.GetKinematicEnabledAttr().Get() is True


# ---------------------------------------------------------------
# 배치
# ---------------------------------------------------------------
def test_과실_높이가_수확_구간_안에_있다(plants, cfg):
    """사람 무릎~어깨 높이. 이 범위를 벗어나면 로봇이 못 딴다."""
    lo, hi = cfg.plants.fruit_height_range
    for f in plants.fruits:
        assert lo <= f["position"][2] <= hi


def test_스케일이_적용된다(plants, stage, cfg):
    """FreeCAD mm -> m. 이게 빠지면 지름 40m 짜리 토마토가 된다."""
    prim = stage.GetPrimAtPath(plants.fruits[0]["path"])
    ops = {o.GetOpName(): o.Get()
           for o in UsdGeom.Xformable(prim).GetOrderedXformOps()}
    # Gf.Vec3f 는 float32 라 정확히 같지 않다 (0.001675 -> 0.00167499994...)
    assert ops["xformOp:scale"][0] == pytest.approx(cfg.tomato_assets.scale,
                                                    rel=1e-6)


def test_익음_클래스는_설정된_것만_나온다(plants, cfg):
    assert {f["class_name"] for f in plants.fruits} <= set(cfg.plants.class_weights)


def test_fruits_목록이_실제_prim과_일치한다(plants, stage):
    """FSM/Detector 가 이 목록을 그대로 믿는다."""
    for f in plants.fruits:
        assert stage.GetPrimAtPath(f["path"]).IsValid()


# ---------------------------------------------------------------
# 재현성 — 디지털트윈 배점이 요구한다
# ---------------------------------------------------------------
def test_같은_시드면_같은_씬이_나온다(new_stage, spawn_plants):
    def build():
        p = spawn_plants(new_stage())
        return [(f["path"], f["class_name"], f["position"]) for f in p.fruits]

    assert build() == build()


def test_다른_시드면_다른_씬이_나온다(new_stage, spawn_plants):
    """시드가 실제로 먹히는지 — 위 테스트만으론 상수 씬도 통과한다."""
    def build(seed):
        return [f["position"] for f in spawn_plants(new_stage(), seed).fruits]

    assert build(1) != build(2)


# ---------------------------------------------------------------
# 온실 프레임 — static 이라야 로봇이 통과 못 한다
# ---------------------------------------------------------------
def test_온실_프레임은_콜라이더고_rigidbody가_아니다(cfg, stage):
    Greenhouse(cfg.greenhouse).spawn(stage)
    beams = [p for p in Usd.PrimRange(stage.GetPrimAtPath("/World/Greenhouse"))
             if p.IsA(UsdGeom.Cube)]
    assert beams, "프레임이 하나도 안 생김"
    for p in beams:
        assert p.HasAPI(UsdPhysics.CollisionAPI), "로봇이 통과해버린다"
        assert not p.HasAPI(UsdPhysics.RigidBodyAPI), "프레임이 무너진다"
