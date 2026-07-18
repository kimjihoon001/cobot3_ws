# -*- coding: utf-8 -*-
"""씬 테스트용 공용 픽스처 — isaacsim 모킹 + 가짜 에셋.

이 디렉터리는 CLAUDE.md 의 "Never run isaac/ code under python3" 에 대한
명시적 예외다. 규칙의 취지는 '실수로 Isaac 코드를 3.10 으로 돌리지 말 것'인데,
씬 코드가 실제로 건드리는 Isaac API 는 add_reference_to_stage 하나뿐이고
나머지는 순수 pxr(usd-core) 이라 그 하나만 모킹하면 GPU 없이 씬이 그대로 지어진다.

이게 있어야 하는 이유:
  Isaac 은 GPU 노트북에만 있고 그건 교육장에서만 만진다. 그 사이 씬 코드를
  고치면 검증할 방법이 없었다. 이 테스트는 GPU 없이 도는 안전망이다.

여기서 검증되는 것 : USD 구조 (콜라이더/RigidBody/밀도/스케일/배치/재현성)
여기서 안 되는 것   : 렌더링, 물리 시뮬 실제 거동, PhysX 질량 계산 결과
                     -> 그건 GPU 머신에서 verify.py

실행: pytest isaacpjt/tests/        (일반 python3. isaac_python 아님)
"""
import os
import random
import sys
import types

import pytest
from pxr import Usd, UsdGeom, Gf

ISAAC_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ISAAC_DIR not in sys.path:
    sys.path.insert(0, ISAAC_DIR)


# ---------------------------------------------------------------
# isaacsim 모킹
# ---------------------------------------------------------------
# 실제 add_reference_to_stage 는 omni.usd 의 현재 전역 스테이지에 참조를 붙인다.
# 테스트는 스테이지를 직접 만들므로 같은 '현재 스테이지' 개념을 흉내낸다.
_current: dict = {"stage": None}


def _add_reference_to_stage(usd_path: str, prim_path: str) -> Usd.Prim:
    stage = _current["stage"]
    assert stage is not None, "use_stage() 로 현재 스테이지를 먼저 지정할 것"
    prim = stage.DefinePrim(prim_path)
    prim.GetReferences().AddReference(usd_path)
    return prim


def _install_isaac_mock() -> None:
    for name in ("isaacsim", "isaacsim.core", "isaacsim.core.utils",
                 "isaacsim.core.utils.stage"):
        sys.modules.setdefault(name, types.ModuleType(name))
    sys.modules["isaacsim.core.utils.stage"].add_reference_to_stage = (
        _add_reference_to_stage)


_install_isaac_mock()


def _use_stage(stage: Usd.Stage) -> None:
    """이후 add_reference_to_stage 가 이 스테이지에 붙게 한다."""
    _current["stage"] = stage


# ---------------------------------------------------------------
# 픽스처
# ---------------------------------------------------------------
@pytest.fixture(scope="session")
def fake_asset_dir(tmp_path_factory) -> str:
    """토마토 USD 대역 — 닫힌 메시(8면체)면 구조 검증엔 충분하다.

    진짜 에셋(tomato_assets_usd)은 Isaac 의 변환기가 있어야 만들어지므로
    이 머신엔 없다. 여기서 보는 건 모양이 아니라 USD 구조다.
    """
    d = tmp_path_factory.mktemp("fake_usd")
    r = 20.0   # mm. 에셋과 같은 스케일 (generate_tomatoes.py BASE_R)
    pts = [(r, 0, 0), (-r, 0, 0), (0, r, 0), (0, -r, 0), (0, 0, r), (0, 0, -r)]
    faces = [(0, 2, 4), (2, 1, 4), (1, 3, 4), (3, 0, 4),
             (2, 0, 5), (1, 2, 5), (3, 1, 5), (0, 3, 5)]

    for name in ("tomato_ripe_01", "tomato_ripe_01_calyx"):
        stage = Usd.Stage.CreateNew(str(d / (name + ".usd")))
        mesh = UsdGeom.Mesh.Define(stage, "/Body")
        mesh.CreatePointsAttr([Gf.Vec3f(*p) for p in pts])
        mesh.CreateFaceVertexCountsAttr([3] * len(faces))
        mesh.CreateFaceVertexIndicesAttr([i for f in faces for i in f])
        # 이걸 빼면 참조가 조용히 실패한다 (에러가 아니라 경고만 뜸)
        stage.SetDefaultPrim(mesh.GetPrim())
        stage.GetRootLayer().Save()
    return str(d)


@pytest.fixture
def cfg(fake_asset_dir):
    from pjt_config.settings import SceneConfig
    c = SceneConfig()
    c.tomato_assets.usd_dir = fake_asset_dir
    return c


@pytest.fixture
def new_stage():
    """빈 스테이지를 만들고 '현재 스테이지'로 등록하는 팩토리.

    팩토리인 이유: 재현성 테스트가 스테이지를 두 개 지어서 비벼야 한다.
    (site-packages 에 이미 tests 패키지가 있어 conftest 를 직접 import 하면
     가려진다. 그래서 픽스처로 넘긴다.)
    """
    def _make() -> Usd.Stage:
        s = Usd.Stage.CreateInMemory()
        UsdGeom.Xform.Define(s, "/World")
        _use_stage(s)
        return s
    return _make


@pytest.fixture
def stage(new_stage) -> Usd.Stage:
    return new_stage()


@pytest.fixture
def spawn_plants(cfg, new_stage):
    """(스테이지, 시드) -> 스폰까지 끝난 TomatoPlants 팩토리."""
    from scene.tomato_plants import TomatoPlants

    def _spawn(stage: Usd.Stage, seed: int | None = None) -> "TomatoPlants":
        p = TomatoPlants(cfg.plants, cfg.tomato_assets, cfg.greenhouse,
                         cfg.physics, random.Random(cfg.seed if seed is None else seed))
        p.spawn(stage)
        return p
    return _spawn


@pytest.fixture
def plants(stage, spawn_plants):
    """기본 시드로 스폰까지 끝난 TomatoPlants."""
    return spawn_plants(stage)
