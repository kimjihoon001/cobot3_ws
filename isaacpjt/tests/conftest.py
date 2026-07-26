"""Isaac 런타임 의존성 stub.

`iw.py` / `mm.py`는 모듈 로드 시점에 isaacsim과 여러 프로젝트 모듈을 import한다.
드라이버의 순수 로직(슬롯 상태, 릴리즈 이벤트)을 Isaac 없이 검증하기 위해 그
의존성만 최소로 채운다. 실제 검증 대상 코드는 stub 하지 않는다.

`pjt_utils.*`는 순수 계산 모듈이라 건드리지 않는다(기존 테스트가 실물을 쓴다).
"""
import sys
import types


def _module(name: str, **attrs) -> types.ModuleType:
    module = types.ModuleType(name)
    for key, value in attrs.items():
        setattr(module, key, value)
    return module


def _package(name: str) -> types.ModuleType:
    module = types.ModuleType(name)
    module.__path__ = []
    return module


class _Driver:
    def __init__(self):
        pass


class _Stub:
    """생성 인자를 무엇으로 받든 통과시키는 자리표시자."""

    def __init__(self, *args, **kwargs):
        pass


def _install() -> None:
    sys.modules.setdefault(
        "robot_base", _module("robot_base", Driver=_Driver,
                              ros_fail=lambda *a, **k: None))

    sys.modules.setdefault("robots", _package("robots"))
    sys.modules.setdefault(
        "robots.iwhub", _module("robots.iwhub", IwHub=_Stub))
    sys.modules.setdefault(
        "robots.harvester_mm",
        _module("robots.harvester_mm", HarvestMM=_Stub, HOME_POSE_DEG=()))

    sys.modules.setdefault("scene", _package("scene"))
    sys.modules.setdefault(
        "scene.ground", _module("scene.ground", COMMON_FLOOR_Z=0.0))

    sys.modules.setdefault(
        "iw_dock", _module("iw_dock", WarehouseDockController=_Stub))

    sys.modules.setdefault(
        "pjt_config.settings_mm",
        _module("pjt_config.settings_mm",
                mm_robot_config=lambda cfg: cfg))

    isaacsim = _package("isaacsim")
    sys.modules.setdefault("isaacsim", isaacsim)
    sys.modules.setdefault("isaacsim.core", _package("isaacsim.core"))
    sys.modules.setdefault(
        "isaacsim.core.utils", _package("isaacsim.core.utils"))
    sys.modules.setdefault(
        "isaacsim.core.utils.types",
        _module("isaacsim.core.utils.types", ArticulationAction=_Stub))


_install()
