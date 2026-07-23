# -*- coding: utf-8 -*-
"""Isaac 에셋 경로 해석 — 후보를 순서대로 시도해서 먼저 잡히는 걸 쓴다.

에셋 루트를 얻는 API 이름과 로봇 경로가 Isaac 버전마다 바뀐다
(omni.isaac.* -> isaacsim.*). 하드코딩하면 버전 하나 올라갈 때 전부 죽는다.
verify.py / ros/graph.py 와 같은 후보 시도 패턴.

실제로 뭐가 있는지는 `spikes/03_asset_check.py` 가 확인한다.
"""
from __future__ import annotations

import os

from pxr import Usd

_ROOT_CANDIDATES = [
    ("isaacsim.storage.native", "get_assets_root_path"),
    ("omni.isaac.nucleus", "get_assets_root_path"),
    ("omni.isaac.core.utils.nucleus", "get_assets_root_path"),
]

_root_cache: str | None = None


def assets_root() -> str:
    """Isaac 에셋 루트. 한 번 찾으면 캐시한다."""
    global _root_cache
    if _root_cache is not None:
        return _root_cache

    for mod_name, fn_name in _ROOT_CANDIDATES:
        try:
            mod = __import__(mod_name, fromlist=[fn_name])
            root = getattr(mod, fn_name)()
            if root:
                _root_cache = root
                return root
        except Exception:
            continue

    raise RuntimeError(
        "Isaac 에셋 루트를 못 찾음. 시도한 후보: "
        f"{[m + '.' + f for m, f in _ROOT_CANDIDATES]}\n"
        "  -> Nucleus 연결 또는 로컬 에셋 설치를 확인할 것.")


def resolve(candidates: tuple[str, ...], what: str) -> str:
    """후보 중 실제로 열리는 첫 URL. 없으면 뭘 시도했는지 알려주고 죽는다."""
    root = assets_root()
    tried = []
    for rel in candidates:
        # 로컬 절대경로(워크스페이스 반입 에셋, 예: m0617.usd)는 에셋 루트를 붙이지
        # 않고 그대로 연다. 그 외는 Isaac 서버 기준 상대경로로 본다.
        url = rel if (os.path.isabs(rel) and os.path.exists(rel)) else root + rel
        tried.append(url)
        try:
            if Usd.Stage.Open(url) is not None:
                return url
        except Exception:
            continue

    raise FileNotFoundError(
        f"{what} 에셋을 못 찾음. 시도한 경로:\n  " + "\n  ".join(tried) +
        "\n  -> spikes/03_asset_check.py 로 실제 경로를 확인하고 "
        "config/settings.py 의 RobotAssetConfig 에 추가할 것.")
