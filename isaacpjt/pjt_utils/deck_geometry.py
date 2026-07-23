"""IW 데크와 표준 팔레트의 수직 정렬 계산.

USD/PhysX에 의존하지 않는 계산만 이 모듈에 둬서 Isaac Sim 밖에서도 검증한다.
"""
from __future__ import annotations

import math


# Isaac 5.1 pallet.usd 메시에서 측정한 포크 채널의 로컬 Z 범위.
PALLET_HOLE_BOTTOM_Z = 0.02053
PALLET_HOLE_TOP_Z = 0.11605
PALLET_HOLE_CENTER_Z = (PALLET_HOLE_BOTTOM_Z + PALLET_HOLE_TOP_Z) / 2.0

# 접촉 솔버가 팔레트와 데크를 관통 상태로 시작하지 않도록 주는 최소 간격.
PALLET_SUPPORT_CLEARANCE = 0.002


def supported_pallet_origin_z(
    deck_top_z: float,
    pallet_local_min_z: float,
    clearance: float = PALLET_SUPPORT_CLEARANCE,
) -> float:
    """팔레트 bbox 하면을 데크 상면 바로 위에 놓는 팔레트 원점 Z."""
    values = (deck_top_z, pallet_local_min_z, clearance)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("데크/팔레트 높이는 유한한 값이어야 합니다")
    if clearance < 0.0:
        raise ValueError("팔레트 지지면 간격은 0 이상이어야 합니다")
    return deck_top_z + clearance - pallet_local_min_z


def supported_pallet_hole_center_z(
    deck_top_z: float,
    pallet_local_min_z: float = 0.0,
    clearance: float = PALLET_SUPPORT_CLEARANCE,
) -> float:
    """데크에 안착한 pallet.usd의 포크 채널 중심 월드 Z."""
    return (
        supported_pallet_origin_z(
            deck_top_z,
            pallet_local_min_z,
            clearance,
        )
        + PALLET_HOLE_CENTER_Z
    )
