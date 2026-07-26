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

# iw.hub의 articulation/root 원점과 실제 적재면 중심 사이의 맵 X 오프셋.
# 초기 스폰에서 chassis bbox를 실측한 값이며, 런타임 PhysX 이동 뒤 USD bbox가
# 초기 좌표에 남아 있어도 Load 생성/창고 인계가 같은 중심을 사용하게 한다.
IW_LOAD_MAP_X_OFFSET_M = 0.3171


def deck_target_xy(
    root_x: float,
    root_y: float,
    root_yaw: float,
    forward_offset: float = 0.0,
) -> tuple[float, float]:
    """현재 IW root 자세에서 팔레트 배치 목표의 월드 X/Y를 계산한다.

    canonical IW는 yaw=180°일 때 root보다 world +X로 0.3171m 떨어진 곳이
    데크 중심이다. 따라서 root 로컬 좌표에서는 deck=(-0.3171, 0)이다.
    추가 배치량은 도킹한 지게차의 전진축(IW yaw + 90°)으로 적용한다.
    """
    values = (root_x, root_y, root_yaw, forward_offset)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("IW 자세와 배치 오프셋은 유한한 값이어야 합니다")
    if not 0.0 <= forward_offset <= 0.8:
        raise ValueError("배치 전진 오프셋은 0.0~0.8m 사이여야 합니다")

    deck_local_x = -IW_LOAD_MAP_X_OFFSET_M
    deck_x = root_x + deck_local_x * math.cos(root_yaw)
    deck_y = root_y + deck_local_x * math.sin(root_yaw)
    fork_heading = root_yaw + math.pi / 2.0
    return (
        deck_x + forward_offset * math.cos(fork_heading),
        deck_y + forward_offset * math.sin(fork_heading),
    )


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
