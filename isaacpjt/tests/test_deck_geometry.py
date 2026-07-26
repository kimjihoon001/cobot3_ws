import math

import pytest

from pjt_utils.deck_geometry import (
    IW_LOAD_MAP_X_OFFSET_M,
    PALLET_HOLE_CENTER_Z,
    deck_target_xy,
    supported_pallet_hole_center_z,
    supported_pallet_origin_z,
)


def test_iw_load_map_x_offset_matches_measured_chassis_center():
    assert IW_LOAD_MAP_X_OFFSET_M == pytest.approx(0.3171)


def test_deck_target_uses_canonical_root_relative_offset():
    x, y = deck_target_xy(0.0, 10.84885, math.pi)

    assert x == pytest.approx(0.3171)
    assert y == pytest.approx(10.84885)


def test_deck_target_applies_placement_offset_along_fork_heading():
    center = deck_target_xy(0.0, 10.84885, math.pi)
    target = deck_target_xy(0.0, 10.84885, math.pi, 0.30)

    assert target[0] == pytest.approx(center[0])
    assert target[1] == pytest.approx(center[1] - 0.30)
    assert math.dist(center, target) == pytest.approx(0.30)


def test_deck_target_rotates_with_actual_iw_pose():
    x, y = deck_target_xy(2.0, 3.0, math.pi / 2.0, 0.30)

    assert x == pytest.approx(1.70)
    assert y == pytest.approx(3.0 - IW_LOAD_MAP_X_OFFSET_M)


def test_pallet_bottom_is_placed_on_deck_with_clearance():
    origin_z = supported_pallet_origin_z(0.291, -0.004, 0.002)

    assert origin_z == pytest.approx(0.297)
    assert origin_z - 0.004 == pytest.approx(0.293)


def test_hole_center_tracks_measured_deck_height():
    hole_z = supported_pallet_hole_center_z(0.291)

    assert hole_z == pytest.approx(0.293 + PALLET_HOLE_CENTER_Z)


@pytest.mark.parametrize(
    "deck_z,pallet_min_z,clearance",
    [
        (float("nan"), 0.0, 0.002),
        (0.291, float("inf"), 0.002),
        (0.291, 0.0, -0.001),
    ],
)
def test_invalid_geometry_is_rejected(deck_z, pallet_min_z, clearance):
    with pytest.raises(ValueError):
        supported_pallet_origin_z(deck_z, pallet_min_z, clearance)
