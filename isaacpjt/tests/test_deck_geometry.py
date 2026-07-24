import pytest

from pjt_utils.deck_geometry import (
    PALLET_HOLE_CENTER_Z,
    supported_pallet_hole_center_z,
    supported_pallet_origin_z,
)


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
