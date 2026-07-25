import math

import pytest

from iwhub_control import lanes


def _max_jump(route):
    return max(
        (
            math.hypot(
                route[index + 1][0] - route[index][0],
                route[index + 1][1] - route[index][1],
            )
            for index in range(len(route) - 1)
        ),
        default=0.0,
    )


@pytest.mark.parametrize(
    "start,target",
    [
        ((1.6955, -12.0, math.pi), (0.0, -9.7)),
        ((0.0, -8.0, math.pi / 2.0), (2.9, -8.0)),
        ((2.9, -8.0, math.pi / 2.0), (-2.9, 0.0)),
        ((0.0, 0.0, math.pi / 2.0), (0.0, 7.0)),
        ((0.0, 10.84885, math.pi), (2.9, -8.0)),
        ((0.18, -8.0, math.pi / 2.0), (0.0, 4.0)),
    ],
)
def test_follow_route_is_continuous(start, target):
    route = lanes.follow_route(*start, *target)

    assert route[0] == start
    assert route[-1][0] == pytest.approx(
        min(lanes.VLANES, key=lambda lane: abs(lane - target[0])))
    assert route[-1][1] == pytest.approx(target[1])
    assert _max_jump(route) <= 0.56


def test_follow_route_changes_lanes_only_in_cross_corridor():
    route = lanes.follow_route(
        0.0, -8.0, math.pi / 2.0,
        2.9, -8.0,
    )
    lateral_points = [
        (x, y)
        for x, y, yaw in route
        if abs(math.sin(yaw)) < 0.5
    ]

    assert lateral_points
    assert all(abs(y - lanes.HLANES[0]) < 0.9 for _, y in lateral_points)


def test_follow_route_rejects_off_lane_pose_inside_bed_segment():
    with pytest.raises(ValueError, match="세로 레인 중심을 벗어남"):
        lanes.follow_route(
            1.45, -8.0, math.pi / 2.0,
            0.0, -6.0,
        )


def test_existing_dock_and_return_routes_remain_clear():
    dock = lanes.dock_route(1.6955, -12.0, math.pi)
    back = lanes.return_route(*lanes.DOCK, ty=-12.0)

    assert lanes.footprint_clear(dock) == (True, None)
    assert lanes.footprint_clear(back) == (True, None)
    assert dock[-1][:2] == lanes.DOCK[:2]
    assert dock[-1][2] == pytest.approx(math.pi / 2.0)


def test_follow_route_can_preserve_approach_standoff_in_free_area():
    route = lanes.follow_route(
        1.6955, -12.0, math.pi,
        1.2, -12.0,
        snap_target_x=False,
    )

    assert route[-1][0] == pytest.approx(1.2)
    assert route[-1][1] == pytest.approx(-12.0)
    assert lanes.footprint_clear(route) == (True, None)


def test_follow_route_does_not_apply_custom_footprint_veto():
    route = lanes.follow_route(
        0.0, -8.0, math.pi / 2.0,
        1.7, -8.0,
        snap_target_x=False,
    )

    assert route[-1][:2] == pytest.approx((1.7, -8.0))


def test_dock_route_does_not_veto_latest_bag_start_pose():
    route = lanes.dock_route(
        -0.1064150270,
        -9.4583024460,
        1.3424210037,
    )

    assert route
    assert route[-1][:2] == lanes.DOCK[:2]
    assert route[-1][2] == pytest.approx(math.pi / 2.0)
