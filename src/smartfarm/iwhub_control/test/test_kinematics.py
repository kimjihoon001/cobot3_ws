import math

import pytest

from iwhub_control.kinematics import stabilize_twist


def test_stopped_twist_stays_zero_below_start_thresholds():
    assert stabilize_twist(
        0.019,
        -0.039,
        True,
        linear_stop=0.01,
        angular_stop=0.02,
        linear_start=0.02,
        angular_start=0.04,
    ) == (0.0, 0.0, True)


def test_stopped_twist_restarts_after_start_threshold():
    assert stabilize_twist(
        0.021,
        0.01,
        True,
        linear_stop=0.01,
        angular_stop=0.02,
        linear_start=0.02,
        angular_start=0.04,
    ) == (0.021, 0.0, False)


def test_rotation_restart_drops_linear_noise_below_linear_start_threshold():
    assert stabilize_twist(
        0.019,
        -0.05,
        True,
        linear_stop=0.01,
        angular_stop=0.02,
        linear_start=0.02,
        angular_start=0.04,
    ) == (0.0, -0.05, False)


def test_moving_twist_stops_below_stop_thresholds():
    assert stabilize_twist(
        -0.01,
        0.02,
        False,
        linear_stop=0.01,
        angular_stop=0.02,
        linear_start=0.02,
        angular_start=0.04,
    ) == (0.0, 0.0, True)


def test_moving_twist_keeps_meaningful_rotation_and_zeros_linear_noise():
    assert stabilize_twist(
        0.005,
        -0.05,
        False,
        linear_stop=0.01,
        angular_stop=0.02,
        linear_start=0.02,
        angular_start=0.04,
    ) == (0.0, -0.05, False)


@pytest.mark.parametrize(
    "kwargs",
    [
        {"linear_stop": -0.01},
        {"linear_start": 0.005},
        {"angular_start": 0.01},
        {"angular_stop": math.nan},
    ],
)
def test_invalid_deadband_configuration_is_rejected(kwargs):
    values = {
        "linear_stop": 0.01,
        "angular_stop": 0.02,
        "linear_start": 0.02,
        "angular_start": 0.04,
    }
    values.update(kwargs)
    with pytest.raises(ValueError):
        stabilize_twist(0.0, 0.0, True, **values)
