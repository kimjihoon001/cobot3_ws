"""IW 데크 팔레트 안착 폐루프 회귀 테스트.

배경 (2026-07-26 sim_diag_20260726_223158):
  개루프 목표까지 정확히 내렸는데도 결속 게이트(|z_error| ≤ 0.025)가 팔레트를
  거부해 transfer 단계가 10.2s/10.0s로 타임아웃했다. 리프트 추종 오차는
  pallet_rise-expected_rise = 5.5e-6 m로, 리프트는 명령대로 움직였다.

  ★단, 그 bag에는 `pallet_min_z`(bbox 최저점) 필드가 없었다. 당시 기록된
  pallet_position.z는 강체 **원점**이고 게이트가 재는 값과 다른 양이므로,
  게이트 기준 잔차의 크기는 그 bag으로 확정할 수 없다. 아래 값들은 bag
  재현값이 아니라 **합성 입력**이다.

여기서는 리프트 명령 계산만 순수 함수로 검증한다(Isaac·ROS 런타임 불필요).
"""
import pytest

from warehouse_dock.fork_lift_node import clamp, deck_place_lift_setpoint


# 합성 입력 — 게이트 허용치(0.025)보다 확실히 큰 잔차를 만든다.
SYNTH_LIFT = 0.140
SYNTH_TARGET_Z = 0.284
SYNTH_RESIDUAL = 0.055
SYNTH_MEASURED_Z = SYNTH_TARGET_Z + SYNTH_RESIDUAL

# 노드 기본값 (Codex 검수 권장값)
TOL = 0.010
MAX_CORRECTION = 0.10
# loaded_lift_speed 0.15 m/s / control_rate 20 Hz
MAX_DELTA = 0.15 / 20.0


def _floor(open_loop_lift: float) -> float:
    return max(0.0, open_loop_lift - MAX_CORRECTION)


def test_positive_residual_drives_the_lift_down():
    nxt = deck_place_lift_setpoint(
        SYNTH_LIFT, SYNTH_MEASURED_Z, SYNTH_TARGET_Z,
        _floor(SYNTH_LIFT), MAX_DELTA,
    )
    assert nxt < SYNTH_LIFT
    assert SYNTH_LIFT - nxt == pytest.approx(MAX_DELTA, abs=1e-9)


def test_residual_converges_into_tolerance():
    """틱을 반복하면 잔차가 허용오차 안으로 수렴한다.

    팔레트는 포크에 강체 결속돼 리프트를 내린 만큼 그대로 내려온다는 전제다
    (bag에서 pallet_rise ≈ expected_rise로 확인된 관계).
    """
    lift = SYNTH_LIFT
    z = SYNTH_MEASURED_Z
    floor = _floor(SYNTH_LIFT)
    for _ in range(200):
        if abs(z - SYNTH_TARGET_Z) <= TOL:
            break
        nxt = deck_place_lift_setpoint(
            lift, z, SYNTH_TARGET_Z, floor, MAX_DELTA)
        z += nxt - lift              # 강체 결속: 리프트 변화량 = 팔레트 변화량
        lift = nxt
    assert abs(z - SYNTH_TARGET_Z) <= TOL
    assert lift == pytest.approx(SYNTH_LIFT - SYNTH_RESIDUAL, abs=TOL)


def test_tick_change_is_speed_limited():
    """한 틱에 max_delta보다 크게 움직이지 않는다."""
    nxt = deck_place_lift_setpoint(0.5, 1.0, 0.0, 0.0, MAX_DELTA)   # 잔차 1m
    assert 0.5 - nxt == pytest.approx(MAX_DELTA, abs=1e-9)


def test_correction_is_clamped_to_the_floor():
    """개루프 목표에서 max_correction 넘게는 내려가지 않는다."""
    open_loop = 0.30
    floor = _floor(open_loop)
    lift = open_loop
    for _ in range(500):
        lift = deck_place_lift_setpoint(lift, 1.0, 0.0, floor, MAX_DELTA)
    assert lift == pytest.approx(floor, abs=1e-9)
    assert open_loop - lift == pytest.approx(MAX_CORRECTION, abs=1e-9)


def test_upward_correction_when_pallet_is_too_low():
    """지지면보다 낮으면(과압) 다시 올린다 — 게이트는 절대값 판정이다."""
    nxt = deck_place_lift_setpoint(0.20, 0.10, 0.15, 0.10, MAX_DELTA)
    assert nxt > 0.20


def test_lift_never_exceeds_the_mechanical_limit():
    nxt = deck_place_lift_setpoint(2.0, 0.0, 5.0, 0.0, 10.0)
    assert nxt == pytest.approx(2.0)


def test_floor_is_respected_even_with_a_huge_single_step():
    """max_delta가 커도 하한을 넘지 않는다."""
    nxt = deck_place_lift_setpoint(0.30, 1.0, 0.0, 0.20, 10.0)
    assert nxt == pytest.approx(0.20)


def test_clamp_helper_is_the_one_used_by_the_node():
    assert clamp(5.0, 0.0, 2.0) == 2.0
    assert clamp(-1.0, 0.0, 2.0) == 0.0
