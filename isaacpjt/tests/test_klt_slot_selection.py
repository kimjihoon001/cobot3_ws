"""빈 KLT 슬롯 선택이 MM '스폰' 위치가 아니라 '현재' 위치를 기준으로 이뤄지는지 검증한다.

기준값은 통합 녹화 sim_diag_20260725_152127의 WAIT_BASKET 시점(t≈70.5s) 실측이다.
당시 iw.py가 /World/Harvester(스폰 자세 고정 Xform)를 기준으로 최근접 슬롯을 골라
실제 MM 기준으로는 가장 먼 슬롯(2.30m)을 발행했고 MM이 플레이스를 포기했다.
"""
import math

import pytest

# 실측(map 프레임) — 녹화 t≈70.5s
MM_POSE = (-0.466, -8.284, math.radians(128.47))    # /World/Harvester/Base/base_link
MM_SPAWN_XY = (0.0, -12.0)                          # /World/Harvester 루트 Xform(고정)
IW_POSE = (-0.046, -9.666, math.radians(83.16))
# 카고 원점의 IW body 오프셋(발행 pose에서 역산). KLT 격자는 robots/iwhub.py
# load_cargo()의 nx=4, ny=2, pitch=(0.31, 0.25)와 같다.
CARGO_BODY_OFFSET = (-0.391, -0.027)
KLT_GRID = [((ix - 1.5) * 0.31, (iy - 0.5) * 0.25)
            for ix in range(4) for iy in range(2)]
# harvest_vision manipulator_target_node의 basket_max_reach_m 기본값.
BASKET_MAX_REACH_M = 1.45


def _slot_map_positions():
    x, y, yaw = IW_POSE
    c, s = math.cos(yaw), math.sin(yaw)
    out = []
    for gx, gy in KLT_GRID:
        ox, oy = CARGO_BODY_OFFSET[0] + gx, CARGO_BODY_OFFSET[1] + gy
        out.append((x + c * ox - s * oy, y + s * ox + c * oy))
    return out


def _nearest(reference):
    return min(_slot_map_positions(),
               key=lambda p: math.hypot(p[0] - reference[0],
                                        p[1] - reference[1]))


def _reach_radius(slot):
    """슬롯의 MM base 프레임 수평 반경(팔 도달 판정에 쓰이는 값)."""
    mx, my, myaw = MM_POSE
    return math.hypot(slot[0] - mx, slot[1] - my)


def test_live_mm_pose_selects_a_slot_inside_arm_reach():
    assert _reach_radius(_nearest(MM_POSE[:2])) == pytest.approx(1.339, abs=0.01)
    assert _reach_radius(_nearest(MM_POSE[:2])) <= BASKET_MAX_REACH_M


def test_spawn_pose_selects_the_farthest_slot_and_fails_reach():
    """회귀 방지: 고정 루트 Xform을 다시 기준으로 삼으면 도달 불가 슬롯이 나온다."""
    stale = _nearest(MM_SPAWN_XY)
    assert _reach_radius(stale) == pytest.approx(2.299, abs=0.01)
    assert _reach_radius(stale) > BASKET_MAX_REACH_M
    assert stale != _nearest(MM_POSE[:2])
