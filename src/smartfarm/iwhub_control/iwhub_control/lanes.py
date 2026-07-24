# -*- coding: utf-8 -*-
"""온실 통로 레인 그래프 — Nav2 반응형 계획 대신 '통로 중심선'만 결정적으로 다니게 한다.

배드 충돌 없는 안정 동선의 핵심: 경로가 통로 중심선(레인) 위에 있으면 설계상 배드에서
떨어져 있음이 보장된다. 좌표는 맵 생성기(isaacpjt/spikes/gen_map.py)와 동일한 온실
지오메트리에서 유도한다.

  이랑 X = (-4.35, -1.45, 1.45, 4.35)   구간 Y = (-9.8..-5.6, -2.1..2.1, 5.6..9.8)
  온실 Y[-13,13]   창고 도어(중앙 4.8m) Y=13   지게차 인계 도크 (0, 10.85, yaw=−X)

레인 = 배드 사이·둘레 통로의 중심선. 코너는 원호(pivot 아님). 도크 진입은 항상 X=0 레인을
아래→위(+Y)로 타되, 최종 정차 자세는 −X(iw 스폰 orientation=canonical 도킹 자세). 즉
+Y로 접근해 도크에서 −X로 90° 정렬(포크가 도크 위 +Y에서 인계).
"""
from __future__ import annotations

import math

# ── 통로 중심선 (gen_map 좌표식에서 유도) ─────────────────────────────
# 세로 레인 X: 이랑(-4.35,-1.45,1.45,4.35) 사이 중점 + 좌우 둘레. 자유폭 2.48m(iw 0.75m).
VLANES = (-6.0, -2.9, 0.0, 2.9, 6.0)
# 가로 레인 Y: 재배 구간 사이 중점(-3.85, 3.85) + 하단(-11.5)/상단(11.5, 도어 앞).
HLANES = (-11.5, -3.85, 3.85, 11.5)
# 지게차 인계 정위치. yaw=π(−X 향함) = iw 스폰 orientation(SPAWN_YAW_DEG=180°) = iw_dock.py
# canonical 도킹 자세. 포크는 도크 바로 위(0,14.5)에서 인계하므로 iw는 −X로 정차한다.
DOCK = (0.0, 10.85, math.pi)
# 도크 직전 X=0 레인에서 이 Y부터 위로 곧게 +Y 접근한다(구간3 위 통로).
DOCK_APPROACH_Y = 3.85

# 배드 사각형(맵 좌표) — 이랑 X±BED_W/2(0.21), 구간 Y±BED_END_MARGIN(0.25). 경로 보장 검사용.
_RIDGES = (-4.35, -1.45, 1.45, 4.35)
_SEGMENTS = ((-9.8, -5.6), (-2.1, 2.1), (5.6, 9.8))
BED_RECTS = [
    (rx - 0.21, rx + 0.21, sy0 - 0.25, sy1 + 0.25)
    for rx in _RIDGES for (sy0, sy1) in _SEGMENTS
]


def clearance(x: float, y: float) -> float:
    """(x,y)에서 가장 가까운 배드 사각형까지 거리(0=내부/접촉). 경로 보장(sweep) 검사용."""
    best = float("inf")
    for x0, x1, y0, y1 in BED_RECTS:
        dx = max(x0 - x, 0.0, x - x1)
        dy = max(y0 - y, 0.0, y - y1)
        best = min(best, math.hypot(dx, dy))
    return best


def _nearest(v: float, options) -> float:
    return min(options, key=lambda o: abs(o - v))


def _yaw(dx: float, dy: float) -> float:
    return math.atan2(dy, dx)


def _corner_arc(cx: float, cy: float, din, dout, r: float, n: int = 6):
    """(cx,cy) 코너에서 din 방향으로 들어와 dout 방향으로 나가는 90° 원호 포즈들.

    din, dout = 단위 방향 (dx,dy). 반환 = [(x,y,yaw), ...] (yaw=접선 진행방향).
    직선 두 구간을 반지름 r 원호로 필렛한다(접점=코너에서 r 뒤/앞).
    """
    sx, sy = cx - r * din[0], cy - r * din[1]         # 원호 시작(들어오는 접점)
    cross = din[0] * dout[1] - din[1] * dout[0]        # +면 좌회전(ccw), -면 우회전(cw)
    if cross >= 0:                                     # ccw → 중심은 왼쪽(+90°)
        nx, ny = -din[1], din[0]
    else:                                              # cw → 중심은 오른쪽(-90°)
        nx, ny = din[1], -din[0]
    ox, oy = sx + r * nx, sy + r * ny                  # 원호 중심
    a0 = math.atan2(sy - oy, sx - ox)
    sweep = (math.pi / 2.0) * (1 if cross >= 0 else -1)
    out = []
    for i in range(1, n + 1):
        a = a0 + sweep * i / n
        x = ox + r * math.cos(a)
        y = oy + r * math.sin(a)
        yaw = a + (math.pi / 2 if cross >= 0 else -math.pi / 2)
        out.append((x, y, yaw))
    return out


def _straight(x0, y0, x1, y1, step: float = 0.5):
    """(x0,y0)→(x1,y1) 직선 위 균등 웨이포인트 (끝점 포함). yaw=진행방향."""
    d = math.hypot(x1 - x0, y1 - y0)
    yaw = _yaw(x1 - x0, y1 - y0)
    n = max(1, int(round(d / step)))
    return [(x0 + (x1 - x0) * i / n, y0 + (y1 - y0) * i / n, yaw)
            for i in range(1, n + 1)]


def dock_route(sx: float, sy: float, syaw: float, arc_r: float = 0.8,
               step: float = 0.5):
    """iw (sx,sy,syaw) → 지게차 도크(0,10.85,+Y). 전진 전용, 시작 자세에서 연속 연결.

    도크는 중앙 레인(X=0) 위에 있고 +Y로 접근해야 하므로:
      1) 현재 위치에서 X=0 레인에 합류(수평 전진 → 원호로 +Y 전환).
      2) X=0 레인을 +Y로 곧게 도크까지. X=0은 이랑 사이 중앙 통로라 아래→위 배드-클리어.
    첫 웨이포인트=현재 자세(측면 점프 제거). 원호는 직선과 접점에서 연속(코너 뒤점프 없음:
    직선이 코너가 아니라 접점 tangent_x/±arc_r 에서 끝난다).
    ⚠ 수평 합류가 현재 Y(sy)에서 일어나므로 sy가 배드 구간 Y 안이면 별도 교차통로 경유 필요
      (현재는 배드-프리 시작구역 가정). 전체 footprint 안전은 sweep 검증이 담당(별도).
    반환 = [(x,y,yaw), ...] (map 프레임).
    """
    dock_x, dock_y, dock_yaw = DOCK
    wps: list[tuple[float, float, float]] = [(sx, sy, syaw)]   # 현재 자세에서 출발
    if abs(sx - dock_x) <= 0.15:
        wps += _straight(dock_x, sy, dock_x, dock_y, step)     # 이미 X=0 — 곧장 +Y
    else:
        hdir = -1.0 if sx > dock_x else 1.0                    # 도크X(0) 쪽 수평 방향
        tangent_x = dock_x - hdir * arc_r                      # 원호 진입 접점(직선 끝)
        wps += _straight(sx, sy, tangent_x, sy, step)          # 수평 전진(접점까지)
        wps += _corner_arc(dock_x, sy, (hdir, 0.0), (0.0, 1.0), arc_r)  # 원호 → +Y
        wps += _straight(dock_x, sy + arc_r, dock_x, dock_y, step)      # +Y로 도크까지
    wps.append((dock_x, dock_y, dock_yaw))
    return wps


def follow_lane_x(mm_x: float) -> float:
    """MM(또는 iw)이 있는 세로 레인 중심선 X. iw follow 목표를 레인에 스냅할 때 쓴다."""
    return _nearest(mm_x, VLANES)


if __name__ == "__main__":   # self-test (ROS 불필요)
    # (x, y, yaw) — 실제 iw 시작(1.6955,-12,180°) 포함
    for (sx, sy, syaw) in [(1.6955, -12.0, math.pi), (-2.9, -12.0, math.pi),
                           (0.0, -11.5, math.pi / 2), (2.9, -12.0, 0.0)]:
        r = dock_route(sx, sy, syaw)
        print(f"\nstart=({sx},{sy},{math.degrees(syaw):.0f}°)  waypoints={len(r)}")
        # 연속성 검사: 인접 웨이포인트 간 최대 점프(원호 뒤점프 버그 재발 방지)
        jumps = [math.hypot(r[i+1][0]-r[i][0], r[i+1][1]-r[i][1])
                 for i in range(len(r)-1)]
        clr = min(clearance(x, y) for x, y, _ in r)
        print(f"   최대 점프 = {max(jumps):.2f}m (step 0.5 근처여야 연속) | "
              f"배드 최소거리(중심) = {clr:.2f}m "
              f"{'✅' if max(jumps) < 0.9 and clr > 0.45 else '⚠'} "
              f"(clr은 참고용 — 전체 footprint sweep 아님)")
