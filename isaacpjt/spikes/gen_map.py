"""실제 절차형 씬과 같은 좌표로 Nav2 정적 occupancy map을 만든다.

좌표 출처:
  scene/greenhouse.py       온실 15.1 x 26.0 m
  scene/tomato_plants.py    4개 이랑, 3개 재배 구간
  scene/warehouse.py        Y=13..21 m 창고, 출입구와 3면 랙

map 프레임은 Isaac 월드 프레임과 같다. 회전/평행이동 보정값을 여기 넣지 않는다.
"""
from pathlib import Path

import numpy as np

RES = 0.05
XMIN, XMAX = -8.0, 8.0
YMIN, YMAX = -13.5, 21.5

GREENHOUSE_W = 15.1
GREENHOUSE_L = 26.0
WAREHOUSE_D = 8.0
WAREHOUSE_DOOR_W = 4.8

RIDGES = (-4.35, -1.45, 1.45, 4.35)
SEGMENTS = ((-9.8, -5.6), (-2.1, 2.1), (5.6, 9.8))
BED_W = 0.42
BED_END_MARGIN = 0.25

WALL_T = 0.15
GLASS_T = 0.05  # 실제 0.02 m를 5 cm 맵 셀 한 칸 이상으로 표시
RACK_DEPTH = 1.0
RACK_Y = 20.4
RACK_BACK_W = 9.68
RACK_SIDE_X = 6.95
RACK_SIDE_L = 4.88

W = int(round((XMAX - XMIN) / RES))
H = int(round((YMAX - YMIN) / RES))

# 건물 밖은 unknown, 실내만 free. RViz에서 온실과 창고 윤곽이 즉시 구분된다.
grid = np.full((H, W), 128, dtype=np.uint8)


def _ix(x: float) -> int:
    return int(np.floor((x - XMIN) / RES))


def _iy(y: float) -> int:
    return int(np.floor((y - YMIN) / RES))


def fill_rect(x0: float, x1: float, y0: float, y1: float, value: int) -> None:
    """월드 좌표 축정렬 사각형을 클리핑해 채운다."""
    xa, xb = sorted((_ix(x0), _ix(x1) + 1))
    ya, yb = sorted((_iy(y0), _iy(y1) + 1))
    xa, xb = max(0, xa), min(W, xb)
    ya, yb = max(0, ya), min(H, yb)
    if xa < xb and ya < yb:
        grid[ya:yb, xa:xb] = value


half_w = GREENHOUSE_W / 2.0
half_l = GREENHOUSE_L / 2.0
warehouse_y0 = half_l
warehouse_y1 = half_l + WAREHOUSE_D

# 온실과 창고 내부.
fill_rect(-half_w, half_w, -half_l, half_l, 254)
fill_rect(-half_w, half_w, warehouse_y0, warehouse_y1, 254)

# 온실 좌/우/앞 유리벽. +Y 벽은 창고 앞벽과 공유한다.
fill_rect(-half_w - GLASS_T / 2, -half_w + GLASS_T / 2,
          -half_l, half_l, 0)
fill_rect(half_w - GLASS_T / 2, half_w + GLASS_T / 2,
          -half_l, half_l, 0)
fill_rect(-half_w, half_w,
          -half_l - GLASS_T / 2, -half_l + GLASS_T / 2, 0)

# 창고 좌/우/뒷벽.
fill_rect(-half_w - WALL_T / 2, -half_w + WALL_T / 2,
          warehouse_y0, warehouse_y1, 0)
fill_rect(half_w - WALL_T / 2, half_w + WALL_T / 2,
          warehouse_y0, warehouse_y1, 0)
fill_rect(-half_w, half_w,
          warehouse_y1 - WALL_T / 2, warehouse_y1 + WALL_T / 2, 0)

# 온실↔창고 공유 앞벽: 가운데 4.8 m 출입구만 비운다.
door_h = WAREHOUSE_DOOR_W / 2.0
fill_rect(-half_w, -door_h,
          warehouse_y0 - WALL_T / 2, warehouse_y0 + WALL_T / 2, 0)
fill_rect(door_h, half_w,
          warehouse_y0 - WALL_T / 2, warehouse_y0 + WALL_T / 2, 0)

# 실제 베드 collider의 footprint. 잎 캐노피가 아니라 고정 구조물을 맵 기준으로 쓴다.
for x in RIDGES:
    for y0, y1 in SEGMENTS:
        fill_rect(x - BED_W / 2, x + BED_W / 2,
                  y0 - BED_END_MARGIN, y1 + BED_END_MARGIN, 0)

# 창고 랙: 뒷벽 1면 + 좌우벽 2면.
fill_rect(-RACK_BACK_W / 2, RACK_BACK_W / 2,
          RACK_Y - RACK_DEPTH / 2, RACK_Y + RACK_DEPTH / 2, 0)
for x in (-RACK_SIDE_X, RACK_SIDE_X):
    fill_rect(x - RACK_DEPTH / 2, x + RACK_DEPTH / 2,
              17.0 - RACK_SIDE_L / 2, 17.0 + RACK_SIDE_L / 2, 0)

# PGM row 0은 YMAX 쪽이므로 뒤집는다.
pgm = np.flipud(grid)
repo = Path(__file__).resolve().parents[2]
output_dirs = (
    repo / "maps",
    repo / "src" / "smartfarm" / "fleet_dispatch" / "maps",
)
yaml_text = (
    "image: farm_gen.pgm\n"
    f"resolution: {RES}\n"
    f"origin: [{XMIN}, {YMIN}, 0.0]\n"
    "negate: 0\n"
    "occupied_thresh: 0.65\n"
    "free_thresh: 0.25\n"
    "mode: trinary\n"
)
for output_dir in output_dirs:
    output_dir.mkdir(parents=True, exist_ok=True)
    out_pgm = output_dir / "farm_gen.pgm"
    out_yaml = output_dir / "farm_gen.yaml"
    with out_pgm.open("wb") as stream:
        stream.write(b"P5\n%d %d\n255\n" % (W, H))
        stream.write(pgm.tobytes())
    out_yaml.write_text(yaml_text, encoding="utf-8")

print(f"맵 생성: {W}x{H}px, X[{XMIN},{XMAX}], Y[{YMIN},{YMAX}]")
print(f"온실 Y[-13,13] + 창고 Y[13,21], 출입구 {WAREHOUSE_DOOR_W}m")
print("출력: " + ", ".join(str(path) for path in output_dirs))
