#!/usr/bin/env python3
"""Isaac 온실/창고 설정에서 IW Nav2 정적 맵을 재생성한다.

맵 프레임은 Isaac 월드 XY와 같다. 기존 IW의 map->odom 정렬을
바꾸지 않도록 원점과 범위는 기존 greenhouse.yaml과 동일하게 유지한다.
동적 로봇·팔레트는 넣지 않고, 바닥 단차를 해소한 경사판은 통행 가능하므로
장애물에서 제외한다.
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path


RESOLUTION = 0.05
ORIGIN_X = -8.5
ORIGIN_Y = -14.0
MAP_WIDTH_M = 17.0
MAP_HEIGHT_M = 36.5

# scene/tomato_plants.py의 실제 충돌 형상.
BED_WIDTH = 0.42
BED_END_MARGIN = 0.50

# scene/warehouse.py의 랙 충돌 형상 상수.
RACK_DEPTH = 1.00
POST_T = 0.08


def find_workspace() -> Path:
    """isaacpjt/pjt_config/settings.py가 있는 워크스페이스를 찾는다."""
    for parent in Path(__file__).resolve().parents:
        if (parent / "isaacpjt" / "pjt_config" / "settings.py").is_file():
            return parent
    raise RuntimeError("워크스페이스의 isaacpjt/pjt_config/settings.py를 찾지 못했습니다")


class OccupancyMap:
    def __init__(self, resolution: float = RESOLUTION):
        self.resolution = resolution
        self.origin_x = ORIGIN_X
        self.origin_y = ORIGIN_Y
        self.width = round(MAP_WIDTH_M / resolution)
        self.height = round(MAP_HEIGHT_M / resolution)
        self.data = bytearray([254]) * (self.width * self.height)
        self.obstacle_count = 0

    def add_rect(self, cx: float, cy: float, sx: float, sy: float) -> None:
        """월드 좌표 축 정렬 직사각형을 occupied(0)로 그린다."""
        x0, x1 = cx - sx / 2.0, cx + sx / 2.0
        y0, y1 = cy - sy / 2.0, cy + sy / 2.0
        ix0 = max(0, math.floor((x0 - self.origin_x) / self.resolution))
        ix1 = min(self.width - 1, math.ceil((x1 - self.origin_x) / self.resolution) - 1)
        iy0 = max(0, math.floor((y0 - self.origin_y) / self.resolution))
        iy1 = min(self.height - 1, math.ceil((y1 - self.origin_y) / self.resolution) - 1)
        if ix0 > ix1 or iy0 > iy1:
            return
        # PGM은 위에서 아래로 저장하므로 맵 cell Y를 반전한다.
        for iy in range(iy0, iy1 + 1):
            row = self.height - 1 - iy
            start = row * self.width + ix0
            self.data[start:start + ix1 - ix0 + 1] = bytes([0]) * (ix1 - ix0 + 1)
        self.obstacle_count += 1

    def write(self, output_dir: Path) -> None:
        output_dir.mkdir(parents=True, exist_ok=True)
        pgm = output_dir / "greenhouse.pgm"
        yaml_path = output_dir / "greenhouse.yaml"
        pgm.write_bytes(
            f"P5\n{self.width} {self.height}\n255\n".encode("ascii") + self.data
        )
        yaml_path.write_text(
            "image: greenhouse.pgm\n"
            f"resolution: {self.resolution}\n"
            f"origin: [{self.origin_x}, {self.origin_y}, 0.0]\n"
            "negate: 0\n"
            "occupied_thresh: 0.65\n"
            "free_thresh: 0.25\n",
            encoding="utf-8",
        )
        print(f"[map] {pgm}: {self.width}x{self.height}, {self.resolution:.3f} m/px")
        print(f"[map] {yaml_path}: origin=({self.origin_x:.2f}, {self.origin_y:.2f})")
        print(f"[map] rasterized rectangles: {self.obstacle_count}")


def generate(output_dir: Path, resolution: float) -> None:
    workspace = find_workspace()
    sys.path.insert(0, str(workspace / "isaacpjt"))
    from pjt_config.settings import SceneConfig

    cfg = SceneConfig()
    greenhouse = cfg.greenhouse
    plants = cfg.plants
    warehouse = cfg.warehouse
    grid = OccupancyMap(resolution)

    half_w = greenhouse.width / 2.0
    half_l = greenhouse.length / 2.0
    wall_t = greenhouse.frame_size

    # 온실: 좌·우 유리벽과 전면 유리벽. 후면은 창고 입구와 공유해 열려 있다.
    grid.add_rect(-half_w, 0.0, wall_t, greenhouse.length)
    grid.add_rect(+half_w, 0.0, wall_t, greenhouse.length)
    grid.add_rect(0.0, -half_l, greenhouse.width, wall_t)

    # 재배 베드: TomatoPlants._column_row_xs/_segment_ys와 동일한 공식.
    col_width = (plants.rows_per_col - 1) * plants.row_spacing
    col_pitch = col_width + plants.aisle_x
    col_span = (plants.sector_cols - 1) * col_pitch
    row_xs: list[float] = []
    for sector_col in range(plants.sector_cols):
        center_x = -col_span / 2.0 + sector_col * col_pitch
        row_xs.extend(
            center_x - col_width / 2.0 + row * plants.row_spacing
            for row in range(plants.rows_per_col)
        )

    segment_length = (plants.plants_per_seg - 1) * plants.plant_spacing
    segment_pitch = segment_length + plants.aisle_y
    segment_span = (plants.sector_rows - 1) * segment_pitch
    segment_ys = [
        -segment_span / 2.0 + row * segment_pitch
        for row in range(plants.sector_rows)
    ]
    for x in row_xs:
        for y in segment_ys:
            grid.add_rect(x, y, BED_WIDTH, segment_length + BED_END_MARGIN)

    # 창고: 온실 후면(+Y)에 gap 없이 붙은 방.
    warehouse_center_y = half_l + warehouse.depth / 2.0
    building_wall_t = 0.15
    back_y = warehouse_center_y + warehouse.depth / 2.0
    grid.add_rect(0.0, back_y, greenhouse.width, building_wall_t)
    grid.add_rect(-half_w, warehouse_center_y, building_wall_t, warehouse.depth)
    grid.add_rect(+half_w, warehouse_center_y, building_wall_t, warehouse.depth)

    # 창고 전면 양쪽 벽. 중앙 entrance_width는 경사판이 있는 통행 구간이다.
    side_width = (greenhouse.width - warehouse.entrance_width) / 2.0
    front_y = warehouse_center_y - warehouse.depth / 2.0
    side_center_x = warehouse.entrance_width / 2.0 + side_width / 2.0
    grid.add_rect(-side_center_x, front_y, side_width, building_wall_t)
    grid.add_rect(+side_center_x, front_y, side_width, building_wall_t)

    # 뒷벽 랙: 하단 선반과 기둥이 차지하는 전체 footprint.
    pitch = warehouse.slot_pitch
    if pitch is None:
        raise ValueError("WarehouseConfig.slot_pitch가 정의되어야 맵을 만들 수 있습니다")
    rear_bays = max(
        warehouse.sectors,
        int((greenhouse.width - 2.0 * (RACK_DEPTH + 1.0)) / pitch),
    )
    rear_rack_width = rear_bays * pitch + POST_T
    rear_rack_local_y = warehouse.depth / 2.0 - RACK_DEPTH / 2.0 - 0.10
    grid.add_rect(
        0.0,
        warehouse_center_y + rear_rack_local_y,
        rear_rack_width,
        RACK_DEPTH,
    )

    # 좌·우벽 랙: 입구쪽 2.5m를 비운 세 면 선반 배치.
    side_bays = max(1, int((warehouse.depth - 2.5) / pitch))
    side_rack_length = side_bays * pitch + POST_T
    side_rack_x = half_w - RACK_DEPTH / 2.0 - 0.10
    grid.add_rect(-side_rack_x, warehouse_center_y, RACK_DEPTH, side_rack_length)
    grid.add_rect(+side_rack_x, warehouse_center_y, RACK_DEPTH, side_rack_length)

    grid.write(output_dir)
    print(
        "[scene] beds="
        f"{len(row_xs) * len(segment_ys)} "
        f"({BED_WIDTH:.2f}m x {segment_length + BED_END_MARGIN:.2f}m), "
        f"warehouse entrance={warehouse.entrance_width:.2f}m"
    )


def main() -> None:
    default_output = Path(__file__).resolve().parents[1] / "maps"
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, default=default_output)
    parser.add_argument("--resolution", type=float, default=RESOLUTION)
    args = parser.parse_args()
    if args.resolution <= 0:
        parser.error("--resolution은 0보다 커야 합니다")
    generate(args.output_dir.resolve(), args.resolution)


if __name__ == "__main__":
    main()
