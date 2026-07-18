# -*- coding: utf-8 -*-
"""트레이 — 수확 MM 이 과실을 담고, AMR 이 통째로 나른다.

★ GPU 미검증. ★

이 구조의 핵심: **AMR 은 토마토를 직접 안 만진다.** 트레이만 다루므로 파지 난이도가
낮다(v3 6.1 "파지 난이도 낮음(트레이 표준화)"). 무른 과실을 다루는 건 MM 혼자다.

**이 모듈은 지오메트리만 만든다.** 적재량 추적·만재 판정·운반 트리거는
`tray_manager_node` (개인 PC, ROS2) 파트다 — v3 7장 6번 단계.
Isaac 은 칸 좌표를 내주고 시키는 대로 놓을 뿐, 어느 칸에 놓을지 결정하지 않는다.
결정과 상태를 ROS2 로 몰아야 다중PC 구조(35점)가 장식이 안 된다.

지오메트리는 6칸 격자. 칸 간격은 과실 지름(68.7mm)보다 커야 한다.
TODO cell_pitch 가 미정이다 — 트레이 에셋이 나오면 실측할 것.
"""
from __future__ import annotations

from pxr import Gf, Usd, UsdGeom

from pjt_config.settings import TrayConfig
from scene import physics

TRAY_COLOR = Gf.Vec3f(0.55, 0.35, 0.20)
WALL_H = 0.05          # 칸막이 높이. 과실이 굴러 나가지 않을 만큼

# [4] 임의 — cell_pitch 미정이라 임시. 과실 지름 68.7mm + 여유.
FALLBACK_PITCH = 0.09  # m


class Tray:
    """6칸 트레이. 담긴 개수를 추적하고 만재를 알린다."""

    def __init__(self, cfg: TrayConfig):
        self._cfg = cfg
        self._root: str | None = None
        self._cells: list[dict] = []

    @property
    def root(self) -> str | None:
        return self._root

    @property
    def cells(self) -> list[dict]:
        """{index, path, position}. ROS2 가 "몇 번 칸에 놓아라" 를 보내면
        Isaac 이 여기서 좌표를 찾아 실행한다."""
        return self._cells

    def spawn(self, stage: Usd.Stage, root: str = "/World/Tray",
              position: tuple[float, float, float] = (0.0, 0.0, 0.0),
              log=print) -> str:
        self._root = root
        UsdGeom.Xform.Define(stage, root)
        UsdGeom.Xformable(stage.GetPrimAtPath(root)).AddTranslateOp().Set(
            Gf.Vec3d(*position))

        pitch = self._cfg.cell_pitch or FALLBACK_PITCH
        if self._cfg.cell_pitch is None:
            log(f"[Tray] ⚠ cell_pitch 미정 -> 임시 {pitch}m 사용. "
                "트레이 에셋이 나오면 실측할 것.")

        # 6칸을 2행 3열로. 3x2 가 아니라 2x3 인 이유는 없다 — [4] 임의.
        rows, cols = 2, self._cfg.capacity // 2
        for i in range(self._cfg.capacity):
            r, c = divmod(i, cols)
            x = (c - (cols - 1) / 2.0) * pitch
            y = (r - (rows - 1) / 2.0) * pitch
            path = f"{root}/Cell_{i:02d}"
            self._add_cell(stage, path, (x, y, 0.0), pitch)
            self._cells.append({
                "index": i,
                "path": path,
                "position": (position[0] + x, position[1] + y, position[2]),
            })

        log(f"[Tray] {self._cfg.capacity}칸 ({rows}x{cols}) 지오메트리 생성. "
            f"적재량 추적은 tray_manager_node(ROS2) 담당")
        return root

    # ----- 내부 -----

    def _add_cell(self, stage: Usd.Stage, path: str,
                  pos: tuple[float, float, float], pitch: float) -> None:
        """칸 바닥 + 칸막이. 과실이 굴러 나가지 않게."""
        floor = UsdGeom.Cube.Define(stage, path)
        floor.CreateSizeAttr(1.0)
        floor.CreateDisplayColorAttr([TRAY_COLOR])
        xf = UsdGeom.Xformable(floor.GetPrim())
        xf.AddTranslateOp().Set(Gf.Vec3d(*pos))
        xf.AddScaleOp().Set(Gf.Vec3f(pitch * 0.95, pitch * 0.95, 0.005))
        physics.add_shape_collider(floor.GetPrim())

        for name, dx, dy, sx, sy in (
            ("W0", -pitch / 2, 0.0, 0.004, pitch),
            ("W1", pitch / 2, 0.0, 0.004, pitch),
            ("W2", 0.0, -pitch / 2, pitch, 0.004),
            ("W3", 0.0, pitch / 2, pitch, 0.004),
        ):
            w = UsdGeom.Cube.Define(stage, f"{path}/{name}")
            w.CreateSizeAttr(1.0)
            w.CreateDisplayColorAttr([TRAY_COLOR])
            wxf = UsdGeom.Xformable(w.GetPrim())
            wxf.AddTranslateOp().Set(
                Gf.Vec3d(pos[0] + dx, pos[1] + dy, pos[2] + WALL_H / 2))
            wxf.AddScaleOp().Set(Gf.Vec3f(sx, sy, WALL_H))
            physics.add_shape_collider(w.GetPrim())
