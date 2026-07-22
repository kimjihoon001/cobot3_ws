# -*- coding: utf-8 -*-
"""지면 스폰 — 물리 지면 + 홀 바닥판 + AMR 유도선 + 안전띠.

물리는 Isaac 기본 지면이 담당한다. 바닥판/유도선/안전띠는 시각 전용이라
콜라이더를 붙이지 않는다 (기본 지면과 겹치면 접촉이 이중이 된다).
"""
from pxr import Gf, Usd, UsdGeom

FLOOR_COLOR = Gf.Vec3f(0.52, 0.49, 0.44)     # 콘크리트 (밝으면 조명에 하얗게 날아감)
LANE_COLOR = Gf.Vec3f(0.30, 0.30, 0.34)      # AMR 유도선 (진회색)
SAFETY_YELLOW = Gf.Vec3f(0.93, 0.74, 0.09)
SAFETY_BLACK = Gf.Vec3f(0.12, 0.12, 0.12)

_LANE_W = 0.09       # m. 유도선 폭
_EPS = 0.006         # m. 바닥판 위 z 오프셋 (z-fighting 방지)
# 창고 바닥 슬래브 상면과 같은 공통 주행면. 온실·베드·식물·로봇도 이 높이를 쓴다.
COMMON_FLOOR_Z = 0.055


class Ground:
    """기본 지면 (콜라이더 포함)."""

    def spawn(self, scene, elevation: float = COMMON_FLOOR_Z) -> None:
        """scene = BaseTask.set_up_scene() 이 넘겨주는 Scene 객체."""
        scene.add_default_ground_plane(z_position=elevation)

    def spawn_hall(self, stage: Usd.Stage,
                   center: tuple[float, float],
                   size: tuple[float, float],
                   root: str = "/World/Hall",
                   elevation: float = COMMON_FLOOR_Z) -> None:
        """홀 바닥판 + 그 둘레의 AMR 유도선 루프 + 한쪽 가장자리 안전띠."""
        UsdGeom.Xform.Define(stage, root)
        UsdGeom.Xformable(stage.GetPrimAtPath(root)).AddTranslateOp().Set(
            Gf.Vec3d(0.0, 0.0, elevation))
        cx, cy = center
        w, l = size

        self._flat(stage, f"{root}/Floor", (cx, cy, _EPS), (w, l), FLOOR_COLOR)

        # 유도선 루프 — 바닥판 가장자리에서 1.2m 안쪽
        inset = 1.2
        self._lane_rect(stage, f"{root}/Lane",
                        (cx, cy), (w - 2 * inset, l - 2 * inset))

        # 안전띠 — +x 가장자리, 황/흑 1m 교대 (통로와 외부의 경계 표시)
        seg, strip_w = 1.0, 0.14
        x = cx + w / 2.0 - strip_w
        n = int(l // seg)
        for i in range(n):
            y = cy - l / 2.0 + (i + 0.5) * seg
            color = SAFETY_YELLOW if i % 2 == 0 else SAFETY_BLACK
            self._flat(stage, f"{root}/Safety_{i:02d}",
                       (x, y, _EPS * 2), (strip_w, seg), color)

    # ----- 내부 -----

    def _lane_rect(self, stage: Usd.Stage, root: str,
                   center: tuple[float, float], size: tuple[float, float]) -> None:
        """직사각 유도선 (변 4개)."""
        UsdGeom.Xform.Define(stage, root)
        cx, cy = center
        w, l = size
        # 상하 (x 방향 변)
        for name, y in (("S", cy - l / 2.0), ("N", cy + l / 2.0)):
            self._flat(stage, f"{root}/{name}", (cx, y, _EPS * 2),
                       (w + _LANE_W, _LANE_W), LANE_COLOR)
        # 좌우 (y 방향 변)
        for name, x in (("W", cx - w / 2.0), ("E", cx + w / 2.0)):
            self._flat(stage, f"{root}/{name}", (x, cy, _EPS * 2),
                       (_LANE_W, l + _LANE_W), LANE_COLOR)

    def _flat(self, stage: Usd.Stage, path: str,
              pos: tuple[float, float, float],
              size_xy: tuple[float, float], color: Gf.Vec3f) -> None:
        cube = UsdGeom.Cube.Define(stage, path)
        cube.CreateSizeAttr(1.0)
        cube.CreateDisplayColorAttr([color])
        xf = UsdGeom.Xformable(cube.GetPrim())
        xf.AddTranslateOp().Set(Gf.Vec3d(*pos))
        xf.AddScaleOp().Set(Gf.Vec3f(size_xy[0], size_xy[1], 0.004))
