# -*- coding: utf-8 -*-
"""온실 프레임 스폰 — 기둥 + 상단 보 (패널 없는 골조).

프레임은 static collider. RigidBody 를 안 붙이므로 움직이지 않고,
로봇이 통과하지 못한다.

투명 지붕/벽 패널은 RTX 반투명 세팅 확인 후 다음 단계에서 추가.
"""
from pxr import Usd, UsdGeom, Gf

from pjt_config.settings import GreenhouseConfig
from scene import physics


class Greenhouse:
    def __init__(self, cfg: GreenhouseConfig):
        self._cfg = cfg

    def spawn(self, stage: Usd.Stage, root: str = "/World/Greenhouse",
              back_wall: bool = True, elevation: float = 0.0) -> None:
        UsdGeom.Xform.Define(stage, root)
        UsdGeom.Xformable(stage.GetPrimAtPath(root)).AddTranslateOp().Set(
            Gf.Vec3d(0.0, 0.0, elevation))
        c = self._cfg
        half_w, half_l = c.width / 2.0, c.length / 2.0
        t = c.frame_size

        # 기둥 y 위치: 길이 방향으로 post_spacing 간격 (양 끝 포함)
        n_spans = max(1, round(c.length / c.post_spacing))
        ys = [-half_l + i * (c.length / n_spans) for i in range(n_spans + 1)]

        # 기둥 (양쪽 벽)
        for i, y in enumerate(ys):
            for side, x in (("L", -half_w), ("R", half_w)):
                self._add_beam(stage, f"{root}/Post_{side}_{i:02d}",
                               center=(x, y, c.height / 2.0), size=(t, t, c.height))

        # 상단 보(TopBeam)·크로스 보(CrossBeam) 제거 — 위에서 내려다볼 때 시야를 가려
        # 안 보기 좋음(사용자 요청 2026-07-20). 기둥+유리벽만 남긴다(구조·충돌 유지).

        # 유리 패널 — 반투명 벽 + static 콜라이더 (로봇이 뚫고 나가지 못하게, 2026-07-20)
        # 지붕은 안 덮는다 — 시연을 위에서 내려다보는 게 우선 (사용자 결정 2026-07-18).
        self._add_glass(stage, f"{root}/Glass_L",
                        (-half_w, 0.0, c.height / 2.0), (0.02, c.length, c.height))
        self._add_glass(stage, f"{root}/Glass_R",
                        (half_w, 0.0, c.height / 2.0), (0.02, c.length, c.height))
        self._add_glass(stage, f"{root}/Glass_Front",
                        (0.0, -half_l, c.height / 2.0), (c.width, 0.02, c.height))

        # 뒷벽(+y, 창고 방향). 창고를 벽 하나로 붙이면(back_wall=False) 온실 뒷벽을 생략하고
        # 창고 앞벽을 공유 칸막이로 쓴다(팀 피드백 2026-07-20: "창고와 재배공간 벽 하나 두고").
        if back_wall:
            door_w = 3.0
            pane_w = (c.width - door_w) / 2.0
            for side, x in (("L", -(door_w + pane_w) / 2.0),
                            ("R", (door_w + pane_w) / 2.0)):
                self._add_glass(stage, f"{root}/Glass_Back_{side}",
                                (x, half_l, c.height / 2.0),
                                (pane_w, 0.02, c.height))
            # 출입구 양옆 문틀 기둥 (골조와 같은 규격, 콜라이더 있음)
            for side, x in (("L", -door_w / 2.0), ("R", door_w / 2.0)):
                self._add_beam(stage, f"{root}/DoorPost_{side}",
                               center=(x, half_l, c.height / 2.0),
                               size=(t, t, c.height))

    def _add_beam(self, stage: Usd.Stage, path: str,
                  center: tuple[float, float, float],
                  size: tuple[float, float, float]) -> None:
        cube = UsdGeom.Cube.Define(stage, path)
        cube.CreateSizeAttr(1.0)
        cube.CreateDisplayColorAttr([Gf.Vec3f(*self._cfg.frame_color)])
        xf = UsdGeom.Xformable(cube.GetPrim())
        xf.AddTranslateOp().Set(Gf.Vec3d(*center))
        xf.AddScaleOp().Set(Gf.Vec3f(*size))
        physics.add_shape_collider(cube.GetPrim())

    def _add_glass(self, stage: Usd.Stage, path: str,
                   center: tuple[float, float, float],
                   size: tuple[float, float, float]) -> None:
        pane = UsdGeom.Cube.Define(stage, path)
        pane.CreateSizeAttr(1.0)
        pane.CreateDisplayColorAttr([Gf.Vec3f(0.80, 0.85, 0.90)])
        # 불투명 벽 — RTX 라이다가 관통 못 하게(AMCL 이 벽을 봐야 스캔↔맵 매칭됨, 2026-07-20).
        # 지붕은 여전히 없어 위에서 내려다보는 데모는 그대로. (반투명 0.06 → 라이다 통과라 폐기)
        pane.CreateDisplayOpacityAttr([1.0])
        xf = UsdGeom.Xformable(pane.GetPrim())
        xf.AddTranslateOp().Set(Gf.Vec3d(*center))
        xf.AddScaleOp().Set(Gf.Vec3f(*size))
        physics.add_shape_collider(pane.GetPrim())   # 로봇이 유리를 뚫고 나가지 못하게(static)
