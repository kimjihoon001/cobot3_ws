# -*- coding: utf-8 -*-
"""온실 프레임 스폰 — 기둥 + 상단 보 (패널 없는 골조).

프레임은 static collider. RigidBody 를 안 붙이므로 움직이지 않고,
로봇이 통과하지 못한다.

투명 지붕/벽 패널은 RTX 반투명 세팅 확인 후 다음 단계에서 추가.
"""
import os

from pxr import Usd, UsdGeom, Gf, Sdf

from pjt_config.settings import GreenhouseConfig
from pjt_utils import textures
from scene import physics

# 단위 박스(±0.5)의 6면. (면 법선, 정점 4개(반시계, 밖에서 볼 때), UV 로 쓸 두 축).
# 면마다 UV·노멀이 달라야 하므로 정점을 공유하지 않는다 (24점 / 6쿼드).
_BOX_FACES = (
    ((1, 0, 0), ((0.5, -0.5, -0.5), (0.5, 0.5, -0.5),
                 (0.5, 0.5, 0.5), (0.5, -0.5, 0.5)), (1, 2)),
    ((-1, 0, 0), ((-0.5, 0.5, -0.5), (-0.5, -0.5, -0.5),
                  (-0.5, -0.5, 0.5), (-0.5, 0.5, 0.5)), (1, 2)),
    ((0, 1, 0), ((0.5, 0.5, -0.5), (-0.5, 0.5, -0.5),
                 (-0.5, 0.5, 0.5), (0.5, 0.5, 0.5)), (0, 2)),
    ((0, -1, 0), ((-0.5, -0.5, -0.5), (0.5, -0.5, -0.5),
                  (0.5, -0.5, 0.5), (-0.5, -0.5, 0.5)), (0, 2)),
    ((0, 0, 1), ((-0.5, -0.5, 0.5), (0.5, -0.5, 0.5),
                 (0.5, 0.5, 0.5), (-0.5, 0.5, 0.5)), (0, 1)),
    ((0, 0, -1), ((-0.5, 0.5, -0.5), (0.5, 0.5, -0.5),
                  (0.5, -0.5, -0.5), (-0.5, -0.5, -0.5)), (0, 1)),
)


def _box_mesh(stage: Usd.Stage, path: str,
              size: tuple[float, float, float], tile_m: float) -> UsdGeom.Mesh:
    """UV 를 가진 단위 박스 Mesh. 실제 크기는 호출부의 ScaleOp 가 준다.

    UV 는 **월드 길이 기준**으로 매긴다 — 벽마다 크기가 다른데 0~1 로 매기면
    긴 벽에서 패널이 늘어난다. `면의 실제 길이 / tile_m` 만큼 반복시키면 어느
    벽에서든 패널 한 장이 같은 크기로 보인다.
    """
    pts, sts, nrm = [], [], []
    for normal, corners, (ai, bi) in _BOX_FACES:
        for p in corners:
            pts.append(Gf.Vec3f(*p))
            nrm.append(Gf.Vec3f(*normal))
            sts.append(Gf.Vec2f((p[ai] + 0.5) * size[ai] / tile_m,
                                (p[bi] + 0.5) * size[bi] / tile_m))
    mesh = UsdGeom.Mesh.Define(stage, path)
    mesh.CreatePointsAttr(pts)
    mesh.CreateFaceVertexCountsAttr([4] * len(_BOX_FACES))
    mesh.CreateFaceVertexIndicesAttr(list(range(len(pts))))
    mesh.CreateNormalsAttr(nrm)
    mesh.SetNormalsInterpolation(UsdGeom.Tokens.faceVarying)
    mesh.CreateExtentAttr([Gf.Vec3f(-0.5, -0.5, -0.5), Gf.Vec3f(0.5, 0.5, 0.5)])
    UsdGeom.PrimvarsAPI(mesh.GetPrim()).CreatePrimvar(
        "st", Sdf.ValueTypeNames.TexCoord2fArray,
        UsdGeom.Tokens.faceVarying).Set(sts)
    return mesh


class Greenhouse:
    def __init__(self, cfg: GreenhouseConfig):
        self._cfg = cfg
        self._wall_tex = (cfg.wall_textured and os.path.isfile(cfg.wall_texture))
        if cfg.wall_textured and not self._wall_tex:
            print(f"[WARN] 벽 텍스처 없음: {cfg.wall_texture} — 단색 벽으로 스폰"
                  " (tools/make_greenhouse_textures.py 를 먼저 실행)")

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
        c = self._cfg
        # 텍스처를 쓰면 Cube 대신 박스 Mesh — Cube 프리미티브엔 primvars:st 가 없어
        # UsdUVTexture 를 물릴 방법이 없다. 콜라이더는 boundingCube 라 거동은 동일.
        if self._wall_tex:
            pane = _box_mesh(stage, path, size, c.wall_tile_m)
        else:
            pane = UsdGeom.Cube.Define(stage, path)
            pane.CreateSizeAttr(1.0)
        pane.CreateDisplayColorAttr([Gf.Vec3f(0.80, 0.85, 0.90)])
        # 불투명 벽 — RTX 라이다가 관통 못 하게(AMCL 이 벽을 봐야 스캔↔맵 매칭됨, 2026-07-20).
        # 지붕은 여전히 없어 위에서 내려다보는 데모는 그대로. (반투명 0.06 → 라이다 통과라 폐기)
        pane.CreateDisplayOpacityAttr([1.0])
        xf = UsdGeom.Xformable(pane.GetPrim())
        xf.AddTranslateOp().Set(Gf.Vec3d(*center))
        xf.AddScaleOp().Set(Gf.Vec3f(*size))
        if self._wall_tex:
            physics.add_box_mesh_collider(pane.GetPrim())
            # 폴리카보네이트는 무광이 아니다 — matte(0.65)보다 낮춰 은은한 반사를 준다.
            textures.bind_texture(stage, path, "/World/Looks/GreenhousePanel",
                                  c.wall_texture, roughness=0.35)
        else:
            physics.add_shape_collider(pane.GetPrim())  # 로봇이 벽을 뚫고 나가지 못하게(static)
