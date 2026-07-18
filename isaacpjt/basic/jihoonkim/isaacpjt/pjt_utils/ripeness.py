# -*- coding: utf-8 -*-
"""토마토 익음/불량 클래스 정의 + 색 적용 헬퍼 (Isaac Sim 5.1).

isaac/tomatest/tomato_lib.py 에서 이식 (테스트 스크립트 쪽은 자체 사본 유지).

클래스 근거 — 익음 기준을 우리가 정하지 않고 공개 데이터셋의 분류를 그대로 채택함.
익음과 불량은 출처가 다른 데이터셋이다. 발표 시 분리해서 제시할 것.

  [익음] green / half_ripe / fully_ripe
    Laboro Tomato 데이터셋 (USDA 토마토 색상 등급 6단계에 뿌리를 둠)
    정의 = 표면의 빨강 비율. 10% 미만 / 30~89% / 90% 이상.
    -> 수치 정의가 있어서 sim 에서 red_fraction 으로 정확히 재현 가능.

  [불량] old
    VegNet: Suryawanshi Y., Patil K., Chumchu P.
    "VegNet: Dataset of vegetable quality images for machine learning
     applications", Data in Brief 45:108657 (2022)
    DOI 10.1016/j.dib.2022.108657 / dataset DOI 10.17632/6nxnjbn9w6.1
    라이선스 CC BY
    토마토를 Unripe / Ripe / Old / Dried / Damaged 로 분류 (총 3,061장).
    그중 Old(1,234장) 를 채택. 클래스명도 VegNet 용어를 그대로 씀.

    한계(정직하게 밝힐 것): VegNet 은 클래스 이름만 주고 정량 정의를 주지
    않는다. 익음의 "빨강 %" 같은 수치 기준이 불량에는 없다.

    미채택: Damaged(물리적 손상) — 이번 수확 범위가 "딸 것(fully_ripe) vs
      버릴 것(old)" 이라 불필요. VegNet 에도 27장뿐이라 근거가 얇다.
      Dried — 토마토 이미지 0장.

색은 primvars:displayColor 로 입힘 → 머티리얼 셋업 없이도 RTX에서 렌더되고
YOLO 학습 이미지에 그대로 보임. half_ripe 는 높이(z) 기반 그라데이션으로
"빨강 몇 %"를 정확히 제어 → 클래스 정의(30~89%)에 매칭 + 라벨 자동 일치.
"""
import random
from pxr import Usd, UsdGeom, UsdShade, Gf, Sdf

GREEN = Gf.Vec3f(0.30, 0.55, 0.15)
RED   = Gf.Vec3f(0.85, 0.12, 0.08)
BROWN = Gf.Vec3f(0.32, 0.18, 0.10)

# red_fraction = 표면 중 빨강 비율 범위 (아래→빨강, 위(어깨)→초록)
CLASSES = {
    "green":      {"red_fraction": (0.00, 0.10)},  # 거의 초록      (Laboro)
    "half_ripe":  {"red_fraction": (0.30, 0.89)},  # 부분 착색      (Laboro)
    "fully_ripe": {"red_fraction": (0.90, 1.00)},  # 거의 빨강      (Laboro)
    "old":        {"red_fraction": None},          # 갈색 + 얼룩    (VegNet Old)
}


def _iter_meshes(stage, root_path):
    root = stage.GetPrimAtPath(root_path)
    for prim in Usd.PrimRange(root):
        if prim.IsA(UsdGeom.Mesh):
            yield UsdGeom.Mesh(prim)


def _set_vertex_colors(mesh, colors):
    pv = UsdGeom.PrimvarsAPI(mesh.GetPrim()).CreatePrimvar(
        "displayColor", Sdf.ValueTypeNames.Color3fArray, UsdGeom.Tokens.vertex)
    pv.Set(colors)


def apply_ripeness_color(stage, prim_path, class_name, rng=random):
    """prim_path 아래 모든 메시에 클래스 색을 displayColor 로 입힘.
    반환: 실제 사용된 red_fraction (라벨 메타 저장용, old 는 None)."""
    spec = CLASSES[class_name]
    frac = None
    for mesh in _iter_meshes(stage, prim_path):
        pts = mesh.GetPointsAttr().Get()
        if not pts:
            continue
        if class_name == "old":
            colors = []
            for _ in pts:
                j = rng.uniform(-0.05, 0.05)
                colors.append(Gf.Vec3f(BROWN[0] + j, BROWN[1] + j * 0.5, BROWN[2]))
        else:
            lo, hi = spec["red_fraction"]
            frac = rng.uniform(lo, hi)
            zs = [p[2] for p in pts]
            zmin, zmax = min(zs), max(zs)
            span = (zmax - zmin) or 1.0
            thresh = zmin + frac * span     # 이 높이 아래=빨강, 위=초록
            band = 0.12 * span              # 경계 부드럽게
            colors = []
            for p in pts:
                d = (p[2] - thresh) / band
                d += rng.uniform(-0.35, 0.35)          # 얼룩덜룩(자연스러운 착색)
                t = max(0.0, min(1.0, 0.5 + 0.5 * d))  # 0=빨강, 1=초록
                colors.append(RED * (1.0 - t) + GREEN * t)
        _set_vertex_colors(mesh, colors)
    return frac


def apply_flat_color(stage, prim_path, color):
    """단색 (꼭지=초록 등)."""
    for mesh in _iter_meshes(stage, prim_path):
        pv = UsdGeom.PrimvarsAPI(mesh.GetPrim()).CreatePrimvar(
            "displayColor", Sdf.ValueTypeNames.Color3fArray, UsdGeom.Tokens.constant)
        pv.Set([color])


def bind_matte_material(stage, prim_path,
                        mat_path="/World/Looks/MatteDisplayColor"):
    """displayColor 를 그대로 읽는 무광 머티리얼을 만들어 바인딩.

    머티리얼이 없으면 RTX 기본 재질(광택)로 렌더돼 과실이 유리구슬처럼 보인다
    (2026-07-18 GUI 확인). displayColor 는 YOLO 데이터셋의 클래스 정의
    (red_fraction)와 연동되므로 건드리지 않는다 — PrimvarReader 로 그대로
    diffuse 에 연결하고 roughness 만 무광으로 올린다. 하위 prim 전체에 상속됨.
    """
    if not stage.GetPrimAtPath(mat_path):
        mat = UsdShade.Material.Define(stage, mat_path)
        surf = UsdShade.Shader.Define(stage, mat_path + "/Surface")
        surf.CreateIdAttr("UsdPreviewSurface")
        surf.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.65)
        surf.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
        surf.CreateInput("specular", Sdf.ValueTypeNames.Float).Set(0.15)
        reader = UsdShade.Shader.Define(stage, mat_path + "/PrimvarReader")
        reader.CreateIdAttr("UsdPrimvarReader_float3")
        # varname 은 스펙상 token 이지만 Kit RTX 구현이 string 을 기대한다
        # (token 으로 넣으면 못 찾고 fallback 회색으로 렌더 — 2026-07-18 확인)
        reader.CreateInput("varname", Sdf.ValueTypeNames.String).Set("displayColor")
        reader.CreateInput("fallback", Sdf.ValueTypeNames.Float3).Set(
            Gf.Vec3f(0.5, 0.5, 0.5))
        surf.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(
            reader.CreateOutput("result", Sdf.ValueTypeNames.Float3))
        mat.CreateSurfaceOutput().ConnectToSource(
            surf.CreateOutput("surface", Sdf.ValueTypeNames.Token))
    else:
        mat = UsdShade.Material(stage.GetPrimAtPath(mat_path))
    UsdShade.MaterialBindingAPI.Apply(
        stage.GetPrimAtPath(prim_path)).Bind(mat)
