# -*- coding: utf-8 -*-
"""토마토 익음/불량 클래스 정의 + 색 적용 헬퍼 (Isaac Sim 5.1).

isaac/tomatest/tomato_lib.py 에서 이식 (테스트 스크립트 쪽은 자체 사본 유지).

클래스 근거 — 익음 기준을 우리가 정하지 않고 공개 데이터셋의 분류를 그대로 채택함.
익음과 불량은 출처가 다른 데이터셋이다. 발표 시 분리해서 제시할 것.

  2026-07-18 수확·운반으로 피벗 → 2클래스. 성숙 다단계(green/half_ripe)는 스코프 밖.
  씬이 완숙 위주라 "딸 것(익은거) vs 버릴 것(상한거)"만 구분한다.

  [익은거] ripe  = 수확 대상
    Laboro Tomato 데이터셋의 fully-ripe 정의(표면 빨강 90% 이상, USDA 색상 등급에
    뿌리). 수치 정의가 있어 sim 에서 red_fraction 으로 정확히 재현 가능.

  [상한거] spoiled = 제거 대상
    VegNet: Suryawanshi Y., Patil K., Chumchu P.
    "VegNet: Dataset of vegetable quality images for machine learning
     applications", Data in Brief 45:108657 (2022)
    DOI 10.1016/j.dib.2022.108657 / dataset DOI 10.17632/6nxnjbn9w6.1
    라이선스 CC BY. VegNet 의 Old(1,234장)/Damaged 계열을 "상한거"로 통합.
    한계(정직하게 밝힐 것): VegNet 은 클래스 이름만 주고 정량 정의가 없다.
    익음의 "빨강 %" 같은 수치 기준이 상한거에는 없다.

색은 primvars:displayColor 로 입힘 → 머티리얼 셋업 없이도 RTX에서 렌더되고
YOLO 학습 이미지에 그대로 보임. ripe 는 높이(z) 기반 그라데이션으로 빨강 비율을
제어(대부분 빨강) → 라벨과 자동 일치. spoiled 는 갈색+얼룩.
"""
import random
from pxr import Usd, UsdGeom, UsdShade, Gf, Sdf

GREEN = Gf.Vec3f(0.30, 0.55, 0.15)
RED   = Gf.Vec3f(0.85, 0.12, 0.08)
BROWN = Gf.Vec3f(0.32, 0.18, 0.10)

# red_fraction = 표면 중 빨강 비율 범위 (아래→빨강, 위(어깨)→초록)
CLASSES = {
    "ripe":    {"red_fraction": (0.90, 1.00)},  # 익은거=수확대상 (거의 빨강, Laboro fully-ripe)
    "spoiled": {"red_fraction": None},          # 상한거=제거대상 (갈색+얼룩, VegNet Old/Damaged)
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
    반환: 실제 사용된 red_fraction (라벨 메타 저장용, spoiled 는 None)."""
    spec = CLASSES[class_name]
    frac = None
    for mesh in _iter_meshes(stage, prim_path):
        pts = mesh.GetPointsAttr().Get()
        if not pts:
            continue
        if class_name == "spoiled":
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
                        mat_path="/World/Looks/MatteDisplayColor",
                        fallback_color=Gf.Vec3f(0.5, 0.5, 0.5)):
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
            fallback_color)
        surf.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(
            reader.CreateOutput("result", Sdf.ValueTypeNames.Float3))
        mat.CreateSurfaceOutput().ConnectToSource(
            surf.CreateOutput("surface", Sdf.ValueTypeNames.Token))
    else:
        mat = UsdShade.Material(stage.GetPrimAtPath(mat_path))
    UsdShade.MaterialBindingAPI.Apply(
        stage.GetPrimAtPath(prim_path)).Bind(mat)
