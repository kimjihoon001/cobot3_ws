# -*- coding: utf-8 -*-
"""토마토 익음/불량 클래스 정의 + 색 적용 헬퍼 (Isaac Sim 5.1).

2026-07-18 수확·운반 피벗 → 2클래스:
- ripe    : 익은거=수확 대상 (Laboro fully-ripe 정의, 표면 빨강 90%+)
- spoiled : 상한거=제거 대상 (VegNet 의 Old/Damaged 계열)
            Data in Brief 45:108657 (2022), CC BY / DOI 10.1016/j.dib.2022.108657

근거 상세는 pjt_utils/ripeness.py 의 docstring 참고 (같은 내용의 사본).
클래스명은 YOLO 라벨이 되므로 씬(pjt_utils/ripeness.py)과 반드시 일치해야 한다.

색은 primvars:displayColor 로 입힘 → 머티리얼 셋업 없이도 RTX에서 렌더되고
YOLO 학습 이미지에 그대로 보임. ripe 는 높이(z) 기반 그라데이션으로 빨강 비율을
제어(대부분 빨강). spoiled 는 갈색+얼룩.
"""
import random
from pxr import Usd, UsdGeom, Gf, Sdf

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
