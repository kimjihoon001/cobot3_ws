# -*- coding: utf-8 -*-
"""USD xform 헬퍼 — reference 로 얹은 prim 에 안전하게 위치를 건다.

`add_reference_to_stage()` 로 만든 prim 은 참조한 에셋의 xformOpOrder
(예: [translate, orient, scale])를 그대로 물려받는다. 거기에 `AddTranslateOp()` 를
다시 부르면 "xformOp 'xformOp:translate' already exists" 로 터진다
(NVIDIA 로봇 에셋에서 확인 — CLAUDE.md §8 2026-07-18).
→ 이미 있는 translate op 는 재사용하고, 없을 때만 새로 만든다.
"""
from pxr import Gf, Usd, UsdGeom


def set_pose(prim: Usd.Prim, pos, quat: Gf.Quatd) -> None:
    """prim 의 local translate+orient 를 설정. 기존 op 재사용, 없으면 생성.

    xformOpOrder 는 [translate, orient(, ...)] — 회전 먼저, 이동 나중의 표준 TRS.
    """
    xf = UsdGeom.Xformable(prim)
    t_op = o_op = None
    for op in xf.GetOrderedXformOps():
        if op.GetOpType() == UsdGeom.XformOp.TypeTranslate and t_op is None:
            t_op = op
        elif op.GetOpType() == UsdGeom.XformOp.TypeOrient and o_op is None:
            o_op = op
    if t_op is None:
        t_op = xf.AddTranslateOp()
    if o_op is None:
        o_op = xf.AddOrientOp()
    if t_op.GetPrecision() == UsdGeom.XformOp.PrecisionFloat:
        t_op.Set(Gf.Vec3f(*pos))
    else:
        t_op.Set(Gf.Vec3d(*pos))
    if o_op.GetPrecision() == UsdGeom.XformOp.PrecisionFloat:
        o_op.Set(Gf.Quatf(quat))
    else:
        o_op.Set(Gf.Quatd(quat))


def set_translate(prim: Usd.Prim, vec) -> None:
    """prim 의 local translate 를 설정. 기존 translate op 가 있으면 재사용한다.

    vec: 3개짜리 (x, y, z) 튜플이나 Gf.Vec3d/Vec3f 무엇이든 받는다.
    """
    xf = UsdGeom.Xformable(prim)
    for op in xf.GetOrderedXformOps():
        if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
            # 기존 op 의 정밀도에 맞춰 값을 넣는다 (에셋마다 float/double 다름).
            if op.GetPrecision() == UsdGeom.XformOp.PrecisionFloat:
                op.Set(Gf.Vec3f(*vec))
            else:
                op.Set(Gf.Vec3d(*vec))
            return
    xf.AddTranslateOp().Set(Gf.Vec3d(*vec))
