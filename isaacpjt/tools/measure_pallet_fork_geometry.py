# -*- coding: utf-8 -*-
"""Isaac 5.1 원본 팔레트 구멍과 ForkliftB 포크 날의 실측 좌표 출력.

실행::

    isaac_python tools/measure_pallet_fork_geometry.py --/log/level=error

씬이나 에셋은 수정하지 않는다. 메시 꼭짓점을 월드 좌표로 변환해 팔레트 슬롯의
수직 경계와 두 포크 날의 lift_joint=0 기준 높이를 출력한다.
"""
from collections import Counter

from isaacsim import SimulationApp

app = SimulationApp({"headless": True})

import omni.usd
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.storage.native import get_assets_root_path
from pxr import Usd, UsdGeom, UsdPhysics


def points_world(stage, suffix: str):
    for prim in Usd.PrimRange(stage.GetPseudoRoot()):
        if prim.IsA(UsdGeom.Mesh) and str(prim.GetPath()).endswith(suffix):
            xf = UsdGeom.XformCache().GetLocalToWorldTransform(prim)
            return [xf.Transform(p) for p in UsdGeom.Mesh(prim).GetPointsAttr().Get()]
    raise RuntimeError(f"mesh not found: {suffix}")


def common(values, digits=5, n=30):
    return Counter(round(float(v), digits) for v in values).most_common(n)


stage = omni.usd.get_context().get_stage()
root = get_assets_root_path()
add_reference_to_stage(root + "/Isaac/Props/Pallet/pallet.usd", "/World/Pallet")
add_reference_to_stage(
    root + "/Isaac/Robots/IsaacSim/ForkliftB/forklift_b.usd", "/World/Forklift"
)
for _ in range(30):
    app.update()

pallet = points_world(stage, "Mesh_015")
fork = points_world(stage, "SM_Forklift_Lift_B01_01")

print("PALLET_Z_LEVELS", common((p[2] for p in pallet), n=20), flush=True)
print("PALLET_X_LEVELS", common((p[0] for p in pallet), n=24), flush=True)

# 두 포크 날이 통과하는 폭대. ForkliftB 원본에서 날 중심은 Y≈±0.304m다.
all_tine = []
for sign, name in ((-1.0, "LEFT"), (1.0, "RIGHT")):
    tine = [
        p for p in fork
        if p[0] < -1.0
        and 0.245 < sign * p[1] < 0.365
        and p[2] < 0.40
    ]
    all_tine.extend(tine)
    print(
        f"FORK_TINE_{name}",
        f"count={len(tine)}",
        f"x=({min(p[0] for p in tine):.6f},{max(p[0] for p in tine):.6f})",
        f"y=({min(p[1] for p in tine):.6f},{max(p[1] for p in tine):.6f})",
        f"z=({min(p[2] for p in tine):.6f},{max(p[2] for p in tine):.6f})",
        flush=True,
    )
    print(f"FORK_TINE_{name}_Z_LEVELS", common((p[2] for p in tine)), flush=True)
    for x0, x1 in ((-2.10, -1.90), (-1.90, -1.70), (-1.70, -1.50),
                   (-1.50, -1.30), (-1.30, -1.10)):
        section = [p for p in tine if x0 <= p[0] < x1]
        if section:
            print(
                f"FORK_TINE_{name}_SECTION x=({x0:.2f},{x1:.2f}) "
                f"z=({min(p[2] for p in section):.6f},"
                f"{max(p[2] for p in section):.6f}) count={len(section)}",
                flush=True,
            )

# 실제 팔레트와 겹치는 포크의 직선 삽입 구간. 끝단 테이퍼 이후의 날 두께를 쓴다.
insertion_blade = [p for p in all_tine if -2.10 <= p[0] <= -1.50]
blade_bottom = min(float(p[2]) for p in insertion_blade)
blade_top = max(float(p[2]) for p in insertion_blade)
print(
    "FORK_INSERTION_BLADE_Z",
    f"bottom={blade_bottom:.6f}",
    f"top={blade_top:.6f}",
    f"center={(blade_bottom + blade_top) / 2.0:.7f}",
    flush=True,
)

for prim in Usd.PrimRange(stage.GetPrimAtPath("/World/Forklift")):
    if prim.IsA(UsdPhysics.PrismaticJoint):
        joint = UsdPhysics.PrismaticJoint(prim)
        print(
            "LIFT_JOINT",
            prim.GetPath(),
            "lower=", joint.GetLowerLimitAttr().Get(),
            "upper=", joint.GetUpperLimitAttr().Get(),
            flush=True,
        )

# 팔레트 구멍은 하부 데크 윗면과 상부 데크 아랫면 사이의 빈 구간이다.
z_levels = sorted(set(round(float(p[2]), 5) for p in pallet))
print("PALLET_ALL_Z", z_levels, flush=True)
hole_bottom, hole_top = 0.02053, 0.11605
print(
    "PALLET_HOLE_Z",
    f"bottom={hole_bottom:.5f}",
    f"top={hole_top:.5f}",
    f"center={(hole_bottom + hole_top) / 2.0:.5f}",
    flush=True,
)

app.close()
