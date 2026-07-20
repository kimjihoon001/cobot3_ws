"""Isaac Sim 팔레트 삽입구와 ForkliftB 포크 메시 치수 측정 도구.

실행::

    isaac_python probe_pallet_fork.py --/log/level=error

장면이나 원본 USD를 수정하지 않고 메시의 월드 바운딩박스와 주요 점 좌표를 출력한다.
"""
from collections import Counter

from isaacsim import SimulationApp

app = SimulationApp({"headless": True})

import omni.usd
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.storage.native import get_assets_root_path
from pxr import Usd, UsdGeom


stage = omni.usd.get_context().get_stage()
root = get_assets_root_path()
print("ASSET_ROOT", root, flush=True)
add_reference_to_stage(root + "/Isaac/Props/Pallet/pallet.usd", "/World/Pallet")
add_reference_to_stage(
    root + "/Isaac/Robots/IsaacSim/ForkliftB/forklift_b.usd", "/World/Forklift"
)
for _ in range(30):
    app.update()

cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
for base in ("/World/Pallet", "/World/Forklift"):
    print("\nBASE", base, "valid=", stage.GetPrimAtPath(base).IsValid(), flush=True)
    for prim in Usd.PrimRange(stage.GetPrimAtPath(base)):
        if not (prim.IsA(UsdGeom.Mesh) or prim.IsA(UsdGeom.Cube)):
            continue
        bound = cache.ComputeWorldBound(prim).ComputeAlignedBox()
        lo, hi = bound.GetMin(), bound.GetMax()
        size = hi - lo
        print(
            f"{prim.GetPath()}|"
            f"min={lo[0]:.5f},{lo[1]:.5f},{lo[2]:.5f}|"
            f"max={hi[0]:.5f},{hi[1]:.5f},{hi[2]:.5f}|"
            f"size={size[0]:.5f},{size[1]:.5f},{size[2]:.5f}",
            flush=True,
        )

        path = str(prim.GetPath())
        measured_mesh = path.endswith("Mesh_015") or path.endswith(
            "SM_Forklift_Lift_B01_01"
        )
        if not measured_mesh:
            continue

        mesh = UsdGeom.Mesh(prim)
        xf = UsdGeom.XformCache().GetLocalToWorldTransform(prim)
        points = [xf.Transform(point) for point in mesh.GetPointsAttr().Get()]
        print("POINTS", prim.GetPath(), len(points), flush=True)
        for axis, label in enumerate("XYZ"):
            common = Counter(
                round(float(point[axis]), 4) for point in points
            ).most_common(30)
            print("COMMON_" + label, common, flush=True)

        if not path.endswith("SM_Forklift_Lift_B01_01"):
            continue

        low_points = [
            point for point in points if point[2] < 0.35 and point[0] < -1.0
        ]
        print("FORK_LOW_POINTS", len(low_points), flush=True)
        for axis, label in enumerate("XYZ"):
            common = Counter(
                round(float(point[axis]), 4) for point in low_points
            ).most_common(40)
            print("FORK_LOW_" + label, common, flush=True)

        tine_points = [
            point for point in points if point[0] < -1.30 and point[2] < 0.18
        ]
        print("TINE_POINTS", len(tine_points), flush=True)
        for axis, label in enumerate("XYZ"):
            values = [float(point[axis]) for point in tine_points]
            if not values:
                continue
            print("TINE_RANGE_" + label, min(values), max(values), flush=True)
            print(
                "TINE_COMMON_" + label,
                Counter(round(value, 4) for value in values).most_common(50),
                flush=True,
            )

app.close()
