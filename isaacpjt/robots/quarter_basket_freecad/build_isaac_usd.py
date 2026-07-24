#!/usr/bin/env python3
"""Build the articulated Isaac USD from the FreeCAD-exported binary STL files."""

import argparse
import struct
from pathlib import Path

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True, "width": 640, "height": 480})

from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade


ROOT = "/CoaxialScoop"
HERE = Path(__file__).resolve().parent
GENERATED = HERE / "generated"


def read_binary_stl(path):
    data = path.read_bytes()
    if len(data) < 84:
        raise RuntimeError("invalid STL: {}".format(path))
    count = struct.unpack_from("<I", data, 80)[0]
    if len(data) != 84 + count * 50:
        raise RuntimeError("only binary STL is supported: {}".format(path))
    points = []
    for index in range(count):
        record = struct.unpack_from("<12fH", data, 84 + index * 50)
        for offset in (3, 6, 9):
            points.append(Gf.Vec3f(
                record[offset] * 0.001,
                record[offset + 1] * 0.001,
                record[offset + 2] * 0.001,
            ))
    return points


def material(stage, path, color, metallic, roughness):
    mat = UsdShade.Material.Define(stage, path)
    shader = UsdShade.Shader.Define(stage, path + "/Preview")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(
        Gf.Vec3f(*color))
    shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(metallic)
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(roughness)
    mat.CreateSurfaceOutput().ConnectToSource(
        shader.CreateOutput("surface", Sdf.ValueTypeNames.Token))
    return mat


def add_body(stage, name, stl_name, mat, mass):
    path = ROOT + "/" + name
    body = UsdGeom.Xform.Define(stage, path).GetPrim()
    body_xf = UsdGeom.Xformable(body)
    body_xf.AddTranslateOp().Set(Gf.Vec3d(0))
    UsdPhysics.RigidBodyAPI.Apply(body)
    UsdPhysics.MassAPI.Apply(body).CreateMassAttr(mass)
    physx_body = PhysxSchema.PhysxRigidBodyAPI.Apply(body)
    physx_body.CreateSolverPositionIterationCountAttr(16)
    physx_body.CreateSolverVelocityIterationCountAttr(4)

    points = read_binary_stl(GENERATED / stl_name)
    mesh = UsdGeom.Mesh.Define(stage, path + "/Mesh")
    mesh.CreatePointsAttr(points)
    mesh.CreateFaceVertexCountsAttr([3] * (len(points) // 3))
    mesh.CreateFaceVertexIndicesAttr(list(range(len(points))))
    mesh.CreateSubdivisionSchemeAttr("none")
    mesh.CreateDoubleSidedAttr(True)
    UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(mat)
    UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
    UsdPhysics.MeshCollisionAPI.Apply(
        mesh.GetPrim()).CreateApproximationAttr("convexDecomposition")
    physx_collision = PhysxSchema.PhysxCollisionAPI.Apply(mesh.GetPrim())
    physx_collision.CreateContactOffsetAttr(0.002)
    physx_collision.CreateRestOffsetAttr(0.0)
    return path


def revolute(stage, name, body0, body1, lower, upper, target=0.0):
    joint = UsdPhysics.RevoluteJoint.Define(stage, ROOT + "/Joints/" + name)
    joint.CreateBody0Rel().SetTargets([body0])
    joint.CreateBody1Rel().SetTargets([body1])
    joint.CreateAxisAttr("Z")
    joint.CreateLocalPos0Attr(Gf.Vec3f(0))
    joint.CreateLocalPos1Attr(Gf.Vec3f(0))
    joint.CreateLocalRot0Attr(Gf.Quatf(1))
    joint.CreateLocalRot1Attr(Gf.Quatf(1))
    joint.CreateLowerLimitAttr(lower)
    joint.CreateUpperLimitAttr(upper)
    drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "angular")
    drive.CreateTypeAttr("force")
    drive.CreateTargetPositionAttr(target)
    drive.CreateStiffnessAttr(1200.0)
    drive.CreateDampingAttr(80.0)
    drive.CreateMaxForceAttr(40.0)
    state = PhysxSchema.JointStateAPI.Apply(joint.GetPrim(), "angular")
    state.CreatePositionAttr().Set(float(target))
    state.CreateVelocityAttr().Set(0.0)
    return joint


def build(output):
    output.parent.mkdir(parents=True, exist_ok=True)
    stage = Usd.Stage.CreateNew(str(output))
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    root = UsdGeom.Xform.Define(stage, ROOT)
    stage.SetDefaultPrim(root.GetPrim())
    root.GetPrim().SetCustomDataByKey(
        "mechanism", "coaxial_ice_cream_scoop_gripper")
    root.GetPrim().SetCustomDataByKey("tool_axis", "+Z")
    root.GetPrim().SetCustomDataByKey("receiving_direction", "+Y")

    fixed_mat = material(
        stage, ROOT + "/Looks/Adapter", (0.42, 0.44, 0.47), 0.65, 0.30)
    inner_mat = material(
        stage, ROOT + "/Looks/Inner", (0.10, 0.52, 0.17), 0.05, 0.48)
    middle_mat = material(
        stage, ROOT + "/Looks/Middle", (0.08, 0.30, 0.66), 0.10, 0.42)
    cutter_mat = material(
        stage, ROOT + "/Looks/Cutter", (0.82, 0.37, 0.06), 0.55, 0.28)

    base = add_body(stage, "Base", "UR10eAdapter.stl", fixed_mat, 0.85)
    q1 = add_body(stage, "ScoopQuarter1", "ScoopQuarter1.stl", inner_mat, 0.16)
    q2 = add_body(stage, "ScoopQuarter2", "ScoopQuarter2.stl", middle_mat, 0.18)
    q3 = add_body(stage, "CutterQuarter3", "CutterQuarter3.stl", cutter_mat, 0.20)

    UsdGeom.Scope.Define(stage, ROOT + "/Joints")
    revolute(stage, "scoop_quarter_1_joint", base, q1, -90.0, 90.0, 0.0)
    revolute(stage, "scoop_quarter_2_joint", base, q2, -90.0, 0.0, -90.0)
    revolute(stage, "cutter_quarter_3_joint", base, q3, -180.0, 50.0, -180.0)

    tcp = UsdGeom.Xform.Define(stage, base + "/HarvestTCP")
    tcp.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 0.120))
    cut = UsdGeom.Xform.Define(stage, base + "/CuttingPoint")
    cut.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.044, 0.120))

    physics_mat = UsdShade.Material.Define(
        stage, ROOT + "/PhysicsMaterials/ScoopContact")
    api = UsdPhysics.MaterialAPI.Apply(physics_mat.GetPrim())
    api.CreateStaticFrictionAttr(1.1)
    api.CreateDynamicFrictionAttr(0.9)
    api.CreateRestitutionAttr(0.02)
    for body_path in (q1, q2, q3):
        mesh_prim = stage.GetPrimAtPath(body_path + "/Mesh")
        UsdShade.MaterialBindingAPI.Apply(mesh_prim).Bind(
            physics_mat, UsdShade.Tokens.weakerThanDescendants, "physics")

    stage.GetRootLayer().Save()
    reopened = Usd.Stage.Open(str(output))
    required = [
        ROOT + "/Base",
        ROOT + "/ScoopQuarter1",
        ROOT + "/ScoopQuarter2",
        ROOT + "/CutterQuarter3",
        ROOT + "/Joints/scoop_quarter_1_joint",
        ROOT + "/Joints/scoop_quarter_2_joint",
        ROOT + "/Joints/cutter_quarter_3_joint",
        ROOT + "/Base/HarvestTCP",
        ROOT + "/Base/CuttingPoint",
    ]
    missing = [path for path in required
               if not reopened.GetPrimAtPath(path).IsValid()]
    if missing:
        raise RuntimeError("generated USD is missing: {}".format(missing))
    print("generated:", output)
    print("validated: 4 bodies, 3 coaxial Z joints, TCP=0.120m")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-o", "--output",
        default=str(GENERATED / "coaxial_quarter_scoop_gripper.usd"))
    args = parser.parse_args()
    build(Path(args.output).resolve())


if __name__ == "__main__":
    try:
        main()
    finally:
        simulation_app.close()
