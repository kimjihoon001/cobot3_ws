# -*- coding: utf-8 -*-
"""USD 물리 속성 헬퍼 — Collider / RigidBody / Mass / 마찰.

여기 쓰인 API 는 usd-core 로 실제 검증함 (속성명까지 확인):
  physics:kinematicEnabled / physics:approximation / physics:mass
  physics:staticFriction / physics:dynamicFriction / physics:breakForce

과실 설계 (성능 + 재현성):
  매달린 과실은 kinematic RigidBody 로 둔다. 물리 솔버가 매 스텝 풀지 않으므로
  수백 개를 놔둬도 비용이 거의 없고, Play/Stop 반복 시 항상 같은 자리에 있다.
  수확 순간에만 kinematic 을 꺼서 dynamic 으로 전환한다 (= 줄기에서 분리).
  -> 트라이앵글 메시 충돌 금지. 콜라이더는 convexHull 로 근사.
"""
from pxr import PhysxSchema, Usd, UsdGeom, UsdPhysics, UsdShade


def add_shape_collider(prim: Usd.Prim) -> None:
    """해석적 도형(Cube/Cylinder 등)에 콜라이더 부여. 온실 프레임·줄기용.

    도형 자체가 충돌 형상이므로 근사(approximation)가 필요 없다.
    """
    UsdPhysics.CollisionAPI.Apply(prim)


def add_mesh_colliders(stage: Usd.Stage, root_path: str,
                       approximation: str = "convexHull") -> int:
    """root_path 아래 모든 Mesh 에 콜라이더 부여. 참조된 USD(과실)용.

    참조로 들여온 USD 는 Xform 아래에 Mesh 가 들어있으므로, 콜라이더는
    최상위가 아니라 하위 Mesh 마다 붙어야 한다. RigidBody 는 상위에 두면
    하위 콜라이더들을 묶어서 하나의 강체로 다룬다.

    트라이앵글 메시 충돌은 금지(성능). convexHull 로 근사한다.
    반환: 콜라이더를 붙인 메시 개수.
    """
    n = 0
    root = stage.GetPrimAtPath(root_path)
    for prim in Usd.PrimRange(root):
        if prim.IsA(UsdGeom.Mesh):
            UsdPhysics.CollisionAPI.Apply(prim)
            UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr(
                approximation)
            n += 1
    return n


def add_convex_decomposition_colliders(
        stage: Usd.Stage, root_path: str,
        max_convex_hulls: int = 64, voxel_resolution: int = 500000,
        error_percentage: float = 1.0) -> int:
    """root_path 아래 모든 Mesh 에 convexDecomposition 콜라이더 부여. 오목 형상용.

    단일 convexHull 은 오목한 구멍(팔레트 포크 슬롯 등)을 메운다. 분해근사는 형상을
    여러 볼록덩이로 쪼개 그 사이 빈 공간을 살린다. 조각 수·복셀 해상도를 올려 얇은
    채널까지 보존한다. PhysX 세부 파라미터 API 는 버전마다 다를 수 있어 실패해도 안 죽게 감쌈.
    반환: 콜라이더를 붙인 메시 개수. (스파이크 06 실측 검증 — 포크 진입·리프트 성공)
    """
    n = 0
    for m in Usd.PrimRange(stage.GetPrimAtPath(root_path)):
        if not m.IsA(UsdGeom.Mesh):
            continue
        UsdPhysics.CollisionAPI.Apply(m)
        UsdPhysics.MeshCollisionAPI.Apply(m).CreateApproximationAttr(
            "convexDecomposition")
        try:
            cd = PhysxSchema.PhysxConvexDecompositionCollisionAPI.Apply(m)
            cd.CreateMaxConvexHullsAttr(max_convex_hulls)
            cd.CreateVoxelResolutionAttr(voxel_resolution)
            cd.CreateErrorPercentageAttr(error_percentage)
        except Exception as ex:
            print(f"  [collider] 세부 파라미터 생략(기본 분해 사용): {ex}")
        n += 1
    return n


def add_rigid_body(prim: Usd.Prim, density: float,
                   kinematic: bool = True) -> None:
    """RigidBody + Density 부여. 하위 콜라이더들을 묶어 하나의 강체가 된다.

    질량이 아니라 밀도(kg/m^3)를 준다. 질량은 PhysX 가 콜라이더 부피에서
    과실마다 계산한다. 변형별 부피가 6배까지 차이나므로(작은 unripe ~
    큰 ripe) 질량을 상수로 박으면 작은 과실이 납덩어리가 된다.

    kinematic=True  : 중력/충돌로 움직이지 않음 (매달린 과실)
    kinematic=False : 물리로 움직임 (수확된 과실)
    """
    rb = UsdPhysics.RigidBodyAPI.Apply(prim)
    rb.CreateKinematicEnabledAttr(kinematic)
    UsdPhysics.MassAPI.Apply(prim).CreateDensityAttr(density)


def set_kinematic(prim: Usd.Prim, kinematic: bool) -> None:
    """수확 순간 호출 — kinematic 을 꺼서 과실을 줄기에서 분리한다.

    물리적 breakForce 대신 이 방식을 쓰는 이유:
      breakForce 로 끊으면 매번 결과가 미세하게 달라져 재현성이 깨진다.
      코드로 끊으면 결정적이라 Play/Stop 반복 시 동일 결과가 나온다.
    """
    rb = UsdPhysics.RigidBodyAPI(prim)
    if rb:
        rb.GetKinematicEnabledAttr().Set(kinematic)


def create_physics_material(stage: Usd.Stage, path: str,
                            static_friction: float,
                            dynamic_friction: float,
                            restitution: float = 0.0) -> UsdShade.Material:
    mat = UsdShade.Material.Define(stage, path)
    api = UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
    api.CreateStaticFrictionAttr(static_friction)
    api.CreateDynamicFrictionAttr(dynamic_friction)
    api.CreateRestitutionAttr(restitution)
    return mat


def bind_physics_material(prim: Usd.Prim, material: UsdShade.Material) -> None:
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(
        material, UsdShade.Tokens.weakerThanDescendants, "physics")
