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
from pxr import Gf, PhysxSchema, Usd, UsdGeom, UsdPhysics, UsdShade


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
    # 절단(kinematic→dynamic) 순간 손가락 침투복구 임펄스로 과실이 튕기는 것 방지.
    # ★ 스폰 시점에 박아야 한다 — PhysX 는 이 속성을 body 생성 때 읽으므로 런타임
    #   set_kinematic 시점에 붙이면 이미 스폰된 body 엔 안 먹는다(2026-07-22 실측).
    px = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
    px.CreateMaxDepenetrationVelocityAttr(0.05)      # m/s — 부드러운 복구로 마찰이 붙잡을 시간
    px.CreateSolverPositionIterationCountAttr(32)    # 접촉 수렴
    px.CreateSolverVelocityIterationCountAttr(4)


def set_kinematic(prim: Usd.Prim, kinematic: bool) -> None:
    """수확 순간 호출 — kinematic 을 꺼서 과실을 줄기에서 분리한다.

    물리적 breakForce 대신 이 방식을 쓰는 이유:
      breakForce 로 끊으면 매번 결과가 미세하게 달라져 재현성이 깨진다.
      코드로 끊으면 결정적이라 Play/Stop 반복 시 동일 결과가 나온다.
    """
    rb = UsdPhysics.RigidBodyAPI(prim)
    if not rb:
        return
    rb.GetKinematicEnabledAttr().Set(kinematic)
    if not kinematic:
        # 파지 중 kinematic 과실에 파고들어 있던 손가락이 dynamic 전환 순간 강한
        # 침투복구 임펄스로 과실을 튕겨낸다(2026-07-22 실측: finger 0.37→0.80 즉시 이탈).
        # 침투복구 속도를 낮추고(부드럽게 밀어냄) 접촉 반복을 늘려 마찰(μ0.9)이 붙잡게 한다.
        px = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
        px.CreateMaxDepenetrationVelocityAttr(0.1)      # m/s (기본 수 m/s → 큰 임펄스)
        px.CreateSolverPositionIterationCountAttr(32)   # 접촉 수렴
        px.CreateSolverVelocityIterationCountAttr(4)
        px.CreateEnableCCDAttr(True)                     # 얇은 손가락 사이 터널링 방지


def disable_physics(stage: Usd.Stage, root_path: str) -> int:
    """서브트리의 물리 잔재(강체·PhysX·콜라이더·아티큘레이션루트·내부 조인트)를 제거해
    순수 시각 프림으로 만든다.

    참조 에셋(예: KLT 빈 small_KLT.usd)이 자체 강체를 갖고 오면, 강체 프림 밑에 자식으로
    넣을 때 PhysX 가 '중첩 강체(missing xformstack reset)' 경고를 매 프레임 낸다. 이 경고는
    RigidBodyAPI 의 '존재'(계층 구조)를 보고 뜨므로 enabled=False 로는 안 사라진다 —
    RemoveAPI 로 실제 제거해야 한다(harvester._add_camera_at 에서 검증된 방식). 참조로 온
    API 도 RemoveAPI 는 컴포지션에 삭제를 얹어 먹는다(D455 에서 실측). 반환: 강체 제거 수.
    """
    n = 0
    for p in Usd.PrimRange(stage.GetPrimAtPath(root_path)):
        if p.IsA(UsdPhysics.Joint):                 # 에셋 내부 조인트 비활성
            p.SetActive(False)
            continue
        if p.HasAPI(UsdPhysics.RigidBodyAPI):
            p.RemoveAPI(UsdPhysics.RigidBodyAPI)
            n += 1
        if p.HasAPI(PhysxSchema.PhysxRigidBodyAPI):
            p.RemoveAPI(PhysxSchema.PhysxRigidBodyAPI)
        if p.HasAPI(UsdPhysics.CollisionAPI):
            p.RemoveAPI(UsdPhysics.CollisionAPI)
        if p.HasAPI(UsdPhysics.ArticulationRootAPI):
            p.RemoveAPI(UsdPhysics.ArticulationRootAPI)
    return n


def create_fixed_joint(stage: Usd.Stage, path: str,
                       body0_path: str, body1_path: str):
    """두 강체를 현재 상대 포즈로 고정. body0=기준(예: 로봇 링크), body1=붙일 강체.

    §8(MountJoint 폭발) 교훈: localPos/Rot 을 기본 identity 로 두면 PhysX 가 두 몸의
    원점을 강제로 일치시키려는 충격으로 시작하자마자 폭발한다. 반드시 '현재 상대 포즈'를
    로컬 프레임에 적어야 한다. 앵커를 body1 원점에 두면 → localFrame1=identity,
    localFrame0 = body1_world · body0_world⁻¹ (row-vector). 대상은 실제 링크여야 한다.
    """
    from pxr import Gf

    j = UsdPhysics.FixedJoint.Define(stage, path)
    j.CreateBody0Rel().SetTargets([body0_path])
    j.CreateBody1Rel().SetTargets([body1_path])
    # 묶인 두 몸(로봇 링크↔적재물)의 상호 충돌을 끈다. 데크 위 적재물 콜라이더가 로봇 자체
    # 콜라이더와 겹치면 조인트는 붙잡고 접촉force는 밀어내며 솔버가 싸워 로봇이 요동친다.
    j.CreateCollisionEnabledAttr(False)
    cw = UsdGeom.Xformable(stage.GetPrimAtPath(body0_path)).ComputeLocalToWorldTransform(
        Usd.TimeCode.Default())
    lw = UsdGeom.Xformable(stage.GetPrimAtPath(body1_path)).ComputeLocalToWorldTransform(
        Usd.TimeCode.Default())
    rel = lw * cw.GetInverse()                 # body1 원점을 body0 프레임으로
    t = rel.ExtractTranslation()
    q = rel.ExtractRotationQuat()
    im = q.GetImaginary()
    j.CreateLocalPos0Attr(Gf.Vec3f(float(t[0]), float(t[1]), float(t[2])))
    j.CreateLocalRot0Attr(Gf.Quatf(float(q.GetReal()),
                                   float(im[0]), float(im[1]), float(im[2])))
    j.CreateLocalPos1Attr(Gf.Vec3f(0.0, 0.0, 0.0))
    j.CreateLocalRot1Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    return j


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


def set_material_friction(stage: Usd.Stage, mat_path: str, mu: float) -> bool:
    """기존 물리 머티리얼의 마찰(static=dynamic=mu)을 실시간 변경 + combineMode='min'.
    스파이크 마찰 스윕용(2026-07-22): 과실/줄기 머티리얼만 바꿔도 combineMode=min 이라
    유효 마찰 = min(mu, 그리퍼 0.9) = mu (mu≤0.9). 케이스마다 Isaac 재시작 없이 μ 변경."""
    mat = stage.GetPrimAtPath(mat_path)
    if not mat.IsValid():
        return False
    api = UsdPhysics.MaterialAPI.Apply(mat)
    api.CreateStaticFrictionAttr(float(mu))
    api.CreateDynamicFrictionAttr(float(mu))
    # 두 접촉면 마찰 결합을 'min' 으로 → 과실 μ 하나가 유효 마찰을 지배(깨끗한 스윕변수).
    px = PhysxSchema.PhysxMaterialAPI.Apply(mat)
    px.CreateFrictionCombineModeAttr().Set("min")
    return True


def add_sphere_collider(stage: Usd.Stage, path: str, radius: float,
                        center: tuple[float, float, float] = (0.0, 0.0, 0.0)) -> None:
    """파지용 작은 구 콜라이더 — 시각 메시(convexHull)보다 작게 둬 그리퍼가 어긋나도
    손가락이 과실을 안 때리고 감싸게 한다(2026-07-22). 안 보이는 해석적 구(정확).
    radius·center 는 프림 로컬 단위(부모 스케일이 곱해져 월드 크기가 된다).

    center: 과실 원점이 메시 중심과 안 맞을 때(토마토 USD 미centering) 구를 메시 중심으로
    옮긴다. 파지 목표(_publish_sim_tomato 의 bbox 중심)와 일치시켜야 그리퍼가 헛 닫히지
    않는다("치고 간다" 방지 2026-07-22)."""
    sph = UsdGeom.Sphere.Define(stage, path)
    sph.CreateRadiusAttr(float(radius))
    if any(center):
        UsdGeom.Xformable(sph.GetPrim()).AddTranslateOp().Set(
            Gf.Vec3d(float(center[0]), float(center[1]), float(center[2])))
    UsdGeom.Imageable(sph.GetPrim()).MakeInvisible()   # 충돌 전용, 시각 숨김
    UsdPhysics.CollisionAPI.Apply(sph.GetPrim())


def add_cylinder_collider(stage: Usd.Stage, path: str, radius: float, height: float,
                          center: tuple[float, float, float] = (0.0, 0.0, 0.0),
                          visible: bool = True) -> Usd.Prim:
    """그립용 원통 콜라이더(줄기) — 과실 위에 수직 원통. 강체 구는 평행패드에서 squeeze-pop
    으로 튕기지만 원통은 옆면을 물어 마찰로 확실히 잡힌다(2026-07-22 줄기 파지). radius·
    height·center 는 프림 로컬 단위(부모 스케일이 곱해져 월드 크기). 반환: 원통 prim."""
    cyl = UsdGeom.Cylinder.Define(stage, path)
    cyl.CreateRadiusAttr(float(radius))
    cyl.CreateHeightAttr(float(height))
    cyl.CreateAxisAttr("Z")
    cyl.CreateExtentAttr([Gf.Vec3f(-radius, -radius, -height / 2.0),
                          Gf.Vec3f(radius, radius, height / 2.0)])
    if any(center):
        UsdGeom.Xformable(cyl.GetPrim()).AddTranslateOp().Set(
            Gf.Vec3d(float(center[0]), float(center[1]), float(center[2])))
    if not visible:
        UsdGeom.Imageable(cyl.GetPrim()).MakeInvisible()
    UsdPhysics.CollisionAPI.Apply(cyl.GetPrim())
    return cyl.GetPrim()


def add_box_collider(stage: Usd.Stage, path: str,
                     size: tuple[float, float, float],
                     center: tuple[float, float, float] = (0.0, 0.0, 0.0),
                     visible: bool = True) -> Usd.Prim:
    """그립용 육면체 콜라이더 — 평평한 면이라 평행조가 '면 접촉(form-fit)'으로 안정 파지.
    강체 구는 점접촉 squeeze-pop, 얇은 원통은 지나침이 있지만 육면체는 면이 맞물려 확실히 잡힌다
    (2026-07-23 사용자 아이디어). size=(sx,sy,sz) 프림 로컬 전체 크기(부모 스케일 곱해져 월드)."""
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)                              # 단위정육면체(-0.5~0.5) → 스케일로 크기
    xf = UsdGeom.Xformable(cube.GetPrim())
    if any(center):
        xf.AddTranslateOp().Set(Gf.Vec3d(float(center[0]), float(center[1]), float(center[2])))
    xf.AddScaleOp().Set(Gf.Vec3f(float(size[0]), float(size[1]), float(size[2])))
    if not visible:
        UsdGeom.Imageable(cube.GetPrim()).MakeInvisible()
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    return cube.GetPrim()
