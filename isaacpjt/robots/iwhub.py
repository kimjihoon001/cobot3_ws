# -*- coding: utf-8 -*-
"""운반 AMR (iw.hub, 언더라이드) — 팔레트+KLT 세트를 싣고 MM↔창고를 오간다.

물류 루프(2026-07-19 확정): MM 은 iw.hub 위 KLT 에 과실을 넣기만 하고, iw.hub 가
팔레트째 나르며, 창고에서 지게차와 표준 팔레트 교환을 한다 — MM→운반 크레이트
이관(근거 없던 갭)이 아예 없다.

에셋 실측 (2026-07-19 Nucleus, tools/iwhub_bridge_check.py 으로 스폰·ROS2 구동 검증):
  1431×659×231mm, 페이로드 1000kg — 폭 0.66m < 이랑 1.5m → 통로 주행 OK.
  DOF: left/right_wheel_joint(차동 구동, 속도), lift_joint(승강, 위치).

이 모듈은 **놓기만 한다** (harvester/transporter 와 같은 규칙). 제어는 ROS2 가
/{ns}/joint_command(JointState) 로 직접 한다 — ros/robot_bridge.py 참조(§5.6).
"""
from __future__ import annotations

import math
import os
import random

from pxr import Gf, PhysxSchema, Usd, UsdGeom, UsdPhysics, UsdShade

from pjt_config.settings import RobotConfig
from pjt_utils.deck_geometry import (
    IW_LOAD_MAP_X_OFFSET_M,
    PALLET_SUPPORT_CLEARANCE,
)
from pjt_utils.xform import set_pose, set_scale
from robots import assets


class IwHub:
    """iw.hub 운반 AMR. 구조: {root} <- iw_hub.usd 참조 (아티큘레이션 루트 = root)."""

    # 실측 DOF 이름 (2026-07-19) — ROS2 JointState 의 name 필드에 이대로 쓴다.
    DRIVE_JOINTS = ("left_wheel_joint", "right_wheel_joint")   # 속도 명령(차동)
    LIFT_JOINT = "lift_joint"                                   # 위치 명령(승강)
    # 적재 팔레트의 접지 마찰을 이기고 좌우 바퀴를 반대로 돌릴 수 있는 velocity drive.
    # angular drive의 damping은 속도 오차에 대한 구동 토크 이득, maxForce는 토크 상한이다.
    # 적재 상태에서 베드/장애물 모서리에 닿아도 전후진·제자리 회전 명령을
    # 실제 바퀴 속도로 밀어낼 수 있게 기존 대비 2배로 보강한다.
    DRIVE_DAMPING = 3000.0
    DRIVE_MAX_FORCE = 5000.0
    WHEEL_STATIC_FRICTION = 1.2
    WHEEL_DYNAMIC_FRICTION = 1.0
    # position[2]는 에셋 루트 높이가 아니라 "주행 바닥 높이"로 취급한다.
    # iw_hub.usd 원점이 차체 중앙 근처라 루트를 바닥 높이에 그대로 놓으면 하부가
    # 약 반 높이만큼 바닥에 박힌다. 로컬 bbox 최저점을 바닥보다 5mm 위에 놓고
    # reset 뒤 중력으로 짧게 정착시켜 초기 관통/반발을 없앤다.
    GROUND_CLEARANCE = 0.005
    # 정지 중 접촉 솔버가 만드는 미세 병진·회전을 빨리 안정화한다. 구동 중에는
    # wheel drive가 아티큘레이션을 깨우므로 주행 응답을 막지 않는다.
    SLEEP_THRESHOLD = 0.01
    STABILIZATION_THRESHOLD = 0.002
    SOLVER_POSITION_ITERATIONS = 16
    SOLVER_VELOCITY_ITERATIONS = 4
    MAX_DEPENETRATION_VELOCITY = 0.5

    def __init__(self, cfg: RobotConfig):
        self._cfg = cfg
        self._root: str | None = None
        self._lidars: list = []          # LidarRtx 참조 보관(GC 되면 렌더프로덕트 파괴됨)

    @property
    def root(self) -> str | None:
        return self._root

    def spawn(self, stage: Usd.Stage, root: str = "/World/IwHub",
              position: tuple[float, float, float] = (0.0, 0.0, 0.0),
              yaw_deg: float = 0.0, log=print) -> str:
        """놓는다. 반환: root 경로."""
        from isaacsim.core.utils.stage import add_reference_to_stage

        url = assets.resolve(self._cfg.assets.iwhub, "운반 AMR(iw.hub)")
        log(f"[IwHub] 에셋 {url}")
        add_reference_to_stage(url, root)
        grounded_position = self._grounded_position(
            stage, root, position, log=log
        )
        # 참조 prim 은 자체 xformOp 을 가질 수 있다 → 기존 op 재사용(§8).
        # yaw=180°이면 긴 후방 오버행이 MM 반대쪽을 향해 추종 회전 시 충돌하지 않는다.
        yaw = Gf.Rotation(Gf.Vec3d(0.0, 0.0, 1.0), yaw_deg).GetQuat()
        set_pose(
            stage.GetPrimAtPath(root), grounded_position,
            Gf.Quatd(yaw.GetReal(), yaw.GetImaginary()),
        )
        self._root = root
        self._configure_drive_torque(stage, log)
        self._configure_rest_stability(stage, log)
        log(
            f"[IwHub] 배치 완료: {root} @ "
            f"{tuple(round(v, 3) for v in grounded_position)}, "
            f"yaw={yaw_deg:.1f}°"
        )
        return root

    def _grounded_position(
        self,
        stage: Usd.Stage,
        root: str,
        position: tuple[float, float, float],
        log=print,
    ) -> tuple[float, float, float]:
        """에셋 로컬 bbox 최저점을 주행 바닥 바로 위로 맞춘 루트 pose를 반환한다."""
        prim = stage.GetPrimAtPath(root)
        try:
            bbox_cache = UsdGeom.BBoxCache(
                Usd.TimeCode.Default(),
                [
                    UsdGeom.Tokens.default_,
                    UsdGeom.Tokens.render,
                ],
            )
            local_range = (
                bbox_cache.ComputeLocalBound(prim).ComputeAlignedRange()
            )
            local_min_z = float(local_range.GetMin()[2])
            if not math.isfinite(local_min_z):
                raise ValueError(f"유효하지 않은 bbox min z: {local_min_z}")
        except Exception as exc:
            log(
                "[IwHub] ⚠ 로컬 bbox 접지 높이 계산 실패 — 요청 높이를 그대로 사용: "
                f"{exc}"
            )
            return tuple(float(v) for v in position)

        floor_z = float(position[2])
        root_z = floor_z - local_min_z + self.GROUND_CLEARANCE
        log(
            "[IwHub] 바닥 관통 방지 자동 보정: "
            f"floor_z={floor_z:.3f}m, local_min_z={local_min_z:+.3f}m, "
            f"root_z={root_z:.3f}m, clearance={self.GROUND_CLEARANCE:.3f}m"
        )
        return float(position[0]), float(position[1]), root_z

    @staticmethod
    def _set_physx_attr(api, creator: str, value, log=print) -> bool:
        """Isaac/PhysX 버전별 선택 속성을 지원되는 경우에만 설정한다."""
        fn = getattr(api, creator, None)
        if fn is None:
            log(f"[IwHub] ⚠ PhysX 속성 API 없음: {creator}")
            return False
        fn(value)
        return True

    def _configure_rest_stability(self, stage: Usd.Stage, log=print) -> None:
        """IW 아티큘레이션의 초기 관통 반발과 정지 잔진동을 줄인다."""
        if not self._root:
            return
        root_prim = stage.GetPrimAtPath(self._root)
        articulation_prim = next(
            (
                prim
                for prim in Usd.PrimRange(root_prim)
                if prim.HasAPI(UsdPhysics.ArticulationRootAPI)
            ),
            None,
        )
        if articulation_prim is None:
            log("[IwHub] ⚠ 안정화할 articulation root를 찾지 못했습니다")
            return

        articulation = PhysxSchema.PhysxArticulationAPI.Apply(
            articulation_prim
        )
        settings = (
            ("CreateSleepThresholdAttr", self.SLEEP_THRESHOLD),
            ("CreateStabilizationThresholdAttr", self.STABILIZATION_THRESHOLD),
            (
                "CreateSolverPositionIterationCountAttr",
                self.SOLVER_POSITION_ITERATIONS,
            ),
            (
                "CreateSolverVelocityIterationCountAttr",
                self.SOLVER_VELOCITY_ITERATIONS,
            ),
        )
        for creator, value in settings:
            try:
                self._set_physx_attr(articulation, creator, value, log=log)
            except Exception as exc:
                log(f"[IwHub] ⚠ {creator} 설정 실패: {exc}")

        # 초기 bbox 오차가 남더라도 한 프레임에 큰 관통 보정 속도가 생기지 않게
        # 섀시 강체의 depenetration 속도를 제한한다.
        chassis = stage.GetPrimAtPath(f"{self._root}/chassis")
        if (
            not chassis.IsValid()
            or not chassis.HasAPI(UsdPhysics.RigidBodyAPI)
        ):
            chassis = next(
                (
                    prim
                    for prim in Usd.PrimRange(root_prim)
                    if prim.HasAPI(UsdPhysics.RigidBodyAPI)
                    and "chassis" in prim.GetName().lower()
                ),
                None,
            )
        if chassis is not None and chassis.IsValid():
            rigid = PhysxSchema.PhysxRigidBodyAPI.Apply(chassis)
            try:
                self._set_physx_attr(
                    rigid,
                    "CreateMaxDepenetrationVelocityAttr",
                    self.MAX_DEPENETRATION_VELOCITY,
                    log=log,
                )
            except Exception as exc:
                log(f"[IwHub] ⚠ chassis depenetration 제한 설정 실패: {exc}")
            chassis_path = str(chassis.GetPath())
        else:
            chassis_path = "찾지 못함"

        log(
            "[IwHub] 정지 안정화: "
            f"sleep={self.SLEEP_THRESHOLD:.3f}, "
            f"stabilization={self.STABILIZATION_THRESHOLD:.3f}, "
            f"solver={self.SOLVER_POSITION_ITERATIONS}/"
            f"{self.SOLVER_VELOCITY_ITERATIONS}, chassis={chassis_path}"
        )

    def _configure_drive_torque(self, stage: Usd.Stage, log=print) -> None:
        """적재 상태에서도 제자리 회전하도록 wheel drive와 접지 마찰을 보강한다."""
        if not self._root:
            return
        material = UsdShade.Material.Define(
            stage, "/World/PhysicsMaterials/IwHubDriveWheel")
        material_api = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
        material_api.CreateStaticFrictionAttr(self.WHEEL_STATIC_FRICTION)
        material_api.CreateDynamicFrictionAttr(self.WHEEL_DYNAMIC_FRICTION)
        material_api.CreateRestitutionAttr(0.0)
        try:
            physx_material = PhysxSchema.PhysxMaterialAPI.Apply(material.GetPrim())
            physx_material.CreateFrictionCombineModeAttr("max")
            physx_material.CreateRestitutionCombineModeAttr("min")
        except Exception as exc:
            log(f"[IwHub] ⚠ PhysX 마찰 결합 모드 설정 생략: {exc}")

        configured = []
        wheel_bodies = []
        root_prim = stage.GetPrimAtPath(self._root)
        for prim in Usd.PrimRange(root_prim):
            if prim.GetName() not in self.DRIVE_JOINTS:
                continue
            drive = UsdPhysics.DriveAPI.Get(prim, "angular")
            if not drive:
                log(f"[IwHub] ⚠ {prim.GetName()} angular drive 없음 — 토크 보강 생략")
                continue
            # stiffness=0이면 위치를 붙잡지 않고 ROS velocityCommand만 추종한다.
            drive.GetStiffnessAttr().Set(0.0)
            drive.GetDampingAttr().Set(self.DRIVE_DAMPING)
            drive.GetMaxForceAttr().Set(self.DRIVE_MAX_FORCE)
            configured.append(prim.GetName())

            # 조인트가 연결한 두 body 중 wheel 쪽에 강한 물리 재질을 상속시킨다.
            joint = UsdPhysics.Joint(prim)
            targets = (
                list(joint.GetBody0Rel().GetTargets())
                + list(joint.GetBody1Rel().GetTargets())
            )
            wheel_targets = [p for p in targets if "wheel" in p.name.lower()]
            if not wheel_targets and targets:
                wheel_targets = [targets[-1]]
            for target in wheel_targets:
                wheel_prim = stage.GetPrimAtPath(target)
                if not wheel_prim.IsValid() or str(target) in wheel_bodies:
                    continue
                UsdShade.MaterialBindingAPI.Apply(wheel_prim).Bind(
                    material, UsdShade.Tokens.strongerThanDescendants, "physics")
                wheel_bodies.append(str(target))
        if configured:
            log(
                "[IwHub] 구동륜 토크 보강: "
                f"joints={configured}, damping={self.DRIVE_DAMPING:.0f}, "
                f"maxForce={self.DRIVE_MAX_FORCE:.0f}"
            )
            if wheel_bodies:
                log(
                    "[IwHub] 구동륜 드리프트 억제: "
                    f"bodies={wheel_bodies}, μs={self.WHEEL_STATIC_FRICTION:.2f}, "
                    f"μd={self.WHEEL_DYNAMIC_FRICTION:.2f}, combine=max"
                )
            else:
                log("[IwHub] ⚠ wheel body를 못 찾아 마찰 재질을 적용하지 못함")
        else:
            log("[IwHub] ⚠ 좌우 구동 조인트를 못 찾아 토크 보강을 적용하지 못함")

    def load_cargo(self, stage: Usd.Stage, tomato_cfg, phys_cfg,
                   deck_z: float = 0.225, log=print,
                   pallet_phys_cfg=None) -> int:
        """iw.hub 데크에 '적재된 세트' — 팔레트 + KLT 8개 + 3칸에 토마토 5개씩(15개, 꼭지 포함).

        ★ 물리 구조 (사용자 정정 2026-07-20 "고정조인트로 결속 / 포크슬롯 살려야"):
          · Load(팔레트+KLT) = **하나의 동적 강체**. chassis 데크에 FixedJoint 로 결속 →
            로봇이 주행하면 강체로 따라간다(아티큘레이션 밑 중첩 아님 — 별도 강체+조인트).
          · 팔레트 = **convexDecomposition 콜라이더** → 포크 슬롯(구멍) 살림. 창고에서
            지게차 포크가 들어가고, DeckJoint 를 SetActive(False) 로 풀면 인수된다(루프 후속).
          · 채운 KLT 3칸 = 오목 콜라이더(그릇, Load 의 일부) — 토마토를 담아 흘리지 않는다.
          · 토마토 = **별도 동적 강체**(Load 아님) → KLT 안에서 흔들리며 접촉으로 실려간다.
          참조 에셋(팔레트·KLT)이 자체 강체를 갖고 오므로 disable_physics 로 벗긴 뒤
          Load 강체의 콜라이더로 붙인다(중첩강체 경고 방지 §8). 꼭지=몸통 자식(장식).
        deck_z: chassis 형상을 읽지 못했을 때만 쓰는 root 기준 비상 오프셋.
                정상 경로는 실제 chassis bbox 상면을 매번 측정해 팔레트 하면을 맞춘다.
        반환: 얹은 토마토 수(0이면 에셋 없음).
        """
        from isaacsim.core.utils.stage import add_reference_to_stage

        from pjt_utils import ripeness
        from scene import physics                          # 물리 헬퍼 재사용(읽기)
        from scene.warehouse import KLT_SIZE, PALLET_SIZE  # 프랍 치수 재사용(§5.7)

        if not self._root:
            return 0
        try:
            pallet_url = assets.resolve(self._cfg.assets.pallet, "팔레트")
            klt_url = assets.resolve(self._cfg.assets.klt_bin, "KLT 빈")
        except Exception as e:
            log(f"[IwHub] 적재 장식 스킵 — 프랍 에셋 없음: {e}")
            return 0
        # 익은 토마토 (몸통, 꼭지) 쌍
        d = tomato_cfg.usd_dir
        ripe = []
        if os.path.isdir(d):
            for f in sorted(os.listdir(d)):
                if (f.startswith("tomato_ripe_") and f.endswith(".usd")
                        and not f.endswith("_calyx.usd")):
                    calyx = os.path.join(d, f[:-4] + "_calyx.usd")
                    ripe.append((os.path.join(d, f),
                                 calyx if os.path.exists(calyx) else None))

        # ── iw.hub 데크 월드 포즈 (적재 세트 원점) ──
        # 컨테이너 root 원점은 실제 차체 기하 중심과 일치하지 않는다. root 오프셋이나
        # 부모 변환을 다시 적용하지 않도록 chassis의 월드 bbox 중심/상면을 직접 쓴다.
        src = f"{self._root}/base_link"
        if not stage.GetPrimAtPath(src).IsValid():
            src = self._root
        bp = UsdGeom.Xformable(stage.GetPrimAtPath(src)).ComputeLocalToWorldTransform(
            Usd.TimeCode.Default()).ExtractTranslation()
        cargo_x = float(bp[0]) + IW_LOAD_MAP_X_OFFSET_M
        cargo_y = float(bp[1])
        cargo_z = float(bp[2]) + deck_z
        cargo_quat = Gf.Quatd(1.0)
        chassis = f"{self._root}/chassis"
        chassis_prim = stage.GetPrimAtPath(chassis)
        if chassis_prim.IsValid():
            try:
                bbox_cache = UsdGeom.BBoxCache(
                    Usd.TimeCode.Default(),
                    [UsdGeom.Tokens.default_, UsdGeom.Tokens.render],
                )
                chassis_range = (
                    bbox_cache.ComputeWorldBound(
                        chassis_prim
                    ).ComputeAlignedRange()
                )
                chassis_center = chassis_range.GetMidpoint()
                chassis_world = UsdGeom.Xformable(
                    chassis_prim).ComputeLocalToWorldTransform(
                        Usd.TimeCode.Default())
                # IW root/base_link 원점은 시각 차체의 길이 중심과 다르다.
                # 팔레트 세트가 후방으로 치우치지 않도록 맵 X만 실제 chassis
                # bbox 중심에 맞춘다. 횡방향 Y는 기존 도킹 축을 유지한다.
                deck_top_z = float(chassis_range.GetMax()[2])
                cargo_z = deck_top_z + PALLET_SUPPORT_CLEARANCE
                cargo_quat = Gf.Quatd(
                    chassis_world.ExtractRotationQuat().GetNormalized())
                log(
                    "[IwHub] 팔레트/IW 실측 데크 정렬: "
                    f"root 대비 dx={cargo_x - float(bp[0]):+.3f}m, "
                    f"bbox dx={float(chassis_center[0]) - float(bp[0]):+.3f}m, "
                    f"dy={cargo_y - float(bp[1]):+.3f}m, "
                    f"deck_top_z={deck_top_z:.5f}m, "
                    f"pallet_base_z={cargo_z:.5f}m"
                )
            except Exception as e:
                log(f"[IwHub] ⚠ chassis bbox 중심 계산 실패 — root 중심 사용: {e}")
        root = "/World/IwHubCargo"
        UsdGeom.Xform.Define(stage, root)
        set_pose(
            stage.GetPrimAtPath(root),
            (cargo_x, cargo_y, cargo_z),
            cargo_quat,
        )
        ident = Gf.Quatd(1.0, 0.0, 0.0, 0.0)
        rng = random.Random(7)

        # ★ 적재 강체(팔레트+KLT) = 하나의 동적 강체 Pallet_00. 뒤에서 chassis 데크에 FixedJoint
        #   로 결속해 로봇을 따라가게 한다(사용자 선택 2026-07-20). 토마토는 Pallet_00에 넣지 않는다
        #   — 별도 동적 강체로 KLT 안에서 흔들리며 접촉으로 실려간다(§5.1 진짜 물리 유지).
        #   팔레트는 convexDecomposition 콜라이더 → 포크 슬롯(구멍)을 살려 창고에서 지게차
        #   포크가 들어갈 수 있게 한다(스파이크 06 검증 방식). 결속 조인트는 창고 도착 시
        #   해제(SetActive False)하면 지게차가 팔레트를 넘겨받는다 — 루프 후속.
        load = f"{root}/Pallet_00"
        UsdGeom.Xform.Define(stage, load)
        add_reference_to_stage(pallet_url, f"{load}/Pallet")
        set_pose(stage.GetPrimAtPath(f"{load}/Pallet"), (0.0, 0.0, 0.0), ident)
        physics.disable_physics(stage, f"{load}/Pallet")       # 에셋 자체 물리 제거(중첩 §8)
        physics.add_convex_decomposition_colliders(              # 포크 슬롯 살린 콜라이더
            stage, f"{load}/Pallet")

        klt_scale = 0.85
        kz = PALLET_SIZE[2] + KLT_SIZE[2] * klt_scale / 2.0
        nx, ny, pitx, pity = 4, 2, 0.31, 0.25

        # KLT 시각 메시는 얇고 오목한 형상이다. convex decomposition을 그대로
        # 충돌체로 쓰면 바닥/벽 사이에 틈이 생겨 작은 토마토가 빠지거나, 반대로
        # 입구를 덮는 convex hull이 생길 수 있다. Load 강체 아래에 바닥+4면의
        # 보이지 않는 analytic box를 두어 열린 바구니 충돌 형상을 명시한다.
        klt_outer_x = KLT_SIZE[0] * klt_scale
        klt_outer_y = KLT_SIZE[1] * klt_scale
        klt_height = KLT_SIZE[2] * klt_scale
        # 시각 KLT의 안쪽 벽면은 10mm 기준이다. 충돌 벽은 내부 공간을
        # 좁히지 않고 바깥쪽으로만 30mm까지 두껍게 해 측면 tunneling을 막는다.
        klt_wall_visual_t = 0.010
        klt_wall_t = 0.030
        # 얇은 12mm 바닥은 물리 step 사이에 과실이 통과(tunneling)할 수 있다.
        # 내부 바닥 윗면 높이는 유지하고 충돌체만 아래쪽으로 30mm 두껍게 둔다.
        klt_floor_visual_t = 0.012
        klt_floor_t = 0.030
        klt_contact_offset = 0.002
        klt_collider_paths: list[str] = []
        # 팔레트의 높은 마찰은 지게차 인수에 필요하지만 KLT 안쪽 벽까지 같은
        # μ=0.5를 쓰면 낙하 과실이 림/벽에 걸쳐 멈춘다. KLT shell만 저마찰로
        # 분리해 벽에 닿은 과실이 바닥으로 미끄러져 내려가게 한다.
        klt_inner_material = physics.create_physics_material(
            stage,
            f"{root}/PhysMat/klt_inner",
            static_friction=0.12,
            dynamic_friction=0.08,
        )
        klt_edge_margin_x = (
            PALLET_SIZE[0] / 2.0
            - ((nx - 1) / 2.0 * pitx + klt_outer_x / 2.0)
        )
        klt_edge_margin_y = (
            PALLET_SIZE[1] / 2.0
            - ((ny - 1) / 2.0 * pity + klt_outer_y / 2.0)
        )
        if klt_edge_margin_x < 0.0 or klt_edge_margin_y < 0.0:
            raise RuntimeError(
                "KLT 배치가 팔레트 바깥으로 나갑니다: "
                f"margin_x={klt_edge_margin_x:.4f}m, "
                f"margin_y={klt_edge_margin_y:.4f}m"
            )

        def add_klt_box_collider(path: str,
                                 center: tuple[float, float, float],
                                 size: tuple[float, float, float]) -> None:
            cube = UsdGeom.Cube.Define(stage, path)
            cube.CreateSizeAttr(1.0)
            prim = cube.GetPrim()
            set_pose(prim, center, ident)
            # pjt_utils.set_scale()은 균일 스케일 전용이다. 충돌 박스는
            # 각 축 길이가 다르므로 새 Cube의 scale op를 직접 지정한다.
            UsdGeom.Xformable(prim).AddScaleOp().Set(Gf.Vec3f(*size))
            collision = UsdPhysics.CollisionAPI.Apply(prim)
            collision.CreateCollisionEnabledAttr(True).Set(True)
            # CollisionAPI만 암묵적으로 해석시키지 않고 PhysX 접촉 속성까지
            # 명시한다. IW 팔레트가 복합 강체로 결속된 뒤에도 KLT 바닥/벽의
            # 접촉 형상이 활성 상태임을 보장한다.
            physx_collision = PhysxSchema.PhysxCollisionAPI.Apply(prim)
            physx_collision.CreateContactOffsetAttr(
                klt_contact_offset).Set(klt_contact_offset)
            physx_collision.CreateRestOffsetAttr(0.0).Set(0.0)
            physics.bind_physics_material(prim, klt_inner_material)
            # 렌더링에서는 숨기되 물리 충돌은 계속 활성 상태로 유지한다.
            UsdGeom.Imageable(prim).MakeInvisible()
            klt_collider_paths.append(path)

        def add_klt_shell(ix: int, iy: int, ox: float, oy: float) -> None:
            shell = f"{load}/KLT_Colliders_{ix}{iy}"
            UsdGeom.Xform.Define(stage, shell)
            bottom_z = kz - klt_height / 2.0
            floor_top_z = bottom_z + klt_floor_visual_t
            inner_half_x = klt_outer_x / 2.0 - klt_wall_visual_t
            inner_half_y = klt_outer_y / 2.0 - klt_wall_visual_t
            add_klt_box_collider(
                f"{shell}/Floor",
                (ox, oy, floor_top_z - klt_floor_t / 2.0),
                (klt_outer_x, klt_outer_y, klt_floor_t),
            )
            add_klt_box_collider(
                f"{shell}/Wall_X_Neg",
                (ox - inner_half_x - klt_wall_t / 2.0, oy, kz),
                (klt_wall_t, klt_outer_y, klt_height),
            )
            add_klt_box_collider(
                f"{shell}/Wall_X_Pos",
                (ox + inner_half_x + klt_wall_t / 2.0, oy, kz),
                (klt_wall_t, klt_outer_y, klt_height),
            )
            # Y벽은 X벽의 안쪽 면 사이만 채워 서로 겹치거나 내부를 막지 않는다.
            inner_x = max(0.001, 2.0 * inner_half_x)
            add_klt_box_collider(
                f"{shell}/Wall_Y_Neg",
                (ox, oy - inner_half_y - klt_wall_t / 2.0, kz),
                (inner_x, klt_wall_t, klt_height),
            )
            add_klt_box_collider(
                f"{shell}/Wall_Y_Pos",
                (ox, oy + inner_half_y + klt_wall_t / 2.0, kz),
                (inner_x, klt_wall_t, klt_height),
            )

        # 데크 앞쪽(IW 코=+x, MM 쪽) 한 열만 비워 MoveIt-MM이 실제로 수확한
        # 토마토를 받는다. 나머지 뒤쪽 6칸은 "이미 수확해 실어둔" 모습으로 5개씩
        # 사전 적재한다. 예전 사전 적재를 껐던 이유는 동적 강체 토마토가 주행/포크
        # 작업 중 팔레트 위를 굴러다니며 실제 하역 결과와 섞였기 때문이므로,
        # 사전 적재분은 KLT 크레이트와 같은 방식으로 Load(팔레트) 강체의 자식
        # 콜라이더로 넣는다 — PhysX가 팔레트 복합 강체에 흡수하므로 IW 주행·
        # 지게차 리프트에 한 덩어리로 딸려가고, 구르거나 튀지 않는다.
        # (자체 강체를 주면 굴러다니고, kinematic 강체를 주면 부모가 움직여도
        #  따라가지 않아 IW가 출발할 때 제자리에 남는다.)
        filled: set[tuple[int, int]] = {
            (ix, iy) for ix in range(nx - 1) for iy in range(ny)
        }
        # 사전 적재분은 낙하로 쌓이지 않으므로 KLT 바닥에 닿는 안착 높이에 바로 둔다.
        # 바닥 윗면 = 팔레트 윗면 + 시각 바닥 두께, 거기에 과실 반지름을 더한다.
        tz_rest = PALLET_SIZE[2] + klt_floor_visual_t + phys_cfg.fruit_collision_radius_m
        # KLT 내부 유효 폭(=벽 안쪽) 안에서 5개가 서로 겹치지 않는 3+2 배치.
        # 지름 68mm < x간격 70mm, y간격 74mm 이고 최외곽도 내벽 안에 들어온다.
        prefill_offsets = (
            (-0.070, -0.037), (0.000, -0.037), (+0.070, -0.037),
            (-0.035, +0.037), (+0.035, +0.037),
        )
        tmat = physics.create_physics_material(
            stage, f"{root}/PhysMat/tomato",
            phys_cfg.fruit_static_friction, phys_cfg.fruit_dynamic_friction)
        tgroup = f"{load}/Tomatoes"          # ★ 팔레트의 자식 → 강체에 흡수
        UsdGeom.Xform.Define(stage, tgroup)
        ripeness.bind_matte_material(
            stage, tgroup, stronger_than_descendants=False)
        n_tom = 0
        for ix in range(nx):
            for iy in range(ny):
                ox = (ix - (nx - 1) / 2.0) * pitx
                oy = (iy - (ny - 1) / 2.0) * pity
                kp = f"{load}/KLT_{ix}{iy}"
                add_reference_to_stage(klt_url, kp)
                set_pose(stage.GetPrimAtPath(kp), (ox, oy, kz), ident)
                set_scale(stage.GetPrimAtPath(kp), klt_scale)
                physics.disable_physics(stage, kp)         # 에셋 자체 강체 제거(중첩경고 §8)
                # 초기 적재 여부와 무관하게 8개 KLT 모두 실제로 토마토를 받을 수
                # 있어야 한다. 시각 메시의 convex 근사 대신 입구가 열려 있음이
                # 보장되는 바닥+4벽 충돌체를 Load 강체 아래에 직접 구성한다.
                add_klt_shell(ix, iy, ox, oy)
                if (ix, iy) not in filled or not ripe:
                    continue
                for k, (dx, dy) in enumerate(prefill_offsets):   # 토마토 5개
                    body, calyx = rng.choice(ripe)
                    # 고정 배치에 아주 작은 흔들림만 줘 격자처럼 보이지 않게 한다.
                    jx = ox + dx + rng.uniform(-0.002, 0.002)
                    jy = oy + dy + rng.uniform(-0.002, 0.002)
                    tz = tz_rest
                    yaw = rng.uniform(-math.pi, math.pi)    # 개체마다 다른 방위
                    tp = f"{tgroup}/T_{ix}{iy}_{k}"
                    tprim = UsdGeom.Xform.Define(stage, tp).GetPrim()
                    set_pose(tprim, (jx, jy, tz), Gf.Quatd(
                        math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0)))
                    set_scale(tprim, tomato_cfg.scale)
                    add_reference_to_stage(body, tp + "/Body")
                    ripeness.apply_ripeness_color(stage, tp + "/Body", "ripe", rng)
                    ripeness.bind_matte_material(
                        stage,
                        tp + "/Body",
                        mat_path="/World/Looks/MatteFruitRipe",
                        fallback_color=ripeness.RED,
                    )
                    if calyx:                              # 꼭지(장식 — 콜라이더 없음)
                        add_reference_to_stage(calyx, tp + "/Calyx")
                        ripeness.apply_flat_color(stage, tp + "/Calyx", ripeness.GREEN)
                        ripeness.bind_matte_material(
                            stage,
                            tp + "/Calyx",
                            mat_path="/World/Looks/MatteCalyx",
                            fallback_color=ripeness.GREEN,
                        )
                    # 몸통에만 콜라이더(꼭지는 장식). 강체는 주지 않는다 —
                    # Load 강체가 흡수해 실제 토마토처럼 부딪히되 고정된다.
                    physics.add_mesh_colliders(stage, tp + "/Body",
                                               phys_cfg.fruit_approximation)
                    physics.bind_physics_material(tprim, tmat)
                    n_tom += 1

        # 8개 KLT × (바닥 1 + 벽 4). 하나라도 빠지면 토마토가 팔레트 아래로
        # 떨어질 수 있으므로 조용히 진행하지 않는다.
        expected_klt_colliders = nx * ny * 5
        active_klt_colliders = sum(
            1
            for path in klt_collider_paths
            if (
                stage.GetPrimAtPath(path).IsValid()
                and stage.GetPrimAtPath(path).HasAPI(
                    UsdPhysics.CollisionAPI)
                and bool(
                    UsdPhysics.CollisionAPI(
                        stage.GetPrimAtPath(path)
                    ).GetCollisionEnabledAttr().Get()
                )
            )
        )
        if active_klt_colliders != expected_klt_colliders:
            raise RuntimeError(
                "IW KLT collider 생성 실패: "
                f"active={active_klt_colliders}, "
                f"expected={expected_klt_colliders}"
            )

        # ── Load 를 독립 강체로 확정 + 안전한 데크 소유 표식 생성 ──
        # Explicit mass가 PhysX의 복합 참조 collider에서 무시되는 경우에도
        # 분해 hull 부피로 수 톤이 계산되지 않도록 fallback density를 낮게 둔다.
        # 실제 동특성은 바로 아래의 mass=40kg 속성이 결정한다.
        load_prim = stage.GetPrimAtPath(load)
        # 참조 팔레트 USD의 하위 Mesh/Xform에 원본 MassAPI가 남아 있으면,
        # Load 루트의 명시 질량과 별도로 convex-decomposition 부피 질량이
        # 합산된다. 그러면 40kg 설정에도 리프트가 30kN에서 눌린다.
        # 하나의 Load 강체는 루트 MassAPI 하나만 소유하게 정규화한다.
        for child in Usd.PrimRange(load_prim):
            if child != load_prim and child.HasAPI(UsdPhysics.MassAPI):
                child.RemoveAPI(UsdPhysics.MassAPI)
        load_density = 1.0
        # 팔레트는 실제 동적 강체로 두고, 아래의 excludeFromArticulation
        # FixedJoint가 주행 중 데크에 결속한다. 매 프레임 set_world_pose로
        # 따라오게 하면 Nav2 주행 중 팔레트만 뒤에 남거나 순간이동 상태가
        # 누적될 수 있다.
        physics.add_rigid_body(load_prim, load_density, kinematic=False)
        # 참조 팔레트 메시와 복합 KLT collider에 density만 지정하면 PhysX가
        # 에셋의 저자 단위/분해 hull 부피를 합산해 1t 이상으로 계산할 수 있다.
        # 실제 팔레트 약 25kg + 빈 소형 KLT 8개를 합친 운반 세트는 약 40kg으로
        # 고정한다. 그렇지 않으면 lift drive가 16kN 이상을 내도 상승하지 않는다.
        load_mass = UsdPhysics.MassAPI.Apply(load_prim)
        load_mass.GetDensityAttr().Clear()
        load_mass.CreateMassAttr(40.0).Set(40.0)
        # 팔레트/KLT는 Load 하나의 복합 강체다. 참조 USD 하위에 남은 재질이
        # 접촉마다 우선되는 일을 막고, 설정의 목재-강철 마찰값을 전체 복합
        # 콜라이더에 일관되게 적용한다. 미지정 호출자는 창고 기본값과 같은
        # μs=0.5, μd=0.35를 사용한다.
        load_static_friction = float(
            getattr(pallet_phys_cfg, "static_friction", 0.5)
        )
        load_dynamic_friction = float(
            getattr(pallet_phys_cfg, "dynamic_friction", 0.35)
        )
        load_material = physics.create_physics_material(
            stage,
            f"{root}/PhysMat/pallet_klt",
            load_static_friction,
            load_dynamic_friction,
        )
        UsdShade.MaterialBindingAPI.Apply(load_prim).Bind(
            load_material,
            UsdShade.Tokens.strongerThanDescendants,
            "physics",
        )
        if stage.GetPrimAtPath(chassis).IsValid():
            physics.create_fixed_joint(
                stage,
                f"{root}/DeckJoint",
                chassis,
                load,
                exclude_from_articulation=True,
            )
            bound = (
                "excludeFromArticulation FixedJoint 결속"
                "(창고서 해제→지게차 인수)"
            )
        else:
            bound = "⚠ chassis 링크 없음 → 데크 위 비결속 배치"
        log(
            f"[IwHub] 데크 적재: 팔레트(포크슬롯)+KLT 8 + 토마토 "
            f"{n_tom}개(사전적재·Load 강체 흡수, 앞열 {ny}칸은 MM용 공석). "
            f"Load {bound}. "
            f"pallet_base_z={cargo_z:.5f}, mass=40.0kg, "
            f"friction=(static {load_static_friction:.2f}, "
            f"dynamic {load_dynamic_friction:.2f}). "
            f"KLT local z={kz:.5f}m, edge_margin="
            f"({klt_edge_margin_x:.5f}, {klt_edge_margin_y:.5f})m, "
            f"KLT colliders={active_klt_colliders}/"
            f"{expected_klt_colliders}, "
            f"wall/floor=({klt_wall_t:.3f}/{klt_floor_t:.3f})m, "
            "KLT friction=(0.12/0.08), "
            f"contact_offset={klt_contact_offset:.3f}m"
        )
        return n_tom

    # 에셋에 이미 있을 법한 라이다 프림 이름/타입 키워드 (Idealworks iw.hub 는 실물 AMR).
    _LIDAR_KEYS = ("lidar", "laser", "scan")

    def find_lidar(self, stage: Usd.Stage) -> str | None:
        """에셋 트리에서 이미 달린 라이다 프림을 찾는다. 없으면 None.

        iw.hub 는 실물 AMR 이라 에셋에 라이다가 이미 있을 수 있다(사용자 지적 2026-07-20).
        있으면 새로 만들 필요 없이 그 경로를 브리지(build_lidar_scan)에 넘긴다.
        실제 프림 경로는 tools/nav2_node_probe.py 의 '에셋 센서 스캔'으로 확인할 것.
        """
        if not self._root:
            return None
        for p in Usd.PrimRange(stage.GetPrimAtPath(self._root)):
            tname = (p.GetTypeName() or "").lower()
            pname = p.GetName().lower()
            if any(k in tname for k in self._LIDAR_KEYS) or \
               any(k in pname for k in self._LIDAR_KEYS):
                return str(p.GetPath())
        return None

    def attach_lidar(self, stage: Usd.Stage, mount, log=print):
        """mount(LidarMount) 위치·방향에 RTX 2D 라이다 1기 + 렌더프로덕트를 만든다.
        반환: (라이다 prim 경로, 렌더프로덕트 경로) 또는 None.

        iw.hub 엔 내장 라이다가 없어(사용자 확인 2026-07-20) 앞/뒤 각 1기를 직접 만든다.
        Isaac 5.1 정식 API `isaacsim.sensors.rtx.LidarRtx` — 라이다 생성 + 렌더프로덕트를 같이
        만들어 준다. ROS2RtxLidarHelper 는 lidarPrim 이 아니라 renderProductPath 를 받는다
        (2026-07-20 GPU 실측 — 이전 IsaacSensorCreateRtxLidar 직접 호출 + lidarPrim 은 실패).
        config = RPLIDAR_S2E (Nova Carter의 SLAMTEC 2D LaserScan). prim 이름 = mount.frame →
        TF child = LaserScan frame_id 일치. LidarRtx 객체는 self._lidars 에 보관(GC 방지).
        """
        import math

        import numpy as np

        # ★ 움직이는 섀시 링크에 붙인다 — /World/IwHub(컨테이너)는 정지라, 거기 붙이면
        #   로봇이 주행해도 라이다가 스폰 자리에 남는다(2026-07-20 사용자 발견). chassis 는
        #   아티큘레이션 base 링크로 물리로 움직인다.
        parent = f"{self._root}/chassis"
        if not stage.GetPrimAtPath(parent).IsValid():
            parent = self._root
        path = f"{parent}/{mount.frame}"             # prim 이름 = TF 프레임(= scan frame_id)
        half = math.radians(mount.yaw_deg) / 2.0
        quat = np.array([math.cos(half), 0.0, 0.0, math.sin(half)])   # Z축 yaw (w,x,y,z)
        try:
            from isaacsim.sensors.rtx import LidarRtx
            lidar = LidarRtx(
                prim_path=path, name=f"lidar_{mount.name}",
                translation=np.array(mount.offset, dtype=float),
                orientation=quat,
                # config 는 파일명 stem 으로 매칭된다(전체경로 X — commands.py 실측 2026-07-20).
                config_file_name="RPLIDAR_S2E")
            self._lidars.append(lidar)               # 참조 유지(GC 되면 렌더프로덕트 파괴)
            rp = lidar.get_render_product_path()
            log(f"[IwHub] RTX 라이다 '{mount.name}': {path} @ {mount.offset} "
                f"yaw={mount.yaw_deg}° rp={rp}")
            return path, rp
        except Exception as e:
            log(f"[IwHub] ⚠ 라이다 '{mount.name}' 생성 실패 — GPU RTX 라이다 API 확인: {e}")
            return None
