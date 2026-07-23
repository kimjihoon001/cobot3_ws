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

import os
import random

from pxr import Gf, PhysxSchema, Usd, UsdGeom, UsdPhysics, UsdShade

from pjt_config.settings import RobotConfig
from pjt_utils.xform import set_pose, set_scale, set_translate
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
        # 참조 prim 은 자체 xformOp 을 가질 수 있다 → 기존 op 재사용(§8).
        # yaw=180°이면 긴 후방 오버행이 MM 반대쪽을 향해 추종 회전 시 충돌하지 않는다.
        yaw = Gf.Rotation(Gf.Vec3d(0.0, 0.0, 1.0), yaw_deg).GetQuat()
        set_pose(
            stage.GetPrimAtPath(root), position,
            Gf.Quatd(yaw.GetReal(), yaw.GetImaginary()),
        )
        self._root = root
        self._configure_drive_torque(stage, log)
        log(
            f"[IwHub] 배치 완료: {root} @ "
            f"{tuple(round(v, 2) for v in position)}, yaw={yaw_deg:.1f}°"
        )
        return root

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
                   deck_z: float = 0.225, log=print) -> int:
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
        deck_z: 데크 높이(root 기준 오프셋). [2] 유도 — bbox 상단은 0.198(measure_deck.py)이나
                GUI 실측상 0.225 라야 팔레트가 데크에 얹힌다(0.25=52mm 뜸, 0.20=박힘). GUI 확인.
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
        # 컨테이너 root 원점은 실제 차체 기하 중심과 일치하지 않는다. 적재물 중심을 차체
        # 중심에 맞추면 짧은 팔레트(1.213m)의 앞면이 IW 앞면보다 10.9cm 뒤로 들어간다.
        # 사용자 지정대로 두 앞면이 같은 수직면이 되게 chassis bbox의 +X 앞면에서
        # 팔레트 길이 절반을 빼 cargo 중심을 정한다. Z는 기존 데크 실측값을 쓴다.
        src = f"{self._root}/base_link"
        if not stage.GetPrimAtPath(src).IsValid():
            src = self._root
        bp = UsdGeom.Xformable(stage.GetPrimAtPath(src)).ComputeLocalToWorldTransform(
            Usd.TimeCode.Default()).ExtractTranslation()
        cargo_x, cargo_y = float(bp[0]), float(bp[1])
        cargo_quat = Gf.Quatd(1.0)
        chassis = f"{self._root}/chassis"
        chassis_prim = stage.GetPrimAtPath(chassis)
        if chassis_prim.IsValid():
            try:
                bbox_cache = UsdGeom.BBoxCache(
                    Usd.TimeCode.Default(),
                    [UsdGeom.Tokens.default_, UsdGeom.Tokens.render],
                )
                # local bound를 사용해야 IW를 180° 돌려도 어느 쪽이 차체의 +X
                # 앞면인지 유지된다. world AABB의 max X는 회전 후 뒤쪽을 뜻한다.
                chassis_range = (
                    bbox_cache.ComputeLocalBound(chassis_prim).ComputeAlignedRange()
                )
                chassis_front_x = float(chassis_range.GetMax()[0])
                chassis_center_y = float(chassis_range.GetMidpoint()[1])
                chassis_world = UsdGeom.Xformable(
                    chassis_prim).ComputeLocalToWorldTransform(
                        Usd.TimeCode.Default())
                cargo_center = chassis_world.Transform(
                    Gf.Vec3d(
                        chassis_front_x - PALLET_SIZE[0] / 2.0,
                        chassis_center_y,
                        0.0,
                    )
                )
                cargo_x, cargo_y = float(cargo_center[0]), float(cargo_center[1])
                cargo_quat = Gf.Quatd(
                    chassis_world.ExtractRotationQuat().GetNormalized())
                log(
                    "[IwHub] 팔레트/IW 앞면 정렬: "
                    f"root 대비 dx={cargo_x - float(bp[0]):+.3f}m, "
                    f"dy={cargo_y - float(bp[1]):+.3f}m, "
                    f"local_front_x={chassis_front_x:.3f}m"
                )
            except Exception as e:
                log(f"[IwHub] ⚠ chassis bbox 중심 계산 실패 — root 중심 사용: {e}")
        root = "/World/IwHubCargo"
        UsdGeom.Xform.Define(stage, root)
        set_pose(
            stage.GetPrimAtPath(root),
            (cargo_x, cargo_y, bp[2] + deck_z),
            cargo_quat,
        )
        ident = Gf.Quatd(1.0, 0.0, 0.0, 0.0)
        rng = random.Random(7)

        # ★ 적재 강체(팔레트+KLT) = 하나의 동적 강체 Load. 뒤에서 chassis 데크에 FixedJoint
        #   로 결속해 로봇을 따라가게 한다(사용자 선택 2026-07-20). 토마토는 Load 에 안 넣는다
        #   — 별도 동적 강체로 KLT 안에서 흔들리며 접촉으로 실려간다(§5.1 진짜 물리 유지).
        #   팔레트는 convexDecomposition 콜라이더 → 포크 슬롯(구멍)을 살려 창고에서 지게차
        #   포크가 들어갈 수 있게 한다(스파이크 06 검증 방식). 결속 조인트는 창고 도착 시
        #   해제(SetActive False)하면 지게차가 팔레트를 넘겨받는다 — 루프 후속.
        load = f"{root}/Load"
        UsdGeom.Xform.Define(stage, load)
        add_reference_to_stage(pallet_url, f"{load}/Pallet")
        set_pose(stage.GetPrimAtPath(f"{load}/Pallet"), (0.0, 0.0, 0.0), ident)
        physics.disable_physics(stage, f"{load}/Pallet")       # 에셋 자체 물리 제거(중첩 §8)
        physics.add_convex_decomposition_colliders(              # 포크 슬롯 살린 콜라이더
            stage, f"{load}/Pallet")

        klt_scale = 0.85
        kz = PALLET_SIZE[2] + KLT_SIZE[2] * klt_scale / 2.0
        nx, ny, pitx, pity = 4, 2, 0.31, 0.25
        filled = {(0, 0), (1, 1), (3, 0)}                  # 토마토 넣을 3칸
        tz0 = PALLET_SIZE[2] + 0.045                        # 첫 토마토 높이(팔레트 윗면 위)
        tmat = physics.create_physics_material(
            stage, f"{root}/PhysMat/tomato",
            phys_cfg.fruit_static_friction, phys_cfg.fruit_dynamic_friction)
        tgroup = f"{root}/Tomatoes"
        UsdGeom.Xform.Define(stage, tgroup)
        ripeness.bind_matte_material(stage, tgroup)        # 무광 — displayColor(익은색) 읽게
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
                if (ix, iy) not in filled or not ripe:
                    continue
                physics.add_convex_decomposition_colliders(stage, kp)   # 담는 그릇(Load 콜라이더)
                for k in range(5):                         # 토마토 5개 — 흩뿌려 떨어뜨림
                    body, calyx = rng.choice(ripe)
                    jx = ox + rng.uniform(-0.06, 0.06)     # 격자 아닌 랜덤 산포
                    jy = oy + rng.uniform(-0.035, 0.035)
                    tz = tz0 + k * 0.05                     # 높이 엇갈려 떨어뜨려 자연스럽게 쌓임
                    tp = f"{tgroup}/T_{ix}{iy}_{k}"
                    tprim = UsdGeom.Xform.Define(stage, tp).GetPrim()
                    set_pose(tprim, (jx, jy, tz), ident)
                    set_scale(tprim, tomato_cfg.scale)
                    add_reference_to_stage(body, tp + "/Body")
                    ripeness.apply_ripeness_color(stage, tp + "/Body", "ripe", rng)
                    if calyx:                              # 꼭지(장식 — 콜라이더 없음)
                        add_reference_to_stage(calyx, tp + "/Calyx")
                        ripeness.apply_flat_color(stage, tp + "/Calyx", ripeness.GREEN)
                    # 동적 강체: 몸통에만 콜라이더(꼭지는 장식) → 흩뿌리면 쌓인다
                    physics.add_mesh_colliders(stage, tp + "/Body",
                                               phys_cfg.fruit_approximation)
                    physics.add_rigid_body(tprim, phys_cfg.fruit_density,
                                           kinematic=False)
                    physics.bind_physics_material(tprim, tmat)
                    n_tom += 1

        # ── Load 를 동적 강체로 확정 + chassis 데크에 FixedJoint 결속 ──
        load_density = 200.0   # [4] 근거없음 — 팔레트+빈 유효밀도. 결속돼 있어 동특성 영향 작음. GPU 보정.
        physics.add_rigid_body(stage.GetPrimAtPath(load), load_density,
                               kinematic=False)
        if stage.GetPrimAtPath(chassis).IsValid():
            physics.create_fixed_joint(stage, f"{root}/DeckJoint", chassis, load)
            bound = "chassis 결속(로봇 따라감·창고서 해제→지게차 인수)"
        else:
            bound = "⚠ chassis 링크 없음 → 데크 위 비결속 배치"
        log(f"[IwHub] 데크 적재: 팔레트(포크슬롯)+KLT 8 + 토마토 {n_tom}개(동적강체). "
            f"Load {bound}. deck_z={deck_z} [4] GPU 보정 요")
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
