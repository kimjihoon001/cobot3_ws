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

from pxr import Gf, Usd, UsdGeom, UsdShade

from pjt_config.settings import RobotConfig
from pjt_utils.xform import set_pose, set_scale, set_translate
from robots import assets


class IwHub:
    """iw.hub 운반 AMR. 구조: {root} <- iw_hub.usd 참조 (아티큘레이션 루트 = root)."""

    # 실측 DOF 이름 (2026-07-19) — ROS2 JointState 의 name 필드에 이대로 쓴다.
    DRIVE_JOINTS = ("left_wheel_joint", "right_wheel_joint")   # 속도 명령(차동)
    LIFT_JOINT = "lift_joint"                                   # 위치 명령(승강)

    # 실측 에셋 치수. iw.hub 상판에 빈 KLT 팔레트를 초기 적재할 때 사용한다.
    DECK_Z = 0.235                         # AMR 로컬 원점 기준 상판 바로 위
    PALLET_HEIGHT = 0.143
    KLT_HEIGHT = 0.146
    KLT_SCALE = 0.85
    KLT_COLOR = Gf.Vec3f(0.60, 0.28, 0.62)

    def __init__(self, cfg: RobotConfig):
        self._cfg = cfg
        self._root: str | None = None

    @property
    def root(self) -> str | None:
        return self._root

    def spawn(self, stage: Usd.Stage, root: str = "/World/IwHub",
              position: tuple[float, float, float] = (0.0, 0.0, 0.0),
              log=print) -> str:
        """놓는다. 반환: root 경로."""
        from isaacsim.core.utils.stage import add_reference_to_stage

        url = assets.resolve(self._cfg.assets.iwhub, "운반 AMR(iw.hub)")
        log(f"[IwHub] 에셋 {url}")
        add_reference_to_stage(url, root)
        # 참조 prim 은 자체 xformOp 을 가질 수 있다 → 기존 op 재사용(§8)
        set_translate(stage.GetPrimAtPath(root), position)
        self._root = root
        log(f"[IwHub] 배치 완료: {root} @ {tuple(round(v, 2) for v in position)}")
        return root

    def load_cargo(self, stage: Usd.Stage, tomato_cfg, phys_cfg,
                   deck_z: float = 0.25, log=print) -> int:
        """iw.hub 데크에 '적재된 세트' — 팔레트 + KLT 8개 + 3칸에 토마토 5개씩(15개, 꼭지 포함).

        ★ 물리 구조 (사용자 정정 2026-07-20 "토마토도 강체로 / 가지런히 놓을 필요 없어"):
          · 채운 KLT 3칸 = **static 오목 콜라이더(그릇)** — 토마토를 담아 흘리지 않는다.
          · 토마토 = **동적 강체**(몸통 콜라이더 + 마찰) → 흩뿌려 떨어뜨리면 자연스럽게 쌓인다.
          · 팔레트·빈 KLT = 시각 전용(콜라이더 없음).
          아티큘레이션(iw.hub) 밑에 강체를 중첩하면 꼬이므로(§8류) 세트는 iw.hub 데크
          **월드 위치에 독립 배치**한다 — 지금은 로봇이 정지 상태라 무방. 주행 연동은 추후
          루프에서 포즈 동기화(TODO). 꼭지(calyx)는 몸통의 자식(장식, 콜라이더 없음).
        deck_z: 데크 높이(base_link 기준 오프셋). [4] 임의 — GPU 에서 iw.hub 데크 실측 보정.
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

        # iw.hub 데크 월드 포즈 — 세트를 아티큘레이션 밖 독립 프림으로 여기에 놓는다.
        src = f"{self._root}/base_link"
        if not stage.GetPrimAtPath(src).IsValid():
            src = self._root
        bp = UsdGeom.Xformable(stage.GetPrimAtPath(src)).ComputeLocalToWorldTransform(
            Usd.TimeCode.Default()).ExtractTranslation()
        root = "/World/IwHubCargo"
        UsdGeom.Xform.Define(stage, root)
        set_translate(stage.GetPrimAtPath(root), (bp[0], bp[1], bp[2] + deck_z))
        ident = Gf.Quatd(1.0, 0.0, 0.0, 0.0)
        rng = random.Random(7)

        add_reference_to_stage(pallet_url, f"{root}/Pallet")   # 팔레트(시각)
        set_pose(stage.GetPrimAtPath(f"{root}/Pallet"), (0.0, 0.0, 0.0), ident)

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
                kp = f"{root}/KLT_{ix}{iy}"
                add_reference_to_stage(klt_url, kp)
                set_pose(stage.GetPrimAtPath(kp), (ox, oy, kz), ident)
                set_scale(stage.GetPrimAtPath(kp), klt_scale)
                if (ix, iy) not in filled or not ripe:
                    continue
                physics.add_convex_decomposition_colliders(stage, kp)   # 그릇(static)
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
        log(f"[IwHub] 데크 적재: 팔레트+KLT 8 + 토마토 {n_tom}개(꼭지 포함, 동적강체, 3칸 산포). "
            f"KLT 3칸 static 그릇. deck_z={deck_z} [4] GPU 보정 요")
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

    def attach_lidar(self, stage: Usd.Stage, offset: tuple[float, float, float],
                     log=print) -> str | None:
        """라이다 경로를 돌려준다 — **먼저 에셋에서 찾고, 없을 때만 RTX 라이다를 만든다.**

        ⚠ GPU 반복 필요: RTX 라이다 생성 API(omni.kit.commands 'IsaacSensorCreateRtxLidar')
          와 프로파일 이름은 Isaac 5.1 에서 실측 확인해야 한다(추측 금지, §8). 에셋에 이미
          있으면 이 생성 경로는 아예 안 탄다 — probe 로 먼저 확인.
        """
        found = self.find_lidar(stage)
        if found:
            log(f"[IwHub] 에셋 내장 라이다 사용: {found}")
            return found

        # 없을 때만 생성. base_link 아래에 RTX 라이다를 offset 위치로.
        parent = f"{self._root}/base_link"
        if not stage.GetPrimAtPath(parent).IsValid():
            parent = self._root                      # base_link 이름이 다르면 루트에
        path = f"{parent}/nav_lidar"
        try:
            import omni.kit.commands
            omni.kit.commands.execute(
                "IsaacSensorCreateRtxLidar",
                path=path, parent=None,
                config="Example_Rotary",              # TODO GPU 실측 — 실제 프로파일명 확인
                translation=offset, orientation=(1.0, 0.0, 0.0, 0.0))
            log(f"[IwHub] RTX 라이다 생성: {path} (에셋에 없어 직접 만듦)")
            return path
        except Exception as e:
            log(f"[IwHub] ⚠ 라이다 생성 실패 — GPU 에서 RTX 라이다 API 확인 필요: {e}")
            return None

    def spawn_empty_pallet(self, stage: Usd.Stage, log=print) -> str:
        """AMR 상판에 나무 팔레트 1개와 빈 KLT 8개(4×2)를 초기 적재한다.

        적재물은 ``{root}/Load`` 아래에 두므로 AMR이 주행하면 함께 이동한다.
        KLT는 비어 있는 실제 small_KLT 에셋이며, 팔레트 윗면에 겹치지 않게 놓는다.
        반환값은 적재 루트 prim 경로다.
        """
        if self._root is None:
            raise RuntimeError("iw.hub를 spawn한 뒤 팔레트를 적재해야 합니다.")

        from isaacsim.core.utils.stage import add_reference_to_stage

        from pjt_utils.xform import set_pose, set_scale, set_translate

        pallet_url = assets.resolve(self._cfg.assets.pallet, "iw.hub 적재 팔레트")
        klt_url = assets.resolve(self._cfg.assets.klt_bin, "iw.hub 빈 KLT")

        load_root = f"{self._root}/Load"
        UsdGeom.Xform.Define(stage, load_root)
        set_translate(stage.GetPrimAtPath(load_root), (0.0, 0.0, self.DECK_Z))

        identity = Gf.Quatd(1.0, Gf.Vec3d(0.0, 0.0, 0.0))
        pallet_path = f"{load_root}/Pallet"
        add_reference_to_stage(pallet_url, pallet_path)
        # 팔레트 USD 원점은 바닥이므로 Load 원점에 그대로 둔다.
        set_pose(stage.GetPrimAtPath(pallet_path), (0.0, 0.0, 0.0), identity)

        # 팔레트 1.213×0.802m 안에 빈 KLT 8개를 여유 있게 4×2 배치한다.
        klt_z = self.PALLET_HEIGHT + self.KLT_HEIGHT * self.KLT_SCALE / 2.0
        for ix in range(4):
            for iy in range(2):
                x = (ix - 1.5) * 0.31
                y = (iy - 0.5) * 0.25
                path = f"{load_root}/KLT_{ix}_{iy}"
                add_reference_to_stage(klt_url, path)
                prim = stage.GetPrimAtPath(path)
                set_pose(prim, (x, y, klt_z), identity)
                set_scale(prim, self.KLT_SCALE)
                # 창고의 KLT와 같은 보라색으로 표시하고 무거운 재질 바인딩은 제거한다.
                for child in Usd.PrimRange(prim):
                    if child.IsA(UsdGeom.Mesh):
                        UsdShade.MaterialBindingAPI(child).UnbindAllBindings()
                        UsdGeom.Gprim(child).CreateDisplayColorAttr([self.KLT_COLOR])

        log(f"[IwHub] 빈 KLT 팔레트 적재 완료: {pallet_path} (KLT 8개)")
        return load_root
