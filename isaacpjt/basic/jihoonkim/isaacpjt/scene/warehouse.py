# -*- coding: utf-8 -*-
"""창고 — AMR 이 포크로 트레이를 지정 슬롯에 올린다.

★ GPU 미검증. spikes/03_asset_check.py 로 에셋 확인 먼저. ★

슬롯 6개 = 3섹터 × 2단. **재배 6섹터와 1:1 매핑**된다.
매핑이 핵심이다 — 슬롯 수 = 섹터 수라야 "섹터N 트레이는 슬롯N 으로" 가 규칙이 되고,
그러면 창고 배치가 탐색 문제가 아니라 상수 조회가 된다. v3 10장이 걱정한
"창고 적재 위치 결정 로직 복잡도" 가 이 매핑 하나로 사라진다.

2단인 이유: 6슬롯을 1단으로 깔면 포크 승강이 필요 없어져서 지게차를 고른 이유가
무너진다. 2단이라야 상하 1축이 실제로 쓰인다.

배경 환경은 Isaac 의 Simple_Warehouse 를 쓴다 (v3 6.2 "Isaac Warehouse 기반 커스텀").
슬롯 자체는 우리가 정의한다 — 배경 선반의 실제 좌표를 모르기 때문이다.
TODO 배경 선반에 슬롯을 맞추려면 GPU 에서 선반 좌표를 재서 slot_pose() 를 고칠 것.
"""
from __future__ import annotations

from pxr import Gf, Usd, UsdGeom, UsdPhysics, UsdShade

from pjt_config.settings import WarehouseConfig
from scene import physics

SLOT_COLOR = Gf.Vec3f(0.30, 0.45, 0.70)
GUIDE_COLOR = Gf.Vec3f(0.85, 0.65, 0.15)
FRAME_COLOR = Gf.Vec3f(0.82, 0.82, 0.84)    # 랙 골조 (밝은 회백)
SHELF_COLOR = Gf.Vec3f(0.55, 0.56, 0.58)    # 선반 (강판 회색)
KLT_COLOR = Gf.Vec3f(0.60, 0.28, 0.62)      # KLT 크레이트 색 박스(무거운 USD 대체) — 보라(원래 KLT 색)

# 슬롯(선반 한 칸) 치수 — 나무 팔레트(EUR)를 통째로 올린다(물류 루프 2026-07-19:
#   지게차가 팔레트째 랙에 적재). 팔레트 실측 1.213×0.802m 를 담게 여유를 준다.
# [2] 유도 — 팔레트 footprint 에서 나온다.
SLOT_SIZE = (1.30, 0.90, 0.06)      # m. 슬롯 바닥판 (팔레트 1.213×0.802 + 여유)

# 팔레트/KLT 실측 (2026-07-19 Nucleus listing, settings RobotAssetConfig 주석과 동일값).
PALLET_USD = ("/Isaac/Props/Pallet/pallet.usd",)
PALLET_SIZE = (1.213, 0.802, 0.143)   # m. EUR 팔레트
# 표준 재사용 컨테이너(KLT 빈) — MM 이 과실을 담는 크레이트. 팔레트 위에 얹어 나른다.
CRATE_USD = ("/Isaac/Props/KLT_Bin/small_KLT.usd",)
KLT_SIZE = (0.297, 0.198, 0.146)      # m. small_KLT (긴 축을 X 로 놓음)

# 팔레트/KLT USD 피벗이 '중심'이라 가정하고 지지면 위에 앉힌다(중심을 반높이만큼 올림).
# 사용자 보고(2026-07-20): KLT 가 팔레트에 박혀 있었다 = 피벗이 중심인데 안 올렸던 것.
# GPU 확인 결과 바닥 피벗이면 해당 값을 0 으로 바꾸면 된다(둘 다 독립).
PALLET_PIVOT_Z = 0.0                   # 팔레트 USD 피벗=바닥(실측: 안 그러면 KLT 가 팔레트에 박힘)
KLT_PIVOT_Z = KLT_SIZE[2] / 2.0        # KLT 는 내가 만드는 큐브(피벗=중심) → 반높이 올림

# 랙 골조 치수 — 시각/충돌용 구조물. 슬롯 좌표(=하역 목표)에는 BASE_Z 만 영향.
BASE_Z = 0.35        # m. 1단 선반 높이. ForkliftB 포크 하한(-0.15m, 실측)보다 위 [2]
RACK_DEPTH = 1.00    # m. 선반 깊이 = 팔레트 깊이 0.802 + 포크/앞뒤 여유 [2]
POST_T = 0.08        # m. 기둥 두께 (온실 프레임과 동일 규격)
TOP_MARGIN = 0.45    # m. 최상단 선반 위 여유 (팔레트 출입 공간)


class Warehouse:
    """창고 슬롯 6개. 배경 환경은 선택적으로 얹는다."""

    def __init__(self, cfg: WarehouseConfig, sector_count: int):
        self._cfg = cfg
        self._sector_count = sector_count
        if cfg.slots != sector_count:
            raise ValueError(
                f"슬롯 {cfg.slots}개 != 재배 섹터 {sector_count}개. "
                "1:1 매핑이 깨지면 창고 배치 로직이 탐색 문제가 된다 "
                "(v3 10장). 둘을 맞출 것.")
        self._slots: list[dict] = []
        self._decor: list[tuple] = []      # 장식 팔레트 위치(비활성 베이)

    @property
    def slots(self) -> list[dict]:
        """{index, sector, path, position, level}.

        **슬롯 할당은 여기서 안 한다.** 섹터->슬롯 매핑과 하역 기록은
        `warehouse_manager_node` (개인 PC, ROS2) 파트다 — v3 6.3.
        Isaac 은 슬롯이 어디 있는지만 알려준다."""
        return self._slots

    def spawn(self, stage: Usd.Stage, root: str = "/World/Warehouse",
              origin: tuple[float, float, float] = (0.0, 0.0, 0.0),
              room_w: float | None = None, log=print) -> None:
        self._root = root
        self._origin = origin
        UsdGeom.Xform.Define(stage, root)
        UsdGeom.Xformable(stage.GetPrimAtPath(root)).AddTranslateOp().Set(
            Gf.Vec3d(*origin))

        lvl_h = self._cfg.level_height
        pitch = self._cfg.slot_pitch
        if lvl_h is None or pitch is None:
            # 값이 없으면 멈추는 게 낫다. 임의로 채우면 [4] 를 조용히 늘리는 것이다.
            raise ValueError(
                "WarehouseConfig.level_height / slot_pitch 가 미정이다.\n"
                "  창고 랙 에셋 또는 배경 선반 좌표를 재서 채울 것.\n"
                "  level_height 는 AMR 포크 승강 범위를 결정하므로 AMR 담당과 합의 필요.")

        # 랙(선반)은 중앙이 아니라 방 뒷벽(+y, 입구 반대쪽) 바로 앞에 붙인다 — 중앙은
        # 지게차 회전 공간으로 비운다(팀 피드백 2026-07-19). 뒷벽 앞 10cm 띄움.
        rack_y = self._cfg.depth / 2.0 - RACK_DEPTH / 2.0 - 0.10
        self._rack_y = rack_y

        # 뒷벽 랙 베이 수 — 방 폭을 채우게 넓힌다(창고가 휑하지 않게, 팀 피드백 2026-07-19).
        # 가운데 sectors 베이만 '실제 슬롯'(재배 섹터와 1:1), 나머지는 장식 팔레트로 채운다.
        # room_w 없으면(단위 테스트) 활성 베이만.
        if room_w:
            # 뒷벽 랙 폭 — 좌우 코너에 측벽 랙(깊이 RACK_DEPTH)이 들어갈 자리를 남긴다.
            n_bays = max(self._cfg.sectors,
                         int((room_w - 2.0 * (RACK_DEPTH + 1.0)) / pitch))
        else:
            n_bays = self._cfg.sectors
        self._n_bays = n_bays
        self._add_rack_frame(stage, root, rack_y, n_bays)

        active_start = (n_bays - self._cfg.sectors) // 2       # 활성 베이를 가운데로
        def bay_x(b: int) -> float:
            return (b - (n_bays - 1) / 2.0) * pitch

        for i in range(self._cfg.slots):
            sector_idx = i
            bay, level = divmod(i, self._cfg.levels)   # 섹터1->베이0 하단, 섹터2->베이0 상단, ...
            x = bay_x(active_start + bay)              # 가운데 sectors 베이에 배치
            z = BASE_Z + level * lvl_h
            path = f"{root}/Slot_{i:02d}"
            self._add_slot(stage, path, (x, rack_y, z))
            self._slots.append({
                "index": i,
                "sector": sector_idx,
                "path": path,
                "position": (origin[0] + x, origin[1] + rack_y, origin[2] + z),
                "local": (x, rack_y, z),       # root 기준 (크레이트 배치용)
                "level": level,
            })

        # 장식 팔레트 위치(뒷벽 비활성 베이) — (x, y, z, yaw). 뒷벽은 yaw=0.
        active = set(range(active_start, active_start + self._cfg.sectors))
        self._decor = [(bay_x(b), rack_y, BASE_Z + lvl * lvl_h, 0.0)
                       for b in range(n_bays) if b not in active
                       for lvl in range(self._cfg.levels)]

        # 좌·우 벽에도 장식 랙 — 입구/공유벽(-y) 제외 = 3면 선반 (팀 피드백 2026-07-20).
        if room_w:
            self._room_w = room_w
            self._build_side_rack(stage, root, sign=-1)   # 좌(-x)
            self._build_side_rack(stage, root, sign=+1)   # 우(+x)

        log(f"[Warehouse] 활성 슬롯 {len(self._slots)} + 장식 팔레트 {len(self._decor)} "
            f"(뒷벽 {n_bays}베이 + 좌우벽 랙 = 3면, 입구벽 제외) "
            f"-> 재배 {self._sector_count}섹터 1:1")

    def spawn_environment(self, stage: Usd.Stage, assets_cfg,
                          origin: tuple[float, float, float] = (0.0, 0.0, 0.0),
                          path: str = "/World/WarehouseEnv", log=print) -> bool:
        """배경 창고 건물(Isaac Simple_Warehouse). 실패해도 슬롯/랙은 살아 있다.

        독립 모듈 — 이 건물은 슬롯·랙과 안 엮인다. 빼고 싶으면 이 호출만 빼면 된다.
        origin: 건물을 놓을 위치(월드). 온실과 안 겹치게 창고 구역으로 민다.
        """
        from isaacsim.core.utils.stage import add_reference_to_stage

        from pjt_utils.xform import set_translate
        from robots import assets
        try:
            url = assets.resolve(assets_cfg.warehouse_env, "창고 환경")
        except FileNotFoundError as e:
            log(f"[Warehouse] 배경 건물 없음 — 슬롯/랙만 쓴다.\n{e}")
            return False
        add_reference_to_stage(url, path)
        set_translate(stage.GetPrimAtPath(path), origin)
        log(f"[Warehouse] 배경 건물: {url}  @ {origin}")
        return True

    def load_crates(self, stage: Usd.Stage, log=print) -> None:
        """슬롯에 '나무 팔레트 + KLT 빈 세트'를 얹어 '적재된 창고' 모습을 만든다.

        물류 루프(2026-07-19): 지게차가 팔레트째(위에 MM 이 채운 KLT) 랙에 올린다.
        어느 슬롯이 실제로 찼는지(할당)는 warehouse_manager_node (개인 PC) 몫 — v3 6.3.
        여기선 시각 연출일 뿐이다(§5.6: Isaac 은 위치만, 정책은 ROS2).
        ※ 팔레트/KLT USD 의 원점(피벗)·업축은 GPU 에서 확인해 z 오프셋을 보정할 것.
        """
        from isaacsim.core.utils.stage import add_reference_to_stage

        from pjt_utils.xform import set_pose, set_scale
        from robots import assets
        try:
            pallet_url = assets.resolve(PALLET_USD, "나무 팔레트(EUR)")
        except FileNotFoundError as e:
            log(f"[Warehouse] 팔레트 에셋 없음 — 빈 슬롯으로 둔다.\n{e}")
            return
        try:
            klt_url = assets.resolve(CRATE_USD, "KLT 빈")
        except FileNotFoundError:
            klt_url = None      # 팔레트만 얹는다
        # 팔레트 마찰 재질은 한 번만 만들어 전 팔레트가 공유한다(§5.7 값은 settings 에서).
        pp = self._cfg.pallet_physics
        pmat = physics.create_physics_material(
            stage, f"{self._root}/PhysMat/pallet",
            pp.static_friction, pp.dynamic_friction)
        for s in self._slots:                    # 활성 슬롯(뒷벽): 팔레트 + 실제 KLT 빈
            self._place_pallet(stage, add_reference_to_stage, set_pose, set_scale,
                               f"{self._root}/Pallet_{s['index']:02d}", s["local"],
                               pallet_url, klt_url, pmat, yaw=0.0)
        for j, (dx, dy, dz, dyaw) in enumerate(self._decor):   # 장식(뒷벽+좌우벽)
            self._place_pallet(stage, add_reference_to_stage, set_pose, set_scale,
                               f"{self._root}/Decor_{j:02d}", (dx, dy, dz),
                               pallet_url, klt_url, pmat, yaw=dyaw)
        log(f"[Warehouse] 팔레트 {len(self._slots) + len(self._decor)}개 + 실제 KLT 8/팔레트 "
            f"(활성 {len(self._slots)}+장식 {len(self._decor)}, 3면 랙 — 할당은 ROS2)")

    def _place_pallet(self, stage, add_ref, set_ps, set_sc, path, local,
                      pallet_url, klt_url, pmat, yaw: float = 0.0) -> None:
        """선반 로컬 좌표에 나무 팔레트(USD) 1개 + 그 위(자식)에 KLT 빈 8개(4×2) = 한 세트.

        팔레트는 진짜 물리 객체다(질량·마찰·convexDecomposition 콜라이더 → 다이내믹) —
        지게차가 형상으로 실제로 들 수 있게(스파이크 06 검증). pmat 은 공유 마찰 재질.

        ★ KLT 는 팔레트 prim 의 **자식**으로 넣는다 → PhysX 가 팔레트 강체 하나에 흡수해
          팔레트가 움직이면 KLT 도 계층으로 같이 딸려 간다(IW 운반·지게차 리프트 시 한 덩어리).
          KLT 에 강체(RigidBodyAPI)를 **주지 않는다** — 따로 강체로 만들면 리프트·주행 때
          미끄러지고 튕겨 개판이 된다(사용자 우려 2026-07-20). 계층 결합이라 고정조인트도 불필요.
          자식이라 회전은 팔레트가 갖고, KLT 로컬 op 엔 회전 안 뺀 격자 오프셋만 쓴다(§8 규칙).
          ※ MM 이 토마토를 담는 '활성 세트'에서는 KLT 안이 비어야 하므로(담김) 그때 KLT 벽에
            오목 콜라이더(convexDecomposition)를 따로 준다 — 지금 랙 세트는 빈 장식이라 생략.

        KLT 는 실제 에셋(빈 모양)이되 텍스처 머티리얼을 벗기고 displayColor 로 칠한다 —
        텍스처째 넣으면 재질 예산이 밀려 식물 색이 죽었다(2026-07-20). yaw 로 세트를 눕힌다(측벽 90°).
        """
        lx, ly, lz = local
        support = lz - SLOT_SIZE[2] / 2.0 + 0.002            # 랙 선반 윗면 = 팔레트가 직접 앉는 면
        quat = self._yaw_quat(yaw)
        add_ref(pallet_url, path)
        set_ps(stage.GetPrimAtPath(path), (lx, ly, support + PALLET_PIVOT_Z), quat)
        self._apply_pallet_physics(stage, path, pmat)
        if not klt_url:
            return
        klt_scale = 0.85                                     # 살짝 줄여 8개가 붙지 않고 구분되게
        kz = PALLET_SIZE[2] + KLT_SIZE[2] * klt_scale / 2.0  # 팔레트 로컬: 윗면 위(부모가 support)
        ident = self._yaw_quat(0.0)                          # 회전은 부모(팔레트)가 가짐
        nx, ny = 4, 2                                        # 4×2 = 8개
        pitx, pity = 0.31, 0.25                              # 간격 벌려 붙어보이지 않게(길이 4·폭 2)
        for ix in range(nx):
            for iy in range(ny):
                ox = (ix - (nx - 1) / 2.0) * pitx            # 팔레트 로컬 격자 오프셋(회전 전)
                oy = (iy - (ny - 1) / 2.0) * pity
                kp = f"{path}/KLT_{ix}{iy}"                  # ★ 팔레트의 자식 → 강체에 흡수
                add_ref(klt_url, kp)
                kprim = stage.GetPrimAtPath(kp)
                set_ps(kprim, (ox, oy, kz), ident)          # 팔레트 로컬 프레임(부모가 회전·이동)
                set_sc(kprim, klt_scale)                     # 실제 KLT 를 살짝 축소
                # 에셋의 무거운 텍스처 머티리얼을 벗기고 displayColor 로 칠한다.
                # 실제 KLT 를 텍스처째 넣으면 재질 예산이 밀려 식물 색이 죽었다(2026-07-20).
                for m in Usd.PrimRange(kprim):
                    if m.IsA(UsdGeom.Mesh):
                        UsdShade.MaterialBindingAPI(m).UnbindAllBindings()
                        UsdGeom.Gprim(m).CreateDisplayColorAttr([KLT_COLOR])

    def _apply_pallet_physics(self, stage: Usd.Stage, path: str, pmat) -> None:
        """팔레트 1개를 물리 객체로 — convexDecomposition 콜라이더 + 다이내믹 + 질량 + 마찰.

        스파이크 06 실측 검증(2026-07-20): 콜라이더가 포크 슬롯을 살려 지게차가 형상으로
        실제로 든다. 값·근거는 settings.PalletPhysicsConfig (§5.7).
        """
        pp = self._cfg.pallet_physics
        physics.add_convex_decomposition_colliders(
            stage, path, pp.max_convex_hulls, pp.voxel_resolution,
            pp.error_percentage)
        prim = stage.GetPrimAtPath(path)
        UsdPhysics.RigidBodyAPI.Apply(prim).CreateKinematicEnabledAttr(False)  # 다이내믹
        UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(pp.mass)                 # 질량 직접
        physics.bind_physics_material(prim, pmat)

    @staticmethod
    def _yaw_quat(deg: float) -> Gf.Quatd:
        r = Gf.Rotation(Gf.Vec3d(0.0, 0.0, 1.0), deg).GetQuat()
        return Gf.Quatd(r.GetReal(), r.GetImaginary())

    def spawn_building(self, stage: Usd.Stage, room_w: float, room_h: float,
                       log=print) -> None:
        """창고 방 — 바닥 + 벽 3면(뒤/좌/우) + 앞면 입구(온실 방향). **천장(지붕) 없음**.

        팀 피드백(2026-07-19):
          · 방은 지게차가 들어와 회전할 만큼 넓다(중앙 비움). 랙은 중앙이 아니라
            뒷벽 앞에 붙어(spawn 참고) AMR 이 입구로 들어와 바로 선반에 적재한다.
          · 폭(room_w)·높이(room_h)는 재배 공간과 같게 받는다(온실 width/height).
          · 천장 제거 — 위에서 내려다보는 시연 우선(온실 지붕과 동일 방침).
        독립 모듈: 랙/슬롯/컨테이너와 안 엮인다 — 빼려면 이 호출만 빼면 된다.
        """
        cfg = self._cfg
        W, D, H, T = room_w, cfg.depth, room_h, 0.15     # 폭(X)·깊이(Y)·높이·벽두께
        b = f"{self._root}/Building"
        UsdGeom.Xform.Define(stage, b)

        WALL = Gf.Vec3f(0.80, 0.80, 0.83)     # 밝은 회백 벽
        FLOOR = Gf.Vec3f(0.58, 0.58, 0.56)    # 콘크리트 바닥
        TRIM = Gf.Vec3f(0.25, 0.42, 0.70)     # 파란 간판

        # 바닥 슬래브 (홀 바닥 위 살짝 — 창고 구역 표시). 방 중심은 root 로컬 원점.
        self._add_box(stage, f"{b}/Floor", (0.0, 0.0, 0.03), (W, D, 0.05), FLOOR)
        # 뒷벽(+y, 랙이 붙는 벽) + 좌우벽
        self._add_box(stage, f"{b}/Wall_Back", (0.0, D / 2, H / 2), (W, T, H), WALL)
        self._add_box(stage, f"{b}/Wall_Left", (-W / 2, 0.0, H / 2), (T, D, H), WALL)
        self._add_box(stage, f"{b}/Wall_Right", (W / 2, 0.0, H / 2), (T, D, H), WALL)
        # 앞면(-y, 온실 방향): 가운데 입구를 비우고 옆벽 2쪽 + 위 상인방(header)만
        door_w = cfg.entrance_width
        side_w = (W - door_w) / 2.0
        for sx, tag in ((-1, "L"), (1, "R")):
            px = sx * (door_w / 2.0 + side_w / 2.0)
            self._add_box(stage, f"{b}/Front_{tag}", (px, -D / 2, H / 2),
                          (side_w, T, H), WALL)
        header_h = 0.9
        self._add_box(stage, f"{b}/Front_Header", (0.0, -D / 2, H - header_h / 2),
                      (door_w, T, header_h), WALL)
        # 앞쪽 간판 (지붕/천장은 없음 — 제거됨)
        self._add_box(stage, f"{b}/Sign", (0.0, -D / 2 - 0.06, H - 0.45),
                      (door_w * 0.95, 0.06, 0.55), TRIM)
        log(f"[Warehouse] 창고 방 {W:.1f}x{D:.1f}x{H:.1f}m (천장 없음), "
            f"입구 {door_w:.1f}m · 랙=뒷벽 · 중앙 지게차 회전공간")

    # ----- 내부 -----

    def _add_rack_frame(self, stage: Usd.Stage, root: str, rack_y: float,
                        n_bays: int) -> None:
        """랙 골조 — 기둥 + 단별 선반 + 상단 보/간판. 방 뒷벽 앞(rack_y)에 선다.

        n_bays 로 방 폭을 채운다(창고가 휑하지 않게). 슬롯 판만 허공에 띄우면 랙으로
        안 읽힌다(시연/발표 화면). 골조는 static collider 라 포크 정렬 실수도 물리로 드러난다.
        """
        cfg = self._cfg
        pitch, lvl_h = cfg.slot_pitch, cfg.level_height
        width = n_bays * pitch                             # 베이 전폭
        post_h = BASE_Z + (cfg.levels - 1) * lvl_h + TOP_MARGIN

        frame = f"{root}/Frame"
        UsdGeom.Xform.Define(stage, frame)

        # 기둥 — 베이 경계마다 앞뒤 한 쌍 (뒷벽 폭에 중앙정렬)
        for i in range(n_bays + 1):
            x = -width / 2.0 + i * pitch
            for side, dy in (("F", -RACK_DEPTH / 2.0), ("B", RACK_DEPTH / 2.0)):
                self._add_box(stage, f"{frame}/Post_{side}_{i:02d}",
                              (x, rack_y + dy, post_h / 2.0),
                              (POST_T, POST_T, post_h), FRAME_COLOR)

        # 단별 선반 — 슬롯 판 바로 아래를 받친다
        for lvl in range(cfg.levels):
            z_slot = BASE_Z + lvl * lvl_h
            shelf_z = z_slot - SLOT_SIZE[2] / 2.0 - 0.025
            self._add_box(stage, f"{frame}/Shelf_{lvl}",
                          (0.0, rack_y, shelf_z),
                          (width + POST_T, RACK_DEPTH, 0.05), SHELF_COLOR)

        # 상단 보 (앞뒤) — 기둥 맨 위를 잇는 연결보. 구조상 유지(사용자 정정 2026-07-20).
        for side, dy in (("F", -RACK_DEPTH / 2.0), ("B", RACK_DEPTH / 2.0)):
            self._add_box(stage, f"{frame}/TopBeam_{side}",
                          (0.0, rack_y + dy, post_h),
                          (width + POST_T, POST_T, POST_T), FRAME_COLOR)
        # 간판 패널(Header)은 제거 — 연결보 위에 0.28m 떠 있어 공중에 뜬 걸로 보였음(사용자 지적).

    def _build_side_rack(self, stage: Usd.Stage, root: str, sign: int) -> None:
        """좌(-1)/우(+1) 벽에 장식 랙(골조+선반)과 팔레트(90° 회전) 위치를 만든다.

        입구/공유벽(-y)엔 안 놓는다 → 뒷벽 랙과 합쳐 3면 선반(팀 피드백 2026-07-20).
        팔레트 위치는 self._decor 에 (x, y, z, yaw=90) 로 쌓여 load_crates 가 얹는다.
        벽을 따라 Y 로 뻗고, 앞(-y, 입구쪽)은 지게차 진입/회전용으로 조금 비운다.
        """
        cfg = self._cfg
        pitch, lvl_h = cfg.slot_pitch, cfg.level_height
        hx = self._room_w / 2.0
        rack_x = sign * (hx - RACK_DEPTH / 2.0 - 0.10)      # 벽 바로 앞
        n = max(1, int((cfg.depth - 2.5) / pitch))          # 입구쪽 여유 남기고 베이 수
        width = n * pitch                                    # Y 방향 전길이
        post_h = BASE_Z + (cfg.levels - 1) * lvl_h + TOP_MARGIN
        tag = "L" if sign < 0 else "R"
        frame = f"{root}/SideRack_{tag}"
        UsdGeom.Xform.Define(stage, frame)

        # 기둥 — 베이 경계마다 (룸측/벽측) 한 쌍
        for i in range(n + 1):
            y = -width / 2.0 + i * pitch
            for s2, dx in (("A", -RACK_DEPTH / 2.0), ("B", RACK_DEPTH / 2.0)):
                self._add_box(stage, f"{frame}/Post_{s2}_{i:02d}",
                              (rack_x + dx, y, post_h / 2.0),
                              (POST_T, POST_T, post_h), FRAME_COLOR)
        # 단별 선반 (깊이 RACK_DEPTH 는 X, 길이는 Y)
        for lvl in range(cfg.levels):
            z_slot = BASE_Z + lvl * lvl_h
            shelf_z = z_slot - SLOT_SIZE[2] / 2.0 - 0.025
            self._add_box(stage, f"{frame}/Shelf_{lvl}",
                          (rack_x, 0.0, shelf_z),
                          (RACK_DEPTH, width + POST_T, 0.05), SHELF_COLOR)
        # 상단 보 (앞뒤) — 기둥 맨 위를 잇는 연결보. 구조상 유지(사용자 정정 2026-07-20).
        for s2, dx in (("A", -RACK_DEPTH / 2.0), ("B", RACK_DEPTH / 2.0)):
            self._add_box(stage, f"{frame}/TopBeam_{s2}",
                          (rack_x + dx, 0.0, post_h),
                          (POST_T, width + POST_T, POST_T), FRAME_COLOR)

        # 장식 팔레트 위치 — 벽을 따라 Y 로 배열, 90° 회전(폭이 벽을 따라 눕게)
        for b in range(n):
            y = (b - (n - 1) / 2.0) * pitch
            for lvl in range(cfg.levels):
                self._decor.append((rack_x, y, BASE_Z + lvl * lvl_h, 90.0))

    def _add_box(self, stage: Usd.Stage, path: str,
                 pos: tuple[float, float, float],
                 size: tuple[float, float, float], color: Gf.Vec3f) -> None:
        box = UsdGeom.Cube.Define(stage, path)
        box.CreateSizeAttr(1.0)
        box.CreateDisplayColorAttr([color])
        xf = UsdGeom.Xformable(box.GetPrim())
        xf.AddTranslateOp().Set(Gf.Vec3d(*pos))
        xf.AddScaleOp().Set(Gf.Vec3f(*size))
        physics.add_shape_collider(box.GetPrim())

    def _add_slot(self, stage: Usd.Stage, path: str,
                  pos: tuple[float, float, float]) -> None:
        """슬롯 = 위치 마커(Xform)만. 실제 지지는 랙 선반이 한다.

        예전엔 여기 파란 바닥판(SLOT_COLOR) + 포크 가이드를 뒀으나(포크 드롭 타깃), 팔레트가
        랙 선반에 직접 얹히면서 시각·기능이 겹쳐 제거했다(2026-07-20 사용자 지적: 팔레트 밑
        파란 박스 불필요). 위치는 self._slots 딕셔너리에 남아 ROS2(warehouse_manager)가
        하역 타깃으로 쓴다 — Isaac 은 위치만, 정책은 ROS2(§5.6).
        """
        UsdGeom.Xform.Define(stage, path)
        UsdGeom.Xformable(stage.GetPrimAtPath(path)).AddTranslateOp().Set(
            Gf.Vec3d(*pos))
