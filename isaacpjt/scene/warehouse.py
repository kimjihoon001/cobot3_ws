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

from pxr import Gf, Usd, UsdGeom

from pjt_config.settings import WarehouseConfig
from scene import physics

SLOT_COLOR = Gf.Vec3f(0.30, 0.45, 0.70)
GUIDE_COLOR = Gf.Vec3f(0.85, 0.65, 0.15)
FRAME_COLOR = Gf.Vec3f(0.82, 0.82, 0.84)    # 랙 골조 (밝은 회백)
SHELF_COLOR = Gf.Vec3f(0.55, 0.56, 0.58)    # 선반 (강판 회색)

# [4] 임의 — 슬롯 치수. 트레이가 정해지면 거기서 유도된다([2]).
#     지금은 트레이 크기 자체가 미정이라 임시값이다.
SLOT_SIZE = (0.50, 0.40, 0.03)      # m. 슬롯 바닥판

# 표준 재사용 컨테이너(트레이) — Isaac 내장 KLT 빈. GRoW(Ridder) 등 상용 수확 로봇도
# 표준 컨테이너에 담아 트롤리에 쌓는다. 우리도 이 표준 컨테이너를 트레이로 쓴다.
CRATE_USD = ("/Isaac/Props/KLT_Bin/small_KLT.usd",)

# 랙 골조 치수 — 시각/충돌용 구조물. 슬롯 좌표(=하역 목표)에는 BASE_Z 만 영향.
BASE_Z = 0.35        # m. 1단 선반 높이. ForkliftB 포크 하한(-0.15m, 실측)보다 위 [2]
RACK_DEPTH = 0.70    # m. 선반 깊이 = SLOT_SIZE[1] + 앞뒤 여유 [4]
POST_T = 0.08        # m. 기둥 두께 (온실 프레임과 동일 규격)
TOP_MARGIN = 0.45    # m. 최상단 선반 위 여유 (트레이 출입 공간)


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

    @property
    def slots(self) -> list[dict]:
        """{index, sector, path, position, level}.

        **슬롯 할당은 여기서 안 한다.** 섹터->슬롯 매핑과 하역 기록은
        `warehouse_manager_node` (개인 PC, ROS2) 파트다 — v3 6.3.
        Isaac 은 슬롯이 어디 있는지만 알려준다."""
        return self._slots

    def spawn(self, stage: Usd.Stage, root: str = "/World/Warehouse",
              origin: tuple[float, float, float] = (0.0, 0.0, 0.0),
              log=print) -> None:
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

        self._add_rack_frame(stage, root)

        for i in range(self._cfg.slots):
            sector_idx = i
            col, level = divmod(i, self._cfg.levels)   # 섹터1->0단, 섹터2->1단, ...
            x = col * pitch
            z = BASE_Z + level * lvl_h
            path = f"{root}/Slot_{i:02d}"
            self._add_slot(stage, path, (x, 0.0, z))
            self._slots.append({
                "index": i,
                "sector": sector_idx,
                "path": path,
                "position": (origin[0] + x, origin[1], origin[2] + z),
                "local": (x, 0.0, z),          # root 기준 (크레이트 배치용)
                "level": level,
            })

        log(f"[Warehouse] 슬롯 {len(self._slots)}개 "
            f"({self._cfg.sectors}섹터 x {self._cfg.levels}단) "
            f"-> 재배 {self._sector_count}섹터와 1:1")

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
        """슬롯에 표준 컨테이너(KLT 빈)를 얹어 '적재된 창고' 모습을 만든다.

        어느 슬롯이 실제로 찼는지(할당)는 warehouse_manager_node (개인 PC) 몫 — v3 6.3.
        여기선 시각 연출일 뿐이다(§5.6: Isaac 은 위치만, 정책은 ROS2).
        """
        from isaacsim.core.utils.stage import add_reference_to_stage

        from pjt_utils.xform import set_translate
        from robots import assets
        try:
            url = assets.resolve(CRATE_USD, "표준 컨테이너(KLT 빈)")
        except FileNotFoundError as e:
            log(f"[Warehouse] 컨테이너 에셋 없음 — 빈 슬롯으로 둔다.\n{e}")
            return
        for s in self._slots:
            lx, ly, lz = s["local"]
            # 슬롯 판(스케일 걸림) 밑이 아니라 root(변환만) 밑에 둬 스케일 오염을 피한다.
            p = f"{self._root}/Crate_{s['index']:02d}"
            add_reference_to_stage(url, p)
            set_translate(stage.GetPrimAtPath(p),
                          (lx, ly, lz + SLOT_SIZE[2] / 2.0 + 0.01))
        log(f"[Warehouse] 표준 컨테이너 {len(self._slots)}개 적재 (시각 — 할당은 ROS2)")

    def spawn_building(self, stage: Usd.Stage, log=print) -> None:
        """커스텀 창고 건물 — 벽 3면 + 지붕 + 앞쪽 로딩 개구부(AMR 진입) + 간판.

        Isaac Simple_Warehouse 는 토마토와 무관한 재고가 딸려와 대신 우리가 짓는다.
        독립 모듈: 랙/슬롯/컨테이너와 안 엮인다 — 빼려면 이 호출만 빼면 된다.
        치수는 랙(폭 sectors*pitch, 높이 post_h)을 감싸고 AMR 통로를 확보하게 잡는다.
        """
        cfg = self._cfg
        mid_x = (cfg.sectors - 1) * cfg.slot_pitch / 2.0     # 랙 열 중심 (root 로컬)
        W, D, H, T = 6.0, 5.2, 3.6, 0.15     # 폭(X)·깊이(Y)·높이·벽두께 [4] 임의
        cx, cy = mid_x, 0.0                   # 건물 중심 (root 로컬)
        b = f"{self._root}/Building"
        UsdGeom.Xform.Define(stage, b)

        WALL = Gf.Vec3f(0.80, 0.80, 0.83)     # 밝은 회백 벽
        ROOF = Gf.Vec3f(0.38, 0.40, 0.45)     # 금속 회색 지붕
        FLOOR = Gf.Vec3f(0.58, 0.58, 0.56)    # 콘크리트 바닥
        TRIM = Gf.Vec3f(0.25, 0.42, 0.70)     # 파란 트림/간판대

        # 바닥 슬래브 (홀 바닥 위 살짝 — 창고 구역 표시)
        self._add_box(stage, f"{b}/Floor", (cx, cy, 0.03), (W, D, 0.05), FLOOR)
        # 벽 3면 (뒤/좌/우)
        self._add_box(stage, f"{b}/Wall_Back", (cx, cy + D / 2, H / 2), (W, T, H), WALL)
        self._add_box(stage, f"{b}/Wall_Left", (cx - W / 2, cy, H / 2), (T, D, H), WALL)
        self._add_box(stage, f"{b}/Wall_Right", (cx + W / 2, cy, H / 2), (T, D, H), WALL)
        # 앞쪽(-Y): 로딩 개구부. 문 옆벽 2쪽 + 위 상인방(header)만, 가운데는 뚫림
        door_w = 3.2
        side_w = (W - door_w) / 2.0
        for sx, tag in ((-1, "L"), (1, "R")):
            px = cx + sx * (door_w / 2.0 + side_w / 2.0)
            self._add_box(stage, f"{b}/Front_{tag}", (px, cy - D / 2, H / 2),
                          (side_w, T, H), WALL)
        self._add_box(stage, f"{b}/Front_Header", (cx, cy - D / 2, H - 0.45),
                      (door_w, T, 0.9), WALL)
        # 지붕(약간 오버행) + 앞쪽 간판대
        self._add_box(stage, f"{b}/Roof", (cx, cy, H + 0.06), (W + 0.3, D + 0.3, 0.12), ROOF)
        self._add_box(stage, f"{b}/Sign", (cx, cy - D / 2 - 0.06, H - 0.45),
                      (door_w * 0.95, 0.06, 0.55), TRIM)
        log(f"[Warehouse] 창고 건물 {W:.0f}x{D:.1f}x{H:.1f}m, 앞 개구부 {door_w:.1f}m (AMR 진입)")

    # ----- 내부 -----

    def _add_rack_frame(self, stage: Usd.Stage, root: str) -> None:
        """랙 골조 — 기둥 + 단별 선반 + 상단 보/간판.

        슬롯 판만 허공에 띄우면 랙으로 안 읽힌다(시연/발표 화면). 골조는
        static collider 라 포크 정렬 실수도 물리로 드러난다.
        """
        cfg = self._cfg
        pitch, lvl_h = cfg.slot_pitch, cfg.level_height
        width = cfg.sectors * pitch                        # 베이 전폭
        mid_x = (cfg.sectors - 1) * pitch / 2.0            # 슬롯 열 중심
        post_h = BASE_Z + (cfg.levels - 1) * lvl_h + TOP_MARGIN

        frame = f"{root}/Frame"
        UsdGeom.Xform.Define(stage, frame)

        # 기둥 — 베이 경계마다 앞뒤 한 쌍
        for i in range(cfg.sectors + 1):
            x = -pitch / 2.0 + i * pitch
            for side, y in (("F", -RACK_DEPTH / 2.0), ("B", RACK_DEPTH / 2.0)):
                self._add_box(stage, f"{frame}/Post_{side}_{i:02d}",
                              (x, y, post_h / 2.0), (POST_T, POST_T, post_h),
                              FRAME_COLOR)

        # 단별 선반 — 슬롯 판 바로 아래를 받친다
        for lvl in range(cfg.levels):
            z_slot = BASE_Z + lvl * lvl_h
            shelf_z = z_slot - SLOT_SIZE[2] / 2.0 - 0.025
            self._add_box(stage, f"{frame}/Shelf_{lvl}",
                          (mid_x, 0.0, shelf_z),
                          (width + POST_T, RACK_DEPTH, 0.05), SHELF_COLOR)

        # 상단 보 (앞뒤) + 간판 패널
        for side, y in (("F", -RACK_DEPTH / 2.0), ("B", RACK_DEPTH / 2.0)):
            self._add_box(stage, f"{frame}/TopBeam_{side}",
                          (mid_x, y, post_h), (width + POST_T, POST_T, POST_T),
                          FRAME_COLOR)
        self._add_box(stage, f"{frame}/Header",
                      (mid_x, 0.0, post_h + 0.28),
                      (width * 0.9, 0.05, 0.35), Gf.Vec3f(0.95, 0.95, 0.95))

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
        """슬롯 바닥판 + 포크 삽입 가이드.

        가이드(테이퍼)를 두는 이유 — v3 10장이 "포크 삽입 정렬 오차" 를 **새로운
        리스크**로 지목했다. 물리 가이드가 있으면 정렬 요구 정밀도가 내려간다.
        """
        plate = UsdGeom.Cube.Define(stage, path)
        plate.CreateSizeAttr(1.0)
        plate.CreateDisplayColorAttr([SLOT_COLOR])
        xf = UsdGeom.Xformable(plate.GetPrim())
        xf.AddTranslateOp().Set(Gf.Vec3d(*pos))
        xf.AddScaleOp().Set(Gf.Vec3f(*SLOT_SIZE))
        physics.add_shape_collider(plate.GetPrim())

        # 양옆 가이드. 포크가 빗나가도 트레이를 슬롯으로 밀어넣는다.
        for side, sign in (("L", -1.0), ("R", 1.0)):
            g = UsdGeom.Cube.Define(stage, f"{path}/Guide_{side}")
            g.CreateSizeAttr(1.0)
            g.CreateDisplayColorAttr([GUIDE_COLOR])
            gxf = UsdGeom.Xformable(g.GetPrim())
            gxf.AddTranslateOp().Set(
                Gf.Vec3d(pos[0], pos[1] + sign * SLOT_SIZE[1] / 2.0,
                         pos[2] + 0.04))
            gxf.AddScaleOp().Set(Gf.Vec3f(SLOT_SIZE[0], 0.01, 0.05))
            physics.add_shape_collider(g.GetPrim())
