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
                "level": level,
            })

        log(f"[Warehouse] 슬롯 {len(self._slots)}개 "
            f"({self._cfg.sectors}섹터 x {self._cfg.levels}단) "
            f"-> 재배 {self._sector_count}섹터와 1:1")

    def spawn_environment(self, stage: Usd.Stage, assets_cfg,
                          path: str = "/World/WarehouseEnv", log=print) -> bool:
        """배경 창고 환경(Isaac Simple_Warehouse). 실패해도 슬롯은 살아 있다."""
        from isaacsim.core.utils.stage import add_reference_to_stage

        from robots import assets
        try:
            url = assets.resolve(assets_cfg.warehouse_env, "창고 환경")
        except FileNotFoundError as e:
            log(f"[Warehouse] 배경 환경 없음 — 슬롯만 쓴다.\n{e}")
            return False
        add_reference_to_stage(url, path)
        log(f"[Warehouse] 배경 환경: {url}")
        return True

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
