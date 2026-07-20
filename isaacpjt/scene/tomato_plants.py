# -*- coding: utf-8 -*-
"""토마토 재배 라인 스폰 — 줄기 + 트렐리스 바 + 매달린 토마토.

- 과실은 tomato_assets_usd 의 모양 변형 중 랜덤 선택 (인스턴스별 참조)
- 익음 클래스(green/half_ripe/fully_ripe/old)를 가중치 랜덤 배정, 색은 ripeness 로 적용
- 줄기/트렐리스 = static collider (로봇이 통과 못 함)
- 과실 = kinematic RigidBody (매달림). 수확 순간 kinematic 을 꺼서 분리한다.
  physics.set_kinematic(prim, False) 참고.
"""
import math
import os
import random

from pxr import Gf, PhysxSchema, Usd, UsdGeom, UsdShade

from isaacsim.core.utils.stage import add_reference_to_stage

from pjt_config.settings import (GreenhouseConfig, PhysicsConfig, PlantConfig,
                             TomatoAssetConfig)
from scene import pedicel, physics
from pjt_utils import ripeness
from pjt_utils.xform import set_pose, set_scale

# aoc 배경 식물 잎 색 (무텍스처 → displayColor 로 초록. 루트 무광재질이 읽는다).
FOLIAGE_COLOR = Gf.Vec3f(0.20, 0.42, 0.16)

STEM_COLOR = Gf.Vec3f(0.25, 0.45, 0.15)


def _positions(usable: float, spacing: float) -> list[float]:
    """usable 구간에 spacing 간격으로 배치했을 때의 좌표 (중앙 기준).

    개수는 재식거리에서 유도된다. 설정값이 아니다.
    """
    n = max(1, int(usable / spacing) + 1)
    span = (n - 1) * spacing
    return [-span / 2.0 + i * spacing for i in range(n)]


class TomatoPlants:
    def __init__(self, cfg: PlantConfig, assets: TomatoAssetConfig,
                 greenhouse: GreenhouseConfig, phys: PhysicsConfig,
                 rng: random.Random):
        self._cfg = cfg
        self._assets = assets
        self._greenhouse = greenhouse
        self._phys = phys
        self._rng = rng
        self._fruit_count = 0
        self._fruits: list[dict] = []   # 수확 대상 목록 (FSM/Detector 가 사용)
        self._ped_cfg = pedicel.PedicelConfig()   # 꽃자루 치수([W2024])

    @property
    def fruits(self) -> list[dict]:
        """스폰된 과실 정보. {path, class_name, position} 목록.

        시뮬 안에는 정답이 이미 있으므로 GroundTruthDetector 가 이걸 그대로 쓴다.
        YOLO 는 나중에 같은 인터페이스로 갈아끼운다.
        """
        return self._fruits

    def spawn(self, stage: Usd.Stage, root: str = "/World/Plants") -> None:
        UsdGeom.Xform.Define(stage, root)
        # 무광 재질 — 없으면 RTX 기본 광택 재질이라 과실이 유리처럼 보인다.
        # displayColor(=클래스 색, YOLO 라벨 근거)는 그대로 읽는다.
        ripeness.bind_matte_material(stage, root)

        # aoc 배경 식물 옵션: 플래그 ON + 에셋 존재해야 켜진다 (없으면 조용히 건너뜀).
        self._aoc_bg = (self._cfg.use_aoc_background
                        and os.path.isfile(self._assets.background_plant_usd))
        if self._cfg.use_aoc_background and not self._aoc_bg:
            print(f"[WARN] 배경 식물 에셋 없음: {self._assets.background_plant_usd}"
                  " — 원기둥 줄기만 스폰")
        variants = self._find_usd_variants()
        self._fruit_material = physics.create_physics_material(
            stage, "/World/PhysicsMaterials/fruit",
            self._phys.fruit_static_friction, self._phys.fruit_dynamic_friction)

        c = self._cfg
        # 2x3 섹터 그리드 — 섹터 사이에 통로(주/교차)를 둬 로봇이 구역 간 이동 가능.
        col_xs = self._column_row_xs()      # 섹터 열별 이랑 x  [[x,x],[x,x]]
        seg_ys = self._segment_ys()         # 섹터 구획별 그루 y [[y..],[y..],[y..]]
        n_sectors = len(col_xs) * len(seg_ys)

        for sc, rxs in enumerate(col_xs):              # 섹터 열 (X)
            for sr, ys in enumerate(seg_ys):           # 섹터 구획 (Y)
                si = sc * len(seg_ys) + sr             # 섹터 인덱스 0..5 (창고 슬롯과 1:1)
                sp = f"{root}/Sector_{si:02d}"
                UsdGeom.Xform.Define(stage, sp)
                cy = (ys[0] + ys[-1]) / 2.0            # 이 구획의 중심 y
                seg_span = (ys[-1] - ys[0]) if len(ys) > 1 else 0.0
                for ri, x in enumerate(rxs):
                    rp = f"{sp}/Row_{ri:02d}"
                    UsdGeom.Xform.Define(stage, rp)
                    self._spawn_bed(stage, rp + "/Bed", x, cy, seg_span)
                    self._spawn_trellis_bar(stage, rp + "/Trellis", x, cy, seg_span)
                    for pi, y in enumerate(ys):
                        # 잎은 일부 그루만(덜 무성하게). 결정적 패턴 → rng 안 흔듦.
                        foliage = ((ri * 37 + pi * 17) % 100) < int(c.foliage_fraction * 100)
                        self._spawn_plant(stage, f"{rp}/Plant_{pi:02d}", x, y,
                                          variants, foliage, si)

        n_rows = c.sector_cols * c.rows_per_col
        n_plants = n_rows * c.sector_rows * c.plants_per_seg
        print("[Scene] %d섹터(2x3) · %d그루 · 과실 %d개 "
              "(통로 주%.1f/교차%.1f/수확%.1fm)"
              % (n_sectors, n_plants, self._fruit_count,
                 c.aisle_x, c.aisle_y, c.row_spacing))

    def _column_row_xs(self) -> list[list[float]]:
        """섹터 열별 이랑 x 좌표. 열 사이 aisle_x(주 통로)."""
        c = self._cfg
        colw = (c.rows_per_col - 1) * c.row_spacing     # 한 열의 이랑 폭
        pitch = colw + c.aisle_x                         # 열 중심 간격
        span = (c.sector_cols - 1) * pitch
        out = []
        for sc in range(c.sector_cols):
            cx = -span / 2.0 + sc * pitch
            out.append([cx - colw / 2.0 + ri * c.row_spacing
                        for ri in range(c.rows_per_col)])
        return out

    def _segment_ys(self) -> list[list[float]]:
        """섹터 구획별 그루 y 좌표. 구획 사이 aisle_y(교차 통로)."""
        c = self._cfg
        seglen = (c.plants_per_seg - 1) * c.plant_spacing
        pitch = seglen + c.aisle_y
        span = (c.sector_rows - 1) * pitch
        out = []
        for sr in range(c.sector_rows):
            cy = -span / 2.0 + sr * pitch
            out.append([cy - seglen / 2.0 + pi * c.plant_spacing
                        for pi in range(c.plants_per_seg)])
        return out

    # ----- 내부 -----

    def _find_usd_variants(self) -> list[tuple[str, str | None]]:
        """(몸통 usd, 꼭지 usd|None) 목록. 폴더 없으면 경고 후 빈 목록."""
        d = self._assets.usd_dir
        if not os.path.isdir(d):
            print(f"[WARN] 토마토 USD 폴더 없음: {d}")
            print("       isaac/tomatest/00_convert_obj_to_usd.py 를 먼저 실행하거나 "
                  "config/settings.py 의 usd_dir 을 수정하세요. 줄기만 스폰합니다.")
            return []
        out = []
        for f in sorted(os.listdir(d)):
            if f.endswith(".usd") and not f.endswith("_calyx.usd"):
                body = os.path.join(d, f)
                calyx = os.path.join(d, f[:-4] + "_calyx.usd")
                out.append((body, calyx if os.path.exists(calyx) else None))
        return out

    def _spawn_bed(self, stage: Usd.Stage, path: str,
                   x: float, cy: float, length: float) -> None:
        """재배 베드 — 배지경 양액재배의 배지백 라인 (흰 상자). cy=구획 중심 y.

        시각 + 충돌. 로봇 베이스가 이랑을 가로지르지 못하게 막는 역할도 한다
        (실제 온실에서도 베드가 통로를 규정한다).
        """
        bed = UsdGeom.Cube.Define(stage, path)
        bed.CreateSizeAttr(1.0)
        bed.CreateDisplayColorAttr([Gf.Vec3f(0.90, 0.90, 0.87)])
        xf = UsdGeom.Xformable(bed.GetPrim())
        # 높이 0.40m — [2] 유도. harvester_nav.lidar_offset z=0.35(로컬, base_link=지면 기준)
        # 보다 높여야 2D 라이다 스캔 평면이 베드를 가로질러 장애물로 잡는다(2026-07-20
        # 사용자 요청 — 기존 0.20m 은 스캔 평면 아래라 라이다에 아예 안 보였다. 0.50m 은
        # 과했다는 피드백으로 0.40m 로 낮춤 — 스캔 평면보다 0.05m 여유).
        xf.AddTranslateOp().Set(Gf.Vec3d(x, cy, 0.20))
        xf.AddScaleOp().Set(Gf.Vec3f(0.42, length + 0.5, 0.40))
        physics.add_shape_collider(bed.GetPrim())

    def _spawn_trellis_bar(self, stage: Usd.Stage, path: str,
                           x: float, cy: float, length: float) -> None:
        """줄기 상단을 잇는 수평 지지대. cy=구획 중심 y."""
        c = self._cfg
        bar = UsdGeom.Cylinder.Define(stage, path)
        r = c.stem_radius
        bar.CreateRadiusAttr(r)
        bar.CreateHeightAttr(length)
        bar.CreateAxisAttr("Y")
        bar.CreateExtentAttr([Gf.Vec3f(-r, -length / 2.0, -r),
                              Gf.Vec3f(r, length / 2.0, r)])
        bar.CreateDisplayColorAttr([Gf.Vec3f(0.55, 0.55, 0.55)])
        UsdGeom.Xformable(bar.GetPrim()).AddTranslateOp().Set(
            Gf.Vec3d(x, cy, c.stem_height))
        physics.add_shape_collider(bar.GetPrim())

    def _spawn_plant(self, stage: Usd.Stage, path: str, x: float, y: float,
                     variants: list[tuple[str, str | None]],
                     foliage: bool = True, sector: int = 0) -> None:
        c = self._cfg
        UsdGeom.Xform.Define(stage, path)

        # 줄기
        stem = UsdGeom.Cylinder.Define(stage, path + "/Stem")
        r = c.stem_radius
        stem.CreateRadiusAttr(r)
        stem.CreateHeightAttr(c.stem_height)
        stem.CreateAxisAttr("Z")
        stem.CreateExtentAttr([Gf.Vec3f(-r, -r, -c.stem_height / 2.0),
                               Gf.Vec3f(r, r, c.stem_height / 2.0)])
        stem.CreateDisplayColorAttr([STEM_COLOR])
        UsdGeom.Xformable(stem.GetPrim()).AddTranslateOp().Set(
            Gf.Vec3d(x, y, c.stem_height / 2.0))
        physics.add_shape_collider(stem.GetPrim())

        # aoc 배경 식물(잎+가지) — 시각 전용. 원기둥 줄기(콜라이더)는 그대로 두고 위에 얹는다.
        # foliage=False 인 그루는 잎 없이 줄기만(레퍼런스처럼 덜 무성하게).
        if self._aoc_bg and foliage:
            self._spawn_background(stage, path + "/Foliage", x, y)

        # 과실
        if not variants:
            return
        n_fruits = self._rng.randint(*c.fruits_per_plant)
        for f in range(n_fruits):
            self._spawn_fruit(stage, path, f"{path}/Fruit_{f:02d}", x, y,
                              variants, f, n_fruits, sector)
        self._fruit_count += n_fruits

    def _spawn_background(self, stage: Usd.Stage, path: str,
                          x: float, y: float) -> None:
        """aoc 배경 식물 (잎+가지 메시). **시각 배경 전용** — 콜라이더도 강체도 없다.

        수확 대상은 obj 과실(위 _spawn_fruit)이다. 이 식물의 열매 메시는 배경 장식일
        뿐이라 물리를 안 붙인다 (aoc 식물은 통짜 메시라 열매만 떼어내기 어렵다 — §설계).
        무텍스처 회색이라 잎 색(초록)을 displayColor 로 넣어 루트 무광재질이 읽게 한다.
        """
        c = self._cfg
        rng = self._rng
        add_reference_to_stage(self._assets.background_plant_usd, path)
        # 참조 prim 은 이미 xformOp 를 갖고 있어 AddTranslateOp 가 터진다 → 재사용(§8).
        # 개체마다 랜덤 요(yaw)+크기 변주 — 132개가 똑같이 복붙된 느낌을 없애 자연스럽게.
        # 크기는 줄여(잎 덜 무성) 과실/트러스가 보이게 + 과실 구간으로 올림.
        q = Gf.Rotation(Gf.Vec3d(0, 0, 1), rng.uniform(0.0, 360.0)).GetQuat()
        set_pose(stage.GetPrimAtPath(path), (x, y, c.foliage_z),
                 Gf.Quatd(q.GetReal(), q.GetImaginary()))
        set_scale(stage.GetPrimAtPath(path), c.foliage_scale * rng.uniform(0.8, 1.2))
        # 변환된 USD 메시엔 흰색 기본 재질이 바인딩돼 있어 루트 무광재질을 덮는다.
        # 바인딩을 풀어 루트 재질을 상속받게 한 뒤 잎 색(초록)을 displayColor 로 준다.
        for prim in Usd.PrimRange(stage.GetPrimAtPath(path)):
            if prim.IsA(UsdGeom.Mesh):
                UsdShade.MaterialBindingAPI(prim).UnbindAllBindings()
        ripeness.apply_flat_color(stage, path, FOLIAGE_COLOR)

    def _spawn_fruit(self, stage: Usd.Stage, plant_path: str, path: str,
                     stem_x: float, stem_y: float,
                     variants: list[tuple[str, str | None]],
                     fi: int, nf: int, sector: int = 0) -> None:
        c = self._cfg
        rng = self._rng
        body_usd, calyx_usd = rng.choice(variants)
        names = list(c.class_weights)
        class_name = rng.choices(names, weights=[c.class_weights[n] for n in names])[0]

        # 화방: 줄기에서 옆으로 조금(pedicel_h_offset) + 아래로 매단다 (인장).
        # spike 02: 수평 캔틸레버는 굽힘모멘트가 break_torque(0.067N·m)를 넘겨 바로 끊긴다.
        # 같은 그루 과실은 줄기 둘레로 고르게 벌린다(겹침→침투복구 튕김 방지) + 약간 지터.
        angle = 2.0 * math.pi * fi / nf + rng.uniform(-0.3, 0.3)
        h = c.pedicel_h_offset
        drop = math.sqrt(max(c.fruit_offset ** 2 - h ** 2, 1e-6))
        fz = rng.uniform(*c.fruit_height_range)
        stem_pt = (stem_x, stem_y, fz + drop)      # 줄기 부착점 (위)
        pos = Gf.Vec3d(stem_x + h * math.cos(angle),
                       stem_y + h * math.sin(angle), fz)   # 과실 (아래)

        # 회전(요)은 넣지 않는다 — 과실 Xform 에 회전이 걸리면 pedicel.spawn 의 조인트
        # 프레임 계산이 어긋나 "disjointed body transforms" 로 스냅된다(spike 02 는 회전
        # 없이 검증됨). 토마토는 구형이라 요는 시각적으로도 거의 무의미.
        fruit = UsdGeom.Xform.Define(stage, path)
        xf = UsdGeom.Xformable(fruit.GetPrim())
        xf.AddTranslateOp().Set(pos)
        s = self._assets.scale
        xf.AddScaleOp().Set(Gf.Vec3f(s, s, s))

        add_reference_to_stage(body_usd, path + "/Body")
        ripeness.apply_ripeness_color(stage, path + "/Body", class_name, rng)
        if calyx_usd:
            add_reference_to_stage(calyx_usd, path + "/Calyx")
            ripeness.apply_flat_color(stage, path + "/Calyx", ripeness.GREEN)

        # 물리: 몸통 메시에만 콜라이더 (꼭지는 장식이라 제외 = 비용 절감).
        # A안: 과실은 dynamic — 꽃자루 파단 조인트가 매단다 (kinematic 아님).
        physics.add_mesh_colliders(stage, path + "/Body",
                                   self._phys.fruit_approximation)
        prim = stage.GetPrimAtPath(path)
        physics.add_rigid_body(prim, self._phys.fruit_density, kinematic=False)
        # sleep 비활성 — 과실이 매달려 가만히 있으면 PhysX 가 잠재우는데, 잠든 강체는
        # 조인트를 끊어도(pedicel.cut) 안 깨어나 안 떨어진다. 절단=낙하가 보장돼야 한다.
        PhysxSchema.PhysxRigidBodyAPI.Apply(prim).CreateSleepThresholdAttr(0.0)
        physics.bind_physics_material(prim, self._fruit_material)

        # 꽃자루 + 파단 조인트로 줄기에 매단다. 자를 땐 이 joint 를 pedicel.cut() 한다.
        # 시각 세그먼트는 plant_path(변환 없음) 밑에 둔다 (줄기·과실 변환에 안 딸리게).
        # 꽃자루는 과실 **꼭지(calyx)**에 붙는다 — 과실 중심이 아니라 꼭대기(+Z 반지름만큼).
        # 과실은 꼭지가 위를 향하게 스폰되므로 위로 올린 점이 꼭지 위치. (조인트 물리 앵커는
        # 과실 원점=중심 그대로 두어 안정적, 시각 꽃자루만 꼭지로 — pedicel.spawn 의 fruit_point
        # 은 세그먼트용이라 조인트 프레임엔 영향 없다.)
        calyx = (pos[0], pos[1], pos[2] + c.fruit_calyx_up)

        # 매달림엔 상향된 hold_force/hold_torque 를 쓴다 — 옆매달림 굽힘·스폰 겹침 충돌이
        # 실제 파단값(40.262N / 0.067N·m)을 넘겨 끊기기 때문(spike 02). settings 주석 참고.
        # 절단은 jointEnabled=False(pedicel.cut)로 하므로 이 값과 무관.
        joint = pedicel.spawn(stage, plant_path + "/Stem", path, stem_pt,
                              calyx, self._ped_cfg,
                              self._phys.pedicel_hold_force,
                              self._phys.pedicel_hold_torque,
                              viz_root=plant_path)

        self._fruits.append({
            "path": path,
            "class_name": class_name,
            "position": (pos[0], pos[1], pos[2]),
            "joint": joint,
            "sector": sector,          # 어느 재배 섹터(0..5) — 창고 슬롯 1:1 매핑에 씀
        })
