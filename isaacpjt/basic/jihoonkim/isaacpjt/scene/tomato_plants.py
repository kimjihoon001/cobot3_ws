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

from pxr import Usd, UsdGeom, UsdShade, Gf

from isaacsim.core.utils.stage import add_reference_to_stage

from pjt_config.settings import (GreenhouseConfig, PhysicsConfig, PlantConfig,
                             TomatoAssetConfig)
from scene import physics
from pjt_utils import ripeness
from pjt_utils.xform import set_translate

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

        c, g = self._cfg, self._greenhouse
        xs = _positions(g.width - 2.0 * c.margin, c.row_spacing)
        ys = _positions(g.length - 2.0 * c.margin, c.plant_spacing)
        row_span = (ys[-1] - ys[0]) if len(ys) > 1 else 0.0

        for r, x in enumerate(xs):
            row_path = f"{root}/Row_{r:02d}"
            UsdGeom.Xform.Define(stage, row_path)
            self._spawn_bed(stage, row_path + "/Bed", x, row_span)
            self._spawn_trellis_bar(stage, row_path + "/Trellis", x, row_span)
            for p, y in enumerate(ys):
                self._spawn_plant(stage, f"{row_path}/Plant_{p:02d}", x, y, variants)

        print("[Scene] %d줄 x %d그루 = %d그루 (조간 %.2fm, 주간 %.2fm), 과실 %d개"
              % (len(xs), len(ys), len(xs) * len(ys),
                 c.row_spacing, c.plant_spacing, self._fruit_count))

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
                   x: float, length: float) -> None:
        """재배 베드 — 배지경 양액재배의 배지백 라인 (흰 상자).

        시각 + 충돌. 로봇 베이스가 이랑을 가로지르지 못하게 막는 역할도 한다
        (실제 온실에서도 베드가 통로를 규정한다).
        """
        bed = UsdGeom.Cube.Define(stage, path)
        bed.CreateSizeAttr(1.0)
        bed.CreateDisplayColorAttr([Gf.Vec3f(0.90, 0.90, 0.87)])
        xf = UsdGeom.Xformable(bed.GetPrim())
        xf.AddTranslateOp().Set(Gf.Vec3d(x, 0.0, 0.10))
        xf.AddScaleOp().Set(Gf.Vec3f(0.42, length + 0.5, 0.20))
        physics.add_shape_collider(bed.GetPrim())

    def _spawn_trellis_bar(self, stage: Usd.Stage, path: str,
                           x: float, length: float) -> None:
        """줄기 상단을 잇는 수평 지지대."""
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
            Gf.Vec3d(x, 0.0, c.stem_height))
        physics.add_shape_collider(bar.GetPrim())

    def _spawn_plant(self, stage: Usd.Stage, path: str, x: float, y: float,
                     variants: list[tuple[str, str | None]]) -> None:
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
        if self._aoc_bg:
            self._spawn_background(stage, path + "/Foliage", x, y)

        # 과실
        if not variants:
            return
        n_fruits = self._rng.randint(*c.fruits_per_plant)
        for f in range(n_fruits):
            self._spawn_fruit(stage, f"{path}/Fruit_{f:02d}", x, y, variants)
        self._fruit_count += n_fruits

    def _spawn_background(self, stage: Usd.Stage, path: str,
                          x: float, y: float) -> None:
        """aoc 배경 식물 (잎+가지 메시). **시각 배경 전용** — 콜라이더도 강체도 없다.

        수확 대상은 obj 과실(위 _spawn_fruit)이다. 이 식물의 열매 메시는 배경 장식일
        뿐이라 물리를 안 붙인다 (aoc 식물은 통짜 메시라 열매만 떼어내기 어렵다 — §설계).
        무텍스처 회색이라 잎 색(초록)을 displayColor 로 넣어 루트 무광재질이 읽게 한다.
        """
        add_reference_to_stage(self._assets.background_plant_usd, path)
        # 참조 prim 은 이미 xformOp 를 갖고 있어 AddTranslateOp 가 터진다 → 재사용(§8)
        set_translate(stage.GetPrimAtPath(path), (x, y, 0.0))
        # 변환된 USD 메시엔 흰색 기본 재질이 바인딩돼 있어 루트 무광재질을 덮는다.
        # 바인딩을 풀어 루트 재질을 상속받게 한 뒤 잎 색(초록)을 displayColor 로 준다.
        for prim in Usd.PrimRange(stage.GetPrimAtPath(path)):
            if prim.IsA(UsdGeom.Mesh):
                UsdShade.MaterialBindingAPI(prim).UnbindAllBindings()
        ripeness.apply_flat_color(stage, path, FOLIAGE_COLOR)

    def _spawn_fruit(self, stage: Usd.Stage, path: str, stem_x: float,
                     stem_y: float, variants: list[tuple[str, str | None]]) -> None:
        c = self._cfg
        rng = self._rng
        body_usd, calyx_usd = rng.choice(variants)
        names = list(c.class_weights)
        class_name = rng.choices(names, weights=[c.class_weights[n] for n in names])[0]

        angle = rng.uniform(0.0, 2.0 * math.pi)
        pos = Gf.Vec3d(stem_x + c.fruit_offset * math.cos(angle),
                       stem_y + c.fruit_offset * math.sin(angle),
                       rng.uniform(*c.fruit_height_range))

        fruit = UsdGeom.Xform.Define(stage, path)
        xf = UsdGeom.Xformable(fruit.GetPrim())
        xf.AddTranslateOp().Set(pos)
        xf.AddRotateZOp().Set(rng.uniform(0.0, 360.0))
        s = self._assets.scale
        xf.AddScaleOp().Set(Gf.Vec3f(s, s, s))

        add_reference_to_stage(body_usd, path + "/Body")
        ripeness.apply_ripeness_color(stage, path + "/Body", class_name, rng)
        if calyx_usd:
            add_reference_to_stage(calyx_usd, path + "/Calyx")
            ripeness.apply_flat_color(stage, path + "/Calyx", ripeness.GREEN)

        # 물리: 몸통 메시에만 콜라이더 (꼭지는 장식이라 제외 = 비용 절감).
        # RigidBody 는 과실 Xform 에 붙여 하위 콜라이더를 하나의 강체로 묶는다.
        physics.add_mesh_colliders(stage, path + "/Body",
                                   self._phys.fruit_approximation)
        prim = stage.GetPrimAtPath(path)
        physics.add_rigid_body(prim, self._phys.fruit_density, kinematic=True)
        physics.bind_physics_material(prim, self._fruit_material)

        self._fruits.append({
            "path": path,
            "class_name": class_name,
            "position": (pos[0], pos[1], pos[2]),
        })
