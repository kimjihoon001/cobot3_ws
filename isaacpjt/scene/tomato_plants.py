# -*- coding: utf-8 -*-
"""토마토 재배 라인 스폰 — 줄기 + 트렐리스 바 + 매달린 토마토.

- 과실은 tomato_assets_usd 의 모양 변형 중 랜덤 선택 (인스턴스별 참조)
- 익음 클래스(green/half_ripe/fully_ripe/old)를 가중치 랜덤 배정, 색은 ripeness 로 적용
- 줄기/트렐리스 = static collider (로봇이 통과 못 함)
- 과실 = dynamic RigidBody + 꽃자루 FixedJoint (매달림).
  수확 순간 jointEnabled=False 로 꽃자루만 끊는다.
"""
import math
import os
import random

from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdShade

from isaacsim.core.utils.stage import add_reference_to_stage

from pjt_config.settings import (GreenhouseConfig, PhysicsConfig, PlantConfig,
                             TomatoAssetConfig)
from scene import pedicel, physics
from pjt_utils import ripeness, textures
from pjt_utils.xform import set_pose, set_scale

# aoc 배경 식물 잎 색 (무텍스처 → displayColor 로 초록. 루트 무광재질이 읽는다).
FOLIAGE_COLOR = Gf.Vec3f(0.20, 0.42, 0.16)

# aoc 식물 파트별 원본 텍스처 (파일명, 알파컷아웃 필요 여부).
# dae→usd 변환기가 재질을 못 옮겨서 usd 안에는 흰색 DefaultMaterial 만 남았지만,
# prim 이름(Leaf1/Blossom2/Branch1…)과 UV(primvars:st)는 살아 있어 여기서 다시 잇는다.
# lef3=lef1, lef4=lef2 로 파일이 중복이라(md5 확인) 잎은 2종만 쓴다.
FOLIAGE_TEXTURES = {
    "Branch1":  ("AG15brn1.png", False),   # 줄기/가지 껍질. UV v 가 0~29 로 타일링됨
    "Leaf1":    ("AG15lef1.png", True),    # 잎 = 사각 판 + 알파. 컷아웃 없으면 초록 사각형
    "Leaf2":    ("AG15lef2.png", True),
    "Blossom1": ("AG15blo1.png", True),    # 꽃도 알파 카드
    "Blossom2": ("AG15blo2.png", True),
    "Blossom3": ("AG15blo3.png", True),
}

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

    def spawn(self, stage: Usd.Stage, root: str = "/World/Plants",
              elevation: float = 0.0) -> None:
        UsdGeom.Xform.Define(stage, root)
        UsdGeom.Xformable(stage.GetPrimAtPath(root)).AddTranslateOp().Set(
            Gf.Vec3d(0.0, 0.0, elevation))
        self._elevation = elevation
        # 무광 재질 — 없으면 RTX 기본 광택 재질이라 과실이 유리처럼 보인다.
        # displayColor(=클래스 색, YOLO 라벨 근거)는 그대로 읽는다.
        # 공통 루트 재질은 줄기·베드의 displayColor만 보조한다. 강한 바인딩이면
        # 하위 과실/잎의 빨강·초록 전용 재질까지 회색 fallback으로 덮을 수 있다.
        ripeness.bind_matte_material(
            stage, root, stronger_than_descendants=False)

        # aoc 배경 식물 옵션: 플래그 ON + 에셋 존재해야 켜진다 (없으면 조용히 건너뜀).
        self._aoc_bg = (self._cfg.use_aoc_background
                        and os.path.isfile(self._assets.background_plant_usd))
        if self._cfg.use_aoc_background and not self._aoc_bg:
            print(f"[WARN] 배경 식물 에셋 없음: {self._assets.background_plant_usd}"
                  " — 원기둥 줄기만 스폰")
        # 고화질 텍스처는 원본 png 폴더가 있어야 켜진다 (없으면 종전 단색으로).
        has_tex = os.path.isdir(self._assets.texture_dir)
        if (self._cfg.foliage_textured or self._cfg.fruit_textured) and not has_tex:
            print(f"[WARN] 텍스처 폴더 없음: {self._assets.texture_dir} — 단색으로 스폰")
        self._foliage_tex = self._cfg.foliage_textured and self._aoc_bg and has_tex
        self._fruit_tex = self._cfg.fruit_textured and has_tex
        # _measure_hq_scale 이 조기 반환(에셋 없음 등)해도 참조되므로 먼저 둔다.
        self._hq_calyx_up = None
        self._hq_scale = self._measure_hq_scale()
        self._hq_count = 0
        self._static_fruit_count = 0
        self._stem_tex = self._cfg.stem_textured and has_tex
        variants = self._find_usd_variants()
        self._fruit_material = physics.create_physics_material(
            stage, "/World/PhysicsMaterials/fruit",
            self._phys.fruit_static_friction, self._phys.fruit_dynamic_friction)

        c = self._cfg
        # 2x3 섹터 그리드 — 섹터 사이에 통로(주/교차)를 둬 로봇이 구역 간 이동 가능.
        col_xs = self._column_row_xs()      # 섹터 열별 이랑 x  [[x,x],[x,x]]
        seg_ys = self._segment_ys()         # 섹터 구획별 그루 y [[y..],[y..],[y..]]
        n_sectors = len(col_xs) * len(seg_ys)
        # 루프의 첫 베드는 min X, min Y인 Sector_00/Row_00이다. 이 시점의 RNG 상태를
        # 모든 베드 시작 전에 복원하면 식물 높이·회전과 과실 수·높이·방향·메쉬·숙도가
        # 첫 베드와 정확히 같은 상대 패턴으로 생성된다. 월드 x/y와 USD path만 달라진다.
        reference_bed_rng_state = self._rng.getstate()

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
                    self._rng.setstate(reference_bed_rng_state)
                    for pi in self._active_plant_indices(len(ys)):
                        y = ys[pi]
                        # 기준 베드의 local plant index만 사용해 모든 베드의 표시 여부도 동일.
                        foliage = ((pi * 17) % 100) < int(c.foliage_fraction * 100)
                        self._spawn_plant(
                            stage, f"{rp}/Plant_{pi:02d}", x, y,
                            variants, foliage, si)

        n_rows = c.sector_cols * c.rows_per_col
        plants_per_bed = len(self._active_plant_indices(c.plants_per_seg))
        n_plants = n_rows * c.sector_rows * plants_per_bed
        print("[Scene] %d섹터(2x3) · %d그루 · 과실 %d개 "
              "(통로 주%.1f/교차%.1f/수확%.1fm)"
              % (n_sectors, n_plants, self._fruit_count,
                 c.aisle_x, c.aisle_y, c.row_spacing))
        print("[Scene] 베드 복제 기준: Sector_00/Row_00 (min X, min Y) — "
              "식물·토마토 상대 패턴 전 베드 동일")
        if self._assets.physics_radius > 0.0:
            live = self._fruit_count - self._static_fruit_count
            print("[Scene] 물리 과실 %d개 (반경 %.1fm) / 시각 전용 %d개 — "
                  "강체·조인트 %d쌍을 솔버에서 제외"
                  % (live, self._assets.physics_radius,
                     self._static_fruit_count, self._static_fruit_count))
        if self._hq_scale is not None:
            cx, cy = self._assets.hq_center
            print("[Scene] 고화질 과실 %d개 (수확 반경 %.1fm @ (%.2f, %.2f)) "
                  "/ 저폴리 %d개" % (self._hq_count, self._assets.hq_radius,
                                    cx, cy, self._fruit_count - self._hq_count))

    def _measure_hq_scale(self) -> float | None:
        """HQ 과실 배율을 에셋 실측 bbox 에서 유도한다. 못 쓰면 None.

        USD 는 참조할 때 단위(metersPerUnit)도 업축(upAxis)도 자동 변환하지 않는다.
        이 usdz 는 metersPerUnit=0.01 / upAxis=Y 라 그냥 참조하면 2.6m 짜리 토마토가
        옆으로 누워 나온다. 배율은 여기서, 회전은 스폰 때 Body 에 건다.

        배율 = 목표 지름 / 원본 수평 지름. 매직넘버를 두지 않고 매번 재므로
        에셋을 바꿔도 자동으로 맞는다([2] 유도).
        """
        a = self._assets
        if not (a.hq_enabled and os.path.isfile(a.hq_usd)):
            if a.hq_enabled:
                print(f"[WARN] HQ 과실 에셋 없음: {a.hq_usd} — 저폴리 과실만 스폰")
            return None
        src = Usd.Stage.Open(a.hq_usd)
        body = next((p for p in src.Traverse()
                     if p.IsA(UsdGeom.Mesh) and "body" in p.GetName().lower()), None)
        if body is None:
            print(f"[WARN] HQ 에셋에서 body 메시를 못 찾음: {a.hq_usd}")
            return None
        # ⚠ 반드시 **합성된** bbox 로 재야 한다. 이 에셋은 메시 위에 scale=100 →
        # 0.00126 → 축스왑 → 100 변환 체인이 걸려 있어, 원시 points 로 재면 12.7배
        # 틀린다(2026-07-27 실측: 원시 2.66 vs 합성 33.68).
        rng_ = UsdGeom.BBoxCache(
            Usd.TimeCode.Default(), [UsdGeom.Tokens.default_]
        ).ComputeWorldBound(body).ComputeAlignedRange()
        if rng_.IsEmpty():
            print(f"[WARN] HQ 에셋 bbox 계산 실패: {a.hq_usd}")
            return None
        # ⚠ 그리고 **에셋 단위(metersPerUnit)로 실제 미터를 만들어야** 한다.
        # Isaac 의 metricsAssembler 가 참조 prim 에 Scale:unitsResolve=0.01 을 자동으로
        # 걸어 cm→m 을 이미 해준다. 그걸 모르고 에셋 단위를 그대로 미터로 보면 배율이
        # 100배 작아져 0.7mm 짜리 토마토가 된다(2026-07-27 실제로 겪음 — 수확 반경의
        # 과실이 통째로 안 보였다). aoc 배경 식물의 foliage_scale 도 같은 규약이다
        # (에셋 1.388m 기준 ×0.85).
        mpu = UsdGeom.GetStageMetersPerUnit(src)
        # 에셋이 Y-up 이므로 수평은 X·Z. 두 축 평균을 지름으로 본다(자연물이라 비대칭).
        size = rng_.GetSize()
        diam_m = (size[0] + size[2]) / 2.0 * mpu
        scale = a.hq_target_diameter_m / diam_m
        # 꽃자루를 **에셋 자체 꼭지 메시의 끝**에 붙이기 위해 그 높이를 재 둔다.
        # 저폴리용 고정값(fruit_calyx_up=33mm)을 그대로 쓰면 HQ 는 꼭지가 더 높아서
        # (실측 63mm) 꽃자루가 몸통 속에 박힌다.
        stem = next((p for p in src.Traverse()
                     if p.IsA(UsdGeom.Mesh) and "stem" in p.GetName().lower()), None)
        self._hq_calyx_up = None
        if stem is not None:
            top = UsdGeom.BBoxCache(
                Usd.TimeCode.Default(), [UsdGeom.Tokens.default_]
            ).ComputeWorldBound(stem).ComputeAlignedRange().GetMax()[1]  # Y-up
            self._hq_calyx_up = top * mpu * scale
        print(f"[Scene] HQ 과실: 원본 지름 {diam_m * 1000:.1f}mm → 배율 {scale:.4f} "
              f"(목표 {a.hq_target_diameter_m * 1000:.1f}mm), "
              f"꼭지 끝 {(self._hq_calyx_up or 0) * 1000:.1f}mm")
        return scale

    def _spawn_hq_body(self, stage: Usd.Stage, path: str) -> None:
        """고화질 과실(usdz)을 얹는다. **자체 PBR 재질을 그대로 살린다.**

        이 에셋은 baseColor/roughness/normal 텍스처가 완비돼 있다. 저폴리 과실처럼
        `bind_matte_material`/`bind_texture` 를 걸면 그 PBR 을 덮어버려 고화질을
        쓰는 의미가 없어진다 → 재질은 손대지 않는다.

        displayColor 는 **몸통 메시에만** 빨강으로 적는다. YOLO 데이터셋 생성이
        이 값을 읽어 단색 재질을 굽기 때문(SDG 라벨 근거). 줄기·잎까지 빨갛게
        칠하면 데이터셋 이미지가 이상해진다.
        """
        add_reference_to_stage(self._assets.hq_usd, path)
        prim = stage.GetPrimAtPath(path)
        # 업축(Y-up)은 **여기서 돌리지 않는다.** Isaac 의 metricsAssembler 가 참조
        # prim 에 Rotate:unitsResolve=90 을 자동으로 걸어 이미 세워 준다. 여기서 또
        # 90도를 주면 합쳐서 180도가 돼 거꾸로 선다(2026-07-27 실제로 겪음 — Property
        # 패널의 unitsResolve 항목을 보고 발견). 단위 변환도 마찬가지로 자동이다
        # (Scale:unitsResolve=0.01) — 배율 계산은 _measure_hq_scale 주석 참조.
        hide = []
        for m in Usd.PrimRange(prim):
            if m.IsA(UsdShade.Shader):
                self._soften_gloss(m)
                continue
            if not m.IsA(UsdGeom.Mesh):
                continue
            name = m.GetName().lower()
            if "leaves" in name and not self._assets.hq_leaves:
                hide.append(m)          # 순회 중 SetActive 금지 — 아래에서 처리
            elif "body" in name:
                ripeness.apply_flat_color(stage, str(m.GetPath()), ripeness.RED)
        for m in hide:
            UsdGeom.Imageable(m).MakeInvisible()

    def _soften_gloss(self, shader: Usd.Prim) -> None:
        """러프니스 텍스처 출력을 bias 만큼 끌어올려 광택을 낮춘다.

        에셋 원본은 러프니스가 0.176~0.373(평균 0.21)이라 젖은 플라스틱처럼 번들거린다.
        텍스처 연결을 끊고 상수로 덮으면 과피 굴곡이 통째로 사라지므로, UsdUVTexture 의
        `bias`(출력 = 텍스처 × scale + bias)만 올려 **굴곡은 살리고 값만** 옮긴다.
        """
        b = self._assets.hq_roughness_bias
        if b <= 0.0 or "roughness" not in shader.GetName().lower():
            return
        sh = UsdShade.Shader(shader)
        if sh.GetIdAttr().Get() != "UsdUVTexture":
            return
        sh.CreateInput("bias", Sdf.ValueTypeNames.Float4).Set(
            Gf.Vec4f(b, b, b, 0.0))

    def _use_physics(self, x: float, y: float) -> bool:
        """이 과실에 물리(강체·콜라이더·꽃자루 조인트)를 붙일 것인가.

        반경 밖은 시각 전용이라 PhysX 솔버에서 통째로 빠진다. 반경은 팔 도달
        범위(약 1m)보다 넉넉히 잡아, 로봇이 닿을 수 있는 과실은 전부 포함되게 한다.
        radius<=0 이면 종전처럼 전 과실에 물리를 붙인다.
        """
        r = self._assets.physics_radius
        if r <= 0.0:
            return True
        cx, cy = self._assets.hq_center
        return math.hypot(x - cx, y - cy) <= r

    def _use_hq(self, x: float, y: float) -> bool:
        """수확 정차 위치 반경 안인가. 전부 바꾸면 무거워서 이 구간만 HQ 로 쓴다."""
        if self._hq_scale is None:
            return False
        cx, cy = self._assets.hq_center
        return math.hypot(x - cx, y - cy) <= self._assets.hq_radius

    def _active_plant_indices(self, total: int) -> list[int]:
        """베드 전체 길이를 유지하며 지정 비율만 균등하게 선택한다."""
        if total <= 0:
            return []
        fraction = max(0.0, min(1.0, float(self._cfg.plant_spawn_fraction)))
        count = max(1, min(total, int(round(total * fraction))))
        if count == 1:
            return [total // 2]
        return [int(round(i * (total - 1) / (count - 1)))
                for i in range(count)]

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

    def _find_usd_variants(self) -> dict[str, list[tuple[str, str | None]]]:
        """클래스별 형상 목록: ripe=정상형, spoiled=손상/함몰형."""
        d = self._assets.usd_dir
        if not os.path.isdir(d):
            print(f"[WARN] 토마토 USD 폴더 없음: {d}")
            print("       isaac/tomatest/00_convert_obj_to_usd.py 를 먼저 실행하거나 "
                  "config/settings.py 의 usd_dir 을 수정하세요. 줄기만 스폰합니다.")
            return {}
        out = {"ripe": [], "spoiled": []}
        for f in sorted(os.listdir(d)):
            if f.endswith(".usd") and not f.endswith("_calyx.usd"):
                if f.startswith("tomato_ripe_"):
                    class_name = "ripe"
                elif f.startswith(("tomato_spoiled_", "tomato_dented_")):
                    class_name = "spoiled"
                else:
                    continue
                body = os.path.join(d, f)
                calyx = os.path.join(d, f[:-4] + "_calyx.usd")
                out[class_name].append(
                    (body, calyx if os.path.exists(calyx) else None))
        missing = [name for name, items in out.items() if not items]
        if missing:
            raise RuntimeError(f"토마토 형상 누락: {', '.join(missing)}")
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
                     variants: dict[str, list[tuple[str, str | None]]],
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
        if self._stem_tex:
            # 해석적 Cylinder 는 콜라이더·꽃자루 조인트 body 로 그대로 두고 **숨기기만**
            # 한다. 그 자리에 UV 를 가진 튜브 메시를 얹어 껍질 텍스처를 입힌다
            # (Cylinder 프리미티브엔 primvars:st 가 없어 텍스처를 못 문다).
            UsdGeom.Imageable(stem.GetPrim()).MakeInvisible()
            self._spawn_stem_skin(stage, path + "/StemSkin", x, y)

        # aoc 배경 식물(잎+가지) — 시각 전용. 원기둥 줄기(콜라이더)는 그대로 두고 위에 얹는다.
        # foliage=False 인 그루는 잎 없이 줄기만(레퍼런스처럼 덜 무성하게).
        if self._aoc_bg and foliage:
            self._spawn_background(
                stage, path + "/Foliage", x, y,
                self._rng.uniform(0.8, 1.2))

        # 과실
        if not variants:
            return
        n_fruits = self._rng.randint(*c.fruits_per_plant)
        for f in range(n_fruits):
            self._spawn_fruit(stage, path, f"{path}/Fruit_{f:02d}", x, y,
                              variants, f, n_fruits, sector)
        self._fruit_count += n_fruits

    def _spawn_stem_skin(self, stage: Usd.Stage, path: str,
                         x: float, y: float) -> None:
        """줄기 껍질 — UV 를 가진 튜브 메시. **시각 전용**(콜라이더·강체 없음).

        UV: u = 둘레 한 바퀴(0~1), v = 길이 / stem_tile_m. 껍질 텍스처가 세로로
        긴 스트립(80×1100)이라 이 방향이 맞다.
        """
        c = self._cfg
        n, r, h = c.stem_segments, c.stem_radius, c.stem_height
        pts, sts, nrm, counts, idx = [], [], [], [], []
        for i in range(n):
            a0 = 2.0 * math.pi * i / n
            a1 = 2.0 * math.pi * (i + 1) / n
            # u 는 i/n → (i+1)/n. 마지막 면은 1.0 으로 닫아 이음매를 없앤다.
            u0, u1 = i / n, (i + 1) / n
            v_top = h / c.stem_tile_m
            quad = ((a0, 0.0, u0, 0.0), (a1, 0.0, u1, 0.0),
                    (a1, h, u1, v_top), (a0, h, u0, v_top))
            base = len(pts)
            for ang, z, u, v in quad:
                pts.append(Gf.Vec3f(r * math.cos(ang), r * math.sin(ang), z))
                nrm.append(Gf.Vec3f(math.cos(ang), math.sin(ang), 0.0))
                sts.append(Gf.Vec2f(u, v))
            counts.append(4)
            idx.extend([base, base + 1, base + 2, base + 3])
        mesh = UsdGeom.Mesh.Define(stage, path)
        mesh.CreatePointsAttr(pts)
        mesh.CreateFaceVertexCountsAttr(counts)
        mesh.CreateFaceVertexIndicesAttr(idx)
        mesh.CreateNormalsAttr(nrm)
        mesh.SetNormalsInterpolation(UsdGeom.Tokens.faceVarying)
        mesh.CreateExtentAttr([Gf.Vec3f(-r, -r, 0.0), Gf.Vec3f(r, r, h)])
        UsdGeom.PrimvarsAPI(mesh.GetPrim()).CreatePrimvar(
            "st", Sdf.ValueTypeNames.TexCoord2fArray,
            UsdGeom.Tokens.faceVarying).Set(sts)
        UsdGeom.Xformable(mesh.GetPrim()).AddTranslateOp().Set(Gf.Vec3d(x, y, 0.0))
        textures.bind_texture(
            stage, path, "/World/Looks/StemBark",
            os.path.join(self._assets.texture_dir, "AG15brn1.png"),
            roughness=0.75)          # 껍질은 무광

    def _spawn_background(self, stage: Usd.Stage, path: str,
                          x: float, y: float, height_scale: float) -> None:
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
        set_scale(stage.GetPrimAtPath(path), c.foliage_scale * height_scale)
        # 변환된 USD 메시엔 흰색 기본 재질이 바인딩돼 있어 루트 재질을 덮는다 → 먼저 뗀다.
        for prim in Usd.PrimRange(stage.GetPrimAtPath(path)):
            if prim.IsA(UsdGeom.Mesh):
                UsdShade.MaterialBindingAPI(prim).UnbindAllBindings()
        # displayColor(초록)는 텍스처를 쓰든 안 쓰든 항상 남긴다 — YOLO 데이터셋
        # 생성(03_generate_from_scene.colorize_subtree)이 이 값을 읽어 데이터셋용
        # 단색 재질을 굽는다. 텍스처는 화면에 보이는 재질만 갈아끼운다.
        ripeness.apply_flat_color(stage, path, FOLIAGE_COLOR)
        if self._foliage_tex:
            self._bind_foliage_textures(stage, path)
            return
        # 참조 메시가 아직 비동기 로딩 중이어도 루트 상속 재질의 fallback이 초록이므로
        # 회색 프레임이 나타나지 않는다. 로딩된 메시에는 아래에서 한 번 더 직접 바인딩한다.
        ripeness.bind_matte_material(
            stage, path,
            mat_path="/World/Looks/MatteFoliage",
            fallback_color=FOLIAGE_COLOR)
        # reference 180개가 재구성될 때 루트의 상속 바인딩이 간헐적으로 회색 fallback으로
        # 남으므로, 각 mesh에 무광 재질을 직접 바인딩한다.
        for prim in Usd.PrimRange(stage.GetPrimAtPath(path)):
            if prim.IsA(UsdGeom.Mesh):
                ripeness.bind_matte_material(
                    stage, str(prim.GetPath()),
                    mat_path="/World/Looks/MatteFoliage",
                    fallback_color=FOLIAGE_COLOR)

    def _bind_foliage_textures(self, stage: Usd.Stage, path: str) -> None:
        """잎/꽃/줄기에 파트별 사진 텍스처를 바인딩.

        메시 경로는 `<Foliage>/<파트>/<메시>` 구조라 **부모 prim 이름**이 파트를 정한다.
        재질은 파트당 1개를 만들어 전 그루(180개)가 공유한다 — 인스턴스마다 만들면
        RTX 셰이더 컴파일이 폭발한다.

        루트에 상속 바인딩을 걸지 않는 이유: `bind_matte_material` 의 기본값인
        strongerThanDescendants 가 아래 메시별 텍스처 바인딩을 이겨서 전부 초록
        단색으로 덮어버린다.
        """
        hide = []
        for prim in Usd.PrimRange(stage.GetPrimAtPath(path)):
            if not prim.IsA(UsdGeom.Mesh):
                continue
            part = prim.GetParent().GetName()
            # 큰 잎 메시(Leaf2 = 전체 잎의 81%)를 숨겨 캐노피를 성기게 만든다.
            # 과실이 잎 사이로 보이게 하는 방법 — 캐노피를 올리거나 줄이면 줄기가
            # 장대처럼 드러난다(2026-07-27 시행착오).
            # ⚠ 순회 중에 prim 을 지우면(SetActive) PrimRange 가 깨져 Isaac 이 씬
            #   빌드 전에 죽는다(실제로 겪음). 목록만 모아 두고 순회가 끝난 뒤 처리한다.
            #   지우지 않고 visibility 로만 감춘다 — 물리·수확에 영향 없는 시각 처리.
            if part == "Leaf2" and not self._cfg.foliage_big_leaves:
                hide.append(prim)
                continue
            entry = FOLIAGE_TEXTURES.get(part)
            if entry is None:
                continue
            fname, cutout = entry
            textures.bind_texture(
                stage, str(prim.GetPath()),
                "/World/Looks/AocFoliage/" + part,
                os.path.join(self._assets.texture_dir, fname),
                cutout=cutout)
        # 순회가 끝난 뒤에 감춘다 (위 ⚠ 참조).
        for prim in hide:
            UsdGeom.Imageable(prim).MakeInvisible()

    def _spawn_fruit(self, stage: Usd.Stage, plant_path: str, path: str,
                     stem_x: float, stem_y: float,
                     variants: list[tuple[str, str | None]],
                     fi: int, nf: int, sector: int = 0) -> None:
        c = self._cfg
        rng = self._rng
        # 현재 통합 수확시험은 품질 분류를 하지 않는다. 모든 과실을 수확 대상
        # ripe 형상/라벨/빨간색으로 고정해 회색 fallback과 숙도별 색 차이를 없앤다.
        class_name = "ripe"
        body_usd, calyx_usd = rng.choice(variants["ripe"])

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
        # 수확 정차 위치 반경 안에서만 고화질 과실을 쓴다(전부 바꾸면 1300만면).
        hq = self._use_hq(stem_x, stem_y)
        fruit = UsdGeom.Xform.Define(stage, path)
        fruit.GetPrim().SetCustomDataByKey("class_name", class_name)
        fruit.GetPrim().SetCustomDataByKey(
            "shape_asset",
            os.path.basename(self._assets.hq_usd if hq else body_usd))
        xf = UsdGeom.Xformable(fruit.GetPrim())
        xf.AddTranslateOp().Set(pos)
        s = self._hq_scale if hq else self._assets.scale
        xf.AddScaleOp().Set(Gf.Vec3f(s, s, s))

        if hq:
            self._spawn_hq_body(stage, path + "/Body")
            self._hq_count += 1
        else:
            add_reference_to_stage(body_usd, path + "/Body")
            # displayColor(빨강)는 텍스처를 쓸 때도 남긴다 — SDG 라벨 근거(red_fraction).
            ripeness.apply_flat_color(stage, path + "/Body", ripeness.RED)
            if self._fruit_tex:
                # 과실 usd 는 points/normals 만 있고 UV 가 없다 → 구면 투영으로 만들어
                # 과피 사진을 입힌다. 과실마다 4종 중 랜덤이라 전부 같아 보이지 않는다.
                textures.add_spherical_uv(stage, path + "/Body")
                skin = rng.choice(self._assets.skin_textures)
                textures.bind_texture(
                    stage, path + "/Body",
                    "/World/Looks/TomatoSkin/" + os.path.splitext(skin)[0],
                    os.path.join(self._assets.texture_dir, skin))
            else:
                ripeness.bind_matte_material(
                    stage, path + "/Body",
                    mat_path="/World/Looks/MatteFruitRipe",
                    fallback_color=ripeness.RED)
        if calyx_usd and not hq:   # HQ 에셋은 꼭지(stem)를 자체 메시로 갖고 있다
            add_reference_to_stage(calyx_usd, path + "/Calyx")
            ripeness.apply_flat_color(stage, path + "/Calyx", ripeness.GREEN)
            ripeness.bind_matte_material(
                stage, path + "/Calyx",
                mat_path="/World/Looks/MatteCalyx",
                fallback_color=ripeness.GREEN)

        # 물리: 몸통 메시에만 콜라이더 (꼭지는 장식이라 제외 = 비용 절감).
        # 과실은 처음부터 dynamic이고 꽃자루 FixedJoint가 매단다. 예전 kinematic 방식은
        # 집게가 닫히며 쌓인 침투/압축 임펄스가 절단 순간 dynamic 전환과 함께 풀려 과실을
        # 사출했다. 동역학 모드를 바꾸지 않고 조인트만 끊어 이 불연속을 제거한다.
        # 충돌은 안정적인 해석적 구를 쓰되 반지름을 시각 몸통 표면과 맞춘다. 보이는
        # 토마토보다 작은 구는 손가락이 메시를 관통한 뒤에야 접촉하는 빈손 파지를 만든다.
        # 반지름은 월드 m 를 과실 스케일로 나눠 로컬 단위로 지정한다.
        # HQ 과실은 배율이 달라서(에셋 원본 크기가 다름) 반드시 이 과실의 s 로 나눈다.
        # self._assets.scale 로 나누면 HQ 만 콜라이더가 17배로 커진다.
        # 반경 밖 과실은 **시각 전용**이다 — 강체·콜라이더·조인트를 아예 안 붙인다.
        # 540개 전부에 dynamic 강체 + FixedJoint + sleep 비활성을 걸면 PhysX 가 매 스텝
        # 540개를 전부 푼다(잠들지 않으므로). 실제로 수확하는 건 수확 정차 위치 주변
        # 몇 개뿐이고 나머지는 배경이라 매달려만 있으면 된다. (2026-07-27 — 시뮬이
        # 실시간의 0.19배까지 떨어져 기동 타임아웃이 연쇄로 터진 뒤 도입)
        calyx_up = (self._hq_calyx_up if (hq and self._hq_calyx_up)
                    else c.fruit_calyx_up)
        live = self._use_physics(stem_x, stem_y)
        if not live:
            self._static_fruit_count += 1
            pedicel.spawn(stage, plant_path + "/Stem", path, stem_pt,
                          (pos[0], pos[1], pos[2] + calyx_up),
                          self._ped_cfg, self._phys.pedicel_hold_force,
                          self._phys.pedicel_hold_torque,
                          viz_root=plant_path, make_joint=False)
            self._fruits.append({
                "id": len(self._fruits),
                "path": path,
                "class_name": class_name,
                "position": (pos[0], pos[1], pos[2] + self._elevation),
                "joint": "",          # 조인트 없음 = 수확 대상 아님
                "sector": sector,
                "static": True,
            })
            return
        physics.add_sphere_collider(
            stage, path + "/Collision",
            self._phys.fruit_collision_radius_m / s)
        # 충돌구를 **시각 몸통 중심**으로 옮긴다 — 토마토 USD 원점이 몸통 중심과 안 맞아
        # (미centering) 원점에 두면 겨냥점(sim/tomato 발행=bbox 중심)과 어긋나 그리퍼가
        # 손가락 사이에 과실 없이 완전히 닫힌다(2026-07-23 빈손 파지 원인). bbox 유효할 때만.
        _rng = UsdGeom.BBoxCache(
            Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render]
        ).ComputeWorldBound(stage.GetPrimAtPath(path + "/Body")).ComputeAlignedRange()
        if not _rng.IsEmpty():
            _cl = UsdGeom.XformCache().GetLocalToWorldTransform(
                stage.GetPrimAtPath(path)).GetInverse().Transform(
                    Gf.Vec3d(_rng.GetMidpoint()))
            UsdGeom.Xformable(
                stage.GetPrimAtPath(path + "/Collision")).AddTranslateOp().Set(
                    Gf.Vec3d(_cl))
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
        # calyx_up 은 위에서 구했다 — HQ 는 꼭지가 자체 메시라 높이가 다르다
        # (실측 63mm vs 저폴리 33mm). 고정값을 쓰면 꽃자루가 몸통 속에 박힌다.
        calyx = (pos[0], pos[1], pos[2] + calyx_up)

        # dynamic 과실을 static 줄기에 **비파단** FixedJoint로 매단다. 팔 kp=1e5에서는
        # 2cm 접촉 오차만으로도 기존 hold_force=2000N 수준에 도달해 접촉 순간 과실이
        # 떨어졌다. 실제 수확은 커터가 jointEnabled=False로만 결정적으로 절단한다.
        joint = pedicel.spawn(stage, plant_path + "/Stem", path, stem_pt,
                              calyx, self._ped_cfg,
                              self._phys.pedicel_hold_force,
                              self._phys.pedicel_hold_torque,
                              viz_root=plant_path, make_joint=True,
                              breakable=False)

        self._fruits.append({
            "id": len(self._fruits),
            "path": path,
            "class_name": class_name,
            # GroundTruth 좌표는 월드 좌표여야 한다. Plants 루트 상승분도 포함한다.
            "position": (pos[0], pos[1], pos[2] + self._elevation),
            "joint": joint,
            "sector": sector,          # 어느 재배 섹터(0..5) — 창고 슬롯 1:1 매핑에 씀
        })
