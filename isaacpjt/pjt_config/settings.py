# -*- coding: utf-8 -*-
"""씬 전역 설정 — 경로/치수/개수는 전부 여기서만 수정.

이 프로젝트는 평가에서 **각 값에 출처**를 요구한다. 값마다 아래 등급을 주석에 밝힌다.
등급이 낮을수록 나쁜 게 아니라, **낮은 등급을 높은 등급으로 옮기는 게 할 일**이다.

  [1] 출처       — 논문/규격에서 온 값. 인용을 단다.
                   예: break_force 40.262N = [W2024] Table 2
  [2] 유도       — 다른 확정값에서 계산된 값. 자유도가 없으므로 따로 정당화할 게 없다.
                   예: 커터 오프셋 하한 = 과실 반지름(실측 34.4mm)
  [3] 민감도     — 출처를 못 찾았지만, 스윕해서 "이 범위면 성립" 을 보인 값.
                   → 그 범위가 곧 **하드웨어 요구사항**이 된다. 값 하나를 근거 없이
                     대는 것보다 오히려 단단하다.
                   예: 마찰계수 μ — 토마토-그리퍼 마찰을 잰 논문이 없어 0.3~1.1 스윕
  [4] 임의       — 아직 아무 근거 없음. `TODO 근거 없음` 을 달고, [3] 으로 옮길 계획을 적는다.

**[4] 를 그럴듯한 [1] 로 위장하지 말 것.** 오늘 한 번 그랬다가 정정했다 —
과피 인장 물성(Matas 2004)을 마찰 근거로 댔는데, 인장은 마찰이 아니다.
"출처 없음" 은 답이 되지만 "틀린 출처" 는 안 된다.
"""
import os
from dataclasses import dataclass, field

ISAAC_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


@dataclass
class TomatoAssetConfig:
    """토마토 USD 에셋 (00_convert_obj_to_usd.py 출력물)."""
    # (2026-07-19 팀 트리 정리: tomatest/tomato_assets_usd → assets/tomato)
    usd_dir: str = os.path.join(ISAAC_DIR, "assets", "tomato")

    # FreeCAD mm -> m 변환(0.001)에 크기 배율 1.675 를 곱한 값.
    #
    # 메시 원본은 generate_tomatoes.py 의 BASE_R = 20mm, 즉 기준 지름 40mm 이고
    # 스크립트 주석도 "Isaac 에서 스케일 조정 가능" 이라고 명시한다.
    # 1.675 를 곱하면 잘 익은 과실이 지름 68.7mm / 부피 120cm^3 이 된다
    # (fruit_density 1000 과 곱해 120g). 대과 완숙토마토 기준.
    #
    # 밀도가 아니라 이 배율이 과실 크기를 정한다. 크기를 바꾸면 질량도 같이 바뀐다.
    #
    # [2] 유도 — obj 를 직접 재서(부호있는 사면체 부피) 나온 값이라 자유도가 없다.
    #     다만 **목표인 68.7mm/120g 자체가 [4] 임의**다. 품종 규격(예: [W2024] 의
    #     Syngenta Spectrum)이나 국내 대과종 규격에서 지름을 확보하면 [1] 이 된다.
    scale: float = 0.001675

    # 배경 식물(잎+가지) — aoc_tomato_farm(Apache-2.0) tomato.dae 변환물.
    # **시각 배경 전용** — 콜라이더·강체 없음. 실제 수확 대상은 위 usd_dir 의 obj 과실.
    # 원본 dae 는 upAxis 를 Y 로 잘못 스탬프해서 00_convert 처럼 Z 로 재스탬프함
    #   (그대로 두면 Z-up 씬에서 눕는다 — RESULTS.md 2026-07-18).
    # 실측: 1.388m 직립, 27k삼각형/그루, 132그루 212fps (폴리곤 여유 충분).
    background_plant_usd: str = os.path.join(
        ISAAC_DIR, "assets", "aoc", "usd", "tomato_plant.usd")


@dataclass
class LightingConfig:
    # 기본 지면(add_default_ground_plane)에 딸려오는 조명과 합쳐지면 씬이 하얗게 날아간다.
    # 2026-07-20: 창고 확대로 흰 벽/바닥이 늘어 Stage Lights 뷰포트에서 과노출(식물이 하얀
    # 덩어리로 탐) → 세기 대폭 낮춤(돔 350→100, 태양 1500→450). 여전히 밝으면 더 낮추고
    # 너무 어두우면 올릴 것. 시각값이라 근거 등급 없음.
    dome_intensity: float = 100.0
    sun_intensity: float = 450.0
    sun_rotation_xyz: tuple[float, float, float] = (-50.0, 0.0, 30.0)


@dataclass
class GreenhouseConfig:
    """온실 프레임 치수 (m). x=폭(좌우), y=길이(앞뒤, 창고 방향), z=높이.

    2x3 섹터 + 통로를 담게 넓혔다(2026-07-18). 크기는 PlantConfig 섹터 레이아웃에서
    유도되는 값 — 재배 구역(2열×3구획) + 통로 + 마진을 감싸야 한다.

    2026-07-19 팀 피드백으로 전체 확대(화면상 긴 축=가로 기준):
      세로(좌우폭 X) = 기존 10 × 1.5 = 15,
      가로(앞뒤길이 Y) = 기존 20 × 1.3 = 26,
      높이 = 기존 3 × 1.5 = 4.5.
    통로/마진도 같이 넓혔다(PlantConfig.aisle_x/aisle_y/margin). [4] 임의 — 배율은 시연 요구.
    """
    # 좌우폭(X) = 재배구역 폭 + 좌우 벽여백 3.0m(=MM 3대) 양쪽에서 유도.
    #   2026-07-20 X 간격 균일화(row_spacing=aisle_x=2.9)로 4이랑이 넓게 퍼져(bed edge ±4.56)
    #   폭이 13→15.1 로 늘었다(= 2×(4.56+3.0)). (균일화 전엔 13.) [4] 임의(시연).
    width: float = 15.1
    length: float = 26.0           # 앞뒤길이(Y, 창고 방향). "가로" ×1.3
    height: float = 4.5            # ×1.5
    post_spacing: float = 3.0      # 기둥 간격 (길이 방향)
    frame_size: float = 0.08       # 기둥/보 두께
    frame_color: tuple[float, float, float] = (0.75, 0.78, 0.80)


@dataclass
class PlantConfig:
    """토마토 재배 라인 배치.

    재식거리 근거 — 스마트팜은 배지경 양액재배 방식이므로 그 규격을 따른다.
      제주 농업기술원 「과채류(토마토)」 정식거리:
        "배지경 양액재배시는 150×25~30cm 로 1열로 심는 것이 통풍채광이나
         작업이 편리하다"
      교차검증: TAMU / UMass 의 protected culture high-wire 조간 1.5~1.8m
      (참고: 토경 보통재배는 90~100 × 40~50cm 로 값이 다르다. 스마트팜에는
       해당 없음.)

    주수(그루 수)는 설정값이 아니다. 온실 크기와 재식거리에서 유도된다.
    """
    # 조간 (m) = 섹터 내 이랑 사이. 2026-07-20 사용자 요청으로 X 간격을 전부 균일하게:
    #   row_spacing = aisle_x = 2.9 (= MM 2.5대 통과폭, 실통로 2.48m). 4이랑이 2.9m 등간격.
    #   ⚠ 원래 1.8m 은 하이와이어 조간 1.5~1.8m(제주농업기술원/UMass) [1] 출처였으나,
    #     균일·로봇통행 우선으로 2.9 로 올리며 그 농업 출처는 더 이상 적용 안 됨. 지금은
    #     로봇 풋프린트 유도값(aisle_x 와 동일 근거). 출처 복원하려면 1.8 로 되돌릴 것.
    row_spacing: float = 2.9
    plant_spacing: float = 0.3     # 주간 (m). 25~30cm 중 상한
    # 온실 벽에서 재배 구역(bed edge)까지 = 벽쪽 통로. 팀 피드백①(2026-07-20):
    #   MM 3대 지나갈 폭 남기기 → 3.0m (MM 1대 통과폭 ≈ 1.0m 기준). 섹터를 벽에 붙임.
    margin: float = 3.0

    # 2x3 섹터 그리드 (v3: 재배 6섹터 : 창고 6슬롯 1:1). 한 덩어리로 두면 로봇이 이랑
    # 사이만 다니고 구역 간 이동을 못 한다 → 섹터로 나누고 통로(교차로)를 넣어 주행 가능하게.
    sector_cols: int = 2           # 섹터 열 (X 방향)
    sector_rows: int = 3           # 섹터 행 (Y 구획) → cols*rows = 6섹터
    rows_per_col: int = 2          # 각 섹터의 식물행(이랑) 수
    plants_per_seg: int = 15       # 각 섹터의 그루 수 (Y 방향)
    # aisle_x = 2열 섹터 사이 중앙 주통로(블록 사이). 팀 피드백①(2026-07-20):
    #   MM 2.5대 지나갈 폭(≈2.5m 클리어) → 베드(0.42m) 감안 중심간격 2.9m.
    #   iw.hub(1.43m)·1.5×(2.15m) 하한도 만족(클리어 2.48m). 값 = 클리어 + 베드폭.
    # aisle_y = 3구획 사이 교차 통로(Y). 이번①은 X만 조정 → 유지.
    aisle_x: float = 2.9           # 중앙 주통로 중심간격 (클리어 ≈ 2.5m = MM 2.5대)
    aisle_y: float = 3.5           # 섹터 행 사이 교차 통로(Y, 유지). ≥2.15m

    # 수확 구간 높이 근거 — 스마트팜 토마토는 하이와이어 + lower&lean(낮추기) 방식.
    # 줄기를 계속 낮춰서 수확 화방을 일정한 높이에 유지한다.
    #   "The ideal space for the harvest zone is a level even with the
    #    knee-to-shoulder range of the person who will be doing the harvesting"
    #   "plants are continuously lowered, the fruit remains at a convenient height"
    #   -> 즉 과실 높이는 식물이 자란 높이가 아니라 '사람 인간공학'이 정한다.
    # 한국인 성인 무릎~어깨 높이로 환산하면 약 0.45~1.45m
    #   (제8차 한국인 인체치수조사: 평균 신장 남 172.5cm / 여 159.6cm)
    # TODO 사이즈코리아(sizekorea.kr)에서 무릎높이/어깨높이 실측값을 직접 뽑을 것.
    #      지금 값은 신장 비례 추정이라 한 단계 약하다.
    fruit_height_range: tuple[float, float] = (0.5, 1.4)

    # TODO 아래 2개는 아직 근거 없음.
    #   stem_height: 실제 하이와이어는 와이어가 베드 위 2.4m 이상이고 줄기는 그보다
    #     훨씬 길다(낮추기 때문). 1.8m 는 데모용 단순화. 근거 아님.
    stem_height: float = 1.8
    fruits_per_plant: tuple[int, int] = (2, 4)   # (min, max)

    stem_radius: float = 0.02
    # 꽃자루(pedicel) 전체 길이 = 줄기 부착점 → 과실 중심. [4] 임의.
    # spike 02(2026-07-18): 과실을 수평으로 매달면 굽힘모멘트가 break_torque(0.067N·m)를
    # 넘어 바로 끊긴다 → 아래로 인장 매달아야 실제 파단값으로 버틴다. 그래서 아래 두 값으로
    # "옆으로 조금 + 아래로" 매단다(수직 낙차 = sqrt(fruit_offset^2 - h^2)).
    fruit_offset: float = 0.11     # m. 꽃자루 길이
    # 수평 성분(줄기에서 옆으로). 나머지는 수직 낙차 → 인장.
    # [2] 유도(하한): 과실 콜라이더(r≈34mm)+줄기(r=20mm)가 안 겹치려면 > 54mm.
    #     하한 근처(60mm)면 스폰 시 이웃/줄기와 살짝 겹쳐 5%가 침투복구로 튕겨 낙하 →
    #     여유를 줘 90mm(클리어런스 36mm). spike 02: 굽힘은 hold_torque 로 처리하므로 무관.
    pedicel_h_offset: float = 0.09  # m

    # 꽃자루가 붙는 과실 꼭지(calyx) 위치 = 과실 중심에서 위로(+Z) 반지름만큼.
    # 실제 토마토는 꼭지에서 화방으로 이어진다(레퍼런스). [2] 유도 — 과실 반지름 ≈ 34mm.
    fruit_calyx_up: float = 0.033  # m

    # aoc 배경 식물(잎+가지 메시)을 원기둥 줄기 위에 얹을지. 시각 전용(콜라이더·강체 없음).
    # 켜도 원기둥 줄기(콜라이더)와 obj 과실(수확대상)·꽃자루 조인트는 그대로 유지된다.
    # 기본 True — main.py 도 잎을 얹어 자연스럽게 보이게(사용자 요청 2026-07-18). 물리엔 무관.
    # 성능이 급하면(로봇+브리지 붙은 무거운 런) False 로 끄면 폴리곤 부담이 준다. 에셋 없으면 자동 스킵.
    use_aoc_background: bool = True
    # 잎이 너무 많고 낮다는 피드백(2026-07-18, 레퍼런스 대비) → 덜 무성하게:
    #   일부 그루만 + 축소 + 과실 구간으로 올림. 시각 전용값(근거 등급 없음).
    foliage_fraction: float = 1.0    # 잎 얹는 그루 비율. 1.0=전 그루(잎 없는 줄기+과실 방지)
    foliage_scale: float = 0.85      # aoc 식물 기준 크기(원본 1.388m). 개체마다 ×0.8~1.2 변주됨
    foliage_z: float = 0.4           # 바닥에서 올림. 잎이 과실 구간(0.5~1.4m)에 오게 위로

    # 2026-07-18 수확·운반 피벗 → 2클래스(익은거/상한거). 성숙단계(green/half_ripe)는 스코프 밖.
    # 데모 씬 분포. 스마트팜은 관리되는 환경이라 상한거가 드물다.
    # 주의 1: 이 비율 자체는 출처가 없다. 관측 데이터가 아니라 씬 연출값이다([4] 임의).
    # 주의 2: YOLO 학습용 데이터셋은 클래스 균형이 필요하므로 이 값을 쓰지 말 것.
    #         데이터셋 생성기는 별도 분포를 써야 한다.
    class_weights: dict[str, float] = field(default_factory=lambda: {
        "ripe":    0.85,   # 익은거=수확 대상 (씬은 완숙 위주)
        "spoiled": 0.15,   # 상한거=제거 대상. 관리된 환경이라 드묾 (VegNet Old/Damaged)
    })


@dataclass
class PhysicsConfig:
    """물리 속성. 온실/줄기는 static, 과실은 kinematic RigidBody.

    출처 [W2024]: Weng et al., "Tomato Pedicel Physical Characterization for
      Fruit-Pedicel Separation Tomato Harvesting Robot", Agronomy 2024, 14(10):2274.
      DOI 10.3390/agronomy14102274 (오픈액세스, CC BY)
      품종 Syngenta Spectrum, early firm-ripening stage, 함수율 74~79%.

    이 논문이 다루는 것은 꽃자루뿐이다. 과실 질량/마찰은 여기서 나오지 않는다.
    """
    # 질량이 아니라 밀도를 준다. 질량은 PhysX 가 콜라이더 부피에서 과실마다
    # 계산한다 (scene/physics.py add_rigid_body 참고).
    #
    # 질량을 상수로 박으면 안 되는 이유 — 메시 부피가 변형별로 6배 차이난다:
    #   tomato_spoiled_03  4.56 cm^3  <->  tomato_ripe_04  27.46 cm^3
    # 전부 120g 으로 박으면 썩은 과실의 밀도가 5.6 g/cm^3 (알루미늄의 2배) 이 된다.
    #
    # [4] 임의 — 물의 밀도. 토마토는 대략 이 근처(익을수록 낮아짐)지만 출처 없음.
    fruit_density: float = 1000.0      # kg/m^3
    fruit_approximation: str = "convexHull"   # 트라이앵글 메시 충돌 금지

    # [3] 민감도. 임계 경로다 — 이 값이 수확 성공/실패를 실제로 가른다
    #     (고정 조인트를 안 쓰므로 절단 순간 과실을 붙잡는 건 마찰뿐이다).
    #
    # 2026-07-17 조사 결과: **아직 못 찾았다.**
    #   [32] Matas 2004 (Am. J. Bot 91:352) 는 무료이고 수치도 있지만
    #        과피 **인장** 물성이다 (파단응력 1.16 MPa, 영률 43.5 MPa).
    #        인장 물성으로 마찰계수를 정당화할 수 없다. 이 논문은 근거가 아니다.
    #   [19] Wang 2021 (Comput. Electron. Agric. 180:105901) 은 유료.
    #        게다가 측정값이 파지력(N)이 아니라 16개 센서의 **압력 합**이라
    #        센서 유효면적을 알아야 N 으로 환산된다. 원문 표 + 센서 규격 필요.
    #   -> 마찰계수 자체의 출처는 없다. **아래 0.9/0.7 은 스윕의 중앙값일 뿐
    #      그 자체론 의미가 없다.** spikes/01 이 μ 0.3~1.1 을 훑어
    #      "μ ≥ X 에서 성립" 을 산출한다 = 그게 그리퍼 선정 기준이 된다.
    #
    # ✅ [3] 실측 완료 (2026-07-19 spike01, GPU 5×5 스윕 — RESULTS.md):
    #      μ ≥ 0.5 → 파지력 2 N 으로 유지 / μ = 0.3 → 5 N 필요 (2 N 은 미끄러짐).
    #      실측 경계가 산술 2μF ≥ mg 와 일치 (0.12 kg: 1.2 N vs 1.18 N) —
    #      솔버 페널티 없음. 파지력 창: 2 N ≤ F ≤ 18 N (패드 2 cm², 90 kPa 상한).
    #      → 그리퍼 요구사항: 2 N 이상 + 패드 0.2 cm² 이상이면 어떤 그리퍼든 된다.
    fruit_static_friction: float = 0.9
    fruit_dynamic_friction: float = 0.7

    # 파지력 상한 — 이 압력을 넘기면 과실이 손상된다.
    #
    # 출처: Zhang Yongnian; Zhang Renfei 외, "Effects of local compression on the
    #   mechanical damage of tomato with different maturity" (공개 원문).
    #   성숙도별 무손상/저손상 경계 압력: 165 / 115 / **90** kPa
    #   (더 심한 단계는 265/245/225, 366/355/337 kPa)
    #   완숙에 가까울수록 손상 압력이 낮아진다. **수확 대상이 ripe(익은거)이므로
    #   가장 낮은 90 kPa 를 상한으로 잡는다.**
    #   ※ 사용자 제공 단서. 원문 표를 아직 직접 확인하지 못했다 (2026-07-17).
    #
    # 쓰는 법 — 이건 시뮬에서 측정할 값이 아니라 설계 제약이다:
    #   PhysX 강체는 변형을 안 해서 접촉이 사실상 점이다. 접촉 압력이 안 나온다.
    #   대신 그리퍼 패드 면적으로 환산해서 파지력 상한을 건다:
    #       F_max = fruit_damage_pressure x 패드면적
    #       예) 패드 2x3cm = 6e-4 m^2  ->  54 N
    #           패드 1x2cm = 2e-4 m^2  ->  18 N
    #   스파이크 01 이 하한(안 미끄러지는 최소 F)을 주면 창이 완성된다:
    #       F_min (시뮬 측정)  <=  파지력  <=  F_max (논문 x 패드면적)
    # [1] 출처
    fruit_damage_pressure: float = 90_000.0    # Pa

    # TODO 그리퍼 미정이라 패드 면적도 미정. 정해지면 F_max 를 계산해 걸 것.
    #      (로봇 플랫폼 확정 아님 — CLAUDE.md)
    gripper_pad_area: float | None = None      # m^2

    # 수확 실패 모드용. 정상 수확은 코드로 kinematic 을 꺼서 처리한다
    # (물리로 끊으면 비결정적이라 Play/Stop 재현성이 깨짐).
    #
    # 두 값 모두 이탈층(abscission zone) 지름 5~6mm 그룹 기준으로 통일.
    # 5~8mm 분포 중 가장 약한 구간이라 실패가 가장 먼저 나타난다 = 보수적.
    #
    # 인장 파단력 [W2024] Table 2 — 로봇이 과실을 잡아당겨 뜯는 실패 모드:
    #   5~6mm: 40.262 ± 12.437 N / 6~7mm: 44.781 ± 15.156 N / 7~8mm: 72.003 ± 23.401 N
    # [1] 출처
    break_force: float = 40.262        # N  [W2024] Table 2, 이탈층 5~6mm 평균

    # 굽힘 파단 모멘트 — 표에 직접 없어 3점 굽힘 파단력에서 유도.
    #   M = F·y/4  (논문 식 (3) σb = 8F·y/πD³ 과 동일한 3점 굽힘 관계)
    #   F = 22.384 N [W2024] Table 10 (이탈층 5~6mm), y = 12mm (2.5절 스팬)
    #   -> 22.384 × 0.012 / 4 = 0.0672 N·m
    # [1] 출처 (Table 10 의 파단력 + 스팬에서 M=F·y/4 로 유도했으므로 [2] 이기도 함)
    break_torque: float = 0.067        # Nm [W2024] Table 10 에서 유도

    # 매달림용 조인트 파단 토크 — 위 break_torque(0.067, 실제 꽃자루 굽힘강도)와 목적이 다르다.
    # spike 02(2026-07-18): 과실을 줄기에서 옆으로 매달면(과실 r34mm+줄기 r20mm 안 겹치려면
    # 최소 5.4cm) 단일 조인트에 걸리는 굽힘모멘트 ≈ 무게×오프셋 ≈ 0.05~0.07 N·m 로 실제
    # 굽힘강도(0.067)를 넘겨 **중력만으로 끊긴다**. 실제 화방은 트러스가 굽힘을 분산하지만
    # 우리 모델은 조인트 하나라 여기 다 몰린다. 그래서 **매달림 조인트엔 이 값**을 써서
    # 중력엔 안 끊기게 하고, 0.067 은 '실제 꽃자루 굽힘강도' 참조로 남긴다.
    # (정상 절단은 jointEnabled=False 로 결정적으로 끊으므로 이 값과 무관.)
    # [4] 임의 — 매달림 하한 이상으로 잡음. spike 02 재현: 0.5 는 부족(대부분 낙하),
    #     큰 값이면 전부 매달림(0 지터). 시작 스냅/질량 요인 조사 전까지 넉넉히 잡는다.
    #     TODO 필요 토크의 실제 원인(스냅 vs 질량) 규명 후 최소값으로 낮출 것.
    pedicel_hold_torque: float = 50.0   # Nm

    # 매달림용 조인트 파단 힘 — break_force(40.262, [W2024] 이탈층 인장)와 목적이 다르다.
    # 일부 과실이 스폰 시 이웃/줄기와 살짝 겹쳐 분리 충돌 임펄스가 40N 을 순간 초과 → 조인트가
    # 끊겨 낙하(hold_torque 만으론 5% 낙하). 매달림엔 이 큰 값을 써 그 충돌에도 안 끊기게 한다.
    # 40.262 는 '실제 이탈층 인장 파단력' 참조로 남긴다. [4] 임의.
    pedicel_hold_force: float = 2000.0  # N


@dataclass
class TrayConfig:
    """수확물 트레이 — MM 이 여기 담고, AMR 이 이걸 통째로 나른다.

    이 구조의 핵심: AMR 은 토마토를 직접 안 만지고 트레이만 다룬다.
    무른 과실을 다루는 건 MM 혼자다. AMR 쪽 파지 난이도가 확 내려간다.
    """
    # [4] 임의. v3 8장 "6칸 격자형" 은 팀 결정일 뿐 근거가 없다.
    #     ※ 창고 슬롯도 6개지만 **무관하다**. 우연히 같은 숫자다 (헷갈리지 말 것).
    # TODO [2]로 옮길 것 — 트레이 크기는 사실 자유롭지 않다. 세 제약의 교집합이다:
    #        · MM 팔이 트레이 전 칸에 닿아야 한다        (팔 도달범위)
    #        · AMR 포크가 들 수 있어야 한다              (포크 폭·하중)
    #        · 창고 슬롯에 들어가야 한다                  (슬롯 치수)
    #      로봇·에셋이 정해지면 칸 수는 이 교집합에서 유도된다. 그때 6이 맞는지 재확인.
    capacity: int = 6

    # [2] 유도. v3 는 "시작 시 50%(3개) 사전 적재" 로 pick 사이클을 줄이려 했으나,
    #     v3 8장이 스스로 "처리량 검증을 과소평가할 위험" 이라고 인정한 단축이다.
    #     사전 적재 = 측정하지 않은 구간을 성공으로 세는 것 -> 성공률이 부풀려진다.
    #     정량 검증(35점)이 목적이면 0 이 유일하게 정합적인 값이다.
    #     빼도 개발 시간은 같다(이 값 하나). 일정이 급하면 3 으로 되돌릴 수 있다.
    preloaded: int = 0

    # [4] 임의 — 아직 값이 없다. 트레이 에셋이 나오면 실측.
    #     하한만은 [2]: 과실 지름 68.7mm 이므로 칸 간격은 그보다 커야 한다.
    cell_pitch: float | None = None    # m. 칸 간격
    size: tuple[float, float, float] | None = None   # m


@dataclass
class SectorConfig:
    """재배 섹터 — MM 이 Nav2 로 섹터 단위 이동한다.

    v3 11장(멘토 제안): 재배 6섹터 : 창고 3섹터×2단 = 6슬롯을 **1:1 매핑**.
    섹터1→창고1 상단, 섹터2→창고1 하단, … 이렇게 규칙으로 정해지면
    창고 배치 로직이 단순해지고 지게차 상하 승강도 실제로 쓰인다.
    """
    # [4] 임의. 6 이라는 숫자 자체엔 근거가 없다.
    #     주의 — 순환 논리를 피할 것: "창고가 6슬롯이라 재배도 6섹터" 이고
    #     "재배가 6섹터라 창고도 6슬롯" 이면 아무것도 정당화하지 못한다.
    #     정당화되는 건 **개수**가 아니라 **1:1 매핑**이다 (WarehouseConfig 참고).
    # TODO [3]으로 옮길 것 — 섹터 수는 사이클 타임에서 나와야 한다:
    #        섹터가 많으면 MM 이동시간↑, 적으면 섹터당 과실↑ 해서 트레이가 빨리 참.
    #        둘의 균형점이 처리량 최대. 사이클 타임을 재면 이 값이 유도된다.
    count: int = 6

    # [4] 임의. 섹터 배치(1열/2열, 간격)가 아직 안 정해졌다.
    #     지금 씬은 온실 하나(4줄×33그루)라 6섹터 구조로 재작업이 필요하다.


@dataclass
class PalletPhysicsConfig:
    """나무 팔레트(EUR) 물리 — 지게차로 실제로 드는지 스파이크 06 에서 실측 확인(2026-07-20).

    ★핵심은 콜라이더 근사다. 단일 convexHull 은 팔레트를 통짜로 감싸 옆면 포크 슬롯을
      메워 포크가 못 들어간다. convexDecomposition(여러 볼록덩이)으로 데크·블록을 조각내면
      그 사이 포크 채널이 빈 공간으로 남아 포크가 들어간다. 리프트는 주로 포크가 형상으로
      받치므로 마찰은 2차(미끄러짐 방지).
    """
    mass: float = 25.0             # kg [1] EPAL EUR 팔레트 규격. 밀도 아님 — 속이 비어
                                   #    부피기반 밀도는 과대. MassAPI 로 질량 직접 지정.
    static_friction: float = 0.5   # [3] 목재-강철 엔지니어링 마찰표 (필요시 스윕)
    dynamic_friction: float = 0.35 # [3] 동일
    # [2] 유도 — 포크 슬롯 보존이 요구사항이라 convexDecomposition 을 쓴다(헬퍼가 강제).
    #   아래 파라미터는 얇은 채널까지 살리려 세밀하게 준다.
    max_convex_hulls: int = 64          # 조각 많이 → 포크 채널 보존
    voxel_resolution: int = 500000      # 세밀 복셀화
    error_percentage: float = 1.0


@dataclass
class WarehouseConfig:
    """창고 — AMR 이 포크로 트레이를 지정 슬롯에 올린다."""
    # [2] 유도. 개수 자체가 아니라 **1:1 매핑**이 정당화된다:
    #     슬롯 수 = 재배 섹터 수여야 "섹터N 트레이는 슬롯N 으로" 가 규칙으로 정해진다.
    #     그러면 창고 배치가 탐색 문제가 아니라 상수 조회가 되고, 트레이 출처 추적도
    #     공짜로 된다(어느 슬롯에 있으면 어느 섹터에서 온 것). v3 10장이 걱정한
    #     "창고 적재 위치 결정 로직 복잡도" 가 이 매핑 하나로 사라진다.
    #     3×2 로 쪼갠 이유: 6슬롯을 1단으로 깔면 포크 승강이 필요 없어진다.
    #     2단이라야 지게차의 상하 1축이 실제로 쓰이고, 그게 AMR 을 지게차형으로
    #     고른 이유(v3 팀 결정)와 맞물린다. 안 그러면 포크가 장식이 된다.
    sectors: int = 3
    levels: int = 2

    # 랙에 얹는 나무 팔레트를 진짜 물리 객체로(지게차 리프트용). 스파이크 06 실측 검증.
    pallet_physics: PalletPhysicsConfig = field(default_factory=PalletPhysicsConfig)

    # 제약은 [2] 실측 — ForkliftB lift_joint 을 GPU 에서 읽음 (2026-07-18, RESULTS.md):
    #   prismatic Z, limits (-0.15, 2.0) m → 최상단 선반(base_z + level_height)이
    #   2.0m 이하면 포크가 닿는다. 아래 값이면 1.25m ≤ 2.0m OK.
    # 값 자체는 [4] 임의 — 트레이 높이가 정해지면 "트레이 높이 + 여유" 로 유도([2])할 것.
    level_height: float | None = 0.9       # m. 1단→2단 높이차
    # 베이(2단 슬롯 컬럼) 간격 (m). 나무 팔레트(EUR 1.213m 폭)를 통째로 얹으므로
    #   팔레트 폭 + 여유로 잡는다. 1.6m → 팔레트 1.213 + 0.39 간격. 3베이 = 4.8m 랙(뒷벽 부착).
    # [2] 유도 — 팔레트 폭에서 하한(≥ 팔레트폭 + 포크 여유)이 나온다.
    slot_pitch: float | None = 1.6

    # 창고 방 치수 (m). 폭·높이는 재배 공간과 같게 런타임에 받는다(온실 width/height,
    #   팀 피드백 2026-07-19: "창고 세로 = 재배공간 세로, 높이 동일, 천장 제거").
    #   depth 만 창고 고유값 — 지게차가 들어와 회전하고 뒷벽 랙에 적재 후 돌아나갈 여유.
    # [4] 임의 — 회전 반경 실측 전. ForkliftB 3.03m(실측) 기준 넉넉히 잡음.
    depth: float = 8.0                     # m. 방 앞뒤 깊이(Y)
    # 입구 폭 — 기존 3.2m × 1.5 (팀 피드백 2026-07-19). ForkliftB 폭 1.13m 대비 충분.
    # [2] 유도 — 기존값의 1.5배 지시에서 나옴.
    entrance_width: float = 4.8            # m

    @property
    def slots(self) -> int:
        return self.sectors * self.levels


@dataclass
class EndEffectorConfig:
    """수확 MM 엔드이펙터 — 2-finger 그리퍼 + 커터 일체형.

    커터를 다는 근거 [W2024]:
      인장(당기기) 수확은 손상 확률 최대. 전단 성공률 100% vs 굽힘 42.83%.
      -> shear(절단) 권장. 자를 곳은 distal pedicel (전단력 33.241 N).
         proximal 은 85.32% 더 힘듦 (Table 4).
    v3 6.2 하드웨어에는 그리퍼만 있었다 = 당겨서 뜯는다는 뜻이 된다.
    2026-07-17 팀 결정으로 커터 추가.

    vacuum 을 안 쓰는 이유: Isaac 에 vacuum 이 없어 고정 조인트로 흉내내야 하는데,
    그러면 흡착력·마찰이 결과를 안 바꾸게 되고 "그 값 근거가 뭐냐" 에 답이 없어진다.
    """
    # [2] 유도 (하한만). 커터는 파지점에서 위로 이만큼 — 한 자세에서 그리퍼가
    #     과실을, 커터가 꽃자루를 잡는다. 파지력과 똑같이 **창**으로 정해진다:
    #
    #       과실 반지름  <  cutter_offset_z  <  과실 반지름 + 꽃자루 길이
    #       34.4mm(실측)                       └ 미확보 (아래 참고)
    #        └ 이보다 작으면 커터가 과실을 자른다
    #
    #     하한 34.4mm 는 obj 를 직접 재서 나온 값이라 [2] 다.
    #     상한은 꽃자루 길이가 있어야 하는데 **[W2024] 가 측정했다고 써놓고 표에
    #     값을 안 실었다** (2.1절). 논문만으론 복원 불가 — 저자 문의 필요.
    #     그때까지 45mm 는 하한 + 여유 10.6mm 인 [4] 임의값이다.
    #
    #     ⚠ 이름은 _z 지만 실제 방향은 **위(+Y, 파지점에서 꽃자루 쪽)** 다 —
    #       주석의 "위로 이만큼" 이 그 뜻. 2026-07-18 이전 코드가 이 값을 접근축(+Z)
    #       으로 잘못 넣어 커터가 손끝보다 100mm 뒤 몸통에 파묻혔다(RESULTS.md).
    cutter_offset_z: float = 0.045     # m  (파지점 기준 +Y 오프셋)

    # [2] 유도 — 파지점(과실 중심)까지 접근축(+Z) 거리. 그리퍼 손끝 bbox Z=148mm
    #     (2026-07-18 실측)에서 과실 반지름 34.4mm 안쪽 = 148-34 ≈ 114mm.
    #     커터/카메라를 파지점 기준으로 놓는 데 쓴다. 실제 파지 자세가 확정되면
    #     (팀 미정 — CLAUDE.md §7) 재검토. 지금은 손끝 형상에서 유도한 값.
    grasp_reach_z: float = 0.115       # m

    # [4] 임의 -> [3] 으로 옮길 값. 절단 성공 판정 반경 — 커터가 이 안에 들어오면
    #     자른 것으로 본다.
    #
    #     칼날이 실제로 절삭하는 물리는 범위 밖이다(절삭 시뮬레이션).
    #     이건 편법이 아니다: 꽃자루가 안 끊긴 상태를 모델링하다가, 끊는 순간부터
    #     진짜 물리(마찰로 붙잡기)로 넘어간다. **잡는 건 진짜 마찰이어야 한다.**
    #
    #     10mm 자체엔 근거가 없다. 다만 이 값은 **스윕하면 요구사항이 된다** —
    #     줄여가며 수확 성공률이 어디서 무너지는지 보면, 그 지점이 곧
    #     "커터 위치 정밀도가 몇 mm 이내여야 하는가" = 엔드이펙터 설계 요구사항이다.
    #     마찰계수를 μ 스윕으로 처리한 것과 같은 방법.
    cut_tolerance: float = 0.01        # m

    # [4] 임의 — 손끝 카메라(eye-in-hand) 장착 위치. 그리퍼 base_link 로컬(m).
    #     +Z=손가락/접근, +Y=몸통 위쪽 (2026-07-18 축 탐침).
    #     그리퍼 몸통 바로 위(5.5cm)에 얹고 파지점을 내려다보게 각을 준다
    #     (_add_camera 가 grasp_reach_z 를 향해 look-at). 12cm 로 띄우면 허공에
    #     뜬 것처럼 보였다(사용자 지적) → 낮추고 시선 각으로 손가락 가림을 푼다.
    camera_offset: tuple[float, float, float] = (0.0, 0.055, -0.02)


@dataclass
class RobotAssetConfig:
    """Isaac 기본 에셋 경로. 루트는 런타임에 get_assets_root_path() 로 얻는다.

    경로가 버전마다 바뀌므로 **후보를 순서대로 시도**한다 (verify.py / ros/graph.py 와
    같은 패턴). 실제로 뭐가 있는지는 `spikes/03_asset_check.py` 가 확인한다.

    ⚠ 아래는 2026-07-17 문서 조사 기준이고 **GPU 실물 확인 전이다.**
    """
    # 수확 MM = 베이스 + 팔. 기본 RidgebackUr 은 UR5 라 도달이 0.4m 부족하므로
    # 베이스와 팔을 따로 불러 조립한다 (RobotConfig 의 도달 검산 참고).
    # 팔 없는 Ridgeback 은 서버에 없다 (2026-07-18 탐침 — Clearpath 폴더 전수 확인).
    # RidgebackUr(UR5 포함)을 쓰되 harvester 가 UR5 서브트리를 끄고 베이스만 남긴다.
    base: tuple[str, ...] = (
        "/Isaac/Robots/Clearpath/RidgebackUr/ridgeback_ur5.usd",
    )
    arm: tuple[str, ...] = (
        "/Isaac/Robots/UniversalRobots/ur10e/ur10e.usd",
        "/Isaac/Robots/UR10e/ur10e.usd",
        "/Isaac/Robots/UniversalRobots/ur10/ur10.usd",
    )
    gripper: tuple[str, ...] = (
        # 실측 (2026-07-18 GPU 탐침, omni.client.list): 이 파일명이 실제로 존재한다.
        # 이전 후보 2개는 파일명이 틀려 spike 03 이 "없음" 으로 봤었다.
        "/Isaac/Robots/Robotiq/2F-85/Robotiq_2F_85_edit.usd",
        "/Isaac/Robots/Robotiq/2F-85/Robotiq_2F_85.usd",
        "/Isaac/Robots/Robotiq/2F-85/2f85.usd",
    )
    # 손끝 카메라. 실측 (2026-07-18 GPU 탐침, omni.client.list): 존재 확인.
    # RealSense D455 실물 센서 에셋이라 화각·해상도가 실제 스펙과 같다 = 출처가 된다.
    camera: tuple[str, ...] = (
        "/Isaac/Sensors/Intel/RealSense/rsd455.usd",
    )
    # 창고 환경. v3 6.2 "Isaac Sim Warehouse 기반 커스텀" 이 그대로 된다.
    # 선반이 이미 들어있는 완성 씬이라 창고 랙도 직접 모델링할 필요가 없을 수 있다.
    #   warehouse_multiple_shelves : 선반 여러 개 (우리 3섹터×2단에 제일 가까움)
    #   full_warehouse             : 선반 + 장애물 + 지게차. 무겁다
    #   warehouse                  : 선반 하나
    warehouse_env: tuple[str, ...] = (
        "/Isaac/Environments/Simple_Warehouse/warehouse_multiple_shelves.usd",
        "/Isaac/Environments/Simple_Warehouse/warehouse.usd",
        "/Isaac/Environments/Simple_Warehouse/full_warehouse.usd",
    )

    # 운반 AMR = 지게차. 포크 승강이 이미 달려 있어 직접 모델링이 필요 없다.
    # B = 오더피커형(운전석이 포크와 함께 승강), C = 카운터밸런스형(포크+마스트만
    # 승강, 운전석 고정). 둘 다 실측 확인 (2026-07-18). 구동계가 다르다:
    #   B: 후륜 1개 조향+구동 / C: 뒷바퀴 2개 구동 + 로테이터 2개 조향.
    forklift: tuple[str, ...] = (
        "/Isaac/Robots/IsaacSim/ForkliftB/forklift_b.usd",
        "/Isaac/Robots/Forklift/forklift_b.usd",
        "/Isaac/Robots/IsaacSim/ForkliftC/forklift_c.usd",
        "/Isaac/Robots/Forklift/forklift_c.usd",
    )
    forklift_c: tuple[str, ...] = (
        "/Isaac/Robots/IsaacSim/ForkliftC/forklift_c.usd",
    )
    # 운반 AMR (물류 루프 확정 2026-07-19: MM 은 iw.hub 위 KLT 에 넣기만, iw.hub 가
    # 팔레트째 나르고 지게차가 랙 적재). 실측 (2026-07-19 Nucleus listing):
    # 1431×659×231mm, 페이로드 1000kg, DOF: left/right_wheel_joint(차동)+lift_joint.
    # 폭 0.66m < 이랑 1.5m → 통로 주행 OK.
    iwhub: tuple[str, ...] = (
        "/Isaac/Robots/Idealworks/iwhub/iw_hub.usd",
    )
    # 물류 프랍 — iw.hub 데크 위에 팔레트+KLT 세트로 얹는다. 실측 (2026-07-19 동일):
    #   small_KLT 198×297×146mm / EUR 팔레트 1213×802×143mm (팔레트가 iw.hub 보다
    #   넓은 건 정상 — 언더라이드 AMR). pallet_holder = 창고 랙(팔레트 거치대).
    klt_bin: tuple[str, ...] = (
        "/Isaac/Props/KLT_Bin/small_KLT.usd",
    )
    pallet: tuple[str, ...] = (
        "/Isaac/Props/Pallet/pallet.usd",
    )
    pallet_holder: tuple[str, ...] = (
        "/Isaac/Props/Pallet/pallet_holder.usd",
    )


@dataclass
class IwHubNavConfig:
    """iw.hub 자율주행(Nav2) 파라미터 — 차동구동·오도메트리·라이다.

    §5.6: Nav2 판단은 dev PC(ROS2), Isaac 은 /cmd_vel 실행 + /odom·/scan·/tf 발행만.
    ⚠ 아래 치수는 전부 [4] 임의 — GPU 실측 전이다. 노드 타입명·정확 배선은
      tools/nav2_node_probe.py 로 GPU 에서 create-probe 해 확정할 것(추측 금지, §8 graph.py 교훈).
    """
    # 차동구동(DifferentialController). 틀리면 주행속도·회전이 스케일만 어긋난다(움직이긴 함)
    #   → odom 정확도에 직접 영향. iw.hub 바퀴를 GPU 에서 재서 [3]/[2] 로 올릴 것.
    wheel_radius: float = 0.1        # m   TODO 근거 없음 — iw.hub 구동륜 반지름 실측
    wheel_base: float = 0.5          # m   TODO 근거 없음 — 좌우 구동륜 간격(트랙폭) 실측
    max_linear_speed: float = 1.0    # m/s   [4] 임의 (안전 상한)
    max_angular_speed: float = 1.5   # rad/s [4] 임의

    # 라이다 마운트 (base_link 로컬). [4] 임의 — 차체 앞 상단 가정. GPU 에서 실측.
    lidar_offset: tuple[float, float, float] = (0.3, 0.0, 0.3)  # m TODO 근거 없음
    # TF 프레임/토픽 (단일 로봇이라 네임스페이스 없이 Nav2 기본명)
    odom_frame: str = "odom"
    base_frame: str = "base_link"
    lidar_frame: str = "laser"
    cmd_vel_topic: str = "/cmd_vel"
    odom_topic: str = "/odom"
    scan_topic: str = "/scan"


@dataclass
class HarvesterNavConfig:
    """수확 MM(Ridgeback) 자율주행(Nav2) 파라미터 — 홀로노믹 베이스·오도메트리·라이다.

    왜 IwHubNavConfig 를 재사용하지 않는가 — 구동 방식 자체가 다르다:
      iw.hub    = 차동구동(v, ω)      → DifferentialController 로 좌우 바퀴 속도
      Ridgeback = 홀로노믹(vx, vy, ω) → 바퀴 조인트가 아니라 **키네마틱 더미 3축**
        (dummy_base_prismatic_x/y, dummy_base_revolute_z)을 파이썬이 적분해 텔레포트한다.
        이 베이스는 위치/속도 드라이브를 무시하고 텔레포트만 먹는다(2026-07-18 실측, mm.py).
        → OmniGraph 홀로노믹 컨트롤러 노드가 필요 없다. 미확정 노드 타입이 하나 줄어든다.

    Nav2 쪽(dev PC)에서 이 값과 짝이 되는 파일:
      src/smartfarm/fleet_dispatch/config/harvester_nav2.yaml (프레임·토픽·속도상한 일치 필수)
    """
    # [1] Clearpath Ridgeback 공식 사양: 0.96 × 0.793 × 0.296 m, 최고 1.1 m/s, 적재 100 kg.
    # [4] 아래 상한은 사양이 아니라 **온실 통로 안전 상한** — 통로 1.5m 에서 1.1m/s 는 과하다.
    max_vx: float = 0.5              # m/s   전후
    max_vy: float = 0.4              # m/s   좌우(게걸음). 측면은 시야가 없어 더 낮춘다
    max_wz: float = 1.0              # rad/s 제자리 회전

    # 라이다 마운트 (Base/base_link 로컬). [2] 유도 — 차체 길이 0.96m 의 앞 끝(+0.45),
    # 높이는 차체 상면(0.30m) 바로 위. 팔이 접힌 상태에서 전방 시야를 가리지 않는 최소 높이.
    # ⚠ 팔이 앞으로 뻗으면 스캔에 자기 팔이 잡힌다 — GPU 에서 확인 후 오프셋/각도 보정할 것.
    lidar_offset: tuple[float, float, float] = (0.45, 0.0, 0.35)  # m

    # TF 프레임/토픽 — **네임스페이스를 붙인다**. iw.hub 도 Nav2 를 켜면 프레임이 충돌하기
    # 때문(map 만 공유, 로봇 프레임은 접두사 분리 = Nav2 다중로봇 표준 방식).
    # ⚠ 지금은 **전역(네임스페이스 없음)** 으로 간다. 로봇이 한 대뿐이고, nav2_bringup 이
    #   네임스페이스에서 일관되지 않기 때문 — navigation_launch.py 는 [('/tf','tf')] 리맵을
    #   걸지만 slam_launch.py 는 안 건다(2026-07-20 실측). 그래서 네임스페이스를 쓰면
    #   slam_toolbox 만 전역 /tf 로 map→odom 을 쏘고 costmap 은 /harvester_0/tf 를 봐서
    #   'Invalid frame ID "map"' 이 난다. 두 번째 로봇에 nav2 를 얹을 때 다시 설계할 것
    #   (그때는 slam_toolbox 를 bringup 대신 직접 띄우고 리맵을 걸어야 한다).
    odom_frame: str = "odom"
    base_frame: str = "base_link"
    lidar_frame: str = "laser"
    # tf_namespace: "" = 전역 /tf. "harvester_0" 로 두면 /harvester_0/tf 로 쏜다.
    # 다중로봇으로 갈 때만 채울 것 — 그때는 slam_toolbox 리맵도 같이 해결해야 한다(위 주석).
    tf_namespace: str = ""
    cmd_vel_topic: str = "/cmd_vel"
    odom_topic: str = "/odom"
    scan_topic: str = "/scan"


@dataclass
class CameraBridgeConfig:
    """손끝 D455 → ROS2 발행 (시뮬 토마토 이미지로 YOLO 파인튜닝).

    렌더프로덕트 1개 → CameraHelper 로 rgb/depth/camera_info 발행. §5.6: Isaac 은
    센서 발행만, 인식(YOLO)은 ROS2(dev PC) vision_node. 노드 타입명은 GPU create-probe.
    """
    width: int = 640         # px  [4] 임의 — D455 컬러 관행 해상도. YOLO 입력에 맞게 조정.
    height: int = 480        # px  [4]
    rgb_topic: str = "/harvester/rgb"
    depth_topic: str = "/harvester/depth"
    info_topic: str = "/harvester/camera_info"
    frame_id: str = "d455_color"


@dataclass
class RobotConfig:
    """로봇 2대. v3 6.1 B안(트레이 핸드오프) + 지게차형 하역.

    왜 2대인가 — v3 문제 정의가 이미 부담을 둘로 나눠놨다:
      "반복 노동(굽힘·들기)"
        굽힘   -> 정밀 위치·힘 제어가 필요한 일 = 수확 MM
        들기·이동 -> 주행·하중이 필요한 일      = 운반 AMR
      요구 능력이 다르므로 하나가 둘 다 하면 어느 쪽에도 최적이 아니다.
      대수는 배점 때문이 아니라 문제 구조에서 따라 나온 결과다.

    3대(창고 하역 전용 암)는 v3 6.1 C안으로 검토했고 일정 리스크로 기각.
    포크를 AMR 에 얹어 로봇 종류를 안 늘리고 하역을 해결한다.
    """
    # ROS2 토픽 네임스페이스. protocol.py 의 topic 함수에 넘긴다.
    harvester_ns: str = "harvester_0"
    transporter_ns: str = "transporter_0"

    end_effector: EndEffectorConfig = field(default_factory=EndEffectorConfig)
    assets: RobotAssetConfig = field(default_factory=RobotAssetConfig)
    iwhub_nav: IwHubNavConfig = field(default_factory=IwHubNavConfig)
    harvester_nav: HarvesterNavConfig = field(default_factory=HarvesterNavConfig)
    camera: CameraBridgeConfig = field(default_factory=CameraBridgeConfig)

    # [2] 유도 — 팔은 베이스 위에 얹는다. Ridgeback 높이 0.30m (Clearpath 공식).
    #     TODO 실물 bbox 로 확인할 것 (spikes/03 이 재준다). 에셋이 다르면 여기만 고친다.
    arm_mount_z: float = 0.30          # m

    # ── 로봇 스펙은 여기 없다. 그게 맞다. ─────────────────────────────
    # 플랫폼 미확정 (CLAUDE.md 5.2). **스펙을 가정하지 말 것.**
    # 부품을 고르고 되는지 보는 게 아니라, 요구사항을 구하고 만족하는 부품을 고른다.
    # 아래는 이미 확정된 값에서 **유도된 요구사항** — 후보 로봇을 여기에 대보면 된다.
    #
    # ── Isaac 기본 에셋 대조 결과 (2026-07-17, 문서 기준. GPU 실물 확인 필요) ──
    #   운반 AMR : **ForkliftB / ForkliftC** (7 DOF 승강). 직접 모델링 불필요.
    #   수확 MM  : Ridgeback(높이 0.30m, 하중 100kg) + 팔.
    #              ⚠ **RidgebackUr5 는 안 된다** — 아래 검산 참고.
    #   그리퍼   : Robotiq 2F-85 / 2F-140. (+ 커터는 직접 붙여야 함)
    #
    #   팔 도달 검산 — 통로 중앙에서 수평 0.66m 를 쓰고 남는 수직 범위
    #   (어깨 중심 반지름 R 의 구로 근사. 1차 판정용):
    #     UR5e   R=0.85m -> 수직 -0.08~1.00m   **0.40m 부족**
    #     Franka R=0.85m -> 수직  0.09~1.17m   **0.23m 부족**
    #     UR10e  R=1.30m -> 수직 -0.64~1.60m   OK
    #   -> 과실 높이 0.5~1.4m 를 덮으려면 **Ridgeback + UR10e** 조합이 필요하다.
    #      (기본 제공 RidgebackUr 은 UR5 라 그대로 못 쓴다. 팔만 갈아끼울 것)
    #   -> 대안: 팔을 기둥 위에 올려 어깨를 높이는 것. 예전 M0609@0.95m 안이 그거였다.
    #   ※ 이 검산은 구 근사다. 실제 도달영역은 모양이 있고 자세 제약도 있으므로
    #     **GPU 에서 실물 확인 필요**. 25점(로봇제어)의 전제라 우선순위 높다.
    #
    # [2] 수직 작업 범위 : 0.5 ~ 1.4 m
    #     과실 높이(PlantConfig.fruit_height_range)에서 그대로 나온다. 그 값은
    #     하이와이어 lower&lean + 한국인 무릎~어깨 높이가 근거이므로 [1] 에 가깝다.
    #     -> 베이스 높이 + 팔의 수직 도달이 이 구간을 덮어야 한다.
    #
    # [2] 수평 작업 범위 : 조간 1.5 m (PlantConfig.row_spacing) 에서 나온다.
    #     통로 중앙에 서면 줄기까지 0.75 m, 과실은 줄기에서 0.09 m 나와 있으므로
    #     실효 0.66 m 부근. 다만 **베이스가 통로 어디에 서는지가 아직 안 정해졌다**
    #     (통로 중앙 / 한쪽 벽 붙어서 / 이랑마다 진입). 그게 정해져야 확정된다.
    #     -> 좁은 이랑에서 이 자세가 나오는지는 GPU 에서 확인할 것. 25점의 전제다.
    #
    # [2] 엔드이펙터 : 그리퍼 파지점 + 그 위 45mm 에 커터 (EndEffectorConfig)
    #
    # [2] AMR 하중 : 트레이 만재 6 x 120g = 0.72 kg + 트레이 자체 무게.
    #     -> **하중은 제약이 아니다.** 웬만한 AMR 이 수십 kg 을 든다.
    #        포크 설계에서 볼 것은 하중이 아니라 **삽입 정렬 정밀도**다
    #        (v3 10장이 "새로운 리스크" 로 지목).
    #
    # [4] 포크 승강 높이 : WarehouseConfig.level_height 에서 나와야 하는데
    #     그 값이 아직 없다. 창고 랙 에셋이 나오면 유도된다.
    #
    # [3] 파지력 : spikes/01 이 하한(안 미끄러지는 최소)을,
    #     PhysicsConfig.fruit_damage_pressure x 패드면적이 상한을 준다.
    #     그 창을 만족하는 그리퍼면 뭐든 된다.


@dataclass
class SceneConfig:
    seed: int = 42
    tomato_assets: TomatoAssetConfig = field(default_factory=TomatoAssetConfig)
    lighting: LightingConfig = field(default_factory=LightingConfig)
    greenhouse: GreenhouseConfig = field(default_factory=GreenhouseConfig)
    plants: PlantConfig = field(default_factory=PlantConfig)
    physics: PhysicsConfig = field(default_factory=PhysicsConfig)

    # v3 파이프라인 (수확 -> 운반 -> 창고). 아직 씬에 구현 안 됨.
    tray: TrayConfig = field(default_factory=TrayConfig)
    sectors: SectorConfig = field(default_factory=SectorConfig)
    warehouse: WarehouseConfig = field(default_factory=WarehouseConfig)
    robots: RobotConfig = field(default_factory=RobotConfig)
