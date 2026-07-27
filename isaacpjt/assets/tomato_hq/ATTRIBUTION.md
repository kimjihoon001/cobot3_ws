# 고화질 토마토 에셋 출처 (third-party)

- **에셋**: `tomato_hq.usdz` (9.3 MB)
- **원본**: "Tomato" by **Claudiu** — Sketchfab
- **라이선스**: **CC Attribution (CC BY 4.0)** — 상업적 이용 가능, **출처 표기 필수**
  - 표기 문구: `"Tomato" by Claudiu, licensed under CC BY 4.0`
  - https://creativecommons.org/licenses/by/4.0/
- **받은 형식**: Sketchfab 변환 USDZ (원본은 .fbx 48MB). USD 네이티브라
  `omni.kit.asset_converter` 를 거치지 않는다 — aoc 식물(`.dae`)은 그 변환에서
  재질이 통째로 날아갔다(`../aoc/ATTRIBUTION.md` 참조).
- **수정 내용**: 없음. 파일 그대로 두고 씬 코드에서 변환만 건다.

## 이 에셋을 쓸 때 알아야 하는 것 (2026-07-27 실측)

| 항목 | 값 |
|---|---|
| 메시 | 3개 — `Cube_Tomato_body_0` / `Cube_001_Tomato_stem_0` / `Cube_002_Tomatol_leaves_0` |
| 폴리곤 | 24,064면 (점 13,983) |
| 노멀 | `interp='vertex'` = 스무스 셰이딩 (저폴리 자체 에셋의 플랫 노멀 문제 없음) |
| 재질 | 완전 PBR — baseColor / roughness / metallic / **normal map** |
| UV | primvar 이름이 `st` 가 아니라 **`st0`** |
| upAxis | **Y** (씬은 Z-up → X축 +90도 회전 필요) |
| metersPerUnit | 0.01 |

**⚠ 크기를 잴 때 원시 `points` 를 쓰면 안 된다.** 메시 위에 `scale=100` →
`0.00126` → 축스왑 → `100` 변환 체인이 걸려 있어 **12.7배 틀린다**
(원시 2.66 vs 합성 33.68). 반드시 `BBoxCache.ComputeWorldBound` 로 합성 후 값을
써야 한다. 이걸 놓치면 1m 짜리 토마토가 나온다 — 실제로 한 번 그렇게 나왔다.

**⚠ 자체 PBR 재질을 덮지 말 것.** 저폴리 과실처럼 `bind_matte_material` /
`bind_texture` 를 걸면 고화질을 쓰는 의미가 없어진다. `displayColor` 는 몸통
메시에만 빨강으로 적는다 — YOLO 데이터셋 생성이 그 값을 읽기 때문이다.

## 사용 범위

`PlantConfig` 가 아니라 `TomatoAssetConfig.hq_*` 로 제어한다. 540개 전부 바꾸면
1300만면이라 **수확 정차 위치 반경 안에서만** 쓴다(`hq_center` / `hq_radius`).
나머지는 자체 저폴리 과실(`../tomato/*.usd`) 그대로.
