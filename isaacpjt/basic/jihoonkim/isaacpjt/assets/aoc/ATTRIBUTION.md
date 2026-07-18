# 배경 식물 에셋 출처 (third-party)

이 폴더의 배경 식물 에셋은 아래 오픈소스 프로젝트에서 가져와 변환한 것이다.

- **출처**: LCAS/aoc_tomato_farm — https://github.com/LCAS/aoc_tomato_farm
- **라이선스**: Apache License 2.0
- **가져온 파일**:
  - `unity_tomato_farm_generator/Assets/Plant/tomato.dae` (잎+가지 메시)
  - 같은 폴더의 텍스처 12개 (`AG15{blo,brn,frt,lef}*.png`) — 잎/열매/꽃 텍스처
- **수정 내용**:
  - `omni.kit.asset_converter` 로 `.dae` → `.usd` 변환 (`usd/tomato_plant.usd`)
  - 변환 결과의 `upAxis` 를 Y → Z 로 재스탬프 (Isaac Z-up 씬에서 눕는 것 교정)
  - Isaac 씬에서 **시각 배경 전용**으로 사용 (콜라이더·강체 없음). 텍스처는 아직
    미적용(회색 지오메트리에 초록 displayColor 만 부여).
- **용도**: 온실 배경 식물(잎+가지)의 시각 품질 향상. 실제 수확 대상 토마토는
  본 프로젝트의 자체 obj 에셋(`tomatest/tomato_assets_usd`)을 그대로 사용한다.

Apache-2.0 전문: https://www.apache.org/licenses/LICENSE-2.0
