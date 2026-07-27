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
  - Isaac 씬에서 **시각 배경 전용**으로 사용 (콜라이더·강체 없음).
  - **텍스처 적용(2026-07-27)**: 변환기가 재질을 옮기지 못해 usd 안에는 흰색
    `DefaultMaterial` 만 남았지만 UV(`primvars:st`)와 파트별 prim 이름
    (`Branch1`/`Leaf1`/`Leaf2`/`Blossom1~3`)은 살아 있다. 이를 근거로
    `scene/tomato_plants.FOLIAGE_TEXTURES` 가 png 를 다시 이어 붙인다
    (잎·꽃은 알파 컷아웃). 그 전에는 초록 displayColor 단색만 썼다.
    - `AG15lef3` = `AG15lef1`, `AG15lef4` = `AG15lef2` 로 파일이 중복이라
      (md5 동일) 잎 텍스처는 2종만 쓴다.
    - `AG15frt*`(과피 4종)는 이 식물 메시엔 대응 파트가 없다. 수확 대상 토마토
      (`assets/tomato/*.usd`)에 구면 투영 UV 를 만들어 입히는 데 쓴다.
- **용도**: 온실 배경 식물(잎+가지)의 시각 품질 향상. 실제 수확 대상 토마토는
  본 프로젝트의 자체 obj 에셋(`assets/tomato`)을 그대로 사용한다(형상은 자체 제작,
  표면 텍스처만 위 `AG15frt*` 를 빌려 씀).

Apache-2.0 전문: https://www.apache.org/licenses/LICENSE-2.0
