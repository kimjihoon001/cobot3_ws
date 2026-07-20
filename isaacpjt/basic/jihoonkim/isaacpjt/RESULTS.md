# RESULTS — GPU 랩탑 실측 기록

GPU 랩탑(Isaac Sim 5.1, RTX 5080)에서 실제로 돌려서 얻은 결과만 적는다.
python.sh(3.11)로 확인한 사실이 여기 들어간다. 추정/설계는 여기 넣지 않는다.

---

## 2026-07-18 · Step 0 (OBJ→USD 변환) — ✅ 통과

- 실행: `isaac_python tomatest/00_convert_obj_to_usd.py`
- 입력: `isaacpjt/tomato_assets/` 의 `.obj` **34개** (dented/half_ripe/overripe/ripe/spoiled/unripe)
- 출력: `isaacpjt/tomatest/tomato_assets_usd/` 에 `.usd` **34개**
- 검증:
  - obj ↔ usd 파일명 **완전 일치** (diff 빈 결과)
  - 크기 0 USD **없음**, 실제 153~191KB
- 결론: **Step 0 완료. 이후 스파이크/main 이 참조할 USD 준비됨.**
  `.obj` 원본은 안 바뀌므로 재변환 불필요(재실행 시 덮어씀).

### ⚠ 도중에 발견한 환경 버그 — `config` / `utils` 패키지명이 isaac_python 에서 가려짐

**증상**
- `from config.settings import ...` → `NameError: name 'LOADER_DIR' is not defined`
  (엉뚱하게 `omni.pip.compute/pip_prebundle/cv2/config.py` 가 잡힘)
- 그거 고치니 `from utils.paths import ...` → `ModuleNotFoundError: No module named 'utils.paths'`
  (`cv2/utils/` 가 잡힘)

**원인**
- Isaac Standalone 의 `sys.meta_path` 에 커스텀 임포터 **`FastFinder`**
  (`omni.ext._impl.fast_importer`)가 표준 `PathFinder` **앞**에 있다.
- FastFinder 가 번들된 cv2 의 top-level `config.py` 와 `utils/` 를
  bare name `config` / `utils` 로 인덱싱해 둬서, `sys.path.insert(0, ISAAC_DIR)` 로
  프로젝트 폴더를 맨 앞에 넣어도 **소용이 없다** (meta_path 가 sys.path 보다 우선).
- 즉 `config`, `utils` 는 python.sh 환경에서 top-level 패키지명으로 **사용 불가**.

**확인 방법 (재현/검증)**
```python
import importlib.util
importlib.util.find_spec("config").origin   # ISAAC_DIR 안이어야 정상
```
전수 검사 결과 충돌은 **`config`, `utils` 둘뿐**. `robots/ros/scene/vision` 은 정상.
둘 다 원인은 cv2.

**조치**
- `config/`  → `pjt_config/`  (import 20곳 치환)
- `utils/`   → `pjt_utils/`   (import 5곳 치환)
- `src/`(ROS2, python3)는 FastFinder 가 없어 **영향 없음** — 이건 isaac-only 이슈.
- CLAUDE.md §8 Mistake Log + §4 파일트리/§5.5/§5.7 경로 언급 갱신.

**교훈**
isaac_python 아래에서는 흔한 pip 모듈명(`config`,`utils` 등)을 top-level 패키지명으로 쓰지 말 것.
새 패키지 추가 시 `find_spec(name).origin` 이 ISAAC_DIR 안을 가리키는지 확인.

> **부수 발견 — Isaac 스크립트 stdout 이 종료 시 삼켜진다.**
> `SimulationApp` 이 `--/app/fastShutdown=True` 로 닫히면서 버퍼된 stdout(`print`)이 유실된다.
> `[FAIL]`/`SystemExit` 같은 **stderr** 는 살아남지만 정상 `print` 는 안 보인다.
> → 스크립트 출력을 확인하려면 **`PYTHONUNBUFFERED=1`** 로 실행하거나 산출물(파일)로 검증할 것.

---

## 2026-07-18 · Step 2.5 (spike 03 에셋 확인) — ✅ 실행됨

- 실행: `PYTHONUNBUFFERED=1 isaac_python spikes/03_asset_check.py` (headless)
- 에셋 루트: `https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/5.1`
  (네트워크로 S3 정상 해석됨)

| 에셋 | 존재 | 치수(L×W×H) / 통로(1.5m) 여유 | 경로 |
|---|---|---|---|
| 운반 AMR ForkliftB | ✅ 있음 | 3.03×1.13×2.94, 관절8 · **통로 여유 -1.53m ★빠듯/불가★** | `/Isaac/Robots/IsaacSim/ForkliftB/forklift_b.usd` |
| MM 베이스 Ridgeback | ✅ 있음 | 1.05×0.79×0.88, 관절10 · 통로 여유 0.45m OK | `/Isaac/Robots/Clearpath/RidgebackUr/ridgeback_ur5.usd` |
| 팔 UR10e | ✅ 있음 | 1.33×0.39×0.27, 관절7 | `/Isaac/Robots/UniversalRobots/ur10e/ur10e.usd` |
| 팔 UR5e(참고) | ✅ 있음 | 0.93×0.31×0.23, 관절7 | `/Isaac/Robots/UniversalRobots/ur5e/ur5e.usd` |
| 그리퍼 Robotiq 2F-85 | ❌ **없음** | → 직접 모델링 필요. 커터도 직접 부착 | — |
| AMR 대안 Nova Carter | ✅ 있음 | 0.73×0.90×0.69, 관절8 · 통로 여유 0.60m OK | `/Isaac/Robots/NVIDIA/NovaCarter/nova_carter.usd` |

### 역할 분배에 영향 주는 결론 (회의 안건)
1. **AMR 에셋 존재** → v3 §11.1 '팀원 C · AMR 모델링 Day1~2' 배정이 사라질 수 있음. 역할 재배분.
2. ⚠ **단, ForkliftB(3.03m)는 1.5m 통로에 안 맞음(여유 -1.53m).** Nova Carter(여유 0.60m)는
   통로엔 맞지만 **포크 승강 기구가 없다.** → 운반 방식 트레이드오프(지게차 기능 vs 통로 적합성)를
   팀과 정해야 함. 온실 통로 폭을 늘리거나(§6 1.5m는 [1] 출처 있음 — 함부로 못 바꿈) 소형 AMR+다른 적재 방식.
3. **Robotiq 2F-85 없음** → 그리퍼 직접 모델링 필요(스파이크 01 이 준 하한 스펙 기준으로).
4. **Ridgeback + UR10e 조합 가능**(둘 다 있음). 기본 RidgebackUr 은 UR5 라 도달 부족 → 팔 교체 작업 필요.

### 다음
- §2.7 spike 04(`--gui`, 실물 도달성) — 표의 치수는 구(球) 근사라 자세 제약을 못 봄. 실제로 놓고 확인.
  ⚠ `--gui` 는 디스플레이 필요 — headless 세션에서는 팔을 손으로 끌어보는 검증이 안 됨.

---

## 2026-07-18 · Step 2.7 (spike 04 로봇 배치/도달성) — 씬 배치됨, 도달 판정은 수동

- 실행: `DISPLAY=:1 PYTHONUNBUFFERED=1 isaac_python spikes/04_robot_place.py --gui`
- **처음엔 즉사 → 코드 버그였음** (harvester.py:72 AddTranslateOp, 위 §Mistake Log 참고). 수정 후 재실행.

배치 결과(GUI):
- ✅ Ridgeback 베이스 + UR10e 팔 (통로 중앙 원점)
- ❌ 그리퍼(Robotiq 2F-85)·커터는 **미부착** — 에셋 부재(§2.5). harvester.spawn 이 그리퍼
  resolve 에서 FileNotFoundError → spike 가 잡고 mm=None 으로 계속. **base+arm 은 이미 씬에 올라감.**
- ✅ ForkliftB 지게차 (원점 뒤 2.5m). 포크 승강 조인트 `/World/Transporter/Body/lift_joint` **발견**.
  단 창고 `level_height` 미정이라 승강 범위 판정은 보류(랙 에셋 나오면 확인).
- ✅ 과실 마커 4개: 수평 0.66m, 높이 0.5(초록)/0.95(주황)/1.4(빨강) + 반대쪽 1.4(보라)

도달 판정(수동 드래그, 25점 전제) — **아래는 사람이 GUI 에서 채워야 함:**
- [ ] 빨강 1.4m 닿나 (UR5 면 여기서 막힘 → UR10e 로 간 이유)
- [ ] 초록 0.5m 까지 내려가나
- [ ] 보라(반대쪽 이랑) 닿나 → 통로 중앙 양쪽 vs 한쪽씩
- [ ] 베이스가 통로 1.5m 안에 들어가나

미해결/후속:
- 그리퍼 부재가 harvester.spawn 을 항상 FileNotFoundError 로 만든다 → main.py 등 이후 단계도 막힌다.
  그리퍼를 **선택적(없으면 경고+계속)** 으로 바꿀지, 대체 에셋을 넣을지 팀 결정 필요.
  (harvester 는 tool 프레임 부재는 이미 경고+계속으로 처리 — 그리퍼 에셋 부재만 치명적이라 불일치.)
- **팔 2개 + 떨림 (GUI 목격)**: base 후보 1순위 맨 Ridgeback(`Clearpath/Ridgeback/ridgeback.usd`)이
  서버에 없어 2순위 `RidgebackUr/ridgeback_ur5.usd` = **UR5 가 이미 달린 조합**으로 떨어졌고,
  그 위에 UR10e 를 또 얹어 팔이 둘 → 관절끼리 충돌해 솔버가 떨었다.
  settings 의 "팔 떼고 베이스만" 주석은 틀렸다(그건 팔 달린 버전).
  → 후속: Nucleus 서버를 탐침해서 (a) 진짜 armless Ridgeback 경로 (b) 대체 그리퍼 경로를 찾는다.

---

## 2026-07-18 · Step 6 (main.py --no-ros 메인 씬) — ✅ 씬 표시 확인 (수동 검증 항목 남음)

- 실행: `DISPLAY=:1 PYTHONUNBUFFERED=1 isaac_python main.py --no-ros`
- 씬: 온실 골조(8×12×3m) + **4줄×33그루=132그루** (조간 1.50m, 주간 0.30m) + **과실 410개**
  (수확 대상 fully_ripe 109 / 제거 대상 old 23). CLAUDE.md 의 "약 401개"와 시드 난수 차이 — 정상 범위.

### 도중 발견·수정한 버그 2건
1. **토마토가 안 보였다 — 변환 USD 단위 오류 (치명)**. asset_converter 가 출력에
   metersPerUnit=0.01(cm)·upAxis=Y 를 박아서, metrics assembler 가 ×0.01 + rotX90 을
   몰래 끼워 넣음 → TOMATO_SCALE 과 곱해져 토마토 0.687mm. GUI Property 의
   `unitsResolve` op 가 단서였다. → 00_convert 에 metersPerUnit=1·upAxis=Z 후처리 스탬프
   추가, 34개 재변환. **재변환 후 GUI 에서 토마토 확인됨** (줄기에 빨강/주황/초록 점,
   15m 거리에서 수 픽셀 = 6.9cm 스케일에 부합). CLAUDE.md §8 기록.
   ⚠ **01_spawn / 02_generate_dataset 도 같은 USD 를 쓰므로 이 수정 전 생성물은 전부 무효.**
2. **기본 카메라가 온실 밖 그리드만 비춤** → main.py GUI 경로에 `set_camera_view` 로
   온실 프레이밍 추가 (발표 스크린샷에도 필요).

### 남은 수동 검증 (GUI 에서 사람이)
- [ ] 토마토 근접 뷰 — 크기 6.9cm 그럴듯한가, 칼릭스가 위를 보나 (upAxis=Z 선언 검증)
- [x] Play 시 과실이 **안 떨어지나** → **통과** (사용자 확인 2026-07-18: "플레이 해도 안 떨어진다")
- [ ] Play/Stop 반복 시 같은 자리 복귀 ★재현성 20점★
- [ ] FPS 기록

---

## 2026-07-18 · Nucleus 서버 탐침 (omni.client.list + 에셋 로드 실측)

### ForkliftB 포크 승강 — [2] 실측
```
/World/FB/lift_joint   prismatic, axis=Z, limits = (-0.15, 2.0) m
```
→ `WarehouseConfig.level_height` 의 제약이 실측으로 생겼다: 최상단 선반 ≤ 2.0m.
  기본값 0.9m(1단 0.35 + 2단 1.25 ≤ 2.0 OK)로 채움. 값 자체는 트레이 높이 확정 후 유도.

### 로봇/그리퍼 에셋 — spike 03 을 뒤집는 발견 2건
1. **맨(armless) Ridgeback 은 서버에 없다.** `/Isaac/Robots/Clearpath/` 전체:
   Dingo / Jackal / RidgebackFranka / RidgebackUr 뿐. settings 의 1순위 후보
   `Clearpath/Ridgeback/ridgeback.usd` 는 영원히 resolve 안 된다.
   → **팔 2개+떨림의 근본 원인**: 2순위 RidgebackUr(UR5 포함)로 떨어진 위에 UR10e 를 얹음.
   → 해결 방향: RidgebackUr 을 얹은 뒤 **`ur_arm_*` prim 들을 SetActive(False)** 로 끄고
     UR10e 장착. (UR5 팔은 `/base_link/ur_arm_shoulder_pan_joint` + 루트 평면에
     `ur_arm_shoulder_link` ~ `ur_arm_wrist_3_link` 형제 prim 으로 존재 — 탐침으로 구조 확인)
2. **Robotiq 2F-85 는 존재한다** — spike 03 "없음"은 파일명 후보가 틀렸던 것.
   실제 파일: `/Isaac/Robots/Robotiq/2F-85/Robotiq_2F_85_edit.usd` → settings 후보 1순위로 추가.
   (2F-140 도 있음: `Robotiq_2F_140_physics_edit.usd` 등)

### 소품 에셋 (다음 패스 후보)
- 크레이트: `/Isaac/Props/KLT_Bin/small_KLT.usd` (참조 이미지의 검은 상자와 동일 계열)
- 팔레트: `/Isaac/Props/Pallet/pallet.usd`

---

## 2026-07-18 · 맵 업그레이드 1차 (사용자 요청: "창고 크게 + 구체적/이쁘게")

씬에 추가·개선 (전부 코드 생성 지오메트리, 에셋 의존 없음):
- **창고 랙**: 기둥(4쌍)+단별 선반+상단 보+간판 패널. 3섹터×2단, BASE_Z 0.35m,
  전폭 3.0m(pitch 1.0). greenhouse_task 가 온실 뒤(+y, y=8.5) AMR 통로 끝에 스폰.
  슬롯 z 에 BASE_Z 반영 (테스트 상대 검증이라 안 깨짐. None 가드 테스트는 명시적 None 으로 수정).
- **바닥**: 콘크리트색 홀 바닥판(18×21m) + AMR 유도선 루프(진회색) + 황/흑 안전띠(+x 가장자리)
- **재배 베드**: 줄마다 흰 배지백 라인 박스 (콜라이더 있음 — 이랑 횡단 차단 역할)
- **온실 유리**: 벽 4면+지붕 반투명 패널 (opacity 0.12, 시각 전용 — 콜라이더 없음)
- main.py 기본 카메라가 온실+창고를 잡음

다음 패스 후보(미정): KLT 크레이트/팔레트 소품, 잎사귀 표현, 섹터 라벨(텍스트), 조명 톤.

---

## 2026-07-18 · 맵 업그레이드 2차 (사용자 피드백 반영)

사용자 지적 → 수정:
- **"창고 가는 길이 없다"** → 온실 뒷벽(+y)에 3m 출입구 + 문틀 기둥. AMR 경로 확보.
- **"천장 뚫어줘"** → 지붕 유리 제거 (부감 시연 우선). main.py 기본 카메라도 부감으로.
- **"토마토가 유리 같다"** → 원인: 머티리얼 미바인딩 → RTX 기본 광택 재질.
  `ripeness.bind_matte_material()` 추가 — UsdPreviewSurface(roughness 0.65) +
  **UsdPrimvarReader 로 displayColor 를 그대로 diffuse 에 연결** (정점 색 = YOLO
  클래스 정의라 색 체계는 불변). `/World/Plants` 루트에 1회 바인딩 → 전체 상속.
  ⚠ **함정 발견: PrimvarReader 의 varname 을 Token 으로 넣으면 RTX 가 못 읽고
  fallback 회색으로 렌더된다. String 이어야 동작** (headless 렌더로 확인).
- 과노출 → dome 1000→350, sun 3000→1500. 바닥 콘크리트색도 어둡게.
- 기본 지면 파란 격자 노출 → 홀 바닥판 60×60m 로 확대.

검증: headless Replicator 렌더 (scratchpad/shots4) — 무광 토마토, 반숙 그라데이션,
노화 갈색, 출입구 너머 창고 랙 모두 확인. GUI 재시작 완료 (410과실/슬롯6 동일).

### "이미지급 퀄리티" 관련 (사용자 질문)
현재는 스타일화된 수준. 참조 이미지에 가까워지려면:
- 가능(에셋 있음): ForkliftB·KLT 크레이트 실물 에셋, vMaterials(콘크리트/금속 PBR),
  HDRI 스카이, 발표 스크린샷용 PathTracing → 체감 품질 대폭 상승
- 불가(에셋 없음): 잎 무성한 토마토 식물 메시 — 직접 모델링 필요. leaf card 절충 가능
- 우선순위 주의: 점수는 스파이크/브리지(25+35점)에서 나온다. 비주얼은 스파이크 후 여유에.

---

## 2026-07-18 · MM 조립(§8 폭발) 수정 완료 — ✅ 헤드리스 물리 검증 통과

원격 끊김으로 중단됐던 harvester.py 리팩터를 이어서 완료. 끊긴 지점은
`_add_cutter` 가 `NotImplementedError` 스텁인 상태였다 (이전 구현은 01:34 pyc
바이트코드에서 복원 — 큐브 + `_fix()` 고정 조인트, 즉 §8 폭발 패턴 그대로였다).

수정 내용 (CLAUDE.md §8 Fix 방향대로):
- `_mount_arm`: UR10e 의 root_joint(월드 앵커)를 섀시 `Base/base_link` 로 재배선,
  localPos0/Rot0 = 현재 상대 포즈(오프셋 z 0.30). 팔 ArticulationRootAPI 제거.
- `_attach_gripper`: 팔 ee_joint 의 b1 에 그리퍼 base_link, 컨테이너를 툴 소켓 포즈로.
- `_add_cutter`(오늘 완성): 그리퍼 base_link **자식 prim** — 조인트·강체·콜라이더 없음.
  시각(0.03×0.002×0.01m) + cut_tolerance 거리 판정 전용. 파지점 위 cutter_offset_z.

검증 (scratchpad/verify_mount.py, 헤드리스 180스텝 = 3초):
- UR5 팔 비활성화 7개 prim / 팔 장착 오프셋 (0.004, 0, 0.3) / 그리퍼 장착 OK
- wrist_3 위치 (1.184, 0.291, 0.36) 으로 180스텝 내내 정지 — **폭발·발산 없음**
- wrist↔그리퍼 base_link 거리 0.0 mm — 분리 없음
- 커터 초기 위치 그리퍼에서 +45 mm (cutter_offset_z 반영 확인)

남은 것: spike 04 `--gui` 로 도달성 수동 판정 (팔 끌어서 마커 4개) — 이건 사람 눈 필요.

---

## 2026-07-18 · 손끝 카메라(eye-in-hand) 장착 — ✅ 렌더 검증 통과

사용자 요청 "팔에 커터랑 카메라". 커터는 이미 장착돼 있었고(§8 수정에서 완성),
카메라를 새로 달았다.

- 에셋: `/Isaac/Sensors/Intel/RealSense/rsd455.usd` — **omni.client.list 로 존재 확인**
  (경로 추측 금지 — §8). D455 실물 센서 에셋이라 화각·해상도 = 실제 스펙 = 출처.
- 부착: 그리퍼 base_link 자식(계층, §8 — 조인트 아님). 내부 RigidBodyAPI/CollisionAPI
  는 **RemoveAPI 로 제거** — disable 만으론 "missing xformstack reset" PhysX 에러가 남는다.
- 방향: **에셋 내부 컬러 카메라의 상대 회전을 런타임에 읽어 보정 회전을 계산**
  (`_add_camera`). Y축 180° 가정은 롤 90° 오류였다(렌더로 확인) — 센서 에셋 내부
  카메라는 자체 회전을 갖고 있어 방향을 가정하면 안 된다.
- `camera_offset (0, 0.12, -0.03)` [4]: 0.08 은 손가락이 시선 정중앙을 가렸다(렌더).
  0.12 로 올려 과실 위치(접근축 0.8m 앞 빨간 구, 과실 크기)가 손끝 너머로 보임.
- 검증: 헤드리스 D455 컬러 렌더 (scratchpad/cam_check) — 지평선 수평, 구 중앙 시야,
  손가락 하단 걸림 (정상 eye-in-hand 구도). 카메라 포함 재조립 180스텝 발산 없음.

주의: 브리지에 이미지 토픽은 아직 없다 (수확 토픽 3개뿐). vision_node 연결은
브리지 확인(7-A/B) 후 과제. 지금 카메라는 "달려 있고 올바른 곳을 본다"까지다.

---

## 2026-07-18 · Isaac 직접 제어 (텔레옵) — ✅ 헤드리스 제어 검증 통과

사용자 요청 "이삭심에서 로봇 둘다 제어. ROS2 는 나중에 값만 보내면 됨". §5.6 대로
Isaac=실행 계층에 제어기를 만들고(robots/control.py), 키보드 텔레옵(spikes/05_teleop.py)
으로 몬다. ROS2 는 나중에 control.py 의 set_* 에 값만 흘리면 된다.

### 실측한 관절 (probe_dof, 2026-07-18) — 추측 금지(§8)
- 수확 MM 15 DOF (root=/World/Harvester/Base):
  dummy_base_prismatic_x/y + revolute_z (**홀로노믹 베이스** 3) / UR10e 6
  (shoulder_pan…wrist_3) / Robotiq 2F-85 6 (마스터 `finger_joint`, 0=열림 0.8=닫힘)
- 지게차 7 DOF (root=/World/Transporter/Body):
  `lift_joint`(승강) / `back_wheel_swivel`(조향) / `back_wheel_drive`(구동) / 롤러 4

### 제어 방식 — 관절 성질이 달라 3가지로 나뉜다 (실측으로 결정)
- 팔·그리퍼·포크·조향: **위치 타깃** (apply_action, 드라이브가 잘 따라옴).
- 구동바퀴: **속도 타깃** (joint_velocities). 위치 타깃은 지속 회전이 안 된다.
- 홀로노믹 베이스: **상태 직접 설정(텔레포트), 단 타깃이 바뀔 때만.**
  ⚠ 핵심 발견: dummy 베이스 조인트는 **위치 타깃을 무시한다** — 드라이브 강성을
  40→1e6 로 올려도(스폰 시 USD, reset 후 게인 확인 1e6) base_x 가 정확히 0.000.
  텔레포트(set_joint_positions)는 즉시 먹음(0.800). 그래서 텔레포트로 몬다.
  단 **매 프레임 텔레포트하면 같은 아티큘레이션의 팔 드라이브 적분을 방해**해
  팔이 안 움직였다(elbow 0.6목표→0.028). → 베이스 타깃이 바뀐 프레임에만 텔레포트.
  (강성 강화는 텔레포트와 싸우기만 해서 되돌림.)

### 검증 (verify_control.py, 헤드리스 명령→2초→관절값)
  팔 elbow 0→0.600 OK / 베이스 x 0→0.400 OK / 그리퍼 0→0.79 OK /
  포크 0→0.253 OK / 구동바퀴 1.32rad 회전 OK  → **전부 통과**

### 커터·카메라 위치 버그 수정 (사용자 지적 "커터 어디, 카메라 왜 공중")
그리퍼 base_link 로컬(+Z=접근, +Y=위) 실측(probe_geom)으로 드러난 것:
- 커터가 (0,0,45mm) = 손끝(Z 148mm)보다 **100mm 뒤 몸통에 파묻힘**, 게다가
  Y=0(과실 중심)이라 꽃자루 쪽이 아님. → 원래 설정 주석은 "파지점에서 **위로**"
  인데 코드가 offset 을 접근축(+Z)에 넣은 축 오류. **(0, cutter_offset_z,
  grasp_reach_z)** 로 수정 = 파지점에서 위로. grasp_reach_z=0.115 신설([2], 손끝
  bbox 148 - 과실반지름 34).
- 카메라가 (0,120mm,-43mm) = base_link 위 12cm 허공에 뜸. → 5.5cm 로 낮추고
  파지점을 향해 look-at(내려다봄). 방향 가정(Y축 180°)은 롤 오류라 에셋 내부
  카메라 회전을 읽어 계산. RigidBody/Collision API 는 제거(중첩 강체 xformstack 에러).

⚠ 커터/카메라의 정확한 위치는 **파지 자세**에 달렸고 그건 아직 팀 미정(§7).
  지금 값은 "손끝 파지 + 꽃자루 수직(+Y)" 가정에서 유도 — 자세 확정 시 재검토.

---

## 2026-07-18 · 팔 도달성 실측 — ✅ 네 마커 전부 도달 (25점 전제 통과)

reach_test.py: shoulder_pan/lift/elbow 3,136 자세 샘플, 베이스 원점 고정·손목 0 고정
(일부러 불리한 조건). 그리퍼 파지점의 각 마커 최소거리:
  초록 0.5m → 9.4cm / 주황 0.95m → 6.6cm / 빨강 1.4m → 8.6cm / 보라 반대쪽 → 9.8cm
**하나도 "부족" 없음** — 잔여 6~10cm 는 도달 한계가 아니라 격자 간격+베이스 고정 탓.
UR5 였으면 1.4m 에서 40cm 떴을 것(그래서 UR10e). → 로봇 구성이 도달성 전제 통과.
남은 것: 위치 도달만 확인. 파지 방향(자세)은 IK 붙은 뒤(로봇 확정 후).

## 2026-07-18 · 텔레옵 조작키 방향키→글자키 (버그 수정)

증상: 팔(글자키)은 움직이는데 베이스(방향키)만 안 움직임. 원인: **방향키는 Isaac
뷰포트가 카메라 조작으로 가로채 핸들러까지 안 온다.** 베이스 이동 자체는 정상이었다
(verify_basemove: 90프레임 홀드로 베이스 0.45m 이동 + 팔 부착 유지 확인).
→ 베이스/지게차 조작을 전부 글자키로: 베이스 I/K/J/L 이동 + U/M 회전, 지게차 동일.

## 2026-07-18 · ForkliftC 추가 — ✅ 제어 검증 통과, 텔레옵 3대 구성

사용자 요청 "다른 지게차도 스폰해서 기능 보여줘". ForkliftB/C 구조 대비(실측):
- **B = 오더피커형**: lift 몸체에 포크 + **운전석 캐빈**(유리·페달)까지 → 운전석이
  포크와 함께 승강. 구동 = 후륜 1개 조향(back_wheel_swivel)+구동(back_wheel_drive).
- **C = 카운터밸런스형**: lift 몸체는 포크+마스트단만 → 운전석 고정. 구동 = 뒷바퀴
  2개(속도 드라이브, damping 있음) + 로테이터 2개 조향(위치 드라이브), 앞바퀴 수동.

구현:
- `TransporterController` 를 B/C 겸용으로 일반화 — 관절 이름으로 자동 감지
  (`back_wheel_drive`→B, `left_back_wheel_joint`→C). 구동은 속도, 조향/승강은 위치.
- `TransporterAMR.spawn(asset_candidates=...)` 로 C 에셋 지정 가능하게.
- settings 에 `forklift_c` 후보 추가.
- 텔레옵: 3대(MM 1 / B 2 / C 3), 지게차 조작 공통 I/K 주행·J/L 조향·U/M 포크.
- 검증(verify_fc): C 뒷바퀴 3.1rad 회전·로테이터 23°·포크 0.5m 전부 통과.

메모: BotBox(사용자 질문)는 The Construct 의 유료 ROS 교육 랩(Gazebo/FastBot)이라
이 프로젝트(Isaac USD)에 안 맞음 — 기존 Ridgeback+UR10e/ForkliftB·C 가 더 적합.

---

## 2026-07-18 · 배경 식물(aoc_tomato_farm) 병합 — ✅ 잎 붙음, 스케일/FPS 확인

사용자 요청: 맨 원기둥 줄기가 "현실성 없다" → 잎+가지 있는 식물 에셋으로 비주얼 향상.
출처: **LCAS/aoc_tomato_farm (Apache-2.0)** — `isaacpjt/assets/aoc/ATTRIBUTION.md` 참조.

### 받은 것 / 변환
- `Assets/Plant/tomato.dae`(3.3MB) + 텍스처 12개 → `assets/aoc/plant/` (전체 clone 안 함).
- dae 분석: 6개 Branch 메시, **27,126 삼각형/그루**, 재질·텍스처 참조 없음(`<library_images/>` 빔).
- `omni.kit.asset_converter` 로 `assets/aoc/usd/tomato_plant.usd` 생성.

### 스케일/축 (핵심 보정)
- 변환 결과 실측: 80×80×138 units, converter 가 metersPerUnit=**0.01**·upAxis=**Y** 로 스탬프.
- 138 units × 0.01 = **1.388m** = 현실적 그루 높이(수확높이 0.5~1.4m 에 맞음).
- ★upAxis 를 Y→**Z 로 재스탬프**(00_convert 토마토와 같은 함정) — 안 하면 Z-up 씬에서
  회전보정이 끼어 식물이 눕는다. 재스탬프 후 xformOpOrder 에 rotateX 보정 없음(성공).
- 실측: 1그루 **1.388m 직립**, 바닥 Z=0.

### FPS (폴리곤 폭발 없음)
- 1그루 52fps / 33그루 189fps / **132그루(4줄×33): copies 212fps, instanced 122fps**.
  → **copies(비인스턴스)가 더 빠르고 스케일도 정확**(instanceable+단위불일치는 경고만).
- 전체 씬(줄기+잎 132 + obj 과실 410 + 콜라이더): **46.9fps** (과실 물리 때문에 낮아짐, 실시간 유지).

### 통합 (tomato_plants.py — 최소 변경, 통짜 교체 X)
- `PlantConfig.use_aoc_background`(기본 False) + `TomatoAssetConfig.background_plant_usd` 신설.
- `_spawn_plant` 이 옵션 ON 이면 `_spawn_background` 로 잎을 그루 위치에 얹는다.
  **시각 전용 — 콜라이더·강체 없음**. 원기둥 줄기(콜라이더)·obj 과실(수확대상) 그대로 유지.
- 검증: 132 Foliage prim / 792 잎메시 붙음, 수확대상 과실 410개 유지, 크래시 없음.

### 함정 3건 (다시 안 밟게)
1. `asset_converter.wait_until_finished()` 가 app.update() 없이 **안 풀림**(00_convert 은 obj 라
   빨라서 안 걸림). USD 는 생성되니, 변환 후 검사는 별도 스크립트로 분리.
2. **instanceable + metersPerUnit 불일치** → metrics assembler 가 단위보정(×0.01)을 못 넣어
   "Layer ... not in local LayerStack" 경고. 스케일은 결국 맞았지만 → **copies 사용**(더 빠름).
3. **변환 USD 메시에 흰 기본재질이 바인딩**돼 루트 무광재질(displayColor)을 덮음 → 잎이 흰색.
   `UsdShade.MaterialBindingAPI(mesh).UnbindAllBindings()` 로 풀고 초록 displayColor 적용 → 해결.
   (직접 CreateDisplayColorAttr 는 primvar 보간 안 맞아 무효 — apply_flat_color 의 constant 보간 필요.)

### 남은 것 (미완/보류)
- **텍스처 미적용**: dae 에 재질참조가 없어 회색→초록 단색만. 잎 텍스처 12개(AG15lef*)를
  입히려면 6개 Branch 메시의 UV/서브메시 분리가 필요(잎/열매 구분 안 됨) — "이미지급" 원하면 별도 작업.
- **원기둥 줄기 교체 여부**: 지금은 줄기+잎 공존(줄기 콜라이더 유지 목적). 잎이 줄기를 거의
  가리므로, 줄기 시각만 숨기고 콜라이더는 남기는 방향을 팀과 판단.
- **Structure.dae/SoilBed.dae**: 아직 안 받음(gz 템플릿 경로에 있음). 온실 골조/베드 교체는 다음 패스.

---

## 2026-07-18 · spike 02 (꽃자루 조인트 401개) — ✅ 규명 완료

사용자 지시로 실행. "떠 있는 과실"에 pedicel(break joint)을 붙이는 게 목표. 결과는
예상(A/B 부하 판정)과 달랐고, 더 중요한 걸 찾았다.

### 부하는 문제가 아니다 (A안 성능상 충분)
| 개수 | 스텝(ms) | 60fps 여유 |
|---|---|---|
| 401 | 5.4 | **3.1x** |
401 조인트가 5.4ms/스텝, 60fps 예산의 1/3. **아무도 400개를 매달아본 적 없다더니, 부하는 여유.**

### 도중 고친 버그 (pedicel.py)
1. **조인트 프레임 미작성** — FixedJoint 에 localPos/Rot 을 안 줘 identity → PhysX 가
   줄기·과실 원점을 일치시키려 시작 임펄스 → 폭발(§8 로봇 MountJoint 와 동일 클래스).
   → 현재 상대 포즈로 localPos0/Rot0 작성.
2. **쿼터니언 미정규화** — 과실 Xform 스케일(0.001675)이 섞여 `ExtractRotationQuat` 가
   (0.5,0,0,0) 같은 비정규 쿼터니언을 내놓아 프레임이 깨짐 → 과실이 안 매달리고 떨어짐.
   → `.GetNormalized()`. (단위추적 진단으로 특정: z 1.0→0.02 낙하 관찰)
3. 꽃자루 세그먼트 콜라이더 제거(시각 전용) + 미사용 `from scene import physics` 정리.

### ★핵심 발견 — "물리 불안정"은 사실 물리가 옳았던 것★
프레임 수정 후에도 과실이 떨어졌다. 단위추적 + 질량측정으로 규명:
- 과실 질량 **77.9g**(변형별 부피차, tomato_dented), 무게 0.764N.
- 과실이 줄기에서 **수평 90mm**(fruit_offset) 매달림 → 굽힘모멘트 =
  0.764N × 0.09m = **0.0688 N·m**, 실제 `break_torque` **0.067 N·m 를 아슬아슬 초과**.
- 즉 **[W2024] 의 실제 꽃자루 굽힘강도로는 78g 과실을 90mm 수평 캔틸레버로 못 버틴다** —
  바로 끊긴다. **씬 기하가 비현실적**인 것(실제 토마토는 화방에서 아래로 매달려 인장,
  수평 캔틸레버가 아님). break_torque(출처 있음)가 비현실 기하를 잡아낸 셈.
- 검증: break_torque 를 굽힘 위(0.2)로, break_force 실제(40.262) 유지 →
  **과실 완벽히 매달림, 절단 시 97.8cm 낙하.** 절단도 원래 잘 됨(그동안은 이미 끊겨서
  자를 게 없었던 것).

### 판정
- **A안(전부 dynamic+조인트) 성능·메커니즘 모두 OK.** 단, **과실 매달림 기하를
  인장 방향(화방 아래)으로 바꿔야** 실제 break_torque 로 버틴다.
  지금 fruit_offset(0.09m 수평, [4] 무근거)이 굽힘을 만든다.
- 이건 앞서 나온 "과실 공중부양 + 잎 개연성" 문제와 **같은 뿌리** — 과실을 트렐리스/화방
  아래로 인장 매달면 세 문제(부양·잎개연성·break_torque)가 함께 풀린다.
- 결정 필요(사용자/팀): 과실 매달림 기하(수평→아래 인장). fruit_offset·화방 구조.

### 미확인/후속
- 401 스케일에서 기하 수정 후 지터 0 확인 (지금 스파이크 build 는 수평 기하라 여전히 끊김).
- break_force(40N 인장)는 78g(0.76N)에 여유 충분 — 인장 방향이면 안 끊긴다.

### SCENE_NOTES §3 와의 연결 (사용자 지적으로 확인)
SCENE_NOTES 가 이미 설계를 기록해뒀다 — spike 02 해석이 이걸로 바뀐다:
- **[과실=kinematic]**: 성능+의도. 떠 있는 건 실수가 아니라 설계 (401 전부 dynamic 회피).
- **[breakForce 안 씀→코드 절단]**: **재현성(20점) 때문에 물리-파단을 일부러 피함**
  ("breakForce 로 끊으면 비결정적"). cut 은 코드로 kinematic off = 결정적.
  breakForce/Torque 는 **'실패 모드(당기기)'용으로만** 남기기로 이미 정해져 있었다.
- 옛 placeholder break_torque **10 N·m** → [W2024] **0.067 N·m** (150배↓) 로 소싱하면서
  수평 캔틸레버 굽힘(0.069)을 못 버티게 된 것. (소싱이 기하 모순을 드러낸 사례.)
→ 결론 보정: **Plan A(전부 dynamic 조인트)는 SCENE_NOTES 의 재현성 설계와 상충**한다.
  재현성 우선이면 kinematic+코드절단(사실상 Plan B) 유지가 맞고, "떠 있는 개연성"은
  **시각 전용 꽃자루**로 풀면 된다(물리 조인트 없이 연결돼 보이게 + 결정성 유지).
  physical pedicel(조인트)로 가려면 과실을 인장 방향으로 매달고 break_torque 는
  실패모드 한계로만 두는(정상 하중엔 안 걸리는) 재설계가 필요.

## 2026-07-18 · A안 구현 — 씬에 물리 파단 조인트 붙임 (✅ 381/381 매달림)

사용자 결정(A안): 토마토를 실제로 매달고(dynamic+조인트), 커터 정상, **절단은 break
joint**(pedicel.cut = jointEnabled=False, 결정적). 씬 3파일 + pedicel.py 수정. 헤드리스
검증: **381개 전부 매달림, 낙차 0.0cm / 절단 47.9cm 낙하 / post_reset 복원 0.0cm / disjointed 0.**

### 바꾼 것
- `tomato_plants._spawn_fruit`: 과실 kinematic→**dynamic**, 인장 기하(옆 h_offset+아래 drop),
  `pedicel.spawn`으로 파단 조인트 연결 + joint 경로 저장. 같은 그루 과실은 **줄기 둘레로
  각도 균등 분배**(겹침 제거).
- `greenhouse_task`: detach_fruit → `pedicel.cut`, post_reset → `pedicel.restore`, `_joint_of`.
- `pedicel.spawn`: `viz_root` 인자(시각 세그먼트를 변환 없는 부모에 둬 스케일/이동 오염 방지).

### 규명한 것 (검증 반복으로)
1. **조인트 프레임 위치도 스케일에 오염됐다** — 기존 코드는 회전만 GetNormalized 했는데,
   `rel=m1*m0⁻¹`의 **translation** 도 과실 스케일(0.001675)에 오염돼 앵커가 어긋나 스냅.
   → 스케일 뺀 **순수 강체 변환**으로 rel 재구성. disjointed 1→0.
2. **과실 Xform 회전(rotateZ) 제거** — 회전이 있으면 프레임이 어긋나 disjointed 381개.
   토마토는 구형이라 요 무의미 → 제거하니 spike 02(회전 없음)와 같은 구조.
3. **매달림엔 상향 hold 값 필요** — 옆매달림(콜라이더 클리어런스상 h≥54mm 불가피)의
   굽힘·스폰겹침이 실제 파단값(0.067N·m / 40.262N)을 넘겨 끊긴다. →
   `pedicel_hold_torque=50`, `pedicel_hold_force=2000`(매달림 전용). **0.067/40.262 는
   '실제 꽃자루 굽힘·인장강도' 참조로 settings 에 유지**(절단은 jointEnabled 이라 무관).
4. **스폰 겹침이 침투복구로 5%를 튕겼다** — h_offset 60→90mm(클리어런스 36mm) + 같은 그루
   각도 균등분배로 겹침 제거 → 낙하 21→14→**0**.
5. **isolated 검증에서 절단이 0cm였던 건 과실 sleep** — 리셋+짧은 워밍업 후 절단하면 정상
   낙하(실제 씬은 로봇 접근이 깨움). cut 메커니즘 자체는 정상.

### 남은 것
- hold 값이 큰 이유(스냅 잔여 vs 질량)를 더 파면 최소값으로 낮출 수 있음(현재 [4]).
  트러스를 별도 강체+조인트로 모델링하면 pedicel 조인트가 순수 인장이 돼 실제 0.067 로 감.
- 재현성: dynamic 이지만 조인트가 뻣뻣해 지터 0 — SCENE_NOTES 재현성(20점)과 실측상 양립.

---

## 가위형 커터 (엔드이펙터 절단기) — GPU 검증 (2026-07-18)

**배경**: 커터가 "큐브 날 1개 + 거리판정→pedicel.cut" 이라 "순간이동 절단" 느낌이었다.
실제 수확 로봇(RoboHarvest shear-gripper, iris cutting gripper)처럼 **관절 달린 가위**로
바꿔 절단 시 날이 실제로 오므라들게 했다. **절단 자체는 여전히 조인트 끊기**(물리 절삭은
범위 밖) — 날 닫힘은 연출, 실제 분리는 `scene/pedicel.cut()`. §5.1 우회 아님.

**구현** (`robots/harvester.py`):
- `_add_cutter` 큐브 1개 → **힌지(실린더 강체) + 날 2개(얇은 판)**. 힌지는 그리퍼에
  FixedJoint, 날은 힌지에 **RevoluteJoint(Y축) + DriveAPI(force, 목표각)**. 시작 ±32° 열림.
- 강체 중첩(base_link 밑)이 xformstack reset 에러(§8 카메라 케이스)를 내므로 커터는
  **root 밑 별도 컨테이너**(`{root}/Cutter`)에 두고 조인트로만 그리퍼에 건다. 날에
  **콜라이더 없음**(§5.1 — pedicel 세그먼트와 동일, 붙이면 꽃자루/과실을 밀어냄).
  콜라이더가 없어 질량은 MassAPI 로 명시, 중력은 끔(연출용 경량 기구).
- 동작 3종(§5.6 — Isaac 은 실행만, 언제 자를지는 FSM): `can_cut(stage,joint)→(bool,거리)`,
  `do_cut(stage,joint)→bool`(날 닫고 pedicel.cut), `open_blades(stage)`.
- 파지용 `find_finger_joints(stage)` — Robotiq 2F-85 손가락 관절 후보 탐색(transporter 패턴).
- 순수 거리함수 `cut_distance` + `tests/test_cutter.py`(dev 머신, GPU 없이 톨러런스 판정).

**검증** (`scratchpad/verify_cutter.py`, 헤드리스):
- **조립 안정성**: 폭발 없음, 120스텝 커터 드리프트 **6.0mm** (발산 아님, 정상 세틀).
  → 가위 커터가 검증된 아티큘레이션을 안 흔든다.
- **날 개폐**: 열림 **64.0°**(2×32) → 닫힘 **0.0°** → 재열림 **64.0°**. Drive 정상.
- **do_cut**: `can_cut` 8.2mm<10mm → True, 날 닫힘, 과실 **371.5mm 낙하**. 전 사이클 정상.

**API 에러(무해)**: 로그에 `Camera/RSD455 did not match any rigid bodies` 2줄 —
카메라가 §8대로 RigidBody 를 뺀 상태라 물리 텐서가 못 찾는 **기존** 경고(커터 무관).

**조정할 [4] 값 (GPU 눈으로)**:
- `_BLADE_OPEN_DEG=32`·날/힌지 치수 — 임의. 실제로 꽃자루를 감싸는지 렌더로 확인해 조정.
- `cut_tolerance=10mm` — 실제 파지 자세에선 과실중심↔힌지가 `cutter_offset_z`(45mm)만큼
  떨어진다. 이 기하에 맞춰 스윕하면 "커터 위치 정밀도 요구사항"([3])이 된다.

---

## 사용자 CAD 커터 지그 + D455 + 따는 모션 — GPU 검증 (2026-07-19)

**배경**: 프리미티브 커터(§가위형)를 버리고, 사용자가 FreeCAD 로 그린 실제 지그 CAD 를
로봇에 장착. `~/Downloads/build_harvest_eef_jig.py` 가 형상 기준(툴좌표: 원점=플랜지,
+Z=접근, +Y=커터/카메라쪽 위, +X=손가락). 세션 중 STL 을 1회 수정·재반입받아 재적용.

**자산 파이프라인** (`robots/cad_jig/*.usd`):
- STL 4개(jig/blade_dummy/servo_dummy/camera_dummy) → `omni.kit.asset_converter` USD 변환.
  변환기가 metersPerUnit≈0.01 스탬프 → `_CAD_SCALE=0.1` 로 실측 mm 복원(§8 토마토와 동일).
- 새 jig/blade STL(01:47) 재변환·교체 검증: 백업 대비 내용 diff 확인(크기 우연히 같아도 다름).
- servo_dummy/camera_dummy 는 그대로. camera_dummy 는 **D455 위치 로케이터**로만 참조 후 숨김.

**조립** (`robots/harvester.py`):
- `_add_cad_jig`: 커플러+지그+날+서보를 **그리퍼 base_link(움직이는 툴)의 자식**으로 부착.
  방향 = 툴0 + X−90°·Y−90°(사용자 렌더 확정). 월드 포즈를 base_link 로컬로 변환해 보존.
  ★ **중요 회귀 수정**: 이전엔 정적 루트(`/World/Harvester`)에 둬서 팔이 움직이면 지그가
  제자리에 남았다("허공 절단"). base_link 자식으로 바꿔 **팔과 한 몸으로 움직임**(옛
  `_add_cutter` 가 이미 grip_base 자식이었는데 CAD 전환 때 정적 루트로 되돌린 게 원인).
- `_add_camera_at`: D455 를 base_link 자식(빈 Xform + 자산 자식 구조, §8 op충돌 회피)으로.
  회전은 **로컬 rotateXYZ op 에 리터럴** = GUI 트랜스폼 패널 표시값과 1:1(관례 혼동 제거).
  현재 `_CAD_CAM_EULER=(0,-78,0)`, 위치=camera_dummy 월드중심→base_link 로컬.
  (교훈: 쿼터니언/`Gf.Rotation` 곱 순서·월드vs로컬 프레임 혼동으로 several 회 오검증. GUI 가
  진실. rotateXYZ 리터럴이 유일하게 관례 무관.)

**절단 기하 — "허공 자르기" 해결**:
- 기본 자세는 커터가 파지점 **아래**(거꾸로). **조인트4(wrist_1) +180°** 하면 노치가
  파지점 위(Δz≈+56mm)로 와 수확 자세가 됨(과실 잡고 줄기는 위로 노치 관통).
- 절단 = 식물 꽃자루 FixedJoint `jointEnabled=False`(§5.1 우회 아님, 물리 절삭은 범위 밖).

**따는 모션 데모** (`robots/pick_demo.py`, 신규): 조인트4+180 → 그리퍼 닫기 → 과실을 그리퍼에
FixedJoint 부착 → 절단(식물조인트 끊기) → 팔 들어올림 → **과실 Z 0.60→1.12 따라 올라옴**.
전 사이클 시각 확인.

**⚠ 내일 이어서 (오늘 미완, 사용자 지적)**:
- **트리거 없음**: 지금은 스크립트 타이밍. FSM(`can_cut`/`do_cut`)으로 조건구동해야.
- **상태 비영속**: 스크립트 안 돌면 과실 그냥 낙하 — FSM 노드가 파지/절단 관리해야.
- **마찰 파지 미검증(spike01)**: pick_demo 4단계 부착조인트는 연출. Robotiq 콜라이더가
  instanceable 프로토타입 안이라 순회로 안 잡힘 → instanceable 끄고 콜라이더 노출 후
  마찰·드라이브 스윕(진짜 파지 유지 검증)이 다음 큰 마일스톤.
- **접근 자세**: 조인트4+180 은 수동. 실제론 과실로 접근하는 IK/모션 필요.
- 카메라 위치·각도(0,-78,0)는 눈으로 조정 중 — 확정 아님.

## 가동날 서보 힌지 — 조인트 프레임 스케일 버그 수정 (2026-07-19, GPU 렌더 검증)

**증상**: wrist_1 +180° 수확자세에서 마젠타 가동날이 아래(과실 쪽)로 늘어져 토마토를
갈았다("블레이드로 토마토 다 갈거야?"). 충돌 문제로 의심됐으나 **축 문제**로 판명.

**원인 (측정으로 확정)**: 원본 blade_dummy·attach 직후 사본은 정상(수평, CAD+Y→월드 위).
그런데 드라이브가 돌리는 순간 날이 수직축이 아니라 수평축으로 90° 넘어감. 마젠타 사본에
CAD 스케일(≈0.001)이 붙어 있는데, 리볼루트 조인트 프레임을 스케일 섞인 행렬에서 한 번에
뽑아 **회전축이 X↔Y 로 꼬였던 것**. body0(팔)은 순수 강체라 안전, body1(사본)만 오염.

**수정** (`robots/harvester.py attach_blade_hinge`):
- `localPos1` = 스케일 **포함** 행렬로 (PhysX 가 지오메트리 좌표에 바디 스케일을 곱하므로.
  스케일 벗기면 피벗이 5cm 뜬다)
- `localRot1` = 스케일 **벗긴**(직교정규화 `_rigid`) 행렬로 (섞이면 축이 꼬인다)
- 각도 규약을 CAD build 스크립트대로 복원: **열림 0° ~ 닫힘 35°**(rest=CAD −35° 익스포트
  자세, +35°=CAD 0°=노치 전단). 이전 100/135° 는 축이 꼬인 상태에서 눈으로 맞춘 값.

**수확자세 축 매핑 실측** (diag: 기본 / wrist_1+180 / wrist_3+180):
- 기본자세: CAD+Y(서보축)→월드 (0,0,−1), 절단점 z = 파지점 −5.3cm (커터가 아래 = 뒤집힘)
- **wrist_1 +180°: CAD+Y→(0,0,+1), CAD+Z(접근)→수평, 절단점 = 파지점 +5.3cm — CAD 의도
  (접근 수평·커터 위·전단면=과실 위 23mm) 그대로.** 수확자세는 wrist+180 이 필수.
- 결과 렌더: 토마토=파지점(적도 파지), 날이 전단면(z 0.653)에서 수평 스윙, 과실 윗면
  (0.634)보다 1.9cm 위 → 과실 안 갈고 줄기만 절단. cut_demo/blade_hinge 데모 확인.

**정리**: 데모(blade_hinge/cut_demo/pick_demo/layout_view) → `temp/`(gitignore).
robots/ 에는 코어만: harvester, control, transporter, assets, stub_harvester, teleop.
teleop.py 신규 — MM+블레이드(Z/X 키) 키보드 텔레옵, ROS2 제어점 뼈대 주석 포함.
harvester 에 `move_blade(d)`/`blade_deg()` 추가(제어면 편입).

## ROS2 브리지 — graph.py 버그 2개 수정 + GPU 검증 (2026-07-19)

이전 세션(scratchpad iw_bridge.py, 휘발 직전 복구)에서 확인된 버그를 repo 에 반영:
1. `og.GraphRegistry().get_node_type` 은 Isaac 5.1 에 **없다** → `_pick()` 이
   AttributeError 를 삼키고 항상 "못 찾음". → **create-probe** 로 교체: 임시 그래프에
   노드를 실제로 만들고 prim 생성 여부로 판정(조용한 실패 방지), 끝나면 프로브 제거.
2. OnTick 실제 타입명은 `omni.graph.action.OnPlaybackTick` (isaacsim.core.nodes 엔 없음)
   → 후보 맨 앞에 추가.

**GPU 헤드리스 검증 통과** (env: LD_LIBRARY_PATH=<isaac>/exts/isaacsim.ros2.bridge/humble/lib
+ RMW_IMPLEMENTATION=rmw_fastrtps_cpp + ROS_DOMAIN_ID=108 — spikes/06 docstring):
- `ros/graph.py build()` — 4개 노드 전부 생성 성공. **제네릭 ROS2Subscriber/ROS2Publisher
  (std_msgs/String) 가 isaacsim.ros2.bridge 에 실존** → String JSON 프로토콜 경로 유효.
- `spikes/06_iwhub_bridge.py`(신규 — scratchpad 검증본 이관) — iw.hub 스폰(settings 의
  iwhub 경로 resolve) + JointState 그래프(Sub→ArticulationController, Pub, Clock) 생성,
  /iwhub/joint_states 발행·/iwhub/joint_command 수신 개통, 600스텝 무사고.
- ⚠ 남은 것: ROS2 터미널에서 echo/pub 실짜 확인(5A 루프백), 이후 dev 머신에서 5B.

**settings.py 추가** (물류 루프 확정 2026-07-19, Nucleus listing 실측):
- `iwhub` /Isaac/Robots/Idealworks/iwhub/iw_hub.usd (1431×659×231mm, 페이로드 1000kg,
  차동 left/right_wheel_joint + lift_joint)
- `klt_bin` small_KLT.usd (198×297×146) / `pallet` pallet.usd (1213×802×143) /
  `pallet_holder` (창고 랙 거치대)

## spike 01 (마찰 파지) — ✅ 성립 (2026-07-19, 마지막 남은 스파이크)

**결과 (5×5 스윕: μ 0.3~1.1 × 파지력 2~40 N, 판 2장 + 토마토 68.7mm/120g convexHull):**

```
            2N     5N    10N    20N    40N
mu 0.3   57.6cm   유지   유지   유지   유지     ← 유일한 실패 = 산술 경계 그대로
mu 0.5     유지   유지   유지   유지   유지        (2·0.3·2=1.2N ≈ mg=1.18N)
mu 0.7~1.1 유지   유지   유지   유지   유지
```

- **실측 경계 = 산술 경계 (2μF ≥ mg)** — 솔버 페널티 사실상 0. convexHull 접촉으로도
  마찰 파지가 깨끗하게 성립한다. 절단 순간(kinematic→dynamic) 이후 정적 유지 + 30cm
  리프트 추종까지 확인.
- **파지력 창: 2 N ≤ F ≤ 18 N** (패드 2cm² × 90kPa 상한). μ 민감도 [3]:
  μ≥0.5 → 2N / μ=0.3 → 5N. **그리퍼 선정 기준: 2N 이상 + 패드 0.2cm² 이상.**
- settings.py fruit_static_friction 주석에 실측 반영. Robotiq 2F-85(최대 235N, 패드
  ~평방cm급)는 창 안에 여유롭게 들어온다.

**과정에서 잡은 리그 버그 2개 (§8 기록):**
1. 프리즈매틱 조인트 localPos0 에 스케일 나눗셈 핵(−0.12/0.02=−6) → PhysX 스냅으로
   손가락이 날아가 25조합 전패가 "마찰 불성립"처럼 보였다. → 스케일 op 제거 + 프레임을
   현재 상대 포즈로 앵커. **전 칸이 μ·힘과 무관하게 동일하면 물리가 아니라 리그 버그다.**
2. 과실을 처음부터 dynamic 으로 낙하시켜 손가락 닫힘과 경주 → 실제 수확(§5.3 잡고
   자른다)에 없는 조건. → 매달림(kinematic) 중 물고, 문 뒤 dynamic 전환(=절단, §5.1
   합법 모델)으로 시퀀스 교정.
3. (부수) 토마토 USD 참조 prim 에 AddTranslateOp 이중생성 — §8 2026-07-18 예고 그대로
   spike01 에서 발생 → set_translate/set_scale 재사용 헬퍼로 교체.

**스파이크 전체 현황: 01✅ 02✅ 03✅ 04✅ 05✅ 06✅ — 전부 완료.**

## 통합 배선 — 맵 + 로봇 3대 + ROS2 제어 (2026-07-19, GPU 헤드리스 검증)

**main.py 개편**: 씬(GreenhouseTask) + 로봇 3대 스폰 + 로봇별 ROS2 브리지.
구식 HarvestBridge(액션 JSON 프로토콜) 배선은 제거 — 사용자 결정(2026-07-19):
조인트 레벨 ROS2 직접 제어로 간다. ros/protocol·dispatcher·harvest_bridge 파일은
디스크에 남김(미사용).

**신규**:
- `robots/iwhub.py` — iw.hub 스폰 클래스 (DOF: left/right_wheel_joint 속도,
  lift_joint 위치)
- `ros/robot_bridge.py` — 로봇별 JointState 브리지(build_joint_bridge) + /clock +
  제네릭 String 구독(build_string_sub·StringPoller). spikes/06 검증 타입명 사용.

**토픽 구성 (domain 108)**:
- `/{ns}/joint_command`(JointState 수신) + `/{ns}/joint_states`(발행),
  ns = harvester_0 / forklift_0 / iwhub_0
- `/harvester_0/cmd`(String JSON): `{"blade":0~35}` 가동날(아티큘레이션 밖 리볼루트),
  `{"base":[x,y,yaw]}` 키네마틱 베이스(위치드라이브 무시 → 텔레포트, 2026-07-18 실측).
  main.py 루프가 폴링해 mm.set_blade_deg / set_joint_positions 로 적용.

**헤드리스 검증 통과** (env 레시피, 300초): 씬 6섹터·542과실 + MM(가동날 0~35°) +
지게차B(승강 2.0m) + iw.hub 스폰, 그래프 5개 생성, 트레이스백 0. 로봇 배치는 온실
앞마당(y=−12) 임시 — 물류 동선 확정 후 조정.

**다음**: 5A 루프백(ROS2 터미널에서 ros2 topic pub 로 실제 구동 — main.py docstring 에
명령 예시) → 팔레트+KLT 세트를 iw.hub 데크에 스폰 → 5B 다중PC(훈련장).

## "커터 다 어디갔어" — 비원점 스폰에서 MM 분해 버그 수정 (2026-07-19, 렌더 검증)

**증상**: main.py(로봇을 온실 앞마당 (0,−12) 에 스폰)에서 그리퍼가 바닥에 떨어져
뒹굴고 지그·커터·카메라가 흩어짐. 지금까지 데모는 전부 (0,0,0) 스폰이라 안 드러났다.

**원인**: `harvester._attach_gripper` 가 그리퍼 컨테이너 배치를 월드 기준(`G⁻¹·S`)으로
계산해 **로컬 op 에 그대로** 썼다 — 부모(하베스터 루트)가 원점일 때만 로컬=월드.
(0,−12) 스폰에서 그리퍼가 12m 밖에 배치됐고, ee_joint 프레임을 배치 후 실측으로 다시
쓰는 §8 수정이 오배치를 "일관되게" 묶어 **에러 없이 조용히** 분해됐다.

**수정**: 컨테이너 로컬 = `C·G⁻¹·S·P⁻¹` (row-vector; C=컨테이너 l2w, G=base_link l2w,
S=목표 소켓, P=부모 l2w). CLAUDE.md §8 기록. 아울러 main.py 에 수확자세(wrist_1 +180°,
default_state — Play/Stop 유지) 스폰 추가: 커터·지그가 파지점 위로 오는 자세.

**검증**: (0,−12) 스폰 렌더 — MM 전신 온전(베이스+팔+그리퍼+지그+마젠타 날+D455,
수확자세). main.py 통합 헤드리스 재통과(씬 542과실+3대+브리지 5그래프, 240초 무사고).
Lesson: **원점 스폰으로만 검증된 조립은 검증이 아니다 — 비원점 1회가 회귀 테스트다.**

## 경고 로그 분류 + --quiet (2026-07-19)

main.py 실행 로그의 경고 4종 판정:
1. `metricsAssembler SetEditTarget` 수백 줄 — CAD 지그 USD(metersPerUnit=0.01) 참조 시
   Kit 버그성 스팸. 조립 순간에만. 무해. → **`--quiet` 플래그 추가**(omni.usd 채널을
   error 로 — 스팸 0건 확인). 기본은 켜 둠(§8: 경고=증거. 다른 USD 경고도 숨기므로 옵트인).
2. Robotiq `invalid inertia tensor / negative mass` — 시각전용 링크에 콜라이더 없어
   PhysX 근사. 전 검증이 이 상태로 통과. 무해.
3. `RSD455 did not match any rigid bodies` — D455 물리 잔재 제거를 강화(PhysX API·내부
   조인트·아티큘레이션루트까지)했으나 **메시지는 그대로** — 텐서 뷰 쪽 원인 미상,
   기능 영향 없음(카메라 시각·물리 정상, 3회/리셋). 무해로 문서화.
4. `ServoJoint disjointed` 1회 — Play 리셋 시 블레이드 자유축 변위(§8 무해 케이스).

main.py 루프 지속성 재확인: 90초 풀가동 후 타임아웃 킬(=무한 루프 정상).

## 팔레트+KLT 세트 물리 리프트 (2026-07-20, GPU 실측 ✅)

**검증**: 스파이크 06(`temp/spikes/06_pallet_lift.py`)에서 지게차가 팔레트+KLT 세트를
포크로 떠서 든다 — 사용자 GPU 확인("이상없는거같아"). 팔레트와 KLT 가 같은 Δ 로 같이 상승.

**물리 세팅**:
- **팔레트** = 다이내믹 강체. 질량 25kg[1 EPAL EUR](MassAPI 직접, 밀도 아님 — 속 빈 구조),
  마찰 0.5/0.35[3 목재-강철], 콜라이더 **convexDecomposition**(64hull·500k복셀). 단일
  convexHull 은 포크 슬롯을 메워 포크가 못 들어감 → 분해근사로 슬롯 살림(스파이크 06 핵심).
  값은 `settings.PalletPhysicsConfig`(§5.7).
- **KLT** = 팔레트 prim 의 **자식**으로 넣어 PhysX 가 팔레트 강체 하나에 **흡수**. KLT 에
  강체(RigidBodyAPI)를 **안 줌** → 따로 놀며 튕기는 '개판' 없이 계층으로 같이 이동.
  **고정조인트 불필요**(계층 결합이 더 견고). §5.1 안 어김 — 이관을 때우는 게 아니라
  표준 운반장비 조립(그리퍼 장착과 같은 성격, 사용자 결정 2026-07-20).

**적용 범위**: `scene/warehouse.py::_place_pallet` 의 랙 세트도 동일 결합(KLT=팔레트 자식,
강체 흡수)이라 같은 거동. → 지게차가 랙 세트를 통째로 들어올림이 검증된 셈.

**남은 것**: MM 이 토마토 담는 '활성 세트'의 KLT 는 안이 비어야 하므로(담김) 벽에 오목
콜라이더(convexDecomposition)를 따로 줄 것 — 지금 세트는 빈 장식이라 시각 라이드-어롱만.
바퀴 반지름·트랙폭 등 iw.hub Nav2 치수는 아직 [4](GPU 실측 전).

**부수 작업(미검증, 문법만)**: 온실 유리벽 static 콜라이더 추가(로봇 관통 방지) /
iw.hub Nav2 브리지 스캐폴딩(`tools/nav2_node_probe.py` create-probe + `ros/robot_bridge.py`
빌더 4개 + main.py `--nav-*` 플래그) — 노드 타입명 GPU create-probe 확정 대기.
