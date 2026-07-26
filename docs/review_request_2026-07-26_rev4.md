# 검수 요청 rev4 — 데크 안착 폐루프 + 릴리즈 이벤트 분리

요청 2026-07-26 / 대상: Codex / 상태: **구현 완료, 시뮬 미검증**

## 0. 검수 범위

rev3 이후 새로 한 것만. 앞선 회차(노드 정리, 2회 픽앤플레이스, 슬롯 초기화)는
이미 조건부 통과를 받았으니 다시 볼 필요 없다.

이번 회차에 들어온 입력 3가지:
1. 2차 검수 지시 — 폐루프 방향 승인, `pallet_min_z` 필수, `forward_offset` 일관성, 권장 초기값
2. 2차 검수가 발견한 결함 — `release_debug`에 유효 메시지 0건
3. 사용자 지적 — **파지 검증은 판정 근거로 못 쓴다** ("토마토를 픽해도 계속 실패가 뜬다")

---

## 1. 근거 (bag `sim_diag_20260726_223158`)

**폐루프 쪽**

| 값 | 실측 |
|---|---|
| `pallet_position.z` (강체 **원점**) | 0.339369 |
| `pallet_target_position[2]` (= 게이트 `support_z`) | 0.284534 |
| 원점 기준 차이 | +0.054835 m |
| `pallet_rise − expected_rise` | 5.5 µm |
| 최종 `lift_joint` | 0.13950 (개루프 목표 0.146805) |

**확정된 것**: 리프트는 명령대로 움직였고(추종 오차 5.5 µm), 그런데도 게이트가
거부했다 → |z_error| > 0.025. 즉 개루프 목표 높이로는 안착 판정을 통과하지 못한다.

**확정 못 한 것**: 게이트 기준 잔차의 **크기**. 게이트는 팔레트 bbox 최저점으로
`z_error`를 재는데 그 bag에는 `pallet_min_z` 필드가 없었다. 위 +0.054835 m는
강체 **원점** 기준이라 피벗 오프셋만큼 다른 양이다. 실제 z_error 값은
`pallet_min_z`를 발행하기 시작한 다음 실행에서 확인해야 한다.

**릴리즈 쪽**

`/harvester_0/scoop/release_debug` **12093건 전부 빈 문자열, 유효 0건.**
→ IW 슬롯 배정기가 적재를 한 건도 못 셈 → 2회 플레이스가 **같은 KLT**로 들어갔다.
(이번 실행의 2개 적재는 실제로 한 칸에 겹쳐 있다)

---

## 2. 변경 목록

### A. 릴리즈 이벤트 판정 기준 교체 (사용자 지적 반영)

| 파일 | 변경 |
|---|---|
| `isaacpjt/mm.py` | `_log_release_offset` → `_publish_scoop_release(reason)`. 이벤트와 진단 분리 |
| `src/smartfarm/harvest_vision/harvest_vision/manipulator_target_node.py:1312` | `PLACE_RELEASING` 개방에만 `"reason": "place"` 부착 |

- **판정 기준: 파지 검증 → 개방 이유.** 검증(`_verified_fruit_id`)은 진단용으로만 쓰고,
  `-1`이어도 이벤트는 발행한다(`fruit_id: null`).
- 좌표 조회 실패/예외에도 이벤트는 반드시 발행. 진단값만 `null`.
- `seq` 필드 추가 — `StringPoller.poll()`이 **직전과 같은 문자열이면 None을 반환**하므로,
  진단이 전부 null이면 두 번째 릴리즈를 놓친다.

`gripper: {closed: False}` 발행 지점 5곳을 전수 확인해 성격을 나눴다:

| 위치 | 성격 | 슬롯 소모 |
|---|---|---|
| 1312 | `PLACE_RELEASING` — 바구니에 실제로 놓음 | ✅ (유일) |
| 711 | 파지 전 접근 개방 | ❌ |
| 1802 / 1820 / 1832 / 1872 | 실패 시 부분 파지 해제 | ❌ |

### B. `pallet_min_z` + `forward_offset` 단일화 (2차 검수 지시)

| 파일 | 변경 |
|---|---|
| `isaacpjt/iw_dock.py` | `pallet_world_min_z()` 추가 — 게이트가 `z_error`를 재는 바로 그 값 |
| `isaacpjt/iw.py` | `warehouse_pallet_min_z()` 위임 (기존 `warehouse_deck_surface`와 같은 방식) |
| `isaacpjt/fork.py` | `pallet_min_z` 발행. `deck_forward_offset`를 지역변수로 한 번만 읽어 목표 계산·echo가 같은 값을 쓰게 함 |

### C. 데크 안착 폐루프 (2차 검수 승인 방향)

`src/smartfarm/warehouse_dock/warehouse_dock/fork_lift_node.py`

- `deck_place_lift_setpoint()` — 순수 함수(모듈 레벨). 테스트 대상
- `deck_place_lower` step kind + `_run_deck_place_lower()`
- `_place_initial_pallet_on_amr`: 기존 `slow_lower`(개루프) **유지**, 그 뒤에 삽입
- 파라미터: `deck_place_tolerance 0.010` / `max_correction 0.10` / `stable_sec 0.4` /
  `stale_sec 0.5` / `timeout 20.0` — **권장값 그대로**
- 관측값 없음·stale·offset echo 불일치 → 추측하지 않고 진행 거부.
  `pallet_min_z`가 없으면 **폴백 없이 실패**한다(3차 검수 반영 — 강체 원점으로
  폴백하면 피벗 오프셋만큼 정상 팔레트를 더 눌러버린다)
- 파라미터 유효성 검사 추가(3차 검수 반영). tolerance는 게이트 허용치 0.025를
  넘을 수 없다 — 넘으면 통과 못 할 높이에서 성공을 선언한다
- 내부 deadline이 제네릭 타임아웃보다 먼저 발동해 **수치와 함께** 실패:
  `z_meas / z_target / lift_cmd / lift_actual / offset_echo / offset_step / state_age`

### D. 테스트

| 파일 | 개수 | 내용 |
|---|---|---|
| `src/smartfarm/warehouse_dock/test/test_deck_place_closed_loop.py` | 8 (신규) | **합성 입력** 수렴, 속도 제한, 하한 클램프, 과압 시 상승, 기계 한계 |
| `isaacpjt/tests/test_scoop_release_event.py` | 9 (신규) | 검증 실패에도 발행, 좌표 없음/예외에도 발행, seq 구분, 접근/abort 개방 제외, abort 시 검증 id 폐기 |
| `isaacpjt/tests/conftest.py` | 신규 | Isaac 런타임 stub 공유 (두 테스트 파일 중복 제거) |
| `src/smartfarm/harvest_vision/test/test_basket_place_sequence.py` | 수정 1 | 릴리즈 명령 dict 정확 비교 → `reason` 포함 검증으로 갱신 |

**현재 상태**: warehouse_dock 8 passed / Isaac 27 passed / harvest_vision 34 passed, 1 failed
(기존 실패 `test_yield_does_not_start_before_place`, 이번 작업과 무관 — rev2에서 확인)

---

## 3. 특히 봐줬으면 하는 것

내가 판단으로 정한 것들이다. 틀렸을 수 있는 순서대로 적었다.

1. **`reason="place"` 단일 지점 전제가 맞나?**
   1312 외에 바구니에 과실이 들어가는 경로가 있으면 적재를 놓친다.
   반대로 `PLACE_RELEASING`까지 갔는데 스쿱이 비어 있으면 빈 칸을 헛소모한다.
   대안으로 `/harvester_0/manipulator/target_state`를 iw.py가 직접 구독하는 방법도 있었는데,
   "실제 스쿱이 열리는 물리 이벤트"에 붙이는 편이 낫다고 판단했다. 동의하나?

2. ~~`_verified_fruit_id` 소비 시점~~ — **3차 검수에서 지적받아 수정 완료.**
   place가 아닌 개방에서도 검증 id를 버린다(과실은 이미 스쿱을 떠났으므로).
   회귀 테스트 `test_non_place_open_discards_the_stale_verified_id`.

3. **폐루프 step 삽입 위치.**
   `slow_lower` → `deck_place_lower` → `wait(0.8, "supported on IW")` → `pallet_owner("deck")`
   순서로 넣었다. `wait 0.8`을 폐루프 **뒤**에 두는 게 맞나, 아니면 폐루프가 이미
   0.4 s stable을 요구하니 중복인가?

4. **실패 시 즉시 `_fail` vs 재시도.**
   폐루프가 20 s 안에 수렴 못 하면 사이클을 중단시킨다. `_take_next_pallet_to_amr`에
   이미 정렬 재시도(5회)가 있는데, 안착도 재시도 대상에 넣어야 하나?

5. ~~`pallet_min_z` 폴백~~ — **3차 검수에서 지적받아 제거 완료.**
   없으면 안전 실패한다.

6. **기존 테스트 수정이 적절한가.**
   `test_release_height_is_lowered_3cm...`가 릴리즈 명령 dict를 정확 비교하고 있어
   `reason` 추가로 깨졌다. 그 사실을 검증하는 쪽으로 바꿨는데(회귀 감지 목적),
   테스트 의도를 훼손한 건 아닌지 봐달라.

---

## 4. 이미 확인해서 재검토 불필요한 것

시간 아끼시라고 적는다. 전부 코드로 확인했다.

- **`_step_stable_since` 공유 안전.** `_run_lift`와 같은 필드를 쓰지만
  `_begin_step()`이 step 전환마다 `None`으로 리셋한다.
- **목표값 정합성.** `pallet_target_position[2]`와 게이트의 `support_z`는
  `iw.warehouse_deck_surface()` → `WarehouseDockController._deck_surface()`로
  **같은 함수 호출**이다. 폐루프가 게이트와 다른 목표를 좇지 않는다.
- **offset 검증 패턴.** `_run_handoff_align`이 쓰던 "echo와 step 값 차이 0.001 초과 시
  진행 거부"를 그대로 재사용했다. 새로 만든 규칙이 아니다.
- **개방 지점 전수.** `manipulator_target_node`의 `"gripper"` 발행 9곳 중
  open은 5곳이고 전부 위 표에 분류했다.

---

## 5. 재현 명령

```bash
# 빌드 + 테스트
source /opt/ros/humble/setup.bash
colcon build --packages-select harvest_vision warehouse_dock smartfarm_interfaces
source install/setup.bash
python3 -m pytest src/smartfarm/warehouse_dock/test/ -q      # 8 passed
python3 -m pytest src/smartfarm/harvest_vision/test/ -q      # 34 passed, 1 failed(기존)
cd isaacpjt && python3 -m pytest tests/ -q                   # 26 passed

# bag 근거 재확인 (39GB, sqlite 직접 조회)
cd diagnostics/bags/sim_diag_20260726_223158
# release_debug 유효 메시지 0건 확인 / handoff_state 잔차 확인
```

---

## 6. 미검증 — 시뮬에서만 판정 가능

여기서는 Isaac을 띄울 수 없다. **완료로 표시하지 말 것.**

1. `/harvester_0/scoop/release_debug`에 **내용 있는** 메시지가 나가는지
   (`[Scoop] release fruit_id=...` 로그)
2. `[IW Basket] KLT_3x 적재 완료 — 남은 빈 칸 1개` → 2회차가 **다른 칸**에 들어가는지
3. `데크 안착 폐루프: 잔차=...` 로그 후 `transfer Pallet_01 from fork to IW deck` 통과
4. 결속 후 IW 주행 시 팔레트가 따라가는지 (공중 결속 아님)
5. **게이트 기준 실제 z_error 값** — `pallet_min_z` 발행이 시작됐으니 이번엔
   확인 가능하다. 폐루프 로그의 `잔차=` 값이 그것이다.
   10 cm 하한에 걸리면 보정 범위 가정이 틀린 것이다
6. rev2~3의 미검증 항목(사전 적재 30개 안착 높이, 주행 추종, 지게차 리프트)은 그대로 남아 있음

---

## 7. 이전 지적 대응 대조

| 회차 | 지적 | 대응 |
|---|---|---|
| 2차 | 폐루프 방향 승인, 개루프 유지 | §2-C 그대로 구현 |
| 2차 | `pallet_min_z` 필수 | §2-B. 게이트와 **같은 함수**로 구현 하나만 유지 |
| 2차 | `forward_offset` 전 구간 동일 | §2-B. fork.py에서 지역변수 단일화 + ROS 측 echo 검증 |
| 2차 | 권장 초기값 | 그대로 파라미터화 (0.10 / 0.010 / 0.4 / 0.5 / 20) |
| 2차 | release 이벤트를 진단과 분리 | §2-A |
| 사용자 | 파지 검증은 무의미 | §2-A. 판정 기준을 개방 이유로 교체 |
| 3차 | 과거 bag의 `pallet_z`를 `pallet_min_z`로 취급한 것은 잘못 | §1 재작성. 확정/미확정 분리, 테스트는 합성 입력으로 교체 |
| 3차 | `pallet_min_z` 폴백 제거, 안전 실패 | §2-C |
| 3차 | 폐루프 파라미터 유효성 검사 | §2-C |
| 3차 | abort 시 `_verified_fruit_id` 초기화 | §2-A, 회귀 테스트 추가 |

---

## 8. 별건 — 문서 참조 깨짐 (사용자 확인 필요)

`docs/` 재편이 있었다(`*_current_branch_reference.md` 삭제 → `*_system_guide.md` 신규).
그래서 `docs/handoff_2026-07-26.md` §B2의 항목 9·10이 **삭제된 파일**을 가리킨다
(`docs/mm_current_branch_reference.md:38/70`).

새 `*_system_guide.md`에는 옛 이름(`nav_harvest_test_node` 등)이 없어 **내용상 문제는 없다.**
handoff 문서의 그 두 줄만 정리하면 되는데, 문서 재편은 사용자 작업이라
임의로 손대지 않았다. 정리할지 알려달라.
