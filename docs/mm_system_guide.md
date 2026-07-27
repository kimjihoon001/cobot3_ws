# MM 파트 현재 구조·코드 학습·발표 가이드

> 기준: `multiple_harvest` 브랜치의 2026-07-26 작업 트리
> 기준 HEAD: `bd06cf3` + 현재 작업 트리 변경
> 대상: MM 담당자, 비전/MoveIt 발표자, IW 연동 담당자

## 1. 한 문장 설명

MM은 Ridgeback 위 M0617과 eye-in-hand D455, 3축 스쿱/커터를 이용해 토마토를
검출·수확하고 IW의 빈 KLT에 기본 2개를 놓은 뒤, IW가 창고로 갈 수 있도록 피항한다.

현재 MM은 단순히 “로봇팔”만 뜻하지 않는다. 아래 네 층을 합친 시스템이다.

1. Ridgeback 베이스 Nav2
2. RGB-D/YOLO와 시뮬레이션 GT 매칭
3. MoveIt2 기반 M0617 모션과 스쿱/커터 제어
4. 수확 횟수·IW FOLLOW·피항을 조정하는 코디네이터

## 2. 발표용 전체 흐름

```text
MM Nav2 수확 위치 이동
  → HOME
  → BED_VIEW
  → D455 + YOLO tomato 검출
  → 시뮬 GT 토마토와 매칭
  → APPROACH → PREGRASP → GRASP
  → 스쿱 닫기 → 파지 검증
  → 커터 절단
  → RETRACT
  → IW FOLLOW 및 빈 KLT pose 확인
  → BASKET 접근 → release → retract
  → 1회차: HOME 왕복 없이 BED_VIEW로 돌아가 다음 수확
  → 2회차: HOME 복귀 후 PREPARE_FORKLIFT
  → IW의 피항 요청 수신
  → MM이 단계형 안전 피항
  → IW 통과 허가
  → 팔레트 교환 후 resume_harvest 수신
```

기본 적재 목표는 `place_target_count=1`이다(real_main 기준 — 수확 1회 후 바로
하역). 2 이상으로 올리면 1회차 플레이스 후 접힌 팔을 HOME까지 왕복시키지 않고
`joint_1` 중심으로 BED_VIEW로 돌아가 다음 수확을 이어간다(multiple_harvest 브랜치
기본값 2, 앞열 빈 KLT 2칸 기준).

## 3. 코드 지도

### 3.1 가장 먼저 읽을 파일

| 순서 | 파일 | 읽을 이유 |
|---:|---|---|
| 1 | `src/smartfarm/harvest_vision/harvest_vision/harvest_fsm_node.py` | MM Nav2·수확 횟수·IW FOLLOW/피항의 상위 FSM |
| 2 | `src/smartfarm/harvest_vision/harvest_vision/manipulator_target_node.py` | 토마토 수확과 KLT 플레이스의 상세 FSM |
| 3 | `src/smartfarm/mm_moveit/scripts/mm_motion_bridge.py` | JSON 모션 명령을 MoveIt goal로 변환 |
| 4 | `src/smartfarm/harvest_vision/harvest_vision/vision_node.py` | RGB-D/YOLO 목표 생성 |
| 5 | `isaacpjt/mm.py` | MM articulation, 카메라, 도구 상태와 Isaac 이벤트 |
| 6 | `src/smartfarm/mm_moveit/urdf/mm_mm.urdf.xacro` | Ridgeback–M0617–스쿱 전체 링크 구조 |
| 7 | `src/smartfarm/mm_moveit/srdf/mm_mm.srdf` | planning group, HOME, 허용 충돌 |
| 8 | `src/smartfarm/mm_moveit/launch/auto_nav_harvest.launch.py` | MM 전체 ROS 기동 구조 |

### 3.2 노드 책임

```text
vision_node
  └─ RGB-D → tomato detection / approach target

manipulator_target_node
  ├─ 목표 안정화·workspace 검사
  ├─ 수확/절단/플레이스 상세 FSM
  └─ /harvester_0/cmd JSON 생성

mm_motion_bridge
  ├─ 다중 seed IK
  ├─ OMPL / Pilz 모션 선택
  └─ MoveGroup 실행 결과 반환

harvest_fsm_node
  ├─ MM Nav2 도착 gate
  ├─ 2회 플레이스 카운트
  ├─ IW FOLLOW/PREPARE_FORKLIFT
  └─ MM 피항·수확 재개
```

## 4. 모델과 좌표계

| 항목 | 현재 값 |
|---|---|
| 실행 플래그 / namespace | `--mm` / `harvester_0` |
| Isaac root | `/World/Harvester` |
| 초기 pose | `(0.0, -12.0)` |
| 베이스 | Ridgeback |
| 팔 | Doosan M0617, 6축 |
| 도구 | 동축 3축 1/4구 스쿱 + 커터 |
| 카메라 | 스쿱 장착 D455 |
| MoveIt group | `mm_manipulator` |
| planning chain | `base_link → harvest_tcp` |

프레임 혼동이 가장 흔한 오류다.

- `mm_base`: 이동 베이스 기준
- `base_link`: M0617 planning 기준, `mm_base`보다 약 `0.30 m` 위
- `harvest_tcp`: 스쿱 작업점
- 카메라 optical frame: 팔 끝 D455 기준

카메라/토마토/KLT pose를 변환할 때 `mm_base`와 `base_link`를 섞으면 Z가 약
`0.30 m` 어긋난다.

## 5. MoveIt 구조

```text
/harvester_0/cmd JSON
  → mm_motion_bridge
  → compute_ik(여러 seed 중 관절 변화가 작은 해 선택)
  → move_group
  → arm_controller/FollowJointTrajectory
  → topic_based_ros2_control
  → /harvester_0/joint_command
  → Isaac M0617
```

스쿱 세 관절은 arm planning group 밖이다. `gripper_controller`가 별도로 position
명령을 보내며 Isaac은 실제 close/cut/release 이벤트를 상태 토픽으로 돌려준다.

### HOME과 도구 상태

```text
HOME joint_1..6:
[+180°, +60°, -120°, 0°, -30°, +90°]

SCOOP OPEN:   (0°, -90°, -180°)
SCOOP CLOSED: (0°,   0°,    0°)
CUT:          (0°,   0°,  +50°)
```

커터는 구동 오차를 고려해 약 `40°` 이상 도달하면 절단 완료로 판단한다.

### 모션 선택

| 단계 | 주 사용 방식 |
|---|---|
| APPROACH / BASKET_APPROACH | OMPL |
| HOME / 관절 자세 전환 | PTP 성격의 관절 목표 |
| 직선 접근·후퇴 | Pilz LIN |
| 과실을 감싸는 진입/후퇴 | Pilz CIRC |

`rmp_target`, `rmp_home`이라는 JSON 키는 호환을 위해 남은 이름이다. 현재 실행기는
RMPflow가 아니라 `mm_motion_bridge`를 통한 MoveIt2다.

## 6. 비전과 목표 선택

카메라 입력:

```text
/harvester_0/rgb
/harvester_0/depth
/harvester_0/camera_info
```

| 항목 | 기본값 |
|---|---|
| 원거리 detector | `finetuned_far.pt`, confidence `0.45` |
| 근거리 quality model | `finetuned_near.pt`, confidence `0.55` |
| quality 재분류 | 기본 `use_quality_model=false` |
| 목표 안정화 | 5프레임, 프레임 간 `≤ 0.01 m` |
| 제어 목표 | 기본적으로 시뮬 GT와 비전 검출을 매칭 |

`vision_node`가 2D box와 depth로 3D 후보를 만들고,
`manipulator_target_node`가 workspace·시간·점프 제한을 검사한다. 실제 통합 안정성을
위해 검출점과 가까운 시뮬레이션 tomato GT를 최종 제어 목표로 사용할 수 있다.

## 7. 수확·플레이스 FSM

대표 상태:

```text
WAIT_TARGET / WAIT_SIM_MATCH
→ APPROACH
→ PREGRASP
→ GRASP
→ CAPTURE_TRIM / GRIPPER_CLOSING
→ GRASP_VERIFY
→ CUTTING / BLADE_OPENING
→ RETRACT_CIRC
→ RETRACT_LIN
→ PRE_PLACE_BED_VIEW
→ BASKET_AZIMUTH_ALIGN
→ BASKET_APPROACH
→ BASKET_WRIST_ROTATE
→ PLACE_RELEASING
→ BASKET_RETRACT
→ POST_PLACE_BED_VIEW 또는 GO_HOME
```

### Workspace

```text
harvest min = [0.15, -1.05, 0.15]
harvest max = [1.25,  1.05, 1.80]
max XY radius = 1.50 m
basket max reach = 1.45 m
```

목표가 workspace 밖이면 무리하게 계획하지 않고 재배치 요청 또는 실패 경로로 간다.

### 다회 플레이스 (real_main 기본값 1 = 비활성)

- real_main 기본 `place_target_count=1` → 아래 경로는 타지 않고 1회 플레이스 후 바로 하역
- 2 이상일 때: 1회차 후 `place_more_pending=true`
- `POST_PLACE_BED_VIEW`에서 HOME 명령 없이 다음 BED_VIEW
- 2회차 후 HOME 복귀
- 목표 개수를 채우면 `/iw/mission=PREPARE_FORKLIFT`
- 1개를 실은 뒤 다음 수확이 최종 실패해도 부분 적재로 출발
- 0개면 빈 IW를 창고로 보내지 않음

IW는 `/iw/basket/empty_slot_pose`로 첫 번째와 두 번째 KLT를 다르게 제공하고,
MM은 스쿱을 열 때 `/harvester_0/scoop/release_debug`를 발생시켜 IW가 해당 슬롯을
사용 완료로 기록하게 한다.

## 8. MM 베이스와 피항

MM 베이스는 Nav2로 수확 위치까지 이동한다. 팔이 작업 중일 때 베이스가 움직이지 않도록
이동–팔 상호잠금이 필요하다.

피항은 IW가 팔레트 교환을 위해 창고로 나갈 때 수행한다.

1. IW가 `/iw/mm_yield_request=true`
2. MM FSM이 플레이스 이전 요청인지, 재시작 복구 신호인지 상태 확인
3. 팔을 안전 자세로 정리
4. costmap과 footprint로 피항 후보 pose 검사
5. 단계형 Nav2 이동
6. 완료 시 `/iw/mm_yield_complete=true`
7. IW 복귀 후 `/iw/resume_harvest=true`를 받아 수확 재개

## 9. 핵심 ROS 인터페이스

| 방향 | 이름 | 타입 | 의미 |
|---|---|---|---|
| Isaac → Vision | `/harvester_0/rgb`, `depth`, `camera_info` | Image/CameraInfo | D455 입력 |
| Vision → Manipulator | `vision/approach_target` | `PoseStamped` | 3D 접근 목표 |
| Vision → Manipulator | `vision/target_class` | `String` | tomato/ripe/spoiled 등 |
| FSM → Manipulator | `harvest_test/enable` | `Bool` | 수확 gate |
| FSM → Manipulator | basket target topic | `PoseStamped` | base frame으로 변환된 KLT 목표 |
| Manipulator → Bridge/Isaac | `/harvester_0/cmd` | JSON String | 팔·도구 명령 |
| Bridge → FSM | `/harvester_0/pipeline_status` | `String` | MoveIt 성공/실패 |
| Isaac → FSM | `/harvester_0/status` | `String` | grasp/cut/tool 상태 |
| MoveIt HW → Isaac | `/harvester_0/joint_command` | `JointState` | 팔·스쿱 position |
| Isaac → ros2_control | `/harvester_0/hw_joint_states` | `JointState` | 실제 관절 |
| FSM → IW | `/iw/mission` | `String` | FOLLOW/PREPARE_FORKLIFT |
| IW → MM | `/iw/basket/empty_slot_pose` | `PoseStamped` | 빈 KLT |
| IW ↔ MM | `/iw/mm_yield_request`, `/iw/mm_yield_complete` | `Bool` | 피항 handshake |

`hw_joint_states`와 MoveIt용 `joint_states`는 분리한다. Isaac의 dummy base 관절과
MoveIt 관절 상태가 같은 토픽에서 충돌하는 것을 막기 위해서다.

## 10. 실행과 확인

Isaac:

```bash
cd /home/rokey/cobot3_ws/isaacpjt
isaac_python main.py --mm --iw --fork --nav --camera
```

MM 단독 ROS:

```bash
cd /home/rokey/cobot3_ws
source install/setup.bash
ros2 launch mm_moveit auto_nav_harvest.launch.py
```

3로봇 통합 ROS:

```bash
ros2 launch harvest_vision smartfarm_integration.launch.py
```

확인할 핵심:

```bash
ros2 topic echo /harvester_0/pipeline_status
ros2 topic echo /iw/basket/empty_slot_pose
ros2 topic echo /iw/mission
ros2 topic echo /iw/mm_yield_request
ros2 control list_controllers -c /harvester_0/controller_manager
```

## 11. 담당자가 코드 리뷰에서 확인할 불변조건

- Isaac, ros2_control, SRDF의 HOME 관절값이 같아야 한다.
- `base_link`와 `mm_base` 좌표를 섞지 않는다.
- 수확 시퀀스가 활성화된 동안 목표 토마토를 새 검출로 바꾸지 않는다.
- 플레이스는 실제 `/iw/basket/empty_slot_pose`가 있을 때만 시작한다.
- 1회차 플레이스 뒤에는 `PREPARE_FORKLIFT`를 보내지 않는다.
- 2회차 또는 부분 적재 종료에서만 IW 출발을 요청한다.
- 0개 적재 실패는 IW 창고 출발을 만들지 않는다.
- 피항 완료 전 IW가 출발하지 않도록 handshake를 유지한다.
- 스쿱 동축 셸의 허용 충돌 행렬을 임의로 제거하지 않는다.

## 12. 현재 검증 상태와 위험

단위 테스트가 있는 영역:

- 2회 플레이스 카운트와 1회차 HOME 생략
- 부분 적재 출발
- MM 피항 단계
- MoveIt bridge의 주요 명령 변환

실제 Isaac 통합에서 재확인할 영역:

- 사전 적재된 KLT 위로 release하지 않는지
- 두 번의 release가 서로 다른 앞열 KLT에 들어가는지
- 1회차 후 BED_VIEW 복귀 시 링크/과실 충돌이 없는지
- 두 번째 과실 실패 시 부분 적재 출발이 통합 상태와 맞는지
- 코디네이터만 재시작했을 때 적재 횟수 복구 의미

기존 테스트 하나는 `_iw_full=false`일 때 피항 요청을 무시해야 한다는 옛 가정과,
현재의 “코디네이터 재시작 복구 신호” 해석이 충돌한 기록이 있다. 발표 전에는 정상
흐름뿐 아니라 재시작 정책도 팀에서 한 번 확정하는 것이 좋다.

## 13. 발표용 핵심 제어기술

| 핵심 기술 | 제어 대상과 방식 | 발표 핵심 문장 |
|---|---|---|
| 계층형 FSM | 상위 코디네이터가 Nav2·적재 횟수·IW 연동을, 상세 FSM이 수확·절단·플레이스를 제어 | “이동, 조작, 물류 협업을 서로 다른 상태 계층으로 분리했습니다.” |
| 비전–GT 목표 융합 | RGB-D/YOLO 후보를 안정화한 뒤 가까운 시뮬 GT와 매칭하고 workspace·점프 제한 적용 | “인식은 비전으로 시작하되 시뮬레이션 제어 목표는 GT 매칭으로 안정성을 높였습니다.” |
| 다중 seed IK와 혼합 모션 계획 | 관절 변화가 작은 IK 해를 고르고 OMPL·PTP·LIN·CIRC를 구간 특성에 맞게 선택 | “전 구간에 한 planner를 강제하지 않고 장애물 회피와 작업 궤적 요구를 분리했습니다.” |
| 도구 이벤트 폐루프 | 스쿱 close, grasp verify, cut, release를 Isaac 상태 피드백으로 확인 | “관절 명령을 보낸 사실이 아니라 파지·절단 이벤트의 실제 완료를 확인합니다.” |
| 좌표계·workspace gate | 카메라 목표를 planning frame으로 변환하고 범위 밖 목표는 계획하지 않음 | “좌표 변환 오류와 도달 불가능 목표를 모션 계획 전에 차단합니다.” |
| 다회 적재·피항 handshake | KLT 슬롯 사용 이력, 적재 개수, MM 피항 완료를 IW와 토픽으로 동기화 | “수확 횟수와 두 로봇의 통로 점유를 명시적인 상태 신호로 조정합니다.” |

발표에서는 **인식 목표 생성 → MoveIt 모션 선택 → 도구 피드백 → 다회 적재와 협업**
순서로 설명하면 MM의 제어 파이프라인이 명확하다.

## 14. 발표자가 답할 수 있어야 하는 질문과 답

### Q1. 왜 비전 검출만 쓰지 않고 시뮬 GT와 매칭하는가?

RGB-D/YOLO만으로 만든 3D 점에는 depth 노이즈와 프레임 간 흔들림이 있어 정밀한
그립·절단 목표로 바로 쓰기 어렵다. 현재 시뮬레이션에서는 비전 후보와 가까운 GT
토마토를 매칭해 인식 파이프라인은 검증하면서도 제어 목표의 안정성을 높인다. 다만
GT는 시뮬레이션 보조 수단이므로 실제 시스템에서는 보정된 3D 추정과 추적기로 대체해야
한다.

### Q2. `rmp_target`이라는 이름인데 왜 실제 실행은 MoveIt인가?

명령 키는 이전 RMPflow 기반 인터페이스와의 호환을 위해 유지됐지만 현재 수신자는
`mm_motion_bridge`이고, 이 노드가 IK 계산과 MoveGroup 실행을 수행한다. 따라서 이름은
레거시 프로토콜이고 실제 제어 백엔드는 MoveIt2다.

### Q3. OMPL, LIN, CIRC를 각 단계에 나눠 쓰는 이유는 무엇인가?

OMPL은 장애물을 피해 넓은 공간에서 접근 자세를 찾는 데 적합하고, LIN은 최종 접근과
후퇴에서 TCP의 직선 경로를 보장한다. CIRC는 과실을 감싸거나 주변 구조물을 피하는
곡선 경로에 적합하다. HOME 같은 자세 전환은 TCP 경로보다 관절 자세가 중요하므로
PTP 성격의 관절 목표를 사용한다.

### Q4. 1회차 플레이스 뒤 HOME을 생략해도 안전한 이유는 무엇인가?

release 후 먼저 `BASKET_RETRACT`로 KLT와 충분히 이격하고, 충돌이 검증된
`POST_PLACE_BED_VIEW` 경로로 다음 관측 자세에 이동하기 때문이다. 즉 안전 후퇴는
생략하지 않고 불필요한 HOME 왕복만 줄인다. 이 경로는 2회 이상 적재할 때만 활성화되며
마지막 플레이스 뒤에는 HOME으로 복귀한다.

### Q5. 두 KLT 슬롯을 MM과 IW가 어떻게 동기화하는가?

IW가 현재 팔레트의 앞열 미사용 슬롯 중 하나를 골라
`/iw/basket/empty_slot_pose`로 제공하고, MM은 그 pose에 release한다. release 시
`/harvester_0/scoop/release_debug` 이벤트를 보내면 IW가 해당 슬롯을 사용 완료로
기록해 다음에는 다른 슬롯을 제공한다. 새 팔레트가 deck에 결속된 뒤에만 사용 이력을
초기화한다.

### Q6. 수확 실패 시 부분 적재와 빈 IW를 어떻게 구분하는가?

성공한 release 횟수를 별도로 누적한다. 목표 개수 전에 다음 수확이 최종 실패해도
누적값이 1 이상이면 부분 적재로 보고 `PREPARE_FORKLIFT`로 진행할 수 있지만, 누적값이
0이면 빈 팔레트를 창고로 보내지 않고 실패·재시도 경로를 유지한다.

### Q7. IW 출발 전 MM 피항 handshake가 필요한 이유는 무엇인가?

MM과 IW는 수확 위치와 창고 진입 통로를 공유하므로 MM 베이스나 펼쳐진 팔이 남아
있으면 경로가 겹칠 수 있다. IW가 요청만 보내고 바로 출발하지 않고, MM이 팔을
안전 자세로 정리해 Nav2 피항을 완료한 뒤 `/iw/mm_yield_complete`를 보낼 때까지
기다려 두 로봇의 동시 점유를 막는다.
