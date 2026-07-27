# Forklift 파트 현재 구조·코드 학습·발표 가이드

> 기준: `multiple_harvest` 브랜치의 2026-07-26 작업 트리
> 기준 HEAD: `bd06cf3` + 현재 작업 트리 변경
> 대상: Forklift 담당자, 창고 물류 발표자, IW 물리 인계 담당자

## 1. 한 문장 설명

Forklift 파트는 IW가 가져온 적재 팔레트를 원래 랙으로 복귀시키고 다음 빈 팔레트를
IW 데크에 올린 뒤, IW가 다시 수확 위치로 갈 수 있도록 출발 허가를 주는 창고
팔레트 교환 시스템이다.

이 파트의 본질은 단순 웨이포인트 주행이 아니라 세 가지다.

1. 후륜 조향 지게차의 결정적 Step 상태기계
2. rack/fork/IW deck 사이 팔레트 물리 소유권의 원자적 전환
3. 실측 데크 기하와 팔레트 높이를 사용한 안전 gate·폐루프 안착

## 2. 발표용 전체 흐름

```text
IW 도크 정렬 완료
  → /forklift/start_cycle(inbound Pallet_n)
  → 최신 Isaac IW 물리 pose와 deck geometry 확인
  → IW canonical dock lock
  → Pallet_n 포크 삽입
  → owner: deck → fork
  → IW에서 후진
  → 랙 n번 위치로 이동
  → Pallet_n 원래 슬롯에 복귀
  → Pallet_(n+1)%6 랙에서 회수
  → IW 공통축 정렬
  → IW 데크 위로 개루프 저속 하강
  → pallet_min_z 기반 폐루프 정밀 안착
  → owner: fork → deck
  → /forklift/pallet_on_iw
  → 도크 lock 해제
  → /forklift/clear
  → IW 복귀 주행
```

서비스 요청은 Nav2의 AMCL pose도 포함하지만 지게차가 실제 인계에 사용하는 것은
`/forklift/handoff_state`의 최신 Isaac 물리 월드 pose다. map 보정 좌표와 실제 PhysX
접촉 좌표를 섞지 않기 위해서다.

## 3. 코드 지도

### 3.1 가장 먼저 읽을 파일

| 순서 | 파일 | 읽을 이유 |
|---:|---|---|
| 1 | `src/smartfarm/warehouse_dock/warehouse_dock/fork_lift_return_node.py` | 현재 반복 팔레트 교환 서비스와 상위 사이클 |
| 2 | `src/smartfarm/warehouse_dock/warehouse_dock/fork_lift_node.py` | 공통 Step 실행기, 랙 경로, 폐루프 안착 |
| 3 | `isaacpjt/fork.py` | 지게차 articulation과 pallet owner 명령의 물리 실행 |
| 4 | `isaacpjt/iw_dock.py` | IW 도크 잠금과 deck 결속/해제 |
| 5 | `isaacpjt/pjt_utils/deck_geometry.py` | 데크 중심·팔레트 지지면·홀 높이 계산 |
| 6 | `isaacpjt/scene/warehouse.py` | 랙 6개 슬롯과 팔레트/KLT 씬 배치 |
| 7 | `src/smartfarm/warehouse_dock/test/` | 안착 setpoint와 상태기계 회귀 조건 |

### 3.2 두 ROS 노드의 관계

`ForkLiftNode`는 공통 기반 클래스이자 초기/단독 시험 상태기계다.
`ForkLiftReturnNode`는 이를 상속해 통합용 반복 교환 서비스를 제공한다.

| 노드 | 역할 |
|---|---|
| `fork_lift_node` | 콘솔 선택 상차, 초기/단독 시험, 공통 Step 실행기 |
| `fork_lift_return_node` | `/forklift/start_cycle`을 받아 적재 팔레트 회수 후 다음 빈 팔레트 상차 |

통합 launch가 어떤 노드를 기동하는지 반드시 확인한다. 두 노드가 같은
`/forklift_0/joint_command`를 동시에 제어하지 않도록 인스턴스 lock과 BUSY 조건이 있지만,
운용 구성에서는 불필요한 중복 실행 자체를 피하는 것이 좋다.

현재 `smartfarm_integration.launch.py`는 이름과 달리 `fork_lift_return_node`가 아니라
`fork_lift_node`를 실행한다. 반면 IW의 `mission_nav_node.py`는
`/forklift/start_cycle`을 호출하며 이 서비스는 `fork_lift_return_node`에만 있다.
이는 문서상의 미래 설계가 아니라 현재 코드의 실제 배선 불일치다. 통합 발표 전에
사용할 노드를 하나로 확정하고 launch 또는 실행 절차를 맞춰야 한다.

## 4. 모델과 기하

| 항목 | 현재 값 |
|---|---|
| 실행 플래그 / namespace | `--fork` / `forklift_0` |
| Isaac root | `/World/Forklift` |
| 초기/대기 pose | `(0.3171, 14.5, -π/2)` |
| 팔레트 수 | 6개, `Pallet_00`~`Pallet_05` |
| 랙 X | `-2.4, -2.4, -0.8, -0.8, 0.8, 0.8` |
| 랙 층 바닥 Z | `0.322 m` 또는 `1.222 m` |
| wheel radius / wheelbase | `0.22 m` / `2.05 m` |
| 포크 삽입 깊이 | 랙/IW 기본 `0.65 m` |
| pickup raise | 기본 `0.20 m` |
| 제어율 | `20 Hz` |

ForkliftB는 앞바퀴가 아니라 후륜 조향/구동 모델이다. 좁은 창고에서 큰 조향각으로
U턴한 뒤 랙 진입 직전에는 조향을 제한하고 X/yaw gate를 통과해야 직선 삽입한다.

## 5. Step 상태기계

`fork_lift_node.py`는 긴 작업을 `Step` deque로 분해한다. 각 Step은 목표 pose, lift,
drive, steering, timeout, pallet ID, 결속 상태 중 필요한 필드만 사용한다.

대표 Step:

| kind | 역할 |
|---|---|
| `drive`, `approach` | 일반 pose 접근 |
| `arc` | U턴 |
| `lane_align` | 작은 S자 조향으로 랙/IW 축 합류 |
| `straight_y` | 조향 0으로 포크 직선 삽입/후퇴 |
| `lift` | 속도 제한된 리프트 목표 |
| `iw_axis_gate` | IW 진입 전 공통 X/yaw 정밀 검증 |
| `coupler` | fork–pallet 결속/해제 |
| `pallet_owner` | `fork`, `deck`, `none` 소유권 원자 전환 |
| `dock_lock` | IW canonical pose 잠금/해제 |
| `deck_place_lower` | 실제 pallet Z 기반 폐루프 데크 안착 |
| `event` | pallet_on_iw, clear 등 외부 완료 신호 |

각 Step은 완료 조건과 timeout을 갖는다. 시간만 지나면 성공시키는 것이 아니라
pose/joint/handoff 피드백을 확인한다.

## 6. 팔레트 소유권과 물리 안전

팔레트의 논리 소유자는 항상 하나여야 한다.

```text
rack/free
  → fork에 결속
  → 운반
  → IW deck에 결속
  → fork 결속 해제
```

Isaac의 `fork.py`와 `iw_dock.py`가 실제 FixedJoint, collision filtering, 강체 추종을
관리한다. `pallet_owner` 명령은 다음을 원자적으로 처리해야 한다.

- fork owner로 바꿀 때 deck joint를 제거
- deck owner로 바꿀 때 fork joint를 제거
- handoff 순간 pallet–fork 또는 pallet–IW 충돌 필터 적용/복원
- IW 데크 결속 전 XY/Z/tilt 안전 gate 통과
- 동일 프레임에 fork와 deck이 팔레트를 동시에 잡지 않음

`/forklift/handoff_state`는 owner, 결속, collision filter, IW pose, pallet pose,
target pose, `pallet_min_z` 등을 JSON으로 제공한다. ROS 상태기계는 이 값을 성공
판정의 근거로 사용한다.

## 7. IW 데크 기하

고정 `amr_hole_center Z=0.45`만 믿고 상차하지 않는다.

```text
Isaac IW 실제 chassis/deck 측정
  → /iwhub_0/deck_geometry
  → Forklift ROS가 deck top과 pallet hole center 갱신
  → 포크 lift 목표와 안착 target 계산
```

`deck_geometry.py`의 핵심:

- 표준 pallet.usd 포크 채널: 로컬 Z `0.02053 ~ 0.11605 m`
- 데크와 팔레트 최소 지지 여유: `0.002 m`
- IW root에서 실제 데크 중심까지의 X 오프셋: `0.3171 m`
- 팔레트 bbox 하면을 데크 상면에 맞춘 origin Z 계산

## 8. 폐루프 데크 안착

과거 개루프 높이식은 포크에 팔레트가 실제로 얹힌 오프셋을 알지 못해 팔레트가
데크보다 약 5.5 cm 높은 상태에서 결속을 계속 요청했고, 2.5 cm 안전 gate에 막혀
timeout이 났다.

현재는 두 단계로 내린다.

1. 기존 slow-lower로 계산 목표까지 거칠게 접근
2. `deck_place_lower`가 `pallet_min_z - target_z` 잔차를 보고 리프트를 추가 보정

```text
desired_lift = current_lift - (z_measured - z_target)
```

안전 조건:

| 파라미터 | 기본값 |
|---|---:|
| 안착 허용오차 | `0.010 m` |
| 최대 추가 보정 | `0.10 m` |
| 안정 유지 | `0.4 s` |
| handoff state stale | `0.5 s` |
| 폐루프 timeout | `20 s` |
| IW 결속 Z gate | `±0.025 m` |

측정값이 없거나 오래됐으면 추측값으로 결속하지 않고 실패한다. 최대 보정을 넘어 계속
낮추는 것도 금지한다. 안전 gate를 완화해 공중 팔레트를 강제로 고정하는 방식은 쓰지 않는다.

## 9. 반복 교환 상태

`ForkLiftReturnNode`의 상위 phase:

```text
WAITING_FOR_IW
→ RETURNING_TO_RACK
→ LOADING_NEXT_PALLET
→ WAITING_FOR_IW
```

팔레트 번호는 다음처럼 순환한다.

```text
inbound = n
outbound = (n + 1) % 6
```

서비스 요청을 받을 때 다음을 만족해야 접수한다.

- inbound ID가 0~5
- 노드가 대기 상태
- 최신 joint state/pose feedback 존재
- 최신 `/iwhub_0/deck_geometry` 존재
- 최신 `/forklift/handoff_state`에 실제 IW world pose 존재

중단 지점에 따라 `resume_next_pallet`, `resume_handoff_pallet`로 제한적 재개 경로가
있다. 이는 물리 장면의 현재 owner 상태와 맞춰 사용해야 하며 무조건적인 재시작 옵션이 아니다.

## 10. 핵심 ROS 인터페이스

| 방향 | 이름 | 타입 | 의미 |
|---|---|---|---|
| IW → Forklift | `/forklift/start_cycle` | `ForkliftCycle` service | inbound 팔레트와 도크 pose |
| Forklift → Isaac | `/forklift_0/joint_command` | `JointState` | drive/steer/lift/owner 명령 |
| Isaac → Forklift | `/forklift_0/joint_states` | `JointState` | 실제 관절 |
| Isaac → Forklift | `/forklift_0/pose` | `PoseStamped` | 실제 차체 pose |
| Isaac → Forklift | `/forklift/handoff_state` | JSON String | 물리 owner·정렬·높이 |
| IW Isaac → Forklift | `/iwhub_0/deck_geometry` | JSON String | 데크 실측 기하 |
| Forklift → IW | `/forklift/pallet_on_iw` | `Int32` | 새 팔레트 번호 |
| Forklift → IW | `/forklift/clear` | `Bool` | 지게차 작업 공간 clear |
| Forklift → 운영 | `/forklift/status` | `String` | 현재 Step/오류 |
| Forklift → 운영 | `/forklift/task_complete` | `Int32` | 완료 팔레트 |
| Forklift → IW | `/iw/request_dock_adjust` | `DockAdjust` service | 인계 정렬 재요청 |

`/handoff/tray_ready`의 `HandoffEvent` 구독은 레거시 호환용이다. 현재 권위 있는
통합 경로는 `/forklift/start_cycle` 서비스다.

## 11. 실행과 확인

창고 단독 Isaac 시험:

```bash
cd /home/rokey/cobot3_ws/isaacpjt
isaac_python main.py --iw --fork
```

3로봇 통합 Isaac:

```bash
isaac_python main.py --mm --iw --fork --nav --camera
```

통합 ROS:

```bash
cd /home/rokey/cobot3_ws
source install/setup.bash
ros2 launch harvest_vision smartfarm_integration.launch.py
```

확인할 핵심:

```bash
ros2 topic echo /forklift/status
ros2 topic echo /forklift/handoff_state
ros2 topic echo /iwhub_0/deck_geometry
ros2 topic echo /forklift/pallet_on_iw
ros2 topic echo /forklift/clear
ros2 service type /forklift/start_cycle
```

마지막 명령이 서비스를 찾지 못하면 IW–Forklift 반복 교환 배선이 완성되지 않은 상태다.
`fork_lift_return_node`를 사용할 때는 기반 클래스와 같은 command topic을 쓰므로
다른 지게차 제어 노드를 중복 실행하지 않았는지도 확인한다.

## 12. 담당자가 코드 리뷰에서 확인할 불변조건

- `joint_command`를 발행하는 지게차 제어 노드는 실질적으로 하나여야 한다.
- pose/joint/deck geometry 피드백 없이 성공 상태를 진행하지 않는다.
- 랙 삽입 전 X/yaw gate를 건너뛰지 않는다.
- 팔레트 owner는 `fork`, `deck`, `none` 중 하나다.
- fork와 deck이 같은 팔레트를 동시에 결속하지 않는다.
- IW deck 결속 전 XY, Z, tilt, collision-filter 상태를 검증한다.
- `pallet_min_z`와 target이 stale이면 결속하지 않는다.
- 폐루프 최대 하강량을 초과하지 않는다.
- `/forklift/clear`는 새 팔레트가 IW에 결속되고 fork가 빠진 뒤에만 발행한다.

## 13. 현재 검증 상태와 위험

단위 테스트가 있는 영역:

- 데크 기하 순수 계산
- 폐루프 lift setpoint
- tolerance 안정 유지와 stale 처리
- 일부 owner/상태 전이

실제 Isaac 통합에서 재확인할 영역:

- 팔레트 각 층별 포크 삽입 높이
- 2층 팔레트 인출 중 상부 보 충돌
- 적재 팔레트 회수 후 원래 랙 슬롯 안착
- 빈 팔레트 폐루프 안착이 `±0.025 m` gate 안에서 완료되는지
- owner 전환 순간 튐, 관통, 이중 결속이 없는지
- 새 팔레트 결속 후 IW 주행에서 팔레트/KLT가 따라가는지
- 전체 `Pallet_00 → Pallet_01` 교환 후 `/forklift/clear`까지 완주
- 통합 launch가 실제 서비스 구현 노드를 기동하도록 정리됐는지

## 14. 발표용 핵심 제어기술

| 핵심 기술 | 제어 대상과 방식 | 발표 핵심 문장 |
|---|---|---|
| 결정적 Step 상태기계 | 주행·조향·리프트·결속을 완료 조건과 timeout이 있는 Step으로 순차 실행 | “정밀 인계 작업을 재현 가능하고 고장 위치가 명확한 이산 상태 제어로 구성했습니다.” |
| 후륜 조향 경로 제어 | `arc`와 `lane_align`으로 진입하고 X/yaw gate 통과 후 `straight_y`로 직선 삽입 | “큰 회전과 정밀 삽입을 분리해 포크가 비스듬히 들어가는 것을 막습니다.” |
| 팔레트 소유권 전환 | `rack/free → fork → deck` 순으로 FixedJoint와 충돌 필터를 원자적으로 전환 | “한 팔레트를 두 로봇이 동시에 잡지 않는 단일 소유자 원칙이 물리 안전의 핵심입니다.” |
| 실측 기하 기반 제어 | `/iwhub_0/deck_geometry`로 데크 상면과 팔레트 홀 높이를 갱신 | “고정 설계값 대신 현재 물리 장면의 실측 기하로 인계 목표를 계산합니다.” |
| 폐루프 높이 안착 | `pallet_min_z - target_z` 잔차로 lift를 보정하고 안정 유지·stale·최대 하강량을 검사 | “결속 gate를 느슨하게 하지 않고 실제 팔레트 하면을 목표 높이에 수렴시킵니다.” |
| 피드백 기반 안전 gate | pose, yaw, joint, owner, XY/Z/tilt, collision 상태가 모두 유효할 때만 다음 Step 진행 | “시간 경과가 아니라 물리 피드백이 성공을 증명해야 상태가 전이됩니다.” |

발표에서는 **Step 상태기계 → 소유권 전환 → 실측 기하 → 폐루프 안착** 순서로
설명하면 지게차의 이동 제어와 물리 인계 제어가 자연스럽게 연결된다.

## 15. 발표자가 답할 수 있어야 하는 질문과 답

### Q1. 왜 지게차는 Nav2 대신 Step 상태기계를 쓰는가?

창고 내 팔레트 인계는 자유 공간의 최단 경로 탐색보다 정해진 순서와 정밀한 접촉 조건이
중요하기 때문이다. U턴, 축 정렬, 직선 포크 삽입, 리프트, 결속을 각각 완료 조건과
timeout이 있는 Step으로 나누면 동작이 결정적이고 실패 지점도 명확하다. Nav2는 일반
이동에는 유리하지만 후륜 조향 지게차의 포크 삽입과 물리 소유권 전환까지 직접 보장하지는
않는다.

### Q2. map pose가 있는데 왜 handoff에는 Isaac world pose를 다시 쓰는가?

map pose는 AMCL이 추정한 항법 좌표이고, 실제 접촉과 FixedJoint 생성은 Isaac/PhysX
월드 좌표에서 일어난다. 두 좌표에는 추정 오차나 보정 오프셋이 있을 수 있으므로 인계
순간에는 최신 Isaac 물리 pose를 사용해야 포크, 팔레트, 데크의 실제 위치가 일치한다.
map pose는 도크까지의 항법에, Isaac world pose는 최종 물리 인계에 사용한다.

### Q3. 팔레트 owner를 별도로 관리해야 하는 이유는 무엇인가?

팔레트가 랙, 포크, IW 데크 중 어디에 결속돼 있는지를 명시하지 않으면 이중 FixedJoint,
충돌 필터 불일치, 순간적인 튐이나 관통이 생길 수 있다. owner 전환은 기존 결속 해제,
새 결속 생성, 충돌 필터 변경을 한 묶음으로 처리해 팔레트의 물리적 책임 주체를 항상
하나로 유지한다.

### Q4. 왜 데크 결속 Z gate를 완화하지 않고 폐루프를 추가했는가?

gate를 완화하면 공중에 뜬 팔레트도 정상으로 판정해 강제로 결속할 수 있고, 이후 IW
주행에서 튐이나 낙하가 발생할 수 있다. 폐루프는 실제 팔레트 하면 높이 오차를 측정해
리프트를 추가 보정하므로 안전 기준은 유지하면서 오차의 원인을 직접 줄인다.

### Q5. `pallet_position.z`보다 `pallet_min_z`가 더 안전한 이유는 무엇인가?

`pallet_position.z`는 팔레트 원점 높이라서 모델 원점 위치와 자세에 따라 실제 접촉면을
직접 나타내지 않는다. `pallet_min_z`는 월드 좌표에서 팔레트 바운딩 박스의 가장 낮은
점이므로 데크 상면과의 간격을 바로 계산할 수 있다. 따라서 안착과 관통 여부를 판단하기에
더 직접적인 물리량이다.

### Q6. 랙 2층 팔레트의 pickup raise가 다른 이유는 무엇인가?

2층은 상부 보와 다음 구조물까지의 수직 여유가 작아 1층과 같은 상승량을 적용하면
팔레트나 포크가 구조물과 충돌할 수 있다. 포크 채널에서 빠지지 않을 최소 상승량과
상부 간극을 함께 고려해 층별 pickup raise를 달리해야 한다.

### Q7. `/forklift/clear`가 발행되기 전에 무엇이 모두 완료돼야 하는가?

새 팔레트가 목표 XY/Z/tilt gate 안에 있고, owner가 `deck`으로 전환돼 IW에 정상
결속됐으며, fork 결속과 관련 충돌 필터가 해제되고, 포크가 안전 거리까지 후퇴해야 한다.
그 뒤 도크 lock을 해제하고 지게차가 IW 작업 공간을 벗어났음이 확인돼야
`/forklift/clear`를 발행할 수 있다.
