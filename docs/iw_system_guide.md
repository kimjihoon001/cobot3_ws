# IW 파트 현재 구조·코드 학습·발표 가이드

> 기준: `multiple_harvest` 브랜치의 2026-07-26 작업 트리
> 기준 HEAD: `bd06cf3` + 현재 작업 트리 변경
> 대상: IW 담당자, 통합 발표자, MM/Forklift 연동 담당자

## 1. 한 문장 설명

IW는 MM 옆으로 이동해 수확물을 KLT에 받고, 적재가 끝나면 MM을 먼저 피항시킨 뒤
창고 도크로 가서 지게차와 팔레트를 교환하고 다시 MM으로 복귀하는 차동구동 AMR이다.

트랙 이름으로는 B에 가까웠지만 현재 코드는 다음 세 책임을 한 파트로 봐야 이해하기 쉽다.

1. Isaac의 IW 차체·적재물·센서·물리 결속
2. ROS2의 차동구동·AMCL·Nav2 레인 주행
3. MM 추종과 Forklift 인계를 연결하는 미션 오케스트레이션

## 2. 발표용 전체 흐름

```text
MM 수확 시작
  → /iw/mission = FOLLOW
  → MM 위치에서 약 1.03 m 떨어진 추종점 계산
  → 온실 레인 경로를 NavigateThroughPoses로 주행
  → IW 앞열 빈 KLT pose를 MM에 제공
  → MM이 기본 2회 플레이스
  → /iw/mission = PREPARE_FORKLIFT
  → IW가 /iw/mm_yield_request 발행
  → MM 피항 완료(/iw/mm_yield_complete)
  → IW가 FORKLIFT 상태로 전환
  → 창고 도크 접근 + 저속 XY/yaw 폐루프 정렬
  → /forklift/start_cycle 서비스 호출
  → 지게차가 적재 팔레트 회수 + 다음 빈 팔레트 상차
  → /forklift/clear
  → MM 근처로 복귀
  → /iw/status = RETURNED, /iw/resume_harvest = true
```

핵심은 `PREPARE_FORKLIFT`가 곧바로 창고 출발을 뜻하지 않는다는 점이다. IW는 먼저
MM에 피항을 요청하고, MM이 통로를 비웠다는 완료 신호를 받은 뒤에만 이동한다.

## 3. 코드 지도

### 3.1 가장 먼저 읽을 파일

| 순서 | 파일 | 읽을 이유 |
|---:|---|---|
| 1 | `src/smartfarm/iwhub_control/iwhub_control/mission_nav_node.py` | FOLLOW/PREPARE_FORKLIFT/FORKLIFT 전체 미션 상태 |
| 2 | `src/smartfarm/iwhub_control/iwhub_control/lanes.py` | 온실 통로 레인, 도크 좌표, swept-footprint 검사 |
| 3 | `isaacpjt/iw.py` | IW 스폰, 센서 브리지, 빈 KLT 선택, 지게차 연동 API |
| 4 | `isaacpjt/iw_dock.py` | 도킹 잠금과 팔레트 데크 결속의 실제 물리 소유권 |
| 5 | `src/smartfarm/iwhub_control/iwhub_control/base_node.py` | `cmd_vel`을 좌우 바퀴 속도로 변환 |
| 6 | `src/smartfarm/iwhub_control/config/nav2_params.yaml` | AMCL, DWB, costmap, footprint 실설정 |
| 7 | `src/smartfarm/iwhub_control/launch/iwhub_nav2.launch.py` | 필터·Nav2·수명주기 기동 관계 |
| 8 | `isaacpjt/robots/iwhub.py` | IW USD, 팔레트/KLT, 사전 적재 토마토 생성 |

### 3.2 파일별 책임

```text
isaacpjt/robots/iwhub.py
  └─ 모델/적재물 생성
isaacpjt/iw.py
  ├─ ROS joint/odom/scan bridge
  ├─ /iwhub_0/deck_geometry
  └─ /iw/basket/empty_slot_pose
isaacpjt/iw_dock.py
  ├─ canonical dock world lock
  └─ pallet ↔ IW deck 결속

iwhub_control/base_node.py
  └─ cmd_vel → joint_command
iwhub_control/scan_self_filter_node.py
  └─ IW 자신의 팔레트/KLT 라이다 반사 제거
iwhub_control/lanes.py
  └─ 결정적 레인 경로와 충돌 사전 검사
iwhub_control/mission_nav_node.py
  └─ MM·Nav2·Forklift 상태 조정
```

## 4. 모델·적재 구조

| 항목 | 현재 값 |
|---|---|
| 실행 플래그 / namespace | `--iw` / `iwhub_0` |
| Isaac root | `/World/IwHub` |
| 초기 pose | `(1.6955, -12.0)`, yaw `180°` |
| 주요 DOF | `left_wheel_joint`, `right_wheel_joint`, `lift_joint` |
| 구동 방식 | 차동구동 |
| 데크 팔레트 | 초기 `Pallet_00` |
| KLT 배치 | 4 × 2 = 8칸 |
| 사전 적재 | 뒤쪽 6칸 × 토마토 5개 = 30개 |
| MM 플레이스용 | 앞열 `KLT_30`, `KLT_31` 두 칸 |

사전 적재 토마토는 개별 키네마틱 강체가 아니다. `Load` 강체 자식 콜라이더로 구성해
IW가 움직일 때 적재물만 월드에 남는 문제를 피한다. MM이 실제로 놓는 두 과실만
앞열의 사용 이력으로 관리한다.

### 빈 KLT 선택

`isaacpjt/iw.py`는 다음 순서로 release pose를 만든다.

1. 현재 데크에 결속된 팔레트 ID와 실제 prim 경로를 확인한다.
2. 앞열 두 슬롯 중 아직 사용하지 않은 슬롯만 남긴다.
3. 고정된 `/World/Harvester`가 아니라 실제 이동 강체
   `/World/Harvester/Base/base_link`에 더 가까운 슬롯을 고른다.
4. KLT 윗면보다 약 5 cm 높은 pose를 `map` 프레임으로 발행한다.
5. `/harvester_0/scoop/release_debug` 이벤트가 오면 그 슬롯을 사용 완료로 표시한다.
6. 지게차가 새 팔레트를 데크에 결속하면 사용 이력을 초기화한다.

## 5. 주행 구조

### 5.1 저수준 데이터 흐름

```text
Nav2 /iwhub_0/cmd_vel_nav
  → velocity_smoother
  → /iwhub_0/cmd_vel
  → base_node
  → /iwhub_0/joint_command
  → Isaac wheel articulation
  → /iwhub_0/odom + TF + LaserScan
```

차동구동 변환은 다음 식이다.

```text
left  = (v - w × wheel_separation / 2) / wheel_radius
right = (v + w × wheel_separation / 2) / wheel_radius
```

`base_node`의 중요한 안전장치는 명령 watchdog과 정지 deadband다. 목표점에서 작은
속도 명령이 반복돼 차체가 흔들리지 않게 히스테리시스를 사용한다.

### 5.2 위치추정과 센서

```text
iwhub_0/map             ← AMCL
└─ iwhub_0/odom         ← Isaac 실제 chassis odom
   └─ iwhub_0/base_link
      └─ iwhub_0/chassis
         ├─ front_2d_lidar
         └─ back_2d_lidar
```

- AMCL만 `map → odom`을 발행한다.
- Isaac이 실제 chassis 기준 `odom → base_link`를 발행한다.
- 전·후방 raw scan은 자기반사 필터를 거쳐 costmap으로 들어간다.
- 같은 TF나 odom을 두 노드가 동시에 발행하면 위치 점프가 생기므로 금지한다.

### 5.3 레인 기반 Nav2

IW는 Nav2가 자유롭게 온실 사이를 가로지르게 두지 않는다.
`lanes.py`가 배드 사이 중심선을 직선과 원호 웨이포인트로 만들고,
`mission_nav_node.py`가 `NavigateThroughPoses`로 보낸다.

| 구성 | 의미 |
|---|---|
| 세로 레인 X | `-6.0, -2.9, 0.0, 2.9, 6.0` |
| 가로 레인 Y | `-11.5, -3.85, 3.85, 11.5` |
| 도크 pose | `(0.0, 10.84885, π)` |
| 경로 간격 | 약 `0.5 m` |
| 안전 검사 | 적재 footprint + `0.05 m` margin |
| 일반 주행 | 전진 전용 BT, DWB |

FOLLOW는 움직이는 MM을 그대로 직선 추종하지 않는다. 현재 IW 위치에서 MM 접근 방향의
standoff 목표를 만들고, 가장 가까운 레인으로 연결한다. 목표가 `0.30 m` 이상 또는
약 `30°` 이상 바뀔 때만 새 goal을 보내 불필요한 재계획을 줄인다.

### 5.4 도크 최종 정렬

도크 근처까지는 Nav2가 담당하고 마지막 정밀 정렬은 별도 20 Hz 폐루프가 맡는다.

1. `POSITION`: 목표 bearing을 먼저 맞춘 뒤 최대 `0.08 m/s`로 전·후진
2. `YAW`: 위치 명령을 섞지 않고 제자리 yaw만 보정
3. 위치 `0.04 m`, yaw 약 `2°` 이내에서 `1 s` 안정 유지
4. 성공하면 `/forklift/start_cycle` 호출
5. 정렬 범위나 시간 초과 시 Nav2 도크 접근부터 재시도

## 6. 핵심 ROS 인터페이스

| 방향 | 이름 | 타입 | 의미 |
|---|---|---|---|
| MM → IW | `/iw/mission` | `std_msgs/String` | `IDLE`, `FOLLOW`, `PREPARE_FORKLIFT` |
| IW → MM/운영 | `/iw/status` | `std_msgs/String` | `WAITING_MM_YIELD`, `MM_CLEAR_TO_FORKLIFT`, `ARRIVED_FORKLIFT`, `RETURNED` 등 |
| IW → MM | `/iw/basket/empty_slot_pose` | `PoseStamped` | 현재 빈 KLT release pose |
| IW → MM | `/iw/mm_yield_request` | `Bool` | 창고 출발 전 MM 피항 요청 |
| MM → IW | `/iw/mm_yield_complete` | `Bool` | MM 피항 완료 |
| IW → MM | `/iw/resume_harvest` | `Bool` | 팔레트 교환 후 수확 재개 |
| Isaac → IW ROS | `/iwhub_0/odom` | `Odometry` | 실제 chassis odom |
| Nav2 → base | `/iwhub_0/cmd_vel` | `Twist` | 최종 속도 |
| base → Isaac | `/iwhub_0/joint_command` | `JointState` | 좌우 wheel 속도 |
| Isaac → Forklift | `/iwhub_0/deck_geometry` | JSON String | 데크·팔레트 홀 실측 기하 |
| IW → Forklift | `/forklift/start_cycle` | `ForkliftCycle` service | 팔레트 교환 요청 |
| Forklift → IW | `/forklift/clear` | `Bool` | 지게차 작업 종료, IW 출발 허가 |

`/iw/mission`과 `/iw/status`는 늦게 기동한 노드도 마지막 상태를 받을 수 있도록
transient-local QoS를 사용한다.

### 현재 launch 배선 주의

`mission_nav_node.py`는 `/forklift/start_cycle` 서비스를 호출한다. 이 서비스의
구현자는 `fork_lift_return_node.py`다. 그러나 현재
`smartfarm_integration.launch.py`는 `fork_lift_node`를 실행한다.

따라서 현재 작업 트리 그대로 통합 launch만 실행하면 IW가 도크에 도착한 뒤
`/forklift/start_cycle`을 계속 기다릴 수 있다. 발표/통합 전에는 다음 중 하나를
명시적으로 확정해야 한다.

- 통합 launch가 `fork_lift_return_node`를 실행하도록 구성
- `fork_lift_return_node`를 별도 실행
- 서비스 기반 반복 교환 대신 `fork_lift_node` 단독 시험 흐름을 쓸 경우 IW 호출 경로도 맞춤

두 지게차 노드가 동시에 저수준 명령을 경쟁하지 않는지도 함께 확인한다.

## 7. 실행과 확인

통합 Isaac:

```bash
cd /home/rokey/cobot3_ws/isaacpjt
isaac_python main.py --mm --iw --fork --nav --camera
```

통합 ROS:

```bash
cd /home/rokey/cobot3_ws
source install/setup.bash
ros2 launch harvest_vision smartfarm_integration.launch.py
```

IW만 분리 확인할 때는 `iwhub_base.launch.py`, `iwhub_nav2.launch.py`를 사용할 수 있다.
다만 통합 실행과 동시에 띄워 TF·Nav2·base controller를 중복 생성하지 않는다.

확인할 핵심:

```bash
ros2 topic echo /iw/status
ros2 topic echo /iw/basket/empty_slot_pose
ros2 topic echo /iwhub_0/deck_geometry
ros2 topic hz /iwhub_0/odom
ros2 action list | grep iwhub_0
ros2 service list | grep /forklift/start_cycle
```

## 8. 담당자가 코드 리뷰에서 확인할 불변조건

- 일반 통합에서는 창고 `Pallet_00`과 초기 IW `Pallet_00`이 중복 생성되지 않는다.
- 데크에 팔레트가 없을 때 빈 KLT pose를 발행하지 않는다.
- 같은 release 이벤트로 두 슬롯을 소비하지 않는다.
- 새 팔레트가 결속된 뒤에만 슬롯 사용 기록을 초기화한다.
- FOLLOW 경로가 배드 swept-footprint 검사에 실패하면 goal을 보내지 않는다.
- AMCL 외에 고정 `map → odom` 발행기를 함께 띄우지 않는다.
- `PREPARE_FORKLIFT`에서 MM 피항 완료 전에는 도크 goal을 보내지 않는다.
- 지게차 작업 중 IW 도크 잠금과 팔레트 소유권은 Isaac에서 단일 소유자만 갖는다.

## 9. 현재 검증 상태와 위험

정적 로직과 단위 테스트가 있는 영역:

- 레인 경로와 footprint 검사
- 데크 기하 계산
- KLT 슬롯 사용/초기화 상태
- base deadband

실제 Isaac 통합에서 반드시 재확인할 영역:

- 사전 적재 30개 토마토의 안착 높이와 IW 주행 추종
- 첫 `Pallet_00`과 교체 팔레트의 KLT pose가 실제 팔레트를 가리키는지
- 전·후방 라이다 자기반사 제거 후 AMCL 안정성
- 도크 최종 정렬과 canonical lock 사이의 pose 오차
- 팔레트 교환 후 `RETURNED → FOLLOW` 재개
- 통합 launch의 지게차 노드와 `/forklift/start_cycle` 서비스 구현자 불일치

## 10. 발표용 핵심 제어기술

| 핵심 기술 | 제어 대상과 방식 | 발표 핵심 문장 |
|---|---|---|
| 차동구동 속도 제어 | Nav2의 선속도·각속도를 좌우 wheel 속도로 변환하고 watchdog·deadband 적용 | “항법 명령을 실제 바퀴 속도로 변환하면서 명령 단절과 목표점 떨림을 억제합니다.” |
| 레인 기반 경로 제어 | 온실 중심선의 직선·원호 waypoint와 적재 footprint 검사를 거쳐 `NavigateThroughPoses` 실행 | “온실 통로의 구조적 제약을 경로 생성 단계에서 강제합니다.” |
| FOLLOW standoff 제어 | MM 접근 방향에 약 1.03 m 떨어진 목표를 만들고 변화량이 클 때만 재계획 | “움직이는 로봇 자체가 아니라 안전 이격점을 레인 경로로 추종합니다.” |
| AMCL–물리 odom 융합 | AMCL이 `map→odom`, Isaac chassis가 `odom→base_link`를 단독 발행 | “전역 위치 보정과 실제 차체 운동을 역할 분담해 TF 중복과 위치 점프를 막습니다.” |
| 하이브리드 도킹 제어 | 원거리 Nav2 접근 후 20 Hz POSITION/YAW 폐루프로 XY·yaw를 순차 정렬 | “항법의 강점과 정밀 상대 정렬의 강점을 구간별로 결합했습니다.” |
| 미션·자원 상태 제어 | MM 피항, KLT 슬롯 소비, Forklift 서비스, 팔레트 소유권을 명시적 handshake로 조정 | “주행뿐 아니라 공유 통로와 적재 자원의 상태까지 제어합니다.” |

발표에서는 **레인 항법 → FOLLOW → 정밀 도킹 → MM/Forklift handshake** 순서로
설명하면 IW가 세 시스템을 연결하는 오케스트레이터라는 점이 드러난다.

## 11. 발표자가 답할 수 있어야 하는 질문과 답

### Q1. 왜 자유 Nav2가 아니라 레인 기반 `NavigateThroughPoses`인가?

온실은 배드 사이 통로가 좁고 적재된 IW의 swept footprint가 크기 때문에 자유 경로가
기하학적으로 짧더라도 배드 모서리나 적재물과 충돌할 수 있다. 레인 중심선과 원호
웨이포인트를 미리 구성하고 footprint 검사를 통과한 경로만 보내면 통로 규칙을
결정적으로 지키면서도 Nav2의 추종·장애물 대응 기능을 사용할 수 있다.

### Q2. 왜 odom은 바퀴 적분이 아니라 Isaac chassis pose를 쓰는가?

시뮬레이션의 실제 이동 결과에는 미끄러짐, 접촉, 물리 solver의 영향이 포함되므로 명령된
바퀴 회전만 적분하면 실제 차체와 오차가 누적될 수 있다. Isaac chassis pose를 odom으로
쓰면 물리 장면의 실제 운동을 반영하고, AMCL은 그 위에서 `map→odom` 전역 보정만
담당한다.

### Q3. 왜 도크 마지막 구간은 Nav2가 아닌 별도 폐루프인가?

Nav2의 costmap 해상도와 일반 goal tolerance는 포크 삽입에 필요한 수 cm·수 도 수준의
반복 정렬을 안정적으로 보장하기 어렵다. 따라서 Nav2는 도크 근처까지의 장애물 회피를
담당하고, 마지막에는 실제 pose 오차를 20 Hz로 보면서 위치와 yaw를 순서대로 수렴시킨다.
허용오차 안에서 1초간 안정된 뒤에만 지게차 서비스를 호출한다.

### Q4. 빈 KLT 두 칸을 어떻게 구분하고 언제 초기화하는가?

IW는 현재 deck에 결속된 팔레트의 앞열 `KLT_30`, `KLT_31` 가운데 사용하지 않은
슬롯만 후보로 두고 실제 MM 베이스에 더 가까운 슬롯을 선택한다. release 이벤트를
받으면 해당 슬롯을 사용 완료로 기록하며, 지게차가 새 팔레트를 deck에 정상 결속한
뒤에만 두 슬롯의 사용 이력을 초기화한다.

### Q5. `PREPARE_FORKLIFT`와 `FORKLIFT` 사이에 MM 피항이 필요한 이유는 무엇인가?

`PREPARE_FORKLIFT`는 적재 완료를 알리는 준비 상태이지 즉시 출발 명령이 아니다.
수확 위치에서 MM과 IW가 통로를 공유하므로 IW는 먼저 피항을 요청하고 MM의 안전 자세와
Nav2 이동 완료를 확인해야 한다. `/iw/mm_yield_complete` 뒤에만 `FORKLIFT`로
전환해 충돌과 교착을 방지한다.

### Q6. 팔레트를 fork와 deck에 동시에 결속하지 않도록 어디서 보장하는가?

ROS 미션은 `/forklift/handoff_state`의 owner와 결속 상태를 확인하고, Isaac의
`fork.py`와 `iw_dock.py`가 실제 FixedJoint와 충돌 필터를 전환한다. `pallet_owner`
명령은 새 owner를 결속하기 전에 기존 owner의 joint를 제거하도록 원자적으로 처리한다.
따라서 상위 상태 gate와 하위 물리 소유권 관리 두 층에서 이중 결속을 막는다.
