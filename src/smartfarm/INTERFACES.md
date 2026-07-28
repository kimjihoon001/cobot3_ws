# SmartFarm 통합 인터페이스 스펙 (초안)

기준 문서: Notion "7/20 (월) 협동 3 - 3일차 project 브리핑" 6.3절(소프트웨어 구조), 7절(기능 플로우), 9절(FR1~FR9).
아래 토픽명·메시지 필드는 브리핑 문서에 없던 부분을 이번에 새로 설계한 것 — **[4] 임의**. 노드 이름·역할·트리거 조건은 브리핑 문서에서 **[1] 그대로 인용**.

**현재 구현 범위:** `harvest_vision`, `fleet_dispatch`, `iwhub_control`,
`warehouse_dock`에 실제 통합 로직이 구현되어 있다. `smartfarm_common`만 노드가
없는 빈 공용 패키지로 남아 있다. 아래 표는 초기 제안과 폐기 이력까지 함께
보존하므로, 취소선이 없는 인터페이스와 각 패키지 README를 현행 계약으로 본다.

**[2026-07-27 갱신]** 위 스켈레톤 중 `tray_manager_node`, `fleet_dispatch_node`, `logger_node`, `warehouse_manager_node`는 **전부 삭제**했습니다 — 실제 동작 경로(MM→IW 바스켓 직접 배치, IW 1대, 지게차 내부 슬롯 결정)에서 각각 역할이 다른 노드에 흡수되었거나 전제 자체가 성립하지 않습니다. 아래 표에서 취소선 처리한 행이 해당합니다.

## 패키지 구성

| 패키지 | 담당 | 포함 노드 |
|---|---|---|
| `smartfarm_interfaces` | 공용 | 커스텀 msg 정의 (아래 표) |
| `harvest_vision` | 트랙 A (이현민) | `vision_node`, `harvest_fsm_node` |
| `fleet_dispatch` | 트랙 B (김지훈) | `cmd_vel_watchdog`, `nav2_lifecycle_activator`, nav2/AMCL 런치 |
| `warehouse_dock` | 트랙 C (김민성) | `fork_lift_node`, `fork_lift_return_node` |
| `smartfarm_common` | - | **노드 없음** (스켈레톤 전량 폐기, 2026-07-27) |

## 토픽 흐름 (기능 플로우 1~13단계 대응)

| # | Publisher | Topic | Type | Subscriber |
|---|---|---|---|---|
| 3 | `vision_node` | `/vision/tomato_detections` | `smartfarm_interfaces/TomatoDetectionArray` | `harvest_fsm_node` |
| 5·6·8 | ~~`tray_manager_node`~~ | ~~`/tray/place_request`, `/tray/status`, `/dispatch/transport_request`~~ | ~~`TrayStatus`, `TransportRequest`~~ | **폐기(2026-07-27)** — MM 위 트레이 버퍼가 없다(IW 바스켓 슬롯에 직접 배치). 만재 판단은 `harvest_fsm_node`의 `place_target_count` 파라미터가 대신한다. `tray_manager_node.py` 삭제 |
| 9 | ~~`fleet_dispatch_node`~~ | ~~(액션) `/<amr_id>/navigate_to_pose`~~ | ~~`nav2_msgs/action/NavigateToPose`~~ | **폐기(2026-07-27)** — IW가 1대뿐이라 배차가 성립하지 않는다(`mission_nav_node.py`가 `iwhub_0` 하드코딩, `amr_id` 파라미터 없음). `fleet_dispatch_node.py` 삭제. **IW를 2대 이상으로 늘리면 배차 계층을 다시 설계해야 한다** |
| 10 | ~~`handoff_node`~~ | ~~`/handoff/tray_ready`~~ | ~~`smartfarm_interfaces/HandoffEvent`~~ | **폐기(2026-07-26)** — `iwhub_control/mission_nav_node.py`의 `_request_forklift_cycle()`이 도착 판정+지게차 활성화를 서비스 `/forklift/start_cycle`(`ForkliftCycle`) 호출로 대체함. `handoff_node.py`는 삭제. (`fork_lift_node.py`에 `HandoffEvent` 레거시 구독이 남아있으나 아무도 발행하지 않음) |
| 11 | ~~`warehouse_manager_node`~~ | ~~`/warehouse/slot_assignment`~~ | ~~`smartfarm_interfaces/SlotAssignment`~~ | **폐기(2026-07-27)** — `fork_lift_node.py`가 6슬롯 랙 기하(`RACK_CENTER_X`)와 팔레트 번호 선택(`_current_pallet`)을 내부에서 이미 결정한다. 슬롯 할당을 외부에서 받을 필요가 없음. `warehouse_manager_node.py` 삭제 |
| 11 | `fork_lift_node` | `/forklift/task_complete` | `std_msgs/Int32` (tray_id) | (현재 구독자 없음) |
| 11 | `fork_lift_node` | `/forklift/clear` | `std_msgs/Bool` | (현재 구독자 없음 — 구 `fleet_dispatch_node` 자리) |
| - | `fork_lift_node` | `/forklift/status` | `std_msgs/String` | 운영자 |
| - | 각 로봇 AMCL | `/<amr_id>/amcl_pose` | `geometry_msgs/PoseWithCovarianceStamped` | `mission_nav_node` (도착 판정용) |

## ⚠ 실제 Isaac 브리지와의 차이 (isaacpjt/README.md, ros/robot_bridge.py 확인 결과 — [1] 출처)

브리핑 문서(6.4절)와 위 표는 전 로봇 **Nav2+AMCL** 을 전제하지만, 실제로 지금 배선된
`isaacpjt/ros/robot_bridge.py` 그래프는 **로봇 3대 전부 저수준 조인트 제어만** 제공하고
Nav2/AMCL은 어디에도 없다. 통합 전에 팀 전체가 확인해야 하는 부분:

| 로봇 | 실제 인터페이스 (isaacpjt/README.md 3절) | 브리핑 문서 가정과 차이 |
|---|---|---|
| 수확 MM (`harvester_0`) | `/harvester_0/joint_command`(팔 6축+그리퍼), `/harvester_0/cmd`(String JSON: `base:[x,y,yaw]` 텔레포트, `blade:각도`) | 베이스가 **키네마틱 텔레포트**(2026-07-18 실측) — Nav2 경로 이동이 아니라 좌표를 순간이동시키는 방식. `settings.py`의 `SectorConfig` 주석은 "Nav2로 이동"이라 적혀 있어 문서 간 불일치 — **트랙 A/멘토 확인 필요**<br>**[2026-07-20 갱신]** Nav2 이식 완료(미검증): `--mm --nav` 로 `/harvester_0/{cmd_vel,odom,scan}` + TF 발행. 키네마틱 텔레포트는 그대로지만 `/cmd_vel(vx,vy,ω)`을 Isaac 이 적분해 홀로노믹 주행이 된다 → `NavigateToPose` 사용 가능. 설정은 `fleet_dispatch/config/harvester_nav2.yaml` |
| 운반 AMR (`iwhub_0`) | `/iwhub_0/joint_command`(좌우 바퀴 velocity=차동구동, lift_joint=승강) | Nav2 `NavigateToPose` 액션 자체가 안 붙어 있음. 위 표 9번 행(`fleet_dispatch_node` → `/<amr_id>/navigate_to_pose`)은 **아직 실제로 못 씀** — 트랙 B가 직접 속도 명령으로 주행 로직을 짜거나, Nav2를 새로 얹어야 함 |
| 지게차 (`forklift_0`) | `/forklift_0/joint_command`(lift_joint 0~2.0m, 후륜 조향/구동) | 고정 웨이포인트 이동도 결국 이 조인트 명령으로 구현해야 함 (Nav2 아님) |
| 카메라(D455, harvester 그리퍼 장착) | **아직 브리지에 안 붙음** — `robot_bridge.py`엔 joint/String/clock 그래프만 있고 카메라 그래프 없음 | `vision_node`가 구독할 `/rgb`,`/depth`는 day1~2 실습에서 검증된 이름을 기본값으로만 잡아둠 — 실제 배선되면 네임스페이스(`/harvester_0/rgb` 등)가 바뀔 수 있음 |

공통 통신 조건: `ROS_DOMAIN_ID=108`, `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`, FastDDS 화이트리스트(10.10.0.1~5+루프백) 일치 필요. Isaac Sim이 도는 GPU 노트북과 노드를 실행하는 개인 PC가 물리적으로 달라도 이 3가지 + 토픽명/타입/QoS만 맞으면 통신된다(DDS는 네트워크 기반).

## 커스텀 메시지 (`smartfarm_interfaces`)

- `TomatoDetection` / `TomatoDetectionArray` — pose, class(ripe/rotten), confidence
- 아래 msg 정의는 **전부 발행자·구독자가 없는 미사용 상태**(정의 파일은 남겨둠)
  - `TrayStatus` — tray_id, capacity(6), filled_slots, ready_for_transport (구 `tray_manager_node`)
  - `TransportRequest` — tray_id, sector_id, pickup_pose, requested_at (구 `tray_manager_node`)
  - `HandoffEvent` — tray_id, amr_id, handoff_pose (구 `handoff_node`)
  - `SlotAssignment` — tray_id, slot_id(1~6), sector_id, occupied (구 `warehouse_manager_node`)

## 아직 안 정해진 것 (통합 전 확정 필요 — TODO)

- ~~**[수정] 사전 적재 값**~~: **레거시(2026-07-26 주석)** — 아래는 전부 `tray_manager_node`(MM 위 트레이 버퍼) 기준이며, **실제 동작 경로에는 그런 트레이가 없다**. MM은 IW 바스켓 슬롯에 직접 놓는다.
  - 원문: 브리핑 FR9는 "시작 시 50%(3개) 사전 적재"였지만 실제 `pjt_config/settings.py`(`TrayConfig.preloaded`)는 **0**으로 확정됨(정량 검증 목적상 사전 적재는 성공률을 부풀린다는 이유) — `tray_manager_node`도 이에 맞춰 `INITIAL_FILLED=0`, 운반 트리거는 일단 만재(6개) 기준. 부분 적재 트리거가 필요하면 트랙 B와 협의 필요
  - **현재 경로에서는 이렇게 바뀌었다** (혼동 주의 — 위의 "사전 적재"와 아래의 "사전 적재"는 서로 다른 것이다):
    - **IW KLT 사전 적재 = 30개**: `isaacpjt/robots/iwhub.py`가 IW 데크 팔레트 뒤쪽 6칸에 5개씩 채운다. 이건 성공률 지표에 들어가는 적재가 아니라 **"이미 수확해 실어둔" 씬 연출**이다(콜라이더만 있고 Load 강체에 흡수됨). MM이 실제로 놓는 칸은 앞열 2칸뿐이다.
    - **운반 트리거 = `harvest_fsm_node`의 `place_target_count`(real_main 기본 1)**: 트레이 만재(6개)와 무관하다. IW 앞열 빈 KLT는 2칸이라 최대 2까지 올릴 수 있고, 다회 수확은 `multiple_harvest` 브랜치에서 2로 검증 중이다.
    - **부분 적재 트리거는 구현됨**: 수확 최종 실패·탐색 타임아웃 시 `_depart_with_partial_load()`가 목표 개수를 못 채워도 실은 게 있으면 `PREPARE_FORKLIFT`를 보낸다(0개면 안 보냄). 트랙 B 협의 항목이 아니라 트랙 A 코디네이터 내부 판단으로 정리됐다.
- **[4] 임의로 둔 파라미터**: 운반 AMR 대수/네임스페이스(`amr_ids`), 인계 위치 좌표(`handoff_pose_x/y`), 정적맵 경로 — 트랙 B의 SLAM 1회 생성 결과 나오면 채움
- ~~**섹터 ID ↔ 슬롯 ID 1:1 매핑 규칙표**~~: **무효(2026-07-27)** — `fork_lift_node.py`가 `RACK_CENTER_X`(6칸) + 팔레트 번호로 어느 랙에 넣을지 내부에서 결정한다. 별도 매핑 노드(`warehouse_manager_node`)는 삭제
- ~~**tray_id 부여/일치 방식**~~: **무효(2026-07-26)** — MM이 트레이 버퍼 없이 IW 바스켓 슬롯(`/iw/basket/empty_slot_pose`)에 직접 배치하므로 tray_id 체인 자체가 없다. 지금 식별자는 지게차 팔레트 번호 기반(`ForkliftCycle` 서비스 인자)
- YOLO 추론, MoveIt 수확·플레이스, IW Nav2 미션, 지게차 팔레트 교환은 현재
  구현되어 있다. 남은 TODO는 각 절에 개별적으로 표시하며, 이 문서 전체를
  스켈레톤 사양으로 해석하지 않는다.
