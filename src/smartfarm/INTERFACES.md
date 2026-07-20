# SmartFarm 통합 인터페이스 스펙 (초안)

기준 문서: Notion "7/20 (월) 협동 3 - 3일차 project 브리핑" 6.3절(소프트웨어 구조), 7절(기능 플로우), 9절(FR1~FR9).
아래 토픽명·메시지 필드는 브리핑 문서에 없던 부분을 이번에 새로 설계한 것 — **[4] 임의**. 노드 이름·역할·트리거 조건은 브리핑 문서에서 **[1] 그대로 인용**.

**구현 범위:** `harvest_vision`(트랙 A) 노드만 실제 초안 로직이 들어있고, `fleet_dispatch`/`warehouse_dock`/`smartfarm_common`은 패키지·노드 스켈레톤만 생성했습니다(빈 `Node` 클래스 + TODO). 아래 토픽 표는 트랙 A 쪽 관점에서 제안하는 계약이며, 트랙 B·C가 검토 후 확정해야 합니다.

## 패키지 구성

| 패키지 | 담당 | 포함 노드 |
|---|---|---|
| `smartfarm_interfaces` | 공용 | 커스텀 msg 정의 (아래 표) |
| `harvest_vision` | 트랙 A (이현민) | `vision_node`, `harvest_fsm_node`, `tray_manager_node` |
| `fleet_dispatch` | 트랙 B (김지훈) | `fleet_dispatch_node`, nav2/AMCL 런치 자리 |
| `warehouse_dock` | 트랙 C (김민성) | `handoff_node`, `fork_lift_node` |
| `smartfarm_common` | A+B 스켈레톤, C가 로직 완성 | `warehouse_manager_node`, `logger_node` |

## 토픽 흐름 (기능 플로우 1~13단계 대응)

| # | Publisher | Topic | Type | Subscriber |
|---|---|---|---|---|
| 3 | `vision_node` | `/vision/tomato_detections` | `smartfarm_interfaces/TomatoDetectionArray` | `harvest_fsm_node` |
| 5 | `harvest_fsm_node` | `/tray/place_request` | `std_msgs/Int32` (tray_id) | `tray_manager_node` |
| 6 | `tray_manager_node` | `/tray/status` | `smartfarm_interfaces/TrayStatus` | `logger_node` |
| 8 | `tray_manager_node` | `/dispatch/transport_request` | `smartfarm_interfaces/TransportRequest` | `fleet_dispatch_node` |
| 9 | `fleet_dispatch_node` | (액션) `/<amr_id>/navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | 운반 AMR Nav2 |
| 10 | `handoff_node` | `/handoff/tray_ready` | `smartfarm_interfaces/HandoffEvent` | `fork_lift_node`, `warehouse_manager_node` |
| 11 | `warehouse_manager_node` | `/warehouse/slot_assignment` | `smartfarm_interfaces/SlotAssignment` | `fork_lift_node`, `logger_node` |
| 11 | `fork_lift_node` | `/forklift/task_complete` | `std_msgs/Int32` (tray_id) | `warehouse_manager_node`, `logger_node` |
| 11 | `fork_lift_node` | `/forklift/clear` | `std_msgs/Bool` | `fleet_dispatch_node` (AMR 출발 허가) |
| - | `fork_lift_node` | `/forklift/status` | `std_msgs/String` | 운영자·logger_node |
| - | 각 로봇 AMCL | `/<amr_id>/amcl_pose` | `geometry_msgs/PoseWithCovarianceStamped` | `handoff_node` (도착 판정용) |

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
- `TrayStatus` — tray_id, capacity(6), filled_slots, ready_for_transport
- `TransportRequest` — tray_id, sector_id, pickup_pose, requested_at
- `HandoffEvent` — tray_id, amr_id, handoff_pose
- `SlotAssignment` — tray_id, slot_id(1~6), sector_id, occupied

## 아직 안 정해진 것 (통합 전 확정 필요 — TODO)

- **[수정] 사전 적재 값**: 브리핑 FR9는 "시작 시 50%(3개) 사전 적재"였지만 실제 `pjt_config/settings.py`(`TrayConfig.preloaded`)는 **0**으로 확정됨(정량 검증 목적상 사전 적재는 성공률을 부풀린다는 이유) — `tray_manager_node`도 이에 맞춰 `INITIAL_FILLED=0`, 운반 트리거는 일단 만재(6개) 기준으로 바꿔둠. 부분 적재 트리거가 필요하면 트랙 B와 협의 필요
- **[4] 임의로 둔 파라미터**: 운반 AMR 대수/네임스페이스(`amr_ids`), 인계 위치 좌표(`handoff_pose_x/y`), 정적맵 경로 — 트랙 B의 SLAM 1회 생성 결과 나오면 채움
- **섹터 ID ↔ 슬롯 ID 1:1 매핑 규칙표**: `warehouse_manager_node`에 TODO로 남김, 브리핑 문서상 트랙 C 담당
- **tray_id 부여/일치 방식**: `harvest_fsm_node`(발생) → `handoff_node`(HandoffEvent.tray_id)까지 어떻게 같은 tray_id로 유지할지 트랙 A·B·C 합의 필요
- 각 노드 콜백 내부 실제 로직(YOLO 추론, Pick&Place, 포크 제어 등)은 전부 TODO — 이번 스켈레톤은 **토픽·메시지 인터페이스 고정**이 목적
