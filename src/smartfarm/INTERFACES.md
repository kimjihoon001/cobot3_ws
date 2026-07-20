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
| - | 각 로봇 AMCL | `/<amr_id>/amcl_pose` | `geometry_msgs/PoseWithCovarianceStamped` | `handoff_node` (도착 판정용) |

## 커스텀 메시지 (`smartfarm_interfaces`)

- `TomatoDetection` / `TomatoDetectionArray` — pose, class(ripe/rotten), confidence
- `TrayStatus` — tray_id, capacity(6), filled_slots, ready_for_transport
- `TransportRequest` — tray_id, sector_id, pickup_pose, requested_at
- `HandoffEvent` — tray_id, amr_id, handoff_pose
- `SlotAssignment` — tray_id, slot_id(1~6), sector_id, occupied

## 아직 안 정해진 것 (통합 전 확정 필요 — TODO)

- **[4] 임의로 둔 임계값**: `tray_manager_node`의 운반 트리거 필터 개수(FR9 "추가 2~3개" 중 정확히 몇 개인지) — 지금은 5개(50%+2)로 가정
- **[4] 임의로 둔 파라미터**: 운반 AMR 대수/네임스페이스(`amr_ids`), 인계 위치 좌표(`handoff_pose_x/y`), 정적맵 경로 — 트랙 B의 SLAM 1회 생성 결과 나오면 채움
- **섹터 ID ↔ 슬롯 ID 1:1 매핑 규칙표**: `warehouse_manager_node`에 TODO로 남김, 브리핑 문서상 트랙 C 담당
- **tray_id 부여/일치 방식**: `harvest_fsm_node`(발생) → `handoff_node`(HandoffEvent.tray_id)까지 어떻게 같은 tray_id로 유지할지 트랙 A·B·C 합의 필요
- 각 노드 콜백 내부 실제 로직(YOLO 추론, Pick&Place, 포크 제어 등)은 전부 TODO — 이번 스켈레톤은 **토픽·메시지 인터페이스 고정**이 목적
