# smartfarm_interfaces

전 트랙 공용 커스텀 메시지/서비스 정의 패키지. 노드는 없다.

## msg

| 메시지 | 용도 |
|---|---|
| `TomatoDetection` | 토마토 1개체 검출 결과 (`vision_node` → `harvest_fsm_node`) |
| `TomatoDetectionArray` | 한 프레임의 검출 전체 (`header` + `TomatoDetection[]`) |
| `SlotAssignment` | 창고 슬롯 배정 |
| `TransportRequest` | 운반 요청 큐 이벤트 |
| `TrayStatus` | 트레이 적재 현황 |
| `HandoffEvent` | 운반 AMR 인계 위치 도착 트리거 |

## srv

| 서비스 | 용도 |
|---|---|
| `ForkliftCycle` | IW가 창고 도크에 도착했을 때 시작하는 팔레트 교환 사이클 요청/접수 |
| `DockAdjust` | 지게차가 IW 도킹 잠금에 실패했을 때 Nav2 정밀 재정렬 요청 |

> `SlotAssignment`·`TransportRequest`·`TrayStatus`·`HandoffEvent`는 초기 설계 당시
> 노드(`warehouse_manager_node`, `fleet_dispatch_node`, `tray_manager_node`,
> `handoff_node`)와 함께 정의됐으나 해당 노드는 이후 삭제되어 현재 발행자가 없다.
> 실제 사용 여부·폐기 사유는 [`../INTERFACES.md`](../INTERFACES.md) 참고.
