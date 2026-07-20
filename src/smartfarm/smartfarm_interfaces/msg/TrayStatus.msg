# 트레이 적재 현황 (tray_manager_node -> fleet_dispatch_node, logger_node)
int32 tray_id
int32 capacity           # 6칸 고정
int32 filled_slots       # 시작 시 3개(50%) 사전 적재, 이후 누적
bool ready_for_transport # 만재/섹터 완료 등 운반 트리거 조건 충족 시 true
