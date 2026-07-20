# 창고 슬롯 배정 (warehouse_manager_node -> fork_lift_node, logger_node)
int32 tray_id
int32 slot_id     # 1~6 (3섹터 x 2단)
int32 sector_id   # 재배 섹터와 1:1 매핑
bool occupied
