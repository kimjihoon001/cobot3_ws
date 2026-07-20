# 운반 요청 큐 이벤트 (tray_manager_node -> fleet_dispatch_node)
int32 tray_id
int32 sector_id
geometry_msgs/PoseStamped pickup_pose   # 운반 AMR이 트레이를 픽업할 정적맵 좌표
builtin_interfaces/Time requested_at
