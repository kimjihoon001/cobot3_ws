# 운반 AMR 인계 위치 도착 -> 지게차 활성화 트리거 (handoff_node -> fork_lift_node, warehouse_manager_node)
int32 tray_id
string amr_id
geometry_msgs/PoseStamped handoff_pose   # 인계 위치 고정 웨이포인트
