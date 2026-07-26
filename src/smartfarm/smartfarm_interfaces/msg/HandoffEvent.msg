# 운반 AMR 인계 위치 도착 -> 지게차 활성화 트리거.
# [폐기 2026-07-26] 발행자였던 handoff_node.py는 삭제됐고, 이 역할은
# iwhub_control/mission_nav_node.py의 _request_forklift_cycle()이 서비스
# /forklift/start_cycle(ForkliftCycle) 호출로 대신한다. 현재 이 메시지를
# 발행하는 노드는 없다 — fork_lift_node.py의 구독만 레거시로 남아 있다.
int32 tray_id
string amr_id
geometry_msgs/PoseStamped handoff_pose   # 인계 위치 고정 웨이포인트
