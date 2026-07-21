# 토마토 1개체 검출 결과 (vision_node -> harvest_fsm_node)
geometry_msgs/PoseStamped pose      # eye-in-hand 카메라 기준 pick point 추정 pose
string tomato_class                 # "tomato"(접근 중) | "ripe" | "spoiled"(근거리 확정)
float32 confidence                  # YOLO confidence score (0.0~1.0)
