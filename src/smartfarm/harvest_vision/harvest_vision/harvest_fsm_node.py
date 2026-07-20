import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Int32, String

from smartfarm_interfaces.msg import TomatoDetectionArray

# 6.4절 확정: 웨이포인트 정지형. 섹터 이동 -> 정지 -> 검출 -> (옵션 필터) -> Pick&Place -> 트레이 적재 -> 다음 섹터
STATE_MOVE_TO_SECTOR = "MOVE_TO_SECTOR"
STATE_DETECT = "DETECT"
STATE_PICK_PLACE = "PICK_PLACE"
STATE_IDLE = "IDLE"


class HarvestFsmNode(Node):
    def __init__(self):
        """수확 State Machine 초안: 섹터 웨이포인트 순회 + Pick&Place 트리거 뼈대 (실제 팔 제어는 TODO)"""
        super().__init__("harvest_fsm_node")

        self.declare_parameter("sector_waypoints", 0)  # TODO: 섹터 좌표 목록(yaml 파라미터)으로 대체

        self._state = STATE_IDLE
        self._latest_detections: TomatoDetectionArray | None = None
        self._tray_id = 1  # TODO: tray_manager_node와 협의해 트레이 식별 방식 확정

        self._nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        self.create_subscription(
            TomatoDetectionArray, "/vision/tomato_detections", self._detections_callback, 10
        )

        self._state_pub = self.create_publisher(String, "/harvest/state", 10)
        self._place_request_pub = self.create_publisher(Int32, "/tray/place_request", 10)

        self.get_logger().info("harvest_fsm_node 시작 (초안): 상태 전이 로직은 TODO")

    def _detections_callback(self, msg: TomatoDetectionArray):
        self._latest_detections = msg

    def _publish_state(self, state: str):
        self._state = state
        self._state_pub.publish(String(data=state))

    def _move_to_sector(self, sector_pose: PoseStamped):
        """TODO: NavigateToPose 액션 goal 전송, 도착 콜백에서 STATE_DETECT로 전이"""
        goal = NavigateToPose.Goal()
        goal.pose = sector_pose
        self._nav_client.wait_for_server()
        self._nav_client.send_goal_async(goal)

    def _pick_and_place(self):
        """TODO: 그리퍼 Pick&Place 시퀀스 실행 후 /tray/place_request 발행"""
        self._place_request_pub.publish(Int32(data=self._tray_id))


def main():
    rclpy.init()
    node = HarvestFsmNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
