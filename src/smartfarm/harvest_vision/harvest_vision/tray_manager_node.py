import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import Int32

from smartfarm_interfaces.msg import TrayStatus, TransportRequest

# FR9: 트레이는 시작 시 50%(3개) 사전 적재, 추가 2~3개 수확 시 만재 간주 -> 운반 요청
CAPACITY = 6
INITIAL_FILLED = 3
TRANSPORT_TRIGGER_FILLED = 5  # TODO: 2~3개 중 정확한 임계값은 트랙 B와 협의 확정


class TrayManagerNode(Node):
    def __init__(self):
        """트레이 적재량 추적 + 운반 요청 큐 발행 초안"""
        super().__init__("tray_manager_node")

        self._filled_by_tray: dict[int, int] = {}

        self.create_subscription(Int32, "/tray/place_request", self._place_request_callback, 10)

        self._status_pub = self.create_publisher(TrayStatus, "/tray/status", 10)
        self._transport_request_pub = self.create_publisher(TransportRequest, "/dispatch/transport_request", 10)

        self.get_logger().info("tray_manager_node 시작 (초안): 섹터->pickup_pose 매핑은 TODO")

    def _place_request_callback(self, msg: Int32):
        tray_id = msg.data
        filled = self._filled_by_tray.get(tray_id, INITIAL_FILLED) + 1
        self._filled_by_tray[tray_id] = filled

        ready = filled >= TRANSPORT_TRIGGER_FILLED
        self._publish_status(tray_id, filled, ready)

        if ready:
            self._publish_transport_request(tray_id)

    def _publish_status(self, tray_id: int, filled: int, ready: bool):
        status = TrayStatus()
        status.tray_id = tray_id
        status.capacity = CAPACITY
        status.filled_slots = filled
        status.ready_for_transport = ready
        self._status_pub.publish(status)

    def _publish_transport_request(self, tray_id: int):
        """TODO: 섹터 ID·pickup_pose는 harvest_fsm_node의 현재 섹터 좌표를 받아와 채워야 함"""
        request = TransportRequest()
        request.tray_id = tray_id
        request.sector_id = 0
        request.pickup_pose = PoseStamped()
        request.requested_at = Time()
        self._transport_request_pub.publish(request)


def main():
    rclpy.init()
    node = TrayManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
