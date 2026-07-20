import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import Int32

from smartfarm_interfaces.msg import TrayStatus, TransportRequest

# 브리핑 문서 FR9는 "시작 시 50%(3개) 사전 적재"였지만, 실제 프로젝트 설정
# (isaacpjt/pjt_config/settings.py TrayConfig)은 preloaded=0으로 확정돼 있다 —
# "사전 적재는 측정 안 한 구간을 성공으로 세는 것 -> 정량 검증엔 0이 유일하게 정합적"이라는 이유.
# capacity=6은 두 문서가 일치. 아래는 그 결정을 따름.
CAPACITY = 6
INITIAL_FILLED = 0
TRANSPORT_TRIGGER_FILLED = CAPACITY  # TODO: 부분 적재 시점에 운반을 트리거할지는 트랙 B와 협의 필요 (지금은 만재 기준)


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
