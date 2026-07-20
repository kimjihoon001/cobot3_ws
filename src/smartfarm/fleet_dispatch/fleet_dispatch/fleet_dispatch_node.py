import rclpy
from rclpy.node import Node


class FleetDispatchNode(Node):
    def __init__(self):
        """운반 요청 Queue를 FIFO로 처리해 유휴 운반 AMR에 배차 (트랙 B 담당 - 구현 TODO)"""
        super().__init__("fleet_dispatch_node")
        # TODO: 트랙 B - /dispatch/transport_request 구독, AMR 배차, nav2 goal 전송 등
        # 참고: src/smartfarm/INTERFACES.md의 TransportRequest 메시지 스펙


def main():
    rclpy.init()
    node = FleetDispatchNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
