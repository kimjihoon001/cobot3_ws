import rclpy
from rclpy.node import Node


class HandoffNode(Node):
    def __init__(self):
        """운반 AMR 인계 위치 도착 이벤트 -> 지게차 AMR 활성화 트리거 (트랙 C 담당 - 구현 TODO)"""
        super().__init__("handoff_node")
        # TODO: 트랙 C - AMR 도착 판정, /handoff/tray_ready 발행 등
        # 참고: src/smartfarm/INTERFACES.md의 HandoffEvent 메시지 스펙


def main():
    rclpy.init()
    node = HandoffNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
