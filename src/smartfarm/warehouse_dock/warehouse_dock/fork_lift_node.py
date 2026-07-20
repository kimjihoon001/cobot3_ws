import rclpy
from rclpy.node import Node


class ForkLiftNode(Node):
    def __init__(self):
        """지게차 AMR 포크 승강 제어 - 수취/삽입/인출, 창고 랙 2단 레벨 위치 제어 (트랙 C 담당 - 구현 TODO)"""
        super().__init__("fork_lift_node")
        # TODO: 트랙 C - /handoff/tray_ready, /warehouse/slot_assignment 구독 후 포크 제어
        # 참고: src/smartfarm/INTERFACES.md


def main():
    rclpy.init()
    node = ForkLiftNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
