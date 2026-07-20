import rclpy
from rclpy.node import Node


class LoggerNode(Node):
    def __init__(self):
        """수확량, 트레이 ID, 창고 위치, 타임스탬프 기록 (A+B 공동 스켈레톤, 로직은 트랙 C가 채움 - TODO)"""
        super().__init__("logger_node")
        # TODO: /tray/status, /warehouse/slot_assignment 등 구독 후 파일/DB에 기록
        # 참고: src/smartfarm/INTERFACES.md


def main():
    rclpy.init()
    node = LoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
