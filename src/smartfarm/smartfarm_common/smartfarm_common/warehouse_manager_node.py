import rclpy
from rclpy.node import Node


class WarehouseManagerNode(Node):
    def __init__(self):
        """창고 6슬롯 할당, 재배 섹터 1:1 매핑, 하역 완료 기록 (A+B 공동 스켈레톤, 로직은 트랙 C가 채움 - TODO)"""
        super().__init__("warehouse_manager_node")
        # TODO: /handoff/tray_ready 구독, 섹터<->슬롯 매핑 규칙표 적용, /warehouse/slot_assignment 발행
        # 참고: src/smartfarm/INTERFACES.md


def main():
    rclpy.init()
    node = WarehouseManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
