"""기억한 수확 위치로 자동 이동한 뒤 m0617 MoveIt 수확을 실행하는 전용 코디네이터."""

from __future__ import annotations

import rclpy

from .nav_harvest_test_node import NavHarvestTestNode


class FixedHarvestMoveItNode(NavHarvestTestNode):
    """map=(-0.54, -8.19) 대기 위치를 기본값으로 사용하는 원샷 수확 노드."""

    def __init__(self) -> None:
        super().__init__(
            node_name="fixed_harvest_moveit_node",
            fixed_goal_defaults={
                "auto_nav_goal": True,
                "fixed_goal_x": -0.54,
                "fixed_goal_y": -8.19,
                "fixed_goal_yaw": 1.91,
                "resume_search_after_start_sec": 0.0,
                # DDS에 남은 과거 성공 goal을 이번 수확 위치 도착으로 오인하지 않는다.
                "accept_initial_succeeded_goal": False,
            },
        )
        self.get_logger().info(
            "고정 위치 MoveIt 수확 전용 모드: Nav2 도착 → 가까운 베드 관측 자세 → "
            "비전 표적 → MoveGroup 수확")


def main() -> None:
    rclpy.init()
    node = FixedHarvestMoveItNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
