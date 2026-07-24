"""Nav2 Twist를 Isaac MM에 안전하게 중계한다.

Isaac ROS2SubscribeTwist는 마지막 값을 유지하므로 Nav2가 죽으면 마지막 속도로 계속
적분될 수 있다. 이 노드는 입력이 끊긴 뒤 timeout_sec가 지나면 0을 계속 발행한다.
"""

import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class CmdVelWatchdog(Node):
    def __init__(self):
        super().__init__("cmd_vel_watchdog")
        self.declare_parameter("input_topic", "cmd_vel")
        self.declare_parameter("output_topic", "cmd_vel_safe")
        self.declare_parameter("timeout_sec", 0.35)
        self.declare_parameter("publish_rate_hz", 20.0)

        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        rate = float(self.get_parameter("publish_rate_hz").value)
        self._timeout = float(self.get_parameter("timeout_sec").value)
        self._last = Twist()
        self._last_received = 0.0
        self._publisher = self.create_publisher(Twist, output_topic, 10)
        self.create_subscription(Twist, input_topic, self._command, 10)
        self.create_timer(1.0 / rate, self._publish)
        self.get_logger().info(
            f"속도 watchdog: {input_topic} -> {output_topic}, "
            f"timeout={self._timeout:.2f}s")

    def _command(self, message: Twist) -> None:
        self._last = message
        self._last_received = time.monotonic()

    def _publish(self) -> None:
        if time.monotonic() - self._last_received > self._timeout:
            self._publisher.publish(Twist())
        else:
            self._publisher.publish(self._last)

    def destroy_node(self):
        # 정상 종료 때도 Isaac 구독 출력이 반드시 0으로 갱신되게 한다.
        if rclpy.ok(context=self.context):
            for _ in range(3):
                self._publisher.publish(Twist())
        return super().destroy_node()


def main():
    rclpy.init()
    node = CmdVelWatchdog()
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
