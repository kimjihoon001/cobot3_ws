"""카메라 좌표의 토마토 목표를 따라 최종 접근하는 visual-servo 노드."""

from __future__ import annotations

import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.node import Node
from sensor_msgs.msg import JointState


ARM_JOINTS = (
    "joint_1",
    "joint_2",
    "joint_3",
    "joint_4",
    "joint_5",
    "joint_6",
)
# isaacpjt/robots/harvester.py::HOME_POSE_DEG. --mm 스폰 직후 대기·이동 자세다.
OBSERVATION_POSE_RAD = tuple(
    math.radians(value) for value in (0.0, 0.0, 60.0, 0.0, 75.0, -90.0)
)


class TargetApproachNode(Node):
    """Nav2가 섹터까지 이동한 뒤, 정면 카메라 목표에만 저속 접근한다.

    ``main.py --mm`` 스폰 직후 HOME을 기본 관찰 자세로 사용한다. 필요하면 카메라
    광축을 유지한 채 팔의 높이를 바꿔 관찰할 수 있다. 엄격한 HOME 관절각 검사는
    ``require_home_pose``를 켠 경우에만 적용한다.
    """

    def __init__(self):
        super().__init__("target_approach_node")
        self.declare_parameter("target_topic", "/vision/approach_target")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("joint_states_topic", "/harvester_0/joint_states")
        self.declare_parameter("enabled", False)
        self.declare_parameter("require_home_pose", False)
        self.declare_parameter("arm_pose_tolerance_deg", 7.0)
        self.declare_parameter("stop_distance_m", 0.50)
        self.declare_parameter("max_linear_speed", 0.15)
        self.declare_parameter("max_angular_speed", 1.05)
        self.declare_parameter("linear_gain", 0.5)
        self.declare_parameter("angular_gain", 3.6)
        self.declare_parameter("heading_tolerance_rad", 0.08)
        self.declare_parameter("target_timeout_sec", 0.5)

        target_topic = str(self.get_parameter("target_topic").value)
        cmd_topic = str(self.get_parameter("cmd_vel_topic").value)
        joint_topic = str(self.get_parameter("joint_states_topic").value)
        self._cmd_pub = self.create_publisher(Twist, cmd_topic, 10)
        self.create_subscription(PoseStamped, target_topic, self._target_callback, 10)
        self.create_subscription(JointState, joint_topic, self._joints_callback, 10)
        self.create_timer(0.05, self._control)
        self._target: PoseStamped | None = None
        self._received_at = 0.0
        self._arm_in_observation_pose = False
        self._was_moving = False
        self.get_logger().info(
            f"visual approach: {target_topic} -> {cmd_topic}, "
            f"enabled={self.get_parameter('enabled').value}"
        )

    def _target_callback(self, msg: PoseStamped):
        self._target = msg
        self._received_at = time.monotonic()

    def _joints_callback(self, msg: JointState):
        positions = dict(zip(msg.name, msg.position))
        if any(name not in positions for name in ARM_JOINTS):
            self._arm_in_observation_pose = False
            return
        tolerance = math.radians(
            float(self.get_parameter("arm_pose_tolerance_deg").value)
        )
        self._arm_in_observation_pose = all(
            abs(self._angle_difference(positions[name], expected)) <= tolerance
            for name, expected in zip(ARM_JOINTS, OBSERVATION_POSE_RAD)
        )

    def _control(self):
        if not bool(self.get_parameter("enabled").value):
            self._stop_if_needed()
            return
        require_pose = bool(self.get_parameter("require_home_pose").value)
        if require_pose and not self._arm_in_observation_pose:
            self._stop_if_needed()
            return
        timeout = float(self.get_parameter("target_timeout_sec").value)
        if self._target is None or time.monotonic() - self._received_at > timeout:
            self._stop_if_needed()
            return

        x = float(self._target.pose.position.x)
        distance = float(self._target.pose.position.z)
        if distance <= 0.0:
            self._stop_if_needed()
            return

        stop_distance = float(self.get_parameter("stop_distance_m").value)
        heading = math.atan2(x, distance)
        cmd = Twist()
        angular_gain = float(self.get_parameter("angular_gain").value)
        max_angular = float(self.get_parameter("max_angular_speed").value)
        cmd.angular.z = max(-max_angular, min(max_angular, -angular_gain * heading))

        if distance > stop_distance:
            linear_gain = float(self.get_parameter("linear_gain").value)
            max_linear = float(self.get_parameter("max_linear_speed").value)
            speed = min(max_linear, linear_gain * (distance - stop_distance))
            tolerance = float(self.get_parameter("heading_tolerance_rad").value)
            if abs(heading) > tolerance:
                speed *= max(0.0, math.cos(heading)) * 0.35
            cmd.linear.x = speed
        else:
            cmd.angular.z = 0.0

        self._cmd_pub.publish(cmd)
        self._was_moving = bool(cmd.linear.x or cmd.angular.z)

    def _stop_if_needed(self):
        if self._was_moving:
            self._cmd_pub.publish(Twist())
            self._was_moving = False

    @staticmethod
    def _angle_difference(actual: float, expected: float) -> float:
        return math.atan2(math.sin(actual - expected), math.cos(actual - expected))

    def destroy_node(self):
        self._cmd_pub.publish(Twist())
        return super().destroy_node()


def main():
    rclpy.init()
    node = TargetApproachNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
