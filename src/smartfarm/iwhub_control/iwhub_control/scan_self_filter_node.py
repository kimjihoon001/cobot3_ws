"""IW 차체·팔레트·KLT에서 되돌아오는 2D 라이다 자기반사를 제거한다."""
from __future__ import annotations

import copy
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class ScanSelfFilterNode(Node):
    def __init__(self) -> None:
        super().__init__("scan_self_filter")
        self.declare_parameter("self_min_x", -1.0835)
        self.declare_parameter("self_max_x", 0.4475)
        self.declare_parameter("self_min_y", -0.451)
        self.declare_parameter("self_max_y", 0.451)

        # iwhub_base.launch.py의 실제 정적 TF와 동일하다.
        self.declare_parameter("front_lidar_x", 0.65)
        self.declare_parameter("front_lidar_y", 0.0)
        self.declare_parameter("front_lidar_yaw", 0.0)
        self.declare_parameter("back_lidar_x", -1.08)
        self.declare_parameter("back_lidar_y", 0.0)
        self.declare_parameter("back_lidar_yaw", math.pi)

        self._front_pub = self.create_publisher(
            LaserScan, "front_2d_lidar/scan_filtered",
            qos_profile_sensor_data)
        self._back_pub = self.create_publisher(
            LaserScan, "back_2d_lidar/scan_filtered",
            qos_profile_sensor_data)
        self.create_subscription(
            LaserScan, "front_2d_lidar/scan",
            lambda msg: self._filter(msg, True), qos_profile_sensor_data)
        self.create_subscription(
            LaserScan, "back_2d_lidar/scan",
            lambda msg: self._filter(msg, False), qos_profile_sensor_data)
        self.get_logger().info(
            "IW scan self-filter 시작: 차체·팔레트·KLT footprint 내부 점 제거")

    def _filter(self, message: LaserScan, front: bool) -> None:
        prefix = "front" if front else "back"
        lx = float(self.get_parameter(f"{prefix}_lidar_x").value)
        ly = float(self.get_parameter(f"{prefix}_lidar_y").value)
        yaw = float(self.get_parameter(f"{prefix}_lidar_yaw").value)
        min_x = float(self.get_parameter("self_min_x").value)
        max_x = float(self.get_parameter("self_max_x").value)
        min_y = float(self.get_parameter("self_min_y").value)
        max_y = float(self.get_parameter("self_max_y").value)
        cy, sy = math.cos(yaw), math.sin(yaw)

        filtered = copy.deepcopy(message)
        angle = float(message.angle_min)
        removed = 0
        for index, distance in enumerate(message.ranges):
            if math.isfinite(distance):
                local_x = float(distance) * math.cos(angle)
                local_y = float(distance) * math.sin(angle)
                base_x = lx + cy * local_x - sy * local_y
                base_y = ly + sy * local_x + cy * local_y
                if min_x <= base_x <= max_x and min_y <= base_y <= max_y:
                    filtered.ranges[index] = math.inf
                    removed += 1
            angle += float(message.angle_increment)

        (self._front_pub if front else self._back_pub).publish(filtered)
        if removed:
            self.get_logger().debug(
                f"{prefix} scan 자기반사 {removed}점 제거")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ScanSelfFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
