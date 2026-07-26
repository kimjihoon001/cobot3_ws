#!/usr/bin/env python3
"""실행 중인 IW를 재시작 없이 지게차 도크 pose로 직접 정렬한다."""

import math
import time

import rclpy
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_msgs.msg import TFMessage


def yaw_from_quaternion(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class AlignOnce(Node):
    def __init__(self) -> None:
        super().__init__("iw_dock_align_once")
        self.target = (0.0, 10.84885, math.pi)
        self.odom = None
        self.map_to_odom = None
        self.stage = "POSITION"
        self.stable_since = None
        self.started = time.monotonic()
        self.last_log = 0.0
        self.done = False
        self.pub = self.create_publisher(
            Twist, "/iwhub_0/cmd_vel_nav", 10
        )
        self.create_subscription(
            Odometry, "/iwhub_0/odom", self.on_odom, 20
        )
        self.create_subscription(
            TFMessage, "/iwhub_0/tf", self.on_tf, 50
        )
        self.cancel = self.create_client(
            CancelGoal,
            "/iwhub_0/navigate_through_poses/_action/cancel_goal",
        )
        self.cancel_sent = False
        self.create_timer(0.05, self.update)

    def on_odom(self, msg: Odometry) -> None:
        self.odom = (
            float(msg.pose.pose.position.x),
            float(msg.pose.pose.position.y),
            yaw_from_quaternion(msg.pose.pose.orientation),
        )

    def on_tf(self, msg: TFMessage) -> None:
        for item in msg.transforms:
            if (
                item.header.frame_id.lstrip("/") == "iwhub_0/map"
                and item.child_frame_id.lstrip("/") == "iwhub_0/odom"
            ):
                self.map_to_odom = (
                    float(item.transform.translation.x),
                    float(item.transform.translation.y),
                    yaw_from_quaternion(item.transform.rotation),
                )

    def pose(self):
        if self.odom is None or self.map_to_odom is None:
            return None
        ox, oy, oyaw = self.odom
        mx, my, myaw = self.map_to_odom
        c, s = math.cos(myaw), math.sin(myaw)
        return (
            mx + c * ox - s * oy,
            my + s * ox + c * oy,
            wrap(myaw + oyaw),
        )

    def publish(self, linear: float, angular: float) -> None:
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.pub.publish(msg)

    def update(self) -> None:
        if self.done:
            return
        if not self.cancel_sent and self.cancel.service_is_ready():
            self.cancel.call_async(CancelGoal.Request())
            self.cancel_sent = True
            self.get_logger().info("기존 NavigateThroughPoses goal 전체 취소 요청")
        pose = self.pose()
        if pose is None:
            if time.monotonic() - self.started > 10.0:
                self.get_logger().error("IW map pose 수신 실패")
                self.done = True
            return
        x, y, yaw = pose
        tx, ty, tyaw = self.target
        ex, ey = tx - x, ty - y
        xy = math.hypot(ex, ey)
        yaw_error = wrap(tyaw - yaw)
        linear = angular = 0.0

        if self.stage == "POSITION":
            if xy <= 0.035:
                self.stage = "YAW"
                self.publish(0.0, 0.0)
                self.get_logger().info(
                    f"위치 정렬 완료: xy={xy:.3f}m → yaw 정렬"
                )
                return
            bearing = math.atan2(ey, ex)
            heading_error = wrap(bearing - yaw)
            direction = 1.0
            if abs(heading_error) > math.pi / 2.0:
                direction = -1.0
                heading_error = wrap(heading_error - math.pi)
            angular = max(-0.35, min(0.35, 1.2 * heading_error))
            if abs(heading_error) <= math.radians(12.0):
                linear = direction * min(0.10, max(0.025, 0.8 * xy))
        else:
            if xy > 0.060:
                self.stage = "POSITION"
                self.stable_since = None
                return
            angular = max(-0.45, min(0.45, 1.0 * yaw_error))
            if abs(yaw_error) > math.radians(2.0) and abs(angular) < 0.05:
                angular = math.copysign(0.05, yaw_error)
            if abs(yaw_error) <= math.radians(2.0):
                linear = angular = 0.0
                if self.stable_since is None:
                    self.stable_since = time.monotonic()
                elif time.monotonic() - self.stable_since >= 1.0:
                    self.publish(0.0, 0.0)
                    self.get_logger().info(
                        "정렬 완료: "
                        f"pose=({x:.3f},{y:.3f},"
                        f"{math.degrees(yaw):.1f}°), "
                        f"xy={xy:.3f}m, "
                        f"yaw_error={math.degrees(yaw_error):.1f}°"
                    )
                    self.done = True
                    return
            else:
                self.stable_since = None

        self.publish(linear, angular)
        now = time.monotonic()
        if now - self.last_log >= 1.0:
            self.last_log = now
            self.get_logger().info(
                f"{self.stage}: pose=({x:.3f},{y:.3f},"
                f"{math.degrees(yaw):.1f}°), "
                f"xy={xy:.3f}m, yaw={math.degrees(yaw_error):.1f}°, "
                f"cmd=({linear:.3f},{angular:.3f})"
            )
        if now - self.started > 60.0:
            self.publish(0.0, 0.0)
            self.get_logger().error("정렬 60초 시간 초과")
            self.done = True


def main() -> None:
    rclpy.init()
    node = AlignOnce()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.publish(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
