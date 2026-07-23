#!/usr/bin/env python3
"""Nav2로 수확 위치까지 이동해 MoveIt 마찰 파지 후 시작점으로 복귀한다."""

from __future__ import annotations

import math
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.duration import Duration
from tf2_ros import Buffer, TransformException, TransformListener

from grasp_proto import Grasp, harvest_once


class NavHarvestDemo:
    """Nav2와 기존 MoveIt 수확 프로토타입을 한 사이클로 묶는다."""

    def __init__(self, node: Grasp):
        self.node = node
        node.declare_parameter("goal_x", 1.0)
        node.declare_parameter("goal_y", 0.0)
        node.declare_parameter("goal_yaw", 0.0)
        node.declare_parameter("nav_frame", "map")
        node.declare_parameter("base_frame", "base_link")
        node.declare_parameter("nav_timeout_sec", 180.0)
        node.declare_parameter("server_timeout_sec", 30.0)
        node.declare_parameter("fruit_wait_sec", 10.0)
        node.declare_parameter("return_to_start", True)
        node.declare_parameter("harvest_at_current_pose", False)

        # ★상대경로(2026-07-23): PushRosNamespace(harvester_moveit) 가 /harvester_moveit/* 로 밀도록
        #   '/' 를 뗀다(Codex 지적). TransformListener 는 /tf·/tf_static 을 **절대경로**로 구독하므로
        #   여기선 안 밀린다 — launch 의 _TF_REMAP(/tf→tf)이 격리를 담당한다(주석 정정).
        self.nav = ActionClient(node, NavigateToPose, "navigate_to_pose")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, node)
        self.stop_pub = node.create_publisher(Twist, "cmd_vel", 10)

    def _spin_future(self, future, timeout_sec: float) -> bool:
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        return future.done()

    def _capture_start(self) -> PoseStamped | None:
        nav_frame = str(self.node.get_parameter("nav_frame").value)
        base_frame = str(self.node.get_parameter("base_frame").value)
        deadline = time.monotonic() + 10.0
        while rclpy.ok() and time.monotonic() < deadline:
            try:
                tf = self.tf_buffer.lookup_transform(
                    nav_frame, base_frame, rclpy.time.Time(),
                    timeout=Duration(seconds=0.2))
                pose = PoseStamped()
                pose.header.frame_id = nav_frame
                pose.header.stamp = self.node.get_clock().now().to_msg()
                pose.pose.position.x = tf.transform.translation.x
                pose.pose.position.y = tf.transform.translation.y
                pose.pose.position.z = tf.transform.translation.z
                pose.pose.orientation = tf.transform.rotation
                return pose
            except TransformException:
                rclpy.spin_once(self.node, timeout_sec=0.1)
        self.node.get_logger().error(
            f"시작 위치 TF를 찾지 못했습니다: {nav_frame} <- {base_frame}")
        return None

    def _goal_pose(self) -> PoseStamped:
        yaw = float(self.node.get_parameter("goal_yaw").value)
        pose = PoseStamped()
        pose.header.frame_id = str(self.node.get_parameter("nav_frame").value)
        pose.header.stamp = self.node.get_clock().now().to_msg()
        pose.pose.position.x = float(self.node.get_parameter("goal_x").value)
        pose.pose.position.y = float(self.node.get_parameter("goal_y").value)
        pose.pose.orientation.z = math.sin(yaw * 0.5)
        pose.pose.orientation.w = math.cos(yaw * 0.5)
        return pose

    def navigate(self, pose: PoseStamped, label: str) -> bool:
        server_timeout = float(
            self.node.get_parameter("server_timeout_sec").value)
        if not self.nav.wait_for_server(timeout_sec=server_timeout):
            self.node.get_logger().error("Nav2 /navigate_to_pose 액션 서버가 없습니다")
            return False

        pose.header.stamp = self.node.get_clock().now().to_msg()
        goal = NavigateToPose.Goal()
        goal.pose = pose
        self.node.get_logger().info(
            f"[{label}] Nav2 목표: ({pose.pose.position.x:.2f}, "
            f"{pose.pose.position.y:.2f}) [{pose.header.frame_id}]")
        send_future = self.nav.send_goal_async(goal)
        if not self._spin_future(send_future, server_timeout):
            self.node.get_logger().error(f"[{label}] 목표 전송 시간 초과")
            return False
        handle = send_future.result()
        if handle is None or not handle.accepted:
            self.node.get_logger().error(f"[{label}] Nav2 목표가 거부됐습니다")
            return False

        result_future = handle.get_result_async()
        nav_timeout = float(self.node.get_parameter("nav_timeout_sec").value)
        if not self._spin_future(result_future, nav_timeout):
            self.node.get_logger().error(f"[{label}] 주행 시간 초과, 목표를 취소합니다")
            cancel = handle.cancel_goal_async()
            self._spin_future(cancel, 5.0)
            self.stop_pub.publish(Twist())
            return False
        result = result_future.result()
        self.stop_pub.publish(Twist())
        if result is None or result.status != GoalStatus.STATUS_SUCCEEDED:
            status = None if result is None else result.status
            self.node.get_logger().error(f"[{label}] Nav2 실패(status={status})")
            return False
        self.node.get_logger().info(f"[{label}] 도착 완료")
        return True

    def wait_for_fruit(self) -> bool:
        timeout = float(self.node.get_parameter("fruit_wait_sec").value)
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            if self.node.nearest() is not None:
                return True
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.node.get_logger().error("도착 위치에서 수확 가능한 토마토 좌표를 받지 못했습니다")
        return False

    def run(self) -> bool:
        current_only = bool(
            self.node.get_parameter("harvest_at_current_pose").value)
        start = self._capture_start() if not current_only else None
        if not current_only and start is None:
            return False
        if not current_only and not self.navigate(self._goal_pose(), "수확지 이동"):
            return False
        if not self.wait_for_fruit():
            return False

        self.node.get_logger().info("MoveIt 마찰 파지·커터 수확을 시작합니다")
        if not harvest_once(self.node):
            self.node.get_logger().error("수확 실패 — 복귀 주행을 시작하지 않습니다")
            return False

        if (start is not None
                and bool(self.node.get_parameter("return_to_start").value)):
            self.node.get_logger().info("토마토를 마찰로 파지한 채 시작점으로 복귀합니다")
            return self.navigate(start, "복귀")
        return True


def main():
    rclpy.init()
    node = Grasp()
    demo = NavHarvestDemo(node)
    try:
        server_timeout = float(node.get_parameter("server_timeout_sec").value)
        if not node.move.wait_for_server(timeout_sec=server_timeout):
            node.get_logger().error("MoveIt /move_action 서버가 없습니다")
            ok = False
        else:
            ok = demo.run()
        node.get_logger().info("NAV→수확→복귀 데모 성공" if ok else "데모 실패")
    except KeyboardInterrupt:
        demo.stop_pub.publish(Twist())
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
