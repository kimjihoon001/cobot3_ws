#!/usr/bin/env python3
"""Nav2로 수확 위치까지 이동해 MoveIt 마찰 파지 후 시작점으로 복귀한다."""

from __future__ import annotations

import math
import time

import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist

from grasp_proto import HOME_Q, Grasp, harvest_once


class NavHarvestDemo:
    """Nav2와 기존 MoveIt 수확 프로토타입을 한 사이클로 묶는다."""

    def __init__(self, node: Grasp):
        self.node = node
        node.declare_parameter("goal_x", 1.0)
        node.declare_parameter("goal_y", 0.0)
        node.declare_parameter("goal_yaw", 0.0)
        node.declare_parameter("nav_frame", "map")
        node.declare_parameter("base_frame", "mm_base")
        node.declare_parameter("nav_timeout_sec", 180.0)
        node.declare_parameter("server_timeout_sec", 30.0)
        node.declare_parameter("nav_xy_tolerance", 0.20)
        # Nav2 SimpleGoalChecker 기본 yaw 허용치와 맞춘다. 이보다 엄격하면 Nav2는
        # Goal succeeded인데 오케스트레이터만 계속 기다리는 이중 판정이 된다.
        node.declare_parameter("nav_yaw_tolerance", 0.30)
        node.declare_parameter("fruit_wait_sec", 10.0)
        node.declare_parameter("return_to_start", True)
        node.declare_parameter("harvest_at_current_pose", False)
        node.declare_parameter("stow_before_nav", True)
        node.declare_parameter("start_x", float("nan"))
        node.declare_parameter("start_y", float("nan"))
        node.declare_parameter("start_yaw", float("nan"))

        # 상대 토픽은 PushRosNamespace(harvester_moveit)가 격리한다.
        # 이 PC의 Fast-DDS에서는 외부 NavigateToPose goal response가 서버에 도착한 뒤
        # 응답만 timeout 나는 현상이 있다. bt_navigator가 기본 제공하는 goal_pose 입력은
        # 내부 액션 클라이언트를 사용하므로 RViz와 통합 데모 모두 이 경로로 통일한다.
        self.goal_pub = node.create_publisher(PoseStamped, "goal_pose", 10)
        self.amcl_pose: PoseStamped | None = None
        node.create_subscription(
            PoseWithCovarianceStamped, "amcl_pose", self._amcl_pose, 10)
        self.stop_pub = node.create_publisher(Twist, "cmd_vel", 10)
        self.nav_action_status: int | None = None
        self.nav_seen_active = False
        node.create_subscription(
            GoalStatusArray, "navigate_to_pose/_action/status",
            self._nav_status, 10)

    def _amcl_pose(self, message: PoseWithCovarianceStamped) -> None:
        pose = PoseStamped()
        pose.header = message.header
        pose.pose = message.pose.pose
        self.amcl_pose = pose

    def _nav_status(self, message: GoalStatusArray) -> None:
        # status_list의 마지막 항목이 goal_pose가 만든 최신 NavigateToPose 목표다.
        if message.status_list:
            self.nav_action_status = int(message.status_list[-1].status)

    def _spin_future(self, future, timeout_sec: float) -> bool:
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        return future.done()

    def _capture_start(self) -> PoseStamped | None:
        nav_frame = str(self.node.get_parameter("nav_frame").value)
        deadline = time.monotonic() + 3.0
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if self.amcl_pose is not None:
                pose = PoseStamped()
                pose.header.frame_id = nav_frame
                pose.header.stamp = self.node.get_clock().now().to_msg()
                pose.pose = self.amcl_pose.pose
                return pose
        x = float(self.node.get_parameter("start_x").value)
        y = float(self.node.get_parameter("start_y").value)
        yaw = float(self.node.get_parameter("start_yaw").value)
        if all(math.isfinite(v) for v in (x, y, yaw)):
            pose = PoseStamped()
            pose.header.frame_id = nav_frame
            pose.header.stamp = self.node.get_clock().now().to_msg()
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.z = math.sin(yaw * 0.5)
            pose.pose.orientation.w = math.cos(yaw * 0.5)
            self.node.get_logger().warn(
                f"AMCL 시작 pose 미수신 — 설정값 ({x:.2f}, {y:.2f})을 복귀점으로 사용")
            return pose
        self.node.get_logger().error(
            f"시작 위치 AMCL pose를 받지 못했고 fallback도 없습니다: {nav_frame}")
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

    @staticmethod
    def _yaw(orientation) -> float:
        return math.atan2(
            2.0 * (orientation.w * orientation.z
                   + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y * orientation.y
                         + orientation.z * orientation.z),
        )

    def navigate(self, pose: PoseStamped, label: str) -> bool:
        pose.header.stamp = self.node.get_clock().now().to_msg()
        self.node.get_logger().info(
            f"[{label}] Nav2 목표: ({pose.pose.position.x:.2f}, "
            f"{pose.pose.position.y:.2f}) [{pose.header.frame_id}]")
        # discovery 전에 한 번 보내 유실되는 것만 막고, 목표는 정확히 한 번 발행한다.
        # 같은 PoseStamped를 반복하면 bt_navigator가 매번 새 목표로 해석해 재시작한다.
        discovery_deadline = time.monotonic() + float(
            self.node.get_parameter("server_timeout_sec").value)
        while (rclpy.ok() and self.goal_pub.get_subscription_count() == 0
               and time.monotonic() < discovery_deadline):
            rclpy.spin_once(self.node, timeout_sec=0.1)
        if self.goal_pub.get_subscription_count() == 0:
            self.node.get_logger().error(
                f"[{label}] Nav2 goal_pose 구독자가 없습니다")
            return False
        self.nav_action_status = None
        self.nav_seen_active = False
        self.goal_pub.publish(pose)

        nav_timeout = float(self.node.get_parameter("nav_timeout_sec").value)
        xy_tol = float(self.node.get_parameter("nav_xy_tolerance").value)
        yaw_tol = float(self.node.get_parameter("nav_yaw_tolerance").value)
        target_yaw = self._yaw(pose.pose.orientation)
        deadline = time.monotonic() + nav_timeout
        settled = 0
        last_log = 0.0
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if self.amcl_pose is None:
                continue
            current = self.amcl_pose.pose
            dx = pose.pose.position.x - current.position.x
            dy = pose.pose.position.y - current.position.y
            distance = math.hypot(dx, dy)
            yaw_error = abs(math.atan2(
                math.sin(target_yaw - self._yaw(current.orientation)),
                math.cos(target_yaw - self._yaw(current.orientation))))
            settled = settled + 1 if distance <= xy_tol and yaw_error <= yaw_tol else 0
            now = time.monotonic()
            if now - last_log >= 2.0:
                self.node.get_logger().info(
                    f"[{label}] 남은 거리={distance:.2f}m, "
                    f"방향오차={math.degrees(yaw_error):.1f}°")
                last_log = now
            status = self.nav_action_status
            if status in (
                    GoalStatus.STATUS_ACCEPTED,
                    GoalStatus.STATUS_EXECUTING,
                    GoalStatus.STATUS_CANCELING):
                self.nav_seen_active = True
            if self.nav_seen_active and status == GoalStatus.STATUS_SUCCEEDED:
                self.stop_pub.publish(Twist())
                self.node.get_logger().info(f"[{label}] 도착 완료")
                return True
            if (self.nav_seen_active
                    and status in (
                        GoalStatus.STATUS_CANCELED,
                        GoalStatus.STATUS_ABORTED)):
                self.stop_pub.publish(Twist())
                self.node.get_logger().error(
                    f"[{label}] Nav2 액션 실패(status={status})")
                return False
        self.stop_pub.publish(Twist())
        self.node.get_logger().error(f"[{label}] 주행 시간 초과")
        return False

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
        if (not current_only
                and bool(self.node.get_parameter("stow_before_nav").value)):
            self.node.get_logger().info("Nav2 주행 전 팔을 차체 안쪽 TRAVEL_HOME으로 접습니다")
            if not self.node.run_goal(
                    self.node.goal_joints(
                        HOME_Q, vel=0.20, pipeline="ompl", planner=""),
                    "TRAVEL_HOME(OMPL)"):
                self.node.get_logger().error("주행용 접힘 자세 이동 실패 — Nav2 출발 금지")
                return False
        start = self._capture_start() if not current_only else None
        if not current_only and start is None:
            # 시작좌표 캡처(복귀용)만 실패 — 주행·수확은 Nav2 액션이라 TF 없이도 되니 계속.
            self.node.get_logger().warn("시작좌표 캡처 실패 — 복귀 없이 주행·수확만 진행")
        if not current_only and not self.navigate(self._goal_pose(), "수확지 이동"):
            return False
        if not self.wait_for_fruit():
            return False

        # ★주행 도착 후 반복 수확 (YOLO 게이트 + 부착모드) — 도달권 과실 N개.
        import json
        import os
        from std_msgs.msg import String
        attach = os.environ.get("ATTACH_GRASP") == "1"
        yolo_gate = os.environ.get("YOLO_GATE") == "1"
        n = int(os.environ.get("HARVEST_N", "3"))
        if attach:      # 이전 부착 과실 놓기
            self.node.cmd.publish(String(data=json.dumps({"detach_grasp": True})))
            self.node.spin_for(1.0)
        done = 0
        for i in range(n):
            self.node.get_logger().info(f"===== 수확 {i + 1}/{n} =====")
            if yolo_gate:
                self.node.get_logger().info("[YOLO] 토마토 탐지 대기...")
                if not self.node.wait_yolo(timeout=20.0):
                    self.node.get_logger().info("[YOLO] 탐지 타임아웃 — 중단")
                    break
                self.node.get_logger().info("[YOLO] tomato 탐지! → 파지")
            if not harvest_once(self.node):
                break
            done += 1
            if attach:      # 다음 과실 위해 놓기
                self.node.cmd.publish(String(data=json.dumps({"detach_grasp": True})))
                self.node.spin_for(1.5)
        self.node.get_logger().info(f"총 {done}/{n} 수확 완료")
        if done == 0:
            self.node.get_logger().error("수확 0 — 복귀 안 함")
            return False

        if (start is not None
                and bool(self.node.get_parameter("return_to_start").value)):
            self.node.get_logger().info("과실 부착한 채 시작점으로 복귀합니다")
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
        if rclpy.ok():
            demo.stop_pub.publish(Twist())
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
