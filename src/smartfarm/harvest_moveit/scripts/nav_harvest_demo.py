#!/usr/bin/env python3
"""Nav2로 수확 위치까지 이동해 MoveIt 마찰 파지 후 시작점으로 복귀한다."""

from __future__ import annotations

import json
import math
import os
import time

import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from std_msgs.msg import String

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
        node.declare_parameter("nav_yaw_tolerance", 0.35)
        node.declare_parameter("fruit_wait_sec", 10.0)
        # 수확 후 복귀는 grasp_proto.harvest_once()가 수행하는 팔 HOME(OMPL)을 뜻한다.
        # 모바일 베이스까지 Nav2 출발점으로 되돌리면 다음 과실 수확 때 같은 주행을
        # 반복하고 사용자가 기대한 "처음 자세"와도 다르므로 기본값은 끈다.
        node.declare_parameter("return_to_start", False)
        node.declare_parameter("harvest_at_current_pose", False)
        node.declare_parameter("stow_before_nav", True)
        # 이동 중 카메라가 작업거리의 토마토를 연속 검출하면 목적지까지 가지 않고
        # 즉시 Nav2를 취소한 뒤 그 자리에서 수확한다.
        node.declare_parameter("stop_on_tomato", True)
        node.declare_parameter("detection_confirm_frames", 3)
        node.declare_parameter("detection_stop_min_depth_m", 0.10)
        node.declare_parameter("detection_stop_max_depth_m", 0.65)
        node.declare_parameter("stop_settle_sec", 0.8)
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
        # controller 출력과 Isaac 입력 양쪽에 0을 넣을 수 있어야 취소 응답을 기다리는
        # 동안에도 마지막 속도가 유지되지 않는다.
        self.nav_stop_pub = node.create_publisher(Twist, "cmd_vel_nav", 10)
        self.stop_pub = node.create_publisher(Twist, "cmd_vel_safe", 10)
        self.cancel_client = node.create_client(
            CancelGoal, "navigate_to_pose/_action/cancel_goal")
        self.nav_action_status: int | None = None
        self.nav_seen_active = False
        self.stopped_for_detection = False
        self._vision_depth: float | None = None
        self._vision_pose_time = 0.0
        self._detection_hits = 0
        node.create_subscription(
            GoalStatusArray, "navigate_to_pose/_action/status",
            self._nav_status, 10)
        node.create_subscription(
            PoseStamped, "vision/approach_target", self._vision_target, 10)
        node.create_subscription(
            String, "vision/target_class", self._vision_class, 10)

    def _amcl_pose(self, message: PoseWithCovarianceStamped) -> None:
        pose = PoseStamped()
        pose.header = message.header
        pose.pose = message.pose.pose
        self.amcl_pose = pose

    def _nav_status(self, message: GoalStatusArray) -> None:
        # status_list의 마지막 항목이 goal_pose가 만든 최신 NavigateToPose 목표다.
        if message.status_list:
            self.nav_action_status = int(message.status_list[-1].status)

    def _vision_target(self, message: PoseStamped) -> None:
        depth = float(message.pose.position.z)
        self._vision_depth = depth if math.isfinite(depth) and depth > 0.0 else None
        self._vision_pose_time = time.monotonic()

    def _vision_class(self, message: String) -> None:
        valid_class = message.data in {"tomato", "quality_check", "ripe"}
        fresh_pose = time.monotonic() - self._vision_pose_time < 0.5
        min_depth = float(
            self.node.get_parameter("detection_stop_min_depth_m").value)
        max_depth = float(
            self.node.get_parameter("detection_stop_max_depth_m").value)
        valid_depth = (
            self._vision_depth is not None
            and min_depth <= self._vision_depth <= max_depth
        )
        self._detection_hits = (
            self._detection_hits + 1
            if valid_class and fresh_pose and valid_depth
            else 0
        )

    def _stop_base(self, duration: float = 0.0) -> None:
        """Nav2 입력과 Isaac 최종 입력을 동시에 0으로 만들어 확실히 정지한다."""
        deadline = time.monotonic() + max(0.0, duration)
        while rclpy.ok():
            zero = Twist()
            self.nav_stop_pub.publish(zero)
            self.stop_pub.publish(zero)
            if time.monotonic() >= deadline:
                break
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def _cancel_navigation(self) -> None:
        """현재 namespace의 NavigateToPose 목표를 모두 취소하고 베이스를 정지한다."""
        future = None
        if self.cancel_client.wait_for_service(timeout_sec=1.0):
            # UUID=0, stamp=0은 현재 액션 서버의 모든 목표 취소 요청이다.
            future = self.cancel_client.call_async(CancelGoal.Request())
        else:
            self.node.get_logger().warn(
                "Nav2 cancel 서비스가 없어 속도 명령만 0으로 정지합니다")

        settle = float(self.node.get_parameter("stop_settle_sec").value)
        deadline = time.monotonic() + max(settle, 0.2)
        while rclpy.ok() and time.monotonic() < deadline:
            self._stop_base()
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if future is not None and future.done():
                future = None
        self._stop_base()

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

    def navigate(
            self, pose: PoseStamped, label: str,
            stop_on_detection: bool = False) -> bool:
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
        self.stopped_for_detection = False
        self._detection_hits = 0
        self.goal_pub.publish(pose)

        nav_timeout = float(self.node.get_parameter("nav_timeout_sec").value)
        xy_tol = float(self.node.get_parameter("nav_xy_tolerance").value)
        yaw_tol = float(self.node.get_parameter("nav_yaw_tolerance").value)
        target_yaw = self._yaw(pose.pose.orientation)
        # Isaac에 여러 로봇을 함께 띄우면 simulation RTF가 1보다 크게 낮아질 수 있다.
        # 벽시계(monotonic)로 제한하면 Nav2/물리는 시뮬 시간상 아직 정상 주행 중인데도
        # 4대 통합 실행에서 먼저 180초가 지나 수확 직전에 취소된다. use_sim_time을
        # 따르는 ROS clock으로 주행 제한을 재야 시뮬 속도와 무관하게 같은 동작량을 준다.
        start_ros_ns = self.node.get_clock().now().nanoseconds
        settled = 0
        last_log = 0.0
        confirm_frames = max(
            1, int(self.node.get_parameter("detection_confirm_frames").value))
        while rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if (stop_on_detection
                    and self._detection_hits >= confirm_frames):
                depth = self._vision_depth
                self.node.get_logger().info(
                    f"[YOLO] 토마토 {confirm_frames}프레임 연속 검출"
                    + (f" (depth={depth:.2f}m)" if depth is not None else "")
                    + " — Nav2 취소·정지")
                self._cancel_navigation()
                self.stopped_for_detection = True
                return True
            now_ros_ns = self.node.get_clock().now().nanoseconds
            # Play/Stop으로 /clock이 뒤로 점프한 경우 새 시뮬 세션에서 다시 센다.
            if now_ros_ns < start_ros_ns:
                start_ros_ns = now_ros_ns
            if (now_ros_ns - start_ros_ns) * 1e-9 >= nav_timeout:
                break
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
                self._stop_base()
                self.node.get_logger().info(f"[{label}] 도착 완료")
                return True
            if (self.nav_seen_active
                    and status in (
                        GoalStatus.STATUS_CANCELED,
                        GoalStatus.STATUS_ABORTED)):
                self._stop_base()
                self.node.get_logger().error(
                    f"[{label}] Nav2 액션 실패(status={status})")
                return False
        self._cancel_navigation()
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
            self.node.get_logger().info(
                "Nav2 주행 전 팔을 식물 방향 HOME 자세로 맞춥니다")
            if not self.node.run_goal(
                    self.node.goal_joints(
                        HOME_Q, vel=0.20, pipeline="ompl", planner=""),
                    "TRAVEL_HOME(OMPL)"):
                # 팔 자세 준비 실패가 모바일 베이스의 Nav2 출발까지 막으면,
                # 컨트롤러가 잠깐 늦게 준비된 경우 전체 데모가 시작조차 못 한다.
                # 베이스 footprint에는 현재 팔 형상이 포함되어 있으므로 주행은 계속하고
                # 수확 접근 직전에 MoveIt이 현재 상태에서 다시 계획하게 한다.
                self.node.get_logger().warn(
                    "주행용 팔 자세 이동 실패 — 현재 자세로 Nav2 주행은 계속합니다")
        start = self._capture_start() if not current_only else None
        if not current_only and start is None:
            # 시작좌표 캡처(복귀용)만 실패 — 주행·수확은 Nav2 액션이라 TF 없이도 되니 계속.
            self.node.get_logger().warn("시작좌표 캡처 실패 — 복귀 없이 주행·수확만 진행")
        stop_on_tomato = bool(
            self.node.get_parameter("stop_on_tomato").value)
        if not current_only and not self.navigate(
                self._goal_pose(), "수확지 이동",
                stop_on_detection=stop_on_tomato):
            return False
        if self.stopped_for_detection:
            self.node.get_logger().info(
                "베이스 정지 완료 — 팔 수확 시퀀스로 전환합니다")
        if not self.wait_for_fruit():
            return False

        # ★주행 도착 후 반복 수확 (YOLO 게이트 + 부착모드) — 도달권 과실 N개.
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
            return self.navigate(start, "복귀", stop_on_detection=False)
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
