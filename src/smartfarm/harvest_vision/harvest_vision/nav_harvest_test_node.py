"""IW 없이 Nav2 도착→근거리 탐색→수확→모의 바스켓 배치를 시험한다."""

from __future__ import annotations

import json

import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String


class NavHarvestTestNode(Node):
    """RViz에서 보낸 NavigateToPose 결과를 관찰하는 단독 통합시험 코디네이터."""

    def __init__(self):
        super().__init__("nav_harvest_test_node")
        self.declare_parameter(
            "nav_status_topic", "/navigate_to_pose/_action/status")
        self.declare_parameter("harvest_enable_topic", "/harvest_test/enable")
        self.declare_parameter(
            "manipulator_state_topic", "/harvester_0/manipulator/target_state")
        self.declare_parameter(
            "mobility_ready_topic", "/harvester_0/manipulator/mobility_ready")
        self.declare_parameter("basket_pose_topic", "/iw/basket/empty_slot_pose")
        self.declare_parameter("basket_frame", "harvester_0/base_link")
        # IW가 붙기 전 시험용 tool0 release pose. 실제 바스켓 중심 좌표가 아니다.
        self.declare_parameter("mock_basket_release_xyz", [0.45, -0.35, 0.45])
        self.declare_parameter("search_timeout_sec", 30.0)
        self.declare_parameter("accept_initial_succeeded_goal", True)
        self.declare_parameter("resume_search_after_start_sec", 2.0)

        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._enable_pub = self.create_publisher(
            Bool, str(self.get_parameter("harvest_enable_topic").value), latched)
        self._basket_pub = self.create_publisher(
            PoseStamped, str(self.get_parameter("basket_pose_topic").value), 10)
        self._status_pub = self.create_publisher(
            String, "/harvest_test/status", latched)
        self.create_subscription(
            GoalStatusArray,
            str(self.get_parameter("nav_status_topic").value),
            self._nav_status_callback,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("manipulator_state_topic").value),
            self._manipulator_state_callback,
            10,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter("mobility_ready_topic").value),
            self._mobility_callback,
            latched,
        )
        self.create_timer(0.2, self._watchdog)

        self._known_goals: set[bytes] = set()
        self._active_goal: bytes | None = None
        self._status_initialized = False
        self._mobility_ready = True
        self._search_deadline_ns = 0
        self._basket_sent = False
        resume = float(self.get_parameter("resume_search_after_start_sec").value)
        self._resume_deadline_ns = (
            self.get_clock().now().nanoseconds + int(resume * 1e9)
            if resume > 0.0 else 0)
        self._state = ""
        self._publish_enable(False)
        self._publish_status("READY_FOR_NAV_GOAL")

    @staticmethod
    def _uuid(entry) -> bytes:
        return bytes(entry.goal_info.goal_id.uuid)

    def _nav_status_callback(self, msg: GoalStatusArray) -> None:
        if not self._status_initialized:
            # DDS 연결이 목표 수락보다 늦으면 첫 status 배열에 이미 EXECUTING 목표가
            # 들어 있다. 이를 전부 과거 목표로 취급하면 성공 시에도 수확 게이트가
            # 영원히 READY_FOR_NAV_GOAL에 머문다. 현재 진행 중인 최신 목표는 추적한다.
            active_entries = [
                entry for entry in msg.status_list
                if entry.status in (GoalStatus.STATUS_ACCEPTED,
                                    GoalStatus.STATUS_EXECUTING)
            ]
            self._known_goals.update(self._uuid(entry) for entry in msg.status_list)
            if active_entries:
                entry = active_entries[-1]
                self._active_goal = self._uuid(entry)
                self._basket_sent = False
                self._search_deadline_ns = 0
                self._publish_enable(False)
                self._publish_status("NAVIGATING")
            elif bool(self.get_parameter("accept_initial_succeeded_goal").value):
                succeeded = [entry for entry in msg.status_list
                             if entry.status == GoalStatus.STATUS_SUCCEEDED]
                if succeeded:
                    # 통합시험 launch를 목표 도착 후 켠 경우 최신 완료 목표를 현재
                    # 정지 위치의 도착으로 받아 수확 단계부터 이어서 시험한다.
                    self._publish_enable(True)
                    timeout = float(self.get_parameter("search_timeout_sec").value)
                    self._search_deadline_ns = (
                        self.get_clock().now().nanoseconds + int(timeout * 1e9))
                    self._publish_status("SEARCHING_TOMATO")
            self._status_initialized = True
            return

        for entry in msg.status_list:
            goal_id = self._uuid(entry)
            if goal_id not in self._known_goals:
                self._known_goals.add(goal_id)
                if entry.status in (GoalStatus.STATUS_ACCEPTED,
                                    GoalStatus.STATUS_EXECUTING):
                    self._active_goal = goal_id
                    self._basket_sent = False
                    self._search_deadline_ns = 0
                    self._publish_enable(False)
                    self._publish_status(
                        "NAVIGATING" if self._mobility_ready
                        else "ERROR_NAV_STARTED_ARM_NOT_HOME")

            if goal_id != self._active_goal:
                continue
            if entry.status == GoalStatus.STATUS_SUCCEEDED:
                self._active_goal = None
                self._publish_enable(True)
                timeout = float(self.get_parameter("search_timeout_sec").value)
                self._search_deadline_ns = (
                    self.get_clock().now().nanoseconds + int(timeout * 1e9))
                self._publish_status("SEARCHING_TOMATO")
            elif entry.status in (GoalStatus.STATUS_CANCELED,
                                  GoalStatus.STATUS_ABORTED):
                self._active_goal = None
                self._publish_enable(False)
                self._publish_status("NAV_FAILED_OR_CANCELED")

    def _manipulator_state_callback(self, msg: String) -> None:
        state = msg.data.strip()
        if state in {"PREGRASP", "GRASP", "GRIPPER_CLOSING", "CUTTING"}:
            self._search_deadline_ns = 0
            self._publish_status(f"HARVEST_{state}")
        elif state == "WAIT_BASKET" and not self._basket_sent:
            self._publish_mock_basket()
            self._basket_sent = True
            self._publish_status("MOCK_BASKET_SENT")
        elif state == "HOME_READY":
            self._publish_enable(False)
            self._publish_status("CYCLE_COMPLETE_HOME_READY")
        elif state.startswith(("ERROR_", "ABORT_")):
            self._publish_enable(False)
            self._publish_status(state)

    def _mobility_callback(self, msg: Bool) -> None:
        self._mobility_ready = bool(msg.data)

    def _publish_mock_basket(self) -> None:
        xyz = list(self.get_parameter("mock_basket_release_xyz").value)
        if len(xyz) != 3:
            self._publish_status("ERROR_BAD_MOCK_BASKET_POSE")
            return
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = str(self.get_parameter("basket_frame").value)
        pose.pose.position.x = float(xyz[0])
        pose.pose.position.y = float(xyz[1])
        pose.pose.position.z = float(xyz[2])
        pose.pose.orientation.w = 1.0
        self._basket_pub.publish(pose)

    def _watchdog(self) -> None:
        if (self._resume_deadline_ns
                and self.get_clock().now().nanoseconds >= self._resume_deadline_ns):
            self._resume_deadline_ns = 0
            if self._state == "READY_FOR_NAV_GOAL" and self._active_goal is None:
                self._publish_enable(True)
                timeout = float(self.get_parameter("search_timeout_sec").value)
                self._search_deadline_ns = (
                    self.get_clock().now().nanoseconds + int(timeout * 1e9))
                self._publish_status("SEARCHING_TOMATO")
        if (self._search_deadline_ns
                and self.get_clock().now().nanoseconds > self._search_deadline_ns):
            self._search_deadline_ns = 0
            self._publish_enable(False)
            self._publish_status("ERROR_TOMATO_SEARCH_TIMEOUT")

    def _publish_enable(self, enabled: bool) -> None:
        self._enable_pub.publish(Bool(data=enabled))

    def _publish_status(self, state: str) -> None:
        self._state = state
        self._status_pub.publish(String(data=json.dumps({"state": state})))
        self.get_logger().info(f"단독 수확 시험: {state}")


def main():
    rclpy.init()
    node = NavHarvestTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
