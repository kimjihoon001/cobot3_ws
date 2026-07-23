"""IW 없이 Nav2 도착→근거리 탐색→수확→모의 바스켓 배치를 시험한다."""

from __future__ import annotations

import json
import math

import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformException, TransformListener


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
        self.declare_parameter("home_after_nav", True)
        self.declare_parameter("home_settle_timeout_sec", 30.0)
        self.declare_parameter("home_command_retry_sec", 1.0)
        self.declare_parameter("rmpflow_status_topic", "/harvester_0/rmpflow/status")
        self.declare_parameter("isaac_command_topic", "/harvester_0/cmd")
        self.declare_parameter(
            "reposition_request_topic", "/harvester_0/nav/reposition_request")
        self.declare_parameter("navigate_to_pose_action", "/navigate_to_pose")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("reposition_max_translation_m", 0.60)
        self.declare_parameter("reposition_tf_timeout_sec", 0.5)

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
        self._isaac_command_pub = self.create_publisher(
            String, str(self.get_parameter("isaac_command_topic").value), 10)
        # IW 연동(2026-07-23, 1차·단순): iw 는 Isaac 실좌표로 MM 뒤를 텔레포트 추종하고,
        # 만재(N=1) 시 지게차로 보낸다. iw 도착 보고를 받으면 지게차 하역을 트리거한다.
        self._iw_mission_pub = self.create_publisher(String, "/iw/mission", latched)
        self._forklift_dock_pub = self.create_publisher(
            Bool, "/forklift/amr_docked", 10)
        self.create_subscription(String, "/iw/status",
                                 self._iw_status_callback, latched)
        self._iw_full = False
        self._placed = False
        self._iw_mission_pub.publish(String(data="FOLLOW"))   # 시작=추종
        self._buffer = Buffer()
        self._listener = TransformListener(self._buffer, self)
        self._nav_client = ActionClient(
            self, NavigateToPose,
            str(self.get_parameter("navigate_to_pose_action").value))
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
        self.create_subscription(
            String,
            str(self.get_parameter("rmpflow_status_topic").value),
            self._rmpflow_status_callback,
            20,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("reposition_request_topic").value),
            self._reposition_callback,
            latched,
        )
        self.create_timer(0.2, self._watchdog)

        self._known_goals: set[bytes] = set()
        self._active_goal: bytes | None = None
        self._status_initialized = False
        self._mobility_ready = True
        self._search_deadline_ns = 0
        self._basket_sent = False
        self._waiting_home = False
        self._home_request_id = 900000
        self._home_deadline_ns = 0
        self._last_home_command_ns = 0
        self._reposition_goal_pending = False
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
                    self._begin_post_nav_home()
            self._status_initialized = True
            return

        for entry in msg.status_list:
            goal_id = self._uuid(entry)
            # ActionClient 응답에서 이미 알려진 goal ID도 포함해 새 주행 시작 시
            # 이전 HOME 대기와 타이머를 무조건 취소한다.
            if (entry.status in (GoalStatus.STATUS_ACCEPTED,
                                 GoalStatus.STATUS_EXECUTING)
                    and goal_id != self._active_goal):
                self._active_goal = goal_id
                self._basket_sent = False
                self._search_deadline_ns = 0
                self._waiting_home = False
                self._home_deadline_ns = 0
                self._last_home_command_ns = 0
                self._publish_enable(False)
                self._publish_status(
                    "NAVIGATING" if self._mobility_ready
                    else "ERROR_NAV_STARTED_ARM_NOT_HOME")
            self._known_goals.add(goal_id)

            if goal_id != self._active_goal:
                continue
            if entry.status == GoalStatus.STATUS_SUCCEEDED:
                self._active_goal = None
                self._begin_post_nav_home()
            elif entry.status in (GoalStatus.STATUS_CANCELED,
                                  GoalStatus.STATUS_ABORTED):
                self._active_goal = None
                self._publish_enable(False)
                self._publish_status("NAV_FAILED_OR_CANCELED")

    def _start_search(self) -> None:
        self._waiting_home = False
        self._home_deadline_ns = 0
        self._last_home_command_ns = 0
        self._publish_enable(True)
        timeout = float(self.get_parameter("search_timeout_sec").value)
        self._search_deadline_ns = (
            self.get_clock().now().nanoseconds + int(timeout * 1e9))
        self._publish_status("SEARCHING_TOMATO")

    def _begin_post_nav_home(self) -> None:
        """Nav 도착 뒤 팔을 실제 홈 오차 0.03rad 안으로 복귀시킨 후에만 수확한다."""
        self._publish_enable(False)
        self._search_deadline_ns = 0
        if not bool(self.get_parameter("home_after_nav").value):
            self._start_search()
            return
        self._waiting_home = True
        self._home_request_id += 1
        timeout = float(self.get_parameter("home_settle_timeout_sec").value)
        self._home_deadline_ns = (
            self.get_clock().now().nanoseconds + int(timeout * 1e9))
        self._publish_home_command()
        self._publish_status("WAIT_ARM_HOME_AFTER_NAV")

    def _publish_home_command(self) -> None:
        self._last_home_command_ns = self.get_clock().now().nanoseconds
        self._isaac_command_pub.publish(String(data=json.dumps({
            "rmp_home": {"id": self._home_request_id},
        })))

    def _rmpflow_status_callback(self, msg: String) -> None:
        if not self._waiting_home:
            return
        try:
            status = json.loads(msg.data)
            status_id = int(status.get("id", -1))
            distance = float(status.get("distance", 999.0))
        except (TypeError, ValueError):
            return
        if status_id != self._home_request_id or status.get("phase") != "HOME":
            return
        if (bool(status.get("at_home", False))
                and bool(status.get("reached", False))
                and distance <= 0.03):
            self._start_search()

    def _manipulator_state_callback(self, msg: String) -> None:
        state = msg.data.strip()
        if state in {
            "PREGRASP", "GRASP", "GRIPPER_CLOSING", "GRASP_VERIFY",
            "VERIFY_RETRACT", "GRASP_FOLLOW_CHECK",
        }:
            self._search_deadline_ns = 0
            self._publish_status(f"HARVEST_{state}")
        elif state == "WAIT_BASKET" and not self._basket_sent:
            self._publish_mock_basket()
            self._basket_sent = True
            self._publish_status("MOCK_BASKET_SENT")
        elif state in {"BASKET_APPROACH", "BASKET_PLACE", "PLACE_RELEASING"}:
            self._placed = True                   # iw 데크에 놓기 진행 중
            self._publish_status(f"HARVEST_{state}")
        elif state == "HOME_READY":
            self._publish_enable(False)
            self._publish_status("CYCLE_COMPLETE_HOME_READY")
            # 1차·단순(N=1): 토마토 1개를 iw 데크에 놓으면 만재 → iw 를 지게차로.
            if self._placed and not self._iw_full:
                self._iw_full = True
                self._iw_mission_pub.publish(String(data="FORKLIFT"))
                self._publish_status("IW_FULL_TO_FORKLIFT")
                self.get_logger().info("적재 1개(만재) → iw 지게차 이동 지시")
        elif state == "RETRY_VISION":
            # 실패 후 홈에 도달한 경우에는 원샷 게이트를 끄지 않는다. 홈 카메라의
            # 새로운 YOLO 프레임을 받도록 탐색 타이머와 수확 게이트를 다시 연다.
            self._basket_sent = False
            self._start_search()
            self._publish_status("RETRY_SEARCHING_TOMATO")
        elif state == "NAV_REPOSITION_REQUIRED":
            self._search_deadline_ns = 0
            self._publish_enable(False)
            self._publish_status("NAV_REPOSITION_REQUIRED")
        elif state.startswith(("ERROR_", "ABORT_")):
            self._publish_enable(False)
            self._publish_status(state)

    def _mobility_callback(self, msg: Bool) -> None:
        self._mobility_ready = bool(msg.data)

    def _reposition_callback(self, msg: String) -> None:
        """팔 밖 목표를 받으면 현재 base 자세에서 필요한 만큼 Nav2로 재정차한다."""
        if self._reposition_goal_pending or self._active_goal is not None:
            return
        try:
            request = json.loads(msg.data)
            forward = float(request["forward_m"])
            lateral = float(request.get("lateral_m", 0.0))
        except (KeyError, TypeError, ValueError, json.JSONDecodeError):
            self._publish_status("ERROR_BAD_REPOSITION_REQUEST")
            return
        if not (math.isfinite(forward) and math.isfinite(lateral)):
            self._publish_status("ERROR_BAD_REPOSITION_REQUEST")
            return
        distance = math.hypot(forward, lateral)
        maximum = float(
            self.get_parameter("reposition_max_translation_m").value)
        if distance < 0.02:
            self._publish_status("ERROR_REPOSITION_TOO_SMALL")
            return
        if distance > maximum:
            scale = maximum / distance
            forward *= scale
            lateral *= scale

        map_frame = str(self.get_parameter("map_frame").value)
        base_frame = str(self.get_parameter("base_frame").value)
        try:
            transform = self._buffer.lookup_transform(
                map_frame, base_frame, rclpy.time.Time(),
                timeout=Duration(seconds=float(self.get_parameter(
                    "reposition_tf_timeout_sec").value)))
        except TransformException as exc:
            self.get_logger().error(f"Nav 재접근 TF 실패: {exc}")
            self._publish_status("ERROR_REPOSITION_TF")
            return

        q = transform.transform.rotation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        goal = NavigateToPose.Goal()
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.header.frame_id = map_frame
        goal.pose.pose.position.x = (
            transform.transform.translation.x
            + math.cos(yaw) * forward - math.sin(yaw) * lateral)
        goal.pose.pose.position.y = (
            transform.transform.translation.y
            + math.sin(yaw) * forward + math.cos(yaw) * lateral)
        goal.pose.pose.position.z = transform.transform.translation.z
        goal.pose.pose.orientation = q

        if not self._nav_client.server_is_ready():
            self._publish_status("ERROR_NAV2_NOT_READY_FOR_REPOSITION")
            return
        self._reposition_goal_pending = True
        self._publish_enable(False)
        self._publish_status("NAV_REPOSITIONING")
        future = self._nav_client.send_goal_async(goal)
        future.add_done_callback(self._reposition_goal_response)
        self.get_logger().info(
            f"Nav2 재접근 목표 전송: forward={forward:.3f}m, "
            f"lateral={lateral:.3f}m -> map=("
            f"{goal.pose.pose.position.x:.3f}, {goal.pose.pose.position.y:.3f})")

    def _reposition_goal_response(self, future) -> None:
        self._reposition_goal_pending = False
        try:
            handle = future.result()
        except Exception as exc:  # rclpy future가 전송 오류를 예외로 전달한다.
            self.get_logger().error(f"Nav2 재접근 목표 전송 실패: {exc}")
            self._publish_status("ERROR_REPOSITION_SEND_FAILED")
            return
        if not handle.accepted:
            self._publish_status("ERROR_REPOSITION_REJECTED")
            return
        goal_id = bytes(handle.goal_id.uuid)
        self._known_goals.add(goal_id)
        self._active_goal = goal_id
        self._basket_sent = False
        self._search_deadline_ns = 0
        self._waiting_home = False
        self._home_deadline_ns = 0
        self._last_home_command_ns = 0
        self._publish_status("NAV_REPOSITIONING")

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

    def _iw_status_callback(self, msg: String) -> None:
        """iw 가 지게차에 도착하면 하역을 트리거한다(/forklift/amr_docked True)."""
        if msg.data.strip().upper() == "ARRIVED_FORKLIFT":
            self._forklift_dock_pub.publish(Bool(data=True))
            self._publish_status("IW_DOCKED_FORKLIFT_TRIGGERED")
            self.get_logger().info(
                "iw 지게차 도착 → /forklift/amr_docked True (하역 트리거)")

    def _watchdog(self) -> None:
        now = self.get_clock().now().nanoseconds
        # iw 데크(=놓을 위치) pose 를 계속 발행해 항상 최신으로 둔다. home_after_attempt=true
        # 라도 _basket_place 가 미리 세팅돼 파지 후 홈으로 안 가고 iw 에 놓는다. iw 가 만재로
        # 지게차로 떠난 뒤에는 발행을 멈춰(놓을 데 없음) 홈 복귀하게 한다.
        if not self._iw_full:
            self._publish_mock_basket()
        if self._waiting_home:
            retry = float(self.get_parameter("home_command_retry_sec").value)
            if (not self._last_home_command_ns
                    or now - self._last_home_command_ns >= int(retry * 1e9)):
                self._publish_home_command()
        if (self._resume_deadline_ns
                and now >= self._resume_deadline_ns):
            self._resume_deadline_ns = 0
            if self._state == "READY_FOR_NAV_GOAL" and self._active_goal is None:
                self._begin_post_nav_home()
        if (self._home_deadline_ns
                and now > self._home_deadline_ns):
            self._waiting_home = False
            self._home_deadline_ns = 0
            self._last_home_command_ns = 0
            self._publish_enable(False)
            self._publish_status("ERROR_ARM_HOME_TIMEOUT")
        if (self._search_deadline_ns
                and now > self._search_deadline_ns):
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
