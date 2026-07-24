"""IW 없이 Nav2 도착→근거리 탐색→수확→모의 바스켓 배치를 시험한다."""

from __future__ import annotations

import json
import math

import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformException, TransformListener


class NavHarvestTestNode(Node):
    """RViz에서 보낸 NavigateToPose 결과를 관찰하는 단독 통합시험 코디네이터."""

    def __init__(
        self,
        node_name: str = "nav_harvest_test_node",
        fixed_goal_defaults: dict[str, float | bool] | None = None,
    ):
        super().__init__(node_name)
        fixed = fixed_goal_defaults or {}
        self.declare_parameter(
            "nav_status_topic", "/navigate_to_pose/_action/status")
        self.declare_parameter("harvest_enable_topic", "harvest_test/enable")
        self.declare_parameter(
            "manipulator_state_topic", "/harvester_0/manipulator/target_state")
        self.declare_parameter(
            "mobility_ready_topic", "/harvester_0/manipulator/mobility_ready")
        self.declare_parameter("basket_pose_topic", "/iw/basket/empty_slot_pose")
        self.declare_parameter("basket_frame", "harvester_0/base_link")
        # IW가 붙기 전 시험용 tool0 release pose. 실제 바스켓 중심 좌표가 아니다.
        self.declare_parameter("mock_basket_release_xyz", [1.32, -0.20, 0.45])
        # 실제 IW 슬롯 선택기가 /iw/basket/empty_slot_pose를 발행해야만 놓는다.
        # 이 시험용 가짜 좌표는 명시적으로 켠 경우에만 사용한다.
        self.declare_parameter("use_mock_basket", False)
        self.declare_parameter("search_timeout_sec", 30.0)
        self.declare_parameter(
            "accept_initial_succeeded_goal",
            bool(fixed.get("accept_initial_succeeded_goal", True)))
        self.declare_parameter(
            "resume_search_after_start_sec",
            float(fixed.get("resume_search_after_start_sec", 2.0)))
        self.declare_parameter("home_after_nav", True)
        self.declare_parameter("post_nav_settle_sec", 0.5)
        self.declare_parameter("home_settle_timeout_sec", 30.0)
        self.declare_parameter("home_command_retry_sec", 1.0)
        self.declare_parameter("rmpflow_status_topic", "/harvester_0/rmpflow/status")
        self.declare_parameter("isaac_command_topic", "/harvester_0/cmd")
        self.declare_parameter(
            "reposition_request_topic", "/harvester_0/nav/reposition_request")
        self.declare_parameter("navigate_to_pose_action", "/navigate_to_pose")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("map_topic", "map")
        # 기존 RMP 전용 실행은 이 명령을 소비하지 못하므로 MoveIt 통합 launch에서 켠다.
        self.declare_parameter("orient_arm_to_nearest_bed", False)
        self.declare_parameter("map_occupied_threshold", 65)
        self.declare_parameter("bed_search_min_lateral_m", 0.55)
        self.declare_parameter("bed_search_max_lateral_m", 3.0)
        self.declare_parameter("bed_search_half_length_m", 1.5)
        # m0617 장착 기준 수평 카메라 광축 방위 = joint_1 - 180deg.
        # 지도에서 추정한 베드 장축의 법선으로 카메라를 향하게 할 때 사용한다.
        self.declare_parameter(
            "bed_view_joint_1_to_camera_offset_deg", -180.0)
        # 지도/TF가 없을 때만 쓰는 기존 안전 fallback.
        self.declare_parameter("bed_view_left_joint_1_deg", 270.0)
        self.declare_parameter("bed_view_right_joint_1_deg", 90.0)
        self.declare_parameter("bed_view_fallback_side", "left")
        # "left"/"right"이면 거리 비교를 건너뛴다. 수확 시험 위치가 두 베드의
        # 중간에 가까울 때 센서 1cm 차이로 반대쪽을 보는 일을 막는다.
        self.declare_parameter("bed_view_forced_side", "")
        self.declare_parameter("reposition_max_translation_m", 0.60)
        self.declare_parameter("reposition_tf_timeout_sec", 0.5)
        # 전용 자동시험에서는 RViz 클릭 없이 기억해 둔 수확 대기 위치로 먼저 이동한다.
        # 기본 NavHarvestTestNode는 기존 동작을 보존하기 위해 auto_nav_goal=false다.
        self.declare_parameter(
            "auto_nav_goal", bool(fixed.get("auto_nav_goal", False)))
        self.declare_parameter(
            "fixed_goal_x", float(fixed.get("fixed_goal_x", -0.54)))
        self.declare_parameter(
            "fixed_goal_y", float(fixed.get("fixed_goal_y", -8.19)))
        self.declare_parameter(
            "fixed_goal_yaw", float(fixed.get("fixed_goal_yaw", 1.91)))
        self.declare_parameter("fixed_goal_send_delay_sec", 2.0)
        self.declare_parameter("fixed_goal_retry_sec", 2.0)

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
        # IW 연동: mission_nav_node가 FOLLOW/FORKLIFT를 IW 전용 Nav2 goal로 변환한다.
        # 만재(N=1) 도킹 완료 보고를 받으면 지게차 하역을 트리거한다.
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
        self.create_subscription(
            OccupancyGrid,
            str(self.get_parameter("map_topic").value),
            self._map_callback,
            latched,
        )
        self.create_timer(0.2, self._watchdog)

        self._known_goals: set[bytes] = set()
        self._active_goal: bytes | None = None
        self._status_initialized = False
        self._mobility_ready = True
        self._search_deadline_ns = 0
        self._basket_sent = False
        self._cycle_failed = False
        self._waiting_home = False
        self._waiting_bed_view = False
        self._home_then_bed_view = False
        self._map: OccupancyGrid | None = None
        self._bed_view_side = ""
        self._bed_view_joint_1 = 0.0
        self._bed_long_axis_heading = 0.0
        self._home_request_id = 900000
        self._home_deadline_ns = 0
        self._last_home_command_ns = 0
        self._post_nav_settle_deadline_ns = 0
        self._reposition_goal_pending = False
        self._fixed_goal_sent = False
        self._fixed_goal_pending = False
        fixed_delay = float(
            self.get_parameter("fixed_goal_send_delay_sec").value)
        self._fixed_goal_deadline_ns = (
            self.get_clock().now().nanoseconds + int(fixed_delay * 1e9))
        resume = float(self.get_parameter("resume_search_after_start_sec").value)
        self._resume_deadline_ns = (
            self.get_clock().now().nanoseconds + int(resume * 1e9)
            if resume > 0.0 else 0)
        self._state = ""
        self._publish_enable(False)
        self._publish_status("READY_FOR_NAV_GOAL")

    def _send_fixed_nav_goal(self) -> None:
        """기억한 map 좌표를 한 번만 전송하고, 도착 뒤 기존 MoveIt 수확 FSM에 넘긴다."""
        if self._fixed_goal_sent or self._fixed_goal_pending:
            return
        if not self._nav_client.server_is_ready():
            return
        x = float(self.get_parameter("fixed_goal_x").value)
        y = float(self.get_parameter("fixed_goal_y").value)
        yaw = float(self.get_parameter("fixed_goal_yaw").value)
        if not all(math.isfinite(value) for value in (x, y, yaw)):
            self._publish_status("ERROR_BAD_FIXED_NAV_GOAL")
            self._fixed_goal_sent = True
            return

        goal = NavigateToPose.Goal()
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.header.frame_id = str(self.get_parameter("map_frame").value)
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = math.sin(yaw * 0.5)
        goal.pose.pose.orientation.w = math.cos(yaw * 0.5)
        self._fixed_goal_pending = True
        future = self._nav_client.send_goal_async(goal)
        future.add_done_callback(self._fixed_goal_response)
        self._publish_status("FIXED_NAV_GOAL_SENDING")
        self.get_logger().info(
            f"고정 수확 위치 Nav2 목표 전송: map=({x:.3f}, {y:.3f}), "
            f"yaw={yaw:.3f}rad")

    def _fixed_goal_response(self, future) -> None:
        self._fixed_goal_pending = False
        try:
            handle = future.result()
        except Exception as exc:
            self._schedule_fixed_goal_retry(
                f"고정 수확 위치 목표 전송 실패({exc})")
            return
        if handle is None or not handle.accepted:
            # Action 서버가 발견됐더라도 bt_navigator lifecycle 전환이 끝나기
            # 전에는 목표를 거절할 수 있다. 시작 순서 경쟁을 오류로 확정하지 않고
            # Nav2 활성화가 끝날 때까지 같은 목표를 다시 보낸다.
            self._schedule_fixed_goal_retry(
                "Nav2가 아직 고정 수확 위치 목표를 수락하지 않음")
            return
        self._fixed_goal_sent = True
        goal_id = bytes(handle.goal_id.uuid)
        self._known_goals.add(goal_id)
        self._active_goal = goal_id
        self._basket_sent = False
        self._search_deadline_ns = 0
        self._post_nav_settle_deadline_ns = 0
        self._publish_enable(False)
        self._publish_status("NAVIGATING_TO_FIXED_HARVEST_POSE")

    def _schedule_fixed_goal_retry(self, reason: str) -> None:
        retry = max(
            0.2, float(self.get_parameter("fixed_goal_retry_sec").value))
        self._fixed_goal_sent = False
        self._fixed_goal_pending = False
        self._fixed_goal_deadline_ns = (
            self.get_clock().now().nanoseconds + int(retry * 1e9))
        self._publish_status("WAITING_FOR_NAV2_READY")
        self.get_logger().warning(f"{reason}; {retry:.1f}초 뒤 재시도")

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
            elif (bool(self.get_parameter(
                    "accept_initial_succeeded_goal").value)
                    and not bool(self.get_parameter("auto_nav_goal").value)):
                succeeded = [entry for entry in msg.status_list
                             if entry.status == GoalStatus.STATUS_SUCCEEDED]
                if succeeded:
                    # 통합시험 launch를 목표 도착 후 켠 경우 최신 완료 목표를 현재
                    # 정지 위치의 도착으로 받아 수확 단계부터 이어서 시험한다.
                    # auto_nav_goal에서는 과거 성공을 받으면 지도 준비 전 270° 베드뷰를
                    # 먼저 찍고 실제 목표 도착 뒤 다시 계산한 각도로 되돌아가므로 금지한다.
                    self._schedule_post_nav_manipulation()
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
                self._waiting_bed_view = False
                self._post_nav_settle_deadline_ns = 0
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
                self._schedule_post_nav_manipulation()
            elif entry.status in (GoalStatus.STATUS_CANCELED,
                                  GoalStatus.STATUS_ABORTED):
                self._active_goal = None
                self._publish_enable(False)
                self._publish_status("NAV_FAILED_OR_CANCELED")

    def _start_search(self) -> None:
        self._waiting_home = False
        self._waiting_bed_view = False
        self._home_then_bed_view = False
        self._home_deadline_ns = 0
        self._last_home_command_ns = 0
        self._publish_enable(True)
        timeout = float(self.get_parameter("search_timeout_sec").value)
        self._search_deadline_ns = (
            self.get_clock().now().nanoseconds + int(timeout * 1e9))
        self._publish_status("SEARCHING_TOMATO")

    def _schedule_post_nav_manipulation(self) -> None:
        """Nav2 성공 후 잔류 cmd_vel이 끊길 시간을 둔 다음 팔 동작을 시작한다."""
        self._publish_enable(False)
        self._waiting_home = False
        self._waiting_bed_view = False
        self._search_deadline_ns = 0
        delay = max(
            0.0, float(self.get_parameter("post_nav_settle_sec").value))
        if delay == 0.0:
            self._begin_post_nav_home()
            return
        self._post_nav_settle_deadline_ns = (
            self.get_clock().now().nanoseconds + int(delay * 1e9))
        self._publish_status("WAIT_BASE_SETTLED_AFTER_NAV")

    def _begin_post_nav_home(self) -> None:
        """Nav 성공 뒤 가까운 베드를 향한 관절 자세에 도달한 후에만 수확한다."""
        self._post_nav_settle_deadline_ns = 0
        self._publish_enable(False)
        self._search_deadline_ns = 0
        if not bool(self.get_parameter("home_after_nav").value):
            self._start_search()
            return
        self._waiting_home = True
        self._home_then_bed_view = bool(
            self.get_parameter("orient_arm_to_nearest_bed").value)
        # 어떤 자세에서 들어왔든 HOME을 먼저 확정하고, 그 다음 BED_VIEW로 간다.
        # HOME에서 곧바로 과실 접근을 시작하면 1축이 베드 반대편 등가 IK로 크게 돈다.
        self._waiting_bed_view = False
        self._home_request_id += 1
        timeout = float(self.get_parameter("home_settle_timeout_sec").value)
        self._home_deadline_ns = (
            self.get_clock().now().nanoseconds + int(timeout * 1e9))
        if self._home_then_bed_view:
            forced_side = str(
                self.get_parameter("bed_view_forced_side").value
            ).strip().lower()
            # 거리 판정 결과를 쓰지 않더라도 지도에서 베드 장축은 먼저 계산한다.
            # 이를 생략하면 장축 기본값 0°로 270° fallback을 보게 된다.
            nearest_side = self._nearest_bed_side()
            self._bed_view_side = (
                forced_side
                if forced_side in {"left", "right"}
                else nearest_side
            )
            normal_heading = self._bed_normal_heading(
                self._bed_long_axis_heading, self._bed_view_side)
            camera_offset = math.radians(float(self.get_parameter(
                "bed_view_joint_1_to_camera_offset_deg").value))
            # camera_heading = joint_1 + offset이므로 역산한다. 같은 방향을 만드는
            # ±2π 해 중 HOME joint_1=π에서 가장 가까운 해를 골라 불필요한 회전을 막는다.
            self._bed_view_joint_1 = self._nearest_equivalent_angle(
                normal_heading - camera_offset, math.pi)
            self.get_logger().info(
                f"Nav2 도착 확정: 가까운 베드={self._bed_view_side}, "
                f"bed_axis={math.degrees(self._bed_long_axis_heading):.1f}deg, "
                f"camera_heading={math.degrees(normal_heading):.1f}deg, "
                f"joint_1={math.degrees(self._bed_view_joint_1):.1f}deg")
        self._publish_home_command()
        self._publish_status(
            "WAIT_ARM_HOME_BEFORE_BED_VIEW"
            if self._home_then_bed_view else "WAIT_ARM_HOME_AFTER_NAV")

    def _publish_home_command(self) -> None:
        self._last_home_command_ns = self.get_clock().now().nanoseconds
        if self._waiting_bed_view:
            command = {
                "moveit_bed_view": {
                    "id": self._home_request_id,
                    "side": self._bed_view_side,
                    "joint_1": self._bed_view_joint_1,
                },
            }
        else:
            command = {"rmp_home": {"id": self._home_request_id}}
        self._isaac_command_pub.publish(String(data=json.dumps(command)))

    def _rmpflow_status_callback(self, msg: String) -> None:
        if not self._waiting_home:
            return
        try:
            status = json.loads(msg.data)
            status_id = int(status.get("id", -1))
        except (TypeError, ValueError):
            return
        if status_id != self._home_request_id:
            return
        phase = str(status.get("phase", ""))
        if (self._waiting_bed_view
                and phase == "BED_VIEW"
                and bool(status.get("reached", False))):
            self._start_search()
            return
        # RMPflow는 HOME + at_home + distance를 보내지만 MoveIt 브리지는 관절
        # constraint 실행 성공을 GO_HOME + reached로 확정한다.
        if phase == "GO_HOME" and bool(status.get("reached", False)):
            if self._home_then_bed_view and not self._waiting_bed_view:
                self._home_request_id += 1
                self._waiting_bed_view = True
                timeout = float(
                    self.get_parameter("home_settle_timeout_sec").value)
                self._home_deadline_ns = (
                    self.get_clock().now().nanoseconds + int(timeout * 1e9))
                self._publish_home_command()
                self._publish_status("WAIT_ARM_BED_VIEW_AFTER_HOME")
                return
            self._start_search()
            return
        try:
            distance = float(status.get("distance", 999.0))
        except (TypeError, ValueError):
            return
        if (phase == "HOME"
                and bool(status.get("at_home", False))
                and bool(status.get("reached", False))
                and distance <= 0.03):
            if self._home_then_bed_view and not self._waiting_bed_view:
                self._home_request_id += 1
                self._waiting_bed_view = True
                timeout = float(
                    self.get_parameter("home_settle_timeout_sec").value)
                self._home_deadline_ns = (
                    self.get_clock().now().nanoseconds + int(timeout * 1e9))
                self._publish_home_command()
                self._publish_status("WAIT_ARM_BED_VIEW_AFTER_HOME")
                return
            self._start_search()

    def _manipulator_state_callback(self, msg: String) -> None:
        state = msg.data.strip()
        if state in {
            "APPROACH", "PREGRASP", "GRASP", "CAPTURE_TRIM",
            "GRIPPER_CLOSING", "GRASP_VERIFY",
            "CUTTING", "CUT_VERIFY", "BLADE_OPENING",
            "VERIFY_RETRACT", "GRASP_FOLLOW_CHECK",
            "RETRACT_CIRC", "RETRACT_LIN",
        }:
            if state == "APPROACH":
                self._cycle_failed = False
            self._search_deadline_ns = 0
            self._publish_status(f"HARVEST_{state}")
        elif state == "HARVEST_FAILED":
            self._cycle_failed = True
            self._publish_status("HARVEST_FAILED_RETURNING_HOME")
        elif state == "WAIT_BASKET" and not self._basket_sent:
            if bool(self.get_parameter("use_mock_basket").value):
                self._publish_mock_basket()
                self._basket_sent = True
                self._publish_status("MOCK_BASKET_SENT")
        elif state in {"BASKET_APPROACH", "BASKET_PLACE", "PLACE_RELEASING"}:
            self._placed = True                   # iw 데크에 놓기 진행 중
            self._publish_status(f"HARVEST_{state}")
        elif state == "HOME_READY":
            self._publish_enable(False)
            self._publish_status(
                "CYCLE_FAILED_HOME_READY"
                if self._cycle_failed else "CYCLE_COMPLETE_HOME_READY")
            # 1차·단순(N=1): 토마토 1개를 iw 데크에 놓으면 만재 → iw 를 지게차로.
            if not self._cycle_failed and self._placed and not self._iw_full:
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
        self._waiting_bed_view = False
        self._post_nav_settle_deadline_ns = 0
        self._home_deadline_ns = 0
        self._last_home_command_ns = 0
        self._publish_status("NAV_REPOSITIONING")

    def _map_callback(self, msg: OccupancyGrid) -> None:
        self._map = msg

    def _nearest_bed_side(self) -> str:
        """가까운 베드 측면과 그 베드의 수평 장축을 지도에서 추정한다."""
        fallback = str(
            self.get_parameter("bed_view_fallback_side").value).lower()
        if fallback not in {"left", "right"}:
            fallback = "left"
        grid = self._map
        if grid is None or grid.info.width == 0 or grid.info.height == 0:
            self._bed_long_axis_heading = 0.0
            self.get_logger().warning(
                f"지도 미수신: 베드 방향 기본값({fallback}) 사용")
            return fallback

        map_frame = str(self.get_parameter("map_frame").value)
        base_frame = str(self.get_parameter("base_frame").value)
        try:
            transform = self._buffer.lookup_transform(
                map_frame, base_frame, rclpy.time.Time(),
                timeout=Duration(seconds=float(self.get_parameter(
                    "reposition_tf_timeout_sec").value)))
        except TransformException as exc:
            self._bed_long_axis_heading = 0.0
            self.get_logger().warning(
                f"베드 방향 TF 실패({exc}): 기본값({fallback}) 사용")
            return fallback

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        base_yaw = math.atan2(
            2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
            1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z))
        left, right = self._nearest_occupied_lateral_distances(
            grid, translation.x, translation.y, base_yaw,
            int(self.get_parameter("map_occupied_threshold").value),
            float(self.get_parameter("bed_search_min_lateral_m").value),
            float(self.get_parameter("bed_search_max_lateral_m").value),
            float(self.get_parameter("bed_search_half_length_m").value),
        )
        if not math.isfinite(left) and not math.isfinite(right):
            self._bed_long_axis_heading = 0.0
            self.get_logger().warning(
                f"주변 점유 셀 없음: 베드 방향 기본값({fallback}) 사용")
            return fallback
        side = "left" if left <= right else "right"
        self._bed_long_axis_heading = self._estimate_bed_long_axis(
            grid, translation.x, translation.y, base_yaw, side,
            int(self.get_parameter("map_occupied_threshold").value),
            float(self.get_parameter("bed_search_min_lateral_m").value),
            float(self.get_parameter("bed_search_max_lateral_m").value),
            float(self.get_parameter("bed_search_half_length_m").value),
        )
        self.get_logger().info(
            "지도 기반 베드 거리: "
            f"left={left:.2f}m, right={right:.2f}m -> {side}, "
            f"장축={math.degrees(self._bed_long_axis_heading):.1f}deg")
        return side

    @staticmethod
    def _bed_normal_heading(long_axis: float, side: str) -> float:
        """베드 장축에 수직인 두 법선 중 로봇에서 해당 베드를 향하는 쪽을 고른다."""
        first = long_axis + math.pi * 0.5
        second = long_axis - math.pi * 0.5
        desired_lateral_sign = 1.0 if side == "left" else -1.0
        return (
            first if desired_lateral_sign * math.sin(first)
            >= desired_lateral_sign * math.sin(second) else second
        )

    @staticmethod
    def _nearest_equivalent_angle(
        target: float, reference: float,
        lower: float = -2.0 * math.pi, upper: float = 2.0 * math.pi,
    ) -> float:
        """같은 회전 자세(target+2kπ) 중 기준 관절값에서 가장 가까운 유효 해."""
        candidates = [
            target + 2.0 * math.pi * k for k in range(-3, 4)
            if lower <= target + 2.0 * math.pi * k <= upper
        ]
        return min(candidates, key=lambda value: abs(value - reference))

    @staticmethod
    def _estimate_bed_long_axis(
        grid: OccupancyGrid,
        base_x: float,
        base_y: float,
        base_yaw: float,
        side: str,
        occupied_threshold: int,
        min_lateral: float,
        max_lateral: float,
        half_length: float,
    ) -> float:
        """선택한 쪽 점유 셀의 2D PCA로 베드 장축 방위를 base frame에서 구한다."""
        origin = grid.info.origin
        oq = origin.orientation
        origin_yaw = math.atan2(
            2.0 * (oq.w * oq.z + oq.x * oq.y),
            1.0 - 2.0 * (oq.y * oq.y + oq.z * oq.z))
        co, so = math.cos(origin_yaw), math.sin(origin_yaw)
        cb, sb = math.cos(base_yaw), math.sin(base_yaw)
        resolution = float(grid.info.resolution)
        width = int(grid.info.width)
        samples: list[tuple[float, float]] = []
        for index, occupancy in enumerate(grid.data):
            if occupancy < occupied_threshold:
                continue
            row, column = divmod(index, width)
            local_x = (column + 0.5) * resolution
            local_y = (row + 0.5) * resolution
            map_x = origin.position.x + co * local_x - so * local_y
            map_y = origin.position.y + so * local_x + co * local_y
            dx, dy = map_x - base_x, map_y - base_y
            forward = cb * dx + sb * dy
            lateral = -sb * dx + cb * dy
            if (abs(forward) > half_length
                    or abs(lateral) < min_lateral
                    or abs(lateral) > max_lateral
                    or (side == "left" and lateral <= 0.0)
                    or (side == "right" and lateral >= 0.0)):
                continue
            samples.append((forward, lateral))
        if len(samples) < 3:
            return 0.0
        mean_x = sum(point[0] for point in samples) / len(samples)
        mean_y = sum(point[1] for point in samples) / len(samples)
        cxx = sum((x - mean_x) ** 2 for x, _ in samples)
        cyy = sum((y - mean_y) ** 2 for _, y in samples)
        cxy = sum(
            (x - mean_x) * (y - mean_y) for x, y in samples)
        # 최대 고유값의 고유벡터 방위. 장축은 π 주기이므로 부호는 중요하지 않다.
        return 0.5 * math.atan2(2.0 * cxy, cxx - cyy)

    @staticmethod
    def _nearest_occupied_lateral_distances(
        grid: OccupancyGrid,
        base_x: float,
        base_y: float,
        base_yaw: float,
        occupied_threshold: int,
        min_lateral: float,
        max_lateral: float,
        half_length: float,
    ) -> tuple[float, float]:
        """OccupancyGrid 셀을 베이스 좌표로 바꿔 좌·우 최단 횡거리를 반환한다."""
        origin = grid.info.origin
        oq = origin.orientation
        origin_yaw = math.atan2(
            2.0 * (oq.w * oq.z + oq.x * oq.y),
            1.0 - 2.0 * (oq.y * oq.y + oq.z * oq.z))
        co, so = math.cos(origin_yaw), math.sin(origin_yaw)
        cb, sb = math.cos(base_yaw), math.sin(base_yaw)
        resolution = float(grid.info.resolution)
        width = int(grid.info.width)
        left = math.inf
        right = math.inf
        for index, occupancy in enumerate(grid.data):
            if occupancy < occupied_threshold:
                continue
            row, column = divmod(index, width)
            local_x = (column + 0.5) * resolution
            local_y = (row + 0.5) * resolution
            map_x = origin.position.x + co * local_x - so * local_y
            map_y = origin.position.y + so * local_x + co * local_y
            dx, dy = map_x - base_x, map_y - base_y
            forward = cb * dx + sb * dy
            lateral = -sb * dx + cb * dy
            absolute_lateral = abs(lateral)
            if (abs(forward) > half_length
                    or absolute_lateral < min_lateral
                    or absolute_lateral > max_lateral):
                continue
            if lateral > 0.0:
                left = min(left, absolute_lateral)
            else:
                right = min(right, absolute_lateral)
        return left, right

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
        if (self._post_nav_settle_deadline_ns
                and now >= self._post_nav_settle_deadline_ns):
            self._post_nav_settle_deadline_ns = 0
            self._begin_post_nav_home()
        if (bool(self.get_parameter("auto_nav_goal").value)
                and not self._fixed_goal_sent
                and not self._fixed_goal_pending
                and self._active_goal is None
                and now >= self._fixed_goal_deadline_ns):
            self._send_fixed_nav_goal()
        # 실제 운용에서는 IW 슬롯 선택기의 좌표만 쓴다. 모의 좌표 자동 주입은
        # use_mock_basket=true인 독립 시험에서만 허용한다.
        if (bool(self.get_parameter("use_mock_basket").value)
                and not self._iw_full):
            self._publish_mock_basket()
        if self._waiting_home:
            retry = float(self.get_parameter("home_command_retry_sec").value)
            if (not self._last_home_command_ns
                    or now - self._last_home_command_ns >= int(retry * 1e9)):
                self._publish_home_command()
        if (self._resume_deadline_ns
                and now >= self._resume_deadline_ns):
            self._resume_deadline_ns = 0
            if (self._state == "READY_FOR_NAV_GOAL"
                    and self._active_goal is None
                    and not bool(self.get_parameter("auto_nav_goal").value)):
                self._begin_post_nav_home()
        if (self._home_deadline_ns
                and now > self._home_deadline_ns):
            self._waiting_home = False
            self._waiting_bed_view = False
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
