"""IDLE/FOLLOW/FORKLIFT 미션을 IW 전용 레인 경로 goal로 변환한다."""
from __future__ import annotations

import math
import os
import time

import rclpy
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateThroughPoses
from nav2_msgs.srv import ManageLifecycleNodes
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from smartfarm_interfaces.srv import DockAdjust, ForkliftCycle
from std_msgs.msg import Bool, Int32, String
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformException, TransformListener

from iwhub_control import lanes


def _yaw_from_quaternion(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


class MissionNavNode(Node):
    """MM 추종 목표와 만재 도킹 목표를 IW Nav2에 전달한다."""

    def __init__(self):
        super().__init__("iw_mission_nav_node")
        self.declare_parameter(
            "navigate_through_poses_action",
            "/iwhub_0/navigate_through_poses")
        self.declare_parameter(
            "lifecycle_manager_service",
            "/iwhub_0/lifecycle_manager_navigation/manage_nodes",
        )
        self.declare_parameter("nav_startup_delay_sec", 7.0)
        self.declare_parameter("nav_startup_retry_sec", 5.0)
        self.declare_parameter("goal_frame", "iwhub_0/map")
        self.declare_parameter("mm_map_frame", "map")
        self.declare_parameter("mm_base_frame", "base_link")
        self.declare_parameter("iw_odom_topic", "/iwhub_0/odom")
        self.declare_parameter("iw_tf_topic", "/iwhub_0/tf")
        # 도킹 standoff: MM 중심에서 IW 접근 방향으로 이 거리에 비접촉 정차점을 둔다.
        # 하한(비접촉) ≈ MM반경 + IW앞0.40 + 여유0.10, 상한 ≈ 팔 도달반경(~1.35).
        # 플레이스 때 1번 링크가 지면과 수평에 가깝게 완전 신전되어, 그리퍼
        # 이송 중 링크가 파지 과실을 건드리는 현상을 줄인다. 최종 X가 레인 중심으로
        # 스냅되는 오차를 포함해 실제 중심 간격을 약 1.23m→1.08m로 줄이는 값이다
        # (2026-07-25 sim_diag_160553 기준).
        self.declare_parameter("dock_standoff", 1.03)
        self.declare_parameter("follow_update_distance", 0.30)
        self.declare_parameter("follow_update_yaw", math.radians(30.0))
        self.declare_parameter("dock_x", 0.0)
        self.declare_parameter("dock_y", 10.84885)
        self.declare_parameter("dock_yaw", math.pi / 2.0)
        self.declare_parameter("max_dock_adjust_retries", 3)
        self.declare_parameter("dock_align_capture_radius", 0.30)
        self.declare_parameter("dock_align_position_tolerance", 0.04)
        self.declare_parameter(
            "dock_align_yaw_tolerance", math.radians(2.0)
        )
        self.declare_parameter("dock_align_max_linear_speed", 0.08)
        self.declare_parameter("dock_align_max_angular_speed", 0.15)
        self.declare_parameter("dock_align_settle_sec", 1.0)
        self.declare_parameter("dock_align_timeout_sec", 45.0)

        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._status_pub = self.create_publisher(
            String, "/iw/status", latched)
        # 지게차 인계는 장시간 작업 시작을 명시적 서비스로 접수하고,
        # 완료/출발 허가는 기존 latched clear 토픽으로 받는다.
        self._forklift_client = self.create_client(
            ForkliftCycle, "/forklift/start_cycle")
        self._dock_adjust_service = self.create_service(
            DockAdjust,
            "/iw/request_dock_adjust",
            self._on_dock_adjust_request,
        )
        # 구형 노드와의 관찰/호환용 도킹 토픽은 유지하지만 정상 통합 경로에서는
        # 서비스만 호출해 중복 사이클이 시작되지 않게 한다.
        self._amr_docked_pub = self.create_publisher(
            Bool, "/forklift/amr_docked", latched)
        self._dock_cmd_pub = self.create_publisher(
            Twist, "/iwhub_0/cmd_vel", 10
        )
        # iw→MM 복귀 완료 신호 — MM(수확 FSM)이 이걸 받아 다음 수확을 재개한다.
        self._resume_pub = self.create_publisher(
            Bool, "/iw/resume_harvest", latched)
        self._yield_request_pub = self.create_publisher(
            Bool, "/iw/mm_yield_request", latched)
        self.create_subscription(
            Bool, "/forklift/clear", self._on_forklift_clear, 10)
        self.create_subscription(
            Bool, "/iw/mm_yield_complete",
            self._on_mm_yield_complete, latched)
        self.create_subscription(
            Int32, "/forklift/pallet_on_iw", self._on_pallet_on_iw, 10)
        self.create_subscription(
            String, "/iw/mission", self._on_mission, latched)
        self.create_subscription(
            Odometry,
            str(self.get_parameter("iw_odom_topic").value),
            self._on_iw_odom,
            10,
        )
        self.create_subscription(
            TFMessage,
            str(self.get_parameter("iw_tf_topic").value),
            self._on_iw_tf,
            100,
        )
        self._through_client = ActionClient(
            self, NavigateThroughPoses,
            str(self.get_parameter("navigate_through_poses_action").value),
        )
        self._lifecycle_client = self.create_client(
            ManageLifecycleNodes,
            str(self.get_parameter("lifecycle_manager_service").value),
        )
        self._buffer = Buffer()
        self._listener = TransformListener(self._buffer, self)
        # MM과 동시 출발하지 않는다. 수확 코디네이터가 APPROACH에 들어가
        # /iw/mission FOLLOW를 보낼 때까지 Nav2 goal을 만들지 않는다.
        self._mission = "IDLE"
        self._iw_pose: tuple[float, float, float] | None = None
        self._iw_odom_pose: tuple[float, float, float] | None = None
        # AMCL이 발행한 동적 map→odom을 받은 뒤에만 map pose를 계산한다.
        self._map_to_odom: tuple[float, float, float] | None = None
        self._last_target: tuple[float, float, float] | None = None
        self._request_pending = False
        self._dock_goal_sent = False
        # FORKLIFT 미션 서브페이즈: APPROACH(도크 주행) → WAITING_CLEAR(인계 대기)
        #   → RETURNING(MM 복귀 주행) → (FOLLOW 재개). 도크 미션 재시작 시 APPROACH로 리셋.
        self._dock_phase = "APPROACH"
        self._current_pallet = 0
        self._forklift_request_pending = False
        self._dock_adjust_attempts = 0
        self._dock_align_started: float | None = None
        self._dock_align_stable_since: float | None = None
        self._dock_align_log_at = 0.0
        self._follow_goal_handle = None    # active FOLLOW goal handle (cancel 용)
        self._goal_gen = 0                 # goal 세대 ID — 취소/교체된 goal의 늦은 콜백 무시
        self._started_at = time.monotonic()
        self._last_startup_attempt = 0.0
        self._startup_pending = False
        self._startup_requested_at = 0.0
        self.create_timer(0.5, self._update_goal)
        self.create_timer(0.05, self._update_dock_alignment)
        self.get_logger().info(
            "IW 미션 Nav2 연결: IDLE=정지, FOLLOW=MM 전방 목표 갱신, "
            "FORKLIFT=(0.0,10.84885)")

    def _on_pallet_on_iw(self, msg: Int32) -> None:
        pallet = int(msg.data)
        if 0 <= pallet < 6:
            self._current_pallet = pallet

    def _on_dock_adjust_request(
        self,
        request: DockAdjust.Request,
        response: DockAdjust.Response,
    ) -> DockAdjust.Response:
        """지게차 잠금 실패를 받아 IW Nav2 최종 도킹을 다시 수행한다."""
        if self._mission != "FORKLIFT":
            response.accepted = False
            response.message = (
                f"현재 IW 미션이 FORKLIFT가 아닙니다: {self._mission}"
            )
            return response
        limit = int(self.get_parameter("max_dock_adjust_retries").value)
        if self._dock_adjust_attempts >= limit:
            response.accepted = False
            response.message = f"도킹 재정렬 최대 횟수({limit}) 초과"
            return response

        self._dock_adjust_attempts += 1
        self._dock_goal_sent = False
        self._forklift_request_pending = False
        if self._dock_alignment_capturable():
            self._start_dock_alignment(
                f"지게차 재조정 요청 {self._dock_adjust_attempts}/{limit}"
            )
        else:
            self._dock_phase = "APPROACH"
        response.accepted = True
        response.message = (
            f"IW Nav2 도킹 재정렬 {self._dock_adjust_attempts}/{limit} 접수"
        )
        self.get_logger().warning(
            f"{response.message}: 지게차 사유={request.reason}"
        )
        return response

    @staticmethod
    def _wrap(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))

    def _on_mission(self, msg: String) -> None:
        mission = msg.data.strip().upper()
        if mission not in {"IDLE", "FOLLOW", "PREPARE_FORKLIFT", "FORKLIFT"}:
            self.get_logger().warning(f"알 수 없는 IW 미션 무시: {mission}")
            return
        if mission == self._mission:
            return
        if self._mission == "FOLLOW":
            self._cancel_follow_goal()
        if self._dock_phase == "ALIGNING":
            self._stop_dock_alignment()
        self._mission = mission
        self._last_target = None
        self._dock_goal_sent = False
        if mission == "PREPARE_FORKLIFT":
            # MM이 HOME 상태로 안전 피항을 끝내기 전에는 중앙 하역 레인으로
            # 출발하지 않는다. transient-local 요청이라 분산 노드가 늦게 붙어도 받는다.
            self._yield_request_pub.publish(Bool(data=True))
            self._status_pub.publish(String(data="WAITING_MM_YIELD"))
        elif mission == "FORKLIFT":
            self._dock_phase = "APPROACH"
            self._dock_adjust_attempts = 0
        self.get_logger().info(f"IW 미션 전환: {mission}")

    def _on_mm_yield_complete(self, msg: Bool) -> None:
        """MM 피항 완료 확인 뒤에만 IW 하역 주행을 시작한다."""
        if not msg.data or self._mission != "PREPARE_FORKLIFT":
            return
        self._yield_request_pub.publish(Bool(data=False))
        self._mission = "FORKLIFT"
        self._last_target = None
        self._dock_goal_sent = False
        self._dock_phase = "APPROACH"
        self._dock_adjust_attempts = 0
        self._status_pub.publish(String(data="MM_CLEAR_TO_FORKLIFT"))
        self.get_logger().info(
            "MM 피항 완료 확인 → IW 지게차 하역 경로 출발")

    def _on_iw_odom(self, msg: Odometry) -> None:
        self._iw_odom_pose = (
            float(msg.pose.pose.position.x),
            float(msg.pose.pose.position.y),
            _yaw_from_quaternion(msg.pose.pose.orientation),
        )
        self._update_iw_map_pose()

    def _on_iw_tf(self, msg: TFMessage) -> None:
        for stamped in msg.transforms:
            parent = stamped.header.frame_id.lstrip("/")
            child = stamped.child_frame_id.lstrip("/")
            if parent != "iwhub_0/map" or child != "iwhub_0/odom":
                continue
            transform = stamped.transform
            self._map_to_odom = (
                float(transform.translation.x),
                float(transform.translation.y),
                _yaw_from_quaternion(transform.rotation),
            )
            self._update_iw_map_pose()

    def _update_iw_map_pose(self) -> None:
        if self._iw_odom_pose is None or self._map_to_odom is None:
            return
        odom_x, odom_y, odom_yaw = self._iw_odom_pose
        map_x, map_y, map_yaw = self._map_to_odom
        c, s = math.cos(map_yaw), math.sin(map_yaw)
        self._iw_pose = (
            map_x + c * odom_x - s * odom_y,
            map_y + s * odom_x + c * odom_y,
            self._wrap(map_yaw + odom_yaw),
        )

    def _follow_target(self) -> tuple[float, float, float] | None:
        try:
            transform = self._buffer.lookup_transform(
                str(self.get_parameter("mm_map_frame").value),
                str(self.get_parameter("mm_base_frame").value),
                Time(),
            ).transform
        except TransformException as exc:
            self.get_logger().warning(
                f"MM TF 대기 중: {exc}", throttle_duration_sec=5.0)
            return None
        mm_x = float(transform.translation.x)
        mm_y = float(transform.translation.y)
        if self._iw_pose is None:
            self.get_logger().warning(
                "IW map pose 대기 중: /iwhub_0/tf map→odom + /iwhub_0/odom",
                throttle_duration_sec=5.0,
            )
            return None
        iw_x, iw_y, iw_yaw = self._iw_pose
        # 도킹점은 MM +X(=베드) 고정 오프셋이 아니라, MM 중심에서 IW가 접근하는
        # 방향으로 standoff 거리에 둔다. 베드 반대편 안전한 쪽에 비접촉 정차한다.
        bearing = math.atan2(iw_y - mm_y, iw_x - mm_x)
        standoff = float(self.get_parameter("dock_standoff").value)
        target_x = mm_x + standoff * math.cos(bearing)
        target_y = mm_y + standoff * math.sin(bearing)
        dx = target_x - iw_x
        dy = target_y - iw_y
        # FOLLOW 목표 자세는 MM 자세를 복사하지 않는다. 현재 IW에서 이동할
        # 추종점의 방위를 사용해야 한 번 방향을 잡은 뒤 전진할 수 있다.
        if math.hypot(dx, dy) > 0.10:
            travel_yaw = math.atan2(dy, dx)
        else:
            travel_yaw = iw_yaw
        return target_x, target_y, travel_yaw

    def _target_changed(self, target: tuple[float, float, float]) -> bool:
        if self._last_target is None:
            return True
        distance = math.hypot(
            target[0] - self._last_target[0],
            target[1] - self._last_target[1],
        )
        yaw_delta = abs(self._wrap(target[2] - self._last_target[2]))
        return (
            distance >= float(
                self.get_parameter("follow_update_distance").value)
            or yaw_delta >= float(
                self.get_parameter("follow_update_yaw").value)
        )

    def _update_goal(self) -> None:
        if self._request_pending:
            return
        if self._mission == "IDLE":
            return
        if self._mission == "PREPARE_FORKLIFT":
            return  # MM 피항 완료 신호 전에는 기존 FOLLOW goal도 새 하역 goal도 금지
        if not self._through_client.server_is_ready():
            self._recover_nav2()
            self.get_logger().warning(
                "IW NavigateThroughPoses 서버 대기 중: "
                f"{self.get_parameter('navigate_through_poses_action').value}",
                throttle_duration_sec=5.0,
            )
            return
        if self._mission == "FORKLIFT":
            # 레인 경로 주행(단일 goal 아님) — 통로 중심선만 타 배드 회피 보장.
            if self._dock_phase == "WAITING_CLEAR":
                return   # 포크 인계 완료(/forklift/clear) 대기 중 — 정차
            if self._dock_phase == "ALIGNING":
                return   # 20Hz 도킹 전용 폐루프가 cmd_vel을 직접 제어
            if self._dock_phase == "REQUESTING_SERVICE":
                self._request_forklift_cycle()
                return
            if self._dock_goal_sent:
                return
            if self._iw_pose is None:
                return   # 레인 경로 계획에 현재 map pose 필요
            if self._dock_phase == "APPROACH":
                self._send_dock_route()
            else:                        # RETURNING — MM 부근으로 복귀
                if not self._send_return_route():
                    return               # MM TF 아직 — 다음 주기 재시도
            self._dock_goal_sent = True
            return
        # FOLLOW
        target = self._follow_target()
        if target is None or not self._target_changed(target):
            return
        self._send_follow_route(target)

    def _request_forklift_cycle(self) -> None:
        """도킹 후 현재 IW 팔레트 번호로 지게차 교환 서비스를 한 번 호출한다."""
        if self._forklift_request_pending:
            return
        if not self._forklift_client.service_is_ready():
            self.get_logger().warning(
                "지게차 서비스 /forklift/start_cycle 대기 중",
                throttle_duration_sec=3.0,
            )
            return
        request = ForkliftCycle.Request()
        request.inbound_pallet = int(self._current_pallet)
        if self._iw_pose is None:
            return
        request.dock_x = float(self._iw_pose[0])
        request.dock_y = float(self._iw_pose[1])
        request.dock_yaw = float(self._iw_pose[2])
        self._forklift_request_pending = True
        future = self._forklift_client.call_async(request)
        future.add_done_callback(self._forklift_cycle_response)
        self.get_logger().info(
            f"지게차 서비스 요청: Pallet_{self._current_pallet:02d} "
            "랙 복귀·다음 팔레트 IW 상차, "
            f"IW pose=({request.dock_x:.3f}, {request.dock_y:.3f}, "
            f"{math.degrees(request.dock_yaw):.1f}deg)")

    def _publish_dock_cmd(self, linear: float, angular: float) -> None:
        command = Twist()
        command.linear.x = float(linear)
        command.angular.z = float(angular)
        self._dock_cmd_pub.publish(command)

    def _dock_alignment_capturable(self) -> bool:
        if self._iw_pose is None:
            return False
        x, y, _ = self._iw_pose
        dx, dy, _ = lanes.DOCK
        return math.hypot(x - dx, y - dy) <= float(
            self.get_parameter("dock_align_capture_radius").value
        )

    def _start_dock_alignment(self, reason: str) -> None:
        """Nav2 주행이 끝난 도크 근처에서만 저속 pose 폐루프를 시작한다."""
        self._dock_phase = "ALIGNING"
        self._dock_goal_sent = True
        self._dock_align_started = time.monotonic()
        self._dock_align_stable_since = None
        self._dock_align_log_at = 0.0
        self._publish_dock_cmd(0.0, 0.0)
        self.get_logger().info(
            f"IW DOCK_ALIGN 시작: {reason} — "
            "고정 도크 X/Y와 yaw를 저속으로 동시 보정"
        )

    def _stop_dock_alignment(self) -> None:
        self._publish_dock_cmd(0.0, 0.0)
        self._dock_align_started = None
        self._dock_align_stable_since = None

    def _update_dock_alignment(self) -> None:
        """도크 근처 전용 unicycle 폐루프. 일반 Nav2 속도에는 관여하지 않는다."""
        if self._mission != "FORKLIFT" or self._dock_phase != "ALIGNING":
            return
        if self._iw_pose is None or self._dock_align_started is None:
            self._publish_dock_cmd(0.0, 0.0)
            return

        now = time.monotonic()
        x, y, yaw = self._iw_pose
        target_x, target_y, target_yaw = lanes.DOCK
        error_x = target_x - x
        error_y = target_y - y
        position_error = math.hypot(error_x, error_y)
        yaw_error = self._wrap(target_yaw - yaw)
        position_tolerance = float(
            self.get_parameter("dock_align_position_tolerance").value
        )
        yaw_tolerance = float(
            self.get_parameter("dock_align_yaw_tolerance").value
        )

        if (
            position_error <= position_tolerance
            and abs(yaw_error) <= yaw_tolerance
        ):
            self._publish_dock_cmd(0.0, 0.0)
            if self._dock_align_stable_since is None:
                self._dock_align_stable_since = now
            if now - self._dock_align_stable_since >= float(
                self.get_parameter("dock_align_settle_sec").value
            ):
                self._stop_dock_alignment()
                self._dock_phase = "REQUESTING_SERVICE"
                self._dock_goal_sent = False
                self.get_logger().info(
                    "IW DOCK_ALIGN 완료: "
                    f"xy_error={position_error:.3f}m, "
                    f"yaw_error={math.degrees(yaw_error):.1f}deg "
                    "안정 유지 → 지게차 서비스 요청"
                )
            return
        self._dock_align_stable_since = None

        capture_radius = float(
            self.get_parameter("dock_align_capture_radius").value
        )
        timeout = float(
            self.get_parameter("dock_align_timeout_sec").value
        )
        if position_error > capture_radius or now - self._dock_align_started > timeout:
            self._stop_dock_alignment()
            self._dock_phase = "APPROACH"
            self._dock_goal_sent = False
            self.get_logger().warning(
                "IW DOCK_ALIGN 범위/시간 초과 → Nav2 도크 접근부터 재시도: "
                f"xy_error={position_error:.3f}m, "
                f"yaw_error={math.degrees(yaw_error):.1f}deg"
            )
            return

        max_linear = float(
            self.get_parameter("dock_align_max_linear_speed").value
        )
        max_angular = float(
            self.get_parameter("dock_align_max_angular_speed").value
        )
        linear = 0.0
        angular = max(-max_angular, min(max_angular, 0.9 * yaw_error))
        # base_node의 각속도 재시작 deadband(0.04rad/s)보다 작은 명령은
        # 실제 바퀴까지 전달되지 않아 약 2도에서 영원히 멈춘다. 허용오차
        # 밖에서는 방향을 유지한 최소 0.045rad/s를 보내 최종 yaw를 끝낸다.
        if (
            abs(yaw_error) > yaw_tolerance
            and abs(angular) < 0.045
        ):
            angular = math.copysign(0.045, yaw_error)
        if position_error > position_tolerance:
            bearing = math.atan2(error_y, error_x)
            forward_error = self._wrap(bearing - yaw)
            direction = 1.0
            if abs(forward_error) > math.pi / 2.0:
                direction = -1.0
                forward_error = self._wrap(forward_error - math.pi)
            linear = direction * min(
                max_linear, max(0.02, 0.7 * position_error)
            )
            # 위치를 향하는 조향과 최종 yaw를 함께 반영한다.
            angular = max(
                -max_angular,
                min(max_angular, 1.2 * forward_error + 0.35 * yaw_error),
            )
        self._publish_dock_cmd(linear, angular)

        if now - self._dock_align_log_at >= 1.0:
            self._dock_align_log_at = now
            self.get_logger().info(
                "IW DOCK_ALIGN: "
                f"xy={position_error:.3f}m, "
                f"yaw={math.degrees(yaw_error):.1f}deg, "
                f"cmd=({linear:.3f}m/s,{angular:.3f}rad/s)"
            )

    def _forklift_cycle_response(self, future) -> None:
        self._forklift_request_pending = False
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().error(f"지게차 서비스 응답 실패: {exc}")
            return
        if response is None or not response.accepted:
            message = "응답 없음" if response is None else response.message
            self.get_logger().warning(
                f"지게차 사이클 접수 거부: {message}")
            return
        self._dock_phase = "WAITING_CLEAR"
        self._status_pub.publish(String(data="ARRIVED_FORKLIFT"))
        self.get_logger().info(
            f"지게차 사이클 접수 완료: 다음 Pallet_"
            f"{int(response.outbound_pallet):02d}, 완료 신호 대기")

    def _mm_map_xy(self) -> tuple[float, float] | None:
        """MM base_link 의 map 좌표 (x, y). TF 없으면 None."""
        try:
            t = self._buffer.lookup_transform(
                str(self.get_parameter("mm_map_frame").value),
                str(self.get_parameter("mm_base_frame").value),
                Time(),
            ).transform
        except TransformException:
            return None
        return float(t.translation.x), float(t.translation.y)

    def _cancel_follow_goal(self) -> None:
        """진행 중 FOLLOW goal을 취소하고, 늦은 콜백이 상태를 덮지 않게 세대를 올린다."""
        if self._follow_goal_handle is not None:
            try:
                self._follow_goal_handle.cancel_goal_async()
            except Exception as exc:
                self.get_logger().warning(f"FOLLOW goal cancel 실패: {exc}")
            self._follow_goal_handle = None
        self._last_target = None
        self._goal_gen += 1

    def _make_pose(self, x: float, y: float, yaw: float) -> PoseStamped:
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = str(self.get_parameter("goal_frame").value)
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    def _send_through_route(
        self,
        route,
        purpose: str,
        log_msg: str,
        target: tuple[float, float, float] | None = None,
    ) -> None:
        """레인 경로(웨이포인트 리스트)를 전진 전용 BT로 NavigateThroughPoses 전송."""
        if purpose == "FOLLOW" and self._follow_goal_handle is not None:
            # 움직이는 MM 목표가 갱신되면 이전 레인 goal을 명시적으로 취소한다.
            # action server의 암묵적 preempt에만 기대면 이전 경로가 잠시 더 진행될 수 있다.
            try:
                self._follow_goal_handle.cancel_goal_async()
            except Exception as exc:
                self.get_logger().warning(f"이전 FOLLOW goal cancel 실패: {exc}")
            self._follow_goal_handle = None
        goal = NavigateThroughPoses.Goal()
        goal.poses = [self._make_pose(x, y, yaw) for (x, y, yaw) in route]
        # 전진 전용 BT — 기본 BT의 Spin/BackUp 복구를 배제(회전이 AMCL/라이다 정합을 흔듦).
        bt_name = (
            "dock_final_pose.xml"
            if purpose == "DOCK_FINAL"
            else "forward_only_through_poses.xml"
        )
        goal.behavior_tree = os.path.join(
            get_package_share_directory("iwhub_control"),
            "behavior_trees", bt_name)
        self._request_pending = True
        self._goal_gen += 1
        gen = self._goal_gen
        self.get_logger().info(log_msg)
        future = self._through_client.send_goal_async(goal)
        future.add_done_callback(
            lambda result, g=gen, p=purpose, t=target:
            self._through_response(result, g, p, t))

    def _send_follow_route(
        self,
        target: tuple[float, float, float],
        purpose: str = "FOLLOW",
    ) -> None:
        """MM 추종점을 레인 경로로 연결해 전진 전용 BT로 보낸다."""
        if self._iw_pose is None:
            return
        iw_x, iw_y, iw_yaw = self._iw_pose
        try:
            route = lanes.follow_route(
                iw_x, iw_y, iw_yaw, target[0], target[1],
                # 정확한 standoff 점이 세로 레인에서 수십 cm 벗어나면 마지막에
                # 차체가 통과할 수 없는 작은 90도 코너가 생겨 제자리 회전한다.
                # 레인 중심 정차를 우선하고 standoff의 수 cm 오차를 허용한다.
                snap_target_x=True,
            )
        except ValueError as exc:
            self.get_logger().error(
                f"FOLLOW 레인 경로 생성 거부: {exc}",
                throttle_duration_sec=3.0,
            )
            self._cancel_follow_goal()
            return
        # 목표 방향은 임의의 직선 bearing이 아니라 검증된 레인 경로의 마지막
        # 접선 방향을 사용한다. 그래야 목표 앞에서 불필요한 제자리 회전을 하지 않는다.
        routed_target = (route[-1][0], route[-1][1], route[-1][2])
        self._send_through_route(
            route,
            purpose,
            f"IW {purpose} 레인 경로 {len(route)}웨이포인트 "
            f"({iw_x:.1f},{iw_y:.1f} → "
            f"{routed_target[0]:.1f},{routed_target[1]:.1f})",
            # 갱신 비교에는 매번 같은 방식으로 계산되는 원래 추종점을 저장한다.
            # 경로 마지막 접선 yaw를 저장하면 정지한 MM에도 yaw 차이로 재전송된다.
            target=target,
        )

    def _send_dock_route(self) -> None:
        """현재 위치→지게차 도크까지 통로 레인 경로를 NavigateThroughPoses로 보낸다."""
        iw_x, iw_y, iw_yaw = self._iw_pose
        route = lanes.dock_route(iw_x, iw_y, iw_yaw)
        self._send_through_route(
            route,
            "DOCK",
            f"IW 도크 레인 경로 {len(route)}웨이포인트 (시작 {iw_x:.1f},{iw_y:.1f} "
            f"→ 도크 {lanes.DOCK[0]:.1f},{lanes.DOCK[1]:.1f})")

    def _send_return_route(self) -> bool:
        """도크→MM 부근 복귀 경로 전송. MM TF 없으면 False(다음 주기 재시도)."""
        mm_xy = self._mm_map_xy()
        if mm_xy is None:
            self.get_logger().warning(
                "복귀 경로: MM TF 대기 중", throttle_duration_sec=5.0)
            return False
        dx, dy, dyaw = lanes.DOCK
        route = lanes.return_route(dx, dy, dyaw, mm_xy[1])
        self._send_through_route(
            route,
            "RETURN",
            f"IW 복귀 레인 경로 {len(route)}웨이포인트 (도크 → MM Y부근 "
            f"{mm_xy[1]:.1f}) — 도착 후 FOLLOW 재개")
        return True

    def _on_forklift_clear(self, msg: Bool) -> None:
        """포크 인계 완료(/forklift/clear=True) → 복귀 개시. WAITING_CLEAR에서만 유효."""
        if not msg.data:
            return
        if self._mission != "FORKLIFT" or self._dock_phase != "WAITING_CLEAR":
            return
        self.get_logger().info("포크 인계 완료(/forklift/clear) → MM 복귀 시작")
        self._dock_phase = "RETURNING"
        self._dock_goal_sent = False

    def _through_response(self, future, gen, purpose, target) -> None:
        self._request_pending = False
        if gen != self._goal_gen:
            return
        try:
            handle = future.result()
        except Exception as exc:
            self.get_logger().error(
                f"IW {purpose} 레인 goal 전송 실패: {exc}")
            if purpose == "FOLLOW":
                self._last_target = None
            else:
                self._dock_goal_sent = False
            return
        if not handle.accepted:
            self.get_logger().warning(f"IW {purpose} 레인 goal 거부")
            if purpose == "FOLLOW":
                self._last_target = None
            else:
                self._dock_goal_sent = False
            return
        if purpose == "FOLLOW":
            self._follow_goal_handle = handle
            self._last_target = target
        result = handle.get_result_async()
        result.add_done_callback(
            lambda done, g=gen, p=purpose:
            self._through_result(done, g, p))

    def _through_result(self, future, gen, purpose) -> None:
        if gen != self._goal_gen:
            return
        if purpose == "FOLLOW":
            self._follow_goal_handle = None
        try:
            status = future.result().status
        except Exception as exc:
            self.get_logger().error(
                f"IW {purpose} 레인 경로 결과 수신 실패: {exc}")
            return
        if purpose == "FOLLOW":
            if status not in {
                GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_CANCELED
            }:
                self.get_logger().warning(
                    f"IW FOLLOW 레인 경로 실패(status={status})")
                if self._mission == "FOLLOW":
                    self._last_target = None
            return
        if status == GoalStatus.STATUS_SUCCEEDED:
            if purpose in {"DOCK", "DOCK_FINAL"}:
                self._start_dock_alignment(
                    "Nav2 도크 접근 완료"
                )
            else:                                               # RETURN 완료
                self._status_pub.publish(String(data="RETURNED"))
                # IW→MM 작업 재개 신호.
                self._resume_pub.publish(Bool(data=True))
                self._mission = "FOLLOW"                        # 추종 재개
                self._dock_phase = "APPROACH"                   # 다음 도크 미션 리셋
                self._dock_goal_sent = False
                self.get_logger().info(
                    "IW MM 복귀 완료 → /iw/status RETURNED + "
                    "/iw/resume_harvest=True (MM 작업 재개 요청) → FOLLOW 재개")
        elif status != GoalStatus.STATUS_CANCELED:
            self.get_logger().warning(
                f"IW {purpose} 레인 경로 실패(status={status})")
            self._dock_goal_sent = False

    def _dock_pose_ready(self) -> bool:
        """Isaac 도크 잠금과 같은 실제 map pose 허용오차를 검사한다."""
        if self._iw_pose is None:
            return False
        x, y, yaw = self._iw_pose
        dx, dy, target_yaw = lanes.DOCK
        # 도킹은 X/Y/yaw 세 조건을 동시에 판정한다. 물리 정착 오차 안에서
        # 통과한 실제 yaw를 서비스로 넘기며 yaw만 별도로 강제하지 않는다.
        return (
            math.hypot(x - dx, y - dy) <= 0.04
            and abs(self._wrap(yaw - target_yaw)) <= math.radians(6.0)
        )

    def _recover_nav2(self) -> None:
        """IW navigation lifecycle이 안 뜨면 STARTUP을 반복 요청한다."""
        now = time.monotonic()
        delay = float(self.get_parameter("nav_startup_delay_sec").value)
        retry = float(self.get_parameter("nav_startup_retry_sec").value)
        if now - self._started_at < delay:
            return
        if self._startup_pending:
            # DDS 응답 자체가 유실된 경우에도 영구 대기하지 않고 다시 요청한다.
            if now - self._startup_requested_at < retry:
                return
            self._startup_pending = False
            self.get_logger().warning(
                "IW lifecycle STARTUP 응답 timeout → 자동 재시도")
        if now - self._last_startup_attempt < retry:
            return
        if not self._lifecycle_client.service_is_ready():
            self.get_logger().warning(
                "IW lifecycle manager 서비스 대기 중",
                throttle_duration_sec=5.0,
            )
            return

        request = ManageLifecycleNodes.Request()
        request.command = ManageLifecycleNodes.Request.STARTUP
        self._startup_pending = True
        self._startup_requested_at = now
        self._last_startup_attempt = now
        future = self._lifecycle_client.call_async(request)
        future.add_done_callback(self._startup_response)
        self.get_logger().warning("IW Nav2 비활성 감지 → lifecycle STARTUP 요청")

    def _startup_response(self, future) -> None:
        self._startup_pending = False
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().error(f"IW lifecycle STARTUP 호출 실패: {exc}")
            return
        if response is not None and response.success:
            self.get_logger().info("IW Nav2 lifecycle STARTUP 완료")
        else:
            self.get_logger().warning(
                "IW Nav2 lifecycle STARTUP 미완료 — 자동 재시도 예정")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MissionNavNode()
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
