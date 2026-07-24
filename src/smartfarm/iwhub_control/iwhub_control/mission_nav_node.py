"""FOLLOW/FORKLIFT 미션을 IW 전용 Nav2 NavigateToPose goal로 변환한다."""
from __future__ import annotations

import math
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateThroughPoses, NavigateToPose
from nav2_msgs.srv import ManageLifecycleNodes
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import String
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
            "navigate_to_pose_action", "/iwhub_0/navigate_to_pose")
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
        self.declare_parameter("follow_offset_x", 2.3)
        self.declare_parameter("follow_offset_y", 0.0)
        self.declare_parameter("follow_update_distance", 0.30)
        self.declare_parameter("follow_update_yaw", math.radians(30.0))
        # 갭 게이팅(히스테리시스): min_gap 이하로 붙으면 정지(active goal cancel), resume_gap
        # 이상으로 벌어지면 재개. follow_offset_x(2.3) 보다 작게 둬야 정상 추종점이 안 걸린다.
        self.declare_parameter("follow_min_gap", 1.8)
        self.declare_parameter("follow_resume_gap", 2.1)
        self.declare_parameter("dock_x", 0.0)
        self.declare_parameter("dock_y", 10.84885)
        self.declare_parameter("dock_yaw", math.pi / 2.0)

        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._status_pub = self.create_publisher(
            String, "/iw/status", latched)
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
        self._client = ActionClient(
            self, NavigateToPose,
            str(self.get_parameter("navigate_to_pose_action").value),
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
        self._mission = "FOLLOW"
        self._iw_pose: tuple[float, float, float] | None = None
        self._iw_odom_pose: tuple[float, float, float] | None = None
        self._map_to_odom: tuple[float, float, float] | None = None
        self._last_target: tuple[float, float, float] | None = None
        self._request_pending = False
        self._dock_goal_sent = False
        self._follow_held = False          # 갭 게이팅 히스테리시스 상태(hold 중?)
        self._follow_goal_handle = None    # active FOLLOW goal handle (cancel 용)
        self._goal_gen = 0                 # goal 세대 ID — 취소/교체된 goal의 늦은 콜백 무시
        self._started_at = time.monotonic()
        self._last_startup_attempt = 0.0
        self._startup_pending = False
        self._startup_requested_at = 0.0
        self.create_timer(0.5, self._update_goal)
        self.get_logger().info(
            "IW 미션 Nav2 연결: FOLLOW=MM 전방 목표 갱신, "
            "FORKLIFT=(0.0,10.84885)")

    @staticmethod
    def _wrap(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))

    def _on_mission(self, msg: String) -> None:
        mission = msg.data.strip().upper()
        if mission not in {"FOLLOW", "FORKLIFT"}:
            self.get_logger().warning(f"알 수 없는 IW 미션 무시: {mission}")
            return
        if mission == self._mission:
            return
        self._mission = mission
        self._last_target = None
        self._dock_goal_sent = False
        self.get_logger().info(f"IW 미션 전환: {mission}")

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
        yaw = _yaw_from_quaternion(transform.rotation)
        ox = float(self.get_parameter("follow_offset_x").value)
        oy = float(self.get_parameter("follow_offset_y").value)
        c, s = math.cos(yaw), math.sin(yaw)
        target_x = transform.translation.x + ox * c - oy * s
        target_y = transform.translation.y + ox * s + oy * c
        if self._iw_pose is None:
            self.get_logger().warning(
                "IW map pose 대기 중: /iwhub_0/tf map→odom + /iwhub_0/odom",
                throttle_duration_sec=5.0,
            )
            return None

        iw_x, iw_y, iw_yaw = self._iw_pose
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
        if not self._client.server_is_ready():
            self._recover_nav2()
            self.get_logger().warning(
                "IW Nav2 action 서버 대기 중: "
                f"{self.get_parameter('navigate_to_pose_action').value}",
                throttle_duration_sec=5.0,
            )
            return
        if self._mission == "FORKLIFT":
            # 도크로 레인 경로 주행(단일 goal 아님) — 통로 중심선만 타 배드 회피 보장.
            if self._dock_goal_sent:
                return
            if self._iw_pose is None:
                return   # 레인 경로 계획에 현재 map pose 필요
            if not self._through_client.server_is_ready():
                self.get_logger().warning(
                    "IW NavigateThroughPoses 서버 대기 중",
                    throttle_duration_sec=5.0)
                return
            self._send_dock_route()
            self._dock_goal_sent = True
            return
        # FOLLOW
        if not self._follow_gap_ok():
            return
        target = self._follow_target()
        if target is None or not self._target_changed(target):
            return
        self._send_goal(target, self._mission)

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

    def _follow_gap_ok(self) -> bool:
        """MM↔IW 실거리 히스테리시스. hold 진입 시 active goal을 실제로 취소한다
        (새 goal 미전송만으론 진행 중 goal이 계속 접근하므로). True=추종 진행 가능."""
        if self._iw_pose is None:
            return False
        mm_xy = self._mm_map_xy()
        if mm_xy is None:
            return False
        gap = math.hypot(mm_xy[0] - self._iw_pose[0],
                         mm_xy[1] - self._iw_pose[1])
        stop_gap = float(self.get_parameter("follow_min_gap").value)
        resume_gap = float(self.get_parameter("follow_resume_gap").value)
        if not self._follow_held and gap < stop_gap:
            self._follow_held = True
            self._cancel_follow_goal()
            self.get_logger().info(
                f"MM 근접 {gap:.2f}m<{stop_gap:.2f} — 추종 정지(goal cancel)")
        elif self._follow_held and gap > resume_gap:
            self._follow_held = False
            self.get_logger().info(
                f"갭 회복 {gap:.2f}m>{resume_gap:.2f} — 추종 재개")
        return not self._follow_held

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

    def _send_dock_route(self) -> None:
        """현재 위치→지게차 도크까지 통로 레인 경로를 NavigateThroughPoses로 보낸다."""
        iw_x, iw_y, _ = self._iw_pose
        route = lanes.dock_route(iw_x, iw_y)
        goal = NavigateThroughPoses.Goal()
        goal.poses = [self._make_pose(x, y, yaw) for (x, y, yaw) in route]
        self._request_pending = True
        self._goal_gen += 1
        gen = self._goal_gen
        self.get_logger().info(
            f"IW 도크 레인 경로 {len(route)}웨이포인트 "
            f"(시작 {iw_x:.1f},{iw_y:.1f} → 도크 "
            f"{lanes.DOCK[0]:.1f},{lanes.DOCK[1]:.1f})")
        future = self._through_client.send_goal_async(goal)
        future.add_done_callback(
            lambda result, g=gen: self._through_response(result, g))

    def _through_response(self, future, gen) -> None:
        self._request_pending = False
        if gen != self._goal_gen:
            return
        try:
            handle = future.result()
        except Exception as exc:
            self.get_logger().error(f"IW 도크 경로 goal 전송 실패: {exc}")
            self._dock_goal_sent = False
            return
        if not handle.accepted:
            self.get_logger().warning("IW 도크 경로 goal 거부")
            self._dock_goal_sent = False
            return
        result = handle.get_result_async()
        result.add_done_callback(
            lambda done, g=gen: self._through_result(done, g))

    def _through_result(self, future, gen) -> None:
        if gen != self._goal_gen:
            return
        try:
            status = future.result().status
        except Exception as exc:
            self.get_logger().error(f"IW 도크 경로 결과 수신 실패: {exc}")
            return
        if status == GoalStatus.STATUS_SUCCEEDED:
            self._status_pub.publish(String(data="ARRIVED_FORKLIFT"))
            self.get_logger().info(
                "IW 지게차 도킹 완료(레인 경로) → /iw/status ARRIVED_FORKLIFT")
        elif status != GoalStatus.STATUS_CANCELED:
            self.get_logger().warning(f"IW 도크 경로 실패(status={status})")
            self._dock_goal_sent = False

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

    def _send_goal(
        self, target: tuple[float, float, float], mission: str
    ) -> None:
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = str(self.get_parameter("goal_frame").value)
        pose.pose.position.x = target[0]
        pose.pose.position.y = target[1]
        pose.pose.orientation.z = math.sin(target[2] / 2.0)
        pose.pose.orientation.w = math.cos(target[2] / 2.0)
        goal = NavigateToPose.Goal()
        goal.pose = pose
        self._request_pending = True
        self._goal_gen += 1
        gen = self._goal_gen
        future = self._client.send_goal_async(goal)
        future.add_done_callback(
            lambda result, m=mission, t=target, g=gen:
            self._goal_response(result, m, t, g)
        )

    def _goal_response(self, future, mission, target, gen) -> None:
        self._request_pending = False
        if gen != self._goal_gen:
            return   # 취소/교체된 goal의 늦은 응답 — 무시
        try:
            handle = future.result()
        except Exception as exc:
            self.get_logger().error(f"IW Nav2 goal 전송 실패: {exc}")
            if mission == "FORKLIFT":
                self._dock_goal_sent = False
            return
        if not handle.accepted:
            self.get_logger().warning(f"IW Nav2 {mission} goal 거부")
            if mission == "FORKLIFT":
                self._dock_goal_sent = False
            return
        self._last_target = target
        if mission == "FOLLOW":
            self._follow_goal_handle = handle
        self.get_logger().info(
            f"IW Nav2 {mission} goal: "
            f"({target[0]:.2f},{target[1]:.2f},{math.degrees(target[2]):.1f}°)")
        result = handle.get_result_async()
        result.add_done_callback(
            lambda done, m=mission, g=gen:
            self._goal_result(done, m, g)
        )

    def _goal_result(self, future, mission: str, gen: int) -> None:
        if gen != self._goal_gen:
            return   # 취소/교체된 goal의 늦은 결과 — 무시
        if mission == "FOLLOW":
            self._follow_goal_handle = None
        try:
            status = future.result().status
        except Exception as exc:
            self.get_logger().error(f"IW Nav2 결과 수신 실패: {exc}")
            return
        if mission == "FORKLIFT" and status == GoalStatus.STATUS_SUCCEEDED:
            self._status_pub.publish(String(data="ARRIVED_FORKLIFT"))
            self.get_logger().info(
                "IW 지게차 도킹 완료 → /iw/status ARRIVED_FORKLIFT")
        elif status not in {
            GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_CANCELED
        }:
            self.get_logger().warning(
                f"IW Nav2 {mission} 실패(status={status})")
            if mission == self._mission:
                self._last_target = None
                if mission == "FORKLIFT":
                    self._dock_goal_sent = False


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
