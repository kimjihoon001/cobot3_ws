#!/usr/bin/env python3
"""harvest_vision의 기존 이동 명령을 MM MoveGroup 액션으로 변환한다.

Isaac 전용 명령(gripper/grasp_check/follow_check)은 같은 ``cmd`` 토픽으로 그대로
전달되고, 이 노드는 rmp_target/rmp_home/rmp_preplace만 소비한다. MoveIt 이동 결과와
Isaac의 파지 상태를 ``pipeline_status`` 한 채널로 합쳐 기존 수확 FSM이 백엔드 변경을
모르고도 같은 상태 머신을 사용할 수 있게 한다.
"""
from __future__ import annotations

import json
import math

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (BoundingVolume, Constraints, JointConstraint,
                             OrientationConstraint, PositionConstraint)
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import String


HOME_Q = {
    "joint_1": math.radians(180.0),
    "joint_2": math.radians(60.0),
    "joint_3": math.radians(-120.0),
    "joint_4": 0.0,
    "joint_5": math.radians(-30.0),
    "joint_6": math.radians(90.0),
}


class MMMotionBridge(Node):
    def __init__(self) -> None:
        super().__init__("mm_motion_bridge")
        self.declare_parameter("group_name", "mm_manipulator")
        self.declare_parameter("planning_frame", "base_link")
        # 기존 harvest_moveit/grasp_proto의 스쿱 수용 정밀도와 동일하다.
        self.declare_parameter("position_tolerance_m", 0.0025)
        self.declare_parameter("planning_time_sec", 8.0)
        self.declare_parameter("velocity_scale", 0.30)
        self.declare_parameter(
            "planning_pipeline", "pilz_industrial_motion_planner")
        self.declare_parameter("planner_id", "PTP")
        self.declare_parameter("constrain_tool_orientation", True)
        self.declare_parameter("orientation_tolerance_rad", 0.035)
        # BED_VIEW 방향에서 1축이 크게 돌아 반대 IK 해로 넘어가지 않도록
        # APPROACH OMPL 경로의 joint_1 허용 범위를 현재각 주변으로 제한한다.
        self.declare_parameter("approach_joint_1_tolerance_rad", 0.55)
        self.declare_parameter("joint_state_max_age_sec", 0.5)
        self.declare_parameter("control_failure_retries", 2)

        self._move = ActionClient(self, MoveGroup, "move_action")
        self._status_pub = self.create_publisher(String, "pipeline_status", 20)
        self.create_subscription(String, "status", self._forward_isaac_status, 20)
        self.create_subscription(String, "cmd", self._command, 20)
        self._goal_handle = None
        self._active_id = 0
        self._active_phase = ""
        self._queued: tuple[int, str, MoveGroup.Goal] | None = None
        self._send_pending = False
        self._last_succeeded: tuple[int, str] | None = None
        self._joint_positions: dict[str, float] = {}
        self._last_joint_state_receive_ns = 0
        self._last_joint_state_stamp_ns = 0
        self._active_goal_message: MoveGroup.Goal | None = None
        self._control_retry_count = 0
        self.create_subscription(
            JointState, "joint_states", self._joint_state, 20)
        self.create_timer(0.1, self._dispatch)
        self.get_logger().info(
            "MM motion bridge 시작: cmd → move_action, "
            "status + MoveIt → pipeline_status")

    def _forward_isaac_status(self, message: String) -> None:
        # Isaac payload는 이미 120 byte 이하의 유효 JSON으로 제한돼 있다.
        self._status_pub.publish(message)

    def _command(self, message: String) -> None:
        try:
            command = json.loads(message.data)
        except (TypeError, ValueError):
            return
        if not isinstance(command, dict):
            return
        target = command.get("rmp_target")
        if isinstance(target, dict):
            try:
                request_id = int(target.get("id", 0))
                phase = str(target.get("phase", "MOVE"))
                position = [float(value) for value in target["position"]]
                if len(position) != 3 or not all(math.isfinite(v) for v in position):
                    raise ValueError("position")
                direction = target.get("approach_direction")
                if direction is not None:
                    direction = [float(value) for value in direction]
                    if (len(direction) != 3
                            or not all(math.isfinite(v) for v in direction)
                            or math.sqrt(sum(v * v for v in direction)) < 1e-6):
                        raise ValueError("approach_direction")
                orientation = target.get("tool_orientation")
                if orientation is not None:
                    orientation = [float(value) for value in orientation]
                    if (len(orientation) != 4
                            or not all(math.isfinite(v) for v in orientation)
                            or math.sqrt(sum(v * v for v in orientation)) < 1e-6):
                        raise ValueError("tool_orientation")
                motion = str(target.get("motion", "PTP")).upper()
                if motion not in {"OMPL", "PTP", "LIN", "CIRC"}:
                    raise ValueError("motion")
                interim = target.get("interim")
                if interim is not None:
                    interim = [float(value) for value in interim]
                    if (len(interim) != 3
                            or not all(math.isfinite(v) for v in interim)):
                        raise ValueError("interim")
                if motion == "CIRC" and interim is None:
                    raise ValueError("CIRC interim")
                velocity_scale = target.get("velocity_scale")
                if velocity_scale is not None:
                    velocity_scale = float(velocity_scale)
                    if not 0.0 < velocity_scale <= 1.0:
                        raise ValueError("velocity_scale")
                lock_joint_1 = bool(target.get("lock_joint_1", False))
            except (KeyError, TypeError, ValueError):
                self.get_logger().warning("잘못된 rmp_target 형식")
                return
            frame = str(target.get(
                "frame_id", self.get_parameter("planning_frame").value))
            self._queue(
                request_id, phase,
                self._pose_goal(
                    position, frame, direction, orientation, motion, interim,
                    velocity_scale, lock_joint_1))
            return
        home = command.get("rmp_home")
        if isinstance(home, dict):
            request_id = int(home.get("id", 0))
            self._queue(request_id, "GO_HOME", self._joint_goal(HOME_Q))
            return
        bed_view = command.get("moveit_bed_view")
        if isinstance(bed_view, dict):
            try:
                request_id = int(bed_view.get("id", 0))
                joint_1 = float(bed_view["joint_1"])
                if not math.isfinite(joint_1):
                    raise ValueError("joint_1")
            except (KeyError, TypeError, ValueError):
                self.get_logger().warning("잘못된 moveit_bed_view 형식")
                return
            positions = dict(HOME_Q)
            positions["joint_1"] = joint_1
            self._queue(
                request_id, "BED_VIEW",
                self._joint_goal(positions, motion="OMPL"))
            return
        preplace = command.get("rmp_preplace")
        if isinstance(preplace, dict):
            # 기존 RMP 구현의 preplace는 과실을 든 채 접힘 자세로 올리는 단계다.
            # MM에서는 충돌회피 OMPL 홈 이동이 같은 목적을 수행한다.
            request_id = int(preplace.get("id", 0))
            self._queue(request_id, "PRE_PLACE", self._joint_goal(HOME_Q))
            return
        if command.get("rmp_stop") is True:
            self._queued = None
            if self._goal_handle is not None:
                self._goal_handle.cancel_goal_async()

    def _base_goal(self) -> MoveGroup.Goal:
        goal = MoveGroup.Goal()
        request = goal.request
        request.group_name = str(self.get_parameter("group_name").value)
        request.allowed_planning_time = float(
            self.get_parameter("planning_time_sec").value)
        request.num_planning_attempts = 3
        request.max_velocity_scaling_factor = float(
            self.get_parameter("velocity_scale").value)
        request.max_acceleration_scaling_factor = 0.15
        request.pipeline_id = str(
            self.get_parameter("planning_pipeline").value)
        request.planner_id = str(self.get_parameter("planner_id").value)
        return goal

    def _pose_goal(
        self,
        position: list[float],
        frame: str,
        approach_direction: list[float] | None = None,
        tool_orientation: list[float] | None = None,
        motion: str = "PTP",
        interim: list[float] | None = None,
        velocity_scale: float | None = None,
        lock_joint_1: bool = False,
    ) -> MoveGroup.Goal:
        goal = self._base_goal()
        if motion == "OMPL":
            goal.request.pipeline_id = "ompl"
            goal.request.planner_id = "RRTConnectkConfigDefault"
        else:
            goal.request.pipeline_id = "pilz_industrial_motion_planner"
            goal.request.planner_id = motion
        # grasp_proto의 접근 속도를 그대로 사용한다. 원호 수용은 천천히 받쳐 올리고
        # LIN 진입/후퇴는 더 느리게 해 과실과 잎을 옆으로 밀지 않는다.
        goal.request.max_velocity_scaling_factor = {
            "OMPL": 0.20,
            "PTP": 0.25,
            "LIN": 0.08,
            "CIRC": 0.10,
        }[motion] if velocity_scale is None else velocity_scale
        goal.request.max_acceleration_scaling_factor = 0.30
        constraint = Constraints()
        pc = PositionConstraint()
        pc.header.frame_id = frame
        pc.link_name = "harvest_tcp"
        tolerance = float(self.get_parameter("position_tolerance_m").value)
        pc.constraint_region = BoundingVolume(
            primitives=[SolidPrimitive(
                type=SolidPrimitive.SPHERE, dimensions=[tolerance])],
            primitive_poses=[Pose()],
        )
        pose = pc.constraint_region.primitive_poses[0]
        pose.position.x, pose.position.y, pose.position.z = position
        pose.orientation.w = 1.0
        pc.weight = 1.0
        constraint.position_constraints = [pc]
        if ((tool_orientation is not None or approach_direction is not None)
                and bool(self.get_parameter(
                    "constrain_tool_orientation").value)):
            if tool_orientation is not None:
                norm = math.sqrt(sum(value * value for value in tool_orientation))
                qx, qy, qz, qw = (
                    value / norm for value in tool_orientation)
            else:
                qx, qy, qz, qw = self._approach_quaternion(
                    approach_direction)
            tolerance = float(
                self.get_parameter("orientation_tolerance_rad").value)
            oc = OrientationConstraint()
            oc.header.frame_id = frame
            oc.link_name = "harvest_tcp"
            oc.orientation.x = qx
            oc.orientation.y = qy
            oc.orientation.z = qz
            oc.orientation.w = qw
            oc.absolute_x_axis_tolerance = tolerance
            oc.absolute_y_axis_tolerance = tolerance
            oc.absolute_z_axis_tolerance = tolerance
            oc.weight = 1.0
            constraint.orientation_constraints = [oc]
        goal.request.goal_constraints = [constraint]
        if ((motion == "OMPL" or lock_joint_1)
                and self._joint_positions):
            path = Constraints()
            if ((motion == "OMPL" or lock_joint_1)
                    and "joint_1" in self._joint_positions):
                tolerance = max(
                    0.05,
                    float(self.get_parameter(
                        "approach_joint_1_tolerance_rad").value),
                )
                path.joint_constraints.append(JointConstraint(
                    joint_name="joint_1",
                    position=float(self._joint_positions["joint_1"]),
                    tolerance_above=tolerance,
                    tolerance_below=tolerance,
                    weight=1.0,
                ))
            if path.joint_constraints:
                goal.request.path_constraints = path
        if motion == "CIRC" and interim is not None:
            # harvester_moveit/grasp_proto.goal_circ와 동일한 Pilz 보조점 규약.
            auxiliary = Constraints()
            auxiliary.name = "interim"
            interim_constraint = PositionConstraint()
            interim_constraint.header.frame_id = frame
            interim_constraint.link_name = "harvest_tcp"
            interim_constraint.constraint_region = BoundingVolume(
                primitives=[SolidPrimitive(
                    type=SolidPrimitive.SPHERE, dimensions=[0.25])],
                primitive_poses=[Pose()],
            )
            interim_pose = (
                interim_constraint.constraint_region.primitive_poses[0])
            (interim_pose.position.x,
             interim_pose.position.y,
             interim_pose.position.z) = interim
            interim_pose.orientation.w = 1.0
            interim_constraint.weight = 1.0
            auxiliary.position_constraints = [interim_constraint]
            goal.request.path_constraints = auxiliary
        return goal

    @staticmethod
    def _approach_quaternion(
        direction: list[float],
    ) -> tuple[float, float, float, float]:
        """harvest_tcp +Z를 접근 방향에 맞추고 +Y를 가능한 한 base +Z로 세운다."""
        zn = math.sqrt(sum(value * value for value in direction))
        z = [value / zn for value in direction]
        hint = [0.0, 0.0, 1.0]
        if abs(sum(a * b for a, b in zip(z, hint))) > 0.95:
            hint = [0.0, 1.0, 0.0]
        dot = sum(a * b for a, b in zip(z, hint))
        y = [hint[i] - dot * z[i] for i in range(3)]
        yn = math.sqrt(sum(value * value for value in y))
        y = [value / yn for value in y]
        # x = y × z. 열벡터 [x y z]가 오른손 회전행렬이 된다.
        x = [
            y[1] * z[2] - y[2] * z[1],
            y[2] * z[0] - y[0] * z[2],
            y[0] * z[1] - y[1] * z[0],
        ]
        matrix = (
            (x[0], y[0], z[0]),
            (x[1], y[1], z[1]),
            (x[2], y[2], z[2]),
        )
        return MMMotionBridge._matrix_quaternion(matrix)

    @staticmethod
    def _matrix_quaternion(matrix) -> tuple[float, float, float, float]:
        """3×3 회전행렬을 geometry_msgs 순서(x,y,z,w) quaternion으로 변환한다."""
        m00, m01, m02 = matrix[0]
        m10, m11, m12 = matrix[1]
        m20, m21, m22 = matrix[2]
        trace = m00 + m11 + m22
        if trace > 0.0:
            s = math.sqrt(trace + 1.0) * 2.0
            return ((m21 - m12) / s, (m02 - m20) / s,
                    (m10 - m01) / s, 0.25 * s)
        if m00 > m11 and m00 > m22:
            s = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
            return (0.25 * s, (m01 + m10) / s,
                    (m02 + m20) / s, (m21 - m12) / s)
        if m11 > m22:
            s = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
            return ((m01 + m10) / s, 0.25 * s,
                    (m12 + m21) / s, (m02 - m20) / s)
        s = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
        return ((m02 + m20) / s, (m12 + m21) / s,
                0.25 * s, (m10 - m01) / s)

    def _joint_goal(
        self, positions: dict[str, float], motion: str = "PTP"
    ) -> MoveGroup.Goal:
        goal = self._base_goal()
        if motion == "OMPL":
            goal.request.pipeline_id = "ompl"
            goal.request.planner_id = "RRTConnectkConfigDefault"
        constraint = Constraints()
        positions = {
            name: self._nearest_joint_equivalent(name, value)
            for name, value in positions.items()
        }
        constraint.joint_constraints = [
            JointConstraint(
                joint_name=name,
                position=value,
                tolerance_above=0.01,
                tolerance_below=0.01,
                weight=1.0,
            )
            for name, value in positions.items()
        ]
        goal.request.goal_constraints = [constraint]
        return goal

    def _joint_state(self, message: JointState) -> None:
        self._joint_positions.update({
            name: float(position)
            for name, position in zip(message.name, message.position)
            if math.isfinite(position)
        })
        self._last_joint_state_receive_ns = self.get_clock().now().nanoseconds
        self._last_joint_state_stamp_ns = (
            int(message.header.stamp.sec) * 1_000_000_000
            + int(message.header.stamp.nanosec))

    def _joint_state_ready(self) -> bool:
        """MoveIt 실행 전에 6축의 유효하고 최신인 시뮬 시간 상태가 있는지 검사한다."""
        required = {f"joint_{index}" for index in range(1, 7)}
        if not required.issubset(self._joint_positions):
            return False
        if self._last_joint_state_stamp_ns <= 0:
            return False
        now = self.get_clock().now().nanoseconds
        max_age_ns = int(max(
            0.1, float(self.get_parameter(
                "joint_state_max_age_sec").value)) * 1e9)
        return (
            now > 0
            and now - self._last_joint_state_receive_ns <= max_age_ns
            and abs(now - self._last_joint_state_stamp_ns) <= max_age_ns
        )

    def _nearest_joint_equivalent(self, name: str, target: float) -> float:
        """±2π 관절은 현재값에서 가장 가까운 동치각을 사용해 한 바퀴 회전을 피한다."""
        if name not in {"joint_1", "joint_2", "joint_4", "joint_5", "joint_6"}:
            return target
        current = self._joint_positions.get(name)
        if current is None:
            return target
        candidates = [
            target + 2.0 * math.pi * k for k in range(-2, 3)
            if -2.0 * math.pi <= target + 2.0 * math.pi * k <= 2.0 * math.pi
        ]
        return min(candidates, key=lambda value: abs(value - current))

    def _queue(self, request_id: int, phase: str, goal: MoveGroup.Goal) -> None:
        # Nav 코디네이터는 홈 도달 전까지 같은 id를 주기적으로 재발행한다.
        # 동일 goal을 매번 취소하면 MoveIt이 영원히 홈에 도달하지 못하므로 무시한다.
        if self._last_succeeded == (request_id, phase):
            # 성공 직후 joint_states 반영 지연으로 같은 BED_VIEW/HOME 요청이 다시
            # 들어오더라도 이미 끝난 모션을 재계획하지 않고 성공만 재통지한다.
            self._active_id = request_id
            self._active_phase = phase
            self._publish_motion(True, phase)
            return
        if (self._queued is not None
                and self._queued[0] == request_id
                and self._queued[1] == phase):
            return
        if ((self._send_pending or self._goal_handle is not None)
                and self._active_id == request_id
                and self._active_phase == phase):
            return
        self._queued = (request_id, phase, goal)
        self._control_retry_count = 0
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()

    def _dispatch(self) -> None:
        if (self._queued is None or self._goal_handle is not None
                or self._send_pending):
            return
        if not self._move.server_is_ready():
            return
        if not self._joint_state_ready():
            self.get_logger().warning(
                "유효한 최신 joint_states 대기 중 — MoveIt 명령 보류",
                throttle_duration_sec=2.0)
            return
        request_id, phase, goal = self._queued
        self._queued = None
        self._active_id = request_id
        self._active_phase = phase
        self._active_goal_message = goal
        self._send_pending = True
        future = self._move.send_goal_async(goal)
        future.add_done_callback(self._goal_response)
        self.get_logger().info(f"MoveIt 명령 id={request_id} phase={phase}")

    def _goal_response(self, future) -> None:
        self._send_pending = False
        handle = future.result()
        if handle is None or not handle.accepted:
            self._publish_motion(False, "ERROR_PLAN_REJECTED")
            self._goal_handle = None
            return
        self._goal_handle = handle
        result = handle.get_result_async()
        result.add_done_callback(self._result)

    def _result(self, future) -> None:
        response = future.result()
        code = None
        if response is not None:
            code = int(response.result.error_code.val)
        if code == 1:
            self._last_succeeded = (self._active_id, self._active_phase)
        elif (code == -4
              and self._active_goal_message is not None
              and self._control_retry_count < int(self.get_parameter(
                  "control_failure_retries").value)):
            # MoveIt CONTROL_FAILED: Nav 도착 직후 JSB가 stamp=0인 초기 상태만 가진
            # 시작 순서 경쟁이면 최신 상태가 들어온 뒤 같은 명령을 다시 실행한다.
            self._control_retry_count += 1
            self._queued = (
                self._active_id, self._active_phase,
                self._active_goal_message)
            self._goal_handle = None
            self.get_logger().warning(
                f"MoveIt 제어 실패(code={code}) — 최신 joint_states 확인 후 "
                f"자동 재시도 {self._control_retry_count}/"
                f"{self.get_parameter('control_failure_retries').value}")
            return
        self._publish_motion(code == 1, self._active_phase, code)
        self._goal_handle = None

    def _publish_motion(
        self, reached: bool, phase: str, error_code: int | None = None
    ) -> None:
        payload = {
            "id": self._active_id,
            "phase": phase,
            "reached": bool(reached),
        }
        if error_code is not None:
            payload["error_code"] = error_code
        if not reached and not phase.startswith("ERROR_"):
            payload["phase"] = "ERROR_IK_PATH"
        self._status_pub.publish(String(
            data=json.dumps(payload, separators=(",", ":"))))


def main() -> None:
    rclpy.init()
    node = MMMotionBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
