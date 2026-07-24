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
                             PositionConstraint)
from rclpy.action import ActionClient
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import String


HOME_Q = {
    "joint_1": 0.0,
    "joint_2": 0.0,
    "joint_3": math.radians(60.0),
    "joint_4": 0.0,
    "joint_5": math.radians(75.0),
    "joint_6": math.radians(-90.0),
}


class MMMotionBridge(Node):
    def __init__(self) -> None:
        super().__init__("mm_motion_bridge")
        self.declare_parameter("group_name", "mm_manipulator")
        self.declare_parameter("planning_frame", "base_link")
        self.declare_parameter("position_tolerance_m", 0.01)
        self.declare_parameter("planning_time_sec", 8.0)
        self.declare_parameter("velocity_scale", 0.25)

        self._move = ActionClient(self, MoveGroup, "move_action")
        self._status_pub = self.create_publisher(String, "pipeline_status", 20)
        self.create_subscription(String, "status", self._forward_isaac_status, 20)
        self.create_subscription(String, "cmd", self._command, 20)
        self._goal_handle = None
        self._active_id = 0
        self._active_phase = ""
        self._queued: tuple[int, str, MoveGroup.Goal] | None = None
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
            except (KeyError, TypeError, ValueError):
                self.get_logger().warning("잘못된 rmp_target 형식")
                return
            frame = str(target.get(
                "frame_id", self.get_parameter("planning_frame").value))
            self._queue(request_id, phase, self._pose_goal(position, frame))
            return
        home = command.get("rmp_home")
        if isinstance(home, dict):
            request_id = int(home.get("id", 0))
            self._queue(request_id, "GO_HOME", self._joint_goal(HOME_Q))
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
        request.max_acceleration_scaling_factor = 0.25
        request.pipeline_id = "ompl"
        return goal

    def _pose_goal(self, position: list[float], frame: str) -> MoveGroup.Goal:
        goal = self._base_goal()
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
        goal.request.goal_constraints = [constraint]
        return goal

    def _joint_goal(self, positions: dict[str, float]) -> MoveGroup.Goal:
        goal = self._base_goal()
        constraint = Constraints()
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

    def _queue(self, request_id: int, phase: str, goal: MoveGroup.Goal) -> None:
        self._queued = (request_id, phase, goal)
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()

    def _dispatch(self) -> None:
        if self._queued is None or self._goal_handle is not None:
            return
        if not self._move.server_is_ready():
            return
        request_id, phase, goal = self._queued
        self._queued = None
        self._active_id = request_id
        self._active_phase = phase
        future = self._move.send_goal_async(goal)
        future.add_done_callback(self._goal_response)
        self.get_logger().info(f"MoveIt 명령 id={request_id} phase={phase}")

    def _goal_response(self, future) -> None:
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
