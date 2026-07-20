#!/usr/bin/env python3
"""ForkliftB 창고 팔레트 자동 상·하차 상태기계.

작업 순서
---------
1. AMR 도킹 신호를 받으면 빈 ``Pallet_00``을 랙에서 꺼내 AMR에 적재한다.
2. AMR이 토마토를 채워 다시 도킹하면 팔레트를 같은 번호의 슬롯에 되돌린다.
3. 다음 빈 팔레트를 AMR에 적재하고 0~5번에 대해 반복한다.

이 노드는 판단과 순서만 담당하고 Isaac Sim의 ForkliftB에는
``/forklift_0/joint_command`` JointState 명령만 보낸다. ForkliftB의 월드 pose
토픽이 아직 없으므로 기본값은 명령 적분(dead reckoning)으로 위치를 추정한다.
나중에 ``/forklift_0/pose``가 연결되면 그 값을 자동으로 우선 사용한다.

대기 위치와 AMR 도킹 위치는 임시 파라미터다. 실제 배치가 정해지면
``wait_pose``와 ``amr_hole_center``만 바꾸면 된다.
"""

from __future__ import annotations

import math
import time
from collections import deque
from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState
from smartfarm_interfaces.msg import HandoffEvent
from std_msgs.msg import Bool, Int32, String


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_pose(msg: PoseStamped) -> float:
    q = msg.pose.orientation
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


@dataclass
class Step:
    """상태기계가 순서대로 실행하는 원자 동작."""

    kind: str
    label: str
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    lift: float = 0.0
    duration: float = 0.0
    max_drive: float = 0.0
    position_tolerance: float = 0.0
    yaw_tolerance: float = 0.0
    timeout: float = 60.0
    pallet_id: int = -1


class ForkLiftNode(Node):
    """랙 6개 팔레트를 순서대로 AMR과 교환하는 ForkliftB 제어 노드."""

    MODE_WAIT_INITIAL = "WAIT_INITIAL_AMR"
    MODE_BUSY = "BUSY"
    MODE_WAIT_RETURN = "WAIT_FILLED_AMR"
    MODE_COMPLETE = "COMPLETE"
    MODE_ERROR = "ERROR"

    PALLET_COUNT = 6
    # 제공받은 6개 팔레트 구멍의 평균 X와 월드 Z.
    RACK_CENTER_X = (-2.4, -2.4, -0.8, -0.8, 0.8, 0.8)
    RACK_HOLE_Z = (0.3903, 1.2903, 0.3903, 1.2903, 0.3903, 1.2903)

    def __init__(self):
        super().__init__("fork_lift_node")

        # 좌표 파라미터. 실제 도킹 위치가 정해지면 이 세 항목을 먼저 보정한다.
        self.declare_parameter("initial_pose", [0.0, 15.5, math.pi / 2.0])
        self.declare_parameter("wait_pose", [4.5, 15.0, math.pi / 2.0])
        # [팔레트 중심 X, 팔레트 중심 Y, 구멍 중심 월드 Z]
        self.declare_parameter("amr_hole_center", [2.0, 14.5, 0.45])
        self.declare_parameter("rack_front_y", 19.9989)
        self.declare_parameter("rack_heading", math.pi / 2.0)
        self.declare_parameter("amr_heading", math.pi / 2.0)

        # 포크/팔레트 기하. fork_tip_offset과 fork_center_z_at_zero는 GPU 실측 후 보정.
        self.declare_parameter("pallet_half_depth", 0.40115)
        self.declare_parameter("fork_tip_offset", 1.90)
        self.declare_parameter("fork_center_z_at_zero", 0.05)
        self.declare_parameter("insertion_depth", 0.65)
        self.declare_parameter("approach_clearance", 0.75)
        self.declare_parameter("staging_distance", 1.0)
        self.declare_parameter("pickup_raise", 0.06)

        # ForkliftB 운동 파라미터. main.py TransporterController와 같은 값이어야 한다.
        self.declare_parameter("wheel_radius", 0.22)
        self.declare_parameter("wheelbase", 2.05)
        self.declare_parameter("max_drive_speed", 3.0)       # wheel rad/s
        self.declare_parameter("creep_drive_speed", 0.8)     # wheel rad/s
        self.declare_parameter("max_steering_angle", math.radians(25.0))
        self.declare_parameter("control_rate", 20.0)
        self.declare_parameter("position_tolerance", 0.08)
        self.declare_parameter("insertion_tolerance", 0.025)
        self.declare_parameter("yaw_tolerance", math.radians(8.0))
        self.declare_parameter("lift_tolerance", 0.015)
        self.declare_parameter("step_timeout", 60.0)
        self.declare_parameter("connection_timeout", 1.0)

        # 안전 때문에 기본은 도킹 신호를 기다린다. 시험 시 true로 두면 연결 후 2초 뒤 시작.
        self.declare_parameter("auto_start", False)
        self.declare_parameter("auto_start_delay", 2.0)

        self._load_parameters()

        self._command_pub = self.create_publisher(
            JointState, "/forklift_0/joint_command", 10
        )
        self._status_pub = self.create_publisher(String, "/forklift/status", 10)
        self._clear_pub = self.create_publisher(Bool, "/forklift/clear", 10)
        self._complete_pub = self.create_publisher(
            Int32, "/forklift/task_complete", 10
        )

        self.create_subscription(
            JointState,
            "/forklift_0/joint_states",
            self._on_joint_states,
            10,
        )
        self.create_subscription(
            PoseStamped, "/forklift_0/pose", self._on_pose, 10
        )
        self.create_subscription(
            HandoffEvent, "/handoff/tray_ready", self._on_handoff, 10
        )
        # AMR 노드가 아직 없어도 Bool 한 번으로 도킹 이벤트를 시험할 수 있다.
        self.create_subscription(
            Bool, "/forklift/amr_docked", self._on_debug_docked, 10
        )
        self.create_subscription(
            Bool, "/forklift/reset_mission", self._on_reset, 10
        )

        self._x, self._y, self._yaw = self._initial_pose
        self._pose_feedback_time: float | None = None
        self._lift_feedback: float | None = None
        self._joint_state_time: float | None = None
        self._lift_target = 0.0
        self._drive_command = 0.0
        self._steer_command = 0.0

        self._mode = self.MODE_WAIT_INITIAL
        self._current_pallet = 0
        self._steps: deque[Step] = deque()
        self._step_started = time.monotonic()
        self._step_stable_since: float | None = None
        self._queue_result_mode = self.MODE_WAIT_RETURN
        self._auto_start_at: float | None = None
        self._last_tick = time.monotonic()
        self._last_status = ""

        period = 1.0 / self._control_rate
        self.create_timer(period, self._tick)
        self._publish_status(
            "초기화 완료: AMR 도킹 신호(/handoff/tray_ready 또는 "
            "/forklift/amr_docked)를 기다립니다"
        )

    def _load_parameters(self) -> None:
        def pose3(name: str) -> tuple[float, float, float]:
            values = [float(v) for v in self.get_parameter(name).value]
            if len(values) != 3:
                raise ValueError(f"{name}은 [x, y, yaw/z] 3개 값이어야 합니다")
            return values[0], values[1], values[2]

        self._initial_pose = pose3("initial_pose")
        self._wait_pose = pose3("wait_pose")
        self._amr_hole = pose3("amr_hole_center")
        self._rack_front_y = float(self.get_parameter("rack_front_y").value)
        self._rack_heading = float(self.get_parameter("rack_heading").value)
        self._amr_heading = float(self.get_parameter("amr_heading").value)
        self._pallet_half_depth = float(
            self.get_parameter("pallet_half_depth").value
        )
        self._fork_tip_offset = float(
            self.get_parameter("fork_tip_offset").value
        )
        self._fork_zero_z = float(
            self.get_parameter("fork_center_z_at_zero").value
        )
        self._insert_depth = float(self.get_parameter("insertion_depth").value)
        self._approach_clearance = float(
            self.get_parameter("approach_clearance").value
        )
        self._staging_distance = float(
            self.get_parameter("staging_distance").value
        )
        self._pickup_raise = float(self.get_parameter("pickup_raise").value)
        self._wheel_radius = float(self.get_parameter("wheel_radius").value)
        self._wheelbase = float(self.get_parameter("wheelbase").value)
        self._max_drive = float(self.get_parameter("max_drive_speed").value)
        self._creep_drive = float(
            self.get_parameter("creep_drive_speed").value
        )
        self._max_steer = float(
            self.get_parameter("max_steering_angle").value
        )
        self._control_rate = float(self.get_parameter("control_rate").value)
        self._position_tol = float(
            self.get_parameter("position_tolerance").value
        )
        self._insert_tol = float(
            self.get_parameter("insertion_tolerance").value
        )
        self._yaw_tol = float(self.get_parameter("yaw_tolerance").value)
        self._lift_tol = float(self.get_parameter("lift_tolerance").value)
        self._step_timeout = float(self.get_parameter("step_timeout").value)
        self._connection_timeout = float(
            self.get_parameter("connection_timeout").value
        )
        self._auto_start = bool(self.get_parameter("auto_start").value)
        self._auto_start_delay = float(
            self.get_parameter("auto_start_delay").value
        )

    # ------------------------------------------------------------------
    # ROS 입력

    def _on_joint_states(self, msg: JointState) -> None:
        self._joint_state_time = time.monotonic()
        try:
            index = msg.name.index("lift_joint")
            value = float(msg.position[index])
            if math.isfinite(value):
                self._lift_feedback = value
                if self._mode == self.MODE_WAIT_INITIAL and not self._steps:
                    self._lift_target = value
        except (ValueError, IndexError):
            pass

        if self._auto_start and self._auto_start_at is None:
            self._auto_start_at = time.monotonic() + self._auto_start_delay

    def _on_pose(self, msg: PoseStamped) -> None:
        self._x = float(msg.pose.position.x)
        self._y = float(msg.pose.position.y)
        self._yaw = yaw_from_pose(msg)
        self._pose_feedback_time = time.monotonic()

    def _on_handoff(self, msg: HandoffEvent) -> None:
        self.get_logger().info(
            f"AMR 도킹 이벤트: tray_id={msg.tray_id}, amr_id={msg.amr_id}"
        )
        self._handle_amr_docked()

    def _on_debug_docked(self, msg: Bool) -> None:
        if msg.data:
            self.get_logger().info("시험용 AMR 도킹 이벤트 수신")
            self._handle_amr_docked()

    def _on_reset(self, msg: Bool) -> None:
        if not msg.data:
            return
        self._stop()
        self._steps.clear()
        self._mode = self.MODE_WAIT_INITIAL
        self._current_pallet = 0
        self._auto_start_at = None
        self._publish_clear(False)
        self._publish_status("미션 리셋: Pallet_00부터 다시 시작")

    # ------------------------------------------------------------------
    # 미션 구성

    def _handle_amr_docked(self) -> None:
        if self._joint_state_time is None:
            self.get_logger().warning(
                "ForkliftB joint_states 연결 전이라 도킹 이벤트를 무시합니다"
            )
            return
        if self._mode == self.MODE_WAIT_INITIAL:
            self._start_initial_load()
        elif self._mode == self.MODE_WAIT_RETURN:
            self._start_return_cycle()
        elif self._mode == self.MODE_COMPLETE:
            self.get_logger().info("이미 Pallet_00~05 작업이 완료됐습니다")
        else:
            self.get_logger().warning(
                f"현재 상태 {self._mode}에서는 새 도킹 이벤트를 무시합니다"
            )

    def _start_initial_load(self) -> None:
        pallet = 0
        self._publish_clear(False)
        steps = self._take_from_rack(pallet)
        steps += self._place_on_amr(pallet)
        steps += [self._event("loaded_on_amr", pallet)]
        steps += self._move_wait_steps()
        steps += [self._event("forklift_clear", pallet)]
        self._start_queue(
            steps,
            result_mode=self.MODE_WAIT_RETURN,
            status="Pallet_00 빈 팔레트 AMR 상차 시작",
        )

    def _start_return_cycle(self) -> None:
        completed = self._current_pallet
        next_pallet = completed + 1
        self._publish_clear(False)

        steps = self._take_from_amr(completed)
        steps += self._place_in_rack(completed)
        steps += [self._event("task_complete", completed)]

        if next_pallet < self.PALLET_COUNT:
            steps += self._take_from_rack(next_pallet)
            steps += self._place_on_amr(next_pallet)
            steps += [self._event("loaded_on_amr", next_pallet)]
            result_mode = self.MODE_WAIT_RETURN
        else:
            result_mode = self.MODE_COMPLETE

        steps += self._move_wait_steps()
        steps += [self._event("forklift_clear", next_pallet)]
        self._current_pallet = min(next_pallet, self.PALLET_COUNT - 1)
        self._start_queue(
            steps,
            result_mode=result_mode,
            status=f"Pallet_{completed:02d} 회수 및 창고 적재 시작",
        )

    def _start_queue(
        self, steps: list[Step], result_mode: str, status: str
    ) -> None:
        self._steps = deque(steps)
        self._queue_result_mode = result_mode
        self._mode = self.MODE_BUSY
        self._begin_step()
        self._publish_status(status)

    def _take_from_rack(self, pallet: int) -> list[Step]:
        x = self.RACK_CENTER_X[pallet]
        pre_y, insert_y, stage_y = self._approach_y(self._rack_front_y)
        lift = self._rack_lift_target(pallet)
        return [
            self._move(x, stage_y, self._rack_heading, f"rack {pallet} staging"),
            self._move(x, pre_y, self._rack_heading, f"rack {pallet} pre-pick"),
            self._lift(lift, f"rack {pallet} hole height"),
            self._move(
                x,
                insert_y,
                self._rack_heading,
                f"rack {pallet} fork insert",
                creep=True,
                precise=True,
            ),
            self._lift(lift + self._pickup_raise, f"rack {pallet} pallet raise"),
            self._wait(0.5, f"rack {pallet} load settle"),
            self._move(
                x,
                pre_y,
                self._rack_heading,
                f"rack {pallet} retract",
                creep=True,
                precise=True,
            ),
        ]

    def _place_in_rack(self, pallet: int) -> list[Step]:
        x = self.RACK_CENTER_X[pallet]
        pre_y, insert_y, stage_y = self._approach_y(self._rack_front_y)
        lift = self._rack_lift_target(pallet)
        return [
            self._move(x, stage_y, self._rack_heading, f"rack {pallet} staging loaded"),
            self._move(x, pre_y, self._rack_heading, f"rack {pallet} pre-place"),
            self._lift(lift + self._pickup_raise, f"rack {pallet} carry height"),
            self._move(
                x,
                insert_y,
                self._rack_heading,
                f"rack {pallet} loaded insert",
                creep=True,
                precise=True,
            ),
            self._lift(lift, f"rack {pallet} pallet lower"),
            self._wait(0.5, f"rack {pallet} unload settle"),
            self._move(
                x,
                pre_y,
                self._rack_heading,
                f"rack {pallet} empty fork retract",
                creep=True,
                precise=True,
            ),
        ]

    def _take_from_amr(self, pallet: int) -> list[Step]:
        x, center_y, _ = self._amr_hole
        front_y = center_y - self._pallet_half_depth
        pre_y, insert_y, stage_y = self._approach_y(front_y)
        lift = self._amr_lift_target()
        return [
            self._move(x, stage_y, self._amr_heading, "AMR staging"),
            self._move(x, pre_y, self._amr_heading, "AMR pre-pick"),
            self._lift(lift, "AMR pallet hole height"),
            self._move(
                x,
                insert_y,
                self._amr_heading,
                "AMR fork insert",
                creep=True,
                precise=True,
            ),
            self._lift(lift + self._pickup_raise, f"Pallet_{pallet:02d} AMR lift"),
            self._wait(0.5, "AMR load settle"),
            self._move(
                x,
                pre_y,
                self._amr_heading,
                "AMR loaded retract",
                creep=True,
                precise=True,
            ),
        ]

    def _place_on_amr(self, pallet: int) -> list[Step]:
        x, center_y, _ = self._amr_hole
        front_y = center_y - self._pallet_half_depth
        pre_y, insert_y, stage_y = self._approach_y(front_y)
        lift = self._amr_lift_target()
        return [
            self._move(x, stage_y, self._amr_heading, "AMR staging loaded"),
            self._move(x, pre_y, self._amr_heading, "AMR pre-place"),
            self._lift(lift + self._pickup_raise, "AMR carry height"),
            self._move(
                x,
                insert_y,
                self._amr_heading,
                "AMR loaded insert",
                creep=True,
                precise=True,
            ),
            self._lift(lift, f"Pallet_{pallet:02d} AMR lower"),
            self._wait(0.5, "AMR unload settle"),
            self._move(
                x,
                pre_y,
                self._amr_heading,
                "AMR empty fork retract",
                creep=True,
                precise=True,
            ),
        ]

    def _move_wait_steps(self) -> list[Step]:
        x, y, yaw = self._wait_pose
        return [self._move(x, y, yaw, "return wait pose")]

    def _approach_y(self, pallet_front_y: float) -> tuple[float, float, float]:
        # 지게차 로컬 +X가 월드 +Y를 향한다고 가정한다.
        pre_y = pallet_front_y - self._fork_tip_offset - self._approach_clearance
        insert_y = pallet_front_y + self._insert_depth - self._fork_tip_offset
        stage_y = pre_y - self._staging_distance
        return pre_y, insert_y, stage_y

    def _rack_lift_target(self, pallet: int) -> float:
        return clamp(self.RACK_HOLE_Z[pallet] - self._fork_zero_z, 0.0, 2.0)

    def _amr_lift_target(self) -> float:
        return clamp(self._amr_hole[2] - self._fork_zero_z, 0.0, 2.0)

    # ------------------------------------------------------------------
    # Step 생성 헬퍼

    def _move(
        self,
        x: float,
        y: float,
        yaw: float,
        label: str,
        *,
        creep: bool = False,
        precise: bool = False,
    ) -> Step:
        return Step(
            kind="move",
            label=label,
            x=x,
            y=y,
            yaw=yaw,
            max_drive=self._creep_drive if creep else self._max_drive,
            position_tolerance=self._insert_tol if precise else self._position_tol,
            yaw_tolerance=self._yaw_tol,
            timeout=self._step_timeout,
        )

    def _lift(self, target: float, label: str) -> Step:
        return Step(
            kind="lift",
            label=label,
            lift=clamp(target, 0.0, 2.0),
            timeout=self._step_timeout,
        )

    @staticmethod
    def _wait(duration: float, label: str) -> Step:
        return Step(kind="wait", label=label, duration=duration, timeout=duration + 5.0)

    @staticmethod
    def _event(label: str, pallet: int) -> Step:
        return Step(kind="event", label=label, pallet_id=pallet, timeout=5.0)

    # ------------------------------------------------------------------
    # 실행 루프

    def _tick(self) -> None:
        now = time.monotonic()
        dt = clamp(now - self._last_tick, 0.0, 0.2)
        self._last_tick = now
        self._integrate_dead_reckoning(dt)

        if (
            self._auto_start_at is not None
            and now >= self._auto_start_at
            and self._mode == self.MODE_WAIT_INITIAL
        ):
            self._auto_start_at = None
            self.get_logger().warning("auto_start=true: AMR이 도킹됐다고 가정하고 시작")
            self._handle_amr_docked()

        if self._mode != self.MODE_BUSY or not self._steps:
            self._publish_command(0.0, 0.0)
            return

        if (
            self._joint_state_time is None
            or now - self._joint_state_time > self._connection_timeout
        ):
            self._fail("/forklift_0/joint_states 연결이 끊겼습니다")
            return

        step = self._steps[0]
        elapsed = now - self._step_started
        if elapsed > step.timeout:
            self._fail(f"단계 시간 초과: {step.label} ({elapsed:.1f}s)")
            return

        if step.kind == "move":
            done = self._run_move(step, now)
        elif step.kind == "lift":
            done = self._run_lift(step, now, elapsed)
        elif step.kind == "wait":
            self._publish_command(0.0, 0.0)
            done = elapsed >= step.duration
        elif step.kind == "event":
            self._run_event(step)
            done = True
        else:
            self._fail(f"알 수 없는 단계 종류: {step.kind}")
            return

        if done:
            self._steps.popleft()
            if self._steps:
                self._begin_step()
            else:
                self._finish_queue()

    def _run_move(self, step: Step, now: float) -> bool:
        dx = step.x - self._x
        dy = step.y - self._y
        distance = math.hypot(dx, dy)
        yaw_error = wrap_angle(step.yaw - self._yaw)

        if distance <= step.position_tolerance and abs(yaw_error) <= step.yaw_tolerance:
            self._publish_command(0.0, 0.0)
            if self._step_stable_since is None:
                self._step_stable_since = now
            return now - self._step_stable_since >= 0.25
        self._step_stable_since = None

        path_heading = math.atan2(dy, dx)
        forward_error = wrap_angle(path_heading - self._yaw)
        reverse_error = wrap_angle(path_heading - (self._yaw + math.pi))
        if abs(reverse_error) < abs(forward_error):
            direction = -1.0
            course_error = reverse_error
        else:
            direction = 1.0
            course_error = forward_error

        # 목표 가까이서는 최종 yaw도 조향에 반영한다. Ackermann이라 제자리 회전은 안 한다.
        heading_term = 1.6 * course_error
        if distance < 0.8:
            heading_term += 0.45 * yaw_error
        linear = direction * min(
            step.max_drive * self._wheel_radius,
            max(0.08, 0.9 * distance),
        )
        desired_yaw_rate = clamp(heading_term, -0.9, 0.9)
        steering = math.atan(
            self._wheelbase
            * desired_yaw_rate
            / (linear if abs(linear) > 1e-4 else direction * 1e-4)
        )
        steering = clamp(steering, -self._max_steer, self._max_steer)
        drive = clamp(linear / self._wheel_radius, -step.max_drive, step.max_drive)
        self._publish_command(drive, steering)
        return False

    def _run_lift(self, step: Step, now: float, elapsed: float) -> bool:
        self._lift_target = step.lift
        self._publish_command(0.0, 0.0)
        if self._lift_feedback is not None:
            reached = abs(self._lift_feedback - step.lift) <= self._lift_tol
            if reached:
                if self._step_stable_since is None:
                    self._step_stable_since = now
                return now - self._step_stable_since >= 0.3
            self._step_stable_since = None
            return False

        # 상태 토픽에 lift position이 없는 환경에서만 시간 기반으로 진행한다.
        if elapsed >= 2.0:
            self.get_logger().warning(
                f"lift_joint 피드백 없음: {step.label}을 시간 기준으로 통과"
            )
            return True
        return False

    def _run_event(self, step: Step) -> None:
        if step.label == "task_complete":
            msg = Int32()
            msg.data = step.pallet_id
            self._complete_pub.publish(msg)
            self._publish_status(
                f"Pallet_{step.pallet_id:02d} 토마토 적재 팔레트 창고 복귀 완료"
            )
        elif step.label == "loaded_on_amr":
            self._publish_status(
                f"Pallet_{step.pallet_id:02d} 빈 팔레트 AMR 상차 완료"
            )
        elif step.label == "forklift_clear":
            self._publish_clear(True)

    def _begin_step(self) -> None:
        self._step_started = time.monotonic()
        self._step_stable_since = None
        if self._steps:
            self.get_logger().info(f"[STEP] {self._steps[0].label}")

    def _finish_queue(self) -> None:
        self._stop()
        self._mode = self._queue_result_mode
        if self._mode == self.MODE_WAIT_RETURN:
            self._publish_status(
                f"Pallet_{self._current_pallet:02d} AMR 작업·복귀 대기"
            )
        elif self._mode == self.MODE_COMPLETE:
            self._publish_status("Pallet_00~05 전체 작업 완료, 대기 위치 유지")

    def _fail(self, reason: str) -> None:
        self._stop()
        self._steps.clear()
        self._mode = self.MODE_ERROR
        self._publish_clear(False)
        self._publish_status("ERROR: " + reason)
        self.get_logger().error(reason)

    # ------------------------------------------------------------------
    # 저수준 명령·추정

    def _publish_command(self, drive: float, steering: float) -> None:
        self._drive_command = float(drive)
        self._steer_command = float(steering)
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ["lift_joint", "back_wheel_swivel", "back_wheel_drive"]
        msg.position = [self._lift_target, steering, math.nan]
        msg.velocity = [math.nan, math.nan, drive]
        self._command_pub.publish(msg)

    def _stop(self) -> None:
        self._publish_command(0.0, 0.0)

    def _integrate_dead_reckoning(self, dt: float) -> None:
        if self._pose_feedback_time is not None:
            if time.monotonic() - self._pose_feedback_time <= 0.5:
                return
        linear = self._drive_command * self._wheel_radius
        self._yaw = wrap_angle(
            self._yaw
            + linear / self._wheelbase * math.tan(self._steer_command) * dt
        )
        self._x += linear * math.cos(self._yaw) * dt
        self._y += linear * math.sin(self._yaw) * dt

    def _publish_status(self, text: str) -> None:
        if text == self._last_status:
            return
        self._last_status = text
        msg = String()
        msg.data = text
        self._status_pub.publish(msg)
        self.get_logger().info(text)

    def _publish_clear(self, clear: bool) -> None:
        msg = Bool()
        msg.data = clear
        self._clear_pub.publish(msg)

    def destroy_node(self):
        for _ in range(3):
            self._stop()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ForkLiftNode()
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
