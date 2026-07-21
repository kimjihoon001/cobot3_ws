#!/usr/bin/env python3
"""ForkliftB 창고 팔레트 자동 상·하차 상태기계.

작업 순서
---------
1. AMR 도킹 신호를 받으면 빈 ``Pallet_00``을 랙에서 꺼내 IW에 적재한다.
2. 다음 도킹 신호를 받으면 IW의 ``Pallet_00``을 다시 집어
   원래 0번 위치에 내려놓는다.
3. 이어서 빈 ``Pallet_01``을 랙에서 꺼내 IW에 적재하고
   대기 위치로 복귀한다.

이 노드는 판단과 순서만 담당하고 Isaac Sim의 ForkliftB에는
``/forklift_0/joint_command`` JointState 명령만 보낸다. ForkliftB의 월드 pose
Warehouse 단독 시험 기본값은 명령 적분 자세를 사용하며 상태 토픽을 기다리지
않고 2초 후 시작한다. 통합 운용에서는 pose/joint-state 피드백 검사를 파라미터로
켤 수 있다.

대기 위치와 AMR 도킹 위치는 임시 파라미터다. 실제 배치가 정해지면
``wait_pose``와 ``amr_hole_center``만 바꾸면 된다.
"""

from __future__ import annotations

import fcntl
import math
import os
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
    drive: float = 0.0
    steering: float = 0.0
    max_steering: float = 0.0
    max_rotation: float = 0.0
    attached: bool = False


class ForkLiftNode(Node):
    """랙 6개 팔레트를 순서대로 AMR과 교환하는 ForkliftB 제어 노드."""

    MODE_WAIT_INITIAL = "WAIT_INITIAL_AMR"
    MODE_BUSY = "BUSY"
    MODE_WAIT_RETURN = "WAIT_FILLED_AMR"
    MODE_HOLDING_PALLET = "HOLDING_PALLET_00"
    MODE_PALLET_01_ON_IW = "PALLET_01_ON_IW"
    MODE_COMPLETE = "COMPLETE"
    MODE_ERROR = "ERROR"

    INSTANCE_LOCK_PATH = "/tmp/warehouse_dock_fork_lift_node.lock"

    PALLET_COUNT = 6
    # Isaac 5.1 원본 pallet.usd를 GPU에서 메시 꼭짓점으로 실측한 값이다.
    # 팔레트 피벗은 바닥이고, 구멍은 하부 데크 윗면~상부 데크 아랫면이다.
    PALLET_LOCAL_HOLE_BOTTOM_Z = 0.02053
    PALLET_LOCAL_HOLE_TOP_Z = 0.11605
    PALLET_LOCAL_HOLE_CENTER_Z = (
        PALLET_LOCAL_HOLE_BOTTOM_Z + PALLET_LOCAL_HOLE_TOP_Z
    ) / 2.0
    PALLET_LOCAL_HOLE_X = (-0.258375, 0.258375)

    # 현재 Warehouse 씬에서 Pallet_00~05의 월드 피벗(바닥) 좌표.
    RACK_CENTER_X = (-2.4, -2.4, -0.8, -0.8, 0.8, 0.8)
    RACK_CENTER_Y = 20.4
    RACK_PALLET_BASE_Z = (0.322, 1.222, 0.322, 1.222, 0.322, 1.222)
    RACK_HOLE_Z = (
        RACK_PALLET_BASE_Z[0] + PALLET_LOCAL_HOLE_CENTER_Z,
        RACK_PALLET_BASE_Z[1] + PALLET_LOCAL_HOLE_CENTER_Z,
        RACK_PALLET_BASE_Z[2] + PALLET_LOCAL_HOLE_CENTER_Z,
        RACK_PALLET_BASE_Z[3] + PALLET_LOCAL_HOLE_CENTER_Z,
        RACK_PALLET_BASE_Z[4] + PALLET_LOCAL_HOLE_CENTER_Z,
        RACK_PALLET_BASE_Z[5] + PALLET_LOCAL_HOLE_CENTER_Z,
    )

    # ForkliftB 원본 lift 메시의 실제 삽입 날 범위(lift_joint=0).
    FORK_BLADE_BOTTOM_Z_AT_ZERO = 0.179587
    FORK_BLADE_TOP_Z_AT_ZERO = 0.232846
    FORK_BLADE_CENTER_Z_AT_ZERO = (
        FORK_BLADE_BOTTOM_Z_AT_ZERO + FORK_BLADE_TOP_Z_AT_ZERO
    ) / 2.0

    def __init__(self):
        # 같은 PC에서 노드 두 개가 /forklift_0/joint_command에 동시에 명령을
        # 보내는 상황을 막는다. 프로세스가 비정상 종료돼도 flock은 자동 해제된다.
        self._instance_lock = open(self.INSTANCE_LOCK_PATH, "a+", encoding="utf-8")
        try:
            fcntl.flock(
                self._instance_lock.fileno(),
                fcntl.LOCK_EX | fcntl.LOCK_NB,
            )
        except BlockingIOError as exc:
            self._instance_lock.close()
            raise RuntimeError(
                "fork_lift_node가 이미 실행 중입니다. 기존 ROS 노드를 먼저 종료하세요"
            ) from exc
        self._instance_lock.seek(0)
        self._instance_lock.truncate()
        self._instance_lock.write(str(os.getpid()))
        self._instance_lock.flush()

        super().__init__("fork_lift_node")

        # 좌표 파라미터. 실제 도킹 위치가 정해지면 이 세 항목을 먼저 보정한다.
        self.declare_parameter("initial_pose", [0.0, 14.5, -math.pi / 2.0])
        # 창고 입구 중앙 대기점. 여기서 월드 -Y로 2m 이동하면 AMR 삽입 pose다.
        self.declare_parameter("wait_pose", [0.0, 14.5, -math.pi / 2.0])
        # [팔레트 중심 X, 팔레트 중심 Y, 구멍 중심 월드 Z]
        self.declare_parameter("amr_hole_center", [0.0, 10.84885, 0.45])
        self.declare_parameter("rack_front_y", 19.99885)
        self.declare_parameter("rack_heading", math.pi / 2.0)
        self.declare_parameter("amr_heading", -math.pi / 2.0)

        # 포크/팔레트 기하. 포크 중심은 원본 GPU 메시의 삽입 날 상·하단 실측값.
        self.declare_parameter("pallet_half_depth", 0.40115)
        self.declare_parameter("fork_tip_offset", 1.90)
        self.declare_parameter(
            "fork_center_z_at_zero", self.FORK_BLADE_CENTER_Z_AT_ZERO
        )
        self.declare_parameter("insertion_depth", 0.65)
        # 팔레트 정면 대기점에서 조향 0°로 이 거리만큼 전진하면 포크 삽입점이다.
        self.declare_parameter("rack_fork_insert_travel", 0.95)
        self.declare_parameter("staging_distance", 1.0)
        # 포크를 구멍에 끝까지 삽입해 팔레트를 연결한 뒤 20cm 들어 올린다.
        self.declare_parameter("pickup_raise", 0.20)

        # ForkliftB 운동 파라미터. main.py TransporterController와 같은 값이어야 한다.
        self.declare_parameter("wheel_radius", 0.22)
        self.declare_parameter("wheelbase", 2.05)
        self.declare_parameter("max_drive_speed", 3.0)       # wheel rad/s
        self.declare_parameter("creep_drive_speed", 0.8)     # wheel rad/s
        # 대기점에서 이만큼 창고 안쪽으로 먼저 후진한 뒤 U턴한다. 입구 경계
        # Y=13에서 회전 궤적과 차체가 충분히 떨어지게 하는 값이다.
        self.declare_parameter("u_turn_clearance", 1.5)
        # U턴 완료 후 입구 쪽으로 다시 후진해 뒷벽/랙과 안전거리를 만든다.
        self.declare_parameter("post_u_turn_reverse_distance", 0.8)
        # Pallet_00 정면 접근은 두 번째 U턴이 되지 않도록 조향을 별도로 제한한다.
        self.declare_parameter(
            "pallet_approach_max_steering_angle", math.radians(35.0)
        )
        # spike 06처럼 후륜 swivel을 크게 꺾어 좁게 회전한다. 25°에서는 반경이
        # 약 4.4m지만 70°에서는 약 0.75m다.
        self.declare_parameter("max_steering_angle", math.radians(70.0))
        self.declare_parameter("control_rate", 20.0)
        self.declare_parameter("position_tolerance", 0.08)
        self.declare_parameter("insertion_tolerance", 0.025)
        self.declare_parameter("yaw_tolerance", math.radians(8.0))
        self.declare_parameter("lift_tolerance", 0.015)
        self.declare_parameter("step_timeout", 60.0)
        self.declare_parameter("u_turn_timeout", 60.0)
        self.declare_parameter("connection_timeout", 1.0)
        # 재시험 때 Isaac을 초기화하지 않은 상태에서 ROS 노드만 다시 켜면 경로
        # 전제가 깨진다. 시작 pose가 대기점에서 벗어나면 움직이지 않고 정지한다.
        self.declare_parameter("initial_position_tolerance", 0.20)
        self.declare_parameter(
            "initial_yaw_tolerance", math.radians(12.0)
        )

        # Warehouse 단독 시험은 joint-state 연결 후 자동 시작한다. 실제 통합 운용에서는
        # -p auto_start:=false 로 두고 /handoff/tray_ready 도킹 이벤트를 사용한다.
        self.declare_parameter("auto_start", True)
        self.declare_parameter("auto_start_delay", 2.0)
        # GUI 차체가 실제로 움직인 사실을 확인한 뒤에만 상태기를 진행한다. 피드백 없이
        # dead reckoning만 쓰면 DDS 도메인이 달라도 ROS 로그상 작업이 끝난 것처럼 보인다.
        self.declare_parameter("require_joint_state_feedback", True)
        self.declare_parameter("require_pose_feedback", True)
        self.declare_parameter("use_pose_feedback", True)

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
        self._pose_feedback_logged = False
        self._lift_feedback: float | None = None
        self._joint_state_time: float | None = None
        self._lift_target = 0.0
        self._drive_command = 0.0
        self._steer_command = 0.0
        self._pallet_attached_command = False
        self._pallet_target_command = 0

        self._mode = self.MODE_WAIT_INITIAL
        self._current_pallet = 0
        self._steps: deque[Step] = deque()
        self._step_started = time.monotonic()
        self._step_stable_since: float | None = None
        self._step_direction: float | None = None
        self._arc_last_yaw: float | None = None
        self._arc_progress = 0.0
        self._arc_report_bucket = -1
        self._queue_result_mode = self.MODE_WAIT_RETURN
        self._auto_start_at: float | None = (
            time.monotonic() + self._auto_start_delay
            if self._auto_start else None
        )
        self._last_tick = time.monotonic()
        self._last_status = ""

        period = 1.0 / self._control_rate
        self.create_timer(period, self._tick)
        self._log_rack_geometry()
        if self._auto_start:
            self._publish_status(
                f"초기화 완료: {self._auto_start_delay:.1f}초 후 Pallet_00 자동작업 시작"
            )
        else:
            self._publish_status(
                "초기화 완료: AMR 도킹 신호(/handoff/tray_ready 또는 "
                "/forklift/amr_docked)를 기다립니다"
            )

    def _log_rack_geometry(self) -> None:
        """현재 씬의 팔레트/구멍 좌표와 계산된 리프트 목표를 한 번 기록한다."""
        rows = []
        for pallet in range(self.PALLET_COUNT):
            center_x = self.RACK_CENTER_X[pallet]
            hole_x = tuple(center_x + x for x in self.PALLET_LOCAL_HOLE_X)
            rows.append(
                f"P{pallet}: pallet=({center_x:.3f},{self.RACK_CENTER_Y:.3f},"
                f"{self.RACK_PALLET_BASE_Z[pallet]:.3f}), "
                f"holes_x=({hole_x[0]:.3f},{hole_x[1]:.3f}), "
                f"hole_z={self.RACK_HOLE_Z[pallet]:.5f}, "
                f"lift={self._rack_lift_target(pallet):.5f}"
            )
        self.get_logger().info(
            "[GPU measured pallet/fork geometry]\n  " + "\n  ".join(rows)
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
        self._rack_fork_insert_travel = float(
            self.get_parameter("rack_fork_insert_travel").value
        )
        if self._rack_fork_insert_travel <= 0.0:
            raise ValueError("rack_fork_insert_travel은 0보다 커야 합니다")
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
        self._u_turn_clearance = float(
            self.get_parameter("u_turn_clearance").value
        )
        self._post_u_turn_reverse = float(
            self.get_parameter("post_u_turn_reverse_distance").value
        )
        self._pallet_approach_max_steer = float(
            self.get_parameter("pallet_approach_max_steering_angle").value
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
        self._u_turn_timeout = float(
            self.get_parameter("u_turn_timeout").value
        )
        self._connection_timeout = float(
            self.get_parameter("connection_timeout").value
        )
        self._initial_position_tol = float(
            self.get_parameter("initial_position_tolerance").value
        )
        self._initial_yaw_tol = float(
            self.get_parameter("initial_yaw_tolerance").value
        )
        self._auto_start = bool(self.get_parameter("auto_start").value)
        self._auto_start_delay = float(
            self.get_parameter("auto_start_delay").value
        )
        self._require_pose_feedback = bool(
            self.get_parameter("require_pose_feedback").value
        )
        self._require_joint_state_feedback = bool(
            self.get_parameter("require_joint_state_feedback").value
        )
        self._use_pose_feedback = bool(
            self.get_parameter("use_pose_feedback").value
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
        if self._use_pose_feedback:
            self._x = float(msg.pose.position.x)
            self._y = float(msg.pose.position.y)
            self._yaw = yaw_from_pose(msg)
        self._pose_feedback_time = time.monotonic()
        if not self._pose_feedback_logged:
            mode = "경로 제어 사용" if self._use_pose_feedback else "연결 확인 전용"
            self.get_logger().info(f"/forklift_0/pose 연결 완료 ({mode})")
            self._pose_feedback_logged = True

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
        self._pallet_attached_command = False
        self._stop()
        self._steps.clear()
        self._mode = self.MODE_WAIT_INITIAL
        self._current_pallet = 0
        self._auto_start_at = (
            time.monotonic() + self._auto_start_delay
            if self._auto_start else None
        )
        self._publish_clear(False)
        self._publish_status("미션 리셋: Pallet_00부터 다시 시작")

    # ------------------------------------------------------------------
    # 미션 구성

    def _handle_amr_docked(self) -> None:
        if self._require_joint_state_feedback and self._joint_state_time is None:
            self.get_logger().warning(
                "ForkliftB joint_states 연결 전이라 도킹 이벤트를 무시합니다"
            )
            return
        if self._require_pose_feedback and self._pose_feedback_time is None:
            self.get_logger().warning(
                "ForkliftB 실제 자세(/forklift_0/pose) 연결 전이라 시작을 기다립니다"
            )
            return
        if self._mode == self.MODE_WAIT_INITIAL:
            dx = self._x - self._wait_pose[0]
            dy = self._y - self._wait_pose[1]
            position_error = math.hypot(dx, dy)
            yaw_error = abs(wrap_angle(self._yaw - self._wait_pose[2]))
            if (
                position_error > self._initial_position_tol
                or yaw_error > self._initial_yaw_tol
            ):
                self._fail(
                    "초기 pose가 대기 위치가 아닙니다: "
                    f"actual=({self._x:.3f},{self._y:.3f},"
                    f"{math.degrees(self._yaw):.1f}deg), "
                    f"expected=({self._wait_pose[0]:.3f},"
                    f"{self._wait_pose[1]:.3f},"
                    f"{math.degrees(self._wait_pose[2]):.1f}deg). "
                    "Isaac Sim을 초기화한 뒤 다시 실행하세요"
                )
                return
            self._start_initial_load()
        elif self._mode == self.MODE_WAIT_RETURN:
            dx = self._x - self._wait_pose[0]
            dy = self._y - self._wait_pose[1]
            position_error = math.hypot(dx, dy)
            yaw_error = abs(wrap_angle(self._yaw - self._wait_pose[2]))
            if (
                position_error > self._initial_position_tol
                or yaw_error > self._initial_yaw_tol
            ):
                self._fail(
                    "IW 회수 시작 pose가 대기 위치가 아닙니다: "
                    f"position_error={position_error:.3f}m, "
                    f"yaw_error={math.degrees(yaw_error):.1f}deg"
                )
                return
            self._start_return_cycle()
        elif self._mode == self.MODE_PALLET_01_ON_IW:
            self.get_logger().info(
                "Pallet_00 복귀·Pallet_01 IW 상차 시험이 이미 완료됐습니다"
            )
        elif self._mode == self.MODE_COMPLETE:
            self.get_logger().info("이미 Pallet_00~05 작업이 완료됐습니다")
        elif self._mode == self.MODE_HOLDING_PALLET:
            self.get_logger().info("Pallet_00을 포크에 연결해 들어 올린 상태입니다")
        else:
            self.get_logger().warning(
                f"현재 상태 {self._mode}에서는 새 도킹 이벤트를 무시합니다"
            )

    def _start_initial_load(self) -> None:
        pallet = 0
        self._publish_clear(False)
        pre_y, insert_y, _ = self._approach_y(self._rack_front_y)
        # Pallet_00 시험에서는 GPU 기하 중심값보다 포크를 6cm 낮춰 삽입한다.
        lift = clamp(self._rack_lift_target(pallet) - 0.06, 0.0, 2.0)
        # 대기 위치에서 먼저 0번 팔레트 구멍 높이를 맞춘 뒤 움직인다.
        # 랙 바로 앞에서 포크를 올리면 선반 밑면과 충돌하므로 접근 중에는
        # 이 높이를 그대로 유지한다.
        steps = [self._lift(lift, "rack 0 hole height at wait pose")]
        steps += self._turn_from_wait_to_rack()
        # U턴 뒤 안전 후진까지 끝난 다음에만 제한 조향 접근을 시작한다.
        # 접근 단계는 최대 회전량도 제한해 두 번째 U턴으로 이어질 수 없다.
        steps += [
            self._approach_pallet(
                self.RACK_CENTER_X[pallet],
                pre_y,
                self._rack_heading,
                "steer and approach Pallet_00 front",
            )
        ]
        # Pallet_00을 실제 포크에 연결해 20cm 들어 올린다.
        steps += [
            self._wait(0.4, "Pallet_00 straight alignment settle"),
            self._straight_y(
                insert_y,
                +self._creep_drive,
                f"rack 0 fork insert straight "
                f"{self._rack_fork_insert_travel:.2f}m",
                expected_x=self.RACK_CENTER_X[pallet],
                expected_yaw=self._rack_heading,
                precise=True,
            ),
            self._wait(0.3, "Pallet_00 insertion settle"),
            self._coupler(True, pallet, "connect Pallet_00 to fork carriage"),
            self._wait(0.4, "Pallet_00 coupler settle"),
        ]
        # 연결 후 총 20cm 상승은 유지하되 2cm씩 나눠 급격히 튀지 않게 올린다.
        steps += [
            self._lift(
                lift + self._pickup_raise * index / 10.0,
                f"rack 0 pallet slow raise {index * 2}cm",
            )
            for index in range(1, 11)
        ]
        steps += [
            self._wait(0.8, "Pallet_00 lifted hold"),
        ]
        # 팔레트 연결과 20cm 상승을 유지한 채 랙에서 충분히 후진한 다음,
        # 기존 반경 1.2m 궤적으로 U턴을 정확히 한 번만 수행한다.
        steps += self._turn_from_rack_to_wait()
        steps += self._move_wait_steps()
        # 여기까지가 이미 검증된 픽업·복귀 경로다. 대기점에서 포크가 IW(-Y)를
        # 향한 상태를 그대로 이용해 조향 없이 상차하고 다시 대기점으로 후진한다.
        steps += self._place_initial_pallet_on_amr(
            pallet,
            carry_lift=lift + self._pickup_raise,
        )
        steps += [
            self._event("loaded_on_amr", pallet),
            self._event("forklift_clear", pallet),
        ]
        self._start_queue(
            steps,
            result_mode=self.MODE_WAIT_RETURN,
            status="Pallet_00 픽업·단일 U턴·IW 상차 자동작업 시작",
        )

    def _start_return_cycle(self) -> None:
        pallet = 0
        next_pallet = 1
        self._publish_clear(False)

        # IW의 Pallet_00을 원래 위치에 되돌린 뒤 대기점으로 복귀하지
        # 않고, 같은 X축의 상단 Pallet_01을 이어서 집어 비어 있는 IW에
        # 상차한다.
        steps = self._take_initial_pallet_from_amr(pallet)
        steps += self._return_initial_pallet_to_rack(
            pallet,
            return_to_wait=False,
        )
        steps += [
            self._event("task_complete", pallet),
        ]
        steps += self._take_next_pallet_to_amr(next_pallet)
        steps += [
            self._event("loaded_on_amr", next_pallet),
            self._event("forklift_clear", next_pallet),
        ]
        self._current_pallet = next_pallet
        self._start_queue(
            steps,
            result_mode=self.MODE_PALLET_01_ON_IW,
            status="Pallet_00 복귀·Pallet_01 IW 상차 연속작업 시작",
        )

    def _start_queue(
        self, steps: list[Step], result_mode: str, status: str
    ) -> None:
        self._steps = deque(steps)
        self._queue_result_mode = result_mode
        self._mode = self.MODE_BUSY
        self._begin_step()
        self._publish_status(status)

    def _take_from_rack(
        self, pallet: int, *, start_at_pre: bool = False
    ) -> list[Step]:
        x = self.RACK_CENTER_X[pallet]
        pre_y, insert_y, stage_y = self._approach_y(self._rack_front_y)
        lift = self._rack_lift_target(pallet)
        steps = [] if start_at_pre else [
            self._move(x, stage_y, self._rack_heading, f"rack {pallet} staging"),
            self._move(x, pre_y, self._rack_heading, f"rack {pallet} pre-pick"),
        ]
        return steps + [
            self._lift(lift, f"rack {pallet} hole height"),
            # 정면 대기점은 삽입점에서 정확히 rack_fork_insert_travel만큼
            # 떨어져 있다. 조향을 0°로 고정해 포크 구멍까지 직진한다.
            self._straight_y(
                insert_y,
                +self._creep_drive,
                f"rack {pallet} fork insert straight "
                f"{self._rack_fork_insert_travel:.2f}m",
                expected_x=x,
                expected_yaw=self._rack_heading,
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
        return self._turn_from_wait_to_rack() + [
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

    def _turn_from_wait_to_rack(self) -> list[Step]:
        """대기점 후진 → 단 한 번 U턴 → 조향 0° 안전 후진."""
        steer, _, turn_x = self._rack_u_turn_geometry()
        drive = min(1.5, self._max_drive)
        wait_x, wait_y, wait_yaw = self._wait_pose
        turn_y = wait_y + self._u_turn_clearance
        return [
            self._move(
                wait_x,
                turn_y,
                wait_yaw,
                "reverse from wait before U-turn",
                creep=True,
                precise=True,
            ),
            # -Y를 보던 차가 우측으로 반 바퀴 돌아 +Y를 보게 된다. 회전 종료
            # X는 wait_x - 2R라서 왼쪽의 Pallet_00 접근선과도 가깝다.
            self._arc(
                +drive,
                steer,
                self._rack_heading,
                "right U-turn toward Pallet_00",
            ),
            # U턴 종료 즉시 조향을 풀고 입구 쪽으로 물러난다. 다음 단계가
            # 조향 접근을 시작하기 전에 랙/뒷벽과 0.8m의 추가 여유를 만든다.
            self._straight_y(
                turn_y - self._post_u_turn_reverse,
                -self._creep_drive,
                "reverse straight after the single U-turn",
                expected_x=turn_x,
                expected_yaw=self._rack_heading,
                precise=True,
            ),
        ]

    def _turn_from_rack_to_wait(self) -> list[Step]:
        """팔레트를 든 채 후진한 뒤 입구 중앙축으로 좁은 우측 U턴."""
        steer, _, turn_x = self._rack_u_turn_geometry()
        drive = min(1.5, self._max_drive)
        wait_x, wait_y, _ = self._wait_pose
        turn_y = wait_y + self._u_turn_clearance
        return [
            # +Y를 보는 상태에서 이 점으로 가는 최단 방향은 후진이다. U턴을
            # 시작하기 전에 랙과 포크를 충분히 분리한다.
            self._move(
                turn_x,
                turn_y,
                self._rack_heading,
                "reverse loaded pallet to U-turn staging",
                creep=True,
            ),
            # 시작 X=wait_x-2R에서 우측 반원을 그리면 종료 X=wait_x가 되어
            # 상하차 위치와 같은 입구 중앙축으로 바로 정렬된다.
            self._arc(
                +drive,
                steer,
                self._amr_heading,
                "right U-turn toward AMR",
            ),
        ]

    def _rack_u_turn_geometry(self) -> tuple[float, float, float]:
        """U턴 종료 X를 Pallet_00 접근축에 맞춘 조향각·반경·시작 X."""
        wait_x = self._wait_pose[0]
        rack_x = self.RACK_CENTER_X[0]
        lateral = rack_x - wait_x
        radius = max(abs(lateral) / 2.0, 0.10)
        steer_magnitude = min(
            self._max_steer, math.atan(self._wheelbase / radius)
        )
        radius = self._wheelbase / math.tan(steer_magnitude)
        steer = math.copysign(steer_magnitude, lateral)
        turn_x = wait_x + math.copysign(2.0 * radius, lateral)
        return steer, radius, turn_x

    def _take_initial_pallet_from_amr(self, pallet: int) -> list[Step]:
        """IW의 Pallet_00을 연결해 20cm 들고 대기점까지 직선 후진한다."""
        _, center_y, _ = self._amr_hole
        insert_y = self._amr_insert_y(center_y)
        wait_x, wait_y, wait_yaw = self._wait_pose
        amr_lift = self._amr_lift_target()
        amr_carry_lift = amr_lift + self._pickup_raise

        # 첫 상차 때 사용한 0번 랙 삽입·운반 높이를 그대로 재사용한다.
        rack_place_lift = clamp(
            self._rack_lift_target(pallet) - 0.06,
            0.0,
            2.0,
        )
        rack_carry_lift = rack_place_lift + self._pickup_raise

        slow_raise = [
            self._lift(
                amr_lift + self._pickup_raise * index / 10.0,
                f"Pallet_{pallet:02d} slow raise from IW {index * 2}cm",
            )
            for index in range(1, 11)
        ]

        adjust_distance = abs(amr_carry_lift - rack_carry_lift)
        adjust_steps = max(1, math.ceil(adjust_distance / 0.02))
        adjust_for_rack = [
            self._lift(
                amr_carry_lift
                + (rack_carry_lift - amr_carry_lift) * index / adjust_steps,
                f"Pallet_{pallet:02d} adjust carry height for rack "
                f"{index}/{adjust_steps}",
            )
            for index in range(1, adjust_steps + 1)
        ]

        return [
            self._lift(amr_lift, "IW Pallet_00 hole height at wait pose"),
            self._straight_y(
                insert_y,
                +self._creep_drive,
                "IW Pallet_00 fork insert straight 2m",
                expected_x=wait_x,
                expected_yaw=self._amr_heading,
                precise=True,
            ),
            self._wait(0.4, "IW Pallet_00 insertion settle"),
            self._coupler(True, pallet, "connect IW Pallet_00 to fork carriage"),
            self._wait(0.4, "IW Pallet_00 coupler settle"),
        ] + slow_raise + [
            self._wait(0.8, "IW Pallet_00 lifted hold"),
            self._straight_y(
                wait_y,
                -self._creep_drive,
                "IW loaded pallet reverse 2m to wait pose",
                expected_x=wait_x,
                expected_yaw=self._amr_heading,
                precise=True,
            ),
            self._pose_check(
                wait_x,
                wait_y,
                wait_yaw,
                "IW loaded return wait pose check",
            ),
        ] + adjust_for_rack + [
            self._wait(0.5, "Pallet_00 rack carry height settle"),
        ]

    def _return_initial_pallet_to_rack(
        self,
        pallet: int,
        *,
        return_to_wait: bool = True,
    ) -> list[Step]:
        """Pallet_00을 기존 검증 경로로 0번 랙에 놓고 포크를 뺀다."""
        rack_x = self.RACK_CENTER_X[pallet]
        pre_y, insert_y, _ = self._approach_y(self._rack_front_y)
        rack_place_lift = clamp(
            self._rack_lift_target(pallet) - 0.06,
            0.0,
            2.0,
        )
        rack_carry_lift = rack_place_lift + self._pickup_raise

        slow_lower = [
            self._lift(
                rack_carry_lift - self._pickup_raise * index / 10.0,
                f"Pallet_{pallet:02d} slow lower into rack {index * 2}cm",
            )
            for index in range(1, 11)
        ]

        steps = self._turn_from_wait_to_rack()
        steps += [
            self._approach_pallet(
                rack_x,
                pre_y,
                self._rack_heading,
                "steer and approach Pallet_00 return front",
            ),
            self._wait(0.4, "Pallet_00 return straight alignment settle"),
            self._straight_y(
                insert_y,
                +self._creep_drive,
                "rack 0 loaded pallet insert straight 0.95m",
                expected_x=rack_x,
                expected_yaw=self._rack_heading,
                precise=True,
            ),
            self._wait(0.4, "Pallet_00 rack placement settle"),
        ]
        steps += slow_lower
        steps += [
            self._wait(0.8, "Pallet_00 supported in rack 0"),
            self._coupler(False, pallet, "release Pallet_00 in rack 0"),
            self._wait(0.8, "Pallet_00 rack coupler release settle"),
            self._straight_y(
                pre_y,
                -self._creep_drive,
                "rack 0 empty fork retract straight 0.95m",
                expected_x=rack_x,
                expected_yaw=self._rack_heading,
                precise=True,
            ),
        ]
        if return_to_wait:
            steps += self._turn_from_rack_to_wait()
            steps += self._move_wait_steps()
            steps += [
                self._pose_check(
                    self._wait_pose[0],
                    self._wait_pose[1],
                    self._wait_pose[2],
                    "Pallet_00 rack return final wait pose check",
                )
            ]
        return steps

    def _take_next_pallet_to_amr(self, pallet: int) -> list[Step]:
        """0번 배치 후 같은 X축의 Pallet_01을 집어 IW에 상차한다."""
        rack_x = self.RACK_CENTER_X[pallet]
        pre_y, insert_y, stage_y = self._approach_y(self._rack_front_y)
        rack_lift = clamp(
            self._rack_lift_target(pallet) - 0.06,
            0.0,
            2.0,
        )
        rack_carry_lift = rack_lift + self._pickup_raise
        amr_carry_lift = self._amr_lift_target() + self._pickup_raise

        slow_raise = [
            self._lift(
                rack_lift + self._pickup_raise * index / 10.0,
                f"rack {pallet} pallet slow raise {index * 2}cm",
            )
            for index in range(1, 11)
        ]
        lower_distance = abs(rack_carry_lift - amr_carry_lift)
        lower_steps = max(1, math.ceil(lower_distance / 0.02))
        lower_for_transport = [
            self._lift(
                rack_carry_lift
                + (amr_carry_lift - rack_carry_lift) * index / lower_steps,
                f"Pallet_{pallet:02d} lower to IW carry height "
                f"{index}/{lower_steps}",
            )
            for index in range(1, lower_steps + 1)
        ]

        # 0번에서 포크를 뺀 직후에는 pre-pick 위치에 있다. 상단
        # 높이로 올리기 전 1m 더 후진해 선반 앞 충돌 여유를 확보한다.
        steps = [
            self._straight_y(
                stage_y,
                -self._creep_drive,
                "rack 0 clear reverse before Pallet_01 lift",
                expected_x=rack_x,
                expected_yaw=self._rack_heading,
                precise=True,
            ),
            self._lift(rack_lift, "rack 1 hole height at safe staging"),
            self._wait(0.5, "rack 1 lift height settle at safe staging"),
            self._straight_y(
                pre_y,
                +self._creep_drive,
                "rack 1 straight approach from safe staging",
                expected_x=rack_x,
                expected_yaw=self._rack_heading,
                precise=True,
            ),
            self._wait(0.4, "Pallet_01 straight alignment settle"),
            self._straight_y(
                insert_y,
                +self._creep_drive,
                f"rack 1 fork insert straight "
                f"{self._rack_fork_insert_travel:.2f}m",
                expected_x=rack_x,
                expected_yaw=self._rack_heading,
                precise=True,
            ),
            self._wait(0.3, "Pallet_01 insertion settle"),
            self._coupler(True, pallet, "connect Pallet_01 to fork carriage"),
            self._wait(0.4, "Pallet_01 coupler settle"),
        ]
        steps += slow_raise
        steps += [
            self._wait(0.8, "Pallet_01 lifted hold"),
            self._straight_y(
                pre_y,
                -self._creep_drive,
                "rack 1 loaded retract straight 0.95m",
                expected_x=rack_x,
                expected_yaw=self._rack_heading,
                precise=True,
            ),
            self._straight_y(
                stage_y,
                -self._creep_drive,
                "rack 1 loaded reverse to safe lowering point",
                expected_x=rack_x,
                expected_yaw=self._rack_heading,
                precise=True,
            ),
        ]
        # 팔레트가 랙에서 완전히 빠진 후에만 낮춰 회전 안정성을 높인다.
        steps += lower_for_transport
        steps += [self._wait(0.5, "Pallet_01 IW carry height settle")]
        steps += self._turn_from_rack_to_wait()
        steps += self._move_wait_steps()
        steps += self._place_initial_pallet_on_amr(
            pallet,
            carry_lift=amr_carry_lift,
        )
        return steps

    def _take_from_amr(self, pallet: int) -> list[Step]:
        x, center_y, _ = self._amr_hole
        insert_y = self._amr_insert_y(center_y)
        wait_x, wait_y, wait_yaw = self._wait_pose
        lift = self._amr_lift_target()
        return [
            self._move(wait_x, wait_y, wait_yaw, "AMR straight approach start"),
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
                wait_x,
                wait_y,
                wait_yaw,
                "AMR loaded retract 2m",
                creep=True,
                precise=True,
            ),
        ]

    def _place_on_amr(self, pallet: int) -> list[Step]:
        x, center_y, _ = self._amr_hole
        insert_y = self._amr_insert_y(center_y)
        wait_x, wait_y, wait_yaw = self._wait_pose
        lift = self._amr_lift_target()
        return self._turn_from_rack_to_wait() + [
            self._move(wait_x, wait_y, wait_yaw, "AMR straight approach start loaded"),
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
                wait_x,
                wait_y,
                wait_yaw,
                "AMR empty fork retract 2m",
                creep=True,
                precise=True,
            ),
        ]

    def _place_initial_pallet_on_amr(
        self, pallet: int, *, carry_lift: float
    ) -> list[Step]:
        """대기점에 복귀한 Pallet_00을 IW에 천천히 내려놓고 복귀한다.

        대기점과 IW 삽입점은 같은 X축이고 지게차도 이미 ``amr_heading``으로
        정렬돼 있다. 일반 경로 추종 대신 조향 0° 직선 이동만 사용해 IW 앞에서
        다시 회전하거나 팔레트가 옆으로 밀리는 것을 막는다.
        """
        _, center_y, _ = self._amr_hole
        insert_y = self._amr_insert_y(center_y)
        wait_x, wait_y, wait_yaw = self._wait_pose
        place_lift = self._amr_lift_target()

        # 랙 운반 높이에서 IW 구멍 중심 높이까지 약 2cm씩 내린다. 현재 기본값은
        # 0.32407m -> 0.24378m라서 정확히 5단계로 천천히 내려간다.
        lower_distance = max(0.0, carry_lift - place_lift)
        lower_steps = max(1, math.ceil(lower_distance / 0.02))
        slow_lower = [
            self._lift(
                carry_lift
                + (place_lift - carry_lift) * index / lower_steps,
                f"Pallet_{pallet:02d} slow lower on IW "
                f"{index}/{lower_steps}",
            )
            for index in range(1, lower_steps + 1)
        ]

        return [
            self._straight_y(
                insert_y,
                +self._creep_drive,
                "IW loaded straight approach 2m",
                expected_x=wait_x,
                expected_yaw=self._amr_heading,
                precise=True,
            ),
            self._wait(0.5, "IW loaded placement settle"),
        ] + slow_lower + [
            self._wait(0.8, f"Pallet_{pallet:02d} supported on IW"),
            self._coupler(
                False,
                pallet,
                f"release Pallet_{pallet:02d} on IW",
            ),
            self._wait(
                0.8,
                f"Pallet_{pallet:02d} coupler release settle",
            ),
            self._straight_y(
                wait_y,
                -self._creep_drive,
                "IW empty fork reverse to wait pose",
                expected_x=wait_x,
                expected_yaw=self._amr_heading,
                precise=True,
            ),
            # 마지막에 좌표 보정 주행을 다시 시작하면 작은 오차 때문에 원을 그릴
            # 수 있다. 직선 인출 결과가 대기 pose인지 정지 상태에서만 검증한다.
            self._pose_check(
                wait_x,
                wait_y,
                wait_yaw,
                "IW wait pose final alignment check",
            ),
        ]

    def _move_wait_steps(self) -> list[Step]:
        x, y, yaw = self._wait_pose
        return [self._move(x, y, yaw, "return wait pose")]

    def _approach_y(self, pallet_front_y: float) -> tuple[float, float, float]:
        # 지게차 로컬 +X가 월드 +Y를 향한다고 가정한다. 먼저 포크 끝이
        # insertion_depth만큼 들어간 베이스 위치를 구한 뒤, 그곳에서 정확히
        # rack_fork_insert_travel만큼 떨어진 지점을 팔레트 정면 대기점으로 삼는다.
        insert_y = pallet_front_y + self._insert_depth - self._fork_tip_offset
        pre_y = insert_y - self._rack_fork_insert_travel
        stage_y = pre_y - self._staging_distance
        return pre_y, insert_y, stage_y

    def _amr_insert_y(self, center_y: float) -> float:
        """AMR 접근 방향(+Y/-Y)에 맞는 지게차 베이스 삽입 Y를 계산한다."""
        direction_y = 1.0 if math.sin(self._amr_heading) >= 0.0 else -1.0
        return center_y + direction_y * (
            self._insert_depth - self._fork_tip_offset - self._pallet_half_depth
        )

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

    def _arc(
        self,
        drive: float,
        steering: float,
        target_yaw: float,
        label: str,
    ) -> Step:
        yaw_rate = abs(
            drive
            * self._wheel_radius
            / self._wheelbase
            * math.tan(steering)
        )
        # 180°에 필요한 이론 시간에 2초 여유를 둔 watchdog이다. 이 시간이
        # 지났다고 U턴 성공으로 처리하지 않고, 실제 pose가 180°에 도달하지
        # 못했으면 안전 실패시킨다.
        hard_stop = (
            math.pi / yaw_rate + 2.0 if yaw_rate > 1e-6 else 1.0
        )
        return Step(
            kind="arc",
            label=label,
            yaw=wrap_angle(target_yaw),
            drive=drive,
            steering=steering,
            duration=min(hard_stop, self._u_turn_timeout),
            yaw_tolerance=math.radians(3.0),
            timeout=self._u_turn_timeout,
        )

    def _approach_pallet(
        self,
        x: float,
        y: float,
        yaw: float,
        label: str,
    ) -> Step:
        """두 번째 U턴 없이 제한 조향으로 Pallet_00 정면에 접근한다."""
        return Step(
            kind="approach",
            label=label,
            x=x,
            y=y,
            yaw=yaw,
            max_drive=min(1.2, self._max_drive),
            max_steering=self._pallet_approach_max_steer,
            max_rotation=math.radians(45.0),
            position_tolerance=self._insert_tol,
            yaw_tolerance=self._yaw_tol,
            timeout=min(30.0, self._step_timeout),
        )

    def _straight_y(
        self,
        target_y: float,
        drive: float,
        label: str,
        *,
        expected_x: float,
        expected_yaw: float,
        precise: bool = False,
    ) -> Step:
        """조향 없이 현재 차체 방향으로 주행해 지정 월드 Y에 도달한다."""
        return Step(
            kind="straight_y",
            label=label,
            x=expected_x,
            y=target_y,
            yaw=wrap_angle(expected_yaw),
            drive=drive,
            position_tolerance=(
                self._insert_tol if precise else self._position_tol
            ),
            yaw_tolerance=self._yaw_tol,
            timeout=self._step_timeout,
        )

    def _pose_check(self, x: float, y: float, yaw: float, label: str) -> Step:
        """추가 주행 없이 현재 pose가 목표 허용오차 안인지 확인한다."""
        return Step(
            kind="pose_check",
            label=label,
            x=x,
            y=y,
            yaw=wrap_angle(yaw),
            position_tolerance=max(self._position_tol, 0.10),
            yaw_tolerance=max(self._yaw_tol, math.radians(10.0)),
            timeout=2.0,
        )

    @staticmethod
    def _coupler(attached: bool, pallet: int, label: str) -> Step:
        """Isaac의 포크-팔레트 물리 커플러 상태를 명령한다."""
        return Step(
            kind="coupler",
            label=label,
            attached=attached,
            pallet_id=pallet,
            timeout=5.0,
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
            # Isaac 브리지가 아직 발견되지 않았으면 자동 시작 시각을 유지하고 다음
            # tick에서 다시 확인한다. 연결 전에 시각을 지우면 영원히 시작하지 않는다.
            if (
                self._require_joint_state_feedback
                and self._joint_state_time is None
            ) or (
                self._require_pose_feedback
                and self._pose_feedback_time is None
            ):
                self._publish_command(0.0, 0.0)
                return
            self._auto_start_at = None
            self.get_logger().warning("auto_start=true: AMR이 도킹됐다고 가정하고 시작")
            self._handle_amr_docked()

        if self._mode != self.MODE_BUSY or not self._steps:
            self._publish_command(0.0, 0.0)
            return

        if self._require_joint_state_feedback and (
            self._joint_state_time is None
            or now - self._joint_state_time > self._connection_timeout
        ):
            self._fail("/forklift_0/joint_states 연결이 끊겼습니다")
            return
        if self._require_pose_feedback and (
            self._pose_feedback_time is None
            or now - self._pose_feedback_time > self._connection_timeout
        ):
            self._fail("/forklift_0/pose 연결이 끊겼습니다")
            return

        step = self._steps[0]
        elapsed = now - self._step_started
        if elapsed > step.timeout:
            self._fail(f"단계 시간 초과: {step.label} ({elapsed:.1f}s)")
            return

        if step.kind == "move":
            done = self._run_move(step, now)
        elif step.kind == "arc":
            done = self._run_arc(step, now)
        elif step.kind == "straight_y":
            done = self._run_straight_y(step, now)
        elif step.kind == "approach":
            done = self._run_pallet_approach(step)
        elif step.kind == "pose_check":
            done = self._run_pose_check(step, now)
        elif step.kind == "lift":
            done = self._run_lift(step, now, elapsed)
        elif step.kind == "coupler":
            self._pallet_target_command = step.pallet_id
            self._pallet_attached_command = step.attached
            self._publish_command(0.0, 0.0)
            done = elapsed >= 0.25
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
        # 한 이동 단계 안에서는 전진/후진을 처음 선택한 방향으로 고정한다.
        # 목표선 경계에서 매 틱 방향이 바뀌면 drive가 +/-로 진동해 차가 멈춘다.
        if self._step_direction is None:
            self._step_direction = (
                -1.0 if abs(reverse_error) < abs(forward_error) else 1.0
            )
        direction = self._step_direction
        course_error = reverse_error if direction < 0.0 else forward_error

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

    def _run_arc(self, step: Step, now: float) -> bool:
        if self._arc_last_yaw is None:
            self._arc_last_yaw = self._yaw
        else:
            delta = abs(wrap_angle(self._yaw - self._arc_last_yaw))
            # 한 제어 틱에 30°를 넘는 값은 pose 초기화/점프로 보고 누적하지 않는다.
            if delta <= math.radians(30.0):
                self._arc_progress += delta
            self._arc_last_yaw = self._yaw

        bucket = int(math.degrees(self._arc_progress) // 30.0)
        if bucket > self._arc_report_bucket:
            self._arc_report_bucket = bucket
            self.get_logger().info(
                f"[U-TURN] actual rotation={math.degrees(self._arc_progress):.1f}deg"
            )

        yaw_error = abs(wrap_angle(step.yaw - self._yaw))
        turned_half_circle = (
            self._arc_progress >= math.pi - step.yaw_tolerance
            and yaw_error <= max(step.yaw_tolerance, math.radians(5.0))
        )
        elapsed = now - self._step_started
        hard_stop = step.duration > 0.0 and elapsed >= step.duration
        if turned_half_circle:
            self._publish_command(0.0, 0.0)
            self.get_logger().info(
                f"[U-TURN] complete once at "
                f"{math.degrees(self._arc_progress):.1f}deg (actual pose)"
            )
            return True
        if self._arc_progress > math.pi + math.radians(10.0):
            self._publish_command(0.0, 0.0)
            self._fail(
                "U턴이 목표 180°를 10° 이상 초과해 안전 정지했습니다: "
                f"rotation={math.degrees(self._arc_progress):.1f}deg, "
                f"yaw_error={math.degrees(yaw_error):.1f}deg"
            )
            return False
        if hard_stop:
            self._publish_command(0.0, 0.0)
            self._fail(
                "U턴 watchdog 시간 내 실제 180° pose에 도달하지 못했습니다: "
                f"rotation={math.degrees(self._arc_progress):.1f}deg, "
                f"yaw_error={math.degrees(yaw_error):.1f}deg"
            )
            return False
        self._publish_command(step.drive, step.steering)
        return False

    def _run_pallet_approach(self, step: Step) -> bool:
        """U턴 후 안전 후진점에서 Pallet_00 정면까지만 제한 조향 전진."""
        if self._arc_last_yaw is None:
            self._arc_last_yaw = self._yaw
        else:
            delta = abs(wrap_angle(self._yaw - self._arc_last_yaw))
            if delta <= math.radians(30.0):
                self._arc_progress += delta
            self._arc_last_yaw = self._yaw
        if self._arc_progress >= step.max_rotation:
            self._publish_command(0.0, 0.0)
            self._fail(
                "Pallet_00 접근 조향이 45°를 초과해 두 번째 회전을 안전 정지했습니다"
            )
            return False

        dx = step.x - self._x
        dy = step.y - self._y
        distance = math.hypot(dx, dy)

        # 목표 Y에 도달하면 횡오차가 조금 남아도 즉시 정지한다. 목표를 지나
        # 다시 돌아오는 원 궤적이 벽 충돌의 원인이므로 재회전은 금지한다.
        if self._y >= step.y - step.position_tolerance or distance <= step.position_tolerance:
            self._publish_command(0.0, 0.0)
            self.get_logger().info(
                f"[APPROACH] stop at ({self._x:.3f}, {self._y:.3f}), "
                f"lateral error={dx:.3f}m"
            )
            return True

        path_heading = math.atan2(dy, dx)
        heading_error = wrap_angle(path_heading - self._yaw)
        # 정상 궤적은 U턴 종료 후 거의 +Y 정렬이므로 작은 S자 보정만 필요하다.
        # ±35° 제한으로 이 단계가 또 하나의 U턴이 되는 것을 막는다.
        steering = clamp(
            1.4 * heading_error,
            -step.max_steering,
            step.max_steering,
        )
        drive = min(
            step.max_drive,
            max(self._creep_drive, distance / self._wheel_radius * 0.35),
        )
        self._publish_command(drive, steering)
        return False

    def _run_straight_y(self, step: Step, now: float) -> bool:
        remaining = step.y - self._y
        lateral_error = abs(step.x - self._x)
        yaw_error = abs(wrap_angle(step.yaw - self._yaw))
        lateral_limit = max(0.20, 4.0 * step.position_tolerance)
        yaw_limit = max(math.radians(12.0), 1.5 * step.yaw_tolerance)
        if lateral_error > lateral_limit or yaw_error > yaw_limit:
            self._publish_command(0.0, 0.0)
            self._fail(
                f"직선 단계 정렬 이탈: {step.label}, "
                f"x_error={lateral_error:.3f}m, "
                f"yaw_error={math.degrees(yaw_error):.1f}deg"
            )
            return False
        # +Y를 향한 랙과 -Y를 향한 IW에서 같은 직선 단계를 사용하므로 현재
        # heading까지 포함한 실제 월드 Y 진행 방향으로 도착 여부를 판정한다.
        y_velocity_sign = math.sin(self._yaw) * step.drive
        if abs(y_velocity_sign) < 1e-4:
            self._publish_command(0.0, 0.0)
            self._fail(f"직선 단계 진행 방향이 Y축이 아닙니다: {step.label}")
            return False
        reached = (
            remaining <= step.position_tolerance
            if y_velocity_sign > 0.0
            else remaining >= -step.position_tolerance
        )
        if reached:
            self._publish_command(0.0, 0.0)
            if self._step_stable_since is None:
                self._step_stable_since = now
            return now - self._step_stable_since >= 0.25
        self._step_stable_since = None
        # 이 단계에서는 좌표 추종 조향을 하지 않는다. U턴으로 맞춘
        # Pallet_00 접근축을 그대로 따라가므로 다시 원을 그릴 수 없다.
        self._publish_command(step.drive, 0.0)
        return False

    def _run_pose_check(self, step: Step, now: float) -> bool:
        position_error = math.hypot(step.x - self._x, step.y - self._y)
        yaw_error = abs(wrap_angle(step.yaw - self._yaw))
        self._publish_command(0.0, 0.0)
        if (
            position_error > step.position_tolerance
            or yaw_error > step.yaw_tolerance
        ):
            self._fail(
                f"최종 pose 검증 실패: {step.label}, "
                f"position_error={position_error:.3f}m, "
                f"yaw_error={math.degrees(yaw_error):.1f}deg"
            )
            return False
        if self._step_stable_since is None:
            self._step_stable_since = now
        return now - self._step_stable_since >= 0.25

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
        elif step.label == "pallet_lifted":
            self._publish_status(
                f"Pallet_{step.pallet_id:02d} 포크 연결 및 "
                f"{self._pickup_raise:.2f}m 리프트 후 대기 위치 복귀 완료"
            )

    def _begin_step(self) -> None:
        self._step_started = time.monotonic()
        self._step_stable_since = None
        self._step_direction = None
        self._arc_last_yaw = (
            self._yaw
            if self._steps and self._steps[0].kind in ("arc", "approach")
            else None
        )
        self._arc_progress = 0.0
        self._arc_report_bucket = -1
        if self._steps:
            step = self._steps[0]
            if step.kind == "move":
                self.get_logger().info(
                    f"[STEP] {step.label} -> "
                    f"pose=({step.x:.3f}, {step.y:.3f}, {step.yaw:.3f})"
                )
            elif step.kind == "lift":
                self.get_logger().info(
                    f"[STEP] {step.label} -> lift={step.lift:.3f}m"
                )
            elif step.kind == "arc":
                self.get_logger().info(
                    f"[STEP] {step.label} -> yaw={math.degrees(step.yaw):.1f}deg, "
                    f"drive={step.drive:.2f}, "
                    f"steer={math.degrees(step.steering):.1f}deg, "
                    f"hard_stop={step.duration:.1f}s"
                )
            elif step.kind == "approach":
                self.get_logger().info(
                    f"[STEP] {step.label} -> "
                    f"target=({step.x:.3f}, {step.y:.3f}), "
                    f"steer_limit={math.degrees(step.max_steering):.1f}deg"
                )
            elif step.kind == "straight_y":
                self.get_logger().info(
                    f"[STEP] {step.label} -> y={step.y:.3f}, "
                    f"drive={step.drive:.2f}, steer=0.0deg"
                )
            elif step.kind == "pose_check":
                self.get_logger().info(
                    f"[STEP] {step.label} -> check pose="
                    f"({step.x:.3f}, {step.y:.3f}, {step.yaw:.3f})"
                )
            else:
                self.get_logger().info(f"[STEP] {step.label}")

    def _finish_queue(self) -> None:
        self._stop()
        self._mode = self._queue_result_mode
        if self._mode == self.MODE_WAIT_RETURN:
            self._publish_status(
                f"Pallet_{self._current_pallet:02d} AMR 작업·복귀 대기"
            )
        elif self._mode == self.MODE_COMPLETE:
            self._publish_status("Pallet_00~05 전체 작업 완료, 대기 위치 유지")
        elif self._mode == self.MODE_HOLDING_PALLET:
            self._publish_status("Pallet_00을 포크에 연결해 들어 올린 상태로 정지")
        elif self._mode == self.MODE_PALLET_01_ON_IW:
            self._publish_status(
                "Pallet_00 0번 위치 복귀·Pallet_01 IW 상차 완료, "
                "대기 위치 유지"
            )

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
        msg.name = [
            "lift_joint",
            "back_wheel_swivel",
            "back_wheel_drive",
            "pallet_attach",
            "pallet_id",
        ]
        msg.position = [
            self._lift_target,
            steering,
            math.nan,
            1.0 if self._pallet_attached_command else 0.0,
            float(self._pallet_target_command),
        ]
        msg.velocity = [math.nan, math.nan, drive, math.nan, math.nan]
        self._command_pub.publish(msg)

    def _stop(self) -> None:
        self._publish_command(0.0, 0.0)

    def _integrate_dead_reckoning(self, dt: float) -> None:
        if self._use_pose_feedback and self._pose_feedback_time is not None:
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
        self._pallet_attached_command = False
        # SIGINT/SIGTERM 처리 과정에서 rclpy context가 먼저 종료된 경우 publisher를
        # 호출하면 RCLError가 난다. context가 살아 있을 때만 정지 명령을 보낸다.
        if rclpy.ok():
            for _ in range(3):
                self._stop()
        try:
            return super().destroy_node()
        finally:
            if not self._instance_lock.closed:
                fcntl.flock(self._instance_lock.fileno(), fcntl.LOCK_UN)
                self._instance_lock.close()


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
