"""비전의 카메라 좌표를 매니퓰레이터 베이스 좌표 목표로 변환한다."""

from __future__ import annotations

import json
import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

# PoseStamped 변환 등록을 위한 side effect import. Buffer.transform API를 사용하면
# Humble(geometry2 0.25)과 Jazzy(geometry2 0.36)의 helper 함수 차이를 피할 수 있다.
import tf2_geometry_msgs  # noqa: F401
from tf2_ros import Buffer, TransformException, TransformListener

ACTIVE_SEQUENCE_STATES = {
    "PREGRASP", "GRASP", "GRASP_YAW_CORRECT", "GRIPPER_CLOSING", "GRASP_VERIFY",
    "VERIFY_RETRACT", "GRASP_FOLLOW_CHECK", "RETRACT",
    "WAIT_BASKET", "BASKET_APPROACH", "BASKET_PLACE", "PLACE_RELEASING",
    "GO_HOME",
    "NAV_REPOSITION_REQUIRED",
}


class ManipulatorTargetNode(Node):
    """검출 pose를 TF 변환하고 안전 조건을 통과한 목표만 발행한다.

    좌표 검증과 접근/파지 상태 순서를 책임지고 IK는 Isaac RMPflow에 맡긴다.
    command_enabled가 true일 때만 실행 명령을 보내며, validated_topic은 RViz와
    dry-run 검증을 위해 항상 발행한다.
    """

    def __init__(self):
        super().__init__("manipulator_target_node")
        self.declare_parameter("input_topic", "/vision/approach_target")
        self.declare_parameter(
            "validated_topic", "/harvester_0/manipulator/validated_target"
        )
        self.declare_parameter("output_topic", "/harvester_0/manipulator/target_pose")
        self.declare_parameter("isaac_command_topic", "/harvester_0/cmd")
        self.declare_parameter("target_class_topic", "/vision/target_class")
        self.declare_parameter(
            "state_topic", "/harvester_0/manipulator/target_state"
        )
        self.declare_parameter(
            "rmp_status_topic", "/harvester_0/rmpflow/status"
        )
        self.declare_parameter("basket_pose_topic", "/iw/basket/empty_slot_pose")
        self.declare_parameter("harvest_enable_topic", "/harvest_test/enable")
        self.declare_parameter("external_harvest_gate_enabled", False)
        self.declare_parameter("use_sim_ground_truth", False)
        self.declare_parameter("sim_tomato_topic", "/harvester_0/sim/tomato")
        self.declare_parameter("sim_match_radius_m", 0.35)
        self.declare_parameter(
            "mobility_ready_topic", "/harvester_0/manipulator/mobility_ready"
        )
        self.declare_parameter(
            "reposition_request_topic", "/harvester_0/nav/reposition_request"
        )
        # 재정차 뒤 과실 중심이 이 거리 안에 오도록 베이스를 전진시킨다. 팔의
        # workspace_max까지 억지로 뻗지 않고 마지막 15 cm 직선 접근 여유를 남긴다.
        self.declare_parameter("reposition_target_x_m", 0.90)
        self.declare_parameter("reposition_target_abs_y_m", 0.45)
        # x/y 개별 상한만으로는 대각선 목표가 통과한다. 예: (1.15, -0.63)은
        # 각 축 범위 안이지만 수평 반경 1.31m라 최종 GRASP에서 팔이 완전히 펴진다.
        self.declare_parameter("workspace_max_xy_radius_m", 1.50)
        self.declare_parameter("base_frame", "harvester_0/base_link")
        self.declare_parameter("command_enabled", False)
        self.declare_parameter("max_target_age_sec", 0.5)
        self.declare_parameter("tf_timeout_sec", 0.2)
        self.declare_parameter("max_jump_m", 0.15)
        self.declare_parameter("auto_grasp_enabled", True)
        self.declare_parameter("pregrasp_clearance_m", 0.15)
        # RMPflow가 고정된 과실 중심을 관통하려 하면 collider 표면에서 3~4cm 오차로
        # 정체된다. 열린 그리퍼는 과실 표면까지 보내고 그 위치에서 닫는다.
        self.declare_parameter("grasp_surface_standoff_m", 0.034)
        # USD HarvestTCP와 같은 값. 현재 제어는 USD TCP를 직접 측정하지만 외부 launch가
        # 이 파라미터를 참조해도 서로 다른 오프셋을 사용하지 않도록 동기화한다.
        self.declare_parameter("tool_grasp_reach_m", 0.132)
        self.declare_parameter("motion_timeout_sec", 10.0)
        self.declare_parameter("gripper_close_settle_sec", 1.0)
        self.declare_parameter("grasp_tcp_max_distance_m", 0.06)
        self.declare_parameter("grasp_verify_retract_m", 0.03)
        self.declare_parameter("grasp_follow_max_delta_m", 0.015)
        self.declare_parameter("grasp_one_side_yaw_deg", 5.0)
        self.declare_parameter("grasp_one_side_max_retries", 1)
        self.declare_parameter("basket_approach_height_m", 0.15)
        self.declare_parameter("basket_workspace_min", [-0.80, -0.80, 0.15])
        self.declare_parameter("basket_workspace_max", [1.25, 0.80, 1.80])
        self.declare_parameter("workspace_min", [0.15, -1.05, 0.15])
        self.declare_parameter("workspace_max", [1.25, 1.05, 1.80])
        # 데모: 성공/실패 무관 매 시도 후 홈 복귀 → 팔이 안 굳고 다음 과실을 계속 시도한다.
        self.declare_parameter("home_after_attempt", True)
        # h 원샷: 한 사이클(인식→수확→홈) 끝나면 게이트를 스스로 끈다. 계속 재인식·재시작
        # 하지 않고 h 를 다시 눌러야 다음 과실을 잡는다.
        self.declare_parameter("single_shot_harvest", True)

        self._buffer = Buffer()
        self._listener = TransformListener(self._buffer, self)
        self._last_position: tuple[float, float, float] | None = None
        self._latest_target: tuple[float, float, float] | None = None
        self._latest_camera: tuple[float, float, float] | None = None
        self._sequence_id = 0
        self._pending_id = 0
        self._deadline_ns = 0
        self._best_motion_distance = float("inf")
        self._grasp_target = np.zeros(3, dtype=float)
        self._fruit_target = np.zeros(3, dtype=float)
        self._grasp_fruit_id: int | None = None
        self._reposition_fruit_id: int | None = None
        self._reposition_requested_ns = 0
        self._pregrasp_target = np.zeros(3, dtype=float)
        self._gripper_command_at_ns = 0
        self._grasp_check_id = 0
        self._grasp_check_sent = False
        self._grasp_yaw_retry_count = 0
        self._follow_check_id = 0
        self._basket_place: np.ndarray | None = None
        self._sim_fruits: dict[int, tuple[np.ndarray, int]] = {}
        self._retry_after_home = False

        input_topic = str(self.get_parameter("input_topic").value)
        validated_topic = str(self.get_parameter("validated_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        command_topic = str(self.get_parameter("isaac_command_topic").value)
        class_topic = str(self.get_parameter("target_class_topic").value)
        state_topic = str(self.get_parameter("state_topic").value)
        status_topic = str(self.get_parameter("rmp_status_topic").value)
        basket_topic = str(self.get_parameter("basket_pose_topic").value)
        enable_topic = str(self.get_parameter("harvest_enable_topic").value)
        mobility_topic = str(self.get_parameter("mobility_ready_topic").value)
        reposition_topic = str(
            self.get_parameter("reposition_request_topic").value)
        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._validated_pub = self.create_publisher(PoseStamped, validated_topic, 10)
        self._command_pub = self.create_publisher(PoseStamped, output_topic, 10)
        self._isaac_command_pub = self.create_publisher(String, command_topic, 10)
        # 상태는 전이할 때만 발행하므로 늦게 붙은 디버거도 마지막 값을 받게 latch한다.
        self._state_pub = self.create_publisher(String, state_topic, latched_qos)
        self._mobility_pub = self.create_publisher(
            Bool, mobility_topic, latched_qos)
        self._reposition_pub = self.create_publisher(
            String, reposition_topic, latched_qos)
        self.create_subscription(PoseStamped, input_topic, self._target_callback, 10)
        self.create_subscription(String, class_topic, self._class_callback, 10)
        self.create_subscription(String, status_topic, self._status_callback, 10)
        self.create_subscription(PoseStamped, basket_topic, self._basket_callback, 10)
        self.create_subscription(Bool, enable_topic, self._enable_callback, 10)
        self.create_subscription(
            String, str(self.get_parameter("sim_tomato_topic").value),
            self._sim_tomato_callback, 20)
        self.create_timer(0.1, self._watchdog)
        self._target_class = ""
        self._state = "NO_TARGET"
        self._harvest_enabled = not bool(
            self.get_parameter("external_harvest_gate_enabled").value)
        self._mobility_pub.publish(Bool(data=True))
        self._state_pub.publish(String(data=self._state))

        enabled = bool(self.get_parameter("command_enabled").value)
        self.get_logger().info(
            f"비전→매니퓰레이터 좌표 브리지 시작: {input_topic} -> "
            f"{self.get_parameter('base_frame').value} (command_enabled={enabled})"
        )

    def _target_callback(self, msg: PoseStamped) -> None:
        if not msg.header.frame_id:
            self.get_logger().warning("frame_id 없는 비전 목표를 무시합니다")
            return
        if self._is_stale(msg):
            self.get_logger().warning("오래된 비전 목표를 무시합니다")
            return
        if self._state in ACTIVE_SEQUENCE_STATES:
            # PREGRASP 시작 때 확정한 타겟을 수확 완료까지 잠근다. 여러 토마토 사이에서
            # detector의 nearest 선택이 바뀌어도 진행 중인 grasp 좌표를 덮어쓰지 않는다.
            return

        base_frame = str(self.get_parameter("base_frame").value)
        timeout = float(self.get_parameter("tf_timeout_sec").value)
        try:
            target = self._buffer.transform(
                msg,
                base_frame,
                timeout=Duration(seconds=timeout),
            )
            camera_origin = PoseStamped()
            camera_origin.header = msg.header
            camera_origin.pose.orientation.w = 1.0
            camera = self._buffer.transform(
                camera_origin,
                base_frame,
                timeout=Duration(seconds=timeout),
            )
        except TransformException as exc:
            self.get_logger().warning(
                f"TF 변환 실패 ({msg.header.frame_id} -> {base_frame}): {exc}",
                throttle_duration_sec=2.0,
            )
            return

        # tf2가 반환한 frame_id를 신뢰하되, 배포판별 구현 차이에 대비해 정규화한다.
        target.header.frame_id = base_frame
        if not self._inside_workspace(target):
            p = target.pose.position
            self.get_logger().warning(
                f"작업영역 밖 목표 차단: ({p.x:.3f}, {p.y:.3f}, {p.z:.3f})"
            )
            return
        # Nav 대기 중에는 팔 명령이 나가지 않으므로 검출 대상이 바뀌어도 최신 좌표를
        # 받아야 한다. 이전에는 여기서 계속 차단돼 _last_position이 영원히 과거
        # 토마토에 고정되고 GT 매칭도 복구되지 않았다.
        if self._harvest_enabled and self._is_jump(target):
            p = target.pose.position
            self.get_logger().warning(
                f"급격한 목표 이동 차단: ({p.x:.3f}, {p.y:.3f}, {p.z:.3f})"
            )
            return

        p = target.pose.position
        cp = camera.pose.position
        self._last_position = (p.x, p.y, p.z)
        self._latest_target = (p.x, p.y, p.z)
        self._latest_camera = (cp.x, cp.y, cp.z)
        self._validated_pub.publish(target)
        if (bool(self.get_parameter("command_enabled").value)
                and self._harvest_enabled
                and self._target_class == "tomato"
                and self._state == "APPROACH"):
            target_values = np.array([p.x, p.y, p.z], dtype=float)
            camera_values = np.array([cp.x, cp.y, cp.z], dtype=float)
            ray = target_values - camera_values
            ray_length = float(np.linalg.norm(ray))
            if ray_length < 1e-6:
                return
            ray /= ray_length
            standoff = (
                float(self.get_parameter("pregrasp_clearance_m").value)
            )
            approach_values = target_values - ray * standoff
            approach = PoseStamped()
            approach.header = target.header
            approach.pose.orientation.w = 1.0
            approach.pose.position.x = float(approach_values[0])
            approach.pose.position.y = float(approach_values[1])
            approach.pose.position.z = float(approach_values[2])
            self._command_pub.publish(approach)
            command = {
                "rmp_target": {
                    "frame_id": base_frame,
                    "phase": "APPROACH",
                    "position": [float(value) for value in approach_values],
                }
            }
            self._isaac_command_pub.publish(String(data=json.dumps(command)))

    def _class_callback(self, msg: String) -> None:
        target_class = msg.data.strip().lower()
        self._target_class = target_class
        if (bool(self.get_parameter("external_harvest_gate_enabled").value)
                and not self._harvest_enabled):
            # Nav2 주행 중에는 검출/좌표 시각화만 유지하고 팔 명령은 완전히 차단한다.
            self._deadline_ns = 0
            self._transition("WAIT_NAV", stop=True)
            return
        if self._state in ACTIVE_SEQUENCE_STATES:
            # 파지 중 검출 흔들림으로 시퀀스를 재시작하지 않는다.
            # 표적 소실과 spoiled 판정만 긴급 중단한다.
            # 시뮬 GT에 매칭된 뒤에는 과실 좌표와 정체가 이미 확정됐다. 팔/카메라가
            # 움직이며 YOLO가 잠깐 quality_check/빈 프레임을 내도 수확을 중단하지 않는다.
            if bool(self.get_parameter("use_sim_ground_truth").value):
                return
            if self._state in {"PREGRASP", "GRASP", "GRIPPER_CLOSING"} and not target_class:
                self._deadline_ns = 0
                self._transition("ABORT_TARGET_LOST", stop=True)
            elif self._state in {"PREGRASP", "GRASP", "GRIPPER_CLOSING"} and target_class == "spoiled":
                self._deadline_ns = 0
                self._transition("ABORT_SPOILED", stop=True)
            return
        # 홈 복귀 직후 이전 프레임의 ripe 판정으로 재수확하지 않는다. 한 번 표적이
        # 사라진 뒤에만 다음 사이클을 받는다.
        if self._state == "HOME_READY" and target_class:
            return
        if not target_class:
            self._transition("NO_TARGET", stop=True)
        elif target_class in ("tomato", "ripe"):
            # 데모(A): 원거리 "tomato" 검출로 **바로 파지**. near(ripe 판정) 모델이 시뮬 크롭에서
            # 검출을 못 해 APPROACH 에서 멈추던 문제 우회(2026-07-22). 익음구분은 생략한다.
            changed = self._transition("RIPE_READY", stop=True)
            if (changed and bool(self.get_parameter("command_enabled").value)
                    and bool(self.get_parameter("auto_grasp_enabled").value)
                    and self._harvest_enabled):
                self._start_grasp_sequence()
        # ── 원래 2단계(익음구분) 동작. 되살리려면 위 elif 를 지우고 아래 둘을 활성화 ──
        # elif target_class == "tomato":
        #     self._transition("APPROACH")          # 다가가서 near 모델로 ripe/spoiled 판정
        # elif target_class == "ripe":
        #     changed = self._transition("RIPE_READY", stop=True)
        #     if (changed and bool(self.get_parameter("command_enabled").value)
        #             and bool(self.get_parameter("auto_grasp_enabled").value)
        #             and self._harvest_enabled):
        #         self._start_grasp_sequence()
        elif target_class == "quality_check":
            self._transition("QUALITY_CHECK", stop=True)
        elif target_class == "spoiled":
            self._transition("SKIP_SPOILED", stop=True)
        else:
            self._transition("UNKNOWN_CLASS", stop=True)

    def _transition(self, state: str, stop: bool = False) -> bool:
        if state == self._state:
            return False
        self._state = state
        self._state_pub.publish(String(data=state))
        self.get_logger().info(f"매니퓰레이터 목표 상태: {state}")
        if stop and bool(self.get_parameter("command_enabled").value):
            self._isaac_command_pub.publish(
                String(data=json.dumps({"rmp_stop": True}))
            )
        return True

    def _enable_callback(self, msg: Bool) -> None:
        if not bool(self.get_parameter("external_harvest_gate_enabled").value):
            return
        self._harvest_enabled = bool(msg.data)
        if not self._harvest_enabled:
            self._deadline_ns = 0
            self._transition("WAIT_NAV", stop=True)
            return
        # 도착 순간 이미 보던 최신 클래스를 다시 처리한다. 다음 카메라 프레임을
        # 기다리는 동안 게이트 상태와 FSM 상태가 어긋나지 않게 한다.
        self._class_callback(String(data=self._target_class))

    def _start_grasp_sequence(self) -> None:
        if self._latest_target is None or self._latest_camera is None:
            self._transition("ERROR_NO_TARGET", stop=True)
            return
        target = np.asarray(self._latest_target, dtype=float)
        if bool(self.get_parameter("use_sim_ground_truth").value):
            sim_match = None
            # Nav 재정차 전 확정한 과실은 이동 뒤 YOLO가 다른 과실을 먼저 내더라도
            # 바꾸지 않는다. base-frame 좌표는 이동하면 달라지므로 재정차 요청 이후에
            # 같은 ID가 다시 발행된 좌표만 사용한다.
            locked_id = self._reposition_fruit_id
            locked = self._sim_fruits.get(locked_id) if locked_id is not None else None
            if locked is not None and locked[1] > self._reposition_requested_ns:
                sim_match = (locked[0].copy(), locked_id)
                self.get_logger().info(
                    f"Nav 재정차 후 기존 토마토 ID={locked_id} 재선택")
            elif locked_id is None:
                sim_match = self._match_sim_tomato(target, np.asarray(
                    self._latest_camera, dtype=float))
            if sim_match is None:
                # 후보가 round-robin 토픽으로 더 들어오거나 다음 검출 프레임에서
                # 광선이 안정되면 자동 재시도한다. 일시 실패로 수확 게이트를 닫지 않는다.
                self._transition("WAIT_SIM_MATCH", stop=True)
                return
            sim_target, sim_fruit_id = sim_match
            self.get_logger().info(
                "비전 검출을 시뮬 토마토 좌표에 매칭: "
                f"vision=({target[0]:.3f}, {target[1]:.3f}, {target[2]:.3f}) -> "
                f"sim=({sim_target[0]:.3f}, {sim_target[1]:.3f}, {sim_target[2]:.3f})")
            target = sim_target
            self._grasp_fruit_id = sim_fruit_id
            self._latest_target = tuple(float(v) for v in target)
        lower = np.asarray(self.get_parameter("workspace_min").value, dtype=float)
        upper = np.asarray(self.get_parameter("workspace_max").value, dtype=float)
        xy_radius = float(np.linalg.norm(target[:2]))
        max_xy_radius = float(
            self.get_parameter("workspace_max_xy_radius_m").value)
        if (not np.all(np.isfinite(target))
                or not np.all((lower <= target) & (target <= upper))
                or xy_radius > max_xy_radius):
            # 비전 좌표는 앞에서 검사되지만 sim GT로 치환한 좌표도 반드시 다시
            # 검사해야 한다. 도달 불가능한 좌표를 IK에 넣으면 관절 한계에서 팔이
            # 위로 뜬 채 가장 가까운 줄기를 건드리게 된다.
            desired_x = float(self.get_parameter("reposition_target_x_m").value)
            desired_abs_y = float(
                self.get_parameter("reposition_target_abs_y_m").value)
            forward = max(0.0, float(target[0]) - desired_x)
            desired_y = float(np.clip(target[1], -desired_abs_y, desired_abs_y))
            lateral = float(target[1]) - desired_y
            self._deadline_ns = 0
            self._reposition_fruit_id = self._grasp_fruit_id
            self._reposition_requested_ns = self.get_clock().now().nanoseconds
            self._mobility_pub.publish(Bool(data=True))
            self._transition("NAV_REPOSITION_REQUIRED", stop=True)
            self._reposition_pub.publish(String(data=json.dumps({
                "forward_m": forward,
                "lateral_m": lateral,
                "target_base": [float(v) for v in target],
                "reason": "target_outside_manipulator_workspace",
            })))
            self.get_logger().warning(
                "GT 목표가 팔 작업영역 밖: "
                f"target=({target[0]:.3f}, {target[1]:.3f}, {target[2]:.3f}), "
                f"xy_radius={xy_radius:.3f}m (limit={max_xy_radius:.3f}m), "
                f"Nav2 재접근 요청=(forward {forward:.3f}m, lateral {lateral:.3f}m)")
            return
        # 동일 ID의 재정차 후 최신 base-frame 좌표가 작업영역 안에 들어왔다.
        self._reposition_fruit_id = None
        self._reposition_requested_ns = 0
        self._mobility_pub.publish(Bool(data=False))
        camera = np.asarray(self._latest_camera, dtype=float)
        ray = target - camera
        length = float(np.linalg.norm(ray))
        if length < 1e-6:
            self._transition("ERROR_BAD_RAY", stop=True)
            return
        ray /= length
        clearance = float(self.get_parameter("pregrasp_clearance_m").value)
        # position은 이제 UR 플랜지가 아니라 실제 HarvestTCP 목표다.
        pregrasp = target - ray * clearance
        surface_standoff = float(
            self.get_parameter("grasp_surface_standoff_m").value)
        # 목표 과실 collider 반지름만큼 카메라 쪽에 멈춘다. 여기서 TCP 오차를
        # 20mm 이내로 수렴시킨 뒤 손가락을 닫아 과실을 패드 사이로 감싼다.
        grasp = target - ray * surface_standoff
        self._pregrasp_target = pregrasp
        self._grasp_target = grasp
        self._fruit_target = target.copy()
        self._grasp_yaw_retry_count = 0
        # 파지 전 그리퍼를 연다 — 닫힌 채로 다가가면 손가락이 과실을 못 감싼다("잡는 느낌이
        # 아니다", 2026-07-22). 직전 사이클에서 닫혀 있어도 여기서 확실히 벌린다.
        self._isaac_command_pub.publish(
            String(data=json.dumps({"gripper": {"closed": False}})))
        self._send_rmp_goal(pregrasp, "PREGRASP")

    def _sim_tomato_callback(self, msg: String) -> None:
        try:
            item = json.loads(msg.data)
            if not isinstance(item, dict):
                return
            fruit_id = int(item["id"])
            position = np.asarray(item["position"], dtype=float)
        except (KeyError, TypeError, ValueError):
            return
        if item.get("class") != "ripe" or position.shape != (3,):
            return
        now = self.get_clock().now().nanoseconds
        self._sim_fruits[fruit_id] = (position, now)
        self._sim_fruits = {
            key: entry for key, entry in self._sim_fruits.items()
            if now - entry[1] <= int(30.0e9)}

    def _match_sim_tomato(
        self, vision_target: np.ndarray, camera: np.ndarray
    ) -> tuple[np.ndarray, int] | None:
        now = self.get_clock().now().nanoseconds
        fresh = [(fruit_id, position) for fruit_id, (position, stamp)
                 in self._sim_fruits.items() if now - stamp <= int(30.0e9)]
        if not fresh:
            return None
        # depth는 앞쪽 잎 때문에 크게 틀릴 수 있지만 검출 중심의 카메라 광선은
        # 유효하다. 3D 점간 거리 대신 각 GT 과실의 광선 횡오차로 대응시킨다.
        ray = vision_target - camera
        vision_depth = float(np.linalg.norm(ray))
        if vision_depth < 1e-6:
            return None
        ray /= vision_depth
        scored = []
        for fruit_id, position in fresh:
            relative = position - camera
            along = float(np.dot(relative, ray))
            if along <= 0.0:
                continue
            lateral = float(np.linalg.norm(relative - ray * along))
            # 같은 광선상 과실이 여럿이면 비전의 대략 깊이에 가까운 것을 우선한다.
            score = lateral + 0.05 * abs(along - vision_depth)
            scored.append((score, lateral, fruit_id, position))
        if not scored:
            return None
        _, lateral, fruit_id, nearest = min(scored, key=lambda item: item[0])
        if lateral > float(self.get_parameter("sim_match_radius_m").value):
            self.get_logger().warning(
                f"시뮬 토마토 광선 매칭 거리 초과: {lateral:.3f}m")
            return None
        return nearest.copy(), fruit_id

    def _send_rmp_goal(self, position: np.ndarray, phase: str) -> None:
        self._sequence_id += 1
        self._pending_id = self._sequence_id
        timeout = float(self.get_parameter("motion_timeout_sec").value)
        self._deadline_ns = self.get_clock().now().nanoseconds + int(timeout * 1e9)
        self._best_motion_distance = float("inf")
        self._transition(phase)
        command = {
            "rmp_target": {
                "id": self._pending_id,
                "phase": phase,
                "frame_id": str(self.get_parameter("base_frame").value),
                "position": [float(value) for value in position],
            }
        }
        self.get_logger().info(
            f"RMPflow 명령 id={self._pending_id} phase={phase} "
            f"target=({position[0]:.3f}, {position[1]:.3f}, {position[2]:.3f})"
        )
        self._isaac_command_pub.publish(String(data=json.dumps(command)))

    def _status_callback(self, msg: String) -> None:
        try:
            status = json.loads(msg.data)
        except (TypeError, ValueError):
            return
        if not isinstance(status, dict):
            return
        if "grasp_id" in status:
            if (self._state != "GRASP_VERIFY"
                    or int(status.get("grasp_id", -1)) != self._grasp_check_id):
                return
            if bool(status.get("ok", False)):
                self.get_logger().info(
                    "GRASP TCP 근접 + 그리퍼 닫힘; "
                    "pedicel FixedJoint 해제 "
                    f"{float(status.get('d', 999.0)):.3f}m")
                self._begin_verify_retract()
            else:
                self._deadline_ns = 0
                self._abort_to_home(
                    f"grasp_verify distance={status.get('d')} "
                    f"contact=L{int(bool(status.get('l', False)))}/"
                    f"R{int(bool(status.get('r', False)))}")
            return
        if "follow_id" in status:
            if (self._state != "GRASP_FOLLOW_CHECK"
                    or int(status.get("follow_id", -1)) != self._follow_check_id):
                return
            if bool(status.get("ok", False)):
                self.get_logger().info(
                    "파지 동반 이동 검증 성공: 상대거리 변화 "
                    f"{float(status.get('delta', 999.0)):.3f}m")
                self._send_rmp_goal(self._pregrasp_target, "RETRACT")
            else:
                self._deadline_ns = 0
                self._abort_to_home(
                    f"grasp_follow delta={status.get('delta')}")
            return
        if self._state == "GRIPPER_CLOSING":
            # watchdog이 닫기 정착 시간 뒤 TCP 거리 검증을 요청한다.
            return
        if self._state == "PLACE_RELEASING":
            try:
                gripper = float(status.get("gripper", 1.0))
            except (TypeError, ValueError):
                return
            if gripper <= 0.08:
                self._send_home()
            return
        try:
            status_id = int(status.get("id", -1))
        except (TypeError, ValueError):
            return
        if status_id != self._pending_id:
            return
        phase = str(status.get("phase", ""))
        if phase in {"ERROR_DIVERGENCE", "ERROR_IK_PATH", "ERROR_STAGNATION"}:
            self._deadline_ns = 0
            self._abort_to_home(phase.lower())
            return
        # 한 번 선택한 목표는 ACTIVE_SEQUENCE_STATES 동안 이미 콜백에서 잠겨 있다.
        # 여기에 진행 기반 watchdog을 더해, 같은 목표로 정상 접근 중인데 고정 시간만
        # 지났다는 이유로 포기하지 않는다. 5mm 이상 개선될 때마다 제한시간을 갱신한다.
        try:
            distance = float(status.get("distance", float("inf")))
        except (TypeError, ValueError):
            distance = float("inf")
        if (phase in {"PREGRASP", "GRASP"}
                and math.isfinite(distance)
                and distance < self._best_motion_distance - 0.005):
            self._best_motion_distance = distance
            timeout = float(self.get_parameter("motion_timeout_sec").value)
            self._deadline_ns = (
                self.get_clock().now().nanoseconds + int(timeout * 1e9))
        if not bool(status.get("reached", False)):
            return
        if self._state == "GRASP_YAW_CORRECT":
            self._transition("GRIPPER_CLOSING", stop=True)
            self._gripper_command_at_ns = self.get_clock().now().nanoseconds
            self._grasp_check_sent = False
            self._deadline_ns = (
                self.get_clock().now().nanoseconds
                + int(float(self.get_parameter("motion_timeout_sec").value) * 1e9)
            )
            self._isaac_command_pub.publish(
                String(data=json.dumps({"gripper": {"closed": True}})))
        elif self._state == "PREGRASP":
            self._send_rmp_goal(self._grasp_target, "GRASP")
        elif self._state == "GRASP":
            self._transition("GRIPPER_CLOSING", stop=True)
            self._gripper_command_at_ns = self.get_clock().now().nanoseconds
            self._grasp_check_sent = False
            self._deadline_ns = (
                self.get_clock().now().nanoseconds
                + int(float(self.get_parameter("motion_timeout_sec").value) * 1e9)
            )
            self._isaac_command_pub.publish(
                String(data=json.dumps({"gripper": {"closed": True}}))
            )
        elif self._state == "VERIFY_RETRACT":
            self._sequence_id += 1
            self._follow_check_id = self._sequence_id
            self._transition("GRASP_FOLLOW_CHECK", stop=True)
            self._deadline_ns = (
                self.get_clock().now().nanoseconds
                + int(float(self.get_parameter("motion_timeout_sec").value) * 1e9)
            )
            self._isaac_command_pub.publish(String(data=json.dumps({
                "follow_check": {
                    "id": self._follow_check_id,
                    "max_delta": float(self.get_parameter(
                        "grasp_follow_max_delta_m").value),
                }
            })))
        elif self._state == "RETRACT":
            if self._basket_place is not None:
                self._start_place()
            elif bool(self.get_parameter("home_after_attempt").value):
                self._deadline_ns = 0
                self._send_home()
            else:
                self._deadline_ns = 0
                self._transition("WAIT_BASKET", stop=True)
        elif self._state == "BASKET_APPROACH":
            self._send_rmp_goal(self._basket_place, "BASKET_PLACE")
        elif self._state == "BASKET_PLACE":
            self._transition("PLACE_RELEASING", stop=True)
            self._deadline_ns = (
                self.get_clock().now().nanoseconds
                + int(float(self.get_parameter("motion_timeout_sec").value) * 1e9)
            )
            self._isaac_command_pub.publish(
                String(data=json.dumps({"gripper": {"closed": False}})))
        elif self._state == "GO_HOME":
            self._deadline_ns = 0
            self._mobility_pub.publish(Bool(data=True))
            if self._retry_after_home:
                # 실패한 표적/광선/ID를 재사용하지 않는다. 홈 자세의 새 카메라 프레임과
                # 새 YOLO 결과가 들어와야 다음 grasp sequence가 시작된다.
                self._retry_after_home = False
                self._target_class = ""
                self._latest_target = None
                self._latest_camera = None
                self._last_position = None
                self._grasp_fruit_id = None
                self._reposition_fruit_id = None
                self._reposition_requested_ns = 0
                self._transition("RETRY_VISION", stop=True)
                self.get_logger().info("홈 복귀 완료 — 비전 재탐색 후 자동 재시도")
            else:
                self._transition("HOME_READY", stop=True)
                self._maybe_single_shot_off()

    def _basket_callback(self, msg: PoseStamped) -> None:
        """IW가 선택한 빈 바스켓 슬롯의 tool-release pose를 base 좌표로 저장한다."""
        if not msg.header.frame_id or self._is_stale(msg):
            return
        base_frame = str(self.get_parameter("base_frame").value)
        try:
            target = self._buffer.transform(
                msg, base_frame,
                timeout=Duration(seconds=float(
                    self.get_parameter("tf_timeout_sec").value)))
        except TransformException as exc:
            self.get_logger().warning(
                f"바스켓 TF 변환 실패 ({msg.header.frame_id} -> {base_frame}): {exc}",
                throttle_duration_sec=2.0)
            return
        p = target.pose.position
        values = np.array([p.x, p.y, p.z], dtype=float)
        lower = np.asarray(self.get_parameter("basket_workspace_min").value)
        upper = np.asarray(self.get_parameter("basket_workspace_max").value)
        if (not np.all(np.isfinite(values))
                or not np.all((lower <= values) & (values <= upper))):
            self.get_logger().warning("작업영역 밖 바스켓 목표를 무시합니다")
            return
        self._basket_place = values
        if self._state == "WAIT_BASKET":
            self._start_place()

    def _start_place(self) -> None:
        if self._basket_place is None:
            self._transition("WAIT_BASKET", stop=True)
            return
        approach = self._basket_place.copy()
        approach[2] += float(
            self.get_parameter("basket_approach_height_m").value)
        self._send_rmp_goal(approach, "BASKET_APPROACH")

    def _send_home(self, retry_after_home: bool = False) -> None:
        self._basket_place = None
        self._retry_after_home = bool(retry_after_home)
        self._sequence_id += 1
        self._pending_id = self._sequence_id
        self._transition("GO_HOME")
        self._deadline_ns = (
            self.get_clock().now().nanoseconds
            + int(float(self.get_parameter("motion_timeout_sec").value) * 1e9)
        )
        self._isaac_command_pub.publish(String(data=json.dumps({
            "rmp_home": {"id": self._pending_id},
        })))

    def _maybe_single_shot_off(self) -> None:
        """h 원샷: 사이클 끝나면 게이트를 끈다 — 다시 h 를 눌러야 다음 과실을 잡는다.
        계속 켜두면 이동 중 재인식으로 시퀀스가 흔들리는 걸 막는다."""
        if bool(self.get_parameter("single_shot_harvest").value):
            self._harvest_enabled = False
            self.get_logger().info("원샷 수확 완료 — h 다시 눌러야 다음 과실")

    def _abort_to_home(self, reason: str) -> None:
        """실패해도 팔을 홈으로 돌려 다음 과실을 계속 시도하게 한다(데모 연속 사이클).
        이미 홈 복귀 중(GO_HOME)에 또 실패하면 무한루프 방지로 멈추기만 한다."""
        self.get_logger().warning(f"수확 실패({reason}) — 홈 복귀 후 다음 시도")
        self._isaac_command_pub.publish(String(data=json.dumps({
            "gripper": {"closed": False},
        })))
        if self._state == "GO_HOME":
            self._deadline_ns = 0
            self._transition("HOME_READY", stop=True)
            self._mobility_pub.publish(Bool(data=True))
            self._maybe_single_shot_off()
            return
        self._send_home(retry_after_home=True)

    def _begin_verify_retract(self) -> None:
        self._gripper_command_at_ns = 0
        direction = self._pregrasp_target - self._grasp_target
        length = float(np.linalg.norm(direction))
        if length < 1e-6:
            self._abort_to_home("bad_verify_retract_direction")
            return
        distance = float(self.get_parameter("grasp_verify_retract_m").value)
        verify_target = self._grasp_target + direction / length * distance
        self._send_rmp_goal(verify_target, "VERIFY_RETRACT")

    def _start_one_side_yaw_correction(self, left: bool, right: bool) -> None:
        """한 손가락만 닿았으면 닿은 방향으로 손목을 돌린 뒤 한 번 다시 파지한다."""
        self._grasp_yaw_retry_count += 1
        magnitude = float(self.get_parameter("grasp_one_side_yaw_deg").value)
        delta = magnitude if left else -magnitude
        side = "left" if left else "right"
        self._sequence_id += 1
        self._pending_id = self._sequence_id
        self._gripper_command_at_ns = 0
        self._grasp_check_sent = False
        self._transition("GRASP_YAW_CORRECT")
        timeout = float(self.get_parameter("motion_timeout_sec").value)
        self._deadline_ns = (
            self.get_clock().now().nanoseconds + int(timeout * 1e9))
        self.get_logger().warning(
            f"단측 접촉({side}) — 닿은 쪽으로 손목 yaw {delta:+.1f}° 보정 후 재파지")
        self._isaac_command_pub.publish(String(data=json.dumps({
            "gripper": {"closed": False},
            "grasp_yaw_adjust": {
                "id": self._pending_id,
                "delta_deg": delta,
            },
        })))

    def _watchdog(self) -> None:
        now = self.get_clock().now().nanoseconds
        # GRASP TCP 도달 직후 보낸 닫기 명령이 정착되면 TCP 거리를 질의한다.
        if (self._state == "GRIPPER_CLOSING"
                and self._gripper_command_at_ns
                and not self._grasp_check_sent
                and now - self._gripper_command_at_ns >= int(float(
                    self.get_parameter("gripper_close_settle_sec").value) * 1e9)):
            self._grasp_check_sent = True
            self._sequence_id += 1
            self._grasp_check_id = self._sequence_id
            self._transition("GRASP_VERIFY", stop=True)
            self._deadline_ns = now + int(float(
                self.get_parameter("motion_timeout_sec").value) * 1e9)
            self._isaac_command_pub.publish(String(data=json.dumps({
                    "grasp_check": {
                    "id": self._grasp_check_id,
                    "fruit_id": (-1 if self._grasp_fruit_id is None
                                  else self._grasp_fruit_id),
                    # 비전 입력은 팔이 움직이는 동안 계속 갱신된다. 검증은 시퀀스 시작 때
                    # 확정한 동일 과실 좌표만 사용해야 빈 공간/다른 과실을 검사하지 않는다.
                    "position": [float(v) for v in self._fruit_target],
                    "max_distance": float(self.get_parameter(
                        "grasp_tcp_max_distance_m").value),
                }
            })))
        if not self._deadline_ns:
            return
        if now <= self._deadline_ns:
            return
        self._deadline_ns = 0
        if bool(self.get_parameter("home_after_attempt").value):
            self._abort_to_home("timeout")
        else:
            self._transition("ERROR_TIMEOUT", stop=True)

    def _is_stale(self, msg: PoseStamped) -> bool:
        # stamp=0은 ros2 topic pub 등 수동 시험을 허용한다.
        if msg.header.stamp.sec == 0 and msg.header.stamp.nanosec == 0:
            return False
        age = (self.get_clock().now() - rclpy.time.Time.from_msg(msg.header.stamp))
        return age.nanoseconds * 1e-9 > float(
            self.get_parameter("max_target_age_sec").value
        )

    def _inside_workspace(self, target: PoseStamped) -> bool:
        lower = list(self.get_parameter("workspace_min").value)
        upper = list(self.get_parameter("workspace_max").value)
        p = target.pose.position
        values = (p.x, p.y, p.z)
        return all(
            math.isfinite(value) and low <= value <= high
            for value, low, high in zip(values, lower, upper)
        )

    def _is_jump(self, target: PoseStamped) -> bool:
        if self._last_position is None:
            return False
        p = target.pose.position
        distance = math.dist(self._last_position, (p.x, p.y, p.z))
        return distance > float(self.get_parameter("max_jump_m").value)


def main():
    rclpy.init()
    node = ManipulatorTargetNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
