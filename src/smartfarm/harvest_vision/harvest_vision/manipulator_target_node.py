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
from std_msgs.msg import Bool, Float64, String

# PoseStamped 변환 등록을 위한 side effect import. Buffer.transform API를 사용하면
# Humble(geometry2 0.25)과 Jazzy(geometry2 0.36)의 helper 함수 차이를 피할 수 있다.
import tf2_geometry_msgs  # noqa: F401
from tf2_ros import Buffer, TransformException, TransformListener

ACTIVE_SEQUENCE_STATES = {
    "PREGRASP", "GRASP", "GRIPPER_CLOSING", "CUTTING", "RETRACT",
    "WAIT_BASKET", "BASKET_APPROACH", "BASKET_PLACE", "PLACE_RELEASING",
    "GO_HOME",
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
        self.declare_parameter("blade_command_topic", "/harvester_0/blade_command")
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
        # YOLO 없이 맵의 시뮬 좌표로 **젤 가까운 토마토를 바로** 잡는다(검출 불필요, 집기 1순위).
        # 켜지면 비전(class/pose) 트리거를 무시하고 watchdog 이 직접 시퀀스를 시작한다.
        self.declare_parameter("direct_sim_grasp", False)
        self.declare_parameter("sim_tomato_topic", "/harvester_0/sim/tomato")
        self.declare_parameter("sim_match_radius_m", 0.35)
        self.declare_parameter(
            "mobility_ready_topic", "/harvester_0/manipulator/mobility_ready"
        )
        self.declare_parameter("base_frame", "harvester_0/base_link")
        self.declare_parameter("command_enabled", False)
        self.declare_parameter("max_target_age_sec", 0.5)
        self.declare_parameter("tf_timeout_sec", 0.2)
        self.declare_parameter("max_jump_m", 0.15)
        self.declare_parameter("auto_grasp_enabled", True)
        self.declare_parameter("pregrasp_clearance_m", 0.15)
        # 파지점이 과실보다 살짝 위로 잡혀서("살짝 위를 잡음", 2026-07-22) 목표 Z 를 조금
        # 내린다(음수=아래). 접근·파지 둘 다 같은 만큼 내려 직선 접근을 유지.
        self.declare_parameter("grasp_z_offset_m", -0.02)
        self.declare_parameter("tool_grasp_reach_m", 0.115)
        self.declare_parameter("motion_timeout_sec", 10.0)
        self.declare_parameter("blade_close_delay_sec", 0.6)
        self.declare_parameter("gripper_close_settle_sec", 1.0)
        self.declare_parameter("cut_match_tolerance_m", 0.10)
        self.declare_parameter("basket_approach_height_m", 0.15)
        self.declare_parameter("basket_workspace_min", [-0.80, -0.80, 0.15])
        self.declare_parameter("basket_workspace_max", [1.25, 0.80, 1.80])
        self.declare_parameter("workspace_min", [0.15, -0.75, 0.15])
        self.declare_parameter("workspace_max", [1.25, 0.75, 1.80])
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
        self._grasp_target = np.zeros(3, dtype=float)
        self._pregrasp_target = np.zeros(3, dtype=float)
        self._cut_id = 0
        self._cut_command_at_ns = 0
        self._cut_sent = False
        self._gripper_command_at_ns = 0
        self._basket_place: np.ndarray | None = None
        self._sim_fruits: list[tuple[np.ndarray, int]] = []

        input_topic = str(self.get_parameter("input_topic").value)
        validated_topic = str(self.get_parameter("validated_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        command_topic = str(self.get_parameter("isaac_command_topic").value)
        blade_topic = str(self.get_parameter("blade_command_topic").value)
        class_topic = str(self.get_parameter("target_class_topic").value)
        state_topic = str(self.get_parameter("state_topic").value)
        status_topic = str(self.get_parameter("rmp_status_topic").value)
        basket_topic = str(self.get_parameter("basket_pose_topic").value)
        enable_topic = str(self.get_parameter("harvest_enable_topic").value)
        mobility_topic = str(self.get_parameter("mobility_ready_topic").value)
        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._validated_pub = self.create_publisher(PoseStamped, validated_topic, 10)
        self._command_pub = self.create_publisher(PoseStamped, output_topic, 10)
        self._isaac_command_pub = self.create_publisher(String, command_topic, 10)
        self._blade_command_pub = self.create_publisher(Float64, blade_topic, 10)
        # 상태는 전이할 때만 발행하므로 늦게 붙은 디버거도 마지막 값을 받게 latch한다.
        self._state_pub = self.create_publisher(String, state_topic, latched_qos)
        self._mobility_pub = self.create_publisher(
            Bool, mobility_topic, latched_qos)
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
        if bool(self.get_parameter("direct_sim_grasp").value):
            return   # YOLO 무시 — 시뮬 좌표로 직접 파지(watchdog)
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
        if bool(self.get_parameter("direct_sim_grasp").value):
            return   # YOLO 무시 — 시뮬 좌표로 직접 파지(watchdog)
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

    def _nearest_sim_fruit(self) -> "np.ndarray | None":
        """맵의 시뮬 과실 중 베이스에서 젤 가까운 것(base_frame 좌표)."""
        now = self.get_clock().now().nanoseconds
        fresh = [pos for pos, stamp in self._sim_fruits
                 if now - stamp <= int(30.0e9)]
        if not fresh:
            return None
        return min(fresh, key=lambda p: float(np.linalg.norm(p)))

    def _start_direct_grasp(self, target: np.ndarray) -> None:
        """YOLO 없이 시뮬 좌표로 바로 파지. 접근 = 베이스에서 과실로 수평(카메라 광선 불필요)."""
        self._mobility_pub.publish(Bool(data=False))
        horiz = np.array([target[0], target[1], 0.0])
        n = float(np.linalg.norm(horiz))
        approach_dir = horiz / n if n > 1e-6 else np.array([1.0, 0.0, 0.0])
        clearance = float(self.get_parameter("pregrasp_clearance_m").value)
        z_off = float(self.get_parameter("grasp_z_offset_m").value)
        pregrasp = target - approach_dir * clearance
        pregrasp[2] += z_off
        self._pregrasp_target = pregrasp
        self._grasp_target = target.copy()
        self._grasp_target[2] += z_off
        self._latest_target = tuple(float(v) for v in target)
        self.get_logger().info(
            "직접 파지(YOLO 없음) 젤 가까운 토마토 "
            f"target=({target[0]:.3f}, {target[1]:.3f}, {target[2]:.3f})")
        self._isaac_command_pub.publish(
            String(data=json.dumps({"gripper": {"closed": False}})))
        self._send_rmp_goal(pregrasp, "PREGRASP")

    def _start_grasp_sequence(self) -> None:
        if self._latest_target is None or self._latest_camera is None:
            self._transition("ERROR_NO_TARGET", stop=True)
            return
        target = np.asarray(self._latest_target, dtype=float)
        if bool(self.get_parameter("use_sim_ground_truth").value):
            sim_target = self._match_sim_tomato(target, np.asarray(
                self._latest_camera, dtype=float))
            if sim_target is None:
                # 후보가 round-robin 토픽으로 더 들어오거나 다음 검출 프레임에서
                # 광선이 안정되면 자동 재시도한다. 일시 실패로 수확 게이트를 닫지 않는다.
                self._transition("WAIT_SIM_MATCH", stop=True)
                return
            self.get_logger().info(
                "비전 검출을 시뮬 토마토 좌표에 매칭: "
                f"vision=({target[0]:.3f}, {target[1]:.3f}, {target[2]:.3f}) -> "
                f"sim=({sim_target[0]:.3f}, {sim_target[1]:.3f}, {sim_target[2]:.3f})")
            target = sim_target
            self._latest_target = tuple(float(v) for v in target)
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
        z_off = float(self.get_parameter("grasp_z_offset_m").value)
        pregrasp = target - ray * clearance
        pregrasp[2] += z_off
        self._pregrasp_target = pregrasp
        self._grasp_target = target.copy()
        self._grasp_target[2] += z_off
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
            position = np.asarray(item["position"], dtype=float)
        except (KeyError, TypeError, ValueError):
            return
        if item.get("class") != "ripe" or position.shape != (3,):
            return
        now = self.get_clock().now().nanoseconds
        for index, (known, _) in enumerate(self._sim_fruits):
            if float(np.linalg.norm(known - position)) < 0.02:
                self._sim_fruits[index] = (position, now)
                break
        else:
            self._sim_fruits.append((position, now))
        self._sim_fruits = [
            entry for entry in self._sim_fruits
            if now - entry[1] <= int(30.0e9)]

    def _match_sim_tomato(
        self, vision_target: np.ndarray, camera: np.ndarray
    ) -> np.ndarray | None:
        now = self.get_clock().now().nanoseconds
        fresh = [position for position, stamp in self._sim_fruits
                 if now - stamp <= int(30.0e9)]
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
        for position in fresh:
            relative = position - camera
            along = float(np.dot(relative, ray))
            if along <= 0.0:
                continue
            lateral = float(np.linalg.norm(relative - ray * along))
            # 같은 광선상 과실이 여럿이면 비전의 대략 깊이에 가까운 것을 우선한다.
            score = lateral + 0.05 * abs(along - vision_depth)
            scored.append((score, lateral, position))
        if not scored:
            return None
        _, lateral, nearest = min(scored, key=lambda item: item[0])
        if lateral > float(self.get_parameter("sim_match_radius_m").value):
            self.get_logger().warning(
                f"시뮬 토마토 광선 매칭 거리 초과: {lateral:.3f}m")
            return None
        return nearest.copy()

    def _send_rmp_goal(self, position: np.ndarray, phase: str) -> None:
        self._sequence_id += 1
        self._pending_id = self._sequence_id
        timeout = float(self.get_parameter("motion_timeout_sec").value)
        self._deadline_ns = self.get_clock().now().nanoseconds + int(timeout * 1e9)
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
        if self._state == "GRIPPER_CLOSING":
            try:
                gripper = float(status.get("gripper", 0.0))
            except (TypeError, ValueError):
                return
            if gripper >= 0.72:
                self._begin_cut()
            return
        if self._state == "PLACE_RELEASING":
            try:
                gripper = float(status.get("gripper", 1.0))
            except (TypeError, ValueError):
                return
            if gripper <= 0.08:
                self._send_home()
            return
        if self._state == "CUTTING":
            try:
                cut_id = int(status.get("cut_id", -1))
            except (TypeError, ValueError):
                return
            if cut_id != self._cut_id:
                return
            self._blade_command_pub.publish(Float64(data=0.0))
            if bool(status.get("cut_success", False)):
                self._cut_command_at_ns = 0
                self._send_rmp_goal(self._pregrasp_target, "RETRACT")
            else:
                self._deadline_ns = 0
                if bool(self.get_parameter("home_after_attempt").value):
                    self._abort_to_home("cut")
                else:
                    self._transition("ERROR_CUT", stop=True)
            return
        try:
            status_id = int(status.get("id", -1))
        except (TypeError, ValueError):
            return
        if status_id != self._pending_id:
            return
        if not bool(status.get("reached", False)):
            return
        if self._state == "PREGRASP":
            self._send_rmp_goal(self._grasp_target, "GRASP")
        elif self._state == "GRASP":
            self._transition("GRIPPER_CLOSING", stop=True)
            self._gripper_command_at_ns = self.get_clock().now().nanoseconds
            self._deadline_ns = (
                self.get_clock().now().nanoseconds
                + int(float(self.get_parameter("motion_timeout_sec").value) * 1e9)
            )
            self._isaac_command_pub.publish(
                String(data=json.dumps({"gripper": {"closed": True}}))
            )
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
            self._transition("HOME_READY", stop=True)
            self._mobility_pub.publish(Bool(data=True))
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

    def _send_home(self) -> None:
        self._basket_place = None
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
        if self._state == "GO_HOME":
            self._deadline_ns = 0
            self._transition("HOME_READY", stop=True)
            self._mobility_pub.publish(Bool(data=True))
            self._maybe_single_shot_off()
            return
        self._send_home()

    def _begin_cut(self) -> None:
        self._gripper_command_at_ns = 0
        self._transition("CUTTING", stop=True)
        self._cut_sent = False
        delay = float(self.get_parameter("blade_close_delay_sec").value)
        now = self.get_clock().now().nanoseconds
        self._cut_command_at_ns = now + int(delay * 1e9)
        self._deadline_ns = now + int(
            float(self.get_parameter("motion_timeout_sec").value) * 1e9)
        self._blade_command_pub.publish(Float64(data=35.0))

    def _watchdog(self) -> None:
        now = self.get_clock().now().nanoseconds
        # YOLO 없이: 수확 켜졌고 시퀀스 안 돌고 있으면 젤 가까운 시뮬 과실을 바로 잡는다.
        if (bool(self.get_parameter("direct_sim_grasp").value)
                and self._harvest_enabled
                and self._state not in ACTIVE_SEQUENCE_STATES
                and bool(self.get_parameter("command_enabled").value)
                and bool(self.get_parameter("auto_grasp_enabled").value)):
            nearest = self._nearest_sim_fruit()
            if nearest is not None:
                self._start_direct_grasp(nearest)
                return
        # 토마토를 정상적으로 물면 손가락은 완전 닫힘(0.8 rad)에 도달하지 않는다.
        # 닫기 목표를 계속 유지한 채 정착 시간이 지나면 파지된 것으로 보고 절단한다.
        if (self._state == "GRIPPER_CLOSING"
                and self._gripper_command_at_ns
                and now - self._gripper_command_at_ns >= int(float(
                    self.get_parameter("gripper_close_settle_sec").value) * 1e9)):
            self._begin_cut()
        if (self._state == "CUTTING" and not self._cut_sent
                and self._cut_command_at_ns and now >= self._cut_command_at_ns):
            self._sequence_id += 1
            self._cut_id = self._sequence_id
            self._cut_sent = True
            request = {
                "cut_fruit": {
                    "id": self._cut_id,
                    "position": list(self._latest_target),
                    "max_distance": float(
                        self.get_parameter("cut_match_tolerance_m").value),
                }
            }
            self._isaac_command_pub.publish(String(data=json.dumps(request)))
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
