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
from rclpy.time import Time
from std_msgs.msg import Bool, String

# PoseStamped 변환 등록을 위한 side effect import. Buffer.transform API를 사용하면
# Humble(geometry2 0.25)과 Jazzy(geometry2 0.36)의 helper 함수 차이를 피할 수 있다.
import tf2_geometry_msgs  # noqa: F401
from tf2_ros import Buffer, TransformException, TransformListener

ACTIVE_SEQUENCE_STATES = {
    "APPROACH", "PREGRASP", "GRASP",
    "CAPTURE_TRIM", "GRASP_YAW_CORRECT",
    "GRIPPER_CLOSING", "GRASP_VERIFY",
    "CUTTING", "CUT_VERIFY", "BLADE_OPENING",
    "VERIFY_RETRACT", "GRASP_FOLLOW_CHECK",
    "RETRACT_CIRC", "RETRACT_LIN",
    "RETRACT", "PRE_PLACE",
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
        # IW 슬롯 선택기가 최근에 발행한 실제 좌표만 사용한다. 오래된 좌표를 들고
        # 이미 떠난 IW를 향해 팔이 다시 움직이지 않도록 유효시간을 둔다.
        self.declare_parameter("basket_pose_max_age_sec", 2.0)
        self.declare_parameter("use_iw_tf_basket_fallback", True)
        self.declare_parameter("iw_base_frame", "iwhub_0/base_link")
        # IwHub cargo의 실제 4×2 KLT 격자 중 초기 적재가 없는 슬롯.
        # [x,y,z]는 IW base_link 기준 release pose이며 z=KLT 윗면+약 5 cm다.
        self.declare_parameter("iw_empty_basket_offsets", [
            -0.465, 0.125, 0.52,
            -0.155, -0.125, 0.52,
            0.155, -0.125, 0.52,
            0.155, 0.125, 0.52,
            0.465, 0.125, 0.52,
        ])
        # 상대 이름이어야 namespace=harvester_0에서 코디네이터가 발행하는
        # /harvester_0/harvest_test/enable과 동일한 토픽으로 해석된다.
        self.declare_parameter("harvest_enable_topic", "harvest_test/enable")
        self.declare_parameter("external_harvest_gate_enabled", False)
        self.declare_parameter("use_sim_ground_truth", False)
        self.declare_parameter("sim_tomato_topic", "/harvester_0/sim/tomato")
        self.declare_parameter("sim_match_radius_m", 0.35)
        # 시뮬 통합시험에서는 검출 광선 매칭이 일시적으로 실패해도, 검출점과 가장
        # 가까운 fresh GT 과실을 선택해 수확을 계속한다.
        self.declare_parameter("direct_sim_grasp", False)
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
        self.declare_parameter("nav_reposition_enabled", True)
        # 비전→GT 매칭의 mm 단위 흔들림 때문에 경계 바로 바깥의 실제 도달 가능
        # 목표를 재정차로 넘기지 않도록 최종 workspace 검사에만 작은 여유를 둔다.
        self.declare_parameter("workspace_boundary_tolerance_m", 0.02)
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
        # 선택 시점의 원호 하강 경로 기본값.
        self.declare_parameter("circ_entry_back_m", 0.17)
        # 수평 시작점은 과실 중심에서 17cm 뒤에 둬 충분한 CIRC 원호를
        # 확보한다. radius_scale=0이면 반지름만큼 앞으로 당기지 않는다.
        self.declare_parameter("circ_entry_radius_scale", 0.0)
        # 수평 진입점 계산용 레거시 값. 최종 Z는 아래에서 명시적으로 덮어쓴다.
        self.declare_parameter("circ_entry_radius_lift_scale", -1.0)
        self.declare_parameter("circ_entry_min_back_m", 0.06)
        self.declare_parameter("circ_entry_drop_m", 0.10)
        # PREGRASP는 최종점보다 5cm 위, CIRC 보조점은 정확히 중간인 2.5cm 위.
        self.declare_parameter("circ_vertical_descent_m", 0.05)
        self.declare_parameter("lin_approach_m", 0.10)
        # CIRC 중 TCP가 안전해도 스쿱 외곽이 과실에 박히지 않도록 실제 CAD 외경과
        # 물리 과실 반경을 합친 swept envelope 바깥에 보조점을 둔다.
        self.declare_parameter("scoop_outer_radius_m", 0.057)
        self.declare_parameter("circ_clearance_margin_m", 0.019)
        # 최종 스쿱 높이를 올려도 안전점까지 같이 올라가면 link_2가 베이스를 가로지르는
        # PTP 경로가 생긴다. 접근점은 현장에서 성공한 +2.5cm 높이를 별도로 유지한다.
        self.declare_parameter("approach_vertical_offset_m", 0.025)
        # 실제 스쿱 수용 중심이 HarvestTCP 표시점보다 위에 있어, 최종 CIRC 목표를
        # 과실 기하 중심보다 8cm 올린다(현장 수용 결과 2026-07-24).
        self.declare_parameter("grasp_vertical_offset_m", 0.08)
        # 고정 8cm에 실제 토마토 높이의 절반을 더해 스쿱 안쪽까지 퍼 올린다.
        self.declare_parameter("grasp_half_fruit_height_scale", 1.0)
        self.declare_parameter("sim_fruit_height_fallback_m", 0.068)
        self.declare_parameter("sim_fruit_radius_fallback_m", 0.034)
        # ── 스쿱 축방향 삽입(2026-07-24 재설계) ──
        # harvest_tcp(=컵 회전중심)를 과실 중심에 오프셋 0으로 포갠다. 닫힘=오른쪽-아래,
        # 개구=왼쪽-위이므로 삽입축 = up(+Z) 가중 + left(정면 기준 왼쪽) 가중의 단위벡터.
        # RViz에서 실제 스쿱 개구 방향과 안 맞으면 두 가중치를 조정(left<0 = 오른쪽).
        self.declare_parameter("scoop_open_up_weight", 1.0)
        self.declare_parameter("scoop_open_left_weight", 0.5)
        # 스쿱 컵 내경(메시 ~47~52mm). 삽입 시작 여유 = cup + 과실반경 + margin.
        self.declare_parameter("scoop_cup_radius_m", 0.050)
        self.declare_parameter("scoop_insertion_margin_m", 0.010)
        self.declare_parameter("scoop_safe_back_m", 0.100)
        self.declare_parameter("reacquire_max_m", 0.12)
        self.declare_parameter("capture_abort_m", 0.07)
        self.declare_parameter("capture_deadband_m", 0.008)
        self.declare_parameter("capture_trim_max_m", 0.035)
        # RMPflow가 고정된 과실 중심을 관통하려 하면 collider 표면에서 3~4cm 오차로
        # 정체된다. 열린 그리퍼는 과실 표면까지 보내고 그 위치에서 닫는다.
        self.declare_parameter("grasp_surface_standoff_m", 0.034)
        # USD HarvestTCP와 같은 값. 현재 제어는 USD TCP를 직접 측정하지만 외부 launch가
        # 이 파라미터를 참조해도 서로 다른 오프셋을 사용하지 않도록 동기화한다.
        self.declare_parameter("tool_grasp_reach_m", 0.132)
        self.declare_parameter("motion_timeout_sec", 10.0)
        self.declare_parameter("gripper_close_settle_sec", 1.5)
        self.declare_parameter("blade_cut_deg", 50.0)
        # +50° 명령은 셸 접촉 때문에 실각 약 41°에 안정된다. 기존 로직과 같이
        # 40° 도달을 절삭 위치로 판정한다.
        self.declare_parameter("blade_cut_complete_deg", 40.0)
        self.declare_parameter("blade_motion_timeout_sec", 8.0)
        self.declare_parameter("blade_open_deg", 0.0)
        self.declare_parameter("blade_angle_tolerance_deg", 1.0)
        self.declare_parameter("grasp_tcp_max_distance_m", 0.06)
        self.declare_parameter("grasp_verify_retract_m", 0.03)
        self.declare_parameter("grasp_follow_max_delta_m", 0.015)
        self.declare_parameter("grasp_one_side_yaw_deg", 5.0)
        self.declare_parameter("grasp_one_side_max_retries", 1)
        self.declare_parameter("basket_approach_height_m", 0.15)
        self.declare_parameter("basket_workspace_min", [-0.80, -0.80, 0.15])
        self.declare_parameter("basket_workspace_max", [1.35, 0.80, 1.80])
        self.declare_parameter("workspace_min", [0.15, -1.05, 0.15])
        self.declare_parameter("workspace_max", [1.25, 1.05, 1.80])
        # 데모: 성공/실패 무관 매 시도 후 홈 복귀 → 팔이 안 굳고 다음 과실을 계속 시도한다.
        self.declare_parameter("home_after_attempt", True)
        # h 원샷: 한 사이클(인식→수확→홈) 끝나면 게이트를 스스로 끈다. 계속 재인식·재시작
        # 하지 않고 h 를 다시 눌러야 다음 과실을 잡는다.
        self.declare_parameter("single_shot_harvest", True)
        # 실패 후에도 명시적 요청 없이 새 과실을 자동으로 쫓지 않는다.
        self.declare_parameter("retry_after_failure", False)

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
        self._circ_target = np.zeros(3, dtype=float)
        self._insertion_axis = np.array([0.0, 0.0, 1.0])
        self._fruit_target = np.zeros(3, dtype=float)
        self._grasp_fruit_id: int | None = None
        self._reposition_fruit_id: int | None = None
        self._reposition_requested_ns = 0
        self._pregrasp_target = np.zeros(3, dtype=float)
        self._approach_target = np.zeros(3, dtype=float)
        self._circ_interim = np.zeros(3, dtype=float)
        self._harvest_orientation: list[float] | None = None
        self._gripper_command_at_ns = 0
        self._grasp_check_id = 0
        self._grasp_check_sent = False
        self._cut_check_id = 0
        self._cut_check_sent = False
        self._grasp_yaw_retry_count = 0
        self._follow_check_id = 0
        self._basket_place: np.ndarray | None = None
        self._basket_received_ns = 0
        self._sim_fruits: dict[int, tuple[np.ndarray, int]] = {}
        self._sim_fruit_heights: dict[int, float] = {}
        self._sim_fruit_radii: dict[int, float] = {}
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
        gated = bool(
            self.get_parameter("external_harvest_gate_enabled").value)
        self.get_logger().info(
            f"비전→매니퓰레이터 좌표 브리지 시작: {input_topic} -> "
            f"{self.get_parameter('base_frame').value} "
            f"(command_enabled={enabled}, external_gate={gated}, "
            f"enable_topic={enable_topic})"
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
            if self._state in {
                    "PREGRASP", "GRASP", "GRIPPER_CLOSING"
            } and not target_class:
                self._deadline_ns = 0
                self._transition("ABORT_TARGET_LOST", stop=True)
            elif self._state in {
                    "PREGRASP", "GRASP", "GRIPPER_CLOSING"
            } and target_class == "spoiled":
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
            # GT를 기다리는 동안 class 토픽이 고주기로 들어와도 상태를
            # RIPE_READY↔WAIT_SIM_MATCH로 계속 뒤집지 않는다. 새 GT가 들어오는
            # _sim_tomato_callback이 즉시 수확 시퀀스를 다시 시작한다.
            if (self._state == "WAIT_SIM_MATCH"
                    and bool(self.get_parameter("use_sim_ground_truth").value)):
                return
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
        self.get_logger().info(
            "외부 수확 게이트 수신: "
            + ("OPEN" if self._harvest_enabled else "CLOSED"))
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
                if (sim_match is None
                        and bool(self.get_parameter("direct_sim_grasp").value)):
                    sim_match = self._nearest_sim_tomato(target)
                    if sim_match is not None:
                        self.get_logger().info(
                            "광선 매칭 대신 검출점 최근접 시뮬 토마토를 선택")
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
        boundary_tolerance = max(
            0.0, float(self.get_parameter(
                "workspace_boundary_tolerance_m").value))
        if (not np.all(np.isfinite(target))
                or not np.all(
                    (lower - boundary_tolerance <= target)
                    & (target <= upper + boundary_tolerance))
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
            if not bool(
                    self.get_parameter("nav_reposition_enabled").value):
                self._transition("ERROR_TARGET_OUT_OF_REACH", stop=True)
                self.get_logger().error(
                    "고정 수확 모드라 Nav2 재접근을 금지함: "
                    f"target=({target[0]:.3f}, {target[1]:.3f}, "
                    f"{target[2]:.3f}), xy_radius={xy_radius:.3f}m")
                return
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
        # 스쿱 축방향 삽입: harvest_tcp(=컵 회전중심)를 과실 중심에 오프셋 0으로 포갠다.
        # 닫힌 쪽(오른쪽-아래)에서 개구축(왼쪽-위)을 따라 대각선 직선으로 밀어 넣는다.
        self._harvest_orientation = self._current_tool_orientation()
        fruit_radius = self._grasp_fruit_radius()
        n = self._scoop_insertion_axis(target)
        self._insertion_axis = n
        standoff = (
            float(self.get_parameter("scoop_cup_radius_m").value)
            + fruit_radius
            + float(self.get_parameter("scoop_insertion_margin_m").value))
        safe_back = float(self.get_parameter("scoop_safe_back_m").value)
        grasp = target.copy()                       # 컵 중심 = 과실 중심 (오프셋 0)
        circ_target = grasp.copy()                  # 후퇴 경로 호환(직선 역주행)
        pregrasp = target - n * standoff            # 개구 바깥 삽입 시작점
        approach = target - n * (standoff + safe_back)   # OMPL 안전점
        interim = 0.5 * (pregrasp + grasp)          # CIRC 미사용, 필드 자리채움
        self._approach_target = approach
        self._pregrasp_target = pregrasp
        self._grasp_target = grasp
        self._circ_target = circ_target
        self._circ_interim = interim
        self._fruit_target = target.copy()
        self._grasp_yaw_retry_count = 0
        # 파지 전 그리퍼를 연다 — 닫힌 채로 다가가면 손가락이 과실을 못 감싼다.
        self._isaac_command_pub.publish(
            String(data=json.dumps({
                "gripper": {"closed": False},
                "blade": float(self.get_parameter("blade_open_deg").value),
            })))
        self.get_logger().info(
            "스쿱 축삽입 접근: "
            f"safe={tuple(round(float(v), 3) for v in approach)} → "
            f"pre={tuple(round(float(v), 3) for v in pregrasp)} → "
            f"seat=T{tuple(round(float(v), 3) for v in grasp)}, "
            f"standoff={standoff:.3f}m, r={fruit_radius:.3f}m, "
            f"axis=({n[0]:.2f},{n[1]:.2f},{n[2]:.2f})")
        self._send_rmp_goal(approach, "APPROACH")

    def _sim_tomato_callback(self, msg: String) -> None:
        try:
            item = json.loads(msg.data)
            if not isinstance(item, dict):
                return
            fruit_id = int(item["id"])
            position = np.asarray(item["position"], dtype=float)
            fruit_height = float(item.get(
                "height",
                self.get_parameter("sim_fruit_height_fallback_m").value))
            fruit_radius = float(item.get(
                "radius",
                self.get_parameter("sim_fruit_radius_fallback_m").value))
        except (KeyError, TypeError, ValueError):
            return
        if (item.get("class") != "ripe" or position.shape != (3,)
                or not math.isfinite(fruit_height)
                or fruit_height <= 0.0
                or not math.isfinite(fruit_radius)
                or fruit_radius <= 0.0):
            return
        now = self.get_clock().now().nanoseconds
        self._sim_fruits[fruit_id] = (position, now)
        self._sim_fruit_heights[fruit_id] = fruit_height
        self._sim_fruit_radii[fruit_id] = fruit_radius
        self._sim_fruits = {
            key: entry for key, entry in self._sim_fruits.items()
            if now - entry[1] <= int(30.0e9)}
        self._sim_fruit_heights = {
            key: height for key, height in self._sim_fruit_heights.items()
            if key in self._sim_fruits}
        self._sim_fruit_radii = {
            key: radius for key, radius in self._sim_fruit_radii.items()
            if key in self._sim_fruits}
        # class 콜백에서 GT가 아직 없어서 WAIT_SIM_MATCH로 들어간 경우, 다음 영상
        # 프레임을 기다리지 말고 GT 수신 자체를 재시도 트리거로 사용한다.
        if (self._state == "WAIT_SIM_MATCH"
                and self._harvest_enabled
                and self._target_class in ("tomato", "ripe")
                and self._latest_target is not None
                and self._latest_camera is not None):
            self._start_grasp_sequence()

    def _grasp_vertical_lift(self) -> float:
        """기존 고정 상승량 + 현재 과실 실제 높이의 절반."""
        fixed = float(self.get_parameter("grasp_vertical_offset_m").value)
        if not bool(self.get_parameter("use_sim_ground_truth").value):
            return fixed
        height = self._sim_fruit_heights.get(
            self._grasp_fruit_id,
            float(self.get_parameter("sim_fruit_height_fallback_m").value))
        scale = max(
            0.0, float(self.get_parameter(
                "grasp_half_fruit_height_scale").value))
        return fixed + 0.5 * height * scale

    def _grasp_fruit_radius(self) -> float:
        """현재 과실의 실제 수평 bbox 반지름."""
        return self._sim_fruit_radii.get(
            self._grasp_fruit_id,
            float(self.get_parameter("sim_fruit_radius_fallback_m").value))

    def _outward_circ_interim(
        self,
        start: np.ndarray,
        end: np.ndarray,
        fruit_center: np.ndarray,
    ) -> np.ndarray:
        """P1→P2→P3 전체가 한 원호가 되도록 CIRC 보조점을 계산한다."""
        start_ray = start - fruit_center
        end_ray = end - fruit_center
        start_norm = float(np.linalg.norm(start_ray))
        end_norm = float(np.linalg.norm(end_ray))
        if start_norm < 1e-8:
            return 0.5 * (start + end)
        if end_norm < 1e-8:
            # 최종 HarvestTCP가 과실 중심 자체인 현재 규약. 단순 중점을 쓰면
            # P1/P2/P3가 공선이 되어 CIRC가 퇴화한다. P1에서 과실 쪽 수평 접선으로
            # 출발하는 원을 구성하고 그 원의 중간각 점을 P2로 사용한다.
            horizontal = fruit_center - start
            horizontal[2] = 0.0
            chord_xy = float(np.linalg.norm(horizontal))
            start_z = float(start[2] - fruit_center[2])
            if chord_xy < 1e-8 or abs(start_z) < 1e-8:
                interim = 0.5 * (start + end)
                interim[2] -= self._grasp_fruit_radius()
                return interim
            # 원 중심은 P1의 수직선 위에 있고 P1/P3에서 같은 거리에 있다.
            center_z = (
                start_z * start_z - chord_xy * chord_xy
            ) / (2.0 * start_z)
            center = start.copy()
            center[2] = fruit_center[2] + center_z
            start_unit = start - center
            end_unit = end - center
            radius = float(np.linalg.norm(start_unit))
            start_unit /= radius
            end_unit /= float(np.linalg.norm(end_unit))
            bisector = start_unit + end_unit
            bisector_norm = float(np.linalg.norm(bisector))
            if bisector_norm < 1e-8:
                interim = 0.5 * (start + end)
                interim[2] -= self._grasp_fruit_radius()
                return interim
            return center + radius * bisector / bisector_norm
        direction = start_ray / start_norm + end_ray / end_norm
        direction_norm = float(np.linalg.norm(direction))
        if direction_norm < 1e-8:
            return 0.5 * (start + end)
        waypoint_radius = (
            self._grasp_fruit_radius()
            + float(self.get_parameter("scoop_outer_radius_m").value)
            + float(self.get_parameter("circ_clearance_margin_m").value)
        )
        return fruit_center + direction / direction_norm * waypoint_radius

    def _nearest_sim_tomato(
        self, vision_target: np.ndarray
    ) -> tuple[np.ndarray, int] | None:
        """fresh GT 중 검출된 3D 점에 가장 가까운 과실을 선택한다."""
        now = self.get_clock().now().nanoseconds
        fresh = [
            (float(np.linalg.norm(position - vision_target)), fruit_id, position)
            for fruit_id, (position, stamp) in self._sim_fruits.items()
            if now - stamp <= int(30.0e9)
        ]
        if not fresh:
            return None
        _, fruit_id, position = min(fresh, key=lambda item: item[0])
        return position.copy(), fruit_id

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
        if phase == "APPROACH":
            # 베드뷰 방향을 유지한 채 충돌회피한다. motion bridge가 현재 joint_1
            # 주변으로 OMPL 경로를 제한해 제자리에서 크게 도는 IK 해를 차단한다.
            command["rmp_target"]["motion"] = "OMPL"
        elif phase in {"GRASP", "RETRACT_CIRC"}:
            # 축방향 직선 삽입/후퇴 — CIRC 원호를 쓰지 않는다.
            command["rmp_target"]["motion"] = "LIN"
            if phase == "GRASP":
                command["rmp_target"]["velocity_scale"] = 0.05
        elif phase in {
            "PREGRASP", "CAPTURE_TRIM", "RETRACT_LIN",
        }:
            command["rmp_target"]["motion"] = "LIN"
            if phase == "PREGRASP":
                # 직선 진입 중에도 베드뷰의 1축 가지를 유지한다. 같은 TCP 자세의
                # 반대쪽 등가 IK를 골라 프리그랩에서 크게 도는 현상을 막는다.
                command["rmp_target"]["lock_joint_1"] = True
            if phase == "CAPTURE_TRIM":
                command["rmp_target"]["velocity_scale"] = 0.035
        else:
            command["rmp_target"]["motion"] = "PTP"
        # PREGRASP마다 카메라 광선으로 새 TCP 자세를 만들면 ±360° 범위의 두산 손목이
        # 먼 등가 IK 해를 골라 제자리에서 여러 번 회전할 수 있다. Nav 도착 뒤 이미
        # 베드를 향한 현재 스쿱 자세를 그대로 잠그고 위치만 목표로 이동한다.
        if phase in {
            "APPROACH", "PREGRASP", "GRASP",
            "CAPTURE_TRIM", "VERIFY_RETRACT",
            "RETRACT_CIRC", "RETRACT_LIN", "RETRACT",
            "BASKET_APPROACH", "BASKET_PLACE",
        }:
            orientation = self._harvest_orientation
            if orientation is None:
                orientation = self._current_tool_orientation()
            if orientation is not None:
                command["rmp_target"]["tool_orientation"] = orientation
        self.get_logger().info(
            f"매니퓰레이터 명령 id={self._pending_id} phase={phase} "
            f"target=({position[0]:.3f}, {position[1]:.3f}, {position[2]:.3f})"
        )
        self._isaac_command_pub.publish(String(data=json.dumps(command)))

    def _fresh_locked_sim_target(self) -> np.ndarray | None:
        """접근 중에도 갱신되는 동일 fruit_id의 최신 GT 중심만 반환한다."""
        if self._grasp_fruit_id is None:
            return None
        entry = self._sim_fruits.get(self._grasp_fruit_id)
        if entry is None:
            return None
        position, stamp = entry
        if self.get_clock().now().nanoseconds - stamp > int(2.0e9):
            return None
        return position.copy()

    def _scoop_insertion_axis(self, target: np.ndarray) -> np.ndarray:
        """스쿱 개구가 향하는 삽입축(단위, base). 닫힘=오른쪽아래→개구=왼쪽위.

        harvest_tcp는 이 축을 따라 아래-뒤(닫힌 쪽)에서 과실 중심으로 밀려 들어간다.
        """
        yaw = math.atan2(float(target[1]), float(target[0]))
        up = np.array([0.0, 0.0, 1.0])
        # 과실을 정면으로 볼 때의 왼쪽 수평 방향(=direction을 +90° 회전).
        left = np.array([-math.sin(yaw), math.cos(yaw), 0.0])
        n = (float(self.get_parameter("scoop_open_up_weight").value) * up
             + float(self.get_parameter("scoop_open_left_weight").value) * left)
        norm = float(np.linalg.norm(n))
        return up if norm < 1e-6 else n / norm

    def _refresh_entry_geometry(self) -> bool:
        """원본처럼 안전점 도착 뒤 같은 과실의 최신 중심으로 LIN/CIRC를 갱신한다."""
        live = self._fresh_locked_sim_target()
        if live is None:
            return True
        delta = live - self._fruit_target
        error = float(np.linalg.norm(delta))
        limit = float(self.get_parameter("reacquire_max_m").value)
        if error > limit:
            self._deadline_ns = 0
            self._abort_to_home(
                f"reacquire_shift={error:.3f}m > {limit:.3f}m")
            return False
        if error <= 0.003:
            return True
        n = self._scoop_insertion_axis(live)
        self._insertion_axis = n
        standoff = (
            float(self.get_parameter("scoop_cup_radius_m").value)
            + self._grasp_fruit_radius()
            + float(self.get_parameter("scoop_insertion_margin_m").value))
        grasp = live.copy()                     # 컵 중심 = 과실 중심 (오프셋 0)
        circ_target = grasp.copy()
        pregrasp = live - n * standoff
        interim = 0.5 * (pregrasp + grasp)
        self._pregrasp_target = pregrasp
        self._grasp_target = grasp
        self._circ_target = circ_target
        self._circ_interim = interim
        self._fruit_target = live.copy()
        self.get_logger().info(
            "안전점 도착 후 동일 과실 중심 갱신: "
            f"delta={tuple(round(float(v) * 1000.0) for v in delta)}mm")
        return True

    def _capture_trim_or_close(self, allow_trim: bool = True) -> None:
        """CIRC 중 움직인 과실만 원본 범위 안에서 짧게 LIN 보정하고 스쿱을 닫는다."""
        live = self._fresh_locked_sim_target()
        if live is not None:
            desired = live.copy()               # 컵 중심 = 과실 중심 (오프셋 0)
            delta = desired - self._grasp_target
            error = float(np.linalg.norm(delta))
            abort = float(self.get_parameter("capture_abort_m").value)
            deadband = float(self.get_parameter("capture_deadband_m").value)
            trim_max = float(self.get_parameter("capture_trim_max_m").value)
            if error > abort:
                self._deadline_ns = 0
                self._abort_to_home(
                    f"capture_shift={error:.3f}m > {abort:.3f}m")
                return
            if allow_trim and error > deadband:
                shift = delta * min(1.0, trim_max / error)
                trim_goal = self._grasp_target + shift
                self._grasp_target = trim_goal
                self._pregrasp_target = self._pregrasp_target + shift
                self._circ_interim = self._circ_interim + shift
                self._fruit_target = live.copy()
                self.get_logger().info(
                    "수용 직전 저속 LIN 보정: "
                    f"delta={tuple(round(float(v) * 1000.0) for v in shift)}mm")
                self._send_rmp_goal(trim_goal, "CAPTURE_TRIM")
                return
        self._transition("GRIPPER_CLOSING", stop=True)
        self._gripper_command_at_ns = self.get_clock().now().nanoseconds
        self._grasp_check_sent = False
        self._deadline_ns = (
            self.get_clock().now().nanoseconds
            + int(float(self.get_parameter("motion_timeout_sec").value) * 1e9)
        )
        self._isaac_command_pub.publish(
            String(data=json.dumps({"gripper": {"closed": True}})))

    def _current_tool_orientation(self) -> list[float] | None:
        """현재 harvest_tcp 자세. 한 수확 사이클 동안 같은 자세로 LIN/CIRC를 잇는다."""
        base_frame = str(self.get_parameter("base_frame").value)
        try:
            transform = self._buffer.lookup_transform(
                base_frame, "harvest_tcp", Time(),
                timeout=Duration(seconds=float(
                    self.get_parameter("tf_timeout_sec").value)),
            )
        except TransformException as exc:
            self.get_logger().warning(
                f"현재 TCP 자세 조회 실패: {exc}",
                throttle_duration_sec=2.0,
            )
            return None
        q = transform.transform.rotation
        return [float(q.x), float(q.y), float(q.z), float(q.w)]

    def _status_callback(self, msg: String) -> None:
        try:
            status = json.loads(msg.data)
        except (TypeError, ValueError):
            return
        if not isinstance(status, dict):
            return
        if self._state == "CUTTING" and not self._cut_check_sent:
            try:
                blade = float(status.get("blade", float("nan")))
            except (TypeError, ValueError):
                blade = float("nan")
            target = float(
                self.get_parameter("blade_cut_complete_deg").value)
            tolerance = float(
                self.get_parameter("blade_angle_tolerance_deg").value)
            if math.isfinite(blade) and blade >= target - tolerance:
                self._cut_check_sent = True
                self._sequence_id += 1
                self._cut_check_id = self._sequence_id
                self._transition("CUT_VERIFY", stop=True)
                self._isaac_command_pub.publish(String(data=json.dumps({
                    "cut_fruit": {
                        "id": self._cut_check_id,
                        "fruit_id": (-1 if self._grasp_fruit_id is None
                                     else self._grasp_fruit_id),
                        "position": [float(v) for v in self._fruit_target],
                        "max_distance": float(self.get_parameter(
                            "grasp_tcp_max_distance_m").value),
                    }
                })))
            return
        if "cut_id" in status:
            if (self._state != "CUT_VERIFY"
                    or int(status.get("cut_id", -1)) != self._cut_check_id):
                return
            if not bool(status.get("cut_success", False)):
                self._deadline_ns = 0
                self._abort_to_home(
                    f"cut_failed blade={status.get('blade')} "
                    f"distance={status.get('d')}")
                return
            self.get_logger().info(
                f"칼날 절단 확인: blade={float(status.get('blade', 0.0)):.1f}°")
            self._transition("BLADE_OPENING", stop=True)
            self._deadline_ns = (
                self.get_clock().now().nanoseconds
                + int(float(self.get_parameter("motion_timeout_sec").value) * 1e9)
            )
            self._isaac_command_pub.publish(String(data=json.dumps({
                "blade": float(self.get_parameter("blade_open_deg").value),
            })))
            return
        if self._state == "BLADE_OPENING":
            try:
                blade = float(status.get("blade", float("nan")))
            except (TypeError, ValueError):
                blade = float("nan")
            target = float(self.get_parameter("blade_open_deg").value)
            tolerance = float(
                self.get_parameter("blade_angle_tolerance_deg").value)
            if math.isfinite(blade) and abs(blade - target) <= tolerance:
                self._deadline_ns = 0
                self._send_rmp_goal(
                    self._pregrasp_target, "RETRACT_CIRC")
            return
        if "grasp_id" in status:
            if (self._state != "GRASP_VERIFY"
                    or int(status.get("grasp_id", -1)) != self._grasp_check_id):
                return
            if bool(status.get("ok", False)):
                self.get_logger().info(
                    "GRASP TCP 근접 + 수용 확인 "
                    f"{float(status.get('d', 999.0)):.3f}m — 칼날 절단 시작")
                self._begin_cut()
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
                self._begin_preplace()
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
        if (phase in {"APPROACH", "PREGRASP", "GRASP", "CAPTURE_TRIM",
                      "RETRACT_CIRC", "RETRACT_LIN"}
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
        elif self._state == "APPROACH":
            if self._refresh_entry_geometry():
                self._send_rmp_goal(self._pregrasp_target, "PREGRASP")
        elif self._state == "PREGRASP":
            self._send_rmp_goal(self._circ_target, "GRASP")
        elif self._state == "GRASP":
            self._capture_trim_or_close()
        elif self._state == "CAPTURE_TRIM":
            # 원본과 동일하게 최대 35 mm 보정은 한 번만 하고 즉시 닫는다.
            self._capture_trim_or_close(allow_trim=False)
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
        elif self._state == "RETRACT_CIRC":
            self._send_rmp_goal(self._approach_target, "RETRACT_LIN")
        elif self._state in ("RETRACT_LIN", "RETRACT", "PRE_PLACE"):
            if (not self._basket_available()
                    and bool(self.get_parameter(
                        "use_iw_tf_basket_fallback").value)):
                self._acquire_nearby_iw_basket()
            if self._basket_available():
                self._start_place()
            else:
                # 실제 IW 좌표가 없거나 오래됐으면 다른 자세를 만들지 않고 홈으로 간다.
                self._basket_place = None
                self._basket_received_ns = 0
                self._deadline_ns = 0
                self._send_home()
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
        self._basket_received_ns = self.get_clock().now().nanoseconds
        self.get_logger().info(
            "실제 IW 바스켓 좌표 수신: "
            f"base=({values[0]:.3f}, {values[1]:.3f}, {values[2]:.3f})",
            throttle_duration_sec=2.0,
        )
        if self._state == "WAIT_BASKET":
            self._start_place()

    def _basket_available(self) -> bool:
        if self._basket_place is None or not self._basket_received_ns:
            return False
        max_age = max(
            0.0, float(self.get_parameter("basket_pose_max_age_sec").value))
        age = self.get_clock().now().nanoseconds - self._basket_received_ns
        return age <= int(max_age * 1e9)

    def _acquire_nearby_iw_basket(self) -> bool:
        """IW TF와 실제 KLT 격자로 도달 가능한 가장 가까운 빈 바스켓을 선택한다."""
        raw = list(self.get_parameter("iw_empty_basket_offsets").value)
        if not raw or len(raw) % 3:
            self.get_logger().warning("iw_empty_basket_offsets 형식 오류")
            return False
        iw_frame = str(self.get_parameter("iw_base_frame").value)
        base_frame = str(self.get_parameter("base_frame").value)
        lower = np.asarray(self.get_parameter("basket_workspace_min").value)
        upper = np.asarray(self.get_parameter("basket_workspace_max").value)
        candidates: list[np.ndarray] = []
        for index in range(0, len(raw), 3):
            source = PoseStamped()
            source.header.frame_id = iw_frame
            source.header.stamp = Time().to_msg()
            source.pose.position.x = float(raw[index])
            source.pose.position.y = float(raw[index + 1])
            source.pose.position.z = float(raw[index + 2])
            source.pose.orientation.w = 1.0
            try:
                target = self._buffer.transform(
                    source, base_frame,
                    timeout=Duration(seconds=float(
                        self.get_parameter("tf_timeout_sec").value)))
            except TransformException:
                # IW가 실행되지 않았거나 공통 map TF에 연결되지 않았으면 정상적으로
                # "근처 IW 없음" 처리한다. 후보마다 같은 경고를 반복하지 않는다.
                return False
            p = target.pose.position
            values = np.array([p.x, p.y, p.z], dtype=float)
            if (np.all(np.isfinite(values))
                    and np.all((lower <= values) & (values <= upper))):
                candidates.append(values)
        if not candidates:
            return False
        # 팔 기준 수평거리가 가장 짧은 KLT를 택해 불필요한 관절 회전을 줄인다.
        selected = min(candidates, key=lambda value: float(
            np.linalg.norm(value[:2])))
        self._basket_place = selected.copy()
        self._basket_received_ns = self.get_clock().now().nanoseconds
        self.get_logger().info(
            "근처 IW 빈 바스켓 선택: "
            f"base=({selected[0]:.3f}, {selected[1]:.3f}, {selected[2]:.3f})")
        return True

    def _start_place(self) -> None:
        if not self._basket_available():
            self._basket_place = None
            self._basket_received_ns = 0
            self._send_home()
            return
        approach = self._basket_place.copy()
        approach[2] += float(
            self.get_parameter("basket_approach_height_m").value)
        self._send_rmp_goal(approach, "BASKET_APPROACH")

    def _begin_preplace(self) -> None:
        """레거시 진입점. 큰 HOME 왕복 대신 접근 경로를 그대로 되짚어 후퇴한다."""
        self._send_rmp_goal(self._pregrasp_target, "RETRACT")

    def _begin_cut(self) -> None:
        """수용부가 과실을 고정한 뒤 외측 칼날만 닫고 실제 각도 도달을 기다린다."""
        self._cut_check_sent = False
        self._transition("CUTTING", stop=True)
        self._deadline_ns = (
            self.get_clock().now().nanoseconds
            + int(float(
                self.get_parameter("blade_motion_timeout_sec").value) * 1e9)
        )
        self._isaac_command_pub.publish(String(data=json.dumps({
            "blade": float(self.get_parameter("blade_cut_deg").value),
        })))

    def _send_home(self, retry_after_home: bool = False) -> None:
        self._basket_place = None
        self._basket_received_ns = 0
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
            self.get_logger().info("원샷 수확 종료 — h 다시 눌러야 다음 과실")

    def _abort_to_home(self, reason: str) -> None:
        """실패해도 팔을 홈으로 돌려 다음 과실을 계속 시도하게 한다(데모 연속 사이클).
        이미 홈 복귀 중(GO_HOME)에 또 실패하면 무한루프 방지로 멈추기만 한다."""
        self.get_logger().warning(f"수확 실패({reason}) — 홈 복귀 후 다음 시도")
        was_going_home = self._state == "GO_HOME"
        # 홈 복귀 자체의 성공을 수확 사이클 성공으로 오인하지 않도록 상위 시험
        # 노드에 실패를 먼저 명시한다. 이어지는 GO_HOME은 안전 복귀 동작일 뿐이다.
        self._transition("HARVEST_FAILED")
        self._isaac_command_pub.publish(String(data=json.dumps({
            "gripper": {"closed": False},
            "blade": float(self.get_parameter("blade_open_deg").value),
        })))
        if was_going_home:
            self._deadline_ns = 0
            self._transition("HOME_READY", stop=True)
            self._mobility_pub.publish(Bool(data=True))
            self._maybe_single_shot_off()
            return
        self._send_home(retry_after_home=bool(
            self.get_parameter("retry_after_failure").value))

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
