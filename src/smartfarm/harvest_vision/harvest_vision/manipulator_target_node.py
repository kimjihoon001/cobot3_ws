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
    "PRE_PLACE_BED_VIEW", "WAIT_BASKET_AT_BED_VIEW", "BASKET_AZIMUTH_ALIGN",
    "BASKET_APPROACH", "PLACE_RELEASING",
    "BASKET_RETRACT", "POST_PLACE_BED_VIEW", "PLACE_FAILED_HOLDING",
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
        # mm_motion_bridge.py(ROS쪽 MoveIt 브리지)의 phase 진행 상태(reached,
        # phase=APPROACH/PREGRASP/GRASP/...) 채널 — _status_callback이 이걸로
        # APPROACH→PREGRASP→GRASP를 이어간다.
        self.declare_parameter(
            "rmp_status_topic", "/harvester_0/rmpflow/status"
        )
        # Isaac(mm.py)의 grasp_check/gripper/blade 응답 채널 — 위와는 완전히
        # 별개 토픽인데 같은 _status_callback이 필드로 구분해서 처리한다.
        # 하나로 합치면(둘 중 하나만 구독하면) 그 절반이 영원히 안 온다
        # (2026-07-27 확인 — rmp_status_topic을 이걸로 바꿨더니 GRASP_VERIFY는
        # 되는데 APPROACH→PREGRASP가 95초 넘게 멈췄다).
        self.declare_parameter(
            "isaac_status_topic", "/harvester_0/status"
        )
        self.declare_parameter("basket_pose_topic", "/iw/basket/empty_slot_pose")
        # IW 슬롯 선택기가 최근에 발행한 실제 좌표만 사용한다. 오래된 좌표를 들고
        # 이미 떠난 IW를 향해 팔이 다시 움직이지 않도록 유효시간을 둔다.
        self.declare_parameter("basket_pose_max_age_sec", 2.0)
        # IW가 정차하기 전에 좌표를 래치하면 IW가 계속 전진해 과실이 칸 뒤쪽
        # 격벽으로 떨어진다(2026-07-25: IW가 목표까지 0.4 m 남기고도 pose 발행).
        # /iw/status에는 "MM 옆 정차" 상태가 없으므로, 발행 pose가 움직이지
        # 않는 것을 정차 판정으로 쓴다.
        # 판정은 직전 샘플이 아니라 구간 기준 좌표 대비 변위로 한다 —
        # 저속으로 계속 기어가는 IW를 정차로 오인하지 않기 위해서다.
        self.declare_parameter("basket_stable_sec", 2.0)
        self.declare_parameter("basket_stable_tolerance_m", 0.01)
        self.declare_parameter("use_iw_tf_basket_fallback", False)
        self.declare_parameter("iw_base_frame", "iwhub_0/base_link")
        # IwHub cargo의 실제 4×2 KLT 격자. [x,y,z]는 IW base_link 기준 release
        # pose이며 z=KLT 윗면+약 5 cm다. 이 표는 /iw/basket/empty_slot_pose가
        # 없을 때만 쓰는 폴백이다.
        # ★팔레트는 base_link 중심이 아니다(55263ea 지게차 도킹 정렬). 카고 원점이
        #   실측 chassis bbox 중심 = base_link 기준 -0.3171 m로 이동했으므로 격자
        #   ±0.465/±0.155에 그 오프셋을 더한다. 2026-07-25 스폰 시점 ground truth
        #   (map (1.561,-12.125,0.588), IW (1.6955,-12.0,180°))로 검증.
        #   초기 적재 토마토를 없앤 뒤로 8칸 모두 빈 슬롯이다.
        self.declare_parameter("iw_empty_basket_offsets", [
            -0.782, -0.125, 0.587,
            -0.782, 0.125, 0.587,
            -0.472, -0.125, 0.587,
            -0.472, 0.125, 0.587,
            -0.162, -0.125, 0.587,
            -0.162, 0.125, 0.587,
            0.148, -0.125, 0.587,
            0.148, 0.125, 0.587,
        ])
        # 상대 이름이어야 namespace=harvester_0에서 코디네이터가 발행하는
        # /harvester_0/harvest_test/enable과 동일한 토픽으로 해석된다.
        self.declare_parameter("harvest_enable_topic", "harvest_test/enable")
        # 코디네이터가 "이번 적재에서 아직 놓을 게 남았다"를 알려주는 채널.
        # true면 플레이스 후 HOME(joint_1=180°)까지 돌아갔다 오지 않는다.
        self.declare_parameter(
            "place_more_pending_topic", "harvest_test/place_more_pending")
        self.declare_parameter("external_harvest_gate_enabled", False)
        self.declare_parameter("use_sim_ground_truth", False)
        self.declare_parameter("sim_tomato_topic", "/harvester_0/sim/tomato")
        self.declare_parameter("sim_match_radius_m", 0.35)
        # 수확 목표와 바스켓 적재 목표는 허용 높이가 다르다. 기존 workspace_min.z=0.15는
        # 낮은 KLT 적재를 위해 필요하지만, 이를 과실 선택에도 공유하면 바닥 가까이 내려간
        # 과실(z≈0.42)의 safe approach(z≈0.25)를 정상 목표로 승인해 팔이 베이스 쪽으로
        # 크게 접힌다. 시뮬 GT 후보 단계에서 수확 과실만 별도로 제한한다.
        self.declare_parameter("harvest_target_min_z_m", 0.70)
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
        # 릴리즈 높이 = 발행된 슬롯 pose + 이 값. 여기서 스쿱을 열어 낙하시키고
        # TCP를 더 내리지 않는다. 값 유도[2] — 그리퍼 끝을 KLT 윗면 3cm 위에 둔다:
        #   harvest_tcp = 플랜지 +Z 120mm(1/4구 회전중심), 스쿱 메시 z 최대 177mm
        #   → 툴 끝은 TCP 앞 57mm. 릴리즈 자세는 연직 하방이라 그대로 아래쪽 여유다.
        #   발행 슬롯 pose = KLT 윗면 + 33.1mm (isaacpjt/iw.py: 0.11205×0.85 − 62.1mm)
        #   ∴ 0.030 + 0.057 − 0.0331 = 0.0539
        # 기존 실행값에서 3cm 더 낮춘다(0.014 → -0.016). 과실을 놓을 때
        # TCP가 발행된 슬롯 pose보다 16mm 아래까지 진입한다.
        # 2026-07-27: 다시 2cm 하향(-0.016 → -0.036). 스쿱 곡면 때문에 과실이
        # 수직 낙하가 아니라 굴러 나가므로, 낙하 높이를 줄이면 수평 이동거리도
        # 준다. 여유 확인 — 릴리즈 시 스쿱 최저점은 KLT 윗면 아래 59.9mm이고
        # KLT 내부 깊이는 약 112mm라 바닥까지 52mm 남는다(충돌 없음).
        self.declare_parameter("basket_approach_height_m", -0.036)
        # 릴리즈 후 LIN으로 빠져나올 바구니 상부 안전점(릴리즈점 기준 추가 상승).
        # 릴리즈 시 스쿱 끝이 림보다 3cm 위다. 추가 8cm만 수직 후퇴해도
        # 총 11cm 여유라 접기에 충분하며, 기존 15cm LIN 왕복 시간을 줄인다.
        self.declare_parameter("basket_retract_height_m", 0.08)
        # 접힘 자세에서 바구니 pose를 기다리는 시간. 일반 모션 타임아웃(10초)과
        # 분리한다 — IW가 이미 정차했으면 한 프레임에 진행하고, 없으면 이만큼만
        # 기다린 뒤 수확물을 든 채 홈으로 복귀한다.
        self.declare_parameter("basket_wait_timeout_sec", 2.0)
        # 릴리즈 높이 검증 로그용 실측 상수.
        #   harvest_tcp → 스쿱 최저점: 스쿱 메시 z 최대 177mm − TCP 120mm
        #   발행 슬롯 pose → KLT 윗면: isaacpjt/iw.py의 0.11205×0.85 − 반높이 62.1mm
        self.declare_parameter("scoop_tip_below_tcp_m", 0.057)
        self.declare_parameter("basket_pose_above_rim_m", 0.0331)
        # 바구니 단계 전용 툴 자세 — 수확용 _harvest_orientation을 재사용하지
        # 않는다. 연직 하방 자세로 릴리즈점까지 이동한 뒤 그 자리에서 연다.
        self.declare_parameter(
            "basket_tool_orientation",
            [1.0, 0.0, 0.0, 0.0])
        # 바스켓은 축정렬 상자가 아니라 수평 반경으로 판정한다. J1이 360° 도는
        # 6축 팔에서 KLT는 원래 MM 측후방에 놓이며(머지 전 grasp_proto.py 방식),
        # 상자 하한 X를 쓰면 도달 가능한 뒤쪽 슬롯이 부호 때문에 거부된다.
        # 반경 상한 근거[2]: 릴리즈 높이(base 기준 z≈+0.29)에서 툴을 아래로
        # 세우면 손목+툴 0.241m가 수직으로 소모돼 어깨(z=0.153)에서 wrist center
        # 까지 0.377m 상승 → 최대 수평 = √((0.845+0.734)² − 0.377²) = 1.533m.
        # 완전 신전(특이점) 직전을 피해 1.45로 둔다.
        self.declare_parameter("basket_max_reach_m", 1.45)
        self.declare_parameter("basket_z_min", 0.15)
        self.declare_parameter("basket_z_max", 1.80)
        # KLT 중심에서 수평면상 MM(base 원점) 방향으로 최종 릴리즈점을 당긴다.
        #
        # 2026-07-27: 기본값 0.08 → 0.04 → **0.0**.
        # 발행되는 슬롯 pose는 이미 KLT 콜라이더(=실제 바구니)의 내부 중심이다.
        # 보정이 필요해 보였던 이유는 스쿱이 1/4구 곡면이라 과실이 굴러 나가며
        # 낙하점이 릴리즈점과 다르기 때문이다(반포물선 낙하, 실행 관찰).
        #
        # 그런데 이 보정의 방향은 **MM base 원점 쪽**이고 굴림 방향은 스쿱 자세가
        # 정한다 — 둘은 무관하다. 따라서 크기를 얼마로 두든 굴림에 자세마다
        # 다른 방향 성분을 더하는 셈이라 재현성만 나빠진다. 실측:
        #   0.08 → KLT 짧은 변(내부 반폭 0.0742) 벽 위에 얹힘
        #          (로컬 y 변위 0.054 + 과실 반지름 0.034 = 0.088 > 외벽 0.0842)
        #   0.04 → 들어가긴 하나 재현성 불량
        #   0.00 → 착지점 = KLT 내부 중심 + 굴림 벡터. 무작위 성분이 없다
        # 굴림 자체의 근본 대응(손목 회전각을 KLT 긴 축에 정렬)은 발표 후 과제다.
        # 기존 반경(MM 원점) 방향 보정. 방향이 스쿱의 실제 낙하 방향과 일치하지
        # 않아 KLT 중앙에서 원하는 오른쪽 보정에 쓸 수 없으므로 기본은 끈다.
        # 근거: docs/investigation_basket_place_offset_2026-07-27.md
        self.declare_parameter("basket_place_toward_mm_m", 0.0)
        # MM base 원점에서 바스켓 중심을 바라볼 때의 수평면 오른쪽 방향 보정.
        # 사용자가 KLT 중앙보다 오른쪽에 놓아야 내부 착지한다고 반복 관찰한
        # 결과를 반영한다. 40mm는 과실 반지름을 제외한 짧은 축 안전 여유다.
        self.declare_parameter("basket_place_right_m", 0.0)
        self.declare_parameter("workspace_min", [0.15, -1.05, 0.15])
        self.declare_parameter("workspace_max", [1.25, 1.05, 1.80])
        # 데모: 성공/실패 무관 매 시도 후 홈 복귀 → 팔이 안 굳고 다음 과실을 계속 시도한다.
        self.declare_parameter("home_after_attempt", True)
        # h 원샷: 한 사이클(인식→수확→홈) 끝나면 게이트를 스스로 끈다. 계속 재인식·재시작
        # 하지 않고 h 를 다시 눌러야 다음 과실을 잡는다.
        self.declare_parameter("single_shot_harvest", True)
        # 실패 후에도 명시적 요청 없이 새 과실을 자동으로 쫓지 않는다.
        self.declare_parameter("retry_after_failure", False)
        # 수확 실패 시 홈까지 가지 않고 APPROACH 안전점으로 후퇴해 재파지하는 최대 횟수.
        # 소진하면 베드뷰 재관측으로 에스컬레이션한다. 0이면 즉시 베드뷰/홈 복귀.
        self.declare_parameter("approach_retry_max", 1)
        # 접근점 재시도까지 소진한 뒤 베드뷰로 돌아가 좌표를 다시 받는 최대 횟수.
        self.declare_parameter("bed_view_retry_max", 1)
        # 타겟 좌표 안정화: 파지 시작 전, 좌표가 eps_m 이내로 N프레임 연속 고정돼야 수락.
        self.declare_parameter("target_stable_frames", 5)
        self.declare_parameter("target_stable_eps_m", 0.01)

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
        self._approach_retry_count = 0
        self._bed_view_retry_count = 0
        self._stable_count = 0
        self._stable_ref: tuple[float, float, float] | None = None
        self._follow_check_id = 0
        self._basket_place: np.ndarray | None = None
        self._basket_map_z: float | None = None
        self._basket_received_ns = 0
        self._basket_stable_since_ns = 0
        self._basket_stable_ref_xy: np.ndarray | None = None
        # 접힌 홈 경유의 **성공 응답**을 받았는지. 명령 발행만으로 True로 만들지
        # 않는다 — 홈이 실패하면 바구니 접근을 시작하면 안 된다.
        self._place_home_done = False
        self._basket_retract_tried = False
        # 방위 정렬 뒤 팔을 펼 릴리즈 목표(슬롯 중심 + 접근 높이).
        self._basket_release: np.ndarray | None = None
        # 릴리즈점에 실제로 도달했는가. 도달 전 실패에 바구니 상부 LIN 후퇴를
        # 시도하면 홈에서 1.3m 직교 이동이 되어 Pilz가 거부한다.
        self._basket_release_reached = False
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
        isaac_status_topic = str(self.get_parameter("isaac_status_topic").value)
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
        self.create_subscription(
            String, isaac_status_topic, self._status_callback, 10)
        self.create_subscription(PoseStamped, basket_topic, self._basket_callback, 10)
        self.create_subscription(Bool, enable_topic, self._enable_callback, 10)
        self.create_subscription(
            Bool, str(self.get_parameter("place_more_pending_topic").value),
            self._place_more_pending_callback, latched_qos)
        self.create_subscription(
            String, str(self.get_parameter("sim_tomato_topic").value),
            self._sim_tomato_callback, 20)
        self.create_timer(0.1, self._watchdog)
        self._target_class = ""
        self._place_more_pending = False
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
            # 시뮬레이션 부하로 센서 stamp가 TF보다 잠깐 앞서면 future
            # extrapolation이 발생한다. 진행 중인 시퀀스는 위에서 잠겨 있고
            # 타깃 획득 시 로봇은 정차 상태이므로 최신 공통 TF로 한 번 재시도한다.
            latest_target = PoseStamped()
            latest_target.header.frame_id = msg.header.frame_id
            latest_target.header.stamp = Time().to_msg()
            latest_target.pose = msg.pose
            latest_camera_origin = PoseStamped()
            latest_camera_origin.header.frame_id = msg.header.frame_id
            latest_camera_origin.header.stamp = Time().to_msg()
            latest_camera_origin.pose.orientation.w = 1.0
            try:
                target = self._buffer.transform(
                    latest_target,
                    base_frame,
                    timeout=Duration(seconds=timeout),
                )
                camera = self._buffer.transform(
                    latest_camera_origin,
                    base_frame,
                    timeout=Duration(seconds=timeout),
                )
                self.get_logger().warning(
                    "센서 시각의 TF가 아직 없어 최신 TF로 타깃을 변환했습니다: "
                    f"{exc}",
                    throttle_duration_sec=2.0,
                )
            except TransformException as fallback_exc:
                self.get_logger().warning(
                    f"TF 변환 실패 ({msg.header.frame_id} -> {base_frame}): "
                    f"{fallback_exc}",
                    throttle_duration_sec=2.0,
                )
                return

        # tf2가 반환한 frame_id를 신뢰하되, 배포판별 구현 차이에 대비해 정규화한다.
        target.header.frame_id = base_frame
        if not self._inside_workspace(target):
            p = target.pose.position
            cp = camera.pose.position
            # 팔 작업영역 밖의 비전 좌표도 Nav2 재접근 계산에는 필요하다.
            # 여기서 좌표를 버리면 ripe 클래스 콜백이 _start_grasp_sequence()를
            # 호출할 때 ERROR_NO_TARGET이 되어, 아래의 GT 작업영역 검사와
            # /harvester_0/nav/reposition_request 경로에 영원히 도달하지 못한다.
            # 팔 명령/validated 토픽은 계속 차단하고 재접근 입력으로만 보관한다.
            self._latest_target = (p.x, p.y, p.z)
            self._latest_camera = (cp.x, cp.y, cp.z)
            self.get_logger().warning(
                "작업영역 밖 목표 — 팔 명령 차단, Nav2 재접근용으로 보관: "
                f"({p.x:.3f}, {p.y:.3f}, {p.z:.3f})",
                throttle_duration_sec=2.0,
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
        # 타겟 좌표 안정화: eps_m 이내로 연속 유지되는 프레임 수를 센다. 흔들리면 리셋.
        pos = (p.x, p.y, p.z)
        eps = float(self.get_parameter("target_stable_eps_m").value)
        if (self._stable_ref is not None
                and abs(pos[0] - self._stable_ref[0]) <= eps
                and abs(pos[1] - self._stable_ref[1]) <= eps
                and abs(pos[2] - self._stable_ref[2]) <= eps):
            self._stable_count += 1
        else:
            self._stable_count = 1
            self._stable_ref = pos
        # 게이트가 열린 순간 class가 Pose보다 먼저 도착할 수 있다. 그 경우
        # ERROR_NO_TARGET로 사이클을 닫지 않고 첫 유효 Pose가 들어온 여기서 시작한다.
        if (self._state == "WAIT_TARGET"
                and self._harvest_enabled
                and self._target_class in ("tomato", "ripe")
                and bool(self.get_parameter("command_enabled").value)
                and bool(self.get_parameter("auto_grasp_enabled").value)):
            need_stable = int(self.get_parameter("target_stable_frames").value)
            if self._stable_count < need_stable:
                self.get_logger().info(
                    f"타겟 좌표 안정화 대기 {self._stable_count}/{need_stable}",
                    throttle_duration_sec=1.0)
                return
            self._transition("RIPE_READY", stop=True)
            self._start_grasp_sequence()
            return
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
        if (self._state == "WAIT_TARGET"
                and target_class in ("tomato", "ripe")
                and (self._latest_target is None or self._latest_camera is None)):
            # 첫 유효 Pose는 _target_callback에서 시퀀스를 시작한다.
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
        if state == "WAIT_BASKET_AT_BED_VIEW":
            # 접는 동안 PRE_PLACE_BED_VIEW에서 관측한 좌표는 IW가 완전히
            # 정지하기 전의 것일 수 있다. WAIT 진입 이후에 받은 새 pose만으로
            # 정차 시간을 다시 측정해 과거 안정 구간을 이어 쓰지 않는다.
            self._basket_place = None
            self._basket_received_ns = 0
            self._basket_stable_since_ns = 0
            self._basket_stable_ref_xy = None
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
            # RGB class와 TF 변환된 Pose는 서로 다른 콜백에서 도착한다. 통합 실행은
            # 부하가 커 gate-open 직후 class가 먼저 오는 일이 흔하므로 정상 대기한다.
            self._transition("WAIT_TARGET", stop=True)
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
        self._approach_retry_count = 0    # 새 과실 시퀀스 시작 — 재접근 카운터 초기화
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
        """fresh하고 안전한 GT 중 검출된 3D 점에 가장 가까운 과실을 선택한다."""
        now = self.get_clock().now().nanoseconds
        fresh = [
            (float(np.linalg.norm(position - vision_target)), fruit_id, position)
            for fruit_id, (position, stamp) in self._sim_fruits.items()
            if (now - stamp <= int(30.0e9)
                and self._sim_harvest_height_ok(position))
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
                 in self._sim_fruits.items()
                 if (now - stamp <= int(30.0e9)
                     and self._sim_harvest_height_ok(position))]
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

    def _sim_harvest_height_ok(self, position: np.ndarray) -> bool:
        """낮은 적재 workspace와 분리된 수확 과실 높이 안전조건."""
        return (
            position.shape == (3,)
            and np.all(np.isfinite(position))
            and float(position[2]) >= float(
                self.get_parameter("harvest_target_min_z_m").value)
        )

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
        elif phase == "BASKET_APPROACH":
            # 바스켓은 등 뒤(약 joint_1 180°)라 자유 IK면 팔이 크게 뒤집힌다.
            # 랭크드 IK로 joint_1 위주 최소변화 해를 골라 방향만 맞춘 뒤 place 한다.
            command["rmp_target"]["motion"] = "OMPL"
        elif phase in {"GRASP", "RETRACT_CIRC"}:
            # 축방향 직선 삽입/후퇴 — CIRC 원호를 쓰지 않는다.
            command["rmp_target"]["motion"] = "LIN"
            if phase == "GRASP":
                command["rmp_target"]["velocity_scale"] = 0.065
        elif phase in {
            "PREGRASP", "CAPTURE_TRIM", "RETRACT_LIN", "BASKET_RETRACT",
        }:
            command["rmp_target"]["motion"] = "LIN"
            if phase == "PREGRASP":
                # 직선 진입 중에도 베드뷰의 1축 가지를 유지한다. 같은 TCP 자세의
                # 반대쪽 등가 IK를 골라 프리그랩에서 크게 도는 현상을 막는다.
                command["rmp_target"]["lock_joint_1"] = True
            if phase == "CAPTURE_TRIM":
                command["rmp_target"]["velocity_scale"] = 0.0455
            if phase == "BASKET_RETRACT":
                command["rmp_target"]["velocity_scale"] = 0.585
        else:
            command["rmp_target"]["motion"] = "PTP"
        # PREGRASP마다 카메라 광선으로 새 TCP 자세를 만들면 ±360° 범위의 두산 손목이
        # 먼 등가 IK 해를 골라 제자리에서 여러 번 회전할 수 있다. Nav 도착 뒤 이미
        # 베드를 향한 현재 스쿱 자세를 그대로 잠그고 위치만 목표로 이동한다.
        if phase in {"BASKET_APPROACH", "BASKET_RETRACT"}:
            # 바구니는 등 뒤 아래에 있다. 베드를 향한 수확 자세를 그대로 쓰면
            # 스쿱 개구가 옆을 향해 과실이 슬롯 밖으로 튄다.
            command["rmp_target"]["tool_orientation"] = [
                float(value)
                for value in self.get_parameter(
                    "basket_tool_orientation").value
            ]
        elif phase in {
            "APPROACH", "PREGRASP", "GRASP",
            "CAPTURE_TRIM", "VERIFY_RETRACT",
            "RETRACT_CIRC", "RETRACT_LIN", "RETRACT",
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
                self._bed_view_retry_count = 0   # 파지 성공 — 재관측 재시도 카운터 리셋
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
        if (self._state == "PLACE_RELEASING"
                and not str(status.get("phase", "")).startswith("ERROR_")):
            # 오류 응답은 아래 공통 오류 처리로 넘긴다. 여기서 먼저 return하면
            # 릴리즈 중 실패가 워치독 타임아웃까지 통째로 무시된다.
            try:
                gripper = float(status.get("gripper", 1.0))
            except (TypeError, ValueError):
                return
            if gripper <= 0.08:
                # KLT 안에서 곧바로 PTP 홈 복귀를 시작하면 스쿱이 바스켓 벽을
                # 가로지를 수 있다. 접근점을 LIN으로 되짚어 빠져나온다.
                self._start_basket_retract()
            return
        try:
            status_id = int(status.get("id", -1))
        except (TypeError, ValueError):
            return
        if status_id != self._pending_id:
            return
        phase = str(status.get("phase", ""))
        if phase in {"ERROR_DIVERGENCE", "ERROR_IK_PATH", "ERROR_STAGNATION",
                     "ERROR_MOVEIT_UNAVAILABLE"}:
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
            # 먼저 베드뷰와 같은 접힘 자세로 간다. 판단은 그 자세에서 한다.
            # 편 자세로 대기하거나 회전하면 반경 1.1m로 베드를 쓸고 지나간다
            # (2026-07-25 sim_diag_154400). 주행/수확 중 들어온 좌표는 이동
            # 중 IW의 과거 위치일 수 있으므로 여기서 폐기하고 새로 받는다.
            self._basket_place = None
            self._basket_received_ns = 0
            self._place_home_done = False
            self._basket_retract_tried = False
            self._basket_release = None
            self._basket_release_reached = False
            self._send_place_fold()
        elif self._state == "PRE_PLACE_BED_VIEW":
            # ── 접힘(베드뷰 자세) 도달. 여기서 바구니 가용성을 판단한다. ──
            #   있으면  → 방위 회전 → 플레이스 접근
            #   없으면  → 수확물을 든 채 홈 복귀(WAIT_BASKET_AT_BED_VIEW 타임아웃 경로)
            self._place_home_done = True
            self._transition("WAIT_BASKET_AT_BED_VIEW", stop=True)
            self._deadline_ns = (
                self.get_clock().now().nanoseconds
                + int(float(self.get_parameter(
                    "basket_wait_timeout_sec").value) * 1e9)
            )
            # TF 폴백 좌표는 접힘이 끝난 뒤에 취득한다. 이동 중에 잡으면
            # basket_pose_max_age_sec(2초)에 걸려 만료된다.
            if (bool(self.get_parameter("use_iw_tf_basket_fallback").value)
                    and self._acquire_nearby_iw_basket()):
                self._start_place()
        elif self._state == "BASKET_AZIMUTH_ALIGN":
            # 방위가 맞았으니 이제 편다. 남은 joint_1 변화가 작아 관절변화
            # 제한에 걸리지 않고, 펴는 동작은 바구니 쪽에서만 일어난다.
            if self._basket_release is None:
                self._abort_basket("no_release_target")
            else:
                self._send_rmp_goal(self._basket_release, "BASKET_APPROACH")
        elif self._state == "BASKET_APPROACH":
            # 접근점이 곧 릴리즈점이다. 도달하면 바로 연다.
            self._basket_release_reached = True
            self._log_release_clearance()
            self._transition("PLACE_RELEASING", stop=True)
            self._deadline_ns = (
                self.get_clock().now().nanoseconds
                + int(float(self.get_parameter("motion_timeout_sec").value) * 1e9)
            )
            # reason="place": 바구니에 실제로 놓는 개방이다. 접근 전 개방이나
            # 실패 시 부분 파지 해제와 구분해야 IW 슬롯 배정기가 적재를 정확히
            # 센다(파지 '검증' 성공 여부로는 구분할 수 없다 — 과실을 제대로
            # 집어도 검증이 실패로 뜨는 경우가 많다).
            self._isaac_command_pub.publish(String(data=json.dumps({
                "gripper": {"closed": False, "reason": "place"}})))
        elif self._state == "BASKET_RETRACT":
            # 바구니 바로 위에서 편 팔을 HOME으로 한 번에 돌리면 스쿱/링크가
            # KLT와 IW를 쓸 수 있다. 방위를 유지한 채 먼저 완전히 접는다.
            self._send_post_place_fold()
        elif self._state == "POST_PLACE_BED_VIEW":
            if self._place_more_pending:
                # 접힌 자세는 HOME_Q에서 joint_1만 바구니 방위인 상태다. 이번
                # 적재에서 한 번 더 놓아야 하면 joint_1을 180°(HOME)로 돌렸다가
                # 다시 베드로 돌리는 왕복이 순수한 낭비다. 접힌 채로 코디네이터에
                # 넘겨 joint_1만 베드 방위로 돌리는 BED_VIEW를 바로 받게 한다.
                self._basket_place = None
                self._basket_received_ns = 0
                self._mobility_pub.publish(Bool(data=True))
                self._transition("HOME_READY", stop=True)
                self.get_logger().info(
                    "플레이스 완료 — 남은 적재가 있어 HOME 왕복 없이 "
                    "접힌 자세에서 베드뷰 복귀 대기")
                self._maybe_single_shot_off()
                return
            # 접기 성공 응답을 받은 뒤에만 joint_1까지 HOME 방위로 복귀한다.
            self._send_home(fast=True)
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
                self._stable_count = 0        # 재관측 — 5프레임 안정도 처음부터
                self._stable_ref = None
                self._transition("RETRY_VISION", stop=True)
                self.get_logger().info("홈 복귀 완료 — 비전 재탐색 후 자동 재시도")
            else:
                self._transition("HOME_READY", stop=True)
                self._maybe_single_shot_off()

    def _place_more_pending_callback(self, msg: Bool) -> None:
        """이번 IW 적재에서 아직 더 놓아야 하는지를 코디네이터로부터 받는다."""
        self._place_more_pending = bool(msg.data)

    def _basket_callback(self, msg: PoseStamped) -> None:
        """IW가 선택한 빈 바스켓 슬롯의 tool-release pose를 base 좌표로 저장한다."""
        # 바스켓 위치는 수확·절단·후퇴가 끝난 뒤에만 사용한다. 통합 시작부터
        # 변환하면 이동 중 IW 좌표가 캐시되고 불필요한 TF/작업영역 경고도 폭주한다.
        # PRE_PLACE_BED_VIEW(접기) 중에도 갱신은 받는다. 접는 몇 초 사이에
        # 좌표가 만료되면 플레이스를 시작하지 못하고 홈 복귀로 빠진다.
        if self._state not in ("WAIT_BASKET_AT_BED_VIEW", "PRE_PLACE_BED_VIEW"):
            return
        if not msg.header.frame_id or self._is_stale(msg):
            return
        base_frame = str(self.get_parameter("base_frame").value)
        timeout = Duration(seconds=float(
            self.get_parameter("tf_timeout_sec").value))
        try:
            target = self._buffer.transform(msg, base_frame, timeout=timeout)
        except TransformException as exc:
            # sim clock이 튀면 발행 stamp가 TF 버퍼 구간 밖으로 나가 매번
            # extrapolation 오류가 난다(2026-07-25: stamp 34.1 vs 버퍼 최초
            # 42.3). 아래 안정화 게이트가 IW 정차를 보장하므로, 이때는 최신
            # TF로 다시 시도한다 — 정차 중이면 최신 TF가 곧 그 시각의 TF다.
            latest = PoseStamped()
            latest.header.frame_id = msg.header.frame_id
            latest.header.stamp = Time().to_msg()
            latest.pose = msg.pose
            try:
                target = self._buffer.transform(
                    latest, base_frame, timeout=timeout)
            except TransformException:
                self.get_logger().warning(
                    "바스켓 TF 변환 실패 "
                    f"({msg.header.frame_id} -> {base_frame}): {exc}",
                    throttle_duration_sec=2.0)
                return
        p = target.pose.position
        values = np.array([p.x, p.y, p.z], dtype=float)
        if not self._basket_reachable(values):
            if self._state != "WAIT_BASKET_AT_BED_VIEW":
                # 홈 경유 중 일시적으로 범위를 벗어난 갱신은 무시하고 직전
                # 유효 좌표를 유지한다(IW가 아직 정차 중일 수 있다).
                return
            self.get_logger().warning(
                "작업영역 밖 바스켓 목표 — 토마토를 잡은 상태로 HOME 복귀: "
                f"base=({values[0]:.3f}, {values[1]:.3f}, "
                f"{values[2]:.3f}), "
                f"수평반경={float(np.linalg.norm(values[:2])):.3f} "
                f"허용반경={float(self.get_parameter('basket_max_reach_m').value):.2f} "
                f"z=[{float(self.get_parameter('basket_z_min').value):.2f}"
                f" .. {float(self.get_parameter('basket_z_max').value):.2f}]")
            # 범위 밖 pose를 계속 무시하면 WAIT_BASKET에서 영구 정지한다.
            # 그리퍼 개방/실패 재시도 없이 곧바로 HOME을 보내 수확물을 든 채
            # 안전 자세에서 IW의 다음 동작을 기다린다. _send_home()이 상태를
            # GO_HOME으로 바꾸므로 뒤따르는 고주기 pose는 이 콜백 입구에서 무시된다.
            self._deadline_ns = 0
            self._send_home()
            return
        self._basket_place = values
        self._basket_map_z = float(msg.pose.position.z)
        now_ns = self.get_clock().now().nanoseconds
        self._basket_received_ns = now_ns
        # IW 정차 판정: 구간 시작의 "기준 좌표" 대비 누적 변위로 본다.
        # 직전 샘플과 비교하면 매 프레임 8 mm씩 꾸준히 기어가는 저속 IW가
        # 프레임마다 허용오차 안이라 영원히 정차로 오인된다. 기준 좌표를
        # 고정해두면 그 변위가 누적돼 허용오차를 넘고 구간이 리셋된다.
        raw_xy = np.array([msg.pose.position.x, msg.pose.position.y])
        tolerance = float(self.get_parameter("basket_stable_tolerance_m").value)
        if (self._basket_stable_ref_xy is None
                or float(np.linalg.norm(raw_xy - self._basket_stable_ref_xy))
                > tolerance):
            self._basket_stable_ref_xy = raw_xy
            self._basket_stable_since_ns = now_ns
        stable_sec = (now_ns - self._basket_stable_since_ns) * 1e-9
        # IW 프레임(iwhub_0/base_link)은 MM의 TF 트리에 없다 — IW는 /iwhub_0/tf
        # 네임스페이스로 발행하고 MM은 /harvester_0/tf만 듣는다. 대신 수신
        # 원본을 프레임 이름과 함께 남긴다. 정상 통합에서는 mm_base로 들어와야
        # 한다 — map으로 들어오면 AMCL 오차가 릴리즈 지점에 그대로 실린다.
        raw = msg.pose.position
        self.get_logger().info(
            "실제 IW 바스켓 좌표 수신: "
            f"base=({values[0]:.3f}, {values[1]:.3f}, {values[2]:.3f})"
            f" | 수신 {msg.header.frame_id}="
            f"({raw.x:.4f}, {raw.y:.4f}, {raw.z:.4f})"
            f" | 안정 {stable_sec:.1f}s",
            throttle_duration_sec=2.0)
        if self._state != "WAIT_BASKET_AT_BED_VIEW":
            return
        required = float(self.get_parameter("basket_stable_sec").value)
        if stable_sec < required:
            # IW가 아직 움직인다. 대기 타임아웃을 미뤄 "바구니 없음" 으로
            # 빠지지 않게 한 뒤, 멈출 때까지 접힌 자세로 기다린다.
            self._deadline_ns = now_ns + int(
                (required - stable_sec + float(self.get_parameter(
                    "basket_wait_timeout_sec").value)) * 1e9)
            self.get_logger().info(
                f"IW 정차 대기 — 발행 좌표가 {stable_sec:.1f}s 안정 "
                f"(필요 {required:.1f}s)",
                throttle_duration_sec=2.0)
            return
        self._start_place()

    def _basket_reachable(self, values: np.ndarray) -> bool:
        """바스켓 릴리즈 좌표(base 프레임)가 팔 도달 범위 안인가."""
        if not np.all(np.isfinite(values)):
            return False
        radius = float(np.linalg.norm(values[:2]))
        return (
            radius <= float(self.get_parameter("basket_max_reach_m").value)
            and float(self.get_parameter("basket_z_min").value)
            <= float(values[2])
            <= float(self.get_parameter("basket_z_max").value)
        )

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
            if self._basket_reachable(values):
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
            # 도달 가능한 바구니 없음 → 수확물을 든 채 홈 복귀(플레이스 생략).
            self.get_logger().warning(
                "도달 가능한 IW 바구니 없음 — 수확물을 잡은 채 홈 복귀")
            self._basket_place = None
            self._basket_received_ns = 0
            self._send_home()
            return
        if not self._place_home_done:
            # 접힘 성공 응답 전에는 절대 바구니로 움직이지 않는다.
            self._fail_place_holding("fold_not_confirmed")
            return
        # 릴리즈 XY는 IW가 발행한 KLT 슬롯 중심에서 수평면상 MM(base 원점)
        # 방향으로 지정 거리만 이동한다. Z 높이는 바꾸지 않는다.
        shift = max(
            0.0,
            float(self.get_parameter("basket_place_toward_mm_m").value),
        )
        if shift > 0.0:
            radius = float(np.linalg.norm(self._basket_place[:2]))
            if radius > shift:
                self._basket_place[:2] *= 1.0 - shift / radius
                self.get_logger().info(
                    f"바스켓 릴리즈 XY: KLT 중심에서 MM 방향으로 "
                    f"{shift:.3f}m 이동 → "
                    f"({self._basket_place[0]:.3f}, "
                    f"{self._basket_place[1]:.3f})")
        # base 원점→바스켓 방향 f=(x,y)/r 에 대한 오른쪽 단위벡터는
        # (f_y,-f_x)다. KLT/map 축에 고정하지 않고 MM이 실제로 바라보는
        # 방향을 기준으로 하므로 사용자가 관찰한 화면상 오른쪽과 일치한다.
        right = max(
            0.0,
            float(self.get_parameter("basket_place_right_m").value),
        )
        if right > 0.0:
            radius = float(np.linalg.norm(self._basket_place[:2]))
            if radius > 1e-6:
                right_vector = np.array([
                    self._basket_place[1] / radius,
                    -self._basket_place[0] / radius,
                ])
                self._basket_place[:2] += right * right_vector
                self.get_logger().info(
                    f"바스켓 릴리즈 XY: KLT 중심에서 바라보는 오른쪽으로 "
                    f"{right:.3f}m 이동 → "
                    f"({self._basket_place[0]:.3f}, "
                    f"{self._basket_place[1]:.3f})")
        # 릴리즈점 = 슬롯 중심 + basket_approach_height_m. 여기서 스쿱을 열어
        # 낙하시킨다. 슬롯 중심(z)까지 TCP를 내리면 스쿱이 KLT 벽에 걸린다.
        release = self._basket_place.copy()
        release[2] += float(
            self.get_parameter("basket_approach_height_m").value)
        self.get_logger().info(
            f"바구니 릴리즈점: 슬롯 z={self._basket_place[2]:.3f} → "
            f"릴리즈 z={release[2]:.3f} (하강 없음)")
        self._basket_release = release
        self._basket_release_reached = False
        # 펴기 전에 방위부터 맞춘다. 편 채로 joint_1을 크게 돌리면 반경 1.1m로
        # 베드를 쓸고 지나간다(2026-07-25 sim_diag_154400).
        self._send_azimuth_align(release)

    def _log_release_clearance(self) -> None:
        """릴리즈 시점의 실제 높이를 남긴다 — 스쿱 최저점이 KLT 윗면 위 3cm인가.

        명령값은 TF 지연과 무관하게 정확하므로 기준으로 쓰고, TF 실측값은 참고로
        함께 찍는다(부하가 높으면 /harvester_0/tf가 수 초 지연될 수 있다).
        """
        if self._basket_release is None:
            return
        tip_offset = float(self.get_parameter("scoop_tip_below_tcp_m").value)
        rim_offset = float(self.get_parameter("basket_pose_above_rim_m").value)
        base_frame = str(self.get_parameter("base_frame").value)
        rim_base = float(self._basket_release[2]) - float(
            self.get_parameter("basket_approach_height_m").value) - rim_offset
        tip_base = float(self._basket_release[2]) - tip_offset
        message = (
            "릴리즈 높이 검증(명령값, base): "
            f"KLT 윗면 z={rim_base:.4f}, harvest_tcp z="
            f"{float(self._basket_release[2]):.4f}, 스쿱 최저점 z="
            f"{tip_base:.4f} → 여유={tip_base - rim_base:.4f} m")
        if self._basket_map_z is not None:
            message += (
                f" | 발행 프레임 기준 KLT 윗면 z="
                f"{self._basket_map_z - rim_offset:.4f}")
        try:
            transform = self._buffer.lookup_transform(
                base_frame, "harvest_tcp", Time()).transform
            measured = float(transform.translation.z)
            # XY 실측도 함께 남긴다. 과실은 스쿱 회전중심(=harvest_tcp)에 앉아
            # 있으므로 낙하점 XY = TCP XY다. 이 오차가 작은데도 KLT 중앙이
            # 아니면 원인은 팔이 아니라 발행 슬롯 좌표다.
            mx = float(transform.translation.x)
            my = float(transform.translation.y)
            lateral = math.hypot(mx - float(self._basket_release[0]),
                                 my - float(self._basket_release[1]))
            message += (
                f" | TF 실측 harvest_tcp z={measured:.4f}, "
                f"스쿱 최저점 z={measured - tip_offset:.4f}, "
                f"여유={measured - tip_offset - rim_base:.4f} m(지연 가능)"
                f" | TF 실측 XY=({mx:.4f}, {my:.4f}) vs 명령 XY="
                f"({float(self._basket_release[0]):.4f}, "
                f"{float(self._basket_release[1]):.4f}) → 횡오차="
                f"{lateral:.4f} m")
        except TransformException as exc:
            message += f" | TF 실측 실패: {exc}"
        self.get_logger().info(message)

    def _send_place_fold(self) -> None:
        """수확 방위(joint_1)를 유지한 채 접는다. 완료는 성공 응답으로만 판정한다.

        홈(joint_1=180°)까지 돌려버리면 바구니 방위(≈348°)에서 168° 떨어져
        관절변화 제한에 전부 걸린다. 방위는 그대로 두고 팔만 접은 뒤,
        BASKET_AZIMUTH_ALIGN에서 joint_1만 돌린다.
        """
        self._sequence_id += 1
        self._pending_id = self._sequence_id
        self._transition("PRE_PLACE_BED_VIEW")
        self._deadline_ns = (
            self.get_clock().now().nanoseconds
            + int(float(self.get_parameter("motion_timeout_sec").value) * 1e9)
        )
        self._isaac_command_pub.publish(String(data=json.dumps({
            "fold_keep_j1": {"id": self._pending_id},
        })))
        self.get_logger().info(
            "바구니 이송 전 접기 — 수확 방위(joint_1) 유지, joint_2~6만 접음")

    def _send_azimuth_align(self, position: np.ndarray) -> None:
        """접힌 상태에서 joint_1만 돌려 바구니 방위로 정렬한다."""
        self._sequence_id += 1
        self._pending_id = self._sequence_id
        self._transition("BASKET_AZIMUTH_ALIGN")
        self._deadline_ns = (
            self.get_clock().now().nanoseconds
            + int(float(self.get_parameter("motion_timeout_sec").value) * 1e9)
        )
        self._isaac_command_pub.publish(String(data=json.dumps({
            "azimuth_align": {
                "id": self._pending_id,
                "position": [float(value) for value in position],
            }
        })))
        self.get_logger().info(
            "바구니 방위 정렬 요청: 목표 XY="
            f"({position[0]:.3f}, {position[1]:.3f}) — 접힌 채 joint_1만 회전")

    def _send_post_place_fold(self) -> None:
        """플레이스 후 현재 방위를 유지한 채 먼저 접어 IW/KLT에서 빠져나온다."""
        self._sequence_id += 1
        self._pending_id = self._sequence_id
        self._transition("POST_PLACE_BED_VIEW")
        self._deadline_ns = (
            self.get_clock().now().nanoseconds
            + int(float(self.get_parameter("motion_timeout_sec").value) * 1e9)
        )
        self._isaac_command_pub.publish(String(data=json.dumps({
            "fold_keep_j1": {
                "id": self._pending_id,
                "phase": "POST_PLACE_BED_VIEW",
                "fast": True,
            },
        })))

    def _start_basket_retract(self) -> None:
        """릴리즈점에서 같은 수직선으로 바구니 상부 안전점까지 LIN 후퇴한다."""
        if self._basket_place is None:
            self._send_home()
            return
        self._basket_retract_tried = True
        retract = self._basket_place.copy()
        retract[2] += (
            float(self.get_parameter("basket_approach_height_m").value)
            + float(self.get_parameter("basket_retract_height_m").value))
        self._send_rmp_goal(retract, "BASKET_RETRACT")

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

    def _send_home(
        self, retry_after_home: bool = False, fast: bool = False
    ) -> None:
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
            "rmp_home": {"id": self._pending_id, "fast": bool(fast)},
        })))

    def _maybe_single_shot_off(self) -> None:
        """h 원샷: 사이클 끝나면 게이트를 끈다 — 다시 h 를 눌러야 다음 과실을 잡는다.
        계속 켜두면 이동 중 재인식으로 시퀀스가 흔들리는 걸 막는다."""
        if bool(self.get_parameter("single_shot_harvest").value):
            self._harvest_enabled = False
            self.get_logger().info("원샷 수확 종료 — h 다시 눌러야 다음 과실")

    _BASKET_PHASE_STATES = (
        "PRE_PLACE_BED_VIEW", "WAIT_BASKET_AT_BED_VIEW", "BASKET_AZIMUTH_ALIGN",
        "BASKET_APPROACH", "PLACE_RELEASING",
        "BASKET_RETRACT",
        "POST_PLACE_BED_VIEW",
    )

    def _abort_basket(self, reason: str) -> None:
        """바구니 단계 실패 — 토마토 재접근으로 되돌아가지 않는다.

        수확물을 놓치지 않도록 그리퍼는 계속 닫아 둔다. 릴리즈점 부근이면 먼저
        바구니 상부로 LIN 후퇴하고, 그게 불가능하면 홈 복귀를 시도한다.
        """
        self._deadline_ns = 0
        self.get_logger().warning(f"바구니 이송 실패({reason})")
        # 릴리즈점에 실제로 도달했을 때만 상부 후퇴가 의미 있다. 도달 전에
        # 시도하면 접힌 자세에서 1.3m 직교 LIN이 되어 Pilz가 거부하거나
        # 타임아웃까지 붙잡는다(2026-07-25 sim_diag_161712: 85초 낭비).
        if (self._basket_release_reached
                and self._basket_place is not None
                and not self._basket_retract_tried):
            self.get_logger().warning("→ 바구니 상부로 LIN 후퇴 후 홈 복귀")
            self._start_basket_retract()
            return
        if self._state in ("PRE_PLACE_BED_VIEW", "POST_PLACE_BED_VIEW", "GO_HOME"):
            # 접힘 홈 자체가 실패한 상태에서 같은 홈 명령을 반복하지 않는다.
            self._fail_place_holding(reason)
            return
        self.get_logger().warning("→ 그리퍼를 닫은 채 홈 복귀")
        self._transition("HARVEST_FAILED")
        self._send_home()

    def _fail_place_holding(self, reason: str) -> None:
        """홈 복귀조차 실패 — 수확물을 잡은 채 정지한다(그리퍼 열지 않음)."""
        self._deadline_ns = 0
        self._basket_place = None
        self._basket_received_ns = 0
        self._place_home_done = False
        self.get_logger().error(
            f"바구니 이송 안전 실패({reason}) — 수확물을 잡은 채 정지, "
            "운영자 확인 필요")
        self._transition("HARVEST_FAILED")
        self._transition("PLACE_FAILED_HOLDING", stop=True)

    def _abort_to_home(self, reason: str) -> None:
        """실패해도 팔을 홈으로 돌려 다음 과실을 계속 시도하게 한다(데모 연속 사이클).
        이미 홈 복귀 중(GO_HOME)에 또 실패하면 무한루프 방지로 멈추기만 한다."""
        if self._state in self._BASKET_PHASE_STATES:
            self._abort_basket(reason)
            return
        was_going_home = self._state == "GO_HOME"
        # 재접근 재시도: 홈까지 가지 않고 APPROACH 안전점으로 후퇴한 뒤 다시 파지한다.
        # 도달하면 기존 흐름(APPROACH→PREGRASP→GRASP)이 그대로 이어진다.
        retry_max = int(self.get_parameter("approach_retry_max").value)
        if (not was_going_home
                and self._state != "APPROACH"
                and self._approach_retry_count < retry_max):
            self._approach_retry_count += 1
            self.get_logger().warning(
                f"수확 실패({reason}) — 접근지점 복귀 후 재시도 "
                f"{self._approach_retry_count}/{retry_max}")
            self._deadline_ns = 0
            self._grasp_check_sent = False
            # 부분 파지 해제: 그리퍼 개방 + 칼날 개방 후 접근점으로 되짚어 나온다.
            self._isaac_command_pub.publish(String(data=json.dumps({
                "gripper": {"closed": False},
                "blade": float(self.get_parameter("blade_open_deg").value),
            })))
            self._send_rmp_goal(self._approach_target, "APPROACH")
            return
        # 2단계: 접근점 재시도 소진 → 베드뷰로 재관측한다. retry_after_home=True로
        # 홈→(코디네이터가)베드뷰→좌표 재수신 경로를 탄다. 새 좌표는 WAIT_TARGET의
        # 5프레임 고정 게이트를 다시 통과해야 파지를 시작한다.
        bed_max = int(self.get_parameter("bed_view_retry_max").value)
        if not was_going_home and self._bed_view_retry_count < bed_max:
            self._bed_view_retry_count += 1
            self.get_logger().warning(
                f"수확 실패({reason}) — 접근점 재시도 소진, 베드뷰 재관측 "
                f"{self._bed_view_retry_count}/{bed_max}")
            self._deadline_ns = 0
            self._grasp_check_sent = False
            self._transition("HARVEST_FAILED")
            self._isaac_command_pub.publish(String(data=json.dumps({
                "gripper": {"closed": False},
                "blade": float(self.get_parameter("blade_open_deg").value),
            })))
            self._send_home(retry_after_home=True)
            return
        # 3단계: 모든 재시도 소진 — 최종 홈 복귀. 다음 사이클 위해 카운터 리셋.
        self._bed_view_retry_count = 0
        self.get_logger().warning(f"수확 실패({reason}) — 홈 복귀 후 다음 시도")
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
