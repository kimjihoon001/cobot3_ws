"""바구니 이송 시퀀스 회귀 테스트.

2026-07-25 sim_diag_154400에서 확인한 실패를 코드 수준에서 고정한다.
  - 완전히 편 자세에서 등 뒤 바구니로 joint_1을 돌려 베드를 관통했다.
  - 컨트롤러가 106° 어긋난 채 성공을 보고해 다음 LIN이 엉뚱한 곳에서 계획됐다.
  - 실패가 토마토 APPROACH 재시도로 이어져 그리퍼가 열렸다.
"""
import json

import numpy as np
import pytest
import rclpy

from harvest_vision.manipulator_target_node import ManipulatorTargetNode


class _Recorder:
    """_isaac_command_pub 대체 — 발행된 명령을 모두 보관한다."""

    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(json.loads(msg.data))

    def targets(self):
        return [m["rmp_target"] for m in self.messages if "rmp_target" in m]

    def aligns(self):
        return [m["azimuth_align"] for m in self.messages
                if "azimuth_align" in m]

    def phases(self):
        return [t["phase"] for t in self.targets()]


@pytest.fixture
def node():
    rclpy.init()
    node = ManipulatorTargetNode()
    node._isaac_command_pub = _Recorder()
    yield node
    node.destroy_node()
    rclpy.shutdown()


def _motion_done(node, phase="MOVE"):
    """브리지의 도달 응답을 흉내낸다."""
    node._status_callback(_string(json.dumps({
        "id": node._pending_id, "phase": phase, "reached": True})))


def _string(data):
    from std_msgs.msg import String
    return String(data=data)


def _arm_at_basket(node, slot_z=0.288):
    node._basket_place = np.array([-1.209, 0.536, slot_z])
    node._basket_received_ns = node.get_clock().now().nanoseconds


def test_no_reachable_basket_goes_home_from_the_folded_pose(node):
    """접힘 자세에서 판단: 도달 가능한 바구니가 없으면 팔을 펴지 않고 홈으로."""
    node._transition("RETRACT_LIN")
    _motion_done(node)
    assert node._state == "PRE_PLACE_BED_VIEW"
    _motion_done(node)                           # 접힘 도달 → 판단 지점
    assert node._state == "WAIT_BASKET_AT_BED_VIEW"
    node._isaac_command_pub.messages.clear()

    node._start_place()                          # 좌표 없음 → 분기

    assert node._state == "GO_HOME"
    assert node._isaac_command_pub.aligns() == []
    assert node._isaac_command_pub.phases() == []
    assert all("gripper" not in message
               for message in node._isaac_command_pub.messages)


def test_fold_keeps_j1_and_is_confirmed_by_response(node):
    """RETRACT_LIN → 방위 유지 접기. 성공 응답 전에는 바구니 명령 금지."""
    node._transition("RETRACT_LIN")
    _motion_done(node)

    assert node._state == "PRE_PLACE_BED_VIEW"
    assert node._place_home_done is False
    # joint_1을 180°로 되돌리는 rmp_home이 아니라 방위 유지 접기여야 한다.
    assert "fold_keep_j1" in node._isaac_command_pub.messages[-1]
    assert all("rmp_home" not in message
               for message in node._isaac_command_pub.messages)

    # 접는 중 좌표가 들어와도 정렬·접근을 발행하지 않는다.
    _arm_at_basket(node)
    assert node._isaac_command_pub.aligns() == []
    assert "BASKET_APPROACH" not in node._isaac_command_pub.phases()

    _motion_done(node)                      # 접힘 성공 응답 → 판단 지점
    assert node._place_home_done is True
    assert node._state == "WAIT_BASKET_AT_BED_VIEW"

    # WAIT 진입 전에 받은 좌표는 폐기되므로 정지 후 새 좌표가 필요하다.
    _arm_at_basket(node)
    node._start_place()                     # 바구니 있음 → 방위 회전부터
    assert node._state == "BASKET_AZIMUTH_ALIGN"


def test_basket_wait_uses_its_own_short_timeout(node):
    """바구니 대기는 일반 모션 타임아웃(10초)이 아니라 전용 1~2초를 쓴다."""
    node._transition("RETRACT_LIN")
    _motion_done(node)                      # → PRE_PLACE_BED_VIEW
    before = node.get_clock().now().nanoseconds
    _motion_done(node)                      # 접힘 도달 → 대기 시작

    assert node._state == "WAIT_BASKET_AT_BED_VIEW"
    window = (node._deadline_ns - before) / 1e9
    expected = float(node.get_parameter("basket_wait_timeout_sec").value)
    assert expected <= 2.0
    assert window == pytest.approx(expected, abs=0.2)
    assert window < float(node.get_parameter("motion_timeout_sec").value)


def test_azimuth_align_runs_before_the_arm_extends(node):
    """정렬 성공 전에는 팔을 펴는 BASKET_APPROACH를 발행하지 않는다."""
    node._place_home_done = True
    node._transition("WAIT_BASKET_AT_BED_VIEW")
    _arm_at_basket(node)
    node._start_place()

    assert node._state == "BASKET_AZIMUTH_ALIGN"
    aligns = node._isaac_command_pub.aligns()
    assert len(aligns) == 1
    assert aligns[0]["position"][2] == pytest.approx(0.272, abs=1e-6)
    assert node._isaac_command_pub.targets() == []

    _motion_done(node)                      # 정렬 성공 응답
    assert node._state == "BASKET_APPROACH"
    assert node._isaac_command_pub.phases() == ["BASKET_APPROACH"]


def test_release_height_is_lowered_3cm_from_the_previous_setting(node):
    """슬롯 z=0.288 → 릴리즈 z=0.272. 기존 값에서 3cm 추가 하향.

    2026-07-26 사용자 지시로 +14mm에서 -16mm로 변경했다.
    """
    node._place_home_done = True
    node._transition("WAIT_BASKET_AT_BED_VIEW")
    _arm_at_basket(node, slot_z=0.288)
    node._start_place()
    _motion_done(node)                      # 방위 정렬 성공

    targets = node._isaac_command_pub.targets()
    assert [t["phase"] for t in targets] == ["BASKET_APPROACH"]
    assert targets[0]["position"][2] == pytest.approx(0.272, abs=1e-6)
    # IW가 발행한 KLT 중심에서 XY 평면상 MM 원점 방향으로 정확히 80mm 이동한다.
    center_xy = np.array([-1.209, 0.536])
    expected_xy = center_xy * (1.0 - 0.08 / np.linalg.norm(center_xy))
    actual_xy = np.asarray(targets[0]["position"][:2])
    assert actual_xy == pytest.approx(expected_xy, abs=1e-6)
    assert np.linalg.norm(actual_xy - center_xy) == pytest.approx(
        0.08, abs=1e-6)
    # 슬롯 pose는 이미 KLT 윗면 +0.0331 m다. TCP를 기존보다 30mm 낮춘다.
    klt_top = 0.288 - 0.0331
    assert targets[0]["position"][2] - 0.057 - klt_top == pytest.approx(
        -0.040, abs=1e-3)

    _motion_done(node)                      # 릴리즈점 도달 → 손목 회전
    assert node._state == "BASKET_WRIST_ROTATE"
    wrist = node._isaac_command_pub.messages[-1]["wrist_rotate"]
    assert wrist["angle_rad"] == pytest.approx(np.pi)
    assert all("gripper" not in message
               for message in node._isaac_command_pub.messages)

    _motion_done(node, "BASKET_WRIST_ROTATE")  # 손목 회전 확인 후 릴리즈
    assert node._state == "PLACE_RELEASING"
    assert node._isaac_command_pub.messages[-1] == {"gripper": {"closed": False}}
    # 슬롯 중심(z=0.288)으로 내려가는 BASKET_PLACE 명령은 존재하지 않는다.
    assert "BASKET_PLACE" not in node._isaac_command_pub.phases()
    assert all(
        target["position"][2] > 0.27
        for target in node._isaac_command_pub.targets())


def test_basket_phase_does_not_reuse_harvest_orientation(node):
    """바구니 자세는 수확용 orientation을 재사용하지 않는다(아래보기 고정)."""
    node._harvest_orientation = [0.11, 0.22, 0.33, 0.91]
    node._place_home_done = True
    node._transition("WAIT_BASKET_AT_BED_VIEW")
    _arm_at_basket(node)
    node._start_place()
    _motion_done(node)                      # 방위 정렬 성공

    orientation = node._isaac_command_pub.targets()[0]["tool_orientation"]
    assert orientation == pytest.approx(
        [1.0, 0.0, 0.0, 0.0])
    assert orientation != node._harvest_orientation


def test_successful_place_folds_before_home(node):
    """릴리즈 후 수직 후퇴→방위 유지 접기 성공→HOME 순서를 강제한다."""
    node._place_home_done = True
    node._transition("WAIT_BASKET_AT_BED_VIEW")
    _arm_at_basket(node)
    node._start_place()
    _motion_done(node)                      # 방위 정렬
    _motion_done(node)                      # 릴리즈점 → 손목 회전
    _motion_done(node, "BASKET_WRIST_ROTATE")

    node._status_callback(_string(json.dumps({"gripper": 0.0})))
    assert node._state == "BASKET_RETRACT"
    assert node._isaac_command_pub.phases()[-1] == "BASKET_RETRACT"

    _motion_done(node)                      # 수직 후퇴 완료
    assert node._state == "POST_PLACE_BED_VIEW"
    fold = node._isaac_command_pub.messages[-1]["fold_keep_j1"]
    assert fold["phase"] == "POST_PLACE_BED_VIEW"
    assert fold["fast"] is True
    assert all("rmp_home" not in message
               for message in node._isaac_command_pub.messages)

    _motion_done(node, "POST_PLACE_BED_VIEW")  # 접기 성공 확인
    assert node._state == "GO_HOME"
    home = node._isaac_command_pub.messages[-1]["rmp_home"]
    assert home["fast"] is True


def test_basket_failure_retreats_and_never_retries_tomato(node):
    """바구니 실패는 토마토 APPROACH 재시도로 이어지지 않고 그리퍼도 안 연다."""
    node._place_home_done = True
    node._approach_target = np.array([0.808, 0.690, 0.917])
    node._transition("WAIT_BASKET_AT_BED_VIEW")
    _arm_at_basket(node)
    node._start_place()
    _motion_done(node)                      # 방위 정렬 성공
    _motion_done(node)                      # 릴리즈점 도달 → 손목 회전
    _motion_done(node, "BASKET_WRIST_ROTATE")
    node._isaac_command_pub.messages.clear()

    node._status_callback(_string(json.dumps({
        "id": node._pending_id, "phase": "ERROR_IK_PATH", "reached": False})))

    assert node._isaac_command_pub.phases() == ["BASKET_RETRACT"]
    assert node._state == "BASKET_RETRACT"
    assert "APPROACH" not in node._isaac_command_pub.phases()
    assert all("gripper" not in message
               for message in node._isaac_command_pub.messages)


def test_failure_before_release_skips_the_impossible_lin_retreat(node):
    """릴리즈점 도달 전 실패는 1.3m 직교 LIN 후퇴 대신 곧바로 홈 복귀한다."""
    node._place_home_done = True
    node._transition("WAIT_BASKET_AT_BED_VIEW")
    _arm_at_basket(node)
    node._start_place()                     # BASKET_AZIMUTH_ALIGN
    node._isaac_command_pub.messages.clear()

    node._status_callback(_string(json.dumps({
        "id": node._pending_id, "phase": "ERROR_IK_PATH", "reached": False})))

    assert "BASKET_RETRACT" not in node._isaac_command_pub.phases()
    assert node._state == "GO_HOME"
    assert all("gripper" not in message
               for message in node._isaac_command_pub.messages)


def test_fold_failure_keeps_fruit_and_stops(node):
    """접힘 홈 실패 시 바구니 접근을 시작하지 않고 그리퍼를 닫은 채 정지한다."""
    node._transition("RETRACT_LIN")
    _motion_done(node)
    assert node._state == "PRE_PLACE_BED_VIEW"
    node._isaac_command_pub.messages.clear()

    node._status_callback(_string(json.dumps({
        "id": node._pending_id,
        "phase": "ERROR_MOVEIT_UNAVAILABLE",
        "reached": False})))

    assert node._state == "PLACE_FAILED_HOLDING"
    assert node._place_home_done is False
    assert node._isaac_command_pub.messages == []


def _publish_basket(node, x, y, z=0.588):
    """map 프레임 슬롯 pose 발행을 흉내낸다(TF 없이 base로 그대로 통과)."""
    from geometry_msgs.msg import PoseStamped

    msg = PoseStamped()
    msg.header.frame_id = "map"
    msg.pose.position.x = x
    msg.pose.position.y = y
    msg.pose.position.z = z
    msg.pose.orientation.w = 1.0

    class _Passthrough:
        @staticmethod
        def transform(source, target_frame, timeout=None):
            out = PoseStamped()
            out.header.frame_id = target_frame
            out.pose.position.x = source.pose.position.x
            out.pose.position.y = source.pose.position.y
            out.pose.position.z = source.pose.position.z - 0.3
            return out

    node._buffer.transform = _Passthrough.transform
    node._basket_callback(msg)


def test_place_waits_until_the_iw_pose_stops_moving(node):
    """IW가 이동 중(발행 좌표가 계속 바뀜)이면 플레이스를 시작하지 않는다.

    2026-07-25: IW가 목표까지 0.4 m 남은 상태에서도 pose를 발행했고, 그 좌표를
    래치하면 IW가 계속 전진해 과실이 칸 뒤쪽 격벽으로 떨어진다.
    """
    node._place_home_done = True
    node._transition("WAIT_BASKET_AT_BED_VIEW")
    node._isaac_command_pub.messages.clear()

    _publish_basket(node, -1.10, 0.45)          # 첫 수신 → 안정 0초
    _publish_basket(node, -1.10, 0.65)          # 20cm 이동 → 안정 타이머 리셋
    assert node._state == "WAIT_BASKET_AT_BED_VIEW"
    assert node._isaac_command_pub.aligns() == []   # 플레이스 미시작

    # 좌표가 멈춘 뒤 필요한 안정 시간이 지나면 시작한다.
    node._basket_stable_since_ns -= int(
        (float(node.get_parameter("basket_stable_sec").value) + 0.1) * 1e9)
    _publish_basket(node, -1.10, 0.65)
    assert node._state == "BASKET_AZIMUTH_ALIGN"
    assert len(node._isaac_command_pub.aligns()) == 1


def test_wait_basket_discards_stability_accumulated_while_folding(node):
    """접는 동안 쌓인 안정 시간으로 WAIT 진입 직후 출발하면 안 된다."""
    node._place_home_done = False
    node._transition("PRE_PLACE_BED_VIEW")
    _publish_basket(node, -1.10, 0.45)
    node._basket_stable_since_ns -= int(
        (float(node.get_parameter("basket_stable_sec").value) + 0.1) * 1e9)

    node._place_home_done = True
    node._transition("WAIT_BASKET_AT_BED_VIEW")
    node._isaac_command_pub.messages.clear()
    _publish_basket(node, -1.10, 0.45)

    assert node._state == "WAIT_BASKET_AT_BED_VIEW"
    assert node._isaac_command_pub.aligns() == []


def test_slowly_creeping_iw_is_never_treated_as_stopped(node):
    """매 프레임 8 mm씩 계속 이동하는 IW를 정차로 오인하지 않는다.

    직전 샘플과만 비교하면 8 mm < 허용오차 10 mm 라 매번 "안정"으로 보여
    영원히 정차 판정이 난다. 구간 기준 좌표 대비 누적 변위로 봐야 한다.
    """
    node._place_home_done = True
    node._transition("WAIT_BASKET_AT_BED_VIEW")
    node._isaac_command_pub.messages.clear()

    required = float(node.get_parameter("basket_stable_sec").value)
    y = 0.45
    for step in range(40):                     # 프레임마다 8 mm 전진
        _publish_basket(node, -1.10, y)
        y += 0.008
        # 시간이 충분히 흘러도(구간 길이 초과) 정차로 보면 안 된다.
        node._basket_stable_since_ns -= int((required + 0.1) * 1e9 / 20)

    assert node._state == "WAIT_BASKET_AT_BED_VIEW"
    assert node._isaac_command_pub.aligns() == []

    # 멈추면(같은 좌표 반복) 정상적으로 정차로 판정한다.
    _publish_basket(node, -1.10, y)
    node._basket_stable_since_ns -= int((required + 0.1) * 1e9)
    _publish_basket(node, -1.10, y)
    assert node._state == "BASKET_AZIMUTH_ALIGN"
