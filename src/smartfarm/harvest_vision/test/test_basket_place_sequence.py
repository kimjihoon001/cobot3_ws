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
    assert aligns[0]["position"][2] == pytest.approx(0.342, abs=1e-6)
    assert node._isaac_command_pub.targets() == []

    _motion_done(node)                      # 정렬 성공 응답
    assert node._state == "BASKET_APPROACH"
    assert node._isaac_command_pub.phases() == ["BASKET_APPROACH"]


def test_release_height_puts_the_tool_tip_3cm_above_the_klt(node):
    """슬롯 z=0.288 → 릴리즈 z=0.342. 그리퍼 끝이 KLT 윗면 3cm 위."""
    node._place_home_done = True
    node._transition("WAIT_BASKET_AT_BED_VIEW")
    _arm_at_basket(node, slot_z=0.288)
    node._start_place()
    _motion_done(node)                      # 방위 정렬 성공

    targets = node._isaac_command_pub.targets()
    assert [t["phase"] for t in targets] == ["BASKET_APPROACH"]
    assert targets[0]["position"][2] == pytest.approx(0.342, abs=1e-6)
    # IW가 발행한 KLT 중심에서 수평면상 MM 원점 방향으로 정확히 2cm 이동한다.
    center_xy = np.array([-1.209, 0.536])
    expected_xy = center_xy * (1.0 - 0.02 / np.linalg.norm(center_xy))
    actual_xy = np.asarray(targets[0]["position"][:2])
    assert actual_xy == pytest.approx(expected_xy, abs=1e-6)
    assert np.linalg.norm(actual_xy - center_xy) == pytest.approx(
        0.02, abs=1e-6)
    # 슬롯 pose는 이미 KLT 윗면 +0.0331 m다. 툴 끝(TCP 앞 0.057 m)이 윗면에서
    # 0.030 m 뜨는 높이여야 한다.
    klt_top = 0.288 - 0.0331
    assert targets[0]["position"][2] - 0.057 - klt_top == pytest.approx(
        0.030, abs=1e-3)

    _motion_done(node)                      # 릴리즈점 도달
    assert node._state == "PLACE_RELEASING"
    assert node._isaac_command_pub.messages[-1] == {"gripper": {"closed": False}}
    # 슬롯 중심(z=0.288)으로 내려가는 BASKET_PLACE 명령은 존재하지 않는다.
    assert "BASKET_PLACE" not in node._isaac_command_pub.phases()
    assert all(
        target["position"][2] > 0.30
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
    assert orientation == [1.0, 0.0, 0.0, 0.0]
    assert orientation != node._harvest_orientation


def test_successful_place_folds_before_home(node):
    """릴리즈 후 수직 후퇴→방위 유지 접기 성공→HOME 순서를 강제한다."""
    node._place_home_done = True
    node._transition("WAIT_BASKET_AT_BED_VIEW")
    _arm_at_basket(node)
    node._start_place()
    _motion_done(node)                      # 방위 정렬
    _motion_done(node)                      # 릴리즈점

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
    _motion_done(node)                      # 릴리즈점 도달 → PLACE_RELEASING
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
