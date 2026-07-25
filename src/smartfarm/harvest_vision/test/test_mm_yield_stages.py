"""MM 피항 단계 FSM 회귀 테스트.

2026-07-25 sim_diag_204420: 플레이스 후 (-2.90,-3.85) 단일 NavigateToPose를
보냈고, MM이 좁은 배드 통로에서 이랑 inflation(253) 안으로 들어가
`GridBased: failed to create plan` → 120초 뒤 ERROR_MM_YIELD_FAILED.
IW는 허가를 못 받아 WAITING_MM_YIELD로 제자리에 남았다.
"""
import json
import math

import pytest
import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from std_msgs.msg import Bool

from harvest_vision.nav_harvest_test_node import NavHarvestTestNode


class _Recorder:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg.data)


def _last_state(node):
    """상태 발행은 {"state": ...} JSON이다."""
    return json.loads(node._status_pub.messages[-1])["state"]


class _FakeCostmapMeta:
    resolution = 0.05
    size_x = 400
    size_y = 400

    class origin:
        class position:
            x = -10.0
            y = -13.0


class _FakeCostmap:
    """배드 이랑(x=±1.45 부근)만 253인 단순 격자. y>-4.5는 전부 free."""

    metadata = _FakeCostmapMeta()

    def __init__(self):
        info = self.metadata
        self.data = bytearray(info.size_x * info.size_y)
        for j in range(info.size_y):
            y = info.origin.position.y + j * info.resolution
            if y > -4.5:                      # 교차통로 — 전부 free
                continue
            for i in range(info.size_x):
                x = info.origin.position.x + i * info.resolution
                # 이랑 ±0.21 + inflation 0.45 → 중심에서 0.66m 안쪽이 253
                if any(abs(x - ridge) <= 0.66 for ridge in (-4.35, -1.45,
                                                            1.45, 4.35)):
                    self.data[j * info.size_x + i] = 253


class _FakeTransform:
    def __init__(self, x, y):
        self.transform = type("T", (), {})()
        self.transform.translation = type("V", (), {"x": x, "y": y, "z": 0.0})()
        self.transform.rotation = type(
            "Q", (), {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0})()


@pytest.fixture
def node():
    rclpy.init()
    node = NavHarvestTestNode()
    node._iw_yield_complete_pub = _Recorder()
    node._status_pub = _Recorder()
    node._sent = []

    def fake_send(goal):
        node._sent.append(goal)

        class _F:
            def add_done_callback(self, cb):
                pass
        return _F()

    node._nav_client.send_goal_async = fake_send
    node._nav_client.server_is_ready = lambda: True
    node._costmap = _FakeCostmap()
    node._buffer.lookup_transform = lambda *a, **k: _FakeTransform(-0.47, -8.26)
    yield node
    node.destroy_node()
    rclpy.shutdown()


def _ready_for_yield(node):
    node._iw_full = True
    node._mobility_ready = True


def _accept(node, goal_id):
    """Nav2가 목표를 수락한 것으로 처리한다."""
    node._yield_goal_pending = False
    node._yield_goal_id = goal_id
    node._yield_goal_ids.add(goal_id)
    node._known_goals.add(goal_id)
    node._active_goal = goal_id


def _status(node, goal_id, status):
    msg = GoalStatusArray()
    entry = GoalStatus()
    entry.status = status
    entry.goal_info.goal_id.uuid = list(goal_id)
    msg.status_list = [entry]
    node._status_initialized = True
    node._nav_status_callback(msg)


def _pose(goal):
    p = goal.pose.pose
    yaw = 2.0 * math.atan2(p.orientation.z, p.orientation.w)
    return round(p.position.x, 3), round(p.position.y, 3), round(yaw, 3)


def test_yield_does_not_start_before_place(node):
    """적재(만재) 전에는 피항을 시작하지 않는다."""
    node._iw_full = False
    node._mobility_ready = True
    node._iw_yield_request_callback(Bool(data=True))
    assert node._sent == []
    assert node._yield_stages == []


def test_yield_requires_arm_home(node):
    """팔이 HOME(mobility_ready)이 아니면 1단계를 시작하지 않는다."""
    node._iw_full = True
    node._mobility_ready = False
    node._iw_yield_request_callback(Bool(data=True))
    assert node._sent == []
    assert _last_state(node) == "ERROR_MM_YIELD_ARM_NOT_HOME"


def test_stage_sequence_center_corridor_turn_yield(node):
    """1단계 성공 → 회전 단계 → 최종 이동. 각 단계는 앞 단계 성공 후에만."""
    _ready_for_yield(node)
    node._iw_yield_request_callback(Bool(data=True))

    labels = [stage[0] for stage in node._yield_stages]
    assert labels == ["LANE_CENTER", "CORRIDOR", "TURN", "YIELD"]
    assert len(node._sent) == 1                    # 1단계만 발행

    # 1단계: 좁은 통로에서 레인 중심으로만 이동(Y 유지)
    x, y, yaw = _pose(node._sent[0])
    assert y == pytest.approx(-8.26, abs=1e-3)
    assert abs(x) < 0.2
    assert yaw == pytest.approx(math.pi / 2, abs=1e-3)

    _accept(node, b"\x01" * 16)
    _status(node, b"\x01" * 16, GoalStatus.STATUS_SUCCEEDED)
    assert len(node._sent) == 2
    _, y2, yaw2 = _pose(node._sent[1])             # 2단계: 교차통로까지 북진
    assert y2 == pytest.approx(-3.85, abs=1e-3)
    assert yaw2 == pytest.approx(math.pi / 2, abs=1e-3)
    assert node._iw_yield_complete_pub.messages == []

    _accept(node, b"\x02" * 16)
    _status(node, b"\x02" * 16, GoalStatus.STATUS_SUCCEEDED)
    assert len(node._sent) == 3
    x3, y3, yaw3 = _pose(node._sent[2])            # 3단계: 통로 중심에서 회전
    assert (x3, y3) == _pose(node._sent[1])[:2]
    assert abs(abs(yaw3) - math.pi) < 1e-3
    assert node._iw_yield_complete_pub.messages == []

    _accept(node, b"\x03" * 16)
    _status(node, b"\x03" * 16, GoalStatus.STATUS_SUCCEEDED)
    assert len(node._sent) == 4
    x4, y4, _ = _pose(node._sent[3])               # 4단계: 최종 피항점
    assert x4 == pytest.approx(-2.90, abs=0.61)
    assert y4 == pytest.approx(-3.85, abs=1e-3)
    assert node._iw_yield_complete_pub.messages == []   # 아직 허가 없음

    _accept(node, b"\x04" * 16)
    _status(node, b"\x04" * 16, GoalStatus.STATUS_SUCCEEDED)
    assert node._iw_yield_complete_pub.messages == [True]
    assert _last_state(node) == "WAITING_IW_RETURN"


def test_middle_stage_failure_keeps_iw_waiting(node):
    """중간 단계 실패 시 다음 단계로 넘어가지 않고 complete=False를 유지한다."""
    _ready_for_yield(node)
    node._iw_yield_request_callback(Bool(data=True))
    _accept(node, b"\x01" * 16)
    _status(node, b"\x01" * 16, GoalStatus.STATUS_SUCCEEDED)
    _accept(node, b"\x02" * 16)
    _status(node, b"\x02" * 16, GoalStatus.STATUS_ABORTED)

    assert len(node._sent) == 2                     # 3단계 미발행
    assert node._iw_yield_complete_pub.messages == [False]
    assert _last_state(node).startswith("ERROR_MM_YIELD_2_CORRIDOR")
    assert node._yield_stages == []


def test_stale_goal_result_is_ignored(node):
    """지난 단계의 늦은 SUCCEEDED가 다음 단계를 완료 처리하지 않는다."""
    _ready_for_yield(node)
    node._iw_yield_request_callback(Bool(data=True))
    _accept(node, b"\x01" * 16)
    _status(node, b"\x01" * 16, GoalStatus.STATUS_SUCCEEDED)
    _accept(node, b"\x02" * 16)

    _status(node, b"\x01" * 16, GoalStatus.STATUS_SUCCEEDED)   # stale
    assert len(node._sent) == 2
    assert node._yield_stage_index == 1
    assert node._iw_yield_complete_pub.messages == []

    _status(node, b"\x01" * 16, GoalStatus.STATUS_ABORTED)     # stale 실패
    assert node._iw_yield_complete_pub.messages == []
    assert node._yield_stages != []


def test_blocked_start_reports_error_without_moving(node):
    """현재 위치가 이미 inflated면 목표를 보내지 않고 오류를 낸다."""
    _ready_for_yield(node)
    node._buffer.lookup_transform = lambda *a, **k: _FakeTransform(-1.45, -8.26)
    node._iw_yield_request_callback(Bool(data=True))

    assert node._sent == []
    assert node._iw_yield_complete_pub.messages == [False]
    assert _last_state(node) == "ERROR_MM_YIELD_START_BLOCKED"
