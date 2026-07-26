"""MM 피항 단계 FSM 회귀 테스트.

2026-07-25 sim_diag_204420: 플레이스 후 (-2.90,-3.85) 단일 NavigateToPose를
보냈고, MM이 좁은 배드 통로에서 이랑 inflation(253) 안으로 들어가
`GridBased: failed to create plan` → 120초 뒤 ERROR_MM_YIELD_FAILED.
IW는 허가를 못 받아 WAITING_MM_YIELD로 제자리에 남았다.

2026-07-25 23:38: 중간 waypoint(0.43 m짜리 정렬 goal)가 MM 몸길이(0.96 m)보다
짧아 DWB가 40초간 제자리에서 진동했다. 같은 날 사용자가 최종 목적지만 찍었을
때는 Nav2가 정상 주행했다 → 경로는 **사용자가 지정한 최종 피항 자세 하나**만
보낸다. 검사 기준은 lethal/occupied(254)뿐이며 inflation(253)만으로는
실패시키지 않는다 — 수확 자세는 이랑 옆이라 시작 footprint가 항상 inscribed
밴드에 얹혀 있기 때문이다.
"""
import json
import math

import pytest
import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from std_msgs.msg import Bool

from harvest_vision.harvest_fsm_node import HarvestFsmNode


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
    """이랑을 lethal(254) + inflation(253) 두 층으로 재현한 격자.

    이랑 중심 x=±1.45/±4.35, 반폭 0.21 → |dx|<=0.21 이 254(실제 장애물),
    거기서 0.40 m 까지가 253(inscribed inflation). y>-4.5 는 교차통로라 전부
    free. 수확 자세(x=-0.54)는 footprint 좌측이 253 에 얹히지만 254 는 아니다.
    """

    metadata = _FakeCostmapMeta()

    def __init__(self, ridges=(-4.35, -1.45, 1.45, 4.35), corridor_y=-4.5):
        info = self.metadata
        self.data = bytearray(info.size_x * info.size_y)
        for j in range(info.size_y):
            y = info.origin.position.y + j * info.resolution
            if y > corridor_y:
                continue
            for i in range(info.size_x):
                x = info.origin.position.x + i * info.resolution
                near = min(abs(x - ridge) for ridge in ridges)
                if near <= 0.21:
                    self.data[j * info.size_x + i] = 254
                elif near <= 0.61:
                    self.data[j * info.size_x + i] = 253

    def block(self, x0, x1, y0, y1, cost=254):
        """지정 사각형을 강제로 채운다(테스트용 장애물 주입)."""
        info = self.metadata
        for j in range(info.size_y):
            y = info.origin.position.y + j * info.resolution
            if not (y0 <= y <= y1):
                continue
            for i in range(info.size_x):
                x = info.origin.position.x + i * info.resolution
                if x0 <= x <= x1:
                    self.data[j * info.size_x + i] = cost


class _FakeTransform:
    def __init__(self, x, y):
        self.transform = type("T", (), {})()
        self.transform.translation = type("V", (), {"x": x, "y": y, "z": 0.0})()
        self.transform.rotation = type(
            "Q", (), {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0})()


@pytest.fixture
def node():
    rclpy.init()
    node = HarvestFsmNode()
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


def _string(data):
    from std_msgs.msg import String
    return String(data=data)


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


def test_yield_request_recovers_full_state_after_coordinator_restart(node):
    """IW가 대기 중이면 코디네이터 재시작 후에도 피항을 재개한다."""
    node._iw_full = False
    node._placed = False
    node._mobility_ready = True

    node._iw_yield_request_callback(Bool(data=True))

    assert node._iw_full is True
    assert node._placed is True
    assert [stage[0] for stage in node._yield_stages] == ["YIELD"]


def test_yield_requires_arm_home(node):
    """팔이 HOME(mobility_ready)이 아니면 1단계를 시작하지 않는다."""
    node._iw_full = True
    node._mobility_ready = False
    node._iw_yield_request_callback(Bool(data=True))
    assert node._sent == []
    assert _last_state(node) == "ERROR_MM_YIELD_ARM_NOT_HOME"


def test_single_goal_to_the_user_specified_yield_pose(node):
    """사용자가 지정한 최종 피항 자세로 goal 하나만 보내고, 성공해야 IW 허가."""
    _ready_for_yield(node)
    node._iw_yield_request_callback(Bool(data=True))

    assert [stage[0] for stage in node._yield_stages] == ["YIELD"]
    assert len(node._sent) == 1

    x, y, yaw = _pose(node._sent[0])
    assert x == pytest.approx(
        float(node.get_parameter("iw_yield_goal_x").value), abs=1e-3)
    assert y == pytest.approx(
        float(node.get_parameter("iw_yield_goal_y").value), abs=1e-3)
    assert yaw == pytest.approx(
        float(node.get_parameter("iw_yield_goal_yaw").value), abs=1e-3)
    assert node._iw_yield_complete_pub.messages == []   # 도착 전에는 허가 없음

    _accept(node, b"\x01" * 16)
    _status(node, b"\x01" * 16, GoalStatus.STATUS_SUCCEEDED)
    assert node._iw_yield_complete_pub.messages == [True]
    assert _last_state(node) == "WAITING_IW_RETURN"
def test_nav_failure_keeps_iw_waiting(node):
    """주행이 실패하면 complete=False를 유지하고 IW를 출발시키지 않는다."""
    _ready_for_yield(node)
    node._iw_yield_request_callback(Bool(data=True))
    _accept(node, b"\x01" * 16)
    _status(node, b"\x01" * 16, GoalStatus.STATUS_ABORTED)

    assert node._iw_yield_complete_pub.messages == [False]
    assert _last_state(node).startswith("ERROR_MM_YIELD_1_YIELD")
    assert node._yield_stages == []
def test_start_inside_inflation_is_allowed_when_escape_is_free(node):
    """시작점이 inflation(253) 안이어도 탈출 방향이 free면 피항을 시작한다.

    수확 자세는 이랑 옆이라 footprint 좌측이 항상 inscribed 밴드에 얹힌다.
    이것만으로 실패시키면(구 ERROR_MM_YIELD_START_BLOCKED) 피항 자체가 불가능하다.
    """
    x0, y0 = -0.47, -8.26
    assert node._footprint_max_cost(x0, y0, 0.0) == 253   # inflation 안
    assert node._footprint_lethal_free(x0, y0, 0.0)       # 그러나 occupied 아님

    _ready_for_yield(node)
    node._iw_yield_request_callback(Bool(data=True))

    assert [stage[0] for stage in node._yield_stages] == ["YIELD"]
    assert len(node._sent) == 1
    assert "START" not in _last_state(node)
    assert node._iw_yield_complete_pub.messages == []


def test_start_on_lethal_cell_still_fails(node):
    """inflation은 통과시키되 실제 occupied(254) 위에서는 여전히 실패한다."""
    _ready_for_yield(node)
    node._buffer.lookup_transform = lambda *a, **k: _FakeTransform(-1.45, -8.26)
    node._iw_yield_request_callback(Bool(data=True))

    assert node._sent == []
    assert node._iw_yield_complete_pub.messages == [False]
    assert _last_state(node) == "ERROR_MM_YIELD_START_OCCUPIED"


def test_blocked_final_waypoint_is_nudged_to_a_free_pose(node):
    """최종 waypoint가 lethal이면 진행축을 유지한 채 가까운 free pose로 보정."""
    corridor_y = float(node.get_parameter("iw_yield_goal_y").value)
    nominal_x = float(node.get_parameter("iw_yield_goal_x").value)
    node._costmap.block(nominal_x - 0.05, nominal_x + 0.55,
                        corridor_y - 0.35, corridor_y + 0.35)
    _ready_for_yield(node)
    node._iw_yield_request_callback(Bool(data=True))

    label, x4, y4, yaw4 = node._yield_stages[-1]
    assert label == "YIELD"
    assert x4 < nominal_x - 0.05                 # 장애물 반대편(서쪽)으로 옮겨졌다
    assert y4 == pytest.approx(corridor_y, abs=1e-6)   # 통로(Y)는 유지
    assert yaw4 == pytest.approx(                # 지정 자세 그대로 유지
        float(node.get_parameter("iw_yield_goal_yaw").value), abs=1e-3)
    assert node._footprint_lethal_free(x4, y4, yaw4)


def test_harvest_gate_stays_closed_while_yielding(node):
    """피항 중에는 어떤 경로로도 수확 게이트를 열지 않는다."""
    node._enable_pub = _Recorder()
    _ready_for_yield(node)
    node._iw_yield_request_callback(Bool(data=True))
    assert node._yield_active is True

    node._publish_enable(True)          # 어느 경로로 들어와도
    node._start_search()                # 탐색 시작 경로까지
    assert True not in node._enable_pub.messages

    # 피항이 끝나고 IW가 복귀하면 다시 열린다.
    node._iw_status_callback(_string("RETURNED"))
    assert node._yield_active is False
    node._publish_enable(True)
    assert node._enable_pub.messages[-1] is True


def test_manual_goal_during_yield_does_not_start_next_grasp(node):
    """피항 중 수동(RViz) 목표에 도착해도 홈→베드뷰→탐색으로 넘어가지 않는다."""
    _ready_for_yield(node)
    node._iw_yield_request_callback(Bool(data=True))

    manual = b"\x09" * 16
    node._known_goals.add(manual)
    node._active_goal = manual
    node._status_initialized = True
    _status(node, manual, GoalStatus.STATUS_SUCCEEDED)

    assert node._post_nav_settle_deadline_ns == 0    # 파지 준비 미예약
    assert node._waiting_home is False
    assert _last_state(node) == "MM_YIELDING_MANUAL_GOAL_REACHED"


def test_iw_is_released_once_mm_clears_the_lane(node):
    """마지막 단계를 기다리지 않고, 레인에서 충분히 비키면 그 시점에 허가한다."""
    _ready_for_yield(node)
    node._iw_yield_request_callback(Bool(data=True))
    assert node._iw_yield_complete_pub.messages == []

    # 아직 레인 위(x=0 부근) — 허가 없음.
    node._watchdog()
    assert node._iw_yield_complete_pub.messages == []

    # 서쪽으로 충분히 물러났다.
    node._buffer.lookup_transform = lambda *a, **k: _FakeTransform(-2.9, -3.57)
    clearance, _ = node._lane_clearance()
    required = float(node.get_parameter("iw_release_clearance_m").value)
    assert clearance >= required
    node._watchdog()

    assert node._iw_yield_complete_pub.messages == [True]
    assert node._yield_released is True
    assert node._yield_stages != []                  # 주행은 아직 진행 중
    assert _last_state(node) == "MM_YIELD_CLEAR_IW_RELEASED"

    # 남은 단계가 끝나도 허가를 중복 발행하지 않는다.
    node._yield_stage_succeeded()
    assert node._iw_yield_complete_pub.messages == [True]
