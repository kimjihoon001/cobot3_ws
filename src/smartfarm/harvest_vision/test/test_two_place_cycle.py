"""IW 데크에 2개를 실은 뒤에야 하역을 보내는 회귀 테스트.

IW 앞열 빈 KLT가 2칸이므로(robots/iwhub.py 사전 적재) 한 번 놓고 바로
지게차로 보내면 한 칸이 빈 채 나간다. 코디네이터는 place_target_count 개를
채울 때까지 수확을 반복하고, 그 중간 복귀는 HOME(joint_1=180°)을 거치지 않고
접힌 자세에서 joint_1만 베드 방위로 돌린다.
"""
import json

import numpy as np
import pytest
import rclpy
from std_msgs.msg import Bool, String

from harvest_vision.harvest_fsm_node import HarvestFsmNode
from harvest_vision.manipulator_target_node import ManipulatorTargetNode


class _Recorder:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg.data)


class _JsonRecorder:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(json.loads(msg.data))

    def keys(self):
        return [k for m in self.messages for k in m]


class _FakeTransform:
    def __init__(self, x, y):
        self.transform = type("T", (), {})()
        self.transform.translation = type(
            "V", (), {"x": x, "y": y, "z": 0.0})()
        self.transform.rotation = type(
            "Q", (), {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0})()


@pytest.fixture
def fsm():
    rclpy.init()
    node = HarvestFsmNode()
    # real_main 기본값은 1(수확 1회)이므로 다회 경로를 검증하려면 명시적으로 올린다.
    node.set_parameters([
        rclpy.Parameter(
            "place_target_count", rclpy.Parameter.Type.INTEGER, 2)])
    node._status_pub = _Recorder()
    node._iw_mission_pub = _Recorder()
    node._iw_yield_complete_pub = _Recorder()
    node._enable_pub = _Recorder()
    node._place_more_pending_pub = _Recorder()
    node._isaac_command_pub = _JsonRecorder()
    node._buffer.lookup_transform = lambda *a, **k: _FakeTransform(-0.47, -8.26)
    # __init__의 초기 발행은 recorder로 바꾸기 전에 끝났으므로 한 번 다시 낸다.
    node._publish_place_more_pending()
    yield node
    node.destroy_node()
    rclpy.shutdown()


@pytest.fixture
def manipulator():
    rclpy.init()
    node = ManipulatorTargetNode()
    node._isaac_command_pub = _JsonRecorder()
    node._state_pub = _Recorder()
    node._mobility_pub = _Recorder()
    yield node
    node.destroy_node()
    rclpy.shutdown()


def _report_place_done(fsm):
    """매니퓰레이터가 한 번 놓고 복귀를 마쳤다고 알린다."""
    fsm._placed = True
    fsm._cycle_failed = False
    fsm._manipulator_state_callback(String(data="HOME_READY"))


def _isaac_keys(node):
    return node._isaac_command_pub.keys()


def test_first_place_returns_to_bed_view_instead_of_forklift(fsm):
    _report_place_done(fsm)

    assert fsm._place_count == 1
    assert not fsm._iw_full, "1개만 놓고 만재로 판정하면 안 된다"
    assert "PREPARE_FORKLIFT" not in fsm._iw_mission_pub.messages
    # HOME을 거치지 않고 곧바로 BED_VIEW를 보낸다 = joint_1 한 축만 돈다.
    assert _isaac_keys(fsm) == ["moveit_bed_view"]
    assert fsm._waiting_bed_view is True


def test_second_place_triggers_forklift(fsm):
    _report_place_done(fsm)
    _report_place_done(fsm)

    assert fsm._place_count == 2
    assert fsm._iw_full is True
    assert "PREPARE_FORKLIFT" in fsm._iw_mission_pub.messages


def test_pending_flag_is_true_only_while_another_place_follows(fsm):
    # 첫 플레이스를 하기 전에는 "이번 다음에도 더 있다" = True.
    assert fsm._place_more_pending_pub.messages[-1] is True
    _report_place_done(fsm)
    # 두 번째가 마지막이므로 그 플레이스 뒤에는 남는 게 없다 = False.
    assert fsm._place_more_pending_pub.messages[-1] is False


def test_place_target_count_one_keeps_the_old_single_place_behavior(fsm):
    fsm.set_parameters([
        rclpy.parameter.Parameter(
            "place_target_count", rclpy.Parameter.Type.INTEGER, 1)])
    _report_place_done(fsm)

    assert fsm._iw_full is True
    assert "PREPARE_FORKLIFT" in fsm._iw_mission_pub.messages


def _report_final_failure(fsm):
    """매니퓰레이터가 모든 재시도를 소진하고 홈으로 돌아온 상황."""
    fsm._manipulator_state_callback(String(data="HARVEST_FAILED"))
    fsm._manipulator_state_callback(String(data="HOME_READY"))


def _forklift_calls(fsm):
    return [m for m in fsm._iw_mission_pub.messages if m == "PREPARE_FORKLIFT"]


def test_retry_exhausted_after_one_place_departs_with_partial_load(fsm):
    _report_place_done(fsm)
    _report_final_failure(fsm)

    assert fsm._iw_full is True
    assert _forklift_calls(fsm) == ["PREPARE_FORKLIFT"], "정확히 한 번만 발행한다"
    assert fsm._place_count == 1


def test_repeated_failures_do_not_resend_the_forklift_trigger(fsm):
    _report_place_done(fsm)
    _report_final_failure(fsm)
    _report_final_failure(fsm)

    assert _forklift_calls(fsm) == ["PREPARE_FORKLIFT"]


def test_failure_with_nothing_loaded_does_not_send_an_empty_iw(fsm):
    _report_final_failure(fsm)

    assert fsm._place_count == 0
    assert fsm._iw_full is False
    assert _forklift_calls(fsm) == [], "빈 IW를 하역장으로 보내지 않는다"


def test_search_timeout_after_one_place_departs_with_partial_load(fsm):
    _report_place_done(fsm)
    fsm._search_deadline_ns = 1            # 이미 만료된 시각
    fsm._watchdog()

    assert fsm._iw_full is True
    assert _forklift_calls(fsm) == ["PREPARE_FORKLIFT"]


def test_search_timeout_with_nothing_loaded_does_not_send_an_empty_iw(fsm):
    fsm._search_deadline_ns = 1
    fsm._watchdog()

    assert fsm._iw_full is False
    assert _forklift_calls(fsm) == []


def _finish_post_place_fold(node):
    node._basket_place = np.array([-1.209, 0.536, 0.288])
    node._send_post_place_fold()
    node._status_callback(String(data=json.dumps({
        "id": node._pending_id,
        "phase": "POST_PLACE_BED_VIEW",
        "reached": True,
    })))


def test_manipulator_skips_home_when_another_place_is_pending(manipulator):
    manipulator._place_more_pending_callback(Bool(data=True))
    _finish_post_place_fold(manipulator)

    assert "rmp_home" not in _isaac_keys(manipulator), (
        "남은 적재가 있으면 joint_1을 180°로 돌렸다가 되돌리는 왕복을 하지 않는다")
    assert manipulator._state == "HOME_READY"


def test_manipulator_returns_home_after_the_last_place(manipulator):
    manipulator._place_more_pending_callback(Bool(data=False))
    _finish_post_place_fold(manipulator)

    assert "rmp_home" in _isaac_keys(manipulator)
    assert manipulator._state == "GO_HOME"
