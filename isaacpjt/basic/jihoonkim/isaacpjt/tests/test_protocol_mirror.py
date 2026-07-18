# -*- coding: utf-8 -*-
"""두 protocol.py 사본이 같은 계약을 지키는지 대조한다.

  isaacpjt/ros/protocol.py                              (Isaac, 3.11)
  src/m0609/jihoonkim/harvest_interfaces/harvest_interfaces/protocol.py  (ROS2, 3.10)

런타임이 갈라져 있어 import 로 합칠 수 없다(CLAUDE.md). 그래서 사본이고,
사본이 갈라지면 **GPU 에서 조용히 안 붙는다** — 에러도 안 나고 그냥 토픽이
안 맞거나 JSON 키가 안 맞을 뿐이라 찾기 최악이다. 여기서 미리 잡는다.

한쪽만 고치면 이 테스트가 터진다. 그게 목적이다.
"""
import importlib.util
import pathlib

import pytest

WS = pathlib.Path(__file__).resolve().parents[2]
ROS_SIDE = (WS / "src/m0609/jihoonkim/harvest_interfaces/harvest_interfaces/protocol.py")


def _load(path: pathlib.Path, name: str):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


@pytest.fixture(scope="module")
def ros_p():
    if not ROS_SIDE.exists():
        pytest.fail(f"ROS 쪽 protocol.py 가 없다: {ROS_SIDE}")
    return _load(ROS_SIDE, "ros_side_protocol")


@pytest.fixture(scope="module")
def isaac_p():
    from ros import protocol
    return protocol


TARGET = {"path": "/World/Plants/Row_00/Plant_00/Fruit_00",
          "class_name": "fully_ripe", "position": [1.0, 2.0, 1.1]}


def test_토픽_이름이_같다(ros_p, isaac_p):
    for ns in ("harvester_0", "transporter_1"):
        assert isaac_p.cmd_topic(ns) == ros_p.cmd_topic(ns)
        assert isaac_p.status_topic(ns) == ros_p.status_topic(ns)
        assert isaac_p.fruits_topic(ns) == ros_p.fruits_topic(ns)


def test_기본_네임스페이스가_같다(ros_p, isaac_p):
    assert isaac_p.DEFAULT_NS == ros_p.DEFAULT_NS


def test_액션_목록이_같다(ros_p, isaac_p):
    assert isaac_p.ACTIONS == ros_p.ACTIONS


def test_상태값이_같다(ros_p, isaac_p):
    assert (isaac_p.SUCCESS, isaac_p.FAILURE) == (ros_p.SUCCESS, ros_p.FAILURE)


def test_ROS가_인코딩한_명령을_Isaac이_읽는다(ros_p, isaac_p):
    """실제 방향 — FSM 이 보내고 브리지가 받는다."""
    for action in ros_p.ACTIONS:
        raw = ros_p.encode_cmd(1, action, TARGET)
        assert isaac_p.decode_cmd(raw) == {"id": 1, "action": action,
                                           "target": TARGET}


def test_Isaac이_인코딩한_상태를_ROS가_읽는다(ros_p, isaac_p):
    """실제 방향 — 브리지가 보내고 FSM 이 받는다."""
    for status in (isaac_p.SUCCESS, isaac_p.FAILURE):
        raw = isaac_p.encode_status(7, status)
        assert ros_p.decode_status(raw) == {"id": 7, "status": status}


def test_Isaac이_인코딩한_과실목록을_ROS가_읽는다(ros_p, isaac_p):
    raw = isaac_p.encode_fruits([TARGET])
    assert ros_p.decode_fruits(raw) == [TARGET]


def test_양쪽_바이트열이_동일하다(ros_p, isaac_p):
    """JSON 키 순서까지 같아야 디버깅 시 로그 대조가 쉽다."""
    assert isaac_p.encode_cmd(3, "cut", TARGET) == \
        ros_p.encode_cmd(3, "cut", TARGET)
    assert isaac_p.encode_status(3, "failure") == \
        ros_p.encode_status(3, "failure")
    assert isaac_p.encode_fruits([TARGET]) == ros_p.encode_fruits([TARGET])
