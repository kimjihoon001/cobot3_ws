# -*- coding: utf-8 -*-
"""브리지 알맹이 검증 — GPU 없이 돈다.

harvest_bridge.py / graph.py 는 OmniGraph 라 여기서 못 돌린다. 대신 로직을
dispatcher.py / stub_harvester.py 로 떼어놨고 그쪽을 검증한다. 거기 남는 건
배선뿐이라 GPU 첫 실행에서 볼 게 "연결이 되나" 하나로 줄어든다.
"""
import json

import pytest

from robots.stub_harvester import RUNNING, StubHarvester
from ros import protocol as P
from ros.dispatcher import CommandDispatcher

TARGET = {"path": "/World/Plants/Row_00/Plant_00/Fruit_00",
          "class_name": "fully_ripe", "position": [0.0, 0.0, 1.0]}


class FakeTask:
    """GreenhouseTask.detach_fruit 만 흉내낸다."""

    def __init__(self, valid=("/World/Plants/Row_00/Plant_00/Fruit_00",)):
        self.valid = set(valid)
        self.detached: list[str] = []

    def detach_fruit(self, path: str) -> bool:
        if path not in self.valid or path in self.detached:
            return False          # 없는 경로거나 이미 딴 것
        self.detached.append(path)
        return True


def run_until_done(disp, limit=200):
    for _ in range(limit):
        st = disp.tick()
        if st is not None:
            return P.decode_status(st)
    pytest.fail("명령이 안 끝남")


# ---------------------------------------------------------------
# 스텁 로봇
# ---------------------------------------------------------------
def test_동작은_정해진_프레임_동안_running이다():
    robot = StubHarvester(FakeTask().detach_fruit, frames_per_action=3)
    assert robot.approach(TARGET) == RUNNING
    assert robot.approach(TARGET) == RUNNING
    assert robot.approach(TARGET) == P.SUCCESS


def test_cut은_실제로_과실을_분리한다():
    """스텁에서 유일하게 진짜인 부분 — 씬 물리 경로."""
    task = FakeTask()
    robot = StubHarvester(task.detach_fruit, frames_per_action=1)
    assert robot.cut(TARGET) == P.SUCCESS
    assert task.detached == [TARGET["path"]]


def test_이미_딴_과실을_또_자르면_실패다():
    task = FakeTask()
    robot = StubHarvester(task.detach_fruit, frames_per_action=1)
    robot.cut(TARGET)
    assert robot.cut(TARGET) == P.FAILURE


def test_없는_경로를_자르면_실패다():
    robot = StubHarvester(FakeTask().detach_fruit, frames_per_action=1)
    assert robot.cut({"path": "/없음"}) == P.FAILURE


def test_grasp하면_물고_place하면_놓는다():
    robot = StubHarvester(FakeTask().detach_fruit, frames_per_action=1)
    assert robot.holding is None
    robot.grasp(TARGET)
    assert robot.holding == TARGET["path"]
    robot.place()
    assert robot.holding is None


def test_release하면_놓는다():
    robot = StubHarvester(FakeTask().detach_fruit, frames_per_action=1)
    robot.grasp(TARGET)
    robot.release()
    assert robot.holding is None


# ---------------------------------------------------------------
# 디스패처
# ---------------------------------------------------------------
def test_명령을_받아_끝나면_status를_한_번_낸다():
    disp = CommandDispatcher(StubHarvester(FakeTask().detach_fruit,
                                           frames_per_action=3))
    disp.submit(P.encode_cmd(1, "approach", TARGET))

    assert disp.tick() is None            # RUNNING
    assert disp.tick() is None
    assert P.decode_status(disp.tick()) == {"id": 1, "status": "success"}
    assert disp.tick() is None, "끝난 명령을 또 보고하면 안 된다"


def test_명령_id가_그대로_돌아온다():
    """id 가 어긋나면 FSM 이 응답을 영원히 못 받는다."""
    disp = CommandDispatcher(StubHarvester(FakeTask().detach_fruit,
                                           frames_per_action=1))
    disp.submit(P.encode_cmd(42, "grasp", TARGET))
    assert run_until_done(disp)["id"] == 42


def test_명령이_없으면_tick은_아무것도_안_한다():
    disp = CommandDispatcher(StubHarvester(FakeTask().detach_fruit))
    assert disp.tick() is None


def test_깨진_명령은_무시한다():
    """남의 메시지나 손상된 JSON 때문에 브리지가 죽으면 안 된다."""
    disp = CommandDispatcher(StubHarvester(FakeTask().detach_fruit), log=lambda *_: None)
    for bad in ('{"id": 1, "action": "fly"}', "not json", "{}", "[]"):
        disp.submit(bad)
        assert disp.tick() is None


def test_같은_명령_재수신은_무시한다():
    """토픽이 재발행돼도 동작을 두 번 시작하면 안 된다."""
    disp = CommandDispatcher(StubHarvester(FakeTask().detach_fruit,
                                           frames_per_action=3))
    disp.submit(P.encode_cmd(1, "approach", TARGET))
    disp.tick()
    disp.submit(P.encode_cmd(1, "approach", TARGET))   # 같은 id
    disp.tick()
    assert P.decode_status(disp.tick())["id"] == 1     # 3틱만에 완료 = 재시작 안 함


def test_release는_응답없이_즉시_처리된다():
    robot = StubHarvester(FakeTask().detach_fruit, frames_per_action=5)
    disp = CommandDispatcher(robot, log=lambda *_: None)
    robot.grasp(TARGET)
    robot.grasp(TARGET)
    robot.grasp(TARGET)
    robot.grasp(TARGET)
    robot.grasp(TARGET)
    assert robot.holding is not None

    disp.submit(P.encode_cmd(9, "release", None))
    assert robot.holding is None
    assert disp.tick() is None, "release 는 응답을 내면 안 된다"


def test_target없는_approach는_실패로_응답한다():
    """FSM 버그로 target 이 빠져도 브리지가 멈추면 안 된다."""
    disp = CommandDispatcher(StubHarvester(FakeTask().detach_fruit),
                             log=lambda *_: None)
    disp.submit(json.dumps({"id": 5, "action": "approach", "target": None}))
    assert P.decode_status(disp.tick()) == {"id": 5, "status": "failure"}


def test_place는_target이_없어도_된다():
    disp = CommandDispatcher(StubHarvester(FakeTask().detach_fruit,
                                           frames_per_action=1))
    disp.submit(P.encode_cmd(6, "place", None))
    assert P.decode_status(disp.tick())["status"] == "success"


def test_reset하면_진행중_명령이_사라진다():
    disp = CommandDispatcher(StubHarvester(FakeTask().detach_fruit,
                                           frames_per_action=5))
    disp.submit(P.encode_cmd(1, "approach", TARGET))
    disp.tick()
    disp.reset()
    assert disp.tick() is None


# ---------------------------------------------------------------
# 전체 흐름 — FSM 이 보낼 순서 그대로
# ---------------------------------------------------------------
def test_한_과실_수확_명령열이_전부_처리된다():
    task = FakeTask()
    disp = CommandDispatcher(StubHarvester(task.detach_fruit,
                                           frames_per_action=2))
    for i, action in enumerate(["approach", "grasp", "cut", "place"]):
        disp.submit(P.encode_cmd(i, action, TARGET))
        assert run_until_done(disp) == {"id": i, "status": "success"}

    assert task.detached == [TARGET["path"]], "cut 에서 과실이 분리됐어야 한다"
