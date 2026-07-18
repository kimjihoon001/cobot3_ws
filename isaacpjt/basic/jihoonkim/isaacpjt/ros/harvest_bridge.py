# -*- coding: utf-8 -*-
"""수확 브리지 — OmniGraph 토픽과 디스패처를 잇고 매 프레임 돌린다.

  dev 머신 fsm_node  --/{ns}/harvest/cmd-->    SubCmd    -> dispatcher -> robot
                     <--/{ns}/harvest/status-- PubStatus <- dispatcher
                     <--/{ns}/harvest/fruits-- PubFruits <- task.pickable_fruits()

Isaac 안에서만 돈다. 알맹이(dispatcher/stub_harvester/protocol)는 순수 파이썬으로
떼어놔서 dev 머신에서 테스트되고, 여기는 배선만 한다.
"""
from __future__ import annotations

import omni.graph.core as og

from ros import graph as graph_builder
from ros import protocol as P
from ros.dispatcher import CommandDispatcher


class HarvestBridge:
    def __init__(self, task, robot, ns: str = P.DEFAULT_NS,
                 domain_id: int = 108, fruits_every: int = 30, log=print):
        """task  : GreenhouseTask (pickable_fruits / detach_fruit 제공)
        robot    : StubHarvester 등 Harvester 프로토콜 구현체
        fruits_every : 과실 목록 갱신 주기(프레임). 400개 JSON 을 60Hz 로
                       내보내면 낭비라 갱신만 늦춘다.
                       (주의: 그래프는 매 tick 발행한다. 발행 자체를 줄이려면
                        Branch 노드로 게이트해야 하는데, 노드 하나 더 늘어나면
                        GPU 첫 실행에서 실패 지점이 늘어서 일단 이렇게 둔다.)
        """
        self._task = task
        self._robot = robot
        self._log = log
        self._fruits_every = fruits_every
        self._frame = 0
        self._last_cmd = ""
        self._dispatcher = CommandDispatcher(robot, log=log)

        nodes = graph_builder.build(P.cmd_topic(ns), P.status_topic(ns),
                                    P.fruits_topic(ns), domain_id, log)
        self._cmd_out = self._attr(nodes["SubCmd"], "outputs:data")
        self._status_in = self._attr(nodes["PubStatus"], "inputs:data")
        self._fruits_in = self._attr(nodes["PubFruits"], "inputs:data")
        self._publish_fruits()

    def _attr(self, node_path: str, name: str):
        """제네릭 pub/sub 의 data 속성은 메시지 타입이 풀린 뒤에 생긴다.
        그래프 생성 직후엔 아직 없을 수 있어 실패 메시지를 분명히 남긴다."""
        try:
            return og.Controller.attribute(name, node_path)
        except Exception as e:
            raise RuntimeError(
                f"{node_path} 의 {name} 속성을 못 찾음: {e}\n"
                f"  -> 제네릭 ROS2Pub/Sub 은 messageName 이 풀려야 data 가 생긴다.\n"
                f"  -> 한 프레임 돌린 뒤 다시 잡아야 할 수도 있다.") from e

    def tick(self) -> None:
        """world.step() 마다 부른다."""
        self._frame += 1

        raw = og.Controller.get(self._cmd_out) or ""
        if raw and raw != self._last_cmd:
            # 같은 문자열이 계속 붙어 있으므로 바뀔 때만 처리한다.
            # (명령마다 id 가 달라서 내용이 반드시 바뀐다)
            self._last_cmd = raw
            self._dispatcher.submit(raw)

        status = self._dispatcher.tick()
        if status is not None:
            # 이 값은 다음 status 가 나올 때까지 매 tick 재발행된다.
            # FSM 쪽은 pending id 와 다른 status 를 무시하므로 무해하다
            # (id 가 단조증가라 옛 status 가 새 명령에 매칭될 수 없다).
            og.Controller.set(self._status_in, status)

        if self._frame % self._fruits_every == 0:
            self._publish_fruits()

    def reset(self) -> None:
        """Play/Stop 시. 진행 중이던 명령과 로봇 상태를 버린다."""
        self._dispatcher.reset()
        if hasattr(self._robot, "reset"):
            self._robot.reset()
        self._last_cmd = ""
        self._frame = 0
        self._publish_fruits()

    def _publish_fruits(self) -> None:
        og.Controller.set(self._fruits_in,
                          P.encode_fruits(self._task.pickable_fruits()))
