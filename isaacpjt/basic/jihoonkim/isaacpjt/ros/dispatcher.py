# -*- coding: utf-8 -*-
"""명령 디스패처 — 받은 JSON 명령을 로봇 동작으로 바꾸고 결과 JSON 을 돌려준다.

브리지에서 OmniGraph(=GPU 에서만 도는 부분)를 걷어낸 알맹이다. 여기는
Isaac 을 import 하지 않으므로 GPU 없이 tests/test_bridge.py 로 검증된다.

흐름 (FSM 쪽 robot_client.py 와 짝):
  FSM 이 명령을 **한 번** 보내고 응답을 기다린다. 그러므로 여기는 명령 하나를
  받아 물고 있다가, 매 tick 로봇을 폴링해서 끝나면 status 를 딱 한 번 낸다.
"""
from __future__ import annotations

from typing import Any, Protocol

from ros import protocol as P

RUNNING = "running"


class Harvester(Protocol):
    """디스패처가 로봇에게 요구하는 것. StubHarvester 가 이걸 만족한다."""

    def approach(self, target: dict) -> str: ...
    def grasp(self, target: dict) -> str: ...
    def cut(self, target: dict) -> str: ...
    def place(self) -> str: ...
    def release(self) -> None: ...


class CommandDispatcher:
    def __init__(self, robot: Harvester, log=print):
        self._robot = robot
        self._log = log
        self._pending: dict[str, Any] | None = None

    def submit(self, raw: str) -> None:
        """cmd 토픽에서 받은 원문. 파싱 실패는 무시한다 (남의 메시지일 수 있다)."""
        try:
            cmd = P.decode_cmd(raw)
        except (ValueError, TypeError) as e:
            self._log(f"[Bridge] 명령 무시: {e}")
            return

        if cmd["action"] == "release":
            # 응답 없는 fire-and-forget. 진행 중이던 명령도 같이 버린다
            # (FSM 은 포기 경로에서만 부르고 바로 다음 과실로 넘어간다).
            self._robot.release()
            self._pending = None
            return

        if self._pending is not None:
            if self._pending["id"] == cmd["id"]:
                return                      # 같은 명령 재수신 — 무시
            self._log(f"[Bridge] 진행 중 명령 폐기: id={self._pending['id']}")
        self._pending = cmd

    def tick(self) -> str | None:
        """로봇을 한 번 폴링. 명령이 끝났으면 status JSON, 아니면 None."""
        if self._pending is None:
            return None

        cmd = self._pending
        action, target = cmd["action"], cmd["target"]

        if action == "place":
            st = self._robot.place()
        elif target is None:
            self._log(f"[Bridge] {action} 에 target 이 없음 — 실패 처리")
            st = P.FAILURE
        elif action == "approach":
            st = self._robot.approach(target)
        elif action == "grasp":
            st = self._robot.grasp(target)
        elif action == "cut":
            st = self._robot.cut(target)
        else:
            self._log(f"[Bridge] 처리 못 하는 액션: {action}")
            st = P.FAILURE

        if st == RUNNING:
            return None

        self._pending = None
        return P.encode_status(cmd["id"], st)

    def reset(self) -> None:
        """Play/Stop 시 진행 중이던 명령을 버린다."""
        self._pending = None
