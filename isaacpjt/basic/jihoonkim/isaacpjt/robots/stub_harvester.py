# -*- coding: utf-8 -*-
"""스텁 수확 로봇 — 실제 M0609+베이스가 붙기 전까지 브리지를 완주시키는 대역.

아직 아무것도 안 움직인다. 각 동작이 정해진 프레임 수만큼 RUNNING 을 반환한 뒤
SUCCESS 가 된다. 단 하나 **cut 만은 진짜다** — detach_fruit 를 불러 과실의
kinematic 을 꺼서 실제로 떨어뜨린다. 그게 씬 물리 경로를 GPU 에서 증명하는 부분이다.

그래서 지금 GPU 에서 돌리면 과실이 그냥 바닥에 떨어진다. 그리퍼가 없으니 당연하고,
그게 곧 "잡고 나서 잘라야 한다"는 걸 눈으로 보여주는 증거이기도 하다.

Isaac 을 import 하지 않는다 (detach_fruit 를 콜러블로 주입받는다). 그래서
GPU 없이 tests/test_bridge.py 로 검증된다.
"""
from __future__ import annotations

from typing import Callable

RUNNING = "running"
SUCCESS = "success"
FAILURE = "failure"


class StubHarvester:
    """동작마다 frames_per_action 프레임을 소모한 뒤 결과를 낸다.

    detach_fruit : (prim_path) -> bool. 보통 GreenhouseTask.detach_fruit.
                   cut 성공 판정을 여기에 맡긴다 — 이미 딴 과실이거나 경로가
                   틀리면 False 를 돌려주므로 그대로 FAILURE 가 된다.
    """

    def __init__(self, detach_fruit: Callable[[str], bool],
                 frames_per_action: int = 30):
        self._detach = detach_fruit
        self._frames = frames_per_action
        self._elapsed = 0
        self._holding: str | None = None      # 물고 있는 과실 경로

    @property
    def holding(self) -> str | None:
        return self._holding

    # ----- 동작 (tick 마다 호출된다) -----

    def approach(self, target: dict) -> str:
        return self._timed()

    def grasp(self, target: dict) -> str:
        st = self._timed()
        if st == SUCCESS:
            self._holding = target["path"]
        return st

    def cut(self, target: dict) -> str:
        st = self._timed()
        if st != SUCCESS:
            return st
        # 여기가 유일한 진짜 물리 — 꽃자루가 끊어지는 순간
        return SUCCESS if self._detach(target["path"]) else FAILURE

    def place(self) -> str:
        st = self._timed()
        if st == SUCCESS:
            self._holding = None
        return st

    def release(self) -> None:
        self._holding = None

    def reset(self) -> None:
        """Play/Stop 시 진행 중이던 동작을 버린다."""
        self._elapsed = 0
        self._holding = None

    # ----- 내부 -----

    def _timed(self) -> str:
        self._elapsed += 1
        if self._elapsed < self._frames:
            return RUNNING
        self._elapsed = 0
        return SUCCESS
