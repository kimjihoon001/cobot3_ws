# -*- coding: utf-8 -*-
"""Isaac <-> ROS2 통신 프로토콜 — **src/m0609/jihoonkim/harvest_fsm/harvest_fsm/
protocol.py 의 사본**.

왜 사본인가:
  두 파일은 런타임이 다르다 (Isaac = python.sh 3.11 / ROS2 = system 3.10).
  CLAUDE.md 가 두 트리의 코드 공유를 금지하므로 import 로 합칠 수 없다.
  대신 tests/test_protocol_mirror.py 가 두 파일이 실제로 같은 계약을 지키는지
  대조한다. 한쪽만 고치면 그 테스트가 터진다.

여기를 고치면 반드시 저쪽도 같이 고칠 것.
"""
from __future__ import annotations

import json
from typing import Any

# 로봇이 여러 대다 (수확 모바일 매니퓰레이터 + 운반 AMR + 지게차 후보).
# 토픽은 전부 로봇 네임스페이스 아래에 둔다.
DEFAULT_NS = "harvester_0"


def cmd_topic(ns: str = DEFAULT_NS) -> str:       # FSM -> Isaac
    return f"/{ns}/harvest/cmd"


def status_topic(ns: str = DEFAULT_NS) -> str:    # Isaac -> FSM
    return f"/{ns}/harvest/status"


def fruits_topic(ns: str = DEFAULT_NS) -> str:    # Isaac -> FSM (주기 발행)
    return f"/{ns}/harvest/fruits"


ACTIONS = ("approach", "grasp", "cut", "release", "place")
SUCCESS = "success"
FAILURE = "failure"
STATUSES = (SUCCESS, FAILURE)

Target = dict[str, Any]


def encode_cmd(cmd_id: int, action: str, target: Target | None) -> str:
    if action not in ACTIONS:
        raise ValueError(f"없는 액션: {action}")
    return json.dumps({"id": cmd_id, "action": action, "target": target})


def _as_dict(raw: str) -> dict:
    """JSON 이 객체인지까지 확인한다.

    '[]' 같은 게 오면 d.get 이 AttributeError 를 내는데, 호출부는 ValueError 만
    잡으므로 그대로 터진다. 토픽에는 뭐가 올지 모르니 여기서 ValueError 로 좁힌다.
    """
    d = json.loads(raw)
    if not isinstance(d, dict):
        raise ValueError(f"JSON 객체가 아님: {type(d).__name__}")
    return d


def decode_cmd(raw: str) -> dict:
    d = _as_dict(raw)
    if d.get("action") not in ACTIONS:
        raise ValueError(f"없는 액션: {d.get('action')}")
    if not isinstance(d.get("id"), int):
        raise ValueError("id 가 없거나 정수가 아님")
    return d


def encode_status(cmd_id: int, status: str) -> str:
    if status not in STATUSES:
        raise ValueError(f"없는 상태: {status}")
    return json.dumps({"id": cmd_id, "status": status})


def decode_status(raw: str) -> dict:
    d = _as_dict(raw)
    if d.get("status") not in STATUSES:
        raise ValueError(f"없는 상태: {d.get('status')}")
    if not isinstance(d.get("id"), int):
        raise ValueError("id 가 없거나 정수가 아님")
    return d


def encode_fruits(fruits: list[Target]) -> str:
    return json.dumps({"fruits": fruits})


def decode_fruits(raw: str) -> list[Target]:
    d = _as_dict(raw)
    if not isinstance(d.get("fruits"), list):
        raise ValueError("fruits 목록이 없음")
    return d["fruits"]
