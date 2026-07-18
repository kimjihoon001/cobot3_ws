# -*- coding: utf-8 -*-
"""경로 표시 헬퍼.

머신이 두 대(dev / GPU 노트북)라 홈 경로가 다르다. 로그에 절대경로를 그대로
찍으면 같은 상황인데도 로그가 달라 보여서 대조가 안 된다. 홈 아래는 ~ 로 줄인다.

표시 전용이다. 경로 계산은 전부 __file__ 기준 상대경로로 하고 있다.
"""
from __future__ import annotations

import os


def short(path: str | os.PathLike) -> str:
    """홈 아래면 ~ 로 줄인다. 아니면 그대로."""
    p = os.path.abspath(str(path))
    home = os.path.expanduser("~")
    if p == home:
        return "~"
    if p.startswith(home + os.sep):
        return "~" + p[len(home):]
    return p
