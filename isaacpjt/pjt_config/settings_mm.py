# -*- coding: utf-8 -*-
"""mm(스쿱/m0617) 전용 설정 오버라이드 (2026-07-24).

settings_moveit.scoop_robot_config 와 같은 목적이지만 **팔을 되돌리지 않는다** —
mm 는 공유 settings 의 기본 팔(Doosan m0617)을 그대로 쓰고, 동축 스쿱에 맞는
end-effector 값만 얹는다. 그래서 여기 남는 차이는 "팔 오버라이드 없음" 하나뿐이다.
"""
from __future__ import annotations

import dataclasses

from pjt_config.settings import RobotConfig


def mm_robot_config(base: RobotConfig) -> RobotConfig:
    """main 공유 RobotConfig 에 스쿱 end-effector 값을 얹은 사본을 돌려준다."""
    return dataclasses.replace(
        base,
        end_effector=dataclasses.replace(
            base.end_effector,
            camera_offset=(0.0, 0.095, -0.035),
            grasp_reach_z=0.120,
            pad_static_friction=1.5,
            pad_dynamic_friction=1.2,
        ),
    )
