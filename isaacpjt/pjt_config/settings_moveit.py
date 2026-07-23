# -*- coding: utf-8 -*-
"""moveit_mm(스쿱/UR10e) 전용 설정 오버라이드 (2026-07-24 격리).

main 공유 ``settings.py`` 는 RG2/Doosan 로봇 값이다. 스쿱 수확 로봇은 UR10e 팔 +
동축 1/4구 스쿱이라 팔 에셋·end-effector 값이 다르다. 공유 settings 를 건드리지 않고
여기서 ``dataclasses.replace`` 로 스쿱 값만 얹은 ``RobotConfig`` 사본을 만들어
``HarvestMM``(스쿱, robots/harvester_moveit.py)에 넘긴다.

- ``scoop_gripper_usd`` 는 RobotAssetConfig 기본값이 이미 스쿱 USD 라 오버라이드 불필요.
- 씬 레벨 값(fruits_per_plant, 과실 마찰)은 main 을 그대로 쓰고, moveit_mm 이 파지
  타겟만 런타임에 ``set_kinematic`` 으로 다룬다.
"""
from __future__ import annotations

import dataclasses

from pjt_config.settings import RobotConfig

# 스쿱 로봇은 UR10e. main assets.arm 은 Doosan m0617 이므로 여기서 되돌린다
# (머지 시 arm 은 conflict 없이 main=Doosan 으로 덮이므로 반드시 오버라이드).
_SCOOP_ARM = (
    "/Isaac/Robots/UniversalRobots/ur10e/ur10e.usd",
    "/Isaac/Robots/UR10e/ur10e.usd",
    "/Isaac/Robots/UniversalRobots/ur10/ur10.usd",
)


def scoop_robot_config(base: RobotConfig) -> RobotConfig:
    """main 공유 RobotConfig 에 스쿱/UR10e 값을 얹은 사본을 돌려준다."""
    return dataclasses.replace(
        base,
        assets=dataclasses.replace(base.assets, arm=_SCOOP_ARM),
        end_effector=dataclasses.replace(
            base.end_effector,
            camera_offset=(0.0, 0.095, -0.035),
            grasp_reach_z=0.120,
            pad_static_friction=1.5,
            pad_dynamic_friction=1.2,
        ),
    )
