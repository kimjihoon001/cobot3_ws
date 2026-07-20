# -*- coding: utf-8 -*-
"""지게차 드라이버 (--fork) — 지게차 B (포크 승강). 창고에서 팔레트를 랙에 적재.

로봇 모델은 robots/transporter.py, ROS 브리지는 ros/robot_bridge.py. 배선만 한다(§5.6).
부가장치 없음 — 조인트 브리지만. 제어는 ROS2 가 /{ns}/joint_command 로 직접 한다.
"""
from __future__ import annotations

from robot_base import Driver, ros_fail
from robots.transporter import TransporterAMR

# 창고 안(입구 지나 개활부) — 랙 적재 담당 (2026-07-20)
POSE = (0.0, 15.5, 0.0)


class ForkDriver(Driver):
    flag = "--fork"
    name = "fk"
    ns = "forklift_0"
    root = "/World/Forklift"

    def __init__(self, cfg):
        super().__init__()
        self._fk = TransporterAMR(cfg.robots, cfg.warehouse)

    def spawn(self, stage):
        self._fk.spawn(stage, self.root, POSE)

    def finalize(self, world, stage, opts):
        if not opts.no_ros:
            try:
                from ros import robot_bridge as RB
                RB.build_joint_bridge(stage, f"/World/RosBridge_{self.ns}",
                                      self.ns, self.art)
            except Exception:
                ros_fail("지게차 조인트 브리지")
