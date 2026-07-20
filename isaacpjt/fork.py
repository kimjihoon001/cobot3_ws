# -*- coding: utf-8 -*-
"""지게차 드라이버 (--fork) — 지게차 B (포크 승강). 창고에서 팔레트를 랙에 적재.

로봇 모델은 robots/transporter.py, ROS 브리지는 ros/robot_bridge.py. 배선만 한다(§5.6).
부가장치 없음 — 조인트 브리지만. 제어는 ROS2 가 /{ns}/joint_command 로 직접 한다.
"""
from __future__ import annotations

import numpy as np

from robot_base import Driver, ros_fail
from robots.control import TransporterController
from robots.transporter import TransporterAMR

# 창고 안(입구 지나 개활부) — 랙 적재 담당 (2026-07-20)
POSE = (0.0, 15.5, 0.0)
YAW_DEG = 90.0       # Forklift 로컬 +X → 팔레트 삽입 방향 월드 +Y


class ForkDriver(Driver):
    flag = "--fork"
    name = "fk"
    ns = "forklift_0"
    root = "/World/Forklift"

    def __init__(self, cfg):
        super().__init__()
        self._fk = TransporterAMR(cfg.robots, cfg.warehouse)
        self._controller = None
        self._poller = None
        self._last_motion = None

    def spawn(self, stage):
        self._fk.spawn(stage, self.root, POSE, yaw_deg=YAW_DEG)

    def configure(self, world):
        self._controller = TransporterController(self.robot)

    def finalize(self, world, stage, opts):
        if not opts.no_ros:
            try:
                from ros import robot_bridge as RB
                RB.build_joint_bridge(stage, f"/World/RosBridge_{self.ns}",
                                      self.ns, self.art, apply_commands=False)
                self._poller = RB.JointCommandPoller(
                    f"/World/RosBridge_{self.ns}/Sub")
            except Exception:
                ros_fail("지게차 조인트 브리지")

    def update(self, is_playing: bool):
        if not is_playing or self._controller is None or self._poller is None:
            return
        cmd = self._poller.poll()
        if cmd:
            names, positions, velocities = cmd
            for name, value in zip(names, positions):
                if not np.isfinite(value):
                    continue
                if name == "lift_joint":
                    self._controller.set_fork(float(value))
                elif name == "back_wheel_swivel":
                    self._controller.set_steer(float(value))
            for name, value in zip(names, velocities):
                if name == "back_wheel_drive" and np.isfinite(value):
                    self._controller.set_drive(float(value))
            motion = (round(self._controller._drive_vel, 2),
                      round(float(self._controller._steer), 3))
            if motion != self._last_motion:
                print(f"[Forklift RX] drive={motion[0]:.2f}rad/s "
                      f"steer={np.degrees(motion[1]):.1f}deg")
                self._last_motion = motion
        # ForkliftB 에셋은 바퀴만 헛도는 경우가 있어 기존 제어기의 평면 운동을 병행한다.
        self._controller.apply(dt=1.0 / 60.0)
