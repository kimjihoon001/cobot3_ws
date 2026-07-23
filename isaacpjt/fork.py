# -*- coding: utf-8 -*-
"""지게차 드라이버 (--fork) — 지게차 B (포크 승강). 창고에서 팔레트를 랙에 적재.

로봇 모델은 robots/transporter.py, ROS 브리지는 ros/robot_bridge.py. 배선만 한다(§5.6).
부가장치 없음 — 조인트 브리지만. 제어는 ROS2 가 /{ns}/joint_command 로 직접 한다.
"""
from __future__ import annotations

import math

import numpy as np
from pxr import UsdPhysics

from robot_base import Driver, ros_fail
from robots.control import TransporterController
from robots.transporter import TransporterAMR
from scene.ground import COMMON_FLOOR_Z

# 창고 입구 중앙축의 내부 대기점. 창고 입구 경계는 Y=13이고 대기점은 그 안쪽이다.
POSE = (0.0, 14.5, COMMON_FLOOR_Z)
# ForkliftB 포크는 모델 로컬 -X 방향이다. +90°로 놓으면 포크가 월드 -Y,
# 즉 창고 밖 입구 중앙의 AMR을 향한다.
YAW_DEG = 90.0
PALLET_PATH_FORMAT = "/World/Warehouse/Pallet_{:02d}"
PALLET_CARRY_JOINT = "/World/ForkliftPalletCarryJoint"


class ForkDriver(Driver):
    flag = "--fork"
    name = "fk"
    ns = "forklift_0"
    root = "/World/Forklift"

    def __init__(self, cfg, iw_driver=None):
        super().__init__()
        self._fk = TransporterAMR(cfg.robots, cfg.warehouse)
        self._iw_driver = iw_driver
        self._controller = None
        self._stage = None
        self._poller = None
        self._pose_publisher = None
        self._pose_ready_logged = False
        self._last_motion = None
        self._pallet_attached = False
        self._deck_pallet_attached = False
        self._iw_dock_locked = False
        self._pallet_id = 0
        self._physics_dt = 1.0 / 60.0

    def spawn(self, stage):
        self._fk.spawn(stage, self.root, POSE, yaw_deg=YAW_DEG)

    def configure(self, world):
        self._controller = TransporterController(self.robot)
        # GUI 렌더 FPS가 아니라 Isaac physics timestep으로 평면 운동을 적분한다.
        self._physics_dt = float(world.get_physics_dt())

    def finalize(self, world, stage, opts):
        self._stage = stage
        if not opts.no_ros:
            try:
                from ros import robot_bridge as RB
                RB.build_joint_bridge(stage, f"/World/RosBridge_{self.ns}",
                                      self.ns, self.art, apply_commands=False)
                self._poller = RB.JointCommandPoller(
                    f"/World/RosBridge_{self.ns}/Sub")
                pose_node = RB.build_pose_publisher(
                    f"/World/RosPose_{self.ns}", f"/{self.ns}/pose"
                )
                self._pose_publisher = RB.PosePublisher(pose_node)
            except Exception:
                ros_fail("지게차 조인트 브리지")

    def update(self, is_playing: bool):
        if not is_playing or self._controller is None or self._poller is None:
            return
        cmd = self._poller.poll()
        if cmd:
            names, positions, velocities = cmd
            pallet_attach_request = None
            deck_attach_request = None
            dock_lock_request = None
            pallet_id_request = self._pallet_id
            for name, value in zip(names, positions):
                if not np.isfinite(value):
                    continue
                if name == "lift_joint":
                    self._controller.set_fork(float(value))
                elif name == "back_wheel_swivel":
                    # temp/spikes/06_pallet_lift.py와 같은 조향 적용 방식.
                    self._controller.set_steer(float(value))
                elif name == "pallet_attach":
                    pallet_attach_request = float(value) >= 0.5
                elif name == "pallet_deck_attach":
                    deck_attach_request = float(value) >= 0.5
                elif name == "iw_dock_lock":
                    dock_lock_request = float(value) >= 0.5
                elif name == "pallet_id":
                    pallet_id_request = max(0, min(5, int(round(float(value)))))
            for name, value in zip(names, velocities):
                if name == "back_wheel_drive" and np.isfinite(value):
                    # 스파이크에서 확인된 ForkliftB 구동 부호: 음수가 전진이다.
                    self._controller.set_drive(-float(value))
            if dock_lock_request is not None:
                self._set_iw_dock_locked(dock_lock_request)
            if (
                pallet_attach_request is not None
                or deck_attach_request is not None
            ):
                self._set_pallet_owner(
                    fork_requested=(
                        self._pallet_attached
                        if pallet_attach_request is None
                        else pallet_attach_request
                    ),
                    deck_requested=(
                        self._deck_pallet_attached
                        if deck_attach_request is None
                        else deck_attach_request
                    ),
                    pallet_id=pallet_id_request,
                )
            motion = (round(self._controller._drive_vel, 2),
                      round(float(self._controller._steer), 3))
            if motion != self._last_motion:
                print(f"[Forklift RX] drive={motion[0]:.2f}rad/s "
                      f"steer={np.degrees(motion[1]):.1f}deg")
                self._last_motion = motion
        # 수신한 포크·조향·구동 목표를 Isaac 아티큘레이션에 매 프레임 반영한다.
        # ForkliftB는 후륜이 회전해도 바닥에서 헛돌 수 있으므로, 검증 스파이크와
        # 같은 physics timestep 기반 평면 차량 운동으로 GUI 차체를 이동한다.
        # 이 에셋은 루트 +X와 포크/논리 전방이 반대다. 구동 부호를 위에서
        # 뒤집은 것과 동일하게 yaw 부호도 뒤집어야 ROS가 계산한 U턴 방향과
        # GUI 차체의 회전 방향이 일치한다.
        self._controller.apply(
            dt=self._physics_dt,
            kinematic_yaw_sign=-1.0,
        )
        if self._pose_publisher is not None:
            position, quat_wxyz = self.robot.get_world_pose()
            # 에셋의 포크/주행 전방은 루트 로컬 -X이므로 루트 yaw에 180°를
            # 더한 값을 ROS 경로 제어의 heading으로 보낸다.
            w, x, y, z = (float(v) for v in quat_wxyz)
            root_yaw = math.atan2(
                2.0 * (w * z + x * y),
                1.0 - 2.0 * (y * y + z * z),
            )
            heading = math.atan2(
                math.sin(root_yaw + math.pi), math.cos(root_yaw + math.pi)
            )
            quat_xyzw = (
                0.0, 0.0, math.sin(heading / 2.0), math.cos(heading / 2.0)
            )
            if self._pose_publisher.publish(position, quat_xyzw):
                if not self._pose_ready_logged:
                    print(f"[Forklift Pose] /{self.ns}/pose 실제 자세 발행 시작")
                    self._pose_ready_logged = True

    def _fork_carriage_body(self) -> str | None:
        """lift_joint에서 포크와 함께 승강하는 강체 경로를 찾는다."""
        if self._stage is None or not self._fk.lift_joint:
            return None
        prim = self._stage.GetPrimAtPath(self._fk.lift_joint)
        if not prim.IsValid():
            return None
        joint = UsdPhysics.PrismaticJoint(prim)
        if not joint:
            return None
        # 스파이크 06 실측과 동일하게 움직이는 Body1을 우선한다.
        for rel in (joint.GetBody1Rel(), joint.GetBody0Rel()):
            targets = rel.GetTargets()
            if targets:
                body = self._stage.GetPrimAtPath(targets[0])
                if body.IsValid() and body.HasAPI(UsdPhysics.RigidBodyAPI):
                    return str(body.GetPath())
        return None

    def _set_pallet_attached(
        self,
        requested: bool,
        *,
        pallet_id: int,
    ) -> None:
        """선택한 팔레트를 포크 캐리지에 현재 자세 그대로 연결한다."""
        if self._stage is None:
            return
        pallet_path = PALLET_PATH_FORMAT.format(pallet_id)
        attached_path = PALLET_PATH_FORMAT.format(self._pallet_id)
        joint_exists = self._stage.GetPrimAtPath(PALLET_CARRY_JOINT).IsValid()
        if requested:
            if joint_exists:
                self._pallet_attached = True
                return
            carriage = self._fork_carriage_body()
            pallet = self._stage.GetPrimAtPath(pallet_path)
            if carriage is None:
                print("[Forklift Coupler] 포크 캐리지 rigid body를 찾지 못했습니다")
                return
            if not pallet.IsValid() or not pallet.HasAPI(UsdPhysics.RigidBodyAPI):
                print(f"[Forklift Coupler] 팔레트 강체 없음: {pallet_path}")
                return
            # identity 프레임으로 붙이면 두 원점이 강제로 합쳐지며 튄다. 공용 헬퍼가
            # 현재 상대 자세를 joint local frame에 기록해 삽입 위치 그대로 연결한다.
            from scene import physics

            joint = physics.create_fixed_joint(
                self._stage,
                PALLET_CARRY_JOINT,
                carriage,
                pallet_path,
            )
            joint.CreateJointEnabledAttr(True)
            self._pallet_attached = True
            self._pallet_id = pallet_id
            print(
                f"[Forklift Coupler] 연결 완료: {carriage} <-> {pallet_path}"
            )
            return

        if joint_exists:
            self._stage.RemovePrim(PALLET_CARRY_JOINT)
            print(f"[Forklift Coupler] 연결 해제: {attached_path}")
        self._pallet_attached = False

    def _set_iw_dock_locked(self, requested: bool) -> None:
        """Lock/snap the IW for handoff, or release it for navigation."""
        if requested == self._iw_dock_locked:
            return
        if self._iw_driver is None:
            print("[IW Dock] --iw 없이 도킹 고정 명령을 처리할 수 없습니다")
            return
        if self._iw_driver.set_warehouse_dock_locked(requested):
            self._iw_dock_locked = requested

    def _set_pallet_owner(
        self,
        *,
        fork_requested: bool,
        deck_requested: bool,
        pallet_id: int,
    ) -> None:
        """Switch ownership without a doubly-constrained physics step."""
        if fork_requested and deck_requested:
            print(
                "[Pallet Handoff] 포크와 IW 데크를 동시에 요청해 명령을 거부합니다"
            )
            return

        if deck_requested:
            if self._iw_driver is None:
                print("[Pallet Handoff] --iw 없이 데크 연결을 만들 수 없습니다")
                return
            # Remove the fork constraint first, then create the deck constraint
            # before the next physics step.
            self._set_pallet_attached(False, pallet_id=pallet_id)
            if self._iw_driver.set_warehouse_pallet_attached(True, pallet_id):
                self._deck_pallet_attached = True
                self._pallet_id = pallet_id
            return

        if fork_requested:
            if self._iw_driver is not None:
                if not self._iw_driver.set_warehouse_pallet_attached(
                    False, pallet_id
                ):
                    print("[Pallet Handoff] IW 데크 연결 해제에 실패했습니다")
                    return
            self._deck_pallet_attached = False
            self._set_pallet_attached(True, pallet_id=pallet_id)
            return

        self._set_pallet_attached(False, pallet_id=pallet_id)
        if self._iw_driver is not None:
            self._iw_driver.set_warehouse_pallet_attached(False, pallet_id)
        self._deck_pallet_attached = False
