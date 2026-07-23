# -*- coding: utf-8 -*-
"""Warehouse handoff constraints for iw.hub.

The IW is locked to the canonical warehouse dock only while the forklift is
loading or unloading it.  A warehouse pallet is separately fixed to the IW
chassis while the IW travels.  Pallet ownership is switched by ``fork.py`` in
one simulation update so a pallet is never constrained to the fork and deck
across a physics step.
"""
from __future__ import annotations

import numpy as np
from pxr import Gf, Usd, UsdGeom, UsdPhysics

from scene import physics


WAREHOUSE_DOCK_XY = (0.0, 10.84885)
IW_WORLD_JOINT = "/World/WarehouseDockIwHubFixed"
IW_PALLET_JOINT = "/World/WarehouseDockPalletJoint"
FORK_PALLET_JOINT = "/World/ForkliftPalletCarryJoint"
PALLET_PATH_FORMAT = "/World/Warehouse/Pallet_{:02d}"


def _rigid_body_path(stage: Usd.Stage, root_path: str) -> str | None:
    root = stage.GetPrimAtPath(root_path)
    if not root.IsValid():
        return None
    if root.HasAPI(UsdPhysics.RigidBodyAPI):
        return str(root.GetPath())
    for prim in Usd.PrimRange(root):
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            return str(prim.GetPath())
    return None


def _fix_body_to_world(stage: Usd.Stage, body_path: str) -> bool:
    """Create a world FixedJoint at the body's current world transform."""
    body = stage.GetPrimAtPath(body_path)
    if not body.IsValid() or not body.HasAPI(UsdPhysics.RigidBodyAPI):
        return False
    world = UsdGeom.Xformable(body).ComputeLocalToWorldTransform(
        Usd.TimeCode.Default()
    )
    position = world.ExtractTranslation()
    rotation = world.ExtractRotationQuat()
    imag = rotation.GetImaginary()

    joint = UsdPhysics.FixedJoint.Define(stage, IW_WORLD_JOINT)
    joint.CreateBody1Rel().SetTargets([body_path])
    joint.CreateCollisionEnabledAttr(False)
    joint.CreateLocalPos0Attr(
        Gf.Vec3f(*[float(value) for value in position])
    )
    joint.CreateLocalRot0Attr(
        Gf.Quatf(
            float(rotation.GetReal()),
            float(imag[0]),
            float(imag[1]),
            float(imag[2]),
        )
    )
    joint.CreateLocalPos1Attr(Gf.Vec3f(0.0, 0.0, 0.0))
    joint.CreateLocalRot1Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    return True


class WarehouseDockController:
    """Own the IW world lock and the warehouse-pallet deck constraint."""

    def __init__(self, stage: Usd.Stage, robot, art_path: str):
        self._stage = stage
        self._robot = robot
        self._art_path = art_path
        position, orientation = robot.get_world_pose()
        # Preserve the asset's settled Z and orientation.  Only warehouse X/Y
        # are canonicalized, so nested articulation-root offsets stay valid.
        self._dock_position = np.asarray(position, dtype=float).copy()
        self._dock_position[0] = WAREHOUSE_DOCK_XY[0]
        self._dock_position[1] = WAREHOUSE_DOCK_XY[1]
        self._dock_orientation = np.asarray(orientation, dtype=float).copy()
        self._deck_body = _rigid_body_path(stage, f"{art_path}/chassis")
        if self._deck_body is None:
            self._deck_body = _rigid_body_path(stage, art_path)
        self._deck_relative_position: tuple[float, float, float] | None = None
        self._deck_relative_rotation: (
            tuple[float, float, float, float] | None
        ) = None
        self._deck_pallet_id: int | None = None

    @property
    def dock_locked(self) -> bool:
        return self._stage.GetPrimAtPath(IW_WORLD_JOINT).IsValid()

    @property
    def pallet_on_deck(self) -> bool:
        return self._stage.GetPrimAtPath(IW_PALLET_JOINT).IsValid()

    def set_dock_locked(self, locked: bool) -> bool:
        """Snap to the canonical dock and lock, or release for IW travel."""
        if locked:
            if self.dock_locked:
                return True
            try:
                self._robot.set_linear_velocity(np.zeros(3, dtype=float))
                self._robot.set_angular_velocity(np.zeros(3, dtype=float))
                self._robot.set_world_pose(
                    position=self._dock_position.copy(),
                    orientation=self._dock_orientation.copy(),
                )
                self._robot.set_linear_velocity(np.zeros(3, dtype=float))
                self._robot.set_angular_velocity(np.zeros(3, dtype=float))
            except Exception as exc:
                print(f"[IW Dock] canonical pose 적용 실패: {exc}")
                return False
            body_path = self._deck_body
            if body_path is None or not _fix_body_to_world(
                self._stage, body_path
            ):
                print("[IW Dock] 월드 고정용 rigid body를 찾지 못했습니다")
                return False
            print(
                "[IW Dock] canonical pose 고정 완료: "
                f"x={self._dock_position[0]:.5f}, "
                f"y={self._dock_position[1]:.5f}"
            )
            return True

        if self.dock_locked:
            self._stage.RemovePrim(IW_WORLD_JOINT)
            print("[IW Dock] 월드 FixedJoint 해제 — IW 이동 가능")
        return True

    def set_pallet_on_deck(
        self, attached: bool, pallet_id: int
    ) -> bool:
        """Attach a warehouse pallet to the IW at one canonical deck frame."""
        if not attached:
            if self.pallet_on_deck:
                self._stage.RemovePrim(IW_PALLET_JOINT)
                print(
                    "[IW Deck] 팔레트 연결 해제: "
                    f"Pallet_{self._deck_pallet_id or 0:02d}"
                )
            self._deck_pallet_id = None
            return True

        if self.pallet_on_deck and self._deck_pallet_id == pallet_id:
            return True
        if self.pallet_on_deck:
            self._stage.RemovePrim(IW_PALLET_JOINT)
            self._deck_pallet_id = None
        if self._stage.GetPrimAtPath(FORK_PALLET_JOINT).IsValid():
            print("[IW Deck] 포크 Joint가 남아 있어 데크 연결을 거부합니다")
            return False
        if self._deck_body is None:
            print("[IW Deck] IW chassis rigid body를 찾지 못했습니다")
            return False

        pallet_path = PALLET_PATH_FORMAT.format(pallet_id)
        pallet = self._stage.GetPrimAtPath(pallet_path)
        if not pallet.IsValid() or not pallet.HasAPI(UsdPhysics.RigidBodyAPI):
            print(f"[IW Deck] 팔레트 강체 없음: {pallet_path}")
            return False

        joint = physics.create_fixed_joint(
            self._stage,
            IW_PALLET_JOINT,
            self._deck_body,
            pallet_path,
        )
        if self._deck_relative_position is None:
            pos = joint.GetLocalPos0Attr().Get()
            rot = joint.GetLocalRot0Attr().Get()
            imag = rot.GetImaginary()
            self._deck_relative_position = tuple(float(value) for value in pos)
            self._deck_relative_rotation = (
                float(rot.GetReal()),
                float(imag[0]),
                float(imag[1]),
                float(imag[2]),
            )
            print(
                "[IW Deck] 첫 상차 자세를 canonical pallet frame으로 저장했습니다"
            )
        else:
            joint.GetLocalPos0Attr().Set(Gf.Vec3f(
                *self._deck_relative_position
            ))
            joint.GetLocalRot0Attr().Set(Gf.Quatf(
                *self._deck_relative_rotation
            ))
        joint.CreateJointEnabledAttr(True)
        self._deck_pallet_id = pallet_id
        print(
            f"[IW Deck] Pallet_{pallet_id:02d} 고정 완료 "
            "(canonical XYZ/RPY 유지)"
        )
        return True
