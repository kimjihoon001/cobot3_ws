# -*- coding: utf-8 -*-
"""Warehouse handoff constraints for iw.hub.

The IW is locked to the canonical warehouse dock only while the forklift is
loading or unloading it.  A warehouse pallet is separately fixed to the IW
chassis while the IW travels.  Pallet ownership is switched by ``fork.py`` in
one simulation update so a pallet is never constrained to the fork and deck
across a physics step.
"""
from __future__ import annotations

import json

import numpy as np
from pxr import Gf, Usd, UsdGeom, UsdPhysics

from pjt_utils.deck_geometry import (
    PALLET_SUPPORT_CLEARANCE,
    supported_pallet_hole_center_z,
)
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


def _world_bbox_range(stage: Usd.Stage, prim_path: str):
    """프림 자식 형상을 포함한 월드 정렬 bbox 범위를 반환한다."""
    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid():
        raise ValueError(f"유효하지 않은 prim: {prim_path}")
    cache = UsdGeom.BBoxCache(
        Usd.TimeCode.Default(),
        [UsdGeom.Tokens.default_, UsdGeom.Tokens.render],
    )
    result = cache.ComputeWorldBound(prim).ComputeAlignedRange()
    minimum = result.GetMin()
    maximum = result.GetMax()
    if any(
        not np.isfinite(float(value))
        for value in (*minimum, *maximum)
    ):
        raise ValueError(f"유효하지 않은 bbox: {prim_path}")
    return result


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
        self._deck_pallet_id: int | None = None
        # Runtime 도킹은 정지/스냅/Joint 생성을 서로 다른 physics frame에
        # 수행한다. 동적 articulation을 순간이동한 직후 같은 frame에
        # FixedJoint까지 만들면 PhysX/Fabric의 native pointer가 어긋나
        # world.step()에서 segmentation fault가 날 수 있다.
        self._dock_lock_phase = "idle"
        self._log_deck_geometry()

    @property
    def dock_locked(self) -> bool:
        return self._stage.GetPrimAtPath(IW_WORLD_JOINT).IsValid()

    @property
    def pallet_on_deck(self) -> bool:
        return self._stage.GetPrimAtPath(IW_PALLET_JOINT).IsValid()

    def _deck_surface(self) -> tuple[Gf.Vec3d, Gf.Quatd]:
        """실제 chassis 월드 bbox의 상면 중심과 월드 방향을 반환한다."""
        if self._deck_body is None:
            raise ValueError("IW chassis rigid body를 찾지 못했습니다")
        world_range = _world_bbox_range(self._stage, self._deck_body)
        # chassis rigid-body bbox의 XY 중심은 에셋 원점에서 약 0.31m 치우쳐 있다.
        # 실제 상하차 축은 검증된 canonical IW root XY이므로, bbox는 Z 상면 측정에만
        # 사용하고 X/Y는 도킹 root 좌표를 유지한다.
        world_point = Gf.Vec3d(
            float(self._dock_position[0]),
            float(self._dock_position[1]),
            float(world_range.GetMax()[2]),
        )
        body = self._stage.GetPrimAtPath(self._deck_body)
        body_world = UsdGeom.Xformable(
            body
        ).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        orientation = Gf.Quatd(
            body_world.ExtractRotationQuat().GetNormalized()
        )
        return world_point, orientation

    def geometry_json(self) -> str:
        """ROS 제어기가 사용할 canonical 도킹/팔레트 높이 실측값."""
        point, _ = self._deck_surface()
        payload = {
            "dock_x": round(float(point[0]), 6),
            "dock_y": round(float(point[1]), 6),
            "deck_top_z": round(float(point[2]), 6),
            "pallet_hole_center_z": round(
                supported_pallet_hole_center_z(float(point[2])),
                6,
            ),
            "pallet_support_clearance": PALLET_SUPPORT_CLEARANCE,
        }
        return json.dumps(payload, separators=(",", ":"), sort_keys=True)

    def _log_deck_geometry(self) -> None:
        try:
            point, _ = self._deck_surface()
            hole_z = supported_pallet_hole_center_z(float(point[2]))
            print(
                "[IW Deck Measure] 실제 chassis bbox 기준: "
                f"center=({float(point[0]):.5f}, "
                f"{float(point[1]):.5f}), "
                f"top_z={float(point[2]):.5f}, "
                f"pallet_hole_z={hole_z:.5f}"
            )
        except Exception as exc:
            print(f"[IW Deck Measure] 측정 실패: {exc}")

    def _stop_robot(self) -> None:
        self._robot.set_linear_velocity(np.zeros(3, dtype=float))
        self._robot.set_angular_velocity(np.zeros(3, dtype=float))

    def _at_canonical_dock(self) -> bool:
        """Return whether the live IW pose is already close enough to dock."""
        position, orientation = self._robot.get_world_pose()
        position = np.asarray(position, dtype=float)
        orientation = np.asarray(orientation, dtype=float)
        target_orientation = np.asarray(self._dock_orientation, dtype=float)

        position_error = float(np.linalg.norm(
            position - np.asarray(self._dock_position, dtype=float)
        ))
        orientation_norm = float(np.linalg.norm(orientation))
        target_norm = float(np.linalg.norm(target_orientation))
        if orientation_norm <= 1e-9 or target_norm <= 1e-9:
            return False
        quaternion_alignment = abs(float(np.dot(
            orientation / orientation_norm,
            target_orientation / target_norm,
        )))
        return position_error <= 0.01 and quaternion_alignment >= 0.99995

    def set_dock_locked(
        self,
        locked: bool,
        *,
        immediate: bool = False,
    ) -> bool:
        """Snap and lock IW without mutating pose and constraints in one frame.

        ``immediate`` is reserved for scene construction before physics starts.
        Runtime callers repeatedly request ``locked=True``; each call advances
        one phase, and the main loop provides a physics step between phases.
        """
        if locked:
            if self.dock_locked:
                self._dock_lock_phase = "idle"
                return True
            try:
                if immediate:
                    self._stop_robot()
                    self._robot.set_world_pose(
                        position=self._dock_position.copy(),
                        orientation=self._dock_orientation.copy(),
                    )
                    self._stop_robot()
                    body_path = self._deck_body
                    if body_path is None or not _fix_body_to_world(
                        self._stage, body_path
                    ):
                        print("[IW Dock] 월드 고정용 rigid body를 찾지 못했습니다")
                        return False
                    self._dock_lock_phase = "idle"
                    print(
                        "[IW Dock] 초기 canonical pose 고정 완료: "
                        f"x={self._dock_position[0]:.5f}, "
                        f"y={self._dock_position[1]:.5f}"
                    )
                    self._log_deck_geometry()
                    return True

                if self._dock_lock_phase == "idle":
                    self._stop_robot()
                    self._dock_lock_phase = "stopped"
                    print("[IW Dock] 도킹 고정 1/3: 속도 정지")
                    return False

                if self._dock_lock_phase == "stopped":
                    self._stop_robot()
                    if not self._at_canonical_dock():
                        self._robot.set_world_pose(
                            position=self._dock_position.copy(),
                            orientation=self._dock_orientation.copy(),
                        )
                        self._stop_robot()
                        print("[IW Dock] 도킹 고정 2/3: canonical pose 보정")
                    else:
                        print("[IW Dock] 도킹 고정 2/3: 현재 pose 유지")
                    self._dock_lock_phase = "pose_settled"
                    return False

                # A world.step() has now occurred after the optional pose snap.
                # Only in this later frame is the PhysX constraint authored.
                self._stop_robot()
                body_path = self._deck_body
                if body_path is None or not _fix_body_to_world(
                    self._stage, body_path
                ):
                    print("[IW Dock] 월드 고정용 rigid body를 찾지 못했습니다")
                    self._dock_lock_phase = "idle"
                    return False
                self._dock_lock_phase = "idle"
                print(
                    "[IW Dock] 도킹 고정 3/3 완료: "
                    f"x={self._dock_position[0]:.5f}, "
                    f"y={self._dock_position[1]:.5f}"
                )
                return True
            except Exception as exc:
                print(f"[IW Dock] canonical pose 적용 실패: {exc}")
                self._dock_lock_phase = "idle"
                return False

        self._dock_lock_phase = "idle"
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

        try:
            deck_point, _deck_orientation = self._deck_surface()
            pallet_range = _world_bbox_range(self._stage, pallet_path)
            pallet_center = pallet_range.GetMidpoint()
            xy_error = np.hypot(
                float(pallet_center[0]) - float(deck_point[0]),
                float(pallet_center[1]) - float(deck_point[1]),
            )
            support_z = float(deck_point[2]) + PALLET_SUPPORT_CLEARANCE
            z_error = float(pallet_range.GetMin()[2]) - support_z
            print(
                "[IW Deck] Joint 전 실제 안착 검증: "
                f"deck_top_z={float(deck_point[2]):.5f}, "
                f"pallet_world_min_z={float(pallet_range.GetMin()[2]):.5f}, "
                f"xy_error={xy_error:.5f}, z_error={z_error:+.5f}"
            )
            # 실행 중인 dynamic rigid body를 USD Xform으로 순간이동한 직후 같은
            # 프레임에 Joint를 만들면 Fabric/PhysX 포인터가 어긋나 네이티브 크래시가
            # 발생한다. 지게차가 실측 높이로 물리적으로 내려놓게 하고 여기서는 현재
            # 자세를 절대 변경하지 않는다.
            if abs(z_error) > 0.025:
                print(
                    "[IW Deck] 팔레트가 데크 지지면에서 너무 멀어 연결을 거부합니다: "
                    f"z_error={z_error:+.5f}m"
                )
                return False
            if xy_error > 0.15:
                print(
                    "[IW Deck] 팔레트 중심이 IW 도킹축에서 너무 멀어 연결을 거부합니다: "
                    f"xy_error={xy_error:.5f}m"
                )
                return False
        except Exception as exc:
            print(f"[IW Deck] 팔레트 안착 검증 실패: {exc}")
            return False

        joint = physics.create_fixed_joint(
            self._stage,
            IW_PALLET_JOINT,
            self._deck_body,
            pallet_path,
        )
        joint.CreateJointEnabledAttr(True)
        self._deck_pallet_id = pallet_id
        print(
            f"[IW Deck] Pallet_{pallet_id:02d} 고정 완료 "
            "(현재 물리 자세 유지)"
        )
        return True
