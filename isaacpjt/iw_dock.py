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
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

from pjt_utils.deck_geometry import (
    IW_LOAD_MAP_X_OFFSET_M,
    PALLET_SUPPORT_CLEARANCE,
    deck_target_xy,
    supported_pallet_hole_center_z,
)
from scene import physics


WAREHOUSE_DOCK_XY = (0.0, 10.84885)
IW_WORLD_JOINT = "/World/WarehouseDockIwHubFixed"
IW_PALLET_JOINT = "/World/WarehouseDockPalletJoint"
FORK_PALLET_JOINT = "/World/ForkliftPalletCarryJoint"
PALLET_PATH_FORMAT = "/World/Warehouse/Pallet_{:02d}"
INITIAL_IW_PALLET_PATH = "/World/IwHubCargo/Pallet_00"
INITIAL_IW_DECK_JOINT = "/World/IwHubCargo/DeckJoint"


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
        self._deck_pallet_id: int | None = (
            0
            if self._stage.GetPrimAtPath(INITIAL_IW_DECK_JOINT).IsValid()
            else None
        )
        self._deck_pallet_rigid = None
        self._deck_root_relative = None
        # Runtime 도킹은 정지/스냅/Joint 생성을 서로 다른 physics frame에
        # 수행한다. 동적 articulation을 순간이동한 직후 같은 frame에
        # FixedJoint까지 만들면 PhysX/Fabric의 native pointer가 어긋나
        # world.step()에서 segmentation fault가 날 수 있다.
        self._dock_lock_phase = "idle"
        self._locked_position = None
        self._locked_orientation = None
        if self._deck_pallet_id is not None:
            self.set_pallet_deck_collision_filtered(
                True, self._deck_pallet_id
            )
        self._log_deck_geometry()

    @staticmethod
    def _pose_matrix(position, quat_wxyz) -> Gf.Matrix4d:
        matrix = Gf.Matrix4d()
        matrix.SetRotate(
            Gf.Quatd(
                float(quat_wxyz[0]),
                float(quat_wxyz[1]),
                float(quat_wxyz[2]),
                float(quat_wxyz[3]),
            )
        )
        matrix.SetTranslateOnly(
            Gf.Vec3d(*(float(value) for value in position))
        )
        return matrix

    def _begin_deck_follow(self, pallet_id: int) -> bool:
        """현재 팔레트 자세를 보존한 채 IW root 상대 자세를 기록한다."""
        pallet_path = self._pallet_path(pallet_id)
        pallet = self._stage.GetPrimAtPath(pallet_path)
        if not pallet.IsValid() or not pallet.HasAPI(UsdPhysics.RigidBodyAPI):
            return False
        try:
            from isaacsim.core.prims import SingleRigidPrim

            rigid = SingleRigidPrim(
                pallet_path, name="iwhub_deck_pallet_probe"
            )
            rigid.initialize()
            pallet_position, pallet_quat = rigid.get_world_pose()
            root_position, root_quat = self._robot.get_world_pose()
            pallet_world = self._pose_matrix(pallet_position, pallet_quat)
            root_world = self._pose_matrix(root_position, root_quat)
            UsdPhysics.RigidBodyAPI(
                pallet
            ).CreateKinematicEnabledAttr(True).Set(True)
            self._deck_pallet_rigid = rigid
            self._deck_root_relative = (
                pallet_world * root_world.GetInverse()
            )
            return True
        except Exception as exc:
            print(f"[IW Deck] kinematic 추종 초기화 실패: {exc}")
            self._deck_pallet_rigid = None
            self._deck_root_relative = None
            return False

    def _stop_deck_follow(self) -> None:
        pallet_path = (
            self._pallet_path(self._deck_pallet_id)
            if self._deck_pallet_id is not None
            else None
        )
        pallet = (
            self._stage.GetPrimAtPath(pallet_path)
            if pallet_path is not None
            else None
        )
        try:
            if (
                pallet is not None
                and pallet.IsValid()
                and pallet.HasAPI(UsdPhysics.RigidBodyAPI)
            ):
                # kinematic body에는 속도를 쓸 수 없다. 먼저 dynamic으로
                # 되돌린 뒤 잔여 속도를 0으로 만들어야 release가 중간에
                # 예외로 끊기지 않고 포크가 실제 팔레트를 넘겨받는다.
                UsdPhysics.RigidBodyAPI(
                    pallet
                ).CreateKinematicEnabledAttr(False).Set(False)
            if self._deck_pallet_rigid is not None:
                self._deck_pallet_rigid.set_linear_velocity(
                    np.zeros(3, dtype=float)
                )
                self._deck_pallet_rigid.set_angular_velocity(
                    np.zeros(3, dtype=float)
                )
        except Exception as exc:
            print(f"[IW Deck] dynamic 복원 경고: {exc}")
        self._deck_pallet_rigid = None
        self._deck_root_relative = None

    def update(self) -> None:
        """도킹 중 속도를 정지하고 구형 kinematic 적재만 호환한다."""
        if self.dock_locked:
            # 포크 접촉력이 IW를 밀지 못하도록 인계가 끝날 때까지 승인된
            # 도킹 pose를 매 physics frame 유지한다. 같은 pose만 반복 적용해
            # 누적 이동은 만들지 않는다.
            if (
                self._locked_position is not None
                and self._locked_orientation is not None
            ):
                self._robot.set_world_pose(
                    position=self._locked_position,
                    orientation=self._locked_orientation,
                )
            self._stop_robot()
        if self._deck_pallet_id is None or not self.pallet_on_deck:
            return
        # 정상 운반 경로는 실제 FixedJoint다. PhysX가 차체와 팔레트를 함께
        # 적분하므로 set_world_pose 추종을 절대 섞지 않는다.
        for joint_path in (IW_PALLET_JOINT, INITIAL_IW_DECK_JOINT):
            joint_prim = self._stage.GetPrimAtPath(joint_path)
            if joint_prim.IsValid() and joint_prim.IsA(
                UsdPhysics.FixedJoint
            ):
                return
        if (
            self._deck_pallet_rigid is None
            or self._deck_root_relative is None
        ):
            if not self._begin_deck_follow(self._deck_pallet_id):
                return
        root_position, root_quat = self._robot.get_world_pose()
        root_world = self._pose_matrix(root_position, root_quat)
        pallet_world = self._deck_root_relative * root_world
        position = pallet_world.ExtractTranslation()
        rotation = pallet_world.ExtractRotationQuat().GetNormalized()
        imaginary = rotation.GetImaginary()
        self._deck_pallet_rigid.set_world_pose(
            position=np.asarray(position, dtype=float),
            orientation=np.asarray(
                [
                    float(rotation.GetReal()),
                    float(imaginary[0]),
                    float(imaginary[1]),
                    float(imaginary[2]),
                ],
                dtype=float,
            ),
        )

    @property
    def dock_locked(self) -> bool:
        return self._stage.GetPrimAtPath(IW_WORLD_JOINT).IsValid()

    @property
    def deck_pallet_id(self) -> int | None:
        """지금 IW 데크에 결속된 팔레트 ID (없으면 None)."""
        return self._deck_pallet_id

    def pallet_world_min_z(self, pallet_path: str) -> float | None:
        """팔레트 월드 bbox 최저점. 결속 게이트가 z_error를 재는 바로 그 값이다.

        ROS 측이 안착 높이를 폐루프로 맞추려면 게이트와 **같은 양**을 봐야 한다.
        강체 원점(pallet_position)은 피벗 위치에 따라 이 값과 달라질 수 있다.
        """
        try:
            return float(
                _world_bbox_range(self._stage, pallet_path).GetMin()[2]
            )
        except Exception:
            return None

    def deck_pallet_path(self) -> str | None:
        """데크에 결속된 팔레트의 prim 경로 (없으면 None).

        초기 IW 적재 팔레트와 창고 팔레트는 ID 공간이 겹친다(둘 다 0일 수 있다).
        `_pallet_path()`는 창고를 먼저 조회하므로, 둘이 동시에 존재하면 ID만으로는
        창고 쪽이 잡힌다. 일반 통합 실행은 main.py가 창고 0번 슬롯을 비워 이를
        피하지만(`--iw --fork` 창고 시험은 의도적으로 유지한다), 경로 결정이 그
        실행 모드 분기에 의존하지 않도록 데크 조인트를 직접 본다 — 이 조인트는
        팔레트를 내릴 때 IW_PALLET_JOINT와 함께 제거되므로 '초기 IW 팔레트가 아직
        데크에 있다'와 등가다.
        """
        if self._deck_pallet_id is None:
            return None
        if self._stage.GetPrimAtPath(INITIAL_IW_DECK_JOINT).IsValid():
            return INITIAL_IW_PALLET_PATH
        return self._pallet_path(self._deck_pallet_id)

    @property
    def pallet_on_deck(self) -> bool:
        return (
            self._stage.GetPrimAtPath(IW_PALLET_JOINT).IsValid()
            or self._stage.GetPrimAtPath(INITIAL_IW_DECK_JOINT).IsValid()
        )

    def set_pallet_deck_collision_filtered(
        self, filtered: bool, pallet_id: int
    ) -> bool:
        """포크 인계 중에만 팔레트와 IW 전체의 상호 충돌을 차단한다.

        DeckJoint를 제거한 뒤에도 팔레트는 섀시 상면과 접촉한다. 이 상태에서
        포크 FixedJoint로 팔레트를 들어 올리면 접촉 솔버와 조인트/리프트
        드라이브가 동시에 같은 강체를 구속해 30kN까지 힘이 포화될 수 있다.
        chassis 하나만 필터하면 상승 중 팔레트가 lift/wheel/caster 링크와
        접촉해 IW 전체를 들어 올릴 수 있으므로 articulation root 전체를
        대상으로 한다. 포크가 소유하는 동안만 차단하고 데크로 돌려놓기
        전에 반드시 복원한다.
        """
        iw_root = self._stage.GetPrimAtPath(self._art_path)
        if not iw_root.IsValid():
            print("[IW Deck] 충돌 필터 대상 IW articulation을 찾지 못했습니다")
            return False
        pallet_path = self._pallet_path(pallet_id)
        pallet = self._stage.GetPrimAtPath(pallet_path)
        if not pallet.IsValid():
            print(
                "[IW Deck] 충돌 필터 대상이 없습니다: "
                f"{pallet_path} <-> {self._art_path}"
            )
            return False

        api = UsdPhysics.FilteredPairsAPI.Apply(pallet)
        rel = api.CreateFilteredPairsRel()
        deck_path = Sdf.Path(self._art_path)
        targets = rel.GetTargets()
        if filtered:
            if deck_path not in targets:
                rel.AddTarget(deck_path)
                print(
                    "[IW Deck] 포크 인계 충돌 차단: "
                    f"{pallet_path} <-> {self._art_path}"
                )
            return True

        if deck_path in targets:
            rel.RemoveTarget(deck_path)
            print(
                "[IW Deck] 데크 접촉 복원: "
                f"{pallet_path} <-> {self._art_path}"
            )
        return True

    def pallet_deck_collision_filtered(self, pallet_id: int) -> bool:
        """현재 팔레트↔IW 섀시 충돌 필터가 적용됐는지 반환한다."""
        if not self._stage.GetPrimAtPath(self._art_path).IsValid():
            return False
        pallet = self._stage.GetPrimAtPath(self._pallet_path(pallet_id))
        if (
            not pallet.IsValid()
            or not pallet.HasAPI(UsdPhysics.FilteredPairsAPI)
        ):
            return False
        api = UsdPhysics.FilteredPairsAPI(pallet)
        return bool(
            api
            and Sdf.Path(self._art_path)
            in api.GetFilteredPairsRel().GetTargets()
        )

    def _pallet_path(self, pallet_id: int) -> str:
        """Resolve the physical body for a logical warehouse pallet ID."""
        warehouse_path = PALLET_PATH_FORMAT.format(pallet_id)
        if self._stage.GetPrimAtPath(warehouse_path).IsValid():
            return warehouse_path
        if (
            pallet_id == 0
            and self._stage.GetPrimAtPath(INITIAL_IW_PALLET_PATH).IsValid()
        ):
            return INITIAL_IW_PALLET_PATH
        return warehouse_path

    def _deck_surface(
        self, forward_offset: float = 0.0
    ) -> tuple[Gf.Vec3d, Gf.Quatd]:
        """실제 IW pose 기준 팔레트 배치 목표와 chassis 상면을 반환한다."""
        if self._deck_body is None:
            raise ValueError("IW chassis rigid body를 찾지 못했습니다")
        world_range = _world_bbox_range(self._stage, self._deck_body)
        # PhysX/Fabric으로 이동한 articulation의 USD bbox X/Y는 초기 스폰
        # 좌표에 남을 수 있다. 실제 root pose에 검증된 로컬 deck offset을
        # 회전 적용하고, bbox는 안정적인 상면 Z에만 사용한다.
        root_position, root_quat = self._robot.get_world_pose()
        w, x, y, z = (float(value) for value in root_quat)
        root_yaw = np.arctan2(
            2.0 * (w * z + x * y),
            1.0 - 2.0 * (y * y + z * z),
        )
        target_x, target_y = deck_target_xy(
            float(root_position[0]),
            float(root_position[1]),
            float(root_yaw),
            forward_offset,
        )
        world_point = Gf.Vec3d(
            target_x,
            target_y,
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
        # 이 토픽은 런타임 목표가 아니라 IW 형상의 canonical 기준이다.
        # 실제 이동한 root X/Y를 내보내면 회수 노드가 이를 새 도킹축으로
        # 오인한다. 동적 목표는 /forklift/handoff_state로만 전달한다.
        runtime_point, _ = self._deck_surface()
        point = Gf.Vec3d(
            float(self._dock_position[0]) + IW_LOAD_MAP_X_OFFSET_M,
            float(self._dock_position[1]),
            float(runtime_point[2]),
        )
        payload = {
            "dock_x": round(float(point[0]), 6),
            "dock_y": round(float(point[1]), 6),
            "deck_top_z": round(float(point[2]), 6),
            "pallet_hole_center_z": round(
                supported_pallet_hole_center_z(float(point[2])),
                6,
            ),
            "pallet_support_clearance": PALLET_SUPPORT_CLEARANCE,
            "frame": "canonical_dock",
        }
        return json.dumps(payload, separators=(",", ":"), sort_keys=True)

    def _log_deck_geometry(self) -> None:
        try:
            runtime_point, _ = self._deck_surface()
            point = Gf.Vec3d(
                float(self._dock_position[0]) + IW_LOAD_MAP_X_OFFSET_M,
                float(self._dock_position[1]),
                float(runtime_point[2]),
            )
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

    def set_dock_locked(
        self,
        locked: bool,
        *,
        immediate: bool = False,
    ) -> bool:
        """Lock an IW that Nav2 has already driven to the dock.

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
                    UsdGeom.Scope.Define(self._stage, IW_WORLD_JOINT)
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
                    # Nav2는 AMCL이 보정한 map pose로 최종 XY/yaw를 이미
                    # 검증한다. Isaac raw world pose를 map canonical 좌표와
                    # 다시 비교하면 map→odom 보정량까지 위치 오차로 오인한다.
                    print("[IW Dock] 도킹 고정 2/3: ROS 검증 pose 유지")
                    self._dock_lock_phase = "pose_settled"
                    return False

                # Nav2가 저속 정렬을 끝낸 현재 X/Y/yaw를 그대로 고정한다.
                # canonical 좌표로 다시 덮어쓰면 GUI에서 순간이동으로 보인다.
                self._stop_robot()
                position, orientation = self._robot.get_world_pose()
                locked_position = np.asarray(position, dtype=float).copy()
                # Nav2 도착 직후에는 휠/서스펜션 접촉 반력으로 chassis Z가
                # 잠깐 튈 수 있다. 그 순간의 Z까지 매 프레임 고정하면 인계
                # 동안 IW가 바닥에서 떠 보인다. X/Y는 스냅 없이 현재값을
                # 유지하되 Z는 씬 생성 시 확보한 정상 접지 높이를 사용한다.
                measured_z = float(locked_position[2])
                locked_position[2] = self._dock_position[2]
                self._locked_position = locked_position
                self._locked_orientation = np.asarray(
                    orientation, dtype=float
                ).copy()
                self._stop_robot()
                UsdGeom.Scope.Define(self._stage, IW_WORLD_JOINT)
                self._dock_lock_phase = "idle"
                print(
                    "[IW Dock] 현재 pose 고정 완료(스냅 없음, 포크 밀림 차단): "
                    f"x={float(self._locked_position[0]):.5f}, "
                    f"y={float(self._locked_position[1]):.5f}, "
                    f"z={float(self._locked_position[2]):.5f} "
                    f"(측정 {measured_z:.5f})"
                )
                return True
            except Exception as exc:
                print(f"[IW Dock] canonical pose 적용 실패: {exc}")
                self._dock_lock_phase = "idle"
                return False

        self._dock_lock_phase = "idle"
        self._locked_position = None
        self._locked_orientation = None
        if self.dock_locked:
            self._stage.RemovePrim(IW_WORLD_JOINT)
            print("[IW Dock] 자세 유지 잠금 해제 — IW 이동 가능")
        return True

    def set_pallet_on_deck(
        self,
        attached: bool,
        pallet_id: int,
        forward_offset: float = 0.0,
    ) -> bool:
        """Attach a warehouse pallet to the IW at one canonical deck frame."""
        if not attached:
            self._stop_deck_follow()
            removed = []
            for joint_path in (IW_PALLET_JOINT, INITIAL_IW_DECK_JOINT):
                if self._stage.GetPrimAtPath(joint_path).IsValid():
                    self._stage.RemovePrim(joint_path)
                    removed.append(joint_path)
            if removed:
                print(
                    "[IW Deck] 팔레트 연결 해제: "
                    f"Pallet_{self._deck_pallet_id or 0:02d} "
                    f"({', '.join(removed)})"
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

        pallet_path = self._pallet_path(pallet_id)
        pallet = self._stage.GetPrimAtPath(pallet_path)
        if not pallet.IsValid() or not pallet.HasAPI(UsdPhysics.RigidBodyAPI):
            print(f"[IW Deck] 팔레트 강체 없음: {pallet_path}")
            return False

        try:
            deck_point, _deck_orientation = self._deck_surface(forward_offset)
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
                f"forward_offset={forward_offset:.3f}, "
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

        try:
            from isaacsim.core.prims import SingleRigidPrim

            # 지게차와 동일하게 런타임 PhysX 자세로 local joint frame을
            # 계산한다. USD authoring pose를 쓰면 이동 후 결속 순간 스냅한다.
            deck_rigid = SingleRigidPrim(
                self._deck_body, name="iwhub_deck_body_probe"
            )
            pallet_rigid = SingleRigidPrim(
                pallet_path, name="iwhub_deck_pallet_probe"
            )
            deck_rigid.initialize()
            pallet_rigid.initialize()
            deck_position, deck_quat = deck_rigid.get_world_pose()
            pallet_position, pallet_quat = pallet_rigid.get_world_pose()
            UsdPhysics.RigidBodyAPI(
                pallet
            ).CreateKinematicEnabledAttr(False).Set(False)
            physics.create_fixed_joint(
                self._stage,
                IW_PALLET_JOINT,
                self._deck_body,
                pallet_path,
                body0_world=self._pose_matrix(
                    deck_position, deck_quat
                ),
                body1_world=self._pose_matrix(
                    pallet_position, pallet_quat
                ),
                exclude_from_articulation=True,
            )
        except Exception as exc:
            print(f"[IW Deck] FixedJoint 결속 실패: {exc}")
            return False
        self._deck_pallet_id = pallet_id
        print(
            f"[IW Deck] Pallet_{pallet_id:02d} 고정 완료 "
            "(현재 물리 자세 유지, excludeFromArticulation FixedJoint)"
        )
        return True
