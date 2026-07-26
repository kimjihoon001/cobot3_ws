# -*- coding: utf-8 -*-
"""지게차 드라이버 (--fork) — 지게차 B (포크 승강). 창고에서 팔레트를 랙에 적재.

로봇 모델은 robots/transporter.py, ROS 브리지는 ros/robot_bridge.py. 배선만 한다(§5.6).
부가장치 없음 — 조인트 브리지만. 제어는 ROS2 가 /{ns}/joint_command 로 직접 한다.
"""
from __future__ import annotations

import json
import math
import time

import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

from robot_base import Driver, ros_fail
from robots.control import TransporterController
from robots.transporter import TransporterAMR
from scene.ground import COMMON_FLOOR_Z
from scene.physics import create_fixed_joint
from pjt_utils.deck_geometry import (
    IW_LOAD_MAP_X_OFFSET_M,
    PALLET_HOLE_BOTTOM_Z,
    PALLET_HOLE_TOP_Z,
    PALLET_SUPPORT_CLEARANCE,
)

# 창고 입구 중앙축의 내부 대기점. 창고 입구 경계는 Y=13이고 대기점은 그 안쪽이다.
POSE = (IW_LOAD_MAP_X_OFFSET_M, 14.5, COMMON_FLOOR_Z)
# ForkliftB 포크는 모델 로컬 -X 방향이다. +90°로 놓으면 포크가 월드 -Y,
# 즉 창고 밖 입구 중앙의 AMR을 향한다.
YAW_DEG = 90.0
PALLET_PATH_FORMAT = "/World/Warehouse/Pallet_{:02d}"
INITIAL_IW_PALLET_PATH = "/World/IwHubCargo/Pallet_00"
PALLET_CARRY_JOINT = "/World/ForkliftPalletCarryJoint"

# Isaac 5.1 ForkliftB 원본 메시에서 측정한 직선 포크 날 구간(root local).
# 좌표를 매 프레임 실제 forklift/pallet 자세로 변환해 결합 전 검증한다.
FORK_TINE_X = (-2.10, -1.50)
FORK_TINE_Y = ((-0.365, -0.245), (0.245, 0.365))
FORK_TINE_Z_AT_ZERO = (0.179587, 0.232846)
PALLET_HALF_EXTENTS = (0.6065, 0.4065)
PICKUP_VERTICAL_CLEARANCE = 0.001
PICKUP_MAX_LATERAL_ERROR = 0.08
PICKUP_MAX_YAW_ERROR = math.radians(3.0)
PICKUP_MIN_INSERTION_OVERLAP = 0.35


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
        # ROS 명령 수신 프레임과 무관하게 IW의 다단계 잠금을 physics
        # frame마다 진행할 수 있도록 마지막 요청값을 래치한다.
        self._iw_dock_lock_requested = False
        self._pallet_id = 0
        self._deck_release_pending = False
        self._deck_clear_streak = 0
        self._deck_pickup_prevalidated = False
        self._handoff_state_publisher = None
        self._physics_dt = 1.0 / 60.0
        self._load_mass_reported = False
        self._carry_pallet_rigid = None
        self._carry_root_relative = None
        self._carry_lift_origin = 0.0
        self._carry_follow_reported = False
        self._carry_start_z = None
        self._carry_pose_error = None
        self._carry_target_position = None
        self._last_command_time = None
        self._command_watchdog_reported = False
        self._pickup_lift_override = None
        self._pickup_alignment = None
        self._pickup_alignment_log_time = 0.0
        self._asset_root_authored_world = None
        self._art_authored_world_inverse = None

    def spawn(self, stage):
        self._fk.spawn(stage, self.root, POSE, yaw_deg=YAW_DEG)

    def configure(self, world):
        self._controller = TransporterController(self.robot)
        # GUI 렌더 FPS가 아니라 Isaac physics timestep으로 평면 운동을 적분한다.
        self._physics_dt = float(world.get_physics_dt())

    def finalize(self, world, stage, opts):
        self._stage = stage
        # 포크 메시 좌표(에셋 바깥 root 기준)를 움직이는 articulation root
        # 좌표로 옮기는 고정 변환을 초기 자세에서 보관한다.
        xform_cache = UsdGeom.XformCache(Usd.TimeCode.Default())
        asset_root = stage.GetPrimAtPath(self.root)
        art_root = stage.GetPrimAtPath(self.art)
        if asset_root.IsValid() and art_root.IsValid():
            self._asset_root_authored_world = (
                xform_cache.GetLocalToWorldTransform(asset_root)
            )
            self._art_authored_world_inverse = (
                xform_cache.GetLocalToWorldTransform(art_root).GetInverse()
            )
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
                state_node = RB.build_string_pub(
                    f"/World/RosHandoffState_{self.ns}",
                    "/forklift/handoff_state",
                )
                self._handoff_state_publisher = RB.StringPublisher(state_node)
            except Exception:
                ros_fail("지게차 조인트 브리지")

    def update(self, is_playing: bool):
        if not is_playing or self._controller is None or self._poller is None:
            return
        cmd = self._poller.poll()
        if cmd:
            self._last_command_time = time.monotonic()
            self._command_watchdog_reported = False
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
            # 결속 사전검사에서 실측한 미세 높이 보정은 ROS의 nominal
            # lift 명령보다 우선한다. 실제 joint가 보정 높이에 도달할 때까지
            # 반복 프레임에서 유지하고, 게이트 통과 즉시 해제한다.
            if (
                pallet_attach_request is True
                and self._pickup_lift_override is not None
            ):
                self._controller.set_fork(self._pickup_lift_override)
            if dock_lock_request is not None:
                self._iw_dock_lock_requested = dock_lock_request
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
        elif (
            self._last_command_time is not None
            and time.monotonic() - self._last_command_time > 0.5
        ):
            # ROS 노드 중단/통신 단절 시 마지막 drive 명령을 계속 유지하면
            # 지게차가 작업 경로 밖까지 달아난다. 0.5초 이상 새 명령이 없으면
            # Isaac 쪽에서 독립적으로 정지한다.
            self._controller.set_drive(0.0)
            self._controller.set_steer(0.0)
            if not self._command_watchdog_reported:
                print("[Forklift Watchdog] ROS 명령 0.5초 단절 — 즉시 정지")
                self._command_watchdog_reported = True
        # IW 잠금은 정지 → pose 확인 → 고정 생성 사이에 각각 physics frame이
        # 필요하다. 새 ROS 메시지가 들어온 프레임에서만 호출하면 첫 단계에
        # 머물 수 있으므로 래치된 요청을 매 frame 진행한다.
        if self._iw_dock_lock_requested or self._iw_dock_locked:
            self._set_iw_dock_locked(self._iw_dock_lock_requested)

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
        self._report_load_mass_once()
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
        self._publish_handoff_state()

    def _report_load_mass_once(self) -> None:
        """PhysX가 실제 사용하는 IW 적재 강체 질량을 시작 시 한 번 확인한다."""
        if self._load_mass_reported or self._stage is None:
            return
        load = self._stage.GetPrimAtPath(INITIAL_IW_PALLET_PATH)
        if not load.IsValid() or not load.HasAPI(UsdPhysics.RigidBodyAPI):
            return
        try:
            from isaacsim.core.prims import SingleRigidPrim

            rigid = SingleRigidPrim(
                INITIAL_IW_PALLET_PATH,
                name="iwhub_cargo_mass_probe",
            )
            rigid.initialize()
            mass = float(rigid.get_mass())
        except Exception as exc:
            print(f"[Pallet Physics] PhysX 실효 질량 조회 실패: {exc}")
            self._load_mass_reported = True
            return
        authored = UsdPhysics.MassAPI(load).GetMassAttr().Get()
        print(
            "[Pallet Physics] IW Pallet_00 질량: "
            f"PhysX={mass:.3f}kg, USD={authored}kg"
        )
        self._load_mass_reported = True

    def _publish_handoff_state(self) -> None:
        """ROS 상태기가 실제 인계 물리 상태를 게이팅할 수 있게 발행한다."""
        if self._stage is None or self._handoff_state_publisher is None:
            return
        fork_attached = self._stage.GetPrimAtPath(PALLET_CARRY_JOINT).IsValid()
        deck_attached = bool(
            self._iw_driver
            and self._iw_driver.has_warehouse_pallet_attached()
        )
        deck_collision_filtered = bool(
            self._iw_driver
            and self._iw_driver.warehouse_pallet_deck_collision_filtered(
                self._pallet_id
            )
        )
        fork_collision_filtered = self._pallet_fork_collision_filtered(
            self._pallet_id
        )
        owner = (
            "conflict" if fork_attached and deck_attached
            else "fork" if fork_attached
            else "deck" if deck_attached
            else "none"
        )
        pallet_path = self._pallet_path(self._pallet_id)
        pallet_z = None
        pallet_rise = None
        pallet_position_json = None
        if self._carry_pallet_rigid is not None:
            try:
                pallet_position, _ = self._carry_pallet_rigid.get_world_pose()
                pallet_z = float(pallet_position[2])
                pallet_position_json = [
                    float(value) for value in pallet_position
                ]
                if self._carry_start_z is not None:
                    pallet_rise = pallet_z - self._carry_start_z
            except Exception:
                pass
        forklift_position_json = None
        if self.robot is not None:
            try:
                forklift_position, _ = self.robot.get_world_pose()
                forklift_position_json = [
                    float(value) for value in forklift_position
                ]
            except Exception:
                pass
        expected_rise = (
            float(self._controller._lift) - self._carry_lift_origin
            if fork_attached and self._controller is not None
            else None
        )
        iw_tilt_deg = None
        iw_position_json = None
        iw_yaw = None
        deck_target_position_json = None
        if self._iw_driver is not None and self._iw_driver.robot is not None:
            try:
                iw_position, iw_quat = self._iw_driver.robot.get_world_pose()
                iw_position_json = [float(value) for value in iw_position]
                w, x, y, z = (float(value) for value in iw_quat)
                iw_yaw = math.atan2(
                    2.0 * (w * z + x * y),
                    1.0 - 2.0 * (y * y + z * z),
                )
                del w, z
                up_z = max(-1.0, min(1.0, 1.0 - 2.0 * (x * x + y * y)))
                iw_tilt_deg = math.degrees(math.acos(up_z))
                deck_point, _ = self._iw_driver._deck_surface()
                deck_target_position_json = [
                    float(deck_point[0]),
                    float(deck_point[1]),
                    float(deck_point[2]) + PALLET_SUPPORT_CLEARANCE,
                ]
            except Exception:
                pass
        self._handoff_state_publisher.publish(json.dumps(
            {
                "owner": owner,
                "pallet_id": self._pallet_id,
                "pallet_path": pallet_path,
                "fork_attached": fork_attached,
                "deck_attached": deck_attached,
                "deck_collision_filtered": deck_collision_filtered,
                "fork_collision_filtered": fork_collision_filtered,
                "dock_locked": self._iw_dock_locked,
                "iw_available": self._iw_driver is not None,
                "iw_world_position": iw_position_json,
                "iw_world_yaw": iw_yaw,
                "pallet_z": pallet_z,
                "pallet_position": pallet_position_json,
                "pallet_target_position": (
                    deck_target_position_json
                ),
                "forklift_position": forklift_position_json,
                "pallet_rise": pallet_rise,
                "expected_rise": expected_rise,
                "carry_pose_error": self._carry_pose_error,
                "iw_tilt_deg": iw_tilt_deg,
                "pickup_alignment": self._pickup_alignment,
            },
            separators=(",", ":"),
        ))

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

    def _set_pallet_fork_collision_filtered(
        self, filtered: bool, pallet_id: int
    ) -> bool:
        """운반 중 팔레트와 지게차 아티큘레이션 전체의 내부 충돌을 제어한다.

        FixedJoint의 collisionEnabled=False는 조인트가 직접 연결한 lift 링크와
        팔레트 사이만 막는다. KLT/팔레트 복합 강체가 마스트나 차체의 다른
        링크와 맞물리면 리프트가 과구속되므로 NVIDIA Robot Assembler의
        mask_all_collisions와 같은 범위로 베이스 아티큘레이션 전체를 필터한다.
        """
        if self._stage is None or self.art is None:
            return False
        pallet_path = self._pallet_path(pallet_id)
        pallet = self._stage.GetPrimAtPath(pallet_path)
        forklift = self._stage.GetPrimAtPath(self.art)
        if not pallet.IsValid() or not forklift.IsValid():
            return False
        api = UsdPhysics.FilteredPairsAPI.Apply(pallet)
        rel = api.CreateFilteredPairsRel()
        forklift_path = Sdf.Path(self.art)
        targets = rel.GetTargets()
        if filtered:
            if forklift_path not in targets:
                rel.AddTarget(forklift_path)
                print(
                    "[Forklift Coupler] 운반 내부충돌 차단: "
                    f"{pallet_path} <-> {self.art}"
                )
            return True
        if forklift_path in targets:
            rel.RemoveTarget(forklift_path)
            print(
                "[Forklift Coupler] 운반 내부충돌 복원: "
                f"{pallet_path} <-> {self.art}"
            )
        return True

    def _pallet_fork_collision_filtered(self, pallet_id: int) -> bool:
        if self._stage is None or self.art is None:
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
            and Sdf.Path(self.art)
            in api.GetFilteredPairsRel().GetTargets()
        )

    @staticmethod
    def _pose_matrix(position, quat_wxyz) -> Gf.Matrix4d:
        matrix = Gf.Matrix4d()
        matrix.SetRotate(
            Gf.Quatd(
                float(quat_wxyz[0]),
                Gf.Vec3d(
                    float(quat_wxyz[1]),
                    float(quat_wxyz[2]),
                    float(quat_wxyz[3]),
                ),
            )
        )
        matrix.SetTranslateOnly(
            Gf.Vec3d(*(float(value) for value in position))
        )
        return matrix

    def _measure_pickup_alignment(
        self,
        pallet_position,
        pallet_quat,
    ) -> dict:
        """실제 PhysX 자세에서 포크 날이 팔레트 채널 안인지 측정한다.

        센서 추정값이나 ROS의 하드코딩 좌표를 쓰지 않는다. 현재 지게차 root
        자세, 실제 lift_joint 위치, 팔레트 강체 자세를 같은 팔레트 로컬
        좌표계로 변환한 뒤 높이·좌우·각도·삽입 겹침을 함께 검사한다.
        """
        # FORK_TINE_*는 바깥 에셋 root 기준이고 self.robot pose는 내부
        # articulation root 기준이다. 초기 고정 변환을 거쳐야 주행 후에도
        # 실제 메시 위치와 같은 좌표가 된다.
        if (
            self._asset_root_authored_world is None
            or self._art_authored_world_inverse is None
        ):
            raise ValueError("지게차 에셋 root↔articulation 변환이 없습니다")
        art_position, art_quat = self.robot.get_world_pose()
        live_art_world = self._pose_matrix(art_position, art_quat)

        def root_point_to_world(point):
            authored_world = self._asset_root_authored_world.Transform(point)
            art_local = self._art_authored_world_inverse.Transform(
                authored_world
            )
            return live_art_world.Transform(art_local)

        pallet_world = self._pose_matrix(pallet_position, pallet_quat)
        world_to_pallet = pallet_world.GetInverse()
        lift = float(self._controller.fork_position())

        points = []
        for x_value in FORK_TINE_X:
            for y_range in FORK_TINE_Y:
                for y_value in y_range:
                    for z_value in FORK_TINE_Z_AT_ZERO:
                        root_local = Gf.Vec3d(
                            float(x_value),
                            float(y_value),
                            float(z_value + lift),
                        )
                        world_point = root_point_to_world(root_local)
                        points.append(world_to_pallet.Transform(world_point))

        values = np.asarray(
            [[float(point[0]), float(point[1]), float(point[2])]
             for point in points],
            dtype=float,
        )
        if values.shape != (16, 3) or not np.all(np.isfinite(values)):
            raise ValueError("포크 날 좌표 변환 결과가 유효하지 않습니다")

        # 포크 길이 방향이 팔레트 로컬 X/Y 중 어느 축과 평행한지 실측한다.
        near_local = world_to_pallet.Transform(
            root_point_to_world(
                Gf.Vec3d(FORK_TINE_X[1], 0.0, sum(FORK_TINE_Z_AT_ZERO) / 2 + lift)
            )
        )
        far_local = world_to_pallet.Transform(
            root_point_to_world(
                Gf.Vec3d(FORK_TINE_X[0], 0.0, sum(FORK_TINE_Z_AT_ZERO) / 2 + lift)
            )
        )
        direction = np.asarray(
            [
                float(far_local[0] - near_local[0]),
                float(far_local[1] - near_local[1]),
            ],
            dtype=float,
        )
        direction_norm = float(np.linalg.norm(direction))
        if direction_norm <= 1e-9:
            raise ValueError("포크 진행 방향을 계산할 수 없습니다")
        direction /= direction_norm
        insertion_axis = int(np.argmax(np.abs(direction)))
        lateral_axis = 1 - insertion_axis
        yaw_error = math.acos(
            max(-1.0, min(1.0, abs(float(direction[insertion_axis]))))
        )

        insertion_min = float(np.min(values[:, insertion_axis]))
        insertion_max = float(np.max(values[:, insertion_axis]))
        pallet_half = PALLET_HALF_EXTENTS[insertion_axis]
        insertion_overlap = max(
            0.0,
            min(insertion_max, pallet_half)
            - max(insertion_min, -pallet_half),
        )
        lateral_center = float(
            (np.min(values[:, lateral_axis]) + np.max(values[:, lateral_axis]))
            / 2.0
        )
        blade_bottom = float(np.min(values[:, 2]))
        blade_top = float(np.max(values[:, 2]))
        blade_center = (blade_bottom + blade_top) / 2.0
        hole_center = (PALLET_HOLE_BOTTOM_Z + PALLET_HOLE_TOP_Z) / 2.0
        z_correction = hole_center - blade_center

        lateral_ok = abs(lateral_center) <= PICKUP_MAX_LATERAL_ERROR
        yaw_ok = yaw_error <= PICKUP_MAX_YAW_ERROR
        insertion_ok = insertion_overlap >= PICKUP_MIN_INSERTION_OVERLAP
        height_ok = (
            blade_bottom >= PALLET_HOLE_BOTTOM_Z + PICKUP_VERTICAL_CLEARANCE
            and blade_top <= PALLET_HOLE_TOP_Z - PICKUP_VERTICAL_CLEARANCE
        )
        valid = lateral_ok and yaw_ok and insertion_ok and height_ok
        reasons = []
        if not lateral_ok:
            reasons.append("lateral")
        if not yaw_ok:
            reasons.append("yaw")
        if not insertion_ok:
            reasons.append("insertion")
        if not height_ok:
            reasons.append("height")
        return {
            "valid": valid,
            "reason": ",".join(reasons) if reasons else "ok",
            "lateral_error": round(lateral_center, 6),
            "yaw_error_deg": round(math.degrees(yaw_error), 4),
            "insertion_overlap": round(insertion_overlap, 6),
            "blade_bottom_z": round(blade_bottom, 6),
            "blade_top_z": round(blade_top, 6),
            "hole_bottom_z": PALLET_HOLE_BOTTOM_Z,
            "hole_top_z": PALLET_HOLE_TOP_Z,
            "z_correction": round(z_correction, 6),
            "lift_actual": round(lift, 6),
        }

    def _pickup_alignment_ready(
        self,
        pallet_position,
        pallet_quat,
    ) -> bool:
        """결합을 허용하거나, 높이만 틀렸다면 실제 측정값으로 보정한다."""
        try:
            alignment = self._measure_pickup_alignment(
                pallet_position,
                pallet_quat,
            )
        except Exception as exc:
            self._pickup_alignment = {
                "valid": False,
                "reason": f"measurement:{exc}",
            }
            print(f"[Forklift Pickup Gate] 실측 실패 — 연결 거부: {exc}")
            return False

        self._pickup_alignment = alignment
        now = time.monotonic()
        if alignment["valid"]:
            self._pickup_lift_override = None
            print(
                "[Forklift Pickup Gate] 통과: "
                f"lateral={alignment['lateral_error']:+.4f}m, "
                f"yaw={alignment['yaw_error_deg']:.2f}deg, "
                f"overlap={alignment['insertion_overlap']:.3f}m, "
                f"blade_z=({alignment['blade_bottom_z']:.4f},"
                f"{alignment['blade_top_z']:.4f})"
            )
            return True

        correction = float(alignment["z_correction"])
        # 랙 안에서는 포크 처짐 때문에 블레이드 중심이 채널 중심보다 최대
        # 15mm 낮을 수 있다. X/yaw/삽입이 맞으면 팔레트를 먼저 결속하고
        # 이후 리프트로 함께 올린다. 결속 전에 빈 포크만 올리면 팔레트를
        # 뒤로 밀어 랙에서 떨어뜨릴 수 있다.
        if (
            str(alignment["reason"]) == "height"
            and 0.0 <= correction <= 0.020
        ):
            self._pickup_lift_override = None
            print(
                "[Forklift Pickup Gate] 처짐 허용 후 선결속: "
                f"z_correction={correction:+.4f}m, "
                f"lateral={alignment['lateral_error']:+.4f}m, "
                f"yaw={alignment['yaw_error_deg']:.2f}deg"
            )
            return True
        if (
            "height" in str(alignment["reason"]).split(",")
            and abs(correction) <= 0.02
        ):
            self._pickup_lift_override = max(
                0.0,
                float(alignment["lift_actual"]) + correction,
            )

        if now - self._pickup_alignment_log_time >= 0.5:
            print(
                "[Forklift Pickup Gate] 정렬 불합격 — 연결 거부: "
                f"reason={alignment['reason']}, "
                f"lateral={alignment['lateral_error']:+.4f}m, "
                f"yaw={alignment['yaw_error_deg']:.2f}deg, "
                f"overlap={alignment['insertion_overlap']:.3f}m, "
                f"blade_z=({alignment['blade_bottom_z']:.4f},"
                f"{alignment['blade_top_z']:.4f}), "
                f"hole_z=({alignment['hole_bottom_z']:.4f},"
                f"{alignment['hole_top_z']:.4f}), "
                f"z_correction={correction:+.4f}m"
            )
            self._pickup_alignment_log_time = now
        return False

    def _deck_pallet_pickup_ready(self, pallet_id: int) -> bool:
        """DeckJoint를 풀기 전에 포크 결속 자세가 안전한지 먼저 확인한다."""
        if self._stage is None:
            return False
        pallet_path = self._pallet_path(pallet_id)
        pallet = self._stage.GetPrimAtPath(pallet_path)
        if not pallet.IsValid() or not pallet.HasAPI(UsdPhysics.RigidBodyAPI):
            print(f"[Forklift Pickup Gate] 팔레트 강체 없음: {pallet_path}")
            return False
        try:
            from isaacsim.core.prims import SingleRigidPrim

            pallet_rigid = SingleRigidPrim(
                pallet_path, name="forklift_preflight_pallet_probe"
            )
            pallet_rigid.initialize()
            pallet_position, pallet_quat = pallet_rigid.get_world_pose()
        except Exception as exc:
            print(f"[Forklift Pickup Gate] 사전 자세 측정 실패: {exc}")
            return False
        return self._pickup_alignment_ready(pallet_position, pallet_quat)

    def _set_pallet_attached(
        self,
        requested: bool,
        *,
        pallet_id: int,
    ) -> None:
        """선택한 팔레트를 포크 캐리지에 현재 자세 그대로 연결한다."""
        if self._stage is None:
            return
        pallet_path = self._pallet_path(pallet_id)
        attached_path = self._pallet_path(self._pallet_id)
        joint_exists = self._stage.GetPrimAtPath(PALLET_CARRY_JOINT).IsValid()
        if requested:
            if joint_exists:
                self._pallet_attached = True
                self._set_pallet_fork_collision_filtered(True, pallet_id)
                return
            carriage = self._fork_carriage_body()
            pallet = self._stage.GetPrimAtPath(pallet_path)
            if carriage is None:
                print("[Forklift Coupler] 포크 캐리지 rigid body를 찾지 못했습니다")
                return
            if not pallet.IsValid() or not pallet.HasAPI(UsdPhysics.RigidBodyAPI):
                print(f"[Forklift Coupler] 팔레트 강체 없음: {pallet_path}")
                return
            if not self._set_pallet_fork_collision_filtered(True, pallet_id):
                print(
                    "[Forklift Coupler] 지게차 내부충돌 차단 실패로 "
                    "팔레트 연결을 보류합니다"
                )
                return
            try:
                from isaacsim.core.prims import SingleRigidPrim

                pallet_rigid = SingleRigidPrim(
                    pallet_path, name="forklift_carry_pallet_probe"
                )
                pallet_rigid.initialize()
                pallet_position, pallet_quat = pallet_rigid.get_world_pose()
                pallet_world = self._pose_matrix(
                    pallet_position, pallet_quat
                )
            except Exception as exc:
                self._set_pallet_fork_collision_filtered(
                    False, pallet_id
                )
                print(
                    "[Forklift Coupler] 런타임 PhysX 자세 조회 실패로 "
                    f"연결을 보류합니다: {exc}"
                )
                return
            if (
                not self._deck_pickup_prevalidated
                and not self._pickup_alignment_ready(
                    pallet_position,
                    pallet_quat,
                )
            ):
                self._set_pallet_fork_collision_filtered(False, pallet_id)
                return
            # 포크 캐리지와 팔레트를 현재 상대 자세로 실제 FixedJoint 결속한다.
            # exclude_from_articulation=True 로 팔레트를 지게차 아티큘레이션
            # topology에 넣지 않으므로 리프트 drive 과구속(30kN) 없이 물리적으로
            # 함께 들린다. 조인트가 제자리에서 잠기도록 런타임 물리 자세를 넘긴다.
            try:
                carriage_rigid = SingleRigidPrim(
                    carriage, name="forklift_carriage_probe"
                )
                carriage_rigid.initialize()
                carriage_position, carriage_quat = carriage_rigid.get_world_pose()
                carriage_world = self._pose_matrix(
                    carriage_position, carriage_quat
                )
                create_fixed_joint(
                    self._stage,
                    PALLET_CARRY_JOINT,
                    carriage,
                    pallet_path,
                    body0_world=carriage_world,
                    body1_world=pallet_world,
                    exclude_from_articulation=True,
                )
            except Exception as exc:
                self._set_pallet_fork_collision_filtered(False, pallet_id)
                print(
                    "[Forklift Coupler] FixedJoint 결속 실패로 "
                    f"연결을 보류합니다: {exc}"
                )
                return
            self._carry_pallet_rigid = pallet_rigid
            self._carry_root_relative = None
            self._carry_lift_origin = float(self._controller._lift)
            self._carry_follow_reported = False
            self._carry_start_z = float(pallet_position[2])
            self._carry_pose_error = None
            self._carry_target_position = None
            self._pallet_attached = True
            self._pallet_id = pallet_id
            self._deck_pickup_prevalidated = False
            self._pickup_lift_override = None
            print(
                "[Forklift Coupler] FixedJoint 운반 결속 완료: "
                f"{carriage} <-> {pallet_path} "
                "(exclude_from_articulation, 물리 결속)"
            )
            return

        if joint_exists:
            self._stage.RemovePrim(PALLET_CARRY_JOINT)
            print(f"[Forklift Coupler] 연결 해제: {attached_path}")
        carried_path = self._pallet_path(self._pallet_id)
        carried = self._stage.GetPrimAtPath(carried_path)
        if carried.IsValid() and carried.HasAPI(UsdPhysics.RigidBodyAPI):
            try:
                if self._carry_pallet_rigid is not None:
                    self._carry_pallet_rigid.set_linear_velocity(
                        np.zeros(3, dtype=float)
                    )
                    self._carry_pallet_rigid.set_angular_velocity(
                        np.zeros(3, dtype=float)
                    )
            except Exception as exc:
                print(
                    "[Forklift Coupler] 해제 중 속도 정리 경고: "
                    f"{carried_path}: {exc}"
                )
        self._carry_pallet_rigid = None
        self._carry_root_relative = None
        self._carry_lift_origin = 0.0
        self._carry_follow_reported = False
        self._carry_start_z = None
        self._carry_pose_error = None
        self._carry_target_position = None
        self._pickup_lift_override = None
        self._pickup_alignment = None
        self._set_pallet_fork_collision_filtered(False, self._pallet_id)
        self._pallet_attached = False

    def _pallet_path(self, pallet_id: int) -> str:
        """Map logical Pallet_00 to the loaded IW cargo in integration mode."""
        warehouse_path = PALLET_PATH_FORMAT.format(pallet_id)
        if (
            self._stage is not None
            and self._stage.GetPrimAtPath(warehouse_path).IsValid()
        ):
            return warehouse_path
        if (
            pallet_id == 0
            and self._stage is not None
            and self._stage.GetPrimAtPath(INITIAL_IW_PALLET_PATH).IsValid()
        ):
            return INITIAL_IW_PALLET_PATH
        return warehouse_path

    def _set_iw_dock_locked(self, requested: bool) -> None:
        """Lock/snap the IW for handoff, or release it for navigation."""
        if requested == self._iw_dock_locked:
            # A lock request can be cancelled while the dock controller is in
            # its multi-frame stop/snap phase. Forward False so that pending
            # native-physics mutations are discarded as well.
            if not requested and self._iw_driver is not None:
                self._iw_driver.set_warehouse_dock_locked(False)
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
            # Joint 제거와 새 Joint 생성을 같은 physics frame에 하지 않는다.
            # 반복 수신되는 동일 ROS 명령이 다음 frame에 deck Joint를 만든다.
            fork_was_attached = self._pallet_attached
            self._set_pallet_attached(False, pallet_id=pallet_id)
            if fork_was_attached:
                self._iw_driver.set_warehouse_pallet_deck_collision_filtered(
                    True, pallet_id
                )
                self._deck_pallet_attached = False
                print(
                    "[Pallet Handoff] 포크 Joint 해제 완료 — "
                    "IW 내부충돌 차단 유지 후 다음 물리 프레임에 데크 추종 시작"
                )
                return
            self._iw_driver.set_warehouse_pallet_deck_collision_filtered(
                True, pallet_id
            )
            if self._iw_driver.set_warehouse_pallet_attached(True, pallet_id):
                self._deck_pallet_attached = True
                self._pallet_id = pallet_id
            return

        if fork_requested:
            if self._iw_driver is not None:
                deck_was_attached = (
                    self._deck_pallet_attached
                    or self._iw_driver.has_warehouse_pallet_attached()
                )
                # 검증 전에 DeckJoint를 풀면 불합격 시 팔레트가 어느 로봇에도
                # 연결되지 않는다. IW에 그대로 둔 채 포크 자세부터 검사한다.
                if (
                    deck_was_attached
                    and not self._deck_pallet_pickup_ready(pallet_id)
                ):
                    return
                if deck_was_attached:
                    self._deck_pickup_prevalidated = True
                if not self._iw_driver.set_warehouse_pallet_attached(
                    False,
                    pallet_id,
                ):
                    print("[Pallet Handoff] IW 데크 연결 해제에 실패했습니다")
                    return
                if deck_was_attached:
                    self._deck_pallet_attached = False
                    self._deck_release_pending = True
                    self._deck_clear_streak = 0
                    print(
                        "[Pallet Handoff] IW Deck Joint 해제 완료 — "
                        "실제 deck-clear 연속 확인 후 포크 Joint 생성"
                    )
                    return
                if self._deck_release_pending:
                    if self._iw_driver.has_warehouse_pallet_attached():
                        self._deck_clear_streak = 0
                        return
                    self._deck_clear_streak += 1
                    if self._deck_clear_streak < 3:
                        return
                    self._deck_release_pending = False
                    self._deck_clear_streak = 0
                if not self._iw_driver.set_warehouse_pallet_deck_collision_filtered(
                    True, pallet_id
                ):
                    print(
                        "[Pallet Handoff] 팔레트-IW 충돌 차단 실패로 "
                        "포크 연결을 보류합니다"
                    )
                    return
            self._deck_pallet_attached = False
            self._set_pallet_attached(True, pallet_id=pallet_id)
            return

        self._deck_release_pending = False
        self._deck_pickup_prevalidated = False
        self._deck_clear_streak = 0
        fork_was_attached = self._pallet_attached
        self._set_pallet_attached(False, pallet_id=pallet_id)
        if fork_was_attached:
            if self._iw_driver is not None:
                self._iw_driver.set_warehouse_pallet_deck_collision_filtered(
                    False, pallet_id
                )
            print(
                "[Pallet Handoff] 포크 Joint 해제 완료 — "
                "다음 물리 프레임에 Deck Joint 상태 정리"
            )
            return
        if self._iw_driver is not None:
            self._iw_driver.set_warehouse_pallet_deck_collision_filtered(
                False, pallet_id
            )
            self._iw_driver.set_warehouse_pallet_attached(False, pallet_id)
        self._deck_pallet_attached = False
