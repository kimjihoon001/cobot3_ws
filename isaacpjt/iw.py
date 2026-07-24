# -*- coding: utf-8 -*-
"""운반 AMR 드라이버 (--iw) — iw.hub 차동구동, 적재물, Nav2 센서 브리지.

통합 모드에서 판단과 경로 계획은 ROS2 IW 전용 Nav2가 담당한다.
Isaac은 /iwhub_0/joint_command를 실제 관절에 적용하고, odom/TF/scan을 발행한다.
"""
from __future__ import annotations

from robot_base import Driver, ros_fail
from robots.iwhub import IwHub
from scene.ground import COMMON_FLOOR_Z
from iw_dock import WarehouseDockController

# [2] Ridgeback 0.96m, IW 1.431m, 차체 사이 빈 공간 0.50m:
# 중심거리 = 0.96/2 + 0.50 + 1.431/2 = 1.6955m.
POSE = (1.6955, -12.0, COMMON_FLOOR_Z)
SPAWN_YAW_DEG = 180.0


class IwDriver(Driver):
    flag = "--iw"
    name = "iw"
    ns = "iwhub_0"
    root = "/World/IwHub"

    def __init__(self, cfg):
        super().__init__()
        self._cfg = cfg
        self._iw = IwHub(cfg.robots)
        self._stage = None
        self._warehouse_dock = None
        self._deck_geometry_pub = None
        self._basket_pose_pub = None
        self._last_deck_geometry = None
        self._deck_geometry_error_logged = False

    def spawn(self, stage):
        self._iw.spawn(stage, self.root, POSE, yaw_deg=SPAWN_YAW_DEG)

    def finalize(self, world, stage, opts):
        self._stage = stage
        self._iw.load_cargo(
            stage, self._cfg.tomato_assets, self._cfg.physics)
        self._warehouse_dock = WarehouseDockController(
            stage, self.robot, self.art
        )

        if opts.no_ros:
            return
        try:
            from ros import robot_bridge as RB
            RB.build_joint_bridge(
                stage, f"/World/RosBridge_{self.ns}",
                self.ns, self.art)
            geometry_node = RB.build_string_pub(
                "/World/RosDeckGeometry_iwhub_0",
                "/iwhub_0/deck_geometry",
            )
            self._deck_geometry_pub = RB.StringPublisher(geometry_node)
            basket_node = RB.build_pose_publisher(
                "/World/RosIwEmptyBasketPose",
                "/iw/basket/empty_slot_pose",
            )
            self._basket_pose_pub = RB.PosePublisher(basket_node)
        except Exception:
            ros_fail("iw.hub 조인트 브리지")

        if opts.nav_odom or opts.nav_scan:
            build_nav_sensors(
                stage, self._iw, self.art,
                self._cfg.robots.iwhub_nav, opts)

    def update(self, is_playing: bool):
        """실측 데크 높이를 ROS 지게차 제어기에 계속 제공한다."""
        if (
            not is_playing
            or self._warehouse_dock is None
            or self._deck_geometry_pub is None
        ):
            return
        try:
            # deck geometry는 이 실행 동안 불변이다. 동적 articulation을
            # 주행시키는 동안 매 frame BBoxCache로 Fabric을 읽지 않는다.
            if self._last_deck_geometry is None:
                payload = self._warehouse_dock.geometry_json()
                if self._deck_geometry_pub.publish(payload):
                    self._last_deck_geometry = payload
                    print(f"[IW Deck Measure] ROS 발행 시작: {payload}")
            self._deck_geometry_error_logged = False
            self._publish_empty_basket_pose()
        except Exception as exc:
            if not self._deck_geometry_error_logged:
                print(f"[IW Deck Measure] ROS 발행 실패: {exc}")
                self._deck_geometry_error_logged = True

    def _publish_empty_basket_pose(self) -> None:
        """IW의 실제 빈 KLT prim에서 map 기준 tool release pose를 발행한다."""
        if self._basket_pose_pub is None or self._stage is None:
            return
        stage = self._stage
        # load_cargo()의 초기 적재 슬롯 (00,11,30)은 제외한다.
        for slot in ("KLT_01", "KLT_10", "KLT_20", "KLT_21", "KLT_31"):
            prim = stage.GetPrimAtPath(f"/World/IwHubCargo/Load/{slot}")
            if not prim.IsValid():
                continue
            from pxr import Gf, Usd, UsdGeom
            world = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(
                Usd.TimeCode.Default())
            # KLT 높이 0.146m × scale 0.85의 윗면보다 5cm 위에서 놓는다.
            release = world.Transform(Gf.Vec3d(0.0, 0.0, 0.11205))
            quat = world.ExtractRotationQuat().GetNormalized()
            imaginary = quat.GetImaginary()
            self._basket_pose_pub.publish(
                (float(release[0]), float(release[1]), float(release[2])),
                (float(imaginary[0]), float(imaginary[1]),
                 float(imaginary[2]), float(quat.GetReal())),
                frame_id="map",
            )
            return

    def set_warehouse_dock_locked(self, locked: bool) -> bool:
        return bool(
            self._warehouse_dock
            and self._warehouse_dock.set_dock_locked(locked)
        )

    def set_warehouse_pallet_attached(
        self, attached: bool, pallet_id: int
    ) -> bool:
        return bool(
            self._warehouse_dock
            and self._warehouse_dock.set_pallet_on_deck(attached, pallet_id)
        )


def build_nav_sensors(stage, iw, art_path: str, nav, opts) -> None:
    """IW Nav2 입력만 배선한다. /cmd_vel 실행은 ROS base_node 한 곳에서 담당한다."""
    from ros import robot_bridge as RB

    chassis = f"{iw.root}/chassis"
    if not stage.GetPrimAtPath(chassis).IsValid():
        chassis = art_path
    try:
        if opts.nav_odom:
            RB.build_odometry(
                stage, "/World/IwNav_odom", chassis, nav)
        if opts.nav_scan:
            for mount in nav.lidars:
                result = iw.attach_lidar(stage, mount)
                if result:
                    _lidar_prim, render_product = result
                    RB.build_lidar_scan_iw(
                        stage, f"/World/IwNav_scan_{mount.name}",
                        render_product, mount.scan_topic, mount.frame)
        print("[IW] Nav2 실행 경로: odom/scan → Nav2 → base_node → joint_command")
    except Exception:
        import traceback
        print("[IW Nav2] odom/scan 그래프 생성 실패")
        traceback.print_exc()
