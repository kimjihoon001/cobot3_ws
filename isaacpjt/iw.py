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
IW_PALLET_PATH = "/World/IwHubCargo/Pallet_00"
# robots/iwhub.py가 사전 적재에서 비워두는 데크 앞열(IW 코=+x, MM 쪽) 2칸.
# 나머지 6칸은 이미 토마토가 차 있으므로 MM의 release 목표가 될 수 없다.
MM_PLACE_SLOTS = ((3, 0), (3, 1))
# MM이 스쿱을 열어 과실을 놓을 때마다 mm.py가 발행하는 디버그 토픽.
# 이 이벤트를 세어 다음 플레이스에는 아직 안 쓴 칸을 내준다.
MM_RELEASE_TOPIC = "/harvester_0/scoop/release_debug"


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
        self._last_basket_slot = None
        self._mm_release_poller = None
        self._used_basket_slots: set[tuple[int, int]] = set()
        self._deck_pallet_id: int | None = None

    def spawn(self, stage):
        self._iw.spawn(stage, self.root, POSE, yaw_deg=SPAWN_YAW_DEG)

    def finalize(self, world, stage, opts):
        self._stage = stage
        self._iw.load_cargo(
            stage,
            self._cfg.tomato_assets,
            self._cfg.physics,
            pallet_phys_cfg=self._cfg.warehouse.pallet_physics,
        )
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
            release_node = RB.build_string_sub(
                "/World/RosMmScoopRelease",
                MM_RELEASE_TOPIC,
            )
            self._mm_release_poller = RB.StringPoller(release_node)
        except Exception:
            ros_fail("iw.hub 조인트 브리지")

        if opts.nav_odom or opts.nav_scan:
            build_nav_sensors(
                stage, self._iw, self.art,
                self._cfg.robots.iwhub_nav, opts)

    def update(self, is_playing: bool):
        """실측 데크 높이를 ROS 지게차 제어기에 계속 제공한다."""
        if is_playing and self._warehouse_dock is not None:
            self._warehouse_dock.update()
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

    def _refresh_deck_pallet(self) -> str | None:
        """데크에 결속된 팔레트를 확인하고, 바뀌었으면 슬롯 기록을 초기화한다.

        하역 후 지게차가 빈 팔레트를 새로 얹으면 그 팔레트의 KLT는 다시 비어
        있다. 초기화 기준은 시간이 아니라 WarehouseDockController가 추적하는
        '데크에 결속된 팔레트 ID' 변화다 — 실제로 새 팔레트가 올라온 시점.
        """
        if self._warehouse_dock is None:
            return IW_PALLET_PATH
        pallet_id = self._warehouse_dock.deck_pallet_id
        if pallet_id != self._deck_pallet_id:
            self._deck_pallet_id = pallet_id
            if pallet_id is None:
                print("[IW Basket] 데크 팔레트 내림 — 빈 슬롯 발행 중단")
            else:
                self._used_basket_slots.clear()
                self._last_basket_slot = None
                print(f"[IW Basket] 데크에 Pallet_{pallet_id:02d} 결속 — "
                      "슬롯 사용 기록 초기화")
        if pallet_id is None:
            return None
        return self._warehouse_dock.deck_pallet_path()

    def _available_place_slots(self) -> tuple[tuple[int, int], ...]:
        """MM이 아직 쓰지 않은 앞열 빈 슬롯."""
        return tuple(
            slot for slot in MM_PLACE_SLOTS
            if slot not in self._used_basket_slots
        )

    def _consume_mm_release_events(self) -> None:
        """MM이 과실을 놓을 때마다 그때 내주던 칸을 '사용됨'으로 넘긴다."""
        if self._mm_release_poller is None:
            return
        if self._mm_release_poller.poll() is None:
            return
        if self._last_basket_slot is None:
            return
        if self._last_basket_slot in self._used_basket_slots:
            return
        self._used_basket_slots.add(self._last_basket_slot)
        ix, iy = self._last_basket_slot
        remaining = len(MM_PLACE_SLOTS) - len(self._used_basket_slots)
        print(f"[IW Basket] KLT_{ix}{iy} 적재 완료 — 남은 빈 칸 {remaining}개")

    def _publish_empty_basket_pose(self) -> None:
        """아직 안 쓴 앞열 빈 KLT 중 MM에 가장 가까운 슬롯의 release pose를 발행한다."""
        if self._basket_pose_pub is None or self._stage is None:
            return
        pallet_path = self._refresh_deck_pallet()
        if pallet_path is None:
            return
        self._consume_mm_release_events()
        from pxr import Gf, Usd, UsdGeom

        stage = self._stage
        # ★MM 루트 Xform(/World/Harvester)은 스폰 자세에 고정돼 있다. MM은 dummy
        #   base 조인트로 움직이므로 실제로 움직이는 강체는 Base/base_link다.
        #   루트를 쓰면 스폰점(0,-12) 기준 최근접 슬롯 = 실제 MM 기준 최원거리
        #   슬롯을 고르게 된다(2026-07-25 sim_diag_152127 실측 2.30m vs 1.34m).
        harvester = stage.GetPrimAtPath("/World/Harvester/Base/base_link")
        if not harvester.IsValid():
            harvester = stage.GetPrimAtPath("/World/Harvester")
            if not harvester.IsValid():
                return
        harvester_world = UsdGeom.Xformable(
            harvester).ComputeLocalToWorldTransform(
                Usd.TimeCode.Default())
        harvester_position = harvester_world.ExtractTranslation()

        candidates = []
        for ix, iy in self._available_place_slots():
            prim = stage.GetPrimAtPath(
                f"{pallet_path}/KLT_{ix}{iy}")
            if not prim.IsValid():
                continue
            world = UsdGeom.Xformable(
                prim).ComputeLocalToWorldTransform(
                    Usd.TimeCode.Default())
            center = world.ExtractTranslation()
            distance_xy = (
                (float(center[0]) - float(harvester_position[0])) ** 2
                + (float(center[1]) - float(harvester_position[1])) ** 2
            )
            candidates.append((distance_xy, world, ix, iy))
        if not candidates:
            # 앞열 두 칸을 다 채웠다 — 더 내줄 빈 칸이 없으니 발행을 멈춘다.
            return

        # 아직 안 쓴 빈 칸 중 MM에 가장 가까운 칸을 release 목표로 쓴다.
        _, world, ix, iy = min(candidates, key=lambda candidate: candidate[0])
        # 어느 칸을 골랐고 그 칸 원점이 맵 어디인지 한 번만 남긴다. 발행 XY가
        # KLT 중앙인지 확인하려면 이 값과 팔레트 격자(pitch 0.31/0.25)를 대조한다.
        chosen = (ix, iy)
        if chosen != self._last_basket_slot:
            self._last_basket_slot = chosen
            origin = world.ExtractTranslation()
            print(f"[IW Basket] KLT_{ix}{iy} 선택 — prim 원점 map="
                  f"({float(origin[0]):.4f}, {float(origin[1]):.4f}, "
                  f"{float(origin[2]):.4f})")
        # KLT 높이 0.146m × scale 0.85의 윗면보다 약 5cm 위.
        release = world.Transform(Gf.Vec3d(0.0, 0.0, 0.11205))
        quat = world.ExtractRotationQuat().GetNormalized()
        imaginary = quat.GetImaginary()
        self._basket_pose_pub.publish(
            (float(release[0]), float(release[1]), float(release[2])),
            (
                float(imaginary[0]),
                float(imaginary[1]),
                float(imaginary[2]),
                float(quat.GetReal()),
            ),
            frame_id="map",
        )

    def set_warehouse_dock_locked(self, locked: bool) -> bool:
        return bool(
            self._warehouse_dock
            and self._warehouse_dock.set_dock_locked(locked)
        )

    def set_warehouse_pallet_attached(
        self,
        attached: bool,
        pallet_id: int,
        forward_offset: float = 0.0,
    ) -> bool:
        return bool(
            self._warehouse_dock
            and self._warehouse_dock.set_pallet_on_deck(
                attached, pallet_id, forward_offset
            )
        )

    def warehouse_pallet_min_z(self, pallet_path: str) -> float | None:
        if self._warehouse_dock is None:
            return None
        return self._warehouse_dock.pallet_world_min_z(pallet_path)

    def warehouse_deck_surface(self, forward_offset: float = 0.0):
        if self._warehouse_dock is None:
            raise ValueError("IW warehouse dock controller가 없습니다")
        return self._warehouse_dock._deck_surface(forward_offset)

    def has_warehouse_pallet_attached(self) -> bool:
        return bool(
            self._warehouse_dock
            and self._warehouse_dock.pallet_on_deck
        )

    def set_warehouse_pallet_deck_collision_filtered(
        self, filtered: bool, pallet_id: int
    ) -> bool:
        return bool(
            self._warehouse_dock
            and self._warehouse_dock.set_pallet_deck_collision_filtered(
                filtered, pallet_id
            )
        )

    def warehouse_pallet_deck_collision_filtered(
        self, pallet_id: int
    ) -> bool:
        return bool(
            self._warehouse_dock
            and self._warehouse_dock.pallet_deck_collision_filtered(pallet_id)
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
