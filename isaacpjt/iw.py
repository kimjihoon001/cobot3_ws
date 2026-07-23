# -*- coding: utf-8 -*-
"""운반 AMR 드라이버 (--iw) — iw.hub (차동 구동 + 승강). 데크에 팔레트+KLT+토마토 적재.

로봇 모델은 robots/iwhub.py, ROS 브리지는 ros/robot_bridge.py. 이 파일은 배선만 한다.
Nav2 브리지(cmd_vel→바퀴 / odom / scan)는 플래그로 켠 것만, 실패해도 씬은 유지(§5.6).
"""
from __future__ import annotations

from robot_base import Driver, ros_fail
from robots.iwhub import IwHub
from scene.ground import COMMON_FLOOR_Z
from iw_dock import WarehouseDockController

# 임시 배치 — 온실 앞마당(MM 옆). 물류 동선 확정 후 조정.
POSE = (2.0, -12.0, COMMON_FLOOR_Z)


class IwDriver(Driver):
    flag = "--iw"
    name = "iw"
    ns = "iwhub_0"
    root = "/World/IwHub"

    def __init__(self, cfg):
        super().__init__()
        self._cfg = cfg
        self._iw = IwHub(cfg.robots)
        self._warehouse_dock = None

    def spawn(self, stage):
        self._iw.spawn(stage, self.root, POSE)

    def finalize(self, world, stage, opts):
        # iw.hub 데크에 '적재된 세트'(팔레트+KLT 8 + 토마토 15개 꼭지포함·동적강체, 3칸 산포)
        self._iw.load_cargo(stage, self._cfg.tomato_assets, self._cfg.physics)
        self._warehouse_dock = WarehouseDockController(
            stage, self.robot, self.art
        )

        if not opts.no_ros:
            try:
                from ros import robot_bridge as RB
                RB.build_joint_bridge(stage, f"/World/RosBridge_{self.ns}",
                                      self.ns, self.art)
            except Exception:
                ros_fail("iw.hub 조인트 브리지")
            if opts.nav_drive or opts.nav_odom or opts.nav_scan:
                build_nav(stage, self._iw, self.art,
                          self._cfg.robots.iwhub_nav, opts)

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


def build_nav(stage, iw, art_path: str, nav, opts) -> None:
    """iw.hub 자율주행 그래프 — 플래그로 켠 것만 배선. 실패해도 씬은 유지(브리지와 동일 방침).

    순서대로 GPU 검증: drive(/cmd_vel→바퀴) → odom(/odom+TF) → scan(라이다→/scan).
    노드 타입명은 tools/nav2_node_probe.py 로 확정 후 robot_bridge.T 갱신할 것(§8).
    """
    from ros import robot_bridge as RB

    base = f"{iw.root}/chassis"          # 움직이는 섀시 링크(정지 컨테이너 아님) — TF·라이다 부모
    chassis = base if stage.GetPrimAtPath(base).IsValid() else art_path
    try:
        if opts.nav_drive:
            RB.build_diff_drive(stage, "/World/Nav_drive", art_path,
                                iw.DRIVE_JOINTS, nav)
        if opts.nav_odom:
            RB.build_odometry(stage, "/World/Nav_odom", chassis, nav)
        if opts.nav_scan:
            for m in nav.lidars:                      # 앞/뒤 라이다 각 1기 → /scan + TF
                res = iw.attach_lidar(stage, m)
                if res:
                    lidar_prim, rp = res              # (라이다 prim, 렌더프로덕트)
                    RB.build_tf_sensor_iw(stage, f"/World/Nav_tf_{m.name}",
                                       chassis, lidar_prim, nav, m.frame)
                    RB.build_lidar_scan_iw(stage, f"/World/Nav_scan_{m.name}",
                                        rp, m.scan_topic, m.frame)
    except Exception:
        import traceback
        print("\n" + "=" * 64)
        print("[Nav] 그래프 생성 실패 — 씬은 유지. tools/nav2_node_probe.py 로 노드명 확인.")
        print("=" * 64)
        traceback.print_exc()
        print("=" * 64 + "\n")
