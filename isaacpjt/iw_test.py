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

# 일반 통합 실행 위치와 창고 자동화 단독 시험용 도킹 위치.
POSE = (2.0, -12.0, COMMON_FLOOR_Z)
# 창고 입구(Y=13) 밖 중앙축. 지게차 대기점 (0, 14.5)에서 월드 -Y로 2m
# 이동했을 때 포크가 insertion_depth=0.65m 들어가는 AMR 중심 좌표.
WAREHOUSE_DOCK_POSE = (0.0, 10.84885, COMMON_FLOOR_Z)


class IwDriver(Driver):
    flag = "--iw"
    name = "iw"
    ns = "iwhub_0"
    root = "/World/IwHub"

    def __init__(self, cfg, warehouse_test: bool = False):
        super().__init__()
        self._cfg = cfg
        self._warehouse_test = warehouse_test
        self._iw = IwHub(cfg.robots)
        self._warehouse_dock = None

    def spawn(self, stage):
        pose = WAREHOUSE_DOCK_POSE if self._warehouse_test else POSE
        self._iw.spawn(stage, self.root, pose)
        if self._warehouse_test:
            print("[WarehouseTest] 빈 AMR을 창고 도킹 위치에 배치했습니다")

    def finalize(self, world, stage, opts):
        # iw.hub 데크에 '적재된 세트'(팔레트+KLT 8 + 토마토 15개 꼭지포함·동적강체, 3칸 산포)
        if self._warehouse_test:
            # 첫 도킹 이벤트는 랙의 빈 팔레트를 AMR에 싣는 순서다. 기본 카고를
            # 만들면 팔레트 두 개가 겹치므로 단독 시험에서는 빈 데크로 시작한다.
            print("[WarehouseTest] 첫 상차 시험을 위해 AMR 기본 카고를 생략합니다")
            self._warehouse_dock = WarehouseDockController(
                stage, self.robot, self.art
            )
            if self._warehouse_dock.set_dock_locked(True):
                print("[WarehouseTest] AMR rigid body를 월드 FixedJoint로 고정했습니다")
        else:
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
                    _lidar_prim, rp = res             # (라이다 prim, 렌더프로덕트)
                    # 고정 마운트 TF는 iwhub_base.launch.py가 /tf_static으로 발행.
                    RB.build_lidar_scan_iw(stage, f"/World/Nav_scan_{m.name}",
                                        rp, m.scan_topic, m.frame)
    except Exception:
        import traceback
        print("\n" + "=" * 64)
        print("[Nav] 그래프 생성 실패 — 씬은 유지. tools/nav2_node_probe.py 로 노드명 확인.")
        print("=" * 64)
        traceback.print_exc()
        print("=" * 64 + "\n")
