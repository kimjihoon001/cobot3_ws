# -*- coding: utf-8 -*-
"""운반 AMR 드라이버 (--iw) — iw.hub (차동 구동 + 승강). 데크에 팔레트+KLT+토마토 적재.

로봇 모델은 robots/iwhub.py, ROS 브리지는 ros/robot_bridge.py. 이 파일은 배선만 한다.
Nav2 브리지(cmd_vel→바퀴 / odom / scan)는 플래그로 켠 것만, 실패해도 씬은 유지(§5.6).

MM 연동(2026-07-23, 1차·단순): Isaac 실좌표로 MM 뒤를 텔레포트 추종하고, 만재 신호를
받으면 지게차 앞으로 이동한다. nav2 없이 ground-truth 로만 움직인다(사용자 선택).
  구독 /iw/mission (std_msgs/String): "FOLLOW"(기본) / "FORKLIFT"
  발행 /iw/status  (std_msgs/String): 지게차 도착 시 "ARRIVED_FORKLIFT"
"""
from __future__ import annotations

import math

import numpy as np

from robot_base import Driver, ros_fail
from robots.iwhub import IwHub
from scene.ground import COMMON_FLOOR_Z

# 임시 배치 — 온실 앞마당(MM 옆). 물류 동선 확정 후 조정.
POSE = (2.0, -12.0, COMMON_FLOOR_Z)

# [4] 임의 — 시뮬에서 맞출 튜닝값 (사용자: 완벽 아닌 단순 1차).
# iw 베이스의 MM 로컬프레임 오프셋(m). MM +X=전방. 팔이 iw 데크에 닿아야 하므로
# MM 작업영역 안(옆/앞)에 둔다. basket pose(놓는 위치)와 이 값을 함께 맞춘다.
MM_FOLLOW_OFFSET = (0.0, -0.85)     # MM 오른쪽 0.85m 나란히
FORKLIFT_DOCK_XY = (0.0, 13.2)      # 지게차(0,14.5) 앞 도킹 위치
FORKLIFT_DOCK_YAW = math.pi / 2.0   # 지게차를 바라보는 방향(+Y)
DRIVE_STEP_M = 0.03                 # FORKLIFT 이동 프레임당 전진(≈1.8m/s @60fps)
ARRIVE_TOL_M = 0.30                 # 도킹 도착 판정 반경


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
        self._mission = "FOLLOW"
        self._mission_poller = None
        self._status_pub = None
        self._arrived_sent = False

    def spawn(self, stage):
        self._iw.spawn(stage, self.root, POSE)

    def finalize(self, world, stage, opts):
        self._stage = stage
        # iw.hub 데크에 '적재된 세트'(팔레트+KLT 8 + 토마토 15개 꼭지포함·동적강체, 3칸 산포)
        self._iw.load_cargo(stage, self._cfg.tomato_assets, self._cfg.physics)

        if not opts.no_ros:
            try:
                from ros import robot_bridge as RB
                RB.build_joint_bridge(stage, f"/World/RosBridge_{self.ns}",
                                      self.ns, self.art)
                # MM 연동 미션/상태 토픽 (ground-truth 추종·도킹 제어).
                sub = RB.build_string_sub("/World/RosIwMission", "/iw/mission")
                self._mission_poller = RB.StringPoller(sub)
                pub = RB.build_string_pub("/World/RosIwStatus", "/iw/status")
                self._status_pub = RB.StringPublisher(pub)
            except Exception:
                ros_fail("iw.hub 조인트/미션 브리지")
            if opts.nav_drive or opts.nav_odom or opts.nav_scan:
                build_nav(stage, self._iw, self.art,
                          self._cfg.robots.iwhub_nav, opts)

    # ----- MM 연동: ground-truth 추종 / 지게차 도킹 -----

    def update(self, is_playing: bool) -> None:
        if not is_playing or self.robot is None or self._stage is None:
            return
        if self._mission_poller is not None:
            m = self._mission_poller.poll()
            if m:
                self._mission = m.strip().upper()
                self._arrived_sent = False
                print(f"[IW] 미션 수신: {self._mission}")
        if self._mission == "FORKLIFT":
            self._drive_to(FORKLIFT_DOCK_XY, FORKLIFT_DOCK_YAW)
        else:
            self._follow_mm()

    def _current_pose(self):
        """iw 아티큘레이션 루트 (x, y, z, yaw)."""
        pos, quat = self.robot.get_world_pose()
        pos = np.asarray(pos, dtype=float)
        w, x, y, z = (float(quat[0]), float(quat[1]),
                      float(quat[2]), float(quat[3]))
        yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        return pos, yaw

    def _mm_pose(self):
        """MM 베이스 (x, y, yaw) 월드 — ground truth."""
        from pxr import UsdGeom
        prim = self._stage.GetPrimAtPath("/World/Harvester/Base/base_link")
        if not prim.IsValid():
            return None
        m = UsdGeom.XformCache().GetLocalToWorldTransform(prim)
        t = m.ExtractTranslation()
        q = m.ExtractRotationQuat()
        imag = q.GetImaginary()
        w, z = float(q.GetReal()), float(imag[2])
        x, y = float(imag[0]), float(imag[1])
        yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        return np.array([float(t[0]), float(t[1])]), yaw

    def _set_base(self, x: float, y: float, yaw: float) -> None:
        pos, _ = self.robot.get_world_pose()
        z = float(pos[2])                       # 현재 높이 유지(지면)
        quat = np.array([math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0)])
        self.robot.set_world_pose(position=np.array([x, y, z], dtype=float),
                                  orientation=quat)

    def _follow_mm(self) -> None:
        res = self._mm_pose()
        if res is None:
            return
        (mx, my), myaw = res
        c, s = math.cos(myaw), math.sin(myaw)
        ox, oy = MM_FOLLOW_OFFSET
        tx = mx + ox * c - oy * s               # MM 로컬 → 월드
        ty = my + ox * s + oy * c
        self._set_base(tx, ty, myaw)

    def _drive_to(self, target_xy, yaw: float) -> None:
        (cx, cy), _ = self._current_pose()
        d = np.array([target_xy[0] - cx, target_xy[1] - cy])
        dist = float(np.linalg.norm(d))
        if dist <= ARRIVE_TOL_M:
            self._set_base(target_xy[0], target_xy[1], yaw)
            if not self._arrived_sent and self._status_pub is not None:
                self._status_pub.publish("ARRIVED_FORKLIFT")
                self._arrived_sent = True
                print("[IW] 지게차 도킹 완료 → /iw/status ARRIVED_FORKLIFT")
            return
        step = min(dist, DRIVE_STEP_M)
        nx, ny = np.array([cx, cy]) + d / dist * step
        self._set_base(float(nx), float(ny), yaw)


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
