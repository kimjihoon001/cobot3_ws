# -*- coding: utf-8 -*-
"""수확 MM 드라이버 (--mm) — Ridgeback + UR10e + 2F-85 + 커터지그 + 가동날 + D455.

로봇 모델은 robots/harvester.py, ROS 브리지는 ros/robot_bridge.py. 이 파일은 그 둘을
'배선'하고 텔레옵/JSON 명령을 매 프레임 적용한다(§5.6 실행측). 판단은 ROS2 가 한다.
"""
from __future__ import annotations

import json

import numpy as np

from robot_base import Driver, ros_fail
from robots.harvester import HarvestMM

# 임시 배치 — 온실 앞마당(빈 홀 바닥, 온실 y −10 앞). 물류 동선 확정 후 조정.
POSE = (0.0, -12.0, 0.0)
# 키네마틱 베이스 조인트 (JSON base 명령용 — 텔레포트만 먹는다, 2026-07-18 실측)
BASE_JOINTS = ("dummy_base_prismatic_x_joint",
               "dummy_base_prismatic_y_joint",
               "dummy_base_revolute_z_joint")


class MMDriver(Driver):
    flag = "--mm"
    name = "mm"
    ns = "harvester_0"
    root = "/World/Harvester"

    def __init__(self, cfg):
        super().__init__()
        self._cfg = cfg
        self._mm = HarvestMM(cfg.robots)
        self._base_idx = None
        self._poller = None
        self._teleop = None

    def spawn(self, stage):
        self._mm.spawn(stage, self.root, POSE)

    def configure(self, world):
        # 수확자세: wrist_1(4번축) +180° 를 스폰 기본자세로.
        # 기본자세면 커터·지그가 파지점 아래(뒤집힘). +180° 라야 절단점이 파지점 위 5.3cm
        # (2026-07-19 실측 — CAD 의도 그대로). default_state 라 Play/Stop 리셋에도 유지.
        r = self.robot
        q0 = np.asarray(r.get_joint_positions(), dtype=float)
        q0[list(r.dof_names).index("wrist_1_joint")] += np.pi
        r.set_joints_default_state(positions=q0)

    def finalize(self, world, stage, opts):
        # 가동날(서보 힌지) — main 이 configure 뒤 reset+settle 했으므로 정착된 자세 기준.
        # 힌지 강체·조인트는 이 뒤 main 의 reset 에서 물리 뷰에 올라가 미리 정착한다(§8).
        self._mm.attach_blade_hinge(stage)
        # 키네마틱 베이스 인덱스 (JSON base 명령용)
        r = self.robot
        self._base_idx = np.array(
            [list(r.dof_names).index(n) for n in BASE_JOINTS])

        if not opts.no_ros:
            try:
                from ros import robot_bridge as RB
                RB.build_joint_bridge(stage, f"/World/RosBridge_{self.ns}",
                                      self.ns, self.art)
                sub = RB.build_string_sub(
                    f"/World/RosCmd_{self.ns}", f"/{self.ns}/cmd")
                self._poller = RB.StringPoller(sub)
            except Exception:
                ros_fail("MM 조인트/명령 브리지")
            if opts.camera:
                self._build_camera(stage)

        if opts.teleop:
            self._teleop = build_teleop(r, self._mm.set_blade_deg, opts.gui)

    def _build_camera(self, stage):
        cam_prim = self._mm.camera_path(stage)
        if not cam_prim:
            print("[Camera] D455 카메라 prim 못 찾음 — rgb/depth 발행 스킵")
            return
        try:
            from ros import robot_bridge as RB
            RB.build_camera(stage, "/World/RosCamera", cam_prim,
                            self._cfg.robots.camera)
        except Exception:
            import traceback
            print("\n[Camera] 그래프 생성 실패 — 씬 유지. probe 로 노드명 확인.")
            traceback.print_exc()

    def update(self, is_playing):
        if self._teleop is not None:                 # 키보드 텔레옵 (재생 중에만 적용)
            self._teleop(is_playing)
        # MM JSON 명령 (블레이드·베이스) — 재생 중에만 적용
        if is_playing and self._poller is not None:
            raw = self._poller.poll()
            if raw:
                try:
                    cmd = json.loads(raw)
                except ValueError:
                    cmd = None
                if isinstance(cmd, dict):
                    if "blade" in cmd:
                        self._mm.set_blade_deg(float(cmd["blade"]))
                    if "base" in cmd:
                        b = [float(v) for v in cmd["base"]]
                        if len(b) == 3:
                            self.robot.set_joint_positions(
                                np.array(b), joint_indices=self._base_idx)


def build_teleop(mm_robot, set_blade, gui: bool):
    """MM 키보드 텔레옵 — 팔6·베이스·그리퍼·블레이드. 반환: step(is_playing) 콜백(실패 시 None).

    글자키만 쓴다(방향키는 뷰포트가 가로챔 — spike05 실측). GUI 전용. mm_robot 이 물리
    초기화(world.reset)된 뒤 호출할 것 — HarvesterController 가 현재 관절값에서 출발한다.
    set_blade: 가동날 각도[deg] setter 콜백 (조립모드=mm.set_blade_deg / 로드모드=드라이브 attr).
    """
    if not gui:
        print("[Teleop] --headless 라 키보드 입력 불가 — 텔레옵 비활성")
        return None
    import carb.input
    import omni.appwindow

    from robots.control import HarvesterController

    ctrl = HarvesterController(mm_robot)
    K = carb.input.KeyboardInput
    pressed: set = set()
    st = {"blade": 0.0}
    active = {"joint": 0}                           # 번호키로 선택된 팔 관절(0~5)
    ARM_SEL = [K.KEY_1, K.KEY_2, K.KEY_3, K.KEY_4, K.KEY_5, K.KEY_6]

    def on_key(e, *_):
        if e.type == carb.input.KeyboardEventType.KEY_PRESS:
            if e.input in ARM_SEL:                   # 번호키 = 조작할 관절 선택(엣지)
                active["joint"] = ARM_SEL.index(e.input)
                print(f"[Teleop] 활성 관절 = {active['joint'] + 1}번")
            else:
                pressed.add(e.input)
        elif e.type == carb.input.KeyboardEventType.KEY_RELEASE:
            pressed.discard(e.input)
        return True

    appwin = omni.appwindow.get_default_app_window()
    carb.input.acquire_input_interface().subscribe_to_keyboard_events(
        appwin.get_keyboard(), on_key)

    DQ, DB, DYAW, DG, DBL = 0.02, 0.01, 0.02, 0.03, 2.0
    print("""
[MM 텔레옵] 플레이 상태에서 (방향키는 뷰포트가 가로챔 — 숫자/글자키만)
  팔    숫자 1~6 으로 관절 선택 → , 반시계 / . 시계 로 그 관절 회전
  베이스 I/K 전후 · J/L 좌우 · U/O 회전
  그리퍼 Z 열기 / X 닫기      블레이드 B 열기(0°) / N 닫기(절단)
""")

    def step(is_playing):
        if not is_playing:
            return
        j = active["joint"]                         # 선택된 관절만 회전
        if K.COMMA in pressed:                       # , = 반시계(CCW, +)
            ctrl.move_arm(j, DQ)
        if K.PERIOD in pressed:                      # . = 시계(CW, −)
            ctrl.move_arm(j, -DQ)
        dx = (K.I in pressed) - (K.K in pressed)
        dy = (K.J in pressed) - (K.L in pressed)
        dyaw = (K.U in pressed) - (K.O in pressed)
        if dx or dy or dyaw:
            ctrl.move_base(dx * DB, dy * DB, dyaw * DYAW)
        if K.Z in pressed:
            ctrl.move_gripper(-DG)
        if K.X in pressed:
            ctrl.move_gripper(DG)
        if K.B in pressed or K.N in pressed:
            st["blade"] = max(0.0, min(35.0,
                              st["blade"] + (DBL if K.N in pressed else -DBL)))
            set_blade(st["blade"])
        ctrl.apply()

    return step


def find_blade_setter(stage):
    """로드된 USD 의 가동날 ServoJoint 드라이브 타깃 attr → 각도 setter. 없으면 no-op."""
    from pxr import UsdPhysics
    sj = stage.GetPrimAtPath("/World/Harvester_CutterBlade/ServoJoint")
    if sj and sj.IsValid():
        attr = UsdPhysics.DriveAPI(sj, "angular").GetTargetPositionAttr()
        if attr and attr.IsValid():
            return lambda deg: attr.Set(float(deg))
    return lambda deg: None
