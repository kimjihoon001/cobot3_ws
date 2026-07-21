# -*- coding: utf-8 -*-
"""수확 MM 키보드 텔레옵 — 팔 + 그리퍼 + 베이스 + **가동날(블레이드)** 을 직접 몬다.

목적: ROS2 를 붙이기 전에 "제어가 되는가"를 눈으로 본다. 그리고 이게 곧 **제어 뼈대**다 —
      입력 소스(키보드)만 나중에 ROS2 토픽으로 바꾸면 된다. 제어면(ctrl.* / mm.*)은 고정.
      (CLAUDE.md §5.6: ROS2 는 판단, Isaac 은 실행. 실행 코드는 여기 그대로 둔다.)

실행: isaac_python robots/teleop.py            (GUI 필수 — 키 입력을 받는다)

조작 (콘솔에 계속 표시). ★방향키는 Isaac 뷰포트가 카메라로 가로채므로 전부 글자키★
  팔   Q/A shoulder_pan   W/S shoulder_lift   E/D elbow
       R/F wrist_1        T/G wrist_2         Y/H wrist_3
  그리퍼   O 열기 / P 닫기
  블레이드 Z 열기(0°) / X 닫기(35°=절단)
  베이스   I/K 앞뒤   J/L 제자리 회전 (옆이동 없음)
  SPACE 현재 관절·날각 출력    ESC 종료
"""
import os
import sys

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import carb
import carb.input
import omni.appwindow
import omni.usd
from isaacsim.core.api import World
from isaacsim.core.api.robots import Robot
from isaacsim.core.utils.viewports import set_camera_view
from pxr import Usd, UsdGeom, UsdLux, UsdPhysics

from pjt_config.settings import SceneConfig
from robots.control import HarvesterController
from robots.harvester import HarvestMM

CFG = SceneConfig()
K = carb.input.KeyboardInput


def art_root(stage, under):
    for p in Usd.PrimRange(stage.GetPrimAtPath(under)):
        if p.HasAPI(UsdPhysics.ArticulationRootAPI):
            return str(p.GetPath())
    return None


def main():
    world = World(stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane()
    stage = omni.usd.get_context().get_stage()
    UsdLux.DistantLight.Define(stage, "/World/Light").CreateIntensityAttr(3000)
    UsdLux.DomeLight.Define(stage, "/World/Dome").CreateIntensityAttr(800)

    mm = HarvestMM(CFG.robots)
    mm.spawn(stage, "/World/Harvester", (0.0, 0.0, 0.0))
    robot = world.scene.add(Robot(prim_path=art_root(stage, "/World/Harvester"),
                                  name="mm"))
    world.reset()
    mm.attach_blade_hinge(stage)      # 블레이드(서보 힌지) 조립 — spawn→reset 뒤 1회
    ctrl = HarvesterController(robot)  # 팔·그리퍼·베이스 제어면

    # 카메라 — 엔드이펙터(파지점 근처)를 본다
    gb = "/World/Harvester/Gripper/Robotiq_2F_85/base_link"
    g = UsdGeom.XformCache().GetLocalToWorldTransform(
        stage.GetPrimAtPath(gb)).Transform((0, 0, CFG.robots.end_effector.grasp_reach_z))
    set_camera_view(eye=[g[0] + 0.9, g[1] - 0.9, g[2] + 0.5], target=[g[0], g[1], g[2]])

    # ---- 키보드: 눌린 키 집합 유지 ----
    pressed = set()
    state = {"run": True}

    def on_key(e, *_):
        if e.type == carb.input.KeyboardEventType.KEY_PRESS:
            pressed.add(e.input)
            if e.input == K.SPACE:
                print(f"  {ctrl.joint_report()}  blade={mm.blade_deg():.0f}deg")
            elif e.input == K.ESCAPE:
                state["run"] = False
        elif e.type == carb.input.KeyboardEventType.KEY_RELEASE:
            pressed.discard(e.input)
        return True

    app_win = omni.appwindow.get_default_app_window()
    kbd = app_win.get_keyboard()
    inp = carb.input.acquire_input_interface()
    sub = inp.subscribe_to_keyboard_events(kbd, on_key)

    # 프레임당 증분
    DA, DB, DYAW, DG, DBL = 0.010, 0.005, 0.010, 0.02, 1.0   # 팔/베이스/회전/그리퍼/날
    ARM_KEYS = [(K.Q, K.A, 0), (K.W, K.S, 1), (K.E, K.D, 2),
                (K.R, K.F, 3), (K.T, K.G, 4), (K.Y, K.H, 5)]

    print(__doc__)
    print(">>> 수확 MM 텔레옵 (Play 자동 시작). 블레이드 Z 열기 / X 닫기.\n")

    while simulation_app.is_running() and state["run"]:
        # ===== 입력 → 제어면 =====  (지금은 키보드. 나중에 ROS2 가 아래를 부른다 — ROS2 제어점 참조)
        for kp, km, i in ARM_KEYS:
            if kp in pressed: ctrl.move_arm(i, DA)
            if km in pressed: ctrl.move_arm(i, -DA)
        if K.O in pressed: ctrl.move_gripper(-DG)     # 그리퍼 열기
        if K.P in pressed: ctrl.move_gripper(DG)      # 그리퍼 닫기
        if K.Z in pressed: mm.move_blade(-DBL)        # 날 열기(→0°)
        if K.X in pressed: mm.move_blade(DBL)         # 날 닫기(→35°=절단)
        forward = (K.I in pressed) - (K.K in pressed) # 앞/뒤
        dw = (K.J in pressed) - (K.L in pressed)      # 제자리 회전
        if forward or dw:
            ctrl.move_base_forward(forward * DB, dw * DYAW)

        ctrl.apply()                                  # 제어 실행(매 프레임 고정)
        world.step(render=True)

    inp.unsubscribe_to_keyboard_events(kbd, sub)
    simulation_app.close()


# ===================== ROS2 제어점 (뼈대) =====================
# Isaac 엔 rclpy 가 없다(CLAUDE.md §2). ROS2 로 넘길 때는 위 while 루프의 '입력→제어면'
# 블록만 바꾸면 된다 — 제어면(ctrl.* / mm.*)과 ctrl.apply() 는 그대로:
#
#   토픽 수신 = OmniGraph ROS2 Subscribe 노드(ros/graph.py 패턴)로 받아, 매 프레임 폴링해
#   키보드 핸들러와 '같은 메서드'를 부른다:
#       팔     : ctrl.set_arm([q0..q5])   또는  ctrl.move_arm(i, dq)
#       그리퍼 : ctrl.set_gripper(0~1)
#       베이스 : ctrl.set_base(x, y, yaw) 또는 ctrl.move_base_forward(ds, dyaw)
#       블레이드: mm.set_blade_deg(0~35)   또는  mm.move_blade(d)
#   즉 '제어 실행'은 여기 고정, '무엇을 할지' 판단만 ROS2(dev 머신)로 간다(§5.6).
# ============================================================

if __name__ == "__main__":
    main()
