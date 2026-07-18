# -*- coding: utf-8 -*-
"""스파이크 5 — Isaac 안에서 로봇 2대를 키보드로 직접 몬다.

목적: ROS2 없이 먼저 "제어가 되는가"를 눈으로 본다. 팔이 마커에 닿는지, 포크가
      2단까지 오르는지, 베이스가 통로를 통과하는지. ROS2 는 나중에 robots/control.py
      의 set_* 에 값만 흘려보내면 된다 (CLAUDE.md §5.6: ROS2 판단 / Isaac 실행).

실행: isaac_python spikes/05_teleop.py            (GUI 필수 — 키 입력을 받는다)

조작 (콘솔에 계속 표시). ★방향키는 Isaac 뷰포트가 카메라로 가로채므로 전부 글자키★
  [1] 수확 MM   [2] 지게차 B(오더피커—운전석도 승강)   [3] 지게차 C(카운터밸런스)
  수확 MM:
    팔  Q/A shoulder_pan   W/S shoulder_lift   E/D elbow
        R/F wrist_1        T/G wrist_2         Y/H wrist_3
    그리퍼  O 열기 / P 닫기
    베이스(모바일)  I/K 앞뒤   J/L 좌우(옆으로도 감—홀로노믹)   U/M 회전
  지게차 B/C 공통:
    I/K 전진/후진   J/L 조향   U 포크↑ / M 포크↓
  SPACE 현재 관절값 출력    ESC 종료
"""
import sys

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import os

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import carb
import carb.input
import omni.appwindow
import omni.usd
from isaacsim.core.api import World
from isaacsim.core.api.robots import Robot
from pxr import Gf, Usd, UsdGeom, UsdLux, UsdPhysics

from pjt_config.settings import SceneConfig
from robots.control import HarvesterController, TransporterController
from robots.harvester import HarvestMM
from robots.transporter import TransporterAMR

CFG = SceneConfig()
K = carb.input.KeyboardInput           # 키 열거 별칭


def art_root(stage, under):
    for p in Usd.PrimRange(stage.GetPrimAtPath(under)):
        if p.HasAPI(UsdPhysics.ArticulationRootAPI):
            return str(p.GetPath())
    return None


def add_marker(stage, path, pos, color):
    s = UsdGeom.Sphere.Define(stage, path)
    s.CreateRadiusAttr(0.0344)
    s.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    UsdGeom.Xformable(s.GetPrim()).AddTranslateOp().Set(Gf.Vec3d(*pos))


def main():
    world = World(stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane()
    stage = omni.usd.get_context().get_stage()
    UsdLux.DistantLight.Define(stage, "/World/Light").CreateIntensityAttr(3000)

    # 로봇 3대 배치: 수확 MM(원점) + 지게차 B(왼쪽 뒤) + 지게차 C(오른쪽 뒤).
    # 둘을 5m 띄워 서로·MM 과 안 겹치게 한다 (사용자 요청).
    HarvestMM(CFG.robots).spawn(stage, "/World/Harvester", (0.0, 0.0, 0.0))
    TransporterAMR(CFG.robots, CFG.warehouse).spawn(
        stage, "/World/Transporter", (-2.5, -4.0, 0.0))
    TransporterAMR(CFG.robots, CFG.warehouse).spawn(
        stage, "/World/TransporterC", (2.5, -4.0, 0.0),
        asset_candidates=CFG.robots.assets.forklift_c)

    # 도달 기준 마커 (spike 04 와 동일한 근거값)
    lo, hi = CFG.plants.fruit_height_range
    horiz = CFG.plants.row_spacing / 2 - CFG.plants.fruit_offset
    add_marker(stage, "/World/M_lo", (horiz, 0.0, lo), (0.2, 0.8, 0.2))
    add_marker(stage, "/World/M_hi", (horiz, 0.0, hi), (0.9, 0.1, 0.1))
    add_marker(stage, "/World/M_op", (-horiz, 0.0, hi), (0.6, 0.1, 0.6))

    mm_robot = world.scene.add(Robot(prim_path=art_root(stage, "/World/Harvester"),
                                     name="mm"))
    fk_robot = world.scene.add(Robot(prim_path=art_root(stage, "/World/Transporter"),
                                     name="fk"))
    fc_robot = world.scene.add(Robot(prim_path=art_root(stage, "/World/TransporterC"),
                                     name="fc"))
    world.reset()
    mm = HarvesterController(mm_robot)
    fk = TransporterController(fk_robot)          # ForkliftB 자동 감지
    fc = TransporterController(fc_robot)          # ForkliftC 자동 감지
    transporters = {"fk": fk, "fc": fc}

    # ---- 키보드: 눌린 키 집합을 유지 ----
    pressed = set()

    def on_key(e, *_):
        if e.type == carb.input.KeyboardEventType.KEY_PRESS:
            pressed.add(e.input)
            if e.input == K.KEY_1:
                state["robot"] = "mm"; print(">>> 수확 MM 선택")
            elif e.input == K.KEY_2:
                state["robot"] = "fk"; print(">>> 지게차 B(오더피커) 선택")
            elif e.input == K.KEY_3:
                state["robot"] = "fc"; print(">>> 지게차 C(카운터밸런스) 선택")
            elif e.input == K.SPACE:
                print(f"  MM  {mm.joint_report()}")
                print(f"  FK  {fk.joint_report()}")
                print(f"  FC  {fc.joint_report()}")
            elif e.input == K.ESCAPE:
                state["run"] = False
        elif e.type == carb.input.KeyboardEventType.KEY_RELEASE:
            pressed.discard(e.input)
        return True

    state = {"robot": "mm", "run": True}
    app_win = omni.appwindow.get_default_app_window()
    kbd = app_win.get_keyboard()
    inp = carb.input.acquire_input_interface()
    sub = inp.subscribe_to_keyboard_events(kbd, on_key)

    # 프레임당 증분
    DA, DB, DYAW, DG = 0.010, 0.005, 0.010, 0.02      # 팔/베이스/회전/그리퍼
    DFORK, DSTEER, DRIVE_V = 0.003, 0.010, 5.0         # 포크/조향/구동속도(과속방지)

    # (키+, 키-, 팔관절i)
    ARM_KEYS = [(K.Q, K.A, 0), (K.W, K.S, 1), (K.E, K.D, 2),
                (K.R, K.F, 3), (K.T, K.G, 4), (K.Y, K.H, 5)]

    print(__doc__)
    print(">>> 수확 MM 선택 (Play 가 자동 시작됨)\n")

    while simulation_app.is_running() and state["run"]:
        if state["robot"] == "mm":
            for kp, km, i in ARM_KEYS:
                if kp in pressed: mm.move_arm(i, DA)
                if km in pressed: mm.move_arm(i, -DA)
            if K.O in pressed: mm.move_gripper(-DG)   # 열기
            if K.P in pressed: mm.move_gripper(DG)    # 닫기
            dx = (K.I in pressed) - (K.K in pressed)   # 앞/뒤
            dy = (K.J in pressed) - (K.L in pressed)   # 좌/우 (홀로노믹 옆이동)
            dw = (K.U in pressed) - (K.M in pressed)   # 회전
            if dx or dy or dw:
                mm.move_base(dx * DB, dy * DB, dw * DYAW)
        else:
            t = transporters[state["robot"]]           # 선택된 지게차(B 또는 C)
            drive = (K.I in pressed) - (K.K in pressed)
            t.set_drive(drive * DRIVE_V)
            if K.J in pressed: t.move_steer(DSTEER)
            if K.L in pressed: t.move_steer(-DSTEER)
            if K.U in pressed: t.move_fork(DFORK)
            if K.M in pressed: t.move_fork(-DFORK)
            # 선택 안 된 지게차는 구동 정지 (계속 굴러가지 않게)
            for key, other in transporters.items():
                if key != state["robot"]:
                    other.set_drive(0.0)

        mm.apply()
        fk.apply()
        fc.apply()
        world.step(render=True)

    inp.unsubscribe_to_keyboard_events(kbd, sub)
    simulation_app.close()


if __name__ == "__main__":
    main()
