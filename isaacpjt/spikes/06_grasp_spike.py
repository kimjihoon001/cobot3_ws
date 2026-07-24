# -*- coding: utf-8 -*-
"""파지 스파이크 씬 (경량) — greenhouse 없이 바닥 + MM 로봇 + 간격 둔 과실 몇 개만.

사용자 요청 2026-07-22:
  · "전용 경량 스파이크 파일" — 275과실 greenhouse 를 띄우는 무거운 main.py 말고 이 위에서.
  · "따닥따닥 붙이면 잡기 더 힘들" — 과실을 여유 두고 배치(dy 기본 0.10m).
  · "Pilz만" — 팔을 HOME_POSE_DEG(=MoveIt HOME_Q)에서 시작 → 매 파지가 작은 재구성 → Pilz 성공.
  · reset 반복 상태오염 회피 — 과실 여러 개 스폰 후 select_fruit 로 fresh 과실 사용.

이 씬은 main.py 의 검증된 배선(브리지·조립 lifecycle)을 그대로 재활용한다. 다른 점은
GreenhouseTask 대신 바닥+조명만 올린다는 것뿐. 로봇/브리지 코드는 mm.py 그대로.

실행 (Isaac 터미널):
  isaac_python spikes/06_grasp_spike.py            # GUI
  isaac_python spikes/06_grasp_spike.py --headless
  AIRFRUIT_N=5 AIRFRUIT_DY=0.10 isaac_python spikes/06_grasp_spike.py
ROS: /harvester_0/joint_command↔joint_states(topic_based), /sim/tomato,
     /harvester_0/cmd {select_fruit:i}/{drop_air}/{set_friction:μ}
스윕: src/.../harvest_moveit/scripts/grasp_sweep.py (별도 ROS2 터미널, MoveIt 스택).
"""
import os
import sys
from pathlib import Path

GUI = "--headless" not in sys.argv
NO_ROS = "--no-ros" in sys.argv
CAMERA = "--camera" in sys.argv            # 기본 끔(가벼움 — 파지 판정은 과실 z 로, 카메라 X)
ISAAC_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _bootstrap_isaac_ros2() -> None:
    """main.py 와 동일 — Isaac 내장 Humble lib 를 LD_LIBRARY_PATH 앞에 넣고 한 번 재실행."""
    if NO_ROS:
        return
    executable = Path(sys.executable).absolute()
    for parent in executable.parents:
        humble = parent / "exts" / "isaacsim.ros2.bridge" / "humble"
        if (humble / "lib").is_dir():
            break
    else:
        print("[Spike] Isaac 내장 Humble 경로 못 찾음 — 브리지 실패할 수 있음")
        return
    marker = str(humble)
    if os.environ.get("ISAACPJT_ROS_ROOT") == marker:
        return
    env = os.environ.copy()
    env["ISAACPJT_ROS_ROOT"] = marker
    env["LD_LIBRARY_PATH"] = os.pathsep.join(
        p for p in (str(humble / "lib"), env.get("LD_LIBRARY_PATH")) if p)
    rclpy_path = humble / "rclpy"
    if rclpy_path.is_dir():
        env["PYTHONPATH"] = os.pathsep.join(
            p for p in (str(rclpy_path), env.get("PYTHONPATH")) if p)
    env.setdefault("ROS_DISTRO", "humble")
    env["ROS_DOMAIN_ID"] = "108"
    env["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp"
    env["ROS_LOCALHOST_ONLY"] = "0"
    print(f"[Spike] Isaac 내장 ROS2 env 적용: {humble}", flush=True)
    os.execve(str(executable),
              [str(executable), str(Path(__file__).resolve()), *sys.argv[1:]], env)


_bootstrap_isaac_ros2()

from isaacsim import SimulationApp                                    # noqa: E402

simulation_app = SimulationApp({"headless": not GUI})

sys.path.insert(0, ISAAC_DIR)

if not NO_ROS:
    from isaacsim.core.utils.extensions import enable_extension      # noqa: E402
    for _ext in ("isaacsim.core.nodes", "isaacsim.ros2.bridge",
                 "omni.graph.bundle.action", "omni.graph.window.action"):
        enable_extension(_ext)
    for _ in range(20):
        simulation_app.update()

import numpy as np                                                   # noqa: E402,F401
import omni.usd                                                      # noqa: E402
from isaacsim.core.api import World                                  # noqa: E402
from pxr import Gf, Usd, UsdGeom, UsdLux                             # noqa: E402

from pjt_config.settings import SceneConfig                          # noqa: E402
from rmp_mm import RmpMMDriver                                              # noqa: E402


class Opts:
    """MMDriver finalize/update 에 넘길 옵션 — 경량: 카메라·nav·rmpflow·teleop 전부 끔."""
    no_ros = NO_ROS
    gui = GUI
    mm_teleop = False
    rmpflow = False
    camera = CAMERA
    nav_drive = False
    nav_odom = False
    nav_scan = False


def _add_light(stage) -> None:
    """간단 조명 — 돔(환경광) + 태양(그림자)."""
    dome = UsdLux.DomeLight.Define(stage, "/World/SpikeDome")
    dome.CreateIntensityAttr(600.0)
    sun = UsdLux.DistantLight.Define(stage, "/World/SpikeSun")
    sun.CreateIntensityAttr(1500.0)
    UsdGeom.Xformable(sun.GetPrim()).AddRotateXYZOp().Set(Gf.Vec3f(-45.0, 0.0, 0.0))


def _spawn_one_airfruit(stage, world_pos, idx: int) -> str:
    """공중 과실 1개 — 실제 토마토 USD Body + 중심 충돌구 + 그립 줄기 원통. 인덱스별 μ 머티리얼."""
    from isaacsim.core.utils.stage import add_reference_to_stage
    from scene.physics import (add_sphere_collider, add_rigid_body, add_cylinder_collider,
                               create_physics_material, bind_physics_material)
    S = 0.001675
    body_usd = os.path.join(ISAAC_DIR, "assets", "tomato", "tomato_ripe_03.usd")
    cache = UsdGeom.XformCache()
    path = f"/World/AirFruit_{idx}"
    fruit = UsdGeom.Xform.Define(stage, path)
    xf = UsdGeom.Xformable(fruit.GetPrim())
    xf.AddTranslateOp().Set(Gf.Vec3d(world_pos))
    xf.AddScaleOp().Set(Gf.Vec3f(S, S, S))
    add_reference_to_stage(body_usd, path + "/Body")
    body_prim = stage.GetPrimAtPath(path + "/Body")
    cw = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                          [UsdGeom.Tokens.default_, UsdGeom.Tokens.render]
                          ).ComputeWorldBound(body_prim).ComputeAlignedRange().GetMidpoint()
    cl = cache.GetLocalToWorldTransform(fruit.GetPrim()).GetInverse().Transform(cw)
    add_sphere_collider(stage, path + "/Collision", 0.025 / S, center=(cl[0], cl[1], cl[2]))
    add_rigid_body(fruit.GetPrim(), 1000.0, kinematic=True)
    # ★질량 명시 오버라이드 — 스케일(S) 프림에 density 만 주면 PhysX 가 로컬 콜라이더로
    #   질량계산해 수백만 kg 가 될 수 있음(2026-07-23 의심). 실제 토마토 120g 으로 못박음.
    from pxr import UsdPhysics
    UsdPhysics.MassAPI.Apply(fruit.GetPrim()).CreateMassAttr(
        float(os.environ.get("GRIP_MASS", "0.12")))
    # ★마찰 재질을 몸통 콜라이더(/Collision)에 '직접' 바인딩 — 루트 상속만으론 PhysX 가
    #   콜라이더 마찰로 안 써서 무마찰이 됐음(2026-07-23 근본원인). 루트+콜라이더 둘 다 바인딩.
    _fmat = create_physics_material(stage, f"/World/PM/airfruit_{idx}", 0.9, 0.7)
    bind_physics_material(fruit.GetPrim(), _fmat)
    bind_physics_material(stage.GetPrimAtPath(path + "/Collision"), _fmat)
    up = (0.025 + 0.025) / S
    # 그립 줄기(원통) — ★실제 꽃자루 굵기. §6: 꽃자루 지름 3.5~5.5mm. env 로 정직하게 스윕.
    stem_r_m = float(os.environ.get("GRIP_STEM_R_MM", "2.75")) / 1000.0   # 반지름[m], 기본 5.5mm지름
    add_cylinder_collider(stage, path + "/GripStem", stem_r_m / S, 0.06 / S,
                          center=(cl[0], cl[1], cl[2] + up), visible=True)
    bind_physics_material(stage.GetPrimAtPath(path + "/GripStem"),
                          create_physics_material(stage, f"/World/PM/airstem_{idx}", 0.9, 0.7))
    # 그립 육면체 — 평평한 면이라 평행조가 면접촉으로 안정(사용자 아이디어 2026-07-23). 월드 크기 env.
    from scene.physics import add_box_collider
    _bx = float(os.environ.get("GRIP_BOX_X_MM", "20")) / 1000.0
    _by = float(os.environ.get("GRIP_BOX_Y_MM", "20")) / 1000.0
    _bz = float(os.environ.get("GRIP_BOX_Z_MM", "40")) / 1000.0
    _boxp = add_box_collider(stage, path + "/GripBox",
                             size=(_bx / S, _by / S, _bz / S),
                             center=(cl[0], cl[1], cl[2] + up), visible=True)
    # ★줄기는 토마토 껍질과 다름 — 식물 줄기는 거칠어 마찰 높음(사용자 2026-07-23). combineMode=max 로
    #   거친 줄기 마찰이 지배(패드 μ0.9 와 결합). env GRIP_BOX_MU 로 값 조정.
    from pxr import PhysxSchema as _PXm
    _bmu = float(os.environ.get("GRIP_BOX_MU", "1.5"))
    _boxmat = create_physics_material(stage, f"/World/PM/airbox_{idx}", _bmu, _bmu * 0.85)
    _PXm.PhysxMaterialAPI.Apply(_boxmat.GetPrim()).CreateFrictionCombineModeAttr().Set("max")
    bind_physics_material(_boxp, _boxmat)
    return path


def _spawn_fruits(stage, driver) -> None:
    """팔 앞 도달권에 실제 토마토 여러 개(가로 한 줄, 여유 간격) — fresh 과실 스윕용."""
    base = stage.GetPrimAtPath(f"{driver.root}/Base/base_link")
    if not base.IsValid():
        print("[Spike] base_link 못 찾음 — 과실 스폰 생략"); return
    n = int(os.environ.get("AIRFRUIT_N", "5"))
    dy = float(os.environ.get("AIRFRUIT_DY", "0.10"))        # 여유 간격(따닥따닥 금지)
    b2w = UsdGeom.XformCache().GetLocalToWorldTransform(base)
    paths = []
    for i in range(n):
        y = (i - (n - 1) / 2.0) * dy
        paths.append(_spawn_one_airfruit(stage, b2w.Transform(Gf.Vec3d(0.6, y, 1.0)), i))
    driver.set_air_fruits(paths)
    print(f"[Spike] 실제 토마토 {n}개 스폰 — 섀시 (0.6, ±, 1.0) dy={dy}m (여유 간격)", flush=True)


def main() -> None:
    cfg = SceneConfig()
    world = World(stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane()                   # 바닥만 (greenhouse 없음 = 경량)
    world.reset()
    stage = omni.usd.get_context().get_stage()
    _add_light(stage)

    # ── MM 조립 lifecycle (main._assemble_robots 와 동일 순서) ──
    d = RmpMMDriver(cfg, task=None)
    d.spawn(stage)
    d.register(world, stage)
    world.reset()
    d.configure(world)                                       # 팔 HOME_POSE_DEG(=MoveIt HOME_Q)
    world.reset()
    for _ in range(15):
        world.step(render=False)
    if not NO_ROS:
        from ros import robot_bridge as RB
        RB.build_clock()
    d.finalize(world, stage, Opts())                         # ROS 브리지(joint/cmd/sim_tomato)
    world.reset()
    for _ in range(5):
        world.step(render=False)

    _spawn_fruits(stage, d)

    if GUI:
        from isaacsim.core.utils.viewports import set_camera_view
        set_camera_view(eye=[1.6, -1.4, 1.6], target=[0.4, 0.0, 1.0])
    if not NO_ROS:
        print("[Spike] 브리지 대기 — /harvester_0/{joint_command,joint_states,cmd,sim/tomato} "
              "(domain 108). MoveIt 스택 + grasp_sweep.py 로 스윕.\n", flush=True)
    if not GUI:
        world.play()

    was_playing = False
    while simulation_app.is_running():
        pre = world.is_playing()
        if pre and not was_playing:
            d.update(False)
            world.reset()
            d.update(False)
        world.step(render=True)
        is_playing = world.is_playing()
        was_playing = is_playing
        d.update(is_playing)


try:
    main()
finally:
    simulation_app.close()
