# -*- coding: utf-8 -*-
"""스마트팜 온실 씬 + 로봇 3대 + ROS2 제어 진입점 (Isaac Sim 5.1 Standalone).

실행 (ROS2 브리지 포함 — env 필수, 없으면 브리지만 실패하고 씬은 뜬다):
  LD_LIBRARY_PATH=<isaac>/exts/isaacsim.ros2.bridge/humble/lib:$LD_LIBRARY_PATH \\
  RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_DOMAIN_ID=108 \\
  isaac_python main.py            (<isaac>=~/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release)
  isaac_python main.py --no-ros   (씬+로봇만)
  isaac_python main.py --headless
  isaac_python main.py --quiet    (metricsAssembler 스팸 끔 — omni.usd 경고도 같이 숨음 주의)

로봇 3대 (물류 루프: MM 수확 → iw.hub 팔레트+KLT 운반 → 지게차 랙 적재):
  /World/Harvester   수확 MM (Ridgeback+UR10e+2F-85+커터지그+가동날+D455)
  /World/Forklift    지게차 B (포크 승강)
  /World/IwHub       운반 AMR (iw.hub, 차동+승강)

ROS2 토픽 (§5.6: 판단은 ROS2 = dev 머신, 실행만 여기. domain 108):
  로봇별  /{ns}/joint_command  sensor_msgs/JointState  ← 관절 명령 (이름 지정)
          /{ns}/joint_states   sensor_msgs/JointState  → 관절 상태
          ns = harvester_0 / forklift_0 / iwhub_0
  MM 전용 /harvester_0/cmd     std_msgs/String(JSON)   ← 아티큘레이션 밖 자유도:
          {"blade": 0~35}      가동날 각도[deg] (별도 리볼루트 — JointState 에 안 잡힘)
          {"base": [x,y,yaw]}  홀로노믹 베이스 (키네마틱 — 위치드라이브 무시, 텔레포트만)
  공용    /clock

5A 루프백 확인 (GPU 의 ROS2 터미널):
  ros2 topic echo /iwhub_0/joint_states --once
  ros2 topic pub -1 /iwhub_0/joint_command sensor_msgs/JointState \\
    '{name: [left_wheel_joint, right_wheel_joint], velocity: [3.0, 3.0]}'
  ros2 topic pub -1 /harvester_0/cmd std_msgs/String '{data: "{\\"blade\\": 35}"}'
"""
import json
import sys

GUI = "--headless" not in sys.argv
NO_ROS = "--no-ros" in sys.argv
QUIET = "--quiet" in sys.argv

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": not GUI})

if QUIET:
    # CAD 지그 USD(단위보정 레이어) 참조 시 omni.usd 가 "SetEditTarget ... metricsAssembler"
    # 스팸을 수백 줄 뿌린다(Kit 버그성 — 조립 순간에만, 무해). 옵트인으로만 끈다:
    # omni.usd 채널을 error 로 낮추면 다른 USD '경고'도 같이 숨으므로 기본은 켜 둔다(§8).
    import carb.settings
    carb.settings.get_settings().set("/log/channels/omni.usd", "error")

import os

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

if not NO_ROS:
    # 브리지 확장은 그래프 생성 전에 켜야 한다 (tools/iwhub_bridge_check.py 검증 순서)
    from isaacsim.core.utils.extensions import enable_extension
    for _ext in ("isaacsim.core.nodes", "isaacsim.ros2.bridge"):
        enable_extension(_ext)
    for _ in range(20):
        simulation_app.update()

import numpy as np
import omni.usd
from isaacsim.core.api import World
from isaacsim.core.api.robots import Robot
from pxr import Usd, UsdPhysics

from pjt_config.settings import SceneConfig
from robots.harvester import HarvestMM
from robots.iwhub import IwHub
from robots.transporter import TransporterAMR
from scene.greenhouse_task import GreenhouseTask

# 로봇 임시 배치 — 온실 앞마당(빈 홀 바닥, 온실 y −10 앞). 물류 동선 확정 후 조정.
ROBOT_POSE = {
    "harvester": (0.0, -12.0, 0.0),
    "forklift": (0.0, 15.5, 0.0),    # 창고 안(입구 지나 개활부) — 랙 적재 담당 (2026-07-20)
    "iwhub": (2.0, -12.0, 0.0),
}
MM_BASE_JOINTS = ("dummy_base_prismatic_x_joint",
                  "dummy_base_prismatic_y_joint",
                  "dummy_base_revolute_z_joint")


def art_root(stage, under: str) -> str | None:
    for p in Usd.PrimRange(stage.GetPrimAtPath(under)):
        if p.HasAPI(UsdPhysics.ArticulationRootAPI):
            return str(p.GetPath())
    return None


def build_ros_control(stage, arts: list[tuple[str, str]]):
    """로봇 3대 조인트 브리지 + /clock + MM JSON 명령 구독.

    실패해도 씬은 계속 띄운다 (기존 방침 — 실패 지점만 분명히 알린다).
    반환: MM cmd 의 StringPoller (실패 시 None).
    """
    try:
        from ros import robot_bridge as RB
        RB.build_clock()
        for ns, art in arts:
            RB.build_joint_bridge(stage, f"/World/RosBridge_{ns}", ns, art)
        sub = RB.build_string_sub("/World/RosCmd_harvester_0", "/harvester_0/cmd")
        return RB.StringPoller(sub)
    except Exception:
        import traceback
        print("\n" + "=" * 64)
        print("[RosBridge] 생성 실패 — 씬만 띄운다. ROS2 명령은 안 먹는다.")
        print("  env 확인: LD_LIBRARY_PATH(브리지 humble/lib), RMW, ROS_DOMAIN_ID")
        print("=" * 64)
        traceback.print_exc()
        print("=" * 64 + "\n")
        return None


def main() -> None:
    cfg = SceneConfig()
    world = World(stage_units_in_meters=1.0)

    task = GreenhouseTask(name="greenhouse", cfg=cfg)
    world.add_task(task)
    world.reset()                                # 씬 생성

    # ── 로봇 3대 ──
    stage = omni.usd.get_context().get_stage()
    mm = HarvestMM(cfg.robots)
    mm.spawn(stage, "/World/Harvester", ROBOT_POSE["harvester"])
    TransporterAMR(cfg.robots, cfg.warehouse).spawn(
        stage, "/World/Forklift", ROBOT_POSE["forklift"])
    IwHub(cfg.robots).spawn(stage, "/World/IwHub", ROBOT_POSE["iwhub"])

    arts = [("harvester_0", art_root(stage, "/World/Harvester")),
            ("forklift_0", art_root(stage, "/World/Forklift")),
            ("iwhub_0", art_root(stage, "/World/IwHub"))]
    for ns, art in arts:
        if art is None:
            raise RuntimeError(f"{ns} 아티큘레이션 루트를 못 찾음 — 에셋 확인")
    mm_robot = world.scene.add(Robot(prim_path=arts[0][1], name="mm"))
    world.scene.add(Robot(prim_path=arts[1][1], name="fk"))
    world.scene.add(Robot(prim_path=arts[2][1], name="iw"))
    world.reset()                                # 로봇 물리 초기화

    # ── MM 수확자세: wrist_1(4번축) +180° 를 스폰 기본자세로 ──
    # 기본자세면 커터·지그가 파지점 아래(뒤집힘). +180° 라야 절단점이 파지점 위 5.3cm
    # (2026-07-19 실측 — CAD 의도 그대로). default_state 라 Play/Stop 리셋에도 유지.
    q0 = np.asarray(mm_robot.get_joint_positions(), dtype=float)
    q0[list(mm_robot.dof_names).index("wrist_1_joint")] += np.pi
    mm_robot.set_joints_default_state(positions=q0)
    world.reset()
    for _ in range(15):                          # 자세 정착(§8 — 안 하면 옛 자세 읽음)
        world.step(render=False)
    mm.attach_blade_hinge(stage)                 # 가동날(서보 힌지) — 정착된 자세 기준

    # MM 키네마틱 베이스 인덱스 (JSON base 명령용 — 텔레포트만 먹는다, 2026-07-18 실측)
    base_idx = np.array([list(mm_robot.dof_names).index(n) for n in MM_BASE_JOINTS])

    if GUI:
        from isaacsim.core.utils.viewports import set_camera_view
        g = cfg.greenhouse
        set_camera_view(eye=[g.width * 0.9, -g.length * 0.8, 12.0],
                        target=[0.0, 2.0, 0.5])

    poller = None if NO_ROS else build_ros_control(stage, arts)

    obs = task.get_observations()
    fruits = obs["fruits"]
    counts: dict[str, int] = {}
    for f in fruits:
        counts[f["class_name"]] = counts.get(f["class_name"], 0) + 1
    print("\n[Scene] 과실 %d개" % len(fruits))
    for name in sorted(counts):
        print("  %-11s %4d" % (name, counts[name]))
    print("  수확 대상(ripe) %d개 / 제거 대상(spoiled) %d개\n"
          % (counts.get("ripe", 0), counts.get("spoiled", 0)))
    if poller is not None:
        print("[RosBridge] 대기 중 — 토픽 목록은 파일 상단 docstring 참조 (domain 108)\n")

    # Play/Stop 반복 시 동일한 초기 상태에서 재시작 (재현성)
    was_playing = False
    while simulation_app.is_running():
        world.step(render=True)
        is_playing = world.is_playing()
        if is_playing and not was_playing:
            world.reset()
        was_playing = is_playing

        # MM JSON 명령 (블레이드·베이스) — 재생 중에만 적용
        if is_playing and poller is not None:
            raw = poller.poll()
            if raw:
                try:
                    cmd = json.loads(raw)
                except ValueError:
                    cmd = None
                if isinstance(cmd, dict):
                    if "blade" in cmd:
                        mm.set_blade_deg(float(cmd["blade"]))
                    if "base" in cmd:
                        b = [float(v) for v in cmd["base"]]
                        if len(b) == 3:
                            mm_robot.set_joint_positions(
                                np.array(b), joint_indices=base_idx)


main()
simulation_app.close()
