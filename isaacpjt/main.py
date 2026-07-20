# -*- coding: utf-8 -*-
"""스마트팜 온실 씬 + 로봇 3대 + ROS2 제어 진입점 (Isaac Sim 5.1 Standalone).

실행 (ROS2 브리지 포함 — env 필수, 없으면 브리지만 실패하고 씬은 뜬다):
  LD_LIBRARY_PATH=<isaac>/exts/isaacsim.ros2.bridge/humble/lib:$LD_LIBRARY_PATH \\
  RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_DOMAIN_ID=108 \\
  isaac_python main.py            (<isaac>=~/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release)
  isaac_python main.py --no-ros   (씬+로봇만)
  isaac_python main.py --headless
  isaac_python main.py --quiet    (metricsAssembler 스팸 끔 — omni.usd 경고도 같이 숨음 주의)
  isaac_python main.py --nav-drive   (iw.hub Nav2 브리지 단계검증: /cmd_vel→바퀴. 순서대로:
                     --nav-odom(/odom+TF) → --nav-scan(라이다→/scan) → --nav(셋 다).
                     ⚠ 노드명 미확정 — tools/nav2_node_probe.py 로 먼저 실측할 것)
  isaac_python main.py --camera      (손끝 D455 → /harvester/rgb·/depth·/camera_info 발행,
                     YOLO 파인튜닝용. ⚠ 노드명 probe 미확정)
  isaac_python main.py --teleop      (MM 키보드 텔레옵 — 팔·베이스·그리퍼·블레이드 직접 조작)

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
# iw.hub Nav2 브리지 — 하나씩 켜며 GPU 검증(순서 drive→odom→scan). --nav 는 셋 다.
# ⚠ 노드 타입명 미확정(tools/nav2_node_probe.py 로 먼저 실측). 기본 꺼짐이라 씬엔 영향 없음.
NAV_DRIVE = "--nav-drive" in sys.argv or "--nav" in sys.argv
NAV_ODOM = "--nav-odom" in sys.argv or "--nav" in sys.argv
NAV_SCAN = "--nav-scan" in sys.argv or "--nav" in sys.argv
# 손끝 D455 → ROS2 rgb/depth 발행 (YOLO 파인튜닝용). ⚠ 노드명 probe 미확정, 기본 꺼짐.
CAMERA = "--camera" in sys.argv
# MM 키보드 텔레옵 (팔·베이스·그리퍼·블레이드 직접 조작). ROS2 대신 키로 움직여 뷰 확보용.
TELEOP = "--teleop" in sys.argv

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
from robots.control import TransporterController
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
    반환: (MM StringPoller, 지게차 JointCommandPoller) 튜플.
    """
    try:
        from ros import robot_bridge as RB
        RB.build_clock()
        for ns, art in arts:
            RB.build_joint_bridge(
                stage,
                f"/World/RosBridge_{ns}",
                ns,
                art,
                apply_commands=ns != "forklift_0",
            )
        mm_sub = RB.build_string_sub("/World/RosCmd_harvester_0", "/harvester_0/cmd")
        forklift_sub = RB.JointCommandPoller(
            "/World/RosBridge_forklift_0/Sub"
        )
        return RB.StringPoller(mm_sub), forklift_sub
    except Exception:
        import traceback
        print("\n" + "=" * 64)
        print("[RosBridge] 생성 실패 — 씬만 띄운다. ROS2 명령은 안 먹는다.")
        print("  env 확인: LD_LIBRARY_PATH(브리지 humble/lib), RMW, ROS_DOMAIN_ID")
        print("=" * 64)
        traceback.print_exc()
        print("=" * 64 + "\n")
        return None, None


def build_nav(stage, iw, art_path: str, nav) -> None:
    """iw.hub 자율주행 그래프 — 플래그로 켠 것만 배선. 실패해도 씬은 유지(브리지와 동일 방침).

    순서대로 GPU 검증: drive(/cmd_vel→바퀴) → odom(/odom+TF) → scan(라이다→/scan).
    노드 타입명은 tools/nav2_node_probe.py 로 확정 후 robot_bridge.T 갱신할 것(§8).
    """
    from ros import robot_bridge as RB

    base = f"{iw.root}/base_link"
    chassis = base if stage.GetPrimAtPath(base).IsValid() else art_path
    try:
        if NAV_DRIVE:
            RB.build_diff_drive(stage, "/World/Nav_drive", art_path,
                                iw.DRIVE_JOINTS, nav)
        if NAV_ODOM:
            RB.build_odometry(stage, "/World/Nav_odom", chassis, nav)
        if NAV_SCAN:
            lidar = iw.attach_lidar(stage, nav.lidar_offset)
            if lidar:
                RB.build_tf_sensor(stage, "/World/Nav_tf", chassis, lidar, nav)
                RB.build_lidar_scan(stage, "/World/Nav_scan", lidar, nav)
    except Exception:
        import traceback
        print("\n" + "=" * 64)
        print("[Nav] 그래프 생성 실패 — 씬은 유지. tools/nav2_node_probe.py 로 노드명 확인.")
        print("=" * 64)
        traceback.print_exc()
        print("=" * 64 + "\n")


def build_teleop(mm, mm_robot):
    """MM 키보드 텔레옵 — 팔6·베이스·그리퍼·블레이드. 반환: step(is_playing) 콜백(실패 시 None).

    글자키만 쓴다(방향키는 뷰포트가 가로챔 — spike05 실측). GUI 전용. mm_robot 이 물리
    초기화(world.reset)된 뒤 호출할 것 — HarvesterController 가 현재 관절값에서 출발한다.
    """
    if not GUI:
        print("[Teleop] --headless 라 키보드 입력 불가 — 텔레옵 비활성")
        return None
    import carb.input
    import omni.appwindow

    from robots.control import HarvesterController

    ctrl = HarvesterController(mm_robot)
    K = carb.input.KeyboardInput
    pressed: set = set()
    st = {"blade": 0.0}

    def on_key(e, *_):
        if e.type == carb.input.KeyboardEventType.KEY_PRESS:
            pressed.add(e.input)
        elif e.type == carb.input.KeyboardEventType.KEY_RELEASE:
            pressed.discard(e.input)
        return True

    appwin = omni.appwindow.get_default_app_window()
    carb.input.acquire_input_interface().subscribe_to_keyboard_events(
        appwin.get_keyboard(), on_key)

    DQ, DB, DYAW, DG, DBL = 0.02, 0.01, 0.02, 0.03, 2.0
    ARM = [(K.Q, K.A), (K.W, K.S), (K.E, K.D),      # 1~6번 관절 ±
           (K.R, K.F), (K.T, K.G), (K.Y, K.H)]
    print("""
[MM 텔레옵] 플레이 상태에서, 글자키만 (방향키는 뷰포트가 가로챔)
  팔    Q/A W/S E/D R/F T/G Y/H = 1~6번 관절 ±
  베이스 I/K 전후 · J/L 좌우 · U/O 회전
  그리퍼 Z 열기 / X 닫기      블레이드 B 열기(0°) / N 닫기(절단)
""")

    def step(is_playing):
        if not is_playing:
            return
        for i, (kp, km) in enumerate(ARM):
            if kp in pressed:
                ctrl.move_arm(i, DQ)
            if km in pressed:
                ctrl.move_arm(i, -DQ)
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
            mm.set_blade_deg(st["blade"])
        ctrl.apply()

    return step


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
    iw = IwHub(cfg.robots)
    iw.spawn(stage, "/World/IwHub", ROBOT_POSE["iwhub"])

    arts = [("harvester_0", art_root(stage, "/World/Harvester")),
            ("forklift_0", art_root(stage, "/World/Forklift")),
            ("iwhub_0", art_root(stage, "/World/IwHub"))]
    for ns, art in arts:
        if art is None:
            raise RuntimeError(f"{ns} 아티큘레이션 루트를 못 찾음 — 에셋 확인")
    mm_robot = world.scene.add(Robot(prim_path=arts[0][1], name="mm"))
    fk_robot = world.scene.add(Robot(prim_path=arts[1][1], name="fk"))
    world.scene.add(Robot(prim_path=arts[2][1], name="iw"))
    world.reset()                                # 로봇 물리 초기화

    # iw.hub 데크에 '적재된 세트' (팔레트+KLT 8 + 토마토 15개 꼭지포함·동적강체, 3칸 산포)
    iw.load_cargo(stage, cfg.tomato_assets, cfg.physics)

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
    # 힌지 강체·조인트는 위 reset 뒤에 추가돼 아직 물리 뷰에 없다 → 여기서 한 번 더 reset 해
    # 미리 초기화·정착시킨다. 안 하면 플레이 순간 첫 reset 에서 날·브라켓이 물리 자리로 스냅해
    # "플레이하면 장착 브라켓이 생기는" 것처럼 보인다(2026-07-20 사용자 지적).
    world.reset()
    for _ in range(5):
        world.step(render=False)

    forklift_controller = TransporterController(fk_robot)

    # MM 키네마틱 베이스 인덱스 (JSON base 명령용 — 텔레포트만 먹는다, 2026-07-18 실측)
    base_idx = np.array([list(mm_robot.dof_names).index(n) for n in MM_BASE_JOINTS])

    if GUI:
        from isaacsim.core.utils.viewports import set_camera_view
        g = cfg.greenhouse
        set_camera_view(eye=[g.width * 0.9, -g.length * 0.8, 12.0],
                        target=[0.0, 2.0, 0.5])

    mm_poller, forklift_poller = (
        (None, None) if NO_ROS else build_ros_control(stage, arts)
    )
    if not NO_ROS and (NAV_DRIVE or NAV_ODOM or NAV_SCAN):
        build_nav(stage, iw, arts[2][1], cfg.robots.iwhub_nav)
    if not NO_ROS and CAMERA:
        cam_prim = mm.camera_path(stage)
        if cam_prim:
            try:
                from ros import robot_bridge as RB
                RB.build_camera(stage, "/World/RosCamera", cam_prim, cfg.robots.camera)
            except Exception:
                import traceback
                print("\n[Camera] 그래프 생성 실패 — 씬 유지. probe 로 노드명 확인.")
                traceback.print_exc()
        else:
            print("[Camera] D455 카메라 prim 못 찾음 — rgb/depth 발행 스킵")

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
    if mm_poller is not None:
        print("[RosBridge] 대기 중 — ROS_DOMAIN_ID=108")
        print("[RosBridge] /forklift_0/joint_command 수신 / "
              "/forklift_0/joint_states 발행")
        print("[RosBridge] 타임라인이 Play 상태일 때만 토픽과 로봇 제어가 동작합니다.\n")

    teleop = build_teleop(mm, mm_robot) if TELEOP else None

    # Play/Stop 반복 시 동일한 초기 상태에서 재시작 (재현성)
    was_playing = False
    last_forklift_motion = None
    while simulation_app.is_running():
        world.step(render=True)
        is_playing = world.is_playing()
        if is_playing and not was_playing:
            world.reset()
            if mm_poller is not None:
                print("[RosBridge] 타임라인 Play — ROS2 통신 동작 중 (domain 108)")
        elif not is_playing and was_playing and mm_poller is not None:
            print("[RosBridge] 타임라인 Stop — ROS2 토픽 처리가 일시 정지됩니다.")
        was_playing = is_playing

        if teleop is not None:                   # MM 키보드 텔레옵 (재생 중에만 적용)
            teleop(is_playing)

        # MM JSON 명령 (블레이드·베이스) — 재생 중에만 적용
        if is_playing and mm_poller is not None:
            raw = mm_poller.poll()
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

        # OmniGraph 구독 출력은 읽되, ForkliftB 명령 적용은 위치/속도를 분리한다.
        if is_playing and forklift_poller is not None:
            cmd = forklift_poller.poll()
            if cmd:
                names, positions, velocities = cmd
                for name, value in zip(names, positions):
                    if not np.isfinite(value):
                        continue
                    if name == "lift_joint":
                        forklift_controller.set_fork(float(value))
                    elif name == "back_wheel_swivel":
                        forklift_controller.set_steer(float(value))
                for name, value in zip(names, velocities):
                    if name == "back_wheel_drive" and np.isfinite(value):
                        forklift_controller.set_drive(float(value))
                motion = (
                    round(forklift_controller._drive_vel, 2),
                    round(float(forklift_controller._steer), 3),
                )
                if motion != last_forklift_motion:
                    print(f"[Forklift RX] drive={motion[0]:.2f} rad/s  "
                          f"steer={np.degrees(motion[1]):.1f} deg")
                    last_forklift_motion = motion
            # ForkliftB는 후륜 관절만 돌고 차체가 헛도는 경우가 있어, 물리 관절
            # 명령과 함께 60 Hz 평면 차량 운동을 적용한다.
            forklift_controller.apply(dt=1.0 / 60.0)


main()
simulation_app.close()
