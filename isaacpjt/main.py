# -*- coding: utf-8 -*-
"""스마트팜 온실 씬 진입점 (Isaac Sim 5.1 Standalone) — 환경만 만들고 로봇은 플래그로 고른다.

★ 이 파일은 '환경(씬) + 생명주기 골격'만 담는다. 로봇 코드는 각 로봇 파일에 있다:
    mm.py=수확 MM(--mm) / iw.py=운반 AMR(--iw) / fork.py=지게차(--fork).
  플래그로 고른 로봇만 스폰한다(조합 가능). 아무 로봇 플래그가 없으면 씬만 띄운다.
  로봇별 파일이 분리돼 있어 팀원끼리 서로 다른 로봇을 건드려도 이 파일은 거의 안 바뀐다.

실행 (ROS2 브리지 포함 — env 필수, 없으면 브리지만 실패하고 씬은 뜬다):
  LD_LIBRARY_PATH=<isaac>/exts/isaacsim.ros2.bridge/humble/lib:$LD_LIBRARY_PATH \\
  RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_DOMAIN_ID=108 \\
  isaac_python main.py --mm --iw --fork   (로봇 3대 전부 — 물류 루프 데모)
  isaac_python main.py --iw               (내 로봇만 — 개인 작업)
  isaac_python main.py                    (로봇 없이 씬만)
  isaac_python main.py --no-ros --mm      (씬+로봇만, 브리지 끔)
  isaac_python main.py --headless
  isaac_python main.py --quiet    (metricsAssembler 스팸 끔 — omni.usd 경고도 같이 숨음 주의)
  isaac_python main.py --iw --nav-drive   (iw.hub Nav2 브리지 단계검증: /cmd_vel→바퀴. 순서대로:
                     --nav-odom(/odom+TF) → --nav-scan(라이다→/scan) → --nav(셋 다).
                     ⚠ 노드명 미확정 — tools/nav2_node_probe.py 로 먼저 실측할 것)
  (카메라는 --mm 일 때 기본 자동 발행 — 손끝 D455 → /harvester_0/... , YOLO 파인튜닝용.
   ROS 켤 때 자동. 끄려면 --no-camera. ⚠ 노드명 probe 미확정 — 실패해도 씬은 그대로)
  isaac_python main.py --mm --mm-teleop   (MM 전용 텔레옵 — iw.hub와 완전 분리)
  isaac_python main.py --mm --iw --fork --export --headless   (조립한 씬을 USD 로 저장하고 종료.
                     기본 ~/cobot3_ws/scene.usd. 이름/경로 지정 가능: --export mm.usda)
  isaac_python main.py --load --mm --mm-teleop --no-ros (기존 USD에서 MM만 텔레옵)

로봇 3대 (물류 루프: MM 수확 → iw.hub 팔레트+KLT 운반 → 지게차 랙 적재):
  /World/Harvester   수확 MM (Ridgeback+UR10e+2F-85+커터지그+가동날+D455)   --mm
  /World/Forklift    지게차 B (포크 승강)                                    --fork
  /World/IwHub       운반 AMR (iw.hub, 차동+승강)                            --iw

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
import os
import sys
from pathlib import Path

GUI = "--headless" not in sys.argv
NO_ROS = "--no-ros" in sys.argv
QUIET = "--quiet" in sys.argv
# --airfruit: nav 없이 팔 앞 도달권에 과실 하나 띄우고 /sim/tomato 로 발행 → MoveIt 로
# 바로 잡는 테스트(주행·정렬 생략, 파지/TCP 빠른 반복). 시작 시 손가락 파지중심도 실측 출력.
AIRFRUIT = "--airfruit" in sys.argv
GRASP_MEAS_FILE = "/tmp/grasp_center_meas.txt"   # 파지중심 실측 결과(버퍼링 우회, ROS쪽서 읽음)
GRIPPER_DUMP_FILE = "/tmp/gripper_structure.txt" # Robotiq 서브트리 링크/메시 덤프(RViz 정합용)
# iw.hub Nav2 브리지 — 하나씩 켜며 GPU 검증(순서 drive→odom→scan). --nav 는 셋 다.
# ⚠ 노드 타입명 미확정(tools/nav2_node_probe.py 로 먼저 실측). 기본 꺼짐이라 씬엔 영향 없음.
NAV_DRIVE = "--nav-drive" in sys.argv or "--nav" in sys.argv
NAV_ODOM = "--nav-odom" in sys.argv or "--nav" in sys.argv
NAV_SCAN = "--nav-scan" in sys.argv or "--nav" in sys.argv
# 손끝 D455 → ROS2 rgb/depth 발행 (YOLO 파인튜닝용). 기본 켜짐(ROS 켤 때 자동). --no-camera 로 끔.
# ⚠ 노드명 probe 미확정 — 실패해도 씬은 그대로(main 이 예외 잡음).
CAMERA = "--no-camera" not in sys.argv
RMPFLOW = "--rmpflow" in sys.argv
# ★제어 모드 분리 (ROS2 겹침 방지, 2026-07-23) — 같은 MM 을 MoveIt 또는 RMPflow 로.
#   --moveit: 팔 브리지가 /joint_command 를 직접 적용(topic_based_ros2_control=MoveIt).
#   없으면(기본): 팔 브리지 apply 안 함 → 팀원 RMPflow 가 팔을 구동(둘이 동시에 안 싸움).
MOVEIT = "--moveit" in sys.argv
# MM 키보드 텔레옵 (팔·베이스·그리퍼·블레이드 직접 조작). ROS2 대신 키로 움직여 뷰 확보용.
# MM 키보드 입력은 명시적인 전용 플래그만 사용한다. --mm와 --iw를 같이 띄워도
# 키 입력이 iw.hub에 전달되거나 전역 teleop 상태를 공유하지 않는다.
MM_TELEOP = "--mm-teleop" in sys.argv
if "--teleop" in sys.argv:
    raise SystemExit("--teleop은 제거됐습니다. MM은 --mm --mm-teleop을 사용하세요.")
if MM_TELEOP and "--mm" not in sys.argv:
    raise SystemExit("--mm-teleop은 --mm과 함께 사용해야 합니다.")
if MM_TELEOP and NAV_DRIVE:
    raise SystemExit("MM 텔레옵과 Nav2는 동시에 베이스를 제어할 수 없습니다.")
# 지게차+운반 AMR만 선택하면 창고 자동화 단독 시험으로 본다. 이 모드에서는
# iw.py가 AMR을 창고 도킹 위치에 빈 상태로 놓아 첫 팔레트 상차를 바로 시험한다.
WAREHOUSE_TEST = (
    "--iw" in sys.argv and "--fork" in sys.argv and "--mm" not in sys.argv
)

# Warehouse 자동화의 공통 도메인은 108이다. ~/.bashrc가 109를 기본으로 내보내므로
# setdefault()를 쓰면 Isaac만 109에 남고 ROS 터미널(108)과 완전히 분리된다.
# 이 진입점에서는 양방향 브리지가 반드시 같은 값으로 뜨도록 명시적으로 고정한다.
if not NO_ROS:
    os.environ["ROS_DOMAIN_ID"] = "108"
    os.environ["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp"
    os.environ["ROS_LOCALHOST_ONLY"] = "0"


def _bootstrap_isaac_ros2() -> None:
    """Isaac 내장 Humble 라이브러리를 잡은 환경으로 main.py를 한 번 재실행한다."""
    if NO_ROS:
        return

    # resolve()하면 kit/python 심볼릭 링크가 Packman 캐시 경로로 바뀔 수 있으므로
    # 링크 경로 자체의 부모를 훑는다(forklift_teleop.py에서 검증한 방식).
    executable = Path(sys.executable).absolute()
    for parent in executable.parents:
        humble = parent / "exts" / "isaacsim.ros2.bridge" / "humble"
        if (humble / "lib").is_dir():
            break
    else:
        print(
            "[RosBridge] Isaac 내장 Humble 경로를 자동으로 찾지 못했습니다. "
            "LD_LIBRARY_PATH를 직접 설정해야 합니다."
        )
        return

    marker = str(humble)
    if os.environ.get("ISAACPJT_ROS_ROOT") == marker:
        return

    env = os.environ.copy()
    env["ISAACPJT_ROS_ROOT"] = marker
    env["LD_LIBRARY_PATH"] = os.pathsep.join(
        part for part in (str(humble / "lib"), env.get("LD_LIBRARY_PATH")) if part
    )
    rclpy_path = humble / "rclpy"
    if rclpy_path.is_dir():
        env["PYTHONPATH"] = os.pathsep.join(
            part for part in (str(rclpy_path), env.get("PYTHONPATH")) if part
        )
    env.setdefault("ROS_DISTRO", "humble")
    env["ROS_DOMAIN_ID"] = "108"
    env["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp"
    env["ROS_LOCALHOST_ONLY"] = "0"
    print(f"[RosBridge] Isaac 내장 ROS 2 환경 적용: {humble}", flush=True)
    os.execve(
        str(executable),
        [str(executable), str(Path(__file__).resolve()), *sys.argv[1:]],
        env,
    )


_bootstrap_isaac_ros2()

if not NO_ROS:
    print(
        "[RosBridge] 실제 실행 환경: "
        f"domain={os.environ['ROS_DOMAIN_ID']}, "
        f"RMW={os.environ['RMW_IMPLEMENTATION']}, "
        f"localhost_only={os.environ['ROS_LOCALHOST_ONLY']}",
        flush=True,
    )


def _arg_value(name: str, default=None):
    """--name <값> 형태 인자 파싱 (다음 토큰이 또 --플래그면 값 없음으로 본다)."""
    if name in sys.argv:
        i = sys.argv.index(name)
        if i + 1 < len(sys.argv) and not sys.argv[i + 1].startswith("--"):
            return sys.argv[i + 1]
    return default


# 씬 조립 후 USD 로 저장. --export <이름|경로> 주면 저장한다. 참조형 — Nucleus·로컬 에셋을
# 절대경로로 참조하므로 같은 GPU 셋업에서 열린다. --headless 와 같이 주면 저장만 하고 종료.
#   기본 저장 폴더 = ~/cobot3_ws. 파일명·상대경로는 여기에 붙는다(절대경로면 그대로).
#   --export 만 주면 scene.usd. 예) --export → ~/cobot3_ws/scene.usd
_EXPORT_DIR = os.path.expanduser("~/cobot3_ws")
if "--export" in sys.argv:
    _ename = _arg_value("--export", "scene.usd")
    EXPORT = _ename if os.path.isabs(_ename) else os.path.join(_EXPORT_DIR, _ename)
else:
    EXPORT = None

# 기존 USD 를 열어 그대로 실행(씬 재조립 안 함). --load <이름|경로>, 생략 시 scene.usd.
#   기본 폴더 ~/cobot3_ws. 텔레옵은 --mm-teleop과 같이 쓴다.
if "--load" in sys.argv:
    _lname = _arg_value("--load", "scene.usd")
    LOAD = _lname if os.path.isabs(_lname) else os.path.join(_EXPORT_DIR, _lname)
else:
    LOAD = None

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": not GUI})

if QUIET:
    # CAD 지그 USD(단위보정 레이어) 참조 시 omni.usd 가 "SetEditTarget ... metricsAssembler"
    # 스팸을 수백 줄 뿌린다(Kit 버그성 — 조립 순간에만, 무해). 옵트인으로만 끈다:
    # omni.usd 채널을 error 로 낮추면 다른 USD '경고'도 같이 숨으므로 기본은 켜 둔다(§8).
    import carb.settings
    carb.settings.get_settings().set("/log/channels/omni.usd", "error")

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

if not NO_ROS:
    # 브리지 확장은 그래프 생성 전에 켜야 한다 (tools/iwhub_bridge_check.py 검증 순서)
    # omni.graph.bundle/window.action = OmniGraph 액션 노드 안정화(팀원 hyeonminlee 확인, 2026-07-20)
    from isaacsim.core.utils.extensions import enable_extension
    # sensors.rtx + replicator: iw.hub RTX 라이다(LidarRtx)·렌더프로덕트 생성용(--nav-scan).
    for _ext in ("isaacsim.core.nodes", "isaacsim.ros2.bridge",
                 "omni.graph.bundle.action", "omni.graph.window.action",
                 "isaacsim.sensors.rtx", "omni.replicator.core"):
        enable_extension(_ext)
    for _ in range(20):
        simulation_app.update()

import numpy as np
import omni.usd
from isaacsim.core.api import World
from isaacsim.core.api.robots import Robot

from pjt_config.settings import SceneConfig
from scene.greenhouse_task import GreenhouseTask


class Opts:
    """드라이버 finalize/update 에 넘기는 실행 옵션 묶음 (모듈 플래그의 스냅샷)."""

    def __init__(self):
        self.no_ros = NO_ROS
        self.gui = GUI
        self.mm_teleop = MM_TELEOP
        self.rmpflow = RMPFLOW
        self.camera = CAMERA
        self.nav_drive = NAV_DRIVE
        self.nav_odom = NAV_ODOM
        self.nav_scan = NAV_SCAN


def build_drivers(cfg, task=None) -> list:
    """플래그로 고른 로봇 드라이버만 만든다. 드라이버는 **지연 import** — 안 고른 로봇 파일은
    아예 불러오지 않는다(팀원이 그 파일을 깨뜨려도 내 로봇은 돌아간다)."""
    drivers = []
    if "--mm" in sys.argv:
        from mm import MMDriver
        drivers.append(MMDriver(cfg, task=task))
    if "--iw" in sys.argv:
        if WAREHOUSE_TEST:                       # --iw --fork (--mm 없음) → 창고 상차 단독 시험
            from iw_test import IwDriver
            drivers.append(IwDriver(cfg, warehouse_test=True))
        else:                                    # 일반 통합 실행 — 깃허브용 iw.py(데크 적재)
            from iw import IwDriver
            drivers.append(IwDriver(cfg))
    if "--fork" in sys.argv:
        from fork import ForkDriver
        drivers.append(ForkDriver(cfg))
    # ★MoveIt MM(내 것, 2026-07-23) — build_drivers 끝에 추가해 f2(iw) 의 --mm/--iw 변경과
    #   영역이 안 겹치게 한다(머지 충돌 회피). --moveit → moveit_mm(/World/HarvesterMoveit,
    #   harvester_moveit). --mm 은 팀원 RMPflow(mm.py) 그대로 → --mm --moveit 동시 스폰 가능.
    if MOVEIT:
        from moveit_mm import MMDriver as MoveitMMDriver
        drivers.append(MoveitMMDriver(cfg, task=task))
    return drivers


def _build_clock() -> None:
    """공용 /clock — 로봇이 하나라도 있으면 한 번만. 실패해도 씬은 유지."""
    try:
        from ros import robot_bridge as RB
        RB.build_clock()
    except Exception:
        import traceback
        print("[RosBridge] /clock 생성 실패 — 계속 진행(로봇 브리지도 실패할 것)")
        traceback.print_exc()


def _assemble_robots(world, stage, drivers: list) -> None:
    """고른 로봇들을 생명주기 단계대로 조립한다. reset 순서만 전역이라 여기 남는다(§8).

    단계: spawn → register →(reset)→ configure →(reset+settle)→ finalize →(reset+settle).
    """
    for d in drivers:
        d.spawn(stage)
    for d in drivers:
        d.register(world, stage)
    world.reset()                                # 로봇 물리 초기화(뷰 준비)

    for d in drivers:
        d.configure(world)                       # 기본 관절자세
    world.reset()
    for _ in range(15):                          # 자세 정착(§8 — 안 하면 옛 자세 읽음)
        world.step(render=False)

    if not NO_ROS:
        _build_clock()                           # 공용 클럭 먼저(조인트 브리지 전)
    opts = Opts()
    for d in drivers:
        d.finalize(world, stage, opts)           # ROS 브리지·부가장치(자세 정착 뒤)
    # 힌지·카고 강체는 위 finalize 뒤에 추가돼 아직 물리 뷰에 없다 → 한 번 더 reset 해
    # 미리 초기화·정착시킨다. 안 하면 플레이 순간 첫 reset 에서 물리 자리로 스냅한다(§8).
    world.reset()
    for _ in range(5):
        world.step(render=False)


def run_loaded(path: str) -> None:
    """기존 USD 를 열어 그대로 실행(씬 재조립 없음) + MM 텔레옵(--mm 일 때).

    ★ 수확자세 재설정이 필요한 이유: 아티큘레이션 '조인트 각도 상태'는 USD 에 안 실린다.
      로드 시 wrist_1 이 0 으로 초기화돼 플레이하면 그리퍼가 0 자세로 떨어진다(사용자 지적
      2026-07-20). build 경로와 똑같이 wrist_1 +180° 를 default_state 로 다시 잡아 고정한다.
      (이 USD 가 이미 수확자세 상태를 담고 있었다면 이 +180° 는 빼야 함 — GPU 에서 확인.)
    """
    from isaacsim.core.utils.stage import open_stage

    from robot_base import art_root

    if not os.path.isfile(path):
        print(f"[Load] USD 없음: {path}\n  --export 로 먼저 저장하거나 경로를 확인하세요.")
        return
    print(f"[Load] USD 로드: {path}")
    open_stage(path)
    world = World(stage_units_in_meters=1.0)
    stage = omni.usd.get_context().get_stage()

    mm_robot = None
    for nm, root in (("mm", "/World/Harvester"), ("fk", "/World/Forklift"),
                     ("iw", "/World/IwHub")):
        art = art_root(stage, root)
        if art:
            r = world.scene.add(Robot(prim_path=art, name=nm))
            if nm == "mm":
                mm_robot = r
    world.reset()

    teleop = None
    if mm_robot is not None:
        q0 = np.asarray(mm_robot.get_joint_positions(), dtype=float)
        q0[list(mm_robot.dof_names).index("wrist_1_joint")] += np.pi   # 수확자세 복원
        mm_robot.set_joints_default_state(positions=q0)
        world.reset()
        for _ in range(15):
            world.step(render=False)
        if MM_TELEOP:
            from mm import build_teleop, find_blade_setter
            teleop = build_teleop(mm_robot, find_blade_setter(stage), GUI)
    else:
        print("[Load] Harvester 아티큘레이션을 못 찾음 — 텔레옵 불가.")

    if GUI:
        from isaacsim.core.utils.viewports import set_camera_view
        set_camera_view(eye=[10.0, -18.0, 12.0], target=[0.0, 2.0, 0.5])

    if not GUI:
        # 헤드리스엔 ▶Play 누를 사람이 없다 — 자동 재생. 안 하면 OnPlaybackTick 이
        # 영원히 안 틱해서 브리지(/clock·joint_states)가 침묵한다(2026-07-22 실측).
        world.play()

    was_playing = False
    while simulation_app.is_running():
        world.step(render=True)
        is_playing = world.is_playing()
        if is_playing and not was_playing:
            world.reset()
        was_playing = is_playing
        if teleop is not None:
            teleop(is_playing)


def _measure_grasp_center(stage, mm) -> None:
    """실제 손가락 패드 중점(=진짜 파지중심)을 HarvestTCP 기준으로 실측 → URDF harvest_tcp 교정.
    HarvestTCP 프레임은 tool0 방향과 일치(harvester.py 가 rot 없이 정의)하므로, 여기서 나온
    오프셋을 그대로 URDF harvest_tcp origin(0,0,0.127)에 더하면 파지중심이 맞는다.
    버퍼링 우회: 파일(GRASP_MEAS_FILE)에 flush 기록. 개별 패드 위치도 남겨 그리퍼 축 확인."""
    from pxr import Gf, Usd, UsdGeom
    cache = UsdGeom.XformCache()
    tcp = None
    for p in stage.Traverse():
        if p.GetName() == "HarvestTCP":
            tcp = p; break
    if tcp is None:
        print("[AirFruit] HarvestTCP 못 찾음 — 파지중심 측정 생략", flush=True); return
    Tinv = cache.GetLocalToWorldTransform(tcp).GetInverse()
    # ★ 링크 원점(=너클)이 아니라 손가락 PAD 콜라이더 메시의 월드 bbox 중심(실제 접촉면)을
    #   쓴다 — 접근축(Z) 높이가 여기서 정해진다(2026-07-22, 너클 -0.115 는 접촉면 아님).
    bbc = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                            [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    fps = {}
    for p in stage.Traverse():
        if p.GetName() in ("left_inner_finger", "right_inner_finger"):
            r = bbc.ComputeWorldBound(p).ComputeAlignedRange()
            if r.IsEmpty():
                continue
            c = r.GetMidpoint()
            fps[p.GetName()] = Gf.Vec3d(Tinv.Transform(Gf.Vec3d(c)))
    li = fps.get("left_inner_finger")
    ri = fps.get("right_inner_finger")
    lines = ["# 손가락 패드 위치 (HarvestTCP 로컬프레임, tool0 방향). 단위 m."]
    for nm, c in fps.items():
        lines.append(f"{nm:26s} ({c[0]:+.4f}, {c[1]:+.4f}, {c[2]:+.4f})")
    if li is not None and ri is not None:
        mid = (li + ri) * 0.5
        span = (li - ri)                      # 두 패드 사이 벡터 = 그리퍼 닫힘축
        lines.append("")
        lines.append(f"GRASP_CENTER_offset  ({mid[0]:+.4f}, {mid[1]:+.4f}, {mid[2]:+.4f})  "
                     f"# HarvestTCP→실제 파지중심. URDF harvest_tcp 에 이만큼 더할 것")
        lines.append(f"CLOSE_AXIS_span      ({span[0]:+.4f}, {span[1]:+.4f}, {span[2]:+.4f})  "
                     f"# 좌-우 패드 벡터 = 그리퍼 닫힘축 방향/거리")
        lines.append(f"# 새 harvest_tcp origin = (0,0,0.127) + GRASP_CENTER_offset "
                     f"= ({mid[0]:+.4f}, {mid[1]:+.4f}, {0.127+mid[2]:+.4f})")
    text = "\n".join(lines) + "\n"
    try:
        with open(GRASP_MEAS_FILE, "w") as f:
            f.write(text)
    except OSError as exc:
        print(f"[AirFruit] 측정파일 기록 실패: {exc}", flush=True)
    print("[AirFruit] ★파지중심 측정 →\n" + text, flush=True)
    _dump_gripper_structure(stage, tcp)


def _dump_gripper_structure(stage, tcp_prim) -> None:
    """그리퍼(Robotiq 2F-85) 서브트리의 각 링크 로컬변환 + 메시참조를 덤프.
    HarvestTCP 로컬프레임(=URDF harvest_tcp 방향) 기준 → URDF 손가락 링키지를 실제 배치로
    재구성해 RViz 를 Isaac 과 일치시키기 위함(사용자 요청 2026-07-22). 버퍼링 우회=파일."""
    from pxr import Gf, UsdGeom, UsdPhysics
    cache = UsdGeom.XformCache()
    Tinv = cache.GetLocalToWorldTransform(tcp_prim).GetInverse()
    lines = ["# Robotiq 2F-85 링크 구조 (HarvestTCP 로컬프레임, tool0 방향, 단위 m).",
             "# 열: 프리즘경로 | pos(x,y,z) | 메시에셋(있으면)"]
    for p in stage.Traverse():
        path = str(p.GetPath())
        if "Robotiq" not in path and "Gripper" not in path:
            continue
        tp = p.GetTypeName()
        if tp not in ("Xform", "Mesh"):
            continue
        m = cache.GetLocalToWorldTransform(p)
        pos = Gf.Vec3d(Tinv.Transform(m.ExtractTranslation()))
        mesh = ""
        try:                                   # 메시 파일 참조(references 메타데이터)
            refs = p.GetMetadata("references")
            if refs and getattr(refs, "prependedItems", None):
                mesh = ";".join(str(i.assetPath) for i in refs.prependedItems)
        except Exception:
            pass
        # 물리 콜라이더 판별용 — 적용 스키마(Collision/Physx 만 표시)
        schemas = [s for s in p.GetAppliedSchemas()
                   if "Colli" in s or "Physx" in s or "RigidBody" in s or "Material" in s]
        sch = ("[" + ",".join(schemas) + "]") if schemas else ""
        lines.append(f"{path:70s} ({pos[0]:+.4f},{pos[1]:+.4f},{pos[2]:+.4f}) {sch} {mesh}")
    # --- 조인트(RevoluteJoint): 손가락 링키지 피벗·축 (URDF 재구성용) ---
    lines.append("")
    lines.append("# --- RevoluteJoint (parent->child, axis, localPos0=부모기준피벗, localPos1=자식기준피벗) ---")
    for p in stage.Traverse():
        path = str(p.GetPath())
        if "Robotiq" not in path or not p.IsA(UsdPhysics.RevoluteJoint):
            continue
        j = UsdPhysics.RevoluteJoint(p)
        b0 = j.GetBody0Rel().GetTargets(); b1 = j.GetBody1Rel().GetTargets()
        b0s = str(b0[0]).split("/")[-1] if b0 else "?"
        b1s = str(b1[0]).split("/")[-1] if b1 else "?"
        lp0 = j.GetLocalPos0Attr().Get(); lp1 = j.GetLocalPos1Attr().Get()
        lr0 = j.GetLocalRot0Attr().Get(); lr1 = j.GetLocalRot1Attr().Get()
        axis = j.GetAxisAttr().Get()
        lines.append(f"JOINT {p.GetName():28s} {b0s}->{b1s} axis={axis} "
                     f"lp0={lp0} lp1={lp1} lr0={lr0} lr1={lr1}")
    text = "\n".join(lines) + "\n"
    try:
        with open(GRIPPER_DUMP_FILE, "w") as f:
            f.write(text)
        print(f"[AirFruit] 그리퍼 구조 덤프 → {GRIPPER_DUMP_FILE} ({len(lines)-2} prim)", flush=True)
    except OSError as exc:
        print(f"[AirFruit] 그리퍼 덤프 기록 실패: {exc}", flush=True)


def _spawn_one_airfruit(stage, world_pos, idx: int) -> str:
    """공중 과실 1개 스폰(실제 토마토 USD Body + 중심 충돌구 + 그립 줄기 원통). 인덱스별
    독립 머티리얼(/World/PM/airfruit_i, airstem_i)로 케이스마다 마찰을 따로 스윕. 반환: prim 경로."""
    from pxr import Gf, Usd, UsdGeom
    from isaacsim.core.utils.stage import add_reference_to_stage
    from scene.physics import (add_sphere_collider, add_rigid_body, add_cylinder_collider,
                               create_physics_material, bind_physics_material)
    S = 0.001675                                       # 씬 토마토 스케일(settings)
    body_usd = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                            "assets", "tomato", "tomato_ripe_03.usd")
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
    add_sphere_collider(stage, path + "/Collision", 0.025 / S,
                        center=(cl[0], cl[1], cl[2]))
    add_rigid_body(fruit.GetPrim(), 1000.0, kinematic=True)
    bind_physics_material(fruit.GetPrim(),
                          create_physics_material(stage, f"/World/PM/airfruit_{idx}", 0.9, 0.7))
    _up = (0.025 + 0.025) / S                              # 과실 중심 위(로컬)
    add_cylinder_collider(stage, path + "/GripStem", 0.005 / S, 0.05 / S,
                          center=(cl[0], cl[1], cl[2] + _up), visible=True)
    bind_physics_material(stage.GetPrimAtPath(path + "/GripStem"),
                          create_physics_material(stage, f"/World/PM/airstem_{idx}", 0.9, 0.7))
    return path


def _setup_air_fruit(stage, drivers) -> None:
    """nav 없이 팔 앞 도달권에 **실제 토마토 USD** 과실을 **여러 개(가로 한 줄)** 띄운다.
    스윕 스파이크가 케이스마다 fresh 과실을 잡게 해 reset 반복 상태오염을 회피(사용자 지적
    2026-07-22 "처음빼고 상태이상". "매번 스폰하지말고 한번에 여러개"). 개수·간격은 env."""
    from pxr import Gf, UsdGeom
    # ★name 분리 대응(2026-07-23, Codex): moveit_mm 은 name="mm_moveit" 이므로 "mm" 로만
    #   찾으면 --moveit --airfruit 에서 못 찾는다. name 이 "mm" 로 시작하는 MM 계열을 잡는다.
    mm = next((d for d in drivers if getattr(d, "name", "").startswith("mm")), None)
    if mm is None:
        print("[AirFruit] MM 드라이버 없음 — --mm 또는 --moveit 필요"); return
    base = stage.GetPrimAtPath(f"{mm.root}/Base/base_link")
    if not base.IsValid():
        print("[AirFruit] base_link 못 찾음"); return
    n = int(os.environ.get("AIRFRUIT_N", "6"))
    dy = float(os.environ.get("AIRFRUIT_DY", "0.07"))     # 과실 간 가로 간격(섀시 Y, m)
    cache = UsdGeom.XformCache()
    b2w = cache.GetLocalToWorldTransform(base)
    paths = []
    for i in range(n):
        y = (i - (n - 1) / 2.0) * dy                       # 중앙 대칭 한 줄
        world_pos = b2w.Transform(Gf.Vec3d(0.6, y, 1.0))
        paths.append(_spawn_one_airfruit(stage, world_pos, i))
    mm.set_air_fruits(paths)
    print(f"[AirFruit] 실제 토마토 {n}개 스폰 — 섀시 (0.6, ±, 1.0) 가로 한 줄 dy={dy} "
          f"(각 Body USD + 충돌구 + 그립줄기, 인덱스별 μ 머티리얼)", flush=True)
    _measure_grasp_center(stage, mm)


def main() -> None:
    if LOAD:                                          # USD 로드 모드 (씬 재조립 안 함)
        run_loaded(LOAD)
        return
    cfg = SceneConfig()
    world = World(stage_units_in_meters=1.0)

    task = GreenhouseTask(name="greenhouse", cfg=cfg)
    world.add_task(task)
    world.reset()                                # 씬 생성

    # ── 로봇: 플래그로 고른 것만 (없으면 환경만) ──
    stage = omni.usd.get_context().get_stage()
    drivers = build_drivers(cfg, task=task)
    if drivers:
        _assemble_robots(world, stage, drivers)
        if AIRFRUIT:
            _setup_air_fruit(stage, drivers)
    else:
        print("[Main] 로봇 플래그 없음 — 환경(씬)만 띄운다. "
              "(--mm / --iw / --fork 로 로봇 선택)")

    if GUI:
        from isaacsim.core.utils.viewports import set_camera_view
        g = cfg.greenhouse
        set_camera_view(eye=[g.width * 0.9, -g.length * 0.8, 12.0],
                        target=[0.0, 2.0, 0.5])

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
    if drivers and not NO_ROS:
        print("[RosBridge] 대기 중 — 토픽 목록은 파일 상단 docstring 참조 (domain 108)\n")

    if EXPORT:
        ok = stage.Export(EXPORT)
        print(f"\n[Export] 씬 USD 저장 → {EXPORT}  ({'성공' if ok else '실패'})")
        print("  ※ 참조형(절대경로) — Isaac GUI 로 열면 같은 씬. Nucleus 접속 필요.")
        if not GUI:
            return                               # 헤드리스면 저장만 하고 종료(아래 close)

    if not GUI:
        # 헤드리스엔 ▶Play 누를 사람이 없다 — 자동 재생. 안 하면 OnPlaybackTick 이
        # 안 틱해서 브리지(/clock·joint_states)가 침묵한다(2026-07-22 실측).
        world.play()

    # Play/Stop 반복 시 동일한 초기 상태에서 재시작 (재현성)
    was_playing = False
    while simulation_app.is_running():
        # 최상위 독립 강체인 커터 날은 첫 물리 스텝 전에 그리퍼 위치로 맞춰야 한다.
        # 기존 순서는 step 후 reset이라 Play 첫 프레임에 힌지가 날을 순간 가속했다.
        pre_playing = world.is_playing()
        if pre_playing and not was_playing:
            for d in drivers:
                d.update(False)
            world.reset()
            for d in drivers:
                d.update(False)
        world.step(render=True)
        is_playing = world.is_playing()
        was_playing = is_playing
        for d in drivers:                        # 로봇별 매 프레임(텔레옵·JSON 명령)
            d.update(is_playing)


try:
    main()
finally:
    # 예외·Ctrl-C 경로에서도 Kit/OmniGraph를 Python 종료 전에 먼저 정리한다.
    simulation_app.close()
