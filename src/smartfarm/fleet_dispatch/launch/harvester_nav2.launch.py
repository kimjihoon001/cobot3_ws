# 수확 MM(Ridgeback) Nav2 실행 — nav2_bringup 을 config/harvester_nav2.yaml 로 띄운다.
#
# 사용:
#   ros2 launch fleet_dispatch harvester_nav2.launch.py slam:=true          # 맵 만들며 주행(수동)
#   ros2 launch fleet_dispatch harvester_nav2.launch.py slam:=true explore:=true  # 자동 탐사 맵핑
#   ros2 launch fleet_dispatch harvester_nav2.launch.py map:=/경로/farm.yaml  # 정적맵 + AMCL
#
# explore:=true 는 explore_lite(m-explore-ros2, src/m-explore-ros2) 를 같이 띄운다 —
# slam_toolbox 는 들어오는 스캔으로 지도만 채울 뿐 로봇을 몰지 않으므로, 이게 없으면
# slam:=true 여도 사람이 teleop 으로 돌아다녀야 한다. explore_lite 가 /harvester_0/map
# 의 미탐사 경계(frontier)를 찾아 navigate_to_pose 액션으로 계속 목표를 보낸다.
# 패키지 기본 파라미터(costmap_topic: map, robot_base_frame: base_link)가 이 프로젝트
# 프레임/토픽 이름과 그대로 맞아 별도 params 파일 없이 씀.
#
# Isaac 쪽 짝: isaac_python main.py --mm --nav  (isaacpjt/mm.py::build_nav)
#   /scan · /odom · TF 를 Isaac 이 발행하고,
#   /cmd_vel 을 Isaac 이 구독해 MM 베이스 위치로 적분한다.
#
# 왜 nav2_bringup 을 include 하나 — Carter(carter_navigation)도 이 구조다. 노드 9개를
# 직접 띄우면 distro 마다 구성이 달라 깨진다. 조합·라이프사이클 관리는 bringup 에 맡긴다.
#
# cmd_vel 최종 출력이 어느 토픽인가 (헷갈리는 지점 — Humble 1.1.20 navigation_launch.py 실측):
#   controller_server : cmd_vel        → 리맵 → cmd_vel_nav
#   velocity_smoother : cmd_vel(입력)  → 리맵 → cmd_vel_nav
#                       cmd_vel_smoothed(출력) → 리맵 → **cmd_vel**
#   즉 로봇이 구독해야 할 최종 토픽은 cmd_vel = /cmd_vel 이고, 이는
#   settings.HarvesterNavConfig.cmd_vel_topic 과 일치한다. (Humble 1.1.20 의
#   navigation_launch.py 에는 collision_monitor 가 없다. 추가된 distro 에서는 그놈이
#   마지막 단이 되지만 출력 토픽 이름은 역시 cmd_vel 이라 로봇 쪽은 안 바뀐다.)
#
# ⚠ 이 워크스페이스엔 nav2 가 없다(2026-07-20 확인). 먼저:
#     sudo apt install ros-$ROS_DISTRO-navigation2 ros-$ROS_DISTRO-nav2-bringup

import os
import re
import tempfile

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Humble 표기 → Jazzy 이상 표기. 값에 '/' 가 들어가는 플러그인은 navfn 과 behaviors 뿐이고
# (costmap·controller·smoother 플러그인은 Humble 도 이미 '::'), 키가 정확히 `plugin` 인
# 줄만 건드리므로 map 경로·BT xml 경로 같은 다른 '/' 는 안 다친다.
_PLUGIN_LINE = re.compile(r'(plugin:\s*")(\w+)/(\w+)(")')


def _params_for_distro(params_file: str) -> str:
    """distro 에 맞게 플러그인 표기를 고친 파라미터 파일 경로를 돌려준다.

    nav2 는 Iron 에서 플러그인 클래스 이름을 `pkg/Class` → `pkg::Class` 로 바꿨다.
    표기가 틀리면 pluginlib 이 못 찾아 해당 서버가 아예 안 뜬다. 파일을 두 벌 두면
    한쪽만 고치는 사고가 나므로, 원본 하나를 두고 실행 시점에 변환한다.
    """
    if os.environ.get("ROS_DISTRO", "") == "humble":
        return params_file
    with open(params_file, encoding="utf-8") as f:
        text = f.read()
    fixed = _PLUGIN_LINE.sub(r"\1\2::\3\4", text)
    if fixed == text:
        return params_file
    tmp = tempfile.NamedTemporaryFile(mode="w", suffix=".yaml", delete=False,
                                      encoding="utf-8")
    tmp.write(fixed)
    tmp.close()
    return tmp.name


def _pybool(context, name: str) -> str:
    """불리언 인자를 파이썬 리터럴('True'/'False')로 정규화한다.

    왜 필요한가 — bringup_launch.py 는 `IfCondition(PythonExpression(['not ', slam]))`
    로 **파이썬 eval** 을 한다. 소문자 'true'/'false' 를 넘기면 eval("not false") 가
    NameError 로 죽는다(2026-07-20 실측: "name 'false' is not defined"). nav2 자신의
    기본값도 대문자 'False' 다. IfCondition 은 대소문자를 다 받으므로 전부 대문자로 통일.
    """
    return str(LaunchConfiguration(name).perform(context).strip().lower()
               in ("true", "1", "yes", "on"))


def _params_with_initial_pose(context, params_file: str) -> str:
    """정적 맵 실행 시 AMCL 초기 위치를 launch 인자로 주입한다."""
    if (_pybool(context, "slam") == "True" or
            _pybool(context, "set_initial_pose") != "True"):
        return params_file

    with open(params_file, encoding="utf-8") as stream:
        params = yaml.safe_load(stream)
    amcl = params.setdefault("amcl", {}).setdefault("ros__parameters", {})
    amcl["set_initial_pose"] = True
    amcl["initial_pose"] = {
        "x": float(LaunchConfiguration("initial_pose_x").perform(context)),
        "y": float(LaunchConfiguration("initial_pose_y").perform(context)),
        "z": 0.0,
        "yaw": float(LaunchConfiguration("initial_pose_yaw").perform(context)),
    }
    tmp = tempfile.NamedTemporaryFile(mode="w", suffix=".yaml", delete=False,
                                      encoding="utf-8")
    yaml.safe_dump(params, tmp, sort_keys=False)
    tmp.close()
    return tmp.name


def _params_with_forward_only_bt(params_file: str) -> str:
    """후진 복구가 없는 NavigateToPose BT를 런타임 파라미터에 주입한다."""
    with open(params_file, encoding="utf-8") as stream:
        params = yaml.safe_load(stream)
    navigator = params.setdefault("bt_navigator", {}).setdefault(
        "ros__parameters", {})
    navigator["default_nav_to_pose_bt_xml"] = os.path.join(
        get_package_share_directory("fleet_dispatch"), "behavior_trees",
        "navigate_to_pose_forward_only.xml")
    tmp = tempfile.NamedTemporaryFile(mode="w", suffix=".yaml", delete=False,
                                      encoding="utf-8")
    yaml.safe_dump(params, tmp, sort_keys=False)
    tmp.close()
    return tmp.name


def _bringup(context, *_args, **_kwargs):
    distro = os.environ.get("ROS_DISTRO", "")
    params_file = _params_for_distro(
        LaunchConfiguration("params_file").perform(context))
    params_file = _params_with_initial_pose(context, params_file)
    params_file = _params_with_forward_only_bt(params_file)
    args = {
        "namespace": LaunchConfiguration("namespace").perform(context),
        "slam": _pybool(context, "slam"),
        "map": LaunchConfiguration("map").perform(context),
        "use_sim_time": _pybool(context, "use_sim_time"),
        "params_file": params_file,
        "autostart": _pybool(context, "autostart"),
    }
    # use_namespace 는 Humble 에만 있는 인자다 (Iron 에서 제거 — namespace 하나로 통합).
    # Jazzy 에 넘기면 "unknown launch argument" 로 죽는다.
    if distro == "humble":
        args["use_namespace"] = str(bool(args["namespace"]))
    actions = [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("nav2_bringup"), "launch",
            "bringup_launch.py")),
        launch_arguments=args.items())]

    # Isaac MoveIt MM의 ROS2SubscribeTwist는 마지막 값을 영구 유지한다. Nav2 목표 종료나
    # 프로세스 이상 시 계속 흘러가지 않도록 같은 namespace에서 안전 토픽을 항상 만든다.
    actions.append(Node(
        package="fleet_dispatch", executable="cmd_vel_watchdog",
        name="cmd_vel_watchdog", namespace=args["namespace"],
        parameters=[{
            "use_sim_time": args["use_sim_time"] == "True",
            "input_topic": "cmd_vel",
            "output_topic": "cmd_vel_safe",
            "timeout_sec": 0.35,
            "publish_rate_hz": 20.0,
        }],
        output="screen"))

    # RViz 는 기본으로 같이 띄운다(rviz:=false 로 끌 수 있음).
    # ★ use_sim_time 을 반드시 넘겨야 한다 — Isaac 은 타임스탬프를 시뮬 시간(수백 초)으로
    #   찍는데 RViz 가 벽시계(17억 초)로 보면 모든 메시지를 '너무 오래됨' 으로 버려서
    #   화면에 아무것도 안 나온다(2026-07-20 실사용 확인). 이건 실행 중 못 바꾸는 값이라
    #   손으로 띄우면 매번 --ros-args -p use_sim_time:=true 를 붙여야 했다.
    if _pybool(context, "rviz") == "True":
        actions.append(Node(
            package="rviz2", executable="rviz2", name="rviz2",
            namespace=args["namespace"],
            arguments=["-d", LaunchConfiguration("rviz_config").perform(context)],
            parameters=[{"use_sim_time": args["use_sim_time"] == "True"}],
            remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
            output="screen"))
    return actions


def _explore(context, *_args, **_kwargs):
    if _pybool(context, "explore") != "True":
        return []
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("explore_lite"), "launch",
            "explore.launch.py")),
        launch_arguments={
            "namespace": LaunchConfiguration("namespace").perform(context),
            "use_sim_time": _pybool(context, "use_sim_time"),
        }.items())]


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory("fleet_dispatch"), "config",
        "harvester_nav2.yaml")
    default_map = os.path.join(
        get_package_share_directory("fleet_dispatch"), "maps",
        "farm_gen.yaml")
    return LaunchDescription([
        # 현재 MM은 전역 /tf, /scan, /odom, /cmd_vel을 쓰므로 namespace 기본값은 비운다.
        DeclareLaunchArgument("namespace", default_value=""),
        DeclareLaunchArgument("slam", default_value="false"),
        DeclareLaunchArgument("explore", default_value="false"),
        DeclareLaunchArgument("map", default_value=default_map),
        DeclareLaunchArgument("use_sim_time", default_value="true"),  # Isaac /clock
        DeclareLaunchArgument("params_file", default_value=default_params),
        DeclareLaunchArgument("autostart", default_value="true"),
        # map=Isaac 월드 프레임. 새 Isaac 실행의 MM 스폰과 같은 베이스 자세.
        # 주행 후 Nav2만 재시작할 때는 현재 위치를 launch 인자로 덮어쓴다.
        DeclareLaunchArgument("set_initial_pose", default_value="true"),
        DeclareLaunchArgument("initial_pose_x", default_value="-3.3"),
        DeclareLaunchArgument("initial_pose_y", default_value="-9.77"),
        DeclareLaunchArgument("initial_pose_yaw", default_value="3.141592653589793"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("rviz_config", default_value=os.path.join(
            get_package_share_directory("fleet_dispatch"), "rviz",
            "harvester_nav2.rviz")),
        OpaqueFunction(function=_bringup),
        OpaqueFunction(function=_explore),
    ])
