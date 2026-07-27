# -*- coding: utf-8 -*-
"""iw.hub Nav2 풀스택 — 베이스 + 정적맵 AMCL + Nav2 네비게이션.

구성(§5.6: Isaac 은 실행/센서, ROS2 는 판단):
  Isaac  : main.py --iw --nav-odom --nav-scan
           → RPLIDAR S2E 전·후방 스캔 + 실제 chassis /odom·TF + /clock
  base_node : /cmd_vel→바퀴만 담당(바퀴 각도 odom은 비활성)
  AMCL : 정적맵과 /front_2d_lidar/scan을 매칭해 map→odom 잔여 오차 보정
  nav2 navigation : controller/planner/bt/behaviors/smoother (코스트맵은 스캔 2개 다 씀)
TF: map(AMCL)→odom(Isaac chassis)→base_link→chassis→front/back_2d_lidar

필요 패키지(dev PC): ros-humble-navigation2 ros-humble-nav2-bringup
실행: ros2 launch iwhub_control iwhub_nav2.launch.py
  (도메인은 Isaac 과 동일하게 export — 예 ROS_DOMAIN_ID=109. rviz2 로 목표점 찍어 주행 확인)
"""
import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace

from fleet_dispatch.nav2_bringup_compat import (
    convert_plugin_names_for_distro,
    remove_docking_server,
)


def _deferred_ns(namespace: str) -> list:
    """TimerAction 안쪽에서 다시 걸어야 하는 PushRosNamespace (Humble 전용).

    배포판마다 지연 액션의 네임스페이스 문맥 처리가 다르다.

    Humble — `GroupAction`(scoped=True)은 **자식 액션을 훑은 시점**에 스코프를
      닫는다. `TimerAction`은 예약만 하고 즉시 반환하므로, 정작 타이머가 발화할
      때는 바깥 `PushRosNamespace`가 이미 pop된 뒤다. 그래서 nav2 노드가
      네임스페이스 없이 `/map_server`, `/controller_server`로 뜨고, YAML 루트 키
      (`map_server:` …)와 실제 노드 경로가 어긋나 **파라미터가 하나도 안 먹는다**
      (2026-07-27 실측: `yaml_filename is not initialized`,
      `No critics defined for FollowPath` — 둘 다 파라미터 미적용이 원인).

    Jazzy — 지연 액션이 문맥을 잡아두므로 여기서 또 push 하면 `/iwhub_0/iwhub_0`
      로 겹친다(7a5951b 에서 실측·수정된 내용). 그쪽은 건드리지 않는다.
    """
    if os.environ.get("ROS_DISTRO", "") == "humble":
        return [PushRosNamespace(namespace)]
    return []


def _navigation_without_charging_dock(nav2_bringup: str) -> str:
    """IW 물류 도킹과 무관한 Nav2 충전 docking_server를 제외한 launch 경로."""
    source_path = os.path.join(nav2_bringup, "launch", "navigation_launch.py")
    with open(source_path, encoding="utf-8") as stream:
        source = stream.read()
    fixed = remove_docking_server(source)
    if fixed == source:
        return source_path
    tmp = tempfile.NamedTemporaryFile(
        mode="w", suffix="_iw_navigation.launch.py", delete=False,
        encoding="utf-8")
    tmp.write(fixed)
    tmp.close()
    return tmp.name


def generate_launch_description():
    pkg = get_package_share_directory("iwhub_control")
    nav2_bringup = get_package_share_directory("nav2_bringup")
    navigation_launch = _navigation_without_charging_dock(nav2_bringup)
    # nav2_params.yaml은 아직 Humble 표기(nav2_navfn_planner/NavfnPlanner,
    # nav2_behaviors/Spin 등)라 Jazzy에서 그대로 주면 pluginlib이 "does not
    # exist"로 planner_server를 못 띄우고, 그 lifecycle 실패가 costmap을 포함한
    # navigation bringup 전체를 중단시킨다(2026-07-27 GPU 실측 — /iwhub_0/map은
    # 정상 발행되는데 global_costmap이 cleaningup에 멈춰 RViz에 안 뜨는 걸로
    # 관찰됨). harvester_nav2.launch.py와 같은 공용 변환 함수로 배포판별 분기.
    nav2_params = convert_plugin_names_for_distro(
        os.path.join(pkg, "config", "nav2_params.yaml"))
    # IW 전용 월드 정렬 맵.
    default_map = os.path.join(pkg, "maps", "greenhouse.yaml")
    map_yaml = LaunchConfiguration("map")
    use_sim_time = LaunchConfiguration("use_sim_time")
    rviz = LaunchConfiguration("rviz")
    nav_autostart = LaunchConfiguration("nav_autostart")
    namespace = "iwhub_0"
    tf_remaps = [("/tf", "tf"), ("/tf_static", "tf_static")]
    with open(os.path.join(pkg, "urdf", "iwhub.urdf")) as urdf_file:
        robot_desc = urdf_file.read()

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true",
                              description="Isaac /clock 사용(시뮬)"),
        DeclareLaunchArgument(
            "map", default_value=default_map,
            description="IW가 사용할 정적 map yaml"),
        DeclareLaunchArgument(
            "rviz", default_value="true",
            description="IW 전용 RViz 실행 여부"),
        DeclareLaunchArgument(
            "nav_autostart", default_value="false",
            description="Nav2 navigation lifecycle 즉시 활성화 여부"),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            namespace=namespace,
            remappings=tf_remaps,
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "robot_description": robot_desc,
                "frame_prefix": f"{namespace}/",
            }],
        ),

        # 1. 베이스: cmd_vel→바퀴. odom은 Isaac의 실제 chassis 자세를 사용한다.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg, "launch", "iwhub_base.launch.py")),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "publish_odom": "false",
            }.items(),
        ),

        # 팔레트/KLT가 전·후방 360° 라이다에 자기 장애물로 보이지 않도록
        # base_link의 실제 적재 footprint 내부 점만 제거한다.
        Node(
            package="iwhub_control",
            executable="scan_self_filter_node",
            name="scan_self_filter",
            namespace=namespace,
            output="screen",
            parameters=[{"use_sim_time": use_sim_time}],
        ),

        # 2~3. Humble의 개별 localization/navigation launch는 namespace 인자를
        # 파라미터 root_key에만 쓰고 PushRosNamespace는 하지 않는다. 여기서 두 스택을
        # 명시적으로 감싸야 노드·액션·costmap·TF가 MM의 전역 Nav2와 분리된다.
        GroupAction(actions=[
            PushRosNamespace(namespace),
            # localization(map_server+amcl)도 즉시 뜨면 MM composed Nav2 플러그인
            # 로딩과 lifecycle activate가 겹쳐 FastDDS service 응답이 유실되고
            # map_server가 activate 실패(맵 미발행)한다. navigation과 동일하게 지연해
            # MM 로딩 버스트 뒤에 뜨게 하고, navigation(3s)보다 먼저 올린다.
            TimerAction(
                period=2.0,
                actions=[
                    GroupAction(actions=_deferred_ns(namespace) + [
                        # 네임스페이스 재적용은 Humble 에서만 필요하다 — 이유는
                        # _deferred_ns() 주석 참조. Jazzy 는 빈 리스트라 종전과 동일.
                        IncludeLaunchDescription(
                            PythonLaunchDescriptionSource(os.path.join(
                                nav2_bringup, "launch", "localization_launch.py")),
                            launch_arguments={
                                "namespace": namespace,
                                "use_sim_time": use_sim_time,
                                "params_file": nav2_params,
                                "map": map_yaml,
                                # 상위 MM Nav2의 use_composition 값이 중첩 launch로
                                # 전파되면 존재하지 않는 iwhub_0/nav2_container를 기다린다.
                                "use_composition": "False",
                            }.items(),
                        ),
                        # localization autostart는 FastDDS 응답 유실 시 map_server가
                        # activate에서 멈춰 /iwhub_0/map 이 안 나간다. activator가 실제
                        # state를 다시 읽으며 configure/activate를 재시도해 확실히
                        # 활성화한다(여기선 localization=map_server+amcl만; navigation은
                        # mission_nav STARTUP이 담당). PushRosNamespace로 iwhub_0 적용됨.
                        Node(
                            package="fleet_dispatch",
                            executable="nav2_lifecycle_activator",
                            name="iw_localization_activator",
                            output="screen",
                            parameters=[{
                                "targets": ["map_server", "amcl"],
                                "startup_delay_sec": 3.0,
                            }],
                        ),
                    ]),
                ],
            ),
            # 통합 launch에서 MM composed Nav2 플러그인 로딩과 IW DWB lifecycle
            # configure가 동시에 겹치면 FastDDS service 응답이 유실될 수 있다.
            # MM 로딩이 끝난 뒤 IW navigation lifecycle을 시작한다.
            TimerAction(
                period=3.0,
                actions=[
                    # 네임스페이스 재적용은 Humble 에서만 — _deferred_ns() 주석 참조.
                    GroupAction(actions=_deferred_ns(namespace) + [
                        # navigation lifecycle 복구. localization 과 달리 여기엔
                        # 재시도 주체가 없어서, 기동 순간 Isaac 의 odom TF 가 아직
                        # 안 올라와 있으면 costmap configure 가 실패하고
                        # lifecycle_manager 가 bringup 을 중단한 채 그대로 굳는다
                        # (2026-07-27 재현: controller/planner=inactive,
                        #  bt_navigator/behavior=unconfigured, costmap 미발행).
                        # TF 는 몇 초 뒤 정상이 되므로 상태를 다시 읽고 재시도하면
                        # 확실히 올라온다. autostart 가 먼저 시도할 시간을 주려고
                        # startup_delay 를 navigation 지연(3s)보다 넉넉히 뒤에 둔다.
                        Node(
                            package="fleet_dispatch",
                            executable="nav2_lifecycle_activator",
                            name="iw_navigation_activator",
                            output="screen",
                            parameters=[{
                                "targets": [
                                    "controller_server", "smoother_server",
                                    "planner_server", "behavior_server",
                                    "bt_navigator", "waypoint_follower",
                                    "velocity_smoother",
                                ],
                                "startup_delay_sec": 12.0,
                            }],
                        ),
                        IncludeLaunchDescription(
                            PythonLaunchDescriptionSource(navigation_launch),
                            launch_arguments={
                                "namespace": namespace,
                                "use_sim_time": use_sim_time,
                                "params_file": nav2_params,
                                "use_composition": "False",
                                # mission_nav_node가 모든 프로세스 로드 후 STARTUP하고
                                # action 서버가 없으면 자동 재시도한다.
                                "autostart": nav_autostart,
                            }.items(),
                        ),
                    ]),
                ],
            ),
        ]),

        # 4. rviz2 (맵·스캔·경로 + Nav2 Goal 툴). 이 launch 로 같이 뜬다.
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            namespace=namespace,
            remappings=tf_remaps,
            output="screen",
            arguments=["-d", os.path.join(pkg, "config", "iwhub_nav2.rviz")],
            parameters=[{"use_sim_time": use_sim_time}],
            condition=IfCondition(rviz),
        ),
    ])
