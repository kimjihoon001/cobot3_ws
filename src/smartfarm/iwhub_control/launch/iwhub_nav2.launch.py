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


def generate_launch_description():
    pkg = get_package_share_directory("iwhub_control")
    nav2_bringup = get_package_share_directory("nav2_bringup")
    nav2_params = os.path.join(pkg, "config", "nav2_params.yaml")
    # IW 전용 월드 정렬 맵.
    default_map = os.path.join(pkg, "maps", "greenhouse.yaml")
    map_yaml = LaunchConfiguration("map")
    use_sim_time = LaunchConfiguration("use_sim_time")
    rviz = LaunchConfiguration("rviz")
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

        # 2~3. Humble의 개별 localization/navigation launch는 namespace 인자를
        # 파라미터 root_key에만 쓰고 PushRosNamespace는 하지 않는다. 여기서 두 스택을
        # 명시적으로 감싸야 노드·액션·costmap·TF가 MM의 전역 Nav2와 분리된다.
        GroupAction(actions=[
            PushRosNamespace(namespace),
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
            # 통합 launch에서 MM composed Nav2 플러그인 로딩과 IW DWB lifecycle
            # configure가 동시에 겹치면 FastDDS service 응답이 유실될 수 있다.
            # MM 로딩이 끝난 뒤 IW navigation lifecycle을 시작한다.
            TimerAction(
                period=3.0,
                actions=[
                    # TimerAction은 지연 실행 시 바깥 PushRosNamespace 문맥을
                    # 보존하지 않는다. 여기서 namespace를 다시 적용해야 MM의
                    # 전역 controller_server와 이름이 충돌하지 않는다.
                    GroupAction(actions=[
                        PushRosNamespace(namespace),
                        IncludeLaunchDescription(
                            PythonLaunchDescriptionSource(os.path.join(
                                nav2_bringup, "launch", "navigation_launch.py")),
                            launch_arguments={
                                "namespace": namespace,
                                "use_sim_time": use_sim_time,
                                "params_file": nav2_params,
                                "use_composition": "False",
                                # mission_nav_node가 모든 프로세스 로드 후 STARTUP하고
                                # action 서버가 없으면 자동 재시도한다.
                                "autostart": "False",
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
