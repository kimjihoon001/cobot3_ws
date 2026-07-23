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
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
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
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(
                    nav2_bringup, "launch", "navigation_launch.py")),
                launch_arguments={
                    "namespace": namespace,
                    "use_sim_time": use_sim_time,
                    "params_file": nav2_params,
                }.items(),
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
        ),
    ])
