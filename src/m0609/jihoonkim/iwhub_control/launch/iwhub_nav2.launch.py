# -*- coding: utf-8 -*-
"""iw.hub Nav2 풀스택 — 베이스 + SLAM(slam_toolbox) + Nav2 네비게이션.

구성(§5.6: Isaac 은 실행/센서, ROS2 는 판단):
  Isaac  : main.py --iw --nav-scan  → /iwhub_0/joint_command·states, /scan_front·back, /tf(laser), /clock
  base_node : /cmd_vel→바퀴, joint_states→/odom + odom→base_link TF
  slam_toolbox : map→odom TF + /map (스캔 1개 /scan_front 사용)
  nav2 navigation : controller/planner/bt/behaviors/smoother (코스트맵은 스캔 2개 다 씀)
TF: map(slam)→odom(base_node)→base_link→laser_front/back(Isaac)

필요 패키지(dev PC): ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-slam-toolbox
실행: ros2 launch iwhub_control iwhub_nav2.launch.py
  (도메인은 Isaac 과 동일하게 export — 예 ROS_DOMAIN_ID=109. rviz2 로 목표점 찍어 주행 확인)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("iwhub_control")
    nav2_bringup = get_package_share_directory("nav2_bringup")
    nav2_params = os.path.join(pkg, "config", "nav2_params.yaml")
    map_yaml = os.path.join(pkg, "maps", "greenhouse.yaml")
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true",
                              description="Isaac /clock 사용(시뮬)"),

        # 1. 베이스 (cmd_vel→바퀴 / joint_states→odom+TF)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg, "launch", "iwhub_base.launch.py")),
            launch_arguments={"use_sim_time": use_sim_time}.items(),
        ),

        # 2. 위치추정 = map_server(정적 맵) + AMCL (map→odom TF). 스캔 1개(/scan_front) 로 매칭.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup, "launch", "localization_launch.py")),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "params_file": nav2_params,
                "map": map_yaml,
            }.items(),
        ),

        # 3. Nav2 네비게이션 스택 (코스트맵 obstacle_layer 가 /scan_front + /scan_back 둘 다 씀)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup, "launch", "navigation_launch.py")),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "params_file": nav2_params,
            }.items(),
        ),

        # 4. rviz2 (맵·스캔·경로 + Nav2 Goal 툴). 이 launch 로 같이 뜬다.
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", os.path.join(pkg, "config", "iwhub_nav2.rviz")],
            parameters=[{"use_sim_time": use_sim_time}],
        ),
    ])
