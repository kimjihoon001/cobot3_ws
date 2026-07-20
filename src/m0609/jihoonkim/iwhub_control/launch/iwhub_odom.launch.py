# -*- coding: utf-8 -*-
"""iw.hub Nav2 — 정적 맵 표시 + 고정 map→odom TF (AMCL 없음). 반복 온실 aliasing 회피.

핵심: 시뮬 odom 은 슬립이 없어 거의 정확 → AMCL 로 보정할 필요가 없다. 대신 로봇 시작
포즈(2,-12)를 고정 map→odom TF 로 박아 맵과 로봇을 정렬한다. 그러면:
  · 맵이 rviz 에 뜬다 → 골을 맵 보고 찍을 수 있다
  · odom 이 정확하니 로봇이 맵에서 안 틀어진다(벽에 안 박음)
  · AMCL 스캔매칭 안 하니 반복 통로 aliasing 문제가 없다
전역 계획은 정적 맵(식물줄·벽) 위에서, 실시간 회피는 로컬 코스트맵(라이다)으로.

  Isaac : main.py --iw --nav-scan   (로봇은 항상 (2,-12) 에 스폰돼야 TF 가 맞음)
  실행  : ros2 launch iwhub_control iwhub_odom.launch.py   (rviz 자동, Fixed Frame=map)
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
    with open(os.path.join(pkg, "urdf", "iwhub.urdf")) as f:
        robot_desc = f.read()
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),

        # 0. 로봇 모델(URDF) 발행 — RViz RobotModel 에 실제 크기(몸체+적재)를 보이게.
        #    base_link→deck_cargo(fixed) TF 도 여기서 냄. Nav2 회피는 nav2_params 의
        #    footprint(1.5×0.84)가 담당하고, 이 모델은 시각화용(2026-07-21).
        Node(
            package="robot_state_publisher", executable="robot_state_publisher",
            name="robot_state_publisher", output="screen",
            parameters=[{"use_sim_time": use_sim_time,
                         "robot_description": robot_desc}],
        ),

        # 1. 베이스 (cmd_vel→바퀴 / joint_states→odom+TF) + base_link↔IwHub 정적 TF
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg, "launch", "iwhub_base.launch.py")),
            launch_arguments={"use_sim_time": use_sim_time}.items(),
        ),

        # 2. 고정 map→odom TF — 로봇 시작 포즈(2,-12) = odom 원점. AMCL 대체(시뮬 odom 정확).
        #    로봇이 다른 위치에 스폰되면 이 값을 그 위치로 바꿀 것.
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="map_to_odom",
            arguments=["2.0", "-12.0", "0.0", "0.0", "0.0", "0.0", "map", "odom"],
            parameters=[{"use_sim_time": use_sim_time}],
        ),

        # 3. map_server (정적 맵 발행 — 표시 + 전역 계획용) + 라이프사이클
        Node(
            package="nav2_map_server", executable="map_server", name="map_server",
            output="screen",
            parameters=[{"use_sim_time": use_sim_time, "yaml_filename": map_yaml}],
        ),
        Node(
            package="nav2_lifecycle_manager", executable="lifecycle_manager",
            name="lifecycle_manager_map", output="screen",
            parameters=[{"use_sim_time": use_sim_time, "autostart": True,
                         "node_names": ["map_server"]}],
        ),

        # 4. Nav2 네비게이션 스택 (global_frame=map, 정적맵+스캔 코스트맵)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup, "launch", "navigation_launch.py")),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "params_file": nav2_params,
            }.items(),
        ),

        # 5. rviz2 (Fixed Frame=map, 맵+스캔+경로 + Nav2 Goal 툴)
        Node(
            package="rviz2", executable="rviz2", name="rviz2", output="screen",
            arguments=["-d", os.path.join(pkg, "config", "iwhub_nav2.rviz")],
            parameters=[{"use_sim_time": use_sim_time}],
        ),
    ])
