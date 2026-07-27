# -*- coding: utf-8 -*-
"""IW Nav2와 MM 추종/지게차 왕복 미션 노드만 실행한다."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    iwhub = get_package_share_directory("iwhub_control")
    default_map = os.path.expanduser("~/cobot3_ws/maps/farm.yaml")
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription([
        DeclareLaunchArgument("map", default_value=default_map),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("iw_rviz", default_value="true"),
        DeclareLaunchArgument("iw_dock_standoff", default_value="1.03"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                iwhub, "launch", "iwhub_nav2.launch.py")),
            launch_arguments={
                "map": LaunchConfiguration("map"),
                "use_sim_time": use_sim_time,
                "rviz": LaunchConfiguration("iw_rviz"),
                # 통합 진입점은 IDLE 상태에서도 costmap을 즉시 표시해야 한다.
                # mission_nav_node의 STARTUP 복구는 mission이 생긴 뒤의 안전망이다.
                "nav_autostart": "true",
            }.items(),
        ),
        Node(
            package="iwhub_control",
            executable="mission_nav_node",
            name="iw_mission_nav_node",
            output="screen",
            parameters=[{
                "use_sim_time": ParameterValue(
                    use_sim_time, value_type=bool),
                "mm_map_frame": "map",
                "mm_base_frame": "base_link",
                "dock_standoff": ParameterValue(
                    LaunchConfiguration("iw_dock_standoff"),
                    value_type=float),
                # navigation lifecycle autostart가 진행 중일 때 중복 STARTUP을
                # 보내지 않는다. 이 시간이 지난 뒤에도 action이 없을 때만 복구한다.
                "nav_startup_delay_sec": 20.0,
            }],
            remappings=[
                ("/tf", "/harvester_0/tf"),
                ("/tf_static", "/harvester_0/tf_static"),
            ],
        ),
    ])
