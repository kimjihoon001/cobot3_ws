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
            }],
            remappings=[
                ("/tf", "/harvester_0/tf"),
                ("/tf_static", "/harvester_0/tf_static"),
            ],
        ),
    ])
