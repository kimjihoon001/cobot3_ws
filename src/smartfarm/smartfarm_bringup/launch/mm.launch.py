# -*- coding: utf-8 -*-
"""MM Nav2 + MoveIt + 비전 수확 코디네이터만 실행한다."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    mm_moveit = get_package_share_directory("mm_moveit")
    default_map = os.path.expanduser("~/cobot3_ws/maps/farm.yaml")

    return LaunchDescription([
        DeclareLaunchArgument("map", default_value=default_map),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("initial_pose_x", default_value="0.0"),
        DeclareLaunchArgument("initial_pose_y", default_value="-12.0"),
        DeclareLaunchArgument("initial_pose_yaw", default_value="0.0"),
        DeclareLaunchArgument("harvest_x", default_value="-0.54"),
        DeclareLaunchArgument("harvest_y", default_value="-8.19"),
        DeclareLaunchArgument("harvest_yaw", default_value="1.91"),
        DeclareLaunchArgument("nav_rviz", default_value="true"),
        DeclareLaunchArgument("moveit_rviz", default_value="true"),
        DeclareLaunchArgument("use_debug", default_value="true"),
        DeclareLaunchArgument(
            "place_target_count", default_value="1",
            description="IW 하역 전 수확·적재할 토마토 개수"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                mm_moveit, "launch", "nav2_harvest_bringup.launch.py")),
            launch_arguments={
                "ns": "harvester_0",
                "map": LaunchConfiguration("map"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "initial_pose_x": LaunchConfiguration("initial_pose_x"),
                "initial_pose_y": LaunchConfiguration("initial_pose_y"),
                "initial_pose_yaw": LaunchConfiguration("initial_pose_yaw"),
                "nav_rviz": LaunchConfiguration("nav_rviz"),
                "moveit_rviz": LaunchConfiguration("moveit_rviz"),
                "debug_view": LaunchConfiguration("use_debug"),
                "coordinator_executable": "fixed_harvest_moveit_node",
                "coordinator_name": "fixed_harvest_moveit_node",
                "auto_nav_goal": "true",
                "fixed_goal_x": LaunchConfiguration("harvest_x"),
                "fixed_goal_y": LaunchConfiguration("harvest_y"),
                "fixed_goal_yaw": LaunchConfiguration("harvest_yaw"),
                "nav_reposition_enabled": "false",
                "resume_search_after_start_sec": "0.0",
                "place_target_count": LaunchConfiguration(
                    "place_target_count"),
            }.items(),
        ),
    ])
