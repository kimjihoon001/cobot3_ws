# -*- coding: utf-8 -*-
"""스마트팜 물리 월드와 로봇 ROS 브리지만 실행한다."""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    workspace = os.path.expanduser("~/cobot3_ws")
    default_python = os.path.expanduser(
        "~/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release/python.sh")

    return LaunchDescription([
        DeclareLaunchArgument(
            "isaac_python",
            default_value=default_python,
            description="Isaac Sim 설치에 포함된 python.sh 절대경로"),
        DeclareLaunchArgument(
            "isaac_working_directory",
            default_value=os.path.join(workspace, "isaacpjt")),
        DeclareLaunchArgument(
            "isaac_main",
            default_value=os.path.join(workspace, "isaacpjt", "main.py")),
        DeclareLaunchArgument("ros_domain_id", default_value="108"),
        DeclareLaunchArgument(
            "rmw_implementation", default_value="rmw_fastrtps_cpp"),
        ExecuteProcess(
            cmd=[
                LaunchConfiguration("isaac_python"),
                LaunchConfiguration("isaac_main"),
                "--mm", "--iw", "--fork", "--nav", "--camera",
            ],
            cwd=LaunchConfiguration("isaac_working_directory"),
            output="screen",
            additional_env={
                "ROS_DOMAIN_ID": LaunchConfiguration("ros_domain_id"),
                "RMW_IMPLEMENTATION":
                    LaunchConfiguration("rmw_implementation"),
            },
        ),
    ])
