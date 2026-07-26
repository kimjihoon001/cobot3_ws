# -*- coding: utf-8 -*-
"""지게차 하역 코디네이터만 실행한다."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        Node(
            package="warehouse_dock",
            executable="fork_lift_node",
            name="fork_lift_node",
            output="screen",
            parameters=[{
                "use_sim_time": ParameterValue(
                    use_sim_time, value_type=bool),
            }],
        ),
    ])
