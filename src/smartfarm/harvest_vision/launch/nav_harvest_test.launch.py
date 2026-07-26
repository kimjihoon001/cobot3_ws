"""IW 없는 Nav2→비전→수확→모의 바스켓 통합시험."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("harvest_vision")
    base = os.path.join(share, "config", "manipulator_target.yaml")
    test = os.path.join(share, "config", "nav_harvest_test.yaml")
    return LaunchDescription([
        Node(
            package="harvest_vision",
            executable="vision_node",
            name="vision_node",
            output="screen",
            parameters=[{"use_sim_time": True}],
        ),
        Node(
            package="harvest_vision",
            executable="manipulator_target_node",
            name="manipulator_target_node",
            output="screen",
            parameters=[base, test, {"use_sim_time": True}],
        ),
        Node(
            package="harvest_vision",
            executable="harvest_fsm_node",
            name="harvest_fsm_node",
            output="screen",
            parameters=[test, {"use_sim_time": True}],
        ),
    ])
