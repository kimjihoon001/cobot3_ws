"""기억한 수확 대기 위치로 이동해 m0617 MoveIt 원샷 수확을 수행한다.

Isaac 짝:
  cd /home/rokey/cobot3_ws/isaacpjt
  isaac_python main.py --mm --nav --camera

ROS 2:
  ros2 launch mm_moveit auto_nav_harvest.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    mm_share = get_package_share_directory("mm_moveit")
    pipeline = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                mm_share, "launch", "nav2_harvest_bringup.launch.py")),
        launch_arguments={
            "ns": LaunchConfiguration("ns"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "map": LaunchConfiguration("map"),
            "nav_rviz": LaunchConfiguration("nav_rviz"),
            "moveit_rviz": LaunchConfiguration("moveit_rviz"),
            "debug_view": LaunchConfiguration("debug_view"),
            "coordinator_executable": "fixed_harvest_moveit_node",
            "coordinator_name": "fixed_harvest_moveit_node",
            "auto_nav_goal": "true",
            "fixed_goal_x": LaunchConfiguration("harvest_x"),
            "fixed_goal_y": LaunchConfiguration("harvest_y"),
            "fixed_goal_yaw": LaunchConfiguration("harvest_yaw"),
            # 이 원샷 시험의 첫 정차 위치를 최종 위치로 사용한다.
            "nav_reposition_enabled": "false",
            # 고정 Nav 목표가 성공하기 전에는 수확 게이트를 절대 열지 않는다.
            "resume_search_after_start_sec": "0.0",
        }.items(),
    )
    return LaunchDescription([
        DeclareLaunchArgument("ns", default_value="harvester_0"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument(
            "map", default_value="/home/rokey/cobot3_ws/maps/farm.yaml"),
        DeclareLaunchArgument("harvest_x", default_value="-0.54"),
        DeclareLaunchArgument("harvest_y", default_value="-8.19"),
        DeclareLaunchArgument("harvest_yaw", default_value="1.91"),
        DeclareLaunchArgument("nav_rviz", default_value="true"),
        DeclareLaunchArgument("moveit_rviz", default_value="true"),
        DeclareLaunchArgument("debug_view", default_value="true"),
        pipeline,
    ])
