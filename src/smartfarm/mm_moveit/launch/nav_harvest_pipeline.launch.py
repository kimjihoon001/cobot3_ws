"""MM Nav2 이동 + MoveIt 수확 통합 시험 런치.

Isaac 짝:
  isaac_python main.py --mm --nav
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


_TF_REMAP = [("/tf", "tf"), ("/tf_static", "tf_static")]


def generate_launch_description():
    mm_share = get_package_share_directory("mm_moveit")
    fleet_share = get_package_share_directory("fleet_dispatch")
    ns = LaunchConfiguration("ns")
    use_sim_time = LaunchConfiguration("use_sim_time")

    harvest = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mm_share, "launch", "harvest_pipeline.launch.py")),
        launch_arguments={
            "ns": ns,
            "use_sim_time": use_sim_time,
            "start_moveit": "true",
            "moveit_rviz": LaunchConfiguration("moveit_rviz"),
            "debug_view": LaunchConfiguration("debug_view"),
            "external_harvest_gate": "true",
        }.items(),
    )
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            fleet_share, "launch", "harvester_nav2.launch.py")),
        launch_arguments={
            "map": LaunchConfiguration("map"),
            "slam": LaunchConfiguration("slam"),
            "rviz": LaunchConfiguration("nav_rviz"),
            "namespace": ns,
            "use_sim_time": use_sim_time,
            "params_file": os.path.join(
                fleet_share, "config", "moveit_nav2.yaml"),
            "set_initial_pose": "true",
            "initial_pose_x": LaunchConfiguration("initial_pose_x"),
            "initial_pose_y": LaunchConfiguration("initial_pose_y"),
            "initial_pose_yaw": LaunchConfiguration("initial_pose_yaw"),
        }.items(),
    )
    coordinator = Node(
        package="harvest_vision",
        executable="nav_harvest_test_node",
        name="nav_harvest_test_node",
        namespace=ns,
        output="screen",
        parameters=[{
            "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
            "nav_status_topic": "navigate_to_pose/_action/status",
            "harvest_enable_topic": "harvest_test/enable",
            "manipulator_state_topic": "manipulator/target_state",
            "mobility_ready_topic": "manipulator/mobility_ready",
            "rmpflow_status_topic": "pipeline_status",
            "isaac_command_topic": "cmd",
            "reposition_request_topic": "nav/reposition_request",
            "navigate_to_pose_action": "navigate_to_pose",
            "basket_frame": "base_link",
            "base_frame": "mm_base",
            "map_frame": "map",
            "home_after_nav": True,
        }],
        remappings=_TF_REMAP,
    )

    return LaunchDescription([
        DeclareLaunchArgument("ns", default_value="harvester_0"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument(
            "map", default_value="/home/rokey/cobot3_ws/maps/farm.yaml"),
        DeclareLaunchArgument("slam", default_value="false"),
        DeclareLaunchArgument("nav_rviz", default_value="true"),
        DeclareLaunchArgument("moveit_rviz", default_value="false"),
        DeclareLaunchArgument("debug_view", default_value="true"),
        DeclareLaunchArgument("initial_pose_x", default_value="0.0"),
        DeclareLaunchArgument("initial_pose_y", default_value="-12.0"),
        DeclareLaunchArgument("initial_pose_yaw", default_value="0.0"),
        harvest,
        nav2,
        coordinator,
    ])
