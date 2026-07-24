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
            os.path.join(
                mm_share, "launch", "vision_harvest_bringup.launch.py")),
        launch_arguments={
            "ns": ns,
            "use_sim_time": use_sim_time,
            "start_moveit": "true",
            "moveit_rviz": LaunchConfiguration("moveit_rviz"),
            "debug_view": LaunchConfiguration("debug_view"),
            "external_harvest_gate": "true",
            "nav_reposition_enabled": LaunchConfiguration(
                "nav_reposition_enabled"),
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
        executable=LaunchConfiguration("coordinator_executable"),
        name=LaunchConfiguration("coordinator_name"),
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
            "orient_arm_to_nearest_bed": True,
            "bed_view_forced_side": "left",
            "use_mock_basket": False,
            "auto_nav_goal": ParameterValue(
                LaunchConfiguration("auto_nav_goal"), value_type=bool),
            "fixed_goal_x": ParameterValue(
                LaunchConfiguration("fixed_goal_x"), value_type=float),
            "fixed_goal_y": ParameterValue(
                LaunchConfiguration("fixed_goal_y"), value_type=float),
            "fixed_goal_yaw": ParameterValue(
                LaunchConfiguration("fixed_goal_yaw"), value_type=float),
            "resume_search_after_start_sec": ParameterValue(
                LaunchConfiguration("resume_search_after_start_sec"),
                value_type=float),
        }],
        remappings=_TF_REMAP,
    )

    return LaunchDescription([
        DeclareLaunchArgument("ns", default_value="harvester_0"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument(
            "map", default_value="/home/rokey/cobot3_ws/maps/farm.yaml"),
        DeclareLaunchArgument("slam", default_value="false"),
        # Nav2와 MoveIt RViz는 각각 nav_rviz/moveit_rviz라는 서로 다른 노드 이름으로
        # 실행한다. 같은 namespace 안에서도 노드·파라미터 서비스가 충돌하지 않는다.
        DeclareLaunchArgument("nav_rviz", default_value="true"),
        DeclareLaunchArgument("moveit_rviz", default_value="true"),
        DeclareLaunchArgument("debug_view", default_value="true"),
        DeclareLaunchArgument("nav_reposition_enabled", default_value="true"),
        DeclareLaunchArgument("initial_pose_x", default_value="0.0"),
        DeclareLaunchArgument("initial_pose_y", default_value="-12.0"),
        DeclareLaunchArgument("initial_pose_yaw", default_value="0.0"),
        DeclareLaunchArgument(
            "coordinator_executable", default_value="nav_harvest_test_node"),
        DeclareLaunchArgument(
            "coordinator_name", default_value="nav_harvest_test_node"),
        # 통합 이동→수확 시험은 기억해 둔 수확 대기 위치로 자동 이동하는 런치다.
        # false이면 좌표가 설정돼 있어도 coordinator가 목표를 전송하지 않고
        # READY_FOR_NAV_GOAL에서 계속 대기한다.
        DeclareLaunchArgument("auto_nav_goal", default_value="true"),
        # 2026-07-24 RViz에서 사용자가 직접 지정해 검증한 수확 대기 위치.
        # yaw=1.91에서 가까운 왼쪽 베드 방향(-sin(yaw), cos(yaw))으로 5 cm 접근.
        # 현장 기준: 기존 정차점에서 map +X 방향으로 정확히 10 cm 이동.
        DeclareLaunchArgument("fixed_goal_x", default_value="-0.487"),
        DeclareLaunchArgument("fixed_goal_y", default_value="-8.207"),
        DeclareLaunchArgument("fixed_goal_yaw", default_value="1.91"),
        DeclareLaunchArgument(
            "resume_search_after_start_sec", default_value="2.0"),
        harvest,
        nav2,
        coordinator,
    ])
