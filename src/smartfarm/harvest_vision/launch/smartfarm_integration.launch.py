# -*- coding: utf-8 -*-
"""스마트팜 통합 실행 — MM Nav2(초기위치 자동) + 비전·파지·수확 오케스트레이션 + 지게차.

복잡한 다중 런치를 하나로 합친다(2026-07-23 사용자). Isaac 은 별도 터미널:
  isaac_python main.py --mm --iw --fork --rmpflow --nav

한 줄 실행:
  ros2 launch harvest_vision smartfarm_integration.launch.py
  (맵/초기위치 바꾸려면)
  ros2 launch harvest_vision smartfarm_integration.launch.py \
      map:=/경로/farm.yaml initial_pose_y:=-12.0 initial_pose_yaw:=0.0

포함:
  1) fleet_dispatch/harvester_nav2  — MM Nav2(전역 ns). AMCL 초기위치를 MM 스폰(0,-12)로
     자동 세팅 → RViz 2D Pose Estimate 불필요.
  2) iwhub_control/iwhub_nav2       — IW 전용 Nav2(iwhub_0 ns), 같은 farm map 사용.
     mission_nav_node가 FOLLOW/FORKLIFT를 장애물 회피 goal로 변환.
  3) harvest_vision/harvest_full    — vision_node + vision_debug_view + manipulator_target_node
     + nav_harvest_test_node(수확 오케스트레이션 + iw 연동/지게차 트리거).
  4) warehouse_dock/fork_lift_node  — iw 만재 도착 시 /forklift/amr_docked 로 하역.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    fleet = get_package_share_directory("fleet_dispatch")
    harvest = get_package_share_directory("harvest_vision")
    iwhub = get_package_share_directory("iwhub_control")
    default_map = os.path.expanduser("~/cobot3_ws/maps/farm.yaml")

    map_yaml = LaunchConfiguration("map")
    set_init = LaunchConfiguration("set_initial_pose")
    init_x = LaunchConfiguration("initial_pose_x")
    init_y = LaunchConfiguration("initial_pose_y")
    init_yaw = LaunchConfiguration("initial_pose_yaw")
    use_debug = LaunchConfiguration("use_debug")

    return LaunchDescription([
        DeclareLaunchArgument("map", default_value=default_map),
        # farm.yaml origin 을 -13 으로 고쳐 맵==월드 정렬. MM 스폰=월드(0,-12)이 그대로
        # 맵 좌표라 자동 초기위치로 준다(2D Pose Estimate 불필요). 방향이 안 맞으면 yaw만 조정.
        DeclareLaunchArgument("set_initial_pose", default_value="true"),
        DeclareLaunchArgument("initial_pose_x", default_value="0.0"),
        DeclareLaunchArgument("initial_pose_y", default_value="-12.0"),
        DeclareLaunchArgument("initial_pose_yaw", default_value="0.0"),
        DeclareLaunchArgument("use_debug", default_value="true"),

        # 1) MM Nav2 (전역 ns)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(fleet, "launch", "harvester_nav2.launch.py")),
            launch_arguments={
                "map": map_yaml,
                "use_sim_time": "true",
                "set_initial_pose": set_init,
                "initial_pose_x": init_x,
                "initial_pose_y": init_y,
                "initial_pose_yaw": init_yaw,
            }.items(),
        ),

        # 2) IW 전용 Nav2 — 같은 map, 별도 namespace/TF/action.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(iwhub, "launch", "iwhub_nav2.launch.py")),
            launch_arguments={
                "map": map_yaml,
                "use_sim_time": "true",
                # IW 글로벌/로컬 코스트맵과 경로를 별도 RViz에서 항상 표시한다.
                "rviz": "true",
            }.items(),
        ),
        Node(
            package="iwhub_control", executable="mission_nav_node",
            name="iw_mission_nav_node", output="screen",
            parameters=[{"use_sim_time": True}],
        ),

        # 3) 비전 + 파지 FSM + 수확 오케스트레이션(iw 연동/지게차 트리거 포함)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(harvest, "launch", "harvest_full.launch.py")),
            # 디버그 창은 아래에서 명시적으로 한 번만 실행한다.
            launch_arguments={"use_debug": "false"}.items(),
        ),
        Node(
            package="harvest_vision", executable="vision_debug_view",
            name="vision_debug_view", output="screen",
            parameters=[{"use_sim_time": True}],
            condition=IfCondition(use_debug),
        ),

        # 4) 지게차 하역 노드
        Node(
            package="warehouse_dock", executable="fork_lift_node",
            name="fork_lift_node", output="screen",
            parameters=[{"use_sim_time": True}],
        ),
    ])
