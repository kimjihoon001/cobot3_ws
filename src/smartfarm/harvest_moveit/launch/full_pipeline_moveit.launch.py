"""MoveIt-MM 수확 → IW 레인 도킹 → 지게차 인계 통합 실행.

Isaac 짝:
  isaac_python main.py --moveit --iw --fork --nav

ROS 2:
  ros2 launch harvest_moveit full_pipeline_moveit.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    moveit = get_package_share_directory("harvest_moveit")
    iwhub = get_package_share_directory("iwhub_control")
    map_yaml = LaunchConfiguration("map")
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription([
        DeclareLaunchArgument(
            "map",
            default_value="/home/rokey/cobot3_ws/maps/farm_gen.yaml"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("harvest_n", default_value="1"),
        DeclareLaunchArgument("yolo", default_value="false"),
        DeclareLaunchArgument("mm_nav_rviz", default_value="true"),
        DeclareLaunchArgument("moveit_rviz", default_value="false"),
        DeclareLaunchArgument("iw_rviz", default_value="true"),

        # MoveIt-MM 전용 Nav2 + ros2_control + 수확 오케스트레이터.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                moveit, "launch", "nav_harvest_demo.launch.py")),
            launch_arguments={
                "map": map_yaml,
                "use_sim_time": use_sim_time,
                "harvest_n": LaunchConfiguration("harvest_n"),
                "yolo": LaunchConfiguration("yolo"),
                "moveit_rviz": LaunchConfiguration("moveit_rviz"),
                "nav_rviz": LaunchConfiguration("mm_nav_rviz"),
                # YOLO를 끈 기본 풀 테스트는 GT 과실 좌표로 목표까지 주행한다.
                "stop_on_tomato": LaunchConfiguration("yolo"),
                # KLT 플레이스까지 토마토를 확실히 운반하도록 검증된 보조
                # FixedJoint를 사용하고, release 단계에서 Isaac KLT에 인계한다.
                "attach": "1",
                "iw_handoff_on_success": "true",
            }.items(),
        ),

        # 기존에 검증한 일반 IW Nav2(창고 고정용 iw_test가 아님).
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                iwhub, "launch", "iwhub_nav2.launch.py")),
            launch_arguments={
                "map": map_yaml,
                "use_sim_time": use_sim_time,
                "rviz": LaunchConfiguration("iw_rviz"),
            }.items(),
        ),

        # MoveIt-MM TF는 /harvester_moveit/tf에 격리되어 있다. IW 자체 pose는
        # mission_nav_node가 /iwhub_0/tf를 별도 구독하므로 이 remap과 충돌하지 않는다.
        Node(
            package="iwhub_control",
            executable="mission_nav_node",
            name="iw_mission_nav_node",
            output="screen",
            remappings=[
                ("/tf", "/harvester_moveit/tf"),
                ("/tf_static", "/harvester_moveit/tf_static"),
            ],
            parameters=[{
                "use_sim_time": ParameterValue(
                    use_sim_time, value_type=bool),
                "mm_map_frame": "map",
                "mm_base_frame": "mm_base",
            }],
        ),

        Node(
            package="warehouse_dock",
            executable="fork_lift_return_node",
            name="fork_lift_return_node",
            output="screen",
            parameters=[{
                "use_sim_time": ParameterValue(
                    use_sim_time, value_type=bool),
                # 통합 씬의 IW는 Pallet_00을 싣고 시작한다. 도킹 시 이를
                # 빈 랙 0번으로 복귀시킨 뒤 Pallet_01을 IW에 공급한다.
                "initial_pallet": 0,
            }],
        ),
    ])
