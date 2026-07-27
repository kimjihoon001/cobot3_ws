# -*- coding: utf-8 -*-
"""스마트팜 통합 실행 — MM 고정 자동주행·MoveIt 파지·IW 플레이스·지게차.

복잡한 다중 런치를 하나로 합친다(2026-07-23 사용자). Isaac 은 별도 터미널:
  isaac_python main.py --mm --iw --fork --nav --camera

한 줄 실행:
  ros2 launch harvest_vision smartfarm_integration.launch.py
  (맵/수확 위치를 바꾸려면)
  ros2 launch harvest_vision smartfarm_integration.launch.py \
      map:=/경로/farm.yaml harvest_x:=-0.54 harvest_y:=-8.19 harvest_yaw:=1.91
  (두 번 수확·적재하려면)
  ros2 launch harvest_vision smartfarm_integration.launch.py place_target_count:=2

포함:
  1) mm_moveit/nav_harvest_pipeline — 현재 검증된 MoveIt 스쿱 파지 파이프라인.
     MM 초기위치를 자동 설정하고 기억한 수확 대기 위치로 Nav2 goal을 보낸다.
  2) iwhub_control/iwhub_nav2       — IW 전용 Nav2(iwhub_0 ns), 같은 farm map 사용.
     mission_nav_node가 FOLLOW/FORKLIFT를 장애물 회피 goal로 변환.
  3) fixed_harvest_moveit_node      — Nav2 도착→홈→베드뷰→파지→실제 IW KLT 플레이스.
  4) warehouse_dock/fork_lift_node  — IW 만재 도착 시 지게차 하역.
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
    mm_moveit = get_package_share_directory("mm_moveit")
    iwhub = get_package_share_directory("iwhub_control")
    default_map = os.path.expanduser("~/cobot3_ws/maps/farm.yaml")

    map_yaml = LaunchConfiguration("map")
    use_sim_time = LaunchConfiguration("use_sim_time")
    init_x = LaunchConfiguration("initial_pose_x")
    init_y = LaunchConfiguration("initial_pose_y")
    init_yaw = LaunchConfiguration("initial_pose_yaw")

    return LaunchDescription([
        DeclareLaunchArgument("map", default_value=default_map),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        # farm.yaml origin 을 -13 으로 고쳐 맵==월드 정렬. MM 스폰=월드(0,-12)이 그대로
        # 맵 좌표라 자동 초기위치로 준다(2D Pose Estimate 불필요).
        DeclareLaunchArgument("initial_pose_x", default_value="0.0"),
        DeclareLaunchArgument("initial_pose_y", default_value="-12.0"),
        DeclareLaunchArgument("initial_pose_yaw", default_value="0.0"),
        # auto_nav_harvest.launch.py 단독에서 검증한 자동 수확 정차 위치.
        DeclareLaunchArgument("harvest_x", default_value="-0.54"),
        DeclareLaunchArgument("harvest_y", default_value="-8.19"),
        DeclareLaunchArgument("harvest_yaw", default_value="1.91"),
        DeclareLaunchArgument("nav_rviz", default_value="true"),
        DeclareLaunchArgument("moveit_rviz", default_value="true"),
        DeclareLaunchArgument("iw_rviz", default_value="true"),
        DeclareLaunchArgument("use_debug", default_value="true"),
        DeclareLaunchArgument(
            "place_target_count", default_value="1",
            description="IW 하역 전 수확·적재할 토마토 개수"),
        # MM 중심에서 현재 IW 접근 방향으로 유지할 비접촉 플레이스 거리.
        DeclareLaunchArgument("iw_dock_standoff", default_value="1.03"),

        # 1) 현재 성공한 MM MoveIt 파이프라인 + 고정 Nav2 goal 코디네이터.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    mm_moveit, "launch", "nav2_harvest_bringup.launch.py")),
            launch_arguments={
                "ns": "harvester_0",
                "map": map_yaml,
                "use_sim_time": use_sim_time,
                "initial_pose_x": init_x,
                "initial_pose_y": init_y,
                "initial_pose_yaw": init_yaw,
                "nav_rviz": LaunchConfiguration("nav_rviz"),
                "moveit_rviz": LaunchConfiguration("moveit_rviz"),
                "debug_view": LaunchConfiguration("use_debug"),
                "coordinator_executable": "fixed_harvest_moveit_node",
                "coordinator_name": "fixed_harvest_moveit_node",
                "auto_nav_goal": "true",
                "fixed_goal_x": LaunchConfiguration("harvest_x"),
                "fixed_goal_y": LaunchConfiguration("harvest_y"),
                "fixed_goal_yaw": LaunchConfiguration("harvest_yaw"),
                # 검증한 최초 정차 위치를 그대로 쓰고 파지 중 재주행하지 않는다.
                "nav_reposition_enabled": "false",
                # 과거 SUCCEEDED goal이 아니라 이번 자동 goal 도착 뒤에만 수확한다.
                "resume_search_after_start_sec": "0.0",
                "place_target_count": LaunchConfiguration(
                    "place_target_count"),
            }.items(),
        ),

        # 2) IW 전용 Nav2 — 같은 map, 별도 namespace/TF/action.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(iwhub, "launch", "iwhub_nav2.launch.py")),
            launch_arguments={
                "map": map_yaml,
                "use_sim_time": use_sim_time,
                "rviz": LaunchConfiguration("iw_rviz"),
            }.items(),
        ),
        Node(
            package="iwhub_control", executable="mission_nav_node",
            name="iw_mission_nav_node", output="screen",
            parameters=[{
                "use_sim_time": ParameterValue(
                    use_sim_time, value_type=bool),
                "mm_map_frame": "map",
                "mm_base_frame": "base_link",
                "dock_standoff": ParameterValue(
                    LaunchConfiguration("iw_dock_standoff"),
                    value_type=float),
            }],
            # MM 파이프라인은 TF를 /harvester_0 아래에 격리한다.
            remappings=[
                ("/tf", "/harvester_0/tf"),
                ("/tf_static", "/harvester_0/tf_static"),
            ],
        ),

        # 4) 지게차 하역 노드
        #
        # fork_lift_node가 아니라 회수 노드를 쓴다. iw.py의 load_cargo()가 IW를
        # Pallet_00을 실은 상태로 스폰하므로 첫 도킹은 "랙 → IW 상차"가 아니라
        # "만재 팔레트 회수"다. fork_lift_node는 IW가 빈 채로 온다는 전제라서
        # 이 씬에서는 랙으로 헛걸음한다. 회수 노드는 ForkLiftNode 서브클래스로
        # 같은 랙 경로를 물려받고, mission_nav_node가 IW 도착 시 호출하는
        # /forklift/start_cycle 서버도 제공한다(fork_lift_node에는 없어서
        # 예전에는 MM↔IW↔지게차가 순환 대기에 빠졌다 — 2026-07-26 실측).
        # 두 노드를 같이 띄우면 같은 도킹 이벤트에 하나는 상차, 하나는 하차를
        # 시도해 /forklift_0/joint_command를 동시에 몰다 실패한다.
        Node(
            package="warehouse_dock", executable="fork_lift_return_node",
            name="fork_lift_return_node", output="screen",
            parameters=[{
                "use_sim_time": ParameterValue(
                    use_sim_time, value_type=bool),
            }],
        ),
    ])
