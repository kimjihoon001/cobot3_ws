"""Isaac MM용 Nav2 이동 → MoveIt 수확 → 시작점 복귀 통합 데모.

★2026-07-23 완전 격리: MoveIt·Nav2·수확 오케스트레이터를 전부 harvester_moveit
네임스페이스로 밀어(PushRosNamespace) 팀원 RMPflow(harvester_0)와 토픽·노드·액션·tf
(/harvester_moveit/tf)까지 안 겹치게 한다. 데모는 부착 파지(ATTACH_GRASP=1).

Isaac 짝:  isaac_python main.py --mm --moveit --nav   (--moveit→moveit_mm=harvester_moveit)
실행:      ros2 launch harvest_moveit nav_harvest_demo.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue

# tf2_ros 는 /tf·/tf_static 을 절대경로로 pub/sub → PushRosNamespace 로 안 밀린다.
# 절대→상대 remap 으로 네임스페이스 안에서 /harvester_moveit/tf 를 보게 한다(Codex 지적).
_TF_REMAP = [("/tf", "tf"), ("/tf_static", "tf_static")]


def generate_launch_description():
    moveit_share = get_package_share_directory("harvest_moveit")
    fleet_share = get_package_share_directory("fleet_dispatch")
    use_sim_time = LaunchConfiguration("use_sim_time")
    ns = LaunchConfiguration("ns")

    # Nav2 — namespace 인자로 스택 전체를 harvester_moveit 로. /cmd_vel·/tf 등이
    # /harvester_moveit/* 가 돼 moveit_mm build_nav 오버라이드(같은 ns)와 짝이 맞는다.
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            fleet_share, "launch", "harvester_nav2.launch.py")),
        condition=IfCondition(LaunchConfiguration("nav")),   # nav:=true 일 때만 Nav2
        launch_arguments={
            "map": LaunchConfiguration("map"),
            "slam": LaunchConfiguration("slam"),
            "rviz": LaunchConfiguration("nav_rviz"),
            "namespace": ns,
            "use_sim_time": use_sim_time,
        }.items(),
    )
    # MoveIt+ros2_control — moveit_isaac 이 내부에서 PushRosNamespace(ns) 로 격리.
    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            moveit_share, "launch", "moveit_isaac.launch.py")),
        launch_arguments={
            "rviz": LaunchConfiguration("moveit_rviz"),
            "ns": ns,
            "use_sim_time": use_sim_time,
        }.items(),
    )
    # 수확 오케스트레이터(Nav2 이동 + MoveIt 수확) — 같은 ns 에서 실행해야
    # /harvester_moveit/move_action·robot_description·cmd 를 찾는다. ATTACH_GRASP=1 로 부착 파지.
    # YOLO 검출(격리된 카메라 토픽 /harvester_moveit/*) + 디버그 뷰(cv2 창).
    #   좌표는 /sim/tomato(정답)로 파지하고, YOLO 는 검출 시연/뷰용.
    yolo_on = IfCondition(LaunchConfiguration("yolo"))
    vision_node = Node(
        package="harvest_vision", executable="vision_node",
        name="vision_node", output="screen", condition=yolo_on,
        parameters=[{
            "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
            "rgb_topic": "/harvester_moveit/rgb",
            "depth_topic": "/harvester_moveit/depth",
            "camera_info_topic": "/harvester_moveit/camera_info",
            "annotated_topic": "/harvester_moveit/vision/annotated_image",
            "target_topic": "/harvester_moveit/vision/approach_target",
            "target_class_topic": "/harvester_moveit/vision/target_class",
        }],
    )
    vision_view = Node(
        package="harvest_vision", executable="vision_debug_view",
        name="vision_debug_view", output="screen", condition=yolo_on,
        parameters=[{
            "annotated_topic": f"/harvester_moveit/vision/annotated_image",
            "depth_topic": f"/harvester_moveit/depth",
        }],
    )

    demo = GroupAction([
        PushRosNamespace(ns),
        vision_node, vision_view,
        TimerAction(
            # ★15s(2026-07-23): arm/gripper 스포너가 8/10s 뒤 뜨므로 그 이후로 미룬다.
            #   안 그러면 짧은 goal·현위치 수확 시 컨트롤러 준비 전 MoveIt 목표가 들어가 실패(Codex).
            period=15.0,
            # 수확 오케스트레이터 = grasp_proto (반복 HARVEST_N회 + YOLO 탐지 게이트 + 부착 파지).
            #   좌표는 /sim/tomato(정답), YOLO 탐지되면 접근·파지. MM 은 스폰위치서 바로 수확(나브 옵션).
            actions=[Node(
                package="harvest_moveit",
                executable="grasp_proto.py",
                name="grasp_proto",
                # ★namespace 명시(2026-07-23): TimerAction 안이라 GroupAction 의 PushRosNamespace 가
                #   전파 안 됨(스포너와 동일). 없으면 전역 /move_action 을 찾아 "move_action 없음"으로 죽음.
                namespace=ns,
                output="screen",
                additional_env={
                    "ATTACH_GRASP": "1",
                    "HARVEST_N": LaunchConfiguration("harvest_n"),
                    "YOLO_GATE": LaunchConfiguration("yolo_gate"),
                },
                remappings=_TF_REMAP,   # TransformListener 가 /harvester_moveit/tf 를 보게
                parameters=[{"use_sim_time": ParameterValue(use_sim_time, value_type=bool)}],
            )],
        ),
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            "map", default_value="/home/rokey/cobot3_ws/maps/farm.yaml"),
        DeclareLaunchArgument("slam", default_value="false"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("ns", default_value="harvester_moveit",
                              description="격리 네임스페이스(팀원 RMPflow=harvester_0)"),
        DeclareLaunchArgument("nav", default_value="false",
                              description="Nav2 자율주행 스택 띄우기(기본 off — 스폰위치 고정 수확)"),
        DeclareLaunchArgument("harvest_n", default_value="10",
                              description="반복 수확 횟수"),
        DeclareLaunchArgument("yolo_gate", default_value="1",
                              description="1=YOLO 탐지 대기 후 수확(원거리 탐지→접근)"),
        DeclareLaunchArgument("nav_rviz", default_value="false"),
        DeclareLaunchArgument("moveit_rviz", default_value="true"),
        DeclareLaunchArgument("yolo", default_value="true",
                              description="YOLO 검출 + 디버그 뷰(cv2 창) 띄우기"),
        nav2,
        moveit,
        demo,
    ])
