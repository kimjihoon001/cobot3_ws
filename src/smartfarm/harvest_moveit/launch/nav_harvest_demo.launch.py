"""Isaac MM용 Nav2 이동 → MoveIt 수확 → 시작점 복귀 통합 데모.

★2026-07-23 완전 격리: MoveIt·Nav2·수확 오케스트레이터를 전부 harvester_moveit
네임스페이스로 밀어(PushRosNamespace) 팀원 RMPflow(harvester_0)와 토픽·노드·액션·tf
(/harvester_moveit/tf)까지 안 겹치게 한다. 스쿱은 기본 순수 충돌 물리 운반이다.

Isaac 짝:  isaac_python main.py --mm --moveit --nav   (--moveit→moveit_mm=harvester_moveit)
실행:      ros2 launch harvest_moveit nav_harvest_demo.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, TimerAction)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
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
            "params_file": LaunchConfiguration("nav_params"),
            "set_initial_pose": "true",
            "initial_pose_x": LaunchConfiguration("initial_pose_x"),
            "initial_pose_y": LaunchConfiguration("initial_pose_y"),
            "initial_pose_yaw": LaunchConfiguration("initial_pose_yaw"),
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
    # /harvester_moveit/move_action·robot_description·cmd 를 찾는다.
    # YOLO 검출(격리된 카메라 토픽 /harvester_moveit/*) + 디버그 뷰(cv2 창).
    #   좌표는 /sim/tomato(정답)로 파지하고, YOLO 는 검출 시연/뷰용.
    yolo_on = IfCondition(LaunchConfiguration("yolo"))
    vision_node = Node(
        package="harvest_vision", executable="vision_node",
        name="vision_node", namespace=ns, output="screen", condition=yolo_on,
        parameters=[{
            "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
            "rgb_topic": "rgb",
            "depth_topic": "depth",
            "camera_info_topic": "camera_info",
            "annotated_topic": "vision/annotated_image",
            "detections_topic": "vision/tomato_detections",
            "target_topic": "vision/approach_target",
            "target_class_topic": "vision/target_class",
        }],
    )
    vision_view = Node(
        package="harvest_vision", executable="vision_debug_view",
        name="vision_debug_view", namespace=ns,
        output="screen", condition=yolo_on,
        parameters=[{
            "annotated_topic": "vision/annotated_image",
            "depth_topic": "depth",
        }],
    )

    demo = GroupAction([
        vision_node, vision_view,
        TimerAction(
            # 컨트롤러 스포너가 4초에 시작한다. 오케스트레이터는 5초에 띄우고 내부의
            # MoveIt/Nav2 서버 대기로 준비를 동기화해 불필요한 15초 고정 지연을 없앤다.
            period=5.0,
            # 수확 오케스트레이터 = grasp_proto (반복 HARVEST_N회 + YOLO 탐지 게이트 + 부착 파지).
            #   좌표는 /sim/tomato(정답), YOLO 탐지되면 접근·파지. MM 은 스폰위치서 바로 수확(나브 옵션).
            actions=[
                # Nav2를 켠 통합 실행은 실제 주행→수확→복귀 오케스트레이터를 사용한다.
                Node(
                    package="harvest_moveit",
                    executable="nav_harvest_demo.py",
                    name="nav_harvest_demo",
                    namespace=ns,
                    output="screen",
                    condition=IfCondition(LaunchConfiguration("nav")),
                    additional_env={
                        "ATTACH_GRASP": LaunchConfiguration("attach"),
                        "HARVEST_N": LaunchConfiguration("harvest_n"),
                        "YOLO_GATE": LaunchConfiguration("yolo_gate"),
                        "STEM_OBSTACLE": LaunchConfiguration("stem_obstacle"),
                        "STEM_GRIP": LaunchConfiguration("stem_grip"),
                    },
                    remappings=_TF_REMAP,
                    parameters=[{
                        "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                        "goal_x": ParameterValue(
                            LaunchConfiguration("goal_x"), value_type=float),
                        "goal_y": ParameterValue(
                            LaunchConfiguration("goal_y"), value_type=float),
                        "goal_yaw": ParameterValue(
                            LaunchConfiguration("goal_yaw"), value_type=float),
                        "stop_on_tomato": ParameterValue(
                            LaunchConfiguration("stop_on_tomato"),
                            value_type=bool),
                        "detection_confirm_frames": ParameterValue(
                            LaunchConfiguration("detection_confirm_frames"),
                            value_type=int),
                        "detection_stop_max_depth_m": ParameterValue(
                            LaunchConfiguration("detection_stop_depth"),
                            value_type=float),
                        "start_x": ParameterValue(
                            LaunchConfiguration("initial_pose_x"), value_type=float),
                        "start_y": ParameterValue(
                            LaunchConfiguration("initial_pose_y"), value_type=float),
                        "start_yaw": ParameterValue(
                            LaunchConfiguration("initial_pose_yaw"), value_type=float),
                        "base_frame": "mm_base",
                    }],
                ),
                # Nav2를 끈 경우에는 현재 위치 수확만 실행한다.
                Node(
                    package="harvest_moveit",
                    executable="grasp_proto.py",
                    name="grasp_proto",
                    namespace=ns,
                    output="screen",
                    condition=UnlessCondition(LaunchConfiguration("nav")),
                    additional_env={
                        "ATTACH_GRASP": LaunchConfiguration("attach"),
                        "HARVEST_N": LaunchConfiguration("harvest_n"),
                        "YOLO_GATE": LaunchConfiguration("yolo_gate"),
                        "STEM_OBSTACLE": LaunchConfiguration("stem_obstacle"),
                        "STEM_GRIP": LaunchConfiguration("stem_grip"),
                    },
                    remappings=_TF_REMAP,
                    parameters=[{
                        "use_sim_time": ParameterValue(
                            use_sim_time, value_type=bool)
                    }],
                ),
            ],
        ),
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            "map", default_value="/home/rokey/cobot3_ws/maps/farm_gen.yaml"),
        DeclareLaunchArgument(
            "nav_params", default_value=os.path.join(
                fleet_share, "config", "moveit_nav2.yaml")),
        # Isaac moveit_mm.py의 실제 월드 스폰과 초기 dummy yaw(π)에 맞춘다.
        DeclareLaunchArgument("initial_pose_x", default_value="-3.3"),
        DeclareLaunchArgument("initial_pose_y", default_value="-9.77"),
        DeclareLaunchArgument("initial_pose_yaw", default_value="3.141592653589793"),
        DeclareLaunchArgument("goal_x", default_value="-3.3",
                              description="수확 정차 위치(map 좌표)"),
        DeclareLaunchArgument("goal_y", default_value="-8.0",
                              description="수확 정차 위치(map 좌표)"),
        DeclareLaunchArgument("goal_yaw", default_value="3.141592653589793"),
        DeclareLaunchArgument(
            "stop_on_tomato", default_value="true",
            description="Nav2 이동 중 작업거리 토마토 연속 검출 시 정지·수확"),
        DeclareLaunchArgument(
            "detection_confirm_frames", default_value="3",
            description="Nav2 정지에 필요한 YOLO 연속 검출 프레임 수"),
        DeclareLaunchArgument(
            "detection_stop_depth", default_value="0.65",
            description="이 거리(m) 안의 YOLO 토마토만 정지 트리거"),
        DeclareLaunchArgument("slam", default_value="false"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("ns", default_value="harvester_moveit",
                              description="격리 네임스페이스(팀원 RMPflow=harvester_0)"),
        DeclareLaunchArgument("nav", default_value="true",
                              description="Nav2 자율주행→수확→복귀 실행"),
        DeclareLaunchArgument("harvest_n", default_value="1",
                              description="반복 수확 횟수"),
        DeclareLaunchArgument("attach", default_value="0",
                              description="0=U자 스쿱 순수 물리 운반. 1=비교용 FixedJoint"),
        DeclareLaunchArgument("stem_obstacle", default_value="0",
                              description="1=과실 위 줄기를 collision object 로 등록(접근 계획이 회피). OMPL 실패 시 0"),
        DeclareLaunchArgument("stem_grip", default_value="0",
                              description="구형 2F 줄기파지 실험만 1. 동축 스쿱은 0"),
        DeclareLaunchArgument(
            "yolo_gate", default_value="0",
            description=(
                "1=YOLO 탐지가 있어야 수확. 기본 0은 YOLO 화면/추론은 유지하되 "
                "검출 실패가 /sim/tomato 좌표 기반 수확을 막지 않음")),
        DeclareLaunchArgument("nav_rviz", default_value="false"),
        DeclareLaunchArgument("moveit_rviz", default_value="true"),
        DeclareLaunchArgument("yolo", default_value="true",
                              description="YOLO 검출 + 디버그 뷰(cv2 창) 띄우기"),
        nav2,
        moveit,
        demo,
    ])
