"""MM MoveIt + 비전 수확 FSM 통합 런치.

Isaac 짝:
  isaac_python main.py --mm

실행:
  ros2 launch mm_moveit vision_harvest_bringup.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


_TF_REMAP = [("/tf", "tf"), ("/tf_static", "tf_static")]


def generate_launch_description():
    share = get_package_share_directory("mm_moveit")
    ns = LaunchConfiguration("ns")
    use_sim_time = LaunchConfiguration("use_sim_time")
    sim = {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)}

    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(share, "launch", "m0617_moveit_bringup.launch.py")),
        condition=IfCondition(LaunchConfiguration("start_moveit")),
        launch_arguments={
            "rviz": LaunchConfiguration("moveit_rviz"),
            "use_sim_time": use_sim_time,
            "ns": ns,
        }.items(),
    )

    vision = Node(
        package="harvest_vision",
        executable="vision_node",
        name="vision_node",
        namespace=ns,
        output="screen",
        parameters=[sim, {
            "rgb_topic": "rgb",
            "depth_topic": "depth",
            "camera_info_topic": "camera_info",
            "annotated_topic": "vision/annotated_image",
            "detections_topic": "vision/tomato_detections",
            "target_topic": "vision/approach_target",
            "target_class_topic": "vision/target_class",
        }],
    )
    debug_view = Node(
        package="harvest_vision",
        executable="vision_debug_view",
        name="vision_debug_view",
        namespace=ns,
        output="screen",
        condition=IfCondition(LaunchConfiguration("debug_view")),
        parameters=[sim, {
            "annotated_topic": "vision/annotated_image",
            "depth_topic": "depth",
        }],
    )
    bridge = Node(
        package="mm_moveit",
        executable="mm_motion_bridge.py",
        name="mm_motion_bridge",
        namespace=ns,
        output="screen",
        parameters=[sim, {
            "group_name": "mm_manipulator",
            "planning_frame": "base_link",
            # 토마토와 바구니 모두 단일 관절 변화 120도까지 허용한다.
            "ik_max_single_joint_change_rad": 2.0943951024,
            "basket_ik_max_single_joint_change_rad": 2.0943951024,
            "ik_candidate_timeout_sec": 0.45,
            "tcp_stall_timeout_sec": 3.0,
            "tcp_stall_min_progress_m": 0.001,
        }],
        remappings=_TF_REMAP,
    )
    manipulator = Node(
        package="harvest_vision",
        executable="manipulator_target_node",
        name="manipulator_target_node",
        namespace=ns,
        output="screen",
        parameters=[sim, {
            "input_topic": "vision/approach_target",
            "validated_topic": "manipulator/validated_target",
            "output_topic": "manipulator/target_pose",
            "isaac_command_topic": "cmd",
            "target_class_topic": "vision/target_class",
            "state_topic": "manipulator/target_state",
            # 이 토픽은 mm_motion_bridge.py의 phase 진행 상태(APPROACH→PREGRASP
            # 등)용이다 — mm_motion_bridge.py 소스에 "pipeline_status"가
            # 하드코딩돼 있어 이름을 바꾸면 안 된다(2026-07-27 확인: "status"로
            # 바꿨더니 GRASP_VERIFY는 되는데 APPROACH→PREGRASP가 95초 넘게
            # 멈췄다). Isaac의 grasp_check/blade 응답은 별도 파라미터
            # isaac_status_topic이 받는다.
            "rmp_status_topic": "pipeline_status",
            "isaac_status_topic": "status",
            "sim_tomato_topic": "sim/tomato",
            "harvest_enable_topic": "harvest_test/enable",
            "mobility_ready_topic": "manipulator/mobility_ready",
            "reposition_request_topic": "nav/reposition_request",
            "base_frame": "base_link",
            "command_enabled": True,
            "use_sim_ground_truth": True,
            "direct_sim_grasp": True,
            "auto_grasp_enabled": True,
            "external_harvest_gate_enabled": ParameterValue(
                LaunchConfiguration("external_harvest_gate"), value_type=bool),
            "nav_reposition_enabled": ParameterValue(
                LaunchConfiguration("nav_reposition_enabled"), value_type=bool),
            "home_after_attempt": True,
            "single_shot_harvest": True,
            "retry_after_failure": False,
            # 실패한 궤적을 되풀이하거나 다른 과실을 자동 선택하지 않는다.
            # 안전 홈 복귀 후 다음 명시적 수확 명령을 기다린다.
            "approach_retry_max": 0,
            "bed_view_retry_max": 0,
            "basket_pose_max_age_sec": 2.0,
            "use_iw_tf_basket_fallback": True,
            "iw_base_frame": "iwhub_0/base_link",
            "tool_grasp_reach_m": 0.120,
            "grasp_tcp_max_distance_m": 0.09,
            # OMPL 계획(최대 8초) + Isaac 궤적 실행 시간을 모두 포함한다.
            "motion_timeout_sec": 30.0,
        }],
        remappings=_TF_REMAP,
    )

    return LaunchDescription([
        DeclareLaunchArgument("ns", default_value="harvester_0"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("start_moveit", default_value="true"),
        DeclareLaunchArgument("moveit_rviz", default_value="true"),
        DeclareLaunchArgument("debug_view", default_value="true"),
        DeclareLaunchArgument("external_harvest_gate", default_value="false"),
        DeclareLaunchArgument("nav_reposition_enabled", default_value="true"),
        moveit,
        vision,
        debug_view,
        bridge,
        manipulator,
    ])
