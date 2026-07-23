"""MoveIt2 + Isaac — 수확 MM 의 UR10e 팔 계획/실행 (2026-07-22).

경로:  MoveIt(OMPL/Pilz/CHOMP) → arm_controller(JTC) → topic_based_ros2_control
       → /harvester_moveit/joint_command → Isaac ArticulationController → 팔

전제:
  - Isaac: isaac_python main.py --mm ... 실행 + ▶Play (Play 안 하면 /clock 정지 → 전부 멈춤)
  - mm.py 가 rmpflow 주석 상태(joint_command 상시 적용) — 2026-07-22 전환 완료
  - 그리퍼/베이스는 MoveIt 밖: 그리퍼 = /harvester_moveit/cmd JSON, 베이스 = 텔레옵/Nav2

사용:
  ros2 launch harvest_moveit moveit_isaac.launch.py            # RViz 포함
  ros2 launch harvest_moveit moveit_isaac.launch.py rviz:=false
  (로컬 검증용: use_sim_time:=false — Isaac 없이 스택만 띄워 파라미터 확인)
"""
import os

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue


def _yaml(path: str) -> dict:
    with open(path, encoding="utf-8") as f:
        return yaml.safe_load(f)


_ADAPTERS = " ".join([
    "default_planner_request_adapters/AddTimeOptimalParameterization",
    "default_planner_request_adapters/FixWorkspaceBounds",
    "default_planner_request_adapters/FixStartStateBounds",
    "default_planner_request_adapters/FixStartStateCollision",
    "default_planner_request_adapters/FixStartStatePathConstraints",
])


# ★멀티로봇 TF 격리(2026-07-23): tf2_ros 는 /tf·/tf_static 을 **절대경로**로 pub/sub 하므로
#   PushRosNamespace 로 안 밀린다. 절대→상대 remap 을 넣어야 네임스페이스 안에서 /harvester_moveit/tf
#   가 된다(TF 쓰는 모든 노드에 적용 — rsp·move_group·rviz). (Codex 지적, 실행 전 필수.)
_TF_REMAP = [("/tf", "tf"), ("/tf_static", "tf_static")]


def generate_launch_description():
    share = get_package_share_directory("harvest_moveit")
    ur_moveit = get_package_share_directory("ur_moveit_config")

    use_sim_time = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)
    sim = {"use_sim_time": use_sim_time}

    # ── URDF/SRDF = 수확 MM 전체 모델(섀시+UR10e+커플러+그리퍼+커터지그+D455+TCP).
    #    팔 단독(harvester_ur10e + ur 표준 srdf)이던 것을 실물 구성으로 교체(2026-07-22).
    #    계획 프레임이 mm_base(=Isaac 섀시 base_link, 지면 원점)가 된다 —
    #    섀시 프레임 과실좌표를 변환 없이 그대로 목표로 쓸 것(팔베이스 -0.30 보정 금지). ──
    urdf = xacro.process_file(
        os.path.join(share, "urdf", "harvester_mm.urdf.xacro")).toxml()
    with open(os.path.join(share, "srdf", "harvester_mm.srdf"),
              encoding="utf-8") as f:
        srdf = f.read()
    robot_description = {"robot_description": urdf}
    robot_description_semantic = {"robot_description_semantic": srdf}

    # kinematics.yaml 은 /** ros__parameters 형식 → 파라미터 '파일'로 그대로 전달
    kinematics_file = os.path.join(ur_moveit, "config", "kinematics.yaml")

    # 계획용 관절/데카르트 한계 (Pilz 는 cartesian_limits 필수)
    joint_limits = {"robot_description_planning": {
        **_yaml(os.path.join(ur_moveit, "config", "joint_limits.yaml")),
        **_yaml(os.path.join(share, "config", "pilz_cartesian_limits.yaml")),
    }}

    # ── 계획 파이프라인 3종: OMPL(기본) + Pilz(직선 LIN) + CHOMP(최적화) ──
    ompl = {
        "planning_plugin": "ompl_interface/OMPLPlanner",
        "request_adapters": _ADAPTERS,
        "start_state_max_bounds_error": 0.1,
    }
    ompl.update(_yaml(os.path.join(ur_moveit, "config", "ompl_planning.yaml")))

    pilz = {
        "planning_plugin": "pilz_industrial_motion_planner/CommandPlanner",
        "request_adapters": "",
        "default_planner_config": "PTP",
        "capabilities": " ".join([
            "pilz_industrial_motion_planner/MoveGroupSequenceAction",
            "pilz_industrial_motion_planner/MoveGroupSequenceService",
        ]),
    }

    chomp = {
        "planning_plugin": "chomp_interface/CHOMPPlanner",
        "request_adapters": _ADAPTERS,
        "start_state_max_bounds_error": 0.1,
    }
    chomp.update(_yaml(os.path.join(share, "config", "chomp_planning.yaml")))

    planning_pipelines = {
        "planning_pipelines": ["ompl", "pilz_industrial_motion_planner", "chomp"],
        "default_planning_pipeline": "ompl",
        "ompl": ompl,
        "pilz_industrial_motion_planner": pilz,
        "chomp": chomp,
    }

    moveit_controllers = _yaml(
        os.path.join(share, "config", "moveit_controllers.yaml"))

    trajectory_execution = {
        "moveit_manage_controllers": True,
        # Isaac 60fps 추종이라 실제 실행이 계획보다 늦을 수 있다 — 넉넉히
        "trajectory_execution.allowed_execution_duration_scaling": 2.0,
        "trajectory_execution.allowed_goal_duration_margin": 5.0,
        "trajectory_execution.allowed_start_tolerance": 0.05,
    }
    planning_scene_monitor = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    # ── 노드들 ──
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description,
                    os.path.join(share, "config", "ros2_controllers.yaml"),
                    sim],
        output="screen",
    )
    # ★스포너에 명시적 namespace(2026-07-23): 이들은 TimerAction 안이라 GroupAction 의
    #   PushRosNamespace 가 전파 안 된다(전역 spawner 가 돼 /controller_manager 를 못 찾음).
    #   namespace=ns 를 직접 줘야 "controller_manager"(상대)가 /{ns}/controller_manager 로 붙는다.
    _ns = LaunchConfiguration("ns")
    spawner_jsb = Node(
        package="controller_manager", executable="spawner", namespace=_ns,
        arguments=["joint_state_broadcaster",
                   "--controller-manager", "controller_manager"],
        output="screen",
    )
    spawner_arm = Node(
        package="controller_manager", executable="spawner", namespace=_ns,
        arguments=["arm_controller",
                   "--controller-manager", "controller_manager"],
        output="screen",
    )
    spawner_gripper = Node(
        package="controller_manager", executable="spawner", namespace=_ns,
        arguments=["gripper_controller",
                   "--controller-manager", "controller_manager"],
        output="screen",
    )
    rsp = Node(
        package="robot_state_publisher", executable="robot_state_publisher",
        parameters=[robot_description, sim],
        remappings=_TF_REMAP,
        output="screen",
    )
    move_group = Node(
        package="moveit_ros_move_group", executable="move_group",
        parameters=[robot_description, robot_description_semantic,
                    kinematics_file, joint_limits, planning_pipelines,
                    moveit_controllers, trajectory_execution,
                    planning_scene_monitor, sim],
        remappings=_TF_REMAP,
        output="screen",
    )
    rviz = Node(
        package="rviz2", executable="rviz2",
        condition=IfCondition(LaunchConfiguration("rviz")),
        arguments=["-d", os.path.join(share, "config", "moveit.rviz")],
        parameters=[robot_description, robot_description_semantic,
                    kinematics_file, planning_pipelines, joint_limits, sim],
        remappings=_TF_REMAP,
        output="screen",
    )

    # ★harvester_moveit 네임스페이스로 전 노드 격리(2026-07-23) — 토픽·노드·액션이
    #   /harvester_moveit/* 로(예: move_action, controller_manager, joint_states).
    #   팀원 RMPflow(harvester_0)와 안 겹친다. tf 프레임(base_link)은 그대로(Option A —
    #   frame_prefix 걸면 MoveIt 이 URDF 무접두 프레임을 못 찾아 깨진다; 완전 tf 격리는 후속).
    isolated = GroupAction([
        PushRosNamespace(LaunchConfiguration("ns")),
        control_node, rsp, move_group, rviz,
        # 스포너는 지연 기동 — Isaac+RViz 동시 기동 CPU 폭주로 서비스 응답이 10초를
        # 넘으면 스포너가 재시도하다 "already loaded"로 죽는 레이스(GPU 실측 2026-07-22).
        TimerAction(period=4.0, actions=[spawner_jsb]),
        TimerAction(period=8.0, actions=[spawner_arm]),
        TimerAction(period=10.0, actions=[spawner_gripper]),
    ])

    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="true",
                              description="RViz(MotionPlanning) 띄우기"),
        DeclareLaunchArgument("use_sim_time", default_value="true",
                              description="Isaac /clock 사용(Play 필수)"),
        DeclareLaunchArgument("ns", default_value="harvester_moveit",
                              description="ROS2 네임스페이스(팀원 RMPflow=harvester_0 와 격리)"),
        isolated,
    ])
