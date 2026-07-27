"""MoveIt2 + Isaac — mm 수확 MM 의 m0617 팔 계획/실행 (2026-07-24).

경로:  MoveIt(OMPL/Pilz/CHOMP) → arm_controller(JTC) → topic_based_ros2_control
       → /harvester_0/joint_command → Isaac ArticulationController → 팔

전제:
  - Isaac: isaac_python main.py --mm ... 실행 + ▶Play
    (Play 안 하면 /clock 정지 → 전부 멈춤)
  - mm.py 가 /harvester_0 토픽으로 joint_command를 적용
  - 스쿱/베이스는 MoveIt 밖: 스쿱 = gripper_controller/commands 3축, 베이스 = 텔레옵/Nav2

사용:
  ros2 launch mm_moveit m0617_moveit_bringup.launch.py            # RViz 포함
  ros2 launch mm_moveit m0617_moveit_bringup.launch.py rviz:=false
  (로컬 검증용: use_sim_time:=false — Isaac 없이 스택만 띄워 파라미터 확인)
"""
import os

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, GroupAction, RegisterEventHandler, TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue


def _yaml(path: str) -> dict:
    with open(path, encoding="utf-8") as f:
        return yaml.safe_load(f)


# Humble과 Jazzy(MoveIt 2.12.4)는 플래닝 파이프라인 파라미터 스키마가 다르다
# (2026-07-27 Jazzy GPU 실측으로 확인). 이 파일은 원래 Humble에서만 검증됐고
# (README §5.5), 이 워크스테이션은 Jazzy라 그 차이를 실측으로 잡았다 — 둘 다
# 돌아가야 하니 ROS_DISTRO로 분기한다.
#   1) planning_plugin(단수, 문자열) → Jazzy는 planning_plugins(복수,
#      string_array)만 읽는다. 단수 키를 주면 "Planning plugin name is empty"
#      로 move_group이 terminate/abort.
#   2) request_adapters — Humble은 공백-join 문자열 하나에 전/후처리(시간
#      파라미터화 포함)를 다 담지만(default_planner_request_adapters/*),
#      Jazzy는 string_array를 요구하고 전처리(default_planning_request_adapters/
#      Check*/Resolve*/Validate*)와 후처리(response_adapters,
#      default_planning_response_adapters/AddTimeOptimalParameterization 등)를
#      분리했다. Jazzy에 Humble 이름을 주면 pluginlib이 "does not exist"로 못
#      찾고, Jazzy에 response_adapters를 안 주면 궤적 시간이 전부 0으로 남아
#      arm_controller가 "Time between points ... not strictly increasing"으로
#      매번 거부한다(계획은 되는데 팔이 전혀 안 움직임).
_DISTRO = os.environ.get("ROS_DISTRO", "humble")

if _DISTRO == "humble":
    _ADAPTERS = " ".join([
        "default_planner_request_adapters/AddTimeOptimalParameterization",
        "default_planner_request_adapters/FixWorkspaceBounds",
        "default_planner_request_adapters/FixStartStateBounds",
        "default_planner_request_adapters/FixStartStateCollision",
        "default_planner_request_adapters/FixStartStatePathConstraints",
    ])
    _RESPONSE_ADAPTERS = None
    _PILZ_REQUEST_ADAPTERS = ""
else:
    _ADAPTERS = [
        "default_planning_request_adapters/CheckForStackedConstraints",
        "default_planning_request_adapters/CheckStartStateBounds",
        "default_planning_request_adapters/CheckStartStateCollision",
        "default_planning_request_adapters/ResolveConstraintFrames",
        "default_planning_request_adapters/ValidateWorkspaceBounds",
    ]
    _RESPONSE_ADAPTERS = [
        "default_planning_response_adapters/AddTimeOptimalParameterization",
        "default_planning_response_adapters/ValidateSolution",
    ]
    # request_adapters 키 자체를 안 준다 — 빈 리스트([])를 주면 ros2 launch가
    # 파라미터 타입을 추론 못 해 "Expected 'value' to be one of [...], but got
    # '()' of type 'tuple'"로 launch 자체가 죽는다(2026-07-27 확인). 안 주면
    # move_group이 빈 배열 기본값으로 처리한다.
    _PILZ_REQUEST_ADAPTERS = None


def _planning_plugin_kv(name: str) -> dict:
    """Humble=planning_plugin(단수 문자열), Jazzy=planning_plugins(복수 리스트)."""
    if _DISTRO == "humble":
        return {"planning_plugin": name}
    return {"planning_plugins": [name]}


# ★멀티로봇 TF 격리(2026-07-23): tf2_ros 는 /tf·/tf_static 을 **절대경로**로 pub/sub 하므로
#   PushRosNamespace 로 안 밀린다. 절대→상대 remap 을 넣어야 네임스페이스 안에서 /harvester_0/tf
#   가 된다(TF 쓰는 모든 노드에 적용 — rsp·move_group·rviz). (Codex 지적, 실행 전 필수.)
# RViz/MoveIt 플러그인 중 일부는 별도 내부 노드를 만들어 상대 remap 대상 `tf`를
# 실행 namespace로 올리지 않는다. Isaac이 실제 발행하는 절대 토픽을 직접 지정한다.
_TF_REMAP = [
    ("/tf", "/harvester_0/tf"),
    ("/tf_static", "/harvester_0/tf_static"),
]
_ROBOT_STATE_REMAP = [
    *_TF_REMAP,
    # event handler에서 시작되는 move_group/RViz 내부 노드는 PushRosNamespace를
    # 상속하지 않는 경우가 있다. 그러면 /joint_states를 기다리며 0 rad 일자 자세를
    # 그리므로 실제 JSB 출력으로 절대 remap한다.
    ("/joint_states", "/harvester_0/joint_states"),
]


def generate_launch_description():
    share = get_package_share_directory("mm_moveit")

    use_sim_time = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)
    sim = {"use_sim_time": use_sim_time}

    # ── URDF/SRDF = 수확 MM 전체 모델(섀시+m0617+동축 3축 1/4구 스쿱+TCP).
    #    계획 프레임이 mm_base(=Isaac 섀시 base_link, 지면 원점)가 된다 —
    #    섀시 프레임 과실좌표를 변환 없이 그대로 목표로 쓸 것(팔베이스 -0.30 보정 금지). ──
    urdf = xacro.process_file(
        os.path.join(share, "urdf", "mm_mm.urdf.xacro")).toxml()
    with open(os.path.join(share, "srdf", "mm_mm.srdf"),
              encoding="utf-8") as f:
        srdf = f.read()
    robot_description = {"robot_description": urdf}
    robot_description_semantic = {"robot_description_semantic": srdf}

    # MoveIt 노드가 기대하는 robot_description_kinematics 파라미터로 전달한다.
    # config/kinematics.yaml 자체는 그룹별 MoveIt 설정 파일이며 ROS 노드 파라미터
    # 파일(/**: ros__parameters:)이 아니므로 Node.parameters 에 경로를 직접 넣으면
    # rcl YAML 파서가 시작 전에 종료된다.
    kinematics = {
        "robot_description_kinematics":
            _yaml(os.path.join(share, "config", "kinematics.yaml"))
    }

    # 계획용 관절/데카르트 한계 (Pilz 는 cartesian_limits 필수)
    joint_limits = {"robot_description_planning": {
        **_yaml(os.path.join(share, "config", "joint_limits.yaml")),
        **_yaml(os.path.join(share, "config", "pilz_cartesian_limits.yaml")),
    }}

    # ── 계획 파이프라인 3종: OMPL(기본) + Pilz(직선 LIN) + CHOMP(최적화) ──
    ompl = {
        **_planning_plugin_kv("ompl_interface/OMPLPlanner"),
        "request_adapters": _ADAPTERS,
        "start_state_max_bounds_error": 0.1,
    }
    if _RESPONSE_ADAPTERS is not None:
        ompl["response_adapters"] = _RESPONSE_ADAPTERS
    ompl.update(_yaml(os.path.join(share, "config", "ompl_planning.yaml")))

    pilz = {
        **_planning_plugin_kv("pilz_industrial_motion_planner/CommandPlanner"),
        # Humble PlanningPipeline은 CIRC 보조점(path_constraints.name=interim)을
        # 일반 경로제약으로 다시 검사해, 생성된 원호의 거의 모든 점을 invalid로 만든다.
        # Pilz 생성기 자체의 관절한계/IK 검사는 유지하고 이 중복 사후검사만 끈다.
        "check_solution_paths": False,
        "default_planner_config": "PTP",
        "capabilities": " ".join([
            "pilz_industrial_motion_planner/MoveGroupSequenceAction",
            "pilz_industrial_motion_planner/MoveGroupSequenceService",
        ]),
    }
    if _PILZ_REQUEST_ADAPTERS is not None:
        pilz["request_adapters"] = _PILZ_REQUEST_ADAPTERS

    chomp = {
        **_planning_plugin_kv("chomp_interface/CHOMPPlanner"),
        "request_adapters": _ADAPTERS,
        "start_state_max_bounds_error": 0.1,
    }
    if _RESPONSE_ADAPTERS is not None:
        chomp["response_adapters"] = _RESPONSE_ADAPTERS
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
    servo_params = {
        "moveit_servo": _yaml(os.path.join(share, "config", "servo.yaml"))
    }

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
    _ns = LaunchConfiguration("ns")
    def _spawner(controller: str) -> Node:
        return Node(
        # move_group/servo/rviz 와 같은 이유로 절대경로("/", _ns)를 쓴다.
        # spawner_jsb 는 TimerAction으로 실행돼 isolated GroupAction의
        # PushRosNamespace(_ns) 컨텍스트를 그대로 물려받지만, spawner_arm/
        # spawner_gripper 는 RegisterEventHandler(OnProcessExit) 콜백으로
        # 비동기 실행되며 그 컨텍스트를 못 물려받는다(2026-07-27 확인 — namespace를
        # 아예 안 주면 jsb는 되는데 arm/gripper 는 /harvester_0 없이 떠서
        # --controller-manager controller_manager가 존재하지 않는
        # /controller_manager를 찾다 60초 타임아웃). 절대경로는 두 실행 경로
        # 모두에서 항상 /harvester_0로 고정돼 안전하다.
        package="controller_manager", executable="spawner",
        namespace=["/", _ns],
        arguments=[
            controller,
            "--controller-manager", "controller_manager",
            "--controller-manager-timeout", "60",
            # 통합 기동 부하에서 FastDDS 서비스 응답 하나가 유실될 수 있다.
            # 60초를 전부 기다리면 MoveIt 시작 이벤트도 그만큼 막히므로 짧게
            # 실패시키고 spawner 자체 재시도로 복구한다.
            "--service-call-timeout", "8",
            "--switch-timeout", "60",
        ],
        output="screen",
    )
    # controller_manager가 첫 load 응답을 늦게 보내도 단일 spawner는 "already loaded"를
    # 복구할 수 있다. 세 개를 한 프로세스에 넣으면 첫 중복에서 나머지 둘까지 전부
    # 건너뛰므로, JSB→arm→gripper 순으로 종료 이벤트를 연결한다.
    spawner_jsb = _spawner("joint_state_broadcaster")
    spawner_arm = _spawner("arm_controller")
    spawner_gripper = _spawner("gripper_controller")
    rsp = Node(
        package="robot_state_publisher", executable="robot_state_publisher",
        parameters=[robot_description, sim],
        remappings=_ROBOT_STATE_REMAP,
        output="screen",
    )
    move_group = Node(
        package="moveit_ros_move_group", executable="move_group",
        namespace=["/", _ns],
        parameters=[robot_description, robot_description_semantic,
                    kinematics, joint_limits, planning_pipelines,
                    moveit_controllers, trajectory_execution,
                    planning_scene_monitor, sim],
        remappings=_ROBOT_STATE_REMAP,
        output="screen",
    )
    # Servo는 평소 정지 상태이며 수확 오케스트레이터가 start/stop 서비스를 호출한
    # 짧은 카메라 미세보정 구간에만 JTC joint_trajectory를 발행한다.
    servo = Node(
        package="moveit_servo",
        # Humble의 moveit_servo 실행 파일은 servo_node_main, Jazzy부터는
        # servo_node로 이름이 바뀌었다(2026-07-27 확인: /opt/ros/jazzy/lib/moveit_servo/
        # 실측 — Humble 값은 git 이력의 원래 코드 그대로).
        executable="servo_node_main" if _DISTRO == "humble" else "servo_node",
        name="servo_node",
        namespace=["/", _ns],
        parameters=[servo_params, robot_description,
                    robot_description_semantic, kinematics, sim],
        remappings=_ROBOT_STATE_REMAP,
        output="screen",
    )
    rviz = Node(
        package="rviz2", executable="rviz2",
        name="moveit_rviz",
        namespace=["/", _ns],
        condition=IfCondition(LaunchConfiguration("rviz")),
        arguments=["-d", os.path.join(share, "config", "moveit.rviz")],
        parameters=[robot_description, robot_description_semantic,
                    kinematics, planning_pipelines, joint_limits, sim],
        remappings=_ROBOT_STATE_REMAP,
        output="screen",
    )

    # ★mm 네임스페이스로 전 노드 격리 — 토픽·노드·액션이
    #   /harvester_0/* 로(예: move_action, controller_manager, joint_states).
    #   팀원 RMPflow(harvester_0)와 안 겹친다. tf 프레임(base_link)은 그대로(Option A —
    #   frame_prefix 걸면 MoveIt 이 URDF 무접두 프레임을 못 찾아 깨진다; 완전 tf 격리는 후속).
    # MoveIt/RViz를 JSB보다 먼저 띄우면 실제 joint_states가 없는 약 5초 동안
    # RViz가 임시(기본) 관절 상태를 그린다. 실행 시점에 따라 홈 자세가 다르게
    # 보이는 기동 경쟁을 없애기 위해 arm_controller까지 활성화된 뒤 시작한다.
    start_moveit_after_arm = RegisterEventHandler(OnProcessExit(
        target_action=spawner_arm,
        on_exit=[spawner_gripper, move_group, servo, rviz],
    ))

    isolated = GroupAction([
        PushRosNamespace(LaunchConfiguration("ns")),
        control_node, rsp,
        TimerAction(period=4.0, actions=[spawner_jsb]),
        RegisterEventHandler(OnProcessExit(
            target_action=spawner_jsb, on_exit=[spawner_arm])),
        start_moveit_after_arm,
    ])

    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="true",
                              description="RViz(MotionPlanning) 띄우기"),
        DeclareLaunchArgument("use_sim_time", default_value="true",
                              description="Isaac /clock 사용(Play 필수)"),
        DeclareLaunchArgument("ns", default_value="harvester_0",
                              description="ROS2 네임스페이스(moveit_mm=harvester_moveit 와 격리)"),
        isolated,
    ])
