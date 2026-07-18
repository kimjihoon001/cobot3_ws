"""
Isaac Sim 노드 (PC A) — 토픽 통신 구조
  발행:  /rgb       (sensor_msgs/Image)   USD 내장 RSD455 + OmniGraph 가 발행 (이 스크립트 아님)
  구독:  /color_id  (std_msgs/Int32)      파랑=1, 초록=2

시나리오:
  1) 파랑/초록 큐브 중 하나를 랜덤으로 pick 영역에 배치(정답은 숨김).
     나머지 한 개와 두 마커는 모두 카메라 시야 밖에 둔다 → 화면엔 큐브 하나만.
  2) 로봇이 큐브 위 관측 자세에서 대기 → RSD455 가 /rgb 발행
  3) PC B(color_detector)가 색을 판단해 /color_id (1 또는 2) 발행
  4) 받은 color_id 로 같은 색 마커 위치에 Place
        color_id=1 → 파란 마커 / color_id=2 → 초록 마커

실행:
  export ROS_DOMAIN_ID=50
  source /opt/ros/humble/setup.bash
  cd ~/cobot3_ws/isaacpjt/M0609
  isaac_python ros_isaac_node.py
"""

import os
import sys

# ══════════════════════════════════════════════════════════════
#  ROS2 rclpy 경로 설정 (★ 중요)
#  Isaac Sim 5.1 = Python 3.11 → 시스템 Humble(3.10) rclpy 사용 불가.
#  대신 ROS2 bridge 가 번들한 py3.11 용 rclpy/std_msgs/sensor_msgs 를 쓴다.
#  LD_LIBRARY_PATH 는 프로세스 시작 시점에만 반영되므로, 없으면 재실행한다.
# ══════════════════════════════════════════════════════════════
_ROS_BRIDGE = (
    "/home/rokey/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release"
    "/exts/isaacsim.ros2.bridge/humble"
)
os.environ.setdefault("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")
# ★ PC B(color_detector)와 반드시 같은 도메인이어야 통신됨.
os.environ["ROS_DOMAIN_ID"] = "109"
_ros_lib = _ROS_BRIDGE + "/lib"
if _ros_lib not in os.environ.get("LD_LIBRARY_PATH", ""):
    os.environ["LD_LIBRARY_PATH"] = _ros_lib + ":" + os.environ.get("LD_LIBRARY_PATH", "")
    os.execv(sys.executable, [sys.executable] + sys.argv)   # 새 LD_LIBRARY_PATH 로 재실행
sys.path.insert(0, _ROS_BRIDGE + "/rclpy")                  # py3.11 rclpy 경로

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

from pathlib import Path
import random

import numpy as np
import omni.usd
from pxr import Usd, UsdGeom, UsdPhysics

from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid, VisualCuboid
from isaacsim.core.api.tasks import BaseTask
from isaacsim.core.api.materials.physics_material import PhysicsMaterial
from isaacsim.core.prims import SingleGeometryPrim
from isaacsim.robot.manipulators.grippers import ParallelGripper
from isaacsim.robot.manipulators.manipulators import SingleManipulator
import isaacsim.core.utils.numpy.rotations as rot_utils

# ROS2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Int32

# 검출기와 동일하게 최신 한 건만. 시뮬 루프는 스텝당 spin_once 를 한 번만 부르므로
# depth 를 키우면 처리 못 한 묵은 판정이 큐에 남아 다음 사이클로 새어든다.
COLOR_ID_QOS = QoSProfile(
    depth=1, history=HistoryPolicy.KEEP_LAST, reliability=ReliabilityPolicy.RELIABLE
)

_THIS_DIR = Path(__file__).resolve().parent
RMPFLOW_DIR = str(_THIS_DIR / "rmpflow")
if RMPFLOW_DIR not in sys.path:
    sys.path.insert(0, RMPFLOW_DIR)
from m0609_pick_place_controller import PickPlaceController

# ── Task 파라미터 (4/5번과 동일) ─────────────────────────────
USD_PATH        = str(_THIS_DIR / "Collected_m0609_camera_view/Collected_m0609_camera_view.usd")
ROBOT_PRIM_PATH = "/World/m0609"
EE_LINK_NAME    = "link_6"
GRIPPER_JOINTS  = ["finger_joint", "right_inner_knuckle_joint"]
DRIVE_STIFFNESS, DRIVE_DAMPING, DRIVE_MAX_FORCE = 1e8, 1e4, 1e8
GRIPPER_OPEN, GRIPPER_CLOSE, GRIPPER_DELTA = [0.0, 0.0], [0.5, 0.5], [-0.5, -0.5]
FINGER_STATIC, FINGER_DYNAMIC = 1.8, 1.4
CUBE_STATIC, CUBE_DYNAMIC = 1.2, 1.0

M0609_URDF_PATH           = str(_THIS_DIR / "doosan-robot2/urdf/m0609_isaac_sim.urdf")
M0609_DESCRIPTION_PATH    = str(_THIS_DIR / "rmpflow/m0609_description.yaml")
M0609_RMPFLOW_CONFIG_PATH = str(_THIS_DIR / "rmpflow/m0609_rmpflow_common.yaml")

EE_OFFSET = np.array([0.0, 0.0, 0.2])
EVENTS_DT = [0.008, 0.005, 0.02, 0.1, 0.0025, 0.01, 0.0025, 1, 0.008, 0.08]

# ── 색상 / 큐브 / 마커 ───────────────────────────────────────
CUBE_SIZE     = 0.05
CUBE_CENTER_Z = CUBE_SIZE / 2.0

# color_id 규칙: 파랑=1, 초록=2
COLOR_RGB = {
    1: np.array([0.0, 0.0, 1.0]),   # blue
    2: np.array([0.0, 1.0, 0.0]),   # green
}
COLOR_NAME = {1: "blue", 2: "green"}

# pick 영역(랜덤 배치 중심) 과 대기(staging) 위치
PICK_CENTER   = np.array([0.40, 0.00])
PICK_RANDOM   = 0.04                                  # ±4cm 랜덤
STAGING_POS   = {                                     # 선택 안 된 큐브 대기 위치(카메라 밖)
    1: np.array([0.15,  0.55, CUBE_CENTER_Z]),
    2: np.array([0.15, -0.55, CUBE_CENTER_Z]),
}
# 같은 색 마커(Place 목표).
# ★ 마커도 큐브와 같은 색이라 관측 자세 시야에 들어오면 검출기가 그걸 세어 오판한다.
#   관측 자세에서 카메라가 보는 테이블 범위는 y=[-0.391, +0.466] (아래 주석 참고).
#   실측으로 y=±0.40 은 둘 다 화면에 잡혔다 → ±0.55 로 밀어 5cm 이상 여유를 둔다.
MARKER_POS = {
    1: np.array([0.55,  0.55, 0.0]),                  # 파란 마커
    2: np.array([0.55, -0.55, 0.0]),                  # 초록 마커
}

# ── 관측 자세 ────────────────────────────────────────────────
# USD 에 내장된 RSD455(그리퍼 장착)가 /rgb 를 발행한다. link_6 로컬 +Z 를 보므로
# 파지 자세(euler [0,180,0])에서 그대로 아래를 내려다본다.
#
# ★ 화각 주의: USD 의 verticalAperture(2.453)로 VFOV 를 계산하면 64.9° 가 나오지만
#   이는 틀렸다. Kit 은 렌더 프로덕트의 종횡비로 세로 화각을 다시 잡는데, 그래프의
#   RenderProduct 가 640x640 정사각형이라 실효 VFOV = HFOV = 90.5° 다.
#   이 높이에서 카메라는 (0.412, 0.037, 0.426) 에 있고 테이블 면에서
#   x=[-0.017, 0.840], y=[-0.391, 0.466] 를 본다. (렌더 픽셀로 실측 검증함)
OBSERVE_HEIGHT   = 0.50                               # link_6 월드 높이(m)
OBSERVE_EULER    = np.array([0.0, 180.0, 0.0])        # 그리퍼가 아래를 향하는 자세

# USD 에 DomeLight 가 두 개(각 intensity 1000) 겹쳐 있어 기본의 2배로 밝다.
# 하나를 꺼서 기본 밝기로 되돌린다.
DISABLE_PRIMS = [
    "/World/DomeLight_01",
    # 부모(RSD455)에 RigidBodyAPI 가 없어 매 스텝 physx 에러를 뱉는다. 사용도 안 함.
    "/World/m0609/onrobot_rg2ft/angle_bracket/realsense_rsd455/RSD455/Imu_Sensor",
]

# USD 그래프의 발행 노드는 queueSize 기본값이 10 이라 이미지를 10장까지 물고 있는다.
# 구독자가 밀리면 그만큼 버퍼가 부풀고 판정도 묵은 프레임 기준이 되므로 1로 낮춘다.
PUBLISHER_QUEUE_SIZE = 1
PUBLISHER_NODES = [
    "/World/Graph/camera_graph/RGBPublish",
    "/World/Graph/camera_graph/DepthPublish",
    "/World/Graph/camera_graph/CameraInfoPublish",
]

# ★ 그래프의 ROS2Context 는 domain_id=108 이 USD 에 박혀 있고 환경변수를 무시한다.
#   위의 ROS_DOMAIN_ID 를 바꿔도 그래프만 108 로 발행해 파이프라인이 갈라진다.
#   환경변수를 따르게 해서 도메인을 이 파일 한 곳에서만 관리한다.
CONTEXT_NODE = "/World/Graph/camera_graph/Context"

MOTION_TIMEOUT_STEPS  = 4000


# ============================================================
# 유틸 (4/5번과 동일)
# ============================================================
def find_prim_path_by_name(root_path, name):
    stage = omni.usd.get_context().get_stage()
    root_prim = stage.GetPrimAtPath(root_path)
    if not root_prim.IsValid():
        return None
    for prim in Usd.PrimRange(root_prim):
        if prim.GetName() == name:
            return str(prim.GetPath())
    return None


def initialize_robot(robot, world):
    robot.initialize()
    robot.gripper.initialize(
        physics_sim_view=world.physics_sim_view,
        articulation_apply_action_func=robot.apply_action,
        get_joint_positions_func=robot.get_joint_positions,
        set_joint_positions_func=robot.set_joint_positions,
        dof_names=robot.dof_names,
    )
    robot.set_joint_positions(np.zeros(robot.num_dof))


# ============================================================
# Task
# ============================================================
class M0609Task(BaseTask):
    def __init__(self, name):
        super().__init__(name=name, offset=None)
        self.cubes = {}          # color_id -> DynamicCuboid

    def set_up_scene(self, scene):
        super().set_up_scene(scene)
        self._load_usd()
        self._disable_prims()
        self._shrink_publisher_queues()
        self._sync_ros_domain()
        self._ee_path = find_prim_path_by_name(ROBOT_PRIM_PATH, EE_LINK_NAME)
        if self._ee_path is None:
            raise RuntimeError(f"'{EE_LINK_NAME}' not found")
        self._setup_physics()
        self._register_robot(scene)
        self._create_scene(scene)
        print("\n  [완료] 씬 구성 성공!\n")

    def _disable_prims(self):
        stage = omni.usd.get_context().get_stage()
        for path in DISABLE_PRIMS:
            prim = stage.GetPrimAtPath(path)
            if prim.IsValid():
                prim.SetActive(False)
                print(f"  [OK] 비활성화: {path}")

    def _shrink_publisher_queues(self):
        stage = omni.usd.get_context().get_stage()
        for path in PUBLISHER_NODES:
            attr = stage.GetPrimAtPath(path).GetAttribute("inputs:queueSize")
            if attr:
                attr.Set(PUBLISHER_QUEUE_SIZE)
                print(f"  [OK] queueSize={PUBLISHER_QUEUE_SIZE}: {path}")

    def _sync_ros_domain(self):
        stage = omni.usd.get_context().get_stage()
        prim = stage.GetPrimAtPath(CONTEXT_NODE)
        attr = prim.GetAttribute("inputs:useDomainIDEnvVar")
        if attr:
            attr.Set(True)
            print(f"  [OK] 그래프가 ROS_DOMAIN_ID={os.environ['ROS_DOMAIN_ID']} 를 따르게 함")

    def _load_usd(self):
        stage = omni.usd.get_context().get_stage()
        world_prim = stage.GetPrimAtPath("/World")
        if not world_prim.IsValid():
            world_prim = UsdGeom.Xform.Define(stage, "/World").GetPrim()
        world_prim.GetReferences().AddReference(USD_PATH)
        for _ in range(15):
            simulation_app.update()

    def _setup_physics(self):
        stage = omni.usd.get_context().get_stage()
        for prim in Usd.PrimRange(stage.GetPrimAtPath(ROBOT_PRIM_PATH)):
            for dt in ["angular", "linear"]:
                drive = UsdPhysics.DriveAPI.Get(prim, dt)
                if drive:
                    drive.GetStiffnessAttr().Set(DRIVE_STIFFNESS)
                    drive.GetDampingAttr().Set(DRIVE_DAMPING)
                    drive.GetMaxForceAttr().Set(DRIVE_MAX_FORCE)

    def _register_robot(self, scene):
        gripper = ParallelGripper(
            end_effector_prim_path=self._ee_path,
            joint_prim_names=GRIPPER_JOINTS,
            joint_opened_positions=np.array(GRIPPER_OPEN),
            joint_closed_positions=np.array(GRIPPER_CLOSE),
            action_deltas=np.array(GRIPPER_DELTA),
        )
        self._robot = scene.add(
            SingleManipulator(
                prim_path=ROBOT_PRIM_PATH,
                name="m0609_robot",
                end_effector_prim_path=self._ee_path,
                gripper=gripper,
            )
        )

    def _create_scene(self, scene):
        cube_material = PhysicsMaterial(
            prim_path="/World/Physics_Materials/cube_material",
            static_friction=CUBE_STATIC, dynamic_friction=CUBE_DYNAMIC, restitution=0.0,
        )
        # 파랑(1)/초록(2) 큐브 + 같은 색 마커
        for cid in (1, 2):
            self.cubes[cid] = scene.add(
                DynamicCuboid(
                    prim_path=f"/World/cube_{COLOR_NAME[cid]}",
                    name=f"cube_{COLOR_NAME[cid]}",
                    position=STAGING_POS[cid],
                    scale=np.array([CUBE_SIZE, CUBE_SIZE, CUBE_SIZE]),
                    color=COLOR_RGB[cid], mass=0.05, physics_material=cube_material,
                )
            )
            scene.add(
                VisualCuboid(
                    prim_path=f"/World/marker_{COLOR_NAME[cid]}",
                    name=f"marker_{COLOR_NAME[cid]}",
                    position=MARKER_POS[cid],
                    scale=np.array([0.06, 0.06, 0.001]),
                    color=COLOR_RGB[cid],
                )
            )
        finger_material = PhysicsMaterial(
            prim_path="/World/Physics_Materials/finger_material",
            static_friction=FINGER_STATIC, dynamic_friction=FINGER_DYNAMIC, restitution=0.0,
        )
        for link_name in ["left_inner_finger", "right_inner_finger"]:
            link_path = find_prim_path_by_name(ROBOT_PRIM_PATH, link_name)
            if link_path:
                SingleGeometryPrim(prim_path=link_path, name=f"{link_name}_geom").apply_physics_material(finger_material)

    def post_reset(self):
        self._robot.gripper.set_joint_positions(self._robot.gripper.joint_opened_positions)

    # 랜덤으로 한 색만 pick 영역에 배치, 나머지는 시야 밖 대기 위치로
    def spawn_random_cube(self):
        cid = random.choice([1, 2])
        px = PICK_CENTER[0] + random.uniform(-PICK_RANDOM, PICK_RANDOM)
        py = PICK_CENTER[1] + random.uniform(-PICK_RANDOM, PICK_RANDOM)
        pick_pos = np.array([px, py, CUBE_CENTER_Z])
        for k, cube in self.cubes.items():
            pos = pick_pos if k == cid else STAGING_POS[k]
            cube.set_world_pose(position=pos, orientation=np.array([1.0, 0.0, 0.0, 0.0]))
            cube.set_linear_velocity(np.zeros(3))
            cube.set_angular_velocity(np.zeros(3))
        return cid, pick_pos


# ============================================================
# ROS2 노드
# ============================================================
class IsaacSimNode(Node):
    def __init__(self):
        super().__init__("isaac_sim_node")
        self.latest_color_id = None
        self.create_subscription(Int32, "/color_id", self._on_color_id, COLOR_ID_QOS)
        self.get_logger().info("구독: /color_id  (/rgb 는 USD 내장 RSD455 그래프가 발행)")

    def _on_color_id(self, msg: Int32):
        if msg.data in (1, 2):
            self.latest_color_id = int(msg.data)


def main():
    rclpy.init()

    my_world = World(stage_units_in_meters=1.0)
    task = M0609Task(name="m0609_color_task")
    my_world.add_task(task)
    my_world.reset()

    robot = my_world.scene.get_object("m0609_robot")
    initialize_robot(robot, my_world)
    for _ in range(30):
        my_world.step(render=True)

    controller = PickPlaceController(
        name="m0609_pick_place_controller",
        gripper=robot.gripper,
        robot_articulation=robot,
        end_effector_initial_height=0.30,
        events_dt=EVENTS_DT,
        urdf_path=M0609_URDF_PATH,
        robot_description_path=M0609_DESCRIPTION_PATH,
        rmpflow_config_path=M0609_RMPFLOW_CONFIG_PATH,
        end_effector_frame_name=EE_LINK_NAME,
    )

    node = IsaacSimNode()
    observe_quat = rot_utils.euler_angles_to_quats(OBSERVE_EULER, degrees=True)
    print("\n[Isaac 노드 실행] 랜덤 큐브 1개 배치 → 관측 자세 대기 → /color_id 수신 → Place\n")

    observe_target = np.array([PICK_CENTER[0], PICK_CENTER[1], OBSERVE_HEIGHT])

    def hold_observe_pose():
        robot.apply_action(
            controller._cspace_controller.forward(
                target_end_effector_position=observe_target,
                target_end_effector_orientation=observe_quat,
            )
        )

    while simulation_app.is_running():
        # ── 새 사이클: 랜덤 큐브 배치 ──
        true_cid, pick_pos = task.spawn_random_cube()
        controller.reset()
        print(f"\n[사이클] 랜덤 배치 = {COLOR_NAME[true_cid]}(정답, 숨김)  pick=({pick_pos[0]:.3f},{pick_pos[1]:.3f})")

        # 배치 안정화 + 관측 자세로 이동. 이 동안 도착하는 /color_id 는 이전 사이클
        # 화면을 보고 판정한 것일 수 있으므로 아래에서 버린다.
        for _ in range(60):
            my_world.step(render=True)
            rclpy.spin_once(node, timeout_sec=0.0)
            hold_observe_pose()
        node.latest_color_id = None                   # 묵은 판정 폐기

        # ── 관측: 자세 유지하며 /color_id 대기 ──
        #    PickPlaceController 는 목표를 미리 알아야 하므로, 이 단계에서는
        #    내부 cspace 컨트롤러를 직접 써서 관측 자세만 잡는다.
        steps = 0
        while (
            simulation_app.is_running()
            and node.latest_color_id is None
            and steps < MOTION_TIMEOUT_STEPS
        ):
            my_world.step(render=True)
            rclpy.spin_once(node, timeout_sec=0.0)
            steps += 1
            hold_observe_pose()

        if node.latest_color_id is None:
            print("  [타임아웃] /color_id 미수신 → 다음 사이클")
            continue

        cid = node.latest_color_id
        # 마커는 바닥(z=0)에 깔린 판이므로, 놓을 목표는 큐브 중심 높이로 올린다.
        placing = MARKER_POS[cid] + np.array([0.0, 0.0, CUBE_CENTER_Z])
        ok = "일치" if cid == true_cid else "불일치"
        print(f"  [color_id 수신] {cid}({COLOR_NAME[cid]}) → {COLOR_NAME[cid]} 마커  (정답과 {ok})")

        # ── pick&place 루프 ──
        controller.reset()
        steps = 0
        while simulation_app.is_running() and steps < MOTION_TIMEOUT_STEPS:
            my_world.step(render=True)
            rclpy.spin_once(node, timeout_sec=0.0)
            steps += 1

            actions = controller.forward(
                picking_position=pick_pos,
                placing_position=placing,
                current_joint_positions=robot.get_joint_positions(),
                end_effector_offset=EE_OFFSET,
            )
            robot.apply_action(actions)

            if controller.is_done():
                print(f"  [완료] {COLOR_NAME[cid]} 큐브 → {COLOR_NAME[cid]} 마커")
                break

    node.destroy_node()
    rclpy.shutdown()
    simulation_app.close()


if __name__ == "__main__":
    main()