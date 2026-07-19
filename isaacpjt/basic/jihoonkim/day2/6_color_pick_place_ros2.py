
import os
os.environ["ROS_DOMAIN_ID"] = "109"   # PC B(color_detector.py)와 동일해야 통신됨

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

from isaacsim.core.utils.extensions import enable_extension
enable_extension("isaacsim.ros2.bridge")
simulation_app.update()

from pathlib import Path
import sys
import time
import random

import numpy as np
import omni.usd
from pxr import Usd, UsdGeom, UsdPhysics, Gf

from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid, VisualCuboid
from isaacsim.core.api.tasks import BaseTask
from isaacsim.core.api.materials.physics_material import PhysicsMaterial
from isaacsim.core.prims import SingleGeometryPrim
from isaacsim.robot.manipulators.grippers import ParallelGripper
from isaacsim.robot.manipulators.manipulators import SingleManipulator
from isaacsim.sensors.camera import Camera
import isaacsim.core.utils.numpy.rotations as rot_utils

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32

_THIS_DIR = Path(__file__).resolve().parent
M0609_DIR = Path(__file__).resolve().parents[3] / "M0609"   # 로봇 자산 루트 (day2 밖, gitignore)

# rmpflow 인프라 폴더 경로 등록
RMPFLOW_DIR = str(M0609_DIR / "rmpflow")
if RMPFLOW_DIR not in sys.path:
    sys.path.insert(0, RMPFLOW_DIR)

from m0609_pick_place_controller import PickPlaceController

# ╔══════════════════════════════════════════════════════════════╗
# ║  A. Task 파라미터 (5번과 동일)                                  ║
# ╚══════════════════════════════════════════════════════════════╝
# USD_PATH        = str(_THIS_DIR / "Collected_m0609_camera_view/Collected_m0609_camera_view.usd")
USD_PATH = str(Path(__file__).resolve().parents[3] / "M0609/Collected_m0609_camera_view/Collected_m0609_camera_view.usd")
ROBOT_PRIM_PATH = "/World/m0609"
EE_LINK_NAME    = "link_6"
GRIPPER_JOINTS  = ["finger_joint", "right_inner_knuckle_joint"]

DRIVE_STIFFNESS = 1e8
DRIVE_DAMPING   = 1e4
DRIVE_MAX_FORCE = 1e8

GRIPPER_OPEN    = [0.0, 0.0]
GRIPPER_CLOSE   = [0.5, 0.5]
GRIPPER_DELTA   = [-0.5, -0.5]

FINGER_STATIC   = 1.8
FINGER_DYNAMIC  = 1.4
CUBE_STATIC     = 1.2
CUBE_DYNAMIC    = 1.0

# ── 인프라 파일 경로 ─────────────────────────────────────────
M0609_URDF_PATH           = str(M0609_DIR / "doosan-robot2/urdf/m0609_isaac_sim.urdf")
M0609_DESCRIPTION_PATH    = str(M0609_DIR / "rmpflow/m0609_description.yaml")
M0609_RMPFLOW_CONFIG_PATH = str(M0609_DIR / "rmpflow/m0609_rmpflow_common.yaml")

# ── Pick & Place 동작 파라미터 ───────────────────────────────
EE_OFFSET = np.array([0.0, 0.0, 0.2])
EVENTS_DT = [0.008, 0.005, 0.02, 0.1, 0.0025, 0.01, 0.0025, 1, 0.008, 0.08]


# ╔══════════════════════════════════════════════════════════════╗
# ║  B. 색상 Pick&Place (ROS2 색상 감지 왕복) 파라미터              ║
# ╚══════════════════════════════════════════════════════════════╝
CUBE_SIZE     = 0.05
CUBE_CENTER_Z = CUBE_SIZE / 2.0

COLORS = ["blue", "green"]
COLOR_ID = {"blue": 1, "green": 2}
ID_TO_COLOR = {v: k for k, v in COLOR_ID.items()}

# 대기(standby) 위치 — 파랑/초록 큐브가 처음부터 놓여 있는 곳
STANDBY_POS = {
    "blue":  (0.15,  0.30),
    "green": (0.15, -0.30),
}
# pick 영역 — 랜덤으로 선택된 큐브가 이동하는 위치
PICK_POS = (0.35, 0.0, CUBE_CENTER_Z)
# 선택 안 된 큐브를 치워두는 위치 — wrist camera 시야 밖(작업공간과 멀리 떨어진 곳)
PARK_POS = (5.0, 5.0, -5.0)

CUBE_RGB = {
    "blue":  np.array([0.0, 0.0, 1.0]),
    "green": np.array([0.0, 1.0, 0.0]),
}
# 같은 색 마커 위치 (color_id 수신 후 place할 목적지)
MARKER_POS = {
    "blue":  np.array([0.58,  0.30, 0.0]),
    "green": np.array([0.58, -0.30, 0.0]),
}

# ── Wrist 카메라 (EE 링크에 종속 → 로봇을 따라 움직임) ─────────
CAM_LOCAL_POS   = np.array([0.0, 0.0, 0.05])   # EE 기준 로컬 오프셋
CAM_LOCAL_ORI   = rot_utils.euler_angles_to_quats(np.array([0, 90, 0]), degrees=True)
CAM_RES         = (640, 480)                    # (width, height)
RGB_PUBLISH_EVERY_N_STEPS = 5                    # 매 N 스텝마다 /rgb 발행


# ============================================================
# 유틸 (5번과 동일)
# ============================================================
def find_prim_path_by_name(root_path: str, name: str):
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
    joint_positions = np.zeros(robot.num_dof)
    arm_joint_names = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
    arm_joint_deg = [0, 0, 90, 0, 90, 0]
    dof_names = list(robot.dof_names)
    for name, deg in zip(arm_joint_names, arm_joint_deg):
        joint_positions[dof_names.index(name)] = np.deg2rad(deg)
    robot.set_joint_positions(joint_positions)


# ============================================================
# ROS2 연동 (PC B의 color_detector.py 와 짝을 이룸)
#   - /rgb (sensor_msgs/Image) 발행
#   - /color_id (std_msgs/Int32, 1=blue/2=green) 구독
# ============================================================
def rgb_to_imgmsg(node, rgb_u8, frame_id="wrist_camera"):
    msg = Image()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.header.frame_id = frame_id
    msg.height, msg.width = rgb_u8.shape[0], rgb_u8.shape[1]
    msg.encoding = "rgb8"
    msg.is_bigendian = 0
    msg.step = msg.width * 3
    msg.data = np.ascontiguousarray(rgb_u8).tobytes()
    return msg


class ColorBridgeNode(Node):
    """Isaac Sim(PC A) 쪽 ROS2 노드: /rgb 발행 + /color_id 구독."""

    def __init__(self):
        super().__init__("isaac_color_bridge")
        self.rgb_pub = self.create_publisher(Image, "/rgb", 10)
        self.color_id_sub = self.create_subscription(
            Int32, "/color_id", self._on_color_id, 10
        )
        self.received_color_id = None

    def _on_color_id(self, msg):
        self.received_color_id = int(msg.data)
        self.get_logger().info(f"[color_id 수신] {self.received_color_id}")

    def publish_rgb(self, rgba_f32):
        rgb_u8 = (rgba_f32[:, :, :3] * 255.0).clip(0, 255).astype(np.uint8)
        self.rgb_pub.publish(rgb_to_imgmsg(self, rgb_u8))


# ============================================================
# Task
# ============================================================
class M0609Task(BaseTask):

    def __init__(self, name):
        super().__init__(name=name, offset=None)
        self._cubes = {}
        self.camera = None

    def set_up_scene(self, scene):
        super().set_up_scene(scene)
        self._load_usd()
        self._park_stray_red_props()
        self._discover_links()
        self._setup_physics()
        self._register_robot(scene)
        self._create_scene(scene)
        self._create_wrist_camera()
        print("\n  [완료] 씬 구성 성공!\n")

    def _load_usd(self):
        print("\n[1.LOAD] USD 로드")
        stage = omni.usd.get_context().get_stage()
        world_prim = stage.GetPrimAtPath("/World")
        if not world_prim.IsValid():
            world_prim = UsdGeom.Xform.Define(stage, "/World").GetPrim()
        world_prim.GetReferences().AddReference(USD_PATH)
        for _ in range(15):
            simulation_app.update()
        print(f"  [OK] {USD_PATH}")

    def _park_stray_red_props(self):
        """USD 원본에 이미 포함된 빨간 오브젝트를 wrist camera 시야 밖으로 이동."""
        print("\n[1b.CLEANUP] 빨간 오브젝트 정리")
        stage = omni.usd.get_context().get_stage()
        world_prim = stage.GetPrimAtPath("/World")
        moved = 0
        for prim in Usd.PrimRange(world_prim):
            path = str(prim.GetPath())
            if path.startswith(ROBOT_PRIM_PATH):
                continue
            if "red" not in prim.GetName().lower():
                continue
            if not UsdGeom.Xformable(prim):
                continue
            UsdGeom.XformCommonAPI(prim).SetTranslate(Gf.Vec3d(*PARK_POS))
            moved += 1
            print(f"  [OK] {path} → {PARK_POS} 로 이동")
        if moved == 0:
            print("  [정보] 'red' 이름의 프림을 찾지 못했습니다. 정확한 prim 경로를 알려주시면 처리하겠습니다.")

    def _discover_links(self):
        print("\n[2.DISCOVER] 링크 경로 탐색")
        self._ee_path = find_prim_path_by_name(ROBOT_PRIM_PATH, EE_LINK_NAME)
        if self._ee_path is None:
            raise RuntimeError(f"'{EE_LINK_NAME}' not found")
        print(f"  EE ({EE_LINK_NAME}) = {self._ee_path}")

    def _setup_physics(self):
        print("\n[3.PHYSICS] 물리 설정")
        stage = omni.usd.get_context().get_stage()
        drive_count = 0
        for prim in Usd.PrimRange(stage.GetPrimAtPath(ROBOT_PRIM_PATH)):
            for dt in ["angular", "linear"]:
                drive = UsdPhysics.DriveAPI.Get(prim, dt)
                if drive:
                    drive.GetStiffnessAttr().Set(DRIVE_STIFFNESS)
                    drive.GetDampingAttr().Set(DRIVE_DAMPING)
                    drive.GetMaxForceAttr().Set(DRIVE_MAX_FORCE)
                    drive_count += 1
        print(f"  [OK] drive updated: {drive_count}")

    def _register_robot(self, scene):
        print("\n[4.REGISTER] 로봇 등록")
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
        print(f"  [OK] SingleManipulator: {ROBOT_PRIM_PATH}")

    def _create_scene(self, scene):
        print("\n[5.SCENE] 파랑/초록 큐브 + 마커 구성")
        cube_material = PhysicsMaterial(
            prim_path="/World/Physics_Materials/cube_material",
            static_friction=CUBE_STATIC,
            dynamic_friction=CUBE_DYNAMIC,
            restitution=0.0,
        )
        for color in COLORS:
            x, y = STANDBY_POS[color]
            pos = np.array([x, y, CUBE_CENTER_Z])
            self._cubes[color] = scene.add(
                DynamicCuboid(
                    prim_path=f"/World/cube_{color}",
                    name=f"cube_{color}",
                    position=pos,
                    scale=np.array([CUBE_SIZE, CUBE_SIZE, CUBE_SIZE]),
                    color=CUBE_RGB[color],
                    mass=0.05,
                    physics_material=cube_material,
                )
            )
            # 같은 색 마커 (얇은 판)
            scene.add(
                VisualCuboid(
                    prim_path=f"/World/marker_{color}",
                    name=f"marker_{color}",
                    position=MARKER_POS[color],
                    scale=np.array([0.06, 0.06, 0.001]),
                    color=CUBE_RGB[color],
                )
            )
            print(f"  [OK] {color:<5} cube standby @ ({x}, {y})  marker @ {MARKER_POS[color][:2]}")

        # 그리퍼 손가락 마찰
        finger_material = PhysicsMaterial(
            prim_path="/World/Physics_Materials/finger_material",
            static_friction=FINGER_STATIC,
            dynamic_friction=FINGER_DYNAMIC,
            restitution=0.0,
        )
        for link_name in ["left_inner_finger", "right_inner_finger"]:
            link_path = find_prim_path_by_name(ROBOT_PRIM_PATH, link_name)
            if link_path:
                SingleGeometryPrim(
                    prim_path=link_path,
                    name=f"{link_name}_geom",
                ).apply_physics_material(finger_material)

    def _create_wrist_camera(self):
        print("\n[6.CAMERA] Wrist 카메라 생성 (EE 링크에 종속)")
        # EE 프림의 자식 경로로 생성 → 로봇 손목을 따라 함께 움직임
        self.camera = Camera(
            prim_path=f"{self._ee_path}/wrist_camera",
            translation=CAM_LOCAL_POS,
            orientation=CAM_LOCAL_ORI,
            frequency=20,
            resolution=CAM_RES,
        )
        print(f"  [OK] wrist_camera @ {self._ee_path}/wrist_camera  res={CAM_RES}")

    def get_observations(self):
        return {
            self._robot.name: {
                "joint_positions": self._robot.get_joint_positions(),
            },
        }

    def post_reset(self):
        self._robot.gripper.set_joint_positions(
            self._robot.gripper.joint_opened_positions
        )

    def teleport_cube(self, color, xyz):
        self._cubes[color].set_world_pose(position=np.array(xyz))


# ╔══════════════════════════════════════════════════════════════╗
# ║  C. 메인                                                        ║
# ╚══════════════════════════════════════════════════════════════╝
def main():
    my_world = World(stage_units_in_meters=1.0)
    task = M0609Task(name="m0609_color_task")
    my_world.add_task(task)
    my_world.reset()

    robot = my_world.scene.get_object("m0609_robot")
    initialize_robot(robot, my_world)

    # 카메라 초기화 + 안정화 렌더
    task.camera.initialize()
    for _ in range(30):
        my_world.step(render=True)

    # ROS2 노드 생성 (PC B의 color_detector.py 와 /rgb, /color_id 로 통신)
    rclpy.init()
    ros_node = ColorBridgeNode()
    print("  [OK] ROS2 노드 생성: /rgb 발행, /color_id 구독")

    # Controller 생성
    print("\n[C-2] PickPlaceController 생성")
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
    print("  [OK] Controller 생성 완료")

    print("\n[ROS2 색상 감지 Pick & Place 시작]  (Play 버튼을 누르세요)\n")
    was_playing = False
    selected_color = None      # 이번 라운드에 pick 영역으로 이동한 큐브 색
    picking_position = None
    placing_position = None
    pick_started = False
    step_count = 0

    def start_new_round():
        nonlocal selected_color, picking_position, placing_position, pick_started, step_count
        initialize_robot(robot, my_world)
        controller.reset()

        selected_color = random.choice(COLORS)
        other_color = [c for c in COLORS if c != selected_color][0]
        task.teleport_cube(selected_color, PICK_POS)
        task.teleport_cube(other_color, PARK_POS)   # 카메라 오검출 방지: 시야 밖으로 이동
        print(f"\n[SETUP] '{selected_color}' 큐브 → pick 영역 {PICK_POS} 이동, "
              f"'{other_color}' 큐브 → 시야 밖 대기")

        ros_node.received_color_id = None
        picking_position = None
        placing_position = None
        pick_started = False
        step_count = 0

    while simulation_app.is_running():
        my_world.step(render=True)
        time.sleep(0.01)
        rclpy.spin_once(ros_node, timeout_sec=0.0)
        is_playing = my_world.is_playing()

        # Play 시작 감지 → 리셋 + 랜덤 큐브를 pick 영역으로 이동
        if is_playing and not was_playing:
            my_world.reset()
            start_new_round()

        if is_playing:
            step_count += 1

            # ── (1) wrist camera 프레임을 /rgb 로 주기적 발행 (색 판정 전까지만) ──
            if not pick_started and step_count % RGB_PUBLISH_EVERY_N_STEPS == 0:
                rgba = task.camera.get_rgba()
                if rgba is not None and rgba.size > 0:
                    ros_node.publish_rgb(rgba)

            # ── (2) /color_id 수신되면 pick/place 목표 확정 ──
            if not pick_started and ros_node.received_color_id is not None:
                color = ID_TO_COLOR.get(ros_node.received_color_id)
                if color is None:
                    print(f"  [경고] 알 수 없는 color_id={ros_node.received_color_id}")
                    ros_node.received_color_id = None
                else:
                    picking_position = np.array(PICK_POS)
                    placing_position = MARKER_POS[color]
                    pick_started = True
                    print(f"  [OK] color_id={COLOR_ID[color]}({color}) 수신 → pick&place 시작")

            # ── (3) pick & place 진행 ──
            if pick_started:
                current_joints = robot.get_joint_positions()
                actions = controller.forward(
                    picking_position=picking_position,
                    placing_position=placing_position,
                    current_joint_positions=current_joints,
                    end_effector_offset=EE_OFFSET,
                )
                robot.apply_action(actions)

                if controller.is_done():
                    print(f"\n[완료] {selected_color} 큐브 → {selected_color} 마커")
                    my_world.reset()
                    start_new_round()   # 다음 라운드: 새 큐브 랜덤 배치 후 반복

        was_playing = is_playing

    ros_node.destroy_node()
    rclpy.shutdown()
    simulation_app.close()


if __name__ == "__main__":
    main()
