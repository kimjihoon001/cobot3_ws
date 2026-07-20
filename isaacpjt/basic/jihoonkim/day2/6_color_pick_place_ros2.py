
import os
os.environ["ROS_DOMAIN_ID"] = "108"   # PC B(color_detector_node.py)와 동일해야 통신됨

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
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.robot.manipulators.grippers import ParallelGripper
from isaacsim.robot.manipulators.manipulators import SingleManipulator
from isaacsim.sensors.camera import Camera
import isaacsim.core.utils.numpy.rotations as rot_utils

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
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

CUBE_RGB = {
    "blue":  np.array([0.0, 0.0, 1.0]),
    "green": np.array([0.0, 1.0, 0.0]),
}

# 공중 대기(standby) — 대기 중인 큐브는 '자기 비콘 위 공중'에 떠 있는다(kinematic).
# ★ 중앙 pick 영역에서 좌우로 벗어나 있어(±0.40) detector 의 중앙 ROI 에는 안 잡힌다.
#   그래도 화면에 크게 들어와 오검출되면 더 옆으로(±0.50) 빼거나 ROI_FRAC 을 줄일 것.
STANDBY_AIR_POS = {
    "blue":  np.array([0.35,  0.40, 0.30]),
    "green": np.array([0.35, -0.40, 0.30]),
}

# pick 영역 — 선택된 큐브가 공중에서 낙하할 랜덤 (x, y).
# ★ 관찰 포즈(home)의 손목 카메라 화각 '안'이어야 한다. GPU에서 튜닝.
PICK_AREA_CENTER = np.array([0.35, 0.0])   # 화각 중심
PICK_AREA_RADIUS = 0.06                    # 이 반경 안에서만 랜덤 → 항상 시야 안
DROP_HEIGHT      = 0.20                     # 낙하 시작 높이(공중)
SETTLE_STEPS     = 60                       # 낙하 후 안착 대기 스텝 수 (≈1초)

# 선택 안 된(빨간) USD 소품을 치워두는 위치 — 카메라 시야 밖
PARK_POS = (5.0, 5.0, -5.0)

# 같은 색 마커(비콘) 위치 (color_id 수신 후 place할 목적지)
# ★ 0.58 은 너무 멀어 RMPFlow 가 경계에서 꼬여 큐브를 쳐버렸다. pick 영역(0.35,0)
#   '바로 좌우'로 당겨 팔은 옆으로만 옮기게 하고, 카메라 정면 화각에서도 빠지게 한다.
MARKER_POS = {
    "blue":  np.array([0.35,  0.40, 0.0]),
    "green": np.array([0.35, -0.40, 0.0]),
}

# ── 관찰 포즈 (팔 6축) — 손목 카메라가 pick 영역을 내려다보는 자세 ──
# ★ home 포즈를 관찰 포즈로 재사용. 손목캠이 pick 영역을 못 보면 여기서 튜닝.
ARM_JOINT_NAMES = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
OBSERVE_ARM_DEG = [0, 0, 90, 0, 90, 0]

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


def observe_joint_positions(robot):
    """관찰 포즈(팔은 OBSERVE_ARM_DEG, 그리퍼는 열림)의 전체 DOF 벡터."""
    jp = np.zeros(robot.num_dof)
    dof_names = list(robot.dof_names)
    for name, deg in zip(ARM_JOINT_NAMES, OBSERVE_ARM_DEG):
        jp[dof_names.index(name)] = np.deg2rad(deg)
    return jp


def initialize_robot(robot, world):
    robot.initialize()
    robot.gripper.initialize(
        physics_sim_view=world.physics_sim_view,
        articulation_apply_action_func=robot.apply_action,
        get_joint_positions_func=robot.get_joint_positions,
        set_joint_positions_func=robot.set_joint_positions,
        dof_names=robot.dof_names,
    )
    robot.set_joint_positions(observe_joint_positions(robot))


# ============================================================
# ROS2 연동 (PC B의 color_detector_node.py 와 짝을 이룸)
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
        # ★ QoS: 밀림(백로그) 방지. /rgb 는 최신 프레임만(BEST_EFFORT depth1),
        #   /color_id 는 최신 판정만 확실히(RELIABLE depth1). 묵은 값이 안 쌓인다.
        rgb_qos = QoSProfile(depth=1, history=QoSHistoryPolicy.KEEP_LAST,
                             reliability=QoSReliabilityPolicy.BEST_EFFORT)
        cid_qos = QoSProfile(depth=1, history=QoSHistoryPolicy.KEEP_LAST,
                             reliability=QoSReliabilityPolicy.RELIABLE)
        self.rgb_pub = self.create_publisher(Image, "/rgb", rgb_qos)
        self.color_id_sub = self.create_subscription(
            Int32, "/color_id", self._on_color_id, cid_qos
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
        print("\n[5.SCENE] 파랑/초록 큐브(공중 대기) + 마커 구성")
        cube_material = PhysicsMaterial(
            prim_path="/World/Physics_Materials/cube_material",
            static_friction=CUBE_STATIC,
            dynamic_friction=CUBE_DYNAMIC,
            restitution=0.0,
        )
        for color in COLORS:
            self._cubes[color] = scene.add(
                DynamicCuboid(
                    prim_path=f"/World/cube_{color}",
                    name=f"cube_{color}",
                    position=STANDBY_AIR_POS[color],
                    scale=np.array([CUBE_SIZE, CUBE_SIZE, CUBE_SIZE]),
                    color=CUBE_RGB[color],
                    mass=0.05,
                    physics_material=cube_material,
                )
            )
            # 공중 대기: 물리 정지(kinematic)로 떠 있게 한다. 낙하는 라운드 시작 때만.
            self.set_cube_kinematic(color, True)
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
            print(f"  [OK] {color:<5} 공중 대기 @ {STANDBY_AIR_POS[color]}  marker @ {MARKER_POS[color][:2]}")

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

    # ── 큐브 물리/포즈 제어 ─────────────────────────────────────
    def set_cube_kinematic(self, color, flag):
        """공중 부양(kinematic=True) / 낙하(False) 전환. (scene/physics.py 패턴)"""
        rb = UsdPhysics.RigidBodyAPI.Apply(self._cubes[color].prim)
        rb.CreateKinematicEnabledAttr().Set(bool(flag))

    def set_cube_pose(self, color, xyz):
        self._cubes[color].set_world_pose(position=np.array(xyz, dtype=float))

    def cube_position(self, color):
        pos, _ = self._cubes[color].get_world_pose()
        return np.array(pos)


# ╔══════════════════════════════════════════════════════════════╗
# ║  C. 메인                                                        ║
# ╚══════════════════════════════════════════════════════════════╝
PHASE_SETTLE, PHASE_DETECT, PHASE_PICKPLACE, PHASE_RETURN = 0, 1, 2, 3


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

    # ROS2 노드 생성 (PC B의 color_detector_node.py 와 /rgb, /color_id 로 통신)
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

    # 관찰 포즈("첫 위치")를 유지/복귀하는 hold 액션 + 복귀 판정용 관절값
    observe_full = observe_joint_positions(robot)
    hold_action = ArticulationAction(joint_positions=observe_full)
    dof_names = list(robot.dof_names)
    arm_idx = [dof_names.index(n) for n in ARM_JOINT_NAMES]
    observe_arm = observe_full[arm_idx]
    RETURN_TOL = np.deg2rad(3.0)     # 첫 위치 복귀 허용 오차
    RETURN_DURATION = 90             # 첫 위치까지 부드럽게 보간하는 스텝 수(≈1.5s)
    RETURN_MAX_STEPS = 400           # 복귀 타임아웃(안전장치)

    # 라운드 상태
    state = {"phase": PHASE_SETTLE, "settle": 0, "return_steps": 0, "return_from": None,
             "selected": None, "picking": None, "placing": None}
    step_count = 0
    was_playing = False

    def start_new_round():
        """무한 반복: 팔을 관찰 포즈로, 두 큐브를 공중 대기로 되돌린 뒤
        랜덤 색 하나만 pick 영역 위 랜덤 위치에서 낙하시킨다."""
        nonlocal step_count
        # 팔은 이미 첫 위치(관찰 포즈)에 있다 — 순간이동시키지 않는다.
        # (첫 라운드는 Play 직후 1회 세팅, 이후 라운드는 RETURN 단계가 복귀시킴)
        controller.reset()

        # 두 큐브 모두 공중 대기(kinematic)로 복귀 (지난 라운드에 놓인 것도 회수)
        for c in COLORS:
            task.set_cube_kinematic(c, True)
            task.set_cube_pose(c, STANDBY_AIR_POS[c])

        # 랜덤 색 + 화각 안 랜덤 낙하 위치
        selected = random.choice(COLORS)
        theta = random.uniform(0.0, 2.0 * np.pi)
        r = PICK_AREA_RADIUS * np.sqrt(random.uniform(0.0, 1.0))
        drop_x = float(PICK_AREA_CENTER[0] + r * np.cos(theta))
        drop_y = float(PICK_AREA_CENTER[1] + r * np.sin(theta))
        # 공중(kinematic)에서 낙하 지점 위로 옮긴 뒤 물리를 켜서 떨어뜨림
        task.set_cube_pose(selected, np.array([drop_x, drop_y, DROP_HEIGHT]))
        task.set_cube_kinematic(selected, False)
        print(f"\n[SETUP] '{selected}' 큐브 공중 스폰 → 낙하 @ ({drop_x:.3f}, {drop_y:.3f})")

        ros_node.received_color_id = None
        state.update(phase=PHASE_SETTLE, settle=0,
                     selected=selected, picking=None, placing=None)
        step_count = 0

    print("\n[ROS2 색상 감지 Pick & Place 시작]  (Play 버튼을 누르세요)\n")

    while simulation_app.is_running():
        my_world.step(render=True)
        time.sleep(0.01)
        rclpy.spin_once(ros_node, timeout_sec=0.0)
        is_playing = my_world.is_playing()

        # Play 시작 감지 → 리셋 + 첫 라운드 시작
        if is_playing and not was_playing:
            my_world.reset()
            initialize_robot(robot, my_world)   # 시작 1회만 첫 위치로 세팅(허용)
            start_new_round()

        if is_playing:
            step_count += 1
            phase = state["phase"]

            # ── (SETTLE) 큐브가 낙하·안착할 때까지 팔을 관찰 포즈로 고정 ──
            if phase == PHASE_SETTLE:
                robot.apply_action(hold_action)
                state["settle"] += 1
                if state["settle"] >= SETTLE_STEPS:
                    state["phase"] = PHASE_DETECT
                    # ★ 직전 라운드에 버퍼에 남아 SETTLE 중 다시 채워진 묵은
                    #   color_id 를 폐기. 이제부터 새 rgb 에 대한 응답만 받는다.
                    ros_node.received_color_id = None
                    print("  [안착] 큐브 정지 → 관찰 포즈에서 /rgb 발행 시작")

            # ── (DETECT) ★첫 위치(관찰 포즈)에서만 /rgb 발행 → color_id 대기 ──
            #    큐브 잡으러 가거나(PICKPLACE) 드는 동안엔 발행하지 않는다:
            #    손목캠이 파란/초록 비콘을 잡아 오검출되는 걸 막기 위함.
            elif phase == PHASE_DETECT:
                robot.apply_action(hold_action)     # 관찰 포즈 유지(카메라가 큐브를 봄)
                if step_count % RGB_PUBLISH_EVERY_N_STEPS == 0:
                    rgba = task.camera.get_rgba()
                    if rgba is not None and rgba.size > 0:
                        ros_node.publish_rgb(rgba)

                if ros_node.received_color_id is not None:
                    color = ID_TO_COLOR.get(ros_node.received_color_id)
                    if color is None:
                        print(f"  [경고] 알 수 없는 color_id={ros_node.received_color_id}")
                        ros_node.received_color_id = None
                    else:
                        cube_pos = task.cube_position(state["selected"])
                        state["picking"] = np.array([cube_pos[0], cube_pos[1], CUBE_CENTER_Z])
                        mk = MARKER_POS[color]
                        state["placing"] = np.array([mk[0], mk[1], CUBE_CENTER_Z])
                        state["phase"] = PHASE_PICKPLACE
                        print(f"  [OK] color_id={COLOR_ID[color]}({color}) 수신 "
                              f"→ pick ({cube_pos[0]:.3f}, {cube_pos[1]:.3f}) → {color} 마커")

            # ── (PICKPLACE) 큐브를 집어 같은 색 마커로 place ──
            elif phase == PHASE_PICKPLACE:
                current_joints = robot.get_joint_positions()
                actions = controller.forward(
                    picking_position=state["picking"],
                    placing_position=state["placing"],
                    current_joint_positions=current_joints,
                    end_effector_offset=EE_OFFSET,
                )
                robot.apply_action(actions)

                if controller.is_done():
                    print(f"\n[완료] {state['selected']} 큐브 → 마커. 첫 위치로 복귀 중…")
                    state["phase"] = PHASE_RETURN
                    state["return_steps"] = 0
                    state["return_from"] = robot.get_joint_positions().copy()

            # ── (RETURN) 첫 위치(관찰 포즈)로 '천천히' 복귀 (순간이동/스냅 X) ──
            #    복귀 동안엔 rgb 발행 안 함. 홈에 도착해야 다음 큐브를 스폰한다.
            elif phase == PHASE_RETURN:
                state["return_steps"] += 1
                # 현재 자세 → 관찰 포즈 선형 보간 → 하드 스냅/이상 자세 방지
                alpha = min(1.0, state["return_steps"] / RETURN_DURATION)
                target = (1.0 - alpha) * state["return_from"] + alpha * observe_full
                robot.apply_action(ArticulationAction(joint_positions=target))

                arm_now = robot.get_joint_positions()[arm_idx]
                reached = alpha >= 1.0 and np.max(np.abs(arm_now - observe_arm)) < RETURN_TOL
                if reached or state["return_steps"] >= RETURN_MAX_STEPS:
                    print(f"  [복귀] 첫 위치 도착 → 큐브 스폰 "
                          f"({'ok' if reached else 'timeout'})")
                    start_new_round()               # 홈 도착 후에만 스폰 + 발행 재개(다음 DETECT)

        was_playing = is_playing

    ros_node.destroy_node()
    rclpy.shutdown()
    simulation_app.close()


if __name__ == "__main__":
    main()
