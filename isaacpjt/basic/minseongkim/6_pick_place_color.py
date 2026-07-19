
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

from isaacsim.core.utils.extensions import enable_extension
enable_extension("isaacsim.ros2.bridge")
simulation_app.update()

from pathlib import Path
import random
import sys
import time

import numpy as np
import omni.usd
from pxr import Usd, UsdGeom, UsdPhysics

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid, VisualCuboid
from isaacsim.core.api.tasks import BaseTask
from isaacsim.core.api.materials.physics_material import PhysicsMaterial
from isaacsim.core.prims import SingleGeometryPrim
from isaacsim.robot.manipulators.grippers import ParallelGripper
from isaacsim.robot.manipulators.manipulators import SingleManipulator

_THIS_DIR = Path(__file__).resolve().parent

# rmpflow 인프라 폴더 경로 등록 (인프라 파일 내부 import가 그대로 동작)
RMPFLOW_DIR = str(_THIS_DIR / "rmpflow")
if RMPFLOW_DIR not in sys.path:
    sys.path.insert(0, RMPFLOW_DIR)

from m0609_pick_place_controller import PickPlaceController

# ╔══════════════════════════════════════════════════════════════╗
# ║  A. Task 파라미터 (이전 장과 동일)                              ║
# ╚══════════════════════════════════════════════════════════════╝
USD_PATH        = str(_THIS_DIR / "Collected_m0609_camera/World0.usd")
ROBOT_PRIM_PATH = "/World/m0609"
EE_LINK_NAME    = "link_6"
GRIPPER_JOINTS  = ["finger_joint", "right_inner_knuckle_joint"]

DRIVE_STIFFNESS = 1e8
DRIVE_DAMPING   = 1e4
DRIVE_MAX_FORCE = 1e8

# 그리퍼 조인트는 팔과 같은 초고강성 drive를 쓰면 열고 닫을 때 순간적으로 큰 힘이 걸려
# 큐브를 놓는 순간 반발력으로 튕겨나간다. 50g짜리 작은 큐브를 쥐는 데는 훨씬 약한 힘으로
# 충분하므로 그리퍼 전용으로 훨씬 부드러운 drive 값을 따로 둔다.
GRIPPER_DRIVE_STIFFNESS = 1e4
GRIPPER_DRIVE_DAMPING   = 1e2
GRIPPER_DRIVE_MAX_FORCE = 20.0

GRIPPER_OPEN    = [0.0, 0.0]
GRIPPER_CLOSE   = [0.5, 0.5]
GRIPPER_DELTA   = [-0.5, -0.5]

FINGER_STATIC   = 1.8
FINGER_DYNAMIC  = 1.4
CUBE_STATIC     = 1.2
CUBE_DYNAMIC    = 1.0


# ╔══════════════════════════════════════════════════════════════╗
# ║  B. Controller 파라미터 (이전 장과 동일)                        ║
# ╚══════════════════════════════════════════════════════════════╝

# ── B-1. 인프라 파일 경로 (RMPFlow가 참조) ────────────────────
M0609_URDF_PATH           = str(_THIS_DIR / "doosan-robot2/urdf/m0609_isaac_sim.urdf")
M0609_DESCRIPTION_PATH    = str(_THIS_DIR / "rmpflow/m0609_description.yaml")
M0609_RMPFLOW_CONFIG_PATH = str(_THIS_DIR / "rmpflow/m0609_rmpflow_common.yaml")

EE_OFFSET = np.array([0.0, 0.0, 0.2])   # 접근 높이

# ── B-2. 10단계 타이밍 (작을수록 빠름) ────────────────────────
EVENTS_DT = [
    0.008,   # 0. 접근 이동
    0.005,   # 1. 하강
    0.02,    # 2. 그리퍼 닫기 대기
    0.1,     # 3. 그리퍼 닫힘 유지
    0.0025,  # 4. 들어올리기
    0.01,    # 5. Place 위치로 이동
    0.0025,  # 6. 하강
    1,       # 7. 그리퍼 열기 대기
    0.008,   # 8. 상승
    0.08,    # 9. 복귀
]


# ╔══════════════════════════════════════════════════════════════╗
# ║  C. 색상 시나리오 파라미터 (★ 이번 장에서 새로 추가)            ║
# ╚══════════════════════════════════════════════════════════════╝
DROP_HEIGHT = 0.20  # 두 큐브가 스폰되는 공중 높이 (m)

# 대기(공중에 떠 있는) 큐브의 xy 위치 — 물리 비활성화 상태로 DROP_HEIGHT에 정지
# (+x, -y 영역에만 스폰되도록 y는 항상 음수)
STANDBY_XY = {
    "blue":  (0.20, -0.35),
    "green": (0.30, -0.60),
}

# pick 영역 (선택된 큐브가 랜덤 오프셋을 받아 낙하할 xy 중심, +x, -y 영역)
PICK_AREA_CENTER_XY = (0.30, -0.40)
PICK_AREA_JITTER = 0.03   # ± 랜덤 오프셋 (m)

# 색상별 place 마커 위치 (색상 ID로 결정)
BLUE_GOAL_POS  = np.array([0.55, -0.35, 0.0])
GREEN_GOAL_POS = np.array([0.55, -0.15, 0.0])

COLOR_ID_TOPIC = "/color_id"
COLOR_ID_BLUE  = 1
COLOR_ID_GREEN = 2
GOAL_POS_BY_COLOR_ID = {
    COLOR_ID_BLUE:  BLUE_GOAL_POS,
    COLOR_ID_GREEN: GREEN_GOAL_POS,
}

DROP_SETTLE_STEPS  = 90   # 낙하 후 착지를 기다리는 스텝 수 (약 1.5초)
ROUND_SETTLE_STEPS = 20   # 라운드 사이 안정화 대기 스텝 수


# ============================================================
# 유틸 (이전 장과 동일)
# ============================================================
def find_prim_path_by_name(root_path: str, name: str):
    """root_path 하위에서 이름이 name인 prim을 찾아 경로를 반환"""
    stage = omni.usd.get_context().get_stage()
    root_prim = stage.GetPrimAtPath(root_path)
    if not root_prim.IsValid():
        return None
    for prim in Usd.PrimRange(root_prim):
        if prim.GetName() == name:
            return str(prim.GetPath())
    return None


def initialize_robot(robot, world):
    """로봇/그리퍼 초기화 후 관절을 0 위치로 세팅 (RMPFlow가 phase 0에서 pick 자세로 움직여감)"""
    robot.initialize()
    robot.gripper.initialize(
        physics_sim_view=world.physics_sim_view,
        articulation_apply_action_func=robot.apply_action,
        get_joint_positions_func=robot.get_joint_positions,
        set_joint_positions_func=robot.set_joint_positions,
        dof_names=robot.dof_names,
    )
    robot.set_joint_positions(np.zeros(robot.num_dof))


def random_pick_xy():
    """PICK_AREA_CENTER_XY 주변으로 ± PICK_AREA_JITTER 랜덤 오프셋을 준 (x, y)를 반환"""
    offset_xy = np.random.uniform(-PICK_AREA_JITTER, PICK_AREA_JITTER, size=2)
    return (
        PICK_AREA_CENTER_XY[0] + offset_xy[0],
        PICK_AREA_CENTER_XY[1] + offset_xy[1],
    )


def set_cube_gravity(cube, enabled: bool):
    """큐브의 중력 반응 여부를 토글 (rigidBodyEnabled 자체는 건드리지 않아 tensor view가 계속 유효함)

    SingleRigidPrim/DynamicCuboid는 gravity 토글용 공개 wrapper가 없어
    내부 배치 뷰(_rigid_prim_view)의 enable_gravities/disable_gravities를 직접 사용한다.
    """
    if enabled:
        cube._rigid_prim_view.enable_gravities()
    else:
        cube._rigid_prim_view.disable_gravities()


# ============================================================
# ROS2 — PC B가 발행하는 /color_id(Int32) 구독 (★ 이번 장 신규)
# ============================================================
class ColorIdSubscriber(Node):
    def __init__(self):
        """/color_id 토픽을 구독하고 최신 값을 보관"""
        super().__init__("color_id_subscriber")
        self._latest = None
        self.create_subscription(Int32, COLOR_ID_TOPIC, self._callback, 10)

    def _callback(self, msg):
        """수신한 색상 ID를 최신값으로 저장"""
        self._latest = msg.data

    def pop_latest(self):
        """보관 중인 최신 색상 ID를 꺼내고(consume) 내부 상태를 비움"""
        value = self._latest
        self._latest = None
        return value


# ============================================================
# Task — 이전 장과 동일한 구조, 큐브/마커만 2색으로 확장
# ============================================================
class M0609Task(BaseTask):

    def __init__(self, name):
        """task 이름 설정"""
        super().__init__(name=name, offset=None)

    def set_up_scene(self, scene):
        """USD 로드 → 링크 탐색 → 물리 설정 → 로봇 등록 → 씬 구성 순서로 전체 씬을 구성"""
        super().set_up_scene(scene)
        self._load_usd()
        self._discover_links()
        self._setup_physics()
        self._register_robot(scene)
        self._create_scene(scene)
        print("\n  [완료] 씬 구성 성공!\n")

    def _load_usd(self):
        """World0.usd 씬 파일을 /World 프림에 참조로 로드"""
        print("\n" + "=" * 60)
        print("[1.LOAD] USD 로드")
        print("=" * 60)
        stage = omni.usd.get_context().get_stage()
        world_prim = stage.GetPrimAtPath("/World")
        if not world_prim.IsValid():
            world_prim = UsdGeom.Xform.Define(stage, "/World").GetPrim()
        world_prim.GetReferences().AddReference(USD_PATH)
        for _ in range(15):
            simulation_app.update()
        print(f"  [OK] {USD_PATH}")

    def _discover_links(self):
        """end-effector 링크와 그리퍼 조인트들의 prim 경로를 탐색해 저장"""
        print("\n" + "=" * 60)
        print("[2.DISCOVER] 링크 경로 탐색")
        print("=" * 60)
        self._ee_path = find_prim_path_by_name(ROBOT_PRIM_PATH, EE_LINK_NAME)
        if self._ee_path is None:
            raise RuntimeError(f"'{EE_LINK_NAME}' not found")
        print(f"  EE ({EE_LINK_NAME}) = {self._ee_path}")
        for jn in GRIPPER_JOINTS:
            print(f"  {jn:<35} = {find_prim_path_by_name(ROBOT_PRIM_PATH, jn)}")

    def _setup_physics(self):
        """로봇의 관절 drive(강성/댐핑/최대 힘)를 설정 — 그리퍼 조인트는 더 부드러운 값 사용"""
        print("\n" + "=" * 60)
        print("[3.PHYSICS] 물리 설정")
        print("=" * 60)
        stage = omni.usd.get_context().get_stage()

        drive_count = 0
        for prim in Usd.PrimRange(stage.GetPrimAtPath(ROBOT_PRIM_PATH)):
            for dt in ["angular", "linear"]:
                drive = UsdPhysics.DriveAPI.Get(prim, dt)
                if drive:
                    if prim.GetName() in GRIPPER_JOINTS:
                        stiffness, damping, max_force = (
                            GRIPPER_DRIVE_STIFFNESS,
                            GRIPPER_DRIVE_DAMPING,
                            GRIPPER_DRIVE_MAX_FORCE,
                        )
                    else:
                        stiffness, damping, max_force = DRIVE_STIFFNESS, DRIVE_DAMPING, DRIVE_MAX_FORCE
                    drive.GetStiffnessAttr().Set(stiffness)
                    drive.GetDampingAttr().Set(damping)
                    drive.GetMaxForceAttr().Set(max_force)
                    drive_count += 1
        print(f"  [OK] drive updated: {drive_count}")

    def _register_robot(self, scene):
        """ParallelGripper를 만들고 SingleManipulator로 씬에 로봇을 등록"""
        print("\n" + "=" * 60)
        print("[4.REGISTER] 로봇 등록")
        print("=" * 60)
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
        """파랑/초록 큐브 2개, 파랑/초록 place 마커 2개, 그리퍼 손가락 마찰 재질을 씬에 추가"""
        print("\n" + "=" * 60)
        print("[5.SCENE] 작업 환경 구성")
        print("=" * 60)
        cube_material = PhysicsMaterial(
            prim_path="/World/Physics_Materials/cube_material",
            static_friction=CUBE_STATIC,
            dynamic_friction=CUBE_DYNAMIC,
            restitution=0.0,
        )

        self._cubes = {}
        cube_specs = [
            ("blue",  "/World/cube_blue",  [0.0, 0.0, 1.0]),
            ("green", "/World/cube_green", [0.0, 1.0, 0.0]),
        ]

        for color_name, prim_path, rgb in cube_specs:
            standby_xy = STANDBY_XY[color_name]
            spawn_pos = np.array([standby_xy[0], standby_xy[1], DROP_HEIGHT])
            cube = scene.add(
                DynamicCuboid(
                    prim_path=prim_path,
                    name=f"cube_{color_name}",
                    position=spawn_pos,
                    scale=np.array([0.05, 0.05, 0.05]),
                    color=np.array(rgb),
                    mass=0.05,
                    physics_material=cube_material,
                )
            )
            set_cube_gravity(cube, enabled=False)  # 초기에는 공중에 정지(대기)
            self._cubes[color_name] = cube
            print(f"  [OK] {color_name} cube @ {spawn_pos} (floating)")

        marker_specs = [
            ("blue",  "/World/marker_blue",  BLUE_GOAL_POS,  [0.0, 0.0, 1.0]),
            ("green", "/World/marker_green", GREEN_GOAL_POS, [0.0, 1.0, 0.0]),
        ]
        for color_name, prim_path, goal_pos, rgb in marker_specs:
            scene.add(
                VisualCuboid(
                    prim_path=prim_path,
                    name=f"marker_{color_name}",
                    position=goal_pos,
                    scale=np.array([0.06, 0.06, 0.001]),
                    color=np.array(rgb),
                )
            )
            print(f"  [OK] {color_name} marker @ {goal_pos}")

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
                print(f"  [OK] friction: {link_path}")

    def get_observations(self):
        """컨트롤러/루프에 전달할 로봇 관절값과 두 큐브의 현재 위치를 반환"""
        obs = {
            self._robot.name: {
                "joint_positions": self._robot.get_joint_positions(),
            }
        }
        for cube in self._cubes.values():
            position, _ = cube.get_world_pose()
            obs[cube.name] = {"position": position}
        return obs

    def drop_cube(self, color_name, xy):
        """지정 큐브를 xy 위 DROP_HEIGHT에 놓고 중력을 켜서 자유낙하시킴"""
        cube = self._cubes[color_name]
        cube.set_world_pose(position=np.array([xy[0], xy[1], DROP_HEIGHT]))
        cube.set_linear_velocity(np.zeros(3))
        cube.set_angular_velocity(np.zeros(3))
        set_cube_gravity(cube, enabled=True)

    def float_cube(self, color_name):
        """지정 큐브를 자신의 STANDBY_XY 위 DROP_HEIGHT에 놓고 중력을 꺼서 공중에 고정"""
        cube = self._cubes[color_name]
        standby_xy = STANDBY_XY[color_name]
        cube.set_world_pose(position=np.array([standby_xy[0], standby_xy[1], DROP_HEIGHT]))
        cube.set_linear_velocity(np.zeros(3))
        cube.set_angular_velocity(np.zeros(3))
        set_cube_gravity(cube, enabled=False)

    def post_reset(self):
        """리셋 시 그리퍼를 열고 두 큐브를 각자의 대기 위치(공중)로 되돌림"""
        self._robot.gripper.set_joint_positions(
            self._robot.gripper.joint_opened_positions
        )
        self.float_cube("blue")
        self.float_cube("green")


# ╔══════════════════════════════════════════════════════════════╗
# ║  D. 메인 — 라운드 반복 Pick & Place 시나리오 (★ 이번 장 핵심)  ║
# ╚══════════════════════════════════════════════════════════════╝

def main():
    """World/Task/Robot/Controller/ROS2 구독자를 초기화하고 색상 시나리오 루프를 실행"""
    # ── D-1. World + Task ──────────────────────────────────
    my_world = World(stage_units_in_meters=1.0)
    task = M0609Task(name="m0609_task")
    my_world.add_task(task)
    my_world.reset()

    robot = my_world.scene.get_object("m0609_robot")
    initialize_robot(robot, my_world)

    # 홈 포지션 안정화 대기
    for _ in range(30):
        my_world.step(render=True)

    # ── D-2. Controller 생성 (initialize 이후에만 가능) ───────
    print("\n" + "=" * 60)
    print("[D-2] PickPlaceController 생성")
    print("=" * 60)
    print(f"  URDF        = {M0609_URDF_PATH}")
    print(f"  description = {M0609_DESCRIPTION_PATH}")
    print(f"  rmpflow     = {M0609_RMPFLOW_CONFIG_PATH}")
    print(f"  events_dt   = {EVENTS_DT}")
    print(f"  EE frame    = {EE_LINK_NAME}")

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

    # ── D-3. ROS2 /color_id 구독자 (PC B가 발행) ─────────────
    rclpy.init()
    color_sub = ColorIdSubscriber()
    print(f"  [OK] '{COLOR_ID_TOPIC}' 구독 시작 (PC B에서 파랑=1, 초록=2 발행 예정)")

    # ── D-4. 한 라운드 = 파랑 큐브 + 초록 큐브를 둘 다 옮겨야 끝남 ──
    # WAIT_PC_B   : 라운드 시작 시 PC B(color_id_publisher)가 /color_id에 실제로 붙을 때까지
    #               제자리 대기. 라운드당 한 번만 확인하고, 두 번째 큐브로 넘어갈 땐 재확인하지 않음
    # SELECT      : 아직 옮기지 않은 큐브 중 하나를 랜덤 선택해 pick 영역 위(DROP_HEIGHT)에서
    #               낙하시키고, 나머지 하나는 대기 xy 위에 공중 정지(float)시킴
    # DROP_SETTLE : 낙하한 큐브가 테이블에 착지해 멈출 때까지 대기
    # PICK_PLACE  : 큐브 위치는 이미 알고 있으므로 색상 수신과 무관하게 즉시 접근/그립/
    #               들어올리기를 시작. place 위치가 실제로 필요한 5단계 직전에
    #               /color_id가 아직 없으면 controller.pause()로 잠깐만 대기
    # CLEANUP     : 방금 옮긴 큐브와 대기 중인 큐브를 공중으로 되돌림.
    #               아직 안 옮긴 큐브가 남아있으면 (재확인 없이) 바로 SELECT로 돌아가 이어서 진행,
    #               둘 다 옮겼으면 DONE으로 라운드 종료
    # DONE        : 파랑/초록 큐브를 각자의 마커에 모두 배치 완료 → 더 이상 아무 것도 안 함
    PLACE_EVENT_INDEX = 5  # 이 단계부터 place 목표 위치가 실제로 사용됨
    PC_B_CHECK_INTERVAL_STEPS = 60  # 대기 메시지 출력 주기 (약 1초)

    def pc_b_connected():
        """PC B의 color_id_publisher 노드가 /color_id에 실제로 구독/발행 연결돼 있는지 확인"""
        return color_sub.count_publishers(COLOR_ID_TOPIC) > 0

    round_num = 0
    trial_phase = "WAIT_PC_B"
    active_color = None
    inactive_color = None
    goal_pos = None
    settle_counter = 0
    wait_pc_b_counter = 0
    remaining_colors = ["blue", "green"]  # 이 세션에서 아직 배치하지 않은 큐브 색

    print("\n[색상 Pick & Place 시나리오 시작]\n")
    was_playing = False

    while simulation_app.is_running():
        my_world.step(render=True)
        time.sleep(0.01)
        rclpy.spin_once(color_sub, timeout_sec=0.0)
        is_playing = my_world.is_playing()

        # Play 시작 감지 → 리셋
        if is_playing and not was_playing:
            my_world.reset()
            initialize_robot(robot, my_world)
            controller.reset()
            color_sub.pop_latest()
            round_num = 0
            trial_phase = "WAIT_PC_B"
            active_color = None
            inactive_color = None
            goal_pos = None
            settle_counter = 0
            wait_pc_b_counter = 0
            remaining_colors = ["blue", "green"]

        if is_playing:
            if trial_phase == "WAIT_PC_B":
                if pc_b_connected():
                    print(f"  [OK] PC B 연결 확인됨 ('{COLOR_ID_TOPIC}') → 시나리오 시작")
                    trial_phase = "SELECT"
                else:
                    if wait_pc_b_counter % PC_B_CHECK_INTERVAL_STEPS == 0:
                        print(f"  [대기] PC B 연결 대기 중... ('{COLOR_ID_TOPIC}'에 발행자 없음)")
                    wait_pc_b_counter += 1

            elif trial_phase == "SELECT":
                round_num += 1
                is_last_cube = len(remaining_colors) == 1  # 이번이 이 라운드의 마지막 큐브
                active_color = random.choice(remaining_colors)
                remaining_colors.remove(active_color)
                inactive_color = "green" if active_color == "blue" else "blue"
                color_sub.pop_latest()  # 이전 큐브의 낡은 값 폐기

                if is_last_cube:
                    # 마지막 남은 큐브는 낙하시키지 않고, 공중에 뜬 상태 그대로 바로 집는다
                    print(f"\n[{round_num}/2번째 큐브] {active_color} 큐브를 공중에서 바로 Pick 시작")
                    goal_pos = None
                    trial_phase = "PICK_PLACE"
                else:
                    pick_xy = random_pick_xy()
                    task.drop_cube(active_color, pick_xy)      # 낙하
                    task.float_cube(inactive_color)             # 공중 대기 유지
                    print(f"\n[{round_num}/2번째 큐브] {active_color} 큐브 낙하 시작 (pick xy={pick_xy})")
                    settle_counter = 0
                    trial_phase = "DROP_SETTLE"

            elif trial_phase == "DROP_SETTLE":
                settle_counter += 1
                if settle_counter >= DROP_SETTLE_STEPS:
                    print(f"  [OK] {active_color} 큐브 착지 완료 → Pick 시작 (색상 수신과 무관)")
                    goal_pos = None
                    trial_phase = "PICK_PLACE"

            elif trial_phase == "PICK_PLACE":
                # 색상 ID는 place 단계(PLACE_EVENT_INDEX)에만 필요하므로,
                # 도착하는 대로 저장해두고 아직 없어도 pick 동작은 계속 진행한다.
                # goal_pos가 한 번 정해지면 이 큐브를 다 옮길 때까지 고정한다 — 그렇지 않으면
                # 이동 중에 카메라가 (같은 색의) 마커를 잠깐 보고 오인식해도 목표가 계속 덮어써져
                # 엉뚱한 마커로 방향을 트는 문제가 생긴다.
                color_id = color_sub.pop_latest()
                if goal_pos is None and color_id in GOAL_POS_BY_COLOR_ID:
                    goal_pos = GOAL_POS_BY_COLOR_ID[color_id]
                    if controller.is_paused():
                        controller.resume()
                        print(f"  [OK] /color_id={color_id} 수신 → place 목표 {goal_pos}, 이동 재개")
                    else:
                        print(f"  [OK] /color_id={color_id} 수신 → place 목표 {goal_pos}")

                obs = task.get_observations()
                cube_position = obs[f"cube_{active_color}"]["position"]
                current_joints = obs["m0609_robot"]["joint_positions"]

                # place 단계 직전인데 아직 색상을 모르면 잠깐 멈추고 기다림
                if goal_pos is None and controller.get_current_event() >= PLACE_EVENT_INDEX:
                    if not controller.is_paused():
                        controller.pause()
                        print("  [대기] /color_id 미수신 → place 이동 보류")

                actions = controller.forward(
                    picking_position=cube_position,
                    placing_position=goal_pos if goal_pos is not None else np.zeros(3),
                    current_joint_positions=current_joints,
                    end_effector_offset=EE_OFFSET,
                )
                robot.apply_action(actions)

                event = controller.get_current_event()
                ee_pos, _ = robot.end_effector.get_world_pose()
                print(f"  [event={event}] cube_z={cube_position[2]:.4f}  ee_z={ee_pos[2]:.4f}")

                if controller.is_done():
                    print(f"  [완료] {active_color} 큐브 Pick & Place 성공!")
                    trial_phase = "CLEANUP"
                    settle_counter = 0

            elif trial_phase == "CLEANUP":
                # 방금 마커에 놓은 active_color 큐브는 그대로 둔다 (다시 띄우면 배치가 취소됨).
                # inactive_color는 이미 공중에 떠 있는 상태라 손댈 필요 없음 — controller만 초기화.
                if settle_counter == 0:
                    controller.reset()
                settle_counter += 1
                if settle_counter >= ROUND_SETTLE_STEPS:
                    if remaining_colors:
                        # 같은 라운드 안에서 이어서 진행 — PC B는 이미 연결 확인했으므로 재확인 없음
                        trial_phase = "SELECT"
                    else:
                        print("\n[라운드 종료] 파랑/초록 큐브를 각자의 마커에 모두 배치 완료!\n")
                        my_world.pause()
                        trial_phase = "DONE"

            elif trial_phase == "DONE":
                pass  # 두 큐브 모두 배치 완료 — Play를 다시 누르면 새 세션으로 리셋됨

        was_playing = is_playing

    color_sub.destroy_node()
    rclpy.shutdown()
    simulation_app.close()


if __name__ == "__main__":
    main()
