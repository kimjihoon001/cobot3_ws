
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

from isaacsim.core.utils.extensions import enable_extension
enable_extension("isaacsim.ros2.bridge")
simulation_app.update()

from pathlib import Path
import sys
import time

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
from isaacsim.sensors.camera import Camera
import isaacsim.core.utils.numpy.rotations as rot_utils

_THIS_DIR = Path(__file__).resolve().parent

# rmpflow 인프라 폴더 경로 등록
RMPFLOW_DIR = str(_THIS_DIR / "rmpflow")
if RMPFLOW_DIR not in sys.path:
    sys.path.insert(0, RMPFLOW_DIR)

from m0609_pick_place_controller import PickPlaceController

# ╔══════════════════════════════════════════════════════════════╗
# ║  A. Task 파라미터 (4번과 동일)                                  ║
# ╚══════════════════════════════════════════════════════════════╝
USD_PATH        = str(_THIS_DIR / "Collected_m0609_camera_view/Collected_m0609_camera_view.usd")
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
M0609_URDF_PATH           = str(_THIS_DIR / "doosan-robot2/urdf/m0609_isaac_sim.urdf")
M0609_DESCRIPTION_PATH    = str(_THIS_DIR / "rmpflow/m0609_description.yaml")
M0609_RMPFLOW_CONFIG_PATH = str(_THIS_DIR / "rmpflow/m0609_rmpflow_common.yaml")

# ── Pick & Place 동작 파라미터 ───────────────────────────────
EE_OFFSET     = np.array([0.0, 0.0, 0.2])
EVENTS_DT = [0.008, 0.005, 0.02, 0.1, 0.0025, 0.01, 0.0025, 1, 0.008, 0.08]


# ╔══════════════════════════════════════════════════════════════╗
# ║  B. 색상 Pick&Place / 카메라 파라미터 (★ 이번 파일 핵심)       ║
# ╚══════════════════════════════════════════════════════════════╝
CUBE_SIZE   = 0.05
CUBE_CENTER_Z = CUBE_SIZE / 2.0     # 바닥에 놓인 큐브의 중심 높이 (picking z)
CUBE_TOP_Z    = CUBE_SIZE           # 큐브 윗면 높이 (카메라가 보는 표면)

# 색상별 큐브 초기 위치(x, y) 와 색(RGB, 0~1)
COLORS = ["red", "green", "blue"]
CUBE_SPECS = {
    "red":   {"pos": (0.35,  0.30), "rgb": np.array([1.0, 0.0, 0.0])},
    "green": {"pos": (0.35,  0.00), "rgb": np.array([0.0, 1.0, 0.0])},
    "blue":  {"pos": (0.35, -0.30), "rgb": np.array([0.0, 0.0, 1.0])},
}
# 같은 색 마커 위치(일부러 섞어서 → 색 매칭이 의미 있게)
MARKER_POS = {
    "red":   np.array([0.58, -0.30, 0.0]),
    "green": np.array([0.58,  0.30, 0.0]),
    "blue":  np.array([0.58,  0.00, 0.0]),
}

# ── 탑뷰 카메라 ──────────────────────────────────────────────
CAM_PRIM_PATH = "/World/top_camera"
CAM_POS       = np.array([0.40, 0.0, 1.5])          # 워크스페이스 위 정면 하방
CAM_RES       = (640, 480)                           # (width, height)
# 탑뷰이므로 광축 방향 depth는 상수: 카메라 높이 - 큐브 윗면
CAM_DEPTH     = float(CAM_POS[2] - CUBE_TOP_Z)
MIN_BLOB_PX   = 30                                   # 이 픽셀 수 미만이면 미검출


# ============================================================
# 유틸 (4번과 동일)
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
    robot.set_joint_positions(np.zeros(robot.num_dof))


# ============================================================
# 색상 인식 (numpy 기반 - cv2 불필요)
# ============================================================
def detect_color_centroids(camera):
    """카메라 RGB에서 각 색의 픽셀 중심(u, v)을 찾아 월드 좌표로 변환.

    반환: {color: np.array([x, y, z])}  (검출된 색만 포함)
    """
    rgba = camera.get_rgba()
    if rgba is None or rgba.size == 0:
        return {}

    rgb = rgba[:, :, :3].astype(np.float32)
    r, g, b = rgb[:, :, 0], rgb[:, :, 1], rgb[:, :, 2]

    # 색이 뚜렷한(어둡지 않은) 픽셀에서 어떤 채널이 우세한지로 분류
    masks = {
        "red":   (r > g * 1.4) & (r > b * 1.4) & (r > 80),
        "green": (g > r * 1.4) & (g > b * 1.4) & (g > 80),
        "blue":  (b > r * 1.4) & (b > g * 1.4) & (b > 80),
    }

    result = {}
    for color, mask in masks.items():
        ys, xs = np.nonzero(mask)          # (row=v, col=u)
        if ys.size < MIN_BLOB_PX:
            continue
        u = float(xs.mean())
        v = float(ys.mean())
        # 픽셀 → 월드 (카메라 실제 pose/intrinsic 사용)
        world_pt = camera.get_world_points_from_image_coords(
            np.array([[u, v]]), np.array([CAM_DEPTH])
        )[0]
        # x, y는 카메라에서 인식, z는 큐브 규격값으로(그랩 안정성)
        result[color] = np.array([world_pt[0], world_pt[1], CUBE_CENTER_Z])
        print(f"  [detect] {color:<5} px=({u:.0f},{v:.0f}) -> world=({world_pt[0]:.3f}, {world_pt[1]:.3f})")
    return result


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
        self._discover_links()
        self._setup_physics()
        self._register_robot(scene)
        self._create_scene(scene)
        self._create_camera()
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
        print("\n[5.SCENE] 색상 큐브 + 마커 구성")
        cube_material = PhysicsMaterial(
            prim_path="/World/Physics_Materials/cube_material",
            static_friction=CUBE_STATIC,
            dynamic_friction=CUBE_DYNAMIC,
            restitution=0.0,
        )
        for color in COLORS:
            spec = CUBE_SPECS[color]
            x, y = spec["pos"]
            pos = np.array([x, y, CUBE_CENTER_Z])
            self._cubes[color] = scene.add(
                DynamicCuboid(
                    prim_path=f"/World/cube_{color}",
                    name=f"cube_{color}",
                    position=pos,
                    scale=np.array([CUBE_SIZE, CUBE_SIZE, CUBE_SIZE]),
                    color=spec["rgb"],
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
                    color=spec["rgb"],
                )
            )
            print(f"  [OK] {color:<5} cube @ ({x}, {y})  marker @ {MARKER_POS[color][:2]}")

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

    def _create_camera(self):
        print("\n[6.CAMERA] 탑뷰 카메라 생성")
        # 탑뷰(수직 하방)를 향하도록: Y축 기준 90도 피치
        self.camera = Camera(
            prim_path=CAM_PRIM_PATH,
            position=CAM_POS,
            frequency=20,
            resolution=CAM_RES,
            orientation=rot_utils.euler_angles_to_quats(
                np.array([0, 90, 0]), degrees=True
            ),
        )
        print(f"  [OK] camera @ {CAM_POS}  res={CAM_RES}  depth={CAM_DEPTH:.3f}")

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

    print("\n[색상 인식 Pick & Place 시작]  (Play 버튼을 누르세요)\n")
    was_playing = False
    perception_done = False
    queue = []                 # 처리할 색 목록
    pick_targets = {}          # color -> 카메라로 검출한 월드 좌표

    while simulation_app.is_running():
        my_world.step(render=True)
        time.sleep(0.01)
        is_playing = my_world.is_playing()

        # Play 시작 감지 → 리셋
        if is_playing and not was_playing:
            my_world.reset()
            initialize_robot(robot, my_world)
            controller.reset()
            perception_done = False
            queue = []
            pick_targets = {}

        if is_playing:
            # ── (1) 최초 1회: 카메라로 색상 인식 → 처리 큐 구성 ──
            if not perception_done:
                print("\n[PERCEPTION] 카메라 색상 인식")
                pick_targets = detect_color_centroids(task.camera)
                queue = [c for c in COLORS if c in pick_targets]
                perception_done = True
                if not queue:
                    print("  [경고] 검출된 색이 없습니다. 카메라 화각/색상을 확인하세요.")
                else:
                    print(f"  [OK] 처리 순서: {queue}\n")

            # ── (2) 큐 순서대로 pick & place ──
            if queue:
                color = queue[0]
                current_joints = robot.get_joint_positions()
                actions = controller.forward(
                    picking_position=pick_targets[color],
                    placing_position=MARKER_POS[color],
                    current_joint_positions=current_joints,
                    end_effector_offset=EE_OFFSET,
                )
                robot.apply_action(actions)

                event = controller.get_current_event()
                print(f"  [{color}] event={event}")

                if controller.is_done():
                    print(f"  [완료] {color} 큐브 → {color} 마커")
                    queue.pop(0)
                    controller.reset()
                    if not queue:
                        print("\n[전체 완료] 모든 색상 Pick & Place 성공!")
                        my_world.pause()

        was_playing = is_playing

    simulation_app.close()


if __name__ == "__main__":
    main()
