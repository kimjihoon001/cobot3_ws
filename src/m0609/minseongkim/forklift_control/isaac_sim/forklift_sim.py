"""Isaac Sim Forklift C를 ROS 2와 키보드로 조종하는 standalone 예제.

실행:
    source <isaac_sim>/setup_ros_env.sh
    isaac_python 8_forklift_ros.py

ROS 2 입력:
    /forklift/cmd_vel  (geometry_msgs/msg/Twist)
    /forklift/lift     (std_msgs/msg/Float32, +1 상승 / -1 하강)

키보드 조작(ROS보다 우선):
    W / S       전진 / 후진
    A / D       좌회전 / 우회전
    I / K       포크 올리기 / 내리기
    Space       즉시 정지
"""

from isaacsim import SimulationApp


simulation_app = SimulationApp({"headless": False})

from isaacsim.core.utils.extensions import enable_extension


enable_extension("isaacsim.ros2.bridge")
simulation_app.update()

import time

import carb
import numpy as np
import omni.appwindow
import omni.graph.core as og
import omni.usd
import rclpy
import usdrt.Sdf
from geometry_msgs.msg import Twist
from isaacsim.core.api import World
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.utils.viewports import set_camera_view
from isaacsim.storage.native import get_assets_root_path
from rclpy.node import Node
from std_msgs.msg import Float32


FORKLIFT_PRIM_PATH = "/World/Forklift"
FORKLIFT_USD_RELATIVE_PATH = "/Isaac/Robots/IsaacSim/ForkliftC/forklift_c.usd"

DRIVE_SPEED = 6.0          # 구동 바퀴 목표 각속도(rad/s)
STEERING_ANGLE = 25.0      # 최대 조향각(deg, USD Drive targetPosition 단위)
LIFT_MIN = 0.0             # 리프트 최저 위치(m)
LIFT_MAX = 2.0             # 리프트 최고 위치(m)
LIFT_SPEED = 0.7           # 초당 리프트 목표 위치 변화량(m/s)

MAX_LINEAR_COMMAND = 1.0   # Twist.linear.x 입력 절댓값 상한
MAX_ANGULAR_COMMAND = 1.0  # Twist.angular.z 입력 절댓값 상한
ROS_COMMAND_TIMEOUT = 0.5  # 이 시간 동안 새 메시지가 없으면 자동 정지(s)

GRAPH_PATH = "/ForkliftKeyboardGraph"


class ForkliftRosNode(Node):
    """주행 및 리프트 ROS 2 명령의 최신값을 보관한다."""

    def __init__(self):
        super().__init__("isaac_forklift")
        self.linear = 0.0
        self.angular = 0.0
        self.lift = 0.0
        self.last_drive_time = None
        self.last_lift_time = None

        self.create_subscription(Twist, "/forklift/cmd_vel", self._on_cmd_vel, 10)
        self.create_subscription(Float32, "/forklift/lift", self._on_lift, 10)

    def _on_cmd_vel(self, msg):
        self.linear = float(msg.linear.x)
        self.angular = float(msg.angular.z)
        self.last_drive_time = time.monotonic()

    def _on_lift(self, msg):
        self.lift = float(msg.data)
        self.last_lift_time = time.monotonic()

    def get_commands(self):
        """오래된 명령은 0으로 바꿔 지게차가 계속 움직이지 않게 한다."""
        now = time.monotonic()
        drive_fresh = (
            self.last_drive_time is not None
            and now - self.last_drive_time <= ROS_COMMAND_TIMEOUT
        )
        lift_fresh = (
            self.last_lift_time is not None
            and now - self.last_lift_time <= ROS_COMMAND_TIMEOUT
        )
        if drive_fresh:
            linear, angular = self.linear, self.angular
        else:
            linear, angular = 0.0, 0.0
        return linear, angular, self.lift if lift_fresh else 0.0


class KeyboardState:
    """동시에 누른 키 상태를 유지하는 키보드 입력 처리기."""

    def __init__(self):
        self.pressed = set()
        app_window = omni.appwindow.get_default_app_window()
        self._input = carb.input.acquire_input_interface()
        self._keyboard = app_window.get_keyboard()
        self._subscription = self._input.subscribe_to_keyboard_events(
            self._keyboard, self._on_keyboard_event
        )

    def _on_keyboard_event(self, event, *args, **kwargs):
        key_name = event.input.name
        if event.type in (
            carb.input.KeyboardEventType.KEY_PRESS,
            carb.input.KeyboardEventType.KEY_REPEAT,
        ):
            self.pressed.add(key_name)
        elif event.type == carb.input.KeyboardEventType.KEY_RELEASE:
            self.pressed.discard(key_name)
        return True

    def is_down(self, key_name):
        return key_name in self.pressed

    def close(self):
        if self._subscription is not None:
            self._input.unsubscribe_to_keyboard_events(
                self._keyboard, self._subscription
            )
            self._subscription = None


def create_control_graph():
    """Forklift C의 바퀴, 조향, 리프트를 구동하는 Action Graph를 생성한다."""
    keys = og.Controller.Keys
    og.Controller.edit(
        {"graph_path": GRAPH_PATH, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("WheelController", "isaacsim.core.nodes.IsaacArticulationController"),
                ("WriteSteerLeft", "omni.graph.nodes.WritePrimAttribute"),
                ("WriteSteerRight", "omni.graph.nodes.WritePrimAttribute"),
                ("WriteLift", "omni.graph.nodes.WritePrimAttribute"),
                ("SteeringTarget", "omni.graph.nodes.ConstantDouble"),
                ("LiftTarget", "omni.graph.nodes.ConstantDouble"),
            ],
            keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "WheelController.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "WriteSteerLeft.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "WriteSteerRight.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "WriteLift.inputs:execIn"),
                ("SteeringTarget.inputs:value", "WriteSteerLeft.inputs:value"),
                ("SteeringTarget.inputs:value", "WriteSteerRight.inputs:value"),
                ("LiftTarget.inputs:value", "WriteLift.inputs:value"),
            ],
            keys.SET_VALUES: [
                ("WheelController.inputs:robotPath", FORKLIFT_PRIM_PATH),
                (
                    "WheelController.inputs:jointNames",
                    ["left_back_wheel_joint", "right_back_wheel_joint"],
                ),
                ("WheelController.inputs:velocityCommand", [0.0, 0.0]),
                (
                    "WriteSteerLeft.inputs:prim",
                    [usdrt.Sdf.Path(FORKLIFT_PRIM_PATH + "/left_rotator_joint")],
                ),
                (
                    "WriteSteerLeft.inputs:name",
                    "drive:angular:physics:targetPosition",
                ),
                (
                    "WriteSteerRight.inputs:prim",
                    [usdrt.Sdf.Path(FORKLIFT_PRIM_PATH + "/right_rotator_joint")],
                ),
                (
                    "WriteSteerRight.inputs:name",
                    "drive:angular:physics:targetPosition",
                ),
                (
                    "WriteLift.inputs:prim",
                    [usdrt.Sdf.Path(FORKLIFT_PRIM_PATH + "/lift_joint")],
                ),
                ("WriteLift.inputs:name", "drive:linear:physics:targetPosition"),
                ("SteeringTarget.inputs:value", 0.0),
                ("LiftTarget.inputs:value", LIFT_MIN),
            ],
        },
    )


def set_drive_command(speed, steering, lift_height):
    """현재 키 입력을 Action Graph의 목표값으로 전달한다."""
    og.Controller.attribute(
        GRAPH_PATH + "/WheelController.inputs:velocityCommand"
    ).set([speed, speed])
    og.Controller.attribute(
        GRAPH_PATH + "/SteeringTarget.inputs:value"
    ).set(steering)
    og.Controller.attribute(
        GRAPH_PATH + "/LiftTarget.inputs:value"
    ).set(lift_height)


def wait_for_stage_loading():
    """원격/로컬 USD 참조가 모두 로드될 때까지 GUI를 갱신한다."""
    while simulation_app.is_running():
        _, _, loading_count = omni.usd.get_context().get_stage_loading_status()
        if loading_count == 0:
            return
        simulation_app.update()
        time.sleep(0.01)


def main():
    assets_root = get_assets_root_path()
    if assets_root is None:
        raise RuntimeError(
            "Isaac Sim Assets 경로를 찾지 못했습니다. 인터넷 연결 또는 "
            "persistent.isaac.asset_root.default 설정을 확인하세요."
        )

    world = World(stage_units_in_meters=1.0, physics_dt=1.0 / 60.0, rendering_dt=1.0 / 60.0)
    world.scene.add_default_ground_plane()

    forklift_usd = assets_root + FORKLIFT_USD_RELATIVE_PATH
    print(f"[LOAD] Forklift C: {forklift_usd}")
    add_reference_to_stage(forklift_usd, FORKLIFT_PRIM_PATH)
    wait_for_stage_loading()

    create_control_graph()
    world.reset()
    world.play()

    set_camera_view(
        eye=np.array([-7.0, -7.0, 4.5]),
        target=np.array([0.5, 0.0, 1.0]),
        camera_prim_path="/OmniverseKit_Persp",
    )

    keyboard = KeyboardState()
    rclpy.init()
    ros_node = ForkliftRosNode()
    lift_height = LIFT_MIN

    print("\n" + "=" * 58)
    print(" Forklift C ROS 2 + 키보드 조종 시작")
    print(" ROS: /forklift/cmd_vel (Twist), /forklift/lift (Float32)")
    print(" 키보드: W/S 전후 | A/D 좌우 | I/K 포크 | Space 정지")
    print(f" 안전 정지: ROS 명령이 {ROS_COMMAND_TIMEOUT:.1f}초 동안 없을 때")
    print(" Space: 정지 | 종료: Isaac Sim 창 닫기")
    print(" 키보드를 누르는 동안에는 ROS 명령보다 키보드를 우선합니다.")
    print("=" * 58 + "\n")

    try:
        while simulation_app.is_running():
            rclpy.spin_once(ros_node, timeout_sec=0.0)
            ros_linear, ros_angular, ros_lift = ros_node.get_commands()

            keyboard_active = any(
                keyboard.is_down(key)
                for key in ("W", "S", "A", "D", "I", "K", "SPACE")
            )

            if keyboard_active:
                forward = float(keyboard.is_down("W")) - float(keyboard.is_down("S"))
                turn = float(keyboard.is_down("A")) - float(keyboard.is_down("D"))
                lift = float(keyboard.is_down("I")) - float(keyboard.is_down("K"))
                speed = DRIVE_SPEED * forward
                steering = STEERING_ANGLE * turn
                if keyboard.is_down("SPACE"):
                    speed = 0.0
            else:
                speed = DRIVE_SPEED * float(
                    np.clip(ros_linear / MAX_LINEAR_COMMAND, -1.0, 1.0)
                )
                steering = STEERING_ANGLE * float(
                    np.clip(ros_angular / MAX_ANGULAR_COMMAND, -1.0, 1.0)
                )
                lift = float(np.clip(ros_lift, -1.0, 1.0))

            lift_height = float(
                np.clip(lift_height + lift * LIFT_SPEED / 60.0, LIFT_MIN, LIFT_MAX)
            )

            set_drive_command(speed, steering, lift_height)
            world.step(render=True)
    finally:
        set_drive_command(0.0, 0.0, lift_height)
        keyboard.close()
        ros_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        simulation_app.close()


if __name__ == "__main__":
    main()
