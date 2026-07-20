#!/usr/bin/env python3
"""터미널 키보드로 ``main.py``의 지게차를 조종하는 ROS 2 노드.

``isaac_python forklift_teleop.py``로 실행하면 Isaac Sim 동봉 ROS 2 환경을 자동 적용한다.
명령 토픽: /forklift_0/joint_command (sensor_msgs/msg/JointState)
상태 토픽: /forklift_0/joint_states  (sensor_msgs/msg/JointState)
"""

import math
import os
import select
import sys
import termios
import time
import tty
from pathlib import Path


def _bootstrap_isaac_ros2():
    """Isaac Python에서 동봉된 Humble 모듈과 라이브러리를 사용해 재실행한다."""
    # resolve()하면 Isaac의 kit/python 심볼릭 링크가 Packman 캐시로 바뀌어
    # release/exts 경로를 찾을 수 없으므로 링크 경로 자체를 사용한다.
    executable = Path(sys.executable).absolute()
    for parent in executable.parents:
        humble = parent / "exts" / "isaacsim.ros2.bridge" / "humble"
        if (humble / "rclpy" / "rclpy").is_dir() and (humble / "lib").is_dir():
            break
    else:
        return

    marker = str(humble)
    if os.environ.get("FORKLIFT_TELEOP_ROS_ROOT") == marker:
        return

    env = os.environ.copy()
    env["FORKLIFT_TELEOP_ROS_ROOT"] = marker
    env["PYTHONPATH"] = os.pathsep.join(
        part for part in (str(humble / "rclpy"), env.get("PYTHONPATH")) if part
    )
    env["LD_LIBRARY_PATH"] = os.pathsep.join(
        part for part in (str(humble / "lib"), env.get("LD_LIBRARY_PATH")) if part
    )
    env.setdefault("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")
    env.setdefault("ROS_DOMAIN_ID", "108")
    os.execve(
        str(executable),
        [str(executable), str(Path(__file__).resolve()), *sys.argv[1:]],
        env,
    )


_bootstrap_isaac_ros2()

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


PUBLISH_RATE_HZ = 30.0
DRIVE_SPEED = 6.0             # back_wheel_drive 목표 속도 (rad/s)
STEERING_ANGLE = math.radians(25.0)
LIFT_MIN = 0.0
LIFT_MAX = 2.0
LIFT_SPEED = 0.7             # 키를 누르는 동안 목표 높이 변화량 (m/s)
KEY_HOLD_TIMEOUT = 0.55
CONNECTION_TIMEOUT = 1.0

HELP = """
============================================================
 main.py ForkliftB 터미널 조종

        W : 전진
   A         D : 좌조향 / 우조향 (각도 유지)
        S : 후진

   X : 핸들 정렬(직진)       Space : 주행만 정지
   I : 포크 상승             K : 포크 하강
   O : 포크 정지             Q : 안전 정지 후 종료

 W/S와 I/K는 키를 떼면 자동 정지합니다. A/D 조향은 X를 누를 때까지 유지됩니다.
============================================================
"""


class ForkliftTeleop(Node):
    def __init__(self):
        super().__init__("forklift_0_teleop_keyboard")
        self.command_pub = self.create_publisher(
            JointState, "/forklift_0/joint_command", 10
        )
        self.create_subscription(
            JointState, "/forklift_0/joint_states", self._on_joint_states, 10
        )
        self.lift_target = 0.0
        self._lift_initialized = False
        self.kind = None
        self.drive_joints = ["back_wheel_drive"]
        self.steering_joints = ["back_wheel_swivel"]
        self.last_state_time = None
        self._connection_alive = False

    def _on_joint_states(self, msg):
        self.last_state_time = time.monotonic()
        if not self._connection_alive:
            self._connection_alive = True
            if self.kind is not None:
                print(f"\n[재연결] main.py의 {self.kind} 상태 토픽을 다시 수신했습니다.")

        names = set(msg.name)
        if self.kind is None:
            if {"back_wheel_drive", "back_wheel_swivel"} <= names:
                self.kind = "ForkliftB"
            if self.kind is not None:
                print(f"\n[연결] main.py의 {self.kind} 관절을 확인했습니다. "
                      "(ROS_DOMAIN_ID=108)")

        if not self._lift_initialized:
            try:
                index = msg.name.index("lift_joint")
                self.lift_target = min(
                    max(float(msg.position[index]), LIFT_MIN), LIFT_MAX
                )
                self._lift_initialized = True
            except (ValueError, IndexError):
                pass

    def publish_command(self, drive, steering):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ["lift_joint", "back_wheel_swivel", "back_wheel_drive"]
        msg.position = [self.lift_target, steering, math.nan]
        msg.velocity = [math.nan, math.nan, drive]
        self.command_pub.publish(msg)

    def is_connected(self):
        return (
            self.kind is not None
            and self.last_state_time is not None
            and time.monotonic() - self.last_state_time <= CONNECTION_TIMEOUT
        )

    def check_connection(self):
        connected = self.is_connected()
        if self._connection_alive and not connected:
            self._connection_alive = False
            print("\n[연결 끊김] /forklift_0/joint_states가 1초 이상 수신되지 않았습니다.")
            print("  main.py 실행 여부, 타임라인 Play, ROS_DOMAIN_ID=108을 확인하세요.")
        return connected


def read_key(timeout):
    readable, _, _ = select.select([sys.stdin], [], [], timeout)
    return sys.stdin.read(1).lower() if readable else None


def print_state(drive, steering, lift_direction, lift_target):
    global _LAST_PRINTED_STATE
    state = (drive, steering, lift_direction)
    if state == _LAST_PRINTED_STATE:
        return
    _LAST_PRINTED_STATE = state

    drive_name = "전진" if drive > 0 else "후진" if drive < 0 else "정지"
    steer_name = "좌" if steering > 0 else "우" if steering < 0 else "직진"
    lift_name = "상승" if lift_direction > 0 else "하강" if lift_direction < 0 else "정지"
    status = (
        f"주행={drive_name:<2}  조향={steer_name:<2}  "
        f"포크={lift_name:<2}({lift_target:.2f}m)"
    )
    print(status, flush=True)


_LAST_PRINTED_STATE = None


def main():
    if not sys.stdin.isatty():
        raise RuntimeError("키보드 입력이 가능한 터미널에서 실행해야 합니다.")

    terminal_settings = termios.tcgetattr(sys.stdin)
    rclpy.init()
    node = ForkliftTeleop()

    drive = 0.0
    steering = 0.0
    lift_direction = 0.0
    last_drive_key = None
    last_lift_key = None
    period = 1.0 / PUBLISH_RATE_HZ

    print(HELP)
    print("[연결 대기] main.py의 /forklift_0/joint_states를 기다립니다.")
    print("  main.py 실행, RosBridge 대기 중, 타임라인 Play, domain 108이 필요합니다.")
    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            started = time.monotonic()
            key = read_key(period)
            now = time.monotonic()

            if key == "w":
                drive, last_drive_key = DRIVE_SPEED, now
            elif key == "s":
                drive, last_drive_key = -DRIVE_SPEED, now
            elif key == "a":
                steering = STEERING_ANGLE
            elif key == "d":
                steering = -STEERING_ANGLE
            elif key == "x":
                steering = 0.0
            elif key == " ":
                drive = 0.0
                last_drive_key = None
            elif key == "i":
                lift_direction, last_lift_key = 1.0, now
            elif key == "k":
                lift_direction, last_lift_key = -1.0, now
            elif key == "o":
                lift_direction, last_lift_key = 0.0, None
            elif key == "q":
                break

            if last_drive_key is not None and now - last_drive_key > KEY_HOLD_TIMEOUT:
                drive, last_drive_key = 0.0, None
            if last_lift_key is not None and now - last_lift_key > KEY_HOLD_TIMEOUT:
                lift_direction, last_lift_key = 0.0, None

            node.lift_target = min(
                max(node.lift_target + lift_direction * LIFT_SPEED * period, LIFT_MIN),
                LIFT_MAX,
            )
            node.publish_command(drive, steering)
            rclpy.spin_once(node, timeout_sec=0.0)
            node.check_connection()
            print_state(drive, steering, lift_direction, node.lift_target)

            elapsed = time.monotonic() - started
            if elapsed < period:
                time.sleep(period - elapsed)
    except KeyboardInterrupt:
        pass
    finally:
        for _ in range(3):
            node.publish_command(0.0, 0.0)
            rclpy.spin_once(node, timeout_sec=0.0)
            time.sleep(0.03)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, terminal_settings)
        print("\n지게차 정지 명령 전송 후 종료했습니다.")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
