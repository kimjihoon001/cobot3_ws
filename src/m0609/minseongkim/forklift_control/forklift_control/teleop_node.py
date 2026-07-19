#!/usr/bin/env python3
"""터미널 키보드로 Isaac Sim 지게차를 조종하는 ROS 2 teleop 노드.

실행 전에 ROS 2 환경을 source한 뒤 일반 python3로 실행한다.
"""

import select
import sys
import termios
import time
import tty

import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32


PUBLISH_RATE_HZ = 10.0
LINEAR_SPEED = 1.0
ANGULAR_SPEED = 0.7
KEY_HOLD_TIMEOUT = 0.25  # 키 반복 입력이 끊기면 이 시간 뒤 자동 정지

HELP = """
============================================================
 Isaac Sim Forklift 터미널 조종

        W : 전진
   A         D : 좌회전 / 우회전
        S : 후진

   X : 핸들 정렬(직진)       Space : 주행 정지
   I : 포크 상승             K : 포크 하강
   O : 포크 정지             Q : 안전 정지 후 종료

 키를 누르고 있는 동안만 움직이며, 키를 떼면 자동 정지합니다.
 Ctrl+C로 종료해도 정지 명령을 보냅니다.
============================================================
"""


def read_key(timeout):
    """터미널을 막지 않고 키 하나를 읽는다."""
    readable, _, _ = select.select([sys.stdin], [], [], timeout)
    if readable:
        return sys.stdin.read(1).lower()
    return None


def publish_commands(cmd_pub, lift_pub, linear, angular, lift):
    twist = Twist()
    twist.linear.x = float(linear)
    twist.angular.z = float(angular)
    cmd_pub.publish(twist)

    lift_msg = Float32()
    lift_msg.data = float(lift)
    lift_pub.publish(lift_msg)


def print_state(linear, angular, lift):
    drive_name = "전진" if linear > 0 else "후진" if linear < 0 else "정지"
    steer_name = "좌" if angular > 0 else "우" if angular < 0 else "직진"
    lift_name = "상승" if lift > 0 else "하강" if lift < 0 else "정지"
    status = f"주행={drive_name:<2}  조향={steer_name:<2}  포크={lift_name:<2}"
    print("\r" + status + " " * 12, end="", flush=True)


def main():
    if not sys.stdin.isatty():
        raise RuntimeError("키보드 입력이 가능한 터미널에서 실행해야 합니다.")

    original_terminal_settings = termios.tcgetattr(sys.stdin)
    rclpy.init()
    node = rclpy.create_node("forklift_teleop_keyboard")
    cmd_pub = node.create_publisher(Twist, "/forklift/cmd_vel", 10)
    lift_pub = node.create_publisher(Float32, "/forklift/lift", 10)

    linear = 0.0
    angular = 0.0
    lift = 0.0
    last_linear_key_time = None
    last_angular_key_time = None
    last_lift_key_time = None
    period = 1.0 / PUBLISH_RATE_HZ

    print(HELP)
    print_state(linear, angular, lift)

    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            loop_started = time.monotonic()
            key = read_key(period)
            now = time.monotonic()

            if key == "w":
                linear = LINEAR_SPEED
                last_linear_key_time = now
            elif key == "s":
                linear = -LINEAR_SPEED
                last_linear_key_time = now
            elif key == "a":
                angular = ANGULAR_SPEED
                last_angular_key_time = now
            elif key == "d":
                angular = -ANGULAR_SPEED
                last_angular_key_time = now
            elif key == "x":
                angular = 0.0
                last_angular_key_time = None
            elif key == " ":
                linear = 0.0
                angular = 0.0
                last_linear_key_time = None
                last_angular_key_time = None
            elif key == "i":
                lift = 1.0
                last_lift_key_time = now
            elif key == "k":
                lift = -1.0
                last_lift_key_time = now
            elif key == "o":
                lift = 0.0
                last_lift_key_time = None
            elif key == "q":
                break

            # 터미널은 KEY_RELEASE 이벤트를 제공하지 않는다. 키를 누르고
            # 있으면 OS의 key-repeat 문자가 계속 도착하므로, 반복 입력이
            # 끊긴 축을 자동으로 0으로 만들어 key-up과 같은 효과를 낸다.
            if (
                last_linear_key_time is not None
                and now - last_linear_key_time > KEY_HOLD_TIMEOUT
            ):
                linear = 0.0
                last_linear_key_time = None
            if (
                last_angular_key_time is not None
                and now - last_angular_key_time > KEY_HOLD_TIMEOUT
            ):
                angular = 0.0
                last_angular_key_time = None
            if (
                last_lift_key_time is not None
                and now - last_lift_key_time > KEY_HOLD_TIMEOUT
            ):
                lift = 0.0
                last_lift_key_time = None

            publish_commands(cmd_pub, lift_pub, linear, angular, lift)
            print_state(linear, angular, lift)
            rclpy.spin_once(node, timeout_sec=0.0)

            elapsed = time.monotonic() - loop_started
            if elapsed < period:
                time.sleep(period - elapsed)
    except KeyboardInterrupt:
        pass
    finally:
        # 종료 패킷 유실 가능성을 줄이기 위해 정지 명령을 여러 번 보낸다.
        for _ in range(3):
            publish_commands(cmd_pub, lift_pub, 0.0, 0.0, 0.0)
            rclpy.spin_once(node, timeout_sec=0.0)
            time.sleep(0.03)

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, original_terminal_settings)
        print("\n지게차 정지 명령 전송 후 종료했습니다.")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
