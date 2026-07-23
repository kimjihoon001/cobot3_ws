#!/usr/bin/env python3
"""수확 텔레옵 — Nav2 없이 키보드로 베이스·팔 조종 + 'h' 로 그 자리에서 수확 트리거.

  베이스 주행 :  /cmd_vel (Twist, 홀로노믹 vx/vy/wz) → Isaac MM 베이스
  팔 조종     :  /harvester_0/cmd (rmp_target) → rmpflow 가 팔 끝(TCP)을 그 좌표로
  수확 게이트 :  /harvest_test/enable (Bool) → manipulator 의 external_harvest_gate

--rmpflow 중이라 조인트 직접제어는 브리지가 무시한다. 그래서 팔은 TCP 목표(rmp_target)를
base_link 기준으로 조금씩 옮겨 조종한다(그리퍼·카메라가 따라온다). g 로 홈 복귀.
manipulator 를 command_enabled=true + external_harvest_gate_enabled=true 로 띄우면
_harvest_enabled 가 False 로 시작 — 'h' 로 True 를 쏘면 그 자리에서 rmpflow 수확 시작.
"""
import json
import select
import sys
import termios
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool, Float64, String

HELP = """
┌─ 수확 텔레옵 ─ (이 터미널에 포커스를 두고 키 입력) ────────────────────┐
  베이스:  w/s 전후   a/d 회전   q/e 횡이동   space 정지
  팔(TCP): i/k 앞뒤(x)  j/l 좌우(y)  u/o 위아래(z)   g 홈 복귀
  수확:    h  수확 시작(그 자리)      n  수확 중단 + 그리퍼 열기   t  잎 표시 토글
  커터:    m  서보 절삭(35°)          b  서보 재개방(0°)
  속도:    z/x 선속 -/+   c/v 각속 -/+   ,/. 팔 스텝 -/+
  종료:    Ctrl-C
└───────────────────────────────────────────────────────────────────────┘
  ※ 팔 첫 조작은 base_link 기준 기본 도달점으로 이동한다. g 로 언제든 홈.
"""


class HarvestTeleop(Node):
    def __init__(self):
        super().__init__("harvest_teleop")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("enable_topic", "/harvest_test/enable")
        self.declare_parameter("isaac_command_topic", "/harvester_0/cmd")
        self.declare_parameter("blade_command_topic", "/harvester_0/blade_command")
        self.declare_parameter("base_frame", "base_link")
        self._cmd = self.create_publisher(
            Twist, str(self.get_parameter("cmd_vel_topic").value), 10)
        self._enable = self.create_publisher(
            Bool, str(self.get_parameter("enable_topic").value), 10)
        self._isaac = self.create_publisher(
            String, str(self.get_parameter("isaac_command_topic").value), 10)
        self._blade = self.create_publisher(
            Float64, str(self.get_parameter("blade_command_topic").value), 10)
        self._frame = str(self.get_parameter("base_frame").value)
        self.lin = 0.25    # m/s
        self.ang = 0.5     # rad/s
        self.step = 0.05   # m, 팔 TCP 스텝
        self.foliage_on = True   # 잎 표시 상태 (f 로 토글)
        # 팔 TCP 목표 (base_link). 첫 조작 전엔 안 보냄 — 첫 키에서 이 기본점부터 시작.
        self.arm = [0.55, 0.0, 1.00]
        self._arm_id = 0

    def twist(self, vx: float, vy: float, wz: float) -> None:
        t = Twist()
        t.linear.x, t.linear.y, t.angular.z = vx, vy, wz
        self._cmd.publish(t)

    def enable(self, on: bool) -> None:
        self._enable.publish(Bool(data=on))

    def open_gripper(self) -> None:
        self._isaac.publish(String(data=json.dumps({"gripper": {"closed": False}})))

    def blade(self, angle_deg: float) -> None:
        self._blade.publish(Float64(data=float(angle_deg)))
        print(f"[커터] 서보 목표 {angle_deg:.0f}°")

    def nudge_arm(self, dx: float, dy: float, dz: float) -> None:
        self.arm[0] += dx
        self.arm[1] += dy
        self.arm[2] += dz
        self._arm_id += 1
        self._isaac.publish(String(data=json.dumps({
            "rmp_target": {
                "id": self._arm_id,
                "phase": "TELEOP",
                "frame_id": self._frame,
                "position": [round(v, 4) for v in self.arm],
            }
        })))
        print(f"[팔] TCP → ({self.arm[0]:.2f}, {self.arm[1]:.2f}, {self.arm[2]:.2f})")

    def home_arm(self) -> None:
        self._arm_id += 1
        self._isaac.publish(String(data=json.dumps({"rmp_home": {"id": self._arm_id}})))
        print("[팔] 홈 복귀 (rmp_home)")


def _key(timeout: float = 0.1) -> str:
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        r, _, _ = select.select([sys.stdin], [], [], timeout)
        return sys.stdin.read(1) if r else ""
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def main() -> None:
    rclpy.init()
    node = HarvestTeleop()
    print(HELP)
    print(f"[속도] 선속 {node.lin:.2f} m/s · 각속 {node.ang:.2f} rad/s · 팔스텝 {node.step:.2f} m\n")
    try:
        while rclpy.ok():
            k = _key()
            vx = vy = wz = 0.0
            # 베이스
            if k == "w":
                vx = node.lin
            elif k == "s":
                vx = -node.lin
            elif k == "a":
                wz = node.ang
            elif k == "d":
                wz = -node.ang
            elif k == "q":
                vy = node.lin
            elif k == "e":
                vy = -node.lin
            # 팔 (TCP nudge)
            elif k == "i":
                node.nudge_arm(node.step, 0, 0)
            elif k == "k":
                node.nudge_arm(-node.step, 0, 0)
            elif k == "j":
                node.nudge_arm(0, node.step, 0)
            elif k == "l":
                node.nudge_arm(0, -node.step, 0)
            elif k == "u":
                node.nudge_arm(0, 0, node.step)
            elif k == "o":
                node.nudge_arm(0, 0, -node.step)
            elif k == "g":
                node.home_arm()
            # 수확 게이트
            elif k == "h":
                node.enable(True)
                print("[수확] 시작 — enable=True (그 자리에서 rmpflow)")
            elif k == "n":
                node.enable(False)
                node.open_gripper()
                print("[수확] 중단 — enable=False, 그리퍼 열림")
            elif k == "t":
                node.foliage_on = not node.foliage_on
                node._isaac.publish(String(data=json.dumps({"foliage": node.foliage_on})))
                print(f"[잎] {'표시' if node.foliage_on else '숨김'}")
            elif k == "m":
                node.blade(35.0)
            elif k == "b":
                node.blade(0.0)
            # 속도/스텝 조절
            elif k in ("z", "x"):
                node.lin = max(0.05, min(1.0, node.lin + (0.05 if k == "x" else -0.05)))
                print(f"[속도] 선속 {node.lin:.2f} m/s")
            elif k in ("c", "v"):
                node.ang = max(0.1, min(2.0, node.ang + (0.1 if k == "v" else -0.1)))
                print(f"[속도] 각속 {node.ang:.2f} rad/s")
            elif k in (",", "."):
                node.step = max(0.01, min(0.20, node.step + (0.01 if k == "." else -0.01)))
                print(f"[팔] 스텝 {node.step:.2f} m")
            elif k == "\x03":   # Ctrl-C
                break
            node.twist(vx, vy, wz)
            rclpy.spin_once(node, timeout_sec=0.0)
    except KeyboardInterrupt:
        pass
    finally:
        node.twist(0.0, 0.0, 0.0)   # 정지
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
