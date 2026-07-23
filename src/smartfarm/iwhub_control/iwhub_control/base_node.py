# -*- coding: utf-8 -*-
"""iw.hub 베이스 노드 — /cmd_vel 을 바퀴 속도로 바꿔 Isaac 에 보내고, Isaac 이 돌려주는
바퀴 각도로 오도메트리(+TF)를 발행한다. 수학은 kinematics.py(순수), 여기는 얇은 ROS2 래퍼(§5.6).

토픽:
  구독 /cmd_vel                     geometry_msgs/Twist       ← Nav2/텔레옵
  발행 /{ns}/joint_command          sensor_msgs/JointState    → Isaac (좌/우 바퀴 속도)
  구독 /{ns}/joint_states           sensor_msgs/JointState    ← Isaac (바퀴 각도)
  발행 /odom                        nav_msgs/Odometry
  발행 TF                            odom → base_link

파라미터(기본값 = isaacpjt IwHubNavConfig; wheel_* 는 [4] iw.hub 실측 필요 — odom 정확도 직결):
  wheel_radius=0.1  wheel_separation=0.5  ns=iwhub_0
  odom_frame=odom   base_frame=base_link
  publish_odom=true — false이면 바퀴 odom을 끄고 Isaac chassis odom을 사용.
  left_wheel_joint=left_wheel_joint  right_wheel_joint=right_wheel_joint
  cmd_timeout=0.5 (s) — cmd_vel 이 끊기면 바퀴를 0 으로 세운다(안전).

Isaac 은 --iw --nav-scan 으로 joint 브리지(자동)+라이다만 띄운다. 차동구동·odom 은 여기서 한다
(Isaac --nav-drive/--nav-odom 과 겹치므로 그건 iw 에 쓰지 않는다).
"""
import math

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster

from iwhub_control.kinematics import (
    DiffDriveOdometry,
    stabilize_twist,
    twist_to_wheel_speeds,
)


def _yaw_to_quat(yaw: float):
    """yaw[rad] → (x, y, z, w) 쿼터니언 (Z축 회전)."""
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class BaseNode(Node):
    def __init__(self):
        super().__init__("iwhub_base_node")

        self.declare_parameter("wheel_radius", 0.1)
        self.declare_parameter("wheel_separation", 0.5)
        self.declare_parameter("ns", "iwhub_0")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("left_wheel_joint", "left_wheel_joint")
        self.declare_parameter("right_wheel_joint", "right_wheel_joint")
        self.declare_parameter("cmd_timeout", 0.5)
        self.declare_parameter("publish_odom", True)
        self.declare_parameter("cmd_vel_topic", "/iwhub_0/cmd_vel")
        self.declare_parameter("odom_topic", "/iwhub_0/odom")
        # Nav2가 목표점 근처에서 내는 극소 정/역회전 명령을 0으로 고정한다.
        # start > stop 히스테리시스로 경계값 부근의 on/off 채터링도 막는다.
        self.declare_parameter("linear_stop_deadband", 0.01)
        self.declare_parameter("angular_stop_deadband", 0.02)
        self.declare_parameter("linear_start_deadband", 0.02)
        self.declare_parameter("angular_start_deadband", 0.04)

        self._r = self.get_parameter("wheel_radius").value
        self._sep = self.get_parameter("wheel_separation").value
        ns = self.get_parameter("ns").value
        self._odom_frame = self.get_parameter("odom_frame").value
        self._base_frame = self.get_parameter("base_frame").value
        self._lname = self.get_parameter("left_wheel_joint").value
        self._rname = self.get_parameter("right_wheel_joint").value
        self._cmd_timeout = self.get_parameter("cmd_timeout").value
        self._publish_odom = self.get_parameter("publish_odom").value
        self._cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        self._odom_topic = self.get_parameter("odom_topic").value
        self._linear_stop_deadband = float(
            self.get_parameter("linear_stop_deadband").value
        )
        self._angular_stop_deadband = float(
            self.get_parameter("angular_stop_deadband").value
        )
        self._linear_start_deadband = float(
            self.get_parameter("linear_start_deadband").value
        )
        self._angular_start_deadband = float(
            self.get_parameter("angular_start_deadband").value
        )
        # 잘못된 launch override는 첫 cmd_vel이 올 때까지 숨기지 않고 시작 시 실패시킨다.
        stabilize_twist(
            0.0,
            0.0,
            True,
            linear_stop=self._linear_stop_deadband,
            angular_stop=self._angular_stop_deadband,
            linear_start=self._linear_start_deadband,
            angular_start=self._angular_start_deadband,
        )
        # 시작 시 바퀴가 정지해 있으므로 start 문턱을 넘는 명령만 운동을 시작한다.
        self._motion_stopped = True

        self._odom = DiffDriveOdometry(self._r, self._sep)
        self._tf = TransformBroadcaster(self)
        self._last_cmd_t = None

        self._cmd_pub = self.create_publisher(JointState, f"/{ns}/joint_command", 10)
        self.create_subscription(Twist, self._cmd_vel_topic, self._on_cmd, 10)
        if self._publish_odom:
            self._odom_pub = self.create_publisher(Odometry, self._odom_topic, 10)
            self.create_subscription(
                JointState, f"/{ns}/joint_states", self._on_joints, 10)
        else:
            self._odom_pub = None
        # cmd_vel 끊김 감시 — 끊기면 바퀴 0 (Nav2 가 멈출 때 0 을 보내지만 안전용)
        self.create_timer(0.1, self._watchdog)

        odom_mode = (
            f"/{ns}/joint_states→/odom+TF "
            f"({self._odom_frame}→{self._base_frame})"
            if self._publish_odom
            else "Isaac chassis odom/TF 사용(바퀴 odom 비활성)"
        )
        self.get_logger().info(
            f"iwhub_base_node: {self._cmd_vel_topic}→/"
            f"{ns}/joint_command, {odom_mode}, "
            f"r={self._r} sep={self._sep}, "
            f"deadband linear={self._linear_stop_deadband:.3f}/"
            f"{self._linear_start_deadband:.3f}m/s angular="
            f"{self._angular_stop_deadband:.3f}/"
            f"{self._angular_start_deadband:.3f}rad/s"
        )

    # ── /cmd_vel → 좌/우 바퀴 속도 → Isaac joint_command ──
    def _on_cmd(self, msg: Twist) -> None:
        linear, angular, self._motion_stopped = stabilize_twist(
            msg.linear.x,
            msg.angular.z,
            self._motion_stopped,
            linear_stop=self._linear_stop_deadband,
            angular_stop=self._angular_stop_deadband,
            linear_start=self._linear_start_deadband,
            angular_start=self._angular_start_deadband,
        )
        left, right = twist_to_wheel_speeds(
            linear, angular, self._r, self._sep)
        self._publish_wheels(left, right)
        self._last_cmd_t = self.get_clock().now()

    def _publish_wheels(self, left: float, right: float) -> None:
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = [self._lname, self._rname]
        js.velocity = [float(left), float(right)]
        self._cmd_pub.publish(js)

    def _watchdog(self) -> None:
        if self._last_cmd_t is None:
            return
        dt = (self.get_clock().now() - self._last_cmd_t).nanoseconds * 1e-9
        if dt > self._cmd_timeout:
            self._publish_wheels(0.0, 0.0)      # 명령 끊김 → 정지
            self._last_cmd_t = None
            self._motion_stopped = True

    def stop_wheels(self) -> None:
        """종료 직전 마지막 drive target을 0으로 덮어 Isaac에 잔류하지 않게 한다."""
        self._publish_wheels(0.0, 0.0)
        self._last_cmd_t = None
        self._motion_stopped = True

    # ── Isaac joint_states(바퀴 각도) → 오도메트리 → /odom + TF ──
    def _on_joints(self, msg: JointState) -> None:
        if not self._publish_odom or self._odom_pub is None:
            return
        try:
            li = msg.name.index(self._lname)
            ri = msg.name.index(self._rname)
        except ValueError:
            return
        if li >= len(msg.position) or ri >= len(msg.position):
            return
        x, y, yaw = self._odom.update(msg.position[li], msg.position[ri])
        stamp = self.get_clock().now().to_msg()
        qx, qy, qz, qw = _yaw_to_quat(yaw)

        od = Odometry()
        od.header.stamp = stamp
        od.header.frame_id = self._odom_frame
        od.child_frame_id = self._base_frame
        od.pose.pose.position.x = x
        od.pose.pose.position.y = y
        od.pose.pose.orientation.x = qx
        od.pose.pose.orientation.y = qy
        od.pose.pose.orientation.z = qz
        od.pose.pose.orientation.w = qw
        self._odom_pub.publish(od)

        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = self._odom_frame
        t.child_frame_id = self._base_frame
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self._tf.sendTransform(t)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BaseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.stop_wheels()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
