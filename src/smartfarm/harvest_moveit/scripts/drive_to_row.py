"""스폰(0,-12)에서 이랑 앞(월드 0.75,-9.0, +X 정면)으로 — 회전+전진(게걸음 없음)."""
import json
import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String

rclpy.init()
n = Node("drive2row")
pub = n.create_publisher(Twist, "/cmd_vel", 10)
od, fruits = {}, []


def odo(m):
    p = m.pose.pose.position
    q = m.pose.pose.orientation
    od["x"], od["y"] = p.x, p.y
    od["yaw"] = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))


n.create_subscription(Odometry, "/odom", odo, 10)
n.create_subscription(String, "/harvester_moveit/sim/tomato",
                      lambda m: fruits.append(m.data) if m.data else None, 20)


def spin(dt):
    end = time.time() + dt
    while time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.05)


def norm(a):
    while a > math.pi:
        a -= 2*math.pi
    while a < -math.pi:
        a += 2*math.pi
    return a


def go(fn, cond, tmax):
    t0 = time.time()
    while time.time() - t0 < tmax:
        rclpy.spin_once(n, timeout_sec=0.02)
        if "x" not in od:
            continue
        if cond():
            break
        pub.publish(fn())
    pub.publish(Twist())
    spin(0.5)


def face(target):
    def fn():
        t = Twist()
        t.angular.z = max(-0.6, min(0.6, 2.0*norm(target - od["yaw"])))
        return t
    go(fn, lambda: abs(norm(target - od["yaw"])) < 0.02, 25)


def forward(tx, ty):
    def fn():
        d = math.hypot(tx - od["x"], ty - od["y"])
        t = Twist()
        t.linear.x = max(0.05, min(0.35, 0.8*d))
        return t
    go(fn, lambda: math.hypot(tx - od["x"], ty - od["y"]) < 0.06, 40)


spin(1.5)
face(math.pi/2)
forward(0.75, 3.0)
face(0.0)
print(f"도착: odom=({od['x']:.2f},{od['y']:.2f}) yaw={math.degrees(od['yaw']):.1f}°")
fruits.clear()
spin(5.0)
uniq = set()
for f in fruits:
    try:
        uniq.add(tuple(round(v, 3) for v in json.loads(f)["position"]))
    except ValueError:
        pass
print(f"작업영역 내 ripe {len(uniq)}개:")
for k in sorted(uniq)[:6]:
    print("  ", k)
