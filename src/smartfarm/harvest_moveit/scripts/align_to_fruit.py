"""검출된 과실 정면 STANDOFF(m) 지점으로 베이스를 정렬(비홀로노믹: 회전→전진→정면).

creep 이 멈춘 자리는 과실이 작업영역 '끝 모서리'(팔 도달 밖)라 IK 가 실패한다. 여기선
현재 검출 과실의 odom 좌표를 역산해, 과실 -X 로 STANDOFF 떨어진 지점에 +X 를 보고 서서
과실이 섀시 (STANDOFF, ~0, z) 에 오게 한다 → UR10e 도달권 안.
"""
import json
import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String

STANDOFF = 0.60             # 과실이 섀시 +X 로 이만큼 앞에 오게 (도달권)
V, W = 0.10, 0.5

rclpy.init()
n = Node("align2fruit")
pub = n.create_publisher(Twist, "/cmd_vel", 10)
od, fr = {}, {"pos": None}


def odo(m):
    p, q = m.pose.pose.position, m.pose.pose.orientation
    od["x"], od["y"] = p.x, p.y
    od["yaw"] = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))


def onf(m):
    if m.data:
        try:
            fr["pos"] = json.loads(m.data)["position"]
        except (ValueError, KeyError):
            pass


n.create_subscription(Odometry, "/odom", odo, 10)
n.create_subscription(String, "/harvester_moveit/sim/tomato", onf, 20)


def norm(a):
    return (a + math.pi) % (2*math.pi) - math.pi


def spin(dt):
    e = time.time() + dt
    while time.time() < e:
        rclpy.spin_once(n, timeout_sec=0.02)


def face(target_yaw, tmax=20):
    t0 = time.time()
    while time.time() - t0 < tmax:
        rclpy.spin_once(n, timeout_sec=0.02)
        e = norm(target_yaw - od["yaw"])
        if abs(e) < 0.02:
            break
        t = Twist(); t.angular.z = max(-W, min(W, 2.0*e)); pub.publish(t)
    pub.publish(Twist()); spin(0.3)


def goto(tx, ty, tmax=40):
    t0 = time.time()
    while time.time() - t0 < tmax:
        rclpy.spin_once(n, timeout_sec=0.02)
        d = math.hypot(tx - od["x"], ty - od["y"])
        if d < 0.05:
            break
        t = Twist(); t.linear.x = max(0.04, min(V, 0.6*d)); pub.publish(t)
    pub.publish(Twist()); spin(0.3)


# 0) odom + 현재 과실 대기
t0 = time.time()
while ("x" not in od or fr["pos"] is None) and time.time() - t0 < 10:
    rclpy.spin_once(n, timeout_sec=0.1)
if "x" not in od or fr["pos"] is None:
    print("odom/과실 없음 — creep 으로 먼저 과실 잡을 것"); raise SystemExit(1)

# 1) 과실 odom 좌표 역산 (섀시→odom)
cx, cy, cz = fr["pos"]
c, s = math.cos(od["yaw"]), math.sin(od["yaw"])
fox = od["x"] + c*cx - s*cy
foy = od["y"] + s*cx + c*cy
print(f"과실 섀시=({cx:.2f},{cy:.2f},{cz:.2f}) → odom=({fox:.2f},{foy:.2f})")

# 2) 목표: 과실 -X 로 STANDOFF, +X 정면
tx, ty = fox - STANDOFF, foy
print(f"목표 베이스 odom=({tx:.2f},{ty:.2f}) yaw=0")
face(math.atan2(ty - od["y"], tx - od["x"]))   # 목표점 향해 회전
goto(tx, ty)                                   # 전진
face(0.0)                                       # +X 정면 재정렬

# 3) 재검출
fr["pos"] = None
spin(3.0)
print(f"정렬 후: odom=({od['x']:.2f},{od['y']:.2f}) yaw={math.degrees(od['yaw']):.0f}°")
print(f"과실 섀시(재검출): {fr['pos']}")
n.destroy_node(); rclpy.shutdown()
