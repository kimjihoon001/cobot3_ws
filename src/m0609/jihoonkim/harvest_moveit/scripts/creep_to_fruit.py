"""이랑 앞으로 저속 정렬 — /sim/tomato 가 뜨는 순간 멈춘다.

drive_to_row 는 키네마틱 베이스+RTF>1 로 크게 오버슈트한다(0.35 m/s). 여기선 0.10 m/s 로
기어가며 매 스텝 작업영역(mm._publish_sim_tomato)에 ripe 과실이 들어왔는지 보고,
들어오면 즉시 정지. 목표 odom (TX,TY) = 월드 이랑 앞(0.75,-9.0). 이미 지나쳤으면 되돌아온다.
"""
import json
import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String

TX, TY = 0.75, 3.0          # odom 목표(= 월드 0.75,-9.0), drive_to_row 와 동일
V, W = 0.10, 0.5            # 저속 (오버슈트 방지)

rclpy.init()
n = Node("creep2fruit")
pub = n.create_publisher(Twist, "/cmd_vel", 10)
od, fruit = {}, {"pos": None}


def odo(m):
    p, q = m.pose.pose.position, m.pose.pose.orientation
    od["x"], od["y"] = p.x, p.y
    od["yaw"] = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))


def on_fruit(m):
    if m.data:
        try:
            fruit["pos"] = json.loads(m.data)["position"]
        except (ValueError, KeyError):
            pass


n.create_subscription(Odometry, "/odom", odo, 10)
n.create_subscription(String, "/harvester_moveit/sim/tomato", on_fruit, 20)


def norm(a):
    return (a + math.pi) % (2*math.pi) - math.pi


def spin(dt):
    end = time.time() + dt
    while time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.02)


t0 = time.time()
while "x" not in od and time.time() - t0 < 8:
    rclpy.spin_once(n, timeout_sec=0.1)
if "x" not in od:
    print("odom 없음 — --nav 브리지 확인"); raise SystemExit(1)
print(f"시작: odom=({od['x']:.2f},{od['y']:.2f}) yaw={math.degrees(od['yaw']):.0f}°")

# 1) 목표 방향으로 정렬(제자리 회전)
heading = math.atan2(TY - od["y"], TX - od["x"])
t0 = time.time()
while time.time() - t0 < 20:
    rclpy.spin_once(n, timeout_sec=0.02)
    e = norm(heading - od["yaw"])
    if abs(e) < 0.03:
        break
    t = Twist(); t.angular.z = max(-W, min(W, 2.0*e)); pub.publish(t)
pub.publish(Twist()); spin(0.3)

# 2) 저속 전진 — 과실 뜨면 즉시 정지
fruit["pos"] = None
t0 = time.time()
stopped_by = "시간초과"
while time.time() - t0 < 60:
    rclpy.spin_once(n, timeout_sec=0.02)
    if fruit["pos"] is not None:
        stopped_by = "과실검출"; break
    d = math.hypot(TX - od["x"], TY - od["y"])
    if d < 0.06:
        stopped_by = "목표도달"; break
    t = Twist(); t.linear.x = max(0.04, min(V, 0.5*d)); pub.publish(t)
pub.publish(Twist()); spin(0.5)

print(f"정지({stopped_by}): odom=({od['x']:.2f},{od['y']:.2f})")
spin(2.0)
print(f"작업영역 과실(섀시): {fruit['pos']}")
n.destroy_node(); rclpy.shutdown()
