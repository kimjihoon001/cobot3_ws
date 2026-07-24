"""그리퍼 카메라 스냅샷 — /harvester/rgb 를 PNG 로 저장 (파지 디버깅 눈)."""
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


def snap(node, out_path, timeout=6.0):
    got = []
    sub = node.create_subscription(Image, "/harvester/rgb", lambda m: got.append(m), 5)
    end = time.time() + timeout
    while time.time() < end and not got:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_subscription(sub)
    if not got:
        print(f"  [snap] 프레임 없음: {out_path}")
        return False
    m = got[-1]
    a = np.frombuffer(bytes(m.data), dtype=np.uint8)
    ch = len(m.data) // (m.width * m.height)
    a = a.reshape(m.height, m.width, ch)
    import cv2
    bgr = cv2.cvtColor(a[:, :, :3], cv2.COLOR_RGB2BGR)
    cv2.imwrite(out_path, bgr)
    print(f"  [snap] {out_path} ({m.width}x{m.height} {m.encoding})")
    return True


if __name__ == "__main__":
    rclpy.init()
    n = Node("snapper")
    snap(n, sys.argv[1] if len(sys.argv) > 1 else "/tmp/snap.png")
