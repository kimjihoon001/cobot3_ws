#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""색상 감지 ROS2 노드 — PC B(일반 노트북)용.

/rgb(sensor_msgs/Image, Isaac Sim Wrist Camera 발행)를 구독해
HSV 기준으로 파랑/초록 큐브를 판별하고 /color_id(std_msgs/Int32)로 발행한다.
  파랑 큐브 검출 → 1
  초록 큐브 검출 → 2

cv_bridge 없이 동작하도록 sensor_msgs/Image → numpy 변환을 직접 수행한다
(일반 노트북에 ROS2 desktop 전체가 안 깔려 있어도 rclpy + opencv-python만 있으면 됨).

실행 (PC B):
  source /opt/ros/humble/setup.bash
  python3 color_detector.py
확인:
  ros2 topic echo /color_id
"""
import os
os.environ["ROS_DOMAIN_ID"] = "109"   # PC A(Isaac Sim)와 동일해야 통신됨

import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Int32

# ── HSV 색 범위 (OpenCV: H 0~179, S/V 0~255) ─────────────────────
HSV_RANGES = {
    "blue":  {"id": 1, "lower": (100, 80, 50), "upper": (130, 255, 255)},
    "green": {"id": 2, "lower": (40, 80, 50),  "upper": (80, 255, 255)},
}
MIN_BLOB_RATIO = 0.01   # ROI 픽셀 대비 이 비율 미만이면 미검출로 간주
CONFIRM_FRAMES = 5      # 노이즈로 인한 플리커 방지: 같은 색이 연속 N프레임 나와야 발행
ROI_FRAC = 0.4          # 중앙 40% x 40% 만 판정 → 가장자리 비콘 배제
FRAME_GAP_NS = 500_000_000  # 0.5s 이상 프레임이 끊기면 새 관찰로 보고 재무장


def imgmsg_to_rgb(msg):
    """sensor_msgs/Image -> (H, W, 3) RGB numpy 배열 (cv_bridge 미사용)."""
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    if msg.encoding in ("rgb8", "bgr8"):
        img = buf.reshape(msg.height, msg.width, 3)
        if msg.encoding == "bgr8":
            img = img[:, :, ::-1]
    elif msg.encoding in ("rgba8", "bgra8"):
        img = buf.reshape(msg.height, msg.width, 4)
        if msg.encoding == "bgra8":
            img = img[:, :, [2, 1, 0]]
        else:
            img = img[:, :, :3]
    else:
        raise ValueError(f"지원하지 않는 encoding: {msg.encoding}")
    return np.ascontiguousarray(img)


class ColorDetector(Node):
    def __init__(self):
        super().__init__("color_detector")
        self.sub = self.create_subscription(
            Image, "/rgb", self.on_image, qos_profile_sensor_data
        )
        self.pub = self.create_publisher(Int32, "/color_id", 10)
        self._candidate_id = None
        self._candidate_streak = 0
        self._last_ns = None
        self.get_logger().info("color_detector 시작 → /rgb 구독, /color_id 발행")

    def on_image(self, msg):
        # 프레임이 한동안 끊기면(팔이 잡으러 갔다 첫 위치로 복귀) 스트릭 재무장
        now = self.get_clock().now().nanoseconds
        if self._last_ns is not None and (now - self._last_ns) > FRAME_GAP_NS:
            self._candidate_id = None
            self._candidate_streak = 0
        self._last_ns = now

        rgb = imgmsg_to_rgb(msg)
        hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
        # 화면 중앙 ROI 만 판정 → 가장자리 비콘 배제
        h, w = hsv.shape[:2]
        ry, rx = int(h * ROI_FRAC / 2), int(w * ROI_FRAC / 2)
        hsv = hsv[h // 2 - ry:h // 2 + ry, w // 2 - rx:w // 2 + rx]
        total_px = hsv.shape[0] * hsv.shape[1]

        best_color = None
        best_count = 0
        for name, spec in HSV_RANGES.items():
            mask = cv2.inRange(hsv, spec["lower"], spec["upper"])
            count = int(cv2.countNonZero(mask))
            if count > best_count:
                best_color, best_count = name, count

        if best_color is None or best_count < total_px * MIN_BLOB_RATIO:
            self._candidate_id = None
            self._candidate_streak = 0
            return  # 뚜렷한 색이 없으면 발행하지 않음

        color_id = HSV_RANGES[best_color]["id"]

        # 같은 색이 연속으로 나올 때만 후보로 인정 (플리커 방지)
        if color_id == self._candidate_id:
            self._candidate_streak += 1
        else:
            self._candidate_id = color_id
            self._candidate_streak = 1

        # 스트릭이 기준에 "막 도달한" 프레임에서만 1회 발행 (라운드가 바뀌어도
        # 이전 발행 색과 무관하게, 새로 안정적으로 보이면 항상 다시 발행됨)
        if self._candidate_streak != CONFIRM_FRAMES:
            return

        out = Int32()
        out.data = color_id
        self.pub.publish(out)
        self.get_logger().info(f"[감지] {best_color} (id={color_id}, px={best_count})")


def main():
    rclpy.init()
    node = ColorDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
