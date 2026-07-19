"""
color_detector 노드 (PC B) — 순수 ROS2
  구독:  /rgb       (sensor_msgs/Image, rgb8)
  발행:  /color_id  (std_msgs/Int32)   파랑=1, 초록=2

OpenCV(HSV)로 파랑/초록 픽셀 수를 비교해 우세한 색을 발행.
cv_bridge 는 쓰지 않고 np.frombuffer 로 직접 디코딩.

★ QoS: /rgb 는 BEST_EFFORT + depth 1 로 구독한다. 기본 QoS(RELIABLE, depth 10)로
  두면 처리가 밀릴 때 DDS 가 이미지를 버리지 않고 쌓아 버퍼가 터지고, 뒤늦게
  묵은 프레임을 판정하게 된다. 최신 한 장만 보고 나머지는 버리는 게 맞다.

실행:
  export ROS_DOMAIN_ID=108          # PC A 와 동일해야 통신됨
  source /opt/ros/humble/setup.bash
  python3 color_detector_node.py
"""

import os
# ★ PC A(Isaac 노드)와 반드시 같은 도메인이어야 통신됨.
os.environ["ROS_DOMAIN_ID"] = "108"

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Int32

# 이미지: 최신 한 장만. 밀리면 쌓지 말고 버린다.
IMAGE_QOS = QoSProfile(
    depth=1, history=HistoryPolicy.KEEP_LAST, reliability=ReliabilityPolicy.BEST_EFFORT
)
# 판정 결과: 작으니 확실히 전달하되, 역시 최신 것만.
RESULT_QOS = QoSProfile(
    depth=1, history=HistoryPolicy.KEEP_LAST, reliability=ReliabilityPolicy.RELIABLE
)

# OpenCV HSV 스케일: H 0~179 (=deg/2), S/V 0~255
SAT_MIN, VAL_MIN = 102, 76          # 0.4, 0.3 에 해당
BLUE_HSV  = (np.array([100, SAT_MIN, VAL_MIN]), np.array([130, 255, 255]))   # 200~260deg
GREEN_HSV = (np.array([40,  SAT_MIN, VAL_MIN]), np.array([80,  255, 255]))   # 80~160deg
MIN_PIXELS = 50            # 이 픽셀 수 미만이면 미검출


class ColorDetectorNode(Node):
    def __init__(self):
        super().__init__("color_detector")
        self.create_subscription(Image, "/rgb", self._on_image, IMAGE_QOS)
        self._pub = self.create_publisher(Int32, "/color_id", RESULT_QOS)
        self.get_logger().info("구독: /rgb  발행: /color_id  (파랑=1, 초록=2)")

    def _on_image(self, msg: Image):
        if msg.encoding != "rgb8":
            return
        rgb = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)

        blue_cnt  = cv2.countNonZero(cv2.inRange(hsv, *BLUE_HSV))
        green_cnt = cv2.countNonZero(cv2.inRange(hsv, *GREEN_HSV))

        # 화면엔 큐브 한 개만 보이므로, 더 많이 잡힌 색을 채택한다.
        top_cnt = max(blue_cnt, green_cnt)
        if top_cnt < MIN_PIXELS:
            return                                    # 미검출: 발행 안 함
        color_id = 1 if blue_cnt >= green_cnt else 2

        self._pub.publish(Int32(data=color_id))
        self.get_logger().info(
            f"blue={blue_cnt} green={green_cnt} → /color_id={color_id}"
            f"({'blue' if color_id == 1 else 'green'})"
        )


def main():
    rclpy.init()
    node = ColorDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
