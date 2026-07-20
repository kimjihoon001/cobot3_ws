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
MIN_PIXELS = 50            # ROI 안에서 이 픽셀 수 미만이면 미검출
# ★ 비콘/대기 큐브가 화면 가장자리에 잡혀 오검출되는 걸 막기 위해 화면 '중앙'만 본다.
#   큐브는 항상 화각 중앙 근처(pick 영역)에 떨어지므로 중앙 ROI 로 충분하다.
ROI_FRAC = 0.4             # 중앙 40% x 40% 영역만 색 판정 (비콘 배제)
CONFIRM_FRAMES = 3         # 같은 색이 이만큼 연속돼야 '한 번' 발행 (오염 방지)
FRAME_GAP_NS = 500_000_000 # 0.5s 이상 프레임이 끊기면 새 관찰로 보고 재무장

# 디버그: DETECTOR_DEBUG=1 로 실행하면 판정 프레임을 PNG 로 저장(ROI 박스+카운트).
# "왜 자꾸 파란색?" 을 눈으로 확인하는 용도. 평소엔 꺼져 있어 동작에 영향 없음.
DEBUG = bool(os.environ.get("DETECTOR_DEBUG"))
DEBUG_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "detector_debug.png")


class ColorDetectorNode(Node):
    def __init__(self):
        super().__init__("color_detector")
        self.create_subscription(Image, "/rgb", self._on_image, IMAGE_QOS)
        self._pub = self.create_publisher(Int32, "/color_id", RESULT_QOS)
        self._reset_streak()
        self._last_ns = None
        self.get_logger().info("구독: /rgb  발행: /color_id  (파랑=1, 초록=2)")

    def _reset_streak(self):
        self._streak_id = None
        self._streak = 0
        self._published = False

    def _on_image(self, msg: Image):
        if msg.encoding != "rgb8":
            return

        # 프레임이 한동안 끊겼다 = 팔이 잡으러 갔다 첫 위치로 돌아온 '새 관찰'.
        # 스트릭을 풀어(re-arm) 다음 큐브가 나타나면 다시 '한 번' 발행하게 한다.
        now = self.get_clock().now().nanoseconds
        if self._last_ns is not None and (now - self._last_ns) > FRAME_GAP_NS:
            self._reset_streak()
        self._last_ns = now

        rgb = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)

        # 화면 중앙 ROI 만 잘라서 판정 → 가장자리 비콘은 계산에서 빠진다.
        h, w = hsv.shape[:2]
        ry, rx = int(h * ROI_FRAC / 2), int(w * ROI_FRAC / 2)
        roi = hsv[h // 2 - ry:h // 2 + ry, w // 2 - rx:w // 2 + rx]

        blue_cnt  = cv2.countNonZero(cv2.inRange(roi, *BLUE_HSV))
        green_cnt = cv2.countNonZero(cv2.inRange(roi, *GREEN_HSV))

        if DEBUG:
            dbg = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
            cv2.rectangle(dbg, (w // 2 - rx, h // 2 - ry),
                          (w // 2 + rx, h // 2 + ry), (0, 0, 255), 2)
            cv2.putText(dbg, f"B={blue_cnt} G={green_cnt}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
            cv2.imwrite(DEBUG_PATH, dbg)

        # ★ 초록도 파랑도 뚜렷하지 않으면(둘 다 MIN_PIXELS 미만) 발행하지 않는다.
        if max(blue_cnt, green_cnt) < MIN_PIXELS:
            self._reset_streak()
            return

        color_id = 1 if blue_cnt >= green_cnt else 2

        # 같은 색이 CONFIRM_FRAMES 연속일 때 '한 번만' 발행 → /color_id 오염 방지
        if color_id == self._streak_id:
            self._streak += 1
        else:
            self._streak_id = color_id
            self._streak = 1
            self._published = False

        if self._streak >= CONFIRM_FRAMES and not self._published:
            self._pub.publish(Int32(data=color_id))
            self._published = True
            self.get_logger().info(
                f"blue={blue_cnt} green={green_cnt} → /color_id={color_id}"
                f"({'blue' if color_id == 1 else 'green'})  [발행]"
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
