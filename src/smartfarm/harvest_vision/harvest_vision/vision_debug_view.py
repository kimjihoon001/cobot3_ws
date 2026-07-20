import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

IMAGE_QOS = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST, reliability=ReliabilityPolicy.BEST_EFFORT)
WINDOW_NAME = "vision_debug_view (rgb | depth)"


class VisionDebugView(Node):
    """vision_node와 같은 /rgb, /depth를 구독해 나란히 띄워서 보는 디버그 창 (rqt_image_view 대용)."""

    def __init__(self):
        super().__init__("vision_debug_view")

        self.declare_parameter("rgb_topic", "/rgb")
        self.declare_parameter("depth_topic", "/depth")

        rgb_topic = self.get_parameter("rgb_topic").value
        depth_topic = self.get_parameter("depth_topic").value

        self._bridge = CvBridge()
        self._rgb_frame = None
        self._depth_frame = None

        self.create_subscription(Image, rgb_topic, self._rgb_callback, IMAGE_QOS)
        self.create_subscription(Image, depth_topic, self._depth_callback, IMAGE_QOS)

        cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
        self.create_timer(1.0 / 30.0, self._render)

        self.get_logger().info(f"'{rgb_topic}', '{depth_topic}' 구독 시작, 디버그 창 표시")

    def _rgb_callback(self, msg: Image):
        self._rgb_frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def _depth_callback(self, msg: Image):
        depth = self._bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        depth = np.nan_to_num(depth, nan=0.0, posinf=0.0, neginf=0.0)
        normalized = cv2.normalize(depth, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
        self._depth_frame = cv2.applyColorMap(normalized, cv2.COLORMAP_JET)

    def _render(self):
        if self._rgb_frame is None or self._depth_frame is None:
            return

        depth_resized = cv2.resize(self._depth_frame, (self._rgb_frame.shape[1], self._rgb_frame.shape[0]))
        combined = np.hstack((self._rgb_frame, depth_resized))
        cv2.imshow(WINDOW_NAME, combined)
        cv2.waitKey(1)


def main():
    rclpy.init()
    node = VisionDebugView()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
