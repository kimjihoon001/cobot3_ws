import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

IMAGE_QOS = QoSProfile(
    depth=1,
    history=HistoryPolicy.KEEP_LAST,
    reliability=ReliabilityPolicy.BEST_EFFORT,
)
WINDOW_NAME = "vision_debug_view (YOLO | depth)"


class VisionDebugView(Node):
    """vision_node의 YOLO 결과와 depth를 나란히 보여주는 디버그 창."""

    def __init__(self):
        super().__init__("vision_debug_view")

        self.declare_parameter("annotated_topic", "/vision/annotated_image")
        self.declare_parameter("depth_topic", "/harvester/depth")

        annotated_topic = self.get_parameter("annotated_topic").value
        depth_topic = self.get_parameter("depth_topic").value

        self._bridge = CvBridge()
        self._annotated_frame = None
        self._depth_frame = None

        self.create_subscription(Image, annotated_topic, self._annotated_callback, IMAGE_QOS)
        self.create_subscription(Image, depth_topic, self._depth_callback, IMAGE_QOS)

        cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
        self.create_timer(1.0 / 30.0, self._render)

        self.get_logger().info(
            f"'{annotated_topic}', '{depth_topic}' 구독 시작, YOLO 디버그 창 표시"
        )

    def _annotated_callback(self, msg: Image):
        self._annotated_frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def _depth_callback(self, msg: Image):
        depth = self._bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        depth = np.nan_to_num(depth, nan=0.0, posinf=0.0, neginf=0.0)
        normalized = cv2.normalize(depth, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
        self._depth_frame = cv2.applyColorMap(normalized, cv2.COLORMAP_JET)

    def _render(self):
        if self._annotated_frame is None:
            return
        if self._depth_frame is None:
            combined = self._annotated_frame
        else:
            depth_resized = cv2.resize(
                self._depth_frame,
                (self._annotated_frame.shape[1], self._annotated_frame.shape[0]),
            )
            combined = np.hstack((self._annotated_frame, depth_resized))
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
