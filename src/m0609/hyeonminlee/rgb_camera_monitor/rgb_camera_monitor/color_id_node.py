import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Int32

from rgb_camera_monitor.rgb_watcher_node import COLOR_PRESETS

# pick_place_color 시나리오 규약: 파랑=1, 초록=2
COLOR_ID_MAP = {"blue": 1, "green": 2}


class ColorIdNode(Node):
    def __init__(self):
        """/rgb 이미지에서 blue/green 큐브를 판별해 /color_id(Int32)로 발행"""
        super().__init__("color_id_node")

        self.declare_parameter("topic", "/rgb")
        self.declare_parameter("min_area", 500)
        self.declare_parameter("confirm_frames", 5)

        self._topic = self.get_parameter("topic").value
        self._min_area = self.get_parameter("min_area").value
        self._confirm_frames = self.get_parameter("confirm_frames").value

        self._bridge = CvBridge()
        self._last_color_id = 0
        self._stable_count = 0

        self.create_subscription(Image, self._topic, self._image_callback, qos_profile_sensor_data)
        self._id_pub = self.create_publisher(Int32, "/color_id", 10)

        self.get_logger().info(f"'{self._topic}' 구독 시작, /color_id 발행 대기")

    def _image_callback(self, msg: Image):
        """blue/green 마스크 면적을 비교해 더 넓은 쪽을 색상으로 판정, N프레임 연속되면 발행"""
        frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        areas = {}
        for color_name in ("blue", "green"):
            mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
            for lower, upper in COLOR_PRESETS[color_name]:
                mask |= cv2.inRange(hsv, np.array(lower), np.array(upper))
            areas[color_name] = int(cv2.countNonZero(mask))

        detected_color = max(areas, key=areas.get)
        if areas[detected_color] < self._min_area:
            self._stable_count = 0
            return

        detected_id = COLOR_ID_MAP[detected_color]
        if detected_id == self._last_color_id:
            self._stable_count += 1
        else:
            self._last_color_id = detected_id
            self._stable_count = 1

        if self._stable_count == self._confirm_frames:
            self._id_pub.publish(Int32(data=detected_id))
            self.get_logger().info(f"색상 판정: {detected_color} (id={detected_id}) -> /color_id 발행")


def main():
    """rclpy 초기화 후 ColorIdNode를 spin"""
    rclpy.init()
    node = ColorIdNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
