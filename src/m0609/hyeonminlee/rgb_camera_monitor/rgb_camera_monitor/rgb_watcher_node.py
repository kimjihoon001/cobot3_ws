import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Bool

# 색상 이름 -> HSV(OpenCV 기준 H:0~179) 범위 프리셋.
# red는 hue가 0 부근에서 감싸 돌기 때문에 두 구간으로 나눠 둠.
COLOR_PRESETS = {
    "blue":  [((100, 80, 40), (130, 255, 255))],
    "green": [((40, 60, 40), (80, 255, 255))],
    "red":   [((0, 100, 60), (10, 255, 255)), ((160, 100, 60), (179, 255, 255))],
}


class RgbWatcherNode(Node):
    def __init__(self):
        """/rgb 토픽 파라미터, 구독/발행, 워치독 타이머를 설정"""
        super().__init__("rgb_watcher_node")

        self.declare_parameter("topic", "/rgb")
        self.declare_parameter("target_color", "blue")
        self.declare_parameter("min_area", 500)
        self.declare_parameter("watchdog_timeout_sec", 2.0)

        self._topic = self.get_parameter("topic").value
        self._target_color = self.get_parameter("target_color").value
        self._min_area = self.get_parameter("min_area").value
        self._watchdog_timeout_sec = self.get_parameter("watchdog_timeout_sec").value

        if self._target_color not in COLOR_PRESETS:
            self.get_logger().warn(
                f"알 수 없는 target_color='{self._target_color}', 기본값 blue 사용"
            )
            self._target_color = "blue"

        self._bridge = CvBridge()
        self._last_msg_time = None
        self._stream_alive = False

        self._sub = self.create_subscription(
            Image, self._topic, self._image_callback, qos_profile_sensor_data
        )
        self._detected_pub = self.create_publisher(Bool, f"{self._topic}/color_detected", 10)
        self._centroid_pub = self.create_publisher(Point, f"{self._topic}/color_centroid", 10)
        self._debug_pub = self.create_publisher(Image, f"{self._topic}/detection_image", 10)

        self.create_timer(1.0, self._watchdog_callback)

        self.get_logger().info(
            f"'{self._topic}' 구독 시작 (target_color={self._target_color}, "
            f"min_area={self._min_area})"
        )

    def _image_callback(self, msg: Image):
        """프레임 수신 시각을 갱신하고, 이미지에서 대상 색상 영역을 검출"""
        self._last_msg_time = self.get_clock().now()
        if not self._stream_alive:
            self._stream_alive = True
            self.get_logger().info(f"'{self._topic}' 카메라 프레임 수신 시작됨")

        frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = self._build_color_mask(hsv)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        detected = False
        if contours:
            largest = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest)
            if area >= self._min_area:
                detected = True
                m = cv2.moments(largest)
                cx = int(m["m10"] / m["m00"])
                cy = int(m["m01"] / m["m00"])
                cv2.drawContours(frame, [largest], -1, (0, 255, 0), 2)
                cv2.circle(frame, (cx, cy), 6, (0, 0, 255), -1)
                self._centroid_pub.publish(Point(x=float(cx), y=float(cy), z=0.0))
                self.get_logger().info(
                    f"[{self._target_color}] 검출됨: 픽셀=({cx},{cy}), area={area:.0f}",
                    throttle_duration_sec=1.0,
                )

        self._detected_pub.publish(Bool(data=detected))
        debug_msg = self._bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        debug_msg.header = msg.header
        self._debug_pub.publish(debug_msg)

    def _build_color_mask(self, hsv: np.ndarray) -> np.ndarray:
        """target_color 프리셋의 HSV 범위(들)를 합쳐 이진 마스크로 반환"""
        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for lower, upper in COLOR_PRESETS[self._target_color]:
            mask |= cv2.inRange(hsv, np.array(lower), np.array(upper))
        return mask

    def _watchdog_callback(self):
        """일정 시간 이상 새 프레임이 없으면 토픽 미발행/끊김을 경고"""
        now = self.get_clock().now()
        if self._last_msg_time is None:
            self.get_logger().warn(
                f"'{self._topic}' 토픽이 아직 수신되지 않음 (발행 중인 노드가 있는지 확인)",
                throttle_duration_sec=5.0,
            )
            return
        elapsed_sec = (now - self._last_msg_time).nanoseconds / 1e9
        if elapsed_sec > self._watchdog_timeout_sec:
            self._stream_alive = False
            self.get_logger().warn(
                f"'{self._topic}' 프레임 끊김: 마지막 수신 후 {elapsed_sec:.1f}초 경과",
                throttle_duration_sec=5.0,
            )


def main():
    """rclpy 초기화 후 RgbWatcherNode를 spin"""
    rclpy.init()
    node = RgbWatcherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
