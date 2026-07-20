import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

from smartfarm_interfaces.msg import TomatoDetection, TomatoDetectionArray

# 팀 컨벤션(day2/color_detector_node.py): 이미지는 최신 한 장만, 밀리면 버린다.
# 기본 QoS(RELIABLE, depth 10)로 두면 처리 지연 시 DDS가 프레임을 쌓아 묵은 프레임을 보게 됨.
IMAGE_QOS = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST, reliability=ReliabilityPolicy.BEST_EFFORT)


class VisionNode(Node):
    def __init__(self):
        """eye-in-hand RGB-D에서 토마토를 검출해 /vision/tomato_detections로 발행 (초안: 검출 로직 TODO)"""
        super().__init__("vision_node")

        # [4] 임의: isaacpjt/ros/robot_bridge.py에는 아직 harvester_0 카메라 브리지가
        # 없음(joint_command/joint_states/String만 배선됨) — /rgb,/depth는 팀이 day1~2에서
        # 검증한 이름을 그대로 기본값으로 둠. 카메라 그래프가 실제로 붙을 때
        # /harvester_0/rgb 처럼 네임스페이스가 바뀔 수 있어 파라미터로 뺐다.
        self.declare_parameter("rgb_topic", "/rgb")
        self.declare_parameter("depth_topic", "/depth")
        self.declare_parameter("model_path", "")  # YOLO 학습 가중치(.pt) 경로

        self._rgb_topic = self.get_parameter("rgb_topic").value
        self._depth_topic = self.get_parameter("depth_topic").value
        self._model_path = self.get_parameter("model_path").value

        self.create_subscription(Image, self._rgb_topic, self._rgb_callback, IMAGE_QOS)
        self.create_subscription(Image, self._depth_topic, self._depth_callback, IMAGE_QOS)

        self._detections_pub = self.create_publisher(TomatoDetectionArray, "/vision/tomato_detections", 10)

        self._latest_depth = None

        self.get_logger().info(f"'{self._rgb_topic}', '{self._depth_topic}' 구독 시작, /vision/tomato_detections 발행 대기")

    def _depth_callback(self, msg: Image):
        self._latest_depth = msg

    def _rgb_callback(self, msg: Image):
        """TODO: YOLO(model_path) 추론 -> pick point(3D pose) 추정 -> TomatoDetectionArray 발행"""
        detections = self._run_detection(msg, self._latest_depth)
        if not detections:
            return

        array_msg = TomatoDetectionArray()
        array_msg.header = msg.header
        array_msg.detections = detections
        self._detections_pub.publish(array_msg)

    def _run_detection(self, rgb_msg: Image, depth_msg: Image) -> list[TomatoDetection]:
        """TODO: 실제 YOLO 추론 + depth 역투영으로 pose 채우기. 지금은 빈 리스트 반환"""
        return []


def main():
    rclpy.init()
    node = VisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
