# -*- coding: utf-8 -*-
"""BEST_EFFORT로 오는 이미지를 RELIABLE로 한 번 되뿌린다.

image_transport republish는 구독 QoS가 RELIABLE로 고정돼 있어서, 센서
QoS(BEST_EFFORT)로 발행되는 토픽을 한 장도 받지 못한다. 발행자 쪽
경고로 확인된다:

    New publisher discovered on topic '...', offering incompatible QoS.
    No messages will be sent to it. Last incompatible policy: RELIABILITY

republish에 qos_overrides 파라미터를 줘도 적용되지 않는다(2026-07-26 실측).
그래서 QoS만 바꿔 통과시키는 이 노드를 앞에 세우고, raw→JPEG 변환은
그대로 republish가 하게 둔다. 압축 품질이 화면마다 달라지지 않는다.

메시지는 손대지 않는다. 여기서 인코딩까지 하면 이 노드가 두 번째
이미지 파이프라인이 되고, 다른 화면과 품질이 갈린다.
"""
from __future__ import annotations

import os

import rclpy
from rclpy.node import Node
from rclpy.exceptions import ParameterUninitializedException
from rclpy.executors import ExternalShutdownException
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class QosBridge(Node):
    def __init__(self) -> None:
        super().__init__("qos_bridge")
        self.declare_parameter("in_topic", "")
        self.declare_parameter(
            "fallback_topics", Parameter.Type.STRING_ARRAY)
        self.declare_parameter("out_topic", "")
        in_topic = str(self.get_parameter("in_topic").value)
        try:
            fallback_value = self.get_parameter("fallback_topics").value
        except ParameterUninitializedException:
            fallback_value = []
        fallback_topics = [
            str(topic) for topic in fallback_value if str(topic)
        ]
        out_topic = str(self.get_parameter("out_topic").value)
        if not in_topic or not out_topic:
            raise SystemExit("in_topic / out_topic 파라미터가 필요합니다.")

        # 발행은 기본(RELIABLE) — 뒤에 붙는 republish가 그걸 요구한다.
        self._pub = self.create_publisher(Image, out_topic, 1)
        # 실행 launch에 따라 vision_node namespace가 달라질 수 있다. 같은 토픽은
        # 중복 구독하지 않고, 실제 발행자가 존재하는 입력 어느 쪽이든 relay한다.
        inputs = list(dict.fromkeys([in_topic, *fallback_topics]))
        self._subscriptions = [
            self.create_subscription(
                Image, topic, self._pub.publish, qos_profile_sensor_data)
            for topic in inputs
        ]
        self.get_logger().info(
            f"QoS 브리지: {', '.join(inputs)} → {out_topic}")


def main() -> None:
    rclpy.init()
    node = QosBridge()
    is_jazzy = os.environ.get("ROS_DISTRO") == "jazzy"
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # Jazzy는 SIGINT 중 destroy_node()가 다시 KeyboardInterrupt를 받을 수
        # 있어서 context를 먼저 닫는다. Humble은 기존 종료 순서를 유지한다.
        try:
            if is_jazzy and rclpy.ok():
                rclpy.shutdown()
            node.destroy_node()
            if not is_jazzy and rclpy.ok():
                rclpy.shutdown()
        except KeyboardInterrupt:
            pass
