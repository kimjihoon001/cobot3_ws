# -*- coding: utf-8 -*-
"""흩어진 로봇 상태를 /ui/status 하나로 모아 5Hz JSON으로 뿌린다.

React가 토픽 열댓 개를 각각 구독하고 커스텀 메시지 타입을 TypeScript로
다시 정의하는 걸 막는 게 목적이다. 여기서 평탄화해 두면 프론트는 구독
하나에 JSON.parse 한 번이면 끝난다.

메시지는 std_msgs/String에 JSON을 담는다. ROS 관점에선 커스텀 .msg가
정석이지만 UI 피드는 필드가 계속 바뀌는 자리라, 필드 하나 추가할 때마다
인터페이스 패키지를 재빌드하는 비용이 더 크다.

카메라 Hz는 압축 토픽 도착 간격으로 잰다. 디버깅에서 제일 자주 필요한
질문이 "이 스트림이 아직 살아있나"라서 화면 오버레이의 핵심 값이 된다.
"""
from __future__ import annotations

import json
import math
import time
from collections import deque

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from smartfarm_interfaces.msg import TomatoDetectionArray

# 공정 순서대로 놓인 스트림. stream.launch.py의 remap 결과와 같아야 한다.
# overview는 격자에 없지만 확대해서 볼 수 있어 Hz를 같이 잰다.
PANES = ("mm_front", "greenhouse", "unloading", "storage", "overview")

# Hz 추정 창. 너무 짧으면 값이 튀고 길면 끊긴 걸 늦게 알아챈다.
HZ_WINDOW_SEC = 2.0

EVENT_LIMIT = 20

LATCHED_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


def _yaw(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class UiStatusNode(Node):
    def __init__(self) -> None:
        super().__init__("ui_status_node")

        self._frames: dict[str, deque] = {name: deque() for name in PANES}
        self._robots: dict[str, dict] = {
            "forklift": {"state": "", "x": None, "y": None, "yaw": None},
            "iw": {"state": "", "x": None, "y": None, "yaw": None},
        }
        self._events: deque = deque(maxlen=EVENT_LIMIT)
        self._harvest = {
            "detected": 0,
            "ripe": 0,
            "spoiled": 0,
            "unknown": 0,
            "harvested": 0,
            "failed": 0,
            "state": "",
            "quality_enabled": False,
        }
        self._market = {
            "available": False,
            "product": "토마토",
            "date": "",
            "average_price": None,
            "minimum_price": None,
            "maximum_price": None,
            "quantity": None,
            "change_rate": None,
            "unit": "원/kg",
            "updated_at": "",
            "message": "공공데이터 API 키 설정 대기",
        }

        for name in PANES:
            # 이미지 데이터는 쓰지 않고 도착 시각만 센다. 압축본이라 이 구독
            # 하나가 추가로 먹는 대역폭은 스트림당 수백 KB/s 수준이다.
            self.create_subscription(
                CompressedImage, f"/ui/{name}/compressed",
                self._make_frame_cb(name), qos_profile_sensor_data)

        self.create_subscription(
            String, "/forklift/status", self._make_state_cb("forklift"), 10)
        self.create_subscription(
            String, "/iw/status", self._make_state_cb("iw"), 10)
        self.create_subscription(
            PoseStamped, "/forklift_0/pose", self._on_forklift_pose, 10)
        self.create_subscription(
            Odometry, "/iwhub_0/odom", self._on_iw_odom, 10)
        self.create_subscription(
            TomatoDetectionArray,
            "/harvester_0/vision/tomato_detections",
            self._on_detections,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            String,
            "/harvester_0/manipulator/target_state",
            self._on_harvest_state,
            10,
        )
        self.create_subscription(
            String, "/market/summary", self._on_market, LATCHED_QOS)

        self._pub = self.create_publisher(String, "/ui/status", 10)
        self.create_timer(0.2, self._publish)

    # ── 수집 ──
    def _make_frame_cb(self, name: str):
        def _cb(_msg) -> None:
            self._frames[name].append(time.monotonic())
        return _cb

    def _make_state_cb(self, robot: str):
        def _cb(msg: String) -> None:
            text = msg.data.strip()
            if not text or text == self._robots[robot]["state"]:
                return
            self._robots[robot]["state"] = text
            self._events.append(
                {"t": self._sim_time(), "src": robot, "text": text})
        return _cb

    def _on_forklift_pose(self, msg: PoseStamped) -> None:
        p = msg.pose
        self._robots["forklift"].update(
            x=p.position.x, y=p.position.y, yaw=_yaw(p.orientation))

    def _on_iw_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose
        self._robots["iw"].update(
            x=p.position.x, y=p.position.y, yaw=_yaw(p.orientation))

    def _on_detections(self, msg: TomatoDetectionArray) -> None:
        classes = [item.tomato_class.strip().lower() for item in msg.detections]
        self._harvest.update(
            detected=len(classes),
            ripe=classes.count("ripe"),
            spoiled=classes.count("spoiled"),
            unknown=sum(name not in {"ripe", "spoiled"} for name in classes),
            quality_enabled=any(name in {"ripe", "spoiled"} for name in classes),
        )

    def _on_harvest_state(self, msg: String) -> None:
        state = msg.data.strip()
        previous = self._harvest["state"]
        if not state or state == previous:
            return
        self._harvest["state"] = state
        # BASKET_RETRACT는 스쿱 개방 응답을 받은 뒤에만 진입하므로 적재 완료로 센다.
        if state == "BASKET_RETRACT":
            self._harvest["harvested"] += 1
        elif state == "HARVEST_FAILED":
            self._harvest["failed"] += 1
        self._events.append(
            {"t": self._sim_time(), "src": "mm", "text": state})

    def _on_market(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except (TypeError, ValueError, json.JSONDecodeError):
            return
        if isinstance(payload, dict):
            self._market.update(payload)

    # ── 발행 ──
    def _sim_time(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _hz(self, name: str) -> float:
        stamps = self._frames[name]
        cutoff = time.monotonic() - HZ_WINDOW_SEC
        while stamps and stamps[0] < cutoff:
            stamps.popleft()
        if len(stamps) < 2:
            return 0.0
        return round((len(stamps) - 1) / (stamps[-1] - stamps[0]), 1)

    def _publish(self) -> None:
        payload = {
            "sim_time": round(self._sim_time(), 2),
            "cameras": {name: self._hz(name) for name in PANES},
            "robots": self._robots,
            "harvest": self._harvest,
            "market": self._market,
            "events": list(self._events),
        }
        self._pub.publish(String(data=json.dumps(payload)))


def main() -> None:
    rclpy.init()
    node = UiStatusNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
