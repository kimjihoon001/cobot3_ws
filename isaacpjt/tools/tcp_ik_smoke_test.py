#!/usr/bin/env python3
"""실제 camera TF와 tomato GT로 HarvestTCP PREGRASP IK를 검증한다."""

import json
import time

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener


class TcpIkSmokeTest(Node):
    def __init__(self):
        super().__init__("tcp_ik_smoke_test")
        self._buffer = Buffer()
        self._listener = TransformListener(self._buffer, self)
        self._fruits: list[np.ndarray] = []
        self._statuses: list[dict] = []
        self._pub = self.create_publisher(String, "/harvester_0/cmd", 10)
        self.create_subscription(
            String, "/harvester_0/sim/tomato", self._fruit_cb, 20)
        self.create_subscription(
            String, "/harvester_0/rmpflow/status", self._status_cb, 20)

    def _fruit_cb(self, msg: String):
        try:
            value = np.asarray(json.loads(msg.data)["position"], dtype=float)
        except (KeyError, TypeError, ValueError, json.JSONDecodeError):
            return
        if value.shape == (3,) and not any(
                np.linalg.norm(value - known) < 0.01 for known in self._fruits):
            self._fruits.append(value)

    def _status_cb(self, msg: String):
        try:
            self._statuses.append(json.loads(msg.data))
        except json.JSONDecodeError:
            pass

    def publish(self, value: dict, seconds: float = 1.0):
        msg = String(data=json.dumps(value, separators=(",", ":")))
        until = time.monotonic() + seconds
        while time.monotonic() < until:
            self._pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)

    def camera_position(self) -> np.ndarray:
        transform = self._buffer.lookup_transform(
            "base_link", "d455_color", Time(), timeout=Duration(seconds=1.0))
        p = transform.transform.translation
        return np.array([p.x, p.y, p.z], dtype=float)


def main():
    rclpy.init()
    node = TcpIkSmokeTest()
    try:
        deadline = time.monotonic() + 8.0
        camera = None
        while time.monotonic() < deadline and (camera is None or not node._fruits):
            rclpy.spin_once(node, timeout_sec=0.1)
            try:
                camera = node.camera_position()
            except TransformException:
                pass
        if camera is None or not node._fruits:
            raise RuntimeError("camera TF 또는 tomato GT를 받지 못했습니다")

        # 카메라에 가장 가까우면서 PREGRASP가 작업영역 안인 과실을 선택한다.
        candidates = []
        for fruit in node._fruits:
            ray = fruit - camera
            distance = float(np.linalg.norm(ray))
            if distance < 1e-6:
                continue
            pregrasp = fruit - ray / distance * 0.15
            if (0.15 <= pregrasp[0] <= 1.25
                    and abs(pregrasp[1]) <= 0.75
                    and 0.15 <= pregrasp[2] <= 1.80):
                candidates.append((distance, fruit, pregrasp))
        if not candidates:
            raise RuntimeError("작업영역 안의 PREGRASP 후보가 없습니다")
        camera_distance, fruit, pregrasp = min(candidates, key=lambda x: x[0])
        print(json.dumps({
            "camera": camera.round(4).tolist(),
            "tomato": fruit.round(4).tolist(),
            "camera_tomato_m": round(camera_distance, 4),
            "pregrasp": pregrasp.round(4).tolist(),
        }))

        node.publish({"gripper": {"closed": False}}, 0.5)
        command_id = 9101
        node.publish({"rmp_target": {
            "id": command_id,
            "phase": "PREGRASP",
            "frame_id": "base_link",
            "position": pregrasp.tolist(),
        }})
        deadline = time.monotonic() + 30.0
        result = None
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            matching = [s for s in node._statuses if s.get("id") == command_id]
            if matching:
                result = matching[-1]
                if result.get("reached") or str(result.get("phase", "")).startswith("ERROR"):
                    break
        print(json.dumps({"pregrasp_status": result}))
        node.publish({"rmp_stop": True}, 0.5)
        node.publish({"rmp_home": {"id": 9102}})
        deadline = time.monotonic() + 30.0
        home = None
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            matching = [s for s in node._statuses if s.get("id") == 9102]
            if matching:
                home = matching[-1]
                if home.get("at_home"):
                    break
        print(json.dumps({"home_status": home}))
        if not result or not result.get("reached") or not home or not home.get("at_home"):
            raise RuntimeError("TCP IK smoke test 실패")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
