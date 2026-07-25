# -*- coding: utf-8 -*-
"""화면의 REC 버튼으로 rosbag 녹화를 켜고 끈다.

결과물은 영상 파일이 아니라 토픽 기록이다. 카메라 4~5개와 로봇 좌표·상태가
같은 타임스탬프로 묶여서, "도킹 실패한 그 순간 카메라엔 뭐가 보였나"를 프레임
단위로 맞춰볼 수 있다. 보려면 `ros2 bag play`로 틀면 이 UI가 그대로 재생기가
된다(시계를 /clock 기준으로 읽으므로 시각도 녹화 당시로 돌아간다).

발표용 mp4가 필요하면 이걸 쓰지 말고 MJPEG를 ffmpeg로 직접 받는 게 맞다.
용도가 달라서 담을 내용도 다르다.

rosbag2_py API 대신 `ros2 bag record` 프로세스를 띄운다. 저장 포맷·QoS·파일
분할을 rosbag2가 알아서 처리해줘서 코드가 훨씬 짧다.
"""
from __future__ import annotations

import json
import os
import signal
import subprocess
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

# raw 이미지는 절대 넣지 않는다. 압축본이 초당 2MB인데 raw는 50MB라
# 시간당 180GB가 된다. 나머지는 전부 작은 메시지라 셈에 안 들어간다.
TOPICS = (
    "/ui/mm_front/compressed",
    "/ui/greenhouse/compressed",
    "/ui/unloading/compressed",
    "/ui/storage/compressed",
    "/ui/overview/compressed",
    "/ui/status",
    "/clock",
    "/forklift/status",
    "/forklift/handoff_state",
    "/iw/status",
    "/iw/mission",
    "/forklift_0/pose",
    "/iwhub_0/odom",
)


class RecorderNode(Node):
    def __init__(self) -> None:
        super().__init__("recorder_node")
        self.declare_parameter("output_dir", os.path.expanduser("~/bags"))
        # 실수로 켜둔 채 두면 시간당 7GB씩 쌓인다. 잘라두면 필요한 구간만
        # 남기고 지우기 쉽다.
        self.declare_parameter("split_seconds", 300)

        self._output_dir = str(self.get_parameter("output_dir").value)
        self._split = int(self.get_parameter("split_seconds").value)
        self._proc: subprocess.Popen | None = None
        self._path = ""
        self._started = 0.0

        self.create_service(Trigger, "/recording/start", self._on_start)
        self.create_service(Trigger, "/recording/stop", self._on_stop)
        self._status_pub = self.create_publisher(String, "/recording/status", 10)
        self.create_timer(0.5, self._publish_status)

    def _on_start(self, _request, response):
        if self._proc and self._proc.poll() is None:
            response.success = False
            response.message = "이미 녹화 중입니다"
            return response

        os.makedirs(self._output_dir, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self._path = os.path.join(self._output_dir, f"monitor_{stamp}")
        command = [
            "ros2", "bag", "record",
            "-s", "mcap",              # Foxglove로 바로 열린다
            "-o", self._path,
            "--max-bag-duration", str(self._split),
            *TOPICS,
        ]
        try:
            # 자체 프로세스 그룹으로 띄워야 정지할 때 SIGINT를 자식까지
            # 한 번에 보낼 수 있다.
            self._proc = subprocess.Popen(command, start_new_session=True)
        except FileNotFoundError:
            response.success = False
            response.message = "ros2 bag 실행 파일을 찾지 못했습니다"
            return response

        self._started = time.monotonic()
        self.get_logger().info(f"녹화 시작: {self._path}")
        response.success = True
        response.message = os.path.basename(self._path)
        return response

    def _on_stop(self, _request, response):
        if not self._proc or self._proc.poll() is not None:
            self._proc = None
            response.success = False
            response.message = "녹화 중이 아닙니다"
            return response

        # SIGKILL로 죽이면 rosbag2가 파일을 못 닫아 기록이 깨진다.
        # SIGINT를 보내 정상 종료를 기다린다.
        os.killpg(os.getpgid(self._proc.pid), signal.SIGINT)
        try:
            self._proc.wait(timeout=10.0)
        except subprocess.TimeoutExpired:
            self.get_logger().warning("녹화 종료가 늦어 강제 종료합니다")
            os.killpg(os.getpgid(self._proc.pid), signal.SIGKILL)
            self._proc.wait(timeout=5.0)

        self._proc = None
        self.get_logger().info(f"녹화 종료: {self._path}")
        response.success = True
        response.message = os.path.basename(self._path)
        return response

    def _publish_status(self) -> None:
        recording = bool(self._proc and self._proc.poll() is None)
        payload = {
            "recording": recording,
            "name": os.path.basename(self._path) if recording else "",
            "elapsed": round(time.monotonic() - self._started, 1) if recording else 0.0,
        }
        self._status_pub.publish(String(data=json.dumps(payload)))


def main() -> None:
    rclpy.init()
    node = RecorderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 노드가 죽어도 bag 프로세스가 남아 계속 디스크를 먹지 않게 한다.
        if node._proc and node._proc.poll() is None:
            os.killpg(os.getpgid(node._proc.pid), signal.SIGINT)
            node._proc.wait(timeout=10.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
