"""모니터링 UI 전체 실행: 영상 + 상태 집계 + 브라우저 연결.

    isaacpjt/main.py --cctv ...          # Isaac 쪽 고정 카메라 켜기
    ros2 launch monitor_ui ui.launch.py  # 이 파일

영상은 HTTP MJPEG(8080), 상태 JSON은 WebSocket(9090)으로 나간다. 둘을 한
채널로 합치지 않는 이유는 raw든 압축이든 이미지를 WebSocket JSON으로
보내면 대역폭이 감당이 안 되기 때문이다.

영상만 따로 보려면 stream.launch.py를 직접 띄우면 된다. 문제가 생겼을 때
영상 배선과 데이터 배선을 갈라서 볼 수 있게 파일을 나눠 뒀다.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import (
    AnyLaunchDescriptionSource, PythonLaunchDescriptionSource)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("monitor_ui")
    rosbridge = os.path.join(
        get_package_share_directory("rosbridge_server"),
        "launch", "rosbridge_websocket_launch.xml")

    return LaunchDescription([
        DeclareLaunchArgument("camera_ns", default_value="harvester_0"),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(share, "launch", "stream.launch.py")),
            launch_arguments={
                "camera_ns": LaunchConfiguration("camera_ns"),
            }.items()),

        IncludeLaunchDescription(AnyLaunchDescriptionSource(rosbridge)),

        # 화면 시계는 wall clock이 아니라 /clock을 읽어야 한다. 안 그러면
        # bag을 재생할 때 그림은 어제 장면인데 시계만 오늘을 가리킨다.
        Node(
            package="monitor_ui",
            executable="ui_status_node",
            name="ui_status_node",
            parameters=[{"use_sim_time": True}],
            output="screen",
        ),
    ])
