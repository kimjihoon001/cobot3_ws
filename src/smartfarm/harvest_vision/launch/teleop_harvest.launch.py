"""Nav2·YOLO 없이 텔레옵 수확 — manipulator 만 띄운다 (2026-07-22).

YOLO 검출이 불안정해 뺐다(direct_sim_grasp=true). 맵의 시뮬 좌표로 **젤 가까운 토마토를
바로** 잡는다. 베이스 위치는 별도 터미널의 키보드 텔레옵으로 잡는다:
    ros2 run harvest_vision harvest_teleop
그 텔레옵에서 'h' 를 누르면 /harvest_test/enable 에 True 가 나가 그 자리에서 수확 시작.

게이트 값은 nav_harvest_test.yaml 을 재사용한다 (command_enabled=true,
external_harvest_gate_enabled=true, direct_sim_grasp=true). nav_harvest_test_node 는
띄우지 않으므로 enable 을 외부(텔레옵)가 직접 제어한다.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("harvest_vision")
    base = os.path.join(share, "config", "manipulator_target.yaml")
    gate = os.path.join(share, "config", "nav_harvest_test.yaml")
    return LaunchDescription([
        # YOLO(vision_node) 제거 — direct_sim_grasp 로 맵 좌표 직접 사용(2026-07-22).
        Node(
            package="harvest_vision",
            executable="manipulator_target_node",
            name="manipulator_target_node",
            output="screen",
            parameters=[base, gate, {"use_sim_time": True}],
        ),
    ])
