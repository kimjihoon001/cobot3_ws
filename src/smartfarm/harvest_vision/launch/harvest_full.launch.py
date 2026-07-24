"""이동→인식→파지 통합 실행 — 비전 + 디버그창 + 오케스트레이션 + 파지(RMPflow).

한 번에 켜는 것: (Isaac 은 별도 — isaac_python main.py --mm --rmpflow [--nav])
  - vision_node            : /harvester_0/rgb,depth → YOLO 검출 → /vision/* 발행
  - vision_debug_view      : /vision/annotated_image 를 OpenCV 창으로 (비전 디버깅)
  - manipulator_target_node: /vision/approach_target → /harvester_0/cmd rmp_target (파지, 검증된 경로)
  - nav_harvest_test_node  : 섹터 이동→인식→수확→모의 바스켓 오케스트레이션

Nav2 는 **따로** 켠다 (아래 README 명령 참조). harvester_0 자체 이동은 nav_harvest_test_node 가
/harvester_0/cmd 의 base 텔레포트로 하며, Nav2(/cmd_vel) 를 쓰려면 main.py --nav + 별도 런치.

사용:
  colcon build --packages-select harvest_vision && source install/setup.bash
  ros2 launch harvest_vision harvest_full.launch.py
  (해제: --no-debug 대신 아래 use_debug 인자로 끈다)
  ros2 launch harvest_vision harvest_full.launch.py use_debug:=false
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("harvest_vision")
    base = os.path.join(share, "config", "manipulator_target.yaml")
    test = os.path.join(share, "config", "nav_harvest_test.yaml")
    sim_time = {"use_sim_time": True}
    use_debug = LaunchConfiguration("use_debug")

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_debug", default_value="true",
            description="vision_debug_view(OpenCV 창) 동시 실행 여부"),
        Node(
            package="harvest_vision", executable="vision_node",
            name="vision_node", output="screen",
            # vision_node 기본이 far-only(use_quality_model=False). 근거리 품질까지 켜려면
            # parameters 에 {"use_quality_model": True} 추가.
            parameters=[sim_time],
        ),
        Node(
            package="harvest_vision", executable="vision_debug_view",
            name="vision_debug_view", output="screen",
            parameters=[sim_time],
            condition=IfCondition(use_debug),
        ),
        Node(
            package="harvest_vision", executable="manipulator_target_node",
            name="manipulator_target_node", output="screen",
            parameters=[base, test, sim_time],
        ),
        Node(
            package="harvest_vision", executable="nav_harvest_test_node",
            name="nav_harvest_test_node", output="screen",
            parameters=[test, sim_time],
        ),
    ])
