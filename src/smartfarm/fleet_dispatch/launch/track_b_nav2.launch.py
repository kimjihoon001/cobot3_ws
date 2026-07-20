# 트랙 B 담당: 정적맵 1회 생성 후 freeze, 로봇별 AMCL은 개별 인스턴스로 실행.
# TODO: map_yaml_path를 실제 occupancy map 산출물 경로로 채우고,
#       수확 MM / 운반 AMR 각각의 네임스페이스로 nav2_bringup의 bringup_launch.py를 include.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("map_yaml_path", default_value=""),  # TODO
            # TODO: IncludeLaunchDescription(nav2_bringup bringup_launch.py, namespace=...) x 로봇 수
        ]
    )
