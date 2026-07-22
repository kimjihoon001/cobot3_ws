# -*- coding: utf-8 -*-
"""iw.hub 베이스만 — /cmd_vel→바퀴, joint_states→odom(+TF). Nav2 없이 텔레옵·odom 확인용.

진단 실행: ros2 launch iwhub_control iwhub_base.launch.py
Nova 실제 chassis odom은 iwhub_nav2.launch.py가 publish_odom:=false로 포함한다.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    publish_odom = LaunchConfiguration("publish_odom")
    tf_remaps = [("/tf", "tf"), ("/tf_static", "tf_static")]
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true",
                              description="Isaac /clock 사용(시뮬)"),
        DeclareLaunchArgument(
            "publish_odom", default_value="true",
            description="true=바퀴 각도 odom, false=Isaac chassis odom을 외부에서 발행"),
        # base_link ↔ Isaac 루트 프레임 잇기: Isaac 은 라이다 TF 부모를 "IwHub"(아티큘레이션
        # 루트 프림 이름)로 낸다 — iw.hub 에셋에 base_link 프림이 없어서다. base_node/Nav2 는
        # "base_link" 를 쓰므로 둘을 항등 정적 TF 로 연결한다(같은 물리 프레임). 이게 없으면
        # 스캔(laser_*)이 base_link 에 안 붙어 slam 이 map→odom 을 못 만든다(2026-07-20 실측).
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="base_link_to_iwhub",
            namespace="iwhub_0",
            remappings=tf_remaps,
            arguments=[
                "--x", "0.0", "--y", "0.0", "--z", "0.0",
                "--yaw", "0.0", "--pitch", "0.0", "--roll", "0.0",
                "--frame-id", "iwhub_0/base_link", "--child-frame-id", "iwhub_0/chassis",
            ],
            parameters=[{"use_sim_time": use_sim_time}],
        ),
        # Nova Carter 방식의 전·후방 RPLIDAR 고정 마운트.
        # /tf_static으로 발행해 LaserScan과 동적 TF의 타임스탬프 경쟁을 막는다.
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="chassis_to_front_2d_lidar",
            namespace="iwhub_0",
            remappings=tf_remaps,
            arguments=[
                "--x", "0.65", "--y", "0.0", "--z", "0.15",
                "--yaw", "0.0", "--pitch", "0.0", "--roll", "0.0",
                "--frame-id", "iwhub_0/chassis", "--child-frame-id", "iwhub_0/front_2d_lidar",
            ],
            parameters=[{"use_sim_time": use_sim_time}],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="chassis_to_back_2d_lidar",
            namespace="iwhub_0",
            remappings=tf_remaps,
            arguments=[
                "--x", "-0.6", "--y", "0.0", "--z", "0.15",
                "--yaw", "3.141592653589793", "--pitch", "0.0", "--roll", "0.0",
                "--frame-id", "iwhub_0/chassis", "--child-frame-id", "iwhub_0/back_2d_lidar",
            ],
            parameters=[{"use_sim_time": use_sim_time}],
        ),
        Node(
            package="iwhub_control",
            executable="base_node",
            name="iwhub_base_node",
            namespace="iwhub_0",
            remappings=tf_remaps,
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                # iw.hub odom 캘리브레이션값(2026-07-20 calib.py — 실제 주행 vs 바퀴각).
                # 기하학적 bbox(0.0811/0.5792)와 달리 실효 rolling 값이라 odom 이 실제와 일치.
                "wheel_radius": 0.0771,        # [2] 직진 캘리브 (기존 0.0811 → 5% 과대 → 방향 틀어짐)
                "wheel_separation": 0.576,     # [2] 회전 캘리브 실효 트랙폭
                "ns": "iwhub_0",
                "odom_frame": "iwhub_0/odom",
                "base_frame": "iwhub_0/base_link",
                "cmd_vel_topic": "/iwhub_0/cmd_vel",
                "odom_topic": "/iwhub_0/odom",
                "publish_odom": ParameterValue(publish_odom, value_type=bool),
            }],
        ),
    ])
