"""4분할 모니터링 화면용 영상 스트리밍 배선.

Isaac이 내보내는 raw RGB를 JPEG로 바꾼 뒤(web_video_server가 raw를 직접
받으면 매 프레임 전체 비트맵이 오간다) MJPEG로 서빙한다.

    isaacpjt/main.py --cctv ...      # Isaac 쪽 고정 카메라 3대 켜기
    ros2 launch monitor_ui stream.launch.py
    → http://localhost:8080 에 /ui/* 네 개가 보이면 배선 성공

Isaac과 같은 ROS_DOMAIN_ID에서 실행해야 한다. 창고 자동화는 108을 쓰는데
~/.bashrc 기본값은 109라 그대로 열면 토픽이 하나도 안 보인다.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# (pane 이름, Isaac raw 토픽). MM 전방은 로봇에 달린 D455라 네임스페이스가
# 실행 모드에 따라 달라져서 아래에서 따로 만든다.
FIXED_CAMERAS = (
    ("greenhouse", "/cctv/greenhouse"),
    ("unloading", "/cctv/unloading"),
    ("storage", "/cctv/storage"),
    ("overview", "/cctv/overview"),
)


def _republish(name, in_topic):
    """raw → JPEG 변환 노드 하나.

    베이스 이름 out만 remap하면 안 먹고 그대로 /out으로 발행된다(확인함).
    transport 접미사까지 붙여 out/compressed를 건다.
    """
    return Node(
        package="image_transport",
        executable="republish",
        name=f"republish_{name}",
        arguments=["raw", "compressed"],
        remappings=[
            ("in", in_topic),
            ("out/compressed", f"/ui/{name}/compressed"),
        ],
        output="screen",
    )


def generate_launch_description():
    # --moveit 실행이면 harvester_moveit, RMP 실행이면 harvester_0으로 뜬다.
    camera_ns = LaunchConfiguration("camera_ns")

    return LaunchDescription([
        DeclareLaunchArgument("camera_ns", default_value="harvester_0"),

        # 생 RGB가 아니라 vision_node가 박스를 그려 낸 쪽을 받는다. 1번 화면은
        # 검출을 보여주는 자리인데 /rgb를 받으면 박스 없는 원본만 나온다.
        # 따라서 vision_node가 떠 있어야 이 화면이 나온다(안 뜨면 offline).
        _republish("mm_front", [camera_ns, "/vision/annotated_image"]),
        *(_republish(name, topic) for name, topic in FIXED_CAMERAS),

        # 브라우저가 base 토픽(/ui/<pane>)을 요청하면 압축 transport를
        # 알아서 골라 쓴다. 확인용 토픽 목록은 http://localhost:8080 루트.
        Node(
            package="web_video_server",
            executable="web_video_server",
            name="web_video_server",
            parameters=[{"port": 8080, "address": "0.0.0.0"}],
            output="screen",
        ),
    ])
