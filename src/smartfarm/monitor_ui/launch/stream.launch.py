"""4분할 모니터링 화면용 영상 스트리밍 배선.

Isaac이 내보내는 raw RGB를 JPEG로 바꾼 뒤(web_video_server가 raw를 직접
받으면 매 프레임 전체 비트맵이 오간다) MJPEG로 서빙한다.

    isaacpjt/main.py --cctv ...      # Isaac 쪽 고정 카메라 3대 켜기
    ros2 launch monitor_ui stream.launch.py
    → http://localhost:8080 에 /ui/* 네 개가 보이면 배선 성공

Jazzy에서 실행하면 로컬 vision_node의 YOLO 결과를 받고, Humble에서 실행하면
domain_bridge가 109→108로 넘긴 같은 토픽을 받는다.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# qos_bridge → republish 사이의 중간 토픽. /ui/* 는 브라우저가 보는 이름이라
# 여기에 섞지 않는다.
MM_FRONT_RELAY = "/relay/mm_front"

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
    # --moveit 실행이면 harvester_moveit, 기본 통합 실행이면 harvester_0으로 뜬다.
    camera_ns = LaunchConfiguration("camera_ns")
    mm_front_topic = LaunchConfiguration("mm_front_topic")

    return LaunchDescription([
        DeclareLaunchArgument("camera_ns", default_value="harvester_0"),
        DeclareLaunchArgument(
            "mm_front_topic",
            default_value=[camera_ns, "/vision/annotated_image"],
            description=(
                "CAM-01 raw Image 토픽. 기본은 vision_debug_view와 같은 "
                "/<camera_ns>/vision/annotated_image."
            ),
        ),

        # CAM-01은 vision_debug_view와 같은 annotated 토픽을 받는다. 통합
        # 파이프라인은 /harvester_0/vision/annotated_image이고, namespace 없이
        # harvest_full.launch.py를 단독 실행할 때는 /vision/annotated_image다.
        # 후자는 mm_front_topic launch 인자로 명시한다.
        #
        # D455와 vision_node 모두 센서 QoS(BEST_EFFORT)로 발행하지만 republish는
        # RELIABLE 구독이라 직접 연결되지 않는다. qos_bridge가 그 한 홉만 바꾼다.
        Node(
            package="monitor_ui",
            executable="qos_bridge",
            name="qos_bridge_mm_front",
            parameters=[{
                "in_topic": ParameterValue(mm_front_topic, value_type=str),
                # 비전 실행 방식마다 namespace가 달라진다. 사용자가 launch
                # 인자를 몰라도 현재 발행 중인 디버깅 영상을 자동으로 받는다.
                "fallback_topics": [
                    "/vision/annotated_image",
                    "/harvester_0/vision/annotated_image",
                    "/harvester_moveit/vision/annotated_image",
                ],
                "out_topic": MM_FRONT_RELAY,
            }],
            output="screen",
        ),
        _republish("mm_front", MM_FRONT_RELAY),
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
