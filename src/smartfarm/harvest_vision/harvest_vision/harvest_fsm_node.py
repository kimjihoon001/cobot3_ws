import json

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32, String

from smartfarm_interfaces.msg import TomatoDetectionArray

# 6.4절 확정: 웨이포인트 정지형. 섹터 이동 -> 정지 -> 검출 -> (옵션 필터) -> Pick&Place -> 트레이 적재 -> 다음 섹터
STATE_MOVE_TO_SECTOR = "MOVE_TO_SECTOR"
STATE_DETECT = "DETECT"
STATE_PICK_PLACE = "PICK_PLACE"
STATE_IDLE = "IDLE"

# isaacpjt/README.md 3절 실측: harvester_0 는 Nav2가 아니라 키네마틱 베이스
# (position drive 무시, /harvester_0/cmd 의 JSON "base":[x,y,yaw] 텔레포트만 먹음 — 2026-07-18 실측).
# settings.py SectorConfig docstring은 "MM이 Nav2로 이동"이라고 적혀 있지만
# robot_bridge.py 쪽 실측이 더 최근이라 이 초안은 실측(텔레포트) 쪽을 따름 — TODO: 트랙 B/멘토와 확인 필요.
ARM_JOINTS = (
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
)
GRIPPER_JOINT = "finger_joint"
GRIPPER_OPEN = 0.0
GRIPPER_CLOSED = 0.8
BLADE_OPEN_DEG = 0
BLADE_CUT_DEG = 35


class HarvestFsmNode(Node):
    def __init__(self):
        """수확 State Machine 초안: 섹터 웨이포인트 순회 + Pick&Place 트리거 뼈대.

        /harvester_0/joint_command 로 팔·그리퍼, /harvester_0/cmd 로 가동날·베이스 제어
        (isaacpjt/README.md 3절 실제 브리지 인터페이스). 각도 계산·시퀀싱 자체는 TODO.
        """
        super().__init__("harvest_fsm_node")

        self.declare_parameter("sector_waypoints", [0.0])  # TODO: [x,y,yaw]*N 섹터 좌표 (아직 미확정, settings.py에도 없음)

        self._state = STATE_IDLE
        self._latest_detections: TomatoDetectionArray | None = None
        self._tray_id = 1  # TODO: tray_manager_node와 협의해 트레이 식별 방식 확정

        self.create_subscription(
            TomatoDetectionArray, "/vision/tomato_detections", self._detections_callback, 10
        )

        self._state_pub = self.create_publisher(String, "/harvest/state", 10)
        self._place_request_pub = self.create_publisher(Int32, "/tray/place_request", 10)
        self._joint_cmd_pub = self.create_publisher(JointState, "/harvester_0/joint_command", 10)
        self._cmd_pub = self.create_publisher(String, "/harvester_0/cmd", 10)

        self.get_logger().info("harvest_fsm_node 시작 (초안): 상태 전이 로직은 TODO")

    def _detections_callback(self, msg: TomatoDetectionArray):
        self._latest_detections = msg

    def _publish_state(self, state: str):
        self._state = state
        self._state_pub.publish(String(data=state))

    def _move_to_sector(self, x: float, y: float, yaw: float):
        """TODO: 텔레포트 후 도착 확인(현재는 즉시 STATE_DETECT로 전이할 수밖에 없음 — 피드백 토픽 없음)"""
        self._cmd_pub.publish(String(data=json.dumps({"base": [x, y, yaw]})))

    def _set_arm(self, positions: list[float]):
        """TODO: 실제 목표각(수확 자세 -> 파지점) 계산. 지금은 호출부만 준비"""
        msg = JointState()
        msg.name = list(ARM_JOINTS)
        msg.position = positions
        self._joint_cmd_pub.publish(msg)

    def _set_gripper(self, closed: bool):
        msg = JointState()
        msg.name = [GRIPPER_JOINT]
        msg.position = [GRIPPER_CLOSED if closed else GRIPPER_OPEN]
        self._joint_cmd_pub.publish(msg)

    def _set_blade(self, cut: bool):
        self._cmd_pub.publish(String(data=json.dumps({"blade": BLADE_CUT_DEG if cut else BLADE_OPEN_DEG})))

    def _pick_and_place(self):
        """TODO: 그리퍼 Pick&Place 시퀀스(팔 이동 -> 그리퍼 닫기 -> 가동날 절단 -> 트레이 배치) 실행 후 /tray/place_request 발행"""
        self._place_request_pub.publish(Int32(data=self._tray_id))


def main():
    rclpy.init()
    node = HarvestFsmNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
