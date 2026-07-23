"""Recover Nav2 lifecycle startup when DDS service replies arrive late."""

import time

import rclpy
from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState
from rclpy.node import Node


class Nav2LifecycleActivator(Node):
    """Retry configure/activate until every required Nav2 server is active."""

    TARGETS = (
        "map_server",
        "amcl",
        "controller_server",
        "planner_server",
        "behavior_server",
        "bt_navigator",
        "waypoint_follower",
    )

    def __init__(self) -> None:
        super().__init__("nav2_lifecycle_activator")
        self.declare_parameter("startup_delay_sec", 8.0)
        self.declare_parameter("timeout_sec", 120.0)

    def _state(self, name: str) -> int | None:
        client = self.create_client(GetState, f"{name}/get_state")
        if not client.wait_for_service(timeout_sec=1.0):
            self.destroy_client(client)
            return None
        future = client.call_async(GetState.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        state = (
            int(future.result().current_state.id)
            if future.done() and future.result() is not None
            else None
        )
        self.destroy_client(client)
        return state

    def _transition(self, name: str, transition_id: int) -> None:
        client = self.create_client(ChangeState, f"{name}/change_state")
        if not client.wait_for_service(timeout_sec=1.0):
            self.destroy_client(client)
            return
        request = ChangeState.Request()
        request.transition.id = transition_id
        future = client.call_async(request)
        # Fast-DDS가 응답을 잃어도 서버에서는 전이가 실행될 수 있다. 결과에만
        # 의존하지 않고 다음 반복에서 실제 state를 다시 읽는다.
        rclpy.spin_until_future_complete(self, future, timeout_sec=4.0)
        self.destroy_client(client)

    def run(self) -> bool:
        time.sleep(float(self.get_parameter("startup_delay_sec").value))
        deadline = time.monotonic() + float(
            self.get_parameter("timeout_sec").value)
        last_report = 0.0
        while rclpy.ok() and time.monotonic() < deadline:
            pending = []
            for name in self.TARGETS:
                state = self._state(name)
                if state == State.PRIMARY_STATE_ACTIVE:
                    continue
                pending.append(name)
                if state == State.PRIMARY_STATE_UNCONFIGURED:
                    self._transition(name, Transition.TRANSITION_CONFIGURE)
                    state = self._state(name)
                if state == State.PRIMARY_STATE_INACTIVE:
                    self._transition(name, Transition.TRANSITION_ACTIVATE)
            if not pending:
                self.get_logger().info(
                    "Nav2 핵심 lifecycle 노드 전부 active — 출발 가능")
                return True
            if time.monotonic() - last_report >= 5.0:
                self.get_logger().warn(
                    "Nav2 활성화 재시도 중: " + ", ".join(pending))
                last_report = time.monotonic()
            time.sleep(0.5)
        self.get_logger().error("Nav2 lifecycle 활성화 시간 초과")
        return False


def main() -> None:
    rclpy.init()
    node = Nav2LifecycleActivator()
    try:
        node.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
