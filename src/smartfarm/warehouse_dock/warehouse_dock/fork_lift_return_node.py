#!/usr/bin/env python3
"""IW에서 돌아온 팔레트를 원래 랙에 복귀시키고 다음 빈 팔레트를 상차한다.

초기 ``fork_lift_node``가 Pallet_n을 IW에 올리면 ``/forklift/pallet_on_iw``로
번호를 넘긴다. 이후 IW의 도킹 이벤트를 받을 때마다 다음 사이클을 반복한다.

    IW의 Pallet_n 회수
      -> 랙 n번 슬롯에 복귀
      -> Pallet_(n+1)%6을 랙에서 꺼내 IW에 상차
      -> 다음 IW 귀환 대기

이 노드는 기존 ``ForkLiftNode``의 검증된 0~5번 랙 접근·복귀 경로와 저수준
Step 실행기를 재사용한다. 두 노드는 동시에 실행되지만 BUSY인 노드만
``/forklift_0/joint_command``를 발행한다.
"""

from __future__ import annotations

import math
import time

import rclpy
from std_msgs.msg import Bool, Int32

from warehouse_dock.fork_lift_node import ForkLiftNode, Step, wrap_angle


class ForkLiftReturnNode(ForkLiftNode):
    """가득 찬 팔레트 복귀와 다음 빈 팔레트 상차를 순환 실행한다."""

    INSTANCE_LOCK_PATH = "/tmp/warehouse_dock_fork_lift_return_node.lock"

    PHASE_WAITING = "WAITING_FOR_IW"
    PHASE_RETURNING = "RETURNING_TO_RACK"
    PHASE_LOADING_NEXT = "LOADING_NEXT_PALLET"

    def __init__(self):
        self._return_phase = self.PHASE_WAITING
        self._expected_pallet = 0
        self._next_pallet = 1

        super().__init__(
            node_name="fork_lift_return_node",
            instance_lock_path=self.INSTANCE_LOCK_PATH,
            console_enabled=False,
        )

        self.declare_parameter("initial_pallet", 0)
        initial_pallet = int(self.get_parameter("initial_pallet").value)
        if not 0 <= initial_pallet < self.PALLET_COUNT:
            raise ValueError("initial_pallet은 0부터 5 사이여야 합니다")

        self._expected_pallet = initial_pallet
        self._next_pallet = (initial_pallet + 1) % self.PALLET_COUNT
        # 회수 노드가 움직일 때 팔레트는 IW DeckJoint에 연결돼 있다.
        # 첫 명령에서 이를 실수로 해제하지 않도록 소유 상태를 이어받는다.
        self._pallet_deck_attached_command = True
        self._iw_dock_locked_command = False
        self._pallet_target_command = initial_pallet
        self._mode = self.MODE_WAIT_INITIAL
        self._auto_start_at = None

        self.create_subscription(
            Int32,
            "/forklift/pallet_on_iw",
            self._on_pallet_on_iw,
            10,
        )

        self._publish_status(
            f"회수 노드 준비: IW의 Pallet_{self._expected_pallet:02d} "
            "귀환 신호 대기"
        )

    def _on_pallet_on_iw(self, msg: Int32) -> None:
        pallet = int(msg.data)
        if not 0 <= pallet < self.PALLET_COUNT:
            self.get_logger().warning(
                f"잘못된 /forklift/pallet_on_iw 값 무시: {pallet}"
            )
            return

        # 다음 팔레트 상차 완료 이벤트는 현재 노드가 BUSY인 마지막 구간에도
        # 들어오므로 항상 기억한다. 실제 회수는 다음 도킹 이벤트에서만 시작한다.
        self._expected_pallet = pallet
        self._next_pallet = (pallet + 1) % self.PALLET_COUNT
        self._pallet_deck_attached_command = True
        self._iw_dock_locked_command = False
        self._pallet_target_command = pallet
        self.get_logger().info(
            f"IW 적재 팔레트 확인: Pallet_{pallet:02d}, "
            f"다음 공급 Pallet_{self._next_pallet:02d}"
        )

    def _handle_amr_docked(self) -> None:
        """IW 도착 이벤트를 회수 사이클 시작 신호로 사용한다."""
        if self._mode == self.MODE_BUSY:
            self.get_logger().warning("회수·상차 작업 중이라 도킹 이벤트를 무시합니다")
            return
        if self._mode == self.MODE_ERROR:
            self.get_logger().warning("ERROR 상태입니다. 원인을 해결한 뒤 재시작하세요")
            return
        if self._mode != self.MODE_WAIT_INITIAL:
            self.get_logger().warning(
                f"현재 상태 {self._mode}에서는 도킹 이벤트를 처리할 수 없습니다"
            )
            return

        now = time.monotonic()
        if self._require_joint_state_feedback and (
            self._joint_state_time is None
            or now - self._joint_state_time > self._connection_timeout
        ):
            self.get_logger().warning(
                "ForkliftB joint_states 연결을 확인한 뒤 다시 도킹 신호를 보내세요"
            )
            return
        if self._require_pose_feedback and (
            self._pose_feedback_time is None
            or now - self._pose_feedback_time > self._connection_timeout
        ):
            self.get_logger().warning(
                "ForkliftB pose 연결을 확인한 뒤 다시 도킹 신호를 보내세요"
            )
            return

        position_error = math.hypot(
            self._x - self._wait_pose[0],
            self._y - self._wait_pose[1],
        )
        yaw_error = abs(wrap_angle(self._yaw - self._wait_pose[2]))
        if (
            position_error > self._initial_position_tol
            or yaw_error > self._initial_yaw_tol
        ):
            self._fail(
                "IW 회수 시작 pose가 대기 위치가 아닙니다: "
                f"position_error={position_error:.3f}m, "
                f"yaw_error={math.degrees(yaw_error):.1f}deg"
            )
            return

        pallet = self._expected_pallet
        self._next_pallet = (pallet + 1) % self.PALLET_COUNT
        self._publish_clear(False)
        self._return_phase = self.PHASE_RETURNING
        self._start_queue(
            self._return_pallet_to_slot_steps(pallet),
            result_mode=self.MODE_WAIT_RETURN,
            status=(
                f"Pallet_{pallet:02d} IW 회수·랙 복귀 시작, "
                f"완료 후 Pallet_{self._next_pallet:02d} 상차 예정"
            ),
        )

    def _lift_ramp(
        self,
        start: float,
        target: float,
        label: str,
    ) -> list[Step]:
        """리프트 목표를 최대 2cm 간격으로 나눠 급격한 하중 변화를 막는다."""
        distance = abs(target - start)
        count = max(1, math.ceil(distance / 0.02))
        return [
            self._lift(
                start + (target - start) * index / count,
                f"{label} {index}/{count}",
            )
            for index in range(1, count + 1)
        ]

    def _rack_to_wait_steps(self, pallet: int) -> list[Step]:
        """팔레트별로 검증된 랙→공통 대기 위치 경로를 반환한다."""
        if pallet in (2, 3):
            return self._stable_center_rack_to_wait(pallet)
        if pallet in (4, 5):
            return self._stable_right_rack_to_wait(pallet)
        return self._turn_from_rack_to_wait(pallet) + self._move_wait_steps()

    def _return_pallet_to_slot_steps(self, pallet: int) -> list[Step]:
        """IW의 Pallet_n을 n번 랙 슬롯에 내려놓고 대기점으로 복귀한다."""
        wait_x, wait_y, wait_yaw = self._wait_pose
        _, amr_center_y, _ = self._amr_hole
        amr_insert_y = self._amr_insert_y(amr_center_y)
        pre_y, rack_insert_y, stage_y = self._approach_y(self._rack_front_y)
        rack_x = self.RACK_CENTER_X[pallet]

        amr_lift = self._amr_lift_target()
        amr_carry_lift = amr_lift + self._pickup_raise
        rack_place_lift = max(0.0, self._rack_lift_target(pallet) - 0.06)
        rack_raise = self._rack_pickup_raise(pallet)
        rack_carry_lift = rack_place_lift + rack_raise

        steps: list[Step] = [
            self._dock_lock(
                True, "snap and lock IW at canonical handoff pose"
            ),
            self._pose_check(
                wait_x,
                wait_y,
                wait_yaw,
                f"Pallet_{pallet:02d} straight pickup alignment gate",
                position_tolerance=0.03,
                yaw_tolerance=math.radians(1.0),
            ),
            self._lift(amr_lift, f"IW Pallet_{pallet:02d} hole height"),
            self._straight_y(
                amr_insert_y,
                +self._creep_drive,
                f"IW Pallet_{pallet:02d} fork insert straight",
                expected_x=wait_x,
                expected_yaw=self._amr_heading,
                precise=True,
            ),
            self._wait(0.4, f"IW Pallet_{pallet:02d} insertion settle"),
            self._pallet_owner(
                "fork",
                pallet,
                f"transfer IW Pallet_{pallet:02d} from deck to fork",
            ),
            self._wait(0.4, f"IW Pallet_{pallet:02d} coupler settle"),
        ]
        steps += self._lift_ramp(
            amr_lift,
            amr_carry_lift,
            f"IW Pallet_{pallet:02d} slow raise",
        )
        steps += [
            self._wait(0.8, f"IW Pallet_{pallet:02d} lifted hold"),
            self._straight_y(
                wait_y,
                -self._creep_drive,
                f"IW Pallet_{pallet:02d} reverse to wait pose",
                expected_x=wait_x,
                expected_yaw=self._amr_heading,
                precise=True,
            ),
            self._pose_check(
                wait_x,
                wait_y,
                wait_yaw,
                f"Pallet_{pallet:02d} loaded wait pose check",
            ),
        ]

        # 적재물을 낮은 IW 운반 높이로 유지한 채 검증된 n번 랙 접근 경로를
        # 사용한다. 랙 정면에 정렬된 뒤에만 해당 층의 높이로 조정한다.
        steps += self._turn_from_wait_to_rack(pallet)
        steps += [
            self._approach_pallet(
                rack_x,
                pre_y,
                self._rack_heading,
                f"steer and approach Pallet_{pallet:02d} return slot",
            ),
            self._wait(0.4, f"Pallet_{pallet:02d} return alignment settle"),
        ]
        steps += self._lift_ramp(
            amr_carry_lift,
            rack_carry_lift,
            f"Pallet_{pallet:02d} adjust rack carry height",
        )
        steps += [
            self._wait(0.5, f"Pallet_{pallet:02d} rack height settle"),
            self._straight_y(
                rack_insert_y,
                +self._creep_drive,
                f"rack {pallet} loaded pallet insert straight",
                expected_x=rack_x,
                expected_yaw=self._rack_heading,
                precise=True,
            ),
            self._wait(0.5, f"Pallet_{pallet:02d} rack placement settle"),
        ]
        steps += self._lift_ramp(
            rack_carry_lift,
            rack_place_lift,
            f"Pallet_{pallet:02d} slow lower into rack",
        )
        steps += [
            self._wait(0.8, f"Pallet_{pallet:02d} supported in rack"),
            self._coupler(
                False,
                pallet,
                f"release Pallet_{pallet:02d} in rack slot {pallet}",
            ),
            self._wait(0.8, f"Pallet_{pallet:02d} rack release settle"),
            self._straight_y(
                pre_y,
                -self._creep_drive,
                f"rack {pallet} empty fork retract straight",
                expected_x=rack_x,
                expected_yaw=self._rack_heading,
                precise=True,
            ),
            self._straight_y(
                stage_y,
                -self._creep_drive,
                f"rack {pallet} empty fork reverse to safe staging",
                expected_x=rack_x,
                expected_yaw=self._rack_heading,
                precise=True,
            ),
        ]
        steps += self._lift_ramp(
            rack_place_lift,
            amr_lift,
            f"Pallet_{pallet:02d} lower empty fork for return",
        )
        steps += self._rack_to_wait_steps(pallet)
        steps += [
            self._pose_check(
                wait_x,
                wait_y,
                wait_yaw,
                f"Pallet_{pallet:02d} return mission final wait pose",
            ),
            self._event("task_complete", pallet),
        ]
        return steps

    def _finish_queue(self) -> None:
        """복귀가 끝나면 다음 번호 상차를 시작하고 다시 IW 귀환을 기다린다."""
        if self._return_phase == self.PHASE_RETURNING:
            self._stop()
            next_pallet = self._next_pallet
            self._return_phase = self.PHASE_LOADING_NEXT
            self._start_selected_load(next_pallet)
            return

        if self._return_phase == self.PHASE_LOADING_NEXT:
            super()._finish_queue()
            self._expected_pallet = self._current_pallet
            self._next_pallet = (
                self._expected_pallet + 1
            ) % self.PALLET_COUNT
            self._return_phase = self.PHASE_WAITING
            self._mode = self.MODE_WAIT_INITIAL
            self._publish_status(
                f"Pallet_{self._expected_pallet:02d} IW 상차 완료: "
                "IW 작업·귀환 신호 대기"
            )
            return

        super()._finish_queue()

    def _on_reset(self, msg: Bool) -> None:
        if not msg.data:
            return
        if self._mode == self.MODE_BUSY:
            self.get_logger().warning(
                "팔레트 운반 중에는 회수 노드를 리셋할 수 없습니다"
            )
            return
        self._steps.clear()
        self._return_phase = self.PHASE_WAITING
        self._mode = self.MODE_WAIT_INITIAL
        self._publish_clear(False)
        self._publish_status(
            f"회수 노드 리셋: IW의 Pallet_{self._expected_pallet:02d} 귀환 신호 대기"
        )


def main(args=None):
    rclpy.init(args=args)
    node = ForkLiftReturnNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
