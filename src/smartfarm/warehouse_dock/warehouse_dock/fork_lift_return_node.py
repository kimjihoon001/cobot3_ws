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
from smartfarm_interfaces.srv import DockAdjust, ForkliftCycle
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
        self.declare_parameter("resume_next_pallet", -1)
        self.declare_parameter("resume_handoff_pallet", -1)
        self.declare_parameter("resume_position_only", False)
        # /iwhub_0/deck_geometry가 이미 실제 팔레트 채널 중심을 반영한다.
        # 예전 고정 높이용 -0.056m를 다시 빼면 포크가 홀보다 낮아져 IW
        # 하부를 미므로 추가 오프셋 없이 실측 중심 목표를 그대로 사용한다.
        self.declare_parameter("iw_pickup_lift_offset", 0.0)
        # GUI 상면에서 Pallet_01과 IW 데크가 원하는 모습으로 일치한 실측값.
        # canonical dock root에서 IW 진행축 방향으로 0.111m 이동한 위치다.
        self.declare_parameter("iw_handoff_root_x_offset", 0.111)
        initial_pallet = int(self.get_parameter("initial_pallet").value)
        self._resume_next_pallet = int(
            self.get_parameter("resume_next_pallet").value
        )
        self._resume_handoff_pallet = int(
            self.get_parameter("resume_handoff_pallet").value
        )
        self._resume_position_only = bool(
            self.get_parameter("resume_position_only").value
        )
        self._iw_pickup_lift_offset = float(
            self.get_parameter("iw_pickup_lift_offset").value
        )
        self._iw_handoff_root_x_offset = float(
            self.get_parameter("iw_handoff_root_x_offset").value
        )
        if not 0 <= initial_pallet < self.PALLET_COUNT:
            raise ValueError("initial_pallet은 0부터 5 사이여야 합니다")
        if self._resume_next_pallet not in (-1, *range(self.PALLET_COUNT)):
            raise ValueError("resume_next_pallet은 -1 또는 0부터 5 사이여야 합니다")
        if self._resume_handoff_pallet not in (-1, *range(self.PALLET_COUNT)):
            raise ValueError("resume_handoff_pallet은 -1 또는 0부터 5 사이여야 합니다")
        if self._resume_next_pallet >= 0 and self._resume_handoff_pallet >= 0:
            raise ValueError("두 resume 옵션은 동시에 사용할 수 없습니다")
        if not -0.070 <= self._iw_pickup_lift_offset <= 0.015:
            raise ValueError(
                "iw_pickup_lift_offset은 포크 삽입 여유를 위해 "
                "-0.070~0.015m 사이여야 합니다"
            )

        self._expected_pallet = initial_pallet
        self._next_pallet = (initial_pallet + 1) % self.PALLET_COUNT
        # 일반 시작은 팔레트가 IW에 있고, handoff 재개는 현재 포크에 이미
        # 결속돼 있다. 첫 명령부터 실제 소유 상태를 유지해야 재시작 순간
        # 팔레트가 떨어지지 않는다.
        resume_handoff = self._resume_handoff_pallet >= 0
        self._pallet_attached_command = resume_handoff
        self._pallet_deck_attached_command = not resume_handoff
        self._iw_dock_locked_command = resume_handoff
        self._pallet_target_command = (
            self._resume_handoff_pallet if resume_handoff else initial_pallet
        )
        self._mode = self.MODE_WAIT_INITIAL
        self._auto_start_at = None
        if self._resume_next_pallet >= 0 or self._resume_handoff_pallet >= 0:
            self.create_timer(0.5, self._try_resume_next_pallet)

        self.create_subscription(
            Int32,
            "/forklift/pallet_on_iw",
            self._on_pallet_on_iw,
            10,
        )
        self._cycle_service = self.create_service(
            ForkliftCycle,
            "/forklift/start_cycle",
            self._on_cycle_request,
        )
        self._dock_adjust_client = self.create_client(
            DockAdjust, "/iw/request_dock_adjust"
        )
        self._dock_adjust_attempt = 0
        self._dock_adjust_future = None
        self._dock_adjust_requested_step = -1.0
        self._dock_adjust_complete = False
        self._service_dock_pose = (0.0, self._amr_hole[1], math.pi)
        self.create_subscription(
            Bool, "/iw/dock_adjusted", self._on_iw_dock_adjusted, 10
        )

        self._publish_status(
            f"회수 노드 준비: IW의 Pallet_{self._expected_pallet:02d} "
            "귀환 신호 대기"
        )

    def _try_resume_next_pallet(self) -> None:
        """시연 중 중단된 상단 팔레트 회수부터 현재 물리 장면에서 재개한다."""
        pallet = max(self._resume_next_pallet, self._resume_handoff_pallet)
        if pallet < 0 or self._mode != self.MODE_WAIT_INITIAL:
            return
        now = time.monotonic()
        if (
            self._joint_state_time is None
            or self._pose_feedback_time is None
            or not self._iw_deck_geometry_received
            or now - self._joint_state_time > self._connection_timeout
            or now - self._pose_feedback_time > self._connection_timeout
        ):
            return
        self._resume_next_pallet = -1
        resume_handoff = self._resume_handoff_pallet >= 0
        self._resume_handoff_pallet = -1
        self._next_pallet = pallet
        self._current_pallet = pallet
        self._return_phase = self.PHASE_LOADING_NEXT
        self._pallet_target_command = pallet
        self._pallet_attached_command = resume_handoff
        self._pallet_deck_attached_command = False
        self._iw_dock_locked_command = True
        if resume_handoff:
            if self._resume_position_only:
                steps = [
                    Step(
                        kind="handoff_align",
                        label="manual fixed X/Y handoff position",
                        yaw=self._amr_heading,
                        max_drive=self._creep_drive,
                        position_tolerance=self._insert_tol,
                        yaw_tolerance=self._yaw_tol,
                        timeout=90.0,
                    ),
                    Step(
                        kind="wait",
                        label="manual X/Y position hold",
                        duration=3600.0,
                        timeout=3601.0,
                    ),
                ]
            else:
                # 실패 지점에서 팔레트는 포크에 결속된 채 운반 높이로 정지해
                # 있다. 재그립/후진을 반복하지 않고 그 상태에서 중심 정렬만
                # 곧바로 이어간다.
                carry_lift = self._lift_feedback
                steps = self._place_initial_pallet_on_amr(
                    pallet, carry_lift=carry_lift
                )
        else:
            # 마지막 동작 단독 재생은 지게차가 공통 대기 pose에 있는 새 장면에서
            # 시작한다. 이전 사이클의 랙 앞 pose를 가정하는 연속 작업 경로가 아니라
            # 검증된 일반 선택 경로로 1번 랙까지 처음부터 이동한다.
            self._mode = self.MODE_WAIT_INITIAL
            self._start_selected_load(pallet)
            return
        steps += [
            self._event("loaded_on_amr", pallet),
            self._event("forklift_clear", pallet),
        ]
        self._start_queue(
            steps,
            result_mode=self.MODE_WAIT_RETURN,
            status=(
                f"Pallet_{pallet:02d} IW 중심 재정렬부터 인계 재개"
                if resume_handoff
                else f"Pallet_{pallet:02d} 랙 회수부터 IW 인계 재개"
            ),
        )

    def _on_cycle_request(
        self,
        request: ForkliftCycle.Request,
        response: ForkliftCycle.Response,
    ) -> ForkliftCycle.Response:
        """도크 오케스트레이터의 명시적 서비스 요청으로 교환 사이클을 시작한다."""
        pallet = int(request.inbound_pallet)
        response.outbound_pallet = (pallet + 1) % self.PALLET_COUNT
        if not 0 <= pallet < self.PALLET_COUNT:
            response.accepted = False
            response.message = f"팔레트 번호 범위 오류: {pallet}"
            return response
        dock_pose = (
            float(request.dock_x),
            float(request.dock_y),
            float(request.dock_yaw),
        )
        if not all(math.isfinite(value) for value in dock_pose):
            response.accepted = False
            response.message = "IW dock pose가 유한수가 아닙니다"
            return response
        if self._mode != self.MODE_WAIT_INITIAL:
            response.accepted = False
            response.message = f"지게차가 요청을 받을 수 없는 상태입니다: {self._mode}"
            return response

        self._expected_pallet = pallet
        self._next_pallet = response.outbound_pallet
        # 서비스 pose는 IW 차체의 최종 위치/yaw다. 팔레트 중심은 chassis
        # 실측 X 오프셋까지 포함한 /iwhub_0/deck_geometry 값을 유지한다.
        # canonical IW yaw=pi일 때 포크 heading은 -pi/2가 된다.
        self._amr_hole = (
            self._amr_hole[0],
            self._amr_hole[1],
            self._amr_hole[2],
        )
        self._amr_heading = wrap_angle(dock_pose[2] + math.pi / 2.0)
        self._service_dock_pose = dock_pose
        self.get_logger().info(
            "서비스 IW pose 적용: "
            f"pallet_center=({self._amr_hole[0]:.3f}, "
            f"{self._amr_hole[1]:.3f}), "
            f"IW center=({dock_pose[0]:.3f}, {dock_pose[1]:.3f}), "
            f"IW yaw={math.degrees(dock_pose[2]):.1f}deg, "
            f"fork heading={math.degrees(self._amr_heading):.1f}deg"
        )
        self._pallet_target_command = pallet
        self._pallet_deck_attached_command = True
        self._iw_dock_locked_command = False
        self._handle_amr_docked()
        response.accepted = self._mode == self.MODE_BUSY
        response.message = (
            f"Pallet_{pallet:02d} 랙 복귀 후 "
            f"Pallet_{response.outbound_pallet:02d} IW 상차 접수"
            if response.accepted
            else "실행 전 연결/초기 자세 검증에서 거부됐습니다"
        )
        return response

    def _request_iw_dock_adjust(self, reason: str) -> None:
        """잠금 실패를 IW에 돌려보내 Nav2 재정렬을 요청한다."""
        if not self._dock_adjust_client.service_is_ready():
            super()._fail(
                f"{reason}; IW 도킹 재정렬 서비스가 준비되지 않았습니다"
            )
            return
        self._dock_adjust_attempt += 1
        request = DockAdjust.Request()
        request.attempt = self._dock_adjust_attempt
        request.reason = reason
        request.target_x = self._service_dock_pose[0]
        request.target_y = self._service_dock_pose[1]
        request.target_yaw = self._service_dock_pose[2]
        future = self._dock_adjust_client.call_async(request)
        future.add_done_callback(self._on_dock_adjust_response)
        self._publish_status(
            f"IW 도킹 잠금 실패 → Nav2 재정렬 요청 "
            f"{self._dock_adjust_attempt}회"
        )

    def _on_dock_adjust_response(self, future) -> None:
        try:
            response = future.result()
        except Exception as exc:
            super()._fail(f"IW 도킹 재정렬 서비스 응답 실패: {exc}")
            return
        if response is None or not response.accepted:
            message = "응답 없음" if response is None else response.message
            super()._fail(f"IW 도킹 재정렬 거부: {message}")
            return
        self.get_logger().warning(
            f"{response.message}; 지게차는 재도킹 서비스 요청을 기다립니다"
        )

    def _on_iw_dock_adjusted(self, msg: Bool) -> None:
        self._dock_adjust_complete = bool(msg.data)

    def _run_iw_adjust(self, step: Step, now: float) -> bool:
        """지게차는 고정하고 IW 차체만 팔레트 삽입축 아래로 이동시킨다."""
        self._publish_command(0.0, 0.0)
        if self._handoff_pallet_position is None:
            return False

        if self._dock_adjust_requested_step != self._step_started:
            if not self._dock_adjust_client.service_is_ready():
                return False
            pallet_x = self._handoff_pallet_position[0]
            target_root_x = (
                self._service_dock_pose[0]
                + self._iw_handoff_root_x_offset
            )
            target_root_y = self._service_dock_pose[1]
            target_root_yaw = self._service_dock_pose[2]

            self._dock_adjust_attempt += 1
            request = DockAdjust.Request()
            request.attempt = self._dock_adjust_attempt
            request.reason = (
                f"Pallet_{step.pallet_id:02d} 축 정렬: "
                "지게차 대기 pose 고정, IW만 X/Y/yaw 보정"
            )
            request.target_x = target_root_x
            request.target_y = target_root_y
            request.target_yaw = target_root_yaw
            self._dock_adjust_complete = False
            self._dock_adjust_future = self._dock_adjust_client.call_async(request)
            self._dock_adjust_requested_step = self._step_started
            self._service_dock_pose = (
                target_root_x, target_root_y, target_root_yaw
            )
            # 재도킹 뒤 데크 채널 중심은 현재 팔레트의 고정 X축에 놓인다.
            self._amr_hole = (
                pallet_x, self._amr_hole[1], self._amr_hole[2]
            )
            self._publish_status(
                f"IW 능동 정렬 요청: root_x={target_root_x:.3f}, "
                f"pallet/deck_x={pallet_x:.3f}; 지게차 정지 유지"
            )
            return False

        if self._dock_adjust_future is not None and self._dock_adjust_future.done():
            try:
                response = self._dock_adjust_future.result()
            except Exception as exc:
                self._fail(f"IW 능동 정렬 서비스 실패: {exc}")
                return False
            if response is None or not response.accepted:
                message = "응답 없음" if response is None else response.message
                self._fail(f"IW 능동 정렬 요청 거부: {message}")
                return False
            self._dock_adjust_future = None
        return self._dock_adjust_complete

    def _handoff_prealign_steps(self, pallet: int) -> list[Step]:
        """Pallet_01 최종 인계는 IW가 축을 맞추고 지게차는 직선만 탄다."""
        _, center_y, _ = self._amr_hole
        wait_x, _, _ = self._wait_pose
        return [
            self._dock_lock(False, "unlock IW for pallet-axis adjustment"),
            Step(
                kind="iw_adjust",
                label=f"IW align under Pallet_{pallet:02d} fixed axis",
                pallet_id=pallet,
                timeout=90.0,
            ),
            self._straight_y(
                self._amr_insert_y(center_y),
                +self._creep_drive,
                f"Pallet_{pallet:02d} final handoff straight only",
                expected_x=wait_x,
                expected_yaw=self._amr_heading,
                precise=True,
            ),
        ]

    def _fail(self, reason: str) -> None:
        """IW 정렬 관련 실패는 재도킹 요청으로 되돌리고 나머지는 안전 정지한다."""
        iw_alignment_timeout = (
            "단계 시간 초과: lock IW after verified final dock alignment"
            in reason
            or "단계 시간 초과: transfer IW Pallet_" in reason
        )
        if not iw_alignment_timeout:
            super()._fail(reason)
            return

        self._steps.clear()
        self._mode = self.MODE_WAIT_INITIAL
        self._iw_dock_locked_command = False
        # Isaac 사전 결속 게이트가 불합격이면 DeckJoint를 유지한다. 재정렬 중에도
        # 팔레트가 IW를 따라가도록 명령 상태를 명시적으로 되돌린다.
        self._pallet_attached_command = False
        self._pallet_deck_attached_command = True
        self._publish_command(0.0, 0.0)
        self._publish_clear(False)
        self.get_logger().warning(
            f"{reason} — 팔레트를 IW 데크에 유지하고 Nav2 재정렬 요청"
        )
        self._request_iw_dock_adjust(reason)

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
        direct_upper_pick = (
            pallet % 2 == 0
            and self._next_pallet == pallet + 1
            and self.RACK_CENTER_X[pallet]
            == self.RACK_CENTER_X[self._next_pallet]
        )
        self._return_phase = (
            self.PHASE_LOADING_NEXT
            if direct_upper_pick
            else self.PHASE_RETURNING
        )
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
        """물리 드라이브가 속도를 제한하므로 단일 목표로 연속 승강한다."""
        del start
        return [self._lift(target, label)]

    def _rack_to_wait_steps(self, pallet: int) -> list[Step]:
        """팔레트별로 검증된 랙→공통 대기 위치 경로를 반환한다."""
        if pallet in (2, 3):
            return self._stable_center_rack_to_wait(pallet)
        if pallet in (4, 5):
            return self._stable_right_rack_to_wait(pallet)
        return self._turn_from_rack_to_wait(pallet) + self._move_wait_steps()

    def _safe_iw_pickup_lift_target(self) -> float:
        """GUI 측면에서 실제 채널 진입이 확인된 IW 리프트 목표."""
        return min(
            2.0,
            max(0.0, self._amr_lift_target() + self._iw_pickup_lift_offset),
        )

    def _return_pallet_to_slot_steps(self, pallet: int) -> list[Step]:
        """IW의 Pallet_n을 n번 랙 슬롯에 내려놓고 대기점으로 복귀한다."""
        wait_x, wait_y, wait_yaw = self._wait_pose
        amr_center_x, amr_center_y, _ = self._amr_hole
        amr_insert_y = self._amr_insert_y(amr_center_y)
        # 지게차 대기 차선 X 자체를 고정된 IW Load 중심에 맞춘다. 멀리서
        # 팔레트 상대각을 계산하지 않고, 마지막 1.5m에서만 실제 IW yaw를
        # 사용해 작은 조향으로 근접 정렬한다.
        amr_lane_align_y = wait_y - 1.5
        # IW가 최종 yaw를 약간 보정한 경우 팔레트 중심과 대기점의 월드 X는
        # 같아도 실제 포크 삽입축은 비스듬하다. 멀리서 각을 추정하지 않고
        # 마지막 1.5m 목표점만 서비스로 받은 실제 yaw의 축 위에 놓는다.
        heading_sin = math.sin(self._amr_heading)
        if abs(heading_sin) < 0.5:
            raise ValueError("IW 포크 삽입 heading이 Y축 방향이 아닙니다")
        axis_distance = (
            (amr_center_y - amr_lane_align_y) / heading_sin
        )
        amr_lane_align_x = (
            amr_center_x
            - axis_distance * math.cos(self._amr_heading)
        )
        pre_y, rack_insert_y, stage_y = self._approach_y(self._rack_front_y)
        rack_x = self.RACK_CENTER_X[pallet]

        amr_lift = self._safe_iw_pickup_lift_target()
        amr_carry_lift = amr_lift + self._pickup_raise
        rack_place_lift = max(
            0.0,
            self._rack_lift_target(pallet) - self.RACK_PICKUP_UNDERSHOOT,
        )
        # 선반을 누른 채 Joint를 끊으면 충돌 복원 순간 위로 튄다.
        # 1cm 위에서 해제해 중력으로 짧게 내려앉게 한다.
        rack_release_lift = rack_place_lift + 0.010
        rack_raise = self._rack_pickup_raise(pallet)
        rack_carry_lift = rack_place_lift + rack_raise

        steps: list[Step] = [
            self._dock_lock(
                True, "lock IW after verified final dock alignment"
            ),
            self._pose_check(
                wait_x,
                wait_y,
                wait_yaw,
                f"Pallet_{pallet:02d} straight pickup alignment gate",
                position_tolerance=0.03,
                yaw_tolerance=math.radians(1.0),
            ),
            # 포크 날이 차체 앞쪽으로 길게 돌출되므로 근접 이동 전에
            # 팔레트 채널 높이를 먼저 맞춘다. 낮은 포크로 IW 하부를
            # 밀고 들어가는 순서를 금지한다.
            self._lift(amr_lift, f"IW Pallet_{pallet:02d} hole height"),
            self._lane_align(
                amr_lane_align_x,
                amr_lane_align_y,
                self._amr_heading,
                +self._creep_drive,
                f"Pallet_{pallet:02d} close-range IW pose alignment",
                steering_limit=math.radians(8.0),
                position_tolerance=0.04,
                yaw_tolerance=math.radians(2.0),
                validate_alignment=False,
            ),
            self._straight_y(
                amr_insert_y,
                +self._creep_drive,
                f"IW Pallet_{pallet:02d} fork insert straight",
                expected_x=amr_lane_align_x,
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
                expected_x=amr_center_x,
                expected_yaw=self._amr_heading,
                precise=True,
            ),
            self._move(
                wait_x,
                wait_y,
                wait_yaw,
                f"Pallet_{pallet:02d} return to canonical wait axis",
                creep=True,
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
            rack_release_lift,
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
        ]
        direct_upper_pick = (
            pallet % 2 == 0
            and self._next_pallet == pallet + 1
            and self.RACK_CENTER_X[pallet]
            == self.RACK_CENTER_X[self._next_pallet]
        )
        if direct_upper_pick:
            steps += [self._event("task_complete", pallet)]
            steps += self._take_next_pallet_to_amr(self._next_pallet)
            steps += [
                self._event("loaded_on_amr", self._next_pallet),
                self._event("forklift_clear", self._next_pallet),
            ]
            return steps

        steps += [
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
