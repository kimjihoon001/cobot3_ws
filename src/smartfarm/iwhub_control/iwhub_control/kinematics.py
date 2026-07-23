# -*- coding: utf-8 -*-
"""차동구동 기구학 — rclpy 비의존 (§5.6: 판단·수학은 순수 파이썬, 노드는 얇은 래퍼).

iw.hub DOF (2026-07-19 Nucleus 실측): left/right_wheel_joint(속도), lift_joint(위치).
여기는 (v, w) ↔ 바퀴 각속도 변환과 바퀴 각도 적분 오도메트리만 담당한다.
"""
from __future__ import annotations

import math


def twist_to_wheel_speeds(
    v: float, w: float, wheel_radius: float, wheel_separation: float
) -> tuple[float, float]:
    """(선속 v[m/s], 각속 w[rad/s]) → (좌, 우) 바퀴 각속도[rad/s]."""
    left = (v - w * wheel_separation / 2.0) / wheel_radius
    right = (v + w * wheel_separation / 2.0) / wheel_radius
    return left, right


def wheel_speeds_to_twist(
    left: float, right: float, wheel_radius: float, wheel_separation: float
) -> tuple[float, float]:
    """(좌, 우) 바퀴 각속도[rad/s] → (v[m/s], w[rad/s]). twist_to_wheel_speeds 의 역."""
    v = wheel_radius * (left + right) / 2.0
    w = wheel_radius * (right - left) / wheel_separation
    return v, w


def stabilize_twist(
    v: float,
    w: float,
    was_stopped: bool,
    *,
    linear_stop: float,
    angular_stop: float,
    linear_start: float,
    angular_start: float,
) -> tuple[float, float, bool]:
    """정지 근처의 작은 ``cmd_vel`` 왕복을 히스테리시스로 0에 고정한다.

    정지 상태에서는 더 큰 start 문턱을 넘어야 다시 움직이고, 이동 상태에서는
    더 작은 stop 문턱 아래에서만 정지한다. Nav2 목표점 근처에서 부호가 바뀌는
    미세 명령이 좌우 바퀴를 계속 깨워 차체를 떨게 만드는 현상을 막는다.
    """
    values = (v, w, linear_stop, angular_stop, linear_start, angular_start)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("twist와 deadband 값은 모두 유한해야 합니다")
    if min(linear_stop, angular_stop, linear_start, angular_start) < 0.0:
        raise ValueError("deadband 값은 0 이상이어야 합니다")
    if linear_start < linear_stop or angular_start < angular_stop:
        raise ValueError("start deadband는 stop deadband보다 작을 수 없습니다")

    started_from_rest = was_stopped
    if started_from_rest:
        stopped = abs(v) < linear_start and abs(w) < angular_start
    else:
        stopped = abs(v) <= linear_stop and abs(w) <= angular_stop
    if stopped:
        return 0.0, 0.0, True

    # 정지 상태에서 한 축만 start 문턱을 넘었다면 다른 축에 섞인 작은 노이즈는
    # 그 축의 start 문턱으로 제거한다. 예: 제자리 회전 재개 시 0.019m/s의 미세
    # 직진을 함께 살리지 않는다. 이미 이동 중이면 작은 stop 문턱을 사용한다.
    linear_threshold = linear_start if started_from_rest else linear_stop
    angular_threshold = angular_start if started_from_rest else angular_stop
    stable_v = 0.0 if abs(v) < linear_threshold else float(v)
    stable_w = 0.0 if abs(w) < angular_threshold else float(w)
    return stable_v, stable_w, False


def _wrap_angle(a: float) -> float:
    """(-pi, pi] 로 정규화."""
    return math.atan2(math.sin(a), math.cos(a))


class DiffDriveOdometry:
    """바퀴 각도 적분 오도메트리 (엔코더 방식).

    속도 적분 대신 각도 차분을 쓰는 이유: 브리지 joint_states 의 velocity 는
    솔버 순시값이라 노이즈가 있고, 각도 차분은 스텝 누락에도 거리가 보존된다.
    스텝당 바퀴 회전이 반바퀴(pi) 미만이면 각도 랩 여부와 무관하게 정확하다
    (60Hz 시뮬에서 pi/스텝을 넘으려면 바퀴가 ~9400rpm — 비현실).
    """

    def __init__(self, wheel_radius: float, wheel_separation: float):
        self.wheel_radius = wheel_radius
        self.wheel_separation = wheel_separation
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self._prev_left: float | None = None
        self._prev_right: float | None = None

    def reset(self, x: float = 0.0, y: float = 0.0, yaw: float = 0.0) -> None:
        self.x, self.y, self.yaw = x, y, yaw
        self._prev_left = None
        self._prev_right = None

    def update(self, left_angle: float, right_angle: float) -> tuple[float, float, float]:
        """바퀴 절대 각도[rad]를 넣으면 (x, y, yaw) 를 갱신해 반환. 첫 호출은 기준점만 잡는다."""
        if self._prev_left is None:
            self._prev_left, self._prev_right = left_angle, right_angle
            return self.x, self.y, self.yaw

        d_left = _wrap_angle(left_angle - self._prev_left) * self.wheel_radius
        d_right = _wrap_angle(right_angle - self._prev_right) * self.wheel_radius
        self._prev_left, self._prev_right = left_angle, right_angle

        d = (d_left + d_right) / 2.0
        d_yaw = (d_right - d_left) / self.wheel_separation
        # 중점 적분: 이동 방향을 스텝 중간 yaw 로 근사 (오일러보다 원호 오차 작음)
        mid_yaw = self.yaw + d_yaw / 2.0
        self.x += d * math.cos(mid_yaw)
        self.y += d * math.sin(mid_yaw)
        self.yaw = _wrap_angle(self.yaw + d_yaw)
        return self.x, self.y, self.yaw
