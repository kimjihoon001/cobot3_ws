# -*- coding: utf-8 -*-
"""로봇 2대 관절 제어기 — Isaac 안에서 직접 몬다. ROS2 는 나중에 이 set_* 에
값만 흘려보내면 된다 (CLAUDE.md §5.6: ROS2 는 판단, Isaac 은 실행).

harvester.py / transporter.py 는 **놓기만** 한다(그쪽 docstring 참조). 제어는
여기 분리 — 조립과 제어를 섞지 않는다. 관절 이름은 2026-07-18 실측(probe_dof):

  수확 MM (15 DOF): dummy_base_prismatic_x/y + revolute_z (홀로노믹 베이스 3)
                    + UR10e 6 + Robotiq 2F-85 6 (마스터 finger_joint)
  지게차 (7 DOF):   lift_joint + back_wheel_swivel(조향) + back_wheel_drive(구동)
                    + 롤러 4

이름을 추측하지 않는다 — 실측 이름으로만 인덱스를 잡고, 없으면 시끄럽게 실패한다(§8).
"""
from __future__ import annotations

import math

import numpy as np
from isaacsim.core.utils.types import ArticulationAction


class _JointMap:
    """dof 이름 → 인덱스. 요청한 이름이 없으면 즉시 에러(조용한 오작동 금지, §8)."""

    def __init__(self, dof_names: list[str]):
        self._idx = {n: i for i, n in enumerate(dof_names)}
        self.names = list(dof_names)

    def idx(self, *names: str) -> list[int]:
        missing = [n for n in names if n not in self._idx]
        if missing:
            raise KeyError(
                f"관절 없음: {missing}\n  실제 dof: {self.names}\n"
                "  -> 에셋이 바뀌었다. control.py 의 관절 이름을 실측으로 갱신할 것.")
        return [self._idx[n] for n in names]


class HarvesterController:
    """수확 MM 제어. 베이스(평면 XY+회전) + 팔(6) + 그리퍼(개폐).

    전부 위치 타깃(홀로노믹 베이스라 바퀴 컨트롤러가 필요 없다). 매 프레임
    `apply()` 로 반영한다. ROS2 는 set_arm/set_base/set_gripper 만 부르면 된다.
    """

    BASE = ("dummy_base_prismatic_x_joint",
            "dummy_base_prismatic_y_joint",
            "dummy_base_revolute_z_joint")
    ARM = ("shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
           "wrist_1_joint", "wrist_2_joint", "wrist_3_joint")
    GRIPPER_MASTER = "finger_joint"      # 0=열림, ~0.8rad=닫힘 (2F-85 mimic 마스터)
    GRIPPER_CLOSED = 0.80                # rad

    def __init__(self, robot):
        self._robot = robot
        self._jm = _JointMap(list(robot.dof_names))
        self._ctrl = self._jm.idx(*self.BASE, *self.ARM, self.GRIPPER_MASTER)
        # 현재 관절값에서 출발 (튀지 않게)
        q = np.asarray(robot.get_joint_positions(), dtype=float)
        self._t = {i: float(q[i]) for i in self._ctrl}
        self._base_i = self._jm.idx(*self.BASE)
        self._base_last = [self._t[i] for i in self._base_i]

    # ---- 명령 (ROS2 가 부를 지점) ----
    def set_arm(self, q6) -> None:
        for name, v in zip(self.ARM, q6):
            self._t[self._jm.idx(name)[0]] = float(v)

    def move_arm(self, i: int, dq: float) -> None:
        """팔 i번 관절(0~5)을 dq[rad] 만큼 증분. 텔레옵용."""
        self._t[self._jm.idx(self.ARM[i])[0]] += dq

    def set_base(self, x: float, y: float, yaw: float) -> None:
        for name, v in zip(self.BASE, (x, y, yaw)):
            self._t[self._jm.idx(name)[0]] = float(v)

    def move_base(self, dx: float, dy: float, dyaw: float) -> None:
        bx, by, bw = self._jm.idx(*self.BASE)
        self._t[bx] += dx; self._t[by] += dy; self._t[bw] += dyaw

    def move_base_forward(self, distance: float, dyaw: float = 0.0) -> None:
        """현재 MM yaw 기준으로 전후 이동하고 회전한다. 횡방향 게걸음은 허용하지 않는다."""
        bx, by, bw = self._jm.idx(*self.BASE)
        yaw = self._t[bw]
        self._t[bx] += float(distance) * math.cos(yaw)
        self._t[by] += float(distance) * math.sin(yaw)
        self._t[bw] += float(dyaw)

    def set_gripper(self, closed_frac: float) -> None:
        """0=완전 열림, 1=완전 닫힘."""
        f = max(0.0, min(1.0, closed_frac))
        self._t[self._jm.idx(self.GRIPPER_MASTER)[0]] = f * self.GRIPPER_CLOSED

    def move_gripper(self, d: float) -> None:
        i = self._jm.idx(self.GRIPPER_MASTER)[0]
        self._t[i] = max(0.0, min(self.GRIPPER_CLOSED, self._t[i] + d))

    # ---- 반영 ----
    def apply(self) -> None:
        # 팔·그리퍼: 위치 타깃(물리 드라이브가 잘 따라온다).
        idx = sorted(self._t)
        self._robot.apply_action(ArticulationAction(
            joint_positions=np.array([self._t[i] for i in idx]),
            joint_indices=np.array(idx)))
        # 베이스: dummy 홀로노믹 루트 조인트는 위치 타깃을 무시한다(2026-07-18 실측 —
        # 강성 1e6 에도 안 움직임, 텔레포트만 먹음). 이상적 베이스로 보고 상태를 직접
        # 설정하되, **바뀔 때만** 한다 — 매 프레임 하면 같은 아티큘레이션의 팔 드라이브
        # 적분을 방해해 팔이 안 움직인다(실측). 안 움직이는 동안은 팔이 자유롭다.
        base_tgt = [self._t[i] for i in self._base_i]
        if base_tgt != self._base_last:
            self._robot.set_joint_positions(
                np.array(base_tgt), joint_indices=np.array(self._base_i))
            self._base_last = base_tgt

    def arm_positions(self) -> list[float]:
        """현재 팔 6관절 실측값[rad]. 스크립트 모션이 시작 포즈로 삼는다."""
        q = np.asarray(self._robot.get_joint_positions(), dtype=float)
        return [float(q[i]) for i in self._jm.idx(*self.ARM)]

    def joint_report(self) -> str:
        q = np.asarray(self._robot.get_joint_positions(), dtype=float)
        arm = ", ".join(f"{np.degrees(q[i]):.0f}" for i in self._jm.idx(*self.ARM))
        return f"arm(deg)=[{arm}]  gripper={q[self._jm.idx(self.GRIPPER_MASTER)[0]]:.2f}"


class TransporterController:
    """지게차 제어. 포크 승강(위치) + 주행(구동 바퀴 속도) + 조향(위치).

    ForkliftB/C 겸용 — 구동계가 달라서 관절 이름으로 자동 감지한다(2026-07-18 실측):
      B: 구동 back_wheel_drive(1) / 조향 back_wheel_swivel(1)
      C: 구동 뒷바퀴 2개(damping 있는 속도드라이브) / 조향 로테이터 2개(위치드라이브)
    승강·조향은 위치 타깃, 구동은 속도 타깃으로 나눠 건다(한 액션에 pos/vel 혼용 불가).
    """

    LIFT = "lift_joint"
    # (구동 관절들, 조향 관절들) — 이름으로 감지
    _B = (("back_wheel_drive",), ("back_wheel_swivel",))
    _C = (("left_back_wheel_joint", "right_back_wheel_joint"),
          ("left_rotator_joint", "right_rotator_joint"))

    def __init__(self, robot):
        self._robot = robot
        self._jm = _JointMap(list(robot.dof_names))
        names = self._jm.names
        if "back_wheel_drive" in names:
            drive, steer = self._B; self.kind = "ForkliftB"
        elif "left_back_wheel_joint" in names:
            drive, steer = self._C; self.kind = "ForkliftC"
        else:
            raise KeyError(f"알 수 없는 지게차 관절 구성: {names}\n"
                           "  -> control.py TransporterController 에 구동/조향 매핑 추가.")
        self._lift_i = self._jm.idx(self.LIFT)[0]
        self._drive_i = self._jm.idx(*drive)
        self._steer_i = self._jm.idx(*steer)
        q = np.asarray(robot.get_joint_positions(), dtype=float)
        self._lift = float(q[self._lift_i])
        self._steer = float(q[self._steer_i[0]])
        self._drive_vel = 0.0

    # ---- 명령 ----
    def set_fork(self, height: float) -> None:
        self._lift = max(0.0, height)

    def move_fork(self, dh: float) -> None:
        self._lift = max(0.0, self._lift + dh)

    def set_steer(self, angle: float) -> None:
        self._steer = angle

    def move_steer(self, da: float) -> None:
        self._steer += da

    def set_drive(self, vel: float) -> None:
        """구동 바퀴 각속도[rad/s]. 0 이면 정지."""
        self._drive_vel = vel

    # ---- 반영 ----
    def apply(self) -> None:
        # 위치(승강 1 + 조향 N)와 속도(구동 N)를 각각 건다. DC 의 pos-target·
        # vel-target 버퍼가 달라 두 액션이 공존한다.
        pos_i = [self._lift_i] + self._steer_i
        pos_v = [self._lift] + [self._steer] * len(self._steer_i)
        self._robot.apply_action(ArticulationAction(
            joint_positions=np.array(pos_v), joint_indices=np.array(pos_i)))
        self._robot.apply_action(ArticulationAction(
            joint_velocities=np.full(len(self._drive_i), self._drive_vel),
            joint_indices=np.array(self._drive_i)))

    def joint_report(self) -> str:
        q = np.asarray(self._robot.get_joint_positions(), dtype=float)
        return (f"{self.kind} fork={q[self._lift_i]:.3f}m "
                f"steer={np.degrees(q[self._steer_i[0]]):.0f}deg "
                f"drive_vel={self._drive_vel:.1f}")
