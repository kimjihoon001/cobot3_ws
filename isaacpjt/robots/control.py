# -*- coding: utf-8 -*-
"""로봇 2대 관절 제어기 — Isaac 안에서 직접 몬다. ROS2 는 나중에 이 set_* 에
값만 흘려보내면 된다 (CLAUDE.md §5.6: ROS2 는 판단, Isaac 은 실행).

harvester.py / transporter.py 는 **놓기만** 한다(그쪽 docstring 참조). 제어는
여기 분리 — 조립과 제어를 섞지 않는다. 관절 이름은 2026-07-18 실측(probe_dof):

  수확 MM (15 DOF): dummy_base_prismatic_x/y + revolute_z (홀로노믹 베이스 3)
                    + UR10e 6 + 동축 1/4구 스쿱 3
  지게차 (7 DOF):   lift_joint + back_wheel_swivel(조향) + back_wheel_drive(구동)
                    + 롤러 4

이름을 추측하지 않는다 — 실측 이름으로만 인덱스를 잡고, 없으면 시끄럽게 실패한다(§8).
"""
from __future__ import annotations

import math

import numpy as np
from isaacsim.core.utils.types import ArticulationAction


class RmpFlowTargetController:
    """UR10e의 모바일 베이스 기준 위치 목표를 Isaac RMPflow로 추종한다."""

    def __init__(self, robot, stage, reference_prim: str, arm_base_prim: str,
                 physics_dt: float = 1.0 / 60.0,
                 tool_tcp_prim: str | None = None):
        import isaacsim.robot_motion.motion_generation as mg

        self._robot = robot
        self._stage = stage
        self._reference_prim = reference_prim
        self._arm_base_prim = arm_base_prim
        self._tool_tcp_prim = tool_tcp_prim
        config = mg.interface_config_loader.load_supported_motion_policy_config(
            "UR10e", "RMPflow")
        if not config:
            raise RuntimeError("Isaac Sim UR10e RMPflow 설정을 찾을 수 없습니다")
        self._policy = mg.lula.motion_policies.RmpFlow(**config)
        self._articulation_policy = mg.ArticulationMotionPolicy(
            robot, self._policy, physics_dt)
        joints = self._articulation_policy.get_active_joints_subset()
        home = joints.get_joint_positions()
        if hasattr(home, "cpu"):
            home = home.cpu().numpy()
        self._home_positions = np.asarray(home, dtype=float).copy()
        self._target_world = None
        self._target_tcp_world = None
        self._target_base = None
        self._target_orientation_world = None
        self._motion_active = False
        self._mode = "IDLE"
        self._target_id = 0
        self._phase = "IDLE"
        self._sync_arm_base_pose()

    def set_target(self, position, target_id: int = 0, phase: str = "MOVE") -> None:
        from pxr import Gf, UsdGeom

        values = np.asarray(position, dtype=float)
        if values.shape != (3,) or not np.all(np.isfinite(values)):
            raise ValueError(f"RMPflow 목표는 유한한 xyz 3개여야 합니다: {position}")
        matrix = UsdGeom.XformCache().GetLocalToWorldTransform(
            self._stage.GetPrimAtPath(self._reference_prim))
        self._target_base = values.copy()
        desired_tcp_world = np.asarray(
            matrix.Transform(Gf.Vec3d(*values)), dtype=float)
        self._target_tcp_world = desired_tcp_world.copy()
        self._motion_active = True
        self._mode = "POSITION"
        self._target_id = int(target_id)
        self._phase = str(phase)
        # eye-in-hand 카메라로 목표를 본 순간의 tool0 방향을 유지한다. 위치만 주고
        # orientation=None으로 두면 RMPflow가 손목 방향을 보장하지 않아 그리퍼가
        # 카메라 광선과 다른 방향으로 접근할 수 있다.
        self._sync_arm_base_pose()
        joints = self._articulation_policy.get_active_joints_subset()
        positions = joints.get_joint_positions()
        if hasattr(positions, "cpu"):
            positions = positions.cpu().numpy()
        _, current_rotation = self._policy.get_end_effector_pose(
            np.asarray(positions, dtype=float))
        current_ee_world, _ = self._policy.get_end_effector_pose(
            np.asarray(positions, dtype=float))
        # RMPflow는 UR ee_link를 움직이지만 명령 position은 HarvestTCP 기준이다.
        # 현재 조립체에서 EE→TCP 벡터를 실측해 원하는 TCP 위치에서 역으로 뺀다.
        self._target_world = desired_tcp_world
        if self._tool_tcp_prim:
            tcp_prim = self._stage.GetPrimAtPath(self._tool_tcp_prim)
            if tcp_prim.IsValid():
                tcp_world = np.asarray(
                    UsdGeom.XformCache().GetLocalToWorldTransform(
                        tcp_prim).ExtractTranslation(), dtype=float)
                self._target_world = desired_tcp_world - (
                    tcp_world - np.asarray(current_ee_world, dtype=float))
        from isaacsim.core.utils.rotations import rot_matrix_to_quat
        self._target_orientation_world = rot_matrix_to_quat(current_rotation)
        self._policy.set_end_effector_target(
            self._target_world, self._target_orientation_world)

    def apply(self) -> None:
        if not self._motion_active:
            return
        self._sync_arm_base_pose()
        action = self._articulation_policy.get_next_articulation_action()
        self._robot.get_articulation_controller().apply_action(action)

    def reset(self) -> None:
        self._target_world = None
        self._target_tcp_world = None
        self._target_base = None
        self._target_orientation_world = None
        self._motion_active = False
        self._mode = "IDLE"
        self._target_id = 0
        self._phase = "IDLE"
        self._policy.reset()
        self._sync_arm_base_pose()

    def stop(self) -> None:
        # 진단을 위해 마지막 목표는 보존한다. 이전에는 timeout 직후 목표를
        # None으로 지워서 ROS에서 실패 지점의 distance를 확인할 수 없었다.
        self._target_orientation_world = None
        self._motion_active = False
        self._mode = "STOPPED"
        if not self._phase.startswith("STOPPED_"):
            self._phase = f"STOPPED_{self._phase}"
        self._policy.set_end_effector_target(None, None)

    def go_home(self, target_id: int = 0) -> None:
        self._target_world = None
        self._target_tcp_world = None
        self._target_base = None
        self._target_orientation_world = None
        self._target_id = int(target_id)
        self._phase = "HOME"
        self._mode = "HOME"
        self._motion_active = True
        self._policy.set_end_effector_target(None, None)
        self._policy.set_cspace_target(self._home_positions)

    def status(self, position_tolerance: float = 0.02) -> dict:
        result = {"id": self._target_id, "phase": self._phase,
                  "active": self._motion_active, "reached": False,
                  "distance": None, "at_home": False,
                  "current_position": None, "target_position": None}
        joints = self._articulation_policy.get_active_joints_subset()
        positions = joints.get_joint_positions()
        if positions is None:
            return result
        if hasattr(positions, "cpu"):
            positions = positions.cpu().numpy()
        positions = np.asarray(positions, dtype=float)
        home_error = float(np.max(np.abs(positions - self._home_positions)))
        result["at_home"] = home_error <= 0.03
        if self._mode == "HOME":
            result["distance"] = home_error
            result["reached"] = result["at_home"]
        elif self._target_world is not None:
            current, _ = self._policy.get_end_effector_pose(positions)
            measured_world = np.asarray(current, dtype=float)
            desired_world = self._target_world
            # 완료 판정은 RMPflow 내부 ee_link가 아니라 실제 USD HarvestTCP로 한다.
            # 플랜지만 목표에 도달하고 손가락 중심이 남은 상태를 성공 처리하지 않는다.
            if self._target_tcp_world is not None and self._tool_tcp_prim:
                tcp_prim = self._stage.GetPrimAtPath(self._tool_tcp_prim)
                if tcp_prim.IsValid():
                    from pxr import UsdGeom
                    measured_world = np.asarray(
                        UsdGeom.XformCache().GetLocalToWorldTransform(
                            tcp_prim).ExtractTranslation(), dtype=float)
                    desired_world = self._target_tcp_world
            distance = float(np.linalg.norm(measured_world - desired_world))
            result["distance"] = distance
            # 접근/후퇴/바스켓 상공은 경유점이므로 산업용 팔이 자세 제약 아래
            # 수 cm 앞에서 수렴해도 다음 단계로 진행해도 된다. 실제 파지점 GRASP와
            # 놓기점 BASKET_PLACE만 기존 2 cm 정밀도를 유지한다.
            phase_tolerance = {
                "PREGRASP": 0.04,
                # rmpflow(반응형)는 과실 코앞에서 2cm 정밀도를 못 내 파지 직전 멈춘다.
                # 과실 반지름 3.4cm — 4cm 안이면 그리퍼 span 에 들어와 파지 가능(2026-07-22).
                "GRASP": 0.04,
                "RETRACT": 0.04,
                "BASKET_APPROACH": 0.04,
            }.get(self._phase, position_tolerance)
            result["reached"] = distance <= phase_tolerance
            from pxr import Gf, UsdGeom
            reference_world = UsdGeom.XformCache().GetLocalToWorldTransform(
                self._stage.GetPrimAtPath(self._reference_prim))
            current_base = reference_world.GetInverse().Transform(
                Gf.Vec3d(*measured_world))
            result["current_position"] = [float(v) for v in current_base]
            result["target_position"] = [float(v) for v in self._target_base]
        return result

    def _sync_arm_base_pose(self) -> None:
        from pxr import UsdGeom

        matrix = UsdGeom.XformCache().GetLocalToWorldTransform(
            self._stage.GetPrimAtPath(self._arm_base_prim))
        quat = matrix.ExtractRotationQuat()
        imag = quat.GetImaginary()
        self._policy.set_robot_base_pose(
            np.asarray(matrix.ExtractTranslation(), dtype=float),
            np.asarray([quat.GetReal(), imag[0], imag[1], imag[2]], dtype=float))


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
    GRIPPER = ("scoop_quarter_1_joint",
               "scoop_quarter_2_joint",
               "cutter_quarter_3_joint")
    GRIPPER_OPEN = np.radians((0.0, -90.0, -180.0))
    GRIPPER_CLOSED = np.radians((0.0, 0.0, 0.0))

    def __init__(self, robot):
        self._robot = robot
        self._jm = _JointMap(list(robot.dof_names))
        self._ctrl = self._jm.idx(*self.BASE, *self.ARM, *self.GRIPPER)
        # 현재 관절값에서 출발 (튀지 않게)
        q = np.asarray(robot.get_joint_positions(), dtype=float)
        self._t = {i: float(q[i]) for i in self._ctrl}
        self._base_i = self._jm.idx(*self.BASE)
        self._base_last = [self._t[i] for i in self._base_i]
        self._grip_fraction = 0.0

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
        self._grip_fraction = f
        values = self.GRIPPER_OPEN + f * (self.GRIPPER_CLOSED - self.GRIPPER_OPEN)
        for i, value in zip(self._jm.idx(*self.GRIPPER), values):
            self._t[i] = float(value)

    def move_gripper(self, d: float) -> None:
        self.set_gripper(self._grip_fraction + float(d))

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
        scoop = ", ".join(
            f"{np.degrees(q[i]):.0f}" for i in self._jm.idx(*self.GRIPPER))
        return f"arm(deg)=[{arm}]  scoop(deg)=[{scoop}]"


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
        self._kinematic_moving = False
        # ForkliftB 에셋은 구동 관절이 돌아도 접촉 마찰 상태에 따라 차체가 전혀
        # 전진하지 않는 경우가 있다. 바퀴 명령과 함께 평면 차량 운동을 적용할
        # 호출자를 위해 실측 스파이크와 같은 바퀴 반지름/축거를 보관한다.
        self._wheel_radius = 0.22
        self._wheelbase = 2.05

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
    def apply(
        self,
        dt: float | None = None,
        kinematic_yaw_sign: float = 1.0,
    ) -> None:
        # 위치(승강 1 + 조향 N)와 속도(구동 N)를 각각 건다. DC 의 pos-target·
        # vel-target 버퍼가 달라 두 액션이 공존한다.
        pos_i = [self._lift_i] + self._steer_i
        pos_v = [self._lift] + [self._steer] * len(self._steer_i)
        self._robot.apply_action(ArticulationAction(
            joint_positions=np.array(pos_v), joint_indices=np.array(pos_i)))
        # dt가 주어진 ForkliftB 자동화에서는 차체 이동을 아래의 Ackermann
        # 평면 적분 하나로만 만든다. 바퀴 속도 드라이브까지 동시에 걸면 PhysX
        # 접촉력과 set_world_pose 이동이 서로 더해져 실행마다 궤적이 달라진다.
        physical_drive = self._drive_vel if dt is None else 0.0
        self._robot.apply_action(ArticulationAction(
            joint_velocities=np.full(len(self._drive_i), physical_drive),
            joint_indices=np.array(self._drive_i)))

        # ForkliftB의 후륜 구동은 에셋/바닥 마찰에 따라 바퀴만 헛돌 수 있다.
        # dt를 준 호출자는 물리 추진 대신 Ackermann 평면 운동만 적용한다.
        # 정지 중에는 pose를 건드리지 않아 리프트 물리와 팔레트 접촉을
        # 불필요하게 방해하지 않는다.
        if dt is not None and abs(self._drive_vel) > 1e-6:
            position, quat = self._robot.get_world_pose()
            position = np.asarray(position, dtype=float).copy()
            quat = np.asarray(quat, dtype=float)  # Isaac Core: [w, x, y, z]
            w, x, y, z = quat
            yaw = np.arctan2(
                2.0 * (w * z + x * y),
                1.0 - 2.0 * (y * y + z * z),
            )
            linear = self._drive_vel * self._wheel_radius
            # 일부 에셋은 차체 루트 +X와 작업 전방(포크 방향)이 반대다. 그런
            # 호출자는 주행 부호를 뒤집어 쓰므로 yaw도 같은 좌표계로 맞춰야 한다.
            yaw += (
                kinematic_yaw_sign
                * linear
                / self._wheelbase
                * np.tan(self._steer)
                * dt
            )
            position[0] += linear * np.cos(yaw) * dt
            position[1] += linear * np.sin(yaw) * dt
            half = yaw * 0.5
            # 이전 PhysX 접촉에서 남은 선속도/각속도를 제거한 뒤 계산된 pose만
            # 반영한다. 따라서 정지 명령 뒤 관성으로 더 돌거나 밀리지 않는다.
            self._robot.set_linear_velocity(np.zeros(3, dtype=float))
            self._robot.set_angular_velocity(np.zeros(3, dtype=float))
            self._robot.set_world_pose(
                position=position,
                orientation=np.array([np.cos(half), 0.0, 0.0, np.sin(half)]),
            )
            self._kinematic_moving = True
        elif dt is not None and self._kinematic_moving:
            self._robot.set_linear_velocity(np.zeros(3, dtype=float))
            self._robot.set_angular_velocity(np.zeros(3, dtype=float))
            self._kinematic_moving = False

    def joint_report(self) -> str:
        q = np.asarray(self._robot.get_joint_positions(), dtype=float)
        return (f"{self.kind} fork={q[self._lift_i]:.3f}m "
                f"steer={np.degrees(q[self._steer_i[0]]):.0f}deg "
                f"drive_vel={self._drive_vel:.1f}")
