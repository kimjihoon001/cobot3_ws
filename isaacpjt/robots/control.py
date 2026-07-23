# -*- coding: utf-8 -*-
"""로봇 2대 관절 제어기 — Isaac 안에서 직접 몬다. ROS2 는 나중에 이 set_* 에
값만 흘려보내면 된다 (CLAUDE.md §5.6: ROS2 는 판단, Isaac 은 실행).

harvester.py / transporter.py 는 **놓기만** 한다(그쪽 docstring 참조). 제어는
여기 분리 — 조립과 제어를 섞지 않는다. 관절 이름은 2026-07-18 실측(probe_dof):

  수확 MM: dummy_base_prismatic_x/y + revolute_z (홀로노믹 베이스 3)
           + m0617 6 + OnRobot RG2 (마스터 finger_joint와 mimic 관절)
  지게차 (7 DOF):   lift_joint + back_wheel_swivel(조향) + back_wheel_drive(구동)
                    + 롤러 4

이름을 추측하지 않는다 — 실측 이름으로만 인덱스를 잡고, 없으면 시끄럽게 실패한다(§8).
"""
from __future__ import annotations

import math

import numpy as np
from isaacsim.core.utils.types import ArticulationAction

# 파지(흡착) 후 플레이스 전 자세(2026-07-23 사용자): 홈에서 joint_3·joint_5 만 조절해
# 그리퍼를 지면과 수직으로 세우고 그리퍼 길이 절반쯤 들어올린다.
# [4] 임의 — 시뮬에서 맞춰야 하는 값(홈: joint_3=60°, joint_5=75°). 절대각(deg).
PREPLACE_JOINT3_DEG = 80.0   # 홈 60→40: 전완을 펴 TCP를 살짝 들어올린다(그리퍼 길이 절반쯤)
PREPLACE_JOINT5_DEG = 120.0   # 그리퍼(link_6 +Z)를 지면과 수직(하향)으로 세운다


class LegacyIkTargetController:
    """레거시 m0617 Cartesian waypoint + Lula IK 관절 위치 제어기.

    RMPflow(반응형)가 특이점 근처에서 발산해(2026-07-22: joint_1 −153°로 튐, 복구 불가)
    목표까지의 최종 관절해를 한 번에 보간하지 않는다. 실제 USD link_6에서 목표 방향으로
    짧은 Cartesian 중간점을 만들고, 매 프레임 현재 관절각을 seed로 Lula IK를 다시 푼다.
    따라서 관절공간 직선 때문에 TCP가 과실 앞에서 다시 멀어지는 경로를 만들지 않는다.
    """

    def __init__(self, robot, stage, reference_prim: str, arm_base_prim: str,
                 physics_dt: float = 1.0 / 60.0,
                 tool_tcp_prim: str | None = None,
                 home_positions=None):
        import isaacsim.robot_motion.motion_generation as mg
        from pathlib import Path

        self._robot = robot
        self._stage = stage
        self._reference_prim = reference_prim
        self._arm_base_prim = arm_base_prim
        self._tool_tcp_prim = tool_tcp_prim
        self._ee_frame = "link_6"
        # link_6 prim: arm_base_prim(.../Arm/base_link) → .../Arm/link_6
        self._link6_prim = arm_base_prim.rsplit("/base_link", 1)[0] + "/link_6"
        cfg_dir = Path(__file__).resolve().parent / "m0617"
        self._ik = mg.LulaKinematicsSolver(
            robot_description_path=str(cfg_dir / "m0617_description.yaml"),
            urdf_path=str(cfg_dir / "m0617.urdf"))
        names = list(robot.dof_names)
        self._arm_dof = np.array([names.index(f"joint_{i}") for i in range(1, 7)])
        self._home_positions = (self._arm_q().copy() if home_positions is None
                                else np.asarray(home_positions, dtype=float).copy())
        if self._home_positions.shape != (6,):
            raise ValueError("home_positions는 팔 6축 값이어야 합니다")
        # 중간점 IK가 내놓은 해를 한 프레임에 너무 크게 적용하지 않는다.
        # 먼 PREGRASP는 약 1m 이동할 수 있다. 사용자 요청(2026-07-23)으로 원거리
        # 먼 PREGRASP의 관절 속도 제한. 마지막 8cm와 GRASP는 아래에서 별도로
        # 원래 저속의 3배를 적용한다.
        self._max_joint_step = 0.060
        self._joint_lower = np.array(
            [-2 * math.pi, -2 * math.pi, -2.8798,
             -2 * math.pi, -2 * math.pi, -2 * math.pi])
        self._joint_upper = np.array(
            [2 * math.pi, 2 * math.pi, 2.8798,
             2 * math.pi, 2 * math.pi, 2 * math.pi])
        # HarvestTCP 위치를 link_6 로컬로 실측(그리퍼 웰드라 불변). 목표 = fruit − R·offset.
        self._tcp_offset_l6 = self._measure_tcp_offset()
        self._solution = self._home_positions.copy()
        # 실제 관절값을 매번 기준으로 증분 목표를 만들지 않고, 별도의 명령 궤적을 적분한다.
        # 그래야 높은 drive gain에서도 HOME setpoint가 순간이동하거나 물리 반동을 쫓지 않는다.
        self._command_q = self._arm_q().copy()
        self._target_orientation_world = None
        self._target_world = None        # link_6 목표 위치(world) — status 참고
        self._target_tcp_world = None    # 과실 위치(world)
        self._target_base = None
        self._motion_active = False
        self._mode = "IDLE"
        self._target_id = 0
        self._phase = "IDLE"
        self._ik_fail_count = 0
        self._cart_step_scale = 1.0
        self._replan_hold_frames = 0
        self._divergence_replans = 0
        self._best_tcp_distance = None
        self._last_tcp_distance = None
        self._distance_increase_frames = 0
        # 최종 GRASP에서 한 방향만 고집하면 Lula가 유효한 해를 반환하면서도 실제 TCP가
        # 더 이상 전진하지 않는 국소 정체가 생긴다. TCP 중심은 그대로 둔 채 손목을
        # 소폭 기울인 후보들을 보관하고, 정체 시 다음 후보로 바꾼다.
        self._pose_candidates = []
        self._pose_candidate_index = 0
        self._stagnation_best = None
        self._stagnation_frames = 0
        self._ik_step_count = 0
        self._jacobian_supported = True
        self._sync_arm_base_pose()

    def _arm_q(self) -> np.ndarray:
        q = self._robot.get_joint_positions()
        if hasattr(q, "cpu"):
            q = q.cpu().numpy()
        return np.asarray(q, dtype=float)[self._arm_dof]

    def _measure_tcp_offset(self) -> np.ndarray:
        """HarvestTCP 위치를 link_6 로컬 좌표로 (그리퍼 웰드라 상수). 없으면 0."""
        from pxr import UsdGeom
        if not self._tool_tcp_prim:
            return np.zeros(3)
        cache = UsdGeom.XformCache()
        l6 = self._stage.GetPrimAtPath(self._link6_prim)
        tcp = self._stage.GetPrimAtPath(self._tool_tcp_prim)
        if not (l6.IsValid() and tcp.IsValid()):
            return np.zeros(3)
        rel = cache.GetLocalToWorldTransform(tcp) * \
            cache.GetLocalToWorldTransform(l6).GetInverse()
        return np.asarray(rel.ExtractTranslation(), dtype=float)

    def fk_consistency(self) -> dict:
        """같은 실제 관절각에서 Lula FK와 USD link_6 월드 위치 오차를 반환한다."""
        from pxr import UsdGeom

        self._sync_arm_base_pose()
        q = self._arm_q()
        usd = np.asarray(
            UsdGeom.XformCache().GetLocalToWorldTransform(
                self._stage.GetPrimAtPath(self._link6_prim)).ExtractTranslation(),
            dtype=float)
        try:
            lula, _ = self._ik.compute_forward_kinematics(self._ee_frame, q)
            lula = np.asarray(lula, dtype=float)
            delta = usd - lula
            error = float(np.linalg.norm(delta))
            print(f"[FKCHK] q={np.degrees(q).round(2).tolist()} "
                  f"usd={usd.round(4).tolist()} lula={lula.round(4).tolist()} "
                  f"delta={delta.round(4).tolist()} error={error:.4f}m")
            return {"ok": True, "error": error, "delta": delta.tolist()}
        except Exception as exc:
            print(f"[FKCHK] Lula FK 실패: {exc}")
            return {"ok": False, "error": 999.0, "delta": [999.0] * 3}

    def _normalise_ik_solution(self, solution, current,
                               reject_branch_jump: bool = True):
        """각 축의 ±2π 동치해 중 현재 자세와 가장 가까운 유효 해를 고른다."""
        solution = np.asarray(solution, dtype=float)
        current = np.asarray(current, dtype=float)
        if solution.shape != (6,) or not np.all(np.isfinite(solution)):
            return None
        chosen = np.empty(6, dtype=float)
        for index, value in enumerate(solution):
            candidates = value + 2.0 * math.pi * np.arange(-2, 3)
            valid = candidates[
                (candidates >= self._joint_lower[index])
                & (candidates <= self._joint_upper[index])]
            if not len(valid):
                return None
            chosen[index] = valid[np.argmin(np.abs(valid - current[index]))]
        delta = np.abs(chosen - current)
        # IK 해의 크게 바뀌는 관절은 branch 변경일 수 있지만, 실제 적용량은
        # max_joint_step(원거리 0.060rad)으로 잘린다. J2는 상완을 수직에서
        # 작업 방향으로 크게 펼 수 있게 branch 변화량 제한에서 제외한다.
        # 최종 GRASP에서 팔이 거의 일자가 되면 특이점 근처의 연속 IK도 관절값 변화가
        # 커질 수 있다. 실제 명령은 프레임당 0.009rad로 제한되므로 GRASP에서만 해
        # 연속성 문턱을 넓혀 더 펴지는 해를 허용한다.
        proximal_limit = 0.50 if self._phase == "GRASP" else 0.35
        shoulder_pan_limit = 0.80
        wrist_limit = 0.65
        # GRASP 대체 자세의 yaw 변화는 주로 J6에 나타난다. 실제 명령은 아래의
        # 0.009rad/frame 제한으로 천천히 적용되므로, J4/J5의 branch 보호는 유지하고
        # J6만 45도까지 허용한다. 기존 0.65rad(37.2도)는 정상 후보 38.3도를 경계에서
        # 반복 거부해 TCP 8.3cm 앞에서 ERROR_IK_PATH를 만들었다.
        wrist_roll_limit = math.radians(45.0) if self._phase == "GRASP" else wrist_limit
        proximal_jump = (delta[0] > shoulder_pan_limit
                         or delta[2] > proximal_limit)
        if reject_branch_jump and (
                proximal_jump
                or np.max(delta[3:5]) > wrist_limit
                or delta[5] > wrist_roll_limit):
            print(f"[IK] branch jump 거부: deg={np.degrees(delta).round(1).tolist()}")
            return None
        margin = float(np.min(np.minimum(
            chosen - self._joint_lower, self._joint_upper - chosen)))
        if margin < 0.02:
            print(f"[IK] 관절 제한 여유 부족: margin={math.degrees(margin):.2f}deg")
            return None
        return chosen

    def _cartesian_step(self, remaining: float) -> float:
        # PREGRASP의 먼 구간은 빠르게, 마지막 8 cm와 실제 GRASP는 저속 직선 접근.
        if self._phase == "GRASP":
            base_step = 0.0015 if remaining <= 0.08 else 0.0045
        elif self._phase == "PREGRASP" and remaining <= 0.08:
            base_step = 0.0045
        elif self._phase == "PREGRASP":
            base_step = 0.030
        elif self._phase in {"VERIFY_RETRACT", "RETRACT"}:
            base_step = 0.004
        else:
            base_step = 0.008
        return base_step * self._cart_step_scale

    @staticmethod
    def _quat_multiply(left, right) -> np.ndarray:
        """wxyz quaternion 곱. right는 현재 tool 좌표계의 국소 회전이다."""
        w1, x1, y1, z1 = np.asarray(left, dtype=float)
        w2, x2, y2, z2 = np.asarray(right, dtype=float)
        result = np.array([
            w1*w2 - x1*x2 - y1*y2 - z1*z2,
            w1*x2 + x1*w2 + y1*z2 - z1*y2,
            w1*y2 - x1*z2 + y1*w2 + z1*x2,
            w1*z2 + x1*y2 - y1*x2 + z1*w2,
        ], dtype=float)
        return result / np.linalg.norm(result)

    @staticmethod
    def _quat_matrix_like(quat, reference_matrix) -> np.ndarray:
        """quat 회전행렬을 USD의 row/column convention에 맞춰 반환한다."""
        w, x, y, z = np.asarray(quat, dtype=float)
        column = np.array([
            [1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
            [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
            [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)],
        ], dtype=float)
        reference = np.asarray(reference_matrix, dtype=float).reshape(3, 3)
        return (column if np.linalg.norm(column - reference)
                <= np.linalg.norm(column.T - reference) else column.T)

    def _grasp_pose_candidates(self, fruit_world, current_orientation,
                               current_rotation, current_joints):
        """TCP는 과실 중심에 둔 채 도달 가능한 손목 기울기 후보를 만든다."""
        # tool-local Y/Z를 기울여 팔꿈치를 더 펼 수 있는 IK 자세도 탐색한다.
        # (tool angle, tool axis, seed joint index, warm-start offset). 같은 TCP/카메라
        # 자세를 유지하면서 joint_3/4/5를 더 감은 seed로 다른 IK branch를 찾는다.
        variants = [(0.0, None, None, 0.0)]
        for seed_joint in (2, 3, 4):  # 사용자 표기 joint_3, joint_4, joint_5
            for seed_deg in (10.0, -10.0, 20.0, -20.0, 30.0, -30.0):
                variants.append((0.0, None, seed_joint, seed_deg))
        for angle in (8.0, -8.0, 14.0, -14.0, 20.0, -20.0, 25.0, -25.0):
            variants.extend(((angle, (0.0, 1.0, 0.0), None, 0.0),
                             (angle, (0.0, 0.0, 1.0), None, 0.0)))
        candidates = []
        for angle_deg, axis, seed_joint, seed_deg in variants:
            orient = np.asarray(current_orientation, dtype=float)
            if axis is not None:
                half = math.radians(angle_deg) * 0.5
                delta = np.array([math.cos(half),
                                  axis[0] * math.sin(half),
                                  axis[1] * math.sin(half),
                                  axis[2] * math.sin(half)])
                orient = self._quat_multiply(orient, delta)
            rotation = self._quat_matrix_like(orient, current_rotation)
            tcp_offset_world = self._tcp_offset_l6 @ rotation
            link6_target = np.asarray(fruit_world) - tcp_offset_world
            self._sync_arm_base_pose()
            warm_start = np.asarray(current_joints, dtype=float).copy()
            if seed_joint is not None:
                warm_start[seed_joint] = float(np.clip(
                    warm_start[seed_joint] + math.radians(seed_deg),
                    self._joint_lower[seed_joint] + 0.03,
                    self._joint_upper[seed_joint] - 0.03))
            sol, ok = self._ik.compute_inverse_kinematics(
                frame_name=self._ee_frame,
                target_position=link6_target,
                target_orientation=orient,
                warm_start=warm_start)
            normalised = (self._normalise_ik_solution(
                sol, current_joints, reject_branch_jump=False)
                if ok and sol is not None else None)
            if normalised is None:
                continue
            # 같은 자세/같은 관절해가 여러 seed에서 반복되면 정체 시 같은 후보를
            # 불필요하게 재시도하지 않는다.
            if any(np.linalg.norm(normalised - item["solution"]) < math.radians(1.0)
                   and np.linalg.norm(orient - item["orientation"]) < 1e-4
                   for item in candidates):
                continue
            delta_q = np.abs(normalised - current_joints)
            margin = float(np.min(np.minimum(
                normalised - self._joint_lower,
                self._joint_upper - normalised)))
            # 가까운 관절해를 우선하되, 같은 수준이면 현재 카메라 방향을 덜 바꾼다.
            score = (float(np.max(delta_q)) + 0.15 * float(np.linalg.norm(delta_q))
                     + 0.20 * abs(float(normalised[2]))
                     + 0.0015 * abs(angle_deg)
                     + 0.0005 * abs(seed_deg)
                     + (2.0 if axis is not None else 0.0)
                     + 0.002 / max(margin, 0.02))
            candidates.append({"orientation": orient,
                               "link6_target": link6_target,
                               "solution": normalised,
                               "angle": angle_deg,
                               "axis": axis,
                               "seed_joint": seed_joint,
                               "seed_deg": seed_deg,
                               "score": score})
        candidates.sort(key=lambda item: item["score"])
        return candidates

    def _activate_pose_candidate(self, index: int) -> None:
        candidate = self._pose_candidates[index]
        self._pose_candidate_index = index
        self._target_orientation_world = candidate["orientation"].copy()
        self._target_world = candidate["link6_target"].copy()
        self._solution = candidate["solution"].copy()
        self._ik_fail_count = 0
        self._stagnation_best = None
        self._stagnation_frames = 0
        axis = candidate["axis"]
        seed_joint = candidate.get("seed_joint")
        seed_deg = float(candidate.get("seed_deg", 0.0))
        axis_name = (f"joint{int(seed_joint) + 1}_seed={seed_deg:+.0f}deg"
                     if axis is None and seed_joint is not None
                     else ("original" if axis is None else
                           ("pitch" if axis[1] else "yaw")))
        print(f"[IK] GRASP 자세 후보 {index + 1}/{len(self._pose_candidates)} "
              f"{axis_name}" + ("" if seed_joint is not None else
              f"={candidate['angle']:+.0f}deg"))

    def _near_singularity(self, joints) -> bool:
        """Lula가 Jacobian API를 제공하면 6D condition number로 특이점을 거른다."""
        if not self._jacobian_supported:
            return False
        try:
            jacobian = np.asarray(
                self._ik.compute_jacobian(self._ee_frame, joints), dtype=float)
            condition = float(np.linalg.cond(jacobian))
            if not np.isfinite(condition) or condition > 1.0e4:
                print(f"[IK] 특이점 여유 부족: Jacobian condition={condition:.1f}")
                return True
        except (AttributeError, TypeError):
            # Isaac/Lula 버전에 API가 없으면 branch jump와 joint-limit 검사만 사용한다.
            self._jacobian_supported = False
            print("[IK] Jacobian API 없음 — branch/joint-limit 안전검사 사용")
        except Exception as exc:
            print(f"[IK] Jacobian 검사 건너뜀: {exc}")
        return False

    def set_target(self, position, target_id: int = 0, phase: str = "MOVE") -> None:
        from pxr import Gf, UsdGeom

        values = np.asarray(position, dtype=float)
        if values.shape != (3,) or not np.all(np.isfinite(values)):
            raise ValueError(f"IK 목표는 유한한 xyz 3개여야 합니다: {position}")
        cache = UsdGeom.XformCache()
        m_ref = cache.GetLocalToWorldTransform(
            self._stage.GetPrimAtPath(self._reference_prim))
        fruit_world = np.asarray(m_ref.Transform(Gf.Vec3d(*values)), dtype=float)
        self._target_base = values.copy()
        self._target_tcp_world = fruit_world.copy()
        self._target_id = int(target_id)
        self._phase = str(phase)
        # 그리퍼 방향 = **현재 link_6 world 방향 유지**(파지 접근 일관). 목표 link_6 위치는
        # 과실에서 TCP 오프셋(현재 방향으로 회전)만큼 뒤로 뺀 곳.
        m_l6 = cache.GetLocalToWorldTransform(
            self._stage.GetPrimAtPath(self._link6_prim))
        R = np.asarray(m_l6.ExtractRotationMatrix(), dtype=float).reshape(3, 3)
        quat = m_l6.ExtractRotationQuat()
        imag = quat.GetImaginary()
        orient = np.array([quat.GetReal(), imag[0], imag[1], imag[2]], dtype=float)
        tcp_offset_world = self._tcp_offset_l6 @ R          # 로컬→월드 (row-vector)
        link6_target = fruit_world - tcp_offset_world
        self._target_world = link6_target.copy()
        self._target_orientation_world = orient
        # 최종점 도달 가능성만 먼저 검사한다. 실제 이동에는 이 최종 관절해를 사용하지 않고
        # apply()가 현재 USD pose에서 만든 짧은 중간점 해만 사용한다.
        self._sync_arm_base_pose()
        current = self._arm_q()
        self._pose_candidates = []
        self._pose_candidate_index = 0
        if self._phase == "GRASP":
            self._pose_candidates = self._grasp_pose_candidates(
                fruit_world, orient, R, current)
        if self._pose_candidates:
            self._activate_pose_candidate(0)
            normalised = self._solution.copy()
            link6_target = self._target_world.copy()
        else:
            sol, ok = self._ik.compute_inverse_kinematics(
                frame_name=self._ee_frame,
                target_position=link6_target,
                target_orientation=orient,
                warm_start=current)
            normalised = (self._normalise_ik_solution(
                sol, current, reject_branch_jump=False)
                if ok and sol is not None else None)
        if normalised is not None:
            self._solution = normalised
            self._motion_active = True
            self._mode = "CARTESIAN"
            self._ik_fail_count = 0
            self._cart_step_scale = 1.0
            self._replan_hold_frames = 0
            self._divergence_replans = 0
            self._best_tcp_distance = None
            self._last_tcp_distance = None
            self._distance_increase_frames = 0
            self._stagnation_best = None
            self._stagnation_frames = 0
        else:
            self._motion_active = False
            print(f"[IK] 도달 불가 — target={link6_target.round(3).tolist()} "
                  f"phase={self._phase} (팔 유지)")

    def is_active(self) -> bool:
        """지금 팔을 능동 제어 중인지(파지/홈이동). False 면 팔을 홈에 고정해도 됨."""
        return bool(self._motion_active)

    def is_pursuing_target(self) -> bool:
        """파지용 Cartesian 목표를 능동 추종 중인지 — 이동 중 홈 접기 판단에 쓴다."""
        return bool(self._motion_active and self._mode == "CARTESIAN")

    def set_fk_probe(self, joint_index: int = 3, delta_deg: float = 5.0) -> None:
        """FK 비교용으로 홈 기준 한 관절만 작게 움직인다(외부 명령은 ±10° 제한)."""
        joint_index = int(joint_index)
        delta_deg = float(delta_deg)
        if not 0 <= joint_index < 6:
            raise ValueError("fk_probe joint는 0..5여야 합니다")
        if not math.isfinite(delta_deg) or abs(delta_deg) > 10.0:
            raise ValueError("fk_probe delta_deg는 ±10도 이하여야 합니다")
        target = self._home_positions.copy()
        target[joint_index] += math.radians(delta_deg)
        if not (self._joint_lower[joint_index] <= target[joint_index]
                <= self._joint_upper[joint_index]):
            raise ValueError("fk_probe 목표가 관절 제한 밖입니다")
        self._solution = target
        self._command_q = self._arm_q().copy()
        self._target_world = None
        self._target_tcp_world = None
        self._target_base = None
        self._target_orientation_world = None
        self._phase = "FK_PROBE"
        self._mode = "JOINT_PROBE"
        self._motion_active = True
        print(f"[FKCHK] probe joint_{joint_index + 1} delta={delta_deg:+.1f}deg")

    def adjust_gripper_yaw(self, delta_deg: float, target_id: int = 0) -> None:
        """단측 접촉 보정: 현재 자세에서 손목 6축을 소폭 회전하고 TCP 위치는 유지한다."""
        delta_deg = float(delta_deg)
        if not math.isfinite(delta_deg) or abs(delta_deg) > 15.0:
            raise ValueError("grasp yaw 보정은 ±15도 이하여야 합니다")
        current = self._arm_q().copy()
        target = current.copy()
        target[5] += math.radians(delta_deg)
        target = self._normalise_ik_solution(
            target, current, reject_branch_jump=False)
        if target is None:
            raise ValueError("grasp yaw 보정 목표가 관절 제한 밖입니다")
        self._target_world = None
        self._target_tcp_world = None
        self._target_base = None
        self._target_orientation_world = None
        self._target_id = int(target_id)
        self._phase = "GRASP_YAW_CORRECT"
        self._mode = "JOINT_ADJUST"
        self._motion_active = True
        self._solution = target
        self._command_q = current.copy()
        print(f"[GraspCorrect] wrist yaw {delta_deg:+.1f}deg id={target_id}")

    def apply(self) -> None:
        if not self._motion_active:
            return
        if self._mode in {"HOME", "JOINT_PROBE", "JOINT_ADJUST"}:
            target = (self._home_positions if self._mode == "HOME"
                      else self._solution)
            # 내부 명령 궤적을 프레임당 0.003rad(~0.17°)만 이동한다. 실제 관절이 관성으로
            # 지연돼도 setpoint가 갑자기 홈으로 점프하거나 반대로 되쫓지 않는다.
            joint_step = 0.009 if self._mode == "JOINT_ADJUST" else 0.003
            self._command_q += np.clip(
                target - self._command_q, -joint_step, joint_step)
            self._robot.apply_action(ArticulationAction(
                joint_positions=self._command_q,
                joint_indices=self._arm_dof))
            return
        if self._mode == "HOLD":
            self._robot.apply_action(ArticulationAction(
                joint_positions=self._solution, joint_indices=self._arm_dof))
            return
        if self._mode != "CARTESIAN" or self._target_world is None:
            return
        cur = self._arm_q()
        if self._replan_hold_frames > 0:
            self._replan_hold_frames -= 1
            self._robot.apply_action(ArticulationAction(
                joint_positions=cur, joint_indices=self._arm_dof))
            return
        from pxr import UsdGeom
        actual = np.asarray(
            UsdGeom.XformCache().GetLocalToWorldTransform(
                self._stage.GetPrimAtPath(self._link6_prim)).ExtractTranslation(),
            dtype=float)
        delta = self._target_world - actual
        remaining = float(np.linalg.norm(delta))
        if remaining < 0.001:
            return
        step = min(remaining, self._cartesian_step(remaining))
        waypoint = actual + delta / remaining * step
        self._sync_arm_base_pose()
        sol, ok = self._ik.compute_inverse_kinematics(
            frame_name=self._ee_frame,
            target_position=waypoint,
            target_orientation=self._target_orientation_world,
            warm_start=cur)
        normalised = (self._normalise_ik_solution(sol, cur)
                      if ok and sol is not None else None)
        self._ik_step_count += 1
        if (normalised is not None and self._ik_step_count % 10 == 0
                and self._near_singularity(normalised)):
            normalised = None
        if normalised is None:
            self._ik_fail_count += 1
            if self._ik_fail_count >= 5:
                self._motion_active = False
                self._mode = "STOPPED"
                self._phase = "ERROR_IK_PATH"
                print(f"[IK] 중간점 IK 5회 실패 — waypoint={waypoint.round(3).tolist()}")
            return
        self._ik_fail_count = 0
        # 토마토 근처는 반동 방지를 위해 계속 저속으로 유지한다. PREGRASP 마지막
        # 8cm는 0.018rad, 실제 접촉 GRASP는 0.009rad만 허용한다(기존 저속의 3배).
        if self._phase == "GRASP":
            max_joint_step = 0.009
        elif self._phase == "PREGRASP" and remaining <= 0.08:
            max_joint_step = 0.018
        else:
            max_joint_step = self._max_joint_step
        cmd = cur + np.clip(
            normalised - cur, -max_joint_step, max_joint_step)
        self._robot.apply_action(ArticulationAction(
            joint_positions=cmd, joint_indices=self._arm_dof))

    def reset(self) -> None:
        self._target_world = None
        self._target_tcp_world = None
        self._target_base = None
        self._target_orientation_world = None
        self._motion_active = False
        self._mode = "IDLE"
        self._target_id = 0
        self._phase = "IDLE"
        self._sync_arm_base_pose()

    def stop(self) -> None:
        # 정지는 제어 해제가 아니라 현재 자세 HOLD다. motion_active를 False로 만들면
        # mm.py의 "RMP 비활성 시 홈 고정"이 즉시 개입해 GRASP 위치에서 팔을 홈으로
        # 튕겨 보낸다. 현재 실제 관절값을 계속 명령해 그리퍼가 닫히는 동안 자세를 유지한다.
        self._solution = self._arm_q().copy()
        self._motion_active = True
        self._mode = "HOLD"
        if not self._phase.startswith("STOPPED_"):
            self._phase = f"STOPPED_{self._phase}"

    def go_home(self, target_id: int = 0) -> None:
        self._target_world = None
        self._target_tcp_world = None
        self._target_base = None
        self._target_orientation_world = None
        self._target_id = int(target_id)
        self._phase = "HOME"
        self._mode = "HOME"
        self._motion_active = True
        self._solution = self._home_positions.copy()
        self._command_q = self._arm_q().copy()

    def go_preplace(self, target_id: int = 0) -> None:
        """파지 후 플레이스 전 자세 — 홈에서 joint_3·joint_5 만 바꿔 그리퍼를 지면과
        수직으로 세우고 살짝 들어올린다. 관절공간(JOINT_ADJUST)으로 이동한다."""
        target = self._home_positions.copy()
        target[2] = math.radians(PREPLACE_JOINT3_DEG)   # joint_3
        target[4] = math.radians(PREPLACE_JOINT5_DEG)   # joint_5
        target = np.clip(target, self._joint_lower, self._joint_upper)
        self._target_world = None
        self._target_tcp_world = None
        self._target_base = None
        self._target_orientation_world = None
        self._target_id = int(target_id)
        self._phase = "PREPLACE"
        self._mode = "JOINT_ADJUST"
        self._motion_active = True
        self._solution = target
        self._command_q = self._arm_q().copy()
        print(f"[PrePlace] joint_3={PREPLACE_JOINT3_DEG:.0f}° "
              f"joint_5={PREPLACE_JOINT5_DEG:.0f}° 로 파지물 들기 id={target_id}")

    def _watch_divergence(self, distance: float) -> None:
        if (not self._motion_active
                or self._phase not in {"PREGRASP", "GRASP"}):
            return
        if self._best_tcp_distance is None:
            self._best_tcp_distance = distance
            self._last_tcp_distance = distance
            return
        self._best_tcp_distance = min(self._best_tcp_distance, distance)
        if distance > self._last_tcp_distance + 0.001:
            self._distance_increase_frames += 1
        else:
            self._distance_increase_frames = 0
        self._last_tcp_distance = distance
        diverged = (self._distance_increase_frames >= 5
                    or distance > self._best_tcp_distance + 0.03)
        if not diverged:
            return
        if self._divergence_replans == 0:
            self._divergence_replans = 1
            self._cart_step_scale = 0.5
            self._replan_hold_frames = 6
            self._distance_increase_frames = 0
            self._best_tcp_distance = distance
            print(f"[Cartesian] 발산 감지 {distance:.3f}m — 정지 후 50% 속도로 재계획")
            return
        # ROS가 ERROR 상태를 받아 홈 명령을 보낼 때까지 현재 관절을 HOLD한다.
        # active=False로 풀면 mm.py의 대기 홈 고정이 같은 프레임에 개입해 더 튄다.
        self._motion_active = True
        self._mode = "HOLD"
        self._phase = "ERROR_DIVERGENCE"
        print(f"[Cartesian] 재계획 후에도 발산 — 자동 정지/홈 복귀 distance={distance:.3f}m")

    def _watch_stagnation(self, distance: float) -> None:
        """유효 IK가 반복되지만 TCP가 전진하지 않는 경우 다른 손목 자세로 재계획."""
        if (not self._motion_active or self._phase != "GRASP"):
            return
        if self._stagnation_best is None or distance < self._stagnation_best - 0.002:
            self._stagnation_best = distance
            self._stagnation_frames = 0
            return
        self._stagnation_frames += 1
        # status는 약 20 Hz. 2초 동안 2 mm도 개선되지 않으면 다음 자세를 사용한다.
        if self._stagnation_frames < 40:
            return
        next_index = self._pose_candidate_index + 1
        if next_index < len(self._pose_candidates):
            print(f"[Cartesian] GRASP 정체 {distance:.3f}m — 다른 손목 자세로 재계획")
            self._activate_pose_candidate(next_index)
            self._replan_hold_frames = 4
            self._best_tcp_distance = distance
            self._last_tcp_distance = distance
            self._distance_increase_frames = 0
            return
        self._motion_active = False
        self._mode = "STOPPED"
        self._phase = "ERROR_STAGNATION"
        print(f"[Cartesian] 모든 GRASP 자세에서 정체 — 자동 홈 복귀 distance={distance:.3f}m")

    def status(self, position_tolerance: float = 0.02) -> dict:
        result = {"id": self._target_id, "phase": self._phase,
                  "active": self._motion_active, "reached": False,
                  "distance": None, "at_home": False,
                  "current_position": None, "target_position": None}
        positions = self._arm_q()
        home_error = float(np.max(np.abs(positions - self._home_positions)))
        # RMPflow cspace 는 손목 여유축(예: joint_4)을 홈으로 완전히 못 당겨 ~0.1rad 잔차가
        # 남는다(2026-07-22 실측: joint_4=0.107). ee 도달은 정확하므로 무해하지만 0.03 이면
        # at_home 이 영영 false → 베이스 이동 게이트가 영구 차단됐다. cspace 잔차 허용.
        result["at_home"] = home_error <= 0.15
        if self._mode == "HOME":
            result["distance"] = home_error
            result["reached"] = result["at_home"]
        elif self._mode == "JOINT_ADJUST":
            joint_error = float(np.max(np.abs(positions - self._solution)))
            result["distance"] = joint_error
            result["reached"] = joint_error <= 0.01
            if result["reached"]:
                self._solution = positions.copy()
                self._mode = "HOLD"
        elif self._target_world is not None:
            from pxr import UsdGeom
            # 실측 = link_6(플랜지) world. 아래에서 tool_tcp_prim 있으면 HarvestTCP로 덮음.
            measured_world = np.asarray(
                UsdGeom.XformCache().GetLocalToWorldTransform(
                    self._stage.GetPrimAtPath(self._link6_prim)).ExtractTranslation(),
                dtype=float)
            desired_world = self._target_world
            # 완료 판정은 RMPflow 내부 플랜지(link_6)가 아니라 실제 USD HarvestTCP로 한다.
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
            # [디버그 2026-07-22] 목표(과실) world vs 실제 HarvestTCP world 축별 차이.
            # 좌우(Y) 어긋남 진단용 — Isaac 콘솔에서 확인. ~0.3s 간격.
            self._dbg = getattr(self, "_dbg", 0) + 1
            if self._dbg % 20 == 0:
                d = measured_world - desired_world
                print(f"[TCP] phase={self._phase} "
                      f"fruit=({desired_world[0]:.3f},{desired_world[1]:.3f},{desired_world[2]:.3f}) "
                      f"tcp=({measured_world[0]:.3f},{measured_world[1]:.3f},{measured_world[2]:.3f}) "
                      f"diff=({d[0]:+.3f},{d[1]:+.3f},{d[2]:+.3f})")
            # 접근/후퇴/바스켓 상공은 경유점이므로 산업용 팔이 자세 제약 아래
            # 수 cm 앞에서 수렴해도 다음 단계로 진행해도 된다. GRASP는 과실 collider
            # 표면에서 닫기를 시작하고, 실제 성공 여부는 별도 접촉/예압 검사로 판정한다.
            phase_tolerance = {
                # PREGRASP는 과실 15cm 전 경유점이다. 6cm 이내에서 다음 저속 GRASP를
                # 시작해도 과실과 TCP 사이에는 약 21cm 여유가 남는다.
                "PREGRASP": 0.06,
                # GRASP 명령 자체가 과실 중심이 아니라 표면 34mm 앞을 목표로 한다.
                # 실측 폐루프 보정 후 이 목표와 29mm에서 수렴했을 때 원래 과실 중심과
                # 실제 TCP 거리는 21mm였다. 32mm에서 닫기를 시작하고 실제 성공은 이후
                # 중심거리·동일 과실 양손 접촉·30N 예압으로 fail-closed 판정한다.
                "GRASP": 0.032,
                "RETRACT": 0.04,
                "BASKET_APPROACH": 0.04,
            }.get(self._phase, position_tolerance)
            result["reached"] = distance <= phase_tolerance
            if result["reached"] and self._mode == "CARTESIAN":
                # 도달 판정 뒤 다음 ROS 목표가 올 때까지 계속 미는 것을 막는다. 현재 실제
                # 관절값을 절대 목표로 고정해 관성 반동과 passive-home 개입을 모두 차단한다.
                self._solution = positions.copy()
                self._mode = "HOLD"
                self._distance_increase_frames = 0
            elif not result["reached"]:
                self._watch_divergence(distance)
                self._watch_stagnation(distance)
            result["phase"] = self._phase
            result["active"] = self._motion_active
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
        self._ik.set_robot_base_pose(
            np.asarray(matrix.ExtractTranslation(), dtype=float),
            np.asarray([quat.GetReal(), imag[0], imag[1], imag[2]], dtype=float))


class RmpFlowTargetController(LegacyIkTargetController):
    """Isaac RMPflow로 m0617 EE를 연속 제어하는 기본 수확 제어기.

    LegacyIkTargetController의 홈/HOLD/FK 진단과 실제 HarvestTCP 기반 상태 판정은
    재사용하지만, PREGRASP/GRASP Cartesian 이동에는 중간점 IK를 전혀 사용하지 않는다.
    """

    def __init__(self, robot, stage, reference_prim: str, arm_base_prim: str,
                 physics_dt: float = 1.0 / 60.0,
                 tool_tcp_prim: str | None = None,
                 home_positions=None):
        super().__init__(
            robot, stage, reference_prim, arm_base_prim,
            physics_dt=physics_dt,
            tool_tcp_prim=tool_tcp_prim,
            home_positions=home_positions)
        import isaacsim.robot_motion.motion_generation as mg
        from pathlib import Path

        cfg_dir = Path(__file__).resolve().parent / "m0617"
        self._rmp_policy = mg.lula.motion_policies.RmpFlow(
            robot_description_path=str(cfg_dir / "m0617_description.yaml"),
            rmpflow_config_path=str(cfg_dir / "m0617_rmpflow_common.yaml"),
            urdf_path=str(cfg_dir / "m0617.urdf"),
            end_effector_frame_name=self._ee_frame,
            maximum_substep_size=0.00334)
        self._articulation_policy = mg.ArticulationMotionPolicy(
            robot, self._rmp_policy, physics_dt)
        self._rmp_command_q = self._arm_q().copy()
        self._rmp_action_shape_warned = False
        self._rmp_orientation_replans = 0
        self._rmp_orientation_free = False
        self._physics_dt = float(physics_dt)
        self._rmp_nominal_link6_target = None
        self._rmp_tcp_error_integral = np.zeros(3, dtype=float)
        self._rmp_feedback_frames = 0
        self._sync_rmp_base_pose()
        self._rmp_policy.set_cspace_target(self._home_positions)
        print("[RMPflow] 실제 motion policy 활성 "
              "(Cartesian waypoint IK는 --legacy-ik 전용)")

    def _sync_rmp_base_pose(self) -> None:
        """이동 베이스 위 arm/base_link의 월드 자세를 RMPflow에 동기화한다."""
        from pxr import UsdGeom

        matrix = UsdGeom.XformCache().GetLocalToWorldTransform(
            self._stage.GetPrimAtPath(self._arm_base_prim))
        quat = matrix.ExtractRotationQuat()
        imag = quat.GetImaginary()
        self._rmp_policy.set_robot_base_pose(
            robot_position=np.asarray(
                matrix.ExtractTranslation(), dtype=float),
            robot_orientation=np.asarray(
                [quat.GetReal(), imag[0], imag[1], imag[2]], dtype=float))

    def set_target(self, position, target_id: int = 0,
                   phase: str = "MOVE") -> None:
        """base_link 좌표의 HarvestTCP 목표를 RMPflow link_6 목표로 변환한다."""
        from pxr import Gf, UsdGeom

        values = np.asarray(position, dtype=float)
        if values.shape != (3,) or not np.all(np.isfinite(values)):
            raise ValueError(f"RMPflow 목표는 유한한 xyz 3개여야 합니다: {position}")
        cache = UsdGeom.XformCache()
        reference_world = cache.GetLocalToWorldTransform(
            self._stage.GetPrimAtPath(self._reference_prim))
        fruit_world = np.asarray(
            reference_world.Transform(Gf.Vec3d(*values)), dtype=float)

        link6_matrix = cache.GetLocalToWorldTransform(
            self._stage.GetPrimAtPath(self._link6_prim))
        rotation = np.asarray(
            link6_matrix.ExtractRotationMatrix(), dtype=float).reshape(3, 3)
        quat = link6_matrix.ExtractRotationQuat()
        imag = quat.GetImaginary()
        orientation = np.asarray(
            [quat.GetReal(), imag[0], imag[1], imag[2]], dtype=float)
        link6_target = fruit_world - self._tcp_offset_l6 @ rotation

        self._target_base = values.copy()
        self._target_tcp_world = fruit_world.copy()
        self._target_world = link6_target.copy()
        self._rmp_nominal_link6_target = link6_target.copy()
        self._target_orientation_world = orientation.copy()
        self._target_id = int(target_id)
        self._phase = str(phase)
        self._mode = "CARTESIAN"
        self._motion_active = True
        self._best_tcp_distance = None
        self._last_tcp_distance = None
        self._distance_increase_frames = 0
        self._divergence_replans = 0
        self._rmp_orientation_replans = 0
        self._rmp_orientation_free = False
        self._stagnation_best = None
        self._stagnation_frames = 0
        self._rmp_tcp_error_integral.fill(0.0)
        self._rmp_feedback_frames = 0
        self._rmp_command_q = self._arm_q().copy()
        self._sync_rmp_base_pose()
        # GRASP에서 홈 자세를 null-space 목표로 계속 주면, 옆으로 뻗어야 하는
        # 과실 앞에서 task-space 인력과 home 인력이 평형을 이뤄 멈출 수 있다.
        # 마지막 접근은 현재 관절 자세를 기준으로 삼아 불필요한 복귀 편향을 없앤다.
        cspace_target = (self._arm_q().copy()
                         if self._phase == "GRASP"
                         else self._home_positions)
        self._rmp_policy.set_cspace_target(cspace_target)
        self._rmp_policy.set_end_effector_target(
            target_position=link6_target,
            target_orientation=orientation)
        print(f"[RMPflow] id={self._target_id} phase={self._phase} "
              f"TCP target={fruit_world.round(3).tolist()} "
              f"link_6 target={link6_target.round(3).tolist()}")

    def apply(self) -> None:
        if not self._motion_active:
            return
        if self._mode != "CARTESIAN":
            super().apply()
            return
        self._sync_rmp_base_pose()
        self._update_tcp_feedback_target()
        action = self._articulation_policy.get_next_articulation_action()
        raw_positions = action.joint_positions
        if raw_positions is None:
            return
        if hasattr(raw_positions, "cpu"):
            raw_positions = raw_positions.cpu().numpy()
        raw_positions = np.asarray(raw_positions, dtype=float)
        indices = action.joint_indices
        if indices is not None:
            if hasattr(indices, "cpu"):
                indices = indices.cpu().numpy()
            indices = np.asarray(indices, dtype=int)
            by_index = dict(zip(indices.tolist(), raw_positions.tolist()))
            if not all(int(index) in by_index for index in self._arm_dof):
                if not self._rmp_action_shape_warned:
                    print("[RMPflow] action에 m0617 6축 인덱스가 없어 명령 거부")
                    self._rmp_action_shape_warned = True
                return
            raw_arm = np.asarray(
                [by_index[int(index)] for index in self._arm_dof], dtype=float)
        elif raw_positions.shape == (6,):
            raw_arm = raw_positions
        elif raw_positions.ndim == 1 and len(raw_positions) > int(np.max(self._arm_dof)):
            raw_arm = raw_positions[self._arm_dof]
        else:
            if not self._rmp_action_shape_warned:
                print(f"[RMPflow] 알 수 없는 action shape={raw_positions.shape}; 명령 거부")
                self._rmp_action_shape_warned = True
            return

        current = self._arm_q()
        # RMPflow가 특이점 근처에서 동치각 branch를 바꿔도 현재 자세에 가장 가까운
        # ±2π 표현만 사용한다. IK를 다시 푸는 것이 아니라 정책 출력의 표현만 정규화한다.
        raw_arm = raw_arm + 2.0 * math.pi * np.round(
            (current - raw_arm) / (2.0 * math.pi))
        raw_arm = np.clip(raw_arm, self._joint_lower, self._joint_upper)
        # 실제 시험에서 무제한 정책 출력이 GRASP 중 TCP를 10~30cm씩 왕복시켰다.
        # 정책 방향은 유지하되 command trajectory 를 제한하고 실제 관절보다 두 프레임
        # 이상 앞서지 않게 한다. 접근 속도는 phase 대신 실제 TCP→과실 거리로 정한다:
        # 6cm 밖이면 빠르게(0.030rad/frame), 6cm 안에서는 거리에 비례해 0.006까지 감속.
        max_step = self._approach_max_step()
        self._rmp_command_q += np.clip(
            raw_arm - self._rmp_command_q, -max_step, max_step)
        self._rmp_command_q = current + np.clip(
            self._rmp_command_q - current, -2.0 * max_step, 2.0 * max_step)
        self._robot.apply_action(ArticulationAction(
            joint_positions=self._rmp_command_q,
            joint_indices=self._arm_dof))

    def _approach_max_step(self) -> float:
        """프레임당 관절 스텝(rad). 사용자 요청(2026-07-23): **거리 상관없이** 균일한
        접근 속도. 흡착 그리퍼용으로 추가 2.5배 → 0.075→0.1875 로 고정(근접 감속 없음)."""
        return 0.1875

    def reset(self) -> None:
        super().reset()
        if hasattr(self, "_rmp_policy"):
            self._rmp_policy.reset()
            self._sync_rmp_base_pose()
            self._rmp_policy.set_cspace_target(self._home_positions)
            self._rmp_command_q = self._arm_q().copy()
            self._rmp_nominal_link6_target = None
            self._rmp_orientation_free = False
            self._rmp_tcp_error_integral.fill(0.0)
            self._rmp_feedback_frames = 0

    def _update_tcp_feedback_target(self) -> None:
        """접근(PREGRASP)~파지(GRASP) 내내 실제 TCP 오차를 RMPflow 목표에 PI 보상한다.

        RMPflow 관절공간 정책이 남기는 계통편향(2026-07-23 사용자: '오른쪽으로 치우침')을
        GRASP 마지막 구간에서만 잡으면 접근 중 이미 벌어진 좌우 오차를 다 못 지운다. 그래서
        PREGRASP 부터 25cm 이내에서 강하게 보상한다."""
        if (self._phase not in {"PREGRASP", "GRASP"}
                or self._target_tcp_world is None
                or self._rmp_nominal_link6_target is None
                or not self._tool_tcp_prim):
            return
        from pxr import Gf, UsdGeom

        tcp_prim = self._stage.GetPrimAtPath(self._tool_tcp_prim)
        if not tcp_prim.IsValid():
            return
        tcp_xform = UsdGeom.XformCache().GetLocalToWorldTransform(tcp_prim)
        actual_tcp = np.asarray(tcp_xform.ExtractTranslation(), dtype=float)
        error = self._target_tcp_world - actual_tcp
        distance = float(np.linalg.norm(error))
        if not np.isfinite(distance) or distance > 0.25:
            return

        # 사용자 요청(2026-07-23): "툴 z축으로 쭉 직선" — 오차를 접근축(툴 +Z)과 그에
        # 수직인 성분으로 나눈다. 옆으로 새는 lateral 성분은 강하게 죽여 궤적을 접근선에
        # 고정하고(계통 우편향 제거), 접근축 along 성분만으로 전진시킨다. I항도 lateral만
        # 누적한다 — 계통편향은 옆방향이라. 실제 관절은 apply() 프레임 스텝 제한을 거친다.
        approach = np.asarray(
            tcp_xform.TransformDir(Gf.Vec3d(0.0, 0.0, 1.0)), dtype=float)
        u_norm = float(np.linalg.norm(approach))
        if u_norm > 1e-9:
            u = approach / u_norm
            along = float(np.dot(error, u)) * u          # 접근축 성분(전진)
            lateral = error - along                       # 옆으로 샌 성분(드리프트)
        else:
            along, lateral = error, np.zeros(3)
        self._rmp_tcp_error_integral += 0.80 * lateral * self._physics_dt
        integral_norm = float(np.linalg.norm(self._rmp_tcp_error_integral))
        if integral_norm > 0.10:
            self._rmp_tcp_error_integral *= 0.10 / integral_norm
        # lateral 게인 ≫ along 게인 → 접근선에서 벗어나는 걸 즉시 되돌린다.
        correction = 1.60 * along + 6.0 * lateral + self._rmp_tcp_error_integral
        correction_norm = float(np.linalg.norm(correction))
        if correction_norm > 0.25:
            correction *= 0.25 / correction_norm
        target_orientation = self._target_orientation_world
        if self._rmp_orientation_free:
            # 손목 방향을 푼 뒤에는 link_6→TCP 오프셋의 월드 방향도 계속 변한다.
            # 현재 USD 회전으로 매 프레임 link_6 명목 목표를 다시 만들어야 실제
            # HarvestTCP가 과실 중심을 계속 추적한다.
            link6_matrix = UsdGeom.XformCache().GetLocalToWorldTransform(
                self._stage.GetPrimAtPath(self._link6_prim))
            rotation = np.asarray(
                link6_matrix.ExtractRotationMatrix(), dtype=float).reshape(3, 3)
            self._rmp_nominal_link6_target = (
                self._target_tcp_world - self._tcp_offset_l6 @ rotation)
            target_orientation = None
        corrected_target = self._rmp_nominal_link6_target + correction
        self._target_world = corrected_target.copy()
        self._rmp_policy.set_end_effector_target(
            target_position=corrected_target,
            target_orientation=target_orientation)
        self._rmp_feedback_frames += 1
        if self._rmp_feedback_frames % 40 == 0:
            print(f"[RMPflow Compensation] tcp_error="
                  f"{error.round(4).tolist()} "
                  f"correction={correction.round(4).tolist()}")

    def _watch_divergence(self, distance: float) -> None:
        """RMPflow가 목표 주변에서 크게 왕복하면 충돌 전에 정지한다."""
        if (not self._motion_active
                or self._phase not in {"PREGRASP", "GRASP"}):
            return
        if self._best_tcp_distance is None:
            self._best_tcp_distance = distance
            self._last_tcp_distance = distance
            return
        self._best_tcp_distance = min(self._best_tcp_distance, distance)
        if distance > self._last_tcp_distance + 0.002:
            self._distance_increase_frames += 1
        else:
            self._distance_increase_frames = 0
        self._last_tcp_distance = distance
        if (self._distance_increase_frames < 10
                and distance <= self._best_tcp_distance + 0.10):
            return
        # 오류 상태를 ROS가 처리할 때까지 현재 자세를 유지해 홈 setpoint의 즉시 개입을 막는다.
        self._motion_active = True
        self._mode = "HOLD"
        self._phase = "ERROR_DIVERGENCE"
        self._solution = self._arm_q().copy()
        print(f"[RMPflow] 큰 발산 감지 — 충돌 방지 정지 "
              f"best={self._best_tcp_distance:.3f}m current={distance:.3f}m")

    def _watch_stagnation(self, distance: float) -> None:
        """GRASP 정체 시 손목 제약을 풀고 위치 우선으로 한 번 재계획한다."""
        if not self._motion_active or self._phase != "GRASP":
            return
        if self._stagnation_best is None or distance < self._stagnation_best - 0.002:
            self._stagnation_best = distance
            self._stagnation_frames = 0
            return
        self._stagnation_frames += 1
        # 방향 제약을 푼 뒤에는 새 자세를 찾을 시간을 더 준다.
        required_frames = 100 if self._rmp_orientation_free else 40
        if self._stagnation_frames < required_frames:
            return
        if self._rmp_orientation_replans == 0 and self._target_tcp_world is not None:
            from pxr import UsdGeom

            matrix = UsdGeom.XformCache().GetLocalToWorldTransform(
                self._stage.GetPrimAtPath(self._link6_prim))
            rotation = np.asarray(
                matrix.ExtractRotationMatrix(), dtype=float).reshape(3, 3)
            link6_target = (
                self._target_tcp_world - self._tcp_offset_l6 @ rotation)
            self._target_world = link6_target
            self._rmp_nominal_link6_target = link6_target.copy()
            self._rmp_tcp_error_integral.fill(0.0)
            self._rmp_orientation_free = True
            # 현재 관절을 null-space 기준으로 잡고 EE 방향 목표는 제거한다.
            # 위치가 닿지 않는데 손목 방향을 고정한 채 같은 문제를 반복하지 않는다.
            self._rmp_policy.set_cspace_target(self._arm_q().copy())
            self._rmp_policy.set_end_effector_target(
                target_position=link6_target,
                target_orientation=None)
            self._rmp_command_q = self._arm_q().copy()
            self._rmp_orientation_replans = 1
            self._stagnation_best = distance
            self._stagnation_frames = 0
            self._best_tcp_distance = distance
            self._last_tcp_distance = distance
            self._distance_increase_frames = 0
            print(f"[RMPflow] GRASP 정체 {distance:.3f}m — "
                  "손목 방향 제약 해제, 위치 우선으로 재계획")
            return
        self._motion_active = True
        self._mode = "HOLD"
        self._phase = "ERROR_STAGNATION"
        self._solution = self._arm_q().copy()
        print(f"[RMPflow] 재설정 후에도 GRASP 정체 — 현재 자세 정지 "
              f"distance={distance:.3f}m")


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
    ARM = ("joint_1", "joint_2", "joint_3",
           "joint_4", "joint_5", "joint_6")
    GRIPPER = ("finger_joint", "right_inner_knuckle_joint")
    GRIPPER_CLOSED = 0.50                # rad

    def __init__(self, robot):
        self._robot = robot
        self._jm = _JointMap(list(robot.dof_names))
        self._ctrl = self._jm.idx(*self.BASE, *self.ARM, *self.GRIPPER)
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
        for i in self._jm.idx(*self.GRIPPER):
            self._t[i] = f * self.GRIPPER_CLOSED

    def move_gripper(self, d: float) -> None:
        for i in self._jm.idx(*self.GRIPPER):
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
        return f"arm(deg)=[{arm}]  gripper={q[self._jm.idx(self.GRIPPER[0])[0]]:.2f}"


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
        self._stationary_pose = None
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
        # 정지 중에는 마지막 root pose를 유지하되 관절 드라이브는 계속
        # 적용하므로 리프트와 조향의 목표 제어는 그대로 동작한다.
        if dt is not None and abs(self._drive_vel) > 1e-6:
            # 정지 중 고정해 둔 pose는 다음 이동을 시작할 때 해제한다.
            self._stationary_pose = None
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
        elif dt is not None:
            # 속도를 한 번만 0으로 만든 뒤 동적 강체로 두면 바퀴 접촉과
            # 포크/팔레트 하중이 차체에 매 프레임 미세한 병진·회전을 만든다.
            # 자동화 경로는 이미 주행 중 root pose를 kinematic하게 적분하므로,
            # 정지 중에도 마지막 pose를 유지해 대기 위치의 creep/jitter를 막는다.
            if self._stationary_pose is None:
                position, orientation = self._robot.get_world_pose()
                self._stationary_pose = (
                    np.asarray(position, dtype=float).copy(),
                    np.asarray(orientation, dtype=float).copy(),
                )
            self._robot.set_linear_velocity(np.zeros(3, dtype=float))
            self._robot.set_angular_velocity(np.zeros(3, dtype=float))
            self._robot.set_world_pose(
                position=self._stationary_pose[0],
                orientation=self._stationary_pose[1],
            )
            self._kinematic_moving = False

    def joint_report(self) -> str:
        q = np.asarray(self._robot.get_joint_positions(), dtype=float)
        return (f"{self.kind} fork={q[self._lift_i]:.3f}m "
                f"steer={np.degrees(q[self._steer_i[0]]):.0f}deg "
                f"drive_vel={self._drive_vel:.1f}")
