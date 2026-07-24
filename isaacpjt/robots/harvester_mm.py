# -*- coding: utf-8 -*-
"""mm 수확 MM — 스쿱 스택(harvester_moveit)에서 **팔만 m0617 로 갈아끼운 것**.

moveit_mm(UR10e + 동축 스쿱)과 그리퍼·카메라·마찰·충돌필터를 전부 공유하고,
팔에 종속된 세 가지만 재정의한다(2026-07-24 사용자: "팔 부분만 갈아끼워"):

  1) HOME_POSE       — m0617 관절 이름(joint_1..6)과 홈 각
  2) ARM_LINKS       — m0617 링크 사슬(flat 형제 link_1..6)
  3) _attach_gripper — m0617 엔 UR 의 ee_joint(툴 소켓)가 없다. 플랜지 link_6 과
                       스쿱 Base 를 잇는 FixedJoint 를 직접 만든다(harvester.py 와 동일 근거,
                       probe 2026-07-22: 마지막 조인트=joint_6, 플랜지=link_6).

에셋 경로는 pjt_config/settings_mm.py 가 준다(팔=m0617, 그리퍼=동축 스쿱 USD).
"""
from __future__ import annotations

from pxr import Gf, Usd, UsdGeom, UsdPhysics

from pjt_utils.xform import set_pose
from robots.harvester_moveit import HarvestMM as _ScoopMM

# m0617 홈 자세 — robots/harvester.py 와 같은 값(사용자가 GUI 텔레옵으로 확정한 자세).
# 근위→원위 순서 유지(_preset_joint 가 원위 링크를 통째로 돌리므로 순서가 결과를 바꾼다).
HOME_POSE_DEG = (("joint_1", 0.0),
                 ("joint_2", 0.0),
                 ("joint_3", 60.0),
                 ("joint_4", 0.0),
                 ("joint_5", 75.0),
                 ("joint_6", -90.0))
# probe(2026-07-22): flat 형제 /Arm/link_1..6, 조인트 /Arm/joints/joint_1..6.
_ARM_LINKS = ("link_1", "link_2", "link_3", "link_4", "link_5", "link_6")
_ARM_DISTAL_FROM = {"joint_1": 0, "joint_2": 1, "joint_3": 2,
                    "joint_4": 3, "joint_5": 4, "joint_6": 5}


class HarvestMM(_ScoopMM):
    """Ridgeback + m0617 + 동축 3축 1/4구 스쿱 + D455."""

    HOME_POSE = HOME_POSE_DEG
    ARM_LINKS = _ARM_LINKS
    ARM_DISTAL_FROM = _ARM_DISTAL_FROM

    def _attach_gripper(self, stage: Usd.Stage, arm_path: str,
                        gripper_path: str, log) -> str | None:
        """동축 스쿱 Base 를 m0617 플랜지(link_6)에 새 FixedJoint 로 웰드한다.

        UR10e 판(부모 클래스)은 팔 에셋이 들고 있는 ee_joint 의 body1 을 스쿱으로
        채우면 끝이지만, m0617 엔 그 툴 소켓 조인트가 없다. 그래서
          1) 스쿱 강체 Base 를 찾고
          2) link_6 월드 포즈 = 툴 소켓(접근축 = link_6 +Z, Doosan 플랜지 관례)
          3) tool-Z 둘레 180° 로 U자 수용부를 아래로 향하게 놓고
          4) link_6 ↔ Base FixedJoint 를 직접 만든다(프레임은 실제 상대 포즈로).
        마지막에 부모와 똑같이 스쿱 내부 강체끼리만 충돌을 필터링한다(동축 셸이
        서로 스치며 회전 → 내부 접촉이 생기면 팔 관절값이 발산).
        """
        grip_base = None
        for prim in Usd.PrimRange(stage.GetPrimAtPath(gripper_path)):
            if (prim.GetName() == "Base"
                    and prim.HasAPI(UsdPhysics.RigidBodyAPI)):
                grip_base = str(prim.GetPath())
                break
        flange_path = f"{arm_path}/link_6"
        flange = stage.GetPrimAtPath(flange_path)
        if grip_base is None or not flange.IsValid():
            log("[Harvester] ⚠ link_6 또는 스쿱 Base 를 못 찾음 — 미장착")
            return grip_base

        # 스쿱 USD 가 독립 로봇으로 저작돼 있으면 아티큘레이션 루트가 남는다. 팔
        # 아티큘레이션에 합류시키려면 제거해야 한다(부모 클래스와 동일).
        for prim in Usd.PrimRange(stage.GetPrimAtPath(gripper_path)):
            if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
                prim.RemoveAPI(UsdPhysics.ArticulationRootAPI)

        cache = UsdGeom.XformCache()
        m_socket = cache.GetLocalToWorldTransform(flange)
        self._tool0_m = Gf.Matrix4d(m_socket)
        mount_roll = Gf.Matrix4d()
        mount_roll.SetRotate(Gf.Rotation(Gf.Vec3d(0.0, 0.0, 1.0), 180.0))
        m_mount = mount_roll * m_socket          # 로컬 tool-Z 180°: U자 바닥을 아래로

        # 컨테이너를 옮겨 CAD 원점(Base)이 회전된 장착 소켓과 정확히 일치하게 한다.
        # ★ 로컬 op 값은 부모 프레임으로 변환해야 한다: L' = C·G⁻¹·S·P⁻¹ (row-vector).
        #   G⁻¹·S 만 쓰면 루트가 원점일 때만 맞아, 스폰 위치가 (0,−12) 면 그리퍼가
        #   12m 밖에 붙어 로봇이 분해된다(§8 2026-07-19).
        m_grip = cache.GetLocalToWorldTransform(stage.GetPrimAtPath(grip_base))
        m_cont = cache.GetLocalToWorldTransform(stage.GetPrimAtPath(gripper_path))
        m_parent = cache.GetLocalToWorldTransform(
            stage.GetPrimAtPath(gripper_path).GetParent())
        m_local = m_cont * m_grip.GetInverse() * m_mount * m_parent.GetInverse()
        set_pose(stage.GetPrimAtPath(gripper_path),
                 m_local.ExtractTranslation(),
                 m_local.ExtractRotationQuat())

        fj = UsdPhysics.FixedJoint.Define(
            stage, f"{arm_path}/joints/gripper_fixed_joint")
        pos, rot = self._rel_pose(stage, flange_path, grip_base)
        fj.CreateBody0Rel().SetTargets([flange_path])
        fj.CreateBody1Rel().SetTargets([grip_base])
        fj.CreateLocalPos0Attr().Set(pos)
        fj.CreateLocalRot0Attr().Set(rot)
        fj.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0))
        fj.CreateLocalRot1Attr().Set(Gf.Quatf(1.0))
        fj.CreateJointEnabledAttr().Set(True)
        fj.CreateExcludeFromArticulationAttr().Set(False)

        scoop_bodies = [
            p for p in Usd.PrimRange(stage.GetPrimAtPath(gripper_path))
            if p.HasAPI(UsdPhysics.RigidBodyAPI)
        ]
        filtered = 0
        for i, body_a in enumerate(scoop_bodies):
            pairs = UsdPhysics.FilteredPairsAPI.Apply(body_a)
            rel = pairs.CreateFilteredPairsRel()
            for body_b in scoop_bodies[i + 1:]:
                rel.AddTarget(body_b.GetPath())
                filtered += 1
        log(f"[Harvester] 동축 스쿱 장착: link_6 → {grip_base} "
            f"(tool-Z 180°; U자 수용부 아래)")
        log(f"[Harvester] 동축 스쿱 내부 충돌 필터: {filtered}쌍 "
            f"(과실/줄기 외부 충돌은 유지)")
        return grip_base
