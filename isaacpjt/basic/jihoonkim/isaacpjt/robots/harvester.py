# -*- coding: utf-8 -*-
"""수확 모바일 매니퓰레이터 — 베이스 + 팔 + 그리퍼 + 커터를 조립해 씬에 놓는다.

★ GPU 에서 한 번도 안 돌려봤다. 에셋 경로·조인트 배치 전부 미검증이다. ★
  먼저 `spikes/03_asset_check.py` 로 에셋이 실제로 있는지 확인할 것.

왜 조립하는가 — 기본 제공 RidgebackUr 을 그대로 못 쓴다:
  통로 중앙에 서면 과실까지 수평 0.66m 를 쓴다(조간 1.5m). 그러면 UR5 의 도달
  0.85m 중 남는 수직이 ±0.54m 뿐이라 최대 1.0m 까지만 올라간다. 과실은 1.4m 까지
  있다 -> **0.4m 부족.** Franka(0.85m)도 0.23m 부족.
  UR10e(1.3m)면 1.6m 까지 닿는다. 그래서 베이스와 팔을 따로 불러 얹는다.
  (settings.py RobotConfig 의 도달 검산 참고)

커터를 다는 이유 [W2024]:
  인장(당기기) 수확은 손상 확률 최대. 전단 성공률 100% vs 굽힘 42.83% -> shear 권장.
  v3 6.2 하드웨어에는 그리퍼만 있었다 = 당겨서 뜯는다는 뜻이 된다.

이 모듈은 **놓기만 한다.** 모션 제어(Nav2/MoveIt2/RMPflow)는 별도다 — CLAUDE.md 가
로봇 로직과 제어를 분리하라고 한다.
"""
from __future__ import annotations

from pxr import Gf, Usd, UsdGeom, UsdPhysics

from pjt_config.settings import RobotConfig
from pjt_utils.xform import set_pose, set_translate
from robots import assets

CUTTER_COLOR = Gf.Vec3f(0.80, 0.82, 0.85)


class HarvestMM:
    """수확 MM. 베이스(Ridgeback) + 팔(UR10e) + 그리퍼(Robotiq) + 커터.

    구조:
        {root}/Base      <- 이동 베이스
        {root}/Arm       <- 팔. 베이스 위 arm_mount_z 에 고정 조인트로 붙는다
        {root}/Gripper   <- 그리퍼. 팔 끝(tool0)에 붙는다
        …/Gripper/…/base_link/Cutter <- 커터. 파지점 위 cutter_offset_z, 계층 자식(조인트 X, §8)
    """

    def __init__(self, cfg: RobotConfig):
        self._cfg = cfg
        self._root: str | None = None

    @property
    def root(self) -> str | None:
        return self._root

    def spawn(self, stage: Usd.Stage, root: str = "/World/Harvester",
              position: tuple[float, float, float] = (0.0, 0.0, 0.0),
              log=print) -> str:
        """조립해서 놓는다. 반환: root 경로."""
        from isaacsim.core.utils.stage import add_reference_to_stage

        self._root = root
        UsdGeom.Xform.Define(stage, root)
        UsdGeom.Xformable(stage.GetPrimAtPath(root)).AddTranslateOp().Set(
            Gf.Vec3d(*position))

        a = self._cfg.assets
        base_url = assets.resolve(a.base, "MM 베이스(Ridgeback)")
        arm_url = assets.resolve(a.arm, "팔(UR10e)")
        log(f"[Harvester] 베이스 {base_url}")
        log(f"[Harvester] 팔     {arm_url}")

        base_path = f"{root}/Base"
        arm_path = f"{root}/Arm"
        add_reference_to_stage(base_url, base_path)
        # RidgebackUr 에는 UR5 가 붙어 있다 (팔 없는 베이스는 서버에 없음 — §8).
        # 끄지 않으면 UR10e 와 팔이 2개가 되고 관절이 충돌해 솔버가 떤다.
        self._deactivate_ur5_arm(stage, base_path, log)
        add_reference_to_stage(arm_url, arm_path)

        # 팔을 베이스 위에 올린다. arm_mount_z = Ridgeback 높이(0.30m).
        # 참조로 얹은 UR10e prim 은 자체 xformOp 을 이미 갖고 있어 AddTranslateOp 가
        # 중복으로 터진다 → 기존 op 재사용 (CLAUDE.md §8 2026-07-18).
        set_translate(stage.GetPrimAtPath(arm_path),
                      (0.0, 0.0, self._cfg.arm_mount_z))
        # 조인트는 "만들면 붙는" 게 아니다 — 프레임을 현재 상대 포즈로 맞춰야 한다(§8).
        self._mount_arm(stage, base_path, arm_path, log)

        # 그리퍼 — 팔 에셋이 제공하는 툴 소켓(ee_joint)에 물린다.
        gripper_url = assets.resolve(a.gripper, "그리퍼(Robotiq)")
        gripper_path = f"{root}/Gripper"
        add_reference_to_stage(gripper_url, gripper_path)
        grip_base = self._attach_gripper(stage, arm_path, gripper_path, log)

        # 커터·카메라 — 조인트가 아니라 **계층**으로 붙인다 (§8).
        if grip_base:
            self._add_cutter(stage, grip_base + "/Cutter")
            self._add_camera(stage, grip_base + "/Camera", log)
        log(f"[Harvester] 조립 완료: {root}")
        return root

    # ----- 내부 -----

    def _deactivate_ur5_arm(self, stage: Usd.Stage, base_path: str, log) -> None:
        """베이스 에셋에 딸려온 UR5 팔을 끈다 (링크 prim + 어깨 조인트).

        구조는 2026-07-18 탐침으로 확인: 링크는 루트 바로 아래 `ur_arm_*` 형제,
        어깨 조인트는 `base_link/ur_arm_shoulder_pan_joint`.
        """
        base = stage.GetPrimAtPath(base_path)
        n = 0
        for prim in base.GetChildren():
            if prim.GetName().startswith("ur_arm_"):
                prim.SetActive(False)
                n += 1
        joint = stage.GetPrimAtPath(
            f"{base_path}/base_link/ur_arm_shoulder_pan_joint")
        if joint.IsValid():
            joint.SetActive(False)
            n += 1
        if n:
            log(f"[Harvester] 베이스의 UR5 팔 비활성화 ({n}개 prim)")
        else:
            log("[Harvester] ⚠ UR5 팔 prim 을 못 찾음 — 에셋 구조가 바뀌었나? "
                "팔이 2개로 보이면 base 하위 prim 이름을 확인할 것.")

    @staticmethod
    def _rel_pose(stage: Usd.Stage, from_path: str,
                  to_path: str) -> tuple[Gf.Vec3f, Gf.Quatf]:
        """to prim 의 포즈를 from prim 로컬 좌표로. 조인트 프레임 계산용."""
        cache = UsdGeom.XformCache()
        m_f = cache.GetLocalToWorldTransform(stage.GetPrimAtPath(from_path))
        m_t = cache.GetLocalToWorldTransform(stage.GetPrimAtPath(to_path))
        rel = m_t * m_f.GetInverse()
        return (Gf.Vec3f(rel.ExtractTranslation()),
                Gf.Quatf(rel.ExtractRotationQuat()))

    def _mount_arm(self, stage: Usd.Stage, base_path: str, arm_path: str,
                   log) -> None:
        """UR10e 의 root_joint(월드 고정)를 Ridgeback 섀시로 재배선.

        그대로 두면 팔이 월드 원점에 고정돼 있어, 베이스 위로 옮긴 것과 싸우다
        시작 순간 날아간다(§8). 스톡 RidgebackUr 이 UR5 를 base_link 에 물린
        것과 같은 패턴으로 만들고, 아티큘레이션 루트는 베이스 쪽 하나만 남긴다.
        """
        chassis = f"{base_path}/base_link"
        arm_base = f"{arm_path}/base_link"
        rj = stage.GetPrimAtPath(f"{arm_path}/root_joint")
        if not rj.IsValid() or not stage.GetPrimAtPath(chassis).IsValid():
            log("[Harvester] ⚠ root_joint 또는 섀시 base_link 없음 — 장착 실패. "
                "에셋 구조를 탐침으로 확인할 것 (RESULTS.md 2026-07-18).")
            return
        rj.RemoveAPI(UsdPhysics.ArticulationRootAPI)
        j = UsdPhysics.Joint(rj)
        j.GetBody0Rel().SetTargets([chassis])
        j.GetBody1Rel().SetTargets([arm_base])
        pos, rot = self._rel_pose(stage, chassis, arm_base)
        j.CreateLocalPos0Attr().Set(pos)
        j.CreateLocalRot0Attr().Set(rot)
        j.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0))
        j.CreateLocalRot1Attr().Set(Gf.Quatf(1.0))
        log(f"[Harvester] 팔 장착: root_joint → {chassis} "
            f"(섀시 좌표 오프셋 {tuple(round(v, 3) for v in pos)})")

    def _attach_gripper(self, stage: Usd.Stage, arm_path: str,
                        gripper_path: str, log) -> str | None:
        """그리퍼를 팔의 ee_joint(툴 소켓, b0=wrist_3)에 물린다.

        1) 그리퍼 강체 base_link 를 찾고
        2) ee_joint 의 b0 프레임(툴 플랜지) 월드 포즈를 계산해 그 자리에 놓고
        3) ee_joint 의 b1 을 그리퍼 base_link 로 채운다 (프레임 identity).
        그리퍼의 자체 아티큘레이션 루트는 제거 — 전체가 한 아티큘레이션이 된다.
        """
        grip_base = None
        for prim in Usd.PrimRange(stage.GetPrimAtPath(gripper_path)):
            if (prim.GetName() == "base_link"
                    and prim.HasAPI(UsdPhysics.RigidBodyAPI)):
                grip_base = str(prim.GetPath())
                break
        ee = stage.GetPrimAtPath(f"{arm_path}/joints/ee_joint")
        if grip_base is None or not ee.IsValid():
            log("[Harvester] ⚠ ee_joint 또는 그리퍼 base_link 를 못 찾음 — "
                "그리퍼 미장착. 에셋 구조를 탐침으로 확인할 것.")
            return grip_base

        for prim in Usd.PrimRange(stage.GetPrimAtPath(gripper_path)):
            if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
                prim.RemoveAPI(UsdPhysics.ArticulationRootAPI)

        j = UsdPhysics.Joint(ee)
        cache = UsdGeom.XformCache()
        wrist = str(j.GetBody0Rel().GetTargets()[0])
        m_wrist = cache.GetLocalToWorldTransform(stage.GetPrimAtPath(wrist))
        p0 = j.GetLocalPos0Attr().Get() or Gf.Vec3f(0.0)
        q0 = j.GetLocalRot0Attr().Get() or Gf.Quatf(1.0)
        l0 = Gf.Matrix4d()
        l0.SetTransform(Gf.Rotation(Gf.Quatd(q0)), Gf.Vec3d(p0))
        m_socket = l0 * m_wrist                      # 툴 소켓의 월드 포즈

        # 그리퍼 컨테이너를 옮겨 base_link 가 소켓 위치에 오게 한다
        m_grip = cache.GetLocalToWorldTransform(stage.GetPrimAtPath(grip_base))
        m_container = m_grip.GetInverse() * m_socket
        set_pose(stage.GetPrimAtPath(gripper_path),
                 m_container.ExtractTranslation(),
                 m_container.ExtractRotationQuat())

        j.GetBody1Rel().SetTargets([grip_base])
        j.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0))
        j.CreateLocalRot1Attr().Set(Gf.Quatf(1.0))
        j.CreateJointEnabledAttr().Set(True)
        j.CreateExcludeFromArticulationAttr().Set(False)
        log(f"[Harvester] 그리퍼 장착: ee_joint → {grip_base}")
        return grip_base

    def _add_cutter(self, stage: Usd.Stage, path: str) -> None:
        """커터 날. 그리퍼 base_link 의 **자식 prim** — 조인트 없이 계층으로 붙는다(§8).

        칼날이 실제로 절삭하는 물리는 범위 밖이다(절삭 시뮬레이션).
        위치만 맞으면 조인트를 끊는다 -> scene/pedicel.py cut().
        그래서 콜라이더도 강체도 없다 — 콜라이더는 꽃자루를 밀어내고,
        강체+조인트는 시작 순간 폭발을 다시 부른다(§8). 시각+거리판정 전용.
        **잡는 건 진짜 마찰이어야 한다** — 그쪽은 우회하지 않는다.
        """
        ee = self._cfg.end_effector
        blade = UsdGeom.Cube.Define(stage, path)   # 새 prim — AddOp 안전(§8은 참조 prim 얘기)
        blade.CreateSizeAttr(1.0)
        blade.CreateDisplayColorAttr([CUTTER_COLOR])
        xf = UsdGeom.Xformable(blade.GetPrim())
        # 파지점(접근축 +Z 로 grasp_reach_z)에서 꽃자루 쪽(+Y 로 cutter_offset_z).
        # 이전엔 (0,0,offset) 이라 손끝보다 뒤 몸통에 파묻혔다 — 축이 틀렸다(§8/RESULTS).
        xf.AddTranslateOp().Set(
            Gf.Vec3d(0.0, ee.cutter_offset_z, ee.grasp_reach_z))
        xf.AddScaleOp().Set(Gf.Vec3f(0.03, 0.002, 0.01))

    def _add_camera(self, stage: Usd.Stage, path: str, log) -> None:
        """손끝 카메라(RealSense D455). 그리퍼 base_link 자식 — 계층 부착(§8).

        시선은 접근축(base_link +Z, 손가락 방향 — 2026-07-18 축 탐침).
        에셋 안의 컬러 카메라는 자체 회전을 갖고 있어 방향을 가정하면 틀린다
        (Y축 180° 가정 → 롤 90° 오류, 렌더로 확인). 대신 **에셋 내부 카메라의
        상대 회전을 읽어** 컨테이너 보정 회전을 계산한다 — 에셋 관례 무관.
        강체·콜라이더 API 는 제거 — base_link(강체) 밑에 중첩 강체가 있으면
        PhysX 가 xformstack reset 에러를 낸다 (disable 로는 안 조용해짐).
        """
        from isaacsim.core.utils.stage import add_reference_to_stage

        url = assets.resolve(self._cfg.assets.camera, "카메라(RealSense D455)")
        add_reference_to_stage(url, path)
        container = stage.GetPrimAtPath(path)
        for prim in Usd.PrimRange(container):
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                prim.RemoveAPI(UsdPhysics.RigidBodyAPI)
            if prim.HasAPI(UsdPhysics.CollisionAPI):
                prim.RemoveAPI(UsdPhysics.CollisionAPI)

        cams = [p for p in Usd.PrimRange(container) if p.IsA(UsdGeom.Camera)]
        cam = next((p for p in cams if "Color" in p.GetName()),
                   cams[0] if cams else None)
        if cam is None:
            log(f"[Harvester] ⚠ 카메라 에셋에 Camera prim 없음: {url}")
            return
        # A = 카메라의 컨테이너 상대 회전 (에셋 내부 체인, 포즈 무관)
        cache = UsdGeom.XformCache()
        rel = (cache.GetLocalToWorldTransform(cam)
               * cache.GetLocalToWorldTransform(container).GetInverse())
        a = rel.ExtractRotationMatrix()
        # 파지점을 내려다보게 look-at. 카메라 위치(eye)에서 파지점(target)을 향하는
        # 방향 f 를 로컬에서 구하고, 카메라 -Z(시선)·+Y(상)를 (f, up) 에 맞춘다.
        # 방향을 가정(Y축 180°)하면 롤 오류가 났었다(렌더 확인) → look-at 으로 계산.
        ee = self._cfg.end_effector
        eye = Gf.Vec3d(*ee.camera_offset)
        target = Gf.Vec3d(0.0, 0.0, ee.grasp_reach_z)
        f = (target - eye).GetNormalized()          # 시선(로컬)
        up0 = Gf.Vec3d(0, 1, 0)
        right = Gf.Cross(f, up0).GetNormalized()
        up = Gf.Cross(right, f)
        # 카메라 규약(-Z=시선, +Y=상, +X=우) → 로컬축. r_look 행벡터 = 각 카메라축의 로컬 표현
        r_look = Gf.Matrix3d(right[0], right[1], right[2],
                             up[0], up[1], up[2],
                             -f[0], -f[1], -f[2])
        rc = Gf.Matrix4d().SetRotate(a.GetInverse() * r_look)
        set_pose(container, ee.camera_offset, rc.ExtractRotationQuat())
        log(f"[Harvester] 카메라 장착: {path} (파지점 {ee.grasp_reach_z}m look-at)")

    def cutter_world_pos(self, stage: Usd.Stage) -> tuple[float, float, float] | None:
        """커터 날의 현재 월드 좌표. 절단 판정에 쓴다 (cut_tolerance 안이면 성공)."""
        if self._root is None:
            return None
        root = stage.GetPrimAtPath(self._root)
        for prim in Usd.PrimRange(root):
            if prim.GetName() == "Cutter":
                m = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(
                    Usd.TimeCode.Default())
                t = m.ExtractTranslation()
                return (float(t[0]), float(t[1]), float(t[2]))
        return None
