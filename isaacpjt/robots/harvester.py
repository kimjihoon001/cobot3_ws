# -*- coding: utf-8 -*-
"""수확 모바일 매니퓰레이터 — 베이스 + 팔 + RG2 + RealSense를 조립해 씬에 놓는다.

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

import math
import os

from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

from pjt_config.settings import RobotConfig
from pjt_utils.xform import set_pose, set_translate
from robots import assets

# [4] 시작 자세 — 에셋 기본(q=0)은 팔이 **수직으로 곧게** 선다(link_6 z≈1.85m, probe 실측).
# 수확엔 팔을 앞으로 눕혀 툴이 식물 쪽을 향해야 하므로 조인트를 굽힌다. 아래는 **시드값**
# 이고 실각은 GUI 텔레옵(--mm-teleop, J/L 관절선택 · I/K 조그)으로 원하는 자세를 잡아
# 읽은 뒤 확정할 것(UR10e 때도 2026-07-20 사용자가 이렇게 지정). 근위→원위 순서 유지.
HOME_POSE_DEG = (("joint_1", 0.0),
                 # [사용자] J2=0°로 상완을 베이스 위 수직으로 세운다.
                 # J3를 양수로 접어 전완이 베이스 전방(+X)을 향하게 하고,
                 # J5로 보상해 기존 카메라/TCP 광축(+X, 하향 30°)은 유지한다.
                 ("joint_2", 0.0),
                 ("joint_3", 60.0),
                 ("joint_4", 0.0),
                 # 기존 60°에서 손목을 아래로 15° 더 내린 홈 자세.
                 ("joint_5", 75.0),
                 # [사용자] TCP 방향(link_6 +Z)을 바라볼 때 기준 반시계 90° = +Z 축 −90°
                 # (오른손 법칙: 시선이 축과 같은 방향이면 양의 회전이 시계방향).
                 # 렌더에서 반대로 돌면 +90 으로 뒤집을 것.
                 ("joint_6", -90.0))
# m0617 링크 사슬(근위→원위). 조인트를 돌릴 때 그 아래 원위 링크를 전부 같이 돌린다.
# probe(2026-07-22): flat 형제 /Arm/link_1..6, 조인트 /Arm/joints/joint_1..6.
_ARM_LINKS = ("link_1", "link_2", "link_3", "link_4", "link_5", "link_6")
_ARM_DISTAL_FROM = {"joint_1": 0, "joint_2": 1, "joint_3": 2,
                    "joint_4": 3, "joint_5": 4, "joint_6": 5}

CUTTER_COLOR = Gf.Vec3f(0.80, 0.82, 0.85)
BLADE_COLOR = Gf.Vec3f(0.72, 0.76, 0.80)

# 서보 단일-날 커터 치수·동작 — 전부 [4] 임의값. 절삭 물리는 시뮬 안 한다(§5.1: 절단=조인트
# 끊기, 날 닫힘은 연출). 실제 크기·각은 GPU 에서 눈으로 확인해 조정.
_BLADE_LEN = 0.032          # m   날 길이
_BLADE_W = 0.009            # m   날 폭
_BLADE_T = 0.0015           # m   날 두께
_ANVIL_W = 0.010            # m   앤빌(고정 날받침) 폭
_ANVIL_T = 0.002            # m   앤빌 두께
_HINGE_R = 0.004            # m   서보 피벗 반지름
_HINGE_L = 0.012            # m   서보 피벗 길이
_BLADE_OPEN_DEG = 45.0      # deg 날 들림(열림); 닫힘=0(앤빌에 닿음)
_CUTTER_MASS = 0.01         # kg  날·마운트 질량 — 콜라이더가 없어 명시(없으면 PhysX 가 경고)
# 톱날(원형 톱) + 커플러 — 전부 [4] 임의값.
_SAW_R = 0.022              # m   톱날 반지름
_SAW_T = 0.0015             # m   톱날 두께
_SAW_SPIN = 720.0          # deg/s 톱날 회전 속도(연출)
_COUPLER_R = 0.025         # m   커플러(플랜지 어댑터) 반지름 ≈ Ø50
_COUPLER_H = 0.018         # m   커플러 길이

# TCP 축을 중심으로 현재 장착 자세에서 RG2를 180° 뒤집는다.
_GRIPPER_ROLL_DEG = 180.0
# D455의 긴 본체가 RG2 넓은 면에서 가로(ㅡ)를 유지하면서 상하가 바로 서도록
# 광축 기준 180° 롤을 적용한다. 90°/270°는 카메라를 세로(ㅣ)로 세우므로 쓰지 않는다.
_CAMERA_ROLL_DEG = 180.0

# ── 사용자 CAD 커터 지그 (FreeCAD build_harvest_eef_jig.py → robots/cad_jig/*.usd) ──
# 커플러 링 + 서보 가위 커터 + D455 마운트 일체형. CAD 프레임: 원점=플랜지 접촉면,
# +Z=접근, +Y=위, +X=손가락 (스크립트 docstring).
_CAD_JIG_DIR = os.path.join(os.path.dirname(__file__), "cad_jig")
_CAD_SCALE = 0.1              # 변환기 metersPerUnit≈0.01 스탬프 → ×0.1 = 실측 mm(§8 동일)
_COUPLER_T = 0.012           # [1] 커플러 두께(COUPLER_T) — 그리퍼를 접근축으로 이만큼 밀어
                             #     커플러가 플랜지↔그리퍼 사이에 들어가게(사용자 지적)
_CAD_CAM_EULER = (0.0, -78.0, 0.0)  # D455 로컬 회전 XYZ(deg, base_link 기준) — 사용자 GUI 확정
# 실물 D455 위치는 CAD camera_dummy prim 을 그대로 참조해 월드좌표를 읽어 쓴다
# (손 계산 시 부호가 꼬였음 — CAD 가 인코딩한 값을 신뢰, §5.7).
# CAD→그리퍼(툴0) 상대 방향 보정: X −90° + Y −90° (2026-07-19 사용자 렌더 확정)


class HarvestMM:
    """수확 MM. 베이스(Ridgeback) + 팔(m0617) + OnRobot RG2 + D455.

    구조:
        {root}/Base      <- 이동 베이스
        {root}/Arm       <- 팔. 베이스 위 arm_mount_z 에 고정 조인트로 붙는다
        {root}/Gripper   <- 그리퍼. 팔 끝(tool0)에 붙는다
        …/Gripper/…/gripper_body/D455 <- 그리퍼 위 eye-in-hand 카메라
    """

    # 가동날 각도 [deg] — 서보 힌지 리볼루트 조인트 드라이브 목표.
    # 조인트 rest(0°) = blade_dummy 익스포트 자세 = CAD OPEN_ANGLE(-35°, 열림). 거기서
    # +35° 돌리면 CAD 0°(닫힘=노치 전단). 그래서 열림 0° ~ 닫힘 35°(CAD build 스크립트 규약).
    BLADE_OPEN_DEG = 0.0          # 열림 (날이 옆으로 펼쳐짐)
    BLADE_CLOSED_DEG = 35.0       # 닫힘 = 노치로 줄기 전단

    def __init__(self, cfg: RobotConfig):
        self._cfg = cfg
        self._root: str | None = None
        self._grip_base: str | None = None       # 그리퍼 base_link (파지 관절 탐색·커터 고정)
        self._gripper_path: str | None = None     # 그리퍼 컨테이너 (finger 관절 탐색 루트)
        self._hinge_path: str | None = None       # 커터 마운트(=절단점) prim
        self._blade_joint: str | None = None       # 단일 날 RevoluteJoint 경로(= 서보)
        self._tool0_m: Gf.Matrix4d | None = None   # 툴0(플랜지) 월드 포즈 — CAD 지그 부착 기준
        self._grasp_tcp: str | None = None         # 실제 파지 중심 프레임
        self._blade_shaft_w: Gf.Vec3d | None = None  # 가동날 절단점(서보축) 월드좌표 (줄기 배치용)
        self._blade_path: str | None = None          # 가동날 프림 경로 (CadJig 자식 blade_dummy)
        self._blade_deg: float = self.BLADE_OPEN_DEG  # 현재 가동날 각도 [deg] (절단 게이트값)

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
        # 같은 Stage에서 재조립하더라도 이전 버전의 칼날/브라켓이 남지 않게 비활성화.
        self._disable_legacy_cutter(stage, root)
        UsdGeom.Xformable(stage.GetPrimAtPath(root)).AddTranslateOp().Set(
            Gf.Vec3d(*position))
        # (2026-07-22 되돌림: root 180° 회전은 base 이동 프레임을 뒤집어 로봇이 엉뚱한
        #  곳으로 순간이동함. MM facing 은 나중에 스폰 위치/nav 초기포즈로 처리.)

        a = self._cfg.assets
        base_url = assets.resolve(a.base, "MM 베이스(Ridgeback)")
        arm_url = assets.resolve(a.arm, "팔(m0617)")
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
        # 시작자세는 RmpMMDriver.configure()의 articulation default joint state에서만 준다.
        # 여기서 링크 xform까지 HOME_POSE_DEG로 미리 돌리고 joint state에도 같은 각을
        # 넣으면 실제 PhysX 체인은 홈 각이 이중 적용된다. 반면 Lula IK는 원본 URDF 체인을
        # 기준으로 해를 풀기 때문에, 올바른 base-frame 목표를 줘도 팔이 엉뚱한 방향으로
        # 움직인다. 정지 뷰포트 모양보다 런타임 기구학 일치를 우선한다.

        # OnRobot RG2 — 팔 플랜지(link_6)에 새 FixedJoint 로 웰드한다.
        gripper_url = assets.resolve(a.gripper, "그리퍼(OnRobot RG2)")
        gripper_path = f"{root}/Gripper"
        add_reference_to_stage(gripper_url, gripper_path)
        grip_base = self._attach_gripper(stage, arm_path, gripper_path, log)
        self._gripper_path = gripper_path
        self._grip_base = grip_base
        self._disable_legacy_cutter(stage, root, grip_base)
        if grip_base:
            # RMPflow 기본 EE(m0617 link_6)와 실제 손가락 파지 중심은 다르다.
            # 고정 길이를 카메라 광선에서 빼지 않고 이 프레임의 실제 월드 포즈로
            # EE→TCP 오프셋을 측정한다.
            self._grasp_tcp = f"{grip_base}/HarvestTCP"
            tcp = UsdGeom.Xform.Define(stage, self._grasp_tcp)
            tcp.AddTranslateOp().Set(Gf.Vec3d(
                self._cfg.end_effector.grasp_center_x,
                0.0,
                self._cfg.end_effector.grasp_reach_z))
            # HarvestTCP 프레임은 IK·거리 판정에만 사용하고 뷰포트 마커는 만들지 않는다.
            self._bind_gripper_friction(stage, gripper_path, log)
            # RG2 2지 그리퍼 → 흡착 그리퍼(겉모습 교체). 카메라보다 먼저 호출해야
            # (카메라는 이 뒤에 추가돼) D455가 숨김 처리에 걸리지 않는다.
            self._make_suction_gripper(stage, gripper_path, grip_base, log)

        # 커터/칼날/커터 브라켓은 사용하지 않는다. D455만 gripper_body 위에
        # 시각 전용으로 달고(같은 방향·방위 유지) HarvestTCP를 바라보게 한다.
        if grip_base:
            self._add_camera(stage, f"{grip_base}/D455", log)
        log(f"[Harvester] 조립 완료: {root}")
        return root

    # ----- 내부 -----

    @staticmethod
    def _disable_legacy_cutter(stage: Usd.Stage, root: str,
                               grip_base: str | None = None) -> None:
        """구형 칼날·칼날 브라켓 prim을 물리/렌더 트리에서 완전히 제외한다."""
        paths = [f"{root}/Cutter"]
        if grip_base:
            paths.extend((f"{grip_base}/CadJig", f"{grip_base}/CutterArm"))
        for path in paths:
            prim = stage.GetPrimAtPath(path)
            if prim.IsValid():
                prim.SetActive(False)

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

    def _preset_pose(self, stage: Usd.Stage, arm_path: str, log) -> None:
        """시작 자세(HOME_POSE_DEG)를 근위→원위 순서로 하나씩 굽는다."""
        for jname, deg in HOME_POSE_DEG:
            self._preset_joint(stage, arm_path, jname, deg, log)

    def _preset_joint(self, stage: Usd.Stage, arm_path: str, jname: str,
                      deg: float, log) -> None:
        """조인트 하나의 각도를 USD 링크 트랜스폼에 굽는다.

        왜 런타임 set_joints_default_state 로는 안 되나 (2026-07-20 사용자 지적):
          정지 중엔 physics view 가 없어 `set_joint_positions` 가 경고만 내고 끝난다
          (isaacsim/core/prims/impl/articulation.py). 그래서 +180° 는 PhysX 안에만
          있고 USD 는 에셋 원본(0°) 그대로 → Stop=0° / Play=180° 로 180° 차이가 난다.
          USD 조인트 초기각(JointStateAPI)만 써도 정지 뷰포트는 안 움직인다(physx
          프로퍼티 위젯도 정지 중엔 joint state 입력을 비활성화한다).
          → 링크 트랜스폼 자체를 돌려야 Play/Stop/Export 가 같은 자세가 된다.

        m0617 은 링크가 flat(형제) 구조라(probe 2026-07-22: link_1..6 전부 /Arm 바로
        아래, 조인트는 /Arm/joints/) 그 조인트의 원위 링크들을 조인트 축 둘레로 직접 돌린다.
        """
        joint = stage.GetPrimAtPath(f"{arm_path}/joints/{jname}")
        if not joint.IsValid():
            log(f"[Harvester] ⚠ {jname} 없음 — 시작자세 프리셋 스킵(기본자세로 뜬다)")
            return
        names = _ARM_LINKS[_ARM_DISTAL_FROM[jname]:]
        distal = [stage.GetPrimAtPath(f"{arm_path}/{n}") for n in names]
        missing = [n for n, pr in zip(names, distal) if not pr.IsValid()]
        if missing:   # 일부만 돌리면 팔이 끊겨 보인다 — 통째로 스킵
            log(f"[Harvester] ⚠ 링크 없음 {missing} — {jname} 프리셋 스킵")
            return
        j = UsdPhysics.Joint(joint)
        parent = stage.GetPrimAtPath(str(j.GetBody0Rel().GetTargets()[0]))
        cache = UsdGeom.XformCache()
        l0 = Gf.Matrix4d()
        l0.SetTransform(Gf.Rotation(Gf.Quatd(j.GetLocalRot0Attr().Get() or Gf.Quatf(1.0))),
                        Gf.Vec3d(j.GetLocalPos0Attr().Get() or Gf.Vec3f(0.0)))
        m_joint = l0 * cache.GetLocalToWorldTransform(parent)   # 조인트 프레임 월드포즈
        axis = {"X": Gf.Vec3d(1, 0, 0), "Y": Gf.Vec3d(0, 1, 0),
                "Z": Gf.Vec3d(0, 0, 1)}[
            UsdPhysics.RevoluteJoint(joint).GetAxisAttr().Get() or "Z"]
        p = m_joint.ExtractTranslation()
        R = (Gf.Matrix4d().SetTranslate(-p)
             * Gf.Matrix4d().SetRotate(Gf.Rotation(m_joint.TransformDir(axis), deg))
             * Gf.Matrix4d().SetTranslate(p))

        # 형제라 서로 영향이 없다 — 새 월드포즈를 먼저 다 구하고 나서 쓴다.
        new = [(pr, cache.GetLocalToWorldTransform(pr) * R
                * cache.GetLocalToWorldTransform(pr.GetParent()).GetInverse())
               for pr in distal]
        for pr, m in new:
            set_pose(pr, m.ExtractTranslation(), m.ExtractRotationQuat())

        # 조인트 초기각·드라이브 목표도 같은 값으로. 안 그러면 물리 시작 순간 드라이브가
        # 목표 0° 로 되돌리려 든다(에셋 기본 target=0).
        js = PhysxSchema.JointStateAPI.Apply(joint, "angular")
        js.CreatePositionAttr().Set(float(deg))
        js.CreateVelocityAttr().Set(0.0)
        UsdPhysics.DriveAPI(joint, "angular").CreateTargetPositionAttr().Set(float(deg))
        log(f"[Harvester] 시작자세 프리셋: {jname} {deg:+.0f}° — USD 에 고정(Play/Stop 동일)")

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

    def _bind_gripper_friction(self, stage: Usd.Stage, gripper_path: str, log) -> None:
        """RG2 손가락 rigid body/콜라이더에 파지용 마찰 재질을 건다."""
        from scene.physics import create_physics_material, bind_physics_material
        ee = self._cfg.end_effector
        mat = create_physics_material(
            stage, "/World/PhysicsMaterials/gripper_pad",
            ee.pad_static_friction, ee.pad_dynamic_friction)
        n = 0
        for p in Usd.PrimRange(stage.GetPrimAtPath(gripper_path)):
            if str(p.GetPath()) == self._grip_base:
                continue
            name = p.GetName().lower()
            is_finger_body = (
                p.HasAPI(UsdPhysics.RigidBodyAPI)
                and ("finger" in name or "knuckle" in name))
            if p.HasAPI(UsdPhysics.CollisionAPI) or is_finger_body:
                bind_physics_material(p, mat)
                n += 1
        log(f"[Harvester] 그리퍼 콜라이더 {n}개 마찰 바인딩 μs={ee.pad_static_friction} "
            "(RG2 finger/knuckle)")

    def _make_suction_gripper(self, stage: Usd.Stage, gripper_path: str,
                              grip_base: str, log) -> None:
        """RG2 2지 그리퍼를 흡착 그리퍼로 교체한다(시각/충돌).

        파지 물리는 그대로 웰드/분리를 쓴다(2지든 흡착이든 Isaac 에 진짜 흡착이 없어
        고정조인트로 흉내낸다 — 이미 mm.py 가 그 방식). 여기서는 **겉모습만** 흡착컵으로
        바꾼다: RG2 메시를 전부 숨기고 손가락 콜라이더를 꺼(과실을 치지 않게) grip_base
        앞에 흡착컵을 절차적으로 붙인다. TCP(=grasp_reach_z)가 컵 입구(끝)에 오도록 컵을
        그 앞에 맞춘다 — 흡착컵의 끝이 토마토 표면에 닿는 접근을 위해서다(사용자 2026-07-23).
        """
        ee = self._cfg.end_effector
        # 1) RG2 손가락을 **숨김 + 콜라이더 비활성**으로 사실상 제거한다. 관절을
        #    SetActive(False)로 죽이면 타임라인 Stop→Play 재파싱 때 아티큘레이션 DOF/기본
        #    자세가 틀어져 팔이 홈 대신 0(수직)으로 펴진다(2026-07-23 실측). 그래서 구조는
        #    건드리지 않고 메시만 숨기고 손가락 콜라이더만 꺼(과실을 안 치게) 흡착컵만 보인다.
        hidden, cols = 0, 0
        for p in Usd.PrimRange(stage.GetPrimAtPath(gripper_path)):
            if p.IsA(UsdGeom.Gprim):
                UsdGeom.Imageable(p).MakeInvisible()
                hidden += 1
            name = p.GetName().lower()
            if ("finger" in name or "knuckle" in name) and p.HasAPI(UsdPhysics.CollisionAPI):
                UsdPhysics.CollisionAPI(p).CreateCollisionEnabledAttr().Set(False)
                cols += 1

        # 2) 흡착컵 절차 생성 — grip_base 로컬(+Z=접근축, X=손가락 중심선).
        x = ee.grasp_center_x
        tip_z = ee.grasp_reach_z                 # 컵 입구(끝) = TCP 위치
        cup_root = f"{grip_base}/SuctionCup"
        UsdGeom.Xform.Define(stage, cup_root)
        # 진공 튜브 — 얇은 실린더.
        tube = UsdGeom.Cylinder.Define(stage, f"{cup_root}/Tube")
        tube.CreateAxisAttr("Z")
        tube.CreateRadiusAttr(0.010)
        tube.CreateHeightAttr(0.022)
        tube.CreateDisplayColorAttr([Gf.Vec3f(0.20, 0.20, 0.22)])
        UsdGeom.Xformable(tube.GetPrim()).AddTranslateOp().Set(
            Gf.Vec3d(x, 0.0, tip_z - 0.031))     # 중심 z=0.101 (0.090~0.112)
        # 흡착컵 — 원뿔(깔때기). 넓은 입구가 토마토(+Z)를 향하게 X축 180° 뒤집는다.
        cup = UsdGeom.Cone.Define(stage, f"{cup_root}/Cup")
        cup.CreateAxisAttr("Z")
        cup.CreateRadiusAttr(0.037)              # 입구 반경 37mm > 토마토 34mm → 살짝 감쌈
        cup.CreateHeightAttr(0.020)
        cup.CreateDisplayColorAttr([Gf.Vec3f(0.12, 0.12, 0.14)])
        cxf = UsdGeom.Xformable(cup.GetPrim())
        cxf.AddTranslateOp().Set(Gf.Vec3d(x, 0.0, tip_z - 0.010))   # 입구 평면 z=tip_z
        cxf.AddRotateXYZOp().Set(Gf.Vec3f(180.0, 0.0, 0.0))
        # 콜라이더는 안 붙인다 — 파지는 웰드가 하고, 컵 콜라이더는 접근 중 과실을 밀어낸다.
        log(f"[Harvester] 흡착 그리퍼로 교체: RG2 메시 {hidden}개 숨김, 손가락 콜라이더 "
            f"{cols}개 비활성, 흡착컵 tip z={tip_z:.3f} (=TCP)")

    def _attach_gripper(self, stage: Usd.Stage, arm_path: str,
                        gripper_path: str, log) -> str | None:
        """그리퍼를 팔 플랜지(link_6)에 새 FixedJoint 로 웰드한다.

        m0617 엔 UR 의 ee_joint(툴 소켓) 같은 여분 조인트가 없다(probe 2026-07-22:
        마지막 조인트=joint_6, 플랜지=link_6). 그래서 소켓을 link_6 프레임으로 잡고
        link_6 ↔ 그리퍼 base_link 를 잇는 FixedJoint 를 **직접 만든다**.
        1) 그리퍼 강체 base_link 를 찾고
        2) link_6(플랜지) 월드 포즈 = 툴 소켓. 접근축 = link_6 +Z (Doosan 플랜지 관례 —
           GUI 렌더로 확인, UR 은 wrist_3 +Y 였던 전례 있음 §isaac-sim-51-assets).
        3) 롤 보정 + 커플러 오프셋 후 그리퍼를 그 자리에 놓고
        4) link_6(b0) ↔ grip_base(b1) FixedJoint 생성 → 전체가 한 아티큘레이션.
        """
        weld_base = None
        tool_base = None
        for prim in Usd.PrimRange(stage.GetPrimAtPath(gripper_path)):
            if (prim.GetName() == "base_link"
                    and prim.HasAPI(UsdPhysics.RigidBodyAPI)):
                weld_base = tool_base = str(prim.GetPath())
                break
        # OnRobot RG2: 플랜지 장착부는 quick_changer, TCP/카메라 기준은 gripper_body.
        if weld_base is None:
            for prim in Usd.PrimRange(stage.GetPrimAtPath(gripper_path)):
                if (prim.GetName() == "quick_changer"
                        and prim.HasAPI(UsdPhysics.RigidBodyAPI)):
                    weld_base = str(prim.GetPath())
                elif (prim.GetName() == "gripper_body"
                      and prim.HasAPI(UsdPhysics.RigidBodyAPI)):
                    tool_base = str(prim.GetPath())
        flange_path = f"{arm_path}/link_6"
        flange = stage.GetPrimAtPath(flange_path)
        if weld_base is None or tool_base is None or not flange.IsValid():
            log("[Harvester] ⚠ link_6 또는 RG2 quick_changer/gripper_body 를 못 찾음 — "
                "그리퍼 미장착. 에셋 구조를 탐침으로 확인할 것.")
            return tool_base

        for prim in Usd.PrimRange(stage.GetPrimAtPath(gripper_path)):
            if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
                prim.RemoveAPI(UsdPhysics.ArticulationRootAPI)

        # RG2 USD는 독립 로봇용 world→quick_changer 고정조인트가 있다. m0617에 붙일 때는
        # 이를 끄고 quick_changer를 새 플랜지 조인트의 body1으로 쓴다.
        for prim in Usd.PrimRange(stage.GetPrimAtPath(gripper_path)):
            if prim.GetName() == "quick_changer_joint" and prim.IsA(UsdPhysics.Joint):
                prim.SetActive(False)
            elif prim.GetName() == "world" and prim.HasAPI(UsdPhysics.RigidBodyAPI):
                prim.SetActive(False)

        cache = UsdGeom.XformCache()
        m_socket = cache.GetLocalToWorldTransform(flange)   # link_6 플랜지 월드 포즈
        # 장착 롤 보정 — 그리퍼 base_link 프레임이 플랜지 접근축(Z) 기준 _GRIPPER_ROLL_DEG
        # 만큼 틀어져 손가락이 어긋난다(GUI 렌더로 맞춤). 롤은 translation 을 보존하고
        # orientation 만 돌린다(pxr row-vector: 순수회전 행렬의 마지막 행 = identity).
        m_socket = Gf.Matrix4d().SetRotate(
            Gf.Rotation(Gf.Vec3d(0, 0, 1), _GRIPPER_ROLL_DEG)) * m_socket

        # CAD 커플러 지그가 붙는 툴0 프레임(그리퍼 밀기 전)을 저장.
        self._tool0_m = Gf.Matrix4d(m_socket)
        # RG2 자체 quick changer가 있으므로 별도 CAD 커플러 두께는 더하지 않는다.
        approach = Gf.Vec3d(m_socket.TransformDir(Gf.Vec3d(0, 0, 1))).GetNormalized()
        is_rg2 = stage.GetPrimAtPath(weld_base).GetName() == "quick_changer"
        m_socket.SetTranslateOnly(
            m_socket.ExtractTranslation() + approach * (0.0 if is_rg2 else _COUPLER_T))

        # 그리퍼 컨테이너를 옮겨 base_link 가 (롤 보정 + 커플러 오프셋된) 소켓에 오게 한다.
        # ★ 로컬 op 에 쓸 값은 부모 프레임으로 변환해야 한다: L' = C·G⁻¹·S·P⁻¹ (row-vector,
        #   C=컨테이너 l2w, G=base_link l2w, S=목표 소켓 월드, P=부모 l2w). 예전 G⁻¹·S 는
        #   부모(하베스터 루트)가 원점일 때만 맞아, main.py 가 (0,−12) 스폰하자 그리퍼가
        #   12m 밖에 붙어 로봇이 분해됐다(§8 2026-07-19).
        m_grip = cache.GetLocalToWorldTransform(stage.GetPrimAtPath(weld_base))
        m_cont = cache.GetLocalToWorldTransform(stage.GetPrimAtPath(gripper_path))
        m_parent = cache.GetLocalToWorldTransform(
            stage.GetPrimAtPath(gripper_path).GetParent())
        m_local = m_cont * m_grip.GetInverse() * m_socket * m_parent.GetInverse()
        set_pose(stage.GetPrimAtPath(gripper_path),
                 m_local.ExtractTranslation(),
                 m_local.ExtractRotationQuat())

        # link_6 ↔ quick_changer FixedJoint 를 새로 만든다. 프레임은 실제 상대 포즈로
        # 준다(§8: 롤 보정 후 프레임을 안 맞추면 시작 순간 스냅). body0(플랜지) 로컬 기준.
        fj = UsdPhysics.FixedJoint.Define(stage, f"{arm_path}/joints/gripper_fixed_joint")
        pos, rot = self._rel_pose(stage, flange_path, weld_base)
        fj.CreateBody0Rel().SetTargets([flange_path])
        fj.CreateBody1Rel().SetTargets([weld_base])
        fj.CreateLocalPos0Attr().Set(pos)
        fj.CreateLocalRot0Attr().Set(rot)
        fj.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0))
        fj.CreateLocalRot1Attr().Set(Gf.Quatf(1.0))
        fj.CreateJointEnabledAttr().Set(True)
        fj.CreateExcludeFromArticulationAttr().Set(False)
        log(f"[Harvester] RG2 웰드: link_6 → {weld_base}; tool base={tool_base} "
            f"(롤 보정 {_GRIPPER_ROLL_DEG:.0f}°)")
        return tool_base

    def _add_cutter(self, stage: Usd.Stage, grip_base: str, log) -> None:
        """서보 구동 **단일 날** 커터 (cut-and-hold) — 손끝 날 1개 + 고정 앤빌.

        실물 참고: 서보 1개가 날을 앤빌로 눌러 줄기를 전단(iris-type cutting gripper
        [MDPI Actuators 2025 14(9):432], sweet-pepper 서보 커터). 레퍼런스 사진과 같은 단일 날.

        §5.1: 절삭 물리는 시뮬 안 한다. 날이 앤빌로 닫히는 건 **연출**이고 실제 분리는
        do_cut() 이 부르는 scene/pedicel.cut()(jointEnabled=False) 이다. 편법 아님 —
        꽃자루가 안 끊긴 상태를 모델링하다 끊는 순간부터 진짜 물리(마찰 파지)로 넘어간다.

        구조 (base_link 밑 강체 중첩 = xformstack 에러 §8 → root 밑 컨테이너 + 조인트로 부착):
            {root}/Cutter
              /Mount        실린더 강체 — 그리퍼에 FixedJoint (서보 하우징=피벗)
              /Mount/Anvil  고정 날받침(Mount 자식, 시각) — 날이 닿는 고정 edge
              /Blade        단일 날 강체 — Mount 에 RevoluteJoint(X축)+DriveAPI (= 서보)
        날·앤빌에 콜라이더 없음(§5.1). grip_base→Mount 시각 마운트 암도 붙인다.
        ★ RevoluteJoint+DriveAPI 는 GPU 검증 코드. 발산하면 RESULTS.md 기록.
        """
        ee = self._cfg.end_effector
        cache = UsdGeom.XformCache()
        m_grip = cache.GetLocalToWorldTransform(stage.GetPrimAtPath(grip_base))

        # 마운트를 파지점 **위(world +Z = 꽃자루 방향)**에 world 축 정렬로 놓는다. 그리퍼
        # 로컬 +Y 로 놓으면 그리퍼 롤에 따라 옆으로 돌아 파지를 막는다(2026-07-18 사용자
        # 렌더 지적: 커터가 옆으로 감). world 위로 직접 놓고 FixedJoint 프레임은 실제 상대
        # 포즈(§8)로 써서 그리퍼 롤과 무관하게 꽃자루 위에 온다.
        # 톱날 중심 = 줄기(파지점 위 꽃자루). 톱날 디스크가 줄기를 가로질러 회전하며 자른다.
        grasp_w = m_grip.Transform(Gf.Vec3d(0.0, 0.0, ee.grasp_reach_z))
        m_mount_w = Gf.Matrix4d(1.0)
        m_mount_w.SetTranslateOnly(Gf.Vec3d(grasp_w[0], grasp_w[1],
                                            grasp_w[2] + ee.cutter_offset_z))

        cutter_root = f"{self._root}/Cutter"
        UsdGeom.Xform.Define(stage, cutter_root)
        inv = cache.GetLocalToWorldTransform(
            stage.GetPrimAtPath(cutter_root)).GetInverse()   # 월드→컨테이너 로컬

        # --- Mount(서보 하우징=피벗) ---
        mount_path = f"{cutter_root}/Mount"
        mount = UsdGeom.Cylinder.Define(stage, mount_path)
        mount.CreateRadiusAttr(_HINGE_R)
        mount.CreateHeightAttr(_HINGE_L)
        mount.CreateAxisAttr("Z")                          # 피벗축(Z=수직)으로 선다
        mount.CreateExtentAttr([Gf.Vec3f(-_HINGE_R, -_HINGE_R, -_HINGE_L/2),
                                Gf.Vec3f(_HINGE_R, _HINGE_R, _HINGE_L/2)])
        mount.CreateDisplayColorAttr([CUTTER_COLOR])
        m_mount_l = m_mount_w * inv
        set_pose(mount.GetPrim(), m_mount_l.ExtractTranslation(),
                 m_mount_l.ExtractRotationQuat())
        self._make_body(mount.GetPrim())

        # --- Saw(원형 톱날) — Mount(=서보) 에 RevoluteJoint + **속도 드라이브로 회전**. 축 =
        # 그리퍼 손가락 방향(approach). 회전은 연출, 실제 절단은 do_cut→pedicel.cut. 콜라이더 X. ---
        approach = Gf.Vec3d(m_grip.TransformDir(Gf.Vec3d(0, 0, 1))).GetNormalized()
        disc_rot = Gf.Rotation(Gf.Vec3d(0, 0, 1), approach)     # 디스크 축(Z) → 손가락 방향
        saw_path = f"{cutter_root}/Saw"
        saw = UsdGeom.Cylinder.Define(stage, saw_path)
        saw.CreateRadiusAttr(_SAW_R)
        saw.CreateHeightAttr(_SAW_T)
        saw.CreateAxisAttr("Z")
        saw.CreateExtentAttr([Gf.Vec3f(-_SAW_R, -_SAW_R, -_SAW_T/2),
                              Gf.Vec3f(_SAW_R, _SAW_R, _SAW_T/2)])
        saw.CreateDisplayColorAttr([BLADE_COLOR])
        m_saw_w = Gf.Matrix4d()
        m_saw_w.SetTransform(disc_rot, m_mount_w.ExtractTranslation())
        m_saw_l = m_saw_w * inv
        set_pose(saw.GetPrim(), m_saw_l.ExtractTranslation(),
                 m_saw_l.ExtractRotationQuat())
        self._make_body(saw.GetPrim())

        jpath = f"{cutter_root}/SawJoint"
        rev = UsdPhysics.RevoluteJoint.Define(stage, jpath)
        rev.CreateBody0Rel().SetTargets([mount_path])
        rev.CreateBody1Rel().SetTargets([saw_path])
        rev.CreateAxisAttr("Z")
        # 조인트 프레임은 실제 상대 포즈로(§8) — Mount(수직축)와 Saw(손가락축)가 달라서 필수.
        p0, r0 = self._rel_pose(stage, mount_path, saw_path)
        rev.CreateLocalPos0Attr().Set(p0)
        rev.CreateLocalRot0Attr().Set(r0)
        rev.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0))
        rev.CreateLocalRot1Attr().Set(Gf.Quatf(1.0))
        drive = UsdPhysics.DriveAPI.Apply(rev.GetPrim(), "angular")
        drive.CreateTypeAttr("force")
        drive.CreateTargetVelocityAttr(_SAW_SPIN)          # deg/s 회전(연출)
        drive.CreateDampingAttr(50.0)

        # --- Mount 를 그리퍼에 FixedJoint (프레임=실제 상대 포즈, §8: world 배치 후 스냅 방지) ---
        fj = UsdPhysics.FixedJoint.Define(stage, f"{cutter_root}/MountJoint")
        fj.CreateBody0Rel().SetTargets([grip_base])
        fj.CreateBody1Rel().SetTargets([mount_path])
        mpos, mrot = self._rel_pose(stage, grip_base, mount_path)
        fj.CreateLocalPos0Attr().Set(mpos)
        fj.CreateLocalRot0Attr().Set(mrot)
        fj.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0))
        fj.CreateLocalRot1Attr().Set(Gf.Quatf(1.0))

        # 커터가 공중에 떠 보이지 않게, 그리퍼 몸통 → 마운트를 잇는 **마운트 암(시각 전용)**.
        # grip_base 자식(강체 없음 = 중첩 아님)이라 그리퍼와 같이 움직인다 (사용자 지적).
        arm = UsdGeom.Cube.Define(stage, grip_base + "/CutterArm")
        arm.CreateSizeAttr(1.0)
        arm.CreateDisplayColorAttr([CUTTER_COLOR])
        a_pt = Gf.Vec3d(0.0, 0.012, 0.0)                                # 그리퍼 몸통(위)
        b_pt = m_grip.GetInverse().Transform(
            m_mount_w.ExtractTranslation())                            # 마운트(그리퍼 로컬)
        d = b_pt - a_pt
        axf = UsdGeom.Xformable(arm.GetPrim())
        axf.AddTranslateOp().Set((a_pt + b_pt) / 2.0)
        axf.AddOrientOp().Set(Gf.Quatf(Gf.Rotation(Gf.Vec3d(0, 0, 1), d).GetQuat()))
        axf.AddScaleOp().Set(Gf.Vec3f(0.007, 0.007, d.GetLength()))

        # --- 커플러 (팔 플랜지 ↔ 그리퍼 어댑터, 시각 전용, grip_base 자식) ---
        cpl = UsdGeom.Cylinder.Define(stage, grip_base + "/Coupler")
        cpl.CreateRadiusAttr(_COUPLER_R)
        cpl.CreateHeightAttr(_COUPLER_H)
        cpl.CreateAxisAttr("Z")                          # 접근축(그리퍼 로컬 Z)
        cpl.CreateExtentAttr([Gf.Vec3f(-_COUPLER_R, -_COUPLER_R, -_COUPLER_H/2),
                              Gf.Vec3f(_COUPLER_R, _COUPLER_R, _COUPLER_H/2)])
        cpl.CreateDisplayColorAttr([Gf.Vec3f(0.20, 0.20, 0.22)])
        UsdGeom.Xformable(cpl.GetPrim()).AddTranslateOp().Set(
            Gf.Vec3d(0.0, 0.0, -_COUPLER_H / 2.0))       # 그리퍼 뒤(플랜지 쪽)

        self._hinge_path = saw_path
        self._blade_joint = jpath
        log(f"[Harvester] 톱날 커터 장착: {cutter_root} "
            f"(원형 톱 R{_SAW_R*1000:.0f}mm, 회전 {_SAW_SPIN:.0f}°/s, "
            f"파지점 위 {ee.cutter_offset_z*1000:.0f}mm)")

    @staticmethod
    def _make_body(prim: Usd.Prim) -> None:
        """커터 강체 공통 — 콜라이더 없음(§5.1), 중력 끔, 질량 명시.

        콜라이더가 없어 PhysX 가 질량을 못 구하므로 MassAPI 로 직접 준다.
        중력을 끄는 이유: 가벼운 연출용 기구라 처지거나 고정조인트를 당기지 않게.
        """
        UsdPhysics.RigidBodyAPI.Apply(prim)
        m = UsdPhysics.MassAPI.Apply(prim)
        m.CreateMassAttr(_CUTTER_MASS)
        m.CreateDiagonalInertiaAttr(Gf.Vec3f(1e-4, 1e-4, 1e-4))
        PhysxSchema.PhysxRigidBodyAPI.Apply(prim).CreateDisableGravityAttr(True)

    def _add_cad_jig(self, stage: Usd.Stage, log) -> None:
        """사용자 CAD 커터 지그(FreeCAD build_harvest_eef_jig.py) 부착.

        커플러 링을 툴0에 동축으로. 방향 = 그리퍼(툴0) 정렬 + X−90° + Y−90°
        (2026-07-19 사용자 렌더 확정). 스케일 0.1(§8 단위). 날·서보는 CAD 더미
        (실제 설계 형상 — Isaac에 대체 에셋 없음, 실측 확인). camera_dummy 대신 실물 D455.
        """
        from isaacsim.core.utils.stage import add_reference_to_stage
        if self._tool0_m is None or self._grip_base is None:
            log("[Harvester] ⚠ tool0/grip_base 미확정 — CAD 지그 미부착")
            return
        # 지그 월드 포즈: 위치=툴0, 방향=툴0 + X−90°·Y−90° (사용자 확정).
        t = self._tool0_m.ExtractTranslation()
        q_cad = (Gf.Rotation(Gf.Vec3d(0, 1, 0), -90.0)
                 * Gf.Rotation(Gf.Vec3d(1, 0, 0), -90.0)
                 * Gf.Rotation(self._tool0_m.ExtractRotationQuat())).GetQuat()

        # ★ 그리퍼 base_link(움직이는 툴)의 자식으로 붙인다 — 팔이 움직이면 지그도 따라감.
        #   (정적 루트에 두면 팔만 가고 지그가 남아 "허공 절단". 옛 _add_cutter 도 이렇게 했음.)
        #   월드 포즈를 base_link 로컬로 변환해 현재 배치를 그대로 보존.
        M_base = UsdGeom.XformCache().GetLocalToWorldTransform(
            stage.GetPrimAtPath(self._grip_base))
        Rbase_inv = M_base.ExtractRotationMatrix().GetInverse()
        local_pos = M_base.GetInverse().Transform(Gf.Vec3d(t))
        local_rot = Gf.Matrix4d(
            Gf.Matrix3d(Gf.Rotation(q_cad)) * Rbase_inv, Gf.Vec3d(0.0)).ExtractRotationQuat()
        root = f"{self._grip_base}/CadJig"
        UsdGeom.Xform.Define(stage, root)
        set_pose(stage.GetPrimAtPath(root), local_pos, local_rot)
        UsdGeom.Xformable(stage.GetPrimAtPath(root)).AddScaleOp().Set(
            Gf.Vec3f(_CAD_SCALE, _CAD_SCALE, _CAD_SCALE))
        # camera_dummy 는 D455 위치·시선을 잡는 로케이터로만 참조(뒤에서 숨김).
        for p in ("jig", "blade_dummy", "servo_dummy", "camera_dummy"):
            url = os.path.join(_CAD_JIG_DIR, p + ".usd")
            if os.path.isfile(url):
                add_reference_to_stage(url, f"{root}/{p}")
            else:
                log(f"[Harvester] ⚠ CAD USD 없음: {url}")

        # 실물 D455 — 위치=camera_dummy 월드 중심(가이드 기준, 내가 선정), 방향=전역 Euler
        # _CAD_CAM_EULER. look-at 자동정렬 안 씀 — 고정 회전만.
        bb = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                               [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
        cam_w = bb.ComputeWorldBound(
            stage.GetPrimAtPath(f"{root}/camera_dummy")).ComputeAlignedRange().GetMidpoint()
        self._add_camera_at(stage, Gf.Vec3d(cam_w), _CAD_CAM_EULER, log)
        stage.GetPrimAtPath(f"{root}/camera_dummy").SetActive(False)  # 로케이터 숨김

        # 커터/지그 = 금속 회색으로 못박는다. CAD USD 메시엔 색/재질이 안 구워져 있어 그대로면
        # RGB(Replicator)·ROS 카메라에서 fallback 회색으로 뜬다. UsdPreviewSurface(metallic)를
        # 직접 바인딩해 main·SDG 모두에서 금속 날로 보이게 한다(camera_dummy 는 비활성이라 제외됨).
        self._bind_metal(stage, root)
        log(f"[Harvester] CAD 커터 지그 부착: {root} "
            f"(커플러 동축, 그리퍼 +{_COUPLER_T*1000:.0f}mm, 실물 D455, 금속 회색 재질)")

    @staticmethod
    def _bind_metal(stage: Usd.Stage, root: str,
                    mat_path: str = "/World/Looks/CutterMetal") -> None:
        """root 하위 모든 메시에 금속 회색 UsdPreviewSurface 를 바인딩(재질 1개 재사용)."""
        if stage.GetPrimAtPath(mat_path):
            metal = UsdShade.Material(stage.GetPrimAtPath(mat_path))
        else:
            metal = UsdShade.Material.Define(stage, mat_path)
            sh = UsdShade.Shader.Define(stage, mat_path + "/S")
            sh.CreateIdAttr("UsdPreviewSurface")
            sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(
                Gf.Vec3f(0.55, 0.57, 0.60))
            sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.9)
            sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.35)
            metal.CreateSurfaceOutput().ConnectToSource(
                sh.CreateOutput("surface", Sdf.ValueTypeNames.Token))
        for prim in Usd.PrimRange(stage.GetPrimAtPath(root)):
            if prim.IsA(UsdGeom.Mesh):
                mb = UsdShade.MaterialBindingAPI.Apply(prim)
                mb.UnbindAllBindings()
                mb.Bind(metal)

    def camera_path(self, stage: Usd.Stage) -> str | None:
        """D455 컬러 카메라 prim 경로 (ROS2 렌더프로덕트용). 없으면 None.

        _add_camera_at 이 붙인 {grip_base}/D455/asset 아래 UsdGeom.Camera 중 'Color' 우선.
        ROS2 카메라 브리지(ros/robot_bridge.build_camera)가 이 경로로 렌더프로덕트를 만든다.
        """
        if not self._grip_base:
            return None
        asset = stage.GetPrimAtPath(f"{self._grip_base}/D455/asset")
        if not asset.IsValid():
            return None
        cams = [p for p in Usd.PrimRange(asset) if p.IsA(UsdGeom.Camera)]
        if not cams:
            return None
        cam = next((p for p in cams if "Color" in p.GetName()), cams[0])
        return str(cam.GetPath())

    @property
    def chassis_path(self) -> str | None:
        """오도메트리 기준 강체 = Ridgeback 섀시(Base/base_link). 없으면 None."""
        return f"{self._root}/Base/base_link" if self._root else None

    def attach_lidar(self, stage: Usd.Stage, offset: tuple[float, float, float],
                     log=print) -> str | None:
        """섀시에 2D 라이다를 붙이고 경로를 돌려준다. 실패하면 None.

        iw.hub 와 달리 **찾아보지 않고 바로 만든다** — RidgebackUr 스톡 에셋에는 라이다가
        없다(2026-07-18 Clearpath 폴더 전수 확인, settings.RobotAssetConfig 주석). 실물
        Ridgeback 도 라이다는 옵션 장비지 기본 탑재가 아니다.
        ★ orientation 은 **튜플이 아니라 Gf.Quatd** 여야 한다. 커맨드 내부가
          `Gf.Rotation(orientation)` 을 호출하는데 튜플 오버로드가 없어 Boost.Python
          ArgumentError 로 죽는다 (2026-07-20 GPU 실측). 이 함수는 예외를 삼키므로
          틀리면 '조용히 라이다 없음' 이 된다 — 그래서 타입을 여기서 못 박는다.
        ★ config 는 **2D 라이다여야 한다.** Nav2 가 쓰는 /scan(LaserScan)은 평면 스캔이라
          IsaacComputeRTXLidarFlatScan 이 3D 프로파일을 거부한다 (2026-07-20 실측:
          Example_Rotary 로 뒀더니 "elevationDeg contains nonzero value -15.0 …
          not a 2D Lidar, and node will not execute" 로 노드가 아예 안 돈다).
          Example_Rotary_2D = NVIDIA 표준 2D 프로파일. 실물 감각을 원하면
          Slamtec_RPLIDAR_S2E(360°·30m) 나 SICK_TIM781 로 바꿔도 된다 —
          단 3D(Example_Rotary·OS1·XT32·VLS_128)는 /scan 용으로 못 쓴다.
        """
        chassis = self.chassis_path
        if not chassis or not stage.GetPrimAtPath(chassis).IsValid():
            log("[Harvester] ⚠ 섀시(base_link) 없음 — 라이다 부착 실패")
            return None
        path = f"{chassis}/nav_lidar"
        try:
            import omni.kit.commands
            omni.kit.commands.execute(
                "IsaacSensorCreateRtxLidar",
                path=path, parent=None,
                config="RPLIDAR_S2E",
                translation=Gf.Vec3d(*offset),
                orientation=Gf.Quatd(1.0, 0.0, 0.0, 0.0))
            sensor = self._rtx_sensor_prim(stage, path)
            if sensor is None:
                log(f"[Harvester] ⚠ {path} 아래에 센서 프림(OmniLidar/Camera) 없음 — "
                    "이 config 는 렌더프로덕트를 못 붙인다")
                return None
            log(f"[Harvester] RTX 라이다 생성: {sensor} (오프셋 {offset})")
            return sensor
        except Exception as e:
            log(f"[Harvester] ⚠ 라이다 생성 실패 — GPU 에서 RTX 라이다 API 확인 필요: {e}")
            return None

    @staticmethod
    def _rtx_sensor_prim(stage: Usd.Stage, root: str) -> str | None:
        """생성된 라이다 트리에서 렌더프로덕트를 붙일 수 있는 프림을 찾는다.

        왜 필요한가 (2026-07-20 실측) — 프로파일마다 만들어지는 모양이 다르다:
          NVIDIA(Example_Rotary_2D) : 커맨드가 라이다 프림을 **직접** 만든다 → root 자신
          벤더(RPLIDAR_S2E, SICK_*) : USD 에셋을 **참조**로 붙인다 → root 는 껍데기
            Xform 이고 진짜 센서는 그 안쪽. 껍데기를 렌더프로덕트에 주면
            "Render product not attached to RTX Lidar (Camera or OmniLidar prims are
            required)" 경고만 반복되고 /scan 이 안 나온다.
        그래서 root 부터 훑어 OmniLidar/Camera 를 찾고, 없으면 None.
        """
        prim = stage.GetPrimAtPath(root)
        if not prim.IsValid():
            return None
        for p in Usd.PrimRange(prim):
            tname = (p.GetTypeName() or "")
            if "Lidar" in tname or p.IsA(UsdGeom.Camera):
                return str(p.GetPath())
        return None

    def grasp_tcp_path(self, stage: Usd.Stage) -> str | None:
        """그리퍼 손가락 사이의 실제 파지 중심 prim 경로."""
        if self._grasp_tcp and stage.GetPrimAtPath(self._grasp_tcp).IsValid():
            return self._grasp_tcp
        return None

    def _add_camera_at(self, stage: Usd.Stage, cam_pos, euler, log) -> None:
        """RealSense D455 를 그리퍼 base_link 자식으로 붙인다(팔 따라감) + 로컬 rotateXYZ
        op 에 euler 를 리터럴로 박는다 — GUI 트랜스폼 패널이 이 값을 그대로 표시.

        회전은 base_link 기준 로컬값(툴에 붙은 카메라의 장착각). 위치는 cam_pos(월드)를
        base_link 로컬로 변환해 월드 자리를 보존. 빈 Xform(/D455) 밑 자산 자식(/D455/asset)
        구조라 상속 op 충돌(§8)이 없다.
        """
        from isaacsim.core.utils.stage import add_reference_to_stage
        path = f"{self._grip_base}/D455"
        mount = UsdGeom.Xform.Define(stage, path)          # 빈 프레임(상속 op 없음)
        url = assets.resolve(self._cfg.assets.camera, "카메라(RealSense D455)")
        add_reference_to_stage(url, path + "/asset")        # 자산은 자식으로
        # 물리 잔재를 전부 벗긴다 (강체·PhysX·콜라이더·아티큘레이션루트·내부 조인트).
        # ⚠ "RSD455 rigid body 못 찾음" 로그는 이걸로도 안 사라진다(2026-07-19 확인) —
        #   텐서 뷰 쪽 원인 미상, 기능 영향 없음(카메라 시각·물리 정상). 무해로 문서화.
        for prim in Usd.PrimRange(stage.GetPrimAtPath(path + "/asset")):
            if prim.IsA(UsdPhysics.Joint):                  # 에셋 내부 조인트 비활성
                prim.SetActive(False)
                continue
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                prim.RemoveAPI(UsdPhysics.RigidBodyAPI)
            if prim.HasAPI(PhysxSchema.PhysxRigidBodyAPI):
                prim.RemoveAPI(PhysxSchema.PhysxRigidBodyAPI)
            if prim.HasAPI(UsdPhysics.CollisionAPI):
                prim.RemoveAPI(UsdPhysics.CollisionAPI)
            if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
                prim.RemoveAPI(UsdPhysics.ArticulationRootAPI)
        M_base = UsdGeom.XformCache().GetLocalToWorldTransform(
            stage.GetPrimAtPath(self._grip_base))
        cam_local = M_base.GetInverse().Transform(Gf.Vec3d(cam_pos))
        xf = UsdGeom.Xformable(mount)
        xf.AddTranslateOp().Set(Gf.Vec3d(cam_local))
        xf.AddRotateXYZOp().Set(Gf.Vec3f(float(euler[0]), float(euler[1]), float(euler[2])))
        log(f"[Harvester] D455 장착: {path} "
            f"(base_link 자식·팔따라감, 로컬 회전XYZ {tuple(euler)} = GUI표시값)")

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
        container = UsdGeom.Xform.Define(stage, path).GetPrim()
        add_reference_to_stage(url, path + "/asset")
        for prim in Usd.PrimRange(stage.GetPrimAtPath(path + "/asset")):
            if prim.IsA(UsdPhysics.Joint):
                prim.SetActive(False)
                continue
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                prim.RemoveAPI(UsdPhysics.RigidBodyAPI)
            if prim.HasAPI(PhysxSchema.PhysxRigidBodyAPI):
                prim.RemoveAPI(PhysxSchema.PhysxRigidBodyAPI)
            if prim.HasAPI(UsdPhysics.CollisionAPI):
                prim.RemoveAPI(UsdPhysics.CollisionAPI)
            if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
                prim.RemoveAPI(UsdPhysics.ArticulationRootAPI)

        cams = [p for p in Usd.PrimRange(container) if p.IsA(UsdGeom.Camera)]
        cam = next((p for p in cams if "Color" in p.GetName()),
                   cams[0] if cams else None)
        if cam is None:
            log(f"[Harvester] ⚠ 카메라 에셋에 Camera prim 없음: {url}")
            return
        ee = self._cfg.end_effector
        # 컨테이너를 파지점 오프셋에 **identity 회전**으로 놓고, 그 상태의 에셋 카메라
        # 실제 시선을 잰다 (에셋 관례를 가정하지 않는다 — 그동안 뒤/바닥으로 틀렸음).
        set_pose(container, ee.camera_offset, Gf.Quatd(1.0))
        cache = UsdGeom.XformCache()
        m_cam = cache.GetLocalToWorldTransform(cam)
        cam_rot = m_cam.ExtractRotationMatrix()
        m_grip = cache.GetLocalToWorldTransform(stage.GetPrimAtPath(self._grip_base))
        grip_rot = m_grip.ExtractRotationMatrix()

        # 원하는 카메라 시선(월드): 옆에 달린 카메라에서 TCP 점으로 기울이는 look-at이
        # 아니라, RG2 TCP 진행축(그리퍼 로컬 +Z)과 정확히 평행하게 맞춘다.
        want = Gf.Vec3d(m_grip.TransformDir(Gf.Vec3d(0, 0, 1))).GetNormalized()
        up0 = Gf.Vec3d(0, 0, 1) if abs(want[2]) < 0.95 else Gf.Vec3d(0, 1, 0)
        right = Gf.Cross(want, up0).GetNormalized()
        upc = Gf.Cross(right, want)
        # D455 긴 축이 가로(ㅡ)가 되도록 광축(want=접근축) 중심 롤을 적용한다.
        roll = Gf.Rotation(want, _CAMERA_ROLL_DEG)
        right = roll.TransformDir(right)
        upc = roll.TransformDir(upc)
        # 원하는 카메라 월드 회전(행=카메라 로컬축의 월드표현; -Z=시선)
        want_rot = Gf.Matrix3d(right[0], right[1], right[2],
                               upc[0], upc[1], upc[2],
                               -want[0], -want[1], -want[2])
        # 컨테이너 identity 일 때 카메라 상대회전 = cam_rot * grip_rot⁻¹.
        # 원하는 카메라월드 = cam_rel * 컨테이너월드 → 컨테이너월드 = cam_rel⁻¹ * want_rot.
        # 컨테이너로컬 = 컨테이너월드 * grip_rot⁻¹.
        cam_rel = cam_rot * grip_rot.GetInverse()
        cont_local = cam_rel.GetInverse() * want_rot * grip_rot.GetInverse()
        q = Gf.Matrix4d(cont_local, Gf.Vec3d(0.0)).ExtractRotationQuat()
        set_pose(container, ee.camera_offset, q)
        # 최종 authored pose 기준 실제 USD 카메라 광축(-Z)과 TCP 진행축(+Z) 오차를 검증한다.
        cache.Clear()
        actual_cam = cache.GetLocalToWorldTransform(cam)
        optical = Gf.Vec3d(
            actual_cam.TransformDir(Gf.Vec3d(0, 0, -1))).GetNormalized()
        tcp_axis = Gf.Vec3d(
            cache.GetLocalToWorldTransform(stage.GetPrimAtPath(self._grip_base))
            .TransformDir(Gf.Vec3d(0, 0, 1))).GetNormalized()
        alignment = max(-1.0, min(1.0, float(Gf.Dot(optical, tcp_axis))))
        alignment_error_deg = math.degrees(math.acos(alignment))
        log(f"[Harvester] 카메라 장착: {path} "
            f"(광축↔TCP축 오차={alignment_error_deg:.4f}°, "
            f"TCP x={ee.grasp_center_x:.5f}m)")

    def cutter_world_pos(self, stage: Usd.Stage) -> tuple[float, float, float] | None:
        """커터 힌지(=절단점) 현재 월드 좌표. can_cut/절단 판정에 쓴다.

        힌지는 그리퍼에 FixedJoint 로 물려 있어 물리 스텝 뒤엔 실제 파지점 위를
        따라간다 (스텝 전엔 스폰 시 놓은 포즈 = 같은 값).
        """
        if not self._hinge_path:
            return None
        prim = stage.GetPrimAtPath(self._hinge_path)
        if not prim.IsValid():
            return None
        t = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(
            Usd.TimeCode.Default()).ExtractTranslation()
        return (float(t[0]), float(t[1]), float(t[2]))

    # ----- 커터 동작 (FSM/dispatcher 가 부른다 — §5.6: Isaac 은 실행만) -----

    @staticmethod
    def cut_distance(cutter_pos, fruit_pos) -> float:
        """커터 절단점 ↔ 과실 거리[m]. 순수 함수 — GPU 없이 tests 로 검증한다."""
        return math.dist(cutter_pos, fruit_pos)

    def _fruit_of(self, stage: Usd.Stage, joint_path: str) -> str | None:
        """꽃자루 조인트(pedicel)가 잇는 과실(Body1) prim 경로."""
        joint = UsdPhysics.Joint(stage.GetPrimAtPath(joint_path))
        if not joint:
            return None
        tgts = joint.GetBody1Rel().GetTargets()
        return str(tgts[0]) if tgts else None

    def can_cut(self, stage: Usd.Stage, joint_path: str) -> tuple[bool, float]:
        """커터 날이 이 꽃자루의 과실에 cut_tolerance 안으로 왔나. 반환 (성공?, 거리m).

        ⚠ [4]: 실제 파지 자세에선 과실중심↔힌지가 cutter_offset_z(45mm)만큼 떨어진다.
        cut_tolerance(10mm)는 그 기하에 맞춰 GPU 에서 튜닝할 값 — 스윕하면 "커터 위치
        정밀도 요구사항"([3])이 된다(settings.py cut_tolerance 주석).
        """
        cpos = self.cutter_world_pos(stage)
        fruit = self._fruit_of(stage, joint_path)
        if cpos is None or fruit is None:
            return (False, float("inf"))
        fp = UsdGeom.XformCache().GetLocalToWorldTransform(
            stage.GetPrimAtPath(fruit)).ExtractTranslation()
        dist = self.cut_distance(cpos, (fp[0], fp[1], fp[2]))
        return (dist <= self._cfg.end_effector.cut_tolerance, dist)

    def do_cut(self, stage: Usd.Stage, joint_path: str) -> bool:
        """자르기 — 날이 닿았으면 날을 오므리고(연출) 조인트를 끊는다(실제 분리).

        멀면 False (팔을 더 움직여야 함). §5.6: 언제 부를지는 harvest.py FSM 이 정한다.
        """
        ok, _ = self.can_cut(stage, joint_path)
        if not ok:
            return False
        # 톱날은 계속 회전 중(속도 드라이브) — 줄기에 닿았으면 바로 절단.
        from scene import pedicel
        return pedicel.cut(stage, joint_path)        # 실제 분리(결정적)

    def open_blades(self, stage: Usd.Stage) -> None:
        """톱날 커터는 계속 회전하므로 여닫기가 없다 (API 호환용 no-op)."""
        return

    # ===== 서보 힌지 가동날 (CAD 의도: 서보축 revolute) — 조립 + 제어 =====

    def attach_blade_hinge(self, stage: Usd.Stage, log=print) -> bool:
        """가동날 = CAD 지그 원본 blade_dummy **그대로**(정적). 서보축 절단점만 기록한다.

        blade_dummy 는 CadJig(=grip_base 자식) 아래라 위치(서보축 위)·스케일(_CAD_SCALE)·
        금속재질(_bind_metal)·브라켓 추종을 **이미 갖췄다** — 그러니 손대지 않는다.
        ★ 예전엔 매 프레임 이 prim 의 xformOp 를 재작성(_set_full_pose)했는데, 아티큘레이션
          내부 Fabric prim 을 매프레임 구조변경하면 PhysX 가 크래시하고 metersPerUnit(0.01)
          보정이 깨져 날이 서보에서 집게 사이로 어긋났다(2026-07-22). → 정적으로 둔다.
        시각 스윙은 생략 — 실제 절단은 detach_fruit(§5.3)이고 blade_deg 는 게이트값으로만 쓴다.
        """
        if not self._grip_base:
            log("[Harvester] ⚠ grip_base 없음 — 블레이드 미부착")
            return False
        blade = next((p for p in Usd.PrimRange(stage.GetPrimAtPath(self._grip_base))
                      if p.GetName() == "blade_dummy"), None)
        if blade is None:
            log("[Harvester] ⚠ blade_dummy 없음 — 블레이드 미부착")
            return False
        M = UsdGeom.XformCache().GetLocalToWorldTransform(blade)
        self._blade_shaft_w = Gf.Vec3d(M.Transform(Gf.Vec3d(0, 53, 132)))  # 서보축 절단점(줄기 배치용)
        self._blade_path = str(blade.GetPath())
        self._blade_deg = self.BLADE_OPEN_DEG
        log("[Harvester] 가동날 = CAD 원본 blade_dummy(정적, 금속·서보 위·브라켓 추종). "
            f"절단점 {tuple(round(float(v), 3) for v in self._blade_shaft_w)}")
        return True

    def sync_blade_pose(self, stage: Usd.Stage) -> None:
        """가동날은 정적(CAD 원본 그대로) — 시각 스윙 없음. 매 프레임 USD 재작성은 Fabric
        크래시·날 어긋남 원인이라 제거했다(2026-07-22). blade_deg 는 절단 게이트로만 쓴다."""
        return

    def set_blade_deg(self, deg: float) -> None:
        """가동날 각도 명령 [deg]. ★§5.6: ROS2 노드가 토픽 받아 이 메서드를 부른다.
        열림 0° ~ 닫힘 35°(=절단). 다음 sync_blade_pose() 가 이 각으로 날을 회전 배치한다."""
        self._blade_deg = float(deg)

    def open_blade(self) -> None:
        """가동날 열기 (미부착이면 no-op)."""
        self.set_blade_deg(self.BLADE_OPEN_DEG)

    def close_blade(self) -> None:
        """가동날 닫기 = 절단 자세."""
        self.set_blade_deg(self.BLADE_CLOSED_DEG)

    def move_blade(self, d_deg: float) -> None:
        """가동날 각도 증분 [deg] — 텔레옵/증분 제어용. [열림, 닫힘]로 제한."""
        self.set_blade_deg(max(self.BLADE_OPEN_DEG,
                               min(self.BLADE_CLOSED_DEG, self._blade_deg + d_deg)))

    def blade_deg(self) -> float:
        """현재 가동날 각도 [deg]."""
        return self._blade_deg

    @property
    def blade_cut_point(self):
        """가동날 절단점(서보축) 월드좌표 (x,y,z). 줄기를 여기 두면 날 닫힘 시 잘린다. None=미부착."""
        if self._blade_shaft_w is None:
            return None
        return (float(self._blade_shaft_w[0]), float(self._blade_shaft_w[1]),
                float(self._blade_shaft_w[2]))

    def find_finger_joints(self, stage: Usd.Stage) -> list[str]:
        """OnRobot RG2 손가락 관절 경로들 — 파지(마찰)용.

        이름은 에셋마다 다르므로 후보로 시도한다(transporter._LIFT_CANDIDATES 패턴).
        """
        if not self._gripper_path:
            return []
        root = stage.GetPrimAtPath(self._gripper_path)
        if not root.IsValid():
            return []
        cands = ("finger_joint", "knuckle_joint", "inner_finger_joint")
        out = []
        for p in Usd.PrimRange(root):
            if p.IsA(UsdPhysics.RevoluteJoint) and any(
                    c in p.GetName().lower() for c in cands):
                out.append(str(p.GetPath()))
        return out
