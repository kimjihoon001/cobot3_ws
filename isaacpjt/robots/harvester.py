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

import math
import os

from pxr import Gf, PhysxSchema, Usd, UsdGeom, UsdPhysics, UsdShade

from pjt_config.settings import RobotConfig
from pjt_utils.xform import set_pose, set_translate
from robots import assets

# [3] 수확자세 — wrist_1(4번축) 각도[deg]. 기본자세(0°)면 커터·지그가 파지점 아래(뒤집힘),
# +180° 라야 절단점이 파지점 위 5.3cm 로 온다(2026-07-19 GPU 실측, CAD 의도 그대로).
WRIST1_HARVEST_DEG = 180.0

# [4] 시작 자세 — 에셋 기본(0°) 기준 **절대각**[deg]. USD 에 구워 Play/Stop 이 같게 한다.
# 1번(shoulder_pan) 0 · 2번(shoulder_lift) 270 · 3번(elbow) 90 · 5번(wrist_2) −90
# = 사용자 지정(2026-07-20).
# 순서는 근위→원위로 둘 것 — 원위 링크를 통째로 돌리므로 순서가 바뀌면 결과가 달라진다.
HOME_POSE_DEG = (("shoulder_pan_joint", 0.0),
                 ("shoulder_lift_joint", 225.0),
                 ("elbow_joint", 135.0),
                 ("wrist_1_joint", WRIST1_HARVEST_DEG),
                 ("wrist_2_joint", -90.0))
# UR10e 링크 사슬(근위→원위). 조인트를 돌릴 때 그 아래 원위 링크를 전부 같이 돌린다.
_UR_LINKS = ("shoulder_link", "upper_arm_link", "forearm_link",
             "wrist_1_link", "wrist_2_link", "wrist_3_link", "ee_link")
_UR_DISTAL_FROM = {"shoulder_pan_joint": 0, "shoulder_lift_joint": 1,
                   "elbow_joint": 2, "wrist_1_joint": 3,
                   "wrist_2_joint": 4, "wrist_3_joint": 5}

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

# [4] 장착 보정 — Robotiq base_link 프레임이 UR 툴 소켓과 접근축(Z) 기준 이만큼 틀어져
# 손가락이 어긋난다(2026-07-18 사용자 렌더 검토 지적). 90/270 중 렌더로 맞춘다.
_GRIPPER_ROLL_DEG = 90.0
# [4] 카메라 이미지 롤 보정 — 측정정렬로 방향은 맞으나 이미지가 90° 굴러 있었다(사용자
# 지적). 광축(접근축) 중심 롤. 90/270 은 렌더로 맞춘다.
_CAMERA_ROLL_DEG = 90.0

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


def _set_full_pose(prim: Usd.Prim, M: Gf.Matrix4d) -> None:
    """행렬 M(스케일 포함)을 translate+orient+scale op 으로 세팅 (커터날 사본 배치용)."""
    xf = UsdGeom.Xformable(prim)
    xf.ClearXformOpOrder()
    s = Gf.Vec3f(Gf.Vec3d(M[0][0], M[0][1], M[0][2]).GetLength(),
                 Gf.Vec3d(M[1][0], M[1][1], M[1][2]).GetLength(),
                 Gf.Vec3d(M[2][0], M[2][1], M[2][2]).GetLength())
    xf.AddTranslateOp().Set(M.ExtractTranslation())
    xf.AddOrientOp().Set(Gf.Quatf(M.ExtractRotationQuat()))
    xf.AddScaleOp().Set(s)


class HarvestMM:
    """수확 MM. 베이스(Ridgeback) + 팔(UR10e) + 그리퍼(Robotiq) + 커터.

    구조:
        {root}/Base      <- 이동 베이스
        {root}/Arm       <- 팔. 베이스 위 arm_mount_z 에 고정 조인트로 붙는다
        {root}/Gripper   <- 그리퍼. 팔 끝(tool0)에 붙는다
        …/Gripper/…/base_link/Cutter <- 커터. 파지점 위 cutter_offset_z, 계층 자식(조인트 X, §8)
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
        self._blade_target_attr = None             # 가동날 드라이브 목표각 attr (§5.6 ROS2 제어점)
        self._blade_shaft_w: Gf.Vec3d | None = None  # 가동날 절단점(서보축) 월드좌표 (줄기 배치용)
        self._blade_rel: Gf.Matrix4d | None = None   # 날 사본 ← grip_base 상대포즈 (정지 중 재배치용)

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
        # 시작자세를 USD 에 굽는다. 그리퍼·지그는 이 뒤에 tool0 월드포즈 기준으로 붙으므로
        # 순서가 중요하다 (먼저 팔을 돌려 놓고 → 그 자리에 그리퍼·지그).
        self._preset_pose(stage, arm_path, log)

        # 그리퍼 — 팔 에셋이 제공하는 툴 소켓(ee_joint)에 물린다.
        gripper_url = assets.resolve(a.gripper, "그리퍼(Robotiq)")
        gripper_path = f"{root}/Gripper"
        add_reference_to_stage(gripper_url, gripper_path)
        grip_base = self._attach_gripper(stage, arm_path, gripper_path, log)
        self._gripper_path = gripper_path
        self._grip_base = grip_base

        # 커터·카메라 = 사용자 CAD 커터 지그(커플러 링 + 서보 가위 + 실물 D455).
        # 프리미티브 커터/카메라(_add_cutter/_add_camera)는 남겨두되 안 쓰고 CAD 로 대체.
        if grip_base:
            self._add_cad_jig(stage, log)
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

        UR10e 는 링크가 flat(형제) 구조라(탐침 2026-07-20: wrist_1/2/3_link, ee_link
        전부 /Arm 바로 아래) 그 조인트의 원위 링크들을 조인트 축 둘레로 직접 돌린다.
        """
        joint = stage.GetPrimAtPath(f"{arm_path}/joints/{jname}")
        if not joint.IsValid():
            log(f"[Harvester] ⚠ {jname} 없음 — 시작자세 프리셋 스킵(기본자세로 뜬다)")
            return
        names = _UR_LINKS[_UR_DISTAL_FROM[jname]:]
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
        # 장착 롤 보정 — Robotiq base_link 프레임이 UR 툴 소켓과 접근축(Z) 기준
        # _GRIPPER_ROLL_DEG 만큼 틀어져 손가락이 어긋난다(2026-07-18 사용자 렌더 지적).
        # 소켓을 그만큼 롤한 자리에 그리퍼를 얹는다.
        m_socket = Gf.Matrix4d().SetRotate(
            Gf.Rotation(Gf.Vec3d(0, 0, 1), _GRIPPER_ROLL_DEG)) * m_socket

        # CAD 커플러 지그가 붙는 툴0 프레임(그리퍼 밀기 전)을 저장.
        self._tool0_m = Gf.Matrix4d(m_socket)
        # 커플러(12mm)가 플랜지↔그리퍼 **사이**에 들어가므로, 그리퍼를 접근축(+Z)으로
        # _COUPLER_T 만큼 밀어 얹는다 (사용자 지적: 커플러가 사이에 들어감).
        approach = Gf.Vec3d(m_socket.TransformDir(Gf.Vec3d(0, 0, 1))).GetNormalized()
        m_socket.SetTranslateOnly(m_socket.ExtractTranslation() + approach * _COUPLER_T)

        # 그리퍼 컨테이너를 옮겨 base_link 가 (롤 보정 + 커플러 오프셋된) 소켓에 오게 한다.
        # ★ 로컬 op 에 쓸 값은 부모 프레임으로 변환해야 한다: L' = C·G⁻¹·S·P⁻¹ (row-vector,
        #   C=컨테이너 l2w, G=base_link l2w, S=목표 소켓 월드, P=부모 l2w). 예전 G⁻¹·S 는
        #   부모(하베스터 루트)가 원점일 때만 맞아, main.py 가 (0,−12) 스폰하자 그리퍼가
        #   12m 밖에 붙어 로봇이 분해됐다(§8 2026-07-19).
        m_grip = cache.GetLocalToWorldTransform(stage.GetPrimAtPath(grip_base))
        m_cont = cache.GetLocalToWorldTransform(stage.GetPrimAtPath(gripper_path))
        m_parent = cache.GetLocalToWorldTransform(
            stage.GetPrimAtPath(gripper_path).GetParent())
        m_local = m_cont * m_grip.GetInverse() * m_socket * m_parent.GetInverse()
        set_pose(stage.GetPrimAtPath(gripper_path),
                 m_local.ExtractTranslation(),
                 m_local.ExtractRotationQuat())

        # 조인트 프레임을 **실제 배치된 상대 포즈**로 다시 쓴다 (§8: 롤 보정 후 프레임을
        # 안 맞추면 시작 순간 스냅). body0(손목) 로컬로 그리퍼 포즈를 준다.
        pos, rot = self._rel_pose(stage, wrist, grip_base)
        j.GetBody1Rel().SetTargets([grip_base])
        j.CreateLocalPos0Attr().Set(pos)
        j.CreateLocalRot0Attr().Set(rot)
        j.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0))
        j.CreateLocalRot1Attr().Set(Gf.Quatf(1.0))
        j.CreateJointEnabledAttr().Set(True)
        j.CreateExcludeFromArticulationAttr().Set(False)
        log(f"[Harvester] 그리퍼 장착: ee_joint → {grip_base} (롤 보정 {_GRIPPER_ROLL_DEG:.0f}°)")
        return grip_base

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
        log(f"[Harvester] CAD 커터 지그 부착: {root} "
            f"(커플러 동축, 그리퍼 +{_COUPLER_T*1000:.0f}mm, 실물 D455)")

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
        config 는 Example_Rotary 로 생성 성공 확인(2026-07-20). 같은 날 실측으로
          SICK_picoScan150 · RPLIDAR_S2E · OS1_REV6_128ch10hz1024res 도 생성되고,
          Hesai_XT32_SD10 · Velodyne_VLS128 은 커맨드는 True 지만 prim 이 안 생긴다.
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
                config="Example_Rotary",
                translation=Gf.Vec3d(*offset),
                orientation=Gf.Quatd(1.0, 0.0, 0.0, 0.0))
            log(f"[Harvester] RTX 라이다 생성: {path} (오프셋 {offset})")
            return path
        except Exception as e:
            log(f"[Harvester] ⚠ 라이다 생성 실패 — GPU 에서 RTX 라이다 API 확인 필요: {e}")
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
        ee = self._cfg.end_effector
        # 컨테이너를 파지점 오프셋에 **identity 회전**으로 놓고, 그 상태의 에셋 카메라
        # 실제 시선을 잰다 (에셋 관례를 가정하지 않는다 — 그동안 뒤/바닥으로 틀렸음).
        set_pose(container, ee.camera_offset, Gf.Quatd(1.0))
        cache = UsdGeom.XformCache()
        m_cam = cache.GetLocalToWorldTransform(cam)
        cam_rot = m_cam.ExtractRotationMatrix()
        m_grip = cache.GetLocalToWorldTransform(stage.GetPrimAtPath(self._grip_base))
        grip_rot = m_grip.ExtractRotationMatrix()

        # 원하는 카메라 시선(월드): 카메라 위치에서 파지점으로.
        cam_pos = m_cam.ExtractTranslation()
        grasp_w = m_grip.Transform(Gf.Vec3d(0.0, 0.0, ee.grasp_reach_z))
        want = (grasp_w - cam_pos).GetNormalized()
        up0 = Gf.Vec3d(0, 0, 1) if abs(want[2]) < 0.95 else Gf.Vec3d(0, 1, 0)
        right = Gf.Cross(want, up0).GetNormalized()
        upc = Gf.Cross(right, want)
        # 이미지 90° 롤 보정 — 광축(want=접근축) 중심 (사용자 지적).
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
        log(f"[Harvester] 카메라 장착: {path} (측정정렬 look-at)")

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
        """CAD 가동날을 서보축 리볼루트 조인트로 붙인다 — 서보 힌지는 고정, 날만 스윙.

        ★ spawn() → world.reset() **한 번** 뒤에 호출할 것 (강체·조인트 물리 초기화됨).
        시각용 blade_dummy(CadJig 자식)는 숨기고, 강체 사본을 grip_base 강체 **밖**(최상위
        prim {root}_CutterBlade)에 두고 revolute+drive 로 grip_base 에 묶는다(강체 중첩 §8 회피).
        이후 set_blade_deg(deg) 로 각도 명령 — §5.6: 나중에 ROS2 가 이 메서드를 부른다.
        축·피벗은 CAD(build_harvest_eef_jig) 값: 축=Y(서보샤프트), 피벗=(0,53,132).
        """
        import omni.kit.app
        from isaacsim.core.utils.stage import add_reference_to_stage
        if not self._grip_base:
            log("[Harvester] ⚠ grip_base 없음 — 블레이드 힌지 미부착")
            return False
        blade = next((p for p in Usd.PrimRange(stage.GetPrimAtPath(self._grip_base))
                      if p.GetName() == "blade_dummy"), None)
        if blade is None:
            log("[Harvester] ⚠ blade_dummy 없음 — 블레이드 힌지 미부착")
            return False

        cache = UsdGeom.XformCache()
        M_orig = cache.GetLocalToWorldTransform(blade)
        shaft_w = M_orig.Transform(Gf.Vec3d(0, 53, 132))                 # 서보축(SHAFT_Z)
        axis_w = Gf.Vec3d(M_orig.TransformDir(Gf.Vec3d(0, 1, 0))).GetNormalized()
        self._blade_shaft_w = Gf.Vec3d(shaft_w)                          # 절단점 저장(줄기 배치용)
        blade.SetActive(False)                                           # 시각용 원본 숨김

        hinge = f"{self._root}_CutterBlade"                              # 최상위(/World 아래)
        UsdGeom.Xform.Define(stage, hinge)
        add_reference_to_stage(os.path.join(_CAD_JIG_DIR, "blade_dummy.usd"), hinge + "/asset")
        omni.kit.app.get_app().update()                                 # metricsAssembler 반영
        asset = stage.GetPrimAtPath(hinge + "/asset")
        U = UsdGeom.Xformable(asset).GetLocalTransformation()
        _set_full_pose(stage.GetPrimAtPath(hinge), U.GetInverse() * M_orig)  # 사본 월드=원본 월드
        UsdPhysics.RigidBodyAPI.Apply(asset)
        mass = UsdPhysics.MassAPI.Apply(asset)
        mass.CreateMassAttr(0.01)
        mass.CreateDiagonalInertiaAttr(Gf.Vec3f(1e-5, 1e-5, 1e-5))
        PhysxSchema.PhysxRigidBodyAPI.Apply(asset).CreateDisableGravityAttr(True)
        for p in Usd.PrimRange(asset):                                  # jig 흰색에 안 묻히게 색칠
            if p.IsA(UsdGeom.Mesh):
                UsdShade.MaterialBindingAPI.Apply(p).UnbindAllBindings()
                UsdGeom.Gprim(p).CreateDisplayColorAttr([Gf.Vec3f(0.9, 0.1, 0.9)])

        # 리볼루트 조인트 프레임: 원점=shaft_w, X축=axis_w(회전축) → AxisAttr("X")
        ref = Gf.Vec3d(0, 0, 1) if abs(axis_w[2]) < 0.9 else Gf.Vec3d(1, 0, 0)
        yw = Gf.Cross(ref, axis_w).GetNormalized()
        zw = Gf.Cross(axis_w, yw)
        Jw = Gf.Matrix4d(axis_w[0], axis_w[1], axis_w[2], 0, yw[0], yw[1], yw[2], 0,
                         zw[0], zw[1], zw[2], 0, shaft_w[0], shaft_w[1], shaft_w[2], 1)
        # ★ 마젠타 사본엔 CAD 스케일(≈0.001)이 붙어 있어 조인트 프레임 계산이 까다롭다:
        #   - 위치(localPos1)는 스케일 **포함** 행렬로 뽑아야 한다(PhysX 가 지오메트리 좌표에
        #     바디 스케일을 곱해 월드 위치를 구하므로). 안 그러면 피벗이 5cm 뜬다.
        #   - 회전(localRot1)은 스케일 **벗긴**(직교정규화) 행렬로 뽑아야 한다. 스케일 섞이면
        #     회전축이 X↔Y 로 꼬여 드라이브가 날을 엉뚱한(수평) 축으로 넘긴다.
        #   (2026-07-19 측정 확인 — 둘을 섞으면 축·위치 다 맞는다.)
        def _rigid(M: Gf.Matrix4d) -> Gf.Matrix4d:
            return Gf.Matrix4d(M.ExtractRotationMatrix().GetOrthonormalized(),
                               M.ExtractTranslation())
        M_gb = cache.GetLocalToWorldTransform(stage.GetPrimAtPath(self._grip_base))
        # 정지 중 재배치용 상대포즈 — sync_blade_pose() 가 매 정지 프레임 이걸로 다시 놓는다.
        self._blade_rel = (cache.GetLocalToWorldTransform(stage.GetPrimAtPath(hinge))
                           * M_gb.GetInverse())
        M_bl_s = cache.GetLocalToWorldTransform(asset)      # 스케일 포함(위치용)
        M_bl_r = _rigid(M_bl_s)                             # 스케일 벗김(회전용)
        J0 = Jw * M_gb.GetInverse()
        J1_pos = Jw * M_bl_s.GetInverse()                  # localPos1
        J1_rot = Jw * M_bl_r.GetInverse()                  # localRot1
        rev = UsdPhysics.RevoluteJoint.Define(stage, hinge + "/ServoJoint")
        rev.CreateBody0Rel().SetTargets([self._grip_base])
        rev.CreateBody1Rel().SetTargets([asset.GetPath()])
        rev.CreateAxisAttr("X")
        rev.CreateExcludeFromArticulationAttr().Set(True)   # 독립 강체(아티큘레이션 DOF 흡수 방지)
        rev.CreateLocalPos0Attr().Set(Gf.Vec3f(J0.ExtractTranslation()))
        rev.CreateLocalRot0Attr().Set(Gf.Quatf(J0.ExtractRotationQuat()))
        rev.CreateLocalPos1Attr().Set(Gf.Vec3f(J1_pos.ExtractTranslation()))
        rev.CreateLocalRot1Attr().Set(Gf.Quatf(J1_rot.ExtractRotationQuat()))
        rev.CreateLowerLimitAttr(-5.0)
        rev.CreateUpperLimitAttr(45.0)      # 열림 0° ~ 닫힘 35° + 여유
        drive = UsdPhysics.DriveAPI.Apply(rev.GetPrim(), "angular")
        drive.CreateTypeAttr("force")
        drive.CreateStiffnessAttr(2000.0)
        drive.CreateDampingAttr(200.0)
        drive.CreateTargetPositionAttr(self.BLADE_OPEN_DEG)
        self._blade_target_attr = drive.GetTargetPositionAttr()
        log(f"[Harvester] 가동날 서보 힌지 부착: {hinge} (축 Y, 피벗 (0,53,132), "
            f"열림 {self.BLADE_OPEN_DEG:.0f}°~닫힘 {self.BLADE_CLOSED_DEG:.0f}°)")
        return True

    def sync_blade_pose(self, stage: Usd.Stage) -> None:
        """정지 중 가동날 사본을 현재 그리퍼 자세에 맞춰 다시 놓는다 (부착 시 상대포즈 유지).

        왜 필요한가 (사용자 지적 2026-07-20): 날 사본은 강체 중첩(§8)을 피하려고 grip_base
        **밖** 최상위에 두는데, 정지 중에는 조인트가 안 걸려 USD 에 적힌 자리에 그대로 그려진다.
        그 자리는 attach_blade_hinge() 시점(정착 자세) 값이라, 이후 reset 으로 팔이 default_state
        로 돌아가면 날만 남아 떠 보이고 → Play 하면 조인트가 끌어와 붙는다.
        CadJig·D455 는 grip_base 자식이라 이 문제가 없다 — 날 사본만 손으로 따라가게 한다.
        """
        if self._blade_rel is None or not self._grip_base:
            return
        hp = stage.GetPrimAtPath(f"{self._root}_CutterBlade")
        if not hp.IsValid():
            return
        cache = UsdGeom.XformCache()
        M_gb = cache.GetLocalToWorldTransform(stage.GetPrimAtPath(self._grip_base))
        M_par = cache.GetParentToWorldTransform(hp)
        _set_full_pose(hp, self._blade_rel * M_gb * M_par.GetInverse())

    def set_blade_deg(self, deg: float) -> None:
        """가동날 각도 명령 [deg]. ★§5.6: 나중에 ROS2 노드가 토픽 받아 이 메서드를 부른다.
        열림 100° ~ 닫힘 135°(=절단). attach_blade_hinge() 선행 필요."""
        if self._blade_target_attr is not None:
            self._blade_target_attr.Set(float(deg))

    def open_blade(self) -> None:
        """가동날 열기 (미부착이면 no-op)."""
        self.set_blade_deg(self.BLADE_OPEN_DEG)

    def close_blade(self) -> None:
        """가동날 닫기 = 절단 자세."""
        self.set_blade_deg(self.BLADE_CLOSED_DEG)

    def move_blade(self, d_deg: float) -> None:
        """가동날 각도 증분 [deg] — 텔레옵/증분 제어용. [열림, 닫힘]로 제한.
        control.py 의 move_gripper 와 같은 패턴 (붙어야 동작, 미부착이면 no-op)."""
        if self._blade_target_attr is None:
            return
        cur = float(self._blade_target_attr.Get() or self.BLADE_OPEN_DEG)
        self.set_blade_deg(max(self.BLADE_OPEN_DEG,
                               min(self.BLADE_CLOSED_DEG, cur + d_deg)))

    def blade_deg(self) -> float:
        """현재 가동날 목표각 [deg]. 미부착이면 열림각."""
        if self._blade_target_attr is None:
            return self.BLADE_OPEN_DEG
        return float(self._blade_target_attr.Get() or self.BLADE_OPEN_DEG)

    @property
    def blade_cut_point(self):
        """가동날 절단점(서보축) 월드좌표 (x,y,z). 줄기를 여기 두면 날 닫힘 시 잘린다. None=미부착."""
        if self._blade_shaft_w is None:
            return None
        return (float(self._blade_shaft_w[0]), float(self._blade_shaft_w[1]),
                float(self._blade_shaft_w[2]))

    def find_finger_joints(self, stage: Usd.Stage) -> list[str]:
        """그리퍼(Robotiq 2F-85) 손가락 관절 경로들 — 파지(마찰)용.

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
