# -*- coding: utf-8 -*-
"""수확 모바일 매니퓰레이터 — 베이스 + 팔 + 동축 스쿱을 조립해 씬에 놓는다.

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

# [3] 수확자세 — wrist_1(4번축) 각도[deg]. 기본자세(0°)면 커터·지그가 파지점 아래(뒤집힘),
# +180° 라야 절단점이 파지점 위 5.3cm 로 온다(2026-07-19 GPU 실측, CAD 의도 그대로).
WRIST1_HARVEST_DEG = 180.0

# [4] 시작/주행 자세 — 에셋 기본(0°) 기준 **절대각**[deg].
# shoulder 240° + elbow 135°로 사선 접힘을 만들고 wrist_1 165°로 방향을 보상한다.
# MoveIt HOME_Q/SRDF home과 반드시 같은 값이어야 시작 직후 점프하지 않는다.
# 순서는 근위→원위로 둘 것 — 원위 링크를 통째로 돌리므로 순서가 바뀌면 결과가 달라진다.
HOME_POSE_DEG = (("shoulder_pan_joint", 0.0),
                 ("shoulder_lift_joint", 240.0),
                 ("elbow_joint", 135.0),
                 ("wrist_1_joint", 165.0),
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
# 실제 finger4step 패드 bbox 중점은 gripper base +Z 105.43mm다.
# grasp_reach_z(115mm)에서 9.57mm 되돌려 끝단 접촉이 아닌 패드 면 중앙을 TCP로 쓴다.
# tool0 기준으로는 커플러 12mm를 더한 117.43mm이며 MoveIt URDF와 동일하다.
_GRASP_TCP_PAD_CORRECTION_Z = -0.00957
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
    """수확 MM. 베이스(Ridgeback) + 팔(UR10e) + 동축 3축 1/4구 스쿱.

    구조:
        {root}/Base      <- 이동 베이스
        {root}/Arm       <- 팔. 베이스 위 arm_mount_z 에 고정 조인트로 붙는다
        {root}/Gripper/Base            <- ISO 9409 어댑터
        {root}/Gripper/ScoopQuarter*   <- 수용기 두 장 + 외측 커터 한 장
    """

    # 가동날 각도 [deg] — 서보 힌지 리볼루트 조인트 드라이브 목표.
    # 조인트 rest(0°) = blade_dummy 익스포트 자세 = CAD OPEN_ANGLE(-35°, 열림). 거기서
    # +35° 돌리면 CAD 0°(닫힘=노치 전단). 그래서 열림 0° ~ 닫힘 35°(CAD build 스크립트 규약).
    BLADE_OPEN_DEG = 0.0          # 열림 (날이 옆으로 펼쳐짐)
    # 실물리에서 50° 명령 시 접촉/구동 오차를 포함해 약 41°에 안정된다. CAD의 절단
    # 슬롯은 40°에서 이미 줄기를 통과하므로 이 각도를 절단 완료 기준으로 사용한다.
    BLADE_CLOSED_DEG = 40.0
    BLADE_SPEED_DEG_S = 100.0     # 50° 절삭 스윙을 0.5초에 연출

    # 팔 의존 상수 — 서브클래스가 팔만 갈아끼울 수 있게 클래스 속성으로 둔다
    # (robots/harvester_0.py = 같은 스쿱 스택 + m0617 팔, 2026-07-24 사용자).
    HOME_POSE = HOME_POSE_DEG
    ARM_LINKS = _UR_LINKS
    ARM_DISTAL_FROM = _UR_DISTAL_FROM

    def __init__(self, cfg: RobotConfig):
        self._cfg = cfg
        self._root: str | None = None
        self._grip_base: str | None = None       # 스쿱 어댑터 Base
        self._gripper_path: str | None = None     # 스쿱 컨테이너
        self._hinge_path: str | None = None       # 커터 마운트(=절단점) prim
        self._blade_joint: str | None = None       # 단일 날 RevoluteJoint 경로(= 서보)
        self._tool0_m: Gf.Matrix4d | None = None   # 툴0(플랜지) 월드 포즈 — CAD 지그 부착 기준
        self._grasp_tcp: str | None = None         # 실제 파지 중심 프레임
        self._blade_shaft_w: Gf.Vec3d | None = None  # 가동날 절단점(서보축) 월드좌표 (줄기 배치용)
        self._blade_path: str | None = None          # 가동날 프림 경로 (grip_base 자식)
        self._blade_L_rest: Gf.Matrix4d | None = None  # 날 rest 로컬(grip_base 기준) — 스윙 기준
        self._blade_pose_op = None                    # CAD 행렬을 손실 없이 쓰는 transform op
        self._blade_target_attr = None               # 가동날 드라이브 목표각 attr (§5.6 ROS2 제어점)
        self._shaft_rel: Gf.Vec3d | None = None      # 서보 피벗 ← grip_base 로컬 (스윙 회전 중심)
        self._axis_rel: Gf.Vec3d | None = None       # 서보축 방향 ← grip_base 로컬
        self._blade_deg: float = self.BLADE_OPEN_DEG  # 현재 가동날 각도 [deg] (키네마틱 서보)
        self._blade_target_deg: float = self.BLADE_OPEN_DEG  # 서보 명령 목표각 [deg]

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

        # 로컬 동축 스쿱 — 팔 에셋의 툴 소켓(ee_joint)에 직접 물린다.
        gripper_url = os.path.abspath(a.scoop_gripper_usd)
        if not os.path.isfile(gripper_url):
            raise FileNotFoundError(f"동축 스쿱 USD 없음: {gripper_url}")
        gripper_path = f"{root}/Gripper"
        add_reference_to_stage(gripper_url, gripper_path)
        self._uninstance(stage, gripper_path)
        grip_base = self._attach_gripper(stage, arm_path, gripper_path, log)
        self._gripper_path = gripper_path
        self._grip_base = grip_base
        if grip_base:
            self._grasp_tcp = f"{grip_base}/HarvestTCP"
            self._hinge_path = f"{grip_base}/CuttingPoint"
            self._bind_gripper_friction(stage, gripper_path, log)
            # 새 동축 스쿱에도 eye-in-hand D455를 장착한다. Isaac 기본 D455 USD는 내부
            # articulation 메타데이터가 남아 강체 API를 제거해도 tensor 오류를 내므로,
            # 순수 시각 바디+UsdGeom.Camera만 만든 센서 전용 모델을 사용한다.
            self._add_scoop_camera(stage, log)

        # 구형 Robotiq, 별도 커플러, CAD 가위 지그는 의도적으로 장착하지 않는다.
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
        """시작 자세(HOME_POSE)를 근위→원위 순서로 하나씩 굽는다."""
        for jname, deg in self.HOME_POSE:
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
        names = self.ARM_LINKS[self.ARM_DISTAL_FROM[jname]:]
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

    def _uninstance(self, stage: Usd.Stage, root_path: str) -> None:
        """서브트리의 instanceable 참조를 전부 해제 — 콜라이더/물리 prim 을 저작 가능하게
        노출한다(2F-85 콜라이더가 인스턴스 안에 숨어 마찰 바인딩 0개가 되던 문제, 2026-07-22).
        인스턴스를 풀면 자식이 새로 드러나 또 인스턴스일 수 있어 변화가 없을 때까지 반복."""
        root = stage.GetPrimAtPath(root_path)
        if not root.IsValid():
            return
        n = 0
        for _ in range(50):                                # 안전 상한
            found = None
            for p in Usd.PrimRange(root):
                if p.IsInstance():
                    found = p; break
            if found is None:
                break
            found.SetInstanceable(False)
            n += 1
        print(f"[Harvester] 그리퍼 인스턴스 해제 {n}개 (콜라이더 노출)", flush=True)

    def _bind_gripper_friction(self, stage: Usd.Stage, gripper_path: str, log) -> int:
        """그리퍼 손가락·패드 콜라이더에 마찰 재질을 건다 — §5.1 마찰 파지의 그리퍼 쪽.

        ★ grip_base(base_link)나 컨테이너에 weakerThanDescendants 로 걸면 그 자식인 가동날
          (CutterBlade)까지 물려받아 블레이드가 깨진다(2026-07-22 실측). 그래서 grip_base 는
          건너뛰고 개별 콜라이더에만 직접 바인딩한다 — 손가락 μ 를 과실(0.9)과 맞춰 spike01
          (μ≥0.5 → 2N 유지)이 검증한 파지가 실제로 성립하게 한다.
        """
        from scene.physics import create_physics_material, bind_physics_material
        ee = self._cfg.end_effector
        mat = create_physics_material(
            stage, "/World/PhysicsMaterials/gripper_pad",
            ee.pad_static_friction, ee.pad_dynamic_friction)
        # ★ 2F-85 콜라이더는 PhysxCollisionAPI 로 걸려 있어 UsdPhysics.CollisionAPI 만
        #   보면 0개가 잡힌다(2026-07-22 실측: "콜라이더 0개" → 과실 미끄러져 낙하).
        #   스파이크 grasp_force_test(μ0.9 로 과실 유지 검증)와 동일하게 둘 다 본다.
        #   CadJig/Blade/Cutter/Camera/D455 는 경로로 제외(마찰 불필요 + 블레이드 보호).
        n = 0
        bound, allcoll = [], []
        for p in Usd.PrimRange(stage.GetPrimAtPath(gripper_path)):
            path = str(p.GetPath())
            is_coll = (p.HasAPI(UsdPhysics.CollisionAPI)
                       or p.HasAPI(PhysxSchema.PhysxCollisionAPI))
            if is_coll:
                allcoll.append(path)
            if path == self._grip_base:                    # 블레이드가 이 자식 → 건너뜀
                continue
            # 구형 별도 Cutter/Blade는 제외하지만, 새 CutterQuarter3는 바구니의
            # 바깥쪽 1/4구이므로 반드시 과실 충돌·마찰이 있어야 한다. 이름에
            # "Cutter"가 들어간다는 이유로 제외하면 3/4구 한 면이 물리적으로 비어
            # 절단 순간 과실이 그대로 낙하한다.
            scoop_outer_shell = "CutterQuarter3" in path
            if (not scoop_outer_shell
                    and any(k in path for k in (
                        "CadJig", "Blade", "blade", "Cutter", "Camera", "D455"))):
                continue
            if is_coll:
                # 실제 Robotiq 고무패드의 눌림/감김을 강체 메시가 표현하지 못하므로
                # 접촉 패드에 얇은 compliant skin을 rest/contact offset으로 근사한다.
                if "finger4step" in path.lower():
                    px_coll = PhysxSchema.PhysxCollisionAPI.Apply(p)
                    # restOffset>0은 두 형상을 떨어뜨려 놓으므로 고무 눌림과 반대다.
                    # 음수 restOffset으로 1.5mm 침투를 허용하고 contactOffset에서 미리
                    # 접촉을 생성해 얇은 줄기에도 안정적인 정상력이 생기게 한다.
                    rest = float(os.environ.get("GRIP_PAD_REST_OFFSET", "-0.0015"))
                    contact = float(os.environ.get("GRIP_PAD_CONTACT_OFFSET", "0.003"))
                    px_coll.CreateRestOffsetAttr().Set(rest)
                    px_coll.CreateContactOffsetAttr().Set(max(contact, rest + 0.001))
                bind_physics_material(p, mat)
                n += 1
                bound.append(path.replace(gripper_path, "…"))
        print(f"[Harvester] 그리퍼 콜라이더 {n}개 마찰 바인딩 μs={ee.pad_static_friction} "
              f"(전체 콜라이더 {len(allcoll)}개) — {bound}", flush=True)
        if n == 0:                                         # 진단: 서브트리 콜라이더 실태
            print(f"[Harvester] ⚠콜라이더 0개! 서브트리 전체 콜라이더={allcoll[:20]}",
                  flush=True)
        return n

    def _attach_gripper(self, stage: Usd.Stage, arm_path: str,
                        gripper_path: str, log) -> str | None:
        """동축 스쿱 Base를 팔의 ee_joint(툴 소켓, b0=wrist_3)에 물린다.

        1) 스쿱 강체 Base를 찾고
        2) ee_joint 의 b0 프레임(툴 플랜지) 월드 포즈를 계산해 그 자리에 놓고
        3) CAD의 U자 수용부가 아래를 향하도록 tool Z축 둘레로 180° 장착하고
        4) ee_joint 의 b1 을 Base로 채운다.
        """
        grip_base = None
        for prim in Usd.PrimRange(stage.GetPrimAtPath(gripper_path)):
            if (prim.GetName() == "Base"
                    and prim.HasAPI(UsdPhysics.RigidBodyAPI)):
                grip_base = str(prim.GetPath())
                break
        ee = stage.GetPrimAtPath(f"{arm_path}/joints/ee_joint")
        if grip_base is None or not ee.IsValid():
            log("[Harvester] ⚠ ee_joint 또는 스쿱 Base를 못 찾음 — 미장착")
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
        self._tool0_m = Gf.Matrix4d(m_socket)
        mount_roll = Gf.Matrix4d()
        mount_roll.SetRotate(Gf.Rotation(Gf.Vec3d(0.0, 0.0, 1.0), 180.0))
        m_mount = mount_roll * m_socket              # 로컬 tool-Z 180°: U자 바닥을 아래로

        # 컨테이너를 옮겨 CAD 원점(Base)이 회전된 장착 소켓과 정확히 일치하게 한다.
        m_grip = cache.GetLocalToWorldTransform(stage.GetPrimAtPath(grip_base))
        m_cont = cache.GetLocalToWorldTransform(stage.GetPrimAtPath(gripper_path))
        m_parent = cache.GetLocalToWorldTransform(
            stage.GetPrimAtPath(gripper_path).GetParent())
        m_local = m_cont * m_grip.GetInverse() * m_mount * m_parent.GetInverse()
        set_pose(stage.GetPrimAtPath(gripper_path),
                 m_local.ExtractTranslation(),
                 m_local.ExtractRotationQuat())

        # 조인트 프레임을 실제 배치된 상대 포즈로 다시 쓴다.
        pos, rot = self._rel_pose(stage, wrist, grip_base)
        j.GetBody1Rel().SetTargets([grip_base])
        j.CreateLocalPos0Attr().Set(pos)
        j.CreateLocalRot0Attr().Set(rot)
        j.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0))
        j.CreateLocalRot1Attr().Set(Gf.Quatf(1.0))
        j.CreateJointEnabledAttr().Set(True)
        j.CreateExcludeFromArticulationAttr().Set(False)

        # 동축 1/4구 셸은 같은 중심을 공유하고 서로 스치며 회전한다. CAD 공차가 있어도
        # 삼각 메시의 contactOffset 때문에 내부 접촉이 생성되면, 닫힌 운동사슬처럼 큰
        # 반발력이 생겨 팔 전체 관절값이 발산한다. 메커니즘 내부 4개 강체끼리만 충돌을
        # 필터링하고 과실/줄기와의 외부 충돌은 그대로 유지한다.
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
        log(f"[Harvester] 동축 스쿱 장착: ee_joint → {grip_base} "
            f"(tool-Z 180°; U자 수용부 아래)")
        log(f"[Harvester] 동축 스쿱 내부 충돌 필터: {filtered}쌍 "
            f"(과실/줄기 외부 충돌은 유지)")
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
        # blade_dummy 는 finalize() 의 attach_blade_hinge() 가 CAD 원본 포즈와
        # 서보 피벗을 읽는 기준 prim. 가동 사본을 만든 뒤 원본은 비활성화한다.
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

        {grip_base}/D455 아래 UsdGeom.Camera 중 'Color'를 우선한다. 구형 CAD 지그의
        /D455/asset 구조도 하위 탐색으로 그대로 지원한다.
        ROS2 카메라 브리지(ros/robot_bridge.build_camera)가 이 경로로 렌더프로덕트를 만든다.
        """
        if not self._grip_base:
            return None
        camera_root = stage.GetPrimAtPath(f"{self._grip_base}/D455")
        if not camera_root.IsValid():
            return None
        cams = [p for p in Usd.PrimRange(camera_root) if p.IsA(UsdGeom.Camera)]
        if not cams:
            return None
        cam = next((p for p in cams if "Color" in p.GetName()), cams[0])
        return str(cam.GetPath())

    def camera_paths(self, stage: Usd.Stage) -> dict[str, str]:
        """장착된 D455 센서 경로.

        키는 ROS 브리지와 공통으로 ``color/depth/infra1/infra2/imu``를 쓴다.
        예전 단일 Camera 모델도 color/depth 폴백으로 계속 사용할 수 있다.
        """
        if not self._grip_base:
            return {}
        root = f"{self._grip_base}/D455"
        wanted = {
            "color": f"{root}/Color",
            "depth": f"{root}/Depth",
            "infra1": f"{root}/Infra1",
            "infra2": f"{root}/Infra2",
            "imu": f"{root}/Imu",
        }
        found = {name: path for name, path in wanted.items()
                 if stage.GetPrimAtPath(path).IsValid()}
        if "color" not in found:
            legacy = self.camera_path(stage)
            if legacy:
                found["color"] = legacy
        if "depth" not in found and "color" in found:
            found["depth"] = found["color"]
        return found

    def _add_scoop_camera(self, stage: Usd.Stage, log) -> None:
        """실제 D455 외형과 Color/Depth/IR/IMU만 골라 동축 스쿱에 장착한다.

        ``/Root/RSD455`` 전체를 참조하면 그 루트의 RigidBodyAPI가 로봇 articulation
        안에 중첩되고 PhysX tensor view가 별도 RSD455 로봇을 만들려 한다. 그래서
        원본 USD의 Visual과 각 센서 prim만 개별 참조한다. 원본 센서 간 baseline과
        intrinsics는 유지하면서 강체·조인트·콜라이더는 하나도 들어오지 않는다.
        """
        if not self._grip_base:
            return
        # 실행 중 Usd.Stage.Open으로 D455를 다시 검증하면 Kit 5.1이 현재 stage와
        # resolver 작업을 겹쳐 조용히 종료되는 경우가 있다. base/arm resolve에서 이미
        # 확보한 같은 Assets root에, GPU probe로 확인한 D455 경로를 바로 붙인다.
        url = assets.assets_root() + self._cfg.assets.camera[0]
        root = f"{self._grip_base}/D455"
        root_prim = UsdGeom.Xform.Define(stage, root).GetPrim()
        print(f"[Harvester] D455 안전 하위 prim 로딩: {url}", flush=True)

        # 원본 D455의 안전한 하위 prim만 가져온다. 이름은 로봇마다 root 아래에 있으므로
        # 같은 씬에 여러 대를 띄워도 prim 이름이 충돌하지 않는다.
        refs = {
            "Visual": "/Root/RSD455/Visual",
            "Color": "/Root/RSD455/Camera_OmniVision_OV9782_Color",
            "Depth": "/Root/RSD455/Camera_Pseudo_Depth",
            "Infra1": "/Root/RSD455/Camera_OmniVision_OV9782_Left",
            "Infra2": "/Root/RSD455/Camera_OmniVision_OV9782_Right",
            "Imu": "/Root/RSD455/Imu_Sensor",
        }
        for name, source in refs.items():
            prim = stage.DefinePrim(f"{root}/{name}")
            prim.GetReferences().AddReference(url, source)

        # Visual 메시에는 의미 없는 MassAPI가 일부 붙어 있다. RigidBody가 없으므로
        # 물리에 참여하지 않지만 혼동과 향후 스키마 전파를 막기 위해 명시적으로 제거한다.
        visual_root = stage.GetPrimAtPath(f"{root}/Visual")
        for prim in Usd.PrimRange(visual_root):
            if prim.HasAPI(UsdPhysics.MassAPI):
                prim.RemoveAPI(UsdPhysics.MassAPI)
            if prim.HasAPI(UsdPhysics.CollisionAPI):
                prim.RemoveAPI(UsdPhysics.CollisionAPI)
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                prim.RemoveAPI(UsdPhysics.RigidBodyAPI)

        # Visual만 따로 참조하면 원본의 sibling Looks 관계는 참조 경계 밖이 된다.
        # 실제 D455의 검정 ABS/전면 유리/금속 마운트 인상을 로컬 재질로 복원한다.
        looks = f"{root}/Looks"
        def _material(name: str, color: Gf.Vec3f, metallic=0.0, roughness=0.35):
            mat = UsdShade.Material.Define(stage, f"{looks}/{name}")
            shader = UsdShade.Shader.Define(stage, f"{looks}/{name}/Shader")
            shader.CreateIdAttr("UsdPreviewSurface")
            shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(color)
            shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(float(metallic))
            shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(float(roughness))
            mat.CreateSurfaceOutput().ConnectToSource(
                shader.CreateOutput("surface", Sdf.ValueTypeNames.Token))
            return mat

        black = _material("BlackABS", Gf.Vec3f(0.025, 0.028, 0.032), 0.0, 0.28)
        glass = _material("SensorGlass", Gf.Vec3f(0.015, 0.030, 0.045), 0.15, 0.08)
        metal = _material("MountMetal", Gf.Vec3f(0.30, 0.32, 0.35), 0.85, 0.24)
        for prim in Usd.PrimRange(visual_root):
            if not prim.IsA(UsdGeom.Mesh):
                continue
            lname = prim.GetName().lower()
            mat = (glass if ("glass" in lname or "mask" in lname)
                   else metal if ("mount" in lname or "usb" in lname)
                   else black)
            binding = UsdShade.MaterialBindingAPI.Apply(prim)
            binding.UnbindAllBindings()
            binding.Bind(mat)

        ee = self._cfg.end_effector
        eye = Gf.Vec3d(*ee.camera_offset)
        target = Gf.Vec3d(0.0, 0.0, ee.grasp_reach_z)
        forward = (target - eye).GetNormalized()
        up_hint = Gf.Vec3d(0.0, 1.0, 0.0)
        if abs(Gf.Dot(forward, up_hint)) > 0.95:
            up_hint = Gf.Vec3d(1.0, 0.0, 0.0)
        right = Gf.Cross(forward, up_hint).GetNormalized()
        up = Gf.Cross(right, forward).GetNormalized()
        # USD Camera는 로컬 -Z가 광축이다.
        rot = Gf.Matrix3d(
            right[0], right[1], right[2],
            up[0], up[1], up[2],
            -forward[0], -forward[1], -forward[2])
        q = Gf.Matrix4d(rot, Gf.Vec3d(0.0)).ExtractRotationQuat()

        # 원본 Color 카메라의 RSD455-root 상대 자세로 D455 mount를 역산한다.
        # 이 값은 Isaac 5.1 rsd455.usd에서 직접 측정한 값(컬러 센서 Y=+11.5mm).
        # 원격 참조 직후 XformCache로 센서 prim을 읽으면 스키마 비동기 로딩과 경합해
        # Kit가 종료될 수 있으므로 런타임 재측정은 하지 않는다.
        set_pose(root_prim, (0.0, 0.0, 0.0), Gf.Quatd(1.0))
        color_rel = Gf.Matrix4d(
            0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 1.0, 0.0,
            1.0, 0.0, 0.0, 0.0,
            0.0, 0.0115, 0.0, 1.0)
        desired = Gf.Matrix4d(1.0)
        desired.SetRotate(q)
        desired.SetTranslateOnly(eye)
        root_local = color_rel.GetInverse() * desired
        _set_full_pose(root_prim, root_local)

        # 센서가 스쿱 위에서 떠 보이지 않도록 얇은 L 브래킷을 추가한다.
        # 시각 전용이며 MoveIt 쪽에는 같은 크기의 d455_mount 충돌박스를 둔다.
        bracket_root = f"{self._grip_base}/D455Mount"
        UsdGeom.Xform.Define(stage, bracket_root)
        for name, pos, size in (
            ("Foot", (0.0, 0.030, -0.014), (0.060, 0.008, 0.040)),
            ("Arm", (0.0, 0.064, -0.029), (0.090, 0.060, 0.006)),
        ):
            cube = UsdGeom.Cube.Define(stage, f"{bracket_root}/{name}")
            cube.CreateSizeAttr(1.0)
            cube.CreateDisplayColorAttr([Gf.Vec3f(0.18, 0.20, 0.23)])
            set_pose(cube.GetPrim(), pos, Gf.Quatd(1.0))
            UsdGeom.Xformable(cube.GetPrim()).AddScaleOp().Set(Gf.Vec3f(*size))

        for name in ("Color", "Depth", "Infra1", "Infra2"):
            cam = UsdGeom.Camera(stage.GetPrimAtPath(f"{root}/{name}"))
            if cam:
                cam.CreateClippingRangeAttr(Gf.Vec2f(0.02, 20.0))
        log(f"[Harvester] 실제 D455 장착: {root} "
            "(Color+Depth+좌/우 IR+IMU, 무강체·무콜라이더, 원본 캘리브레이션)")

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

    def log_gripper_alignment(self, stage: Usd.Stage, log=print) -> bool:
        """실제 Isaac 패드 중심·닫힘축과 HarvestTCP를 출력한다.

        URDF의 수치만 믿고 ±90° 롤을 뒤집지 않도록 런타임 에셋에서 직접 잰다.
        닫힘축은 접근축과 거의 직교해야 하며 TCP는 좌우 패드 중점 근처여야 한다.
        """
        if not self._grip_base or not self._grasp_tcp:
            return False
        left = right = None
        root = stage.GetPrimAtPath(self._gripper_path)
        for prim in Usd.PrimRange(root):
            if prim.GetName() == "left_inner_finger":
                left = prim
            elif prim.GetName() == "right_inner_finger":
                right = prim
        if left is None or right is None:
            log("[Harvester] ⚠ 패드 링크를 못 찾아 그리퍼 축 진단 생략")
            return False
        cache = UsdGeom.XformCache()
        bbox = UsdGeom.BBoxCache(
            Usd.TimeCode.Default(),
            [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
        # 링크 전체 bbox에는 너클까지 섞인다. 실제 평평 패드(finger4step) 메시를
        # 우선 사용하고, 에셋 이름이 바뀐 경우에만 링크 bbox로 폴백한다.
        def _pad_mesh(link):
            for child in Usd.PrimRange(link):
                if child.IsA(UsdGeom.Mesh) and "finger4step" in child.GetName().lower():
                    return child
            return link

        left = _pad_mesh(left)
        right = _pad_mesh(right)
        lc = bbox.ComputeWorldBound(left).ComputeAlignedRange().GetMidpoint()
        rc = bbox.ComputeWorldBound(right).ComputeAlignedRange().GetMidpoint()
        midpoint = (Gf.Vec3d(lc) + Gf.Vec3d(rc)) * 0.5
        close_axis = (Gf.Vec3d(lc) - Gf.Vec3d(rc)).GetNormalized()
        grip_m = cache.GetLocalToWorldTransform(stage.GetPrimAtPath(self._grip_base))
        approach_axis = Gf.Vec3d(
            grip_m.TransformDir(Gf.Vec3d(0, 0, 1))).GetNormalized()
        tcp = cache.GetLocalToWorldTransform(
            stage.GetPrimAtPath(self._grasp_tcp)).ExtractTranslation()
        tcp_error = (Gf.Vec3d(tcp) - midpoint).GetLength()
        orthogonality = abs(Gf.Dot(close_axis, approach_axis))
        log("[Harvester] 그리퍼 축 실측: "
            f"close=({close_axis[0]:+.3f},{close_axis[1]:+.3f},{close_axis[2]:+.3f}) "
            f"approach=({approach_axis[0]:+.3f},{approach_axis[1]:+.3f},"
            f"{approach_axis[2]:+.3f}) dot={orthogonality:.3f} "
            f"TCP↔패드중점={tcp_error*1000:.1f}mm")
        if orthogonality > 0.15 or tcp_error > 0.015:
            log("[Harvester] ⚠ MoveIt 파지 전 그리퍼 축/TCP 정합 확인 필요")
        return True

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
            if prim.IsA(UsdPhysics.Joint):
                prim.SetActive(False)
                continue
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                prim.RemoveAPI(UsdPhysics.RigidBodyAPI)
            if prim.HasAPI(PhysxSchema.PhysxRigidBodyAPI):
                prim.RemoveAPI(PhysxSchema.PhysxRigidBodyAPI)
            if prim.HasAPI(UsdPhysics.CollisionAPI):
                prim.RemoveAPI(UsdPhysics.CollisionAPI)
            if prim.HasAPI(PhysxSchema.PhysxCollisionAPI):
                prim.RemoveAPI(PhysxSchema.PhysxCollisionAPI)
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
        """CAD ``blade_dummy``를 제자리에서 서보축 중심으로 스윙시킨다.

        ``CadJig`` 아래 원본 prim의 로컬 포즈를 rest로 사용해 지그·서보와
        날의 CAD 배치를 유지한다. CAD 축=Y(서보샤프트), 피벗=(0,53,132).
        """
        if not self._grip_base:
            log("[Harvester] ⚠ grip_base 없음 — 블레이드 힌지 미부착")
            return False
        # 이전 조립 방식의 별도 사본이 저장 Stage/핫리로드에 남아 있으면
        # CAD 원본 날과 동시에 보인다. 제자리 blade_dummy만 사용하도록 제거한다.
        for legacy_path in (f"{self._grip_base}/CutterBlade",
                            f"{self._root}_CutterBlade"):
            if stage.GetPrimAtPath(legacy_path).IsValid():
                stage.RemovePrim(legacy_path)
                log(f"[Harvester] 기존 공중 블레이드 제거: {legacy_path}")
        source_path = f"{self._grip_base}/CadJig/blade_dummy"
        blade = stage.GetPrimAtPath(source_path)
        if not blade.IsValid():
            log(f"[Harvester] ⚠ CAD 기준날 없음: {source_path} — 블레이드 미부착")
            return False

        cache = UsdGeom.XformCache()
        M_orig = cache.GetLocalToWorldTransform(blade)
        shaft_w = M_orig.Transform(Gf.Vec3d(0, 53, 132))                 # 서보축(SHAFT_Z)
        axis_w = Gf.Vec3d(M_orig.TransformDir(Gf.Vec3d(0, 1, 0))).GetNormalized()
        self._blade_shaft_w = Gf.Vec3d(shaft_w)                          # 절단점 저장(줄기 배치용)

        # CAD 원본 날을 별도 사본으로 재배치하지 않고 그 자리에서 돌린다.
        # 그러면 CadJig의 0.1 스케일과 reference 내부 xform이 중복 적용되지 않아
        # 날이 서보 아래 CAD 위치에 그대로 남는다.
        parent = blade.GetParent()                                    # .../base_link/CadJig
        M_parent_inv = cache.GetLocalToWorldTransform(parent).GetInverse()
        L_rest = UsdGeom.Xformable(blade).GetLocalTransformation()
        # translate/orient/scale 분해는 변환기가 준 축·스케일 행렬을 손실시켜
        # 날을 지그 밖으로 밀어냈다. 원래 로컬 행렬을 단일 op로 그대로 저장한다.
        blade.RemoveProperty("xformOp:transform:bladeServo")
        blade_xf = UsdGeom.Xformable(blade)
        blade_xf.ClearXformOpOrder()
        self._blade_pose_op = blade_xf.AddTransformOp(opSuffix="bladeServo")
        self._blade_pose_op.Set(L_rest)
        self._blade_path = source_path
        self._blade_L_rest = L_rest                                   # rest 로컬(CadJig 기준)
        self._shaft_rel = M_parent_inv.Transform(Gf.Vec3d(shaft_w))   # 피벗 ← CadJig 로컬
        self._axis_rel = Gf.Vec3d(
            M_parent_inv.TransformDir(Gf.Vec3d(axis_w))).GetNormalized()  # 축 ← CadJig 로컬
        self._blade_deg = self.BLADE_OPEN_DEG                         # rest = 열림(export 자세)
        self._blade_target_deg = self.BLADE_OPEN_DEG
        meshes = [p for p in Usd.PrimRange(blade) if p.IsA(UsdGeom.Mesh)]
        log(f"[Harvester] CAD 가동날 제자리 활성: {source_path} "
            f"(parent={parent.GetPath()}, mesh={len(meshes)}개, 피벗=(0,53,132))")
        return True

    def sync_blade_pose(self, stage: Usd.Stage, dt: float = 1.0 / 60.0) -> None:
        """CadJig 아래 원본 가동날에 서보 각도를 로컬로 반영한다.

        자식이라도 물리링크(grip_base) 월드가 USD 로 전파 안 오면 날이 뜬다 → 여기서
        grip_base 현재 월드를 읽어 **날의 월드포즈를 직접 세팅**한다(로컬 세팅이 아니라).
        rest(열림) 에서 (현재각-열림각)만큼 피벗축 회전만 얹는다. 절단은 detach_fruit(§5.3)."""
        if self._blade_L_rest is None or not self._blade_path:
            return
        hp = stage.GetPrimAtPath(self._blade_path)
        if not hp.IsValid() or self._blade_pose_op is None:
            return
        # 명령 각으로 순간이동하지 않고 서보 회전 속도로 닫힌다.
        error = self._blade_target_deg - self._blade_deg
        max_step = self.BLADE_SPEED_DEG_S * max(0.0, float(dt))
        if abs(error) <= max_step:
            self._blade_deg = self._blade_target_deg
        elif error > 0.0:
            self._blade_deg += max_step
        else:
            self._blade_deg -= max_step

        theta = self._blade_deg - self.BLADE_OPEN_DEG        # 열림 기준 서보 변위 [deg]
        if abs(theta) > 1e-6:                                # 피벗축(CadJig 로컬) 중심 회전
            swing = (Gf.Matrix4d().SetTranslate(-self._shaft_rel)
                     * Gf.Matrix4d().SetRotate(Gf.Rotation(self._axis_rel, theta))
                     * Gf.Matrix4d().SetTranslate(self._shaft_rel))
            L = self._blade_L_rest * swing
        else:
            L = self._blade_L_rest
        # 행렬 분해 없이 원본 CAD 로컬 행렬+서보 회전을 직접 저작한다.
        self._blade_pose_op.Set(L)

    def set_blade_deg(self, deg: float) -> None:
        """가동날 각도 명령 [deg]. ★§5.6: ROS2 노드가 토픽 받아 이 메서드를 부른다.
        열림 0° ~ 닫힘 35°(=절단). 다음 sync_blade_pose() 가 이 각으로 날을 회전 배치한다."""
        self._blade_target_deg = max(
            self.BLADE_OPEN_DEG,
            min(self.BLADE_CLOSED_DEG, float(deg)))

    def open_blade(self) -> None:
        """가동날 열기 (미부착이면 no-op)."""
        self.set_blade_deg(self.BLADE_OPEN_DEG)

    def close_blade(self) -> None:
        """가동날 닫기 = 절단 자세."""
        self.set_blade_deg(self.BLADE_CLOSED_DEG)

    def move_blade(self, d_deg: float) -> None:
        """가동날 각도 증분 [deg] — 텔레옵/증분 제어용. [열림, 닫힘]로 제한."""
        self.set_blade_deg(self._blade_target_deg + d_deg)

    def blade_deg(self) -> float:
        """현재 가동날의 실제 연출 각도 [deg]."""
        return self._blade_deg

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
