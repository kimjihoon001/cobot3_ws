# -*- coding: utf-8 -*-
"""꽃자루(pedicel) — 과실을 줄기에 매다는 자루. 지오메트리 + 파단 조인트.

이게 없으면 [W2024] 에서 뽑은 절단력/파단력/굽힘모멘트가 적용될 대상이 없다.
(그 전까지 과실은 줄기에서 9cm 떨어진 허공에 kinematic 으로 떠 있었다.)

구조 — [W2024] Figure 2 의 3부분을 그대로 따른다:

  줄기 ──[proximal]──[이탈층]──╫──[distal]── 꼭지 ── 과실
         └─ 줄기의 자식 (static) ─┘  ↑  └ 과실의 자식 ┘
                                  FixedJoint

물리 설계 — 조인트 하나가 이탈층 역할을 한다:
  꽃자루를 3개 강체 + 4개 조인트로 만들면 과실 400개에 조인트 1600개가 된다.
  대신 지오메트리는 시각/충돌용으로 두고, **물리는 조인트 하나가 대표**한다.
  그 조인트의 breakForce/breakTorque 가 곧 이탈층의 파단 특성이다:
    - 로봇이 세게 당기면       -> 40.262 N 에서 끊김 (당기기 실패 모드)
    - 과실을 비틀면            -> 0.067 N·m 에서 끊김
    - 커터가 distal 을 자르면  -> jointEnabled=False (결정적, 되돌릴 수 있음)

분할 지점이 distal 인 이유:
  [W2024] Table 4 — proximal 전단력 62.054N vs distal 33.241N (proximal 이 85.32% 큼).
  distal 을 자르면 힘도 덜 들고 과실에 남는 자루도 짧다. 그래서 distal 만 과실의
  자식이고, 자르면 과실이 그걸 달고 떠난다. 줄기엔 proximal+이탈층이 남는다.

Isaac 을 import 하지 않는다 (순수 pxr) — GPU 없이 tests/test_pedicel.py 로 검증된다.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

from pxr import Gf, Usd, UsdGeom, UsdPhysics

PEDICEL_COLOR = Gf.Vec3f(0.30, 0.50, 0.18)


@dataclass
class PedicelConfig:
    """치수 근거 [W2024] — 품종 Syngenta Spectrum, early firm-ripening.

    지름은 논문 실측 분포의 대표값을 쓴다. 길이 비율은 근거가 없다(아래 TODO).
    """
    # 2.4.1절 "distal and proximal pedicel diameters were distributed between
    # 3 and 5 mm", Table 9 검증시료 proximal 평균 3.23~3.59mm
    proximal_diameter: float = 0.0035          # m

    # 2.3절 "majority of the tomato abscission zone diameters ... 5-8 mm".
    # 5~6mm 그룹을 쓴다 — settings.py 의 break_force/break_torque 와 같은 그룹.
    # (그 그룹 기준으로 40.262N / 0.067N·m 를 뽑았으므로 지름도 맞춰야 일관된다)
    abscission_diameter: float = 0.0055        # m

    # 2.4.2절 "most of the tomato distal pedicels were between 3.5-4 mm"
    distal_diameter: float = 0.00375           # m

    # TODO 근거 없음. [W2024] 는 distal/proximal 길이를 재긴 했지만(2.1절)
    #      값을 표에 싣지 않았다. 아래는 전체 길이(=기존 fruit_offset)를
    #      셋으로 나눈 비율일 뿐이다. 품종 자료나 재배 사진에서 확보할 것.
    proximal_ratio: float = 0.45
    abscission_ratio: float = 0.15
    distal_ratio: float = 0.40

    def segment_lengths(self, total: float) -> tuple[float, float, float]:
        s = self.proximal_ratio + self.abscission_ratio + self.distal_ratio
        return (total * self.proximal_ratio / s,
                total * self.abscission_ratio / s,
                total * self.distal_ratio / s)


def _segment(stage: Usd.Stage, path: str, start: Gf.Vec3d, end: Gf.Vec3d,
             diameter: float) -> None:
    """start->end 를 잇는 원기둥. 축 방향으로 회전시켜 놓는다."""
    d = end - start
    length = d.GetLength()
    cyl = UsdGeom.Cylinder.Define(stage, path)
    r = diameter / 2.0
    cyl.CreateRadiusAttr(r)
    cyl.CreateHeightAttr(length)
    cyl.CreateAxisAttr("Z")
    cyl.CreateExtentAttr([Gf.Vec3f(-r, -r, -length / 2.0),
                          Gf.Vec3f(r, r, length / 2.0)])
    cyl.CreateDisplayColorAttr([PEDICEL_COLOR])

    xf = UsdGeom.Xformable(cyl.GetPrim())
    xf.AddTranslateOp().Set((start + end) / 2.0)
    # +Z 를 d 방향으로 돌린다
    rot = Gf.Rotation(Gf.Vec3d(0, 0, 1), d)
    xf.AddOrientOp().Set(Gf.Quatf(rot.GetQuat()))
    # 콜라이더를 안 붙인다 — 물리는 파단 조인트가 대표한다. 콜라이더를 붙이면
    # 과실쪽 distal 과 줄기쪽 abscission 이 접촉해 조인트와 싸워 과실이 떨리고(지터),
    # 조인트를 끊어도 그 접촉이 과실을 받쳐 안 떨어진다 (spike 02 에서 확인). 시각 전용.


def spawn(stage: Usd.Stage, stem_path: str, fruit_path: str,
          stem_point: tuple[float, float, float],
          fruit_point: tuple[float, float, float],
          cfg: PedicelConfig, break_force: float, break_torque: float,
          joint_path: str | None = None, viz_root: str | None = None) -> str:
    """줄기와 과실 사이에 꽃자루를 놓고 파단 조인트로 잇는다.

    stem_point  : 줄기 쪽 부착점 (월드)
    fruit_point : 과실 쪽 부착점 (월드, 보통 과실 중심)
    viz_root    : 시각 세그먼트를 놓을 부모(월드 좌표를 쓰므로 **변환 없는 prim** 이어야
                  한다). 씬에서 줄기·과실 Xform 은 이동/스케일이 걸려 있어 그 밑에 두면
                  세그먼트가 어긋나거나(이동) 찌그러진다(과실 스케일 0.001675). 안 주면
                  예전처럼 stem/fruit 자식으로 둔다(스파이크는 헤드리스라 배치 무관).
    반환: 생성한 조인트의 prim 경로 (자를 때 이걸로 찾는다)
    """
    a, b = Gf.Vec3d(*stem_point), Gf.Vec3d(*fruit_point)
    total = (b - a).GetLength()
    if total <= 1e-6:
        raise ValueError("줄기와 과실이 같은 자리다 — 꽃자루를 놓을 수 없다")

    l_prox, l_absc, l_dist = cfg.segment_lengths(total)
    u = (b - a) / total                      # 줄기 -> 과실 단위벡터

    p0 = a
    p1 = a + u * l_prox                      # proximal 끝 = 이탈층 시작
    p2 = p1 + u * l_absc                     # 이탈층 끝 = distal 시작 (절단 지점)

    # 세그먼트 부모 — viz_root 를 주면 변환 없는 그 밑에(월드 좌표대로), 안 주면 예전처럼.
    if viz_root is not None:
        leaf = fruit_path.rsplit("/", 1)[-1]             # 과실별 고유 이름
        prox_p = f"{viz_root}/Ped_{leaf}_proximal"
        absc_p = f"{viz_root}/Ped_{leaf}_abscission"
        dist_p = f"{viz_root}/Ped_{leaf}_distal"
    else:
        prox_p = f"{stem_path}/Pedicel_proximal"
        absc_p = f"{stem_path}/Pedicel_abscission"
        dist_p = f"{fruit_path}/Pedicel_distal"
    # 줄기에 남는 부분
    _segment(stage, prox_p, p0, p1, cfg.proximal_diameter)
    _segment(stage, absc_p, p1, p2, cfg.abscission_diameter)
    # 과실이 달고 가는 부분 (커터가 p2 를 자른다)
    _segment(stage, dist_p, p2, b, cfg.distal_diameter)

    # 이탈층을 대표하는 조인트. 여기서 끊어진다.
    jp = joint_path or f"{fruit_path}/PedicelJoint"
    joint = UsdPhysics.FixedJoint.Define(stage, jp)
    joint.CreateBody0Rel().SetTargets([stem_path])
    joint.CreateBody1Rel().SetTargets([fruit_path])
    joint.CreateBreakForceAttr(break_force)
    joint.CreateBreakTorqueAttr(break_torque)
    joint.CreateJointEnabledAttr(True)

    # 조인트 로컬 프레임을 현재 상대 포즈로 작성한다. 안 하면 프레임이 identity 라
    # PhysX 가 줄기·과실 원점을 일치시키려 시작 순간 보정 임펄스를 줘서 과실이
    # 폭발한다 (spike 02 에서 지터 152mm 로 확인 — §8 로봇 MountJoint 와 같은 버그).
    # body1(과실) 원점에 앵커를 두고, body0(줄기) 로컬로 과실의 상대 포즈를 준다.
    cache = UsdGeom.XformCache()
    m0 = cache.GetLocalToWorldTransform(stage.GetPrimAtPath(stem_path))
    m1 = cache.GetLocalToWorldTransform(stage.GetPrimAtPath(fruit_path))

    # ★ 과실 Xform 의 스케일(0.001675)이 L2W 에 섞여 있다. rel=m1*m0⁻¹ 를 그대로 쓰면
    #   회전뿐 아니라 **위치(translation)도 스케일에 오염**돼 앵커가 어긋나고, PhysX 가
    #   시작 순간 큰 보정 토크로 스냅시켜 조인트가 끊긴다(과실 낙하). 회전만 정규화하던
    #   기존 코드는 위치 오염을 못 잡았다 — 그래서 낮은 break_torque 로는 끊겼다(spike 02
    #   재분석 2026-07-18). 스케일을 제거한 **순수 강체 변환**으로 프레임을 만든다.
    def _rigid(m):
        r = Gf.Matrix4d()
        r.SetRotate(m.ExtractRotationQuat().GetNormalized())   # 순수 회전(이동 0)
        r.SetTranslateOnly(m.ExtractTranslation())             # 이동만 덮어씀
        return r

    rel = _rigid(m1) * _rigid(m0).GetInverse()     # 스케일 없는 과실→줄기 상대 강체변환
    joint.CreateLocalPos0Attr().Set(Gf.Vec3f(rel.ExtractTranslation()))
    joint.CreateLocalRot0Attr().Set(Gf.Quatf(rel.ExtractRotationQuat().GetNormalized()))
    joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
    joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0))
    return jp


def cut(stage: Usd.Stage, joint_path: str) -> bool:
    """커터가 distal 을 자른 순간 — 조인트를 끊는다.

    prim 을 지우지 않고 jointEnabled 만 끄는 이유:
      되돌릴 수 있어서 post_reset() 에서 Play/Stop 재현성을 지킬 수 있다.
      breakForce 로 자연히 끊기길 기다리면 매번 결과가 미세하게 달라진다.
    """
    joint = UsdPhysics.Joint(stage.GetPrimAtPath(joint_path))
    if not joint:
        return False
    joint.GetJointEnabledAttr().Set(False)
    return True


def restore(stage: Usd.Stage, joint_path: str) -> bool:
    """Play/Stop 복원 — 다시 매단다."""
    joint = UsdPhysics.Joint(stage.GetPrimAtPath(joint_path))
    if not joint:
        return False
    joint.GetJointEnabledAttr().Set(True)
    return True
