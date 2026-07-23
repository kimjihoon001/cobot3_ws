# -*- coding: utf-8 -*-
"""m0617.usd 내부 구조 실측 — harvester.py 를 m0617 로 이식하기 전 필수 probe.

메모리 §8 원칙(이름을 추측하지 않는다)에 따라, 아래 항목을 GPU 에서 실측한 뒤
harvester.py 의 _mount_arm / _preset_pose / _attach_gripper 를 그 이름으로 고친다:
  - 아티큘레이션 루트 prim (PhysicsArticulationRootAPI 보유 prim)
  - root_joint(월드 고정 fixed joint) 유무 → 있으면 base_link 재배선 대상
  - base 링크 이름, 링크가 flat sibling 인지 nested 인지
  - revolute 조인트 경로·이름(joint_1..6 확인)과 joints/ 스코프 존재 여부
  - 플랜지(link_6) 프레임 = 그리퍼 웰드 지점, 툴축 방향
  - SingleArticulation.dof_names (제어 인덱스 매칭용)

실행:
  cd ~/cobot3_ws/isaacpjt && isaac_python tools/probe_m0617.py
  (GUI 창이 뜬다. 콘솔 출력을 복사해 전달할 것. 확인 후 창 닫기)
"""
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

from pathlib import Path

import omni.usd
from isaacsim.core.utils.stage import add_reference_to_stage
from pxr import Sdf, Usd, UsdGeom, UsdPhysics

ARM_ROOT = "/World/Arm"
# import_m0617_urdf.py 로 생성한 네이티브 USD 를 우선 쓴다(반입 m0617.usd 는 손상됨).
import os
_GEN = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                    "robots", "m0617", "m0617_isaac", "m0617.usd")
USD_PATH = _GEN if os.path.exists(_GEN) else \
    "/home/rokey/cobot3_ws/isaacpjt/M0609/doosan-robot2/usd/m0617.usd"

sep = "=" * 70

# ── 0단계: 레이어 자체 구조 진단 (defaultPrim 유무 = 참조가 뭘 끌어오는지 결정) ──
print("\n" + sep)
print("m0617.usd 레이어 진단")
print(sep)
layer = Sdf.Layer.FindOrOpen(USD_PATH)
if layer is None:
    print(f"⚠ 레이어를 못 엶: {USD_PATH}")
    default_prim = None
    root_names = []
else:
    default_prim = layer.defaultPrim or None
    root_names = [rp.name for rp in layer.rootPrims]
    print(f"defaultPrim: {default_prim or '(없음 — AddReference 가 아무것도 안 끌어온 원인)'}")
    print(f"루트 prim {len(root_names)}개:")
    for rp in layer.rootPrims:
        kids = [c.name for c in rp.nameChildren]
        print(f"  /{rp.name}  [{rp.typeName}]  자식:{kids[:12]}")

# ── 1단계: harvester 가 실제 쓰는 add_reference_to_stage 로 얹는다 ──
# defaultPrim 이 없으면 첫 루트 prim 을 명시 참조한다(harvester 도 이 보정이 필요).
stage = omni.usd.get_context().get_stage()
UsdGeom.Xform.Define(stage, "/World")
ref_target = default_prim
if default_prim:
    add_reference_to_stage(USD_PATH, ARM_ROOT)
else:
    UsdGeom.Xform.Define(stage, ARM_ROOT)
    prim_path = f"/{root_names[0]}" if root_names else None
    if prim_path:
        ref_target = prim_path
        stage.GetPrimAtPath(ARM_ROOT).GetReferences().AddReference(
            USD_PATH, primPath=prim_path)
        print(f"\n(defaultPrim 없음 → 루트 prim '{prim_path}' 을 명시 참조)")
print(f"참조 대상 prim: {ref_target or '(불명)'}")

for _ in range(30):
    simulation_app.update()


def rel(path: str) -> str:
    """ARM_ROOT 기준 상대경로 (harvester 는 {arm_path}/... 로 접근)."""
    p = str(path)
    return p[len(ARM_ROOT):] if p.startswith(ARM_ROOT) else p


print("\n" + sep)
print(f"m0617.usd prim 트리 (참조 루트 {ARM_ROOT})")
print(sep)
arm_prim = stage.GetPrimAtPath(ARM_ROOT)
art_roots, root_joints, rev_joints, links = [], [], [], []
for prim in Usd.PrimRange(arm_prim):
    path = prim.GetPath()
    depth = len(str(path).split("/")) - len(ARM_ROOT.split("/"))
    tname = prim.GetTypeName()
    apis = prim.GetAppliedSchemas()
    tags = []
    if "PhysicsArticulationRootAPI" in apis or "PhysxArticulationAPI" in apis:
        tags.append("ARTICULATION_ROOT")
        art_roots.append(path)
    if prim.IsA(UsdPhysics.RevoluteJoint):
        tags.append("REVOLUTE")
        rev_joints.append(path)
    if prim.IsA(UsdPhysics.FixedJoint):
        tags.append("FIXED")
        if prim.GetName() == "root_joint":
            root_joints.append(path)
    if "PhysicsRigidBodyAPI" in apis:
        tags.append("RIGID_BODY")
        links.append(path)
    tag = ("  <- " + ", ".join(tags)) if tags else ""
    print(f"{'  ' * depth}{prim.GetName()}  [{tname}]{tag}")

print("\n" + sep)
print("요약 (harvester.py 이식용)")
print(sep)
print(f"아티큘레이션 루트: {[rel(p) for p in art_roots] or '없음(⚠ 팔이 안 움직임)'}")
print(f"root_joint(월드고정): {[rel(p) for p in root_joints] or '없음'}")
print("  -> 있으면 harvester._mount_arm 이 이 fixed joint 를 Ridgeback 섀시로 재배선.")
print(f"강체(링크) {len(links)}개: {[rel(p) for p in links]}")
print(f"revolute 조인트 {len(rev_joints)}개:")
for j in rev_joints:
    jp = UsdPhysics.RevoluteJoint(stage.GetPrimAtPath(j))
    b0 = jp.GetBody0Rel().GetTargets()
    b1 = jp.GetBody1Rel().GetTargets()
    print(f"  {rel(j)}  body0={[rel(str(b)) for b in b0]} body1={[rel(str(b)) for b in b1]}")

# joints/ 스코프 존재 여부 (harvester._preset_joint 가 {arm}/joints/{name} 로 접근)
joints_scope = stage.GetPrimAtPath(f"{ARM_ROOT}/joints")
print(f"\n'joints' 스코프 존재: {joints_scope.IsValid()}"
      f"  (harvester._preset_joint 접근 경로 확인용)")

# base_link / link_6(플랜지) 프레임 실측 — 그리퍼 웰드·마운트 기준
xf = UsdGeom.XformCache()
for name in ("base_link", "link_6"):
    for cand in (f"{ARM_ROOT}/{name}", f"{ARM_ROOT}/m0617/{name}"):
        p = stage.GetPrimAtPath(cand)
        if p.IsValid():
            m = xf.GetLocalToWorldTransform(p)
            t = m.ExtractTranslation()
            print(f"{name}: {rel(cand)}  world_pos=({t[0]:.4f},{t[1]:.4f},{t[2]:.4f})")
            break

# dof_names — 제어 인덱스(control.py _JointMap) 매칭
try:
    from isaacsim.core.prims import SingleArticulation
    from isaacsim.core.api import World
    world = World()
    art = SingleArticulation(prim_path=ARM_ROOT, name="m0617")
    world.scene.add(art)
    world.reset()
    print(f"\ndof_names ({len(art.dof_names)}): {list(art.dof_names)}")
except Exception as exc:  # noqa: BLE001
    print(f"\n[dof_names 실패] {exc}  (수동으로 revolute 이름 확인)")

print("\n" + sep)
print("위 '요약' 블록을 복사해 전달하면 harvester.py 이식을 마무리한다.")
print(sep)

while simulation_app.is_running():
    simulation_app.update()

simulation_app.close()
