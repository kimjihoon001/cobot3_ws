# -*- coding: utf-8 -*-
"""m0617 URDF → 네이티브 아티큘레이션 USD 임포트 (반입 m0617.usd 손상 대체).

워크스페이스 m0617.usd 는 이 Isaac 이 못 읽는 크레이트라(크레이트 read 실패) 못 쓴다.
공식 dsr_description2 xacro+메시로 만든 robots/m0617/m0617_full.urdf 를 Isaac URDF
Importer 로 임포트해 이 Isaac 이 읽을 수 있는 USD 를 생성한다.

핵심 옵션:
  fix_base=True        → root_joint(월드고정)+ArticulationRootAPI 생성. harvester._mount_arm
                         이 이 root_joint 를 Ridgeback 섀시로 재배선(UR10e 와 동일 패턴).
  make_default_prim=True→ defaultPrim 설정 → add_reference_to_stage 가 제대로 끌어온다
                         (손상 USD 는 이게 없어 빈 Xform 이었다).
  merge_fixed_joints=False, self_collision=False.

실행:
  cd ~/cobot3_ws/isaacpjt && isaac_python tools/import_m0617_urdf.py
  → robots/m0617/m0617_isaac/m0617.usd 생성. 이후 probe_m0617.py 의 USD_PATH 를 이걸로
    바꿔 구조 확인 → settings.py arm 을 이 경로로 교체.
"""
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

from pathlib import Path

import omni.kit.commands
from isaacsim.core.utils.extensions import enable_extension

enable_extension("isaacsim.asset.importer.urdf")
simulation_app.update()

BASE = Path(__file__).resolve().parent.parent / "robots" / "m0617"
URDF = str(BASE / "m0617_full.urdf")
OUT_DIR = BASE / "m0617_isaac"
OUT_DIR.mkdir(exist_ok=True)
OUT_USD = str(OUT_DIR / "m0617.usd")

status, import_config = omni.kit.commands.execute("URDFCreateImportConfig")
import_config.merge_fixed_joints = False
import_config.convex_decomp = False
import_config.fix_base = True
import_config.make_default_prim = True
import_config.self_collision = False
import_config.distance_scale = 1.0
import_config.density = 0.0
# 위치 드라이브(RMPflow position 타깃 추종). mm.py 가 런타임에 게인 강화하지만
# 임포트 기본값도 0 이 아니어야 한다. 버전차 대비 try 로 감싼다.
try:
    from isaacsim.asset.importer.urdf import _urdf
    import_config.default_drive_type = _urdf.UrdfJointTargetType.JOINT_DRIVE_POSITION
except Exception as exc:  # noqa: BLE001
    print(f"[import] drive_type 기본값 유지: {exc}")
try:
    import_config.default_drive_strength = 1e7
    import_config.default_position_drive_damping = 1e5
except Exception as exc:  # noqa: BLE001
    print(f"[import] drive gain 기본값 유지: {exc}")

print(f"[import] URDF: {URDF}")
print(f"[import] OUT : {OUT_USD}")
status, prim_path = omni.kit.commands.execute(
    "URDFParseAndImportFile",
    urdf_path=URDF,
    import_config=import_config,
    dest_path=OUT_USD,
)
print(f"[import] status={status}  prim_path={prim_path}")

# 생성 결과 즉시 검증 — defaultPrim/루트/조인트 확인
from pxr import Usd, UsdPhysics
stage = Usd.Stage.Open(OUT_USD)
if stage is None:
    print("[import] ⚠ 생성 USD 를 다시 못 엶")
else:
    dp = stage.GetDefaultPrim()
    print(f"[import] defaultPrim: {dp.GetPath() if dp else '(없음)'}")
    joints, roots = [], []
    for prim in stage.Traverse():
        if prim.IsA(UsdPhysics.RevoluteJoint):
            joints.append(prim.GetName())
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            roots.append(str(prim.GetPath()))
        if prim.GetName() == "root_joint":
            print(f"[import] root_joint 존재: {prim.GetPath()}")
    print(f"[import] ArticulationRoot: {roots}")
    print(f"[import] revolute 조인트({len(joints)}): {joints}")

print("[import] 완료. probe_m0617.py 의 USD_PATH 를 위 OUT 경로로 바꿔 재확인할 것.")
simulation_app.close()
