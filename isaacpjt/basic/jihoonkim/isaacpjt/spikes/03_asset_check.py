# -*- coding: utf-8 -*-
"""스파이크 3 — Isaac 기본 에셋에 우리 로봇이 있는가. **내일 아침 제일 먼저.**

실행: isaac_python spikes/03_asset_check.py          (headless, 5분)
      isaac_python spikes/03_asset_check.py --gui

무엇을 묻는가:
  에셋이 있으면 며칠을 번다. 없으면 직접 모델링해야 하고 그게 일정의 제일 큰 줄이다.
  **역할 분배를 이걸 모르고 하면 "AMR 2일" 이 실제로 5일이 된다.**
  v3 11.1 이 팀원 C 에게 Day1~2 로 배정한 AMR·창고랙 모델링이 통째로 사라질 수도 있다.

문서 조사 결과(2026-07-17, Isaac Sim 5.1 문서 기준) — 이 스크립트가 실물로 확인한다:
  운반 AMR : ForkliftB / ForkliftC  (7 DOF 승강)
  수확 MM  : Ridgeback (h=0.30m, 하중 100kg) + 팔
             ⚠ 기본 RidgebackUr 은 UR5 라 도달 0.4m 부족 -> UR10e 로 갈아끼워야 함
  그리퍼   : Robotiq 2F-85 / 2F-140

요구사항(확정값에서 유도. settings.py RobotConfig 참고):
  수직 0.5~1.4m (과실 높이) · 수평 0.66m (조간 1.5m) · 베이스 폭 < 조간
"""
import sys

GUI = "--gui" in sys.argv

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": not GUI})

import os

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import omni.usd
from pxr import Usd, UsdGeom, UsdPhysics

from pjt_config.settings import SceneConfig
from pjt_utils.paths import short

CFG = SceneConfig()

# 에셋 루트를 얻는 API 가 버전마다 다르다. verify.py 와 같은 후보 시도 패턴.
ROOT_CANDIDATES = [
    ("isaacsim.storage.native", "get_assets_root_path"),
    ("omni.isaac.nucleus", "get_assets_root_path"),
    ("omni.isaac.core.utils.nucleus", "get_assets_root_path"),
]

# (별칭, 후보 경로들, 왜 필요한가)
WANTED = [
    ("운반 AMR (지게차)", [
        "/Isaac/Robots/IsaacSim/ForkliftB/forklift_b.usd",
        "/Isaac/Robots/Forklift/forklift_b.usd",
        "/Isaac/Robots/IsaacSim/ForkliftC/forklift_c.usd",
        "/Isaac/Robots/Forklift/forklift_c.usd",
    ], "포크 승강 기구. 있으면 AMR 모델링이 사라진다"),

    ("MM 베이스 (Ridgeback)", [
        "/Isaac/Robots/Clearpath/RidgebackUr/ridgeback_ur5.usd",
        "/Isaac/Robots/Clearpath/Ridgeback/ridgeback.usd",
    ], "높이 0.30m. 팔은 UR10e 로 갈아끼워야 함"),

    ("팔 UR10e", [
        "/Isaac/Robots/UniversalRobots/ur10e/ur10e.usd",
        "/Isaac/Robots/UR10e/ur10e.usd",
        "/Isaac/Robots/UniversalRobots/ur10/ur10.usd",
    ], "도달 1.3m. 요구사항을 통과하는 유일한 후보"),

    ("팔 UR5e (참고)", [
        "/Isaac/Robots/UniversalRobots/ur5e/ur5e.usd",
    ], "도달 0.85m -> 0.4m 부족. 안 되는 걸 확인용"),

    ("그리퍼 Robotiq 2F-85", [
        "/Isaac/Robots/Robotiq/2F-85/Robotiq_2F_85.usd",
        "/Isaac/Robots/Robotiq/2F-85/2f85.usd",
    ], "커터는 여기에 직접 붙여야 함"),

    ("AMR 대안 (Nova Carter)", [
        "/Isaac/Robots/NVIDIA/NovaCarter/nova_carter.usd",
        "/Isaac/Robots/Carter/nova_carter.usd",
    ], "포크 없음. 지게차가 없을 때 대안"),
]


def find_root() -> str | None:
    for mod_name, fn_name in ROOT_CANDIDATES:
        try:
            mod = __import__(mod_name, fromlist=[fn_name])
            root = getattr(mod, fn_name)()
            if root:
                print(f"[Asset] 루트 확인: {mod_name}.{fn_name}() -> {root}")
                return root
        except Exception:
            continue
    print("[Asset] 에셋 루트를 못 찾음. 후보:", ROOT_CANDIDATES)
    return None


def probe(url: str) -> bool:
    """USD 를 열어보는 게 존재 확인의 가장 확실한 방법."""
    try:
        return Usd.Stage.Open(url) is not None
    except Exception:
        return False


def measure(url: str) -> dict:
    """bbox 와 관절 수. 베이스 폭이 조간을 통과하는지 보려면 bbox 가 필요하다."""
    stage = Usd.Stage.Open(url)
    if stage is None:
        return {}
    prim = stage.GetDefaultPrim() or stage.GetPseudoRoot()
    try:
        cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                                  [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
        r = cache.ComputeWorldBound(prim).ComputeAlignedRange()
        size = (r.GetSize()[0], r.GetSize()[1], r.GetSize()[2])
    except Exception:
        size = None
    joints = sum(1 for p in Usd.PrimRange(prim)
                 if p.HasAPI(UsdPhysics.ArticulationRootAPI)
                 or p.IsA(UsdPhysics.RevoluteJoint)
                 or p.IsA(UsdPhysics.PrismaticJoint))
    return {"size": size, "joints": joints}


def main() -> None:
    root = find_root()
    if root is None:
        raise SystemExit("에셋 루트 없이는 확인 불가. Nucleus/로컬 에셋 설치 확인할 것.")

    aisle = CFG.plants.row_spacing
    lo, hi = CFG.plants.fruit_height_range
    print(f"[Asset] 요구사항 — 수직 {lo}~{hi}m · 통로(조간) {aisle}m\n")

    print("%-24s %-6s %s" % ("에셋", "존재", "치수 / 비고"))
    print("-" * 78)

    found = {}
    for alias, paths, why in WANTED:
        hit = None
        for p in paths:
            if probe(root + p):
                hit = p
                break
        if hit is None:
            print("%-24s %-6s %s" % (alias, "없음", "-> 직접 모델링 필요. " + why))
            continue

        found[alias] = root + hit
        m = measure(root + hit)
        note = ""
        if m.get("size"):
            w, d, h = m["size"]
            note = "%.2f x %.2f x %.2f m, 관절 %d" % (w, d, h, m.get("joints", 0))
            # 베이스 폭이 통로를 통과하나
            if "베이스" in alias or "AMR" in alias:
                clear = aisle - max(w, d)
                note += "  | 통로 여유 %.2fm %s" % (
                    clear, "OK" if clear > 0.2 else "★빠듯/불가★")
        print("%-24s %-6s %s" % (alias, "있음", note))
        print("%-24s %-6s   %s" % ("", "", short(hit)))

    print("\n" + "=" * 78)
    have_fork = any("지게차" in k for k in found)
    have_base = any("베이스" in k for k in found)
    have_ur10 = any("UR10e" in k for k in found)

    if have_fork:
        print("운반 AMR: 에셋 있음 -> **직접 모델링 불필요.**")
        print("  v3 11.1 의 '팀원 C · AMR 모델링 Day1~2' 배정이 사라진다. 역할 재배분할 것.")
        print("  단, 포크 승강 높이가 창고 2단에 닿는지 확인 필요 (WarehouseConfig.level_height)")
    else:
        print("운반 AMR: 에셋 없음 -> 직접 모델링. 일정의 제일 큰 줄이 된다.")

    if have_base and have_ur10:
        print("수확 MM: Ridgeback + UR10e 조합 가능. 기본 RidgebackUr(UR5)는 도달 부족이므로")
        print("  팔을 갈아끼워야 한다. 두 에셋을 합치는 작업 필요.")
    elif have_base:
        print("수확 MM: 베이스는 있으나 UR10e 없음 -> 팔을 기둥에 올려 어깨를 높이거나")
        print("  다른 긴 팔을 찾을 것. UR5/Franka 는 0.5~1.4m 를 못 덮는다.")
    else:
        print("수확 MM: 베이스 없음 -> AMR 에셋에 팔을 얹어 조립해야 한다.")

    print("\n다음: 실물 도달성 확인. 위 검산은 구 근사라 자세 제약을 못 본다.")
    print("      로봇을 씬에 놓고 과실 0.5m / 1.4m 에 실제로 닿는지 볼 것. (25점의 전제)")
    print("=" * 78)


main()
simulation_app.close()
