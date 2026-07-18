# -*- coding: utf-8 -*-
"""스파이크 4 — 로봇 2대를 실제로 놓고 도달성을 본다. **25점의 전제.**

실행: isaac_python spikes/04_robot_place.py --gui   (눈으로 보는 게 핵심)
      isaac_python spikes/04_robot_place.py         (수치만)

선행: spikes/03_asset_check.py 로 에셋이 있는지 먼저 확인할 것.

무엇을 묻는가:
  settings.py 의 도달 검산은 **구 근사**다 — 어깨에서 반지름 R 인 구로 계산했다.
  실제 도달영역은 모양이 있고 관절 한계·자세 제약이 있다. 그래서 실물로 봐야 한다.
  안 닿으면 온실 치수나 로봇 구성이 바뀐다 = 일정의 제일 큰 줄이 흔들린다.

무엇을 놓는가:
  · 수확 MM (Ridgeback + UR10e + Robotiq + 커터)  — 통로 중앙
  · 운반 AMR (ForkliftB)                          — 옆에
  · 과실 마커 3개 — 최저 0.5m / 중간 / 최고 1.4m (fruit_height_range 에서)
    수평은 조간/2 - 과실오프셋 = 0.66m 지점

  --gui 로 띄워놓고 **팔을 손으로 끌어서 마커에 닿는지** 보는 게 제일 빠르다.
  IK 로 자동 판정하려면 Lula/RMPflow 가 필요한데 그건 로봇 확정 후에 할 일이다.
"""
import sys

GUI = "--gui" in sys.argv

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": not GUI})

import os

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import omni.usd
from isaacsim.core.api import World
from pxr import Gf, UsdGeom, UsdLux

from pjt_config.settings import SceneConfig
from robots.harvester import HarvestMM
from robots.transporter import TransporterAMR

CFG = SceneConfig()
MARKER_R = 0.0344          # 과실 반지름 (지름 68.7mm)


def add_marker(stage, path: str, pos, color) -> None:
    """과실 자리 표시. 물리 없음 — 도달만 본다."""
    s = UsdGeom.Sphere.Define(stage, path)
    s.CreateRadiusAttr(MARKER_R)
    s.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    UsdGeom.Xformable(s.GetPrim()).AddTranslateOp().Set(Gf.Vec3d(*pos))


def main() -> None:
    world = World(stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane()
    stage = omni.usd.get_context().get_stage()
    UsdLux.DistantLight.Define(stage, "/World/Light").CreateIntensityAttr(3000)

    lo, hi = CFG.plants.fruit_height_range
    horiz = CFG.plants.row_spacing / 2 - CFG.plants.fruit_offset

    print("\n[Spike] 요구사항 (근거 있는 값에서 유도)")
    print(f"  과실 높이  {lo} ~ {hi} m   (하이와이어 lower&lean + 한국인 인체치수)")
    print(f"  수평 도달  {horiz:.2f} m   (조간 {CFG.plants.row_spacing}m / 2 "
          f"- 과실오프셋 {CFG.plants.fruit_offset}m)")
    print(f"  통로 폭    {CFG.plants.row_spacing} m  (제주농기원 배지경 양액재배)\n")

    # --- 수확 MM: 통로 중앙(원점) ---
    try:
        mm = HarvestMM(CFG.robots)
        mm.spawn(stage, "/World/Harvester", (0.0, 0.0, 0.0))
    except FileNotFoundError as e:
        print(f"\n[Spike] 수확 MM 조립 실패:\n{e}\n")
        mm = None

    # --- 운반 AMR: 통로 뒤쪽 ---
    try:
        amr = TransporterAMR(CFG.robots, CFG.warehouse)
        amr.spawn(stage, "/World/Transporter", (0.0, -2.5, 0.0))
    except FileNotFoundError as e:
        print(f"\n[Spike] 운반 AMR 배치 실패:\n{e}\n")
        amr = None

    # --- 과실 마커: 통로 중앙에서 horiz 만큼 옆, 높이 3단 ---
    mid = (lo + hi) / 2
    add_marker(stage, "/World/Marker_lo", (horiz, 0.0, lo), (0.2, 0.8, 0.2))
    add_marker(stage, "/World/Marker_mid", (horiz, 0.0, mid), (0.9, 0.6, 0.1))
    add_marker(stage, "/World/Marker_hi", (horiz, 0.0, hi), (0.9, 0.1, 0.1))
    print(f"[Spike] 과실 마커 3개: 수평 {horiz:.2f}m, 높이 "
          f"{lo} / {mid:.2f} / {hi} m  (초록/주황/빨강)")

    # 반대쪽 이랑도 — 베이스가 통로 중앙이면 양쪽을 다 따야 한다
    add_marker(stage, "/World/Marker_hi_opposite", (-horiz, 0.0, hi), (0.6, 0.1, 0.6))
    print(f"[Spike] 반대쪽 이랑 마커: 수평 -{horiz:.2f}m, 높이 {hi}m (보라)")
    print("        통로 중앙에 서서 양쪽을 다 따는 구성인지, 한쪽씩 도는지가")
    print("        아직 안 정해졌다. 이 마커가 그 결정을 눈으로 보게 해준다.\n")

    world.reset()

    if mm is not None:
        pos = mm.cutter_world_pos(stage)
        if pos:
            print(f"[Spike] 커터 초기 위치: ({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})")
            print(f"        그리퍼 파지점 위 {CFG.robots.end_effector.cutter_offset_z*1000:.0f}mm "
                  f"— 과실 반지름 {MARKER_R*1000:.1f}mm 보다 커야 과실을 안 자른다\n")

    print("=" * 70)
    print("눈으로 확인할 것:")
    print("  [ ] 팔이 빨강 마커(1.4m)에 닿나          ← UR5 면 여기서 막힌다")
    print("  [ ] 초록 마커(0.5m)까지 내려가나         ← 낮은 화방")
    print("  [ ] 보라 마커(반대쪽 이랑)도 닿나        ← 양쪽 vs 한쪽씩")
    print("  [ ] 베이스가 통로 폭 안에 들어가나       ← 조간 1.5m")
    print("  [ ] 커터가 그리퍼 위에 제대로 붙었나")
    print("  [ ] 지게차 포크가 승강하나               ← 창고 2단")
    print("=" * 70)

    if GUI:
        print("\n창을 띄웠다. 팔을 끌어서 마커에 닿는지 보고, 닫으면 끝난다.")
        while simulation_app.is_running():
            world.step(render=True)


main()
simulation_app.close()
