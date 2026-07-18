# -*- coding: utf-8 -*-
"""스파이크 2 — 꽃자루 조인트가 씬 규모에서 버티는가.

실행: isaac_python spikes/02_pedicel_feasibility.py          (스윕, headless)
      isaac_python spikes/02_pedicel_feasibility.py --gui
      isaac_python spikes/02_pedicel_feasibility.py --n 401  (한 개수만)

무엇을 묻는가:
  지금 과실 401개는 전부 kinematic 이라 솔버가 안 푼다 = 공짜, 재현성 완벽.
  꽃자루를 달면 401개가 dynamic + 조인트 401개가 된다. 그게 도는지가 문제다.
  검색으로 답이 안 나온다 — 선행연구는 과실을 아예 안 붙이거나(Find the Fruit,
  arXiv 2505.16547) 몇 개짜리 RL 환경이다. 아무도 400개를 매달아본 적이 없다.

세 가지를 재고 설계를 고른다:
  A. 전부 dynamic + 조인트  -> 제일 정직. 이 스파이크가 통과하면 이걸로 간다.
  B. 하이브리드             -> 평소 kinematic, 로봇이 접근한 것만 전환. A 가 안 되면.
  C. 꽃자루는 장식만        -> 물리값이 장식이 되므로 채택 안 함.

판정 기준:
  - 스텝 시간: 물리만 60fps(16.7ms) 안에. 렌더+로봇+센서가 더 붙으므로 절반 이하가 안전
  - 지터: 가만 놔뒀을 때 과실이 안 떨려야 한다 (재현성 20점)
  - 재현성: Play/Stop 반복 시 같은 자리

참고 기준선: Find the Fruit 는 1/60s + solver iteration 8 (RTX 4090).
"""
import sys
import time

GUI = "--gui" in sys.argv


def _arg(name, default):
    if name in sys.argv:
        return type(default)(sys.argv[sys.argv.index(name) + 1])
    return default


from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": not GUI})

import os

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import omni.usd
from isaacsim.core.api import World
from isaacsim.core.utils.stage import add_reference_to_stage
from pxr import Gf, Usd, UsdGeom, UsdPhysics

from pjt_config.settings import SceneConfig
from pjt_utils.paths import short
from scene import pedicel, physics

CFG = SceneConfig()
N_SWEEP = (50, 100, 200, 401)
WARMUP_STEPS = 30
MEASURE_STEPS = 120
JITTER_TOLERANCE = 0.001      # m. 가만 놔뒀는데 이만큼 움직이면 지터
FRAME_BUDGET_MS = 16.7        # 60fps


def fruit_usd() -> str:
    d = CFG.tomato_assets.usd_dir
    if not os.path.isdir(d):
        raise SystemExit(
            f"토마토 USD 폴더가 없다: {short(d)}\n"
            f"  -> isaac_python tomatest/00_convert_obj_to_usd.py 를 먼저 실행할 것.")
    for f in sorted(os.listdir(d)):
        if f.endswith(".usd") and "_calyx" not in f:
            return os.path.join(d, f)
    raise SystemExit(f"{short(d)} 에 토마토 USD 가 없다.")


def build(stage: Usd.Stage, n: int, usd: str) -> list[dict]:
    """줄기 하나에 과실 n 개를 꽃자루로 매단다.

    실제 씬 배치가 아니라 부하만 본다 — 조인트 개수가 변수다.
    """
    UsdGeom.Xform.Define(stage, "/World/Spike")
    pcfg = pedicel.PedicelConfig()
    out = []

    per_row = 20
    for i in range(n):
        row, col = divmod(i, per_row)
        sx, sy = col * 0.35, row * 0.35

        stem_path = f"/World/Spike/Stem_{i:03d}"
        stem = UsdGeom.Cylinder.Define(stage, stem_path)
        stem.CreateRadiusAttr(0.02)
        stem.CreateHeightAttr(1.8)
        stem.CreateAxisAttr("Z")
        UsdGeom.Xformable(stem.GetPrim()).AddTranslateOp().Set(
            Gf.Vec3d(sx, sy, 0.9))
        physics.add_shape_collider(stem.GetPrim())
        # 줄기는 static (RigidBody 없음) — 조인트의 body0 로 쓴다

        fruit_path = f"/World/Spike/Fruit_{i:03d}"
        fx, fy, fz = sx + CFG.plants.fruit_offset, sy, 1.0
        # 신선한 Xform 을 만들고 그 밑에 토마토를 참조한다 (tomato_plants 와 같은 패턴).
        # 재스탬프된 토마토 USD 는 자체 xformOp 를 가져 참조 prim 에 직접 AddTranslateOp
        # 하면 터진다 (§8 이 예측한 케이스). 신선한 부모엔 op 가 없어 안전하다.
        fruit = UsdGeom.Xform.Define(stage, fruit_path)
        xf = UsdGeom.Xformable(fruit.GetPrim())
        xf.AddTranslateOp().Set(Gf.Vec3d(fx, fy, fz))
        s = CFG.tomato_assets.scale
        xf.AddScaleOp().Set(Gf.Vec3f(s, s, s))
        add_reference_to_stage(usd, fruit_path + "/Body")
        physics.add_mesh_colliders(stage, fruit_path + "/Body",
                                   CFG.physics.fruit_approximation)
        # 꽃자루가 붙으면 과실은 kinematic 이 아니다 — 조인트가 매단다
        physics.add_rigid_body(fruit.GetPrim(), CFG.physics.fruit_density,
                               kinematic=False)

        jp = pedicel.spawn(stage, stem_path, fruit_path,
                           (sx, sy, fz), (fx, fy, fz), pcfg,
                           CFG.physics.break_force, CFG.physics.break_torque)
        out.append({"fruit": fruit_path, "joint": jp})
    return out


def positions(stage: Usd.Stage, items: list[dict]) -> list[tuple]:
    out = []
    for it in items:
        m = UsdGeom.Xformable(stage.GetPrimAtPath(it["fruit"])) \
            .ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        t = m.ExtractTranslation()
        out.append((float(t[0]), float(t[1]), float(t[2])))
    return out


def measure(world: World, stage: Usd.Stage, items: list[dict]) -> dict:
    """스텝 시간 + 지터."""
    world.reset()
    for _ in range(WARMUP_STEPS):
        world.step(render=GUI)

    before = positions(stage, items)
    t0 = time.perf_counter()
    for _ in range(MEASURE_STEPS):
        world.step(render=GUI)
    elapsed = time.perf_counter() - t0
    after = positions(stage, items)

    drift = max(max(abs(a[k] - b[k]) for k in range(3))
                for a, b in zip(before, after)) if items else 0.0
    return {"ms": elapsed / MEASURE_STEPS * 1000.0, "drift": drift}


def test_reproducibility(world: World, stage: Usd.Stage,
                         items: list[dict]) -> float:
    """Play/Stop 두 번 -> 같은 자리인가. 디지털트윈 20점."""
    def once():
        world.reset()
        for it in items:
            pedicel.restore(stage, it["joint"])
        for _ in range(WARMUP_STEPS):
            world.step(render=GUI)
        return positions(stage, items)

    a, b = once(), once()
    return max(max(abs(p[k] - q[k]) for k in range(3))
               for p, q in zip(a, b)) if items else 0.0


def test_cut(world: World, stage: Usd.Stage, item: dict) -> float:
    """조인트를 끊으면 실제로 떨어지는가. 얼마나 떨어졌는지 반환."""
    world.reset()
    pedicel.restore(stage, item["joint"])
    for _ in range(WARMUP_STEPS):
        world.step(render=GUI)
    z0 = positions(stage, [item])[0][2]

    pedicel.cut(stage, item["joint"])
    for _ in range(90):
        world.step(render=GUI)
    return z0 - positions(stage, [item])[0][2]


def main() -> None:
    usd = fruit_usd()
    print(f"[Spike] 과실 USD: {short(usd)}")
    print(f"[Spike] break_force={CFG.physics.break_force} N  "
          f"break_torque={CFG.physics.break_torque} Nm  "
          f"density={CFG.physics.fruit_density}")

    one_n = _arg("--n", 0)
    sweep = (one_n,) if one_n else N_SWEEP

    print(f"\n{'개수':>6}{'스텝(ms)':>12}{'60fps 여유':>12}{'지터(mm)':>12}")
    rows = []
    for n in sweep:
        world = World(stage_units_in_meters=1.0)
        world.scene.add_default_ground_plane()
        stage = omni.usd.get_context().get_stage()
        items = build(stage, n, usd)
        world.reset()

        r = measure(world, stage, items)
        headroom = FRAME_BUDGET_MS / r["ms"] if r["ms"] > 0 else 0.0
        rows.append((n, r, items, world, stage))
        print(f"{n:>6}{r['ms']:>12.2f}{headroom:>11.1f}x"
              f"{r['drift'] * 1000:>12.3f}")

        if n != sweep[-1]:
            world.clear()

    # 마지막(가장 큰) 구성으로 나머지 검사
    n, r, items, world, stage = rows[-1]
    repro = test_reproducibility(world, stage, items)
    fall = test_cut(world, stage, items[0])

    print("\n" + "=" * 64)
    print(f"과실 {n}개 기준")
    print(f"  스텝 시간   : {r['ms']:.2f} ms  (60fps 예산 {FRAME_BUDGET_MS} ms)")
    print(f"  지터        : {r['drift'] * 1000:.3f} mm  "
          f"({'OK' if r['drift'] < JITTER_TOLERANCE else '불안정'})")
    print(f"  Play/Stop   : {repro * 1000:.3f} mm 차이  "
          f"({'재현됨' if repro < JITTER_TOLERANCE else '재현 안 됨'})")
    print(f"  절단 낙하   : {fall * 100:.1f} cm  "
          f"({'OK' if fall > 0.05 else '안 떨어짐 — 조인트 확인'})")

    print("\n[판정]")
    ok_perf = r["ms"] < FRAME_BUDGET_MS / 2
    ok_jitter = r["drift"] < JITTER_TOLERANCE
    ok_repro = repro < JITTER_TOLERANCE

    if ok_perf and ok_jitter and ok_repro:
        print("  -> A안 (전부 dynamic + 조인트) 으로 간다. 제일 정직한 설계가 된다.")
    elif ok_jitter and ok_repro:
        print("  -> 물리는 맞는데 부하가 크다. B안 (하이브리드) 로 간다:")
        print("     평소 kinematic, 로봇이 접근한 과실만 dynamic+조인트로 전환.")
        print("     전환 시점 로직이 붙지만 부하는 몇 개 수준으로 떨어진다.")
    else:
        print("  -> 물리 자체가 불안정하다. 부하 문제가 아니다. 확인할 것:")
        print("     - solver iteration (Find the Fruit 기준선: 1/60s, 8회)")
        print("     - 꽃자루 조인트가 과실 무게에 비해 너무 뻣뻣한가")
        print("     - convexHull 콜라이더끼리 겹쳐 있나 (과실 간격)")
    print("=" * 64)


main()
simulation_app.close()
