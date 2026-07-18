# -*- coding: utf-8 -*-
"""스파이크 1 — 마찰 파지가 PhysX 에서 되는가. 프로젝트 최대 위험.

실행: isaac_python spikes/01_grasp_feasibility.py           (스윕, headless)
      isaac_python spikes/01_grasp_feasibility.py --gui     (눈으로 확인)
      isaac_python spikes/01_grasp_feasibility.py --mu 0.9 --force 20   (한 조합만)

무엇을 묻는가:
  고정 조인트를 안 쓰기로 했으므로 과실을 붙잡는 건 접촉 마찰뿐이다. 절단 순간
  kinematic 이 풀리면 마찰만으로 버텨야 한다. 이게 안 되면 수확 방식 자체를
  다시 정해야 한다. 그래서 씬/FSM/로봇 없이 이것만 먼저 판가름낸다.

**그리퍼를 고르지 않는다. 그리퍼의 요구사항을 구한다.**
  "이 그리퍼로 되나?" 가 아니라 "토마토를 붙잡으려면 몇 N 이 필요한가?" 를 묻는다.
  나온 값이 곧 그리퍼 선정 기준이 된다 — 그 힘을 낼 수 있는 그리퍼면 뭐든 된다.
  로봇 플랫폼은 아직 안 정해졌고(CLAUDE.md), 이 순서라야 근거를 갖고 고를 수 있다.

산술적으로는 자명하다:
  두 손가락으로 질량 m 을 들려면  2·μ·F >= m·g
  120g 과실, μ=0.9  ->  F >= 0.65 N. 웬만한 그리퍼가 다 낸다.
  **그러므로 이 스파이크가 묻는 건 힘이 모자라냐가 아니라 솔버가 버티냐다.**
  convexHull 접촉, 관통, 지터, 과실이 손가락 사이로 튀어나가는 현상.
  실측이 산술값보다 훨씬 크게 나오면 그건 물리가 아니라 솔버 문제라는 신호다.

무엇이 아닌가:
  특정 그리퍼 모델이 아니다. 손가락은 판 두 장이다. 기구를 보는 게 아니라
  "PhysX 가 이 질량/마찰/형상 조합을 붙잡을 수 있나"를 격리해서 본다.
  여기서 실패하면 어떤 그리퍼를 붙여도 소용없다.

출력: μ x 파지력 표 (유지/미끄러짐 + 미끄러진 거리). 그대로 근거 자료가 된다.
"""
import sys

GUI = "--gui" in sys.argv


def _arg(name, default):
    if name in sys.argv:
        return float(sys.argv[sys.argv.index(name) + 1])
    return default


from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": not GUI})

import os

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np
from isaacsim.core.api import World
from isaacsim.core.utils.stage import add_reference_to_stage
from pxr import Gf, PhysxSchema, Usd, UsdGeom, UsdLux, UsdPhysics

from pjt_config.settings import SceneConfig
from pjt_utils.paths import short
from scene import physics

# ===== 파라미터 =====
FRUIT_USD_DIR = SceneConfig().tomato_assets.usd_dir
SCALE = SceneConfig().tomato_assets.scale
DENSITY = SceneConfig().physics.fruit_density
APPROX = SceneConfig().physics.fruit_approximation
CFG_PHYS = SceneConfig().physics

# 스윕 범위. mu 는 settings.py 의 0.9(근거 없는 임의값)를 가운데 두고,
# 힘은 흔한 협동로봇 그리퍼 범위를 훑는다. 특정 모델 전제 아님.
MU_SWEEP = (0.3, 0.5, 0.7, 0.9, 1.1)
FORCE_SWEEP = (2.0, 5.0, 10.0, 20.0, 40.0)

SETTLE_STEPS = 60      # 손가락이 닫히고 접촉이 안정될 때까지
LIFT_STEPS = 120       # 들어올리는 동안
LIFT_HEIGHT = 0.30     # m
SLIP_TOLERANCE = 0.01  # m. 그리퍼 기준 이만큼 흘러내리면 실패로 본다

FINGER = (0.008, 0.05, 0.05)   # 손가락 판 크기 (두께, 폭, 높이)


def pick_fruit_usd() -> str:
    """잘 익은 과실 하나. 없으면 00_convert 를 먼저 돌리라고 알려준다."""
    if not os.path.isdir(FRUIT_USD_DIR):
        raise SystemExit(
            f"토마토 USD 폴더가 없다: {short(FRUIT_USD_DIR)}\n"
            f"  -> isaac_python tomatest/00_convert_obj_to_usd.py 를 먼저 실행할 것.")
    for f in sorted(os.listdir(FRUIT_USD_DIR)):
        if f.startswith("tomato_ripe_01") and f.endswith(".usd") \
                and "_calyx" not in f:
            return os.path.join(FRUIT_USD_DIR, f)
    raise SystemExit(f"{short(FRUIT_USD_DIR)} 에 tomato_ripe_01.usd 가 없다.")


class GraspRig:
    """손가락 판 두 장 + 과실. 손가락은 kinematic 베이스에 프리즈매틱으로 물린다.

    베이스를 kinematic 으로 두면 들어올리는 동작이 결정적이고, 손가락은 dynamic
    이라 파지력이 드라이브의 maxForce 로 실제 제어된다. (손가락까지 kinematic 이면
    관통 깊이가 힘을 정해버려서 '파지력'이라는 값이 의미를 잃는다.)
    """

    def __init__(self, stage: Usd.Stage, fruit_usd: str):
        self._stage = stage
        self._fruit_path = "/World/Fruit"

        UsdGeom.Xform.Define(stage, "/World")
        UsdLux.DistantLight.Define(stage, "/World/Light").CreateIntensityAttr(3000)

        # --- 과실 ---
        add_reference_to_stage(fruit_usd, self._fruit_path)
        fruit = stage.GetPrimAtPath(self._fruit_path)
        xf = UsdGeom.Xformable(fruit)
        xf.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 1.0))
        xf.AddScaleOp().Set(Gf.Vec3f(SCALE, SCALE, SCALE))
        physics.add_mesh_colliders(stage, self._fruit_path, APPROX)
        # 실제 수확과 같은 조건: 절단 직후 = dynamic
        physics.add_rigid_body(fruit, DENSITY, kinematic=False)

        self._fruit_mat = physics.create_physics_material(
            stage, "/World/Mat/fruit", 0.9, 0.7)
        physics.bind_physics_material(fruit, self._fruit_mat)

        # --- 그리퍼 베이스 (kinematic) ---
        base = UsdGeom.Cube.Define(stage, "/World/Gripper")
        base.CreateSizeAttr(1.0)
        b_xf = UsdGeom.Xformable(base.GetPrim())
        self._base_t = b_xf.AddTranslateOp()
        self._base_t.Set(Gf.Vec3d(0.0, 0.0, 1.12))
        b_xf.AddScaleOp().Set(Gf.Vec3f(0.02, 0.02, 0.02))
        rb = UsdPhysics.RigidBodyAPI.Apply(base.GetPrim())
        rb.CreateKinematicEnabledAttr(True)

        # --- 손가락 두 장 ---
        self._drives = []
        for side, sign in (("L", -1.0), ("R", 1.0)):
            self._add_finger(f"/World/Finger_{side}", sign)

    def _add_finger(self, path: str, sign: float):
        stage = self._stage
        f = UsdGeom.Cube.Define(stage, path)
        f.CreateSizeAttr(1.0)
        xf = UsdGeom.Xformable(f.GetPrim())
        xf.AddTranslateOp().Set(Gf.Vec3d(sign * 0.06, 0.0, 1.0))
        xf.AddScaleOp().Set(Gf.Vec3f(*FINGER))
        UsdPhysics.CollisionAPI.Apply(f.GetPrim())
        UsdPhysics.RigidBodyAPI.Apply(f.GetPrim())
        UsdPhysics.MassAPI.Apply(f.GetPrim()).CreateMassAttr(0.05)
        physics.bind_physics_material(f.GetPrim(), self._fruit_mat)

        # 베이스에 프리즈매틱으로 물린다 (X 축으로만 열고 닫힘)
        joint = UsdPhysics.PrismaticJoint.Define(stage, path + "/Joint")
        joint.CreateBody0Rel().SetTargets(["/World/Gripper"])
        joint.CreateBody1Rel().SetTargets([path])
        joint.CreateAxisAttr("X")
        joint.CreateLocalPos0Attr(Gf.Vec3f(0.0, 0.0, -0.12 / 0.02))
        joint.CreateLocalPos1Attr(Gf.Vec3f(0.0, 0.0, 0.0))
        joint.CreateLowerLimitAttr(-0.08)
        joint.CreateUpperLimitAttr(0.08)

        drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "linear")
        drive.CreateTypeAttr("force")
        drive.CreateDampingAttr(1e3)
        drive.CreateStiffnessAttr(1e4)
        # 목표: 안쪽으로 물기. maxForce 가 곧 파지력이다.
        drive.CreateTargetPositionAttr(sign * 0.02)
        self._drives.append(drive)

    # ----- 조정 -----

    def set_friction(self, mu: float) -> None:
        api = UsdPhysics.MaterialAPI(self._fruit_mat.GetPrim())
        api.GetStaticFrictionAttr().Set(mu)
        api.GetDynamicFrictionAttr().Set(mu * 0.8)

    def set_grip_force(self, newtons: float) -> None:
        for d in self._drives:
            d.GetMaxForceAttr().Set(newtons) if d.GetMaxForceAttr() \
                else d.CreateMaxForceAttr(newtons)

    def lift_to(self, z: float) -> None:
        self._base_t.Set(Gf.Vec3d(0.0, 0.0, z))

    # ----- 관측 -----

    def fruit_z(self) -> float:
        prim = self._stage.GetPrimAtPath(self._fruit_path)
        m = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(
            Usd.TimeCode.Default())
        return float(m.ExtractTranslation()[2])

    def fruit_xy(self) -> tuple[float, float]:
        prim = self._stage.GetPrimAtPath(self._fruit_path)
        m = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(
            Usd.TimeCode.Default())
        t = m.ExtractTranslation()
        return float(t[0]), float(t[1])


def run_case(world: World, rig: GraspRig, mu: float, force: float) -> dict:
    """한 조합. 물고 -> 들어올리고 -> 얼마나 흘렀는지 잰다."""
    world.reset()
    rig.set_friction(mu)
    rig.set_grip_force(force)
    rig.lift_to(1.12)

    for _ in range(SETTLE_STEPS):
        world.step(render=GUI)

    z_after_grip = rig.fruit_z()

    # 그리퍼를 서서히 올린다
    for i in range(LIFT_STEPS):
        rig.lift_to(1.12 + LIFT_HEIGHT * (i + 1) / LIFT_STEPS)
        world.step(render=GUI)

    z_end = rig.fruit_z()
    # 과실이 그리퍼를 따라 올라왔으면 유지, 뒤처졌으면 미끄러진 것
    slip = (z_after_grip + LIFT_HEIGHT) - z_end
    held = abs(slip) < SLIP_TOLERANCE and z_end > z_after_grip + 0.5 * LIFT_HEIGHT
    return {"mu": mu, "force": force, "held": held, "slip": slip,
            "z_end": z_end}


def main() -> None:
    fruit_usd = pick_fruit_usd()
    print(f"[Spike] 과실 USD: {short(fruit_usd)}")
    print(f"[Spike] scale={SCALE}  density={DENSITY}  approx={APPROX}")

    world = World(stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane()
    import omni.usd
    stage = omni.usd.get_context().get_stage()
    rig = GraspRig(stage, fruit_usd)
    world.reset()

    one_mu, one_force = _arg("--mu", 0.0), _arg("--force", 0.0)
    if one_mu and one_force:
        r = run_case(world, rig, one_mu, one_force)
        print(f"\n{r}")
        return

    print("\n[Spike] 세로=마찰계수, 가로=파지력. 칸은 유지 또는 미끄러진 거리.")
    print(f"{'':>7}" + "".join(f"{f:>10.0f}N" for f in FORCE_SWEEP))

    results = []
    for mu in MU_SWEEP:
        cells = []
        for force in FORCE_SWEEP:
            r = run_case(world, rig, mu, force)
            results.append(r)
            cells.append("유지" if r["held"] else f"{r['slip'] * 100:.1f}cm")
        print(f"mu{mu:>4.1f} " + "".join(f"{c:>11}" for c in cells))

    _report(results)


def _report(results: list[dict]) -> None:
    held = [r for r in results if r["held"]]
    print("\n" + "=" * 64)
    if not held:
        print("전부 미끄러짐 — 이 조건에서 마찰 파지가 성립 안 한다.")
        print("  산술적으로는 0.65N 이면 충분하다. 전부 실패면 힘 문제가 아니라")
        print("  솔버 문제다. 확인할 것:")
        print("   - convexHull 근사가 과실을 너무 둥글게 만들어 접촉면이 점에 가까운가")
        print("   - solver position/velocity iteration 이 모자란가")
        print("   - 접촉 관통(contact offset / rest offset)이 큰가")
        print("   - 물리 스텝(dt)이 큰가")
        print("  -> 다 아니면 마찰 파지를 포기하고 수확 방식을 다시 정해야 한다.")
        print("=" * 64)
        return

    min_force = min(r["force"] for r in held)
    print(f"성립한다. 최소 성공 파지력(하한): {min_force:.0f} N")
    print(f"  (산술 하한 0.65N. 실측이 이보다 큰 만큼이 솔버 여유분이다)")

    ok = [r for r in held if abs(r["mu"] - 0.9) < 1e-6]
    if ok:
        print(f"  settings.py 의 mu=0.9 에서는 최소 "
              f"{min(r['force'] for r in ok):.0f} N")
    else:
        mus = sorted({r["mu"] for r in held})
        print(f"  주의: mu=0.9 에서 전부 실패. 성공한 마찰계수: {mus}")
        print(f"        0.9 는 근거 없는 임의값이다 — 이 결과로 재검토할 것.")

    # 상한은 시뮬이 아니라 논문에서 온다. 강체는 접촉 압력을 못 주기 때문에
    # 패드 면적으로 환산해서 건다.
    p = CFG_PHYS.fruit_damage_pressure
    print(f"\n  [상한] 손상 압력 {p / 1000:.0f} kPa (완숙 기준) x 패드 면적:")
    for cm2 in (2.0, 4.0, 6.0, 10.0):
        f_max = p * cm2 * 1e-4
        verdict = "가능" if min_force <= f_max else "불가 — 잡으면 손상"
        print(f"    패드 {cm2:>4.0f} cm^2 -> 상한 {f_max:>5.1f} N   "
              f"[{min_force:.0f} <= F <= {f_max:.1f}]  {verdict}")
    print(f"\n  -> **그리퍼 선정 기준**: {min_force:.0f} N 이상 내면서,")
    print(f"     패드 면적이 {min_force / p * 1e4:.1f} cm^2 이상이면 된다.")
    print(f"     (그보다 패드가 좁으면 붙잡는 데 필요한 힘이 곧 손상 압력을 넘는다)")
    print("=" * 64)


main()
simulation_app.close()
