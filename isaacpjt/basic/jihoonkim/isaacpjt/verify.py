# -*- coding: utf-8 -*-
"""GPU 머신 일괄 검증 스크립트 — 밀린 미검증 항목을 한 번에 확인한다.

실행:  isaac_python verify.py          (isaac/ 안에서, headless)
       isaac_python verify.py --gui    (창 띄워서 눈으로도 확인)

왜 이게 필요한가:
  개발은 GPU 없는 머신에서 하고 Isaac 은 GPU 노트북에만 있다. 항목을 하나씩
  켜보면 시간이 다 가므로, 여기서 전부 찍고 로그만 보면 되게 한다.

설계 원칙:
  - 모든 검사는 try/except 로 격리. 하나 실패해도 나머지는 계속 돈다.
  - API 이름이 틀릴 수 있는 항목은 여러 후보를 순서대로 시도하고 뭐가 먹혔는지 출력.
  - 판정은 [OK] / [FAIL] / [SKIP] / [INFO] 로 통일. 마지막에 요약.
"""
import sys

GUI = "--gui" in sys.argv

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": not GUI})

import os
import time
import traceback

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

RESULTS: list[tuple[str, str, str]] = []   # (판정, 항목, 비고)


def record(verdict: str, name: str, note: str = "") -> None:
    RESULTS.append((verdict, name, note))
    print("  [%s] %s%s" % (verdict, name, ("  -> " + note) if note else ""))


def section(title: str) -> None:
    print("\n" + "=" * 64)
    print(title)
    print("=" * 64)


def try_imports(candidates: list[str], attr: str | None = None):
    """후보 import 경로를 순서대로 시도. (성공한 경로, 객체) 또는 (None, None)."""
    for path in candidates:
        try:
            mod = __import__(path, fromlist=["*"])
            obj = getattr(mod, attr) if attr else mod
            return path, obj
        except Exception:
            continue
    return None, None


# ============================================================
# 1. 환경
# ============================================================
def check_environment() -> None:
    section("1. 환경")
    record("INFO", "Python", sys.version.split()[0] + "  (3.11 이어야 정상)")

    try:
        import carb
        settings = carb.settings.get_settings()
        ver = settings.get("/app/version") or "unknown"
        record("INFO", "Isaac Sim 버전", str(ver))
    except Exception as e:
        record("FAIL", "Isaac 버전 조회", str(e)[:60])

    # rclpy 가 Isaac 안에서 import 되면 안 된다 (3.10 C 확장이라 3.11 에 못 올라감).
    # 만약 성공하면 환경 가정이 틀린 것이므로 알려준다.
    try:
        import rclpy  # noqa: F401
        record("INFO", "rclpy import", "성공?! 환경 가정과 다름 - 확인 필요")
    except Exception:
        record("OK", "rclpy import 불가", "정상 (브리지로 통신하는 게 맞음)")


# ============================================================
# 2. import 경로 (CHECKLIST 항목 E)
# ============================================================
def check_imports() -> None:
    section("2. import 경로")

    path, fn = try_imports(
        ["isaacsim.core.utils.semantics", "omni.isaac.core.utils.semantics"],
        "add_update_semantics")
    if path:
        record("OK", "add_update_semantics", path)
        if path.startswith("omni.isaac"):
            record("INFO", "02_generate_dataset.py 수정 필요",
                   "isaacsim.* -> omni.isaac.* 로 교체")
    else:
        record("FAIL", "add_update_semantics", "두 경로 모두 실패")

    path, _ = try_imports(["omni.replicator.core"])
    record("OK" if path else "FAIL", "omni.replicator.core", path or "import 실패")

    path, _ = try_imports(["isaacsim.core.api.tasks"], "BaseTask")
    record("OK" if path else "FAIL", "BaseTask", path or "import 실패")


# ============================================================
# 3. 토마토 에셋 + TOMATO_SCALE 자동 측정  ★가장 오래 묵은 항목★
# ============================================================
def check_tomato_scale() -> None:
    section("3. 토마토 에셋 + TOMATO_SCALE (자동 측정)")

    from pjt_config.settings import SceneConfig
    cfg = SceneConfig()
    usd_dir = cfg.tomato_assets.usd_dir

    if not os.path.isdir(usd_dir):
        record("FAIL", "USD 폴더", "%s 없음 -> 00_convert_obj_to_usd.py 먼저 실행" % usd_dir)
        return

    usds = [f for f in sorted(os.listdir(usd_dir))
            if f.endswith(".usd") and "_calyx" not in f]
    if not usds:
        record("FAIL", "토마토 USD", "폴더는 있는데 USD 가 없음")
        return
    record("OK", "토마토 USD", "%d개 (%s)" % (len(usds), usd_dir))

    # 눈으로 보지 말고 재본다: bounding box * scale = 실제 지름
    try:
        import omni.usd
        from pxr import Usd, UsdGeom
        from isaacsim.core.utils.stage import add_reference_to_stage

        stage = omni.usd.get_context().get_stage()
        scale = cfg.tomato_assets.scale
        cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])

        print("\n  %-32s %10s %10s" % ("USD", "원본크기", "x scale"))
        print("  " + "-" * 56)
        diameters = []
        for i, f in enumerate(usds[:5]):
            path = "/World/_probe_%d" % i
            add_reference_to_stage(os.path.join(usd_dir, f), path)
            simulation_app.update()
            rng = cache.ComputeWorldBound(stage.GetPrimAtPath(path)).GetRange()
            size = rng.GetSize()
            raw = max(size[0], size[1], size[2])
            scaled = raw * scale
            diameters.append(scaled)
            print("  %-32s %8.2f   %7.4f m" % (f[:32], raw, scaled))
            stage.RemovePrim(path)

        avg = sum(diameters) / len(diameters)
        print()
        record("INFO", "평균 지름", "%.4f m  (scale=%.4f 적용 후)" % (avg, scale))

        # 목표: 실제 토마토 지름 5~6cm
        if 0.04 <= avg <= 0.08:
            record("OK", "TOMATO_SCALE", "지름 %.1fcm - 실제 토마토 범위" % (avg * 100))
        else:
            suggest = 0.055 / (avg / scale) if avg else 0
            record("FAIL", "TOMATO_SCALE",
                   "지름 %.1fcm 는 비정상. scale 을 %.5f 로 바꿔볼 것"
                   % (avg * 100, suggest))
    except Exception:
        record("FAIL", "TOMATO_SCALE 측정", traceback.format_exc().splitlines()[-1][:70])


# ============================================================
# 4. 씬 빌드 + 물리 + 성능
# ============================================================
def check_scene() -> None:
    section("4. 온실 씬 빌드 + 물리 + 성능")

    try:
        from isaacsim.core.api import World
        from pxr import Usd, UsdGeom, UsdPhysics
        from pjt_config.settings import SceneConfig
        from scene.greenhouse_task import GreenhouseTask
        import omni.usd

        cfg = SceneConfig()
        world = World(stage_units_in_meters=1.0)
        task = GreenhouseTask(name="verify_greenhouse", cfg=cfg)

        t0 = time.time()
        world.add_task(task)
        world.reset()
        build_s = time.time() - t0
        record("OK", "씬 빌드", "%.1f 초" % build_s)
        if build_s > 60:
            record("INFO", "빌드가 느림",
                   "과실마다 정점 색을 칠하느라 인스턴싱이 안 됨. 개수를 줄이거나 색 방식 변경 검토")

        fruits = task.get_observations()["fruits"]
        record("OK", "과실 스폰", "%d개" % len(fruits))

        stage = omni.usd.get_context().get_stage()
        rb = sum(1 for f in fruits
                 if UsdPhysics.RigidBodyAPI(stage.GetPrimAtPath(f["path"])))
        record("OK" if rb == len(fruits) else "FAIL",
               "과실 RigidBody", "%d/%d" % (rb, len(fruits)))

        # 성능: 렌더 포함 스텝
        for _ in range(10):
            world.step(render=True)
        t0 = time.time()
        N = 60
        for _ in range(N):
            world.step(render=True)
        fps = N / (time.time() - t0)
        verdict = "OK" if fps >= 20 else ("INFO" if fps >= 10 else "FAIL")
        record(verdict, "렌더 FPS", "%.1f fps (과실 %d개)" % (fps, len(fruits)))

        # 수확 + 재현성
        ripe = [f for f in fruits if f["class_name"] == "fully_ripe"]
        if ripe:
            before = len(task.pickable_fruits())
            task.detach_fruit(ripe[0]["path"])
            after = len(task.pickable_fruits())
            prim = stage.GetPrimAtPath(ripe[0]["path"])
            kin = UsdPhysics.RigidBodyAPI(prim).GetKinematicEnabledAttr().Get()
            record("OK" if (after == before - 1 and kin is False) else "FAIL",
                   "수확(kinematic 해제)", "%d -> %d, kinematic=%s" % (before, after, kin))

            task.post_reset()
            kin2 = UsdPhysics.RigidBodyAPI(prim).GetKinematicEnabledAttr().Get()
            record("OK" if kin2 is True else "FAIL",
                   "post_reset 복원", "kinematic=%s" % kin2)
        else:
            record("SKIP", "수확 검증", "fully_ripe 과실이 없음")

    except Exception:
        record("FAIL", "씬 빌드", traceback.format_exc().splitlines()[-1][:70])


# ============================================================
# 5. 로봇 자산 / RMPflow
# ============================================================
def check_robot() -> None:
    section("5. 로봇 자산 / RMPflow")

    path, loader = try_imports(
        ["isaacsim.robot_motion.motion_generation.interface_config_loader",
         "omni.isaac.motion_generation.interface_config_loader"])
    if not path:
        record("FAIL", "motion_generation", "import 실패 - 경로 확인 필요")
        return
    record("OK", "motion_generation", path)

    # RMPflow 기본 제공 로봇 목록 (API 이름이 버전마다 다를 수 있어 후보를 다 시도)
    for fn_name in ["get_supported_robot_policy_pairs",
                    "get_supported_robots_with_lula_kinematics"]:
        try:
            fn = getattr(loader, fn_name)
            record("INFO", fn_name, str(fn())[:160])
        except Exception as e:
            record("SKIP", fn_name, str(e)[:50])

    # Isaac 기본 에셋 서버 (지게차/AMR 확인용)
    try:
        from isaacsim.storage.native import get_assets_root_path
        root = get_assets_root_path()
        record("OK" if root else "FAIL", "Isaac 에셋 루트", str(root))
    except Exception:
        p, fn = try_imports(["omni.isaac.nucleus"], "get_assets_root_path")
        if p:
            record("OK", "Isaac 에셋 루트", "%s -> %s" % (p, fn()))
        else:
            record("FAIL", "Isaac 에셋 루트", "경로 확인 필요")


# ============================================================
# 6. ROS2 브리지
# ============================================================
def check_ros2_bridge() -> None:
    section("6. ROS2 브리지")
    try:
        from isaacsim.core.utils.extensions import enable_extension
        ok = enable_extension("isaacsim.ros2.bridge")
        simulation_app.update()
        record("OK" if ok else "FAIL", "isaacsim.ros2.bridge", "enable=%s" % ok)
    except Exception:
        record("FAIL", "ROS2 브리지", traceback.format_exc().splitlines()[-1][:70])

    for var in ["ROS_DOMAIN_ID", "RMW_IMPLEMENTATION",
                "FASTRTPS_DEFAULT_PROFILES_FILE", "LD_LIBRARY_PATH"]:
        v = os.environ.get(var)
        if var == "LD_LIBRARY_PATH":
            hit = "ros2.bridge" in (v or "")
            record("OK" if hit else "FAIL", "LD_LIBRARY_PATH",
                   "브리지 humble/lib 포함됨" if hit else "브리지 경로 없음! export 확인")
        else:
            record("OK" if v else "INFO", var, v or "미설정")


# ============================================================
def main() -> None:
    print("\n" + "#" * 64)
    print("# 스마트팜 프로젝트 - GPU 머신 일괄 검증")
    print("#" * 64)

    for fn in [check_environment, check_imports, check_tomato_scale,
               check_scene, check_robot, check_ros2_bridge]:
        try:
            fn()
        except Exception:
            record("FAIL", fn.__name__, traceback.format_exc().splitlines()[-1][:70])

    section("요약")
    for v in ["FAIL", "OK", "SKIP", "INFO"]:
        items = [r for r in RESULTS if r[0] == v]
        if items:
            print("\n[%s] %d건" % (v, len(items)))
            for _, name, note in items:
                print("   - %s%s" % (name, ("  (%s)" % note) if note else ""))

    n_fail = sum(1 for r in RESULTS if r[0] == "FAIL")
    print("\n" + "=" * 64)
    print("FAIL %d건. 0이면 다음 단계(로봇/FSM)로 진행 가능." % n_fail)
    print("=" * 64)


main()
simulation_app.close()
