# -*- coding: utf-8 -*-
"""Nav2 브리지 노드 타입명 create-probe — GPU 에서 1회 돌려 실제 타입명을 확정한다.

왜 필요한가 (§8 graph.py 교훈): Isaac 5.1 의 OmniGraph 노드 타입 문자열은 버전마다
다르고(`isaacsim.*` vs `omni.isaac.*`), 추측으로 박으면 Play 시 그래프가 터진다.
조인트 브리지의 `T` 딕셔너리도 이렇게 create-probe 로 확정한 값이다
(tools/iwhub_bridge_check.py · ros/robot_bridge.py). 이 스크립트는 Nav2 에 필요한
Twist구독·차동컨트롤러·오도메트리·TF·라이다 노드를 **후보별로 실제 생성해 보고** OK/FAIL 을
찍는다. OK 로 뜬 이름을 ros/robot_bridge.py 의 `T` 에 넣으면 된다.

실행 (env 필수 — 조인트 브리지와 동일 레시피):
  LD_LIBRARY_PATH=<isaac>/exts/isaacsim.ros2.bridge/humble/lib:$LD_LIBRARY_PATH \\
  RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_DOMAIN_ID=108 \\
  isaac_python tools/nav2_node_probe.py [--headless]

출력: 후보별 [OK]/[--] 표. 각 역할에서 첫 [OK] 가 쓸 이름이다. 전부 [--] 면 그 역할의
확장(extension)이 안 켜졌거나 이름이 완전히 다른 것 — 하단 '레지스트리 덤프'에서
키워드로 찾는다.
"""
import os
import sys

from isaacsim import SimulationApp

HEADLESS = "--headless" in sys.argv
app = SimulationApp({"headless": HEADLESS})

from isaacsim.core.utils.extensions import enable_extension  # noqa: E402

# Nav2 노드가 사는 확장들 — 켜지 않으면 타입이 등록조차 안 돼 전부 FAIL 로 보인다.
for ext in ("isaacsim.core.nodes", "isaacsim.ros2.bridge",
            "isaacsim.robot.wheeled_robots", "isaacsim.sensors.rtx",
            "omni.graph.nodes"):
    try:
        enable_extension(ext)
    except Exception as e:  # 확장명이 다를 수 있음 — 죽지 말고 알린다
        print(f"  [ext] {ext} 활성화 실패(이름 다를 수 있음): {e}")
for _ in range(20):
    app.update()

import omni.graph.core as og  # noqa: E402
import omni.usd  # noqa: E402
from pxr import Usd  # noqa: E402

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from pjt_config.settings import SceneConfig  # noqa: E402
from robots import assets  # noqa: E402

# 역할 -> 후보 타입명(우선순위). 첫 [OK] 를 robot_bridge.py T 에 넣는다.
CANDIDATES = {
    "SubTwist (/cmd_vel 구독)": [
        "isaacsim.ros2.bridge.ROS2SubscribeTwist",
        "omni.isaac.ros2_bridge.ROS2SubscribeTwist",
    ],
    "DiffCtrl (차동구동)": [
        "isaacsim.robot.wheeled_robots.DifferentialController",
        "omni.isaac.wheeled_robots.DifferentialController",
    ],
    "Break3 (벡터 분해)": [
        "omni.graph.nodes.BreakVector3",
    ],
    "ComputeOdom (오도메트리 계산)": [
        "isaacsim.core.nodes.IsaacComputeOdometry",
        "omni.isaac.core_nodes.IsaacComputeOdometry",
    ],
    "PubOdom (/odom 발행)": [
        "isaacsim.ros2.bridge.ROS2PublishOdometry",
        "omni.isaac.ros2_bridge.ROS2PublishOdometry",
    ],
    "PubRawTf (odom->base_link)": [
        "isaacsim.ros2.bridge.ROS2PublishRawTransformTree",
        "omni.isaac.ros2_bridge.ROS2PublishRawTransformTree",
    ],
    "PubTf (base_link->laser)": [
        "isaacsim.ros2.bridge.ROS2PublishTransformTree",
        "omni.isaac.ros2_bridge.ROS2PublishTransformTree",
    ],
    "RtxLidarHelper (/scan 발행)": [
        "isaacsim.ros2.bridge.ROS2RtxLidarHelper",
        "omni.isaac.ros2_bridge.ROS2RtxLidarHelper",
    ],
    "ReadLidar (라이다 읽기, 대안)": [
        "isaacsim.sensors.rtx.IsaacReadRTXLidarData",
        "isaacsim.ros2.bridge.ROS2PublishLaserScan",
        "omni.isaac.ros2_bridge.ROS2PublishLaserScan",
    ],
}


def can_create(type_name: str, idx: int) -> bool:
    """후보 타입으로 임시 그래프에 노드를 실제 생성해 본다. 되면 유효한 이름."""
    gp = f"/World/_probe_{idx}"
    try:
        og.Controller.edit(
            {"graph_path": gp, "evaluator_name": "execution"},
            {og.Controller.Keys.CREATE_NODES: [("n", type_name)]})
        return True
    except Exception:
        return False


def dump_iwhub_sensors():
    """iw.hub 에셋을 스폰해 이미 달린 센서(라이다/카메라) 프림을 찾는다.

    Idealworks iw.hub 는 실물 AMR 이라 에셋에 라이다가 이미 있을 수 있다. 있으면
    새로 만들지 말고 그 프림 경로를 robots/iwhub.py::find_lidar 가 쓰게 한다.
    """
    from isaacsim.core.utils.stage import add_reference_to_stage
    stage = omni.usd.get_context().get_stage()
    root = "/World/IwHubProbe"
    try:
        url = assets.resolve(SceneConfig().robots.assets.iwhub, "iw.hub")
        add_reference_to_stage(url, root)
    except Exception as e:
        print(f"  iw.hub 스폰 실패: {e}")
        return

    KEYS = ("lidar", "laser", "scan", "camera", "imu", "sensor", "range")
    hits = []
    for p in Usd.PrimRange(stage.GetPrimAtPath(root)):
        tname = (p.GetTypeName() or "").lower()
        pname = p.GetName().lower()
        path = str(p.GetPath())
        if any(k in tname for k in KEYS) or any(k in pname for k in KEYS):
            hits.append((path, p.GetTypeName()))
    print(f"\n===== iw.hub 에셋 센서 프림 스캔 ({url}) =====")
    if hits:
        for path, tname in hits:
            print(f"  {tname:24s} {path}")
        print("  ↑ 라이다가 있으면 새로 만들지 말고 이 경로를 iwhub.find_lidar 에 넣는다.")
    else:
        print("  센서로 보이는 프림 없음 — attach_lidar 로 RTX 라이다를 직접 만들어야 함.")


def main():
    print("\n===== Nav2 노드 create-probe (첫 [OK] 를 robot_bridge.py T 에) =====")
    idx = 0
    resolved = {}
    for role, cands in CANDIDATES.items():
        print(f"\n[{role}]")
        first_ok = None
        for c in cands:
            ok = can_create(c, idx)
            idx += 1
            print(f"  {'[OK]' if ok else '[--]'} {c}")
            if ok and first_ok is None:
                first_ok = c
        resolved[role] = first_ok
        if first_ok is None:
            print("   ⚠ 전부 FAIL — 확장 미활성 or 이름 상이. 아래 레지스트리 덤프 참조.")

    # 레지스트리 덤프 — 후보가 전부 틀렸을 때 키워드로 실제 이름을 찾는다.
    print("\n===== 레지스트리 키워드 매칭 (후보가 다 틀렸을 때 참고) =====")
    keywords = ("Twist", "Differential", "Odometry", "Odom", "Transform",
                "Lidar", "LaserScan", "BreakVector")
    try:
        names = list(og.get_registered_node_types())  # 버전에 따라 없을 수 있음
        names = [getattr(n, "get_node_type", lambda: n)() for n in names]
        names = [str(n) for n in names]
    except Exception:
        names = []
    if names:
        for kw in keywords:
            hits = sorted({n for n in names if kw.lower() in n.lower()})
            if hits:
                print(f"  '{kw}':")
                for h in hits:
                    print(f"      {h}")
    else:
        print("  (레지스트리 열거 API 미지원 — create-probe 결과만 신뢰)")

    print("\n===== 요약 (robot_bridge.py T 갱신값) =====")
    for role, name in resolved.items():
        print(f"  {role:34s} -> {name}")

    dump_iwhub_sensors()   # 에셋에 라이다가 이미 있나 (사용자 지적 2026-07-20)
    print()
    app.close()


if __name__ == "__main__":
    main()
