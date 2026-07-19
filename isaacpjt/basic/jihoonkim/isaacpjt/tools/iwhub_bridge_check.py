# -*- coding: utf-8 -*-
"""스파이크 6 — iw.hub ROS2 브리지 (Isaac 안 OmniGraph, 코드 생성 §5.8).

2026-07-19 GPU 검증 완료본(세션 scratchpad iw_bridge.py에서 이관). "GPU 없이 검증 안
되던 유일 코드(ros/graph.py 계열)"의 실측 레퍼런스 — 노드 타입명은 create-probe 확인값.

  ROS2 --/iwhub/joint_command(sensor_msgs/JointState)--> Sub -> ArtController -> iw.hub
  ROS2 <--/iwhub/joint_states------------------------- Pub <- iw.hub
  + Context(domain 108) + Clock(/clock)

실행 (env 필수 — 없으면 "ROS2 Bridge startup failed"):
  LD_LIBRARY_PATH=<isaac>/exts/isaacsim.ros2.bridge/humble/lib:$LD_LIBRARY_PATH \\
  RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_DOMAIN_ID=108 \\
  isaac_python tools/iwhub_bridge_check.py [--headless]
  (<isaac> = ~/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release.
   시스템 ROS2 는 여전히 source 안 한다 — 브리지 내부 humble lib 만 쓴다. §2)

확인 (GPU 의 ROS2 터미널에서):
  ros2 topic echo /iwhub/joint_states --once
  ros2 topic pub -1 /iwhub/joint_command sensor_msgs/JointState \\
    '{name: [left_wheel_joint, right_wheel_joint], velocity: [3.0, 3.0]}'
"""
import os
import sys

from isaacsim import SimulationApp

HEADLESS = "--headless" in sys.argv
app = SimulationApp({"headless": HEADLESS})

from isaacsim.core.utils.extensions import enable_extension  # noqa: E402

for ext in ("isaacsim.core.nodes", "isaacsim.ros2.bridge"):
    enable_extension(ext)
for _ in range(20):
    app.update()

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import omni.graph.core as og  # noqa: E402
import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.api.robots import Robot  # noqa: E402
from isaacsim.core.utils.stage import add_reference_to_stage  # noqa: E402
from pxr import Usd, UsdLux, UsdPhysics  # noqa: E402

from pjt_config.settings import SceneConfig  # noqa: E402
from pjt_utils.xform import set_translate  # noqa: E402
from robots import assets  # noqa: E402

NS = "iwhub"
GRAPH = "/World/IwBridge"
# create-probe(2026-07-19)로 검증한 실제 타입명 — ros/graph.py 후보 갱신의 근거
T = {
    "OnTick": "omni.graph.action.OnPlaybackTick",
    "Ctx": "isaacsim.ros2.bridge.ROS2Context",
    "Sub": "isaacsim.ros2.bridge.ROS2SubscribeJointState",
    "Pub": "isaacsim.ros2.bridge.ROS2PublishJointState",
    "Art": "isaacsim.core.nodes.IsaacArticulationController",
    "SimTime": "isaacsim.core.nodes.IsaacReadSimulationTime",
    "Clock": "isaacsim.ros2.bridge.ROS2PublishClock",
}


def art_root(stage, under):
    for p in Usd.PrimRange(stage.GetPrimAtPath(under)):
        if p.HasAPI(UsdPhysics.ArticulationRootAPI):
            return str(p.GetPath())
    return None


def main():
    world = World(stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane()
    stage = omni.usd.get_context().get_stage()
    UsdLux.DistantLight.Define(stage, "/World/Light").CreateIntensityAttr(4000)

    add_reference_to_stage(
        assets.resolve(SceneConfig().robots.assets.iwhub, "iw.hub"), "/World/IwHub")
    set_translate(stage.GetPrimAtPath("/World/IwHub"), (0.0, 0.0, 0.0))
    art = art_root(stage, "/World/IwHub")
    print(f">>> iw.hub articulation root: {art}")

    keys = og.Controller.Keys
    og.Controller.edit(
        {"graph_path": GRAPH, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnTick", T["OnTick"]), ("Ctx", T["Ctx"]), ("SimTime", T["SimTime"]),
                ("Clock", T["Clock"]), ("Sub", T["Sub"]), ("Art", T["Art"]),
                ("Pub", T["Pub"]),
            ],
            keys.CONNECT: [
                ("OnTick.outputs:tick", "Clock.inputs:execIn"),
                ("OnTick.outputs:tick", "Sub.inputs:execIn"),
                ("OnTick.outputs:tick", "Art.inputs:execIn"),
                ("OnTick.outputs:tick", "Pub.inputs:execIn"),
                ("Ctx.outputs:context", "Clock.inputs:context"),
                ("Ctx.outputs:context", "Sub.inputs:context"),
                ("Ctx.outputs:context", "Pub.inputs:context"),
                ("SimTime.outputs:simulationTime", "Clock.inputs:timeStamp"),
                ("SimTime.outputs:simulationTime", "Pub.inputs:timeStamp"),
                ("Sub.outputs:jointNames", "Art.inputs:jointNames"),
                ("Sub.outputs:positionCommand", "Art.inputs:positionCommand"),
                ("Sub.outputs:velocityCommand", "Art.inputs:velocityCommand"),
                ("Sub.outputs:effortCommand", "Art.inputs:effortCommand"),
            ],
            keys.SET_VALUES: [
                ("Ctx.inputs:domain_id", 108),
                ("Ctx.inputs:useDomainIDEnvVar", False),
                ("Sub.inputs:topicName", f"/{NS}/joint_command"),
                ("Pub.inputs:topicName", f"/{NS}/joint_states"),
                ("Clock.inputs:topicName", "/clock"),
            ],
        },
    )
    # targetPrim(관절 대상)은 relationship 이라 USD 로 건다
    for node in ("Art", "Pub"):
        prim = stage.GetPrimAtPath(f"{GRAPH}/{node}")
        rel = prim.GetRelationship("inputs:targetPrim")
        if not rel:
            rel = prim.CreateRelationship("inputs:targetPrim")
        rel.SetTargets([art])
    print(f">>> 그래프 생성 완료: {GRAPH}")

    world.scene.add(Robot(prim_path=art, name="iw"))
    world.reset()
    omni.timeline.get_timeline_interface().play()

    print(f">>> 발행: /{NS}/joint_states  |  수신: /{NS}/joint_command  (domain 108)")
    print(">>> ROS2 터미널에서: ros2 topic echo /iwhub/joint_states --once")

    n = 0
    while app.is_running():
        world.step(render=True)
        n += 1
        if HEADLESS and n >= 600:      # 헤드리스는 10초쯤 돌고 종료(토픽 확인용)
            break
    app.close()


if __name__ == "__main__":
    main()
