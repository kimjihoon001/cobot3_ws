from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

from isaacsim.core.utils.extensions import enable_extension
enable_extension("isaacsim.ros2.bridge")
enable_extension("omni.graph.bundle.action")
enable_extension("omni.graph.window.action")
simulation_app.update()

from pathlib import Path
import time
import omni.usd
import omni.graph.core as og
from pxr import Usd, UsdGeom, UsdLux

USD_PATH = str(Path(__file__).resolve().parent / "Collected_m0609_camera/World0.usd")

# /World prim 명시적 생성 후 USD reference 연결
stage = omni.usd.get_context().get_stage()
UsdGeom.Xform.Define(stage, "/World")
world_prim = stage.GetPrimAtPath("/World")
world_prim.GetReferences().AddReference(USD_PATH)

# 조명 추가 (DomeLight)
dome_light = UsdLux.DomeLight.Define(stage, "/World/DomeLight")
dome_light.CreateIntensityAttr(1000.0)

for _ in range(15):
    simulation_app.update()

# USD 안에 rgb/depth를 ROS2로 publish하는 Action Graph가 없으므로 직접 생성
camera_paths = [
    str(prim.GetPath())
    for prim in Usd.PrimRange(stage.GetPrimAtPath("/World"))
    if prim.GetTypeName() == "Camera"
]
print("발견된 카메라 prim:", camera_paths)

keys = og.Controller.Keys
for i, camera_path in enumerate(camera_paths):
    graph_path = f"/World/ROS2CameraGraph_{i}"
    og.Controller.edit(
        {"graph_path": graph_path, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnTick", "omni.graph.action.OnPlaybackTick"),
                ("CreateRenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                ("RGBPublish", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("DepthPublish", "isaacsim.ros2.bridge.ROS2CameraHelper"),
            ],
            keys.CONNECT: [
                ("OnTick.outputs:tick", "CreateRenderProduct.inputs:execIn"),
                ("CreateRenderProduct.outputs:execOut", "RGBPublish.inputs:execIn"),
                ("CreateRenderProduct.outputs:execOut", "DepthPublish.inputs:execIn"),
                ("CreateRenderProduct.outputs:renderProductPath", "RGBPublish.inputs:renderProductPath"),
                ("CreateRenderProduct.outputs:renderProductPath", "DepthPublish.inputs:renderProductPath"),
            ],
            keys.SET_VALUES: [
                ("CreateRenderProduct.inputs:cameraPrim", camera_path),
                ("RGBPublish.inputs:topicName", "rgb"),
                ("RGBPublish.inputs:type", "rgb"),
                ("DepthPublish.inputs:topicName", "depth"),
                ("DepthPublish.inputs:type", "depth"),
            ],
        },
    )

# # 로드된 prim 구조 출력
# print("\n" + "=" * 60)
# print("Stage prim 구조")
# print("=" * 60)
# for prim in Usd.PrimRange(stage.GetPseudoRoot()):
#     depth = len(str(prim.GetPath()).split("/")) - 2
#     indent = "  " * depth
#     print(f"{indent}{prim.GetName()}  [{prim.GetTypeName()}]")

print("\n시뮬레이션 실행 중 (Play 버튼을 눌러 확인하세요)")

while simulation_app.is_running():
    simulation_app.update()
    time.sleep(0.016)

simulation_app.close()
