from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})     # 1. Application

import numpy as np
import omni.usd
from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid

# =========================================================
# Q4. 2단계에서 stage 를 어떻게 사용할 것 같은가?
#     -> Stage 는 USD Scene 전체를 직접 제어하는 객체.
#        Prim 검색 / 속성 변경 / Transform 수정 / 삭제 등을
#        world.scene.add(...) 대신 stage 로 직접 다룰 수 있다.
# =========================================================

world = World(stage_units_in_meters=1.0)                # 2. World
stage = omni.usd.get_context().get_stage()              # 3. Stage (USD Scene 직접 제어)

world.scene.add_default_ground_plane()

cube = DynamicCuboid(
    prim_path="/World/Cube",
    name="cube",
    position=np.array([0.0, 0.0, 1.0]),
    scale=np.array([0.3, 0.3, 0.3]),
    color=np.array([0.2, 0.6, 1.0]),
)
world.scene.add(cube)

world.reset()

# --- Stage 직접 사용 예시 ---------------------------------
# 1) Prim 검색: Stage 에서 경로로 Prim 을 직접 가져오기
prim = stage.GetPrimAtPath("/World/Cube")
print("[검색] GetPrimAtPath('/World/Cube') ->", prim.GetPath(), "| valid:", prim.IsValid())

# 2) Prim 타입 / 속성 확인
print("[정보] Prim Type:", prim.GetTypeName())
print("[정보] 속성 목록(일부):", [a.GetName() for a in prim.GetAttributes()][:5])

# 3) Stage 전체 Prim 순회
print("[순회] Stage 안의 Prim 목록:")
for p in stage.Traverse():
    print("   -", p.GetPath())
# ---------------------------------------------------------

step_count = 0
moved = False
while simulation_app.is_running():
    world.step(render=True)
    step_count += 1

    # 4) 200 스텝 후 Stage 로 가져온 Prim 을 이용해 위치 변경
    if step_count == 200 and not moved:
        cube.set_world_pose(position=np.array([1.0, 1.0, 1.5]))
        moved = True
        print("[수정] Stage 로 찾은 큐브의 위치를 변경")

    if step_count >= 400:
        break

simulation_app.close()
