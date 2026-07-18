from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})     # 1. Application

import numpy as np
import omni.usd
from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid

# =========================================================
# Q3. 루프 안에서 world.reset() 을 호출하면 어떻게 될까?
#     -> 매 프레임마다 초기 상태로 되돌아간다.
#        큐브가 떨어지려다가 매번 원래 위치로 리셋되어
#        정상적인 시뮬레이션(낙하)이 진행되지 않는다.
# =========================================================

world = World(stage_units_in_meters=1.0)                # 2. World
stage = omni.usd.get_context().get_stage()              # 3. Stage

world.scene.add_default_ground_plane()

cube = DynamicCuboid(
    prim_path="/World/Cube",
    name="cube",
    position=np.array([0.0, 0.0, 2.0]),   # 공중에서 시작 -> 원래는 떨어져야 함
    scale=np.array([0.3, 0.3, 0.3]),
    color=np.array([1.0, 0.5, 0.0]),
)
world.scene.add(cube)

world.reset()

# ★ 루프 안에서 매 프레임 reset() 호출
print("[관찰] 루프 안에서 매 프레임 world.reset() 호출")
print("       -> 큐브가 떨어지지 못하고 계속 z=2.0 부근으로 초기화됨")

step_count = 0
while simulation_app.is_running():
    world.reset()                # 매 프레임 초기화 (의도적으로 잘못된 예시)
    world.step(render=True)
    step_count += 1
    

simulation_app.close()
