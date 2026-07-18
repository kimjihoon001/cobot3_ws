from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})     # 1. Application

import numpy as np
import time
import omni.usd
from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid

world = World(stage_units_in_meters=1.0)                # 2. World
stage = omni.usd.get_context().get_stage()              # 3. Stage

cube_prim = DynamicCuboid(                              # 4. Prim
    prim_path="/World/RedCube",
    name="red_cube",
    position=np.array([0.0, 0.0, 0.15]),
    scale=np.array([0.3, 0.3, 0.3]),
    color=np.array([1.0, 0.0, 0.0]),
)

world.scene.add_default_ground_plane()                  # 5. Scene
world.scene.add(cube_prim)

world.reset()

step_count = 0
moved = False
was_playing = False

while simulation_app.is_running():                      # 6. Simulation
    world.step(render=True)
    time.sleep(0.01)

    is_playing = world.is_playing()

    # Stop → Play 전환 감지: 처음부터 다시 시작
    if is_playing and not was_playing:
        step_count = 0
        moved = False
        print("[리셋] Play 시작 → step_count = 0")
    was_playing = is_playing

    # Stop 상태면 카운트하지 않음
    if not is_playing:
        continue

    step_count += 1

    if step_count % 100 == 0:
        print(f"step: {step_count}")

    # 300 스텝 마단
    if step_count % 300 ==0:
        cube_prim.set_world_pose(position=np.array([0.0, 0.0, 1.0]))
        moved = True
        print("[이동] 큐브 순간이동")

simulation_app.close()