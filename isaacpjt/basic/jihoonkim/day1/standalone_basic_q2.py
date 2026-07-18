from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})     # 1. Application

import numpy as np
import omni.usd
from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid

# =========================================================
# Q2. world.reset() 은 무슨 역할을 하는가?
#
#  -> reset 시점의 상태(위치/자세/속도)를 "초기 상태"로 저장하고,
#     이후 world.reset() 을 다시 부르면 그 초기 상태로 되돌린다.
#     (물리 엔진 play + 파이썬 물리 핸들 생성도 함께 수행)
#
#  확인 방법:
#     z=3 에서 큐브를 떨어뜨림 -> 바닥에 닿음(z≈0.15)
#     -> world.reset() 호출 -> 큐브가 순간 z=3 으로 복귀
# =========================================================

world = World(stage_units_in_meters=1.0)                # 2. World
stage = omni.usd.get_context().get_stage()              # 3. Stage

world.scene.add_default_ground_plane()

cube = DynamicCuboid(
    prim_path="/World/Cube",
    name="cube",
    position=np.array([0.0, 0.0, 3.0]),   # 초기 위치 z=3
    scale=np.array([0.3, 0.3, 0.3]),
    color=np.array([1.0, 0.0, 0.0]),
)
world.scene.add(cube)

world.reset()                             # 초기 상태(z=3) 저장 + 물리 시작
print("[reset] 초기 상태 저장 (z=3.0)\n")

step_count = 0
while simulation_app.is_running():        # 6. Simulation
    world.step(render=True)
    step_count += 1

    # 큐브 높이 출력
    if step_count % 30 == 0:
        z = cube.get_world_pose()[0][2]
        print(f"step {step_count:>4} | 큐브 z={z:5.2f}")

    # 150 스텝마다 reset -> 초기 위치(z=3)로 복귀하는지 확인
    if step_count % 150 == 0:
        z_before = cube.get_world_pose()[0][2]
        world.reset()
        z_after = cube.get_world_pose()[0][2]
        print(f"  >> world.reset() 호출: z {z_before:.2f} -> {z_after:.2f} (초기 위치로 복귀)\n")


simulation_app.close()
