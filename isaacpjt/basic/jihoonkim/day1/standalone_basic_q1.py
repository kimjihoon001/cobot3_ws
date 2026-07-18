from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})     # 1. Application

import numpy as np
import omni.usd
from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid

# =========================================================
# Q1. prim_path 와 name 이 겹치면 왜 에러가 나는가?
#     -> prim_path 는 USD Stage 안의 고유 주소,
#        name 은 World.Scene 에서 객체를 찾는 식별자.
#        둘 다 중복되면 어떤 객체인지 구분 불가 -> 에러.
# =========================================================

world = World(stage_units_in_meters=1.0)                # 2. World
stage = omni.usd.get_context().get_stage()              # 3. Stage

# 첫 번째 큐브: 정상 추가
cube1 = DynamicCuboid(
    prim_path="/World/Cube",
    name="cube",
    position=np.array([0.0, 0.0, 1.0]),
    scale=np.array([0.3, 0.3, 0.3]),
    color=np.array([1.0, 0.0, 0.0]),
)
world.scene.add_default_ground_plane()
world.scene.add(cube1)
print("[성공] 첫 번째 큐브 추가 (prim_path=/World/Cube, name=cube)")

# 두 번째 큐브: 같은 prim_path + 같은 name -> 에러 발생 확인
try:
    cube2 = DynamicCuboid(
        prim_path="/World/Cube",   # 중복 주소
        name="cube",               # 중복 이름
        position=np.array([0.0, 0.0, 2.0]),
        scale=np.array([0.3, 0.3, 0.3]),
        color=np.array([0.0, 0.0, 1.0]),
    )
    world.scene.add(cube2)
    print("[?] 중복 추가가 되어버림 (예상과 다름)")
except Exception as e:
    print("[에러 발생] prim_path / name 중복 -> 추가 불가")
    print("   이유:", e)

world.reset()

step_count = 0
while simulation_app.is_running():
    world.step(render=True)
    step_count += 1

simulation_app.close()
