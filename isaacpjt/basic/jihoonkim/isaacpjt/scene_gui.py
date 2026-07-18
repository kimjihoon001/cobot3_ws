# -*- coding: utf-8 -*-
"""현재 씬을 GUI 로 띄워 눈으로 보는 용도 (배경 잎 ON, ROS 없음).

main.py --no-ros 와 같은 씬이되 use_aoc_background=True 로 배경 식물을 얹는다.
발표/확인용 미리보기. 실제 진입점은 main.py.
"""
import sys

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import os

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from isaacsim.core.api import World
from isaacsim.core.utils.viewports import set_camera_view

from pjt_config.settings import SceneConfig
from scene.greenhouse_task import GreenhouseTask


def main() -> None:
    cfg = SceneConfig()
    cfg.plants.use_aoc_background = True        # 배경 잎 ON (시각 전용)

    world = World(stage_units_in_meters=1.0)
    task = GreenhouseTask(name="greenhouse", cfg=cfg)
    world.add_task(task)
    world.reset()

    g = cfg.greenhouse
    set_camera_view(eye=[g.width * 0.9, -g.length * 0.8, 10.0],
                    target=[0.0, 2.0, 0.6])

    obs = task.get_observations()
    print("\n[미리보기] 배경 잎 ON, 과실 %d개. 마우스로 둘러보세요. 닫으면 종료.\n"
          % len(obs["fruits"]))

    while simulation_app.is_running():
        world.step(render=True)


main()
simulation_app.close()
