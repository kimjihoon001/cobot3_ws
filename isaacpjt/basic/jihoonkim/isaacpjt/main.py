# -*- coding: utf-8 -*-
"""스마트팜 온실 씬 진입점 (Isaac Sim 5.1 Standalone).

실행: isaac_python main.py            (씬 + ROS2 브리지)
      isaac_python main.py --no-ros   (씬만. 브리지가 안 붙을 때 씬만 확인용)
      isaac_python main.py --headless

FSM 은 여기서 안 돈다. dev 머신의 ros2 run harvest_fsm fsm_node 가 돌리고,
여기는 그 명령을 받아 로봇을 움직이는 쪽이다 (다중PC 구성).
"""
import sys

GUI = "--headless" not in sys.argv
NO_ROS = "--no-ros" in sys.argv

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": not GUI})

import os

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from isaacsim.core.api import World

from pjt_config.settings import SceneConfig
from scene.greenhouse_task import GreenhouseTask


def build_bridge(task):
    """ROS2 브리지. 실패해도 씬은 계속 띄운다.

    graph.py 는 GPU 없이 검증이 안 된 유일한 코드다. 여기서 죽으면 씬 확인까지
    같이 못 하게 되므로, 실패를 알리고 씬만이라도 굴린다.
    """
    from robots.stub_harvester import StubHarvester
    from ros.harvest_bridge import HarvestBridge

    try:
        robot = StubHarvester(task.detach_fruit)
        return HarvestBridge(task, robot)
    except Exception as e:
        import traceback
        print("\n" + "=" * 64)
        print("[Bridge] 생성 실패 — 씬만 띄운다. FSM 명령은 안 먹는다.")
        print("=" * 64)
        traceback.print_exc()
        print("=" * 64 + "\n")
        return None


def main() -> None:
    cfg = SceneConfig()
    world = World(stage_units_in_meters=1.0)

    task = GreenhouseTask(name="greenhouse", cfg=cfg)
    world.add_task(task)
    world.reset()

    if GUI:
        # 기본 카메라는 원점에서 멀어 바닥 그리드만 보인다.
        # 지붕이 없으므로 높은 3/4 부감으로 온실 내부 + 창고까지 한눈에 잡는다.
        from isaacsim.core.utils.viewports import set_camera_view
        g = cfg.greenhouse
        set_camera_view(
            eye=[g.width * 0.9, -g.length * 0.8, 12.0],
            target=[0.0, 2.0, 0.5])

    bridge = None if NO_ROS else build_bridge(task)

    obs = task.get_observations()
    fruits = obs["fruits"]
    counts: dict[str, int] = {}
    for f in fruits:
        counts[f["class_name"]] = counts.get(f["class_name"], 0) + 1
    print("\n[Scene] 과실 %d개" % len(fruits))
    for name in sorted(counts):
        print("  %-11s %4d" % (name, counts[name]))
    print("  수확 대상(ripe) %d개 / 제거 대상(spoiled) %d개\n"
          % (counts.get("ripe", 0), counts.get("spoiled", 0)))

    if bridge is not None:
        print("[Bridge] 대기 중 — dev 머신에서 ros2 run harvest_fsm fsm_node\n")

    # Play/Stop 반복 시 동일한 초기 상태에서 재시작 (재현성)
    was_playing = False
    while simulation_app.is_running():
        world.step(render=True)
        is_playing = world.is_playing()
        if is_playing and not was_playing:
            world.reset()
            if bridge is not None:
                bridge.reset()
        was_playing = is_playing

        # 브리지는 재생 중에만 돈다 (그래프도 OnPlaybackTick 으로 물려 있다)
        if is_playing and bridge is not None:
            bridge.tick()


main()
simulation_app.close()
