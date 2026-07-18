# -*- coding: utf-8 -*-
"""온실 씬 Task — Isaac 의 BaseTask 로 감싼다.

SceneBuilder 를 대체한다. BaseTask 를 쓰는 이유:
  - World 가 Play/Stop 시 post_reset() 을 불러줘서 초기 상태 복원이 한 곳에 모임
  - get_observations() 가 관측 인터페이스 표준이 됨 (FSM/Detector 가 여기서 읽음)
  - 강의 예제(M0609 pick&place)와 같은 구조 -> 로봇 Task 와 합치기 쉬움
"""
import random

import omni.usd
from pxr import Usd

from isaacsim.core.api.tasks import BaseTask

from pjt_config.settings import SceneConfig
from scene import physics
from scene.ground import Ground
from scene.lighting import Lighting
from scene.greenhouse import Greenhouse
from scene.tomato_plants import TomatoPlants
from scene.warehouse import Warehouse


class GreenhouseTask(BaseTask):
    """온실 + 토마토 재배 라인. 로봇은 아직 없음 (별도 Task 로 추가 예정)."""

    def __init__(self, name: str, cfg: SceneConfig):
        super().__init__(name=name, offset=None)
        self._cfg = cfg
        self._plants: TomatoPlants | None = None
        self._warehouse: Warehouse | None = None
        self._harvested: set[str] = set()

    # ----- BaseTask -----

    def set_up_scene(self, scene) -> None:
        super().set_up_scene(scene)
        stage: Usd.Stage = omni.usd.get_context().get_stage()
        cfg = self._cfg

        # 씬 구성은 매번 같은 시드에서 시작 -> 재현성
        rng = random.Random(cfg.seed)

        g = cfg.greenhouse
        ground = Ground()
        ground.spawn(scene)
        # 홀 바닥: 온실 + 창고 구역 + AMR 통로를 덮는다.
        # 기본 지면의 파란 격자가 지평선에도 안 보이게 시야보다 훨씬 넉넉히 깐다.
        ground.spawn_hall(stage, center=(0.0, 1.5), size=(60.0, 60.0))
        Lighting(cfg.lighting).spawn(stage)
        Greenhouse(g).spawn(stage)

        self._plants = TomatoPlants(cfg.plants, cfg.tomato_assets,
                                    g, cfg.physics, rng)
        self._plants.spawn(stage)

        # 창고 랙 — 온실 뒤(+y) AMR 통로 끝. 랙 중심이 x=0 에 오게 slot0 을 왼쪽으로.
        wh_cfg = cfg.warehouse
        self._warehouse = Warehouse(wh_cfg, cfg.sectors.count)
        origin_x = -(wh_cfg.sectors - 1) * wh_cfg.slot_pitch / 2.0
        self._warehouse.spawn(stage,
                              origin=(origin_x, g.length / 2.0 + 2.5, 0.0))

    def get_observations(self) -> dict:
        """시뮬 정답(ground truth). GroundTruthDetector 가 이걸 그대로 쓴다.

        YOLO 로 갈아끼울 때도 같은 형식을 반환하면 FSM 은 안 바뀐다.
        """
        return {
            "fruits": self.pickable_fruits(),
            "harvested_count": len(self._harvested),
        }

    def post_reset(self) -> None:
        """Play/Stop 반복 시 매번 동일한 초기 상태로 복원.

        수확된 과실을 다시 kinematic 으로 되돌려 원래 자리에 매단다.
        """
        stage = omni.usd.get_context().get_stage()
        for path in self._harvested:
            prim = stage.GetPrimAtPath(path)
            if prim.IsValid():
                physics.set_kinematic(prim, True)
        self._harvested.clear()

    # ----- 수확 -----

    def pickable_fruits(self) -> list[dict]:
        """아직 안 딴 과실만. 수확 대상은 fully_ripe, 제거 대상은 old."""
        if self._plants is None:
            return []
        return [f for f in self._plants.fruits
                if f["path"] not in self._harvested]

    def detach_fruit(self, path: str) -> bool:
        """과실을 줄기에서 분리 (= 커터가 꽃자루를 자른 순간).

        kinematic 을 꺼서 dynamic 으로 전환한다. 물리 breakForce 로 끊지 않는
        이유는 그쪽이 비결정적이라 Play/Stop 재현성이 깨지기 때문.
        """
        if path in self._harvested:
            return False
        stage = omni.usd.get_context().get_stage()
        prim = stage.GetPrimAtPath(path)
        if not prim.IsValid():
            return False
        physics.set_kinematic(prim, False)
        self._harvested.add(path)
        return True
