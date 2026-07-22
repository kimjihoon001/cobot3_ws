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
from scene import pedicel, physics
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
        Greenhouse(g).spawn(stage, back_wall=False)   # 뒷벽은 창고와 공유(벽 하나로 붙임)

        self._plants = TomatoPlants(cfg.plants, cfg.tomato_assets,
                                    g, cfg.physics, rng)
        self._plants.spawn(stage)

        # 창고 방 — 온실 뒷벽에 **벽 하나로 붙인다**(gap 없음, 팀 피드백 2026-07-20).
        # 방 중심 x=0, 방 앞벽(-y)이 온실 뒷벽(+y=length/2)과 겹쳐 공유 칸막이가 된다.
        wh_cfg = cfg.warehouse
        self._warehouse = Warehouse(wh_cfg, cfg.sectors.count)
        wh_origin = (0.0, g.length / 2.0 + wh_cfg.depth / 2.0, 0.0)
        self._warehouse.spawn(stage, origin=wh_origin, room_w=g.width)
        # 방 폭·높이는 재배 공간과 동일(팀 피드백 2026-07-19), 천장 없음
        self._warehouse.spawn_building(stage, room_w=g.width, room_h=g.height)
        self._warehouse.load_crates(stage)      # 슬롯에 표준 컨테이너(시각)

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

        끊었던 꽃자루 조인트를 다시 이어(jointEnabled=True) 과실을 원래 자리에 매단다.
        World 가 prim 변환을 초기 상태로 되돌리므로 조인트만 복원하면 된다.
        """
        stage = omni.usd.get_context().get_stage()
        for path in self._harvested:
            joint = self._joint_of(path)
            if joint:
                pedicel.restore(stage, joint)
        self._harvested.clear()

    # ----- 수확 -----

    def pickable_fruits(self) -> list[dict]:
        """아직 안 딴 과실만. 수확 대상은 ripe(익은거), 제거 대상은 spoiled(상한거)."""
        if self._plants is None:
            return []
        return [f for f in self._plants.fruits
                if f["path"] not in self._harvested]

    def _joint_of(self, path: str) -> str | None:
        """과실 경로 -> 꽃자루 조인트 경로."""
        if self._plants is None:
            return None
        for f in self._plants.fruits:
            if f["path"] == path:
                return f.get("joint")
        return None

    def detach_fruit(self, path: str) -> bool:
        """과실을 줄기에서 분리 (= 커터가 distal 꽃자루를 자른 순간).

        꽃자루 조인트를 끊는다(jointEnabled=False). 과실은 이미 dynamic 이라 낙하한다.
        물리 breakForce 로 자연히 끊기길 기다리지 않는 이유는 그쪽이 비결정적이라
        Play/Stop 재현성이 깨지기 때문 — cut 은 코드로 끊어 결정적이다.
        """
        if path in self._harvested:
            return False
        stage = omni.usd.get_context().get_stage()
        fruit = stage.GetPrimAtPath(path)
        if not fruit.IsValid():
            return False
        # kinematic(고정) 과실을 dynamic 으로 전환 = 줄기에서 분리·낙하/그리퍼 파지
        # (§5.3: 절단 순간에만). 조인트 없이 이것만으로 매달림이 풀린다.
        physics.set_kinematic(fruit, False)
        joint = self._joint_of(path)          # 옛 조인트 방식 호환 — 있으면 같이 끊는다
        if joint:                             # ""(조인트 없음)·None 이면 건너뜀
            pedicel.cut(stage, joint)
        self._harvested.add(path)
        return True
