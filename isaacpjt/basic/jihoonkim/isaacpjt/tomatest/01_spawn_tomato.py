# -*- coding: utf-8 -*-
"""(1) 토마토 1개 스폰 + 익음/불량 색 입히기 — Isaac 흐름 익히기용.

제공된 기반코드(RedCube 예제)에서 딱 한 발짝:
  큐브 -> 토마토 USD,  color=red -> 클래스별 색(그라데이션).

실행: isaac_python 01_spawn_tomato.py
CLASS_NAME 만 바꿔가며 ripe/spoiled 확인 (2026-07-18 수확·운반 피벗 → 2클래스).
"""
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

import os
import sys
import omni.usd
from isaacsim.core.api import World
from isaacsim.core.utils.stage import add_reference_to_stage
from pxr import UsdGeom, Gf

BASE = os.path.dirname(os.path.abspath(__file__))  # 이 스크립트 폴더(isaacpjt)
sys.path.insert(0, BASE)                           # tomato_lib import 보장
import tomato_lib as T

sys.path.insert(0, os.path.dirname(BASE))          # isaacpjt/ (config import 용)
from pjt_config.settings import SceneConfig

# ===== 설정 =====
_CFG = SceneConfig()
USD_DIR = _CFG.tomato_assets.usd_dir
BODY_USD = os.path.join(USD_DIR, "tomato_ripe_01.usd")
CALYX_USD = os.path.join(USD_DIR, "tomato_ripe_01_calyx.usd")
CLASS_NAME = "ripe"               # ripe(익은거) / spoiled(상한거)
TOMATO_SCALE = _CFG.tomato_assets.scale   # 씬과 같은 값을 쓴다. 여기서 따로 정하지 말 것

world = World(stage_units_in_meters=1.0)
stage = omni.usd.get_context().get_stage()
world.scene.add_default_ground_plane()


def place(prim_path, usd_path, scale):
    add_reference_to_stage(usd_path, prim_path)
    xf = UsdGeom.Xform(stage.GetPrimAtPath(prim_path))
    xf.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 0.1))   # 지면 위
    xf.AddScaleOp().Set(Gf.Vec3f(scale, scale, scale))


# 몸통
place("/World/Tomato", BODY_USD, TOMATO_SCALE)
frac = T.apply_ripeness_color(stage, "/World/Tomato", CLASS_NAME)
print("class=%s  red_fraction=%s" % (CLASS_NAME, frac))

# 꼭지 (항상 초록)
if os.path.exists(CALYX_USD):
    place("/World/Calyx", CALYX_USD, TOMATO_SCALE)
    T.apply_flat_color(stage, "/World/Calyx", T.GREEN)

world.reset()
while simulation_app.is_running():
    world.step(render=True)
simulation_app.close()
