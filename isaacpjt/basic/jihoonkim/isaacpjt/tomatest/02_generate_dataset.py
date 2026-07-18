# -*- coding: utf-8 -*-
"""(2) Replicator 로 YOLO 학습 데이터셋 자동 생성 — Isaac Sim 5.1.

매 프레임: 토마토 여러 개를 랜덤 클래스/모양/포즈로 스폰 + 조명 랜덤 →
BasicWriter 가 RGB + 2D bounding box(클래스 라벨 포함)를 자동 저장.
색을 우리가 지정하므로 라벨이 물리적으로 일치 (라벨 오류 0).

실행: isaac_python 02_generate_dataset.py
출력: OUT_DIR (rgb/, bounding_box_2d_tight/ ...) → YOLO 포맷으로 변환해 학습.
"""
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})   # 데이터 생성은 headless 가 빠름

import os
import sys
import random
import omni.usd
import omni.replicator.core as rep
from isaacsim.core.api import World
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.utils.semantics import add_update_semantics
from pxr import UsdGeom, UsdLux, Gf

BASE = os.path.dirname(os.path.abspath(__file__))    # 이 스크립트 폴더(isaacpjt)
sys.path.insert(0, BASE)                             # tomato_lib import 보장
import tomato_lib as T

sys.path.insert(0, os.path.dirname(BASE))          # isaacpjt/ (config import 용)
from pjt_config.settings import SceneConfig
from vision.yolo_dataset import CLASSES as YOLO_CLASSES

# ===== 설정 =====
_CFG = SceneConfig()
USD_DIR = _CFG.tomato_assets.usd_dir
OUT_DIR = os.path.join(BASE, "tomato_dataset")
NUM_FRAMES = 500
TOMATOES_PER_FRAME = (4, 12)

# 씬과 같은 크기여야 한다. 여기 따로 박아두면 학습셋의 토마토 크기가
# 실제 씬과 달라지고, YOLO 는 조용히 학습되고 mAP 만 낮게 나온다.
TOMATO_SCALE = _CFG.tomato_assets.scale

# YOLO 라벨. vision/yolo_dataset.py 가 class_id 를 이 순서로 매기므로 같아야 한다.
CLASSES = list(YOLO_CLASSES)

world = World(stage_units_in_meters=1.0)
stage = omni.usd.get_context().get_stage()
world.scene.add_default_ground_plane()

# 사용할 몸통 USD 목록 (모양 변형들 = 데이터 다양성). 꼭지는 제외.
body_usds = [os.path.join(USD_DIR, f) for f in sorted(os.listdir(USD_DIR))
             if f.endswith(".usd") and "_calyx" not in f]
assert body_usds, "USD_DIR 에 토마토 USD 가 없음 (먼저 00_convert 실행)"

# 카메라 (위에서 내려다봄) + 렌더 프로덕트
cam = UsdGeom.Camera.Define(stage, "/World/Camera")
cam_xf = UsdGeom.Xformable(cam.GetPrim())
cam_t = cam_xf.AddTranslateOp()
cam_t.Set(Gf.Vec3d(0, 0, 3.0))
render_product = rep.create.render_product("/World/Camera", (640, 640))

# 조명
light = UsdLux.DistantLight.Define(stage, "/World/Light")

# 라이터: RGB + 2D bbox (YOLO 변환용)
writer = rep.WriterRegistry.get("BasicWriter")
writer.initialize(output_dir=OUT_DIR, rgb=True, bounding_box_2d_tight=True)
writer.attach([render_product])

spawned = []


def randomize_scene():
    global spawned
    # 이전 토마토 제거
    for p in spawned:
        stage.RemovePrim(p)
    spawned = []

    # 조명 랜덤 (밝기/색온도)
    light.GetIntensityAttr().Set(random.uniform(1500, 5000))
    c = random.uniform(0.85, 1.0)
    light.GetColorAttr().Set(Gf.Vec3f(1.0, c, c * random.uniform(0.9, 1.0)))

    # 카메라 높이/살짝 흔들기
    cam_t.Set(Gf.Vec3d(random.uniform(-0.2, 0.2),
                       random.uniform(-0.2, 0.2),
                       random.uniform(2.5, 3.5)))

    # 토마토 랜덤 스폰
    n = random.randint(*TOMATOES_PER_FRAME)
    for i in range(n):
        cls = random.choice(CLASSES)
        body = random.choice(body_usds)
        path = "/World/Tomato_%d" % i
        add_reference_to_stage(body, path)
        xf = UsdGeom.Xform(stage.GetPrimAtPath(path))
        xf.AddTranslateOp().Set(Gf.Vec3d(random.uniform(-0.4, 0.4),
                                         random.uniform(-0.4, 0.4),
                                         random.uniform(0.03, 0.06)))
        xf.AddRotateXYZOp().Set(Gf.Vec3f(random.uniform(0, 360),
                                         random.uniform(0, 360),
                                         random.uniform(0, 360)))
        xf.AddScaleOp().Set(Gf.Vec3f(TOMATO_SCALE, TOMATO_SCALE, TOMATO_SCALE))
        T.apply_ripeness_color(stage, path, cls)
        # 시맨틱 라벨 = 클래스 (bbox 에 이 이름이 붙음)
        add_update_semantics(stage.GetPrimAtPath(path), semantic_label=cls, type_label="class")
        spawned.append(path)


# 메인 루프: 씬 랜덤화 → 렌더/저장
for frame in range(NUM_FRAMES):
    randomize_scene()
    rep.orchestrator.step(rt_subframes=4)
    if (frame + 1) % 50 == 0:
        print("[%d/%d] frames" % (frame + 1, NUM_FRAMES))

rep.orchestrator.wait_until_complete()
print("데이터셋 생성 완료 ->", OUT_DIR)
simulation_app.close()
