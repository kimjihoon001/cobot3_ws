# -*- coding: utf-8 -*-
"""(3) 실제 온실 씬 + MM 손끝 D455 시점에서 YOLO 도메인 파인튜닝 데이터셋 생성 — Isaac Sim 5.1.

02_generate_dataset.py 와의 차이(왜 새로 짰나):
  02 는 바닥에 토마토를 흩뿌리고 3m 위 top-down 으로 찍는다. 그건 **1차 학습용 일반
  검출기** 도메인이지, 우리 추론 도메인이 아니다. 실제 추론은 수확 MM 의 손끝 D455 가
  트렐리스에 매달린 토마토를 **옆에서** 본다(scene/tomato_plants.py). 파인튜닝 데이터는
  추론과 같은 소스에서 나와야 도메인 갭이 준다 → **실제 greenhouse 씬 + 진짜 D455 프림**.

파이프라인:
  GreenhouseTask 로 온실 빌드(과실이 ripe/spoiled 로 이미 태깅됨)
    → 각 과실 프림에 semantic 라벨 부착(= class_weights 그대로, 라벨 오류 0)
    → MM 스폰 → 손끝 D455 에 render_product 연결
    → 매 프레임 베이스를 섹터 웨이포인트로 텔레포트(+지터) + 조명 랜덤 → 렌더/저장
  출력(BasicWriter: rgb/ + bounding_box_2d_tight/)은 vision/yolo_dataset.py 가
  그대로 YOLO 포맷으로 변환한다(재사용, 손 안 댐).

실행: isaac_python tomatest/03_generate_from_scene.py [프레임수]
  예)  isaac_python tomatest/03_generate_from_scene.py 800

★ GPU 미검증. 아래 [4] 표시(STANDOFF_X·CAM_YAW_OFFSET_DEG·베이스 좌표계)는 첫 실행에서
  DIAG 로그(D455 월드포즈·프레임당 박스수)를 보고 맞춘다 — 이 레포 관행대로 GPU 에서 보정.
"""
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})   # 데이터 생성은 headless 가 빠름

import os
import sys
import math
import random

# 상단 패키지(scene/robots/mm/pjt_config)를 임포트하려면 isaacpjt/ 를 path 에 올린다.
_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_HERE)                        # isaacpjt/
sys.path.insert(0, _ROOT)

# 렌더프로덕트·시맨틱을 쓰려면 replicator 확장을 그래프 생성 전에 켜야 한다(main.py 와 동일).
from isaacsim.core.utils.extensions import enable_extension
for _ext in ("isaacsim.core.nodes", "omni.replicator.core"):
    enable_extension(_ext)
for _ in range(20):
    simulation_app.update()

import numpy as np
import omni.usd
import omni.replicator.core as rep
from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdShade
from isaacsim.core.api import World
from isaacsim.core.api.robots import Robot
from isaacsim.core.utils.semantics import add_update_semantics

from pjt_config.settings import SceneConfig
from pjt_utils.ripeness import RED, BROWN         # 클래스 단색 (씬 색 정의와 동일)
from scene.greenhouse_task import GreenhouseTask
from robots.harvester import HOME_POSE_DEG, HarvestMM
from robot_base import art_root
from mm import POSE as MM_POSE, BASE_JOINTS

# ===== 설정 =====
FRAME_CAP = int(sys.argv[1]) if len(sys.argv) > 1 else None  # 빠른 테스트용 상한(없으면 전체)
Y_STOPS = 5           # 베드-세그먼트당 등간격 y 정지 수 (위 +Y → 아래)
SHOTS_PER_STOP = 15   # 각 정지에서 찍는 장수 (샷마다 위치·조준·팔z·조명 지터) → 60×15=900장
OUT_DIR = os.path.join(_HERE, "scene_dataset")        # BasicWriter 원본
YOLO_DIR = os.path.join(_HERE, "scene_yolo")          # 변환 후 YOLO 데이터셋
IMG_WH = (640, 640)                                   # YOLO 입력 관행(02 와 동일)

# [4] 임의 — GPU 에서 DIAG 보고 보정할 3개 노브:
STANDOFF_X = 1.3           # m  베이스를 이랑에서 X 로 이만큼 떨어뜨려 세운다(D455 가시거리)
CAM_YAW_OFFSET_DEG = 0     # deg 측정 자동조준 위에 얹는 미세보정만(기본 0). 카메라 방위각은
                           #     시작 시 실측(cam_az0)해 이랑을 향하도록 yaw 를 자동 계산한다.
# 베이스엔 z 축이 없어(지면 3축) 카메라 상하 이동은 팔로 한다. shoulder_lift 를 홈자세 기준
# ±ARM_LIFT_DEG 로 흔들어 손끝 D455 가 과실 높이(0.5~1.4m, fruit_height_range)를 훑게 한다.
ARM_LIFT_DEG = 18.0        # [4] 임의 — EE 상하 스윕 폭. 카메라가 하늘/바닥을 보면 줄일 것.
# 한 스테이션의 SHOTS_PER_STOP 장이 같은 시점이면 다양성이 없다 → 샷마다 베이스 x·y·yaw 를
# 미세하게 흔들어 진짜 다른 시점으로 만든다(과적합 방지). [4] 임의 — 통로 폭·프레이밍 보고 조정.
POS_JITTER = 0.15          # m   샷마다 베이스 x·y 이동
YAW_JITTER_DEG = 6.0       # deg 샷마다 조준 미세 회전
# 베이스 3축(x,y,yaw)은 월드가 아니라 스폰 원점(POSE=(0,-12)) 기준 오프셋이다. 가정하지 않고
# 시작 시 실측(joint=0 → 섀시 월드좌표 = base_origin)해 목표 월드좌표에서 빼서 넣는다(벽밖 이탈 방지).

DIAG_FRAMES = 5            # 앞 몇 프레임은 D455 월드포즈·박스수를 찍어 보정용으로 남긴다

random.seed(42)

# ── 씬 빌드 (과실 스폰 + 클래스 태깅은 GreenhouseTask 가 이미 한다) ──
cfg = SceneConfig()
world = World(stage_units_in_meters=1.0)
task = GreenhouseTask(name="greenhouse", cfg=cfg)
world.add_task(task)
world.reset()
stage = omni.usd.get_context().get_stage()

# ── MM 스폰 → 손끝 D455 = 렌더 카메라(추론과 같은 렌즈·장착각) ──
mm = HarvestMM(cfg.robots)
mm.spawn(stage, "/World/Harvester", MM_POSE)
art = art_root(stage, "/World/Harvester")
assert art, "Harvester 아티큘레이션 루트를 못 찾음 — MM 조립 실패"
robot = world.scene.add(Robot(prim_path=art, name="mm"))
world.reset()

# 시작자세(HOME_POSE_DEG)를 기본자세로 박고 정착 (mm.MMDriver.configure 와 동일 로직).
q0 = np.asarray(robot.get_joint_positions(), dtype=float)
dof = list(robot.dof_names)
for jname, deg in HOME_POSE_DEG:
    q0[dof.index(jname)] = np.radians(deg)
robot.set_joints_default_state(positions=q0)
world.reset()
for _ in range(15):
    world.step(render=False)

base_idx = np.array([dof.index(n) for n in BASE_JOINTS])
lift_idx = np.array([dof.index("shoulder_lift_joint")])       # 카메라 상하 스윕용
lift_home = np.radians(dict(HOME_POSE_DEG)["shoulder_lift_joint"])

# 베이스 조인트(x,y)는 스폰 원점(POSE) 기준 오프셋 → 실측해 매핑을 잡는다(월드 목표에서 뺀다).
robot.set_joint_positions(np.zeros(3), joint_indices=base_idx)
for _ in range(2):
    world.step(render=False)
_bt = UsdGeom.XformCache().GetLocalToWorldTransform(
    stage.GetPrimAtPath(mm.chassis_path)).ExtractTranslation()
base_origin = np.array([_bt[0], _bt[1]])
print("[SDG] base_origin(world) =", base_origin.round(2))
_gh = cfg.greenhouse                                          # 벽 안으로만 서게 경계 클램프
X_LIM, Y_LIM = _gh.width / 2.0 - 0.5, _gh.length / 2.0 - 0.5

# ── 과실 색 재질 + semantic 라벨 ──
# 왜 재질을 새로 굽나: 씬은 색을 primvars:displayColor 로만 넣고 무광 머티리얼의 PrimvarReader
# 가 읽게 하는데(ripeness.bind_matte_material), 이 경로가 GUI 뷰포트에선 vertex color 로 보이지만
# **Replicator RGB 렌더에선 fallback 회색**으로 나온다(2회 확인). 그래서 클래스별 단색
# UsdPreviewSurface(빨강=ripe / 갈색=spoiled)를 과실 메시에 직접 바인딩해 렌더 경로와 무관하게
# 컬러가 나오게 한다. YOLO 는 색 그라데이션이 아니라 '빨강 vs 갈색'만 있으면 되므로 단색으로 충분.
def _solid_mat(path: str, color, metallic=0.0, roughness=0.7) -> UsdShade.Material:
    mat = UsdShade.Material.Define(stage, path)
    sh = UsdShade.Shader.Define(stage, path + "/S")
    sh.CreateIdAttr("UsdPreviewSurface")
    sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
    sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(roughness)
    sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(metallic)
    mat.CreateSurfaceOutput().ConnectToSource(
        sh.CreateOutput("surface", Sdf.ValueTypeNames.Token))
    return mat


fruit_mats = {"ripe": _solid_mat("/World/Looks/SolidRipe", RED),
              "spoiled": _solid_mat("/World/Looks/SolidSpoiled", BROWN)}

fruits = task.get_observations()["fruits"]            # {path, class_name, position, sector}
assert fruits, "과실이 0개 — 토마토 USD/씬 설정 확인(scene/tomato_plants.py)"
n_tagged = 0
for f in fruits:
    body = stage.GetPrimAtPath(f["path"] + "/Body")   # 콜라이더가 붙은 메시(tomato_plants.py)
    if not body.IsValid():
        continue
    add_update_semantics(body, semantic_label=f["class_name"], type_label="class")
    mat = fruit_mats.get(f["class_name"])
    if mat:
        for prim in Usd.PrimRange(body):
            if prim.IsA(UsdGeom.Mesh):
                mb = UsdShade.MaterialBindingAPI.Apply(prim)
                mb.UnbindAllBindings()                # 구워진 기본 재질 제거(무광재질 덮음 방지)
                mb.Bind(mat)                          # 클래스 단색 직접 바인딩
    n_tagged += 1
print(f"[SDG] 과실 {len(fruits)}개 태깅+재질 {n_tagged} (ripe=빨강/spoiled=갈색)")

# ── displayColor→회색 문제는 과실만이 아니라 배경(줄기·잎·베드)·커터날 전부에 해당한다.
#    각 메시의 displayColor 를 읽어 그 색 단색재질을 바인딩(색 캐시). displayColor 가 없으면
#    fallback 색을 쓴다(커터 CAD 지그처럼 색이 안 구워진 메시용). ──
_mat_cache: dict = {}


def _solid_for(color, metallic=0.0, roughness=0.7) -> UsdShade.Material:
    key = (tuple(round(float(c), 2) for c in color), metallic, roughness)
    if key not in _mat_cache:
        _mat_cache[key] = _solid_mat(
            f"/World/Looks/Solid_{len(_mat_cache)}", key[0], metallic, roughness)
    return _mat_cache[key]


def colorize_subtree(root_path, fallback=None, metallic=0.0, roughness=0.7,
                     skip_substr=None) -> int:
    """root 하위 메시를 displayColor(없으면 fallback) 단색재질로 바인딩. 바인딩한 개수 반환."""
    root = stage.GetPrimAtPath(root_path)
    if not root.IsValid():
        return 0
    cnt = 0
    for prim in Usd.PrimRange(root):
        if not prim.IsA(UsdGeom.Mesh):
            continue
        if skip_substr and skip_substr in str(prim.GetPath()):
            continue
        dc = UsdGeom.PrimvarsAPI(prim).GetPrimvar("displayColor")
        vals = dc.Get() if (dc and dc.HasValue()) else None
        if vals:
            n = len(vals)                             # 평균색(대부분 단색이라 그대로)
            col = (sum(v[0] for v in vals) / n, sum(v[1] for v in vals) / n,
                   sum(v[2] for v in vals) / n)
        elif fallback is not None:
            col = fallback
        else:
            continue
        mb = UsdShade.MaterialBindingAPI.Apply(prim)
        mb.UnbindAllBindings()
        mb.Bind(_solid_for(col, metallic, roughness))
        cnt += 1
    return cnt


# 배경(줄기/잎/베드/트렐리스) — displayColor 그대로. 과실 몸통(/Body)은 위에서 처리했으니 skip.
n_bg = colorize_subtree("/World/Plants", skip_substr="/Body")
print(f"[SDG] 배경 메시 {n_bg}개 단색 재질 바인딩 (줄기/잎/베드/트렐리스)")

# 커터날/CAD 지그는 소스(robots/harvester._bind_metal)에서 금속 회색으로 못박으므로 여기선 skip.

# ── 베드별 카메라 스테이션. 카메라는 **한 섹터의 두 베드 사이 통로**에 서서 각 베드를 향한다:
#    베드에서 열 중심(ccx = 두 베드 사이)으로 STANDOFF 만큼 물러서 바깥의 그 베드를 본다.
#    → 바깥 베드는 바깥(∓X), 안쪽 베드는 안쪽(중앙통로 쪽, ±X)을 향한다. 로봇이 이랑 사이
#    통로 하나에 서서 양쪽 베드를 다 찍는 셈. 설정에서 베드 x·세그먼트 y 를 그대로 유도.
pc = cfg.plants
_colw = (pc.rows_per_col - 1) * pc.row_spacing
_pitch_x = _colw + pc.aisle_x
_span_x = (pc.sector_cols - 1) * _pitch_x
_seglen = (pc.plants_per_seg - 1) * pc.plant_spacing
_pitch_y = _seglen + pc.aisle_y
_span_y = (pc.sector_rows - 1) * _pitch_y
seg_centers = [-_span_y / 2 + sr * _pitch_y for sr in range(pc.sector_rows)]
_seg_half = _seglen / 2.0

stations = []
n_beds = 0
for sc in range(pc.sector_cols):
    ccx = -_span_x / 2 + sc * _pitch_x                # 이 섹터의 두 베드 사이 중앙
    for ri in range(pc.rows_per_col):
        bx = ccx - _colw / 2 + ri * pc.row_spacing    # 베드(이랑) x
        n_beds += 1
        to_gap = 1.0 if ccx >= bx else -1.0           # 베드→두 베드 사이 통로 쪽으로 물러선다
        cam_x = bx + to_gap * STANDOFF_X
        dx = bx - cam_x                               # 카메라→베드 방향
        for sy in seg_centers:
            y_top, y_bot = sy + _seg_half, sy - _seg_half
            step = (y_top - y_bot) / (Y_STOPS - 1) if Y_STOPS > 1 else 0.0
            for i in range(Y_STOPS):
                ty = y_top - i * step
                # 조준 = 세그먼트 중심(bx, sy). 중앙 y 는 수직, 끝으로 갈수록 베드 안쪽(sy)으로 토우인.
                az = math.atan2(sy - ty, dx)
                stations.append((cam_x, az, ty))
TOTAL = len(stations) * SHOTS_PER_STOP
print(f"[SDG] 베드 {n_beds}개 × 세그먼트 {len(seg_centers)}개 × y정지 {Y_STOPS} "
      f"= 스테이션 {len(stations)}개, 각 {SHOTS_PER_STOP}장 → 총 {TOTAL}프레임"
      + (f" (상한 {FRAME_CAP})" if FRAME_CAP else ""))

# ── D455 렌더프로덕트 + BasicWriter (rgb + 2D bbox) ──
cam_prim = mm.camera_path(stage)
assert cam_prim, "D455 컬러 카메라 prim 을 못 찾음 — harvester._add_camera_at 확인"


def _cam_forward_az() -> float:
    """D455 전방(-Z) 방위각[rad]. 홈자세·yaw=0 에서 재두면 이랑을 향하는 yaw 를 역산할 수 있다."""
    m = UsdGeom.XformCache().GetLocalToWorldTransform(stage.GetPrimAtPath(cam_prim))
    z = m.ExtractRotationMatrix().GetRow(2)           # 카메라 로컬 +Z 의 월드 표현
    return math.atan2(-z[1], -z[0])                   # 카메라는 -Z 를 본다


cam_az0 = _cam_forward_az()                           # 베이스 yaw=0·홈자세에서의 기준 방위각
print(f"[SDG] cam_az0(deg) = {math.degrees(cam_az0):.1f}")

render_product = rep.create.render_product(cam_prim, IMG_WH)

# 조명 — 씬 조명 위에 얹어 밝기/색온도를 프레임마다 흔든다(도메인 랜덤화). 워밍업 전에 켠다.
sdg_light = UsdLux.DistantLight.Define(stage, "/World/SDGLight")
sdg_light.GetIntensityAttr().Set(1200)

# 렌더 파이프라인 워밍업 — RTX 가 머티리얼(displayColor 무광재질 포함)을 컴파일하도록 **실제
# 렌더를 몇 번 돌린다**. writer 를 붙이기 전이라 파일은 안 써진다. simulation_app.update() 만으론
# render_product 의 RTX 가 안 돌아 첫 캡처가 fallback 회색(=그레이스케일)으로 나오는 걸 막는다.
for _ in range(20):
    rep.orchestrator.step(rt_subframes=8)

writer = rep.WriterRegistry.get("BasicWriter")
writer.initialize(output_dir=OUT_DIR, rgb=True, bounding_box_2d_tight=True)
writer.attach([render_product])


def place_base(cam_x: float, az: float, ty: float, jitter: bool = False) -> None:
    """베드 앞(cam_x)에 세우고 그 베드(az 방향)를 향하게 한다. 월드 목표를 base_origin 기준으로
    변환하고 온실 벽 안으로 클램프. yaw 는 실측 기준각(cam_az0)으로 역산.
    jitter=True 면 샷마다 x·y·yaw 를 미세하게 흔들어 시점을 다양화한다."""
    jx = random.uniform(-POS_JITTER, POS_JITTER) if jitter else 0.0
    jy = random.uniform(-POS_JITTER, POS_JITTER) if jitter else 0.0
    jyaw = math.radians(random.uniform(-YAW_JITTER_DEG, YAW_JITTER_DEG)) if jitter else 0.0
    tx = max(-X_LIM, min(X_LIM, cam_x + jx))
    tyc = max(-Y_LIM, min(Y_LIM, ty + jy))
    yaw = az - cam_az0 + math.radians(CAM_YAW_OFFSET_DEG) + jyaw
    robot.set_joint_positions(
        np.array([tx - base_origin[0], tyc - base_origin[1], yaw]),
        joint_indices=base_idx)


def lift_arm() -> None:
    """홈자세의 shoulder_lift 를 ±ARM_LIFT_DEG 로 흔들어 D455 를 상하로 훑는다."""
    d = np.radians(random.uniform(-ARM_LIFT_DEG, ARM_LIFT_DEG))
    robot.set_joint_positions(np.array([lift_home + d]), joint_indices=lift_idx)


def randomize_light() -> None:
    sdg_light.GetIntensityAttr().Set(random.uniform(400, 2500))
    c = random.uniform(0.9, 1.0)
    sdg_light.GetColorAttr().Set(Gf.Vec3f(1.0, c, c * random.uniform(0.92, 1.0)))


def diag(frame: int) -> None:
    """보정용 — D455 월드 위치·방위각을 찍어 STANDOFF/조준/좌표계가 맞는지 눈으로 확인."""
    m = UsdGeom.XformCache().GetLocalToWorldTransform(stage.GetPrimAtPath(cam_prim))
    t = m.ExtractTranslation()
    az = math.degrees(_cam_forward_az())
    print(f"[DIAG f{frame}] D455 world=({t[0]:.2f},{t[1]:.2f},{t[2]:.2f}) az={az:.0f}°")


# ── 메인 루프: 스테이션마다 SHOTS_PER_STOP 장 (샷마다 위치·조준·팔z·조명 지터로 다른 시점) ──
_cap = min(FRAME_CAP, TOTAL) if FRAME_CAP is not None else TOTAL
frame = 0
for cam_x, az, ty in stations:
    if frame >= _cap:
        break
    for _shot in range(SHOTS_PER_STOP):
        if frame >= _cap:
            break
        place_base(cam_x, az, ty, jitter=True)        # 샷마다 x·y·yaw 미세 지터
        lift_arm()                                    # 각 장마다 팔 z 높이 변주
        randomize_light()
        for _ in range(2):                            # 키네마틱 텔레포트를 물리에 반영
            world.step(render=False)
        rep.orchestrator.step(rt_subframes=4)         # 렌더 + BasicWriter 저장
        if frame < DIAG_FRAMES:
            diag(frame)
        if (frame + 1) % 50 == 0:
            print("[SDG] %d/%d 프레임" % (frame + 1, _cap))
        frame += 1

rep.orchestrator.wait_until_complete()
print("[SDG] 원본 데이터셋 ->", OUT_DIR)

# ── YOLO 포맷 변환 (지훈님 vision/yolo_dataset.py 재사용, Isaac 의존 없음) ──
sys.path.insert(0, os.path.join(_ROOT, "basic", "jihoonkim", "isaacpjt"))
try:
    from vision.yolo_dataset import convert
    convert(OUT_DIR, YOLO_DIR)
    print("[SDG] YOLO 데이터셋 ->", YOLO_DIR)
except Exception as e:
    print(f"[SDG] 변환 스킵({e}) — 수동: python -m vision.yolo_dataset 로 {OUT_DIR} 변환")

simulation_app.close()
