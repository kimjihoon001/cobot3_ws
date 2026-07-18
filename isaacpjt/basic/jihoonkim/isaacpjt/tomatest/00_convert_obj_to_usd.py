# -*- coding: utf-8 -*-
"""FreeCAD 에서 뽑은 토마토 OBJ 들을 USD 로 일괄 변환 (Isaac Sim 5.1).

실행: isaac_python 00_convert_obj_to_usd.py
  (alias isaac_python="~/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release/python.sh")
결과: OUT_DIR 에 tomato_*.usd 생성 → 이후 스크립트들이 이걸 참조(인스턴싱).
"""
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})

import os
import sys
import asyncio
import omni.kit.asset_converter as converter

# 이 스크립트는 tomatest/ 안에 있고 obj 는 그 위(isaacpjt/tomato_assets)에 있다.
# 예전엔 BASE 를 스크립트 폴더로 잡아서 없는 폴더를 뒤졌고, os.walk 는 없는
# 폴더에 조용히 0회 반복해서 "변환 완료: 0 개" 를 에러 없이 찍었다.
ISAAC_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ISAAC_DIR)

from pjt_config.settings import SceneConfig
from pjt_utils.paths import short

IN_DIR = os.path.join(ISAAC_DIR, "tomato_assets")   # FreeCAD 출력 (종류별 하위폴더)
# 출력 경로는 settings.py 를 그대로 쓴다. 여기서 따로 정하면 씬이 못 찾는다.
OUT_DIR = SceneConfig().tomato_assets.usd_dir


async def _convert(src, dst):
    task = converter.get_instance().create_converter_task(src, dst, None)
    ok = await task.wait_until_finished()
    if not ok:
        print("[FAIL]", os.path.basename(src), "-", task.get_error_message())
        return ok
    _stamp_units(dst)
    return ok


def _stamp_units(dst):
    """변환 결과에 단위 메타데이터를 바로잡는다.

    asset_converter 는 출력에 무조건 metersPerUnit=0.01(cm)·upAxis=Y 를 찍는다.
    그러면 미터 씬에서 metrics assembler 가 x0.01 + rotX90 보정을 몰래 끼워 넣어
    settings.py 의 TOMATO_SCALE 과 곱해져 토마토가 1/100 크기(0.7mm)로 사라진다
    (CLAUDE.md §8 2026-07-18). obj 수치는 원시 단위이고 변환은 TOMATO_SCALE 이
    책임지므로, 레이어는 "보정 불필요"(metersPerUnit=1, Z-up)로 선언한다.
    """
    from pxr import Usd, UsdGeom
    stage = Usd.Stage.Open(dst)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    stage.Save()


def main():
    print("입력 :", short(IN_DIR))
    print("출력 :", short(OUT_DIR))

    srcs = sorted(os.path.join(r, f)
                  for r, _d, fs in os.walk(IN_DIR)
                  for f in fs if f.lower().endswith(".obj"))
    if not srcs:
        # 조용히 0개 성공하면 뒤의 스파이크/main 이 전부 죽는다. 여기서 멈춘다.
        raise SystemExit(
            f"\n[중단] {short(IN_DIR)} 에 obj 가 하나도 없다.\n"
            f"  -> 경로가 맞는지, FreeCAD 로 생성했는지 확인할 것 "
            f"(generate_tomatoes.py).")

    os.makedirs(OUT_DIR, exist_ok=True)
    loop = asyncio.get_event_loop()
    ok_n = 0
    for src in srcs:
        dst = os.path.join(OUT_DIR, os.path.basename(src)[:-4] + ".usd")
        if loop.run_until_complete(_convert(src, dst)):
            ok_n += 1
            print("[OK]", os.path.basename(dst))

    print("\n변환: 성공 %d / 전체 %d -> %s" % (ok_n, len(srcs), short(OUT_DIR)))
    if ok_n != len(srcs):
        raise SystemExit("[중단] 일부 변환 실패 — 위 [FAIL] 확인할 것.")


main()
simulation_app.close()
