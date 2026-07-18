# -*- coding: utf-8 -*-
"""FreeCAD 에서 뽑은 토마토 OBJ 들을 USD 로 일괄 변환 (Isaac Sim 5.1).

실행: isaac_python 00_convert_obj_to_usd.py
  (alias isaac_python="~/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release/python.sh")
결과: OUT_DIR 에 tomato_*.usd 생성 → 이후 스크립트들이 이걸 참조(인스턴싱).
"""
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})

import os
import asyncio
import omni.kit.asset_converter as converter

BASE = os.path.dirname(os.path.abspath(__file__))  # 이 스크립트 폴더(isaacpjt) 기준
IN_DIR = os.path.join(BASE, "tomato_assets")       # FreeCAD 출력 (종류별 하위폴더 포함)
OUT_DIR = os.path.join(BASE, "tomato_assets_usd")  # USD 출력 (한 폴더에 평탄화)


async def _convert(src, dst):
    task = converter.get_instance().create_converter_task(src, dst, None)
    ok = await task.wait_until_finished()
    if not ok:
        print("[FAIL]", src, "-", task.get_error_message())
    return ok


def main():
    if not os.path.isdir(OUT_DIR):
        os.makedirs(OUT_DIR)
    loop = asyncio.get_event_loop()
    n = 0
    for root, _dirs, files in os.walk(IN_DIR):
        for f in files:
            if f.lower().endswith(".obj"):
                src = os.path.join(root, f)
                dst = os.path.join(OUT_DIR, f[:-4] + ".usd")
                loop.run_until_complete(_convert(src, dst))
                n += 1
                print("[OK]", dst)
    print("\n변환 완료: %d 개 -> %s" % (n, OUT_DIR))


main()
simulation_app.close()
