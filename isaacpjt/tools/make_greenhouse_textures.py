# -*- coding: utf-8 -*-
"""온실 벽 패널 텍스처를 절차적으로 생성한다 (외부 에셋 없음, 1회 실행).

왜 절차 생성인가 — 온실 벽에 쓸 만한 CC0 텍스처를 외부에서 받아오면 출처·라이선스를
또 관리해야 한다. 폴리카보네이트 중공판은 규칙적인 패턴이라 코드로 만드는 편이
깔끔하고, 타일 피치(=실제 몇 m 인지)를 우리가 정확히 통제할 수 있다.

만드는 것 — 1타일 = 실제 1m x 1m 기준:
  greenhouse_panel.png  벽 패널. 알루미늄 프레임 테두리 + 중공판 세로 리브 +
                        결로/때 얼룩. 타일링하면 프레임이 이어져 멀리언 격자가 된다.

실행:
  cd ~/cobot3_ws/isaacpjt && python3 tools/make_greenhouse_textures.py
  (Isaac 불필요 — numpy + Pillow 만 쓴다)
"""
import os

import numpy as np
from PIL import Image

OUT_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "assets", "greenhouse")

SIZE = 512                 # 타일 해상도 (1m 당 512px = 2mm/px)
FRAME_PX = 9               # 알루미늄 프레임 폭 (약 18mm — 실제 온실 멀리언 규격대)
RIB_PERIOD_PX = 32         # 중공판 세로 리브 간격 (약 62mm)


def _panel(rng: np.random.Generator) -> np.ndarray:
    y, x = np.mgrid[0:SIZE, 0:SIZE].astype(np.float32)

    # 바탕 — 유리/폴리카보네이트의 창백한 청백색. 위로 갈수록 하늘빛이 조금 더 든다.
    base = np.stack([
        np.full((SIZE, SIZE), 0.74, np.float32),
        np.full((SIZE, SIZE), 0.80, np.float32),
        np.full((SIZE, SIZE), 0.84, np.float32),
    ], axis=-1)
    base += (1.0 - y / SIZE)[..., None] * np.array([0.04, 0.05, 0.07], np.float32)

    # 중공판 세로 리브 — 판 안쪽 격벽이 비쳐 보이는 것. 얇고 규칙적인 명암.
    rib = np.cos(2.0 * np.pi * x / RIB_PERIOD_PX)
    base += (rib * 0.022)[..., None]
    # 리브 바로 옆의 가는 하이라이트 (빛이 격벽 모서리에 걸림)
    edge = np.exp(-((x % RIB_PERIOD_PX) - 1.0) ** 2 / 2.0)
    base += (edge * 0.05)[..., None]

    # 결로/때 — 위에서 아래로 흘러내린 세로 얼룩. 저주파 랜덤을 세로로 늘여 만든다.
    streak = rng.random(SIZE, dtype=np.float32)
    streak = np.convolve(streak, np.ones(11, np.float32) / 11.0, mode="same")
    streak = (streak - streak.mean()) * 0.10
    fade = (y / SIZE) ** 1.5          # 아래쪽일수록 때가 짙다
    base += (streak[None, :] * fade)[..., None]

    # 미세 노이즈 — 완전히 매끈하면 CG 티가 난다.
    base += (rng.random((SIZE, SIZE), dtype=np.float32) - 0.5)[..., None] * 0.012

    # 알루미늄 프레임 — 타일 테두리. 이어 붙으면 격자 멀리언이 된다.
    m = ((x < FRAME_PX) | (x >= SIZE - FRAME_PX) |
         (y < FRAME_PX) | (y >= SIZE - FRAME_PX))
    alu = np.array([0.62, 0.64, 0.66], np.float32)
    # 프레임 안쪽 모서리에 어두운 그림자 한 줄 → 두께감
    shadow = (((x >= FRAME_PX) & (x < FRAME_PX + 2)) |
              ((x >= SIZE - FRAME_PX - 2) & (x < SIZE - FRAME_PX)) |
              ((y >= FRAME_PX) & (y < FRAME_PX + 2)) |
              ((y >= SIZE - FRAME_PX - 2) & (y < SIZE - FRAME_PX)))
    base[m] = alu
    base[m] += (rng.random((m.sum(), 1), dtype=np.float32) - 0.5) * 0.03
    base[shadow] *= 0.80
    return np.clip(base, 0.0, 1.0)


def main() -> None:
    os.makedirs(OUT_DIR, exist_ok=True)
    rng = np.random.default_rng(20260727)      # 고정 시드 — 매번 같은 그림이 나온다
    path = os.path.join(OUT_DIR, "greenhouse_panel.png")
    Image.fromarray((_panel(rng) * 255).astype(np.uint8), "RGB").save(path)
    print(f"생성: {path}  ({SIZE}x{SIZE}, 1타일 = 실제 1m)")


if __name__ == "__main__":
    main()
