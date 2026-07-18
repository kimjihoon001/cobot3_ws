# -*- coding: utf-8 -*-
"""Replicator BasicWriter 출력 -> YOLO 학습 포맷 변환.

  tomato_dataset/                        yolo/
    rgb_0000.png                           images/train/rgb_0000.png
    bounding_box_2d_tight_0000.npy    ->   labels/train/rgb_0000.txt
    bounding_box_2d_tight_labels_0000.json data.yaml

BasicWriter 의 bbox .npy 는 구조화 배열이고 필드가 대략 이렇다:
  semanticId, x_min, y_min, x_max, y_max, occlusionRatio
클래스 이름은 같은 인덱스의 _labels_*.json 이 semanticId -> {"class": 이름} 으로 준다.

YOLO 라벨은 한 줄에 `class_id cx cy w h` (전부 0~1 정규화).

**Isaac 을 import 하지 않는다.** 데이터 생성은 GPU 에서 하지만 변환은 순수
파이썬이라 dev 머신에서 돌고 tests/test_yolo_dataset.py 로 검증된다.
"""
from __future__ import annotations

import json
import pathlib
import random
import shutil
from dataclasses import dataclass

import numpy as np

# 클래스 순서 = YOLO class_id. 02_generate_dataset.py / settings.py 와 일치해야 한다.
CLASSES = ("green", "half_ripe", "fully_ripe", "old")

# 너무 가려진 개체는 학습에 해가 된다. Replicator 가 주는 occlusionRatio 로 거른다.
MAX_OCCLUSION = 0.8
# 너무 작은 박스도 뺀다 (정규화 기준 변 길이)
MIN_SIDE = 0.005


@dataclass
class Stats:
    frames: int = 0
    boxes: int = 0
    dropped_occluded: int = 0
    dropped_tiny: int = 0
    dropped_unknown_class: int = 0
    empty_frames: int = 0

    def summary(self) -> str:
        return (f"프레임 {self.frames} / 박스 {self.boxes}\n"
                f"  제외: 가려짐 {self.dropped_occluded} / "
                f"너무작음 {self.dropped_tiny} / "
                f"모르는클래스 {self.dropped_unknown_class}\n"
                f"  빈 프레임 {self.empty_frames}")


def _class_map(labels_json: pathlib.Path) -> dict[int, str]:
    """semanticId -> 클래스 이름. Replicator 는 키를 문자열로 쓴다."""
    raw = json.loads(labels_json.read_text())
    out = {}
    for sid, v in raw.items():
        name = v.get("class") if isinstance(v, dict) else v
        if name is not None:
            out[int(sid)] = name
    return out


def _field(box, *names):
    """구조화 배열 필드 이름이 버전마다 달라서 후보를 순서대로 본다."""
    for n in names:
        if n in box.dtype.names:
            return box[n]
    raise KeyError(f"bbox 에 {names} 중 아무 필드도 없음. 실제: {box.dtype.names}")


def convert_frame(npy: pathlib.Path, labels_json: pathlib.Path,
                  img_w: int, img_h: int, stats: Stats) -> list[str]:
    """한 프레임의 bbox 를 YOLO 라벨 줄 목록으로."""
    boxes = np.load(str(npy))
    cmap = _class_map(labels_json)
    lines: list[str] = []

    for b in boxes:
        sid = int(_field(b, "semanticId"))
        name = cmap.get(sid)
        if name not in CLASSES:
            stats.dropped_unknown_class += 1
            continue

        try:
            occ = float(_field(b, "occlusionRatio"))
        except KeyError:
            occ = 0.0                       # 이 필드가 없는 버전도 있다
        if occ > MAX_OCCLUSION:
            stats.dropped_occluded += 1
            continue

        x0 = float(_field(b, "x_min", "xMin"))
        y0 = float(_field(b, "y_min", "yMin"))
        x1 = float(_field(b, "x_max", "xMax"))
        y1 = float(_field(b, "y_max", "yMax"))

        # 화면 밖으로 삐져나온 박스를 자른다 (Replicator 가 음수/초과를 준다)
        x0, x1 = max(0.0, min(x0, x1)), min(float(img_w), max(x0, x1))
        y0, y1 = max(0.0, min(y0, y1)), min(float(img_h), max(y0, y1))

        w, h = (x1 - x0) / img_w, (y1 - y0) / img_h
        if w < MIN_SIDE or h < MIN_SIDE:
            stats.dropped_tiny += 1
            continue

        cx, cy = (x0 + x1) / 2 / img_w, (y0 + y1) / 2 / img_h
        lines.append(f"{CLASSES.index(name)} "
                     f"{cx:.6f} {cy:.6f} {w:.6f} {h:.6f}")
        stats.boxes += 1

    return lines


def _image_size(png: pathlib.Path) -> tuple[int, int]:
    """PNG 헤더에서 크기만 읽는다 (PIL 의존을 피한다)."""
    data = png.read_bytes()[:24]
    if data[:8] != b"\x89PNG\r\n\x1a\n":
        raise ValueError(f"PNG 가 아님: {png}")
    return int.from_bytes(data[16:20], "big"), int.from_bytes(data[20:24], "big")


def convert(src: str | pathlib.Path, dst: str | pathlib.Path,
            val_ratio: float = 0.2, seed: int = 42,
            log=print) -> Stats:
    """BasicWriter 출력 폴더를 YOLO 데이터셋으로 변환한다.

    val_ratio : 검증셋 비율. 시드 고정이라 매번 같은 분할 = 재현성.
    """
    src, dst = pathlib.Path(src), pathlib.Path(dst)
    if not src.is_dir():
        raise FileNotFoundError(f"입력 폴더 없음: {src}")

    frames = sorted(src.glob("rgb_*.png"))
    if not frames:
        raise FileNotFoundError(
            f"{src} 에 rgb_*.png 가 없다. 02_generate_dataset.py 를 먼저 돌릴 것.")

    rng = random.Random(seed)
    shuffled = list(frames)
    rng.shuffle(shuffled)
    n_val = int(len(shuffled) * val_ratio)
    val = set(shuffled[:n_val])

    for split in ("train", "val"):
        (dst / "images" / split).mkdir(parents=True, exist_ok=True)
        (dst / "labels" / split).mkdir(parents=True, exist_ok=True)

    stats = Stats()
    for png in frames:
        idx = png.stem.split("_")[-1]
        npy = src / f"bounding_box_2d_tight_{idx}.npy"
        labels = src / f"bounding_box_2d_tight_labels_{idx}.json"
        if not npy.exists() or not labels.exists():
            log(f"[YOLO] {png.name} 의 라벨이 없어 건너뜀")
            continue

        w, h = _image_size(png)
        lines = convert_frame(npy, labels, w, h, stats)
        if not lines:
            # 배경만 있는 이미지도 학습에 쓴다 (false positive 억제).
            # YOLO 는 빈 .txt 를 그렇게 해석한다.
            stats.empty_frames += 1

        split = "val" if png in val else "train"
        shutil.copy2(png, dst / "images" / split / png.name)
        (dst / "labels" / split / f"{png.stem}.txt").write_text(
            "\n".join(lines) + ("\n" if lines else ""))
        stats.frames += 1

    (dst / "data.yaml").write_text(
        f"path: {dst.resolve()}\n"
        f"train: images/train\n"
        f"val: images/val\n"
        f"nc: {len(CLASSES)}\n"
        f"names: {list(CLASSES)}\n")

    log("[YOLO] " + stats.summary())
    log(f"[YOLO] 출력: {dst}")
    return stats
