# -*- coding: utf-8 -*-
"""YOLO 변환 검증 — 가짜 Replicator 출력을 만들어 돌린다. GPU 없이 된다.

라벨 좌표가 틀리면 YOLO 는 조용히 학습되고 mAP 만 낮게 나온다. 원인을 찾기
어려우므로 좌표 변환을 여기서 못박는다.
"""
import json
import pathlib
import struct
import zlib

import numpy as np
import pytest

from vision.yolo_dataset import CLASSES, Stats, convert, convert_frame

BBOX_DTYPE = np.dtype([("semanticId", "<u4"), ("x_min", "<i4"), ("y_min", "<i4"),
                       ("x_max", "<i4"), ("y_max", "<i4"),
                       ("occlusionRatio", "<f4")])


def write_png(path: pathlib.Path, w: int, h: int) -> None:
    """실제 PNG 헤더가 필요하다 (_image_size 가 읽는다)."""
    def chunk(tag, data):
        c = tag + data
        return struct.pack(">I", len(data)) + c + struct.pack(">I", zlib.crc32(c))

    ihdr = struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0)
    raw = b"".join(b"\x00" + b"\x00" * (w * 3) for _ in range(h))
    path.write_bytes(b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", ihdr)
                     + chunk(b"IDAT", zlib.compress(raw))
                     + chunk(b"IEND", b""))


def write_frame(d: pathlib.Path, idx: str, boxes, classes, size=(640, 640)):
    write_png(d / f"rgb_{idx}.png", *size)
    np.save(str(d / f"bounding_box_2d_tight_{idx}.npy"),
            np.array(boxes, dtype=BBOX_DTYPE))
    (d / f"bounding_box_2d_tight_labels_{idx}.json").write_text(
        json.dumps({str(k): {"class": v} for k, v in classes.items()}))


@pytest.fixture
def src(tmp_path):
    d = tmp_path / "raw"
    d.mkdir()
    return d


# ---------------------------------------------------------------
# 좌표 변환
# ---------------------------------------------------------------
def test_박스가_YOLO_정규화_좌표로_바뀐다(src):
    # 640x640 에서 (100,200)~(300,400) -> cx=200/640, cy=300/640, w=h=200/640
    write_frame(src, "0000", [(0, 100, 200, 300, 400, 0.0)], {0: "fully_ripe"})
    stats = Stats()
    lines = convert_frame(src / "bounding_box_2d_tight_0000.npy",
                          src / "bounding_box_2d_tight_labels_0000.json",
                          640, 640, stats)

    assert len(lines) == 1
    cid, cx, cy, w, h = lines[0].split()
    assert int(cid) == CLASSES.index("fully_ripe")
    assert float(cx) == pytest.approx(200 / 640)
    assert float(cy) == pytest.approx(300 / 640)
    assert float(w) == pytest.approx(200 / 640)
    assert float(h) == pytest.approx(200 / 640)


def test_클래스_id가_CLASSES_순서를_따른다(src):
    write_frame(src, "0000",
                [(0, 10, 10, 100, 100, 0.0), (1, 200, 200, 300, 300, 0.0),
                 (2, 400, 400, 500, 500, 0.0), (3, 500, 500, 600, 600, 0.0)],
                {0: "green", 1: "half_ripe", 2: "fully_ripe", 3: "old"})
    lines = convert_frame(src / "bounding_box_2d_tight_0000.npy",
                          src / "bounding_box_2d_tight_labels_0000.json",
                          640, 640, Stats())
    assert [int(l.split()[0]) for l in lines] == [0, 1, 2, 3]


def test_화면_밖으로_나간_박스는_잘린다(src):
    """Replicator 가 음수/초과 좌표를 준다. 그대로 두면 YOLO 가 거부한다."""
    write_frame(src, "0000", [(0, -50, -50, 100, 100, 0.0)], {0: "green"})
    lines = convert_frame(src / "bounding_box_2d_tight_0000.npy",
                          src / "bounding_box_2d_tight_labels_0000.json",
                          640, 640, Stats())
    _, cx, cy, w, h = lines[0].split()
    # 잘리면 (0,0)~(100,100)
    assert float(w) == pytest.approx(100 / 640)
    assert float(cx) == pytest.approx(50 / 640)
    for v in (cx, cy, w, h):
        assert 0.0 <= float(v) <= 1.0


def test_모든_값이_0에서_1_사이다(src):
    write_frame(src, "0000",
                [(0, 0, 0, 640, 640, 0.0), (1, 639, 639, 640, 640, 0.0)],
                {0: "green", 1: "old"})
    for line in convert_frame(src / "bounding_box_2d_tight_0000.npy",
                              src / "bounding_box_2d_tight_labels_0000.json",
                              640, 640, Stats()):
        for v in line.split()[1:]:
            assert 0.0 <= float(v) <= 1.0


# ---------------------------------------------------------------
# 필터
# ---------------------------------------------------------------
def test_많이_가려진_박스는_제외된다(src):
    write_frame(src, "0000",
                [(0, 10, 10, 200, 200, 0.95), (1, 300, 300, 500, 500, 0.1)],
                {0: "green", 1: "fully_ripe"})
    stats = Stats()
    lines = convert_frame(src / "bounding_box_2d_tight_0000.npy",
                          src / "bounding_box_2d_tight_labels_0000.json",
                          640, 640, stats)
    assert len(lines) == 1
    assert stats.dropped_occluded == 1


def test_너무_작은_박스는_제외된다(src):
    write_frame(src, "0000", [(0, 100, 100, 101, 101, 0.0)], {0: "green"})
    stats = Stats()
    assert convert_frame(src / "bounding_box_2d_tight_0000.npy",
                         src / "bounding_box_2d_tight_labels_0000.json",
                         640, 640, stats) == []
    assert stats.dropped_tiny == 1


def test_모르는_클래스는_제외된다(src):
    """씬에 다른 semantic 이 섞여도 학습셋을 오염시키면 안 된다."""
    write_frame(src, "0000",
                [(0, 10, 10, 200, 200, 0.0), (1, 300, 300, 500, 500, 0.0)],
                {0: "지면", 1: "fully_ripe"})
    stats = Stats()
    lines = convert_frame(src / "bounding_box_2d_tight_0000.npy",
                          src / "bounding_box_2d_tight_labels_0000.json",
                          640, 640, stats)
    assert len(lines) == 1
    assert stats.dropped_unknown_class == 1


def test_occlusionRatio가_없어도_된다(src):
    """이 필드가 없는 Replicator 버전도 있다."""
    dt = np.dtype([("semanticId", "<u4"), ("x_min", "<i4"), ("y_min", "<i4"),
                   ("x_max", "<i4"), ("y_max", "<i4")])
    np.save(str(src / "bounding_box_2d_tight_0000.npy"),
            np.array([(0, 10, 10, 200, 200)], dtype=dt))
    (src / "bounding_box_2d_tight_labels_0000.json").write_text(
        json.dumps({"0": {"class": "green"}}))
    assert len(convert_frame(src / "bounding_box_2d_tight_0000.npy",
                             src / "bounding_box_2d_tight_labels_0000.json",
                             640, 640, Stats())) == 1


# ---------------------------------------------------------------
# 전체 변환
# ---------------------------------------------------------------
def test_train_val_분할과_data_yaml이_만들어진다(src, tmp_path):
    for i in range(10):
        write_frame(src, f"{i:04d}", [(0, 100, 100, 200, 200, 0.0)],
                    {0: "fully_ripe"})

    dst = tmp_path / "yolo"
    stats = convert(src, dst, val_ratio=0.2, log=lambda *_: None)

    assert stats.frames == 10 and stats.boxes == 10
    assert len(list((dst / "images/val").glob("*.png"))) == 2
    assert len(list((dst / "images/train").glob("*.png"))) == 8
    # 이미지마다 라벨이 하나씩 짝을 이뤄야 한다
    for split in ("train", "val"):
        imgs = {p.stem for p in (dst / "images" / split).glob("*.png")}
        lbls = {p.stem for p in (dst / "labels" / split).glob("*.txt")}
        assert imgs == lbls

    yaml = (dst / "data.yaml").read_text()
    assert "nc: 4" in yaml
    assert "fully_ripe" in yaml


def test_분할이_시드로_재현된다(src, tmp_path):
    """재현성 — 같은 시드면 같은 val 셋."""
    for i in range(10):
        write_frame(src, f"{i:04d}", [(0, 100, 100, 200, 200, 0.0)],
                    {0: "green"})

    def val_set(out):
        convert(src, out, seed=42, log=lambda *_: None)
        return {p.name for p in (out / "images/val").glob("*.png")}

    assert val_set(tmp_path / "a") == val_set(tmp_path / "b")


def test_배경만_있는_프레임은_빈_라벨로_남는다(src, tmp_path):
    """YOLO 는 빈 .txt 를 배경 샘플로 쓴다 — false positive 억제."""
    write_frame(src, "0000", [], {})
    dst = tmp_path / "yolo"
    stats = convert(src, dst, val_ratio=0.0, log=lambda *_: None)

    assert stats.empty_frames == 1
    assert (dst / "labels/train/rgb_0000.txt").read_text() == ""


def test_라벨이_없는_프레임은_건너뛴다(src, tmp_path):
    write_png(src / "rgb_0000.png", 640, 640)      # npy/json 없음
    write_frame(src, "0001", [(0, 100, 100, 200, 200, 0.0)], {0: "green"})

    stats = convert(src, tmp_path / "yolo", val_ratio=0.0, log=lambda *_: None)
    assert stats.frames == 1


def test_빈_폴더는_바로_알려준다(tmp_path):
    (tmp_path / "empty").mkdir()
    with pytest.raises(FileNotFoundError, match="02_generate_dataset"):
        convert(tmp_path / "empty", tmp_path / "out", log=lambda *_: None)
