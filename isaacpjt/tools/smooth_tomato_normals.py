# -*- coding: utf-8 -*-
"""토마토 USD 에셋의 노멀을 플랫 → 스무스로 굽는다 (에셋 1회 수정, in-place).

문제 — `assets/tomato/*.usd` 는 삼각형 3.5k 개로 실루엣은 충분한데(면당 약 3.2도)
노멀이 **면마다 따로**다. 같은 정점을 공유하는 faceVarying 노멀이 최대 117도까지
벌어져 있어(2026-07-27 실측) RTX 에서 각진 골프공처럼 렌더된다. 텍스처를 아무리
좋은 걸 입혀도 이 각짐은 그대로다.

해결 — 폴리곤을 늘리지 않고 정점 노멀을 면적 가중 평균으로 다시 계산한다.
지오메트리·토폴로지·UV·물리는 손대지 않는다. 콜라이더는 어차피 별도 해석적 구라
(`physics.add_sphere_collider`) 물리에는 아무 영향이 없다.

crease 각도 — 인접면 사이 각이 이 값보다 크면 평균에서 **제외**해 모서리를 살린다.
꼭지(calyx)의 꽃받침 갈래처럼 실제로 각져야 하는 곳까지 뭉개지 않기 위한 것이다.
과실 몸통은 각이 완만해서(면당 3.2도) 사실상 전부 평균된다.

⚠ **아직 실행해서 검증하지 않았다(2026-07-27).** Isaac 의 `python.sh` 는
SimulationApp 을 띄우기 전에는 `pxr` 을 import 하지 못한다(실측: ModuleNotFoundError).
쓰려면 `tools/import_m0617_urdf.py` 처럼 SimulationApp 부트스트랩을 앞에 붙이거나,
pxr 이 있는 다른 파이썬으로 돌려야 한다. 그 경우 크레이트를 쓰는 USD 버전이
Isaac 과 달라 못 읽는 파일이 나올 수 있으니(m0617 에서 겪은 적 있음) 저장 후
Isaac 으로 열어 확인할 것.

용도 — 고화질 과실(assets/tomato_hq)은 이미 스무스 노멀이라 해당 없고, 수확 반경
바깥의 저폴리 과실 470개가 여전히 플랫 노멀이다. 그쪽 각짐을 없애려면 이 도구를
쓴다.

실행:
  <pxr 있는 python> tools/smooth_tomato_normals.py          # 미리보기
  <pxr 있는 python> tools/smooth_tomato_normals.py --write  # 실제 덮어쓰기
"""
import argparse
import glob
import math
import os
from collections import defaultdict

from pxr import Gf, Usd, UsdGeom

ASSET_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "assets", "tomato")


def _face_normals(pts, idx, counts):
    """면별 노멀. 정규화하지 않아 크기가 곧 면적 가중치가 된다."""
    out = []
    base = 0
    for c in counts:
        a = Gf.Vec3f(pts[idx[base]])
        b = Gf.Vec3f(pts[idx[base + 1]])
        d = Gf.Vec3f(pts[idx[base + 2]])
        out.append(Gf.Cross(b - a, d - a))
        base += c
    return out


def smooth_normals(mesh: UsdGeom.Mesh, crease_deg: float = 60.0) -> float:
    """faceVarying 노멀을 정점 평균으로 다시 굽는다. 반환: 최대 편차(도, 수정 전)."""
    pts = mesh.GetPointsAttr().Get()
    idx = mesh.GetFaceVertexIndicesAttr().Get()
    counts = mesh.GetFaceVertexCountsAttr().Get()
    fn = _face_normals(pts, idx, counts)

    # 정점 → 인접 면 목록
    adj = defaultdict(list)
    base = 0
    for f, c in enumerate(counts):
        for k in range(c):
            adj[idx[base + k]].append(f)
        base += c

    cos_lim = math.cos(math.radians(crease_deg))
    unit = [n.GetNormalized() if n.GetLength() > 1e-12 else Gf.Vec3f(0, 0, 1)
            for n in fn]

    out = []
    base = 0
    for f, c in enumerate(counts):
        for k in range(c):
            v = idx[base + k]
            acc = Gf.Vec3f(0, 0, 0)
            for g in adj[v]:
                # crease 를 넘는 인접면은 빼서 모서리를 뭉개지 않는다.
                if Gf.Dot(unit[f], unit[g]) >= cos_lim:
                    acc += fn[g]            # 정규화 전 = 면적 가중
            out.append(acc.GetNormalized() if acc.GetLength() > 1e-12 else unit[f])
        base += c

    old = mesh.GetNormalsAttr().Get()
    dev = 0.0
    if old:
        for a, b in zip(old, out):
            d = max(-1.0, min(1.0, Gf.Dot(Gf.Vec3f(a).GetNormalized(), b)))
            dev = max(dev, math.degrees(math.acos(d)))
    mesh.GetNormalsAttr().Set(out)
    mesh.SetNormalsInterpolation(UsdGeom.Tokens.faceVarying)
    return dev


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--write", action="store_true", help="실제로 usd 를 덮어쓴다")
    ap.add_argument("--crease", type=float, default=60.0, help="모서리 보존 각도(도)")
    ap.add_argument("--dir", default=ASSET_DIR)
    args = ap.parse_args()

    files = sorted(glob.glob(os.path.join(args.dir, "*.usd")))
    if not files:
        raise SystemExit(f"토마토 USD 없음: {args.dir}")

    for path in files:
        stage = Usd.Stage.Open(path)
        touched = []
        for prim in stage.Traverse():
            if not prim.IsA(UsdGeom.Mesh):
                continue
            dev = smooth_normals(UsdGeom.Mesh(prim), args.crease)
            touched.append((prim.GetName(), dev))
        if args.write:
            stage.GetRootLayer().Save()
        for name, dev in touched:
            print(f"  {os.path.basename(path):30s} {name:8s} 노멀 최대 변화 {dev:5.1f}도")
    print(f"\n{len(files)}개 파일 " + ("저장 완료" if args.write else "미리보기(--write 로 저장)"))


if __name__ == "__main__":
    main()
