# -*- coding: utf-8 -*-
"""운반 AMR (지게차형) — 트레이를 창고 랙에 올린다.

★ GPU 에서 한 번도 안 돌려봤다. 먼저 spikes/03_asset_check.py 로 확인할 것. ★

수확 MM 보다 단순하다 — Isaac 의 ForkliftB/C 가 포크 승강을 이미 갖고 있어서
조립할 게 없다. v3 11.1 이 팀원 C 에게 Day1~2 로 배정한 "운반 AMR 모델링" 이
에셋이 실제로 있으면 통째로 사라진다.

이 로봇이 토마토를 직접 안 만진다는 게 설계의 핵심이다. 트레이만 다루므로
파지 난이도가 낮다 — 무른 과실을 다루는 건 수확 MM 혼자다.
(v3 6.1: "파지 난이도 낮음(트레이 표준화)")

이 모듈은 **놓기와 포크 승강만** 한다. 주행(Nav2)·배차(fleet)는 별도다.
"""
from __future__ import annotations

from pxr import Gf, Usd, UsdGeom, UsdPhysics

from pjt_config.settings import RobotConfig, WarehouseConfig
from robots import assets


class TransporterAMR:
    """지게차형 운반 AMR.

    구조:
        {root}/Body   <- ForkliftB 에셋 (포크 승강 조인트 포함)
    """

    # 포크 승강 조인트 이름 후보. 에셋마다 다르다.
    _LIFT_CANDIDATES = ("lift", "lift_joint", "fork_lift", "mast", "carriage")

    def __init__(self, cfg: RobotConfig, warehouse: WarehouseConfig):
        self._cfg = cfg
        self._warehouse = warehouse
        self._root: str | None = None
        self._lift_joint: str | None = None

    @property
    def root(self) -> str | None:
        return self._root

    @property
    def lift_joint(self) -> str | None:
        """포크 승강 조인트 경로. 없으면 에셋에 승강 기구가 없다는 뜻."""
        return self._lift_joint

    def spawn(self, stage: Usd.Stage, root: str = "/World/Transporter",
              position: tuple[float, float, float] = (0.0, 0.0, 0.0),
              log=print, asset_candidates: tuple[str, ...] | None = None) -> str:
        from isaacsim.core.utils.stage import add_reference_to_stage

        self._root = root
        UsdGeom.Xform.Define(stage, root)
        UsdGeom.Xformable(stage.GetPrimAtPath(root)).AddTranslateOp().Set(
            Gf.Vec3d(*position))

        # 기본은 설정의 forklift 후보(B 우선). 다른 지게차를 놓으려면 넘긴다(예: C).
        url = assets.resolve(asset_candidates or self._cfg.assets.forklift,
                             "운반 AMR(지게차)")
        log(f"[Transporter] 지게차 {url}")
        body = f"{root}/Body"
        add_reference_to_stage(url, body)

        self._lift_joint = self._find_lift_joint(stage, body)
        if self._lift_joint:
            log(f"[Transporter] 포크 승강 조인트: {self._lift_joint}")
            self._check_lift_range(stage, log)
        else:
            log("[Transporter] ⚠ 포크 승강 조인트를 못 찾음. 에셋의 실제 조인트 "
                "이름을 확인해 _LIFT_CANDIDATES 에 추가할 것. "
                "승강이 없으면 창고 2단에 못 올린다 = 지게차를 고른 이유가 무너진다.")
        return root

    # ----- 내부 -----

    def _find_lift_joint(self, stage: Usd.Stage, body: str) -> str | None:
        prim = stage.GetPrimAtPath(body)
        if not prim.IsValid():
            return None
        for p in Usd.PrimRange(prim):
            if not p.IsA(UsdPhysics.PrismaticJoint):
                continue
            name = p.GetName().lower()
            if any(c in name for c in self._LIFT_CANDIDATES):
                return str(p.GetPath())
        # 이름으로 못 찾으면 프리즈매틱 조인트 아무거나 (승강은 보통 1축뿐)
        for p in Usd.PrimRange(prim):
            if p.IsA(UsdPhysics.PrismaticJoint):
                return str(p.GetPath())
        return None

    def _check_lift_range(self, stage: Usd.Stage, log) -> None:
        """승강 범위가 창고 2단에 닿나. 안 닿으면 창고 설계를 바꿔야 한다."""
        need = self._warehouse.level_height
        if need is None:
            log("[Transporter] 창고 level_height 미정 -> 승강 범위 판정 보류. "
                "랙 에셋이 나오면 확인할 것.")
            return

        j = UsdPhysics.PrismaticJoint(stage.GetPrimAtPath(self._lift_joint))
        upper = j.GetUpperLimitAttr().Get() if j else None
        if upper is None:
            log("[Transporter] 승강 상한을 못 읽음 -> 수동 확인 필요.")
            return

        ok = upper >= need
        log(f"[Transporter] 승강 상한 {upper:.2f}m vs 창고 2단 {need:.2f}m "
            f"-> {'OK' if ok else '★부족 — 창고 단 높이를 낮추거나 다른 지게차★'}")
