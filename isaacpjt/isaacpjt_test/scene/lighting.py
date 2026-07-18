# -*- coding: utf-8 -*-
"""조명 스폰 — 돔라이트(전역) + 디스턴트라이트(태양, 그림자)."""
from pxr import Usd, UsdGeom, UsdLux, Gf

from pjt_config.settings import LightingConfig


class Lighting:
    def __init__(self, cfg: LightingConfig):
        self._cfg = cfg

    def spawn(self, stage: Usd.Stage, root: str = "/World/Lights") -> None:
        dome = UsdLux.DomeLight.Define(stage, root + "/Dome")
        dome.CreateIntensityAttr(self._cfg.dome_intensity)

        sun = UsdLux.DistantLight.Define(stage, root + "/Sun")
        sun.CreateIntensityAttr(self._cfg.sun_intensity)
        sun.CreateAngleAttr(0.53)
        xf = UsdGeom.Xformable(sun.GetPrim())
        xf.AddRotateXYZOp().Set(Gf.Vec3f(*self._cfg.sun_rotation_xyz))
