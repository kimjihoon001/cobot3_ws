# -*- coding: utf-8 -*-
"""운반 AMR (iw.hub, 언더라이드) — 팔레트+KLT 세트를 싣고 MM↔창고를 오간다.

물류 루프(2026-07-19 확정): MM 은 iw.hub 위 KLT 에 과실을 넣기만 하고, iw.hub 가
팔레트째 나르며, 창고에서 지게차와 표준 팔레트 교환을 한다 — MM→운반 크레이트
이관(근거 없던 갭)이 아예 없다.

에셋 실측 (2026-07-19 Nucleus, tools/iwhub_bridge_check.py 으로 스폰·ROS2 구동 검증):
  1431×659×231mm, 페이로드 1000kg — 폭 0.66m < 이랑 1.5m → 통로 주행 OK.
  DOF: left/right_wheel_joint(차동 구동, 속도), lift_joint(승강, 위치).

이 모듈은 **놓기만 한다** (harvester/transporter 와 같은 규칙). 제어는 ROS2 가
/{ns}/joint_command(JointState) 로 직접 한다 — ros/robot_bridge.py 참조(§5.6).
"""
from __future__ import annotations

from pxr import Usd, UsdGeom

from pjt_config.settings import RobotConfig
from pjt_utils.xform import set_translate
from robots import assets


class IwHub:
    """iw.hub 운반 AMR. 구조: {root} <- iw_hub.usd 참조 (아티큘레이션 루트 = root)."""

    # 실측 DOF 이름 (2026-07-19) — ROS2 JointState 의 name 필드에 이대로 쓴다.
    DRIVE_JOINTS = ("left_wheel_joint", "right_wheel_joint")   # 속도 명령(차동)
    LIFT_JOINT = "lift_joint"                                   # 위치 명령(승강)

    def __init__(self, cfg: RobotConfig):
        self._cfg = cfg
        self._root: str | None = None

    @property
    def root(self) -> str | None:
        return self._root

    def spawn(self, stage: Usd.Stage, root: str = "/World/IwHub",
              position: tuple[float, float, float] = (0.0, 0.0, 0.0),
              log=print) -> str:
        """놓는다. 반환: root 경로."""
        from isaacsim.core.utils.stage import add_reference_to_stage

        url = assets.resolve(self._cfg.assets.iwhub, "운반 AMR(iw.hub)")
        log(f"[IwHub] 에셋 {url}")
        add_reference_to_stage(url, root)
        # 참조 prim 은 자체 xformOp 을 가질 수 있다 → 기존 op 재사용(§8)
        set_translate(stage.GetPrimAtPath(root), position)
        self._root = root
        log(f"[IwHub] 배치 완료: {root} @ {tuple(round(v, 2) for v in position)}")
        return root
