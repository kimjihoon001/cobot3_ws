# -*- coding: utf-8 -*-
"""로봇 드라이버 공통 베이스 — main.py 가 생명주기 단계에서 균일하게 호출한다.

main.py 는 씬(환경)만 만들고, 어떤 로봇을 띄울지는 플래그(--mm/--iw/--fork)로 고른다.
고른 로봇마다 Driver 하나가 아래 단계로 조립된다. 순서는 물리 초기화 제약이다(§8):

  spawn      지오메트리·조인트 배치 (world.reset 전)
  register   world.scene.add(Robot) + 아티큘레이션 루트 확인
  ── reset ──  물리 뷰 초기화
  configure  기본 관절자세 지정 (핸들이 유효해진 뒤)
  ── reset + settle ──
  finalize   ROS 브리지·부가장치(블레이드·카고·nav·카메라) — 자세가 정착한 뒤
  ── reset + settle ──
  update     매 프레임(텔레옵·JSON 명령). 재생 중에만 적용

reset 순서만 전역이라 main 의 골격에 남는다 — 각 로봇은 위 훅으로 끼어든다.
로봇 로직과 ROS 를 섞지 않는다(CLAUDE.md §4): Driver 는 robots/ 모델과 ros/ 브리지를
'배선'만 한다. 제어 판단은 ROS2(dev 머신)가 한다(§5.6).
"""
from __future__ import annotations

import traceback

from isaacsim.core.api.robots import Robot
from pxr import Usd, UsdPhysics


def art_root(stage: Usd.Stage, under: str) -> str | None:
    """under 서브트리에서 아티큘레이션 루트 prim 경로를 찾는다. 없으면 None."""
    for p in Usd.PrimRange(stage.GetPrimAtPath(under)):
        if p.HasAPI(UsdPhysics.ArticulationRootAPI):
            return str(p.GetPath())
    return None


def ros_fail(label: str) -> None:
    """ROS 브리지 배선 실패 시 공통 배너 — 씬은 계속 띄운다(기존 방침)."""
    print("\n" + "=" * 64)
    print(f"[RosBridge] {label} 생성 실패 — 씬은 유지한다. 이 로봇 ROS 명령은 안 먹는다.")
    print("  env 확인: LD_LIBRARY_PATH(브리지 humble/lib), RMW, ROS_DOMAIN_ID")
    print("=" * 64)
    traceback.print_exc()
    print("=" * 64 + "\n")


class Driver:
    """로봇 한 대의 생명주기. 하위 클래스가 flag/name/ns/root 와 각 단계를 채운다."""

    flag = ""          # 선택 플래그 (예: "--mm")
    name = ""          # world.scene 핸들 이름 (짧게: mm/fk/iw)
    ns = ""            # ROS2 네임스페이스 (예: harvester_0)
    root = ""          # prim 경로 (예: /World/Harvester)

    def __init__(self):
        self.robot = None      # isaacsim Robot 핸들 (register 후 유효)
        self.art = None        # 아티큘레이션 루트 경로 (register 후 유효)

    # 1. 지오메트리·조인트 배치
    def spawn(self, stage: Usd.Stage) -> None:
        raise NotImplementedError

    # 2. 물리 핸들 등록 (직후 main 이 world.reset 로 뷰 초기화)
    def register(self, world, stage: Usd.Stage) -> None:
        self.art = art_root(stage, self.root)
        if self.art is None:
            raise RuntimeError(f"{self.ns} 아티큘레이션 루트를 못 찾음 — 에셋 확인")
        self.robot = world.scene.add(Robot(prim_path=self.art, name=self.name))

    # 3. 기본 관절자세 (reset 로 핸들이 유효해진 뒤)
    def configure(self, world) -> None:
        pass

    # 4. ROS 브리지·부가장치 (자세 정착 뒤)
    def finalize(self, world, stage: Usd.Stage, opts) -> None:
        pass

    # 매 프레임 (재생 중에만 적용)
    def update(self, is_playing: bool) -> None:
        pass
