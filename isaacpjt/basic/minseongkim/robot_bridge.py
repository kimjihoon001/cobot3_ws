# -*- coding: utf-8 -*-
"""로봇별 ROS2 조인트 브리지 (OmniGraph 코드 생성, §5.8) — tools/iwhub_bridge_check.py 검증 패턴의 일반화.

로봇 1대당 그래프 1개:
  ROS2 --/{ns}/joint_command(sensor_msgs/JointState)--> Sub -> ArtController -> 로봇
  ROS2 <--/{ns}/joint_states------------------------- Pub <- 로봇
공용:
  /clock 발행 그래프 1개
  제네릭 String 구독(JSON 명령) — 아티큘레이션 밖 자유도용:
    MM 가동날(별도 리볼루트, excludeFromArticulation → JointState 에 안 잡힘)과
    키네마틱 베이스(위치드라이브 무시, 텔레포트만 — 2026-07-18 실측)는
    ArticulationController 로 제어가 안 되므로 JSON 명령을 파이썬이 폴링해 적용한다.

노드 타입명은 2026-07-19 create-probe 실측값(tools/iwhub_bridge_check.py · ros/graph.py GPU 검증).
env 레시피(LD_LIBRARY_PATH 등)는 tools/iwhub_bridge_check.py docstring 참조.
§5.6: 여기는 배선만 — 무엇을 명령할지는 ROS2(dev 머신)가 정한다.
"""
from __future__ import annotations

import omni.graph.core as og

DOMAIN_ID = 108           # dev 머신과 일치 필수 (CLAUDE.md)

# Isaac 5.1 실측 타입명 (2026-07-19 create-probe — tools/iwhub_bridge_check.py)
T = {
    "OnTick": "omni.graph.action.OnPlaybackTick",
    "Ctx": "isaacsim.ros2.bridge.ROS2Context",
    "SubJS": "isaacsim.ros2.bridge.ROS2SubscribeJointState",
    "PubJS": "isaacsim.ros2.bridge.ROS2PublishJointState",
    "Art": "isaacsim.core.nodes.IsaacArticulationController",
    "SimTime": "isaacsim.core.nodes.IsaacReadSimulationTime",
    "Clock": "isaacsim.ros2.bridge.ROS2PublishClock",
    "SubStr": "isaacsim.ros2.bridge.ROS2Subscriber",       # 제네릭(String) — GPU 확인
}


def _edit(graph_path: str, nodes, connects, values) -> None:
    keys = og.Controller.Keys
    og.Controller.edit(
        {"graph_path": graph_path, "evaluator_name": "execution"},
        {keys.CREATE_NODES: nodes, keys.CONNECT: connects, keys.SET_VALUES: values})


def build_clock(graph_path: str = "/World/RosClock",
                domain_id: int = DOMAIN_ID, log=print) -> None:
    """/clock 발행 그래프 (전체에 1개면 된다)."""
    _edit(graph_path,
          [("OnTick", T["OnTick"]), ("Ctx", T["Ctx"]),
           ("SimTime", T["SimTime"]), ("Clock", T["Clock"])],
          [("OnTick.outputs:tick", "Clock.inputs:execIn"),
           ("Ctx.outputs:context", "Clock.inputs:context"),
           ("SimTime.outputs:simulationTime", "Clock.inputs:timeStamp")],
          [("Ctx.inputs:domain_id", domain_id),
           ("Ctx.inputs:useDomainIDEnvVar", False),
           ("Clock.inputs:topicName", "/clock")])
    log(f"[RosBridge] /clock 발행 그래프: {graph_path}")


def build_joint_bridge(stage, graph_path: str, ns: str, art_path: str,
                       domain_id: int = DOMAIN_ID, log=print,
                       apply_commands: bool = True) -> tuple[str, str]:
    """로봇 1대의 JointState 명령/상태 브리지. 반환: (명령 토픽, 상태 토픽).

    art_path: 아티큘레이션 루트 prim 경로. targetPrim 은 relationship 이라
    og 값 세팅이 아니라 USD 로 건다 (2026-07-19 실측).
    """
    cmd_topic = f"/{ns}/joint_command"
    states_topic = f"/{ns}/joint_states"
    nodes = [
        ("OnTick", T["OnTick"]), ("Ctx", T["Ctx"]),
        ("SimTime", T["SimTime"]), ("Sub", T["SubJS"]), ("Pub", T["PubJS"]),
    ]
    connects = [
        ("OnTick.outputs:tick", "Sub.inputs:execIn"),
        ("OnTick.outputs:tick", "Pub.inputs:execIn"),
        ("Ctx.outputs:context", "Sub.inputs:context"),
        ("Ctx.outputs:context", "Pub.inputs:context"),
        ("SimTime.outputs:simulationTime", "Pub.inputs:timeStamp"),
    ]
    if apply_commands:
        nodes.append(("Art", T["Art"]))
        connects.extend([
            ("OnTick.outputs:tick", "Art.inputs:execIn"),
            ("Sub.outputs:jointNames", "Art.inputs:jointNames"),
            ("Sub.outputs:positionCommand", "Art.inputs:positionCommand"),
            ("Sub.outputs:velocityCommand", "Art.inputs:velocityCommand"),
            ("Sub.outputs:effortCommand", "Art.inputs:effortCommand"),
        ])
    _edit(graph_path,
          nodes,
          connects,
          [("Ctx.inputs:domain_id", domain_id),
           ("Ctx.inputs:useDomainIDEnvVar", False),
           ("Sub.inputs:topicName", cmd_topic),
           ("Pub.inputs:topicName", states_topic)])
    target_nodes = ("Art", "Pub") if apply_commands else ("Pub",)
    for node in target_nodes:
        prim = stage.GetPrimAtPath(f"{graph_path}/{node}")
        rel = prim.GetRelationship("inputs:targetPrim")
        if not rel:
            rel = prim.CreateRelationship("inputs:targetPrim")
        rel.SetTargets([art_path])
    log(f"[RosBridge] {ns}: {cmd_topic} 수신 / {states_topic} 발행  ({art_path})")
    return cmd_topic, states_topic


def build_string_sub(graph_path: str, topic: str,
                     domain_id: int = DOMAIN_ID, log=print) -> str:
    """제네릭 String 구독 그래프. 반환: Sub 노드 경로 (StringPoller 에 넣는다)."""
    _edit(graph_path,
          [("OnTick", T["OnTick"]), ("Ctx", T["Ctx"]), ("Sub", T["SubStr"])],
          [("OnTick.outputs:tick", "Sub.inputs:execIn"),
           ("Ctx.outputs:context", "Sub.inputs:context")],
          [("Ctx.inputs:domain_id", domain_id),
           ("Ctx.inputs:useDomainIDEnvVar", False),
           ("Sub.inputs:messagePackage", "std_msgs"),
           ("Sub.inputs:messageSubfolder", "msg"),
           ("Sub.inputs:messageName", "String"),
           ("Sub.inputs:topicName", topic)])
    log(f"[RosBridge] String 구독: {topic} ({graph_path}/Sub)")
    return f"{graph_path}/Sub"


class StringPoller:
    """제네릭 String Sub 의 outputs:data 폴링 — 새 메시지 원문만 돌려준다.

    data 속성은 messageName 이 풀린 뒤(한 프레임 이상 지나서) 생기므로 늦게 잡고,
    같은 문자열이 계속 붙어 있으므로 바뀔 때만 반환한다 (harvest_bridge 와 동일 요령).
    """

    def __init__(self, node_path: str):
        self._path = node_path
        self._attr = None
        self._last = ""

    def poll(self) -> str | None:
        if self._attr is None:
            try:
                self._attr = og.Controller.attribute("outputs:data", self._path)
            except Exception:
                return None
        raw = og.Controller.get(self._attr) or ""
        if raw and raw != self._last:
            self._last = raw
            return str(raw)
        return None


class JointCommandPoller:
    """ROS2SubscribeJointState의 최신 명령 출력을 Python 제어기에 전달한다."""

    def __init__(self, node_path: str):
        self._path = node_path
        self._attrs = None

    def poll(self):
        if self._attrs is None:
            try:
                self._attrs = tuple(
                    og.Controller.attribute(name, self._path)
                    for name in (
                        "outputs:jointNames",
                        "outputs:positionCommand",
                        "outputs:velocityCommand",
                    )
                )
            except Exception:
                return None
        names, positions, velocities = (
            og.Controller.get(attr) for attr in self._attrs
        )
        if not names:
            return None
        return list(names), list(positions), list(velocities)
