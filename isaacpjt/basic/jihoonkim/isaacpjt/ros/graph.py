# -*- coding: utf-8 -*-
"""ROS2 브리지 OmniGraph 를 코드로 생성한다.

CLAUDE.md 예외 조항에 해당하는 부분이다 — ROS2 브리지는 OmniGraph 로만 닿을 수
있고(Isaac 에 rclpy 가 없다), GUI 로 클릭한 그래프는 버전 관리가 안 되므로
og.Controller.edit() 로 만든다.

**이 파일만 GPU 없이 검증이 안 된다.** 나머지(protocol/dispatcher/stub_harvester)는
전부 순수 파이썬으로 떼어놔서 dev 머신에서 테스트된다.

노드 타입명은 Isaac 버전마다 바뀌어서(omni.isaac.* -> isaacsim.*) 후보를 순서대로
시도하고 뭐가 먹혔는지 출력한다. 실패하면 사용할 수 있는 타입명을 같이 찍어준다.
"""
from __future__ import annotations

import omni.graph.core as og

# (별칭, 후보 타입명들) — 앞에서부터 시도
NODE_CANDIDATES = {
    "OnTick": ("isaacsim.core.nodes.OnPlaybackTick",
               "omni.isaac.core_nodes.OnPlaybackTick"),
    "Context": ("isaacsim.ros2.bridge.ROS2Context",
                "omni.isaac.ros2_bridge.ROS2Context"),
    "Sub": ("isaacsim.ros2.bridge.ROS2Subscriber",
            "omni.isaac.ros2_bridge.ROS2Subscriber"),
    "Pub": ("isaacsim.ros2.bridge.ROS2Publisher",
            "omni.isaac.ros2_bridge.ROS2Publisher"),
}

GRAPH_PATH = "/World/HarvestBridge"


def _pick(alias: str) -> str:
    """등록된 노드 타입 중 먼저 잡히는 후보를 고른다."""
    registry = og.GraphRegistry()
    for name in NODE_CANDIDATES[alias]:
        try:
            if registry.get_node_type(name) is not None:
                return name
        except Exception:
            pass
    raise RuntimeError(
        f"{alias} 노드 타입을 못 찾음. 시도한 후보: {NODE_CANDIDATES[alias]}\n"
        f"  -> ROS2 브리지 확장이 켜져 있는지 확인 (isaacsim.ros2.bridge).\n"
        f"  -> 켜져 있는데도 실패하면 실제 타입명을 찾아 NODE_CANDIDATES 에 추가할 것.")


def build(cmd_topic: str, status_topic: str, fruits_topic: str,
          domain_id: int = 108, log=print) -> dict[str, str]:
    """브리지 그래프를 만들고 {별칭: 노드 경로} 를 반환한다.

    반환된 경로로 bridge.py 가 data 속성을 읽고 쓴다.
    """
    types = {alias: _pick(alias) for alias in NODE_CANDIDATES}
    for alias, name in types.items():
        log(f"[Bridge] 노드 타입 {alias:8s} -> {name}")

    keys = og.Controller.Keys
    og.Controller.edit(
        {"graph_path": GRAPH_PATH, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnTick", types["OnTick"]),
                ("Context", types["Context"]),
                ("SubCmd", types["Sub"]),
                ("PubStatus", types["Pub"]),
                ("PubFruits", types["Pub"]),
            ],
            keys.CONNECT: [
                ("OnTick.outputs:tick", "SubCmd.inputs:execIn"),
                ("OnTick.outputs:tick", "PubStatus.inputs:execIn"),
                ("OnTick.outputs:tick", "PubFruits.inputs:execIn"),
                ("Context.outputs:context", "SubCmd.inputs:context"),
                ("Context.outputs:context", "PubStatus.inputs:context"),
                ("Context.outputs:context", "PubFruits.inputs:context"),
            ],
            keys.SET_VALUES: [
                # 도메인 ID 는 dev 머신과 같아야 한다 (CLAUDE.md: 108)
                ("Context.inputs:domain_id", domain_id),
                ("Context.inputs:useDomainIDEnvVar", False),

                ("SubCmd.inputs:messagePackage", "std_msgs"),
                ("SubCmd.inputs:messageSubfolder", "msg"),
                ("SubCmd.inputs:messageName", "String"),
                ("SubCmd.inputs:topicName", cmd_topic),

                ("PubStatus.inputs:messagePackage", "std_msgs"),
                ("PubStatus.inputs:messageSubfolder", "msg"),
                ("PubStatus.inputs:messageName", "String"),
                ("PubStatus.inputs:topicName", status_topic),

                ("PubFruits.inputs:messagePackage", "std_msgs"),
                ("PubFruits.inputs:messageSubfolder", "msg"),
                ("PubFruits.inputs:messageName", "String"),
                ("PubFruits.inputs:topicName", fruits_topic),
            ],
        },
    )

    log(f"[Bridge] 그래프 생성 완료: {GRAPH_PATH}")
    return {alias: f"{GRAPH_PATH}/{alias}"
            for alias in ("SubCmd", "PubStatus", "PubFruits")}
