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
    # ── Nav2 노드 (⚠ 아래 타입명·속성명은 아직 create-probe 미확정) ──────────────
    # tools/nav2_node_probe.py 를 GPU 에서 돌려 첫 [OK] 값으로 갱신할 것. graph.py 처럼
    # 추측으로 두면 Play 때 터진다(§8). 그때까지 이 브리지는 main.py 플래그로 옵트인만.
    "SubTwist": "isaacsim.ros2.bridge.ROS2SubscribeTwist",
    "DiffCtrl": "isaacsim.robot.wheeled_robots.DifferentialController",
    "Break3": "omni.graph.nodes.BreakVector3",
    "ComputeOdom": "isaacsim.core.nodes.IsaacComputeOdometry",
    "PubOdom": "isaacsim.ros2.bridge.ROS2PublishOdometry",
    "PubRawTf": "isaacsim.ros2.bridge.ROS2PublishRawTransformTree",
    "PubTf": "isaacsim.ros2.bridge.ROS2PublishTransformTree",
    "RtxLidar": "isaacsim.ros2.bridge.ROS2RtxLidarHelper",
    "RenderProduct": "isaacsim.core.nodes.IsaacCreateRenderProduct",
    "CamHelper": "isaacsim.ros2.bridge.ROS2CameraHelper",
}


def _set_target(stage, node_path: str, attr: str, target: str) -> None:
    """og 값이 아니라 USD relationship 으로 걸어야 하는 targetPrim 류 헬퍼(§build_joint_bridge)."""
    prim = stage.GetPrimAtPath(node_path)
    rel = prim.GetRelationship(attr) or prim.CreateRelationship(attr)
    rel.SetTargets([target])


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
                       domain_id: int = DOMAIN_ID, log=print) -> tuple[str, str]:
    """로봇 1대의 JointState 명령/상태 브리지. 반환: (명령 토픽, 상태 토픽).

    art_path: 아티큘레이션 루트 prim 경로. targetPrim 은 relationship 이라
    og 값 세팅이 아니라 USD 로 건다 (2026-07-19 실측).
    """
    cmd_topic = f"/{ns}/joint_command"
    states_topic = f"/{ns}/joint_states"
    _edit(graph_path,
          [("OnTick", T["OnTick"]), ("Ctx", T["Ctx"]), ("SimTime", T["SimTime"]),
           ("Sub", T["SubJS"]), ("Art", T["Art"]), ("Pub", T["PubJS"])],
          [("OnTick.outputs:tick", "Sub.inputs:execIn"),
           ("OnTick.outputs:tick", "Art.inputs:execIn"),
           ("OnTick.outputs:tick", "Pub.inputs:execIn"),
           ("Ctx.outputs:context", "Sub.inputs:context"),
           ("Ctx.outputs:context", "Pub.inputs:context"),
           ("SimTime.outputs:simulationTime", "Pub.inputs:timeStamp"),
           ("Sub.outputs:jointNames", "Art.inputs:jointNames"),
           ("Sub.outputs:positionCommand", "Art.inputs:positionCommand"),
           ("Sub.outputs:velocityCommand", "Art.inputs:velocityCommand"),
           ("Sub.outputs:effortCommand", "Art.inputs:effortCommand")],
          [("Ctx.inputs:domain_id", domain_id),
           ("Ctx.inputs:useDomainIDEnvVar", False),
           ("Sub.inputs:topicName", cmd_topic),
           ("Pub.inputs:topicName", states_topic)])
    for node in ("Art", "Pub"):
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


# ══════════════════════════════════════════════════════════════════════════
#  Nav2 브리지 (iw.hub 자율주행) — /cmd_vel·/odom·/tf·/scan
#  ⚠ 전부 GPU create-probe 미확정. main.py 플래그로 하나씩 켜며 검증(순서: drive→odom→scan).
#  판단(Nav2 경로계획)은 dev PC(ROS2), 여기는 배선+발행/구독만(§5.6).
# ══════════════════════════════════════════════════════════════════════════

def build_diff_drive(stage, graph_path: str, art_path: str,
                     drive_joints: tuple[str, str], nav,
                     domain_id: int = DOMAIN_ID, log=print) -> str:
    """/cmd_vel(Twist) → 차동컨트롤러 → 좌우 구동륜 속도. 반환: cmd_vel 토픽.

    Twist 의 linear/angular 는 vec3 라 BreakVector3 로 x(전진)·z(회전)만 뽑아 넣는다.
    nav: IwHubNavConfig (wheel_radius/wheel_base/max_* — §5.7 값은 settings 에서).
    """
    _edit(graph_path,
          [("OnTick", T["OnTick"]), ("Ctx", T["Ctx"]),
           ("Sub", T["SubTwist"]), ("BrkLin", T["Break3"]), ("BrkAng", T["Break3"]),
           ("Diff", T["DiffCtrl"]), ("Art", T["Art"])],
          [("OnTick.outputs:tick", "Sub.inputs:execIn"),
           ("OnTick.outputs:tick", "Diff.inputs:execIn"),
           ("OnTick.outputs:tick", "Art.inputs:execIn"),
           ("Ctx.outputs:context", "Sub.inputs:context"),
           ("Sub.outputs:linearVelocity", "BrkLin.inputs:tuple"),
           ("Sub.outputs:angularVelocity", "BrkAng.inputs:tuple"),
           ("BrkLin.outputs:x", "Diff.inputs:linearVelocity"),
           ("BrkAng.outputs:z", "Diff.inputs:angularVelocity"),
           ("Diff.outputs:velocityCommand", "Art.inputs:velocityCommand")],
          [("Ctx.inputs:domain_id", domain_id),
           ("Ctx.inputs:useDomainIDEnvVar", False),
           ("Sub.inputs:topicName", nav.cmd_vel_topic),
           ("Diff.inputs:wheelRadius", nav.wheel_radius),
           ("Diff.inputs:wheelDistance", nav.wheel_base),
           ("Diff.inputs:maxLinearSpeed", nav.max_linear_speed),
           ("Diff.inputs:maxAngularSpeed", nav.max_angular_speed),
           ("Art.inputs:jointNames", list(drive_joints))])
    _set_target(stage, f"{graph_path}/Art", "inputs:targetPrim", art_path)
    log(f"[Nav] diff_drive: {nav.cmd_vel_topic} → {drive_joints} ({art_path})")
    return nav.cmd_vel_topic


def build_odometry(stage, graph_path: str, chassis_prim: str, nav,
                   domain_id: int = DOMAIN_ID, log=print) -> str:
    """섀시 오도메트리 → /odom 발행 + odom→base_link TF(raw). 반환: odom 토픽.

    chassis_prim: 오도메트리 기준 강체(보통 base_link). Nav2 localization 의 입력.
    """
    _edit(graph_path,
          [("OnTick", T["OnTick"]), ("Ctx", T["Ctx"]), ("SimTime", T["SimTime"]),
           ("Odom", T["ComputeOdom"]), ("Pub", T["PubOdom"]), ("RawTf", T["PubRawTf"])],
          [("OnTick.outputs:tick", "Odom.inputs:execIn"),
           ("OnTick.outputs:tick", "Pub.inputs:execIn"),
           ("OnTick.outputs:tick", "RawTf.inputs:execIn"),
           ("Ctx.outputs:context", "Pub.inputs:context"),
           ("Ctx.outputs:context", "RawTf.inputs:context"),
           ("SimTime.outputs:simulationTime", "Pub.inputs:timeStamp"),
           ("SimTime.outputs:simulationTime", "RawTf.inputs:timeStamp"),
           ("Odom.outputs:linearVelocity", "Pub.inputs:linearVelocity"),
           ("Odom.outputs:angularVelocity", "Pub.inputs:angularVelocity"),
           ("Odom.outputs:position", "Pub.inputs:position"),
           ("Odom.outputs:orientation", "Pub.inputs:orientation"),
           ("Odom.outputs:position", "RawTf.inputs:translation"),
           ("Odom.outputs:orientation", "RawTf.inputs:rotation")],
          [("Ctx.inputs:domain_id", domain_id),
           ("Ctx.inputs:useDomainIDEnvVar", False),
           ("Pub.inputs:topicName", nav.odom_topic),
           ("Pub.inputs:odomFrameId", nav.odom_frame),
           ("Pub.inputs:chassisFrameId", nav.base_frame),
           ("RawTf.inputs:parentFrameId", nav.odom_frame),
           ("RawTf.inputs:childFrameId", nav.base_frame)])
    _set_target(stage, f"{graph_path}/Odom", "inputs:chassisPrim", chassis_prim)
    log(f"[Nav] odometry: {nav.odom_topic} + TF {nav.odom_frame}→{nav.base_frame} ({chassis_prim})")
    return nav.odom_topic


def build_tf_sensor(stage, graph_path: str, parent_prim: str, sensor_prim: str,
                    nav, domain_id: int = DOMAIN_ID, log=print) -> None:
    """base_link→센서(라이다) 정적 TF 발행 (/tf). Nav2 가 스캔을 로봇에 붙이는 데 필요."""
    _edit(graph_path,
          [("OnTick", T["OnTick"]), ("Ctx", T["Ctx"]), ("SimTime", T["SimTime"]),
           ("Tf", T["PubTf"])],
          [("OnTick.outputs:tick", "Tf.inputs:execIn"),
           ("Ctx.outputs:context", "Tf.inputs:context"),
           ("SimTime.outputs:simulationTime", "Tf.inputs:timeStamp")],
          [("Ctx.inputs:domain_id", domain_id),
           ("Ctx.inputs:useDomainIDEnvVar", False)])
    _set_target(stage, f"{graph_path}/Tf", "inputs:parentPrim", parent_prim)
    _set_target(stage, f"{graph_path}/Tf", "inputs:targetPrims", sensor_prim)
    log(f"[Nav] tf: {nav.base_frame}→{nav.lidar_frame} ({sensor_prim})")


def build_lidar_scan(stage, graph_path: str, lidar_prim: str, nav,
                     domain_id: int = DOMAIN_ID, log=print) -> str:
    """RTX 라이다 → /scan(LaserScan) 발행. 반환: scan 토픽.

    ⚠ 가장 probe 의존적. RtxLidarHelper 는 보통 renderProductPath 를 요구한다 —
      lidar_prim 에서 렌더프로덕트를 만드는 배선은 GPU 에서 실측 보정 필요(§8 예고).
    """
    _edit(graph_path,
          [("OnTick", T["OnTick"]), ("Ctx", T["Ctx"]), ("Lidar", T["RtxLidar"])],
          [("OnTick.outputs:tick", "Lidar.inputs:execIn"),
           ("Ctx.outputs:context", "Lidar.inputs:context")],
          [("Ctx.inputs:domain_id", domain_id),
           ("Ctx.inputs:useDomainIDEnvVar", False),
           ("Lidar.inputs:topicName", nav.scan_topic),
           ("Lidar.inputs:frameId", nav.lidar_frame),
           ("Lidar.inputs:type", "laser_scan")])
    _set_target(stage, f"{graph_path}/Lidar", "inputs:lidarPrim", lidar_prim)
    log(f"[Nav] lidar_scan: {nav.scan_topic} (frame {nav.lidar_frame}, {lidar_prim})")
    return nav.scan_topic


# ══════════════════════════════════════════════════════════════════════════
#  카메라 브리지 (손끝 D455) — /rgb·/depth·/camera_info  (YOLO 파인튜닝용 시뮬 이미지)
#  ⚠ GPU create-probe 미확정. main.py --camera 로 옵트인. §5.6: 발행만, 인식은 ROS2.
# ══════════════════════════════════════════════════════════════════════════

def build_camera(stage, graph_path: str, camera_prim: str, cam,
                 domain_id: int = DOMAIN_ID, log=print) -> tuple[str, str]:
    """D455 카메라 → /rgb + /depth + /camera_info 발행. 반환: (rgb, depth) 토픽.

    렌더프로덕트 1개(카메라에서 width×height) → CameraHelper 3개(rgb/depth/camera_info).
    cam: CameraBridgeConfig (§5.7 값은 settings). camera_prim: UsdGeom.Camera 경로
    (harvester.camera_path). cameraPrim 은 relationship 이라 USD 로 건다.
    """
    _edit(graph_path,
          [("OnTick", T["OnTick"]), ("Ctx", T["Ctx"]), ("RP", T["RenderProduct"]),
           ("Rgb", T["CamHelper"]), ("Depth", T["CamHelper"]), ("Info", T["CamHelper"])],
          [("OnTick.outputs:tick", "RP.inputs:execIn"),
           ("RP.outputs:execOut", "Rgb.inputs:execIn"),
           ("RP.outputs:execOut", "Depth.inputs:execIn"),
           ("RP.outputs:execOut", "Info.inputs:execIn"),
           ("Ctx.outputs:context", "Rgb.inputs:context"),
           ("Ctx.outputs:context", "Depth.inputs:context"),
           ("Ctx.outputs:context", "Info.inputs:context"),
           ("RP.outputs:renderProductPath", "Rgb.inputs:renderProductPath"),
           ("RP.outputs:renderProductPath", "Depth.inputs:renderProductPath"),
           ("RP.outputs:renderProductPath", "Info.inputs:renderProductPath")],
          [("Ctx.inputs:domain_id", domain_id),
           ("Ctx.inputs:useDomainIDEnvVar", False),
           ("RP.inputs:width", cam.width),
           ("RP.inputs:height", cam.height),
           ("Rgb.inputs:type", "rgb"),
           ("Rgb.inputs:topicName", cam.rgb_topic),
           ("Rgb.inputs:frameId", cam.frame_id),
           ("Depth.inputs:type", "depth"),
           ("Depth.inputs:topicName", cam.depth_topic),
           ("Depth.inputs:frameId", cam.frame_id),
           ("Info.inputs:type", "camera_info"),
           ("Info.inputs:topicName", cam.info_topic),
           ("Info.inputs:frameId", cam.frame_id)])
    _set_target(stage, f"{graph_path}/RP", "inputs:cameraPrim", camera_prim)
    log(f"[Camera] {cam.rgb_topic} + {cam.depth_topic} + {cam.info_topic} "
        f"({cam.width}x{cam.height}, {camera_prim})")
    return cam.rgb_topic, cam.depth_topic


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
