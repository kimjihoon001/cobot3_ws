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

import os

import omni.graph.core as og

# main.py와 외부 ROS 2 노드가 사용하는 도메인을 그대로 따른다. 기존에는 여기만
# 109로 고정되어 있어서 사용자가 ROS_DOMAIN_ID=108로 실행해도 Isaac 조인트
# 브리지가 다른 DDS 도메인에 생성되는 문제가 있었다.
DOMAIN_ID = int(os.environ.get("ROS_DOMAIN_ID", "108"))

# Isaac 5.1 실측 타입명 (2026-07-19 create-probe — tools/iwhub_bridge_check.py)
T = {
    "OnTick": "omni.graph.action.OnPlaybackTick",
    "Ctx": "isaacsim.ros2.bridge.ROS2Context",
    "SubJS": "isaacsim.ros2.bridge.ROS2SubscribeJointState",
    "PubJS": "isaacsim.ros2.bridge.ROS2PublishJointState",
    "PubAny": "isaacsim.ros2.bridge.ROS2Publisher",
    "Art": "isaacsim.core.nodes.IsaacArticulationController",
    "SimTime": "isaacsim.core.nodes.IsaacReadSimulationTime",
    "Clock": "isaacsim.ros2.bridge.ROS2PublishClock",
    "SubStr": "isaacsim.ros2.bridge.ROS2Subscriber",       # 제네릭(String) — GPU 확인
    "PubStr": "isaacsim.ros2.bridge.ROS2Publisher",
    # ── Nav2 노드 (2026-07-20 create-probe 로 전부 확정 — tools/nav2_node_probe.py) ──
    # 아래 8개는 GPU 에서 실제 생성 성공한 이름이다. 같은 날 ROS2SubscribeTwist 의
    # 출력 속성도 실측했다: outputs:linearVelocity / outputs:angularVelocity (TwistPoller).
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
    # camera_info 는 CameraHelper 의 type 이 아니다(allowedTokens=rgb,depth,depth_pcl,…엔
    # camera_info 없음). 주면 "type is not supported" 가 매 프레임 터진다(2026-07-20 GPU 실측).
    # 전용 노드로 발행한다(Isaac 5.1 에 OgnROS2CameraInfoHelper 존재 확인).
    "CamInfoHelper": "isaacsim.ros2.bridge.ROS2CameraInfoHelper",
}


def _set_target(stage, node_path: str, attr: str, target: str) -> None:
    """og 값이 아니라 USD relationship 으로 걸어야 하는 targetPrim 류 헬퍼(§build_joint_bridge)."""
    prim = stage.GetPrimAtPath(node_path)
    rel = prim.GetRelationship(attr) or prim.CreateRelationship(attr)
    rel.SetTargets([target])


def _tf_topic(nav) -> str:
    """이 로봇의 TF 토픽명. nav.tf_namespace 가 있으면 /{ns}/tf, 없으면 전역 /tf.

    nav2 는 네임스페이스를 쓰면 상대 이름 'tf' 를 구독한다 = /{ns}/tf. 전역 /tf 로
    쏘면 nav2 가 못 듣는다(2026-07-20 실측). 절대 이름(앞 '/')으로 박아 nodeNamespace
    합성 규칙에 기대지 않는다. getattr 인 이유 — IwHubNavConfig 엔 이 필드가 없다.
    """
    ns = getattr(nav, "tf_namespace", "")
    return f"/{ns}/tf" if ns else "/tf"


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
    nodes = [("OnTick", T["OnTick"]), ("Ctx", T["Ctx"]),
             ("SimTime", T["SimTime"]), ("Sub", T["SubJS"]),
             ("Pub", T["PubJS"])]
    connects = [("OnTick.outputs:tick", "Sub.inputs:execIn"),
                ("OnTick.outputs:tick", "Pub.inputs:execIn"),
                ("Ctx.outputs:context", "Sub.inputs:context"),
                ("Ctx.outputs:context", "Pub.inputs:context"),
                ("SimTime.outputs:simulationTime", "Pub.inputs:timeStamp")]
    if apply_commands:
        nodes.append(("Art", T["Art"]))
        connects += [
            ("OnTick.outputs:tick", "Art.inputs:execIn"),
            ("Sub.outputs:jointNames", "Art.inputs:jointNames"),
            ("Sub.outputs:positionCommand", "Art.inputs:positionCommand"),
            ("Sub.outputs:velocityCommand", "Art.inputs:velocityCommand"),
            ("Sub.outputs:effortCommand", "Art.inputs:effortCommand"),
        ]
    _edit(graph_path, nodes, connects,
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
    mode = "직접 적용" if apply_commands else "Python 제어기로 전달"
    log(f"[RosBridge] {ns}: {cmd_topic} 수신 / {states_topic} 발행  "
        f"({art_path}, {mode}, domain={domain_id})")
    return cmd_topic, states_topic


def build_pose_publisher(graph_path: str, topic: str,
                         domain_id: int = DOMAIN_ID, log=print) -> str:
    """Python에서 채운 실제 차체 자세를 PoseStamped로 매 프레임 발행한다."""
    _edit(
        graph_path,
        [("OnTick", T["OnTick"]), ("Ctx", T["Ctx"]),
         ("Pub", T["PubAny"])],
        [("OnTick.outputs:tick", "Pub.inputs:execIn"),
         ("Ctx.outputs:context", "Pub.inputs:context")],
        [("Ctx.inputs:domain_id", domain_id),
         ("Ctx.inputs:useDomainIDEnvVar", False),
         ("Pub.inputs:messagePackage", "geometry_msgs"),
         ("Pub.inputs:messageSubfolder", "msg"),
         ("Pub.inputs:messageName", "PoseStamped"),
         ("Pub.inputs:topicName", topic)],
    )
    log(f"[RosBridge] 실제 자세 발행: {topic} (domain={domain_id})")
    return f"{graph_path}/Pub"


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


def build_string_pub(graph_path: str, topic: str,
                     domain_id: int = DOMAIN_ID, log=print) -> str:
    """제네릭 std_msgs/String 발행 그래프. 반환값은 Publisher 노드 경로다."""
    _edit(graph_path,
          [("OnTick", T["OnTick"]), ("Ctx", T["Ctx"]), ("Pub", T["PubStr"])],
          [("OnTick.outputs:tick", "Pub.inputs:execIn"),
           ("Ctx.outputs:context", "Pub.inputs:context")],
          [("Ctx.inputs:domain_id", domain_id),
           ("Ctx.inputs:useDomainIDEnvVar", False),
           ("Pub.inputs:messagePackage", "std_msgs"),
           ("Pub.inputs:messageSubfolder", "msg"),
           ("Pub.inputs:messageName", "String"),
           ("Pub.inputs:topicName", topic)])
    log(f"[RosBridge] String 발행: {topic} ({graph_path}/Pub)")
    return f"{graph_path}/Pub"


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


def build_twist_sub(graph_path: str, topic: str,
                    domain_id: int = DOMAIN_ID, log=print) -> str:
    """/cmd_vel(Twist) 구독만 하는 그래프. 반환: Sub 노드 경로 (TwistPoller 에 넣는다).

    왜 컨트롤러 노드를 안 붙이나 — Ridgeback 홀로노믹 베이스는 바퀴 조인트가 아니라
    키네마틱 더미 3축(텔레포트)이라 DifferentialController 같은 실행 노드를 못 쓴다.
    속도를 파이썬이 받아 적분한다(mm.py). 미확정 OmniGraph 노드도 하나 줄어든다.
    """
    _edit(graph_path,
          [("OnTick", T["OnTick"]), ("Ctx", T["Ctx"]), ("Sub", T["SubTwist"])],
          [("OnTick.outputs:tick", "Sub.inputs:execIn"),
           ("Ctx.outputs:context", "Sub.inputs:context")],
          [("Ctx.inputs:domain_id", domain_id),
           ("Ctx.inputs:useDomainIDEnvVar", False),
           ("Sub.inputs:topicName", topic)])
    log(f"[Nav] Twist 구독: {topic} ({graph_path}/Sub)")
    return f"{graph_path}/Sub"


def build_odometry(stage, graph_path: str, chassis_prim: str, nav,
                   domain_id: int = DOMAIN_ID, log=print) -> str:
    """섀시 오도메트리 → /odom 발행 + odom→base_link TF(raw). 반환: odom 토픽.

    chassis_prim: 오도메트리 기준 강체(보통 base_link). Nav2 localization 의 입력.
    """
    _edit(graph_path,
          [("OnTick", T["OnTick"]), ("Ctx", T["Ctx"]), ("SimTime", T["SimTime"]),
           ("Odom", T["ComputeOdom"]), ("Pub", T["PubOdom"]), ("RawTf", T["PubRawTf"])],
          [("OnTick.outputs:tick", "Odom.inputs:execIn"),
           # Nova Carter 공식 그래프와 같이 계산 완료 후 발행한다.
           # OnTick에 모두 물리면 이전 프레임 odom/TF가 발행될 수 있다.
           ("Odom.outputs:execOut", "Pub.inputs:execIn"),
           ("Odom.outputs:execOut", "RawTf.inputs:execIn"),
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
           ("RawTf.inputs:childFrameId", nav.base_frame),
           ("RawTf.inputs:topicName", _tf_topic(nav))])
    _set_target(stage, f"{graph_path}/Odom", "inputs:chassisPrim", chassis_prim)
    log(f"[Nav] odometry: {nav.odom_topic} + TF "
        f"{nav.odom_frame}→{nav.base_frame} ({chassis_prim})")
    return nav.odom_topic


def build_tf_sensor(stage, graph_path: str, parent_prim: str, sensor_prim: str,
                    nav, domain_id: int = DOMAIN_ID, log=print) -> None:
    """base_link→센서(라이다) TF 발행. Nav2 가 스캔을 로봇에 붙이는 데 필요.

    ★ PubTf(ROS2PublishTransformTree)를 쓰면 안 된다 — 그 노드는 프레임 이름을
      **prim 이름에서** 만든다. 그래서 'base_link → nav_lidar' 가 나오고
      (2026-07-20 실측), 우리가 쓰는 'harvester_0/base_link → harvester_0/laser' 와
      안 맞아 TF 트리가 두 조각으로 끊긴다. 스캔의 frame_id 에는 TF 가 아예 없게 된다.
      RawTf 는 프레임 이름을 문자열로 받으므로 이걸 쓴다.
    오프셋은 설정값이 아니라 **씬에서 실측**한다 — 에셋에 이미 달린 라이다를 쓰는
    경우(iw.hub) 설정의 lidar_offset 과 실제 장착 위치가 다르기 때문.
    """
    from pxr import UsdGeom

    cache = UsdGeom.XformCache()
    m_p = cache.GetLocalToWorldTransform(stage.GetPrimAtPath(parent_prim))
    m_s = cache.GetLocalToWorldTransform(stage.GetPrimAtPath(sensor_prim))
    rel = m_s * m_p.GetInverse()
    t = rel.ExtractTranslation()
    # 회전은 기본값(단위)으로 둔다 — attach_lidar 가 단위 쿼터니언으로 붙이므로.
    # 기울여 달면 여기서 rotation 도 넣어야 한다(입력 포맷 미실측 — 그때 probe 할 것).
    q = rel.ExtractRotationQuat()
    if abs(q.GetReal()) < 0.999:
        log(f"[Nav] ⚠ 라이다가 기울어 장착됨(w={q.GetReal():.3f}) — "
            "TF 회전은 단위로 나간다. 스캔이 틀어지면 rotation 입력을 채울 것.")
    _edit(graph_path,
          [("OnTick", T["OnTick"]), ("Ctx", T["Ctx"]), ("SimTime", T["SimTime"]),
           ("Tf", T["PubRawTf"])],
          [("OnTick.outputs:tick", "Tf.inputs:execIn"),
           ("Ctx.outputs:context", "Tf.inputs:context"),
           ("SimTime.outputs:simulationTime", "Tf.inputs:timeStamp")],
          [("Ctx.inputs:domain_id", domain_id),
           ("Ctx.inputs:useDomainIDEnvVar", False),
           ("Tf.inputs:topicName", _tf_topic(nav)),
           ("Tf.inputs:parentFrameId", nav.base_frame),
           ("Tf.inputs:childFrameId", nav.lidar_frame),
           ("Tf.inputs:translation", [t[0], t[1], t[2]])])
    log(f"[Nav] tf: {nav.base_frame}→{nav.lidar_frame} ({sensor_prim})")


def build_lidar_scan(stage, graph_path: str, lidar_prim: str, nav,
                     domain_id: int = DOMAIN_ID, log=print) -> str:
    """RTX 라이다 → /scan(LaserScan) 발행. 반환: scan 토픽.

    ★ RtxLidarHelper 에는 lidarPrim 입력이 **없다** — renderProductPath 를 받는다
      (2026-07-20 실측: inputs 에 renderProductPath 는 있고 lidarPrim 은 없음).
      예전 배선은 없는 relationship 에 타깃을 걸어 조용히 무시됐고 /scan 이 안 나왔다.
      그래서 카메라와 같은 모양으로 간다: 라이다 프림 → 렌더프로덕트 → Helper.
      해상도는 1x1 — 라이다는 픽셀이 아니라 스캔을 뽑으므로 크기가 의미 없다.
    inputs:type 의 allowedTokens 는 laser_scan / point_cloud 두 개뿐(실측).
    """
    _edit(graph_path,
          [("OnTick", T["OnTick"]), ("Ctx", T["Ctx"]),
           ("RP", T["RenderProduct"]), ("Lidar", T["RtxLidar"])],
          [("OnTick.outputs:tick", "RP.inputs:execIn"),
           ("RP.outputs:execOut", "Lidar.inputs:execIn"),
           ("Ctx.outputs:context", "Lidar.inputs:context"),
           ("RP.outputs:renderProductPath", "Lidar.inputs:renderProductPath")],
          [("Ctx.inputs:domain_id", domain_id),
           ("Ctx.inputs:useDomainIDEnvVar", False),
           ("RP.inputs:width", 1),
           ("RP.inputs:height", 1),
           ("Lidar.inputs:topicName", nav.scan_topic),
           ("Lidar.inputs:frameId", nav.lidar_frame),
           ("Lidar.inputs:type", "laser_scan")])
    _set_target(stage, f"{graph_path}/RP", "inputs:cameraPrim", lidar_prim)
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
           ("Rgb", T["CamHelper"]), ("Depth", T["CamHelper"]),
           ("Info", T["CamInfoHelper"])],
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
           # Info = ROS2CameraInfoHelper (type 입력 없음 — 렌더프로덕트에서 내부파라미터 읽음)
           ("Info.inputs:topicName", cam.info_topic),
           ("Info.inputs:frameId", cam.frame_id)])
    _set_target(stage, f"{graph_path}/RP", "inputs:cameraPrim", camera_prim)
    log(f"[Camera] {cam.rgb_topic} + {cam.depth_topic} + {cam.info_topic} "
        f"({cam.width}x{cam.height}, {camera_prim})")
    return cam.rgb_topic, cam.depth_topic


def build_camera_optical_tf(stage, graph_path: str, base_prim: str,
                            camera_prim: str, frame_id: str,
                            domain_id: int = DOMAIN_ID, log=print) -> None:
    """손끝 USD Camera의 ROS optical frame을 네이티브 동적 TF로 발행한다.

    USD Camera(+X 오른쪽,+Y 위,-Z 전방) 아래에 X축 180° 회전한 프림을 두면
    ROS optical(+X 오른쪽,+Y 아래,+Z 전방)이 된다. Python이 매 프레임 OG 값을
    쓰지 않고 USD 계층과 ROS2PublishTransformTree가 팔 움직임을 직접 추종한다.
    """
    from pxr import Gf, UsdGeom

    optical_path = f"{camera_prim}/{frame_id}"
    optical = UsdGeom.Xform.Define(stage, optical_path)
    optical.ClearXformOpOrder()
    optical.AddRotateXOp().Set(180.0)
    optical.AddTranslateOp().Set(Gf.Vec3d(0.0))
    _edit(graph_path,
          [("OnTick", T["OnTick"]), ("Ctx", T["Ctx"]),
           ("SimTime", T["SimTime"]), ("Tf", T["PubTf"])],
          [("OnTick.outputs:tick", "Tf.inputs:execIn"),
           ("Ctx.outputs:context", "Tf.inputs:context"),
           ("SimTime.outputs:simulationTime", "Tf.inputs:timeStamp")],
          [("Ctx.inputs:domain_id", domain_id),
           ("Ctx.inputs:useDomainIDEnvVar", False),
           ("Tf.inputs:topicName", "/tf"),
           ("Tf.inputs:staticPublisher", False)])
    _set_target(stage, f"{graph_path}/Tf", "inputs:parentPrim", base_prim)
    _set_target(stage, f"{graph_path}/Tf", "inputs:targetPrims", optical_path)
    log(f"[Camera] 동적 TF: {stage.GetPrimAtPath(base_prim).GetName()}→{frame_id}")


class TwistPoller:
    """ROS2SubscribeTwist 출력 폴링 → (vx, vy, wz). 홀로노믹 베이스(Ridgeback)용.

    StringPoller 와 달리 '바뀔 때만' 이 아니라 **매 프레임 현재값**을 준다 —
    속도 명령은 상태라서, 마지막 값을 계속 적분해야 Nav2 가 의도한 궤적이 나온다.
    ⚠ 그래서 **퍼블리셔가 죽으면 마지막 속도로 계속 흘러간다.** Nav2 는 목표 도달·취소
    시 0 을 보내므로 정상 흐름에선 문제없지만, dev PC 쪽이 강제종료되면 Stop 을 눌러야 한다.
    (수신 시각을 주는 출력이 없어 타임아웃 워치독을 못 단다 — GPU probe 때 확인할 것.)
    """

    def __init__(self, node_path: str):
        self._path = node_path
        self._lin = None
        self._ang = None

    def poll(self) -> tuple[float, float, float]:
        if self._lin is None:
            try:
                self._lin = og.Controller.attribute("outputs:linearVelocity", self._path)
                self._ang = og.Controller.attribute("outputs:angularVelocity", self._path)
            except Exception:
                return (0.0, 0.0, 0.0)
        lin = og.Controller.get(self._lin)
        ang = og.Controller.get(self._ang)
        if lin is None or ang is None:
            return (0.0, 0.0, 0.0)
        return (float(lin[0]), float(lin[1]), float(ang[2]))


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


class StringPublisher:
    """제네릭 String Publisher의 data 입력을 갱신한다."""

    def __init__(self, node_path: str):
        self._path = node_path
        self._attr = None

    def publish(self, value: str) -> bool:
        if self._attr is None:
            try:
                self._attr = og.Controller.attribute("inputs:data", self._path)
            except Exception:
                return False
        og.Controller.set(self._attr, value)
        return True


class JointCommandPoller:
    """ROS2SubscribeJointState 출력값을 Python 제어기에 전달한다.

    ForkliftB는 위치 명령(포크·조향)과 속도 명령(구동)을 별도
    ``ArticulationAction``으로 적용해야 해서 OmniGraph ArtController에 직접 연결하지
    않고 main.py가 이 값을 폴링한다. 같은 명령도 매 프레임 반환해 마지막 목표가
    물리 제어기에 계속 적용되도록 한다.
    """

    def __init__(self, node_path: str):
        self._path = node_path
        self._names = None
        self._positions = None
        self._velocities = None

    def _resolve(self) -> bool:
        if self._names is not None:
            return True
        try:
            self._names = og.Controller.attribute("outputs:jointNames", self._path)
            self._positions = og.Controller.attribute(
                "outputs:positionCommand", self._path
            )
            self._velocities = og.Controller.attribute(
                "outputs:velocityCommand", self._path
            )
        except Exception:
            self._names = None
            return False
        return True

    def poll(self) -> tuple[list[str], list[float], list[float]] | None:
        if not self._resolve():
            return None
        raw_names = og.Controller.get(self._names)
        names = [] if raw_names is None else list(raw_names)
        if not names:
            return None
        raw_positions = og.Controller.get(self._positions)
        raw_velocities = og.Controller.get(self._velocities)
        positions = [] if raw_positions is None else list(raw_positions)
        velocities = [] if raw_velocities is None else list(raw_velocities)
        return names, positions, velocities


class PosePublisher:
    """ROS2Publisher의 동적 PoseStamped 입력을 Python에서 갱신한다."""

    _FIELDS = (
        "header:frame_id",
        "pose:position:x", "pose:position:y", "pose:position:z",
        "pose:orientation:x", "pose:orientation:y",
        "pose:orientation:z", "pose:orientation:w",
    )

    def __init__(self, node_path: str):
        self._path = node_path
        self._attrs = None

    def _resolve(self) -> bool:
        if self._attrs is not None:
            return True
        try:
            self._attrs = {
                field: og.Controller.attribute(f"inputs:{field}", self._path)
                for field in self._FIELDS
            }
        except Exception:
            # messageName이 적용된 다음 프레임에 동적 입력이 생긴다.
            self._attrs = None
            return False
        return True

    def publish(self, position, orientation_xyzw) -> bool:
        if not self._resolve():
            return False
        values = {
            "header:frame_id": "world",
            "pose:position:x": float(position[0]),
            "pose:position:y": float(position[1]),
            "pose:position:z": float(position[2]),
            "pose:orientation:x": float(orientation_xyzw[0]),
            "pose:orientation:y": float(orientation_xyzw[1]),
            "pose:orientation:z": float(orientation_xyzw[2]),
            "pose:orientation:w": float(orientation_xyzw[3]),
        }
        for field, value in values.items():
            og.Controller.set(self._attrs[field], value)
        return True


# ══════════════════════════════════════════════════════════════════════════
#  iw.hub 전용 라이다/TF 브리지 — MM(팀원)의 동명 함수와 시그니처가 달라 _iw 로 분리.
#  iw 는 프림 이름=프레임 이름(chassis/laser_front)이라 PubTf 로 충분, 라이다 2기(앞/뒤).
#  (MM 처럼 네임스페이스 프레임이 필요하면 위쪽 build_tf_sensor/build_lidar_scan(RawTf) 사용)
# ══════════════════════════════════════════════════════════════════════════

def build_tf_sensor_iw(stage, graph_path: str, parent_prim: str, sensor_prim: str,
                       nav, child_frame: str, domain_id: int = DOMAIN_ID, log=print) -> None:
    """[iw 전용] base_link→센서(라이다) 정적 TF 발행 (/tf). Nav2 가 스캔을 로봇에 붙이는 데 필요.
    child_frame 은 sensor_prim 이름과 같아야 한다(TF child = prim 이름 = LaserScan frame_id)."""
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
    log(f"[Nav] tf(iw): {nav.base_frame}→{child_frame} ({sensor_prim})")


def build_lidar_scan_iw(stage, graph_path: str, render_product_path: str, topic: str,
                        frame: str, domain_id: int = DOMAIN_ID, log=print) -> str:
    """[iw 전용] RTX 라이다 렌더프로덕트 → /scan(LaserScan). 앞/뒤 라이다별 topic·frame 파라미터.
    render_product_path 는 LidarRtx 가 만든 것(iwhub.attach_lidar → get_render_product_path)."""
    _edit(graph_path,
          [("OnTick", T["OnTick"]), ("Ctx", T["Ctx"]), ("Lidar", T["RtxLidar"])],
          [("OnTick.outputs:tick", "Lidar.inputs:execIn"),
           ("Ctx.outputs:context", "Lidar.inputs:context")],
          [("Ctx.inputs:domain_id", domain_id),
           ("Ctx.inputs:useDomainIDEnvVar", False),
           ("Lidar.inputs:renderProductPath", render_product_path),
           ("Lidar.inputs:topicName", topic),
           ("Lidar.inputs:frameId", frame),
           ("Lidar.inputs:type", "laser_scan")])
    log(f"[Nav] lidar_scan(iw): {topic} (frame {frame}, rp {render_product_path})")
    return topic
