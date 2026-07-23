# -*- coding: utf-8 -*-
"""수확 MM 드라이버 (--mm) — Ridgeback + UR10e + 동축 3축 1/4구 스쿱.

로봇 모델은 robots/harvester.py, ROS 브리지는 ros/robot_bridge.py. 이 파일은 그 둘을
'배선'하고 텔레옵/JSON 명령을 매 프레임 적용한다(§5.6 실행측). 판단은 ROS2 가 한다.
"""
from __future__ import annotations

import json
import math
import os
import zlib

import numpy as np
from isaacsim.core.utils.types import ArticulationAction

from robot_base import Driver, ros_fail
from robots.harvester import HOME_POSE_DEG, HarvestMM

# ★바닥 z 머지-세이프(2026-07-23): f2(팀원)가 공통바닥을 COMMON_FLOOR_Z=0.055 로 올리고
#   mm.py POSE.z 도 같이 올린다. moveit_mm 도 같은 값을 fallback import 로 따라가 머지 전(0.0)/
#   후(0.055) 모두 바닥에 정합(안 그러면 머지 후 MoveIt MM 만 5.5cm 잠김 — Codex 지적).
try:
    from scene.ground import COMMON_FLOOR_Z as _FLOOR_Z
except Exception:
    _FLOOR_Z = 0.0
# ★데모 스폰(2026-07-23): 맵(farm.pgm, Y≥-1.08) 안의 row2 플랜트(-4.35 이랑, Y≈+6) 마주봄.
#   Nav2 로컬라이제이션이 되게 맵 범위 안에 둔다. 베이스 yaw 180°(configure)로 −X(과실) 향함.
#   POSE 는 (x,y,z) 이동값(yaw 없음 — dummy_base_revolute_z_joint 로 준다).
POSE = (-3.3, -9.77, _FLOOR_Z)  # 수확테스트 위치(그립 과실 Y=-9.8). 생성맵이 이 영역 덮음 → Nav2 가능.
# 키네마틱 베이스 조인트 (JSON base 명령용 — 텔레포트만 먹는다, 2026-07-18 실측)
BASE_JOINTS = ("dummy_base_prismatic_x_joint",
               "dummy_base_prismatic_y_joint",
               "dummy_base_revolute_z_joint")


def _fruit_id(path: str) -> int:
    """USD 경로를 ROS JSON에 넣기 좋은 안정적인 32-bit ID로 바꾼다."""
    return zlib.crc32(path.encode("utf-8")) & 0xffffffff


class MMDriver(Driver):
    flag = "--mm"
    name = "mm_moveit"      # ★scene.add 등록명 분리(2026-07-23, Codex): 팀원 mm.py=name"mm" 와
    #                        동시(--mm --moveit) 실행 시 world.scene.add(name=) 충돌 방지.
    ns = "harvester_0"
    root = "/World/HarvesterMoveit"       # ★stage 격리(2026-07-23): 팀원 mm.py=/World/Harvester 와 별개

    SCOOP_JOINTS = ("scoop_quarter_1_joint",
                    "scoop_quarter_2_joint",
                    "cutter_quarter_3_joint")
    # CAD dimensions.json 규약. 열림 → 수용(닫힘) → 외측 날 +50° 절삭.
    SCOOP_OPEN = np.radians((0.0, -90.0, -180.0))
    SCOOP_CLOSED = np.radians((0.0, 0.0, 0.0))
    SCOOP_CUT = np.radians((0.0, 0.0, 50.0))

    def __init__(self, cfg, task=None):
        super().__init__()
        self._cfg = cfg
        self._task = task
        # ★제어모드별 ROS2 네임스페이스 분리(2026-07-23) — MoveIt 데모는 harvester_moveit,
        #   팀원 RMPflow 는 harvester_0 → 토픽/노드 안 겹침(같은 MM 을 각자 모드로).
        import sys as _sys
        self.ns = "harvester_moveit" if "--moveit" in _sys.argv else "harvester_0"
        self._mm = HarvestMM(cfg.robots)
        self._base_idx = None
        self._scoop_indices = None
        self._gripper_idx = None
        self._gripper_target = None
        self._air_fruit_prim = None            # --airfruit 모드: 현재 선택된 과실만 발행
        self._air_fruit_home = None            # 선택 과실 스폰 위치(reset_air 로 복귀)
        self._air_fruits = []                  # 여러 과실 경로(스윕이 select_fruit 로 전환)
        self._air_fruit_sel = 0                # 선택 인덱스(set_friction 대상 머티리얼)
        self._grasped_fruit = None             # 부착 파지 중인 과실 prim(씬/airfruit) — detach 대상
        self._contact_fruit = None             # 순수마찰 접촉계측 대상(절단 뒤 pickable 제외돼도 유지)
        self._grip_direct = False              # 파지 스파이크: Isaac 직접 그리퍼 구동(MoveIt 없이)
        self._grip_force = None                # 힘/토크 제어(위치제어 대신 일정 토크 유지)
        self._pinch_point = None               # 그리퍼 닫았을 때 좌우 패드 첫 접촉점(TCP)
        self._poller = None
        self._blade_poller = None
        self._blade_state_pub = None
        self._status_pub = None
        self._contact_pub = None
        self._fruit_pub = None
        self._fruit_cursor = 0
        # 절단 후 pickable_fruits()에서 빠진 대상도 운반 검증을 위해 계속 추적한다.
        self._tracked_harvested_fruit = None
        self._teleop = None
        self._stage = None
        self._twist = None                    # /cmd_vel 폴러 (Nav2 주행)
        self._dt = 1.0 / 60.0                 # finalize 에서 월드 실제 물리 dt 로 덮어씀
        self._rmpflow = None
        self._was_playing = False
        self._cut_status = {}
        self._friction_retry_limit = 120

    def spawn(self, stage):
        self._mm.spawn(stage, self.root, POSE)

    def configure(self, world):
        # 시작자세(HOME_POSE_DEG)를 아티큘레이션 기본자세로도 박는다 — Play/Stop 리셋에도 유지.
        # ★ 대입(=)이지 누적(+=)이 아니다: spawn 의 _preset_pose 가 이미 USD 에 같은 각을
        #   구워놨다. 거기에 또 더하면 wrist_1 이 360°(≡0°) 가 돼 Play 순간 팔이 0° 로
        #   떨어진다(사용자 지적 2026-07-20).
        r = self.robot
        q0 = np.asarray(r.get_joint_positions(), dtype=float)
        names = list(r.dof_names)
        for jname, deg in HOME_POSE_DEG:
            q0[names.index(jname)] = np.radians(deg)
        # ★베이스 yaw 180°(2026-07-23): 팔은 섀시 +X 를 보는데, 잡을 그립 과실은 −X(-4.15)에
        #   있다. 베이스를 π 돌려 팔이 −X(과실)를 향하게 한다. dummy_base_revolute_z_joint=π.
        if "dummy_base_revolute_z_joint" in names:
            q0[names.index("dummy_base_revolute_z_joint")] = math.pi
        r.set_joints_default_state(positions=q0)

    def finalize(self, world, stage, opts):
        self._stage = stage
        self._dt = world.get_physics_dt()
        # 키네마틱 베이스 인덱스 (JSON base 명령용)
        r = self.robot
        self._base_idx = np.array(
            [list(r.dof_names).index(n) for n in BASE_JOINTS])
        self._scoop_indices = np.array(
            [list(r.dof_names).index(n) for n in self.SCOOP_JOINTS])
        # 구형 상태 JSON과 일부 진단 코드는 scalar를 기대하므로 외측 커터축을 대표값으로 둔다.
        self._gripper_idx = int(self._scoop_indices[2])
        # RMPflow 목표를 따라갈 때 링크가 처져 보이지 않도록 UR10e 6축의
        # articulation position drive 게인만 강화한다. 베이스/그리퍼는 건드리지 않는다.
        arm_indices = np.array([
            list(r.dof_names).index(name) for name, _ in HOME_POSE_DEG])
        # g/rmp_home 으로 확실히 홈 가도록 — 팔 6축 홈 관절값(라디안)과 인덱스를 보관한다.
        # rmpflow.go_home 의 cspace 자세 목표가 약해(EE 목표 없음) 안 갈 때, 또는
        # rmp_stop 에 막혔을 때 이 값으로 직접 위치구동한다(아래 apply 루프).
        self._arm_indices = arm_indices
        self._arm_home_q = np.radians(np.array([deg for _, deg in HOME_POSE_DEG]))
        self._homing = False
        controller = r.get_articulation_controller()
        kps, kds = controller.get_gains()
        if hasattr(kps, "cpu"):
            kps = kps.cpu().numpy()
        if hasattr(kds, "cpu"):
            kds = kds.cpu().numpy()
        kps = np.asarray(kps, dtype=float).copy()
        kds = np.asarray(kds, dtype=float).copy()
        kps[arm_indices] = np.maximum(kps[arm_indices] * 2.0, 10000.0)
        kds[arm_indices] = np.maximum(kds[arm_indices] * 2.0, 1000.0)
        # 세 동축 회전축은 같은 낮은 position gain으로 구동해 중첩 셸의 접촉 폭발을 막는다.
        gi = self._scoop_indices
        kps[gi] = float(os.environ.get("SCOOP_KP", "1200"))
        kds[gi] = float(os.environ.get("SCOOP_KD", "80"))
        self._grip_position_kp = float(kps[gi[2]])
        self._grip_position_kd = float(kds[gi[2]])
        controller.set_gains(kps=kps, kds=kds)
        try:
            efforts = controller.get_max_efforts()
            if hasattr(efforts, "cpu"):
                efforts = efforts.cpu().numpy()
            efforts = np.asarray(efforts, dtype=float).copy()
            # ★파지력 상한 — 강체 과조임 반발력으로 튕기는 것 방지. 논문: F≥mg/(2μ)≈0.65N.
            # 5.0 N·m→패드 ~85N(과함). env 로 낮춰 antipodal 마찰 파지(안전배수 몇 N)만 걸리게.
            grip_effort = float(os.environ.get("SCOOP_EFFORT", "40.0"))
            efforts[gi] = grip_effort
            controller.set_max_efforts(values=efforts)
        except Exception as exc:
            print(f"[MM] ⚠ 그리퍼 effort 상한 실패(계속 진행): {exc}")
        print(f"[MM] 동축 스쿱 3축 위치제어: kp={kps[gi].tolist()} "
              f"maxEffort={grip_effort:.1f} "
              f"joints={list(self.SCOOP_JOINTS)}")
        self._gripper_target = self.SCOOP_OPEN.copy()
        # 그리퍼 마찰 재바인딩 one-shot — 조립시점엔 PhysX 콜라이더가 없어(play 전) 0개가
        # 걸렸다(2026-07-22 실측 "콜라이더 0개" → 과실 미끄러짐). play+step 후 콜라이더가
        # 생기면 그때 스파이크와 동일하게 바인딩한다. 아래 update() 에서 실행.
        self._friction_bound = False
        self._play_steps = 0
        print(f"[MM] UR10e drive gain 강화: kp={kps[arm_indices].tolist()} "
              f"kd={kds[arm_indices].tolist()}")

        if not opts.no_ros:
            try:
                from ros import robot_bridge as RB
                RB.build_joint_bridge(stage, f"/World/RosBridge_{self.ns}",
                                      self.ns, self.art,
                                      # MoveIt 전환(2026-07-22): /joint_command 를 항상 직접
                                      # 적용한다(topic_based_ros2_control 이 이 토픽으로 구동).
                                      # rmpflow 복원 시 not opts.rmpflow 로 되돌릴 것.
                                      apply_commands=True,
                                      # ★HW 상태채널 분리(2026-07-23): JSB 가 /{ns}/joint_states 로
                                      # arm 만 발행 → Isaac 전체관절과 겹치면 move_group 이 dummy_base
                                      # 관절 못 찾아 에러. Isaac 은 hw_joint_states 로 발행(URDF 도 동일).
                                      states_topic=f"/{self.ns}/hw_joint_states")
                sub = RB.build_string_sub(
                    f"/World/RosCmd_{self.ns}", f"/{self.ns}/cmd")
                self._poller = RB.StringPoller(sub)
                blade_sub = RB.build_float64_sub(
                    f"/World/RosBladeCmd_{self.ns}",
                    f"/{self.ns}/blade_command")
                self._blade_poller = RB.Float64Poller(blade_sub)
                blade_pub = RB.build_float64_pub(
                    f"/World/RosBladeState_{self.ns}",
                    f"/{self.ns}/blade_state")
                self._blade_state_pub = RB.Float64Publisher(blade_pub)
                pub = RB.build_string_pub(
                    f"/World/RosRmpStatus_{self.ns}",
                    f"/{self.ns}/rmpflow/status")
                self._status_pub = RB.StringPublisher(pub)
                contact_pub = RB.build_string_pub(
                    f"/World/RosGripContact_{self.ns}",
                    f"/{self.ns}/grasp_contact")
                self._contact_pub = RB.StringPublisher(contact_pub)
                fruit_pub = RB.build_string_pub(
                    f"/World/RosSimTomato_{self.ns}",
                    f"/{self.ns}/sim/tomato")
                self._fruit_pub = RB.StringPublisher(fruit_pub)
            except Exception:
                ros_fail("MM 조인트/명령 브리지")
            if opts.camera:
                self._build_camera(stage)
            if opts.nav_drive or opts.nav_odom or opts.nav_scan:
                self._dt = world.get_physics_dt()
                # ★nav 격리(2026-07-23): 공유 harvester_nav config 는 전역(/cmd_vel,tf_namespace="")
                #   이라 팀원 RMPflow 와 겹친다. moveit_mm 에서만 self.ns 로 오버라이드 —
                #   /harvester_moveit/{cmd_vel,odom,scan} + /harvester_moveit/tf. config 원본 불변.
                import dataclasses as _dc
                # ★odom 자식프레임을 mm_base 로(2026-07-23): base_link 은 URDF 에서 mm_base→base_link
                #   (팔마운트 +0.3z)의 자식이다. Isaac Nav 가 odom→base_link 를 쏘면 base_link 부모가
                #   둘(odom·mm_base)이 돼 TF 트리가 쪼개진다(사용자·Codex 진단). odom→mm_base 로 내보내면
                #   odom→mm_base→base_link 단일 트리 → move_group 이 센서/odom 을 mm_base 로 변환 가능.
                nav = _dc.replace(
                    self._cfg.robots.harvester_nav,
                    tf_namespace=self.ns,
                    # Nav2 원출력 대신 0.35초 watchdog 중계 토픽을 받는다. OmniGraph
                    # SubscribeTwist가 마지막 비영(非零) 값을 영구 유지하는 runaway 방지.
                    cmd_vel_topic=f"/{self.ns}/cmd_vel_safe",
                    odom_topic=f"/{self.ns}/odom",
                    scan_topic=f"/{self.ns}/scan",
                    base_frame="mm_base")
                self._twist = build_nav(stage, self._mm, nav, opts)

        # ── MoveIt 전환(2026-07-22): RMPflow 비활성(주석 보존). 팔은 MoveIt 이
        # /harvester_0/joint_command 로 직접 구동한다(topic_based_ros2_control).
        # 되살리려면 아래 주석 해제 + 위 apply_commands=not opts.rmpflow 복원.
        # if opts.rmpflow:
        #     from robots.control import RmpFlowTargetController
        #     self._rmpflow = RmpFlowTargetController(
        #         r, stage,
        #         reference_prim=f"{self.root}/Base/base_link",
        #         arm_base_prim=f"{self.root}/Arm/base_link",
        #         physics_dt=world.get_physics_dt(),
        #         tool_tcp_prim=self._mm.grasp_tcp_path(stage))
        #     print("[RMPflow] UR10e 목표 추종 활성: /harvester_0/cmd rmp_target")
        if opts.rmpflow:
            print("[MM] --rmpflow 는 MoveIt 전환으로 비활성(무시) — mm.py 주석 참고")

        if opts.mm_teleop and opts.rmpflow:
            print("[MM] --rmpflow와 --mm-teleop 동시 제어는 충돌하므로 텔레옵 비활성")
        elif opts.mm_teleop:
            self._teleop = build_teleop(r, self._set_scoop_cutter_deg, opts.gui)

    def _set_scoop_cutter_deg(self, deg: float) -> None:
        """텔레옵/레거시 blade 명령을 외측 1/4구 관절로 변환한다."""
        if self._scoop_indices is None:
            return
        target = self.SCOOP_CLOSED.copy()
        target[2] = math.radians(max(0.0, min(50.0, float(deg))))
        self.robot.apply_action(ArticulationAction(
            joint_positions=target, joint_indices=self._scoop_indices))

    def _build_camera(self, stage):
        sensor_paths = self._mm.camera_paths(stage)
        if not sensor_paths.get("color"):
            print("[Camera] D455 센서 prim 못 찾음 — 전체 스트림 발행 스킵")
            return
        try:
            from ros import robot_bridge as RB
            import dataclasses as _dc
            # MoveIt 카메라는 Isaac ROS 노드 네임스페이스·상대 토픽·frame_id까지 모두
            # harvester_moveit 식별자를 사용한다. RMP MM(/harvester_0 또는 /harvester/*),
            # 같은 프로세스의 두 번째 D455 그래프와 이름이 겹칠 여지를 없앤다.
            cam = _dc.replace(
                self._cfg.robots.camera,
                node_namespace=self.ns,
                rgb_topic="rgb",
                depth_topic="depth",
                info_topic="camera_info",
                depth_info_topic="depth/camera_info",
                pointcloud_topic="depth/points",
                infra1_topic="infra1/image_raw",
                infra2_topic="infra2/image_raw",
                infra1_info_topic="infra1/camera_info",
                infra2_info_topic="infra2/camera_info",
                imu_topic="imu",
                frame_id=f"{self.ns}_d455_color_optical_frame",
                depth_frame_id=f"{self.ns}_d455_depth_optical_frame",
                infra1_frame_id=f"{self.ns}_d455_infra1_optical_frame",
                infra2_frame_id=f"{self.ns}_d455_infra2_optical_frame",
                imu_frame_id=f"{self.ns}_d455_imu_frame")
            graph_base = f"/World/RosD455_{self.ns}"
            RB.build_d455(stage, graph_base, sensor_paths, cam)
            # TF parent는 URDF의 base_link와 같은 Arm/base_link. 센서마다 원본 D455
            # extrinsic이 다르므로 optical frame도 각각의 실제 prim 아래에 만든다.
            base_prim = f"{self.root}/Arm/base_link"
            tf_topic = f"/{self.ns}/tf"
            for key, frame in (
                ("color", cam.frame_id),
                ("depth", cam.depth_frame_id),
                ("infra1", cam.infra1_frame_id),
                ("infra2", cam.infra2_frame_id),
            ):
                if key in sensor_paths:
                    RB.build_camera_optical_tf(
                        stage, f"/World/RosD455Tf_{self.ns}_{key}",
                        base_prim, sensor_paths[key], frame, tf_topic=tf_topic)
            if "imu" in sensor_paths:
                RB.build_sensor_tf(
                    stage, f"/World/RosD455Tf_{self.ns}_imu",
                    base_prim, sensor_paths["imu"], cam.imu_frame_id,
                    tf_topic=tf_topic)
        except Exception:
            import traceback
            print("\n[Camera] 그래프 생성 실패 — 씬 유지. probe 로 노드명 확인.")
            traceback.print_exc()

    def update(self, is_playing):
        if is_playing and not self._was_playing:
            self._tracked_harvested_fruit = None
            self._cut_status = {}
            if self._rmpflow is not None:
                self._rmpflow.reset()
        self._was_playing = is_playing
        if self._blade_state_pub is not None and self._scoop_indices is not None:
            q = np.asarray(self.robot.get_joint_positions(), dtype=float)
            self._blade_state_pub.publish(float(np.degrees(q[self._scoop_indices[2]])))
        if self._teleop is not None:                 # 키보드 텔레옵 (재생 중에만 적용)
            self._teleop(is_playing)
        if is_playing and not self._friction_bound:
            # play 후 PhysX 콜라이더가 생긴 뒤 한 번만 그리퍼 마찰 바인딩(스파이크 방식).
            self._play_steps += 1
            if self._play_steps >= 10 and self._stage is not None:
                try:
                    bound = self._mm._bind_gripper_friction(
                        self._stage, self._mm._gripper_path, print)
                except Exception as exc:
                    print(f"[MM] ⚠ 그리퍼 마찰 재바인딩 실패: {exc}")
                    bound = 0
                self._friction_bound = bound > 0
                if not self._friction_bound and self._play_steps >= self._friction_retry_limit:
                    print("[MM] ⚠ 그리퍼 콜라이더 마찰 바인딩을 120프레임 동안 못 함 — "
                          "파지 명령을 보내기 전에 에셋 콜라이더를 확인하세요.", flush=True)
                    # 계속 재시도하되 경고는 한 번만 낸다.
                    self._friction_retry_limit = 10**9
        if is_playing and self._twist is not None:   # Nav2 /cmd_vel → 홀로노믹 베이스
            self._drive_base()
        if is_playing and self._blade_poller is not None:
            blade_target = self._blade_poller.poll()
            if blade_target is not None:
                target = self.SCOOP_CLOSED.copy()
                target[2] = math.radians(float(blade_target))
                self.robot.apply_action(ArticulationAction(
                    joint_positions=target, joint_indices=self._scoop_indices))
        # MM JSON 명령 (블레이드·베이스) — 재생 중에만 적용
        if is_playing and self._poller is not None:
            raw = self._poller.poll()
            if raw:
                try:
                    cmd = json.loads(raw)
                except ValueError:
                    cmd = None
                if isinstance(cmd, dict):
                    if "blade" in cmd:
                        target = self.SCOOP_CLOSED.copy()
                        target[2] = math.radians(float(cmd["blade"]))
                        self.robot.apply_action(ArticulationAction(
                            joint_positions=target, joint_indices=self._scoop_indices))
                    if "base" in cmd:
                        b = [float(v) for v in cmd["base"]]
                        home_ready = (self._rmpflow is None
                                      or self._rmpflow.status()["at_home"])
                        if len(b) == 3 and home_ready:
                            self.robot.set_joint_positions(
                                np.array(b), joint_indices=self._base_idx)
                        elif len(b) == 3:
                            print("[MM] 홈 자세 전 베이스 이동 차단")
                    if "rmp_target" in cmd and self._rmpflow is not None:
                        target = cmd["rmp_target"]
                        if isinstance(target, dict):
                            self._homing = False   # 새 목표가 오면 홈 복귀 취소
                            try:
                                self._rmpflow.set_target(
                                    target["position"], int(target.get("id", 0)),
                                    str(target.get("phase", "MOVE")))
                            except (KeyError, TypeError, ValueError) as exc:
                                print(f"[RMPflow] 잘못된 목표 무시: {exc}")
                    if cmd.get("rmp_stop") is True and self._rmpflow is not None:
                        self._rmpflow.stop()
                    if "rmp_home" in cmd:
                        home = cmd["rmp_home"]
                        if isinstance(home, dict):
                            if self._rmpflow is not None:
                                self._rmpflow.go_home(int(home.get("id", 0)))
                            self._homing = True   # 관절 직접 구동 — rmpflow 없어도 g 홈 동작
                    if "gripper" in cmd and isinstance(cmd["gripper"], dict):
                        closed = bool(cmd["gripper"].get("closed", False))
                        self._gripper_target = (
                            self.SCOOP_CLOSED.copy() if closed else self.SCOOP_OPEN.copy())
                    if "grip_width" in cmd:
                        print("[MM] grip_width는 2F-85 전용이라 무시합니다. "
                              "새 스쿱에는 gripper.closed 또는 3축 controller 명령을 쓰세요.")
                    if "cut_fruit" in cmd:
                        self._handle_cut(cmd["cut_fruit"])
                    if cmd.get("drop_air") and self._air_fruit_prim is not None:
                        # 공중 과실을 dynamic 으로 = 중력 ON(절단 모사). 이제 그리퍼가
                        # 마찰로 안 잡으면 떨어진다 → 진짜 파지 검증(사용자 지적).
                        from scene.physics import set_kinematic
                        af = self._stage.GetPrimAtPath(self._air_fruit_prim)
                        if af.IsValid():
                            set_kinematic(af, False)
                            print("[AirFruit] dynamic 전환(중력 ON) — 마찰 파지 검증")
                    if cmd.get("reset_air") and self._air_fruit_prim is not None:
                        # airfruit 를 다시 kinematic + 스폰위치로 복귀 → Isaac 재시작 없이
                        # TCP/그립 스윕 반복(2026-07-22). drop_air 로 바닥에 떨어진 과실 리셋.
                        from scene.physics import set_kinematic
                        from pxr import UsdGeom
                        af = self._stage.GetPrimAtPath(self._air_fruit_prim)
                        if af.IsValid():
                            set_kinematic(af, True)
                            if self._air_fruit_home is not None:
                                for op in UsdGeom.Xformable(af).GetOrderedXformOps():
                                    if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                                        op.Set(self._air_fruit_home); break
                            print("[AirFruit] reset — kinematic 복귀(스폰위치)")
                    if "grip" in cmd:
                        # 파지 스파이크: Isaac 직접 그리퍼 구동(MoveIt/ros2_control 없이).
                        self._grip_direct = True
                        self._grip_force = None                # 위치제어 모드
                        self._gripper_target = float(cmd["grip"])
                    if cmd.get("attach_fruit"):
                        # ★부착 파지 — 마찰 대신 FixedJoint 로 과실을 그리퍼에 붙여 따라오게.
                        #   {"position":[x,y,z]} 면 그 위치 최근접 ripe **씬 과실**(맵에 달린 것),
                        #   True 면 airfruit(레거시). 진짜 마찰은 지그 고친 뒤 복원.
                        _req = cmd["attach_fruit"]
                        if isinstance(_req, dict) and "position" in _req:
                            _ok = self._attach_scene_fruit(_req)
                            if self._status_pub is not None and "attach_id" in _req:
                                self._status_pub.publish(json.dumps({
                                    "attach_id": int(_req["attach_id"]),
                                    "attach_success": bool(_ok),
                                }, separators=(",", ":")))
                        elif self._air_fruit_prim is not None:
                            if self._attach_prim_to_gripper(self._air_fruit_prim):
                                print("[Grasp] attach_fruit(airfruit)", flush=True)
                    if cmd.get("detach_grasp"):
                        # 파지 해제 — 부착 조인트 끄면 과실이 (팔레트 위로) 떨어진다.
                        _fp = self._grasped_fruit or self._air_fruit_prim
                        if _fp:
                            _gj = self._stage.GetPrimAtPath(_fp + "/GraspJoint")
                            if _gj.IsValid():
                                from pxr import UsdPhysics as _UP6
                                _UP6.FixedJoint(_gj).GetJointEnabledAttr().Set(False)
                                print("[Grasp] detach_grasp — 부착 해제(과실 낙하/적재)", flush=True)
                    if "grip_force" in cmd:
                        # ★힘/토크 제어 — 위치제어(진동·침투) 대신 일정 닫힘토크 유지(사용자 #4).
                        # 런타임에 finger 위치 drive만 0으로 내려 직접 effort와의 간섭을 없앤다.
                        self._grip_direct = True
                        self._grip_force = float(cmd["grip_force"])
                        if self._gripper_idx is not None:
                            ctrl = self.robot.get_articulation_controller()
                            kps, kds = ctrl.get_gains()
                            if hasattr(kps, "cpu"):
                                kps = kps.cpu().numpy()
                            if hasattr(kds, "cpu"):
                                kds = kds.cpu().numpy()
                            kps = np.asarray(kps, dtype=float).copy()
                            kds = np.asarray(kds, dtype=float).copy()
                            kps[self._gripper_idx] = 0.0
                            # 직접 토크만 걸고 kd까지 0으로 만들면 강체 줄기에서 finger 속도가
                            # ±2.5rad/s로 계속 튄다. 작은 속도감쇠는 유지하되 위치 스프링만
                            # 제거해 일정 정상력(force closure)과 정착을 동시에 얻는다.
                            kds[self._gripper_idx] = float(
                                os.environ.get("GRIP_FORCE_KD", "10.0"))
                            ctrl.set_gains(kps=kps, kds=kds)
                        print(f"[Grip] 일정 닫힘토크 {self._grip_force:.2f} N·m", flush=True)
                    if cmd.get("grip_position_mode") and self._gripper_idx is not None:
                        # 해제/다음 사이클: 저장해 둔 위치 drive 게인을 복원하고 ROS2 position
                        # controller가 다시 finger_joint를 열고 닫게 한다.
                        ctrl = self.robot.get_articulation_controller()
                        kps, kds = ctrl.get_gains()
                        if hasattr(kps, "cpu"):
                            kps = kps.cpu().numpy()
                        if hasattr(kds, "cpu"):
                            kds = kds.cpu().numpy()
                        kps = np.asarray(kps, dtype=float).copy()
                        kds = np.asarray(kds, dtype=float).copy()
                        kps[self._gripper_idx] = self._grip_position_kp
                        kds[self._gripper_idx] = self._grip_position_kd
                        ctrl.set_gains(kps=kps, kds=kds)
                        self._grip_force = None
                        self._grip_direct = False
                        print("[Grip] 위치제어 복원", flush=True)
                    if "grip_effort" in cmd and self._gripper_idx is not None:
                        # 파지력 상한 실시간 변경 — 재시작 없이 힘 스윕(강체 튕김 vs 미끄럼 창 찾기).
                        ctrl = self.robot.get_articulation_controller()
                        eff = ctrl.get_max_efforts()
                        if hasattr(eff, "cpu"):
                            eff = eff.cpu().numpy()
                        eff = np.asarray(eff, dtype=float).copy()
                        eff[self._gripper_idx] = float(cmd["grip_effort"])
                        ctrl.set_max_efforts(values=eff)
                        print(f"[Spike] grip_effort → {float(cmd['grip_effort']):.2f} N·m", flush=True)
                    if cmd.get("dump_pads"):
                        # 손가락 하위 모든 메시의 월드 bbox(min/max)+시각재질 displayColor → 흰색 앞패드 식별.
                        from pxr import Usd as _Ud, UsdGeom as _UGd, UsdShade as _UShd
                        _bbd = _UGd.BBoxCache(_Ud.TimeCode.Default(),
                                              [_UGd.Tokens.default_, _UGd.Tokens.render])
                        _tpd = self._mm.grasp_tcp_path(self._stage)
                        _robd = _tpd.rsplit("/base_link/", 1)[0] if _tpd else None
                        _lf = self._stage.GetPrimAtPath(_robd + "/left_inner_finger")
                        for _p in _Ud.PrimRange(_lf):
                            if not _p.IsA(_UGd.Mesh):
                                continue
                            _rg = _bbd.ComputeWorldBound(_p).ComputeAlignedRange()
                            _mn, _mx = _rg.GetMin(), _rg.GetMax()
                            _dc = _UGd.Gprim(_p).GetDisplayColorAttr().Get()
                            _mp = _UShd.MaterialBindingAPI(_p).GetDirectBinding().GetMaterialPath()
                            print(f"[Pads] {_p.GetName()} x[{_mn[0]:.3f},{_mx[0]:.3f}] "
                                  f"y[{_mn[1]:.3f},{_mx[1]:.3f}] z[{_mn[2]:.3f},{_mx[2]:.3f}] "
                                  f"color={_dc} mat={_mp}", flush=True)
                    if cmd.get("find_pinch"):
                        # ★그리퍼 닫은 현재 상태에서 좌우 안쪽 패드 콜라이더의 최근접(첫 접촉)점 = TCP.
                        # 토마토 없이 그리퍼만 닫고 호출 → 그 점에 파지타깃 배치(사용자 지시 2026-07-23).
                        from pxr import Usd as _U3, UsdGeom as _UG3
                        _bb3 = _UG3.BBoxCache(_U3.TimeCode.Default(),
                                              [_UG3.Tokens.default_, _UG3.Tokens.render])
                        _tp3 = self._mm.grasp_tcp_path(self._stage)
                        _rob3 = _tp3.rsplit("/base_link/", 1)[0] if _tp3 else None
                        _pad3 = os.environ.get("GRIP_PAD", "finger4step")
                        _msh3 = (f"/visuals/Defeatured_2F_85_PAD_OPEN_{_pad3}_01"
                                 f"/Defeatured_2F_85_PAD_OPEN_{_pad3}")
                        _lp3 = self._stage.GetPrimAtPath(_rob3 + "/left_inner_finger" + _msh3)
                        _rp3 = self._stage.GetPrimAtPath(_rob3 + "/right_inner_finger" + _msh3)
                        if _lp3.IsValid() and _rp3.IsValid():
                            from pxr import Gf as _Gf3
                            _lr = _bb3.ComputeWorldBound(_lp3).ComputeAlignedRange()
                            _rr = _bb3.ComputeWorldBound(_rp3).ComputeAlignedRange()
                            _lc = _lr.GetMidpoint(); _rc = _rr.GetMidpoint()
                            _lmin, _lmax = _lr.GetMin(), _lr.GetMax()
                            _rmin, _rmax = _rr.GetMin(), _rr.GetMax()
                            # 닫힘축 = 좌우 패드 중심차 최대 축. 마주보는 '안쪽 면'의 중점이 실제 접촉점.
                            _ax = max(range(3), key=lambda k: abs(_lc[k]-_rc[k]))
                            if _lc[_ax] > _rc[_ax]:
                                _face = (_lmin[_ax] + _rmax[_ax]) / 2; _gap3 = _lmin[_ax] - _rmax[_ax]
                            else:
                                _face = (_lmax[_ax] + _rmin[_ax]) / 2; _gap3 = _rmin[_ax] - _lmax[_ax]
                            _pp = [(_lc[k]+_rc[k])/2 for k in range(3)]; _pp[_ax] = _face
                            self._pinch_point = _Gf3.Vec3d(*_pp)
                            print(f"[Spike] find_pinch — 접촉면중점(TCP)="
                                  f"{tuple(round(float(v),4) for v in self._pinch_point)} "
                                  f"안쪽면간격={_gap3*1000:.1f}mm 닫힘축={'XYZ'[_ax]} (닫힘상태)", flush=True)
                        else:
                            print(f"[Spike] find_pinch 실패 — 패드 prim 없음", flush=True)
                    if cmd.get("dump_gripper"):
                        # ★그리퍼 에셋 실제 설정 덤프 — 조인트/드라이브/미믹/한계/dof 순서.
                        from pxr import Usd as _U2, UsdPhysics as _UP2, PhysxSchema as _PX2
                        r = self.robot
                        print(f"[Dump] dof_names(순서)={list(r.dof_names)}", flush=True)
                        try:
                            ctrl = r.get_articulation_controller()
                            kp, kd = ctrl.get_gains()
                            me = ctrl.get_max_efforts()
                            _g = lambda a: (a.cpu().numpy() if hasattr(a, "cpu") else a)
                            print(f"[Dump] kp={list(_g(kp))}", flush=True)
                            print(f"[Dump] kd={list(_g(kd))}", flush=True)
                            print(f"[Dump] maxEffort={list(_g(me))}", flush=True)
                        except Exception as _e:
                            print(f"[Dump] gains 실패 {_e}", flush=True)
                        _tp = self._mm.grasp_tcp_path(self._stage)
                        _robp = _tp.rsplit("/base_link/", 1)[0] if _tp else None
                        for _p in _U2.PrimRange(self._stage.GetPrimAtPath(_robp)):
                            _isj = _p.IsA(_UP2.RevoluteJoint) or _p.IsA(_UP2.PrismaticJoint)
                            if not _isj:
                                continue
                            _line = f"[Dump] JOINT {_p.GetName()} ({_p.GetTypeName()})"
                            for _tok in ("angular", "linear"):
                                if _p.HasAPI(_UP2.DriveAPI, _tok):
                                    _d = _UP2.DriveAPI.Get(_p, _tok)
                                    _line += (f" | drive[{_tok}] type={_d.GetTypeAttr().Get()}"
                                              f" tgtPos={_d.GetTargetPositionAttr().Get()}"
                                              f" stiff={_d.GetStiffnessAttr().Get()}"
                                              f" damp={_d.GetDampingAttr().Get()}"
                                              f" maxF={_d.GetMaxForceAttr().Get()}")
                            _j = _UP2.RevoluteJoint(_p) if _p.IsA(_UP2.RevoluteJoint) else _UP2.PrismaticJoint(_p)
                            _line += f" | limit[{_j.GetLowerLimitAttr().Get()},{_j.GetUpperLimitAttr().Get()}]"
                            if _p.HasAPI(_PX2.PhysxMimicJointAPI, "rotX") or _p.HasAPI(_PX2.PhysxMimicJointAPI):
                                _line += " | MIMIC"
                            print(_line, flush=True)
                    if cmd.get("report_grip") and self._gripper_idx is not None:
                        # 실측 — 드라이브가 finger 를 실제로 밀고 있나(측정 effort). 0 이면 무접촉/무력.
                        r = self.robot
                        gi = self._gripper_idx
                        try:
                            pos = np.asarray(r.get_joint_positions()).flatten()
                            eff = np.asarray(r.get_measured_joint_efforts()).flatten()
                            vel = np.asarray(r.get_joint_velocities()).flatten()
                            rep = {"finger": float(pos[gi]), "eff": float(eff[gi]),
                                   "vel": float(vel[gi])}
                            if self._status_pub is not None:   # ★Isaac 실측을 ROS 로(토픽 vel 은 쓰레기)
                                self._status_pub.publish(json.dumps(rep))
                            print(f"[Spike] ★finger pos={rep['finger']:.3f} "
                                  f"측정effort={rep['eff']:.4f} vel={rep['vel']:.4f}", flush=True)
                        except Exception as _e:
                            print(f"[Spike] report_grip 실패: {_e}", flush=True)
                    if cmd.get("report_grasp_contact"):
                        self._report_grasp_contact(cmd["report_grasp_contact"])
                    if cmd.get("fruit_to_grip") and self._air_fruit_prim is not None:
                        # ★그립줄기를 '손끝 패드 실제 위치'에 결정론적으로 배치(추측·튜닝 X).
                        # 좌우 fingertip 패드 메시의 월드 bbox 중점 = 손끝 파지점.
                        from pxr import Gf, Usd, UsdGeom
                        from scene.physics import set_kinematic
                        cache = UsdGeom.XformCache()
                        bb = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                                               [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
                        tpath = self._mm.grasp_tcp_path(self._stage)
                        rob = tpath.rsplit("/base_link/", 1)[0] if tpath else None
                        # 실제 파지면 = 안쪽 평평 패드(finger4step)의 '실제 콜라이더 자식 메시'
                        # (visual 컨테이너 bbox 아님 — 사용자 지적 2026-07-23). 손끝은 호로 비켜감.
                        _pad = os.environ.get("GRIP_PAD", "finger4step")
                        _mesh = (f"/visuals/Defeatured_2F_85_PAD_OPEN_{_pad}_01"
                                 f"/Defeatured_2F_85_PAD_OPEN_{_pad}")
                        lp = self._stage.GetPrimAtPath(rob + "/left_inner_finger" + _mesh) if rob else None
                        rp = self._stage.GetPrimAtPath(rob + "/right_inner_finger" + _mesh) if rob else None
                        af = self._stage.GetPrimAtPath(self._air_fruit_prim)
                        # 파지 대상: "stem"=원통, "box"=육면체, 그 외=몸통 구. 분리 시험용.
                        part = {"stem": "/GripStem", "box": "/GripBox"}.get(
                            str(cmd.get("fruit_to_grip")), "/Collision")
                        col = self._stage.GetPrimAtPath(self._air_fruit_prim + part)
                        if lp and lp.IsValid() and rp and rp.IsValid() and af.IsValid() and col.IsValid():
                            set_kinematic(af, True)
                            # ★실측 — 질량·마찰이 실제 무엇인지(스케일 질량버그/마찰 미적용 진단)
                            try:
                                from isaacsim.core.prims import SingleRigidPrim
                                _q = SingleRigidPrim(self._air_fruit_prim, name="mq")
                                _q.initialize()
                                _m = float(_q.get_mass())
                            except Exception as _e:
                                _m = f"조회실패({_e})"
                            from pxr import Usd as _U, UsdPhysics as _UP, UsdShade as _USh
                            _mat = self._stage.GetPrimAtPath(f"/World/PM/airfruit_{self._air_fruit_sel}")
                            _ma = _UP.MaterialAPI(_mat)
                            print(f"[Spike] ★실측 질량={_m} kg, 과실마찰 static="
                                  f"{_ma.GetStaticFrictionAttr().Get()} dyn={_ma.GetDynamicFrictionAttr().Get()}",
                                  flush=True)
                            # 과실 하위 모든 콜라이더 + 바인딩된 물리재질 덤프(마찰 없는 콜라이더 색출)
                            for _p in _U.PrimRange(af):
                                if _p.HasAPI(_UP.CollisionAPI):
                                    _b = _USh.MaterialBindingAPI(_p).GetDirectBinding("physics").GetMaterialPath()
                                    print(f"[Spike]   과실콜라이더 {_p.GetName()}  물리재질={_b}", flush=True)
                            # ★손끝 패드 콜라이더 재질도 덤프(패드 쪽 무마찰 진단)
                            for _pp in (lp, rp):
                                _bb = _USh.MaterialBindingAPI(_pp).GetDirectBinding("physics").GetMaterialPath()
                                _hasc = _pp.HasAPI(_UP.CollisionAPI)
                                print(f"[Spike]   패드 {_pp.GetName()} CollisionAPI={_hasc} 물리재질={_bb}", flush=True)
                            if self._pinch_point is not None:      # ★find_pinch 첫접촉점 사용
                                gp = self._pinch_point
                            else:
                                lc = bb.ComputeWorldBound(lp).ComputeAlignedRange().GetMidpoint()
                                rc = bb.ComputeWorldBound(rp).ComputeAlignedRange().GetMidpoint()
                                gp = Gf.Vec3d((lc[0]+rc[0])/2, (lc[1]+rc[1])/2, (lc[2]+rc[2])/2)
                            sw = cache.GetLocalToWorldTransform(col).ExtractTranslation()
                            fw = cache.GetLocalToWorldTransform(af).ExtractTranslation()
                            tgt = Gf.Vec3d(gp[0]-(sw[0]-fw[0]), gp[1]-(sw[1]-fw[1]),
                                           gp[2]-(sw[2]-fw[2]))
                            for op in UsdGeom.Xformable(af).GetOrderedXformOps():
                                if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                                    op.Set(tgt); break
                            _src = "핀치점" if self._pinch_point is not None else "패드중점"
                            print(f"[Spike] fruit_to_grip[{part[1:]}] — {_src} "
                                  f"{tuple(round(float(v),3) for v in gp)} 배치", flush=True)
                        else:
                            print("[Spike] fruit_to_grip 실패 — 손끝패드/과실/줄기 prim 없음 "
                                  f"(rob={rob})", flush=True)
                    if "select_fruit" in cmd:
                        # 스윕: 대상 과실 전환(fresh 과실 → reset 반복 상태오염 회피, 2026-07-22).
                        self._select_air_fruit(int(cmd["select_fruit"]))
                        print(f"[AirFruit] select_fruit {self._air_fruit_sel} "
                              f"({self._air_fruit_prim})")
                    if "set_friction" in cmd and self._air_fruit_prim is not None:
                        # 스파이크 마찰 스윕(2026-07-22): 선택 과실+줄기 머티리얼 μ 실시간 변경.
                        # combineMode=min 이라 유효 마찰=min(μ,그리퍼0.9)=μ. Isaac 재시작 없이 반복.
                        from scene.physics import set_material_friction
                        mu = float(cmd["set_friction"])
                        i = self._air_fruit_sel
                        ok = (set_material_friction(self._stage, f"/World/PM/airfruit_{i}", mu)
                              and set_material_friction(self._stage, f"/World/PM/airstem_{i}", mu))
                        print(f"[AirFruit] set_friction[{i}] μ={mu:.2f} ({'적용' if ok else '실패'})")
                    if "foliage" in cmd:
                        self._toggle_foliage(bool(cmd["foliage"]))
        if is_playing and self._homing:
            # 홈 직접 구동 — rmpflow 없이도 g(rmp_home)가 동작(MoveIt 전환 중 유지).
            # ⚠ MoveIt 이 /joint_command 스트리밍 중이면 서로 싸움 — MoveIt 붙인 뒤엔
            # g 홈도 MoveIt 목표로 대체 예정. 새 목표(rmp_target)가 오면 _homing=False.
            self.robot.apply_action(ArticulationAction(
                joint_positions=self._arm_home_q,
                joint_indices=self._arm_indices))
        elif is_playing and self._rmpflow is not None:
            self._rmpflow.apply()
        # MoveIt 모드에서는 RMPflow가 없어도 커터 결과는 ROS로 내보내야 한다.
        # 그렇지 않으면 외부 수확 노드가 절단 명령 전송만으로 성공을 오판한다.
        if (is_playing and self._rmpflow is None and self._status_pub is not None
                and "cut_id" in self._cut_status):
            gripper = float(self.robot.get_joint_positions()[self._gripper_idx])
            self._status_pub.publish(json.dumps({
                "cut_id": self._cut_status["cut_id"],
                "cut_success": self._cut_status["cut_success"],
                "gripper": round(gripper, 3),
            }, separators=(",", ":")))
        if is_playing and self._rmpflow is not None:
            if self._status_pub is not None:
                status = self._rmpflow.status()
                status["gripper"] = float(
                    self.robot.get_joint_positions()[self._gripper_idx])
                status["blade"] = math.degrees(status["gripper"])
                status.update(self._cut_status)
                # Isaac 5.1 generic ROS2Publisher의 std_msgs/String data는 긴
                # 문자열을 약 128 byte에서 "..."로 잘라 버린다. 잘린 JSON은
                # manipulator_target_node가 파싱할 수 없어 reached=true를 놓치고
                # 모든 동작이 ERROR_TIMEOUT으로 끝난다. 제어 루프에 필요한 값만
                # 짧게 보내고, 좌표 상세값은 Isaac 콘솔의 RMPflow 로그로 본다.
                if "cut_id" in status:
                    # 절단 응답은 FSM이 기다리는 필드만 보낸다. 일반 모션 상태에
                    # 두 필드를 덧붙이면 generic String의 128-byte 한계를 넘는다.
                    wire_status = {
                        "cut_id": status["cut_id"],
                        "cut_success": status["cut_success"],
                        "gripper": round(status["gripper"], 3),
                    }
                else:
                    wire_status = {
                        "id": status["id"],
                        "phase": status["phase"],
                        "active": status["active"],
                        "reached": status["reached"],
                        "gripper": round(status["gripper"], 3),
                    }
                    if status["distance"] is not None:
                        wire_status["distance"] = round(status["distance"], 4)
                    if status["phase"] == "HOME":
                        wire_status["at_home"] = status["at_home"]
                payload = json.dumps(wire_status, separators=(",", ":"))
                if len(payload.encode("utf-8")) > 120:
                    # 향후 필드가 늘어도 조용히 다시 JSON을 깨뜨리지 않는다.
                    print(f"[RMPflow] 상태 payload 초과({len(payload)}): {payload}")
                else:
                    self._status_pub.publish(payload)
        # 기본 경로는 ros2_control의 3축 명령이다. JSON 직접 모드는 독립 시험용이다.
        if is_playing and self._grip_direct:
            gi = self._scoop_indices
            if self._gripper_target is not None:
                self.robot.apply_action(ArticulationAction(
                    joint_positions=np.asarray(self._gripper_target), joint_indices=gi))
            if self._status_pub is not None:
                try:
                    _p = np.asarray(self.robot.get_joint_positions()).flatten()
                    _v = np.asarray(self.robot.get_joint_velocities()).flatten()
                    self._status_pub.publish(json.dumps(
                        {"scoop": [float(_p[i]) for i in gi],
                         "vel": [float(_v[i]) for i in gi]}))
                except Exception:
                    pass
        if is_playing:
            self._publish_sim_tomato()

    def set_air_fruits(self, paths: list) -> None:
        """--airfruit 모드: 여러 공중 과실 경로 등록. 스윕이 select_fruit i 로 대상 전환."""
        self._air_fruits = list(paths)
        if self._air_fruits:
            self._select_air_fruit(0)

    def _select_air_fruit(self, idx: int) -> None:
        """발행·drop·reset·set_friction 대상을 idx 과실로 전환 + 스폰 위치 저장."""
        if not (0 <= idx < len(self._air_fruits)):
            return
        self._air_fruit_sel = idx
        path = self._air_fruits[idx]
        self._air_fruit_prim = path
        if self._stage is not None:
            from pxr import UsdGeom
            prim = self._stage.GetPrimAtPath(path)
            if prim.IsValid():
                for op in UsdGeom.Xformable(prim).GetOrderedXformOps():
                    if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                        self._air_fruit_home = op.Get(); break

    def set_air_fruit(self, path: str) -> None:
        """단일 과실 등록(하위호환) — 발행할 공중 과실 prim 경로 + 스폰 위치 저장."""
        self._air_fruits = [path]
        self._air_fruit_sel = 0
        self._air_fruit_prim = path
        if self._stage is not None:
            from pxr import UsdGeom
            prim = self._stage.GetPrimAtPath(path)
            if prim.IsValid():
                for op in UsdGeom.Xformable(prim).GetOrderedXformOps():
                    if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                        self._air_fruit_home = op.Get(); break

    def _attach_prim_to_gripper(self, fruit_path: str) -> bool:
        """과실 prim 을 그리퍼(grip_base)에 FixedJoint 로 부착 — dynamic 전환 + §8 프레임.
        절단(dynamic) 순간에도 과실이 그리퍼에 매달려 유지된다(마찰 대체, 데모용)."""
        from pxr import Gf, UsdGeom, UsdPhysics
        from scene.physics import set_kinematic
        grip = self._mm._grip_base
        fp = self._stage.GetPrimAtPath(fruit_path)
        if not (grip and fp.IsValid()):
            return False
        set_kinematic(fp, False)                       # dynamic(조인트가 붙잡게)
        jp = fruit_path + "/GraspJoint"
        j = UsdPhysics.FixedJoint.Define(self._stage, jp)
        j.CreateBody0Rel().SetTargets([grip])
        j.CreateBody1Rel().SetTargets([fruit_path])
        j.CreateJointEnabledAttr(True)
        cache = UsdGeom.XformCache()
        m0 = cache.GetLocalToWorldTransform(self._stage.GetPrimAtPath(grip))
        m1 = cache.GetLocalToWorldTransform(fp)

        def _rig(m):
            r = Gf.Matrix4d(); r.SetRotate(m.ExtractRotationQuat().GetNormalized())
            r.SetTranslateOnly(m.ExtractTranslation()); return r
        rel = _rig(m1) * _rig(m0).GetInverse()         # 그리퍼 기준 과실 상대포즈(스케일 제거)
        j.CreateLocalPos0Attr().Set(Gf.Vec3f(rel.ExtractTranslation()))
        j.CreateLocalRot0Attr().Set(Gf.Quatf(rel.ExtractRotationQuat().GetNormalized()))
        j.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
        j.CreateLocalRot1Attr().Set(Gf.Quatf(1.0))
        self._grasped_fruit = fruit_path
        return True

    def _attach_scene_fruit(self, request) -> bool:
        """ROS가 선택한 정확한 ripe 과실만 그리퍼에 부착한다.

        fruit_id가 있으면 거리 기반 대체 선택을 절대 하지 않는다. 좌표만 보내는 구형
        클라이언트는 5 cm 안의 과실만 허용해 이웃 식물을 잘못 붙이지 못하게 한다.
        """
        import numpy as np
        from pxr import Gf, Usd, UsdGeom
        if self._task is None or self._stage is None:
            return False
        base_position = request["position"]
        expected_id = request.get("fruit_id")
        if expected_id is not None:
            expected_id = int(expected_id)
        ref = self._stage.GetPrimAtPath(f"{self.root}/Base/base_link")
        matrix = UsdGeom.XformCache().GetLocalToWorldTransform(ref)
        target = np.asarray(matrix.Transform(Gf.Vec3d(*base_position)), dtype=float)
        cache = UsdGeom.XformCache()
        bbox = UsdGeom.BBoxCache(
            Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
        best, best_d = None, float("inf")
        candidates = list(self._task.pickable_fruits())
        # GreenhouseTask.detach_fruit()는 정상적으로 수확한 과실을 pickable 목록에서
        # 제거한다. 예전에는 이때 /sim/tomato도 끊겨, 실제로 스쿱 안에 있어도 ROS가
        # 위치 소실(inf)로 오판했다. 마지막 절단 대상은 동적 강체 상태 그대로 계속 발행한다.
        tracked = self._tracked_harvested_fruit
        if tracked and not any(f.get("path") == tracked for f in candidates):
            candidates.append({
                "path": tracked, "class_name": "ripe", "harvested": True})
        for fruit in candidates:
            if fruit.get("class_name") != "ripe":
                continue
            if expected_id is not None and _fruit_id(fruit["path"]) != expected_id:
                continue
            prim = self._stage.GetPrimAtPath(fruit["path"])
            if not prim.IsValid():
                continue
            body = self._stage.GetPrimAtPath(fruit["path"] + "/Body")
            geom = body if body.IsValid() else prim
            pos = np.asarray(
                bbox.ComputeWorldBound(geom).ComputeAlignedRange().GetMidpoint(),
                dtype=float)
            d = float(np.linalg.norm(pos - target))
            if expected_id is not None or d < best_d:
                best, best_d = fruit, d
                if expected_id is not None:
                    break
        attach_tolerance = float(os.environ.get("SCENE_FRUIT_ATTACH_TOLERANCE_M", "0.05"))
        if best is None:
            print(f"[Grasp] 씬 과실 ID 불일치 — id={expected_id}", flush=True)
            return False
        # ID는 대상을 특정할 뿐, 멀리 날아간 과실을 순간이동시켜 붙이라는 뜻이 아니다.
        # 스쿱 중심 8 cm 밖이면 수용 실패로 판정하고 pedicel 절단도 진행하지 않는다.
        id_attach_tolerance = float(os.environ.get(
            "SCENE_FRUIT_ID_ATTACH_TOLERANCE_M", "0.08"))
        allowed = id_attach_tolerance if expected_id is not None else attach_tolerance
        if best_d > allowed:
            print(f"[Grasp] 씬 과실 못 찾음(최근접 {best_d:.2f}m > "
                  f"{allowed:.2f}m, id={expected_id})", flush=True)
            return False
        if self._attach_prim_to_gripper(best["path"]):
            print(f"[Grasp] attach_fruit(씬) — id={_fruit_id(best['path'])} "
                  f"path={best['path']} 거리={best_d:.3f}m", flush=True)
            return True
        return False

    def _report_grasp_contact(self, request) -> None:
        """좌·우 실제 패드 콜라이더와 GripStem의 기하 간격을 ROS 상태로 보낸다.

        finger_joint 하나만 보면 한쪽 패드/지그에 걸린 것도 파지로 오판한다. 여기서는
        실제 충돌 메시 AABB 사이의 최소 간격을 좌우 각각 계산하고, 패드 중점 대비 줄기의
        상대 위치도 함께 보내 절단 전 양면 접촉과 절단 후 미끄러짐을 검증한다.
        """
        if self._stage is None or self._contact_pub is None:
            return
        try:
            from pxr import Gf, PhysxSchema, Usd, UsdGeom, UsdPhysics

            target = None
            if isinstance(request, dict) and "position" in request and self._task is not None:
                base = self._stage.GetPrimAtPath(f"{self.root}/Base/base_link")
                b2w = UsdGeom.XformCache().GetLocalToWorldTransform(base)
                wanted = np.asarray(
                    b2w.Transform(Gf.Vec3d(*request["position"])), dtype=float)
                best_d = float("inf")
                for fruit in self._task.pickable_fruits():
                    prim = self._stage.GetPrimAtPath(fruit["path"])
                    if not prim.IsValid():
                        continue
                    p = np.asarray(
                        UsdGeom.XformCache().GetLocalToWorldTransform(
                            prim).ExtractTranslation(), dtype=float)
                    d = float(np.linalg.norm(p - wanted))
                    if d < best_d:
                        target, best_d = fruit["path"], d
                if target is not None and best_d <= 0.25:
                    self._contact_fruit = target
                else:
                    # 절단 후 원래 과실은 pickable 목록에서 빠진다. 이때 31cm 옆의 다음
                    # 과실(best)을 그대로 쓰면 "314mm 미끄러짐"으로 오판하므로 버리고,
                    # 절단 전에 저장한 _contact_fruit를 계속 추적한다.
                    target = None
            target = target or self._contact_fruit or self._air_fruit_prim
            stem = self._stage.GetPrimAtPath((target or "") + "/GripStem")
            tcp = self._mm.grasp_tcp_path(self._stage)
            robot = tcp.rsplit("/base_link/", 1)[0] if tcp else None
            if not target or not stem.IsValid() or not robot:
                self._contact_pub.publish(json.dumps({
                    "grasp_contact": False, "contact_reason": "target_or_stem_missing"}))
                return

            def pad(side):
                root = self._stage.GetPrimAtPath(f"{robot}/{side}_inner_finger")
                candidates = [
                    p for p in Usd.PrimRange(root)
                    if "finger4step" in str(p.GetPath()).lower()
                    and (p.HasAPI(UsdPhysics.CollisionAPI)
                         or p.HasAPI(PhysxSchema.PhysxCollisionAPI))]
                return candidates[-1] if candidates else None

            left, right = pad("left"), pad("right")
            if left is None or right is None:
                self._contact_pub.publish(json.dumps({
                    "grasp_contact": False, "contact_reason": "pad_collider_missing"}))
                return

            bb = UsdGeom.BBoxCache(
                Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
            sr = bb.ComputeWorldBound(stem).ComputeAlignedRange()

            def range_data(prim):
                r = bb.ComputeWorldBound(prim).ComputeAlignedRange()
                lo = np.asarray(r.GetMin(), dtype=float)
                hi = np.asarray(r.GetMax(), dtype=float)
                return lo, hi, (lo + hi) * 0.5

            slo, shi = np.asarray(sr.GetMin(), dtype=float), np.asarray(sr.GetMax(), dtype=float)
            sc = (slo + shi) * 0.5

            def gap(prim):
                lo, hi, center = range_data(prim)
                axis_gap = np.maximum(np.maximum(lo - shi, slo - hi), 0.0)
                return float(np.linalg.norm(axis_gap)), center

            lg, lc = gap(left)
            rg, rc = gap(right)
            pc = (lc + rc) * 0.5
            # 패드 contactOffset(3mm) + 상대 콜라이더 contact envelope를 포함한다.
            # 실측 양면 접촉 시 AABB 표면 간격이 5.8~6.1mm이므로 7mm를 사용한다.
            tol = float(os.environ.get("GRIP_CONTACT_GAP_M", "0.007"))
            rel = sc - pc
            payload = {
                "grasp_contact": bool(lg <= tol and rg <= tol),
                "left_gap": round(lg, 5), "right_gap": round(rg, 5),
                "stem_rel": [round(float(v), 5) for v in rel],
            }
            # ROS StringPublisher 브리지 버퍼가 120 byte라 target path 등을 넣으면 JSON이
            # 잘려 수신측 json.loads가 실패한다. 계측 필수값만 보내 120 byte 아래 유지.
            self._contact_pub.publish(json.dumps(payload))
            print(f"[GripContact] L={lg*1000:.1f}mm R={rg*1000:.1f}mm "
                  f"양면={payload['grasp_contact']} rel="
                  f"{tuple(round(float(v)*1000, 1) for v in rel)}mm", flush=True)
        except Exception as exc:
            self._contact_pub.publish(json.dumps({
                "grasp_contact": False, "contact_reason": str(exc)}))
            print(f"[GripContact] 계측 실패: {exc}", flush=True)

    def _publish_sim_tomato(self) -> None:
        """로봇 주변 ripe 과실의 실제 base-frame 좌표를 한 개씩 발행한다."""
        if self._fruit_pub is None or self._stage is None:
            return
        from pxr import Gf, Usd, UsdGeom
        cache = UsdGeom.XformCache()
        if self._air_fruit_prim is not None:      # airfruit — Body bbox 중심 발행(씬과 동일)
            body = self._stage.GetPrimAtPath(self._air_fruit_prim + "/Body")
            base = self._stage.GetPrimAtPath(f"{self.root}/Base/base_link")
            if body.IsValid() and base.IsValid():
                bb = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                                       [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
                w = bb.ComputeWorldBound(body).ComputeAlignedRange().GetMidpoint()
                w2b = cache.GetLocalToWorldTransform(base).GetInverse()
                p = w2b.Transform(Gf.Vec3d(w))
                self._fruit_pub.publish(json.dumps(
                    {"class": "ripe", "fruit_id": _fruit_id(self._air_fruit_prim),
                     "position": [round(float(v), 4) for v in p]},
                    separators=(",", ":")))
            return
        if self._task is None:
            return
        bbox_cache = UsdGeom.BBoxCache(
            Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
        base = self._stage.GetPrimAtPath(f"{self.root}/Base/base_link")
        if not base.IsValid():
            return
        world_to_base = cache.GetLocalToWorldTransform(base).GetInverse()
        nearby = []
        candidates = list(self._task.pickable_fruits())
        # detach_fruit() 뒤에는 정상 수확한 과실이 pickable 목록에서 빠진다. 절단한
        # 대상은 물리 강체로 계속 존재하므로 마지막 대상 경로를 다시 넣어 실제 운반
        # 위치를 발행한다. 이 항목에는 작업영역 필터를 적용하지 않아, 떨어졌을 때도
        # `inf`가 아니라 실제 낙하 좌표/오차로 실패 원인을 확인할 수 있게 한다.
        tracked = self._tracked_harvested_fruit
        if tracked and not any(f.get("path") == tracked for f in candidates):
            candidates.append({
                "path": tracked, "class_name": "ripe", "harvested": True})
        for fruit in candidates:
            if fruit.get("class_name") != "ripe":
                continue
            prim = self._stage.GetPrimAtPath(fruit["path"])
            if not prim.IsValid():
                continue
            # 파지점 = 과실 원점이 아니라 **몸통 메시의 기하 중심**(bbox 미드포인트).
            # 토마토 USD 원점이 중심 정렬 안 돼 있어(00_convert 미centering) 원점을 쓰면
            # 그리퍼가 빗나간다 — 실제 과실 중심을 겨냥해야 파지가 맞는다(2026-07-22).
            body = self._stage.GetPrimAtPath(fruit["path"] + "/Body")
            geom = body if body.IsValid() else prim
            world = bbox_cache.ComputeWorldBound(geom).ComputeAlignedRange().GetMidpoint()
            p = world_to_base.Transform(Gf.Vec3d(world))
            # 매 프레임 수백 개를 순회 발행하지 않고 현재 팔 작업영역만 보낸다.
            is_tracked = fruit["path"] == tracked
            if is_tracked or (
                    0.0 <= p[0] <= 1.5 and abs(p[1]) <= 0.9
                    and 0.1 <= p[2] <= 1.9):
                nearby.append((fruit["path"], [float(v) for v in p]))
        if not nearby:
            return
        nearby.sort(key=lambda item: item[0])
        path, position = nearby[self._fruit_cursor % len(nearby)]
        self._fruit_cursor += 1
        payload = json.dumps({
            "class": "ripe",
            "fruit_id": _fruit_id(path),
            "position": [round(v, 4) for v in position],
        }, separators=(",", ":"))
        self._fruit_pub.publish(payload)

    def _toggle_foliage(self, visible: bool) -> None:
        """잎(Foliage) 프림 표시/숨김 — 카메라 가림 확인용 런타임 토글."""
        from pxr import UsdGeom
        if self._stage is None:
            return
        n = 0
        for p in self._stage.Traverse():
            if p.GetName() == "Foliage":
                img = UsdGeom.Imageable(p)
                (img.MakeVisible if visible else img.MakeInvisible)()
                n += 1
        print(f"[Foliage] 잎 {'표시' if visible else '숨김'} ({n}개)")

    def _handle_cut(self, request) -> None:
        """닫힌 커터 근처 ripe 과실의 pedicel FixedJoint를 해제한다."""
        if not isinstance(request, dict) or self._task is None or self._stage is None:
            return
        try:
            cut_id = int(request["id"])
            base_position = np.asarray(request["position"], dtype=float)
            tolerance = float(request.get("max_distance", 0.10))
            expected_id = request.get("fruit_id")
            if expected_id is not None:
                expected_id = int(expected_id)
            if base_position.shape != (3,) or not np.all(np.isfinite(base_position)):
                raise ValueError("position은 유한한 xyz여야 함")
        except (KeyError, TypeError, ValueError) as exc:
            print(f"[Cutter] 잘못된 절단 요청 무시: {exc}")
            return

        from pxr import Gf, Usd, UsdGeom
        reference = self._stage.GetPrimAtPath(f"{self.root}/Base/base_link")
        matrix = UsdGeom.XformCache().GetLocalToWorldTransform(reference)
        target_world = np.asarray(
            matrix.Transform(Gf.Vec3d(*base_position)), dtype=float)
        nearest = None
        nearest_distance = float("inf")
        cache = UsdGeom.XformCache()
        bbox = UsdGeom.BBoxCache(
            Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
        for fruit in self._task.pickable_fruits():
            if fruit.get("class_name") != "ripe":
                continue
            if expected_id is not None and _fruit_id(fruit["path"]) != expected_id:
                continue
            prim = self._stage.GetPrimAtPath(fruit["path"])
            if not prim.IsValid():
                continue
            body = self._stage.GetPrimAtPath(fruit["path"] + "/Body")
            geom = body if body.IsValid() else prim
            position = np.asarray(
                bbox.ComputeWorldBound(geom).ComputeAlignedRange().GetMidpoint(),
                dtype=float)
            distance = float(np.linalg.norm(position - target_world))
            if expected_id is not None or distance < nearest_distance:
                nearest, nearest_distance = fruit, distance
                if expected_id is not None:
                    break
        q = np.asarray(self.robot.get_joint_positions(), dtype=float)
        blade_deg = float(np.degrees(q[self._scoop_indices[2]]))
        blade_closed = blade_deg >= self._mm.BLADE_CLOSED_DEG - 1.0
        # ID는 대상 식별용일 뿐 수용 성공의 증거가 아니다. 예전 0.45m 허용 때문에
        # 이미 스쿱 밖으로 밀려난 같은 ID 과실도 절단 성공으로 처리됐다. 요청 좌표
        # 주변의 실제 수용 범위 안에 있을 때만 pedicel을 해제한다.
        id_cut_tolerance = float(os.environ.get(
            "SCENE_FRUIT_ID_CUT_TOLERANCE_M", "0.07"))
        allowed_distance = id_cut_tolerance if expected_id is not None else tolerance
        success = bool(nearest is not None and nearest_distance <= allowed_distance
                       and blade_closed and self._task.detach_fruit(nearest["path"]))
        if success:
            self._tracked_harvested_fruit = nearest["path"]
        self._cut_status = {
            "cut_id": cut_id, "cut_success": success,
            "cut_distance": None if nearest is None else nearest_distance,
            "fruit_path": "" if nearest is None else nearest["path"]}
        print(f"[Cutter] pedicel joint {'해제' if success else '실패'}: "
              f"cut_id={cut_id}, fruit_id={expected_id}, "
              f"path={'' if nearest is None else nearest['path']}, "
              f"distance={nearest_distance:.3f}m, "
              f"blade_closed={blade_closed}")

    def _drive_base(self) -> None:
        """/cmd_vel(vx, vy, wz) 을 더미 3축에 적분해 넣는다 — 홀로노믹 베이스의 '주행'.

        왜 적분인가 — 이 베이스는 속도/위치 드라이브를 무시하고 텔레포트만 먹는다
        (2026-07-18 실측). 그래서 Isaac 이 속도를 위치로 바꿔 매 프레임 새 포즈를 찍는다.
        vx/vy 는 **로봇 기준**(Twist 규약)이라 yaw 로 월드에 회전시켜 더한다.
        적분 상태를 따로 안 들고 매 프레임 조인트를 읽는 이유 — Play/Stop 리셋이나 JSON
        base 텔레포트로 조인트가 바뀌어도 자동으로 그 자리에서 이어간다(상태 두 벌 금지).
        """
        vx, vy, wz = self._twist.poll()
        nav = self._cfg.robots.harvester_nav
        vx = max(-nav.max_vx, min(nav.max_vx, vx))   # Nav2 가 상한을 어겨도 여기서 막는다
        vy = max(-nav.max_vy, min(nav.max_vy, vy))
        wz = max(-nav.max_wz, min(nav.max_wz, wz))
        if vx == 0.0 and vy == 0.0 and wz == 0.0:
            return
        x, y, yaw = np.asarray(
            self.robot.get_joint_positions(), dtype=float)[self._base_idx]
        c, s = math.cos(yaw), math.sin(yaw)
        self.robot.set_joint_positions(
            np.array([x + (vx * c - vy * s) * self._dt,
                      y + (vx * s + vy * c) * self._dt,
                      yaw + wz * self._dt]),
            joint_indices=self._base_idx)


def build_nav(stage, mm, nav, opts):
    """수확 MM 자율주행 그래프 — 플래그로 켠 것만. 실패해도 씬은 유지(iw.py 와 동일 방침).

    반환: /cmd_vel 폴러 (nav_drive 를 안 켰으면 None).
      drive : /harvester_0/cmd_vel 구독만. 실행(적분)은 MMDriver._drive_base 가 한다.
      odom  : /harvester_0/odom + TF harvester_0/odom→harvester_0/base_link.
              ⚠ 섀시가 키네마틱이라 IsaacComputeOdometry 의 **속도**는 0 으로 나올 수 있다.
                AMCL/Nav2 가 실제로 쓰는 건 TF·위치라 주행 자체엔 문제없지만, odom twist 를
                보고 판단하는 노드를 붙일 땐 확인할 것.
      scan  : RTX 라이다 → /harvester_0/scan + base_link→laser TF.
    """
    from ros import robot_bridge as RB

    poller = None
    chassis = mm.chassis_path
    if not chassis or not stage.GetPrimAtPath(chassis).IsValid():
        chassis = mm.root
    # ★나브 그래프 프림도 ns 접미(2026-07-23): 하드코딩 /World/HarvNav_* 는 팀원과 겹친다.
    sfx = f"_{nav.tf_namespace}" if nav.tf_namespace else ""
    try:
        if opts.nav_drive:
            sub = RB.build_twist_sub(f"/World/HarvNav_drive{sfx}", nav.cmd_vel_topic)
            poller = RB.TwistPoller(sub)
        if opts.nav_odom:
            RB.build_odometry(stage, f"/World/HarvNav_odom{sfx}", chassis, nav)
        if opts.nav_scan:
            lidar = mm.attach_lidar(stage, nav.lidar_offset)
            if lidar:
                RB.build_tf_sensor(stage, f"/World/HarvNav_tf{sfx}", chassis, lidar, nav)
                RB.build_lidar_scan(stage, f"/World/HarvNav_scan{sfx}", lidar, nav)
    except Exception:
        import traceback
        print("\n" + "=" * 64)
        print("[Nav] MM 그래프 생성 실패 — 씬은 유지. tools/nav2_node_probe.py 로 노드명 확인.")
        print("=" * 64)
        traceback.print_exc()
        print("=" * 64 + "\n")
    return poller


def build_teleop(mm_robot, set_blade, gui: bool):
    """MM 키보드 텔레옵 — 팔6·베이스·동축 스쿱. 반환: step(is_playing) 콜백.

    글자키만 쓴다(방향키는 뷰포트가 가로챔 — spike05 실측). GUI 전용. mm_robot 이 물리
    초기화(world.reset)된 뒤 호출할 것 — HarvesterController 가 현재 관절값에서 출발한다.
    set_blade: 가동날 각도[deg] setter 콜백 (조립모드=mm.set_blade_deg / 로드모드=드라이브 attr).
    """
    if not gui:
        print("[Teleop] --headless 라 키보드 입력 불가 — 텔레옵 비활성")
        return None
    import carb.input
    import omni.appwindow

    from robots.control import HarvesterController

    ctrl = HarvesterController(mm_robot)
    K = carb.input.KeyboardInput
    pressed: set = set()
    st = {"blade": 0.0}
    active = {"joint": 0}                           # 번호키로 선택된 팔 관절(0~5)
    ARM_SEL = [K.KEY_1, K.KEY_2, K.KEY_3, K.KEY_4, K.KEY_5, K.KEY_6]

    def on_key(e, *_):
        if e.type == carb.input.KeyboardEventType.KEY_PRESS:
            if e.input in ARM_SEL:                   # 번호키 = 조작할 관절 선택(엣지)
                active["joint"] = ARM_SEL.index(e.input)
                print(f"[Teleop] 활성 관절 = {active['joint'] + 1}번")
            elif e.input == K.N:                    # 한 번 누르면 절삭 스윙
                st["blade"] = 50.0
                set_blade(st["blade"])
                print("[Cutter] 외측 1/4구 절삭: 0° → 50°")
            elif e.input == K.B:                    # 한 번 누르면 재개방
                st["blade"] = 0.0
                set_blade(st["blade"])
                print("[Cutter] 외측 1/4구 수용 위치: 50° → 0°")
            else:
                pressed.add(e.input)
        elif e.type == carb.input.KeyboardEventType.KEY_RELEASE:
            pressed.discard(e.input)
        return True

    appwin = omni.appwindow.get_default_app_window()
    carb.input.acquire_input_interface().subscribe_to_keyboard_events(
        appwin.get_keyboard(), on_key)

    DQ, DB, DYAW, DG = 0.02, 0.01, 0.02, 0.03
    print("""
[MM 텔레옵] 플레이 상태에서 (방향키는 뷰포트가 가로챔 — 숫자/글자키만)
  팔    숫자 1~6 으로 관절 선택 → , 반시계 / . 시계 로 그 관절 회전
  베이스 I/K 전후 · J/L 제자리 회전 (옆 이동 없음: 회전 후 전진)
  스쿱   Z 열기 / X 수용닫기   커터 B 수용(0°) / N 절단(+50°)
""")

    def step(is_playing):
        if not is_playing:
            return
        j = active["joint"]                         # 선택된 관절만 회전
        if K.COMMA in pressed:                       # , = 반시계(CCW, +)
            ctrl.move_arm(j, DQ)
        if K.PERIOD in pressed:                      # . = 시계(CW, −)
            ctrl.move_arm(j, -DQ)
        forward = (K.I in pressed) - (K.K in pressed)
        dyaw = (K.J in pressed) - (K.L in pressed)
        if forward or dyaw:
            ctrl.move_base_forward(forward * DB, dyaw * DYAW)
        if K.Z in pressed:
            ctrl.move_gripper(-DG)
        if K.X in pressed:
            ctrl.move_gripper(DG)
        ctrl.apply()

    return step


def find_blade_setter(stage):
    """로드된 USD 의 가동날 ServoJoint 드라이브 타깃 attr → 각도 setter. 없으면 no-op."""
    from pxr import UsdPhysics
    sj = stage.GetPrimAtPath("/World/Harvester_CutterBlade/ServoJoint")
    if sj and sj.IsValid():
        attr = UsdPhysics.DriveAPI(sj, "angular").GetTargetPositionAttr()
        if attr and attr.IsValid():
            return lambda deg: attr.Set(float(deg))
    return lambda deg: None
