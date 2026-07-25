# -*- coding: utf-8 -*-
"""MoveIt 기반 수확 MM 드라이버 (--mm) — Ridgeback + m0617 + 동축 스쿱 + D455.

전신(前身)은 RMPflow 기반 `rmp_mm.py` 였다. 2026-07-24 사용자 지시로 제어를 MoveIt 로
갈아끼우고 이름을 `mm` 로 바꿨다(rmp_mm 스택은 제자리 교체돼 사라짐):

  팔     m0617 그대로 (moveit_mm 의 UR10e 와 다른 점은 여기뿐 — "팔만 갈아끼워")
  그리퍼     동축 3축 1/4구 스쿱 (moveit_mm 과 동일 에셋)
  제어   RMPflow(Isaac 내부 반응형) → MoveIt(ROS2 가 계획, /joint_command 스트리밍)

로봇 모델은 robots/harvester_mm.py, ROS 브리지는 ros/robot_bridge.py. 이 파일은 그 둘을
'배선'하고 텔레옵/JSON 명령을 매 프레임 적용한다(§5.6 실행측). 판단은 ROS2 가 한다.

★MoveIt 설정은 MM 마다 완전히 분리한다(2026-07-24 사용자). mm = src/smartfarm/mm_moveit,
  moveit_mm = src/smartfarm/harvest_moveit. 네임스페이스도 harvester_0 로 갈라 둔다.
"""
from __future__ import annotations

import json
import math
import os

import numpy as np
from isaacsim.core.utils.types import ArticulationAction

from robot_base import Driver, ros_fail
from robots.harvester_mm import HOME_POSE_DEG, HarvestMM
from pjt_config.settings_mm import mm_robot_config
from scene.ground import COMMON_FLOOR_Z

# 임시 배치 — 온실 앞마당(빈 홀 바닥, 온실 y −10 앞). 물류 동선 확정 후 조정.
POSE = (0.0, -12.0, COMMON_FLOOR_Z)
# 키네마틱 베이스 조인트 (JSON base 명령용 — 텔레포트만 먹는다, 2026-07-18 실측)
BASE_JOINTS = ("dummy_base_prismatic_x_joint",
               "dummy_base_prismatic_y_joint",
               "dummy_base_revolute_z_joint")


class MMDriver(Driver):
    flag = "--mm"
    name = "mm"
    # ★MoveIt 격리(2026-07-24): moveit_mm(harvester_moveit)·팀원 RMP(harvester_0)와
    #   토픽이 겹치면 두 move_group 이 서로의 /joint_command 를 먹는다. mm 전용 ns.
    ns = "harvester_0"
    root = "/World/Harvester"

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
        self._mm = HarvestMM(mm_robot_config(cfg.robots))
        self._base_idx = None
        self._scoop_idx = None                 # 스쿱 3축 dof 인덱스
        self._scoop_target = self.SCOOP_OPEN.copy()
        self._gripper_closed = False           # 스쿱 개폐 상태
        self._pending_grasp_check = None
        self._poller = None
        self._status_pub = None
        self._fruit_pub = None
        self._fruit_cursor = 0
        self._teleop = None
        self._stage = None
        self._twist = None                    # /cmd_vel 폴러 (Nav2 주행)
        self._nav = cfg.robots.harvester_nav   # finalize 에서 ns 격리 사본으로 덮어씀
        self._dt = 1.0 / 60.0                 # finalize 에서 월드 실제 물리 dt 로 덮어씀
        self._was_playing = False
        self._grasp_status = {}
        self._cut_status = {}
        self._follow_status = {}
        self._tcp_status = {}
        self._verified_fruit_path: str | None = None
        # 레거시 follow_check용 상대 위치 기준값. 과실을 부착하거나 위치 보정하는 데
        # 사용하지 않고, 스쿱 안에서 물리적으로 함께 이동했는지만 측정한다.
        self._grasp_tcp_offset: np.ndarray | None = None
        self._base_moving = False
        self._arm_idx = None                  # 팔 6축 dof 인덱스 (finalize 에서 채움)
        self._arm_home_q = None               # 홈 관절각(rad)

    def spawn(self, stage):
        self._mm.spawn(stage, self.root, POSE)

    def configure(self, world):
        # 시작자세(HOME_POSE_DEG)는 아티큘레이션 관절 기본값으로 한 번만 적용한다.
        # 링크 xform에도 같은 자세를 굽으면 Lula URDF와 실제 PhysX 체인이 어긋난다.
        r = self.robot
        q0 = np.asarray(r.get_joint_positions(), dtype=float)
        names = list(r.dof_names)
        for jname, deg in HOME_POSE_DEG:
            q0[names.index(jname)] = np.radians(deg)
        r.set_joints_default_state(positions=q0)

    def finalize(self, world, stage, opts):
        self._stage = stage
        # 키네마틱 베이스 인덱스 (JSON base 명령용)
        r = self.robot
        self._base_idx = np.array(
            [list(r.dof_names).index(n) for n in BASE_JOINTS])
        self._scoop_idx = np.array(
            [list(r.dof_names).index(n) for n in self.SCOOP_JOINTS])
        # m0617 6축 position drive 게인 강화 — MoveIt 궤적 추종 시 처짐 방지 + **이동(nav) 중
        # 대기자세를 단단히 유지해 흔들리지 않게**(2026-07-22 사용자). 하한 상향.
        arm_indices = np.array([
            list(r.dof_names).index(name) for name, _ in HOME_POSE_DEG])
        self._arm_idx = arm_indices
        self._arm_home_q = np.radians(
            np.array([deg for _, deg in HOME_POSE_DEG], dtype=float))
        controller = r.get_articulation_controller()
        kps, kds = controller.get_gains()
        if hasattr(kps, "cpu"):
            kps = kps.cpu().numpy()
        if hasattr(kds, "cpu"):
            kds = kds.cpu().numpy()
        kps = np.asarray(kps, dtype=float).copy()
        kds = np.asarray(kds, dtype=float).copy()
        # 주행 중 팔이 base 가속에 밀려 미세하게 흐느적거리지 않도록 6축을 더 단단히
        # 잡는다. 환경변수로 현장 튜닝은 가능하게 두되 기존 하한(1e5/1e4)보다 기본값을
        # 2배 올린다. 원본 USD import 기본값(1e7/1e5)보다는 충분히 낮아 발산 여유가 있다.
        arm_kp = float(os.environ.get("MM_ARM_KP", "200000"))
        arm_kd = float(os.environ.get("MM_ARM_KD", "20000"))
        kps[arm_indices] = np.maximum(kps[arm_indices] * 2.0, arm_kp)
        kds[arm_indices] = np.maximum(kds[arm_indices] * 2.0, arm_kd)
        # 세 동축 회전축은 같은 낮은 position gain 으로 구동해 중첩 셸의 접촉 폭발을 막는다
        # (moveit_mm 과 동일 근거·동일 env 이름 — 스쿱 에셋이 같다).
        gi = self._scoop_idx
        kps[gi] = float(os.environ.get("SCOOP_KP", "1200"))
        kds[gi] = float(os.environ.get("SCOOP_KD", "80"))
        controller.set_gains(kps=kps, kds=kds)
        try:
            efforts = controller.get_max_efforts()
            if hasattr(efforts, "cpu"):
                efforts = efforts.cpu().numpy()
            efforts = np.asarray(efforts, dtype=float).copy()
            # ★파지력 상한 — 강체 과조임 반발력으로 튕기는 것 방지. 논문: F≥mg/(2μ)≈0.65N.
            efforts[gi] = float(os.environ.get("SCOOP_EFFORT", "40.0"))
            controller.set_max_efforts(values=efforts)
        except Exception as exc:
            print(f"[MM] ⚠ 그리퍼 effort 상한 실패(계속 진행): {exc}")
        print(f"[MM] m0617 drive gain 강화: kp={kps[arm_indices].tolist()} "
              f"kd={kds[arm_indices].tolist()}")
        print(f"[MM] 동축 스쿱 3축 위치제어: kp={kps[gi].tolist()} "
              f"joints={list(self.SCOOP_JOINTS)}")

        if not opts.no_ros:
            try:
                from ros import robot_bridge as RB
                RB.build_joint_bridge(stage, f"/World/RosBridge_{self.ns}",
                                      self.ns, self.art,
                                      # MoveIt 전환(2026-07-24): /joint_command 를 항상 직접
                                      # 적용한다(topic_based_ros2_control 이 이 토픽으로 구동).
                                      apply_commands=True,
                                      # ★HW 상태채널 분리: JSB 가 /{ns}/joint_states 로 arm 만
                                      # 발행 → Isaac 전체관절과 겹치면 move_group 이 dummy_base
                                      # 관절을 못 찾아 에러. Isaac 은 hw_joint_states 로 발행.
                                      states_topic=f"/{self.ns}/hw_joint_states")
                sub = RB.build_string_sub(
                    f"/World/RosCmd_{self.ns}", f"/{self.ns}/cmd")
                self._poller = RB.StringPoller(sub)
                pub = RB.build_string_pub(
                    f"/World/RosMmStatus_{self.ns}",
                    f"/{self.ns}/status")
                self._status_pub = RB.StringPublisher(pub)
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
                # ★nav 격리(2026-07-24): 공유 harvester_nav 는 전역(/cmd_vel, tf_namespace="")
                #   이라 팀원 RMP MM·moveit_mm 과 겹친다. mm 는 자기 ns 로 오버라이드하고
                #   _drive_base() 도 반드시 이 사본을 써야 한다(원본은 max_vy=0 이라 안 움직임).
                import dataclasses as _dc
                self._nav = _dc.replace(
                    self._cfg.robots.harvester_nav,
                    tf_namespace=self.ns,
                    # Ridgeback은 기존 MM 규약대로 x 전진/후진 + z 회전만 허용한다.
                    # y 횡이동은 Nav2 설정과 Isaac 적용부 양쪽에서 차단한다.
                    max_vx=0.8,
                    max_vy=0.0,
                    max_wz=8.0,
                    # MoveIt URDF의 이동 베이스 루트. robot_state_publisher가
                    # mm_base→base_link(팔 장착 프레임)를 이어 주므로 Nav2 odom/scan도
                    # mm_base에 물려야 하나의 TF 트리가 된다.
                    base_frame="mm_base",
                    cmd_vel_topic=f"/{self.ns}/cmd_vel_safe",
                    odom_topic=f"/{self.ns}/odom",
                    scan_topic=f"/{self.ns}/scan")
                self._twist = build_nav(stage, self._mm, self._nav, opts)

        # RMPflow 는 MoveIt 전환(2026-07-24)으로 제거했다. 팔은 move_group 이
        # /{ns}/joint_command 로 직접 구동한다(topic_based_ros2_control).
        if opts.rmpflow:
            print("[MM] --rmpflow 는 mm 의 MoveIt 전환으로 비활성(무시)")

        if opts.mm_teleop:
            self._teleop = build_teleop(r, opts.gui)
        self._setup_foliage_key(opts.gui)

    def _setup_foliage_key(self, gui: bool) -> None:
        """F 키로 잎(Foliage) 표시/숨김 토글 — 자율(rmpflow) 모드에서도 동작.
        카메라가 잎에 가려 토마토를 못 볼 때 눌러서 잎을 치운다(2026-07-23 사용자)."""
        if not gui:
            return
        import carb.input
        import omni.appwindow
        self._foliage_visible = True

        def on_key(e, *_):
            if (e.type == carb.input.KeyboardEventType.KEY_PRESS
                    and e.input == carb.input.KeyboardInput.F):
                self._foliage_visible = not self._foliage_visible
                self._toggle_foliage(self._foliage_visible)
            return True

        appwin = omni.appwindow.get_default_app_window()
        carb.input.acquire_input_interface().subscribe_to_keyboard_events(
            appwin.get_keyboard(), on_key)
        print("[Foliage] F 키 = 잎 표시/숨김 토글")

    def _build_camera(self, stage):
        cam_prim = self._mm.camera_path(stage)
        if not cam_prim:
            print("[Camera] D455 카메라 prim 못 찾음 — rgb/depth 발행 스킵")
            return
        try:
            from ros import robot_bridge as RB
            import dataclasses as _dc

            # 수확 파이프라인은 harvester_0 namespace 안에서 rgb/depth/camera_info를
            # 구독한다. 공용 카메라 설정의 구형 namespace를 그대로 쓰면
            # 디버그 창은 열려도 영상 콜백이 한 번도 오지 않는다.
            cam = _dc.replace(
                self._cfg.robots.camera,
                node_namespace=self.ns,
                rgb_topic="rgb",
                depth_topic="depth",
                info_topic="camera_info",
                frame_id=f"{self.ns}_d455_color_optical_frame")
            RB.build_camera(stage, "/World/RosCamera", cam_prim, cam)
            # MoveIt mm_manipulator의 planning frame `base_link`는 Ridgeback 섀시가
            # 아니라 0.30m 위 m0617 베이스다. 카메라 TF도 실제 Arm/base_link에서
            # 계산해야 비전 목표 높이가 MoveIt 목표와 30cm 어긋나지 않는다.
            RB.build_camera_optical_tf(
                stage, "/World/RosCameraTf",
                self._manipulator_base_path(stage), cam_prim,
                cam.frame_id, tf_topic=f"/{self.ns}/tf")
        except Exception:
            import traceback
            print("\n[Camera] 그래프 생성 실패 — 씬 유지. probe 로 노드명 확인.")
            traceback.print_exc()

    def _view_ready(self) -> bool:
        """물리 시뮬레이션 뷰가 준비됐는지. Stop→Play 재초기화 전엔 get_joint_positions 가
        경고만 내고 None 을 반환(크래시 X)하므로, 이걸 게이트로 쓴다."""
        try:
            return self.robot.get_joint_positions() is not None
        except Exception:
            return False

    def update(self, is_playing):
        # ★ Stop→Play 직후엔 물리뷰가 재생성되기 전이라, 이때 아티큘레이션(RMPflow·관절값)에
        #   접근하면 tensors 플러그인이 무효 뷰를 건드려 아이작심이 통째로 죽는다(2026-07-22
        #   로그: "Physics Simulation View is not created yet"). 뷰가 준비될 때까지 프레임을
        #   건너뛴다. _was_playing 은 갱신 안 해서 준비된 뒤 리셋 트리거가 살아 있게 한다.
        if is_playing and not self._view_ready():
            return
        if is_playing and not self._was_playing:
            self._gripper_closed = False
            self._pending_grasp_check = None
            self._cut_status = {}
            self._apply_scoop()
        self._was_playing = is_playing
        if self._teleop is not None:                 # 키보드 텔레옵 (재생 중에만 적용)
            self._teleop(is_playing)
        if is_playing and self._twist is not None:   # Nav2 /cmd_vel → 홀로노믹 베이스
            self._drive_base()
        # MM JSON 명령 — 재생 중에만 적용
        if is_playing and self._poller is not None:
            raw = self._poller.poll()
            if raw:
                try:
                    cmd = json.loads(raw)
                except ValueError:
                    cmd = None
                if isinstance(cmd, dict):
                    if "base" in cmd:
                        b = [float(v) for v in cmd["base"]]
                        if len(b) == 3:
                            self.robot.set_joint_positions(
                                np.array(b), joint_indices=self._base_idx)
                    if "tcp_check" in cmd:
                        request = cmd["tcp_check"]
                        check_id = (int(request.get("id", 0))
                                    if isinstance(request, dict) else 0)
                        tcp_world = self._tcp_world()
                        base = self._stage.GetPrimAtPath(
                            self._manipulator_base_path(self._stage))
                        if tcp_world is not None and base.IsValid():
                            from pxr import Gf, UsdGeom
                            world_to_base = UsdGeom.XformCache().GetLocalToWorldTransform(
                                base).GetInverse()
                            tcp_base = world_to_base.Transform(Gf.Vec3d(*tcp_world))
                            self._tcp_status = {
                                "tcp_id": check_id,
                                "x": round(float(tcp_base[0]), 5),
                                "y": round(float(tcp_base[1]), 5),
                                "z": round(float(tcp_base[2]), 5),
                            }
                    if "gripper" in cmd and isinstance(cmd["gripper"], dict):
                        # 스쿱 개폐. 과실은 강제 부착하지 않으며, 절단 후 중력과
                        # 충돌에 의해 스쿱 안에 담기고 OPEN 시 물리적으로 배출된다.
                        self._gripper_closed = bool(
                            cmd["gripper"].get("closed", False))
                        print(f"[Scoop] {'CLOSE' if self._gripper_closed else 'OPEN'}")
                        self._apply_scoop()
                        if not self._gripper_closed:
                            self._pending_grasp_check = None
                            self._cut_status = {}
                    if "blade" in cmd:
                        # 수용부 두 축은 닫힌 상태를 유지하고 외측 커터만 0~50° 구동한다.
                        blade_deg = max(
                            0.0, min(50.0, float(cmd["blade"])))
                        if self._gripper_closed:
                            target = self.SCOOP_CLOSED.copy()
                            target[2] = math.radians(blade_deg)
                        else:
                            # 다음 과실 접근 전 OPEN 명령에서는 세 셸을 모두 원위치로
                            # 복귀시킨다. blade=0이 수용부를 다시 닫아버리면 안 된다.
                            target = self.SCOOP_OPEN.copy()
                        self._scoop_target = target
                        # 새 칼날 동작이 시작되면 이전 단계 응답이 일반 blade 상태를
                        # 가리지 않도록 비운다.
                        self._grasp_status = {}
                        self._cut_status = {}
                        print(f"[Scoop] BLADE {blade_deg:.1f}deg")
                    if "grasp_check" in cmd:
                        self._handle_grasp_check(cmd["grasp_check"])
                    if "cut_fruit" in cmd:
                        self._handle_cut(cmd["cut_fruit"])
                    if "follow_check" in cmd:
                        self._handle_follow_check(cmd["follow_check"])
                    if "foliage" in cmd:
                        self._toggle_foliage(bool(cmd["foliage"]))
        # MoveIt topic_based_ros2_control이 팔 명령을 계속 스트리밍한다. 스쿱 명령을
        # 한 프레임만 적용하면 다음 articulation update에서 사라지므로 원본
        # moveit_mm과 동일하게 3축 위치 목표를 매 물리 프레임 다시 건다.
        if is_playing and self._scoop_idx is not None:
            self.robot.apply_action(ArticulationAction(
                joint_positions=self._scoop_target.copy(),
                joint_indices=self._scoop_idx))
        # 상태 발행 — 팔 모션 상태는 MoveIt(ROS2)이 자기 액션 피드백으로 안다. Isaac 은
        # 시뮬레이터만 아는 것(파지 검증·TCP 실측·그리퍼)만 돌려준다.
        # Isaac 5.1 generic ROS2Publisher 의 std_msgs/String data 는 약 128 byte 에서
        # "..." 로 잘리므로 payload 를 짧게 유지한다(잘린 JSON 은 파싱 불가 → 전 동작 타임아웃).
        if is_playing and self._status_pub is not None:
            if self._tcp_status:
                wire_status = dict(self._tcp_status)
            elif self._grasp_status:
                wire_status = dict(self._grasp_status)
            elif self._cut_status:
                wire_status = dict(self._cut_status)
            elif self._follow_status:
                wire_status = dict(self._follow_status)
            else:
                q = np.asarray(self.robot.get_joint_positions(), dtype=float)
                wire_status = {
                    "gripper": 1.0 if self._gripper_closed else 0.0,
                    "blade": round(float(np.degrees(q[self._scoop_idx[2]])), 2),
                }
            payload = json.dumps(wire_status, separators=(",", ":"))
            if len(payload.encode("utf-8")) > 120:
                # 향후 필드가 늘어도 조용히 다시 JSON을 깨뜨리지 않는다.
                print(f"[MM] 상태 payload 초과({len(payload)}): {payload}")
            else:
                self._status_pub.publish(payload)
                # 진단 응답은 한 번만 보내 일반 상태 publication 을 막지 않는다.
                if self._tcp_status:
                    self._tcp_status = {}
        # ★팔 홈 고정 없음(2026-07-24): rmp_mm 은 주행 중 홈 관절각을 매 프레임 위치
        #   타깃으로 재지정했다. mm 는 MoveIt 이 /joint_command 로 팔을 몰기 때문에
        #   같은 짓을 하면 두 명령이 매 프레임 싸워 궤적이 끊긴다.
        if is_playing:
            self._publish_sim_tomato()

    def _fruit_center_world(self, path: str) -> np.ndarray | None:
        from pxr import Usd, UsdGeom
        prim = self._stage.GetPrimAtPath(path)
        if not prim.IsValid():
            return None
        body = self._stage.GetPrimAtPath(path + "/Body")
        geom = body if body.IsValid() else prim
        bbox = UsdGeom.BBoxCache(
            Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
        return np.asarray(
            bbox.ComputeWorldBound(geom).ComputeAlignedRange().GetMidpoint(), dtype=float)

    def _tcp_world(self) -> np.ndarray | None:
        from pxr import UsdGeom
        path = self._mm.grasp_tcp_path(self._stage)
        if not path:
            return None
        prim = self._stage.GetPrimAtPath(path)
        if not prim.IsValid():
            return None
        return np.asarray(
            UsdGeom.XformCache().GetLocalToWorldTransform(prim).ExtractTranslation(),
            dtype=float)

    def _nearest_ripe(self, base_position: np.ndarray):
        from pxr import Gf, UsdGeom
        reference = self._stage.GetPrimAtPath(
            self._manipulator_base_path(self._stage))
        target_world = np.asarray(
            UsdGeom.XformCache().GetLocalToWorldTransform(reference).Transform(
                Gf.Vec3d(*base_position)), dtype=float)
        nearest, nearest_center, nearest_distance = None, None, float("inf")
        for fruit in self._task.pickable_fruits():
            if fruit.get("class_name") != "ripe":
                continue
            center = self._fruit_center_world(fruit["path"])
            if center is None:
                continue
            distance = float(np.linalg.norm(center - target_world))
            if distance < nearest_distance:
                nearest, nearest_center, nearest_distance = fruit, center, distance
        return nearest, nearest_center, nearest_distance

    def _ripe_by_id(self, fruit_id: int):
        """선택 때 잠근 과실 ID를 같은 USD prim으로 해석한다."""
        if self._task is None:
            return None, None
        for fruit in self._task.pickable_fruits():
            if (fruit.get("class_name") == "ripe"
                    and int(fruit.get("id", -1)) == fruit_id):
                return fruit, self._fruit_center_world(fruit["path"])
        return None, None

    def _handle_grasp_check(self, request) -> None:
        """스쿱이 닫힌 상태에서 선택한 과실이 수용 범위 안에 있는지만 확인한다."""
        self._follow_status = {}
        try:
            check_id = int(request["id"])
            fruit_id = int(request["fruit_id"])
            base_position = np.asarray(request["position"], dtype=float)
            max_distance = float(request.get("max_distance", 0.06))
            if base_position.shape != (3,):
                raise ValueError("position shape")
        except (KeyError, TypeError, ValueError):
            return
        # 좌표로 nearest를 다시 고르면 같은 화방의 이웃 과실로 바뀔 수 있다. 비전-GT
        # 매칭 순간 확정한 ID만 사용해 접근/접촉/분리를 하나의 과실에 묶는다.
        if fruit_id >= 0:
            fruit, center = self._ripe_by_id(fruit_id)
        else:
            fruit, center, _ = self._nearest_ripe(base_position)
        tcp = self._tcp_world()
        fruit_path = "" if fruit is None else fruit["path"]
        distance = (float("inf") if center is None or tcp is None
                    else float(np.linalg.norm(center - tcp)))
        # 수용부 닫힘 + TCP 근접만 확인한다. 과실을 TCP로 스냅하거나 FixedJoint로
        # 붙이지 않는다. 꽃자루는 외측 칼날 50° 도달을 확인한 _handle_cut에서만 끊는다.
        grasp_confirmed = bool(distance <= max_distance and self._gripper_closed)
        # 여기서는 수용 위치만 확인한다. 꽃자루 해제는 외측 칼날이 실제 50°에
        # 도달한 뒤 별도 cut_fruit 단계에서만 수행한다.
        success = bool(grasp_confirmed and fruit is not None)
        self._grasp_tcp_offset = (
            center - tcp if success and center is not None and tcp is not None
            else None
        )
        self._verified_fruit_path = fruit_path if success else None
        self._grasp_status = {
            "grasp_id": check_id, "ok": success,
            "fruit_id": fruit_id,
            "d": 999.0 if not np.isfinite(distance) else round(distance, 4),
        }
        print(f"[Scoop] grasp id={check_id} fruit_id={fruit_id} "
              f"tcp_distance={distance:.4f} closed={int(self._gripper_closed)} "
              f"forced_joint=none pedicel=intact success={success}")

    def _handle_cut(self, request) -> None:
        """외측 칼날 닫힘과 동일 과실 수용을 확인한 뒤에만 꽃자루를 해제한다."""
        if not isinstance(request, dict) or self._task is None:
            return
        try:
            cut_id = int(request["id"])
            fruit_id = int(request["fruit_id"])
            max_distance = float(request.get("max_distance", 0.09))
        except (KeyError, TypeError, ValueError):
            return
        if fruit_id >= 0:
            fruit, center = self._ripe_by_id(fruit_id)
        else:
            fruit = next(
                (item for item in self._task.pickable_fruits()
                 if item.get("path") == self._verified_fruit_path),
                None,
            )
            center = (
                None if fruit is None
                else self._fruit_center_world(fruit["path"])
            )
        tcp = self._tcp_world()
        fruit_path = "" if fruit is None else fruit["path"]
        distance = (float("inf") if center is None or tcp is None
                    else float(np.linalg.norm(center - tcp)))
        q = np.asarray(self.robot.get_joint_positions(), dtype=float)
        blade_deg = float(np.degrees(q[self._scoop_idx[2]]))
        # harvester_moveit 원본 규약: +50° 명령은 셸/과실 접촉으로 실각 약 41°에
        # 안정되므로 BLADE_CLOSED_DEG(40°)-1° 이상이면 절삭 위치 도달이다.
        blade_closed = blade_deg >= self._mm.BLADE_CLOSED_DEG - 1.0
        same_captured_fruit = bool(
            fruit_path
            and self._verified_fruit_path == fruit_path)
        detached = bool(
            blade_closed
            and same_captured_fruit
            and distance <= max_distance
            and self._task.detach_fruit(fruit_path))
        self._cut_status = {
            "cut_id": cut_id,
            "cut_success": detached,
            "blade": round(blade_deg, 2),
            "d": 999.0 if not np.isfinite(distance) else round(distance, 4),
        }
        print(
            f"[Scoop] cut id={cut_id} fruit_id={fruit_id} "
            f"blade={blade_deg:.1f}deg captured={int(same_captured_fruit)} "
            f"tcp_distance={distance:.4f} detached={int(detached)}")

    def _handle_follow_check(self, request) -> None:
        """후퇴 중 과실이 스쿱 안에서 물리적으로 함께 이동했는지 거리로 검증한다."""
        self._grasp_status = {}
        try:
            check_id = int(request["id"])
            max_delta = float(request.get("max_delta", 0.015))
        except (KeyError, TypeError, ValueError):
            return
        center = (None if not self._verified_fruit_path else
                  self._fruit_center_world(self._verified_fruit_path))
        tcp = self._tcp_world()
        offset = None if center is None or tcp is None else center - tcp
        delta = (float("inf") if offset is None or self._grasp_tcp_offset is None
                 else float(np.linalg.norm(offset - self._grasp_tcp_offset)))
        success = bool(delta <= max_delta)
        self._follow_status = {
            "follow_id": check_id, "ok": success,
            "delta": 999.0 if not np.isfinite(delta) else round(delta, 4),
        }
        print(f"[GraspVerify] follow id={check_id} delta={delta:.4f} success={success}")

    def _publish_sim_tomato(self) -> None:
        """로봇 주변 ripe 과실의 실제 base-frame 좌표를 한 개씩 발행한다."""
        if self._fruit_pub is None or self._task is None or self._stage is None:
            return
        from pxr import Gf, Usd, UsdGeom
        cache = UsdGeom.XformCache()
        bbox_cache = UsdGeom.BBoxCache(
            Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
        base = self._stage.GetPrimAtPath(
            self._manipulator_base_path(self._stage))
        if not base.IsValid():
            return
        world_to_base = cache.GetLocalToWorldTransform(base).GetInverse()
        nearby = []
        for fruit in self._task.pickable_fruits():
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
            world_range = bbox_cache.ComputeWorldBound(geom).ComputeAlignedRange()
            world = world_range.GetMidpoint()
            fruit_size = world_range.GetSize()
            fruit_height = float(fruit_size[2])
            fruit_radius = 0.5 * max(float(fruit_size[0]), float(fruit_size[1]))
            p = world_to_base.Transform(Gf.Vec3d(world))
            # 매 프레임 수백 개를 순회 발행하지 않고 현재 팔 작업영역만 보낸다.
            if 0.0 <= p[0] <= 1.5 and abs(p[1]) <= 1.2 and 0.1 <= p[2] <= 1.9:
                nearby.append((fruit["path"], int(fruit["id"]),
                               [float(v) for v in p],
                               fruit_height, fruit_radius))
        if not nearby:
            return
        nearby.sort(key=lambda item: item[0])
        _, fruit_id, position, fruit_height, fruit_radius = nearby[
            self._fruit_cursor % len(nearby)]
        self._fruit_cursor += 1
        payload = json.dumps({
            "class": "ripe",
            "id": fruit_id,
            "position": [round(v, 4) for v in position],
            "height": round(fruit_height, 4),
            "radius": round(fruit_radius, 4),
        }, separators=(",", ":"))
        self._fruit_pub.publish(payload)

    def _manipulator_base_path(self, stage) -> str:
        """MoveIt planning frame `base_link`에 해당하는 실제 m0617 USD prim.

        Ridgeback에도 이름이 같은 ``Base/base_link``가 있지만 그것은 지면 기준 섀시다.
        m0617은 그보다 ``arm_mount_z=0.30m`` 위의 ``Arm/base_link``에 장착된다.
        MoveIt 목표·비전 TF·GT 검증은 모두 후자를 기준으로 해야 한다.
        """
        arm_base = f"{self.root}/Arm/base_link"
        if stage.GetPrimAtPath(arm_base).IsValid():
            return arm_base
        # 에셋 구조 진단 중에도 씬을 죽이지 않도록 기존 섀시 프레임으로만 fallback.
        fallback = f"{self.root}/Base/base_link"
        print(f"[MM] ⚠ m0617 base_link 없음 — 섀시 기준 fallback: {fallback}")
        return fallback

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

    def _apply_scoop(self) -> None:
        """현재 개폐 상태를 스쿱 3축 위치 타깃으로 건다.

        팔은 MoveIt 이 /joint_command 로 몰지만 그리퍼는 Isaac 이 직접 잡는다 — 스쿱
        3축은 MoveIt 플래닝 그룹에 넣지 않는다(동축 셸이라 충돌 모델이 서로 겹쳐
        플래너가 항상 self-collision 으로 실패한다).
        """
        if self._scoop_idx is None:
            return
        target = self.SCOOP_CLOSED if self._gripper_closed else self.SCOOP_OPEN
        self._scoop_target = target.copy()
        self.robot.apply_action(ArticulationAction(
            joint_positions=self._scoop_target.copy(), joint_indices=self._scoop_idx))

    def _drive_base(self) -> None:
        """/cmd_vel(vx, vy, wz) 을 더미 3축에 적분해 넣는다 — 홀로노믹 베이스의 '주행'.

        왜 적분인가 — 이 베이스는 속도/위치 드라이브를 무시하고 텔레포트만 먹는다
        (2026-07-18 실측). 그래서 Isaac 이 속도를 위치로 바꿔 매 프레임 새 포즈를 찍는다.
        vx/vy 는 **로봇 기준**(Twist 규약)이라 yaw 로 월드에 회전시켜 더한다.
        적분 상태를 따로 안 들고 매 프레임 조인트를 읽는 이유 — Play/Stop 리셋이나 JSON
        base 텔레포트로 조인트가 바뀌어도 자동으로 그 자리에서 이어간다(상태 두 벌 금지).
        """
        vx, vy, wz = self._twist.poll()
        # finalize()에서 MM namespace와 홀로노믹 한계를 반영해 만든 전용 사본.
        # 공유 설정을 다시 읽으면 max_vy=0 및 전역 토픽 설정으로 되돌아간다.
        nav = self._nav
        vx = max(-nav.max_vx, min(nav.max_vx, vx))   # Nav2 가 상한을 어겨도 여기서 막는다
        vy = max(-nav.max_vy, min(nav.max_vy, vy))
        wz = max(-nav.max_wz, min(nav.max_wz, wz))
        self._base_moving = not (vx == 0.0 and vy == 0.0 and wz == 0.0)
        if not self._base_moving:
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
      drive : /{ns}/cmd_vel_safe 구독만. 실행(적분)은 MMDriver._drive_base 가 한다.
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
    if opts.nav_drive:
        try:
            # Isaac 프로세스 안에서 별도 rclpy Context를 초기화하면 내장 ROS2 bridge와
            # 충돌해 앱이 종료될 수 있다. Twist는 공식 OmniGraph bridge로만 받는다.
            sub = RB.build_twist_sub(
                "/World/HarvNav_drive", nav.cmd_vel_topic)
            poller = RB.TwistPoller(sub)
        except Exception:
            import traceback
            print("[Nav] MM cmd_vel 그래프 생성 실패")
            traceback.print_exc()
    if opts.nav_odom:
        try:
            RB.build_odometry(stage, "/World/HarvNav_odom", chassis, nav)
        except Exception:
            import traceback
            print("[Nav] MM odometry 그래프 생성 실패")
            traceback.print_exc()
    if opts.nav_scan:
        try:
            lidar = mm.attach_lidar(stage, nav.lidar_offset)
            if lidar:
                # TF와 scan을 따로 보호한다. 센서 TF 값 문제 하나 때문에 /scan까지
                # 사라지면 AMCL이 map→odom을 만들 수 없어 맵/코스트맵 전체가 멈춘다.
                try:
                    RB.build_tf_sensor(
                        stage, "/World/HarvNav_tf", chassis, lidar, nav)
                except Exception:
                    import traceback
                    print("[Nav] MM lidar TF 그래프 생성 실패")
                    traceback.print_exc()
                RB.build_lidar_scan(
                    stage, "/World/HarvNav_scan", lidar, nav)
        except Exception:
            import traceback
            print("[Nav] MM lidar scan 그래프 생성 실패")
            traceback.print_exc()
    return poller


def build_teleop(mm_robot, gui: bool):
    """MM 키보드 텔레옵 — 팔6·베이스·스쿱. 반환: step(is_playing) 콜백(실패 시 None).

    글자키만 쓴다(방향키는 뷰포트가 가로챔 — spike05 실측). GUI 전용. mm_robot 이 물리
    초기화(world.reset)된 뒤 호출할 것 — 컨트롤러가 현재 관절값에서 출발한다.
    """
    if not gui:
        print("[Teleop] --headless 라 키보드 입력 불가 — 텔레옵 비활성")
        return None
    import carb.input
    import omni.appwindow

    # 스쿱 컨트롤러(control_moveit)는 그리퍼가 동축 3축이라 mm 와 맞지만 ARM 이 UR
    # 이름이다. 여기서도 "팔만 갈아끼운다" — m0617 관절명으로 덮어쓴 1회용 서브클래스.
    from robots.control_moveit import HarvesterController as _ScoopController

    class HarvesterController(_ScoopController):
        ARM = ("joint_1", "joint_2", "joint_3",
               "joint_4", "joint_5", "joint_6")

    ctrl = HarvesterController(mm_robot)
    K = carb.input.KeyboardInput
    pressed: set = set()
    active = {"joint": 0}                           # 번호키로 선택된 팔 관절(0~5)
    ARM_SEL = [K.KEY_1, K.KEY_2, K.KEY_3, K.KEY_4, K.KEY_5, K.KEY_6]

    def on_key(e, *_):
        if e.type == carb.input.KeyboardEventType.KEY_PRESS:
            if e.input in ARM_SEL:                   # 번호키 = 조작할 관절 선택(엣지)
                active["joint"] = ARM_SEL.index(e.input)
                print(f"[Teleop] 활성 관절 = {active['joint'] + 1}번")
            else:
                pressed.add(e.input)
        elif e.type == carb.input.KeyboardEventType.KEY_RELEASE:
            pressed.discard(e.input)
        return True

    appwin = omni.appwindow.get_default_app_window()
    carb.input.acquire_input_interface().subscribe_to_keyboard_events(
        appwin.get_keyboard(), on_key)

    DQ, DB, DYAW, DG = 0.02, 0.01, 0.06, 0.03
    print("""
[MM 텔레옵] 플레이 상태에서 (방향키는 뷰포트가 가로챔 — 숫자/글자키만)
  팔    숫자 1~6 으로 관절 선택 → , 반시계 / . 시계 로 그 관절 회전
  베이스 I/K 전후 · J/L 제자리 회전 (옆 이동 없음: 회전 후 전진)
  스쿱 그리퍼 Z 열기 / X 닫기
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
