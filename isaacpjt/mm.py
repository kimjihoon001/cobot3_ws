# -*- coding: utf-8 -*-
"""MoveIt 기반 수확 MM 드라이버 (--mm) — Ridgeback + m0617 + 동축 스쿱 + D455.

전신(前身)은 RMPflow 기반 `rmp_mm.py` 였다. 2026-07-24 사용자 지시로 제어를 MoveIt 로
갈아끼우고 이름을 `mm` 로 바꿨다(rmp_mm 스택은 제자리 교체돼 사라짐):

  팔     m0617 그대로 (moveit_mm 의 UR10e 와 다른 점은 여기뿐 — "팔만 갈아끼워")
  그리퍼 흡착 → 동축 3축 1/4구 스쿱 (moveit_mm 과 동일 에셋)
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

    def __init__(self, cfg, task=None):
        super().__init__()
        self._cfg = cfg
        self._task = task
        self._mm = HarvestMM(mm_robot_config(cfg.robots))
        self._base_idx = None
        self._scoop_idx = None                 # 스쿱 3축 dof 인덱스
        self._gripper_closed = False           # 스쿱 개폐 상태 (파지 확정은 웰드가 한다)
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
        self._follow_status = {}
        self._tcp_status = {}
        self._verified_fruit_path: str | None = None
        self._grasp_tcp_offset: np.ndarray | None = None
        # 파지 확정 시 과실을 그리퍼에 FixedJoint 로 웰드해 결정적으로 든다(구형 과실은
        # 평행 패드 마찰만으로는 미끄러져 안 잡힘 — 큐브와 달리 면접촉이 없다, 2026-07-23).
        self._welded_fruit_path: str | None = None
        self._weld_joint_path: str | None = None
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
        # 주행 중 팔이 base 가속에 밀려 흔들리지 않도록 6축 전부 아주 단단히 잡는다
        # (2026-07-22 사용자: 주행 중 팔이 막 딴 방향으로 움직임 → 게인 대폭 상향).
        kps[arm_indices] = np.maximum(kps[arm_indices] * 2.0, 1.0e5)
        kds[arm_indices] = np.maximum(kds[arm_indices] * 2.0, 1.0e4)
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
            RB.build_camera_optical_tf(
                stage, "/World/RosCameraTf",
                f"{self.root}/Base/base_link", cam_prim,
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
            self._release_welded_fruit()
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
                            f"{self.root}/Base/base_link")
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
                        # 스쿱 개폐. 닫힘 상태에서 수용부가 과실에 닿으면 grasp_check 가
                        # 웰드한다. 열면 웰드를 끊어 과실을 놓는다(플레이스).
                        self._gripper_closed = bool(
                            cmd["gripper"].get("closed", False))
                        print(f"[Scoop] {'CLOSE' if self._gripper_closed else 'OPEN'}")
                        self._apply_scoop()
                        if not self._gripper_closed:
                            self._pending_grasp_check = None
                            self._release_welded_fruit()
                    if "grasp_check" in cmd:
                        self._handle_grasp_check(cmd["grasp_check"])
                    if "follow_check" in cmd:
                        self._handle_follow_check(cmd["follow_check"])
                    if "foliage" in cmd:
                        self._toggle_foliage(bool(cmd["foliage"]))
        # 상태 발행 — 팔 모션 상태는 MoveIt(ROS2)이 자기 액션 피드백으로 안다. Isaac 은
        # 시뮬레이터만 아는 것(파지 검증·TCP 실측·그리퍼)만 돌려준다.
        # Isaac 5.1 generic ROS2Publisher 의 std_msgs/String data 는 약 128 byte 에서
        # "..." 로 잘리므로 payload 를 짧게 유지한다(잘린 JSON 은 파싱 불가 → 전 동작 타임아웃).
        if is_playing and self._status_pub is not None:
            if self._tcp_status:
                wire_status = dict(self._tcp_status)
            elif self._grasp_status:
                wire_status = dict(self._grasp_status)
            elif self._follow_status:
                wire_status = dict(self._follow_status)
            else:
                wire_status = {"gripper": 1.0 if self._gripper_closed else 0.0}
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
        reference = self._stage.GetPrimAtPath(f"{self.root}/Base/base_link")
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

    def _weld_fruit_to_gripper(self, fruit_path: str,
                               center: np.ndarray, tcp: np.ndarray) -> bool:
        """과실 강체를 스쿱 수용부(그리퍼 base)에 FixedJoint 로 웰드한다.

        구형 과실은 마찰만으로는 미끄러져 안 잡힌다(큐브와 달리 면접촉이 없다). 그래서
        파지 확정 시점에 결정적으로 웰드한다. 웰드 전에 과실을 **TCP 축(+Z) 위로** 스냅해
        옆으로 샌 오차를 없애고 수용부 안쪽에 앉힌다. 축방향 전진거리는 그대로 둬서 이미
        닿아 있는 접촉 상태를 유지한다.
        과실 Xform 미세 스케일 오염을 막으려 프레임은 스케일 제거 순수 강체로 만든다.
        """
        from pxr import Gf, UsdGeom, UsdPhysics

        grip = self._mm._grip_base
        if not grip or not fruit_path or self._stage is None:
            return False
        grip_prim = self._stage.GetPrimAtPath(grip)
        fruit_prim = self._stage.GetPrimAtPath(fruit_path)
        if not grip_prim.IsValid() or not fruit_prim.IsValid():
            return False
        self._release_welded_fruit()             # 이전 웰드가 남아 있으면 먼저 제거
        joint_path = f"{grip}/GraspWeld"
        cache = UsdGeom.XformCache()
        m_grip = cache.GetLocalToWorldTransform(grip_prim)
        m_fruit = cache.GetLocalToWorldTransform(fruit_prim)

        def _rigid(m):
            r = Gf.Matrix4d()
            r.SetRotate(m.ExtractRotationQuat().GetNormalized())
            r.SetTranslateOnly(m.ExtractTranslation())
            return r

        rigid_grip = _rigid(m_grip)
        # 컵 축 u = TCP 프림의 월드 +Z(접근축). 과실 중심을 축 위로만 옮긴다.
        center = np.asarray(center, dtype=float)
        tcp = np.asarray(tcp, dtype=float)
        shift = np.zeros(3, dtype=float)
        tcp_path = self._mm.grasp_tcp_path(self._stage)
        if tcp_path:
            tcp_prim = self._stage.GetPrimAtPath(tcp_path)
            if tcp_prim.IsValid():
                u = np.asarray(cache.GetLocalToWorldTransform(tcp_prim)
                               .TransformDir(Gf.Vec3d(0.0, 0.0, 1.0)), dtype=float)
                un = float(np.linalg.norm(u))
                if un > 1e-9:
                    u /= un
                    d = float(np.dot(center - tcp, u))   # 축방향 전진거리(표면 여유)
                    desired_center = tcp + u * max(d, 0.0)  # 옆오차 제거, 전진거리 유지
                    shift = desired_center - center
        # follow-check 기준 오프셋은 스냅 **후** 정착할 값이어야 한다. 스냅 전 값으로 두면
        # 웰드가 과실을 옮긴 만큼 delta 가 커져 검증 실패 → 홈복귀(안 들림). 정착 offset =
        # (스냅된 중심) − tcp.
        self._grasp_tcp_offset = (center + shift) - tcp
        origin_world = np.asarray(m_fruit.ExtractTranslation(), dtype=float) + shift
        local_pos0 = rigid_grip.GetInverse().Transform(
            Gf.Vec3d(float(origin_world[0]), float(origin_world[1]),
                     float(origin_world[2])))
        rel = _rigid(m_fruit) * rigid_grip.GetInverse()   # 방향은 현 상대자세 유지

        joint = UsdPhysics.FixedJoint.Define(self._stage, joint_path)
        joint.CreateBody0Rel().SetTargets([grip])
        joint.CreateBody1Rel().SetTargets([fruit_path])
        joint.CreateLocalPos0Attr().Set(Gf.Vec3f(local_pos0))
        joint.CreateLocalRot0Attr().Set(
            Gf.Quatf(rel.ExtractRotationQuat().GetNormalized()))
        joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0))
        joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0))
        joint.CreateJointEnabledAttr().Set(True)
        joint.CreateExcludeFromArticulationAttr().Set(True)
        self._welded_fruit_path = fruit_path
        self._weld_joint_path = joint_path
        print(f"[Scoop] 과실 웰드(TCP 축 정렬): {grip} ↔ {fruit_path}")
        return True

    def _release_welded_fruit(self) -> None:
        """그리퍼를 열 때 웰드 조인트를 끊어 과실을 놓는다(플레이스)."""
        if self._weld_joint_path and self._stage is not None:
            prim = self._stage.GetPrimAtPath(self._weld_joint_path)
            if prim.IsValid():
                self._stage.RemovePrim(self._weld_joint_path)
                print(f"[Grasp] 과실 웰드 해제: {self._weld_joint_path}")
        self._welded_fruit_path = None
        self._weld_joint_path = None

    def _handle_grasp_check(self, request) -> None:
        """TCP 근접과 동일 과실 양측 접촉을 확인한 뒤 FixedJoint를 해제한다."""
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
        # 흡착 ON + 컵이 과실 표면 근접(TCP-중심 거리 ≤ max_distance) → 꽃자루를 끊고
        # 과실을 컵에 웰드한다. 손가락/접촉/파지력 없음(흡착).
        grasp_confirmed = bool(distance <= max_distance and self._gripper_closed)
        detached = welded = False
        if (grasp_confirmed and fruit is not None
                and center is not None and tcp is not None):
            self._grasp_tcp_offset = center - tcp
            welded = self._weld_fruit_to_gripper(fruit_path, center, tcp)
            detached = bool(self._task.detach_fruit(fruit_path))
        success = bool(grasp_confirmed and detached and welded)
        self._verified_fruit_path = fruit_path if success else None
        self._grasp_status = {
            "grasp_id": check_id, "ok": success,
            "fruit_id": fruit_id,
            "d": 999.0 if not np.isfinite(distance) else round(distance, 4),
        }
        print(f"[Scoop] grasp id={check_id} fruit_id={fruit_id} "
              f"tcp_distance={distance:.4f} closed={int(self._gripper_closed)} "
              f"detached={detached} welded={welded} success={success}")

    def _handle_follow_check(self, request) -> None:
        """FixedJoint 해제 후 후퇴했을 때 과실-TCP 상대벡터가 유지됐는지 검증한다."""
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
        base = self._stage.GetPrimAtPath(f"{self.root}/Base/base_link")
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
            world = bbox_cache.ComputeWorldBound(geom).ComputeAlignedRange().GetMidpoint()
            p = world_to_base.Transform(Gf.Vec3d(world))
            # 매 프레임 수백 개를 순회 발행하지 않고 현재 팔 작업영역만 보낸다.
            if 0.0 <= p[0] <= 1.5 and abs(p[1]) <= 1.2 and 0.1 <= p[2] <= 1.9:
                nearby.append((fruit["path"], int(fruit["id"]),
                               [float(v) for v in p]))
        if not nearby:
            return
        nearby.sort(key=lambda item: item[0])
        _, fruit_id, position = nearby[self._fruit_cursor % len(nearby)]
        self._fruit_cursor += 1
        payload = json.dumps({
            "class": "ripe",
            "id": fruit_id,
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

    def _apply_scoop(self) -> None:
        """현재 개폐 상태를 스쿱 3축 위치 타깃으로 건다.

        팔은 MoveIt 이 /joint_command 로 몰지만 그리퍼는 Isaac 이 직접 잡는다 — 스쿱
        3축은 MoveIt 플래닝 그룹에 넣지 않는다(동축 셸이라 충돌 모델이 서로 겹쳐
        플래너가 항상 self-collision 으로 실패한다).
        """
        if self._scoop_idx is None:
            return
        target = self.SCOOP_CLOSED if self._gripper_closed else self.SCOOP_OPEN
        self.robot.apply_action(ArticulationAction(
            joint_positions=target.copy(), joint_indices=self._scoop_idx))

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
    try:
        if opts.nav_drive:
            sub = RB.build_twist_sub("/World/HarvNav_drive", nav.cmd_vel_topic)
            poller = RB.TwistPoller(sub)
        if opts.nav_odom:
            RB.build_odometry(stage, "/World/HarvNav_odom", chassis, nav)
        if opts.nav_scan:
            lidar = mm.attach_lidar(stage, nav.lidar_offset)
            if lidar:
                RB.build_tf_sensor(stage, "/World/HarvNav_tf", chassis, lidar, nav)
                RB.build_lidar_scan(stage, "/World/HarvNav_scan", lidar, nav)
    except Exception:
        import traceback
        print("\n" + "=" * 64)
        print("[Nav] MM 그래프 생성 실패 — 씬은 유지. tools/nav2_node_probe.py 로 노드명 확인.")
        print("=" * 64)
        traceback.print_exc()
        print("=" * 64 + "\n")
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
