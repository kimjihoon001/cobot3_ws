# -*- coding: utf-8 -*-
"""수확 MM 드라이버 (--mm) — Ridgeback + m0617 + OnRobot RG2 + D455.

로봇 모델은 robots/harvester.py, ROS 브리지는 ros/robot_bridge.py. 이 파일은 그 둘을
'배선'하고 텔레옵/JSON 명령을 매 프레임 적용한다(§5.6 실행측). 판단은 ROS2 가 한다.
"""
from __future__ import annotations

import json
import math

import numpy as np
from isaacsim.core.utils.types import ArticulationAction

from robot_base import Driver, ros_fail
from robots.harvester import HOME_POSE_DEG, HarvestMM
from scene.ground import COMMON_FLOOR_Z

# 임시 배치 — 온실 앞마당(빈 홀 바닥, 온실 y −10 앞). 물류 동선 확정 후 조정.
POSE = (0.0, -12.0, COMMON_FLOOR_Z)
# 키네마틱 베이스 조인트 (JSON base 명령용 — 텔레포트만 먹는다, 2026-07-18 실측)
BASE_JOINTS = ("dummy_base_prismatic_x_joint",
               "dummy_base_prismatic_y_joint",
               "dummy_base_revolute_z_joint")
RG2_OPEN_RAD = 0.0
RG2_JOINTS = ("finger_joint", "right_inner_knuckle_joint")


class MMDriver(Driver):
    flag = "--mm"
    name = "mm"
    ns = "harvester_0"
    root = "/World/Harvester"

    def __init__(self, cfg, task=None):
        super().__init__()
        self._cfg = cfg
        self._task = task
        self._mm = HarvestMM(cfg.robots)
        self._base_idx = None
        self._gripper_idx = None
        self._gripper_indices = None
        self._suction_on = False               # 흡착 on/off (그리퍼 열고닫기·파지력 없음)
        self._pending_grasp_check = None
        self._poller = None
        self._status_pub = None
        self._fruit_pub = None
        self._fruit_cursor = 0
        self._teleop = None
        self._stage = None
        self._twist = None                    # /cmd_vel 폴러 (Nav2 주행)
        self._dt = 1.0 / 60.0                 # finalize 에서 월드 실제 물리 dt 로 덮어씀
        self._rmpflow = None
        self._was_playing = False
        self._grasp_status = {}
        self._follow_status = {}
        self._fk_status = {}
        self._tcp_status = {}
        self._verified_fruit_path: str | None = None
        self._grasp_tcp_offset: np.ndarray | None = None
        # 파지 확정 시 과실을 그리퍼에 FixedJoint 로 웰드해 결정적으로 든다(구형 과실은
        # 평행 패드 마찰만으로는 미끄러져 안 잡힘 — 큐브와 달리 면접촉이 없다, 2026-07-23).
        self._welded_fruit_path: str | None = None
        self._weld_joint_path: str | None = None
        self._base_moving = False             # 이동(nav) 중이면 팔을 홈으로 접는다
        self._nav_home_sent = False
        self._arm_idx = None                  # 팔 6축 dof 인덱스 (홈 고정용, finalize 에서 채움)
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
        names = list(r.dof_names)
        # 검증된 RG2 ParallelGripper 예제와 동일하게 구동 관절 두 개만 같은
        # 방향으로 명령한다. 나머지 네 관절은 RG2 링크 기구를 따라 수동으로 움직인다.
        self._gripper_indices = np.array([
            names.index(name) for name in RG2_JOINTS])
        self._gripper_idx = int(self._gripper_indices[0])
        # m0617 6축 position drive 게인 강화 — RMPflow 추종 시 처짐 방지 + **이동(nav) 중
        # 대기자세를 단단히 유지해 흔들리지 않게**(2026-07-22 사용자). 하한 상향.
        arm_indices = np.array([
            list(r.dof_names).index(name) for name, _ in HOME_POSE_DEG])
        # 주행/대기 중 팔을 홈에 능동 고정하기 위한 인덱스·목표각(rad) 저장.
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
        # 검증된 ParallelGripper 예제의 drive 게인. 기존 kd=3000은 30N에 맞춰
        # 낮춘 maxForce를 속도 항만으로 포화시켜 0.19rad에서 정지시켰다.
        gi = self._gripper_indices
        kps[gi] = 1.0e4
        kds[gi] = 1.0e2
        controller.set_gains(kps=kps, kds=kds)
        # 흡착 그리퍼 — 손가락 구동/파지력 코드 없음(2026-07-23 사용자). 파지는 흡착=웰드로만.
        print(f"[MM] m0617 drive gain 강화: kp={kps[arm_indices].tolist()} "
              f"kd={kds[arm_indices].tolist()}")

        if not opts.no_ros:
            try:
                from ros import robot_bridge as RB
                RB.build_joint_bridge(stage, f"/World/RosBridge_{self.ns}",
                                      self.ns, self.art,
                                      apply_commands=not opts.rmpflow)
                sub = RB.build_string_sub(
                    f"/World/RosCmd_{self.ns}", f"/{self.ns}/cmd")
                self._poller = RB.StringPoller(sub)
                pub = RB.build_string_pub(
                    f"/World/RosRmpStatus_{self.ns}",
                    f"/{self.ns}/rmpflow/status")
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
                self._twist = build_nav(stage, self._mm,
                                        self._cfg.robots.harvester_nav, opts)

        if opts.rmpflow:
            from robots.control import (LegacyIkTargetController,
                                        RmpFlowTargetController)
            controller_type = (LegacyIkTargetController
                               if opts.legacy_ik else RmpFlowTargetController)
            self._rmpflow = controller_type(
                r, stage,
                reference_prim=f"{self.root}/Base/base_link",
                arm_base_prim=f"{self.root}/Arm/base_link",
                physics_dt=world.get_physics_dt(),
                tool_tcp_prim=self._mm.grasp_tcp_path(stage),
                home_positions=self._arm_home_q)
            mode_name = "legacy waypoint IK" if opts.legacy_ik else "Isaac RMPflow"
            print(f"[RMPflow] m0617 목표 추종 활성 ({mode_name}): "
                  "/harvester_0/cmd rmp_target")

        if opts.mm_teleop and opts.rmpflow:
            print("[MM] --rmpflow와 --mm-teleop 동시 제어는 충돌하므로 텔레옵 비활성")
        elif opts.mm_teleop:
            # 두 번째 인자는 구형 main.py 호출 호환용이며 칼날은 더 이상 제어하지 않는다.
            self._teleop = build_teleop(r, self._mm.set_blade_deg, opts.gui)
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
            RB.build_camera(stage, "/World/RosCamera", cam_prim,
                            self._cfg.robots.camera)
            RB.build_camera_optical_tf(
                stage, "/World/RosCameraTf",
                f"{self.root}/Base/base_link", cam_prim,
                self._cfg.robots.camera.frame_id)
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
        if is_playing and not self._was_playing and self._rmpflow is not None:
            self._rmpflow.reset()
            self._fk_status = {}
            self._suction_on = False
            self._pending_grasp_check = None
            self._release_welded_fruit()
        self._was_playing = is_playing
        if self._teleop is not None:                 # 키보드 텔레옵 (재생 중에만 적용)
            self._teleop(is_playing)
        if is_playing and self._twist is not None:   # Nav2 /cmd_vel → 홀로노믹 베이스
            self._drive_base()
            self._maybe_fold_home_for_nav()
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
                        home_ready = (self._rmpflow is None
                                      or self._rmpflow.status()["at_home"])
                        if len(b) == 3 and home_ready:
                            self.robot.set_joint_positions(
                                np.array(b), joint_indices=self._base_idx)
                        elif len(b) == 3:
                            print("[MM] 홈 자세 전 베이스 이동 차단")
                    if "rmp_target" in cmd and self._rmpflow is not None:
                        self._grasp_status = {}
                        self._follow_status = {}
                        self._fk_status = {}
                        target = cmd["rmp_target"]
                        if isinstance(target, dict):
                            try:
                                self._rmpflow.set_target(
                                    target["position"], int(target.get("id", 0)),
                                    str(target.get("phase", "MOVE")))
                            except (KeyError, TypeError, ValueError) as exc:
                                print(f"[RMPflow] 잘못된 목표 무시: {exc}")
                    if cmd.get("rmp_stop") is True and self._rmpflow is not None:
                        self._rmpflow.stop()
                    if "grasp_yaw_adjust" in cmd and self._rmpflow is not None:
                        # 이전 grasp_check 응답이 남아 있으면 상태 publisher가 새
                        # GRASP_YAW_CORRECT 진행/완료보다 그 응답을 영구 우선한다.
                        # 보정 명령은 새 모션이므로 검사 응답 캐시를 먼저 비운다.
                        self._grasp_status = {}
                        self._follow_status = {}
                        self._fk_status = {}
                        adjust = cmd["grasp_yaw_adjust"]
                        if isinstance(adjust, dict):
                            try:
                                self._rmpflow.adjust_gripper_yaw(
                                    float(adjust["delta_deg"]),
                                    int(adjust.get("id", 0)))
                            except (KeyError, TypeError, ValueError) as exc:
                                print(f"[GraspCorrect] 잘못된 yaw 보정 무시: {exc}")
                    if "rmp_home" in cmd and self._rmpflow is not None:
                        self._grasp_status = {}
                        self._follow_status = {}
                        self._fk_status = {}
                        home = cmd["rmp_home"]
                        if isinstance(home, dict):
                            self._rmpflow.go_home(int(home.get("id", 0)))
                    if "fk_check" in cmd and self._rmpflow is not None:
                        request = cmd["fk_check"]
                        check_id = (int(request.get("id", 0))
                                    if isinstance(request, dict) else 0)
                        check = self._rmpflow.fk_consistency()
                        delta = check["delta"]
                        self._fk_status = {
                            "fk_id": check_id, "ok": bool(check["ok"]),
                            "e": round(float(check["error"]), 5),
                            "dx": round(float(delta[0]), 5),
                            "dy": round(float(delta[1]), 5),
                            "dz": round(float(delta[2]), 5),
                        }
                    if "fk_probe" in cmd and self._rmpflow is not None:
                        probe = cmd["fk_probe"]
                        if isinstance(probe, dict):
                            try:
                                self._rmpflow.set_fk_probe(
                                    int(probe.get("joint", 3)),
                                    float(probe.get("delta_deg", 5.0)))
                            except (TypeError, ValueError) as exc:
                                print(f"[FKCHK] 잘못된 probe 무시: {exc}")
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
                        # 흡착 on/off. ON 상태에서 컵이 과실 표면에 닿으면 grasp_check가
                        # 웰드한다. OFF 면 웰드를 끊어 과실을 놓는다(플레이스).
                        self._suction_on = bool(cmd["gripper"].get("closed", False))
                        print(f"[Suction] {'ON' if self._suction_on else 'OFF'}")
                        if not self._suction_on:
                            self._pending_grasp_check = None
                            self._release_welded_fruit()
                    if "grasp_check" in cmd:
                        self._handle_grasp_check(cmd["grasp_check"])
                    if "follow_check" in cmd:
                        self._handle_follow_check(cmd["follow_check"])
                    if "foliage" in cmd:
                        self._toggle_foliage(bool(cmd["foliage"]))
        if is_playing and self._rmpflow is not None:
            self._rmpflow.apply()
            if self._status_pub is not None:
                status = self._rmpflow.status()
                # 흡착 상태를 그리퍼 필드로 보고한다(1=흡착, 0=해제). ROS FSM 의
                # PLACE_RELEASING 은 이 값이 ≤0.08 이면 놓기 완료로 본다.
                status["gripper"] = 1.0 if self._suction_on else 0.0
                # Isaac 5.1 generic ROS2Publisher의 std_msgs/String data는 긴
                # 문자열을 약 128 byte에서 "..."로 잘라 버린다. 잘린 JSON은
                # manipulator_target_node가 파싱할 수 없어 reached=true를 놓치고
                # 모든 동작이 ERROR_TIMEOUT으로 끝난다. 제어 루프에 필요한 값만
                # 짧게 보내고, 좌표 상세값은 Isaac 콘솔의 RMPflow 로그로 본다.
                if self._fk_status:
                    wire_status = dict(self._fk_status)
                elif self._tcp_status:
                    wire_status = dict(self._tcp_status)
                elif self._grasp_status:
                    wire_status = dict(self._grasp_status)
                elif self._follow_status:
                    wire_status = dict(self._follow_status)
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
                    # FK 진단 응답은 한 번만 보내 일반 모션 상태 publication을 막지 않는다.
                    if self._fk_status:
                        self._fk_status = {}
                    if self._tcp_status:
                        self._tcp_status = {}
        # 주행/대기 중 팔 고정 — RMPflow 가 파지로 능동 제어 중이 아니면 홈 관절각을 매
        # 프레임 위치 타깃으로 재지정해 base 회전·가속에도 팔이 안 흔들리게 잠근다(2026-07-22
        # 사용자: 주행 중 팔 홈 고정). 텔레옵 모드는 직접 조작하므로 제외.
        if (is_playing and self._teleop is None
                and self._arm_home_q is not None
                and not (self._rmpflow is not None and self._rmpflow.is_active())):
            self.robot.apply_action(ArticulationAction(
                joint_positions=self._arm_home_q, joint_indices=self._arm_idx))
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
        """과실 강체를 흡착컵(그리퍼 base)에 FixedJoint 로 웰드한다.

        흡착은 컵이 과실을 자기 축 위로 빨아들이는 것이므로, 웰드 전에 과실을 **컵 축
        (TCP +Z) 위로** 스냅해 옆으로 샌 오차를 없애고 컵 끝(흡착부)에 붙인다. 전진거리
        (컵→과실중심 축성분)는 그대로 둬서 표면이 컵 끝에 닿은 상태를 유지한다.
        (2지 집게 때의 '중심→TCP' 순간이동과 달리, 흡착에선 이게 실제 물리 동작이다.)
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
        print(f"[Suction] 과실 흡착 웰드(컵 축 정렬): {grip} ↔ {fruit_path}")
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
        grasp_confirmed = bool(distance <= max_distance and self._suction_on)
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
        print(f"[Suction] grasp id={check_id} fruit_id={fruit_id} "
              f"tcp_distance={distance:.4f} suction={int(self._suction_on)} "
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

    def _maybe_fold_home_for_nav(self) -> None:
        """이동(nav) 중이면 팔을 홈으로 접는다 — 긴 m0617 팔이 주행 중 걸리지 않게
        (2026-07-23 사용자: 이동 시 홈자세). 파지 Cartesian 추종 중엔 개입하지 않는다."""
        if self._rmpflow is None:
            return
        if not self._base_moving:
            self._nav_home_sent = False
            return
        if self._rmpflow.is_pursuing_target():
            return
        if not self._nav_home_sent:
            self._rmpflow.go_home(0)
            self._nav_home_sent = True
            print("[MM] 이동 감지 — 팔 홈 복귀")

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


def build_teleop(mm_robot, _unused_blade_setter, gui: bool):
    """MM 키보드 텔레옵 — 팔6·베이스·RG2. 반환: step(is_playing) 콜백(실패 시 None).

    글자키만 쓴다(방향키는 뷰포트가 가로챔 — spike05 실측). GUI 전용. mm_robot 이 물리
    초기화(world.reset)된 뒤 호출할 것 — HarvesterController 가 현재 관절값에서 출발한다.
    두 번째 인자는 구형 호출부 API 호환용이며 사용하지 않는다.
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
  RG2 그리퍼 Z 열기 / X 닫기
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
