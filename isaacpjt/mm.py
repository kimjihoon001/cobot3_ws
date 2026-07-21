# -*- coding: utf-8 -*-
"""수확 MM 드라이버 (--mm) — Ridgeback + UR10e + 2F-85 + 커터지그 + 가동날 + D455.

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

# 임시 배치 — 온실 앞마당(빈 홀 바닥, 온실 y −10 앞). 물류 동선 확정 후 조정.
POSE = (0.0, -12.0, 0.0)
# 키네마틱 베이스 조인트 (JSON base 명령용 — 텔레포트만 먹는다, 2026-07-18 실측)
BASE_JOINTS = ("dummy_base_prismatic_x_joint",
               "dummy_base_prismatic_y_joint",
               "dummy_base_revolute_z_joint")


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
        self._gripper_target = None
        self._poller = None
        self._status_pub = None
        self._teleop = None
        self._stage = None
        self._twist = None                    # /cmd_vel 폴러 (Nav2 주행)
        self._dt = 1.0 / 60.0                 # finalize 에서 월드 실제 물리 dt 로 덮어씀
        self._rmpflow = None
        self._was_playing = False
        self._cut_status = {}

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
        r.set_joints_default_state(positions=q0)

    def finalize(self, world, stage, opts):
        self._stage = stage                      # update() 의 가동날 재배치용
        # 가동날(서보 힌지) — main 이 configure 뒤 reset+settle 했으므로 정착된 자세 기준.
        # 힌지 강체·조인트는 이 뒤 main 의 reset 에서 물리 뷰에 올라가 미리 정착한다(§8).
        self._mm.attach_blade_hinge(stage)
        # 키네마틱 베이스 인덱스 (JSON base 명령용)
        r = self.robot
        self._base_idx = np.array(
            [list(r.dof_names).index(n) for n in BASE_JOINTS])
        self._gripper_idx = list(r.dof_names).index("finger_joint")

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
            except Exception:
                ros_fail("MM 조인트/명령 브리지")
            if opts.camera:
                self._build_camera(stage)
            if opts.nav_drive or opts.nav_odom or opts.nav_scan:
                self._dt = world.get_physics_dt()
                self._twist = build_nav(stage, self._mm,
                                        self._cfg.robots.harvester_nav, opts)

        if opts.rmpflow:
            from robots.control import RmpFlowTargetController
            self._rmpflow = RmpFlowTargetController(
                r, stage,
                reference_prim=f"{self.root}/Base/base_link",
                arm_base_prim=f"{self.root}/Arm/base_link",
                physics_dt=world.get_physics_dt(),
                tool_tcp_prim=self._mm.grasp_tcp_path(stage))
            print("[RMPflow] UR10e 목표 추종 활성: /harvester_0/cmd rmp_target")

        if opts.mm_teleop and opts.rmpflow:
            print("[MM] --rmpflow와 --mm-teleop 동시 제어는 충돌하므로 텔레옵 비활성")
        elif opts.mm_teleop:
            self._teleop = build_teleop(r, self._mm.set_blade_deg, opts.gui)

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

    def update(self, is_playing):
        if is_playing and not self._was_playing and self._rmpflow is not None:
            self._rmpflow.reset()
            self._cut_status = {}
        self._was_playing = is_playing
        if not is_playing and self._stage is not None:
            # 정지 중엔 조인트가 안 걸린다 — 가동날 사본을 그리퍼에 손으로 붙여둔다.
            self._mm.sync_blade_pose(self._stage)
        if self._teleop is not None:                 # 키보드 텔레옵 (재생 중에만 적용)
            self._teleop(is_playing)
        if is_playing and self._twist is not None:   # Nav2 /cmd_vel → 홀로노믹 베이스
            self._drive_base()
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
                        self._mm.set_blade_deg(float(cmd["blade"]))
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
                            try:
                                self._rmpflow.set_target(
                                    target["position"], int(target.get("id", 0)),
                                    str(target.get("phase", "MOVE")))
                            except (KeyError, TypeError, ValueError) as exc:
                                print(f"[RMPflow] 잘못된 목표 무시: {exc}")
                    if cmd.get("rmp_stop") is True and self._rmpflow is not None:
                        self._rmpflow.stop()
                    if "rmp_home" in cmd and self._rmpflow is not None:
                        home = cmd["rmp_home"]
                        if isinstance(home, dict):
                            self._rmpflow.go_home(int(home.get("id", 0)))
                    if "gripper" in cmd and isinstance(cmd["gripper"], dict):
                        closed = bool(cmd["gripper"].get("closed", False))
                        self._gripper_target = 0.80 if closed else 0.0
                    if "cut_fruit" in cmd:
                        self._handle_cut(cmd["cut_fruit"])
        if is_playing and self._rmpflow is not None:
            self._rmpflow.apply()
            if self._status_pub is not None:
                status = self._rmpflow.status()
                status["gripper"] = float(
                    self.robot.get_joint_positions()[self._gripper_idx])
                status["blade"] = self._mm.blade_deg()
                status.update(self._cut_status)
                # Isaac 5.1 generic ROS2Publisher의 std_msgs/String data는 긴
                # 문자열을 약 128 byte에서 "..."로 잘라 버린다. 잘린 JSON은
                # manipulator_target_node가 파싱할 수 없어 reached=true를 놓치고
                # 모든 동작이 ERROR_TIMEOUT으로 끝난다. 제어 루프에 필요한 값만
                # 짧게 보내고, 좌표 상세값은 Isaac 콘솔의 RMPflow 로그로 본다.
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
                if "cut_id" in status:
                    wire_status["cut_id"] = status["cut_id"]
                    wire_status["cut_success"] = status["cut_success"]
                payload = json.dumps(wire_status, separators=(",", ":"))
                if len(payload.encode("utf-8")) > 120:
                    # 향후 필드가 늘어도 조용히 다시 JSON을 깨뜨리지 않는다.
                    print(f"[RMPflow] 상태 payload 초과({len(payload)}): {payload}")
                else:
                    self._status_pub.publish(payload)
        # 손가락 위치 목표는 한 프레임짜리 명령으로 끝내지 않고 계속 유지한다.
        # 물체 접촉과 mimic 관절 부하가 있는 2F-85는 단발 action에서 목표가
        # 유지되지 않거나 중간 위치에 멈출 수 있다.
        if is_playing and self._gripper_target is not None:
            self.robot.apply_action(ArticulationAction(
                joint_positions=np.array([self._gripper_target]),
                joint_indices=np.array([self._gripper_idx])))

    def _handle_cut(self, request) -> None:
        """닫힌 커터 근처 ripe 과실의 pedicel FixedJoint를 해제한다."""
        if not isinstance(request, dict) or self._task is None or self._stage is None:
            return
        try:
            cut_id = int(request["id"])
            base_position = np.asarray(request["position"], dtype=float)
            tolerance = float(request.get("max_distance", 0.10))
            if base_position.shape != (3,) or not np.all(np.isfinite(base_position)):
                raise ValueError("position은 유한한 xyz여야 함")
        except (KeyError, TypeError, ValueError) as exc:
            print(f"[Cutter] 잘못된 절단 요청 무시: {exc}")
            return

        from pxr import Gf, UsdGeom
        reference = self._stage.GetPrimAtPath(f"{self.root}/Base/base_link")
        matrix = UsdGeom.XformCache().GetLocalToWorldTransform(reference)
        target_world = np.asarray(
            matrix.Transform(Gf.Vec3d(*base_position)), dtype=float)
        nearest = None
        nearest_distance = float("inf")
        cache = UsdGeom.XformCache()
        for fruit in self._task.pickable_fruits():
            if fruit.get("class_name") != "ripe":
                continue
            prim = self._stage.GetPrimAtPath(fruit["path"])
            if not prim.IsValid():
                continue
            position = np.asarray(
                cache.GetLocalToWorldTransform(prim).ExtractTranslation(), dtype=float)
            distance = float(np.linalg.norm(position - target_world))
            if distance < nearest_distance:
                nearest, nearest_distance = fruit, distance
        blade_closed = self._mm.blade_deg() >= self._mm.BLADE_CLOSED_DEG - 1.0
        success = bool(nearest is not None and nearest_distance <= tolerance
                       and blade_closed and self._task.detach_fruit(nearest["path"]))
        self._cut_status = {
            "cut_id": cut_id, "cut_success": success,
            "cut_distance": None if nearest is None else nearest_distance,
            "fruit_path": "" if nearest is None else nearest["path"]}
        print(f"[Cutter] pedicel joint {'해제' if success else '실패'}: "
              f"id={cut_id}, distance={nearest_distance:.3f}m, "
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


def build_teleop(mm_robot, set_blade, gui: bool):
    """MM 키보드 텔레옵 — 팔6·베이스·그리퍼·블레이드. 반환: step(is_playing) 콜백(실패 시 None).

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
            else:
                pressed.add(e.input)
        elif e.type == carb.input.KeyboardEventType.KEY_RELEASE:
            pressed.discard(e.input)
        return True

    appwin = omni.appwindow.get_default_app_window()
    carb.input.acquire_input_interface().subscribe_to_keyboard_events(
        appwin.get_keyboard(), on_key)

    DQ, DB, DYAW, DG, DBL = 0.02, 0.01, 0.02, 0.03, 2.0
    print("""
[MM 텔레옵] 플레이 상태에서 (방향키는 뷰포트가 가로챔 — 숫자/글자키만)
  팔    숫자 1~6 으로 관절 선택 → , 반시계 / . 시계 로 그 관절 회전
  베이스 I/K 전후 · J/L 제자리 회전 (옆 이동 없음: 회전 후 전진)
  그리퍼 Z 열기 / X 닫기      블레이드 B 열기(0°) / N 닫기(절단)
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
        if K.B in pressed or K.N in pressed:
            st["blade"] = max(0.0, min(35.0,
                              st["blade"] + (DBL if K.N in pressed else -DBL)))
            set_blade(st["blade"])
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
