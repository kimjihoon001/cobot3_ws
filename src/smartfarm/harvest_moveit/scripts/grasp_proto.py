#!/usr/bin/env python3
"""MoveIt 수확 — OMPL로 아래 진입점에 접근하고 Pilz CIRC로 퍼 담고 되돌아간다.
계획 프레임 = mm_base(=Isaac 섀시 base_link) — mm URDF 전환(2026-07-22)으로
섀시 프레임 과실좌표를 변환 없이 그대로 harvest_tcp 목표로 쓴다.
성공 기준: Isaac 로그에 [Cutter] pedicel joint 해제 + 그리퍼에 과실."""
import json
import math
import os
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseStamped, Quaternion, Vector3
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (AllowedCollisionEntry, AttachedCollisionObject,
                             BoundingVolume, CollisionObject, Constraints,
                             JointConstraint, OrientationConstraint,
                             PlanningScene, PlanningSceneComponents,
                             PositionConstraint)
from moveit_msgs.srv import (ApplyPlanningScene, GetPlanningScene,
                             GetPositionFK, GetPositionIK)
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import String
from std_msgs.msg import Float64, Float64MultiArray

FRAME = "mm_base"           # 계획 프레임 = 섀시(base_link) — URDF 루트
TOOL_LEN = 0.120            # tool0→1/4구 과실 중심 = URDF harvest_tcp
CLEARANCE = 0.15            # pregrasp 여유
# 주행/대기용 접힘 자세. shoulder를 -120° 사선으로 눕히고 elbow를 135°로
# 접는다. wrist_1=165°로 합각을 보상해 U자 스쿱 방향은 그대로 유지한다.
HOME_Q = {"shoulder_pan_joint": 0.0, "shoulder_lift_joint": 4.1887902,
          "elbow_joint": 2.3561945, "wrist_1_joint": 2.8797933,
          "wrist_2_joint": -1.5707963, "wrist_3_joint": 0.0}
JOINTS = list(HOME_Q)


def qmul(a, b):
    return (a[0]*b[0]-a[1]*b[1]-a[2]*b[2]-a[3]*b[3],
            a[0]*b[1]+a[1]*b[0]+a[2]*b[3]-a[3]*b[2],
            a[0]*b[2]-a[1]*b[3]+a[2]*b[0]+a[3]*b[1],
            a[0]*b[3]+a[1]*b[2]-a[2]*b[1]+a[3]*b[0])


class Grasp(Node):
    def __init__(self):
        super().__init__("grasp_proto")
        # ★상대경로(2026-07-23): PushRosNamespace(harvester_moveit) 가 /harvester_moveit/* 로
        #   밀도록 앞의 '/' 를 뗀다. 절대경로면 전역에 남아 격리된 move_group 에 안 붙는다(Codex 지적).
        self.move = ActionClient(self, MoveGroup, "move_action")
        self.fk = self.create_client(GetPositionFK, "compute_fk")
        self.ik = self.create_client(GetPositionIK, "compute_ik")
        self.apply_scene = self.create_client(ApplyPlanningScene, "apply_planning_scene")
        self.get_scene = self.create_client(GetPlanningScene, "get_planning_scene")
        # 명령 계측 — ±2π 초과 명령(=지난번 -40π 와인딩 범인)을 현행범으로 잡는다
        self.phase = "INIT"
        self.cmd_minmax = {}
        self.create_subscription(JointState, "joint_command",
                                 self._cmd_watch, 50)
        self.cmd = self.create_publisher(String, "cmd", 10)
        self.cut_results = {}
        self.attach_results = {}
        self.grip_measurement = {}
        self.contact_measurement = {}
        self._cut_seq = 0
        self._attach_seq = 0
        self.create_subscription(
            String, "rmpflow/status", self._cut_status, 20)
        self.create_subscription(
            String, "grasp_contact", self._grasp_contact_status, 10)
        # 동축 스쿱 3축 = ros2_control. 배열 순서는 inner, middle, outer cutter.
        self.grip = self.create_publisher(Float64MultiArray,
                                          "gripper_controller/commands", 10)
        self._grip_force_mode = False
        self.fruits = {}
        self.fruit_ids = {}
        self.fruit_tracks = {}
        self.create_subscription(String, "sim/tomato", self._fruit, 20)
        # ★YOLO 탐지 게이트(2026-07-23): vision_node 가 tomato 를 잡으면 target_class 발행.
        #   YOLO_GATE=1 이면 이 탐지를 기다렸다가 수확 시작(원거리 탐지→접근→파지). 좌표는 /sim/tomato.
        self.yolo_det_t = 0.0
        self.create_subscription(String, "vision/target_class",
                                 self._yolo_class, 10)
        self.js = {}
        self.create_subscription(JointState, "joint_states",
                                 lambda m: self.js.update(dict(zip(m.name, m.position))), 10)
        self.isaac_js = {}   # 그리퍼 finger 등 Isaac 원본 (jsb 는 팔 6축만) — HW 채널서 받음
        self.create_subscription(JointState, "hw_joint_states",
                                 lambda m: self.isaac_js.update(dict(zip(m.name, m.position))), 10)
        # ★가동날 실각도(2026-07-23): moveit_mm 이 매 프레임 blade_deg 를 발행. 절단 전 이 값이
        #   34°(=BLADE_CLOSED_DEG-1) 이상인지 확인해야 cut 이 성공. 고정시간 대기는 sim 속도가
        #   느리면 블레이드가 안 닫혀 blade_closed=False 로 절단 실패(2026-07-23 실측 진단).
        self.blade_ang = 0.0
        self.create_subscription(Float64, "blade_state",
                                 lambda m: setattr(self, "blade_ang", float(m.data)), 10)

    def _cmd_watch(self, m):
        for name, p in zip(m.name, m.position):
            lo, hi = self.cmd_minmax.get(name, (p, p))
            self.cmd_minmax[name] = (min(lo, p), max(hi, p))
            if abs(p) > 6.4:
                print(f"  ★★ 이상 명령: [{self.phase}] {name}={p:.3f} rad ★★")

    def _fruit(self, m):
        if not m.data:
            return
        try:
            d = json.loads(m.data)
            pos = tuple(round(v, 3) for v in d["position"])
            self.fruits[pos] = time.time()
            if "fruit_id" in d:
                fruit_id = int(d["fruit_id"])
                self.fruit_ids[pos] = fruit_id
                self.fruit_tracks[fruit_id] = (pos, time.time())
        except (ValueError, KeyError):
            pass

    def _yolo_class(self, m):
        """YOLO가 수확 가능 후보를 보고 있으면 최근 탐지 시각을 갱신한다."""
        # 가까워지면 vision_node가 tomato→quality_check→ripe 순서로 바뀐다.
        # spoiled는 화면에는 표시하되 수확 게이트를 열지 않는다.
        if m.data in {"tomato", "quality_check", "ripe"}:
            self.yolo_det_t = time.time()

    def wait_yolo(self, timeout: float = 15.0) -> bool:
        """YOLO 가 최근(1s 내) tomato 를 탐지할 때까지 대기. 탐지=True."""
        t0 = time.time()
        while time.time() - t0 < timeout and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - self.yolo_det_t < 1.0:
                return True
        return False

    def _cut_status(self, m):
        if not m.data:
            return
        try:
            d = json.loads(m.data)
            if "cut_id" in d:
                self.cut_results[int(d["cut_id"])] = bool(d.get("cut_success"))
            if "attach_id" in d:
                self.attach_results[int(d["attach_id"])] = bool(
                    d.get("attach_success"))
            # 매 프레임 상태(finger,vel)가 report_grip 응답(finger,eff,vel)을
            # 곧바로 덮지 않게 effort가 포함된 명시적 응답만 저장한다.
            if "finger" in d and "eff" in d:
                self.grip_measurement = {
                    key: float(d[key]) for key in ("finger", "eff", "vel")
                    if key in d
                }
        except (TypeError, ValueError):
            pass

    def _grasp_contact_status(self, m):
        if not m.data:
            return
        try:
            d = json.loads(m.data)
            if "grasp_contact" in d:
                self.contact_measurement = dict(d)
        except (TypeError, ValueError):
            pass

    def spin_for(self, sec):
        end = time.time() + sec
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_blade_closed(self, target: float = 34.0, timeout: float = 8.0) -> bool:
        """가동날이 target°(=BLADE_CLOSED_DEG-1) 이상 닫힐 때까지 대기. sim 속도 무관.
        닫힘=True. 고정시간 대기를 대체 — blade_state 텔레메트리로 실제 각을 확인한다."""
        t0 = time.time()
        while time.time() - t0 < timeout and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.blade_ang >= target:
                return True
        return False

    def nearest(self):
        now = time.time()
        fresh = [p for p, t in self.fruits.items() if now - t < 10]
        if not fresh:
            return None
        return min(fresh, key=lambda p: p[0]*p[0]+p[1]*p[1])

    def tracked_fruit(self, fruit_id, max_age=1.0):
        """같은 과실 ID의 최신 섀시 좌표. 오래된 좌표로 보정하지 않는다."""
        if fruit_id is None or fruit_id not in self.fruit_tracks:
            return None
        pos, stamp = self.fruit_tracks[fruit_id]
        return pos if time.time() - stamp <= max_age else None

    def wait_tracked_fruit(self, fruit_id, timeout=1.5):
        deadline = time.time() + timeout
        while time.time() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            pos = self.tracked_fruit(fruit_id, max_age=0.5)
            if pos is not None:
                return pos
        return None

    def home_tool_quat(self):
        """홈 자세의 harvest_tcp 방향.

        목표 제약 링크도 harvest_tcp이므로 같은 링크의 방향을 사용한다. tool0 방향을
        사용하면 어댑터의 Z축 180°가 중복 적용돼 U자 받침이 ∩자로 뒤집힌다.
        """
        req = GetPositionFK.Request()
        req.header.frame_id = FRAME
        req.fk_link_names = ["harvest_tcp"]
        req.robot_state.joint_state.name = JOINTS
        req.robot_state.joint_state.position = [HOME_Q[j] for j in JOINTS]
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5)
        res = fut.result()
        if res is None or not res.pose_stamped:
            raise RuntimeError("compute_fk 실패")
        p = res.pose_stamped[0].pose
        print(f"홈 harvest_tcp: pos=({p.position.x:.3f},{p.position.y:.3f},{p.position.z:.3f}) "
              f"quat=({p.orientation.w:.3f},{p.orientation.x:.3f},"
              f"{p.orientation.y:.3f},{p.orientation.z:.3f})")
        return (p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z), \
               (p.position.x, p.position.y, p.position.z)

    def goal_pose(self, pos, quat, pipeline, planner="", vel=0.3):
        g = MoveGroup.Goal()
        r = g.request
        r.group_name = "ur_manipulator"
        r.allowed_planning_time = 5.0
        r.num_planning_attempts = 3
        r.max_velocity_scaling_factor = vel
        r.max_acceleration_scaling_factor = 0.3
        r.pipeline_id = pipeline
        r.planner_id = planner
        c = Constraints()
        pc = PositionConstraint()
        pc.header.frame_id = FRAME
        # SRDF planning tip과 동일한 실제 패드 중점. tool0를 쓰면 커플러+손가락
        # 길이(131.9 mm)만큼 과진입한다.
        pc.link_name = "harvest_tcp"
        position_tolerance = float(os.environ.get(
            "GRASP_POSITION_TOLERANCE_M", "0.0025"))
        prim = SolidPrimitive(
            type=SolidPrimitive.SPHERE, dimensions=[position_tolerance])
        bv = BoundingVolume()
        bv.primitives = [prim]
        centre = Pose()
        centre.position.x, centre.position.y, centre.position.z = pos
        centre.orientation.w = 1.0
        bv.primitive_poses = [centre]
        pc.constraint_region = bv
        pc.weight = 1.0
        oc = OrientationConstraint()
        oc.header.frame_id = FRAME
        oc.link_name = "harvest_tcp"
        oc.orientation = Quaternion(w=quat[0], x=quat[1], y=quat[2], z=quat[3])
        orientation_tolerance = float(os.environ.get(
            "GRASP_ORIENTATION_TOLERANCE_RAD", "0.035"))
        oc.absolute_x_axis_tolerance = orientation_tolerance
        oc.absolute_y_axis_tolerance = orientation_tolerance
        oc.absolute_z_axis_tolerance = orientation_tolerance
        oc.weight = 1.0
        c.position_constraints = [pc]
        c.orientation_constraints = [oc]
        r.goal_constraints = [c]
        return g

    def goal_circ(self, pos, quat, interim, vel=0.10):
        """Pilz CIRC 원호 목표.

        현재 TCP에서 ``interim``을 반드시 지나 ``pos``로 이동한다. Pilz 규약상
        보조점은 path_constraints.name="interim"과 정확히 한 개의
        PositionConstraint로 전달해야 한다.
        """
        g = self.goal_pose(
            pos, quat, pipeline="pilz_industrial_motion_planner",
            planner="CIRC", vel=vel)
        aux = Constraints()
        aux.name = "interim"
        pc = PositionConstraint()
        pc.header.frame_id = FRAME
        pc.link_name = "harvest_tcp"
        # Pilz는 이 구의 중심만 CIRC 보조점으로 사용한다. 반면 Humble의 공통
        # PlanningPipeline은 같은 객체를 일반 path constraint로도 재검사하므로,
        # 원호 전체를 포함할 반경을 줘야 생성된 정상 CIRC가 -2로 폐기되지 않는다.
        tolerance = float(os.environ.get("CIRC_INTERIM_TOLERANCE_M", "0.25"))
        pc.constraint_region.primitives = [
            SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[tolerance])]
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = interim
        pose.orientation.w = 1.0
        pc.constraint_region.primitive_poses = [pose]
        pc.weight = 1.0
        aux.position_constraints = [pc]
        g.request.path_constraints = aux
        return g

    def goal_joints(self, qmap, vel=0.4, pipeline="pilz_industrial_motion_planner",
                    planner="PTP"):
        g = MoveGroup.Goal()
        r = g.request
        r.group_name = "ur_manipulator"
        r.allowed_planning_time = 5.0
        r.num_planning_attempts = 3
        r.max_velocity_scaling_factor = vel
        r.max_acceleration_scaling_factor = 0.3
        # 기본 Pilz PTP(결정적). 큰 재구성은 OMPL 로 충돌회피 계획하도록 pipeline 지정 가능.
        r.pipeline_id = pipeline
        r.planner_id = planner
        c = Constraints()
        c.joint_constraints = [
            JointConstraint(joint_name=j, position=q, tolerance_above=0.01,
                            tolerance_below=0.01, weight=1.0)
            for j, q in qmap.items()]
        r.goal_constraints = [c]
        return g

    def solve_ik(self, pos, quat, seed):
        """pregrasp 관절해 — 홈 시드로 근처 브랜치를 강제 (OMPL 이 pan 4.57 같은
        반대편 해를 골라 크게 휘두르는 것 방지)."""
        req = GetPositionIK.Request()
        r = req.ik_request
        r.group_name = "ur_manipulator"
        r.avoid_collisions = True
        r.timeout.sec = 2
        r.robot_state.joint_state.name = JOINTS
        r.robot_state.joint_state.position = [seed[j] for j in JOINTS]
        ps = PoseStamped()
        ps.header.frame_id = FRAME
        ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = pos
        ps.pose.orientation = Quaternion(w=quat[0], x=quat[1], y=quat[2], z=quat[3])
        r.pose_stamped = ps
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            print(f"IK 실패: {None if res is None else res.error_code.val}")
            return None
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        # KDL 이 감긴 표현(예: wrist_2 −5.25rad ≡ +1.03)으로 반환하면 그대로 관절목표로
        # 보낼 때 손목을 −300° 돌리려다 계획이 실패한다. 각 관절을 시드 쪽으로 언랩해
        # 실행가능한(작은 이동) 등가 해로 만든다(2026-07-22 진단: 측면 과실 감김 원인).
        def unwrap(v, ref):
            while v - ref > math.pi:
                v -= 2 * math.pi
            while v - ref < -math.pi:
                v += 2 * math.pi
            return v
        out = {j: unwrap(sol[j], seed[j]) for j in JOINTS}
        print("IK 해:", {j: round(v, 2) for j, v in out.items()})
        return out

    def run_goal(self, goal, tag):
        self.phase = tag
        fut = self.move.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=15)
        h = fut.result()
        if h is None or not h.accepted:
            print(f"[{tag}] goal 거부"); return False
        rfut = h.get_result_async()
        rclpy.spin_until_future_complete(self, rfut, timeout_sec=90)
        res = rfut.result()
        code = res.result.error_code.val if res else None
        print(f"[{tag}] error_code={code}")
        return code == 1

    SCOOP_OPEN = (0.0, -math.pi / 2.0, -math.pi)
    SCOOP_CLOSED = (0.0, 0.0, 0.0)
    SCOOP_CUT = (0.0, 0.0, math.radians(50.0))

    def _grip(self, positions):
        values = [float(v) for v in positions]
        if len(values) != 3:
            raise ValueError("스쿱 명령은 [inner, middle, cutter] 3개여야 합니다")
        self.grip.publish(Float64MultiArray(data=values))

    def gripper(self, closed):
        self._grip(self.SCOOP_CLOSED if closed else self.SCOOP_OPEN)

    def grip_width(self, w):
        """2F-85 호환 호출. 새 스쿱은 폭 대신 정해진 닫힘 자세를 쓴다."""
        self._grip(self.SCOOP_CLOSED)

    def set_grip_effort(self):
        """스쿱 effort 상한은 Isaac/ros2_control 설정에서 고정한다."""

    def hold_grip_force(self):
        """닫힌 1/4구 세 장의 위치 drive가 수용 상태를 유지한다."""
        self._grip(self.SCOOP_CLOSED)

    def cutter(self, cutting):
        """외측 1/4구만 +50° 회전해 줄기를 자르거나 닫힘 위치로 복귀한다."""
        self._grip(self.SCOOP_CUT if cutting else self.SCOOP_CLOSED)

    def report_grip(self):
        """Isaac의 실제 finger 위치·속도·effort를 한 번 요청한다."""
        self.grip_measurement = {}
        self.cmd.publish(String(data=json.dumps({"report_grip": True})))
        self.spin_for(0.2)
        return dict(self.grip_measurement)

    def report_grasp_contact(self, chassis_pos):
        """Isaac의 좌·우 패드↔GripStem 간격과 줄기 상대 위치를 한 번 요청한다."""
        self.contact_measurement = {}
        self.cmd.publish(String(data=json.dumps({"report_grasp_contact": {
            "position": [float(v) for v in chassis_pos]}})))
        # RTX 렌더링 중에는 command→status 왕복이 0.25초를 넘는다. 고정 sleep 대신
        # 실제 응답이 올 때까지 기다려 다음 요청이 직전 응답을 지우지 않게 한다.
        deadline = time.time() + float(os.environ.get("GRIP_CONTACT_TIMEOUT_SEC", "2.5"))
        while time.time() < deadline and not self.contact_measurement:
            # 같은 status 토픽의 60Hz finger 텔레메트리보다 빨리 큐를 소비해야 그 뒤에
            # 들어온 grasp_contact 응답까지 도달한다(50ms spin은 20Hz라 영원히 밀림).
            rclpy.spin_once(self, timeout_sec=0.005)
        return dict(self.contact_measurement)

    def verify_bilateral_hold(self, chassis_pos, duration=1.0, after_cut=False):
        """일정 시간 양면 접촉·관절 정착·줄기 상대위치 드리프트를 함께 확인한다."""
        deadline = time.time() + max(0.2, float(duration))
        samples = []
        wanted = int(os.environ.get("GRIP_VERIFY_SAMPLES", "3"))
        while time.time() < deadline and len(samples) < wanted:
            contact = self.report_grasp_contact(chassis_pos)
            grip = self.report_grip()
            # report_grip은 60Hz status 큐에 묻힐 수 있다. finger 위치는 별도
            # hw_joint_states에서 계속 수신하므로 응답이 없을 때 그 실측값을 사용한다.
            if "finger" not in grip and "finger_joint" in self.isaac_js:
                grip = {"finger": float(self.isaac_js["finger_joint"])}
            if contact:
                samples.append((contact, grip))
            self.spin_for(0.1)
        if len(samples) < 2:
            print("  파지 유지검증: Isaac 접촉 계측 응답 부족", flush=True)
            return False
        # 첫 샘플은 위치→토크 전환 과도응답이므로 정착 판정에서 제외한다.
        stable_samples = samples[-2:]
        bilateral = all(bool(c.get("grasp_contact")) for c, _ in stable_samples)
        fingers = [g.get("finger") for _, g in stable_samples if "finger" in g]
        vels = [abs(g.get("vel", 0.0)) for _, g in stable_samples]
        rels = [c.get("stem_rel") for c, _ in stable_samples
                if len(c.get("stem_rel", [])) == 3]
        finger_drift = max(fingers) - min(fingers) if len(fingers) >= 2 else float("inf")
        rel_drift = 0.0
        if len(rels) >= 2:
            rel0 = rels[0]
            rel_drift = max(math.sqrt(sum(
                (float(r[i]) - float(rel0[i])) ** 2 for i in range(3))) for r in rels[1:])
        max_finger_drift = float(os.environ.get("GRIP_MAX_FINGER_DRIFT_RAD", "0.015"))
        max_rel_drift = float(os.environ.get(
            "GRIP_MAX_REL_DRIFT_M", "0.008" if after_cut else "0.004"))
        max_vel = float(os.environ.get("GRIP_MAX_SETTLED_VEL", "0.50"))
        # 이 2F-85 에셋은 finger 위치가 고정돼도 get_joint_velocities가 ±2.556rad/s를
        # 반환하는 mimic 버그가 있다. 기본은 위치/상대좌표 드리프트를 쓰고 명시할 때만
        # 속도 게이트를 추가한다.
        require_vel = os.environ.get("GRIP_REQUIRE_VEL", "0") == "1"
        settled = (not require_vel) or (bool(vels) and max(vels) <= max_vel)
        ok = bilateral and settled and finger_drift <= max_finger_drift and rel_drift <= max_rel_drift
        print("  파지 유지검증: "
              f"양면={bilateral}, fingerΔ={finger_drift:.4f}rad, "
              f"상대Δ={rel_drift*1000:.1f}mm, "
              f"vmax={f'{max(vels):.4f}' if vels else 'n/a'} "
              f"→ {'통과' if ok else '실패'}", flush=True)
        return ok

    def cut(self, chassis_pos, fruit_id=None):
        self._cut_seq += 1
        cut_id = self._cut_seq
        request = {
            "id": cut_id, "position": list(chassis_pos), "max_distance": 0.05}
        if fruit_id is not None:
            request["fruit_id"] = int(fruit_id)
        self.cmd.publish(String(data=json.dumps({"cut_fruit": request})))
        return cut_id

    def request_scene_attach(self, chassis_pos, fruit_id, timeout=3.0):
        """선택 과실이 실제 스쿱 안에 있을 때만 Isaac FixedJoint 부착을 승인받는다."""
        self._attach_seq += 1
        attach_id = self._attach_seq
        request = {
            "attach_id": attach_id,
            "position": [round(float(v), 4) for v in chassis_pos],
        }
        if fruit_id is not None:
            request["fruit_id"] = int(fruit_id)
        self.cmd.publish(String(data=json.dumps({"attach_fruit": request})))
        deadline = time.time() + timeout
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if attach_id in self.attach_results:
                return self.attach_results.pop(attach_id)
        return None

    def wait_cut_result(self, cut_id, timeout=3.0):
        deadline = time.time() + timeout
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if cut_id in self.cut_results:
                return self.cut_results.pop(cut_id)
        return None

    def drop_air(self):
        """공중과실 테스트 — 절단 시 과실을 dynamic(중력 ON) → 마찰 파지 검증."""
        self.cmd.publish(String(data=json.dumps({"drop_air": True})))

    def reset_air(self):
        """공중과실을 kinematic+스폰위치로 복귀 — Isaac 재시작 없이 스윕 반복."""
        self.cmd.publish(String(data=json.dumps({"reset_air": True})))

    # ── MoveIt planning scene: 과실 장애물 스폰 + 어태치 (사용자 요청) ──
    EE_TOUCH = [
        "scoop_adapter", "scoop_quarter_1", "scoop_quarter_2",
        "cutter_quarter_3", "harvest_tcp", "wrist_3_link",
        "wrist_2_link", "flange", "tool0", "ft_frame",
    ]
    OBJ = "target_fruit"

    def _fetch_acm(self):
        req = GetPlanningScene.Request()
        req.components.components = PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
        fut = self.get_scene.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5)
        return fut.result().scene.allowed_collision_matrix

    def add_fruit_object(self, pos, r=0.03):
        """과실을 CollisionObject 로 스폰 + 그리퍼 링크와 충돌 허용(접근·파지 통과)."""
        acm = self._fetch_acm()
        if self.OBJ not in acm.entry_names:
            acm.entry_names.append(self.OBJ)
            for e in acm.entry_values:
                e.enabled.append(False)
            acm.entry_values.append(
                AllowedCollisionEntry(enabled=[False] * len(acm.entry_names)))
        fi = acm.entry_names.index(self.OBJ)
        for link in self.EE_TOUCH:
            if link in acm.entry_names:
                li = acm.entry_names.index(link)
                acm.entry_values[fi].enabled[li] = True
                acm.entry_values[li].enabled[fi] = True
        co = CollisionObject()
        co.header.frame_id = FRAME
        co.id = self.OBJ
        co.primitives = [SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[float(r)])]
        p = Pose()
        p.position.x, p.position.y, p.position.z = pos
        p.orientation.w = 1.0
        co.primitive_poses = [p]
        co.operation = CollisionObject.ADD
        ps = PlanningScene(is_diff=True)
        ps.world.collision_objects = [co]
        ps.allowed_collision_matrix = acm
        fut = self.apply_scene.call_async(ApplyPlanningScene.Request(scene=ps))
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5)
        print(f"[Scene] 과실 장애물 스폰 {tuple(round(v,3) for v in pos)} r={r} (그리퍼 충돌 허용)")

    def attach_fruit(self, r=0.03):
        """파지 시 world 과실을 harvest_tcp 에 어태치 — 이송을 충돌회피 계획."""
        aco = AttachedCollisionObject()
        aco.link_name = "harvest_tcp"
        aco.object.header.frame_id = "harvest_tcp"
        aco.object.id = self.OBJ
        aco.object.primitives = [SolidPrimitive(type=SolidPrimitive.SPHERE,
                                                dimensions=[float(r)])]
        pp = Pose(); pp.orientation.w = 1.0
        aco.object.primitive_poses = [pp]
        aco.object.operation = CollisionObject.ADD
        aco.touch_links = self.EE_TOUCH
        ps = PlanningScene(is_diff=True)
        ps.robot_state.is_diff = True
        ps.robot_state.attached_collision_objects = [aco]
        ps.world.collision_objects = [
            CollisionObject(id=self.OBJ, operation=CollisionObject.REMOVE)]
        fut = self.apply_scene.call_async(ApplyPlanningScene.Request(scene=ps))
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5)
        print("[Scene] 과실 어태치(harvest_tcp) — 이송 충돌회피")

    STEM_OBJ = "target_stem"

    def add_stem_object(self, fruit_pos, up=0.30, r=0.025):
        """과실 위 pedicel/줄기 구간을 수직 실린더 CollisionObject 로 등록 — 팔이 줄기를
        뚫고 접근하는 걸 막는다. EE 링크는 ACM 허용(과실·절단 접근은 가능)."""
        acm = self._fetch_acm()
        if self.STEM_OBJ not in acm.entry_names:
            acm.entry_names.append(self.STEM_OBJ)
            for e in acm.entry_values:
                e.enabled.append(False)
            acm.entry_values.append(
                AllowedCollisionEntry(enabled=[False] * len(acm.entry_names)))
        fi = acm.entry_names.index(self.STEM_OBJ)
        for link in self.EE_TOUCH:
            if link in acm.entry_names:
                li = acm.entry_names.index(link)
                acm.entry_values[fi].enabled[li] = True
                acm.entry_values[li].enabled[fi] = True
        co = CollisionObject()
        co.header.frame_id = FRAME
        co.id = self.STEM_OBJ
        co.primitives = [SolidPrimitive(type=SolidPrimitive.CYLINDER,
                                        dimensions=[float(up), float(r)])]  # [높이, 반경]
        p = Pose()
        p.position.x, p.position.y = fruit_pos[0], fruit_pos[1]
        p.position.z = fruit_pos[2] + up / 2.0 + 0.03      # 과실 바로 위부터 위로
        p.orientation.w = 1.0
        co.primitive_poses = [p]
        co.operation = CollisionObject.ADD
        ps = PlanningScene(is_diff=True)
        ps.world.collision_objects = [co]
        ps.allowed_collision_matrix = acm
        fut = self.apply_scene.call_async(ApplyPlanningScene.Request(scene=ps))
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5)
        print(f"[Scene] 줄기 장애물 XY={tuple(round(v,3) for v in fruit_pos[:2])} "
              f"↑{up}m r={r} (EE ACM 허용)")

    def remove_object(self, obj_id):
        """플래닝씬에서 collision object 제거(스폰해제)."""
        ps = PlanningScene(is_diff=True)
        ps.world.collision_objects = [
            CollisionObject(id=obj_id, operation=CollisionObject.REMOVE)]
        fut = self.apply_scene.call_async(ApplyPlanningScene.Request(scene=ps))
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5)

    def despawn_fruit(self):
        """작업완료: 어태치된 과실을 로봇에서 떼고 플래닝씬에서도 제거(스폰해제)."""
        aco = AttachedCollisionObject()
        aco.link_name = "harvest_tcp"
        aco.object.id = self.OBJ
        aco.object.operation = CollisionObject.REMOVE
        ps = PlanningScene(is_diff=True)
        ps.robot_state.is_diff = True
        ps.robot_state.attached_collision_objects = [aco]
        ps.world.collision_objects = [
            CollisionObject(id=self.OBJ, operation=CollisionObject.REMOVE)]
        fut = self.apply_scene.call_async(ApplyPlanningScene.Request(scene=ps))
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5)
        print("[Scene] 과실 어태치 해제 + 스폰해제")


def harvest_once(g: Grasp) -> bool:
    """수확 원샷: 최근접 과실 → 열기 → GRASP 로 직접 접근(OMPL 관절목표, 충돌회피) →
    파지 → 절단 → 홈. 실패 시 False (h 키 트리거 재사용 — sys.exit 금지).

    별도 pregrasp+LIN 을 안 쓴다: pregrasp 를 과실에서 뒤로 빼면 수평접근 IK 가 도달권을
    벗어나 반대 브랜치(pan −117°)로 도망가 PTP 가 실패했다(2026-07-22 진단). grasp 포즈는
    깨끗한 해(pan −42°)가 있고, 과실은 절단 전까지 kinematic 이라 직접 접근이 안전하다."""
    # ★접촉검증형 grasp constraint(기본): 패드 접촉폭을 먼저 확인한 경우에만 FixedJoint를
    #   생성한다. PhysX 강체 마찰만으로는 유연한 고무패드×얇은 줄기를 재현하지 못해 절단
    # 동축 U자 스쿱은 과실을 아래에서 받치므로 기본은 순수 충돌 물리 운반이다.
    # FixedJoint는 비교/디버그가 필요할 때만 ATTACH_GRASP=1로 명시한다.
    attach_mode = os.environ.get("ATTACH_GRASP", "0") == "1"
    # 이전 낙하 과실 리셋(kinematic 복귀) — 스윕 반복. 과실이 스폰높이로 돌아올 때까지 검증.
    for attempt in range(4):
        g.reset_air(); g.spin_for(2.0)
        fr = g.nearest()
        if fr is not None and fr[2] > 0.5:     # 스폰높이(~1.0) 복귀 확인
            break
        print(f"  [reset] 재시도 {attempt+1} (현재 과실 z={fr[2] if fr else None})")
    fruit = g.nearest()
    if fruit is None:
        print("작업영역 내 과실 없음"); return False
    fruit_id = g.fruit_ids.get(fruit)
    print(f"대상 과실(섀시): {fruit}, fruit_id={fruit_id}")
    quat, home_pos = g.home_tool_quat()

    # mm_base(섀시) 프레임 목표. ★MoveIt 그룹 tip = harvest_tcp(파지중심, wrist_3+0.127z)
    # 이고 solve_ik 가 그 tip 을 겨냥한다 → 과실을 '그대로' 목표로 줘야 파지중심이 과실에
    # 온다. 예전엔 fruit−TOOL_LEN 을 줘서 파지중심이 과실 11.5cm 앞에 서 빈손이었다(실측
    # 덤프로 harvest_tcp==Isaac HarvestTCP 확인, 2026-07-22).
    fx, fy, fz = fruit[0], fruit[1], fruit[2]
    yaw = math.atan2(fy, fx)
    # 적도정렬 스윕용 오프셋 — GRASP_ZOFF(수직), GRASP_XOFF(접근/이랑방향). 단위 m.
    zoff = float(os.environ.get("GRASP_ZOFF", "0"))
    xoff = float(os.environ.get("GRASP_XOFF", "0"))
    grasp = (fx + xoff, fy, fz + zoff)
    if zoff or xoff:
        print(f"  [스윕] grasp 오프셋 z={zoff:+.3f} x={xoff:+.3f} → 타겟 {grasp}")
    # 새 스쿱은 줄기를 집지 않는다. TCP는 항상 과실 중심을 겨냥하고, 줄기는 마지막
    # 외측 셸이 지나가며 절단할 위치일 뿐이다.
    held_max = float(os.environ.get("GRIP_CONTACT_MAX_RAD", "0.70"))
    # 홈 harvest_tcp 방향(홈 wrist_1=180° 라 커터·D455 가 이미 파지점 위)을 과실 방위로
    # 요 회전만. 예전 180° 접근축 롤은 지그가 +Y 에 잘못 있어 카메라가 아래로 보이던 때의
    # 임시방편 — 지그를 −Y 로 바로잡고 모델 동기화 후 불필요(제거, 2026-07-22 사용자 지적).
    q = qmul((math.cos(yaw/2), 0.0, 0.0, math.sin(yaw/2)), quat)
    print(f"과실=({fx:.3f},{fy:.3f},{fz:.3f}) yaw={math.degrees(yaw):.1f}° "
          f"(고정 장착된 U자 아래받침)")

    g.gripper(False); g.spin_for(1.0)                       # 열고
    # g.add_fruit_object(grasp, r=0.03)   # 장애물 스폰 — OMPL 99999 유발(ACM 미흡), 별도 수정 후 재활성
    # 스쿱은 과실을 받으므로 가짜 줄기 원통을 기본 planning obstacle로 만들지 않는다.
    # 실제 줄기 회피 실험이 필요할 때만 STEM_OBSTACLE=1로 켠다.
    stem_obs = os.environ.get("STEM_OBSTACLE", "0") == "1"
    if stem_obs:
        g.add_stem_object(fruit)
    # 2단계 접근: OMPL로 pregrasp까지 충돌회피 → Pilz CIRC로 아래에서 퍼올리듯 수용.
    #   pregrasp = 과실서 접근축(yaw) 뒤로 −CLEARANCE. pregrasp IK 를 grasp_q 로 시딩해 같은
    #   브랜치 유지(07-22 LIN 실패는 HOME 시드라 반대 브랜치 pan−117° 로 도망 — 시드로 해결).
    #   직진 삽입이라 잎·줄기 헛건드림 없이 그리퍼가 과실 중심에 정합(빈손 방지).
    # 스쿱 입구를 먼저 과실보다 아래·뒤에 놓는다. 여기서부터 원호로 상승해야
    # 토마토를 옆으로 밀지 않고 아이스크림 스쿱처럼 밑에서 받아 올릴 수 있다.
    entry_back = float(os.environ.get("CIRC_ENTRY_BACK_M", str(CLEARANCE)))
    entry_drop = float(os.environ.get("CIRC_ENTRY_DROP_M", "0.10"))
    pre = (grasp[0] - entry_back * math.cos(yaw),
           grasp[1] - entry_back * math.sin(yaw),
           grasp[2] - entry_drop)
    # OMPL은 과실에서 충분히 떨어진 안전 대기점까지만 담당한다. 이후 직선 진입점까지
    # Pilz LIN으로 자세를 유지하고, 마지막 수용 구간만 CIRC로 받쳐 올린다.
    lin_approach = float(os.environ.get("LIN_APPROACH_M", "0.10"))
    safe = (pre[0] - lin_approach * math.cos(yaw),
            pre[1] - lin_approach * math.sin(yaw),
            pre[2])
    circ_sag = float(os.environ.get("CIRC_SAG_M", "0.025"))
    circ_interim = (
        0.5 * (pre[0] + grasp[0]),
        0.5 * (pre[1] + grasp[1]),
        0.5 * (pre[2] + grasp[2]) - circ_sag,
    )
    print("  [Pilz CIRC 밑받침] "
          f"start={tuple(round(v, 3) for v in pre)} → "
          f"interim={tuple(round(v, 3) for v in circ_interim)} → "
          f"goal={tuple(round(v, 3) for v in grasp)}", flush=True)
    # IK도 실제 실행 방향으로 푼다. 예전에는 grasp→pre→safe 역순으로 풀어 HOME에서
    # 최종 grasp 해가 바로 안 나오면 접근 가능한 목표도 시작 전에 버렸다. 이제 접힌
    # HOME→safe→pre→grasp 순서로 같은 관절 브랜치를 이어 팔을 점진적으로 뻗는다.
    safe_q = g.solve_ik(safe, q, HOME_Q)
    pre_q = None if safe_q is None else g.solve_ik(pre, q, safe_q)
    grasp_q = None if pre_q is None else g.solve_ik(grasp, q, pre_q)
    if safe_q is None or pre_q is None or grasp_q is None:
        failed_stage = (
            "safe" if safe_q is None else
            "pregrasp" if pre_q is None else "grasp")
        print(f"{failed_stage} IK 실패 — 접힘 자세에서 전방 신장 경로를 만들 수 없음",
              flush=True)
        if stem_obs:
            g.remove_object(g.STEM_OBJ)
        return False
    if safe_q is not None and g.run_goal(
            g.goal_joints(safe_q, vel=0.25, pipeline="ompl", planner=""),
            "APPROACH(OMPL)"):
        # 멀리서 얻은 좌표를 그대로 쓰지 않는다. OMPL 안전점에 도착한 뒤 같은 과실의
        # 최신 3-D 좌표를 다시 받아 LIN/CIRC 구간을 갱신한다. 큰 점프는 오검출/낙하로
        # 보고 중단하고, 정상적인 잎 흔들림·검출 오차만 제한적으로 보정한다.
        corrected = g.wait_tracked_fruit(fruit_id)
        # Nav2 최종 yaw 정착에 따른 섀시 좌표 재표현까지 포함한다. 12cm 이내는 같은
        # fruit_id의 최신 좌표로 다시 계획하고, 그 이상만 실제 이탈/오검출로 중단한다.
        max_reacquire = float(os.environ.get("REACQUIRE_MAX_M", "0.12"))
        if corrected is None:
            print("접근 후 과실 재검출 실패 — 오래된 좌표로 진입하지 않음", flush=True)
            return False
        corrected_grasp = (
            corrected[0] + xoff, corrected[1], corrected[2] + zoff)
        reacquire_delta = tuple(
            corrected_grasp[i] - grasp[i] for i in range(3))
        reacquire_error = math.sqrt(sum(v*v for v in reacquire_delta))
        if reacquire_error > max_reacquire:
            print(f"접근 후 과실 이동 {reacquire_error*1000:.0f}mm > "
                  f"{max_reacquire*1000:.0f}mm — 진입 중단", flush=True)
            return False
        if reacquire_error > 0.003:
            grasp = corrected_grasp
            yaw = math.atan2(grasp[1], grasp[0])
            q = qmul((math.cos(yaw/2), 0.0, 0.0, math.sin(yaw/2)), quat)
            pre = (grasp[0] - entry_back * math.cos(yaw),
                   grasp[1] - entry_back * math.sin(yaw),
                   grasp[2] - entry_drop)
            circ_interim = (
                0.5 * (pre[0] + grasp[0]),
                0.5 * (pre[1] + grasp[1]),
                0.5 * (pre[2] + grasp[2]) - circ_sag)
            pre_q = g.solve_ik(pre, q, safe_q)
            grasp_q = None if pre_q is None else g.solve_ik(grasp, q, pre_q)
            if pre_q is None or grasp_q is None:
                print("재검출 좌표 IK 실패 — 진입 중단", flush=True)
                return False
            print(f"  [접근 중 보정] Δ={tuple(round(v*1000) for v in reacquire_delta)}mm",
                  flush=True)
        if not g.run_goal(g.goal_pose(
                pre, q, pipeline="pilz_industrial_motion_planner",
                planner="LIN", vel=0.08), "PREGRASP(Pilz LIN)"):
            print("LIN 진입 실패 — CIRC 시작 전 중단", flush=True)
            if stem_obs:
                g.remove_object(g.STEM_OBJ)
            return False
        if not g.run_goal(
                g.goal_circ(grasp, q, circ_interim, vel=0.10),
                "GRASP(Pilz CIRC)"):
            # 접촉 경로가 달라지는 OMPL 폴백은 파지 물리 검증을 오염시킨다.
            # 장애물 없는 레거시 데모만 명시적으로 허용한다.
            if os.environ.get("ALLOW_NONLINEAR_GRASP_FALLBACK") == "1":
                print("CIRC 실패 — 명시적 OMPL 폴백", flush=True)
                if not g.run_goal(g.goal_joints(
                        grasp_q, vel=0.25, pipeline="ompl", planner=""),
                        "GRASP(OMPL 폴백)"):
                    if stem_obs:
                        g.remove_object(g.STEM_OBJ)
                    return False
            else:
                print("CIRC 실패 — 폴백 없이 중단", flush=True)
                if stem_obs:
                    g.remove_object(g.STEM_OBJ)
                return False
        # CIRC 중 과실이 밀렸으면 닫기 전에 짧은 저속 LIN으로 수용 중심을 한 번 더 맞춘다.
        # 70mm 밖은 이미 스쿱 밖으로 이탈한 것이므로 쫓아가 절단하지 않는다.
        live = g.wait_tracked_fruit(fruit_id)
        if live is None:
            print("CIRC 후 과실 추적 소실 — 스쿱을 닫지 않음", flush=True)
            return False
        live_grasp = (live[0] + xoff, live[1], live[2] + zoff)
        trim_delta = tuple(live_grasp[i] - grasp[i] for i in range(3))
        trim_error = math.sqrt(sum(v*v for v in trim_delta))
        trim_abort = float(os.environ.get("CAPTURE_ABORT_M", "0.07"))
        trim_deadband = float(os.environ.get("CAPTURE_DEADBAND_M", "0.008"))
        trim_limit = float(os.environ.get("CAPTURE_TRIM_MAX_M", "0.035"))
        if trim_error > trim_abort:
            print(f"CIRC 중 과실 이탈 {trim_error*1000:.0f}mm — 절단 금지", flush=True)
            return False
        if trim_error > trim_deadband:
            scale = min(1.0, trim_limit / trim_error)
            shift = tuple(v * scale for v in trim_delta)
            trim_goal = tuple(grasp[i] + shift[i] for i in range(3))
            if not g.run_goal(g.goal_pose(
                    trim_goal, q, pipeline="pilz_industrial_motion_planner",
                    planner="LIN", vel=0.035), "CAPTURE_TRIM(Pilz LIN)"):
                print("과실 중심 미세보정 실패 — 절단 금지", flush=True)
                return False
            grasp = trim_goal
            pre = tuple(pre[i] + shift[i] for i in range(3))
            circ_interim = tuple(circ_interim[i] + shift[i] for i in range(3))
            print(f"  [수용 직전 보정] Δ={tuple(round(v*1000) for v in shift)}mm",
                  flush=True)
    else:
        print("pregrasp OMPL 실패 — LIN 시작점이 없으므로 중단", flush=True)
        if stem_obs:
            g.remove_object(g.STEM_OBJ)
        return False
    if stem_obs:            # 도달 완료 — 파지·절단·복귀는 줄기 장애물 없이
        g.remove_object(g.STEM_OBJ)

    # 동축 스쿱 시퀀스: 두 수용 셸과 외측 셸을 0°로 모아 과실을 감싼 뒤,
    # 외측 셸만 +50° 더 회전해 상단 슬릿의 줄기를 전단한다.
    g.gripper(True)
    g.spin_for(1.5)
    print("  스쿱 닫힘: [0°, 0°, 0°] — 과실 수용", flush=True)
    live = g.wait_tracked_fruit(fruit_id)
    capture_error = float("inf") if live is None else math.dist(live, grasp)
    capture_tolerance = float(os.environ.get("SCOOP_CAPTURE_TOLERANCE_M", "0.06"))
    if capture_error > capture_tolerance:
        print(f"스쿱 닫힘 후 과실 수용 실패: 중심오차="
              f"{capture_error*1000:.0f}mm > {capture_tolerance*1000:.0f}mm",
              flush=True)
        g.gripper(False)
        return False
    print(f"  스쿱 수용 검증: 과실 중심오차 {capture_error*1000:.0f}mm", flush=True)
    if attach_mode:
        attach_ok = g.request_scene_attach(fruit, fruit_id)
        if attach_ok is not True:
            print(f"과실 부착 검증 실패({attach_ok}) — pedicel 절단 금지", flush=True)
            g.gripper(False)
            return False
        g.attach_fruit(r=0.045)
        print("  과실 constraint 부착 확인(ATTACH_GRASP=1)", flush=True)

    g.cutter(True)
    # 실제 관절/메시 간섭 한계가 약 41.3°이므로 물리 운반 여부와 무관하게 40°에서
    # 절삭 위치 도달로 판정한다. 49°를 요구하면 정상 커터도 영원히 실패한다.
    cut_target = 40.0
    if not g.wait_blade_closed(target=cut_target):
        print(f"외측 커터 미닫힘: 현재 {g.blade_ang:.1f}°/{cut_target:.0f}°", flush=True)
        g.cutter(False)
        g.gripper(False)
        return False
    print(f"  외측 커터 절삭 위치 확인: {g.blade_ang:.1f}°", flush=True)
    cut_id = g.cut(grasp, fruit_id)
    cut_ok = g.wait_cut_result(cut_id, timeout=3.0)
    if cut_ok is not True and not attach_mode:
        print(f"절단 검증 실패: cut_id={cut_id}, 응답={cut_ok}", flush=True)
        g.cutter(False)
        g.gripper(False)
        return False
    if cut_ok is not True:
        print("  부착모드: pedicel 없는 테스트 과실은 절단 응답을 생략", flush=True)

    g.cutter(False)  # 외측 셸을 0° 수용 위치로 복귀. 전체 개방은 투하 때만 한다.
    g.spin_for(0.5)
    # 수용 후에도 같은 원호를 역으로 내려가 잎과 과실을 옆으로 긁지 않는다.
    if not g.run_goal(
            g.goal_circ(pre, q, circ_interim, vel=0.08),
            "RETRACT(Pilz CIRC)"):
        print("역방향 CIRC 후퇴 실패 — 수용물을 보호하기 위해 홈 이동 중단", flush=True)
        return False
    if not g.run_goal(g.goal_pose(
            safe, q, pipeline="pilz_industrial_motion_planner",
            planner="LIN", vel=0.08), "RETRACT(Pilz LIN)"):
        print("LIN 후퇴 실패 — 수용물을 보호하기 위해 홈 이동 중단", flush=True)
        return False
    carried = g.wait_tracked_fruit(fruit_id)
    carry_error = float("inf") if carried is None else math.dist(carried, safe)
    carry_tolerance = float(os.environ.get("SCOOP_CARRY_TOLERANCE_M", "0.10"))
    if carry_error > carry_tolerance:
        print(f"이탈 후 과실 유실: 중심오차={carry_error*1000:.0f}mm", flush=True)
        return False
    if not g.run_goal(g.goal_joints(
            HOME_Q, vel=0.2, pipeline="ompl", planner=""), "HOME(OMPL)"):
        return False
    held = g.wait_tracked_fruit(fruit_id)
    held_error = float("inf") if held is None else math.dist(held, home_pos)
    if held_error > carry_tolerance:
        print(f"홈 복귀 후 과실 유실: 중심오차={held_error*1000:.0f}mm — 수확 실패",
              flush=True)
        return False
    print(f"시퀀스 완료: 동축 스쿱 수용·{cut_target:.0f}° 절단·홈 복귀",
          flush=True)
    return True

    # 아래는 구형 2F-85 접촉폭/양면 패드 검증 코드로, 이전 실험 기록을 위해 남겨 둔다.
    def finger(tag):
        g.spin_for(0.3)
        v = g.isaac_js.get("finger_joint", -1)
        held = "물고 있음" if 0.1 < v < held_max else ("빈손!" if v >= held_max else "열림")
        print(f"  finger[{tag}] = {v:.3f} ({held}, 상한 {held_max})")
        return v

    g.set_grip_effort()
    g.gripper(True); g.spin_for(2.0)                        # 완전닫힘 목표 → 대상에 막혀 접촉폭 W
    w = finger("접촉")                                       # 접촉폭 읽기
    if not (0.1 < w < held_max):
        if not attach_mode:
            print("파지 접촉 검증 실패 — 절단하지 않음", flush=True)
            g.gripper(False)
            return False
        print("[부착모드] 접촉폭 미달이나 물리 부착으로 유지 진행", flush=True)
    # ★ 접촉폭 유지로 전환(0.8 안 쫓음) — 절단(dynamic) 후 대상을 적도 너머로 안 밀어
    #   squeeze-pop 회피(2026-07-22 spike grasp_force_test: 마찰·힘 전 조합 유지).
    if 0.0 < w < held_max:
        g.grip_width(w); g.spin_for(0.8)
        measure = g.report_grip()
        if measure:
            print("  Isaac 파지 실측: "
                  f"finger={measure.get('finger', float('nan')):.3f} "
                  f"effort={measure.get('eff', float('nan')):.4f} "
                  f"vel={measure.get('vel', float('nan')):.4f}", flush=True)
            min_effort = float(os.environ.get("MIN_GRIP_EFFORT", "0"))
            if min_effort > 0.0 and abs(measure.get("eff", 0.0)) < min_effort:
                print(f"파지 effort 부족(<{min_effort}) — 절단하지 않음", flush=True)
                g.gripper(False)
                return False
    finger("너비유지")
    g.hold_grip_force()
    hold_measure = g.report_grip()
    if hold_measure:
        print("  토크전환 실측: "
              f"finger={hold_measure.get('finger', float('nan')):.3f} "
              f"effort={hold_measure.get('eff', float('nan')):.4f}",
              flush=True)
    # 순수마찰 모드는 관절각 하나가 아니라 좌·우 패드가 GripStem에 모두 닿고,
    # 일정 시간 손가락/줄기 상대위치가 정착했을 때만 절단한다.
    hold_sec = float(os.environ.get("GRIP_HOLD_VERIFY_SEC", "7.0"))
    bilateral_ok = g.verify_bilateral_hold(fruit, duration=hold_sec)
    if not bilateral_ok and not attach_mode:
        print("양면 접촉 유지검증 실패 — 과실을 자르지 않음", flush=True)
        g.gripper(False)
        return False
    if not bilateral_ok:
        print("[부착모드] 양면 접촉 미달 — 비교용 constraint로 계속", flush=True)
    if attach_mode:
        # 물리 부착: moveit_mm 이 이 위치 최근접 ripe 씬 과실↔그리퍼에 FixedJoint 생성(절단
        # 전에 부착해야 절단=dynamic 전환 순간 과실이 그리퍼에 매달린 채 유지된다).
        attach_request = {"position": [round(float(v), 4) for v in fruit]}
        if fruit_id is not None:
            attach_request["fruit_id"] = int(fruit_id)
        g.cmd.publish(String(data=json.dumps({
            "attach_fruit": attach_request})))
        g.spin_for(1.0)
        print("[부착] 과실을 그리퍼에 물리 부착(FixedJoint) — 절단 후에도 유지", flush=True)
        g.attach_fruit(r=0.03)     # MoveIt ACO — HOME 복귀를 '든 과실 충돌회피'로 계획
    g.cmd.publish(String(data=json.dumps({"blade": 35.0}))) # 칼날 닫기 (BLADE_CLOSED_DEG)
    if not g.wait_blade_closed():                           # sim 느려도 실제 닫힘 확인 후 절단
        print(f"  ⚠ 칼날 미닫힘(현재 {g.blade_ang:.1f}°/34°) — 절단 시도하나 실패 가능", flush=True)
    else:
        print(f"  칼날 닫힘 확인 {g.blade_ang:.1f}°", flush=True)
    cut_sent_at = time.time()
    cut_id = g.cut(fruit, fruit_id)
    cut_ok = g.wait_cut_result(cut_id, timeout=3.0)
    if cut_ok is None:
        # 재시작 전 Isaac처럼 MoveIt 모드에서 cut status를 아직 발행하지 않는 경우의
        # 런타임 검증: 다른 과실 메시지는 계속 오는데 목표만 발행 목록에서 사라졌다면
        # GreenhouseTask가 해당 과실을 harvested로 전환한 것이다.
        target_updated = g.fruits.get(fruit, 0.0) > cut_sent_at
        other_updated = any(
            p != fruit and seen > cut_sent_at for p, seen in g.fruits.items())
        if other_updated and not target_updated:
            cut_ok = True
            print("절단 결과: 목표 과실이 pickable 목록에서 제거됨(호환 검증)", flush=True)
    if cut_ok is not True:
        if not attach_mode:
            print(f"절단 검증 실패: cut_id={cut_id}, 응답={cut_ok}", flush=True)
            g.cmd.publish(String(data=json.dumps({"blade": 0.0})))
            g.gripper(False)
            return False
        # 부착모드 공중과실은 pedicel 조인트가 없어 cut status 가 안 온다 — 칼날은
        # 시연용으로 닫고, 과실은 이미 FixedJoint 로 그리퍼에 붙어 절단검증 불필요.
        print("[부착모드] 공중과실(줄기 없음) — 칼날 시연만, 절단검증 스킵", flush=True)
    g.spin_for(1.5)                                         # 과실 dynamic 후 마찰 유지 확인
    after_cut = finger("절단 직후")
    dynamic_hold_ok = g.verify_bilateral_hold(fruit, duration=float(
        os.environ.get("GRIP_AFTER_CUT_VERIFY_SEC", "7.0")), after_cut=True)
    if not (0.1 < after_cut < held_max) or not dynamic_hold_ok:
        if not attach_mode:
            print("절단 후 양면 파지 유실/미끄러짐", flush=True)
            g.cmd.publish(String(data=json.dumps({"blade": 0.0})))
            g.gripper(False)
            return False
        print("[부착모드] 그리퍼 개방과 무관하게 FixedJoint 로 과실 유지 — 계속", flush=True)
    g.cmd.publish(String(data=json.dumps({"blade": 0.0}))) # 칼날 열기
    g.spin_for(0.5)
    # 줄기를 문 상태에서 곧바로 OMPL로 옆으로 휘두르면 횡력이 걸린다. 진입했던
    # 직선의 역방향으로 pregrasp까지 먼저 빠진 뒤 자유공간 계획으로 전환한다.
    if not g.run_goal(g.goal_pose(
            pre, q, pipeline="pilz_industrial_motion_planner",
            planner="LIN", vel=0.10), "RETRACT(Pilz LIN)"):
        print("직선 후퇴 실패 — 파지물 보호를 위해 홈 이동 중단", flush=True)
        return False
    # 홈 복귀 — 과실을 마찰로 문 채. OMPL 충돌회피.
    if not g.run_goal(g.goal_joints(
            HOME_Q, vel=0.2, pipeline="ompl", planner=""), "HOME(OMPL)"):
        return False
    home_width = finger("홈 도착")
    # 부착모드는 FixedJoint 가 잡으므로 finger 너비와 무관하게 유지(들고 대기 완료).
    held = True if attach_mode else (0.1 < home_width < held_max)
    print(f"시퀀스 완료: cut_success=true, held_at_home={held}"
          f"{' (부착 유지)' if attach_mode else ''}", flush=True)
    return held


def main():
    rclpy.init()
    g = Grasp()
    if not g.move.wait_for_server(timeout_sec=40):   # move_group 로딩 여유(통합 launch 동시기동)
        print("move_action 없음"); sys.exit(1)
    attach = os.environ.get("ATTACH_GRASP", "0") == "1"
    n = int(os.environ.get("HARVEST_N", "1"))     # 반복 수확 횟수
    # 이전 실행서 그리퍼에 붙어있던 과실이 있으면 먼저 놓는다(반복 시작 정리).
    if attach:
        g.cmd.publish(String(data=json.dumps({"detach_grasp": True}))); g.spin_for(1.0)
        g.despawn_fruit()          # MoveIt ACO 도 제거(스폰해제) — 시작 정리
    done = 0
    yolo_gate = os.environ.get("YOLO_GATE") == "1"
    for i in range(n):
        print(f"\n===== 수확 {i + 1}/{n} =====", flush=True)
        if yolo_gate:      # 원거리 YOLO 탐지 대기 → 탐지되면 접근·파지
            print("  [YOLO] 토마토 탐지 대기...", flush=True)
            if not g.wait_yolo(timeout=20.0):
                print("  [YOLO] 탐지 타임아웃 — 중단", flush=True)
                break
            print("  [YOLO] tomato 탐지! → 접근 시작", flush=True)
        if not harvest_once(g):
            print("수확 실패 — 중단", flush=True)
            break
        done += 1
        # 여러 개를 연속 수확할 때만 다음 과실 전에 현재 과실을 놓는다. 마지막 과실은
        # 스쿱 안에 유지해야 "수확 완료"이며, DROP_AFTER_HARVEST=1을 명시한 경우에만 투하한다.
        drop_after = os.environ.get("DROP_AFTER_HARVEST", "0") == "1"
        if i < n - 1 or drop_after:
            print("  [배출] 스쿱 완전 개방 → 중력 배출", flush=True)
            g.gripper(False)
            g.spin_for(1.2)
            # 비교용 ATTACH_GRASP=1에서만 보조 FixedJoint를 해제한다. 기본 물리 모드는
            # 조인트가 없으므로 셸 개방만으로 과실이 떨어진다.
            if attach:
                g.cmd.publish(String(data=json.dumps({"detach_grasp": True})))
            g.spin_for(1.5)
            if attach:
                g.despawn_fruit()  # 비교용 MoveIt ACO 제거
        else:
            print("  마지막 수확물 스쿱 내부 유지", flush=True)
    print(f"\n총 {done}/{n} 수확 완료", flush=True)
    sys.exit(0 if done > 0 else 1)


if __name__ == "__main__":
    main()
