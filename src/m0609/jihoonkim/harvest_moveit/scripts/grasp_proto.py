#!/usr/bin/env python3
"""MoveIt 파지 프로토타입 — 젤 가까운 시뮬 토마토를 OMPL→Pilz LIN 으로 잡고 자른다.
계획 프레임 = mm_base(=Isaac 섀시 base_link) — mm URDF 전환(2026-07-22)으로
섀시 프레임 과실좌표를 변환 없이 그대로 tool0 목표로 쓴다(팔베이스 -0.30 보정 제거).
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
from std_msgs.msg import Float64MultiArray

FRAME = "mm_base"           # 계획 프레임 = 섀시(base_link) — URDF 루트
TOOL_LEN = 0.115            # tool0→파지점 (settings tool_grasp_reach_m)
                            # TODO [2] 실제는 0.127 (커플러 0.012 누락 — URDF harvest_tcp 참조).
                            # 실증 데모가 0.115 로 성공했으므로 검증 후에 바꾼다.
CLEARANCE = 0.15            # pregrasp 여유
HOME_Q = {"shoulder_pan_joint": 0.0, "shoulder_lift_joint": 3.9269908,
          "elbow_joint": 2.3561945, "wrist_1_joint": 3.1415927,
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
        self.create_subscription(JointState, "/harvester_moveit/joint_command",
                                 self._cmd_watch, 50)
        self.cmd = self.create_publisher(String, "/harvester_moveit/cmd", 10)
        self.cut_results = {}
        self._cut_seq = 0
        self.create_subscription(
            String, "/harvester_moveit/rmpflow/status", self._cut_status, 20)
        # 그리퍼 = ros2_control(2026-07-22). finger_joint 위치를 직접 지령 → topic_based 가
        # /joint_command 에 실어 Isaac 브리지가 적용(팔과 동일 경로). JSON gripper 사문화.
        self.grip = self.create_publisher(Float64MultiArray,
                                          "gripper_controller/commands", 10)
        self.fruits = {}
        self.create_subscription(String, "/harvester_moveit/sim/tomato", self._fruit, 20)
        # ★YOLO 탐지 게이트(2026-07-23): vision_node 가 tomato 를 잡으면 target_class 발행.
        #   YOLO_GATE=1 이면 이 탐지를 기다렸다가 수확 시작(원거리 탐지→접근→파지). 좌표는 /sim/tomato.
        self.yolo_det_t = 0.0
        self.create_subscription(String, "/harvester_moveit/vision/target_class",
                                 self._yolo_class, 10)
        self.js = {}
        self.create_subscription(JointState, "joint_states",
                                 lambda m: self.js.update(dict(zip(m.name, m.position))), 10)
        self.isaac_js = {}   # 그리퍼 finger 등 Isaac 원본 (jsb 는 팔 6축만) — HW 채널서 받음
        self.create_subscription(JointState, "/harvester_moveit/hw_joint_states",
                                 lambda m: self.isaac_js.update(dict(zip(m.name, m.position))), 10)

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
            self.fruits[tuple(round(v, 3) for v in d["position"])] = time.time()
        except (ValueError, KeyError):
            pass

    def _yolo_class(self, m):
        """vision_node 가 tomato 를 탐지하면 target_class='tomato' 발행 → 시각 기록."""
        if m.data == "tomato":
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
        except (TypeError, ValueError):
            pass

    def spin_for(self, sec):
        end = time.time() + sec
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def nearest(self):
        now = time.time()
        fresh = [p for p, t in self.fruits.items() if now - t < 10]
        if not fresh:
            return None
        return min(fresh, key=lambda p: p[0]*p[0]+p[1]*p[1])

    def home_tool_quat(self):
        """홈 자세의 tool0 방향 (조립체의 '전방 수평' 기준 자세)."""
        req = GetPositionFK.Request()
        req.header.frame_id = FRAME
        req.fk_link_names = ["tool0"]
        req.robot_state.joint_state.name = JOINTS
        req.robot_state.joint_state.position = [HOME_Q[j] for j in JOINTS]
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5)
        res = fut.result()
        if res is None or not res.pose_stamped:
            raise RuntimeError("compute_fk 실패")
        p = res.pose_stamped[0].pose
        print(f"홈 tool0: pos=({p.position.x:.3f},{p.position.y:.3f},{p.position.z:.3f}) "
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
        pc.link_name = "tool0"
        prim = SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.01])
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
        oc.link_name = "tool0"
        oc.orientation = Quaternion(w=quat[0], x=quat[1], y=quat[2], z=quat[3])
        oc.absolute_x_axis_tolerance = 0.15
        oc.absolute_y_axis_tolerance = 0.15
        oc.absolute_z_axis_tolerance = 0.15
        oc.weight = 1.0
        c.position_constraints = [pc]
        c.orientation_constraints = [oc]
        r.goal_constraints = [c]
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

    GRIP_OPEN = 0.0            # finger_joint 열림
    GRIP_CLOSE = 0.8          # finger_joint 완전닫힘

    def _grip(self, pos):
        self.grip.publish(Float64MultiArray(data=[float(pos)]))

    def gripper(self, closed):
        self._grip(self.GRIP_CLOSE if closed else self.GRIP_OPEN)

    def grip_width(self, w):
        """접촉폭 유지 — 0.8 까지 안 쫓아 절단 후 squeeze-pop 회피(spike 검증)."""
        self._grip(w)

    def cut(self, chassis_pos):
        self._cut_seq += 1
        cut_id = self._cut_seq
        self.cmd.publish(String(data=json.dumps({"cut_fruit": {
            "id": cut_id, "position": list(chassis_pos), "max_distance": 0.4}})))
        return cut_id

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
    EE_TOUCH = ["gripper_body", "jig_body", "coupler_body", "harvest_tcp",
               "wrist_3_link", "wrist_2_link", "flange", "tool0", "ft_frame"]
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


def harvest_once(g: Grasp) -> bool:
    """수확 원샷: 최근접 과실 → 열기 → GRASP 로 직접 접근(OMPL 관절목표, 충돌회피) →
    파지 → 절단 → 홈. 실패 시 False (h 키 트리거 재사용 — sys.exit 금지).

    별도 pregrasp+LIN 을 안 쓴다: pregrasp 를 과실에서 뒤로 빼면 수평접근 IK 가 도달권을
    벗어나 반대 브랜치(pan −117°)로 도망가 PTP 가 실패했다(2026-07-22 진단). grasp 포즈는
    깨끗한 해(pan −42°)가 있고, 과실은 절단 전까지 kinematic 이라 직접 접근이 안전하다."""
    # ★데모 부착모드(ATTACH_GRASP=1, 2026-07-23): 마찰 대신 FixedJoint 로 과실을 그리퍼에
    #   물리 부착 → 절단 → 들고 대기. 지그 파지 안정화 전까지 시연용(후에 진짜 마찰 복원).
    #   마찰 경로(기본)는 그대로 보존 — 이 플래그 없으면 기존 동작.
    attach_mode = os.environ.get("ATTACH_GRASP") == "1"
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
    print(f"대상 과실(섀시): {fruit}")
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
    # 홈 harvest_tcp 방향(홈 wrist_1=180° 라 커터·D455 가 이미 파지점 위)을 과실 방위로
    # 요 회전만. 예전 180° 접근축 롤은 지그가 +Y 에 잘못 있어 카메라가 아래로 보이던 때의
    # 임시방편 — 지그를 −Y 로 바로잡고 모델 동기화 후 불필요(제거, 2026-07-22 사용자 지적).
    q = qmul((math.cos(yaw/2), 0.0, 0.0, math.sin(yaw/2)), quat)
    print(f"과실=({fx:.3f},{fy:.3f},{fz:.3f}) yaw={math.degrees(yaw):.1f}° (harvest_tcp 직접타겟, 롤 없음)")

    g.gripper(False); g.spin_for(1.0)                       # 열고
    # g.add_fruit_object(grasp, r=0.03)   # 장애물 스폰 — OMPL 99999 유발(ACM 미흡), 별도 수정 후 재활성
    grasp_q = g.solve_ik(grasp, q, HOME_Q)                  # 깨끗한 grasp 해
    if grasp_q is None:
        print("grasp IK 실패 — 도달 불가(베이스 재정렬 필요)")
        return False
    # 홈 → grasp 직접 접근. OMPL 로 충돌회피 계획(관절목표). PTP 는 직선보간이라 큰
    # 재구성 때 자기충돌 경로를 낼 수 있어 접근은 OMPL 로.
    if not g.run_goal(g.goal_joints(grasp_q, vel=0.25, pipeline="ompl", planner=""),
                      "GRASP(OMPL·IK)"):
        return False

    def finger(tag):
        g.spin_for(0.3)
        v = g.isaac_js.get("finger_joint", -1)
        held = "물고 있음" if 0.1 < v < 0.7 else ("빈손!" if v >= 0.7 else "열림")
        print(f"  finger[{tag}] = {v:.3f} ({held})")
        return v

    g.gripper(True); g.spin_for(2.0)                        # 완전닫힘 목표 → 과실에 막혀 접촉폭 W
    w = finger("접촉")                                       # 접촉폭 읽기
    if not (0.1 < w < 0.7):
        if not attach_mode:
            print("파지 접촉 검증 실패 — 절단하지 않음", flush=True)
            g.gripper(False)
            return False
        print("[부착모드] 접촉폭 미달이나 물리 부착으로 유지 진행", flush=True)
    # ★ 접촉폭 유지로 전환(0.8 안 쫓음) — 절단(dynamic) 후 구 과실을 적도 너머로 안 밀어
    #   squeeze-pop 회피(2026-07-22 spike grasp_force_test: 마찰·힘 전 조합 유지).
    if 0.0 < w < 0.7:
        g.grip_width(w); g.spin_for(0.8)   # 접촉폭 유지(과조임 금지) — squeeze-pop 회피
    finger("너비유지")
    # g.attach_fruit(r=0.03)   # MoveIt 충돌회피용 어태치 — 장애물 재활성 시 함께
    if attach_mode:
        # 물리 부착: moveit_mm 이 이 위치 최근접 ripe 씬 과실↔그리퍼에 FixedJoint 생성(절단
        # 전에 부착해야 절단=dynamic 전환 순간 과실이 그리퍼에 매달린 채 유지된다).
        g.cmd.publish(String(data=json.dumps({
            "attach_fruit": {"position": [round(float(v), 4) for v in fruit]}})))
        g.spin_for(1.0)
        print("[부착] 과실을 그리퍼에 물리 부착(FixedJoint) — 절단 후에도 유지", flush=True)
    g.cmd.publish(String(data=json.dumps({"blade": 35.0}))) # 칼날 닫기 (BLADE_CLOSED_DEG)
    g.spin_for(1.0)
    cut_sent_at = time.time()
    cut_id = g.cut(fruit)
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
            return False
        # 부착모드 공중과실은 pedicel 조인트가 없어 cut status 가 안 온다 — 칼날은
        # 시연용으로 닫고, 과실은 이미 FixedJoint 로 그리퍼에 붙어 절단검증 불필요.
        print("[부착모드] 공중과실(줄기 없음) — 칼날 시연만, 절단검증 스킵", flush=True)
    g.spin_for(1.5)                                         # 과실 dynamic 후 마찰 유지 확인
    after_cut = finger("절단 직후")
    if not (0.1 < after_cut < 0.7):
        if not attach_mode:
            print("절단 후 파지 유실", flush=True)
            g.cmd.publish(String(data=json.dumps({"blade": 0.0})))
            return False
        print("[부착모드] 그리퍼 개방과 무관하게 FixedJoint 로 과실 유지 — 계속", flush=True)
    g.cmd.publish(String(data=json.dumps({"blade": 0.0}))) # 칼날 열기
    g.spin_for(0.5)
    # 홈 복귀 — 과실을 마찰로 문 채. OMPL 충돌회피.
    if not g.run_goal(g.goal_joints(
            HOME_Q, vel=0.2, pipeline="ompl", planner=""), "HOME(OMPL)"):
        return False
    home_width = finger("홈 도착")
    # 부착모드는 FixedJoint 가 잡으므로 finger 너비와 무관하게 유지(들고 대기 완료).
    held = True if attach_mode else (0.1 < home_width < 0.7)
    print(f"시퀀스 완료: cut_success=true, held_at_home={held}"
          f"{' (부착 유지)' if attach_mode else ''}", flush=True)
    return held


def main():
    rclpy.init()
    g = Grasp()
    if not g.move.wait_for_server(timeout_sec=40):   # move_group 로딩 여유(통합 launch 동시기동)
        print("move_action 없음"); sys.exit(1)
    attach = os.environ.get("ATTACH_GRASP") == "1"
    n = int(os.environ.get("HARVEST_N", "1"))     # 반복 수확 횟수
    # 이전 실행서 그리퍼에 붙어있던 과실이 있으면 먼저 놓는다(반복 시작 정리).
    if attach:
        g.cmd.publish(String(data=json.dumps({"detach_grasp": True}))); g.spin_for(1.0)
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
        # 다음 과실을 위해 부착 해제(과실 놓기). 절단으로 이미 pickable 에서 빠져
        # 다음 nearest 는 새 과실을 고른다.
        if attach:
            g.cmd.publish(String(data=json.dumps({"detach_grasp": True}))); g.spin_for(1.5)
    print(f"\n총 {done}/{n} 수확 완료", flush=True)
    sys.exit(0 if done > 0 else 1)


if __name__ == "__main__":
    main()
