"""파지 스파이크 — TCP·마찰·힘을 쓸어(sweep) 값을 찾는다 (2026-07-22 사용자 요청).

Isaac 재시작 없이 --airfruit(실제 토마토 USD + 그립 줄기)를 케이스마다 리셋(재스폰)하며:
  홈 → reset_air(kinematic 복귀) → set_friction(μ) → MoveIt 파지 → 닫기 →
  drop_air(절단=중력 ON) → 정착 → /sim/tomato 의 과실 z 로 held/drop 판정(카메라 안 씀).

성공기준(사용자): 절단 후 과실 z 가 스폰높이 − DROP_MARGIN 이하로 내려가면 DROP, 아니면 HELD.

스윕 축:
  TCP  = 접근축 오프셋 XOFF (FK 진단: 패드 수렴점이 harvest_tcp 보다 22mm 뒤 → 앞으로 밀어 접촉)
  μ    = 과실+줄기 마찰 (combineMode=min → 유효=μ)
  힘   = 접촉 후 추가 조임 δ (위치제어 오버슈트 = 정상력)

2단계 분해(빠름): ①XOFF 스윕으로 '접촉되는' TCP 창 찾기 → ②최적 XOFF 에서 μ×힘 그리드로 held.
결과표는 stdout + /tmp/grasp_sweep_results.txt.
"""
import json
import math
import os
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseStamped, Quaternion
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

FRAME = "mm_base"
HOME_Q = {"shoulder_pan_joint": 0.0, "shoulder_lift_joint": 3.9269908,
          "elbow_joint": 2.3561945, "wrist_1_joint": 3.1415927,
          "wrist_2_joint": -1.5707963, "wrist_3_joint": 0.0}
JOINTS = list(HOME_Q)

# ── 스윕 범위 (env 로 덮어쓰기 가능) ──
XOFFS = [float(v) for v in os.environ.get("XOFFS", "0.0,0.01,0.02,0.03").split(",")]
MUS = [float(v) for v in os.environ.get("MUS", "0.3,0.5,0.7,0.9").split(",")]
FORCES = [float(v) for v in os.environ.get("FORCES", "0.0,0.10").split(",")]  # 접촉 후 δ
ZOFF = float(os.environ.get("ZOFF", "0.05"))            # 줄기 중심(과실+5cm)
DROP_MARGIN = float(os.environ.get("DROP_MARGIN", "0.20"))
AIRFRUIT_N = int(os.environ.get("AIRFRUIT_N", "5"))     # Isaac 이 스폰한 과실 수(맞출 것)
ROLL180 = os.environ.get("ROLL180", "0") != "0"         # 접근축 180° 롤(기본 끔 — 뒤집힘 방지)
XOFF0 = float(os.environ.get("XOFF0", "0.0"))           # 접근축 보정(실측 접촉점 0.0)
TCP_Z = float(os.environ.get("TCP_Z", "0.0"))           # ★TCP 접근축(tool Z) 연장 — 원통에 더 닿게
FRUIT_IDS = [int(v) for v in os.environ.get("FRUIT_IDS", "").split(",") if v != ""]  # 도달되는 과실만
CLOSE_CMD = 0.8                                          # 완전닫힘 지령(접촉서 멈춤)
CONTACT_MAX = 0.78                                       # finger < 이 값 = 접촉(빈손 아님)
RESULTS_FILE = "/tmp/grasp_sweep_results.txt"


def qmul(a, b):
    return (a[0]*b[0]-a[1]*b[1]-a[2]*b[2]-a[3]*b[3],
            a[0]*b[1]+a[1]*b[0]+a[2]*b[3]-a[3]*b[2],
            a[0]*b[2]-a[1]*b[3]+a[2]*b[0]+a[3]*b[1],
            a[0]*b[3]+a[1]*b[2]-a[2]*b[1]+a[3]*b[0])


def tool_z_axis(q):
    """쿼터니언 q(w,x,y,z) 로 회전한 tool 로컬 Z(=접근축)의 월드방향."""
    w, x, y, z = q
    return (2*(x*z + w*y), 2*(y*z - w*x), 1 - 2*(x*x + y*y))


class Sweep(Node):
    def __init__(self):
        super().__init__("grasp_sweep")
        self.move = ActionClient(self, MoveGroup, "/move_action")
        self.fk = self.create_client(GetPositionFK, "/compute_fk")
        self.ik = self.create_client(GetPositionIK, "/compute_ik")
        self.cmd = self.create_publisher(String, "/harvester_moveit/cmd", 10)
        self.grip = self.create_publisher(Float64MultiArray,
                                          "/gripper_controller/commands", 10)
        self.arm_traj = self.create_publisher(JointTrajectory,
                                              "/arm_controller/joint_trajectory", 10)
        self.fruits = {}
        self.create_subscription(String, "/harvester_moveit/sim/tomato", self._fruit, 20)
        self.isaac_js = {}
        self.create_subscription(JointState, "/harvester_moveit/joint_states",
                                 lambda m: self.isaac_js.update(
                                     dict(zip(m.name, m.position))), 10)

    # ── 콜백/유틸 ──
    def _fruit(self, m):
        if not m.data:
            return
        try:
            d = json.loads(m.data)
            self.fruits[tuple(round(v, 3) for v in d["position"])] = time.time()
        except (ValueError, KeyError):
            pass

    def spin_for(self, sec):
        end = time.time() + sec
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def nearest(self):
        now = time.time()
        fresh = [p for p, t in self.fruits.items() if now - t < 5]
        return min(fresh, key=lambda p: p[0]*p[0]+p[1]*p[1]) if fresh else None

    def send_cmd(self, d):
        self.cmd.publish(String(data=json.dumps(d)))

    def _grip(self, v):
        self.grip.publish(Float64MultiArray(data=[float(v)]))

    def finger(self):
        self.spin_for(0.3)
        return self.isaac_js.get("finger_joint", -1.0)

    # ── MoveIt ──
    def home_quat(self):
        req = GetPositionFK.Request()
        req.header.frame_id = FRAME
        req.fk_link_names = ["tool0"]
        req.robot_state.joint_state.name = JOINTS
        req.robot_state.joint_state.position = [HOME_Q[j] for j in JOINTS]
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5)
        o = fut.result().pose_stamped[0].pose.orientation
        return (o.w, o.x, o.y, o.z)

    def solve_ik(self, pos, quat, seed):
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
            return None
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))

        def unwrap(v, ref):
            while v - ref > math.pi:
                v -= 2 * math.pi
            while v - ref < -math.pi:
                v += 2 * math.pi
            return v
        return {j: unwrap(sol[j], seed[j]) for j in JOINTS}

    def goal_joints(self, qmap, vel, pipeline, planner):
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
        c.joint_constraints = [
            JointConstraint(joint_name=j, position=q, tolerance_above=0.01,
                            tolerance_below=0.01, weight=1.0)
            for j, q in qmap.items()]
        r.goal_constraints = [c]
        return g

    def run_goal(self, goal, tag, tmo=90):
        fut = self.move.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=15)
        h = fut.result()
        if h is None or not h.accepted:
            return False
        rfut = h.get_result_async()
        rclpy.spin_until_future_complete(self, rfut, timeout_sec=tmo)
        res = rfut.result()
        return bool(res) and res.result.error_code.val == 1

    def go_home(self):
        """홈 복귀 — MoveIt 계획(Pilz) 대신 컨트롤러에 직접 궤적(충돌검사 없음)이라 항상 성공.
        나쁜 자세에서 Pilz -2 로 못 빠져나오던 문제 해소. 파지는 여전히 Pilz."""
        t = JointTrajectory()
        t.joint_names = JOINTS
        pt = JointTrajectoryPoint()
        pt.positions = [HOME_Q[j] for j in JOINTS]
        pt.time_from_start.sec = 4
        t.points = [pt]
        for _ in range(3):
            self.arm_traj.publish(t); self.spin_for(0.1)
        self.spin_for(4.5)
        tcp = self.actual_tcp()                            # 홈 도착 확인(harvest_tcp)
        return tcp is not None and tcp[2] > 0.9            # 홈이면 z≈1.03

    def actual_tcp(self):
        """현재 Isaac 팔 관절로 harvest_tcp FK — 팔이 실제 어디 있나(거짓양성 방지)."""
        q = {j: self.isaac_js.get(j, HOME_Q[j]) for j in JOINTS}
        req = GetPositionFK.Request()
        req.header.frame_id = FRAME
        req.fk_link_names = ["harvest_tcp"]
        req.robot_state.joint_state.name = list(q)
        req.robot_state.joint_state.position = list(q.values())
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5)
        res = fut.result()
        if res is None or not res.pose_stamped:
            return None
        p = res.pose_stamped[0].pose.position
        return (p.x, p.y, p.z)

    # ── 케이스 1건 (고정 과실 idx — reset 으로 복원해 재사용) ──
    def run_case(self, idx, xoff, mu, force, quat, tcp_z=0.0):
        """returns dict: idx, tcp_z, mu, force, w(접촉폭), z_after, held, note."""
        rec = {"idx": idx, "tcp_z": tcp_z, "mu": mu, "force": force,
               "w": None, "z_after": None, "held": None, "note": ""}
        # 1) 그리퍼 열고 홈 복귀 (직접 궤적 — 항상 성공)
        self._grip(0.0); self.spin_for(0.6)
        if not self.go_home():
            rec["note"] = "home_fail"; return rec
        # 2) 고정 과실 선택 + reset(kinematic 복원 — 잘렸던 것도 스폰위치로). 신뢰성 홈 덕에 오염 없음.
        self.fruits.clear()
        z_spawn = None
        for _ in range(6):
            self.send_cmd({"select_fruit": idx}); self.spin_for(0.3)
            self.send_cmd({"reset_air": True}); self.spin_for(1.0)
            fr = self.nearest()
            if fr and fr[2] > 0.5:
                z_spawn = fr[2]; break
        if z_spawn is None:
            rec["note"] = "select_fail"; return rec
        # 3) 마찰 설정(선택 과실 머티리얼)
        self.send_cmd({"set_friction": mu}); self.spin_for(0.4)
        # 4) 파지 타겟
        fr = self.nearest()
        fx, fy, fz = fr
        yaw = math.atan2(fy, fx)
        q = qmul((math.cos(yaw/2), 0.0, 0.0, math.sin(yaw/2)), quat)
        if ROLL180:                                        # 접근축(tool Z) 180° 롤(기본 끔)
            q = qmul(q, (0.0, 0.0, 0.0, 1.0))
        grasp = [fx + xoff, fy, fz + ZOFF]
        if tcp_z:                                          # TCP 접근축 오프셋 — 손끝으로 잡게
            az = tool_z_axis(q)
            grasp = [grasp[k] + tcp_z * az[k] for k in range(3)]
        grasp = tuple(grasp)
        gq = self.solve_ik(grasp, q, HOME_Q)
        if gq is None:
            rec["note"] = "ik_fail"; return rec
        # 5) 접근 — Pilz PTP(결정적, 사용자 요청). 과실은 절단 전까지 kinematic 이라 안전.
        if not self.run_goal(self.goal_joints(
                gq, 0.25, "pilz_industrial_motion_planner", "PTP"), "GRASP"):
            rec["note"] = "plan_fail"; self._grip(0.0); self.spin_for(0.4); return rec
        # 5.5) ★팔이 실제로 과실에 갔는지 FK 검증 (거짓양성 방지 — "시도조차 못함" 잡아냄)
        self.spin_for(0.5)
        tcp = self.actual_tcp()
        if tcp is not None:
            rec["reach"] = round(math.dist(tcp, grasp), 3)
            if rec["reach"] > 0.05:                  # 5cm 이상 벗어나면 팔 미도달
                rec["note"] = f"팔 미도달({rec['reach']*100:.0f}cm)"; rec["held"] = False
                self._grip(0.0); self.spin_for(0.4); return rec
        # 6) 닫기 → 접촉폭
        self._grip(CLOSE_CMD); self.spin_for(2.0)
        w = self.finger()
        rec["w"] = round(w, 3)
        if not (0.05 < w < CONTACT_MAX):
            rec["note"] = "빈손(접촉X)"; rec["held"] = False
            self._grip(0.0); self.spin_for(0.4); return rec
        # 7) 힘(접촉 후 추가 조임 δ) 유지
        self._grip(min(CLOSE_CMD, w + force)); self.spin_for(1.0)
        # 8) 절단 = drop_air(중력 ON) — 이제 마찰이 안 잡으면 진짜 떨어진다
        self.send_cmd({"drop_air": True}); self.spin_for(2.0)
        # 9) 판정 (카메라 X — 과실 z 로)
        fr2 = self.nearest()
        z_after = fr2[2] if fr2 else -1.0
        rec["z_after"] = round(z_after, 3)
        rec["held"] = z_after > (z_spawn - DROP_MARGIN)
        rec["note"] = "HELD" if rec["held"] else "DROP"
        # 10) 과실 놓기 — 다음 케이스 오염/그리퍼 불안정 방지
        self._grip(0.0); self.spin_for(0.8)
        return rec


def fmt(r):
    w = f"{r['w']:.3f}" if r['w'] is not None else "  -  "
    z = f"{r['z_after']:.3f}" if r['z_after'] is not None else "  -  "
    reach = f"{r.get('reach', 0)*100:.0f}cm" if r.get('reach') is not None else "  - "
    held = "HELD✔" if r['held'] else ("drop✖" if r['held'] is False else "  ?  ")
    return (f"  #{r['idx']} TCP_Z={r['tcp_z']:+.3f} μ={r['mu']:.2f} δ={r['force']:.2f}  "
            f"도달={reach:>5}  w={w}  z={z}  {held}  {r['note']}")


def main():
    rclpy.init()
    s = Sweep()
    if not s.move.wait_for_server(timeout_sec=15):
        print("move_action 없음 — MoveIt 스택 먼저 띄울 것"); sys.exit(1)
    s.fk.wait_for_service(timeout_sec=10)
    s.ik.wait_for_service(timeout_sec=10)
    s.spin_for(1.0)
    quat = s.home_quat()
    lines = []

    def log(msg):
        print(msg, flush=True); lines.append(msg)

    ids = FRUIT_IDS if FRUIT_IDS else list(range(min(len(MUS), AIRFRUIT_N)))
    log("=== 파지 μ 스윕 — fresh 과실(reset 없음), Pilz, XOFF=%.2f, TCP_Z=%+.3f, ROLL180=%s ==="
        % (XOFF0, TCP_Z, ROLL180))
    log(f"과실 idx={ids}  MUS={MUS}  δ(힘)={max(FORCES):.2f}  ZOFF={ZOFF}  DROP_MARGIN={DROP_MARGIN}")

    # 과실 idx[k] → μ=MUS[k], 고정 XOFF0·TCP_Z·δ(힘). 도달오차 FK 검증 포함.
    results = []
    for k, mu in enumerate(MUS):
        if k >= len(ids):
            break
        r = s.run_case(ids[k], XOFF0, mu, max(FORCES), quat)
        log(fmt(r)); results.append(r)

    # ── 요약 ──
    reached = [r for r in results if r.get("reach") is not None and r["reach"] <= 0.05]
    contact = [r for r in results if r["w"] is not None and 0.05 < r["w"] < CONTACT_MAX]
    held = [r for r in results if r["held"]]
    log("\n=== 요약 ===")
    log(f"팔 도달 {len(reached)}/{len(results)}, 접촉 {len(contact)}/{len(results)}, "
        f"HELD {len(held)}/{len(results)}")
    if held:
        mu_min = min(r["mu"] for r in held)
        log("→ [3] 결론: XOFF={:+.2f}·ROLL180·δ={:.2f} 에서 μ≥{:.2f} 줄기 파지 유지".format(
            XOFF0, max(FORCES), mu_min))
    elif contact:
        log("→ 접촉은 되나 유지 실패 — 힘(δ)·ZOFF 재스윕 필요")
    elif reached:
        log("→ 팔은 도달하나 접촉 실패 — XOFF·ZOFF·ROLL180 재검토")
    else:
        log("→ 팔이 과실에 도달 못함 — 도달권/계획(Pilz) 문제")
    _dump(lines)
    s.destroy_node(); rclpy.shutdown()
    sys.exit(0)


def _dump(lines):
    try:
        with open(RESULTS_FILE, "w") as f:
            f.write("\n".join(lines) + "\n")
        print(f"\n[결과 저장] {RESULTS_FILE}", flush=True)
    except OSError as exc:
        print(f"[결과 저장 실패] {exc}", flush=True)


if __name__ == "__main__":
    main()
