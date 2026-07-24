"""grasp 자세로 가서 멈추고 — 그리퍼 카메라 스냅 + harvest_tcp vs 과실 실측.
빈손 원인(TCP 오프셋? 방향?)을 눈+숫자로 진단. 닫기/절단/홈 안 함."""
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Quaternion
from tf2_ros import Buffer, TransformListener

import snap_cam
from grasp_proto import HOME_Q, TOOL_LEN, Grasp, qmul


def main():
    rclpy.init()
    g = Grasp()
    if not g.move.wait_for_server(timeout_sec=10):
        print("move_action 없음"); return
    tf_buf = Buffer(); TransformListener(tf_buf, g)
    g.spin_for(2.0)
    if len(sys.argv) >= 4:                      # 고정 타겟(디버깅 안정화)
        fruit = tuple(float(v) for v in sys.argv[1:4])
    else:
        fruit = g.nearest()
    if fruit is None:
        print("과실 없음"); return
    fx, fy, fz = fruit
    print(f"과실(섀시)=({fx:.3f},{fy:.3f},{fz:.3f})")
    quat, _ = g.home_tool_quat()
    yaw = math.atan2(fy, fx)
    grasp = (fx, fy, fz)                     # grasp_proto 와 동일: 과실 직접(harvest_tcp)
    q = qmul((math.cos(yaw/2), 0.0, 0.0, math.sin(yaw/2)), quat)   # 롤 없음
    g.gripper(False); g.spin_for(1.0)
    gq = g.solve_ik(grasp, q, HOME_Q)
    if gq is None:
        print("grasp IK 실패"); return
    if not g.run_goal(g.goal_joints(gq, vel=0.25, pipeline="ompl", planner=""),
                      "GRASP"):
        print("grasp 이동 실패"); return
    g.spin_for(1.5)
    # harvest_tcp 실제 위치 (mm_base 프레임) vs 과실
    for _ in range(30):
        rclpy.spin_once(g, timeout_sec=0.1)
        if tf_buf.can_transform("mm_base", "harvest_tcp", rclpy.time.Time()):
            break
    try:
        t = tf_buf.lookup_transform("mm_base", "harvest_tcp", rclpy.time.Time())
        p = t.transform.translation
        print(f"harvest_tcp(mm_base)=({p.x:.3f},{p.y:.3f},{p.z:.3f})")
        print(f"과실 - tcp = ({fx-p.x:+.3f},{fy-p.y:+.3f},{fz-p.z:+.3f})  "
              f"|dist|={math.dist((fx,fy,fz),(p.x,p.y,p.z))*1000:.0f}mm")
    except Exception as e:
        print(f"TF 실패: {e}")
    # 손끝 카메라 스냅
    snap_cam.snap(g, "/tmp/claude-1000/-home-rokey-cobot3-ws/"
                     "e95ff9f4-aba7-4667-a8e9-c70b2eebbaf1/scratchpad/grasp_view.png")
    print("grasp 자세 유지 — 육안 확인용")
    g.destroy_node(); rclpy.shutdown()


if __name__ == "__main__":
    main()
