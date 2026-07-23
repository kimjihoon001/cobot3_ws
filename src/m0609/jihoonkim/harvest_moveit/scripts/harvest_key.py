#!/usr/bin/env python3
"""키 트리거 수확 — 터미널에서 [h] 를 누르면 MoveIt 수확 원샷(grasp_proto.harvest_once).

  h = 수확 원샷 (최근접 과실 → 파지 → 절단 → 홈)
  o / c = 그리퍼 열기 / 닫기
  0 = 홈 복귀 (Pilz PTP 저속)
  q = 종료

전제: Isaac ▶Play + moveit_isaac.launch.py 스택 실행 중.
사용: ros2 run 없이 직접 —  python3 harvest_key.py  (rosh 환경)
"""
import select
import sys
import termios
import tty

import rclpy

from grasp_proto import HOME_Q, Grasp, harvest_once


def main() -> None:
    rclpy.init()
    g = Grasp()
    if not g.move.wait_for_server(timeout_sec=10):
        print("move_action 없음 — MoveIt 스택이 떠 있는지 확인")
        return
    print("[h]=수확  [o]=그리퍼 열기  [c]=닫기  [0]=홈  [q]=종료")
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        while True:
            g.spin_for(0.05)          # 과실/관절 토픽 수신 유지
            if not select.select([sys.stdin], [], [], 0)[0]:
                continue
            k = sys.stdin.read(1)
            if k == "q":
                break
            if k == "h":
                # 시퀀스 동안은 일반 터미널 모드로 (출력 줄바꿈 정상화)
                termios.tcsetattr(fd, termios.TCSADRAIN, old)
                ok = harvest_once(g)
                print(f"수확 {'성공' if ok else '실패'} — [h] 재시도 가능")
                tty.setcbreak(fd)
            elif k == "o":
                g.gripper(False); print("그리퍼 열기")
            elif k == "c":
                g.gripper(True); print("그리퍼 닫기")
            elif k == "0":
                termios.tcsetattr(fd, termios.TCSADRAIN, old)
                g.run_goal(g.goal_joints(HOME_Q, vel=0.2), "HOME")
                tty.setcbreak(fd)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
