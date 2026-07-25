# 단독 auto_nav_harvest 실행 기준선

- 수집일: 2026-07-24
- ROS_DOMAIN_ID: 108
- RMW: rmw_fastrtps_cpp
- Isaac 명령: `main.py --mm --nav --camera`
- ROS 명령: `ros2 launch mm_moveit auto_nav_harvest.launch.py`
- 수확 런치 PID: 43772
- Isaac PID: 43184
- ROS launch 로그:
  `/home/rokey/.ros/log/2026-07-24-21-21-27-373867-IsaacSim03-43772/launch.log`

## 결과 요약

단독 실행은 Nav2 목표 도착부터 파지, 절단, 후퇴, 홈 복귀까지 성공했다.
모든 arm_controller 목표와 MoveIt trajectory가 `SUCCEEDED`로 끝났다.
IW가 실행되지 않았으므로 바스켓 적재 상태는 진입하지 않았다.

## 상태 전이

```text
READY_FOR_NAV_GOAL
FIXED_NAV_GOAL_SENDING
NAVIGATING_TO_FIXED_HARVEST_POSE
WAIT_ARM_HOME_BEFORE_BED_VIEW
WAIT_ARM_BED_VIEW_AFTER_HOME
HARVEST_APPROACH
HARVEST_PREGRASP
HARVEST_GRASP
HARVEST_GRIPPER_CLOSING
HARVEST_GRASP_VERIFY
HARVEST_CUTTING
HARVEST_CUT_VERIFY
HARVEST_BLADE_OPENING
HARVEST_RETRACT_CIRC
HARVEST_RETRACT_LIN
GO_HOME
HOME_READY
CYCLE_COMPLETE_HOME_READY
```

## 주요 시각과 목표

ROS simulation clock 기준:

| 이벤트 | 시각 |
|---|---:|
| Nav2 목표 전송 성공 | 1784895694.553 |
| Nav2 도착 확정 | 1784895725.055 |
| APPROACH | 1784895735.977 |
| PREGRASP | 1784895747.129 |
| GRASP | 1784895754.359 |
| GRASP_VERIFY | 1784895769.682 |
| RETRACT_CIRC | 1784895771.334 |
| RETRACT_LIN | 1784895778.595 |
| GO_HOME | 1784895786.859 |
| HOME_READY | 1784895800.520 |

MoveIt Cartesian 목표:

```text
APPROACH:  (0.867, 0.623, 0.917)
PREGRASP:  (0.838, 0.657, 1.006)
GRASP:     (0.810, 0.689, 1.091)
RETRACT_CIRC -> PREGRASP
RETRACT_LIN  -> APPROACH
```

파지 검증:

```text
GRASP TCP distance: 0.001 m
```

MoveIt bridge 명령:

```text
id=900001 GO_HOME
id=1 APPROACH
id=2 PREGRASP
id=3 GRASP
id=6 RETRACT_CIRC
id=7 RETRACT_LIN
id=8 GO_HOME
```

## 컨트롤러 결과

- `joint_state_broadcaster`, `arm_controller`, `gripper_controller` 활성화 성공
- arm_controller가 받은 8개 action goal 모두 `Goal reached, success`
- MoveIt trajectory 8개 모두 `Completed trajectory execution with status SUCCEEDED`
- Nav2 controller가 고정 목표에 `Reached the goal`

## 비교 시 반드시 확인할 항목

통합 실행에서 다음 항목을 같은 순서로 수집한다.

1. APPROACH/PREGRASP/GRASP 좌표가 위 기준선과 같은지
2. GRASP 이후 `BASKET_APPROACH`, `BASKET_PLACE`, `BASKET_RETRACT`가 추가되는지
3. IW가 어느 상태에서 FOLLOW를 시작하는지
4. 팔 동작 중 IW 바스켓 TF가 이동하는지
5. MoveIt/arm_controller의 abort, cancel, start tolerance 오류
6. GO_HOME 직전 팔이 바스켓 밖으로 직선 후퇴했는지

## 기준선 경고

- 비전 TF에서 한 차례 future extrapolation 경고가 있었으나 다음 프레임에서 회복했다.
- HOME_READY 이후 카메라 목표 `(약 0.130, -0.029, 0.957)`가 작업영역 밖으로 반복 차단됐다.
  수확 게이트가 닫혀 추가 팔 명령은 발생하지 않았다.
- coordinator 로그에 `HARVEST_STARTED_IW_FOLLOW`가 찍혔지만 단독 실행에는
  `mission_nav_node`와 IW Nav2가 없으므로 실제 IW 이동은 발생하지 않았다.
