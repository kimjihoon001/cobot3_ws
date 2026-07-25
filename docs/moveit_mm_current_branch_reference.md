# MoveIt MM 구현 기준 스냅샷·머지 비교 가이드

이 문서는 현재 M0617 수확 MM의 MoveIt2 모델·설정·컨트롤러·모션 브리지를 별도로
정리한 비교 기준이다.

- 패키지: `src/smartfarm/mm_moveit`
- namespace: `harvester_0`
- planning group: `mm_manipulator`
- planning base/tip: `base_link → harvest_tcp`
- 기본 pipeline: OMPL
- 실행 하드웨어: `topic_based_ros2_control` → Isaac

`isaacpjt/moveit_mm.py`와 `src/smartfarm/harvest_moveit`은 UR10e 기반 별도/레거시
구현이다. 이 문서의 MoveIt MM은 현재 `--mm`과 연결된 `mm_moveit` 패키지를 뜻한다.

## 1. 데이터 흐름

```text
manipulator_target_node
  └─ /harvester_0/cmd JSON
       └─ mm_motion_bridge
            ├─ compute_ik 다중 seed·랭킹
            └─ MoveGroup move_action
                 └─ arm_controller/FollowJointTrajectory
                      └─ topic_based_ros2_control
                           └─ /harvester_0/joint_command
                                └─ Isaac M0617 articulation
```

스쿱 3축은 MoveIt planning group 밖이며 `gripper_controller/commands`의 position
명령으로 제어한다.

## 2. 관련 파일

- `src/smartfarm/mm_moveit/urdf/mm_mm.urdf.xacro`
- `src/smartfarm/mm_moveit/urdf/m0617_macro.xacro`
- `src/smartfarm/mm_moveit/srdf/mm_mm.srdf`
- `src/smartfarm/mm_moveit/config/kinematics.yaml`
- `src/smartfarm/mm_moveit/config/joint_limits.yaml`
- `src/smartfarm/mm_moveit/config/ompl_planning.yaml`
- `src/smartfarm/mm_moveit/config/chomp_planning.yaml`
- `src/smartfarm/mm_moveit/config/pilz_cartesian_limits.yaml`
- `src/smartfarm/mm_moveit/config/moveit_controllers.yaml`
- `src/smartfarm/mm_moveit/config/ros2_controllers.yaml`
- `src/smartfarm/mm_moveit/config/servo.yaml`
- `src/smartfarm/mm_moveit/launch/m0617_moveit_bringup.launch.py`
- `src/smartfarm/mm_moveit/launch/vision_harvest_bringup.launch.py`
- `src/smartfarm/mm_moveit/scripts/mm_motion_bridge.py`

## 3. URDF 구조

```text
mm_base
└─ fixed, z=0.30 m
   └─ base_link
      └─ joint_1 ... joint_6
         └─ link_6
            └─ scoop_adapter (Z축 180° 장착)
               ├─ scoop_quarter_1
               ├─ scoop_quarter_2
               ├─ cutter_quarter_3
               ├─ harvester_d455_mount
               └─ harvest_tcp (adapter +Z 0.120 m)
```

Ridgeback 크기는 `0.96 × 0.79 × 0.30 m`로 모델링한다. M0617 mesh와 joint
origin/limit은 Isaac M0617 USD와 같은 공식 URDF 소스를 사용한다.

## 4. planning group과 HOME

SRDF:

| 항목 | 값 |
|---|---|
| group | `mm_manipulator` |
| chain | `base_link → harvest_tcp` |
| end effector | `harvest_end_effector` |
| gripper group | `harvest_gripper` |

HOME:

```text
joint_1 = +180°
joint_2 =  +60°
joint_3 = -120°
joint_4 =    0°
joint_5 =  -30°
joint_6 =  +90°
```

Isaac 기본 관절 상태, ros2_control initial value, SRDF group state가 모두 같은 값을
유지해야 Play/Stop과 RViz 자세가 일치한다.

## 5. 충돌 행렬

비활성 충돌:

- M0617 인접 링크
- `mm_base`–`base_link/link_1`
- `link_6`–`scoop_adapter`
- 플랜지와 D455 mount
- 의도적으로 포개진 세 동축 스쿱 셸 상호 간

스쿱 외부와 환경·토마토 충돌은 유지한다. 다른 브랜치의 SRDF로 덮으면 동축 셸
자기충돌 때문에 모든 계획이 실패할 수 있다.

## 6. IK

```yaml
solver: KDLKinematicsPlugin
search_resolution: 0.005
timeout: 0.05 s
attempts: 3
```

Doosan 전용 해석 IK 플러그인이 워크스페이스에 없어 KDL을 사용한다.

APPROACH와 BASKET_APPROACH의 OMPL 목표는 bridge가 `compute_ik`에 여러 seed를
보내고 현재 관절 변화가 작은 해를 선택한다.

일반 IK 가중치:

```text
[2.0, 1.5, 1.5, 2.0, 3.0, 3.0]
```

바스켓 IK 가중치:

```text
[0.3, 3.0, 3.0, 3.0, 4.0, 4.0]
```

바스켓에서는 joint_1 회전은 싸게, 다른 관절 변화는 비싸게 평가한다. 단일 관절
최대 변화는 `2.40 rad`, joint state 최대 나이는 `0.5 s`다.

## 7. planning pipelines

### OMPL

- 기본 pipeline
- 기본 planner: `RRTConnectkConfigDefault`
- 대안: RRTstar, PRM
- projection: `joint_1`, `joint_2`
- longest valid segment fraction: `0.005`

APPROACH와 BASKET_APPROACH에 사용한다.

### Pilz

- `PTP`, `LIN`, `CIRC`
- Cartesian max velocity: `0.5 m/s`
- Cartesian max acceleration/deceleration: `1.0 / -1.0 m/s²`
- max rotation velocity: `1.0 rad/s`

Humble에서 CIRC 보조점을 일반 path constraint로 재검사해 실패하는 문제를 피하기
위해 Pilz `check_solution_paths=false`다.

### CHOMP

pipeline에는 등록돼 있으나 현재 자동 수확의 주 모션 경로는 OMPL과 Pilz다.

## 8. 관절 한계

| joint | max velocity | max acceleration |
|---|---:|---:|
| joint_1 | `1.7453` | `2.0` |
| joint_2 | `1.7453` | `2.0` |
| joint_3 | `2.6180` | `3.0` |
| joint_4 | `3.9270` | `5.0` |
| joint_5 | `3.9270` | `5.0` |
| joint_6 | `3.9270` | `5.0` |

속도는 공식 URDF 값이고 가속도는 현재 튜닝값이다.

## 9. 모션 브리지

입력:

```json
{
  "rmp_target": {
    "id": 1,
    "phase": "APPROACH",
    "frame_id": "base_link",
    "position": [0, 0, 0],
    "motion": "OMPL",
    "tool_orientation": [0, 0, 0, 1]
  }
}
```

명령명이 `rmp_target/rmp_home`인 것은 과거 호환용이며 실제 실행은 MoveIt이다.

기본 파라미터:

| 항목 | 값 |
|---|---:|
| position tolerance | `0.0025 m` |
| orientation tolerance | `0.035 rad` |
| planning time | `8 s` |
| home velocity scale | `0.50` |
| acceleration scale | `0.30` |
| control failure retries | `2` |

모션별 속도:

| motion | scale |
|---|---:|
| OMPL | `0.35` |
| PTP | `0.40` |
| LIN | `0.15` |
| CIRC | `0.15` |
| GRASP override | `0.05` |
| CAPTURE_TRIM override | `0.035` |

정밀 GRASP/CAPTURE는 전역 속도 상향과 무관하게 저속을 유지한다.

## 10. ros2_control

update rate: `60 Hz`, Isaac physics와 동일하다.

Controller:

| 이름 | 타입 | 관절 |
|---|---|---|
| joint_state_broadcaster | JSB | 전체 상태 |
| arm_controller | JointTrajectoryController | joint_1..6 |
| gripper_controller | JointGroupPositionController | 스쿱 3축 |

arm controller는 position command, position+velocity state를 사용한다. partial joint
goal은 허용하지 않는다.

하드웨어 토픽은 절대경로다.

```text
joint command: /harvester_0/joint_command
hardware state: /harvester_0/hw_joint_states
```

TopicBasedSystem 내부 노드는 controller manager namespace를 자동 상속하지 않으므로
상대 토픽으로 바꾸면 Isaac 연결이 끊긴다.

## 11. 기동 순서

`m0617_moveit_bringup.launch.py` 순서:

1. ros2_control node + robot_state_publisher
2. `4 s` 뒤 joint_state_broadcaster spawner
3. JSB 종료 후 arm controller spawner
4. arm spawner 종료 후 gripper controller, move_group, Servo, RViz

spawner를 하나의 프로세스에 묶지 않는다. 첫 controller가 이미 loaded인 경우 나머지
controller까지 건너뛰는 문제를 피하기 위한 순차 기동이다.

trajectory execution:

- duration scaling: `2.0`
- goal duration margin: `5.0 s`
- start tolerance: `0.05`

## 12. TF와 namespace

MoveIt 관련 노드는 `/harvester_0` namespace에 둔다.

```text
/tf          → /harvester_0/tf
/tf_static   → /harvester_0/tf_static
/joint_states→ /harvester_0/joint_states
```

MoveIt URDF 프레임에는 `frame_prefix`를 적용하지 않는다. prefix를 넣으면 SRDF의
`base_link/harvest_tcp`와 실제 TF 이름이 달라져 planning scene이 깨진다.

## 13. MoveIt Servo

Servo는 평소 정지하며 카메라 미세보정 구간에만 사용하도록 구성됐다.

| 항목 | 값 |
|---|---:|
| publish period | `0.02 s` |
| linear/rotational/joint scale | `0.08 / 0.20 / 0.10` |
| output | `arm_controller/joint_trajectory` |
| incoming timeout | `0.10 s` |
| collision check | `20 Hz` |
| self/scene proximity | `0.015 / 0.025 m` |

Servo 설정에는 planning frame `mm_base`가 남아 있지만 motion bridge의 planning
frame은 `base_link`다. Servo를 실제 수확 미세보정에 다시 사용할 때 이 프레임
의도를 재검증해야 한다.

## 14. 실행

MoveIt만:

```bash
ros2 launch mm_moveit m0617_moveit_bringup.launch.py
```

비전+MoveIt:

```bash
ros2 launch mm_moveit vision_harvest_bringup.launch.py
```

Nav2+비전+MoveIt:

```bash
ros2 launch mm_moveit auto_nav_harvest.launch.py
```

Isaac은 `main.py --mm`으로 실행하고 반드시 Play 상태여야 한다.

## 15. 현재 검증·주의점

확인:

- M0617 joint state와 MoveIt RViz 일치
- OMPL APPROACH
- Pilz LIN 정밀 접근·후퇴
- grasp/cut/retract
- ranked IK를 거친 BASKET_APPROACH
- BASKET_PLACE와 retract

주의:

- `rmp_*` 이름을 보고 RMPflow consumer를 다시 붙이지 않는다.
- UR10e `harvest_moveit` 설정을 M0617에 덮지 않는다.
- `mm_base`와 `base_link` 좌표를 혼용하지 않는다.
- Isaac/MoveIt HOME 값을 동시에 바꾼다.
- ranked IK 가중치 제거 시 손목이 크게 뒤집힐 수 있다.
- 동시 기동 시 controller/lifecycle DDS 응답 유실 가능성이 남아 있다.

## 16. 머지 우선순위

1. M0617 URDF/SRDF와 `mm_manipulator`
2. HOME 6축 값의 Isaac–URDF–SRDF 일치
3. absolute hardware topics
4. 순차 controller spawner
5. OMPL/Pilz pipeline 분기
6. APPROACH/BASKET_APPROACH ranked IK
7. 모션별 속도와 정밀 구간 override
8. TF remap과 namespace 격리
9. 동축 스쿱 collision disable matrix

## 17. 머지 후 체크리스트

- [ ] RViz와 Isaac의 HOME 자세가 같은가
- [ ] controller 3종이 active인가
- [ ] 실제 joint state가 60 Hz로 들어오는가
- [ ] `/joint_command`가 Isaac M0617을 움직이는가
- [ ] planning group이 `base_link→harvest_tcp`인가
- [ ] OMPL APPROACH가 반대 IK 해로 뒤집히지 않는가
- [ ] LIN GRASP가 직선·저속으로 실행되는가
- [ ] 스쿱 3축이 arm controller와 분리돼 움직이는가
- [ ] 절단 후 RETRACT 경로가 성공하는가
- [ ] BASKET_APPROACH가 joint_1 위주로 부드럽게 움직이는가
- [ ] MoveIt/Nav2 RViz가 서로 다른 노드 이름으로 뜨는가

