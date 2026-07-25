# MM 전체 구현 기준 스냅샷·머지 비교 가이드

이 문서는 `harvest/rmp-mm-suction` 브랜치의 현재 MM 전체 구현을 다른 브랜치와
비교하거나 머지할 때 사용하는 기준 스냅샷이다.

- 기준 브랜치: `harvest/rmp-mm-suction`
- 기준 커밋: `b235251`
- 현재 주 실행: `isaac_python main.py --mm --nav --camera`
- 현재 MM: Ridgeback + Doosan M0617 + 동축 3축 스쿱 + eye-in-hand D455
- 팔 제어: MoveIt2 (`mm_moveit`), RMPflow 아님

## 1. 역할과 전체 흐름

MM은 온실 수확 위치로 이동하고, 카메라로 토마토를 선택해 파지·절단한 뒤 IW의
빈 KLT에 플레이스하는 모바일 매니퓰레이터다.

```text
Nav2 고정 수확 위치 이동
→ 팔 HOME
→ 베드 방향 BED_VIEW
→ RGB-D + YOLO 토마토 검출
→ 시뮬 GT와 대상 매칭
→ APPROACH / PREGRASP / GRASP
→ 스쿱 닫기·파지 검증
→ 커터 절단
→ RETRACT
→ IW 빈 KLT pose 대기
→ BASKET_APPROACH / PLACE / RELEASE / RETRACT
→ HOME
```

책임 분리:

- Isaac: 로봇 조립·물리·카메라·라이다·odom·joint 명령 실행·과실 GT
- Nav2: MM 베이스 이동
- `vision_node`: RGB-D 검출과 3D 목표 생성
- `manipulator_target_node`: 수확·플레이스 FSM
- `nav_harvest_test_node`: Nav2·팔·IW를 연결하는 코디네이터
- `mm_motion_bridge`: FSM JSON 명령을 MoveIt goal로 변환

## 2. 현재 구현과 레거시 구분

| 이름 | 현재 의미 |
|---|---|
| `isaacpjt/mm.py` | 현재 `--mm` 실행 드라이버, M0617+MoveIt |
| `src/smartfarm/mm_moveit` | 현재 M0617 MoveIt 패키지 |
| `isaacpjt/moveit_mm.py` | 과거/별도 UR10e MoveIt 데모 |
| `src/smartfarm/harvest_moveit` | UR10e MoveIt 패키지 |
| `rmp_target`, `rmp_home` JSON | 이름만 남은 레거시 명령명; 실제 소비자는 MoveIt bridge |

다른 브랜치에서 `mm.py`를 RMPflow 구현으로 덮으면 현재 통합 수확 경로가 사라진다.
`moveit_mm.py`의 UR10e 설정을 현재 M0617 설정과 섞지 않는다.

## 3. 관련 파일

### Isaac

- `isaacpjt/mm.py`
- `isaacpjt/robots/harvester_mm.py`
- `isaacpjt/robots/harvester_moveit.py`
- `isaacpjt/robots/m0617/`
- `isaacpjt/robots/quarter_basket_freecad/generated/`
- `isaacpjt/ros/robot_bridge.py`
- `isaacpjt/pjt_config/settings_mm.py`

### ROS 통합

- `src/smartfarm/harvest_vision/harvest_vision/vision_node.py`
- `src/smartfarm/harvest_vision/harvest_vision/manipulator_target_node.py`
- `src/smartfarm/harvest_vision/harvest_vision/nav_harvest_test_node.py`
- `src/smartfarm/harvest_vision/harvest_vision/fixed_harvest_moveit_node.py`
- `src/smartfarm/harvest_vision/launch/smartfarm_integration.launch.py`
- `src/smartfarm/mm_moveit/launch/auto_nav_harvest.launch.py`
- `src/smartfarm/mm_moveit/launch/nav2_harvest_bringup.launch.py`
- `src/smartfarm/mm_moveit/launch/vision_harvest_bringup.launch.py`
- `src/smartfarm/fleet_dispatch/config/moveit_nav2.yaml`

MoveIt 내부 상세는 `docs/moveit_mm_current_branch_reference.md`를 기준으로 한다.

## 4. 모델·스폰·관절

| 항목 | 값 |
|---|---|
| driver flag/name | `--mm` / `mm` |
| ROS namespace | `harvester_0` |
| Isaac root | `/World/Harvester` |
| 초기 pose | `(0.0, -12.0, floor_z)` |
| 베이스 | Ridgeback |
| 팔 | Doosan M0617, 6축 |
| 팔 장착 높이 | `0.30 m` |
| 그리퍼 | 동축 3축 1/4구 스쿱 |
| 카메라 | 스쿱 장착 D455 |

팔 관절:

```text
joint_1 ... joint_6
```

스쿱 관절:

```text
scoop_quarter_1_joint
scoop_quarter_2_joint
cutter_quarter_3_joint
```

스쿱 상태:

| 상태 | 관절각 |
|---|---|
| OPEN | `(0°, -90°, -180°)` |
| CLOSED | `(0°, 0°, 0°)` |
| CUT | `(0°, 0°, +50°)` |

커터는 실제 접촉·구동 오차를 고려해 약 `40°` 도달을 절단 완료로 판단한다.

## 5. Isaac articulation 설정

M0617 6축 drive는 베이스 가속 중 팔이 흔들리지 않도록 강화한다.

| 항목 | 기본값 |
|---|---:|
| arm minimum kp | `200000` |
| arm minimum kd | `20000` |
| scoop kp | `1200` |
| scoop kd | `80` |
| scoop effort 상한 | `40` |

환경변수:

- `MM_ARM_KP`
- `MM_ARM_KD`
- `SCOOP_KP`
- `SCOOP_KD`
- `SCOOP_EFFORT`

스쿱 gain을 팔과 같이 높이면 동축 셸 접촉이 폭발할 수 있으므로 별도 저 gain을
유지한다.

## 6. ROS 하드웨어 채널

| 방향 | 토픽 | 의미 |
|---|---|---|
| Isaac → ros2_control | `/harvester_0/hw_joint_states` | 실제 전체 관절 상태 |
| ros2_control → Isaac | `/harvester_0/joint_command` | 팔·스쿱 position 명령 |
| JSB → MoveIt | `/harvester_0/joint_states` | MoveIt용 관절 상태 |
| FSM → bridge/Isaac | `/harvester_0/cmd` | JSON 명령 |
| Isaac → FSM | `/harvester_0/status` | grasp/cut/tool 상태 |
| bridge → FSM | `/harvester_0/pipeline_status` | MoveIt 완료/실패 |

`hw_joint_states`와 `joint_states`를 분리한 이유는 Isaac의 dummy base 관절과 MoveIt
관절 상태가 같은 토픽에서 충돌하는 것을 막기 위해서다.

## 7. 베이스 Nav2

### 7.1 프레임과 토픽

| 용도 | 값 |
|---|---|
| map | `map` |
| odom | `odom` |
| 이동 베이스 | `mm_base` |
| 팔 planning base | `base_link` (`mm_base`보다 0.30 m 위) |
| cmd_vel | `/harvester_0/cmd_vel_safe` |
| odom | `/harvester_0/odom` |
| scan | `/harvester_0/scan` |

카메라·수확 좌표에서 `base_link`와 `mm_base`를 혼동하면 z가 `0.30 m` 어긋난다.

### 7.2 주행 제약

현재 Ridgeback 운용 규약은 x 전후진 + yaw 회전이다. y 횡이동은 막는다.

| 항목 | 값 |
|---|---:|
| min/max vx | `-0.3 / 0.8 m/s` |
| max vy | `0` |
| max wz | `8.0 rad/s` |
| xy/yaw goal tolerance | `0.15 m / 0.35 rad` |
| footprint | `0.96 × 0.80 m` |
| inflation radius | `0.45 m` |

Controller는 RotationShim + DWB, planner는 Navfn이다. 수확 통합에서는 기억된
위치 `(-0.54, -8.19, yaw=1.91)`로 자동 이동한다.

### 7.3 이동–팔 상호잠금

- Nav 도착 전 수확 gate를 열지 않는다.
- Nav 도착 후 베이스 정착을 기다린다.
- HOME을 먼저 확인한 뒤 BED_VIEW로 이동한다.
- 팔이 수확 시퀀스 중이면 베이스 재주행을 억제한다.
- 현재 통합 기본은 `nav_reposition_enabled=false`로 최초 정차 위치를 유지한다.

## 8. D455와 비전

카메라 토픽:

```text
/harvester_0/rgb
/harvester_0/depth
/harvester_0/camera_info
/harvester_0/tf
```

카메라 optical TF는 Ridgeback 섀시가 아니라 M0617 `base_link` 기준으로 계산한다.

비전 모델:

| 용도 | 기본 파일 | confidence |
|---|---|---:|
| 원거리 tomato detector | `finetuned_far.pt` | `0.45` |
| 근거리 품질 classifier | `finetuned_near.pt` | `0.55` |

현재 `use_quality_model=false`라 근거리 ripe/spoiled 재분류를 생략하고 검출된
`tomato`를 수확 대상으로 사용한다. depth는 `16UC1`이면 mm→m로 변환하며 검출
박스 내부 유효 depth의 중앙값·색상 마스크를 이용한다.

실제 제어 목표는 통합 안정성을 위해 `use_sim_ground_truth=true`이며, 비전 검출과
가까운 `/harvester_0/sim/tomato`를 매칭한다.

## 9. 수확 FSM

주요 상태:

```text
WAIT_NAV
→ WAIT_TARGET / WAIT_SIM_MATCH / RIPE_READY
→ APPROACH
→ PREGRASP
→ GRASP
→ GRIPPER_CLOSING
→ GRASP_VERIFY
→ CUT_VERIFY / BLADE_OPENING
→ RETRACT_CIRC
→ RETRACT_LIN
→ WAIT_BASKET
→ BASKET_APPROACH
→ BASKET_PLACE
→ PLACE_RELEASING
→ BASKET_RETRACT
→ GO_HOME
```

목표 안정화:

- `5`프레임 연속
- 프레임 간 위치 변화 `≤ 0.01 m`
- active sequence에 들어가면 목표를 완료까지 고정

수확 workspace:

```text
min = [0.15, -1.05, 0.15]
max = [1.25,  1.05, 1.80]
max horizontal radius = 1.50 m
minimum fruit z = 0.70 m
```

바스켓 workspace:

```text
min = [-1.05, -0.80, 0.15]
max = [ 1.35,  0.80, 1.80]
```

정밀 구간:

- pregrasp clearance: `0.15 m`
- LIN approach: `0.10 m`
- tool grasp reach: launch override `0.120 m`
- grasp TCP 허용거리: launch override `0.09 m`
- basket approach height: `0.15 m`
- basket center 보정: MM 쪽으로 `0.06 m`

## 10. 실패 재시도

1. APPROACH 안전점으로 후퇴 후 동일 목표 재파지: 최대 1회
2. HOME → BED_VIEW 재관측 후 새 목표 선택: 최대 1회
3. 모두 실패하면 최종 HOME 복귀

파지 성공 시 재시도 카운터를 초기화한다. 코디네이터는 `RETRY_VISION`을 받으면
홈 카메라 검색이 아니라 HOME→BED_VIEW 재관측 흐름을 수행한다.

## 11. IW 연동과 플레이스

- MM이 실제 `APPROACH`에 들어갈 때 `/iw/mission=FOLLOW`를 한 번 발행한다.
- IW의 `/iw/basket/empty_slot_pose`를 `map`에서 MM `base_link`로 변환한다.
- pose가 `2 s`보다 오래되면 사용하지 않는다.
- 직접 pose가 없으면 IW TF 기반 fallback을 사용할 수 있다.
- BASKET_APPROACH는 release보다 `0.15 m` 위다.
- release 후 같은 높이로 BASKET_RETRACT한다.

현재 플레이스 제어 흐름은 릴리즈까지 검증됐다. 빈 KLT collider 누락은 IW 쪽에서
수정했으며 수정 후 실제 안착은 재검증 대상이다.

## 12. 실행 방법

Isaac:

```bash
cd ~/cobot3_ws/isaacpjt
isaac_python main.py --mm --iw --fork --nav --camera
```

MM 단독 자동 수확:

```bash
ros2 launch mm_moveit auto_nav_harvest.launch.py
```

전체 통합:

```bash
ros2 launch harvest_vision smartfarm_integration.launch.py
```

Isaac Play가 멈추면 `/clock`과 ros2_control도 멈춘다.

## 13. 현재 검증 상태

확인:

- 자동 Nav 목표 도착
- HOME→BED_VIEW
- 비전–시뮬 GT 매칭
- APPROACH/PREGRASP/GRASP
- 파지 검증과 절단
- RETRACT
- IW FOLLOW와 바스켓 pose 수신
- BASKET_APPROACH/PLACE/RELEASE/RETRACT

재검증:

- KLT collider 수정 후 실제 토마토 안착
- 플레이스 후 IW 주행 중 토마토 유지
- 실패 재시도 전체 흐름
- 동시 기동 시 MoveIt/Nav2/IW lifecycle 안정성

## 14. 머지 우선순위

1. `mm.py`의 현재 M0617 MoveIt 경로
2. namespace `harvester_0` 및 hw/joint state 채널 분리
3. `mm_base`와 `base_link`의 0.30 m 프레임 구분
4. M0617+동축 스쿱+D455 조립
5. 수확 FSM의 목표 고정·재시도
6. IW FOLLOW 시점과 실제 빈 KLT pose 사용
7. basket workspace와 `0.06 m` 중심 보정
8. Nav2 도착 후 HOME→BED_VIEW gate

## 15. 머지 후 체크리스트

- [ ] `--mm`이 M0617 MoveIt 로봇을 하나만 생성하는가
- [ ] UR5/UR10e 레거시 팔이 함께 보이지 않는가
- [ ] `/harvester_0/hw_joint_states`와 `joint_states`가 분리되는가
- [ ] `mm_base→base_link` TF 높이가 0.30 m인가
- [ ] RGB/depth/camera_info와 optical TF가 같은 namespace인가
- [ ] Nav2가 y 횡이동 없이 정차 위치로 가는가
- [ ] Nav 도착 전에 수확이 시작되지 않는가
- [ ] HOME→BED_VIEW 뒤 안정된 목표만 선택하는가
- [ ] 파지·절단·후퇴가 정상인가
- [ ] APPROACH에서 IW FOLLOW가 한 번만 발행되는가
- [ ] 실제 빈 KLT pose로 플레이스하는가
- [ ] KLT 안착과 IW 주행 유지가 확인되는가

