# 통합 수확 디버그 로그 (단독 OK → 통합에서 파지 실패)

> 목적: 이 문서만 읽고도 제3자(GPT/다른 개발자)가 현재 상황·검증된 사실·다음 액션을
> 이어받을 수 있게 한다. **작업하면서 계속 갱신**한다.
> 최종 갱신: 2026-07-24

---

## 0. 한 줄 요약

`auto_nav_harvest.launch.py`(단독 MM: 고정지점 자율주행 → 그리퍼 파지)는 **완벽하게 동작**한다.
그런데 이걸 그대로 감싸고 IW/지게차를 얹은 `smartfarm_integration.launch.py`에서는
**MM이 토마토 목표를 제대로 못 잡고 엉뚱한 곳으로 팔을 꼬라박는다.**
→ 파지에 관여하는 ROS 배선/런치 인자는 **단독과 통합이 동일함을 코드로 검증**했다(§3, §4).
→ 남은 원인은 **Isaac 씬(`--iw --fork`)과 리소스 경합**뿐이다(§6). 아직 런타임 확증 전.

---

## 1. 시스템 구성 & 파일 구조 (2026-07-24 리네임 반영)

MM 수확 런치는 계층형 include다. 파일명이 최근 바뀌었다:

| 역할 | 현재 파일 (mm_moveit/launch) | 이전 이름 |
|---|---|---|
| 단독 골드스탠다드 (주행+파지) | `auto_nav_harvest.launch.py` | fixed_harvest_moveit |
| Nav2+수확 파이프라인 | `nav2_harvest_bringup.launch.py` | nav_harvest_pipeline |
| 비전+MoveIt+bridge+manipulator | `vision_harvest_bringup.launch.py` | harvest_pipeline |
| MoveIt(m0617) bringup | `m0617_moveit_bringup.launch.py` | moveit_isaac |

### include 트리

```
[단독]  auto_nav_harvest.launch.py                     (Isaac: --mm --nav --camera)
         └─ nav2_harvest_bringup.launch.py
             ├─ vision_harvest_bringup.launch.py
             │   ├─ m0617_moveit_bringup.launch.py   (move_group·rsp·servo·rviz)
             │   ├─ vision_node                       (harvest_vision)
             │   ├─ vision_debug_view
             │   ├─ mm_motion_bridge.py               (mm_moveit)
             │   └─ manipulator_target_node           (harvest_vision)
             ├─ fleet_dispatch/harvester_nav2.launch.py
             └─ coordinator = fixed_harvest_moveit_node (harvest_vision)

[통합]  smartfarm_integration.launch.py                (Isaac: --mm --iw --fork --nav --camera)
         ├─ nav2_harvest_bringup.launch.py   ← 단독과 동일 include, 인자도 동일(§4)
         ├─ iwhub_control/iwhub_nav2.launch.py         (ns=iwhub_0, TF 격리)
         ├─ iwhub_control/mission_nav_node             (IW 팔로우, 읽기전용 TF)
         └─ warehouse_dock/fork_lift_node
```

---

## 2. 증상

- 단독: 주행 → 홈 → 베드뷰 자세 → 토마토 탐지 → 파지 **정상**.
- 통합: 동일 시퀀스인데 **목표 좌표가 틀려서 팔이 이상한 곳으로 이동/충돌**.

---

## 3. 파지 좌표가 만들어지는 경로 (프레임 체인)

1. `vision_node`: RGB-D + YOLO로 토마토 검출 → **핀홀 역투영**으로 3D점 계산.
   - `vision_node.py:375 _pose_for_candidate` — `pose.header = source_msg.header`
     즉 **depth 이미지의 frame_id(카메라 광학 프레임) 그대로** `vision/approach_target`(PoseStamped) 발행.
   - **TF를 전혀 쓰지 않는다**(TransformListener/lookup_transform 없음).
2. `manipulator_target_node`: `vision/approach_target`를 받아 `base_link`(planning frame)로 **TF 변환**.
   - 이 노드만 카메라프레임→base_link 변환을 수행. `/harvester_0/tf` 사용.
3. `mm_motion_bridge.py` → MoveIt(`mm_manipulator`, planning_frame=base_link)로 파지 실행.

### ★ 실제 파지 목표는 비전이 아니라 Isaac 그라운드트루스 (2026-07-24 확인)

launch 파라미터(vision_harvest_bringup의 manipulator)에 `use_sim_ground_truth=True`,
`direct_sim_grasp=True` 가 있다. 즉 **정밀 파지점은 비전이 아니라 Isaac이 계산한 GT**다.
비전(`approach_target`)은 클래스/게이팅·매칭용으로만 쓰인다.

- Isaac `isaacpjt/mm.py:562 _publish_sim_tomato`:
  - `world_to_base = inverse(base_link 월드 변환)` — **MM base_link의 실시간 월드 포즈**로 생성 (mm.py:574)
  - ripe 과실 bbox 중심 world → **base 프레임으로 변환** `p = world_to_base.Transform(world)` (mm.py:592)
  - 팔 작업영역 필터 `0.0<=p.x<=1.5 and abs(p.y)<=1.2 and 0.1<=p.z<=1.9` 통과분만 (mm.py:594)
  - `/harvester_0/sim/tomato`(std_msgs/String JSON: id, position[base-frame], height, radius) 발행
- manipulator: `use_sim_ground_truth`면 이 GT를 파지 목표로 사용
  (`_nearest_sim_tomato`/`_match_sim_tomato`, manipulator_target_node.py:453-469).

**따라서 파지 목표(base-frame)는 오직 "MM이 최종 정차한 base_link 월드 포즈"의 결정함수다.**
카메라프레임 3D점(vision) 경로와 무관하게, base 포즈가 같으면 목표가 같고 다르면 통째로
평행이동/회전한다.

---

## 4. 단독 vs 통합: nav2_harvest_bringup 인자 대조 (검증: **완전 동일**)

| 인자 | 단독 auto_nav_harvest | 통합 smartfarm_integration | 일치 |
|---|---|---|---|
| coordinator_executable/name | fixed_harvest_moveit_node | fixed_harvest_moveit_node | ✓ |
| auto_nav_goal | true | true | ✓ |
| nav_reposition_enabled | false | false | ✓ |
| resume_search_after_start_sec | 0.0 | 0.0 | ✓ |
| fixed_goal_x / y / yaw | -0.54 / -8.19 / 1.91 | -0.54 / -8.19 / 1.91 | ✓ |
| initial_pose x/y/yaw | (미전달→기본) 0 / -12 / 0 | 0 / -12 / 0 | ✓ |
| ns | harvester_0 | harvester_0 | ✓ |
| nav_rviz/moveit_rviz/debug_view | true | true | ✓ |

> 참고: 통합이 예전엔 좌표를 -0.487/-8.207로 넣어 5cm 어긋나 있었으나, 2026-07-24 단독값
> -0.54/-8.19로 통일 완료. 지금은 좌표 차이 없음.

---

## 5. TF 격리 검증 (파지 배선이 IW로 오염되지 않음)

| 노드 | /tf 배선 | 근거 |
|---|---|---|
| move_group·rsp·servo·rviz | `/tf→/harvester_0/tf` 절대 remap | m0617_moveit_bringup.launch.py:52-54 (_ROBOT_STATE_REMAP) |
| manipulator_target_node | `/tf→tf`(ns harvester_0) | vision_harvest_bringup.launch.py:119 |
| mm_motion_bridge | `/tf→tf` | vision_harvest_bringup.launch.py:79 |
| vision_node | remap 없음 but **TF 미사용** → 무해 | vision_node.py (listener 없음) |
| IW nav2 | `/tf→/iwhub_0/tf`, frame_prefix=iwhub_0/ | iwhub_nav2.launch.py:42,61,66 |
| mission_nav_node | **읽기 전용**(TransformListener), `/iwhub_0/tf`만 | mission_nav_node.py:84-85,156 |
| fork_lift_node | TF broadcast 없음 | (grep 결과 없음) |

**결론: IW/지게차 어느 노드도 `/harvester_0/tf`(MM 파지가 쓰는 트리)에 쓰지 않는다.**
ROS 레벨에서 파지 좌표 오염 경로는 발견되지 않음.

---

## 6. 가설 (파지 목표 = MM 최종 base 포즈의 결정함수, §3 참조)

파지 목표는 base-frame GT라 **MM 최종 정차 base 포즈가 같으면 목표도 같다**. 따라서
"통합에서만 꼬라박음" = **통합에서 MM 최종 base 포즈가 단독과 다르다**로 귀결. 왜 다른가:

### H4. IW가 MM Nav2 코스트맵의 장애물이 되어 최종 정차 포즈가 어긋남 (최유력)
`mission_nav_node`가 IW를 MM 뒤 `follow_offset_x=1.6955m`로 팔로우시킨다. MM의 360° 라이다
범위 안이라 **MM local costmap에 IW가 장애물로 찍힌다.** 그럼 MM Nav2가 고정 goal(-0.54,-8.19)
도달 시 IW 회피로 **goal 허용오차(기본 xy~0.25m/yaw~0.25rad) 안에서 다른 위치·yaw로 정차**.
base가 어긋난 만큼 GT 과실 좌표가 평행이동·회전 → 팔이 어긋난 점을 겨냥 → ram.
- 카메라는 왼쪽 베드를 보므로 IW가 시야엔 안 들어옴(사용자 지적과 일치). 문제는 **시야가 아니라 base 포즈**.
- 판별: §7-A (map→base_link 최종 포즈 단독 vs 통합 비교), §7-D (MM costmap에 IW 찍히나).

### H2(잔여). 리소스 경합 → base 미정착/드리프트
nav2 2스택+rviz3+Isaac 3로봇 → RTF 급락. base가 완전히 서기 전 목표 확정/실행되면 어긋남.
- 판별: §7-A 에서 정차 후 base 포즈가 시간에 따라 흔들리는지.

### 배제됨
- ~~H1 카메라 가림~~: 비전은 최종 정차 위치·왼쪽 베드에서만 사용, IW는 시야 밖(사용자 지적).
- ~~TF 오염~~: IW/지게차 어느 노드도 /harvester_0/tf 미기록(§5).
- ~~TF stale 값~~: manipulator는 이미지 stamp 기준 transform, 못 찾으면 드롭(코드 확인).

---

## 7. 다음에 실행할 진단 (런타임 필요 — 사용자 실행)

통합 실행 중:
```
ros2 launch harvest_vision smartfarm_integration.launch.py
```

- **A) ★핵심: MM 최종 정차 base 포즈 단독 vs 통합 비교 (H4/H2)**
  두 실행 각각, 파지 직전(pipeline_status가 SEARCHING_TOMATO일 때):
  `ros2 run tf2_ros tf2_echo map base_link --ros-args -r /tf:=/harvester_0/tf`
  → 단독과 통합의 (x,y,yaw)가 다르면 H4 확정. 정차 후에도 값이 계속 흔들리면 H2.

- **B) 실제 파지 목표(GT) 비교**
  `ros2 topic echo /harvester_0/sim/tomato`        # base-frame 과실 좌표(파지 목표)
  `ros2 topic echo /harvester_0/manipulator/target_pose`
  → 단독 대비 position이 평행이동/회전돼 있으면 base 포즈 어긋남(A와 교차검증).

- **C) 어느 단계에서 틀어지나**
  `ros2 topic echo /harvester_0/pipeline_status`
  → NAVIGATING / WAIT_ARM_HOME.. / WAIT_ARM_BED_VIEW.. / SEARCHING_TOMATO 중 위치.

- **D) IW가 MM 코스트맵 장애물인지 (H4 원인)**
  RViz(nav_rviz)에서 MM local_costmap에 IW 위치에 장애물(검정) 찍히는지, 또는
  `ros2 topic echo /harvester_0/local_costmap/costmap` 확인. Nav2 goal 결과(SUCCEEDED 위치)도 비교.

### H4 확정 시 해결 방향(후보)
- MM 수확 정차 완료까지 IW를 팔로우시키지 않고 **먼 대기 지점**에 세운다(mission_nav_node 게이팅).
- 또는 IW 팔로우 오프셋을 MM 라이다/코스트맵 밖으로 키운다.
- 또는 MM Nav2 goal 도달 허용오차를 좁혀 정차 포즈 편차를 줄인다(단, IW가 goal을 막으면 근본 해결 아님).

---

## 8. 검증된 사실 vs 미확인

- [x] 파지 ROS 배선/런치 인자 단독==통합 (§4, §5)
- [x] IW/지게차가 /harvester_0/tf 오염 안 함 (§5)
- [x] **실제 파지 목표 = Isaac base-frame GT(sim/tomato), 비전 아님 → base 포즈의 결정함수 (§3)**
- [x] manipulator는 이미지 stamp 기준 TF 변환(stale값 미사용) — TF지연설 약함
- [ ] **MM 최종 정차 base 포즈가 단독≠통합인가? (§7-A) — 미확인 (최우선 판별)**
- [ ] IW가 MM local_costmap 장애물로 찍히나? (§7-D) — **미확인**

---

## 9. 이번 세션에서 바꾼 것

- `smartfarm_integration.launch.py`: `harvest_x/y` 기본값 `-0.487/-8.207` → `-0.54/-8.19`
  (단독 골드스탠다드와 통일). docstring 예시 좌표도 정리. **커밋 안 함(로컬).**
