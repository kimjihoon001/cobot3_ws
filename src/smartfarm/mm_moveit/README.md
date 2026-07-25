# mm_moveit — mm 수확 MM(m0617 + 동축 스쿱) MoveIt2

`harvest_moveit`(UR10e)에서 **팔만 Doosan m0617 로 갈아끼운** 사본이다.
두 MM 이 같은 씬에 동시에 뜰 수 있어야 하므로 MoveIt 설정을 MM 마다 완전히 분리했다
(2026-07-24 사용자 지시). 이 패키지는 `harvest_moveit` 을 전혀 참조하지 않는다.

| | harvest_moveit | **mm_moveit** |
|---|---|---|
| 팔 | UR10e | **Doosan m0617** |
| 팔 관절 | `shoulder_pan_joint` … | **`joint_1`…`joint_6`** |
| 플랜지 | `tool0` (ee_joint) | **`link_6`** (m0617 엔 tool0 없음) |
| 플래닝 그룹 | `ur_manipulator` | **`mm_manipulator`** |
| 네임스페이스 | `harvester_moveit` | **`harvester_0`** |
| Isaac 드라이버 | `isaacpjt/moveit_mm.py` (`--moveit`) | **`isaacpjt/mm.py` (`--mm`)** |
| 팔 description | `ur_description` 패키지 | **패키지 내장** (`urdf/m0617_macro.xacro`) |

그리퍼(스쿱 3축)·베이스는 양쪽 다 MoveIt 밖이다 — 스쿱은 동축 셸이라 충돌 모델이
서로 겹쳐 플래너가 항상 self-collision 으로 실패한다. Isaac 이 직접 위치 지령한다.

## 왜 m0617 description 을 패키지가 들고 있나

이 워크스페이스엔 `dsr_description2` 가 설치돼 있지 않다. 그래서 공식
`macro.m0617.white.xacro` 를 평탄화한 `isaacpjt/robots/m0617/m0617_full.urdf` 에서
링크/조인트/메시를 그대로 복사해 `urdf/m0617_macro.xacro` + `meshes/m0617/` 로 넣었다.
조인트 이름·origin·limit 이 Isaac 이 쓰는 `m0617.usd` 와 같은 소스라 MoveIt 해가
아티큘레이션에 그대로 적용된다.

## 실행

```bash
# 1) Isaac — isaacpjt 디렉터리에서 실행. ▶Play 필수.
isaac_python main.py --mm --camera

# 2-A) 팔/컨트롤러만 확인
ros2 launch mm_moveit m0617_moveit_bringup.launch.py
ros2 launch mm_moveit m0617_moveit_bringup.launch.py rviz:=false

# 2-B) 현재 위치에서 비전→MoveIt 수확 파이프라인
ros2 launch mm_moveit vision_harvest_bringup.launch.py

# 2-C) Nav2 이동→비전→MoveIt 수확 통합 시험
# 이 경우 Isaac도 --nav를 추가해 실행한다.
ros2 launch mm_moveit nav2_harvest_bringup.launch.py

# 2-D) 검증된 고정 목표 자동 이동→원샷 수확
ros2 launch mm_moveit auto_nav_harvest.launch.py
```

경로: MoveIt(OMPL/Pilz/CHOMP) → `arm_controller`(JTC) → `topic_based_ros2_control`
→ `/harvester_0/joint_command` → Isaac ArticulationController → 팔

상태 되돌림은 `/harvester_0/hw_joint_states`(Isaac 발행). `joint_states` 와 분리한
이유는 JSB 가 팔만 발행하는데 Isaac 전체관절과 겹치면 move_group 이 `dummy_base_*`
관절을 못 찾아 에러가 나기 때문이다.

## 검증 상태

2026-07-24 로컬에서 다음을 확인했다.

- `colcon build --packages-select harvest_vision mm_moveit` 성공
- xacro 전개 및 `check_urdf` 전체 링크 트리 파싱 성공
- `move_group`의 OMPL/Pilz/CHOMP 로딩 성공
- `joint_state_broadcaster`, `arm_controller`, `gripper_controller` 활성화 성공
- `vision_node`, `manipulator_target_node`, `mm_motion_bridge` 동시 기동 성공
- 이동+수확 통합 launch 인자 전개 성공

Isaac ▶Play를 포함한 실동작 시험에서 추가 확인할 항목:

- KDL IK의 실제 m0617 목표별 수렴성과 스쿱 방향
- `joint_limits.yaml` 의 `max_acceleration` — [4] 임의값, URDF 에 근거 없음
- SRDF `disable_collisions` 목록이 충분한지 (Setup Assistant 로 재생성 권장)
- 스쿱 어댑터 메시(`UR10eAdapter.stl`)가 m0617 플랜지 볼트패턴과 맞는지 —
  현재는 기하만 재사용. 실물 어댑터는 별도 CAD 필요.
