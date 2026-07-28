# iwhub_control

iw.hub 운반 AMR(트랙 B) 베이스 제어 + 주행 미션 패키지.

## 노드

| 노드 | 역할 |
|---|---|
| `base_node` | `/cmd_vel` → 좌/우 바퀴 속도(`joint_command`)로 변환해 Isaac에 보내고, Isaac이 돌려주는 바퀴 각도(`joint_states`)로 오도메트리(+TF, `odom → base_link`)를 발행한다. 기구학 계산은 `kinematics.py`(rclpy 비의존), 노드는 얇은 래퍼 |
| `mission_nav_node` | IDLE/FOLLOW/FORKLIFT 미션을 온실 레인 그래프(`lanes.py`) 기반 경로 goal로 변환해 `NavigateThroughPoses`로 실행. 도킹 판정 후 지게차 사이클 서비스 호출 |
| `scan_self_filter_node` | IW 차체·팔레트·KLT에서 되돌아오는 2D 라이다 자기반사를 제거 |

## 순수 로직 모듈

- `kinematics.py` — (v, w) ↔ 좌우 바퀴 각속도 변환, 바퀴 각도 적분 오도메트리
- `lanes.py` — 온실 통로 레인 그래프. Nav2 반응형 계획 대신 통로 중심선만 결정적으로 주행해 베드 충돌을 설계상 방지

## Launch

- `iwhub_base.launch.py` — `base_node` 단독
- `iwhub_odom.launch.py` — 오도메트리 확인용
- `iwhub_nav2.launch.py` — IW 전용 Nav2 스택

## Behavior Tree

| 파일 | 역할 |
|---|---|
| `behavior_trees/forward_only_through_poses.xml` | FOLLOW·도크 접근·복귀 레인 경로를 Spin/BackUp 복구 없이 전진 위주로 수행 |
| `behavior_trees/dock_final_pose.xml` | 마지막 도크 goal의 yaw를 삭제하지 않고 `ComputePathThroughPoses → FollowPath`로 전달 |

`mission_nav_node`가 목적에 따라 두 트리를 선택한다. 도크 근처에서 Nav2가 끝난
뒤에는 별도 20 Hz POSITION/YAW 폐루프가 최종 정렬을 담당한다.

## 도구

- `tools/generate_greenhouse_map.py` — Isaac 온실/창고 설정에서 IW Nav2 정적 맵(`maps/greenhouse.yaml`)을 재생성. 기존 `map → odom` 정렬을 깨지 않도록 원점·범위를 유지한 채 갱신할 때 사용

## 테스트

```bash
colcon test --packages-select iwhub_control
```

`test/test_kinematics.py`, `test/test_lanes.py` — 기구학·레인 그래프 순수 로직 단위 테스트.
