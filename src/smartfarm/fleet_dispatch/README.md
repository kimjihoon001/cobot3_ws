# fleet_dispatch

트랙 B 공용 — Nav2/AMCL 런치와 주행 안전 노드.

## 노드

| 노드 | 역할 |
|---|---|
| `cmd_vel_watchdog` | Nav2 `Twist`를 Isaac에 중계하다가, 입력이 `timeout_sec` 동안 끊기면 0을 계속 발행한다. Isaac의 `ROS2SubscribeTwist`는 마지막 값을 유지해 적분을 계속하므로, Nav2가 죽어도 로봇이 마지막 속도로 계속 움직이는 사고를 막는다 |
| `nav2_lifecycle_activator` | DDS 서비스 응답이 늦게 오는 환경에서 Nav2 lifecycle 노드(`map_server` 등)가 configure/activate에서 멈추는 문제를 재시도로 복구 |

## Launch

- `harvester_nav2.launch.py` — 수확 MM(Ridgeback) Nav2. `config/harvester_nav2.yaml` 기반 `nav2_bringup` 조합.
  - `slam:=true` — SLAM Toolbox로 맵 생성(수동 주행)
  - `slam:=true explore:=true` — `explore_lite`(`src/m-explore-ros2`)를 같이 띄워 프런티어 자동 탐사
  - `map:=<yaml>` — 정적맵 + AMCL
  - Isaac 쪽 짝: `isaac_python main.py --mm --nav`
- `track_b_nav2.launch.py` — IW/AMR용 골격만 있는 TODO 스텁(미완성, 실사용 아님)

## 설정/자산

- `config/harvester_nav2.yaml`, `config/moveit_nav2.yaml` — Nav2 파라미터
- `behavior_trees/navigate_to_pose_forward_only.xml` — 후진 없이 전진만 하는 네비게이션 비헤이비어 트리
- `maps/farm_gen.*` — 생성된 정적맵
