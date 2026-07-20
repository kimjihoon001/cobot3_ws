# warehouse_dock

ForkliftB가 창고의 빈 팔레트 6개를 순서대로 AMR에 공급하고, 토마토가 채워져
돌아온 팔레트를 원래 슬롯에 되돌리는 ROS 2 패키지다.

## 현재 작업 순서

1. 첫 AMR 도킹 이벤트: `Pallet_00`을 랙에서 꺼내 AMR에 상차
2. 다음 도킹 이벤트: 채워진 `Pallet_00`을 `Slot_00`에 복귀
3. 같은 도킹 상태에서 `Pallet_01`을 AMR에 상차
4. 위 과정을 `Pallet_05`가 창고에 복귀할 때까지 반복
5. 지게차가 대기 위치로 이동하고 `COMPLETE` 유지

## 빌드와 실행

ROS 2 터미널:

```bash
cd ~/cobot3_ws
source /opt/ros/humble/setup.bash
colcon build --packages-up-to warehouse_dock
source install/setup.bash
export ROS_DOMAIN_ID=108
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

ros2 run warehouse_dock fork_lift_node
```

Isaac Sim 터미널은 시스템 ROS를 source하지 않고 별도로 실행한다.

```bash
cd ~/cobot3_ws/isaacpjt
isaac_python main.py --fork
```

AMR 노드가 아직 없을 때는 시험용 도킹 신호를 한 번씩 보낼 수 있다.

```bash
ros2 topic pub --once /forklift/amr_docked std_msgs/msg/Bool "{data: true}"
```

- 첫 번째 신호: 빈 `Pallet_00` 공급
- 이후 신호: 현재 팔레트 회수·창고 적재 후 다음 빈 팔레트 공급
- 여섯 번째 복귀 신호까지 처리하면 전체 완료

상태 확인:

```bash
ros2 topic echo /forklift/status
ros2 topic echo /forklift/clear
ros2 topic echo /forklift/task_complete
```

## 주요 토픽

| 토픽 | 타입 | 방향 | 용도 |
|---|---|---|---|
| `/handoff/tray_ready` | `smartfarm_interfaces/HandoffEvent` | 입력 | 실제 AMR 도킹 이벤트 |
| `/forklift/amr_docked` | `std_msgs/Bool` | 입력 | 개발용 도킹 이벤트 |
| `/forklift_0/joint_states` | `sensor_msgs/JointState` | 입력 | 포크 높이와 연결 확인 |
| `/forklift_0/pose` | `geometry_msgs/PoseStamped` | 입력(선택) | 지게차 위치 보정. 없으면 dead reckoning |
| `/forklift_0/joint_command` | `sensor_msgs/JointState` | 출력 | 포크·조향·구동 명령 |
| `/forklift/clear` | `std_msgs/Bool` | 출력 | 포크 인출 및 대기 위치 복귀 완료 |
| `/forklift/task_complete` | `std_msgs/Int32` | 출력 | 채워진 팔레트의 창고 복귀 완료 ID |
| `/forklift/status` | `std_msgs/String` | 출력 | 현재 작업 상태 |

## 임시 위치 파라미터

현재 대기 위치와 AMR 위치는 임시값이다.

```yaml
initial_pose: [0.0, 15.5, 1.5708]
wait_pose: [4.5, 15.0, 1.5708]
amr_hole_center: [2.0, 14.5, 0.45]
```

실제 위치가 정해지면 다음처럼 실행 시 덮어쓴다.

```bash
ros2 run warehouse_dock fork_lift_node --ros-args \
  -p wait_pose:="[4.0, 15.0, 1.5708]" \
  -p amr_hole_center:="[2.0, 14.5, 0.45]"
```

`amr_hole_center`의 세 번째 값은 AMR 위 팔레트의 **구멍 중심 월드 Z**다.

실물 확인 후 반드시 보정할 값:

- `fork_tip_offset`: 지게차 기준점에서 포크 끝까지 거리
- `fork_center_z_at_zero`: `lift_joint=0`일 때 포크 중심 월드 Z
- `insertion_depth`: 팔레트 안으로 실제 삽입할 깊이
- `wait_pose`: 지게차 대기 위치
- `amr_hole_center`: AMR 도킹 시 팔레트 구멍 중심

현재 위치 추정은 명령 적분을 사용한다. `/forklift_0/pose`가 연결되면 실제 pose를
자동으로 우선하므로 최종 통합에서는 Isaac 쪽 pose 발행을 연결하는 것이 권장된다.
