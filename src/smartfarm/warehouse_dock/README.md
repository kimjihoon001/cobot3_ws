# warehouse_dock

ForkliftB가 창고의 빈 팔레트 6개를 순서대로 AMR에 공급하고, 토마토가 채워져
돌아온 팔레트를 원래 슬롯에 되돌리는 ROS 2 패키지다.

## 순환 운용: 상차 노드 + 회수 노드

순환 운용에서는 두 ROS 노드가 역할을 나눈다.

- `fork_lift_node`: 최초 빈 `Pallet_n`을 랙에서 꺼내 IW에 상차
- `fork_lift_return_node`: IW 귀환 신호를 받으면 `Pallet_n`을 n번 슬롯에
  복귀하고, 다음 `Pallet_(n+1)%6`을 IW에 상차한 뒤 다음 귀환을 기다림

두 노드는 같은 지게차를 제어하지만 동시에 명령하지 않는다. 상차 노드는 최초
상차 큐가 끝나면 명령 발행을 멈추고, 회수 노드는 IW 도킹 이벤트를 받은 동안만
명령을 발행한다. 기존 한 노드 안의 Pallet_00→01 시험을 다시 사용할 때만
`fork_lift_node`에 `-p enable_internal_return_cycle:=true`를 지정한다.

랙 직선 삽입 전 기본 정렬 조건은 X축 오차 ±4cm, 진입각 오차 ±3°이며,
조건을 만족하지 못하면 2.5m 후진 후 재진입한다. 랙 재진입과 IW 축 보정은
각각 최대 5회로 제한한다. 관련 파라미터는 `rack_entry_x_tolerance`,
`rack_entry_yaw_tolerance`, `max_reentry_attempts`다.

```bash
# 터미널 1: Isaac Sim
cd ~/cobot3_ws/isaacpjt
isaac_python main.py --iw --fork

# 터미널 2: 최초 상차 작업. 실행 후 0~5번 중 시작 팔레트를 입력한다.
ros2 run warehouse_dock fork_lift_node

# 터미널 3: 순환 회수 작업
ros2 run warehouse_dock fork_lift_return_node
```

최초 상차가 끝나면 `/forklift/pallet_on_iw`에 팔레트 번호가 전달된다. IW가
상하차 위치로 돌아왔을 때 실제 연동은 `/handoff/tray_ready`, 단독 시험은 아래
Bool 토픽으로 회수 사이클을 시작한다.

```bash
ros2 topic pub --once /forklift/amr_docked std_msgs/msg/Bool "{data: true}"
```

회수 노드를 최초 상차 완료 후 늦게 실행해 번호 이벤트를 놓쳤다면 시작 번호를
직접 지정한다.

```bash
ros2 run warehouse_dock fork_lift_return_node --ros-args -p initial_pallet:=2
```

## 기존 단일 노드 Pallet_00→01 시험

`enable_internal_return_cycle:=true`로 실행하는 기존 시험은 다음 순서로
`Pallet_00`을 IW에 상차한 뒤 대기한다.

1. 대기 위치에서 포크를 실측 구멍 중심보다 6cm 낮은 `0.12407m`로 조정
2. 포크 높이를 유지한 채 1.5m 후진
3. 반경 1.2m U턴으로 `Pallet_00` 접근축 정렬
4. U턴 직후 조향 0°로 입구 쪽 0.8m 안전 후진
5. 최대 ±35° 제한 조향으로 팔레트 정면 대기점까지 전진
6. 조향 0°로 0.95m 직진해 포크 삽입
7. `/World/Warehouse/Pallet_00`을 포크 캐리지에 FixedJoint로 연결
8. 리프트를 2cm씩 10단계로 `0.32407m`까지 올림
9. 팔레트를 든 상태로 랙에서 직선 후진
10. 벽과 떨어진 안전 지점에서 반경 1.2m U턴을 정확히 1회 수행
11. 창고 입구 중앙의 기존 대기 위치 `(0.0, 14.5, -90°)`로 이동해 정렬
12. 조향 없이 2m 전진해 IW 팔레트 중심 `(0.0, 10.84885)`에 정렬
13. 리프트를 약 2cm씩 5단계로 IW 높이용 `0.24378m`까지 천천히 내림
14. 포크-팔레트 FixedJoint를 해제해 팔레트를 IW에 내려놓음
15. 조향 없이 2m 후진해 포크를 빼고 대기 위치 `(0.0, 14.5, -90°)`로 복귀

첫 상차가 완료된 뒤 다음 도킹 신호를 받으면 반대 작업을 실행한다.

1. 대기 위치에서 포크를 IW의 `Pallet_00` 구멍 높이로 조정
2. 조향 0°로 IW까지 2m 직진해 포크 삽입
3. 포크-팔레트를 연결하고 2cm씩 10단계로 20cm 상승
4. 팔레트를 든 상태로 2m 직선 후진해 대기 위치로 복귀
5. 기존에 검증된 후진·단일 U턴·제한 조향 경로로 0번 팔레트 앞에 정렬
6. 조향 0°로 0.95m 직진해 `Pallet_00`을 원래 위치에 삽입
7. 2cm씩 10단계로 20cm 하강한 뒤 연결 해제
8. 포크를 0.95m 후진으로 빼고, 선반에서 멀어지도록 1m 더 후진
9. 안전 지점에서 포크를 상단 `Pallet_01` 구멍 높이 `1.02407m`로 조정
10. 1m 전진 후 조향 0°로 0.95m 진입해 `Pallet_01` 연결
11. 2cm씩 10단계로 20cm 상승하고 1.95m 후진해 랙에서 완전히 인출
12. 랙과 충분히 멀어진 지점에서 IW 운반 높이로 천천히 하강
13. 기존 단일 U턴 경로로 입구 중앙 대기 위치에 정렬
14. 비어 있는 IW에 `Pallet_01`을 내려놓고 포크를 뺀
15. 대기 위치 `(0.0, 14.5, -90°)`로 복귀

현재 시험 범위는 `Pallet_00` 복귀와 `Pallet_01` IW 상차까지이며,
상차 완료 후에는 대기 위치를 유지한다.

고정조인트는 연결 순간의 상대 자세를 보존하므로 팔레트가 포크 원점으로 순간이동하거나
튕기지 않는다. IW에 팔레트를 내려놓고 안정화한 뒤 연결을 해제하며, 포크가 완전히
대기 위치로 빠질 때까지 IW 상하차 축에서 조향하지 않는다.

입구 중앙 대기점에서는 포크가 AMR(-Y)을 향한다. 랙 작업을 시작할 때는 먼저
창고 안쪽으로 1.5m 후진하고, `temp/spikes/06_pallet_lift.py`와 동일한 ForkliftB
조향 방식으로 우측 U턴한다. 최대 조향은 70°로 제한하되, 첫 U턴은 종료점이
`Pallet_00`의 X축에 정확히 맞도록 약 59.7°(반경 1.2m)를 사용한다. U턴이 끝나면
실제 `/forklift_0/pose`의 누적 회전각과 목표 yaw가 모두 180°에 도달했을 때만
성공 처리한다. 약 13.4초 조건은 성공 조건이 아니라 pose 미도달 시 안전 정지하는
watchdog이다. 이어서
조향 0°로 0.8m 후진해 벽과 거리를 만든 뒤, 최대 ±35° 제한 조향으로
`Pallet_00` pre-pick 위치까지 접근한다. 이 접근 단계도 누적 회전 45°를 넘으면
안전 정지하므로 두 번째 U턴이나 제자리 반복 회전으로 이어지지 않는다.
포크 구멍 높이는 대기 위치에서 먼저 맞추고 랙 접근 중 그대로 유지한다. 정면
대기점에서는 조향 0°를 유지하고 정확히 0.95m 직진해 포크를 팔레트 구멍에
삽입한다. 이 0.95m는 `rack_fork_insert_travel`로
조정할 수 있으며 후진·U턴 궤적에는 영향을 주지 않는다.
ForkliftB 원본 승강 드라이브는 자중으로 목표보다 약 47mm 처져 첫 리프트 단계가
끝나지 않았다. Isaac 생성 시 승강 드라이브를 `stiffness=1,000,000`,
`damping=50,000`, `maxForce=30,000N`으로 보강하며, GPU 단독 측정에서
`0.18407m` 명령에 실제 `0.179m`까지 도달해 현재 허용오차 15mm 안으로 확인됐다.
팔레트를 집은 뒤에는 포크를 빼며 후진하고, 입구 쪽 U턴 시작점까지
후진한 다음 다시 우측 U턴해 입구 중앙축에 정렬한다. 이후 상하차 위치까지
직진해 AMR 위에 팔레트를 내려놓고 대기 위치로 복귀한다.

## 반복 실행 안정화

- ForkliftB 차체 평면 이동은 Isaac physics timestep 기반 Ackermann 적분 하나만
  사용한다. 동시에 적용되던 물리 바퀴 추진은 자동화 모드에서 0으로 두어 궤적이
  실행마다 달라지는 현상을 막는다.
- 이동 중 남은 PhysX 선속도·각속도는 제거하며, GPU 시험에서 0.343m 이동 후
  정지 드리프트는 약 0.43mm였다.
- U턴은 제한시간만 지났다고 다음 단계로 넘어가지 않는다. 실제 180° pose가 아니면
  오류로 정지하고, 190°를 넘겨 반복 회전하는 것도 차단한다.
- 시작 pose가 대기 위치에서 0.20m 또는 12° 이상 벗어나거나 pose/joint-state
  피드백이 끊기면 움직이지 않고 오류로 정지한다.
- 같은 PC에서 `fork_lift_node`가 두 개 실행되면 두 번째 프로세스는 시작을 거부한다.

반복 시험은 Isaac Sim과 기존 ROS 노드를 모두 종료해 지게차·팔레트를 초기화한 뒤
각각 한 번만 실행한다.

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
`--iw --fork` 조합은 Warehouse 단독 시험 모드로 동작한다. 빈 AMR을 상하차 위치
창고 입구 밖 중앙축 `(0.0, 10.84885)`에 배치하고 기본 적재 화물은 생성하지 않는다.
`main.py`가 Isaac 설치 안의 Humble 라이브러리를 찾아 ROS 브리지 환경으로 한 번
자동 재실행하므로 별도의 `LD_LIBRARY_PATH` 설정은 일반적으로 필요하지 않다.
또한 지게차 접촉 시험 중 AMR이 중력이나 충격으로 넘어지지 않도록 월드
FixedJoint로 고정한다. 매 프레임 텔레포트하지 않으므로 AMR이 날아가는 현상을
피한다. 이 고정은 `--iw --fork` 단독 시험 조합에만 적용된다.

```bash
cd ~/cobot3_ws/isaacpjt
isaac_python main.py --iw --fork
```

`fork_lift_node`는 `/forklift_0/joint_states`가 연결되면 2초 뒤 첫 빈 팔레트
상차를 자동으로 시작한다. 실제 도킹 토픽으로 통합 시험할 때는 자동 시작을 끈다.

```bash
ros2 run warehouse_dock fork_lift_node --ros-args -p auto_start:=false
```

AMR 노드가 아직 없을 때는 시험용 도킹 신호를 한 번씩 보낼 수 있다.

```bash
ros2 topic pub --once /forklift/amr_docked std_msgs/msg/Bool "{data: true}"
```

- `auto_start:=true`로 지정하면 첫 번째 상차는 연결 후 2초 뒤 자동 시작
- 첫 상차 완료 후 한 번 보낸 신호: IW의 `Pallet_00` 회수·0번 복귀 후
  `Pallet_01`을 비어 있는 IW에 상차
- `Pallet_01` 상차 완료 후 추가 신호는 무시하고 대기 위치를 유지

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
| `/forklift_0/joint_command` | `sensor_msgs/JointState` | 출력 | 포크·조향·구동 및 `pallet_attach`/`pallet_id` 커플러 명령 |
| `/forklift/clear` | `std_msgs/Bool` | 출력 | 포크 인출 및 대기 위치 복귀 완료 |
| `/forklift/task_complete` | `std_msgs/Int32` | 출력 | 채워진 팔레트의 창고 복귀 완료 ID |
| `/forklift/status` | `std_msgs/String` | 출력 | 현재 작업 상태 |

## 임시 위치 파라미터

현재 대기 위치와 AMR 위치는 임시값이다.

```yaml
initial_pose: [0.0, 14.5, -1.5708]
wait_pose: [0.0, 14.5, -1.5708]
amr_hole_center: [0.0, 10.84885, 0.45]
```

실제 위치가 정해지면 다음처럼 실행 시 덮어쓴다.

```bash
ros2 run warehouse_dock fork_lift_node --ros-args \
  -p wait_pose:="[4.0, 15.0, 1.5708]" \
  -p amr_hole_center:="[2.0, 14.5, 0.45]"
```

`amr_hole_center`의 세 번째 값은 AMR 위 팔레트의 **구멍 중심 월드 Z**다.

## 팔레트/포크 GPU 실측 좌표

Isaac Sim 5.1 원본 `pallet.usd`와 `forklift_b.usd`의 메시 꼭짓점을 월드 좌표로
변환해 측정했다. 아래 팔레트 좌표는 피벗, 즉 팔레트 바닥 중심 `(X, Y, Z)`이며,
구멍 좌표는 각 팔레트의 좌·우 채널 중심이다. 단위는 m다.

| 번호 | 팔레트 바닥 중심 | 왼쪽 구멍 중심 | 오른쪽 구멍 중심 | 리프트 목표 |
|---:|---|---|---|---:|
| 0 | `(-2.400, 20.400, 0.322)` | `(-2.65838, 20.400, 0.39029)` | `(-2.14163, 20.400, 0.39029)` | `0.18407` |
| 1 | `(-2.400, 20.400, 1.222)` | `(-2.65838, 20.400, 1.29029)` | `(-2.14163, 20.400, 1.29029)` | `1.08407` |
| 2 | `(-0.800, 20.400, 0.322)` | `(-1.05838, 20.400, 0.39029)` | `(-0.54163, 20.400, 0.39029)` | `0.18407` |
| 3 | `(-0.800, 20.400, 1.222)` | `(-1.05838, 20.400, 1.29029)` | `(-0.54163, 20.400, 1.29029)` | `1.08407` |
| 4 | `( 0.800, 20.400, 0.322)` | `( 0.54163, 20.400, 0.39029)` | `( 1.05838, 20.400, 0.39029)` | `0.18407` |
| 5 | `( 0.800, 20.400, 1.222)` | `( 0.54163, 20.400, 1.29029)` | `( 1.05838, 20.400, 1.29029)` | `1.08407` |

표의 리프트 목표는 포크를 구멍에 삽입할 때의 값이다. 팔레트 연결 후 20cm
상승 목표는 하단 팔레트 `0.38407`, 상단 팔레트 `1.28407`이다.

팔레트 로컬 구멍의 수직 범위는 `Z=0.02053~0.11605`, 중심은 `0.06829`다.
`lift_joint=0`일 때 실제 삽입 구간 포크 날은 월드 `Z=0.179587~0.232846`,
중심은 `0.2062165`다. 따라서 기존 기본값 `fork_center_z_at_zero=0.05`를
`0.2062165`로 교정했다. 포크 날 중심을 구멍 중심에 맞추면 위 표의 리프트 목표가
되고, 구멍 상·하단과 포크 사이에 각각 약 21mm의 수직 여유가 생긴다.

측정은 다음 명령으로 재현할 수 있다.

```bash
cd ~/cobot3_ws/isaacpjt
isaac_python tools/measure_pallet_fork_geometry.py --/log/level=error
```

배치나 에셋을 바꿀 때 다시 확인할 값:

- `fork_tip_offset`: 지게차 기준점에서 포크 끝까지 거리
- `fork_center_z_at_zero`: `lift_joint=0`일 때 포크 중심 월드 Z(현재 GPU 실측 `0.2062165`)
- `insertion_depth`: 팔레트 안으로 실제 삽입할 깊이
- `rack_fork_insert_travel`: 팔레트 정면 대기점부터 포크 삽입점까지 직진 거리(기본 0.95m)
- `wait_pose`: 지게차 대기 위치
- `amr_hole_center`: AMR 도킹 시 팔레트 구멍 중심

Warehouse 단독 시험도 실제 pose/joint-state 연결을 확인한 뒤 자동 시작하며 실제
GUI 자세를 경로 제어에 사용한다. U턴 단계 제한시간은 60초다.
