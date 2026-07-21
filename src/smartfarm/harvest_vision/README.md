# harvest_vision

ROS 2 Humble와 Jazzy를 지원한다. `manipulator_target_node`는 배포판마다 세부
helper 함수가 다른 `do_transform_pose*`를 직접 호출하지 않고, 두 배포판의 공통
`tf2_ros.Buffer.transform()` API로 `PoseStamped`를 변환한다.

## 빌드

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --packages-select smartfarm_interfaces harvest_vision --symlink-install
source install/setup.bash
```

## 좌표 브리지 dry-run

```bash
ros2 run harvest_vision manipulator_target_node --ros-args \
  --params-file src/smartfarm/harvest_vision/config/manipulator_target.yaml
```

기본 설정은 `command_enabled: false`다. 카메라 프레임에서
`harvester_0/base_link`로 이어지는 TF와 변환된 좌표를 확인하기 전에는 활성화하지
않는다.

Isaac Sim 쪽은 `--mm --rmpflow`로 실행한다. 검증된 목표가
`/harvester_0/cmd`의 `rmp_target` JSON으로 전달되며, Isaac의 공식 UR10e
RMPflow 설정이 이를 관절 action으로 변환한다. `command_enabled`가 false이면
RMPflow 명령은 발행되지 않는다.

## 근거리 품질 판정 상태 디버깅

```bash
ros2 topic echo /vision/target_class
ros2 topic echo /harvester_0/manipulator/target_state
ros2 topic echo /harvester_0/manipulator/validated_target
ros2 topic echo /harvester_0/rmpflow/status
ros2 topic echo /harvester_0/manipulator/mobility_ready
```

정상 상태 전이는 `NO_TARGET -> APPROACH -> QUALITY_CHECK -> RIPE_READY ->
PREGRASP -> GRASP -> GRIPPER_CLOSING -> CUTTING -> RETRACT -> WAIT_BASKET ->
BASKET_APPROACH -> BASKET_PLACE -> PLACE_RELEASING -> GO_HOME -> HOME_READY`이며, 불량이면
`SKIP_SPOILED`로 끝난다. 0.5m에서 `QUALITY_CHECK`가 되는 즉시 Isaac에
`rmp_stop`을 보내고 정지 상태에서 5프레임 품질 투표를 완료한다. 거리 노이즈로
접근/정지가 반복되지 않도록 0.6m를 넘어야 원거리 접근으로 복귀한다. 실제 파지
시퀀스는 `RIPE_READY`에서만 시작한다.

`PREGRASP`는 카메라→토마토 광선의 반대 방향으로 tool reach 0.115 m와 여유
0.15 m만큼 떨어진 목표이며, 도달 피드백 후 `GRASP` 목표로 전진한다. 각 동작의
기본 제한 시간은 10초다. 파지 도중 표적이 사라지거나 `spoiled`로 바뀌면 각각
`ABORT_TARGET_LOST`, `ABORT_SPOILED`로 정지하며 제한 시간을 넘으면
`ERROR_TIMEOUT`으로 정지한다. 그리퍼가 닫힘 위치에 도달하면 커터 날을 35도로
닫고 0.6초 뒤 비전 목표에 가장 가까운 ripe 과실의 pedicel `FixedJoint`를
`jointEnabled=False`로 전환한다. 절단 성공 피드백 후 날을 열고 `PREGRASP`
위치로 후퇴한다. 목표와 과실 ground truth가 0.10 m보다 멀거나 날이 닫히지 않았으면
`ERROR_CUT`으로 정지한다. 현재 파지 확인은 접촉 센서가 아니라 그리퍼 관절 위치와
물리 마찰에 의존한다.

## IW 바스켓 연동과 이동 인터록

IW 측 빈 슬롯 선택기는 `/iw/basket/empty_slot_pose`에 `PoseStamped`를 발행한다.
이 pose는 **바스켓 중심이 아니라 tool0가 토마토를 놓을 release pose**이며 frame_id는
IW 바스켓 TF 프레임이나 map/base 계열 어느 것이어도 된다. 노드가
`harvester_0/base_link`로 변환한 뒤 0.15 m 위로 접근하고, release pose까지 내려가
그리퍼를 연다. IW가 아직 없거나 pose가 없으면 토마토를 든 채 `WAIT_BASKET`에서
정지한다.

배치가 끝나면 RMPflow의 초기 관절 자세로 복귀하고, 관절 최대 오차가 0.03 rad
이하일 때만 `/harvester_0/manipulator/mobility_ready=true`와 `HOME_READY`를 발행한다.
Isaac의 JSON `base` 명령도 홈이 아니면 차단한다. Nav2/외부 이동 노드는 같은
`mobility_ready` 인터록을 구독해 true일 때만 주행 목표를 실행해야 한다.

## IW 없는 Nav2→수확 통합시험

Isaac은 MM Nav 브리지와 RMPflow를 함께 켠다.

```bash
isaac_python isaacpjt/basic/jihoonkim/isaacpjt/main.py \
  --mm --nav --rmpflow
```

Nav2를 실행한 뒤 단독 시험 노드를 띄운다.

```bash
ros2 launch fleet_dispatch harvester_nav2.launch.py \
  slam:=false map:=/home/rokey/cobot3_ws/maps/farm.yaml use_sim_time:=true

ros2 launch harvest_vision nav_harvest_test.launch.py
ros2 topic echo /harvest_test/status
```

RViz에서 `Nav2 Goal`을 찍으면 시험 노드가 새 NavigateToPose goal ID를 추적한다.
주행 중에는 `/harvest_test/enable=false`라 수확을 시작하지 않고, goal이
`SUCCEEDED`가 된 뒤에만 비전 수확을 허용한다. 근방에서 ripe 토마토가 검출되면
정상 수확 시퀀스를 실행한다. 30초 동안 검출되지 않으면
`ERROR_TOMATO_SEARCH_TIMEOUT`으로 다시 비활성화한다.

IW가 없으므로 수확 후 `WAIT_BASKET`에서 시험 노드가
`harvester_0/base_link` 기준 `[0.45, -0.35, 0.45]`의 모의 빈 슬롯 release pose를
발행한다. 실제 로봇 형상에 맞지 않으면 `config/nav_harvest_test.yaml`의
`mock_basket_release_xyz`를 조정한다. 배치와 홈 복귀가 완료되면 시험 상태는
`CYCLE_COMPLETE_HOME_READY`가 된다. 이 시험 중에는 `target_approach_node`나 MM
teleop을 동시에 실행하지 않는다.
