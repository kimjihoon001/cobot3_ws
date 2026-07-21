# Harvester MM 비전·이동·매니퓰레이터 연동 기록

Harvester MM(AMR + UR10e + 그리퍼 + 커터 + D455) 관련 변경 사항을 누적 기록한다.
MM 관련 코드가 변경될 때 이 문서의 변경 이력과 현재 인터페이스를 함께 갱신한다.

## 시스템 역할 분리

```text
Nav2
  └─ 온실 섹터 및 토마토 행까지 장거리 이동

YOLO + Visual Approach
  ├─ 원거리 tomato 검출
  ├─ 선택 목표의 카메라 3D 좌표 계산
  └─ 토마토 전방 0.5 m까지 저속 최종 접근

근거리 품질 모델
  └─ ripe / spoiled 다수결 판정

매니퓰레이터
  ├─ 관찰 높이 조정
  ├─ 목표 위치로 IK 이동
  ├─ 그리퍼 파지
  └─ 커터 절단 및 트레이 배치
```

Nav2와 visual approach가 동시에 `/cmd_vel`을 발행하면 안 된다. Nav2 목표가 완료되고
정지한 후 visual approach를 활성화한다.

## HOME 및 관찰 자세

`isaacpjt/main.py --mm` 스폰 직후 HOME 자세를 기본 대기·이동·관찰 시작 자세로 사용한다.

| 관절 | 각도 |
|---|---:|
| `shoulder_pan_joint` | 0° |
| `shoulder_lift_joint` | 225° |
| `elbow_joint` | 135° |
| `wrist_1_joint` | 180° |
| `wrist_2_joint` | -90° |
| `wrist_3_joint` | 0° |

HOME은 고정 자세가 아니다. 토마토 높이에 따라 매니퓰레이터의 Z 높이를 조절해서
관찰할 수 있다. 이때 카메라 광축을 베이스 전방으로 유지해야 현재 visual approach의
카메라 좌표 기반 조향을 그대로 사용할 수 있다. 카메라가 기울어지는 동작을 허용하려면
`camera frame -> base_link` TF 변환을 추가해야 한다.

`target_approach_node`의 `require_home_pose` 기본값은 `false`다. 엄격한 HOME 자세
검사가 필요한 시험에서만 `true`로 설정한다.

## 모델

| 용도 | 기본 파일 | 클래스 |
|---|---|---|
| 원거리 탐지 | `finetuned_far.pt` | `tomato` |
| 근거리 품질 | `finetuned_near.pt` | `ripe`, `spoiled` |

두 모델 모두 `src/smartfarm/harvest_vision/resource/`에 배치되어 있다. 모델 클래스
검증 결과는 다음과 같다.

```text
finetuned_far.pt  -> {0: tomato}
finetuned_near.pt -> {0: ripe, 1: spoiled}
```

### 원거리 모델의 Isaac Sim 도메인 적응

실사 토마토 데이터만 학습한 원거리 모델은 Isaac Sim의 조명·재질·렌더링 차이로 검출
성능이 떨어질 수 있다. `yolo_finetune_far_scene.py`는 원본 `scene_yolo`를 수정하지 않고
`ripe`와 `spoiled`를 모두 `tomato=0`으로 합친 파생 데이터셋을 만든 뒤 기존
`finetuned_far.pt`를 낮은 학습률로 파인튜닝한다.

```bash
cd /home/rokey/cobot3_ws

# 파생 데이터만 확인
python3 yolo_training/yolo_finetune_far_scene.py --prepare-only

# 기본 25 epoch 시뮬 도메인 적응
python3 yolo_training/yolo_finetune_far_scene.py --device 0
```

파생 데이터는 `yolo_training/processed/scene_tomato_detection`에 생성된다. 이미지는
복제하지 않고 원본을 가리키는 심볼릭 링크를 사용하며, 라벨만 1클래스로 변환한다.
학습 결과는 `yolo_training/runs/far_scene_finetune_<시간>/weights/best.pt`에 저장된다.

시뮬 파인튜닝 모델로 리소스를 교체하기 전에 기존 실사 검증셋과 시뮬 검증셋을 모두
평가해야 한다. 시뮬 데이터만 과도하게 학습하면 실사 성능을 잊을 수 있다.

원거리 학습 결과:

```text
yolo_training/runs/tomato_detector_20260721_120648/weights/best.pt
```

## ROS 토픽

### 입력

| 토픽 | 타입 | 설명 |
|---|---|---|
| `/harvester/rgb` | `sensor_msgs/Image` | D455 RGB |
| `/harvester/depth` | `sensor_msgs/Image` | D455 depth |
| `/harvester/camera_info` | `sensor_msgs/CameraInfo` | 카메라 내부 파라미터 |
| `/harvester_0/joint_states` | `sensor_msgs/JointState` | MM 관절 상태 |

### 비전 출력

| 토픽 | 타입 | 설명 |
|---|---|---|
| `/vision/tomato_detections` | `TomatoDetectionArray` | 전체 검출 결과 |
| `/vision/annotated_image` | `sensor_msgs/Image` | YOLO 박스가 표시된 RGB |
| `/vision/approach_target` | `geometry_msgs/PoseStamped` | 선택 목표의 카메라 좌표 |

### 제어 출력

| 토픽 | 타입 | 설명 |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | Nav2 또는 visual approach 베이스 속도 |
| `/harvester_0/joint_command` | `sensor_msgs/JointState` | UR10e 및 그리퍼 명령 |
| `/harvester_0/cmd` | `std_msgs/String` | 커터 및 베이스 JSON 명령 |

## 실행

```bash
# Isaac Sim: MM, 카메라, Nav2 브리지
isaac_python isaacpjt/main.py --mm --nav

# 2단계 YOLO
ros2 run harvest_vision vision_node

# YOLO + depth 확인
ros2 run harvest_vision vision_debug_view

# 최종 접근 노드(기본 비활성)
ros2 run harvest_vision target_approach_node

# Nav2 정지 후 활성화
ros2 param set /target_approach_node enabled true

# 수확 접근 완료 또는 이상 발생 시 비활성화
ros2 param set /target_approach_node enabled false
```

## 텔레옵 주행 검출 시험

자동 접근을 끈 상태에서 MM을 텔레옵으로 이동하며 원거리 검출, 거리 전환, 근거리 품질
판정을 확인한다. 모든 터미널은 같은 `ROS_DOMAIN_ID=108`을 사용한다.

```bash
# 터미널 1: Isaac Sim MM + 손끝 D455 + 키보드 텔레옵
cd /home/rokey/cobot3_ws
ROS_DOMAIN_ID=108 isaac_python isaacpjt/main.py --mm --mm-teleop

# 터미널 2: YOLO 2단계 비전
source /opt/ros/humble/setup.bash
source /home/rokey/cobot3_ws/install/setup.bash
export ROS_DOMAIN_ID=108
ros2 run harvest_vision vision_node

# 터미널 3: YOLO 결과 + depth 창
source /opt/ros/humble/setup.bash
source /home/rokey/cobot3_ws/install/setup.bash
export ROS_DOMAIN_ID=108
ros2 run harvest_vision vision_debug_view
```

Isaac 창에 키보드 포커스를 둔 다음 `I/K` 전후, `J/L` 제자리 회전을 사용한다.
MM은 게걸음을 지원하지 않는다. 옆 방향으로 가려면 먼저 회전하고 전진한다.
`--mm-teleop`과 `--nav`/`--nav-drive`는 동시에 사용할 수 없도록 시작 단계에서 차단한다.
이번 검출 시험에서는 `target_approach_node`를 실행하지 않거나 `enabled=false`로 유지한다.

확인 순서:

1. 먼 거리에서 토마토 박스가 `tomato`로 안정적으로 유지되는지 확인한다.
2. 좌우 회전과 전후 이동 중 박스가 끊기거나 배경에 생기는지 확인한다.
3. 박스의 거리 표기가 실제 접근 방향과 함께 감소하는지 확인한다.
4. 기본 `near_distance_m=0.5` 안으로 들어가 최근 5프레임 이후 목표가 `ripe` 또는
   `spoiled`로 바뀌는지 확인한다.
5. 토마토가 없는 방향에서 `NO TOMATO`가 표시되는지 확인한다.

시험 중 기록할 항목은 원거리 최초 검출 거리, 미검출 구간, 오검출 대상, 근거리 클래스,
표시 거리, 조명·가림·카메라 높이다.

## 현재 구현 상태

- [x] 원거리 `tomato` 검출
- [x] 근거리 `ripe/spoiled` 판정
- [x] RGB-D 기반 카메라 3D 좌표 계산
- [x] YOLO 디버그 영상 발행 및 표시
- [x] 목표 유실 시 정지하는 저속 visual approach
- [x] HOME 자세를 선택적으로 검사하는 안전 조건
- [ ] 카메라 좌표를 `base_link`로 변환하는 TF 처리
- [ ] Nav2 완료와 visual approach 활성화의 자동 상호 배제
- [ ] 관찰 Z 높이 자동 스캔
- [ ] 목표 3D 위치 기반 UR10e IK
- [ ] 충돌 회피 팔 경로
- [ ] 그리퍼·커터 Pick & Place 시퀀스

## 저장 맵 기반 Nav2 시험

저장 맵은 `/home/rokey/cobot3_ws/maps/farm.yaml`이며 312×687 셀, 해상도 0.05 m다.
`farm.pgm`과 YAML의 상대 이미지 경로가 정상이고 Nav2 패키지도 설치되어 있다.

```bash
# 설정을 수정한 뒤 최초 1회(또는 fleet_dispatch 설정 변경 후)
cd /home/rokey/cobot3_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select fleet_dispatch

# 터미널 1: Isaac MM + Nav2 브리지. 텔레옵과 함께 실행하지 않는다.
cd /home/rokey/cobot3_ws
export ROS_DOMAIN_ID=108
isaac_python isaacpjt/main.py --mm --nav

# 터미널 2: 저장 맵 + AMCL + Nav2 + RViz
source /opt/ros/humble/setup.bash
source /home/rokey/cobot3_ws/install/setup.bash
export ROS_DOMAIN_ID=108
ros2 launch fleet_dispatch harvester_nav2.launch.py \
  slam:=false \
  map:=/home/rokey/cobot3_ws/maps/farm.yaml \
  use_sim_time:=true
```

`harvester_nav2.launch.py`는 기본값 `rviz:=true`로 전용 RViz 설정을 함께
실행하므로 `rviz2`를 별도로 띄우지 않는다. RViz 없이 실행할 때만
마지막에 `rviz:=false`를 추가한다.

현재 MM 회전 상한은 `1.3 rad/s`(약 75°/s), 회전 가·감속 절대값은
`2.5 rad/s²`다. Nav2의 Rotation Shim·DWB·Velocity Smoother와 Isaac의
`HarvesterNavConfig.max_wz`가 같은 상한을 사용한다. YAML 수정 후에는
`fleet_dispatch`를 다시 빌드하고 Nav2를 재시작해야 하며, `settings.py` 수정
후에는 Isaac Sim도 재시작해야 한다.

정적 맵 모드에서는 launch가 기본 초기 위치 `(x=0, y=0, yaw=0)`를 AMCL에 자동
적용한다. 저장 맵의 시작점과 다른 위치에서 spawn할 때는 다음처럼 덮어쓴다.

```bash
ros2 launch fleet_dispatch harvester_nav2.launch.py \
  slam:=false map:=/home/rokey/cobot3_ws/maps/farm.yaml \
  initial_pose_x:=0.11 initial_pose_y:=-0.11 initial_pose_yaw:=0.0
```

수동 `2D Pose Estimate`를 사용하려면 `set_initial_pose:=false`를 지정한다.

RViz에서 다음 순서로 확인한다.

1. `Map`이 정상 표시되는지 확인한다.
2. `LaserScan`이 지도 벽과 대략 겹치는지 확인한다.
3. `2D Pose Estimate`로 SLAM 맵을 만들 때의 시작 위치와 방향을 지정한다.
4. `map -> odom -> base_link -> laser` TF가 연결됐는지 확인한다.
5. 가까운 자유 공간에 `Nav2 Goal`을 보내 회전 후 전진하는지 확인한다.
6. `/cmd_vel`의 `linear.y`가 계속 0인지 확인한다.

기본 RViz 설정은 `map` 고정 프레임에서 다음 항목을 자동 표시한다.

- TF의 `base_link` 좌표축/화살표: 빨간 X축이 MM 전방
- `/local_costmap/published_footprint`: 실제 주행 충돌 외곽선
- `/global_costmap/costmap`: 전체 경로 계획용 코스트맵(옅게 표시)
- `/local_costmap/costmap`: 근거리 장애물 회피용 코스트맵
- `/plan`, `/local_plan`: 전역 경로(빨강), 로컬 경로(파랑)
- `/particle_cloud`: AMCL 위치 추정 분포

코스트맵은 원본 지도 자체가 아니라 장애물과 로봇 크기/안전 여유를 반영한 주행 비용
레이어다. 검정/진한 영역은 장애물 또는 높은 비용, 퍼지는 띠는 inflation 안전 여유다.
기본 시점은 15.6 × 34.35 m 저장 맵 전체와 오버레이를 보기 쉬운 Top-Down으로 설정했다.
`Map`은 바닥, Global/Local Costmap은 각각 alpha 0.25/0.55의 반투명 레이어다.

### 저장 맵과 실시간 LaserScan 정렬 불일치

1. Nav2 Goal을 취소하고 MM을 정지한다.
2. RViz Fixed Frame이 `map`인지 확인한다.
3. `2D Pose Estimate`로 실제 지도상의 위치와 **방향**을 다시 지정한다.
4. 제자리에서 천천히 360도 회전해 AMCL particle cloud가 수렴하는지 확인한다.

맵의 모든 벽이 같은 거리/각도로 평행하게 어긋나면 초기 pose 문제다. 위치마다 오차가
달라지거나 스캔 모양 자체가 회전·왜곡되면 `base_link -> laser` 정적 TF, LaserScan
frame_id, `/clock` 중복/역행 또는 SLAM 맵 품질 문제를 점검한다. 맵 YAML의 `origin`은
초기 위치 보정용 값이 아니므로 임의로 수정하지 않는다. 정렬되지 않은 상태에서는
코스트맵과 경로 계획도 잘못되므로 자율주행을 계속하지 않는다.

```bash
ros2 topic echo /scan --once --field header
ros2 run tf2_ros tf2_echo base_link laser
ros2 topic info /clock --verbose   # publisher 1개
ros2 run tf2_ros tf2_echo map base_link
```

```bash
ros2 topic hz /clock
ros2 topic hz /scan
ros2 topic hz /odom
ros2 run tf2_ros tf2_echo map base_link
ros2 topic echo /cmd_vel
ros2 action list | grep navigate_to_pose
```

AMCL 초기 위치를 지정하기 전에는 목표를 보내지 않는다. Nav2 시험 중에는
`--mm-teleop`과 `target_approach_node enabled=true`를 사용하지 않는다.

### `map` 프레임이 없다는 오류

2026-07-21 실행 로그에서 map server와 Nav2 lifecycle 활성화는 정상 완료됐지만,
AMCL이 다음 경고를 반복했다.

```text
AMCL cannot publish a pose or update the transform. Please set the initial pose...
```

따라서 `Invalid frame ID "map"`은 프레임 이름 설정 문제가 아니라, 초기 위치를
받지 못한 AMCL이 `map -> odom` TF를 아직 발행하지 않은 결과다. RViz의
`2D Pose Estimate`로 지도상 로봇 위치와 방향을 지정한다. 지정 후 아래 항목이
응답하는지 확인한 다음 Nav2 Goal을 보낸다.

RViz의 `Global Options > Fixed Frame`은 반드시 `map`이어야 한다. `odom`인 상태에서
초기 위치를 지정하면 로그에 `Setting estimate pose: Frame:odom`이 찍히며, AMCL의
전역 초기 위치로 적용되지 않아 `map -> odom`이 생성되지 않는다.

아직 `map` TF가 없어 RViz를 `map`으로 바꾸면 LaserScan이 사라지는 순환 상태에서는
RViz 대신 `/set_initial_pose` 서비스를 이용해 `frame_id: map`인 초기 포즈를 먼저
AMCL에 전달한다. 서비스 응답 뒤 `/amcl_pose`와 `map -> base_link`가 출력되면 RViz의
Fixed Frame을 `map`으로 바꾼다. 초기화 직후 1~2초 동안 TF tree가 분리됐다는 메시지가
나올 수 있으므로 최종적으로 좌표가 연속 출력되는지를 기준으로 판단한다.

```bash
ros2 topic echo /amcl_pose --once
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo map base_link
```

초기화 직후 한두 번 발생하는 `laser`의 오래된 timestamp 메시지 drop은 시작 순서에
따른 현상일 수 있다. 초기 위치 지정 후에도 계속 반복될 때만 `/clock`, `/scan`, TF의
시간 동기화를 별도로 점검한다.

2026-07-21 추가 진단에서는 `/clock` publisher가 2개 발견됐고, 두 publisher 모두
`_World_RosClock_Clock`이라는 Isaac 노드였다. 서로 다른 시뮬레이션 시간이 섞이면서
`Detected jump back in time. Clearing TF buffer.`가 매 프레임 반복됐으며, 이 경우에는
초기 위치를 지정해도 AMCL이 `map -> odom`을 유지할 수 없다. 같은
`ROS_DOMAIN_ID=108`에서 실행 중인 중복 Isaac/Stage 인스턴스를 종료하고 `/clock`
publisher가 정확히 1개인지 확인한 뒤 Nav2를 재시작한다.

```bash
ros2 topic info /clock --verbose
# 반드시 Publisher count: 1
```

## 변경 이력

### 2026-07-21

- MM 최대 회전 속도를 `1.0`에서 `1.3 rad/s`로, 회전 가·감속 절대값을
  `2.0`에서 `2.5 rad/s²`로 높였다. Nav2와 Isaac 최종 클램프 값을 동일하게
  맞춰 설정 단계에서 속도가 다시 제한되지 않게 했다.
- Nav2 실행 절차에 `fleet_dispatch` 재빌드 명령과 RViz 자동 실행/비활성화
  방법을 명시했다.
- `vision_node.py`를 원거리 탐지/근거리 품질의 2단계 구조로 변경했다.
- `finetuned_far.pt`, `finetuned_near.pt`를 기본 모델명으로 지정했다.
- YOLO 오버레이 영상과 접근 목표 Pose 발행을 추가했다.
- `vision_debug_view.py`에 YOLO 박스·클래스·신뢰도·거리 표시를 연결했다.
- `target_approach_node.py`를 추가했다.
- HOME을 기본 관찰 자세로 정의하되, Z 높이 조절을 허용하도록 엄격한 자세 검사는
  기본 해제했다.
- 학습 완료된 원거리 1클래스 모델 `finetuned_far.pt`를 리소스에 배치하고 클래스가
  `{0: tomato}`인지 확인했다.
- MM 텔레옵 주행 중 2단계 YOLO를 검증하는 실행 순서와 판정 항목을 추가했다.
- Isaac Sim RGB 추론 후 `/vision/annotated_image` 발행 과정에서 Humble `cv_bridge`가
  `CV_8UC3` 키를 찾지 못하는 오류(`KeyError: 16`)를 확인했다. 출력 `bgr8` 메시지를
  직접 구성하도록 변경해 `cv_bridge.cv2_to_imgmsg()` 의존을 제거했다.
- 전역 `--teleop` 플래그를 제거하고 MM 전용 `--mm-teleop`으로 분리했다. `--mm`과
  `--iw`를 함께 스폰해도 키보드 텔레옵은 MM에만 연결된다.
- MM 키보드 텔레옵과 독립 실행 텔레옵에서 횡이동을 제거했다. 전진은 현재 yaw 기준으로
  계산하며 `J/L`로 제자리 회전한 뒤 `I/K`로 이동한다. Nav2의 `vy=0` 정책과 같다.
- 사용자 조작 통일을 위해 MM 회전 키를 기존 `U/O`에서 `J/L`로 변경했다.
- `scene_yolo`의 `ripe/spoiled`를 `tomato` 한 클래스로 합쳐 원거리 모델을 시뮬레이션
  도메인에 적응시키는 `yolo_finetune_far_scene.py`를 추가했다.
- 저장된 `maps/farm.yaml`로 MM Nav2를 검증하는 절차를 추가했다. 게걸음 금지 정책에
  맞춰 AMCL을 DifferentialMotionModel로 바꾸고 DWB의 유효한 `vy_samples=5`와
  `min/max_vel_y=0` 조합으로 수정했다. Nav2/Isaac 토픽 주석도 실제 전역 토픽에 맞췄다.
- Nav2 로그의 `map` 프레임 부재 원인이 AMCL 초기 위치 미설정임을 확인하고,
  RViz `2D Pose Estimate`를 이용한 복구 및 TF 검증 절차를 추가했다.
- 후속 실행에서 Isaac `/clock` publisher 2개로 인해 시뮬레이션 시간이 계속 역행하고
  TF 버퍼가 삭제되는 문제를 확인했다. 중복 Isaac 인스턴스 종료 및 단일 clock 확인
  절차를 추가했다.
- RViz가 초기 포즈를 `odom` 프레임으로 발행한 실행을 확인했다. Fixed Frame을
  `map`으로 지정한 뒤 초기 포즈를 다시 보내도록 진단 절차를 보완했다.
- `harvester_nav2.launch.py`에 저장 맵용 AMCL 자동 초기화와
  `initial_pose_x/y/yaw`, `set_initial_pose` 인자를 추가했다.
- RViz 기본 설정에 MM 전방을 보여주는 TF 축/화살표, footprint, 전역·로컬 코스트맵,
  전역·로컬 계획 경로와 AMCL particle cloud 표시를 추가했다.
