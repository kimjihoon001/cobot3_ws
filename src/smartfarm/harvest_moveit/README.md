# MM Nav2 → MoveIt 수확 데모

Isaac Sim에서 MM과 Nav 브리지를 먼저 실행한다.

```bash
cd /home/rokey/cobot3_ws
export ROS_DOMAIN_ID=108
isaac_python isaacpjt/main.py --mm --nav
```

다른 터미널에서 저장 지도 위 수확 지점을 `map` 좌표로 지정한다. `goal_yaw`는
라디안이며, 수확 성공 후 토마토를 마찰로 파지한 채 시작 위치로 돌아온다.

```bash
cd /home/rokey/cobot3_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=108
ros2 launch harvest_moveit nav_harvest_demo.launch.py \
  goal_x:=1.0 goal_y:=0.0 goal_yaw:=0.0
```

다른 지도를 쓸 때는 `map:=/absolute/path/to/map.yaml`을 추가한다. 주행만 확인하려면
`return_to_start:=false`로 복귀를 끌 수 있다. 이 데모는 `/cmd_vel`을 직접 발행하는
텔레옵, 기존 `nav_harvest_test.launch.py`, 별도의 MoveIt launch와 동시에 실행하지 않는다.

진행 순서는 다음과 같다.

1. 현재 `map → base_link` TF를 시작 위치로 저장
2. Nav2 `NavigateToPose`로 지정 위치 이동
3. `/harvester_moveit/sim/tomato`에서 도착 위치의 토마토 확인
4. OMPL 접근, 접촉 폭 유지, 커터 절단, 마찰 파지
5. 팔을 HOME으로 접고 Nav2로 시작 위치 복귀

## Launch와 실행 노드

이 패키지는 UR10e 시절의 레거시 데모다. 현행 m0617 시연은 `mm_moveit`과
`smartfarm_bringup/mm.launch.py`를 사용한다.

| 진입점 | 역할 |
|---|---|
| `moveit_isaac.launch.py` | UR10e MoveIt·ros2_control·RViz 기본 스택 |
| `nav_harvest_demo.launch.py` | Nav2 이동 후 비전 수확 데모. `nav_harvest_demo.py`와 `grasp_proto.py` 실행 |
| `full_pipeline_moveit.launch.py` | UR10e 수확 데모에 IW 미션과 지게차 반복 교환까지 포함한 구 통합 launch |

`grasp_proto.py`는 OMPL/Pilz 기반 원샷 수확 시퀀스를, `nav_harvest_demo.py`는
Nav2 목표 도착과 수확·복귀를 조정한다. 세 launch를 현행 `smartfarm_bringup`
launch와 동시에 실행하지 않는다.

```bash
# 팔·컨트롤러만 확인
ros2 launch harvest_moveit moveit_isaac.launch.py

# 구 전체 통합 경로 확인
ros2 launch harvest_moveit full_pipeline_moveit.launch.py
```

## 개발용 스크립트

`scripts/`에는 현행 launch에 설치되는 `grasp_proto.py`, `nav_harvest_demo.py`
외에도 초기 동작을 분리 검증했던 개발 도구가 남아 있다.

| 파일 | 용도 |
|---|---|
| `align_to_fruit.py` | 과실 방향 정렬 실험 |
| `creep_to_fruit.py` | 저속 접근 실험 |
| `drive_to_row.py` | 수확 열 접근 주행 실험 |
| `grasp_debug.py` | grasp pose·IK 디버깅 |
| `grasp_sweep.py` | grasp 후보 sweep 시험 |
| `harvest_key.py` | 키 입력으로 원샷 수확 트리거 |
| `snap_cam.py` | 카메라 프레임 캡처 보조 |

이 파일들은 패키지 실행 진입점으로 설치되지 않은 개발용 프로토타입이다. 현행
시연 절차로 사용하지 않으며, 실행 전 소스의 전제와 토픽 네임스페이스를 확인한다.
