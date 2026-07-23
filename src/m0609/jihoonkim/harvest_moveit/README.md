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
