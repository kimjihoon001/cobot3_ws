# forklift_control

Isaac Sim의 Forklift C를 ROS 2 토픽과 터미널 키보드로 조종하는 패키지다.

## 구성

- `forklift_control/teleop_node.py`: 터미널 키보드 ROS 2 노드
- `isaac_sim/forklift_sim.py`: Isaac Sim standalone 지게차 시뮬레이터

사용 토픽:

- `/forklift/cmd_vel` (`geometry_msgs/msg/Twist`)
- `/forklift/lift` (`std_msgs/msg/Float32`)

## 빌드

```bash
cd /home/rokey/cobot3_ws
source /opt/ros/humble/setup.bash
# 사용자 영역 setuptools와 Ubuntu 시스템 패키지의 버전 충돌을 피하고,
# 이 패키지만 탐색/빌드한다.
PYTHONNOUSERSITE=1 colcon build \
  --base-paths src/m0609/minseongkim/forklift_control \
  --packages-select forklift_control
source install/setup.bash
```

## 터미널 1: Isaac Sim 실행

Isaac Sim과 시스템 ROS의 Python 버전이 다르므로 시뮬레이터는
`ros2 run`이 아니라 Isaac Sim의 `python.sh`로 실행한다.

```bash
source ~/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release/setup_ros_env.sh
export ROS_DOMAIN_ID=108

isaac_python \
  /home/rokey/cobot3_ws/src/m0609/minseongkim/forklift_control/isaac_sim/forklift_sim.py
```

## 터미널 2: Teleop 실행

```bash
source /opt/ros/humble/setup.bash
source /home/rokey/cobot3_ws/install/setup.bash
export ROS_DOMAIN_ID=108
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

ros2 run forklift_control forklift_teleop
```

## 조작

- `W/S`: 누르는 동안 전진/후진
- `A/D`: 누르는 동안 좌/우 조향
- `I/K`: 누르는 동안 포크 상승/하강
- `X`: 조향 중앙
- `Space`: 주행 정지
- `Q` 또는 `Ctrl+C`: 안전 정지 후 종료
