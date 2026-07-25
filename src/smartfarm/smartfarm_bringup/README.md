# smartfarm_bringup

기존 일체형 `harvest_vision/smartfarm_integration.launch.py`는 유지한다.
분산 실행 시에는 각 프로세스를 별도 터미널 또는 호스트에서 실행한다.

```bash
# GPU/Isaac 호스트
ros2 launch smartfarm_bringup isaac_sim.launch.py

# ROS 호스트(각각 독립 실행 가능)
ros2 launch smartfarm_bringup mm.launch.py
ros2 launch smartfarm_bringup iw.launch.py
ros2 launch smartfarm_bringup forklift.launch.py
```

모든 호스트는 같은 `ROS_DOMAIN_ID`, RMW 구현, 네트워크 DDS 설정을 사용해야 한다.
Isaac 설치 경로나 워크스페이스 경로가 다르면 `isaac_python:=...`,
`isaac_main:=...`, `isaac_working_directory:=...`를 지정한다.
