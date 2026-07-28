# cobot3_ws — 스마트팜 토마토 수확·운반·하역 자동화

ROKEY 협동3기 A-2 팀 프로젝트 워크스페이스입니다.
Isaac Sim 위에 온실·창고 씬을 세우고, **로봇 3대가 토마토를 수확해 창고 랙에
적재하기까지의 전 과정**을 ROS 2로 자동화합니다.

ROS 2 패키지(`src/`)와 Isaac Sim 스크립트(`isaacpjt/`)를 하나의 저장소로 관리합니다.

- ROS 2: Humble (`/opt/ros/humble`)
- 통신: `ROS_DOMAIN_ID=108`, `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`
- 원격 저장소: <https://github.com/kimjihoon001/cobot3_ws.git>

---

## 1. 시스템 개요

로봇 3대가 각자 독립 프로세스로 돌고, 토픽/서비스로 서로 핸드셰이크합니다.

| 로봇 | 네임스페이스 | 구성 | 역할 |
|---|---|---|---|
| **MM** (수확 모바일 매니퓰레이터) | `harvester_0` | Ridgeback 베이스 + Doosan M0617 6축 + 스쿱/커터 + D455(eye-in-hand) | 토마토 검출·수확 후 IW 데크의 빈 KLT에 적재 |
| **IW** (운반 AMR) | `iwhub_0` | 차동구동 + 승강 데크 | 수확 지점에서 MM을 따라다니다 적재 완료 시 창고 하역장으로 이동 |
| **지게차** | `forklift_0` | 승강 포크 + 후륜 조향 | IW의 만재 팔레트를 받아 창고 랙에 넣고 빈 팔레트를 되돌려줌 |

### 동작 흐름

```text
MM Nav2로 수확 위치 이동 → HOME → BED_VIEW
  → D455 + YOLO 토마토 검출 → 접근·파지·절단
  → IW FOLLOW 호출 → IW가 빈 KLT pose 제공 → 데크에 적재
  → 목표 개수(place_target_count) 충족 → /iw/mission = PREPARE_FORKLIFT
  → IW가 피항 요청 → MM이 통로 밖으로 비켜줌 → IW 하역장 이동
  → 지게차 사이클(팔레트 인계 → 랙 적재 → 빈 팔레트 반환)
  → IW 복귀 → 수확 재개
```

> **수확 횟수**는 `harvest_fsm_node`의 `place_target_count` 파라미터로 정합니다.
> `real_main`은 **1**(1개 적재 후 바로 하역)이고, 다회 수확은 `multiple_harvest`
> 브랜치에서 2로 검증 중입니다. IW 데크 앞열 빈 KLT가 2칸이라 최대 2까지 의미가 있습니다.

---

## 2. 실행 방법

> 새 PC라면 [5. 설치 및 빌드](#5-설치-및-빌드)를 먼저 끝내세요. `ros-humble-desktop`
> 만으로는 MoveIt·Nav2·UI 패키지가 없어 launch가 바로 실패합니다.

**터미널 4개**를 쓰고, 순서대로 띄웁니다. Isaac Sim이 먼저 떠 있어야 나머지가 붙습니다.

### ① Isaac Sim (GPU 호스트)

```bash
cd ~/cobot3_ws/isaacpjt
isaac_python main.py --mm --iw --fork --nav --camera

# 모니터링 UI(4분할 CCTV)를 같이 볼 때 — --cctv 추가
cd ~/cobot3_ws/isaacpjt
isaac_python main.py --mm --iw --fork --nav --camera --cctv
```

| 플래그 | 의미 |
|---|---|
| `--mm` / `--iw` / `--fork` | 각 로봇을 씬에 스폰 |
| `--nav` | Nav2용 `cmd_vel`·`odom`·`scan` 브리지를 한꺼번에 켬 (개별로는 `--nav-drive`/`--nav-odom`/`--nav-scan`) |
| `--camera` | **인식되지 않는 인자**. 손끝 D455는 원래 기본으로 켜지며, 끌 때만 `--no-camera`를 쓴다. 붙여도 무해해서 관례로 남아 있음 |
| `--cctv` | 모니터링 UI용 **고정 감시 카메라 3대**. 카메라마다 씬을 다시 렌더해서 기본은 꺼져 있음 — 4분할 화면을 쓰려면 이 플래그가 필요 |
| `--headless` / `--no-ros` | GUI 없이 / ROS 브리지 없이 실행 |

> `main.py`는 `ROS_DOMAIN_ID`를 **108로 강제**합니다. MM·IW·지게차·비전·UI
> 터미널도 모두 108로 맞춰 직접 통신하며, 별도의 Docker/domain bridge는
> 사용하지 않습니다.

### ② MM (수확)

```bash
ros2 launch smartfarm_bringup mm.launch.py \
  harvest_x:=-0.54 harvest_y:=-8.19 harvest_yaw:=1.91
```

`harvest_*`는 MM이 정차해 수확할 자세입니다(위 값이 기본값이라 생략해도 동일).
Nav2 + MoveIt + 비전 + 수확 FSM이 한 번에 뜹니다.

### ③ IW (운반)

```bash
ros2 launch smartfarm_bringup iw.launch.py
```

IW 전용 Nav2와 `mission_nav_node`(FOLLOW/PREPARE_FORKLIFT 미션 처리, 도킹 판정 후
지게차 사이클 서비스 호출)를 띄웁니다.

### ④ 지게차 (하역)

```bash
ros2 launch smartfarm_bringup forklift.launch.py
```

`fork_lift_return_node`가 팔레트 인계·랙 적재·빈 팔레트 반환 시퀀스를 수행합니다.

### (선택) 모니터링 UI

4분할 CCTV형 관제 화면입니다. **Isaac을 `--cctv`와 함께 띄워야** 고정 감시 카메라가
나옵니다. 자세한 사용법은 [`src/smartfarm/monitor_ui/README.md`](src/smartfarm/monitor_ui/README.md) 참고.

UI 전용 apt 패키지 4개(`web_video_server` 등)와 `npm install`이 먼저 필요합니다 —
[5.1](#51-apt-의존-패키지)·[5.3](#53-모니터링-ui-웹-node) 참고.

```bash
# 1) ROS 쪽 — 영상 스트리밍 + QoS 브리지 + 상태 집계 + 녹화 제어
ros2 launch monitor_ui ui.launch.py

# 2) 화면 — 별도 터미널
cd ~/cobot3_ws/src/smartfarm/monitor_ui/web && npm run dev   # 처음 한 번은 npm install
```

그다음 <http://localhost:5173>. `ui.launch.py`가 `stream.launch.py`를 포함하므로
둘을 같이 띄우면 안 된다(영상 노드가 이중으로 뜬다). 영상 배선만 따로 볼 때만
`stream.launch.py`를 단독으로 쓴다.

### 주요 launch 인자

| launch | 인자 | 기본값 |
|---|---|---|
| `mm.launch.py` | `harvest_x` / `harvest_y` / `harvest_yaw` | `-0.54` / `-8.19` / `1.91` |
| | `initial_pose_x` / `initial_pose_y` | `0.0` / `-12.0` |
| | `nav_rviz` / `moveit_rviz` | `true` |
| `iw.launch.py` | `iw_dock_standoff` | `1.03` |
| | `iw_rviz` | `true` |
| 공통 | `map` | `~/cobot3_ws/maps/farm.yaml` |
| | `use_sim_time` | `true` |

> 로봇을 여러 호스트에 나눠 띄울 수 있습니다. 이때 모든 호스트가 같은
> `ROS_DOMAIN_ID`·RMW·DDS 설정을 써야 합니다.

---

## 3. 패키지 구성

모든 ROS 2 패키지는 `src/smartfarm/` 아래에 있습니다.

| 패키지 | 담당 | 내용 |
|---|---|---|
| [`smartfarm_bringup`](src/smartfarm/smartfarm_bringup/README.md) | 공용 | **실행 진입점.** `isaac_sim` / `mm` / `iw` / `forklift` launch |
| [`harvest_vision`](src/smartfarm/harvest_vision/README.md) | 트랙 A | 수확 파이프라인. `vision_node`(YOLO 검출), `harvest_fsm_node`(상위 코디네이터: Nav2 게이트·적재 카운트·IW 미션·MM 피항), `manipulator_target_node`(파지/절단/플레이스 상세 FSM) |
| [`mm_moveit`](src/smartfarm/mm_moveit/README.md) | 트랙 A | MM용 MoveIt2 설정, URDF/SRDF, `mm_motion_bridge`(JSON 명령 → MoveIt goal), Nav2 bringup |
| [`harvest_moveit`](src/smartfarm/harvest_moveit/README.md) | 트랙 A | 초기 MM Nav2→MoveIt 수확 데모용 설정(UR10e 시절 URDF 포함). 현행 실행 경로는 `mm_moveit` |
| [`iwhub_control`](src/smartfarm/iwhub_control/README.md) | 트랙 B | IW 베이스 제어. `base_node`(cmd_vel→차동 바퀴, odom/TF, 승강), `mission_nav_node`(미션·도킹), `scan_self_filter_node` |
| [`fleet_dispatch`](src/smartfarm/fleet_dispatch/README.md) | 트랙 B | `cmd_vel_watchdog`, `nav2_lifecycle_activator`, Nav2/AMCL 런치·설정 |
| [`warehouse_dock`](src/smartfarm/warehouse_dock/README.md) | 트랙 C | 창고 하역. `fork_lift_node`, `fork_lift_return_node`(랙 6슬롯 적재·빈 팔레트 반환) |
| [`monitor_ui`](src/smartfarm/monitor_ui/README.md) | 트랙 D | 4분할 CCTV 관제 UI. `ui_status_node`, `recorder_node`(rosbag 녹화), `qos_bridge` + React 웹(`web/`) |
| [`smartfarm_interfaces`](src/smartfarm/smartfarm_interfaces/README.md) | 공용 | 커스텀 메시지/서비스. `TomatoDetection(Array)`, `ForkliftCycle.srv`, `DockAdjust.srv` 등 |
| `smartfarm_common` | 공용 | 현재 노드 없음(초기 스켈레톤 폐기) |

토픽·서비스 계약과 폐기 이력은 [`src/smartfarm/INTERFACES.md`](src/smartfarm/INTERFACES.md)에 정리돼 있습니다.

---

## 4. 저장소 구조

```text
~/cobot3_ws/
├── src/
│   ├── smartfarm/          # 프로젝트 ROS 2 패키지 (위 표)
│   ├── m0609/              # 부트캠프 실습 (개인 폴더)
│   └── m-explore-ros2/     # 외부 서브모듈: frontier 기반 자동 탐사·다중 로봇 맵 병합
├── isaacpjt/               # Isaac Sim 씬·로봇·ROS 브리지
│   ├── main.py             # 시뮬레이션 진입점
│   ├── robots/  scene/  ros/
│   ├── pjt_config/         # 씬·로봇 설정값
│   └── tests/              # 시뮬 로직 pytest
├── maps/                   # Nav2 정적맵 (기본 farm.yaml, 생성본 farm_gen.yaml)
├── docs/                   # 파트별 시스템 가이드·조사 기록
├── src/smartfarm/harvest_vision/resource/  # 시연용 YOLO 가중치
├── diagnostics/            # 통합 시뮬 진단 캡처 스크립트
├── scripts/  tools/        # 빌드 환경 수정, 디버그 bag 분석 등 (아래 표)
└── build/ install/ log/    # colcon 산출물 (.gitignore)
```

### `m-explore-ros2` 서브모듈

`src/m-explore-ros2`는
[`robo-friends/m-explore-ros2`](https://github.com/robo-friends/m-explore-ros2)의
커밋 `326cf8a`를 고정해 사용하는 외부 ROS 2 패키지다. 주요 구성은 다음과 같다.

- `explore_lite`: SLAM 지도의 미탐사 경계(frontier)를 찾아 Nav2
  `navigate_to_pose` 목표를 자동 생성한다.
- `map_merge`: 여러 로봇이 만든 occupancy grid를 하나로 병합한다.
- `explore_lite_msgs`: 탐사 제어용 ROS 2 메시지를 제공한다.

현재 프로젝트에서는
`fleet_dispatch/launch/harvester_nav2.launch.py`의 `slam:=true explore:=true`에서
`explore_lite`를 사용한다. 평상시 정적맵·AMCL 주행에는 실행되지 않는다.

저장소를 처음 받거나 서브모듈이 비어 있을 때:

```bash
git submodule update --init --recursive
```

일반 `git pull`은 서브모듈 내부 커밋을 자동으로 바꾸지 않는다. 팀에서 검증한 gitlink를
재현하려면 위 명령을 다시 실행한다. 외부 코드를 수정하거나 다른 upstream 커밋으로
올릴 때는 서브모듈 내부 커밋과 루트 저장소의 gitlink 변경을 별도 검토한다.

### 스크립트/도구

| 경로 | 용도 |
|---|---|
| `diagnostics/capture_sim_diag.sh` | 문제 재현 순간의 전체 ROS 토픽을 `ros2 bag record -a`로 캡처(`diagnostics/bags/`, gitignore). IW 도킹 충돌·MM 파지 등 사후 분석용 |
| `scripts/fix_ros_build_env.sh` | `colcon build` 시 `em`/`catkin_pkg` 누락 오류 해결 ([7. 자주 겪는 문제](#7-자주-겪는-문제) 참고) |
| `scripts/record_forklift_debug_bag.sh` | 지게차 관련 토픽만 골라 `debug_bags/`에 녹화 (전체 캡처인 `capture_sim_diag.sh`보다 좁은 지게차 전용 버전) |
| `scripts/analyze_forklift_debug_bag.py` | 위 지게차 디버그 bag을 읽어 리뷰용 요약 패킷(JSON)을 생성 |
| `tools/iw_dock_align_once.py` | 실행 중인 IW를 재시작 없이 지게차 도크 pose로 1회 직접 정렬(도킹 파라미터 튜닝용) |
| `tools/plot_harvest_tcp_center_path.py` | 현재 MM 수확 파라미터 기준 TCP 측면 경로를 그려 접근·파지 궤적을 시각 검증 |

### 문서

| 문서 | 내용 |
|---|---|
| [`docs/mm_system_guide.md`](docs/mm_system_guide.md) | MM 구조·좌표계·MoveIt·수확 FSM |
| [`docs/iw_system_guide.md`](docs/iw_system_guide.md) | IW 주행·도킹·미션 |
| [`docs/forklift_system_guide.md`](docs/forklift_system_guide.md) | 지게차 시퀀스·랙 기하 |
| [`docs/monitor_ui_current_branch_reference.md`](docs/monitor_ui_current_branch_reference.md) | 모니터링 UI 데이터 흐름·카메라 배치·설계 판단 기준 |
| [`src/smartfarm/INTERFACES.md`](src/smartfarm/INTERFACES.md) | 전 트랙 토픽/서비스 계약 |

> `docs/`에는 이 외에도 날짜가 찍힌 인수인계·검수요청·조사보고 문서(`handoff_*`,
> `review_request_*`, `investigation_*`, `change_summary_*`)가 있다. 특정 세션의
> 작업 기록이라 위 표에는 넣지 않았고, 필요할 때 파일명으로 찾아본다.

---

## 5. 설치 및 빌드

### 5.1 apt 의존 패키지

기준 배포판은 **Humble**(Ubuntu 22.04)입니다.

`ros-*-desktop`만으로는 **부족**합니다. 빠뜨리면 launch가 `package '...' not found`로
즉사합니다.

```bash
source /opt/ros/humble/setup.bash
sudo apt update

# 기반 — MoveIt / ros2_control / 주행
sudo apt install -y \
  ros-$ROS_DISTRO-moveit \
  ros-$ROS_DISTRO-moveit-servo \
  ros-$ROS_DISTRO-moveit-planners-chomp \
  ros-$ROS_DISTRO-moveit-task-constructor-core \
  ros-$ROS_DISTRO-moveit-task-constructor-capabilities \
  ros-$ROS_DISTRO-ros2-control \
  ros-$ROS_DISTRO-ros2-controllers \
  ros-$ROS_DISTRO-ur-description \
  ros-$ROS_DISTRO-ur-moveit-config \
  ros-$ROS_DISTRO-navigation2 \
  ros-$ROS_DISTRO-nav2-bringup \
  ros-$ROS_DISTRO-nav2-smac-planner \
  ros-$ROS_DISTRO-nav2-regulated-pure-pursuit-controller \
  ros-$ROS_DISTRO-slam-toolbox

# Isaac ↔ MoveIt 하드웨어 인터페이스
sudo apt install -y ros-$ROS_DISTRO-topic-based-ros2-control

# 모니터링 UI 전용 (트랙 D — UI를 안 쓰면 생략 가능)
sudo apt install -y \
  ros-$ROS_DISTRO-web-video-server \
  ros-$ROS_DISTRO-compressed-image-transport \
  ros-$ROS_DISTRO-rosbridge-server \
  ros-$ROS_DISTRO-rosbag2-storage-mcap
```

목록을 외우는 대신 `package.xml`에서 뽑아 쓰는 쪽이 안전합니다. 새 의존이 추가돼도
따라옵니다.

```bash
cd ~/cobot3_ws
rosdep install --from-paths src --ignore-src -r -y --rosdistro $ROS_DISTRO
# 확인: "All system dependencies have been satisfied"
rosdep check --from-paths src --ignore-src --rosdistro $ROS_DISTRO
```

> `ROS_DISTRO`가 설정돼 있지 않으면 rosdep이 키를 하나도 못 풀고 그냥 넘어갑니다.
> `source /opt/ros/<distro>/setup.bash`를 먼저 하거나 `--rosdistro`를 명시하세요.

### 5.2 파이썬 패키지 (트랙 A 비전)

`harvest_vision`의 YOLO 검출에 필요합니다. **`numpy`는 2.x로 올리면 안 됩니다** —
`cv_bridge`·Isaac 쪽 확장 모듈이 1.x ABI로 빌드돼 있어서 import가 깨집니다.

```bash
pip install ultralytics "numpy<2"
# torch는 GPU에 맞는 CUDA 빌드로 (RTX 50 계열은 cu128 이상)
```

시연용 가중치는 `src/smartfarm/harvest_vision/resource/`에 포함되어 있다.
원본 학습 데이터와 도메인 적응 스크립트는 현재 저장소에 포함되어 있지 않아
이 저장소만으로 재학습을 재현할 수는 없다.

### 5.3 모니터링 UI 웹 (Node)

```bash
cd ~/cobot3_ws/src/smartfarm/monitor_ui/web && npm install
```

Node 18 이상(vite 6). `npm install`을 건너뛰면 `npm run dev`가 `> vite`만 찍고
멈춘 것처럼 보입니다. 설치 중 뜨는 `npm warn allow-scripts esbuild` 경고는
무시해도 됩니다(프리빌트 바이너리라 install 스크립트가 필요 없습니다).

### 5.4 빌드

```bash
cd ~/cobot3_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

---

## 6. 협업 규칙

- 작업 시작 전 `git pull`, 끝나면 `add` → `commit` → `push`.
- 커밋은 의미 단위로 자주. 메시지는 한국어로 무엇을 왜 바꿨는지 쓴다.
- 트랙 담당: **A 이현민**(수확 MM·비전), **B 김지훈**(IW 운반·Nav2), **C 김민성**(지게차·창고).
  다른 트랙 파일을 고쳐야 하면 담당자와 먼저 상의한다.
- `build/`, `install/`, `log/`, `__pycache__/`, USD/가중치 파일은 커밋하지 않는다.

---

## 7. 자주 겪는 문제

| 상황 | 해결 |
|---|---|
| `package '...' not found` 로 launch 즉사 | apt 의존 패키지 누락 → [5.1](#51-apt-의존-패키지). `rosdep check --from-paths src --ignore-src --rosdistro humble` 로 확인 |
| `npm run dev`가 `> vite`에서 멈춤 | `web/`에 `npm install`을 안 한 것 → [5.3](#53-모니터링-ui-웹-node) |
| `git push` 거부(rejected) | 원격에 새 커밋이 있음 → `git pull` 후 다시 push |
| 로봇이 토픽을 못 받음 | 모든 호스트의 `ROS_DOMAIN_ID`(108)·RMW·DDS 화이트리스트가 같은지 확인 |
| Nav2가 목표를 못 세움 | `map` 인자(기본 `maps/farm.yaml`)가 맞는 지도인지, AMCL 초기 pose가 맞는지 확인 |
| CAM-01 화면이 비어 있음 | `vision_node`가 떠 있어야 함(생 RGB가 아니라 검출 결과를 받는 화면) |
| `colcon build` 시 `smartfarm_interfaces`에서 `ModuleNotFoundError: No module named 'em'` / `'catkin_pkg'` | `./scripts/fix_ros_build_env.sh` 실행 — `$ROS_DISTRO`를 감지해 `.venv`에 `catkin_pkg`/`lark`/맞는 버전의 `empy`를 설치한다(Humble/Iron=3.3.4, Jazzy 이상=4.x). ROS 2 배포판이 여러 개면 쓸 `/opt/ros/<distro>/setup.bash`를 먼저 source한다. 수동으로 `pip install empy`만 하지 말 것 |
