# 스마트팜 토마토 수확 시뮬레이션 (Isaac Sim 5.1)

수확 MM(모바일 매니퓰레이터) + 운반 AMR(iw.hub) + 지게차(ForkliftB)가 온실에서
토마토를 **수확 → 운반 → 적재**하는 시뮬레이션.

핵심 계약: **판단은 ROS2, 실행은 Isaac.**
어느 과실을 딸지·언제 자를지·어디로 갈지는 ROS2 노드(개인 PC)가 정해 토픽으로 보내고,
이 트리(Isaac)는 씬·물리·로봇 실행만 담당한다. 토픽만 쏘면 로봇이 움직인다.

---

## 1. 폴더 구성

```
isaacpjt/
├── main.py              ★ 진입점 — 씬 + 로봇 3대 + ROS2 브리지 전부 여기서 뜬다
├── pjt_config/
│   └── settings.py      모든 수치의 단일 출처 (값마다 근거 등급 [1]출처~[4]임의 주석)
├── pjt_utils/           xform(참조 prim 안전 배치)·paths·ripeness
├── robots/              로봇 조립 (놓기만 — 제어는 ROS2)
│   ├── harvester.py       수확 MM: Ridgeback+UR10e+2F-85+커터지그+가동날+D455
│   ├── transporter.py     지게차 B (포크 승강 0~2.0m)
│   ├── iwhub.py           운반 AMR iw.hub (차동+승강, 팔레트 언더라이드)
│   ├── control.py         Isaac 내부 제어면 (텔레옵/스크립트용)
│   ├── teleop.py          키보드 조종 (GUI, 디버깅용)
│   ├── assets.py          Nucleus 에셋 경로 해석
│   └── cad_jig/           커터 지그 CAD USD (커플러+서보가위+D455 마운트)
├── ros/
│   └── robot_bridge.py   로봇별 OmniGraph ROS2 브리지 (코드 생성 — GUI 클릭 금지 규칙)
├── scene/               온실 씬 (지오메트리만 — 정책 없음)
│   ├── greenhouse_task.py 씬 조립 태스크 (온실+식물+창고 전부)
│   ├── tomato_plants.py   4줄×… 재배라인, 과실 542개(ripe 455/spoiled 87), 시드 고정
│   ├── warehouse.py       창고 랙 3섹터×2단 + 건물
│   └── ground/lighting/greenhouse/physics/pedicel/tray
├── assets/
│   ├── tomato/          과실 USD 20개 (2클래스: ripe/spoiled)
│   └── aoc/             배경 식물(시각 전용)
└── tools/
    └── iwhub_bridge_check.py  ROS2 브리지 단독 점검 (iw.hub 1대만 — 문제 시 최소재현)
```

---

## 2. 실행 방법

### 터미널 규칙 (중요)
**Isaac 터미널과 ROS2 터미널을 절대 섞지 말 것.** 한 터미널에서 둘 다 source 하면
librcl 심볼 충돌로 죽는다.

| 터미널 | 준비 | 하는 일 |
|---|---|---|
| Isaac | 아래 env 4개 export | `isaac_python main.py` 로 시뮬 실행 |
| ROS2 | `rosenv` (alias) | `ros2 topic pub/echo`, 노드 실행 |

### ⚠ `isaac_python` / `rosenv` 는 개인 별칭이다 — 컴퓨터마다 다를 수 있음

이 문서의 두 명령은 GPU 노트북 `~/.bashrc` 에 정의된 **별칭**이다. 자기 컴퓨터에
없으면 아래를 자기 경로에 맞게 `~/.bashrc` 에 추가할 것:

```bash
# Isaac Sim 동봉 파이썬 (Isaac 설치 경로는 컴퓨터마다 다름 — 자기 경로로!)
alias isaac_python="$HOME/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release/python.sh"

# ROS2 터미널 환경 (system Humble + 도메인 + 통신설정 + 워크스페이스 오버레이)
alias rosenv='source /opt/ros/humble/setup.bash; \
  export ROS_DOMAIN_ID=108; export ROS_DISTRO=humble; \
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp; \
  export FASTRTPS_DEFAULT_PROFILES_FILE="$HOME/.config/net_loadtest/fastdds_whitelist.xml"; \
  [ -f ~/cobot3_ws/install/setup.bash ] && source ~/cobot3_ws/install/setup.bash'
```

- `isaac_python` = Isaac Sim 에 **동봉된** python.sh (3.11). 시스템 python3 로
  이 트리를 돌리면 isaacsim 모듈이 없어서 안 된다 (반대로 `src/` ROS2 노드는
  시스템 python3 전용).
- `rosenv` = ROS2 Humble + 도메인 108 + FastDDS 화이트리스트 + colcon 오버레이.
  마지막 줄은 `colcon build` 를 한 번 해야 생기는 파일이라 조건부로 걸었다.
- 화이트리스트 XML(`fastdds_whitelist.xml`)이 없는 컴퓨터는 그 export 줄을 빼거나
  파일을 복사받을 것 — 훈련장(10.10.0.x) 밖에서는 루프백만 통신된다(정상).

### 시뮬 띄우기 (Isaac 터미널)

ROS2 브리지용 env 를 걸고 실행한다 (시스템 ROS 는 **source 하지 않는다** —
Isaac 내부 humble lib 와 심볼 충돌. 아래는 내부 lib 경로만 거는 것):

```bash
ISAAC=~/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release
export LD_LIBRARY_PATH=$ISAAC/exts/isaacsim.ros2.bridge/humble/lib:$LD_LIBRARY_PATH
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=108
export FASTRTPS_DEFAULT_PROFILES_FILE=~/.config/net_loadtest/fastdds_whitelist.xml

cd ~/cobot3_ws/isaacpjt
isaac_python main.py                # GUI + ROS2 브리지 (기본)
isaac_python main.py --quiet        # + 경고 스팸(metricsAssembler) 끔 — 평소 권장
isaac_python main.py --no-ros       # 씬+로봇만 (브리지 없이 확인용. env 도 불필요)
isaac_python main.py --headless     # 렌더 없음 (검증용)
```

env 없이 띄우면 "ROS2 Bridge startup failed" — 씬은 뜨지만 토픽이 안 생긴다.

뜨면: 온실(6섹터·과실 542개) + 수확MM·지게차B·iw.hub(온실 앞마당, 임시 배치)
+ 브리지 그래프 5개. **Play 상태여야 토픽이 돈다** (GUI 는 자동 Play).

---

## 3. ROS2 인터페이스 (토픽 명세)

공통: **ROS_DOMAIN_ID=108** / RMW=rmw_fastrtps_cpp / FastDDS 화이트리스트(10.10.0.1~5 + 루프백).
양쪽 머신이 전부 같아야 서로 보인다. `ROS_LOCALHOST_ONLY` 는 해제(0).

### 토픽 목록

| 토픽 | 타입 | 방향 | 내용 |
|---|---|---|---|
| `/harvester_0/joint_command` | sensor_msgs/JointState | ROS2→Isaac | 수확 MM 팔·그리퍼 명령 |
| `/harvester_0/joint_states`  | sensor_msgs/JointState | Isaac→ROS2 | 수확 MM 관절 상태 (15 DOF) |
| `/harvester_0/cmd`           | std_msgs/String (JSON) | ROS2→Isaac | 가동날·베이스 (아래 설명) |
| `/forklift_0/joint_command`  | sensor_msgs/JointState | ROS2→Isaac | 지게차 구동·조향·포크 |
| `/forklift_0/joint_states`   | sensor_msgs/JointState | Isaac→ROS2 | 지게차 관절 상태 |
| `/iwhub_0/joint_command`     | sensor_msgs/JointState | ROS2→Isaac | iw.hub 바퀴·승강 |
| `/iwhub_0/joint_states`      | sensor_msgs/JointState | Isaac→ROS2 | iw.hub 관절 상태 |
| `/clock`                     | rosgraph_msgs/Clock    | Isaac→ROS2 | 시뮬 시간 |

JointState 명령은 **name 배열에 관절 이름을 명시**하고 해당 인덱스에 position 또는
velocity 를 채운다 (이름 없는 관절은 건드리지 않음).

### 로봇별 관절 이름

**수확 MM (`harvester_0`)**

| 관절 | 명령 | 범위/단위 |
|---|---|---|
| `shoulder_pan_joint` `shoulder_lift_joint` `elbow_joint` `wrist_1_joint` `wrist_2_joint` `wrist_3_joint` | position | rad (UR10e 6축) |
| `finger_joint` | position | 0(열림) ~ 0.8(닫힘) rad — 2F-85 마스터, 나머지 손가락 관절은 자동 종속 |
| `dummy_base_*` 3개 | ✗ JointState 로 안 먹음 | 키네마틱 베이스 — 아래 JSON `base` 로 |

스폰 자세 = 수확자세(wrist_1 이 +180° 돌아가 커터가 파지점 위로 온 상태).

**`/harvester_0/cmd` (String, data 에 JSON)** — 아티큘레이션 밖 자유도 2개:

```json
{"blade": 35}              // 가동날 각도[deg]. 0=열림 ~ 35=닫힘(줄기 전단)
{"base": [1.0, -12.0, 0.0]} // 베이스 절대 [x(m), y(m), yaw(rad)] — 텔레포트 방식
{"blade": 0, "base": [0, -12, 0]}  // 같이 보내도 됨 (온 필드만 적용)
```
- 가동날은 별도 서보 리볼루트(아티큘레이션 미포함)라 JointState 에 안 잡힌다.
- 베이스는 키네마틱(위치드라이브 무시·순간이동만 — 실측)이라 좌표 지정 방식이다.

**지게차 B (`forklift_0`)**

| 관절 | 명령 | 범위/단위 |
|---|---|---|
| `lift_joint` | position | 0 ~ 2.0 m (포크 승강. 창고 2단 0.9m 커버) |
| `back_wheel_swivel` | position | rad (후륜 조향) |
| `back_wheel_drive` | velocity | rad/s (후륜 구동) |

**iw.hub (`iwhub_0`)**

| 관절 | 명령 | 범위/단위 |
|---|---|---|
| `left_wheel_joint` `right_wheel_joint` | velocity | rad/s (차동 — 같으면 직진, 다르면 회전) |
| `lift_joint` | position | m (팔레트 승강) |

### 동작 확인 예시 (ROS2 터미널, `rosenv` 후)

```bash
ros2 topic list                                  # 위 토픽들 보이면 브리지 연결 OK
ros2 topic echo /iwhub_0/joint_states --once     # 상태 수신 확인

# iw.hub 전진
ros2 topic pub -1 /iwhub_0/joint_command sensor_msgs/msg/JointState \
  '{name: [left_wheel_joint, right_wheel_joint], velocity: [3.0, 3.0]}'

# 지게차 포크 올리기
ros2 topic pub -1 /forklift_0/joint_command sensor_msgs/msg/JointState \
  '{name: [lift_joint], position: [0.9]}'

# MM 팔 4번축 살짝 + 그리퍼 닫기
ros2 topic pub -1 /harvester_0/joint_command sensor_msgs/msg/JointState \
  '{name: [wrist_1_joint, finger_joint], position: [2.9, 0.8]}'

# 가동날 닫기(절단 연출) / 열기
ros2 topic pub -1 /harvester_0/cmd std_msgs/msg/String '{data: "{\"blade\": 35}"}'
ros2 topic pub -1 /harvester_0/cmd std_msgs/msg/String '{data: "{\"blade\": 0}"}'
```

---

## 4. 설계 규칙 (요약 — 자세한 건 CLAUDE.md)

- **판단/실행 분리**: 씬·로봇은 좌표와 능력만 제공. "어느 과실·어느 슬롯" 같은 정책은
  전부 ROS2 쪽. 값이 ROS2 로 돌아가서야 쓰이면 그 로직은 ROS2 소속이다.
- **물리 우회 금지**: 파지는 접촉 마찰(실측: 파지력 2N 이상이면 유지 — spike01),
  절단은 꽃자루 조인트 절단. 고정조인트로 때우는 연출 금지.
- **모든 수치는 settings.py** 한 곳에, 근거 등급 주석과 함께. 스크립트에 재선언 금지.
- **OmniGraph 는 코드로만 생성** (GUI 클릭 금지 — 버전 관리 때문).

## 5. 자주 겪는 것

| 증상 | 원인/조치 |
|---|---|
| `ROS2 Bridge startup failed` | env 누락 — §2 의 export 4개를 걸었는지 확인 |
| 토픽은 보이는데 로봇이 안 움직임 | 시뮬이 Play 상태인지 확인. JointState 의 `name` 이 위 표와 일치하는지 확인 |
| `metricsAssembler SetEditTarget` 경고 수백 줄 | CAD 지그 단위보정 스팸(무해). `--quiet` 로 끔 |
| `RSD455 did not match any rigid bodies` | D455 카메라 잔재 로그(무해 — 기능 정상) |
| Robotiq `invalid inertia tensor` | 에셋 시각 링크 근사(무해) |
| 다른 PC 에서 토픽이 안 보임 | 도메인 108·RMW·화이트리스트 셋 다 일치? 유선 10.10.0.x 인지? `ROS_LOCALHOST_ONLY` 해제? |

로봇 배치(온실 앞마당 y=−12)는 임시 — 물류 동선 확정 후 조정 예정.

---

## 6. git 참고 (다른 컴퓨터에서 pull 할 때)

`.gitignore` 가 `*.usd` 를 전역 무시하지만(재생성 방침), **실행에 필요한 USD 는
예외로 추적**되어 있어 pull 만 받으면 바로 돈다:

| 자산 | git | 비고 |
|---|---|---|
| `assets/tomato/*.usd` (과실 20개) | ✅ 추적 | obj 소스에서 재생성도 가능 (개인폴더 tomatest/00_convert) |
| `assets/aoc/usd/tomato_plant.usd` (배경) | ✅ 추적 | 없어도 씬은 뜸 (경고 후 원기둥 줄기로 대체) |
| `robots/cad_jig/*.usd` (커터 지그) | ✅ 추적 | 재생성 스크립트가 repo 밖 — 필수 |
| 로봇·환경 에셋 (Ridgeback/UR10e/지게차/iw.hub…) | — | git 아님 — **Nucleus 서버**에서 런타임 로드 (인터넷/캐시 필요) |
