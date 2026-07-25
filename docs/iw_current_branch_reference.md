# IW 전체 구현 기준 스냅샷·머지 비교 가이드

이 문서는 `harvest/rmp-mm-suction` 브랜치에 구현된 IW 관련 내용을 한곳에 모은
비교용 기준 스냅샷이다. 다른 브랜치의 IW 코드와 기능·설정·인터페이스를 비교하거나
머지할 때 사용한다.

- 기준 브랜치: `harvest/rmp-mm-suction`
- 기준 커밋: `b235251`
- 대상: IW Nav2, 도킹 목표 생성, 라이다 자기반사 필터, 팔레트·KLT·토마토 물리
- 주의: 빈 KLT 충돌체 수정은 코드·정적 테스트까지 완료했으며, Isaac Sim 재시작 후
  실제 토마토 안착 여부는 통합 재검증이 필요하다.

## 1. 시스템 역할과 전체 데이터 흐름

IW는 MM이 수확한 토마토를 KLT에 받아 창고로 운반하고, 지게차와 팔레트를
인수인계하는 차동구동 AMR이다.

```text
Isaac Sim
  IwDriver / IwHub
    ├─ IW articulation·바퀴·lift joint
    ├─ 팔레트+KLT Load 및 토마토 물리
    ├─ 실제 chassis odom/TF
    ├─ 전·후방 RTX LaserScan
    ├─ 빈 KLT release pose
    └─ 데크 실측 geometry
             │
             ▼
ROS2 iwhub_control
    ├─ scan_self_filter: raw scan → filtered scan
    ├─ AMCL: map → odom
    ├─ Nav2: map/scan/goal → cmd_vel
    ├─ base_node: cmd_vel → wheel joint_command
    └─ mission_nav_node: IDLE/FOLLOW/FORKLIFT → NavigateToPose
             │
             ▼
warehouse_dock / forklift
    ├─ canonical dock 고정
    ├─ 팔레트 joint 소유권 전환
    └─ 데크 높이 기반 포크 정렬
```

책임 분리:

- Isaac: 실제 물리, 센서, joint 명령 실행, 실제 chassis odom
- ROS2: 위치추정, 경로계획, 회피, 미션 상태, 바퀴 속도 계산
- MM 수확 FSM: 수확 시작 시 `FOLLOW`, 플레이스 시 빈 KLT pose 사용
- 지게차 FSM: IW 데크 실측값을 받아 팔레트 상하차

## 2. 관련 파일

### ROS2 / Nav2

- `src/smartfarm/iwhub_control/config/nav2_params.yaml`
- `src/smartfarm/iwhub_control/launch/iwhub_nav2.launch.py`
- `src/smartfarm/iwhub_control/iwhub_control/mission_nav_node.py`
- `src/smartfarm/iwhub_control/iwhub_control/scan_self_filter_node.py`
- `src/smartfarm/iwhub_control/setup.py`
- `src/smartfarm/fleet_dispatch/fleet_dispatch/nav2_lifecycle_activator.py`

### Isaac Sim / 적재물 물리

- `isaacpjt/robots/iwhub.py`
- `isaacpjt/iw.py`
- `isaacpjt/scene/physics.py`
- `isaacpjt/pjt_config/settings.py`
- `isaacpjt/iw_dock.py`

### ROS2 베이스·통합·창고 인수인계

- `src/smartfarm/iwhub_control/iwhub_control/base_node.py`
- `src/smartfarm/iwhub_control/iwhub_control/kinematics.py`
- `src/smartfarm/iwhub_control/launch/iwhub_base.launch.py`
- `src/smartfarm/iwhub_control/urdf/iwhub.urdf`
- `src/smartfarm/harvest_vision/launch/smartfarm_integration.launch.py`
- `src/smartfarm/warehouse_dock/warehouse_dock/fork_lift_node.py`
- `src/smartfarm/warehouse_dock/warehouse_dock/fork_lift_return_node.py`
- `isaacpjt/pjt_utils/deck_geometry.py`

## 3. IW 모델·스폰·구동

### 3.1 기본 모델과 스폰

| 항목 | 현재 값 |
|---|---|
| Isaac driver flag | `--iw` |
| driver name / namespace | `iw` / `iwhub_0` |
| prim root | `/World/IwHub` |
| 초기 위치 | `(1.6955, -12.0, floor_z)` |
| 초기 yaw | `180°` |
| 차체 실측 크기 | `1.431 × 0.659 × 0.231 m` |
| 적재물 시각 크기 | 약 `1.2 × 0.8 × 0.32 m` |
| DOF | `left_wheel_joint`, `right_wheel_joint`, `lift_joint` |

초기 yaw를 180°로 두어 IW의 긴 후방 오버행이 MM 반대쪽을 향하게 한다. 스폰 z는
고정값을 그대로 쓰지 않고 에셋 local bbox 최저점을 읽어 바닥보다 `0.005 m` 위에
놓은 뒤 중력으로 정착시킨다.

### 3.2 차동구동

Nav2의 `/iwhub_0/cmd_vel`을 ROS `base_node`가 다음 식으로 바퀴 각속도로 바꾼다.

```text
left  = (v - w × separation / 2) / radius
right = (v + w × separation / 2) / radius
```

캘리브레이션 값:

| 항목 | 값 |
|---|---:|
| 실효 wheel radius | `0.0771 m` |
| 실효 wheel separation | `0.576 m` |
| command timeout | `0.5 s` |
| watchdog 주기 | `0.1 s` |

명령이 `0.5 s` 끊기면 좌우 바퀴 속도를 0으로 보낸다. 노드 종료 시에도 마지막
drive target을 0으로 덮는다.

목표점 근처 잔진동 방지 deadband:

| 축 | stop | restart |
|---|---:|---:|
| linear | `0.01 m/s` | `0.02 m/s` |
| angular | `0.02 rad/s` | `0.04 rad/s` |

start 문턱을 stop보다 크게 둔 히스테리시스다. 다른 브랜치의 base controller로
교체할 경우 watchdog과 이 정지 안정화가 사라지지 않는지 확인한다.

### 3.3 Isaac wheel drive와 안정화

| 항목 | 값 |
|---|---:|
| angular drive stiffness | `0` |
| angular drive damping | `3000` |
| drive max force | `5000` |
| wheel static/dynamic friction | `1.2 / 1.0` |
| restitution | `0` |
| friction combine | `max` |
| sleep threshold | `0.01` |
| stabilization threshold | `0.002` |
| solver position/velocity iterations | `16 / 4` |
| max depenetration velocity | `0.5` |

적재 팔레트의 마찰을 이기고 제자리 회전할 수 있도록 drive torque를 보강한 값이다.
차체가 정지 중 접촉 솔버로 흔들리는 것을 줄이기 위해 articulation solver와 sleep
설정도 적용한다.

### 3.4 odom 모드

`iwhub_base.launch.py` 단독 실행은 바퀴 각도 적분 odom을 발행할 수 있다. 하지만
현재 Nav2 통합 경로는 `publish_odom=false`이며 Isaac chassis의 실제 pose에서 만든
`/iwhub_0/odom`과 TF를 사용한다.

- 단독 진단: joint_states의 바퀴 절대각 → 중점 적분 odom
- 통합 Nav2: Isaac chassis odom → AMCL 보정
- 두 odom 발행기를 동시에 켜면 TF가 중복되므로 금지

## 4. ROS 인터페이스 기준표

| 방향 | 이름 | 타입/의미 |
|---|---|---|
| Nav2 → base | `/iwhub_0/cmd_vel` | `geometry_msgs/Twist` |
| base → Isaac | `/iwhub_0/joint_command` | `sensor_msgs/JointState`, wheel velocity |
| Isaac → ROS | `/iwhub_0/joint_states` | wheel/lift state |
| Isaac → ROS | `/iwhub_0/odom` | 실제 chassis odometry |
| Isaac → ROS | `/iwhub_0/tf`, `/tf_static` | IW 전용 TF |
| Isaac → ROS | front/back raw scan | 전·후방 LaserScan |
| filter → Nav2 | front/back filtered scan | 자기반사 제거 LaserScan |
| coordinator → IW | `/iw/mission` | `IDLE`, `FOLLOW`, `FORKLIFT` |
| IW → coordinator | `/iw/status` | 미션 상태 문자열 |
| Isaac → MM | `/iw/basket/empty_slot_pose` | `map` 기준 빈 KLT release pose |
| Isaac → forklift | `/iwhub_0/deck_geometry` | dock/deck/pallet-hole JSON |
| forklift → system | `/forklift/pallet_on_iw` | IW에 올라간 pallet ID |

`/iw/mission`과 `/iw/status`는 transient-local QoS를 사용한다. 늦게 뜬 노드도 마지막
미션/상태를 받을 수 있어야 한다.

## 5. TF와 센서

TF 체인:

```text
iwhub_0/map              AMCL
└─ iwhub_0/odom          Isaac chassis odom
   └─ iwhub_0/base_link
      └─ iwhub_0/chassis
         ├─ iwhub_0/front_2d_lidar
         └─ iwhub_0/back_2d_lidar
```

`base_link → chassis`는 identity 정적 TF다. 전·후방 라이다 정적 TF:

| 센서 | xyz | yaw | 범위/주기 |
|---|---|---:|---|
| front | `(0.65, 0, 0.15)` | `0` | `0.05~30 m`, 10 Hz 프로파일 |
| back | `(-1.08, 0, 0.15)` | `π` | `0.05~30 m`, 10 Hz 프로파일 |

RViz용 `iwhub.urdf`는 차체와 적재물의 시각 크기를 보여줄 뿐이다. Nav2 footprint는
URDF collision에서 자동 생성되지 않고 `nav2_params.yaml`에 별도로 존재한다.

## 6. IW Nav2 구성

### 6.1 프레임과 토픽

| 용도 | 현재 값 |
|---|---|
| namespace | `iwhub_0` |
| global frame | `iwhub_0/map` |
| odom frame | `iwhub_0/odom` |
| base frame | `iwhub_0/base_link` |
| odom | `/iwhub_0/odom` |
| 전방 raw scan | `/iwhub_0/front_2d_lidar/scan` |
| 후방 raw scan | `/iwhub_0/back_2d_lidar/scan` |
| 전방 filtered scan | `/iwhub_0/front_2d_lidar/scan_filtered` |
| 후방 filtered scan | `/iwhub_0/back_2d_lidar/scan_filtered` |
| Nav2 action | `/iwhub_0/navigate_to_pose` |
| 미션 입력 | `/iw/mission` |
| 상태 출력 | `/iw/status` |

AMCL은 전방 filtered scan 하나를 사용하고, local/global costmap은 전·후방
filtered scan을 모두 사용한다. odom은 ROS에서 바퀴 적분으로 새로 만들지 않고
Isaac의 실제 chassis 자세를 사용한다.

### 6.2 초기 위치추정

```yaml
initial_pose:
  x: 1.6955
  y: -12.0
  yaw: 3.141592653589793
```

주요 AMCL 값:

| 항목 | 값 |
|---|---:|
| particles | `800 ~ 3000` |
| max beams | `180` |
| laser range | `0.05 ~ 25.0 m` |
| update distance / angle | `0.05 m / 0.05 rad` |
| odom noise alpha1/2 | `0.35 / 0.35` |
| odom noise alpha3/4 | `0.30 / 0.30` |
| z_hit / z_rand | `0.80 / 0.20` |
| sigma_hit | `0.12` |
| beam skip | 활성 |

충돌이나 바퀴 헛돎이 발생할 수 있는 시뮬레이션 특성상 odom을 과신하지 않고,
정적 맵과 일치하는 라이다 관측으로 자주 보정하는 설정이다.

### 6.3 풋프린트

풋프린트는 IW와 팔레트/KLT 적재물을 위에서 본 2D 충돌 외곽선이다. 라이다가
차체 외곽보다 앞뒤로 돌출되어 있으므로 라이다 원점까지 포함한다.

#### Local costmap

```yaml
footprint: "[[0.65, 0.401], [0.65, -0.401],
             [-1.08, -0.401], [-1.08, 0.401]]"
footprint_padding: 0.05
```

- 전방: `+0.65 m`
- 후방: `-1.08 m`
- 좌우: `±0.401 m` (폭 `0.802 m`)
- 추가 안전 여유: 전 방향 `0.05 m`

#### Global costmap

```yaml
footprint: "[[0.70, 0.451], [0.70, -0.451],
             [-1.08, -0.401], [-1.08, 0.401]]"
footprint_padding: 0.0
```

전역 계획에서는 회전 시 앞 모서리가 베드에 붙지 않도록 전방과 좌우를 local보다
각각 `0.05 m` 더 크게 잡는다.

#### 머지 시 주의

- 다른 브랜치의 차체 크기만 사용해 풋프린트를 축소하면 실제 라이다·팔레트 외곽이
  Nav2 충돌 검사 밖으로 나갈 수 있다.
- 풋프린트를 바꾸면 `scan_self_filter_node.py`의 자기반사 제거 범위도 함께 검토한다.
- Isaac collider는 3D 실제 충돌, Nav2 footprint는 2D 충돌 예측이므로 둘 중 하나만
  맞춰서는 안 된다.

### 6.4 Costmap

#### Local

- frame: `iwhub_0/odom`
- rolling window: `6 × 6 m`
- resolution: `0.05 m`
- update/publish: `10 / 5 Hz`
- layers: static + obstacle + inflation
- obstacle range: `20 m`
- raytrace range: `25 m`
- inflation radius: `0.35 m`
- cost scaling factor: `7.0`

Local costmap에도 static layer를 유지한다. 전·후방 라이다 최소 감지거리 때문에
차체 중앙 측면의 정적 벽이 scan에서 사라지는 구간을 보완하기 위한 것이다.

#### Global

- frame: `iwhub_0/map`
- resolution: `0.05 m`
- layers: static + obstacle + inflation
- inflation radius: `0.05 m`
- cost scaling factor: `12.0`
- unknown space 통과 허용

local/global 모두 `always_send_full_costmap: true`이다. 시뮬레이션 타임라인 재시작
직후 RViz가 첫 full map을 놓쳐도 다음 발행에서 복구하기 위한 설정이다.

### 6.5 Planner와 Controller

전역 플래너는 Navfn A*이다.

```yaml
plugin: nav2_navfn_planner/NavfnPlanner
use_astar: true
allow_unknown: true
tolerance: 0.25
```

로컬 컨트롤러는 DWB이다.

| 항목 | 값 |
|---|---:|
| min/max linear velocity | `-0.25 / 0.50 m/s` |
| max angular velocity | `0.80 rad/s` |
| linear acceleration/deceleration | `1.2 / -1.5 m/s²` |
| angular acceleration/deceleration | `1.6 / -2.0 rad/s²` |
| vx/vtheta samples | `25 / 35` |
| simulation time | `2.0 s` |
| goal tolerance | `0.25 m / 0.25 rad` |
| BaseObstacle scale | `2.0` |

기존 `BaseObstacle.scale=0.05`는 경로 critic에 비해 장애물을 사실상 무시해 회피가
늦었으므로 `2.0`을 보존한다. velocity smoother도 같은 속도·가속도 한계를 사용한다.

### 6.6 라이다 자기반사 필터

팔레트와 KLT가 360도 라이다에 자기 장애물로 잡히는 문제를 막기 위해 raw scan을
base frame으로 변환하고 다음 사각형 내부의 점을 `inf`로 제거한다.

```text
x: -1.0835 ~ +0.4475 m
y: -0.4510 ~ +0.4510 m
```

라이다 위치:

- front: `(x=+0.65, y=0, yaw=0)`
- back: `(x=-1.08, y=0, yaw=π)`

머지 대상 브랜치에서 라이다 TF나 적재 외곽이 바뀌면 필터 범위도 같이 바꿔야 한다.
필터 노드의 console script 등록(`setup.py`)과 launch 기동도 누락하면 안 된다.

### 6.7 미션과 도킹

미션 상태:

- `IDLE`: 목표를 보내지 않음
- `FOLLOW`: MM 근처로 이동
- `FORKLIFT`: 창고 도킹점으로 1회 이동

초기 상태는 반드시 `IDLE`이다. 수확 코디네이터가 `FOLLOW`를 발행하기 전에 IW가
MM과 동시에 출발하지 않도록 한다.

FOLLOW 목표는 MM의 고정 +X 방향이 아니다.

```text
목표 = MM 중심 + normalize(MM → 현재 IW) × dock_standoff
dock_standoff = 1.2 m
```

이 로직은 IW가 MM의 베드 쪽이 아니라 자신이 접근한 쪽에 비접촉 정차하게 한다.
기존 `MM +X × 1.6955 m` 방식으로 되돌리면 IW가 베드 안으로 들어갈 수 있다.

FOLLOW 목표 갱신 임계값:

- 위치 변화 `0.30 m`
- yaw 변화 `30°`

창고 목표:

- `(x=0.0, y=10.84885, yaw=π/2)`

### 6.8 Nav2 기동 순서와 자동 복구

MM Nav2와 IW Nav2가 동시에 뜨면 FastDDS lifecycle service 응답이 유실되어
`/iwhub_0/map`이 발행되지 않을 수 있다.

현재 순서:

1. IW base와 scan self-filter 시작
2. `2 s` 후 localization(map_server + AMCL) 시작
3. localization activator가 `3 s` 후 상태를 확인하며 configure/activate 재시도
4. `3 s` 후 navigation stack 시작 (`autostart=False`)
5. mission node가 action server 미활성 시 lifecycle `STARTUP`을 반복 요청

TimerAction 내부에서 `PushRosNamespace("iwhub_0")`를 다시 적용하는 구조를 보존한다.
이를 제거하면 MM의 전역 Nav2 노드와 이름이 충돌할 수 있다.

## 7. 팔레트·KLT·토마토 물리

### 7.1 강체 계층

```text
IwHub chassis (articulation rigid body)
└─ FixedJoint: /World/IwHubCargo/DeckJoint
   └─ Load (팔레트 + KLT 8개를 묶는 별도 dynamic rigid body)
      ├─ Pallet colliders
      └─ KLT_00 ... KLT_31 colliders

Tomatoes/T_* (각 토마토는 Load 밖의 별도 dynamic rigid body)
```

핵심:

- `Load`는 chassis에 FixedJoint로 결속되어 IW 주행을 따라간다.
- 팔레트와 KLT는 Load의 collider이며 별도 rigid body가 아니다.
- 토마토는 Load의 자식 강체가 아니라 독립 dynamic rigid body다.
- 따라서 토마토는 KLT 내부에서 중력·마찰·접촉에 따라 움직인다.
- 창고에서 팔레트를 인수할 때 joint를 해제할 수 있어야 한다.

참조 USD가 자체 rigid body/collider를 포함할 수 있으므로 먼저
`disable_physics()`로 제거한다. 이를 생략하면 중첩 rigid body와
`missing xformstack reset` 경고가 발생할 수 있다.

### 7.2 팔레트

창고에 독립 배치되어 지게차가 드는 팔레트의 기준 설정:

| 속성 | 값 |
|---|---:|
| 물리 역할 | Load의 collider |
| collider | `convexDecomposition` |
| max convex hulls | `64` |
| voxel resolution | `500000` |
| error percentage | `1.0` |
| 기준 질량 | `25 kg` |
| static friction | `0.5` |
| dynamic friction | `0.35` |

단일 `convexHull`을 사용하면 팔레트 포크 슬롯이 막힌다. 포크 채널을 유지하려면
`convexDecomposition`을 보존해야 한다.

현재 IW 초기 적재 세트에서는 팔레트와 KLT를 합친 `Load`에 유효 밀도
`200 kg/m³`를 적용한다. 위 `25 kg`, 마찰 `0.5/0.35`는 창고의 독립 팔레트
물리에 사용하는 값이며, IW의 초기 `Load` 생성 경로에는 직접 바인딩되지 않는다.
두 경로를 혼동하지 않는다.

### 7.3 KLT

| 항목 | 값 |
|---|---:|
| 배열 | `4 × 2` |
| 간격 | x `0.31 m`, y `0.25 m` |
| scale | `0.85` |
| collider | 8개 모두 `convexDecomposition` |
| 초기 채움 슬롯 | `(0,0)`, `(1,1)`, `(3,0)` |
| 초기 토마토 수 | 슬롯당 5개, 총 15개 |

**반드시 보존할 수정:** 초기 채움 여부와 관계없이 8개 KLT 모두 collider를 가져야
한다. `filled` 조건 안에서만 collider를 추가하면 빈 플레이스 대상 `KLT_31`이
시각적으로만 존재하고 토마토가 그대로 관통한다.

올바른 순서:

1. KLT USD 참조 및 pose/scale 설정
2. 에셋 자체 물리 제거
3. 모든 KLT에 `convexDecomposition` collider 적용
4. `filled`이면 초기 토마토만 생성

빈 슬롯 release pose는 초기 채움 슬롯을 제외한 후보 중 데크 로컬 x가 가장 큰
슬롯을 선택한다. 현재는 `KLT_31`이 선택되며, KLT 윗면보다 `0.05 m` 높은 위치를
`map` frame으로 발행한다.

### 7.4 토마토

| 속성 | 값 |
|---|---:|
| rigid body | dynamic (`kinematic=False`) |
| collider | 몸통 mesh의 `convexHull` |
| density | `1000 kg/m³` |
| static friction | `1.2` |
| dynamic friction | `1.0` |
| calyx | 시각 장식, collider 없음 |
| 색/종류 | ripe asset 중 고정 seed로 선택 |

토마토마다 동일 질량을 강제하지 않고 collider 부피 × 밀도로 질량을 계산한다.
에셋 변형별 부피 차이를 반영하기 위한 설계다. 수확 대상 토마토의 파지용 구형
collider 반지름은 `0.034 m`지만, IW에 초기 적재되는 토마토는 몸통 mesh에
`convexHull`을 적용한다.

### 7.5 데크 정렬과 joint

- chassis world bounding box의 상면을 실측해 팔레트 바닥 z를 배치한다.
- 실측 실패 시에만 root 기준 `deck_z=0.225 m`를 사용한다.
- `Load`와 chassis는 현재 상대 pose를 유지하는 FixedJoint로 연결한다.
- 실행 중 dynamic body를 순간이동하고 같은 frame에 joint를 생성하면
  Fabric/PhysX 불일치로 크래시할 수 있으므로, 창고 팔레트 재결속 시에는 현재
  물리 pose를 유지하고 위치 오차만 검사한다.
- 창고 팔레트 결속 허용 오차: z `0.025 m`, xy `0.15 m`.

## 8. 창고 도킹·지게차 인수인계

### 8.1 canonical dock

창고 기준 도킹 위치는 `(x=0.0, y=10.84885)`이다. Nav2가 이 위치로 이동한 뒤
Isaac의 `WarehouseDockController`가 작업 중 IW를 world FixedJoint로 고정한다.

런타임 고정은 한 physics frame에 정지·순간이동·joint 생성을 모두 하지 않고
다음 3프레임 단계로 수행한다.

1. 선속도·각속도 0
2. 필요하면 canonical pose로 보정
3. 다음 physics step 이후 world FixedJoint 생성

이 순서를 합치면 PhysX/Fabric native pointer 불일치로 segmentation fault가 날 수
있으므로 머지 시 단일 함수 호출로 축약하지 않는다.

고정 해제 시 world joint prim을 제거해 IW를 다시 이동 가능하게 만든다.

### 8.2 데크 geometry 발행

Isaac이 `/iwhub_0/deck_geometry`에 다음 JSON을 발행한다.

```json
{
  "dock_x": 0.0,
  "dock_y": 10.84885,
  "deck_top_z": "<실측값>",
  "pallet_hole_center_z": "<계산값>",
  "pallet_support_clearance": 0.002
}
```

`deck_top_z`는 chassis rigid-body world bbox 상면에서 측정한다. 팔레트 포크 채널:

- local bottom z: `0.02053 m`
- local top z: `0.11605 m`
- local center z: `0.06829 m`
- deck support clearance: `0.002 m`

지게차 제어기는 이 값을 받아 포크 높이를 맞춘다. 고정된 추정 높이로 되돌리면
에셋 원점이나 차체 정착 높이가 바뀔 때 포크가 팔레트에 충돌할 수 있다.

### 8.3 팔레트 소유권

동시에 포크 joint와 데크 joint가 한 팔레트를 잡지 않도록 소유권을 한 simulation
update에서 전환한다.

관련 joint:

| 용도 | prim |
|---|---|
| IW world lock | `/World/WarehouseDockIwHubFixed` |
| pallet → IW deck | `/World/WarehouseDockPalletJoint` |
| pallet → forklift | `/World/ForkliftPalletCarryJoint` |

포크 joint가 남아 있으면 IW deck 결속을 거부한다. 팔레트는 현재 물리 pose를
유지한 채 z/xy 오차를 검사한 다음 IW chassis에 FixedJoint로 결속한다.

## 9. 실행 구성

### 9.1 Isaac

통합 실행 기준:

```bash
cd isaacpjt
isaac_python main.py --mm --iw --fork --nav --camera
```

IW 관점에서 필요한 기능:

- `--iw`: IW articulation, cargo, joint bridge
- nav odom/scan 옵션: chassis odom, 전·후방 scan graph
- ROS joint bridge: `/iwhub_0/joint_command` ↔ joint states

`isaacpjt/iw.py` 또는 `isaacpjt/robots/iwhub.py`의 씬 생성·물리 설정 변경은 Isaac
Sim을 완전히 재시작해야 반영된다.

### 9.2 ROS 통합

```bash
ros2 launch harvest_vision smartfarm_integration.launch.py
```

통합 launch는 IW Nav2 launch, mission node, forklift node를 함께 시작한다.
IW RViz는 `iw_rviz` launch argument로 제어한다.

### 9.3 단독 진단

```bash
# base/joint/바퀴 odom 진단
ros2 launch iwhub_control iwhub_base.launch.py

# IW 전체 Nav2
ros2 launch iwhub_control iwhub_nav2.launch.py
```

현재 패키지는 symlink-install을 전제로 Python과 YAML 변경이 재시작만으로 반영되는
구성이지만, 새 console script나 package metadata 변경 후에는 설치 상태를 확인한다.

## 10. 현재 검증 상태와 알려진 주의점

### 확인된 동작

- IW 전용 map 발행 및 Nav2 namespace 분리
- 전·후방 라이다와 filtered scan
- 접근 방향 기반 FOLLOW로 베드 반대편 비접촉 정차
- MM 수확 후 앞쪽 빈 KLT release pose 전달
- MM의 `BASKET_APPROACH → BASKET_PLACE → RELEASE → RETRACT`
- chassis deck geometry 발행
- Python 문법 검사와 deck geometry 테스트 5개

### 수정 후 재검증할 항목

- 8개 KLT 전체 collider 적용 후 플레이스 토마토 실제 안착
- IW 주행 중 새로 플레이스한 토마토 유지
- 팔레트/KLT collider 증가에 따른 PhysX 성능
- 지게차 상하차 전체 회귀

### 미구현 또는 잔여 위험

- cmd_vel 비영(0)이지만 odom 이동이 거의 없는 상태를 감지해 후진·재계획하는
  stuck recovery watchdog은 미구현
- 거리별 강제 감속·정지를 담당하는 Nav2 Collision Monitor는 미구현
- 동시 기동 시 FastDDS lifecycle 응답 유실은 지연·재시도로 완화했지만 MM Nav2와
  IW navigation 전체를 포괄하는 근본 해결(예: Discovery Server)은 미적용
- RViz가 Isaac과 GPU/OpenGL 자원을 경쟁해 기동 실패할 가능성은 별도 문제
- 연속 플레이스는 실제 점유 상태를 갱신하지 않고 초기 빈 슬롯 후보를 반복 사용함

### 현재 코드에서 발견되는 비교 포인트

`smartfarm_integration.launch.py`는 mission node에 과거 파라미터
`follow_offset_x/y`를 아직 전달한다. 현재 mission node의 실제 동작 파라미터는
`dock_standoff=1.2`이며 과거 offset은 도킹 계산에 사용되지 않는다. 다른 브랜치와
머지할 때 이 launch override를 신규 `dock_standoff` 인자로 정리할 필요가 있다.

`src/smartfarm/INTERFACES.md` 일부 설명은 “IW Nav2가 아직 없음”이라고 적힌 과거
상태일 수 있다. 비교 기준은 현재 실행 코드와 이 문서이며, 인터페이스 문서는 별도
동기화 대상으로 본다.

## 11. 머지 우선순위

충돌 시 다음 동작은 현재 브랜치 구현을 우선한다.

1. `mission_nav_node.py`의 `IDLE` 초기 상태와 접근 방향 기반 `dock_standoff`
2. local/global footprint와 filtered scan 토픽
3. scan self-filter 노드, entry point, launch 연결
4. localization/navigation 지연 기동과 lifecycle 재시도
5. 팔레트 `convexDecomposition`과 chassis FixedJoint
6. 8개 KLT 전체의 `convexDecomposition` collider
7. 토마토 독립 dynamic rigid body·밀도·마찰 설정
8. 앞쪽 빈 KLT 슬롯 선택과 release pose 발행

다른 브랜치가 IW 차체 USD, 라이다 위치, 팔레트 크기 또는 KLT 배열을 바꿨다면 값을
그대로 덮어쓰기보다 다음 항목을 새 형상에 맞춰 함께 재계산한다.

- front/back lidar TF
- local/global footprint
- scan self-filter 범위
- KLT 슬롯 좌표와 빈 슬롯 선택
- chassis deck top과 팔레트 support 높이
- 도킹 standoff 하한과 MM 팔 도달 상한

## 12. 머지 후 검증 체크리스트

- [ ] IW prim root, DOF 이름, 초기 pose/yaw가 일치하는가
- [ ] wheel radius/separation과 Isaac drive torque·마찰 설정이 보존되는가
- [ ] cmd timeout과 정지 deadband가 동작하는가
- [ ] 통합 모드에서 wheel odom과 Isaac chassis odom이 중복 발행되지 않는가
- [ ] map→odom→base_link→chassis→lidar TF가 단일 parent로 연결되는가
- [ ] `/iwhub_0/map`이 발행되고 map_server·AMCL이 active인가
- [ ] 전·후방 filtered scan에 팔레트/KLT 자기반사가 제거되는가
- [ ] 실제 외부 장애물은 filtered scan과 costmap에 남는가
- [ ] local/global footprint가 RViz에서 라이다·적재물 외곽을 포함하는가
- [ ] 초기 상태에서 IW가 움직이지 않고 `FOLLOW` 후에만 출발하는가
- [ ] IW가 베드 반대편 접근 방향에서 MM과 비접촉 정차하는가
- [ ] DWB가 장애물 근처에서 유효한 회피 궤적을 선택하는가
- [ ] 팔레트 포크 슬롯이 collider에서 열려 있는가
- [ ] 팔레트+KLT Load가 주행 중 chassis를 안정적으로 따라가는가
- [ ] 8개 KLT 모두 토마토를 관통시키지 않는가
- [ ] 플레이스한 토마토가 KLT 내부에 안착하고 IW 주행 중 유지되는가
- [ ] `/iwhub_0/deck_geometry`가 유효한 실측 높이를 발행하는가
- [ ] canonical dock 고정이 3 physics frame 순서로 수행되는가
- [ ] 지게차 인수 시 deck joint 해제와 팔레트 재결속이 가능한가
- [ ] 포크 joint와 deck joint가 같은 팔레트를 동시에 잡지 않는가
