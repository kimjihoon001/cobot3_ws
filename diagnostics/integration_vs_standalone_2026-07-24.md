# smartfarm_integration vs 단독 실행 비교

- 통합 실행: `ros2 launch harvest_vision smartfarm_integration.launch.py`
- 통합 ROS launch PID: 46452
- 통합 Isaac: `main.py --mm --iw --fork --nav --camera`
- 통합 launch 로그:
  `/home/rokey/.ros/log/2026-07-24-21-25-09-204307-IsaacSim03-46452/launch.log`
- 비교 기준:
  `diagnostics/standalone_auto_nav_harvest_baseline_2026-07-24.md`

## 결정적 차이

Nav2 목표와 베드 관찰 방향은 거의 같지만, 비전이 선택하고 시뮬 GT에 매칭한
토마토가 완전히 다르다.

| 항목 | 단독 | 통합 |
|---|---:|---:|
| Nav goal | (-0.54, -8.19, 1.91) | (-0.54, -8.19, 1.91) |
| bed axis | -35.6 deg | -36.1 deg |
| camera heading | 54.4 deg | 53.9 deg |
| vision target | (0.793, 0.670, 1.095) | (0.639, 0.680, 0.430) |
| sim tomato | (0.810, 0.689, 1.091) | (0.656, 0.703, 0.420) |
| APPROACH | (0.867, 0.623, 0.917) | (0.719, 0.644, 0.246) |
| PREGRASP | (0.838, 0.657, 1.006) | (0.687, 0.674, 0.335) |
| GRASP | (0.810, 0.689, 1.091) | (0.656, 0.703, 0.420) |

통합 APPROACH의 Z는 단독보다 0.671 m 낮다. MoveIt은 이 목표를 오류로
판단하지 않고 정상 계획·실행했기 때문에 팔의 큰 하강/회전은 컨트롤러 오작동이 아니라
잘못 선택된 저위치 목표를 충실히 추종한 결과다.

## 시간 관계

```text
1784895972.043  vision→sim tomato 매칭
1784895972.044  APPROACH (0.719, 0.644, 0.246) 생성
1784895972.046  coordinator가 IW FOLLOW 발행
1784895978.081  IW FOLLOW Nav2 goal 수락
1784895980.167  arm_controller가 잘못된 APPROACH 실행 시작
1784896003.618  APPROACH trajectory SUCCEEDED
```

IW FOLLOW는 잘못된 팔 목표가 생성된 뒤 시작됐다. 따라서 최초 이상 팔 동작의
직접 원인은 IW 이동이나 바스켓 적재가 아니라 토마토 목표 선택이다. 다만 팔 동작 중
IW를 움직이는 정책은 별도의 안전 문제다.

## 추가 이상

- 통합에서 `/iw/basket/empty_slot_pose`가 `map` frame으로 들어오지만
  manipulator의 TF buffer에서 `map -> base_link`가 반복 실패한다.
- 이후 바스켓 목표가 작업영역 밖이라는 경고가 고주기로 반복된다.
- 잘못된 APPROACH도 workspace 최소 Z=0.15 m보다 높아서 현재 안전검사를 통과한다.
- 통합 APPROACH는 OMPL 계획 약 8초 + 실행 약 23.45초가 걸려 설정된
  `motion_timeout_sec=30`과 거의 같거나 초과한다.

## 1차 원인 판정

`vision_node`가 통합 장면에서 낮은 토마토를 선택했고,
`use_sim_ground_truth=true`가 이를 `(0.656, 0.703, 0.420)` 시뮬 과실에 매칭했다.
이후 `_scoop_insertion_axis()`가 안전점을 Z=0.246 m로 만들었고 MoveIt이 그대로 실행했다.

통합 장면에서 낮은 과실을 선택한 이유는 다음 후보를 추가 분리해야 한다.

1. 다른 로봇/IW가 추가된 렌더 장면에서 detector의 선택 대상이 달라짐
2. 낮은 과실이 원래 존재하거나 물리 정착 중 아래로 이동함
3. 단독과 통합의 약 0.5 deg 정차/카메라 방향 차이로 중앙 표적 선택이 바뀜
4. 통합 부하로 카메라/TF 시각이 지연되어 다른 프레임의 목표가 선택됨
