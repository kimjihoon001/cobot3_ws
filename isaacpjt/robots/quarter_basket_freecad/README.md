# 3중 1/4구 토마토 수확 그리퍼

FreeCAD 1.1.x용 파라메트릭 생성기다. Robotiq 2F-85는 사용하지 않는다.

- `LeftBasket`, `RightBasket`: 두 개의 내부 1/4구가 닫혀 하부 반구 바구니를 만든다.
- `OuterCutterQuarter`: 반지름이 더 큰 외부 1/4구이며 Z축으로 회전한다.
- `CutterBladeInsert`: 외부 1/4구에 장착되는 교체형 줄기 절단 날이다.
- `BaseFrame`: 로봇 플랜지와 세 회전부를 지지하는 고정부다.

## 생성

터미널 실행 파일이 등록된 경우:

```bash
cd ~/cobot3_ws/isaacpjt/robots/quarter_basket_freecad
FreeCADCmd build_quarter_basket.py
```

GUI만 설치된 경우 FreeCAD에서 `매크로 → 매크로... → 생성`으로 매크로를 만든 뒤
`build_quarter_basket.py` 내용을 실행한다.

결과는 `generated/`에 저장된다.

- `quarter_basket_harvester.FCStd`
- 전체 조립 STEP
- 부품별 STEP/STL
- `dimensions.json`

## 기본 운동

| 조인트 | 축 | 닫힘/대기 | 동작 |
|---|---|---:|---:|
| `LeftBasketHinge` | Y | 0° | +60° 열림 |
| `RightBasketHinge` | Y | 0° | -60° 열림 |
| `OuterCutterRotation` | Z | 0° | +95° 절단 |

외부 커터 1/4구의 안쪽 반지름은 59 mm이고 내부 바구니 외경과 약 3.8 mm의
간극을 둔다. 실제 제작 전에는 출력 공차와 베어링 흔들림을 포함해 간극을 다시
확인해야 한다.
