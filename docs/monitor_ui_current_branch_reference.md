# 모니터링 UI 구현 기준 스냅샷

이 문서는 4분할 CCTV 관제 화면의 데이터 흐름·카메라 배치·설계 판단을 정리한
비교 기준이다. 실행 절차와 단축키는 `src/smartfarm/monitor_ui/README.md`에 있고,
여기서는 **왜 그렇게 됐는지**를 남긴다.

- 패키지: `src/smartfarm/monitor_ui`
- Isaac 쪽: `isaacpjt/scene/monitor_cams.py`, `ros/robot_bridge.build_rgb_camera`
- 진입 플래그: `main.py --cctv`
- 도메인: 108
- 브랜치: `cctv-ui` (`harvest/rmp-mm-suction` 병합본 위)

## 1. 데이터 흐름

```text
Isaac 고정 카메라 4대 + MM D455
  └─ /cctv/{greenhouse,unloading,storage,overview}, /harvester_0/rgb  (raw)
       └─ image_transport republish ×5
            └─ /ui/<pane>/compressed  (JPEG)
                 ├─ web_video_server :8080  → <img> MJPEG
                 └─ rosbag2 (MCAP)          → 같은 압축본을 재인코딩 없이 기록

로봇 상태 토픽 (String·PoseStamped·Odometry)
  └─ ui_status_node
       └─ /ui/status  std_msgs/String(JSON) 5Hz
            └─ rosbridge :9090 → roslibjs → React

REC 버튼
  └─ /recording/start·stop (Trigger)
       └─ recorder_node → `ros2 bag record` 서브프로세스
            └─ /recording/status  std_msgs/String(JSON)
```

**영상은 HTTP, 데이터는 WebSocket으로 채널을 나눈 것이 이 설계의 축이다.**
이미지를 rosbridge로 보내면 JSON base64로 부풀어 대역폭이 감당이 안 된다.

## 2. 카메라 배치

장소별 CCTV가 아니라 공정 순서대로 놓는다. 왼쪽 위에서 오른쪽 아래로
검출 → 수확·전달 → 인계 → 적재로 읽힌다.

| pane | 토픽 | 위치 → 대상 | 부감각 |
|---|---|---|---|
| CAM-01 MM D455 / YOLO | `/harvester_0/rgb` | 로봇 장착 | — |
| CAM-02 GREENHOUSE | `/cctv/greenhouse` | (6.05, −4.0, 4.2) → (0, 0, 1.0) | 24° |
| CAM-03 UNLOADING | `/cctv/unloading` | (3.0, 15.5, 3.8) → (0, 11.5, 0.8) | 31° |
| CAM-04 STORAGE | `/cctv/storage` | (5.5, 15.0, 3.9) → (0, 20.4, 1.2) | 19° |
| CAM-05 OVERVIEW | `/cctv/overview` | ortho, 격자 밖 (`5` 키) | — |

좌표는 온실·창고 치수와 `scene.warehouse.RACK_DEPTH`에서 유도하므로 씬 크기를
바꾸면 따라온다. 하드코딩된 값은 `IW_DOCK_Y = 10.85` 하나이며,
`iwhub_control/lanes.py`의 `DOCK`과 같아야 한다.

### CAM-03이 창고 안에 있는 이유

인계 지점이 벽으로 갈려 있다.

```text
온실  Y −13 ~ 13     IW 도크 (0, 10.85)   ← 온실 쪽
   ─── 벽 Y=13, 입구 폭 4.8m ───
창고  Y 13 ~ 21      지게차 대기 (0.3, 14.5)  ← 창고 안
```

온실 쪽에 두면 지게차가, 창고 안쪽 깊이 두면 IW가 안 보인다. 창고 안에서
입구를 통해 비스듬히 보는 구도만 둘을 한 화면에 담는다. 시선이 Y=13을
x≈1.1에서 통과하므로 입구 폭(±2.4m) 안쪽이다. **카메라 X를 키우면 벽에 가린다.**

### 부감각과 커버리지의 상충

온실 높이가 4.5m뿐이라, 먼 모서리에 두면 내려보는 각이 12°까지 떨어져 화면이
평면처럼 보인다. 25~40°를 맞추려면 대상에서 5m 이내로 붙여야 하고 그만큼
전경을 포기한다. CAM-02는 "MM과 IW가 만나는 지점"이 주인공이라 붙이는 쪽을
택했다.

## 3. 설계 판단

**RGB 전용 헬퍼** — `build_camera()`는 같은 렌더프로덕트에 depth·camera_info
헬퍼까지 붙인다. 사람이 보기만 하는 화면이라 `build_rgb_camera()`로 depth 렌더를
뺐다. `nodeNamespace`를 비우면 절대 토픽이 그대로 나간다(GPU 실측 확인).

**해상도 640×360** — 화면 칸이 16:9라 4:3으로 뽑으면 좌우에 검은 여백이 크게
남는다. 세로 aperture가 해상도 비율을 따라가므로 화각도 같이 줄어 벽 위 빈
공간이 잘리고, 렌더 픽셀은 25% 준다.

**조감도는 격자 밖** — 4화면이 전부 클로즈업이면 "지금 전체가 어디쯤인가"가
사라진다. `5` 키로만 꺼내며, 안 볼 때는 `<img>` 자체를 안 붙여 스트림 하나를
아낀다. ortho aperture는 월드 단위의 10배로 적고(USD 규약), 긴 축(Y)이 화면
가로로 오도록 `up_hint=(−1,0,0)`으로 90° 굴린다.

**`/ui/status`가 `std_msgs/String` + JSON인 이유** — 커스텀 `.msg`가 정석이지만
UI 피드는 필드가 계속 바뀌는 자리라, 하나 추가할 때마다 인터페이스 패키지를
재빌드하는 비용이 더 크다. React는 구독 하나에 `JSON.parse` 한 번이면 끝난다.

**카메라 Hz는 압축 토픽 도착 간격으로 실측한다** — 디버깅에서 가장 자주 필요한
질문이 "이 스트림이 아직 살아있나"라서 화면 판정의 근거로 쓴다. 하드코딩한
fps 값을 쓰면 그 판정이 무의미해진다.

**데모용·디버깅용 화면을 나누지 않는다** — `showDebug` 토글 하나로 처리한다.
두 벌을 만들면 한쪽이 반드시 뒤처진다.

**Demo Mode를 만들지 않는다** — Isaac이 진짜 데모다. 가짜 상태머신을 두면
발표 중 "화면은 수확 중인데 로봇은 가만히 있는" 상황이 나온다.

## 4. 밟은 함정

**`republish`의 `out` remap이 안 먹는다** — 베이스 이름만 remap하면 그대로
`/out`으로 발행된다. transport 접미사까지 붙여 `out/compressed`를 걸어야 한다.

**스트림 URL에 `type=ros_compressed`가 필수** — 기본 type은 raw 토픽을 찾는다.
`/ui/*`에는 압축본만 있어서, 없으면 경계 헤더만 나오고 **프레임이 하나도 안
나온다**. 이 값을 주면 republish가 만든 JPEG가 재인코딩 없이 그대로 나간다.

**영상 표시를 상태 피드에 묶지 말 것** — 초기 구현은 `hz > 0`일 때만 `<img>`를
걸어서, `ui_status_node`가 죽으면 영상이 멀쩡해도 4칸이 전부 사라졌다.

**MJPEG는 끊기면 `<img>`가 스스로 재연결하지 않는다** — 3초마다 URL의 값을
올려 새로 붙인다.

**녹화 정지는 SIGINT** — SIGKILL로 죽이면 rosbag2가 파일을 못 닫아 기록이
깨진다. 그래서 정지에 1~2초 걸린다.

**헤드리스 브라우저로 화면 확인이 안 된다** — MJPEG 스트림이 끝나지 않아
페이지 로드가 완료로 안 잡힌다. `--dump-dom`·`--screenshot`은 멈추고,
CDP로 붙어 로드 완료와 무관하게 캡처해야 한다.

**`ko-KR` 시각 포맷** — `toLocaleTimeString("ko-KR")`은 "16시 45분 36초"로
뽑는다. 관제 화면에는 `sv-SE`(`16:45:36`)를 쓴다.

**CSS `alertPulse`는 `outline-color`만 애니메이션한다** — 텍스트·버튼에는
안 먹는다. opacity를 쓰는 `textPulse`가 따로 있다.

## 5. 녹화

- 저장: `~/bags/monitor_YYYYMMDD_HHMMSS`, MCAP, 5분 단위 분할
- 실측 **5.4 MB/s ≈ 시간당 19GB** (카메라 5대 + 상태). raw로 담으면 시간당 180GB
- `-a` 금지. 압축 토픽과 상태 토픽만 명시한다
- zstd 압축은 켜지 않는다 — 내용물이 이미 JPEG라 CPU만 먹는다
- 검증: 20초 녹화 후 `ros2 bag info` 정상 판독, 2936 메시지

UI가 토픽 구독자일 뿐이라 `ros2 bag play`로 틀면 같은 화면이 재생기가 된다.
시계를 `/clock` 기준(`use_sim_time`)으로 읽으므로 시각도 녹화 당시로 돌아간다.
`recorder_node`만 `use_sim_time`을 주지 않는다 — 경과시간과 파일명은 벽시계로
세야 시뮬이 멈췄을 때 녹화가 멈춘 것처럼 보이지 않는다.

## 6. 남은 작업

- **CAM-03 구도** — 인계 동작이 실제로 도는 것을 보고 조정. 지게차 대기 자세만
  본 상태라 IW가 들어왔을 때 화면이 어떻게 채워지는지 미확인
- **CAM-02 위치** — MM이 토마토를 IW 팔레트에 넣는 지점 좌표가 정해지면 이동
- **YOLO 바운딩 박스 오버레이** — `vision_node`가 `vision/tomato_detections`를
  실제로 발행하므로 진짜 데이터로 붙일 수 있다. `vision/annotated_image`(박스가
  그려진 영상)를 pane에 직접 띄우는 더 싼 선택지도 있다(색·스타일 제어는 포기)
- **로봇 상태 오버레이** — 지금 `/forklift/status`·`/iw/status`는 자유 텍스트라
  enum으로 쓰려면 로봇 노드가 구조화된 상태를 발행하도록 고쳐야 한다
- **`src/web_video_server/`** — apt 바이너리와 중복인 소스 클론. 워크스페이스
  오버레이가 이기며 매 빌드에 낀다. 지워도 무방하다
