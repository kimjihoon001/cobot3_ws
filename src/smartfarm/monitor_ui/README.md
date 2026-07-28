# monitor_ui — CCTV · 수확현황 · 도매시세 모바일 관제 화면

수확 → 인계 → 적재 전 공정을 한 화면에서 본다. 영상은 HTTP MJPEG(8080),
상태는 WebSocket JSON(9090)으로 채널을 나눴다. 이미지를 rosbridge로 보내면
대역폭이 감당이 안 되기 때문이다.

## 실행

처음 한 번만:

```bash
# 1) ROS 의존 패키지 (기본 desktop 설치에는 없다)
#    $ROS_DISTRO를 쓰므로 humble/jazzy 어느 쪽이든 그대로 붙여넣으면 된다.
source /opt/ros/humble/setup.bash    # 또는 /opt/ros/jazzy/setup.bash
sudo apt update && sudo apt install -y \
  ros-$ROS_DISTRO-web-video-server \
  ros-$ROS_DISTRO-compressed-image-transport \
  ros-$ROS_DISTRO-rosbridge-server \
  ros-$ROS_DISTRO-rosbag2-storage-mcap

# 2) 빌드 + 웹 의존성
cd ~/cobot3_ws && colcon build --packages-select monitor_ui
cd ~/cobot3_ws/src/smartfarm/monitor_ui/web && npm install
```

apt 목록 대신
`cd ~/cobot3_ws && rosdep install --from-paths src --ignore-src -y --rosdistro $ROS_DISTRO`
로 한 번에 받아도 된다(`package.xml`에 전부 선언돼 있다).

빠뜨렸을 때 나오는 증상:

| 빠진 것 | 증상 |
|---|---|
| `web_video_server` | `ros2 launch`가 `package 'web_video_server' not found`로 즉사 |
| `compressed_image_transport` | launch는 뜨는데 `republish`가 JPEG를 못 만들어 전 화면 NO SIGNAL |
| `rosbag2-storage-mcap` | 실행은 되는데 `R`(녹화)만 실패한다 |
| `npm install` | `npm run dev`가 `> vite`만 찍고 멈춘 것처럼 보인다 |

웹 쪽은 Node 18 이상이면 된다(vite 6). 확인은 `node -v`.

그다음부터는 터미널 3개.

```bash
# 1) Isaac — --cctv 를 빼면 고정 카메라가 안 뜬다
python isaacpjt/main.py --cctv --mm --iw --fork

# 2) UI 스택 (영상 + rosbridge + 상태 집계 + 녹화 제어)
# Jazzy 워크스테이션: 로컬 YOLO 토픽 사용
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=109
source ~/cobot3_ws/install/setup.bash
ros2 launch monitor_ui ui.launch.py

# 3) 화면
cd ~/cobot3_ws/src/smartfarm/monitor_ui/web && npm run dev
```

그다음 <http://localhost:5173>.

휴대폰에서 볼 때는 같은 Wi-Fi에 연결한 뒤 Vite를 외부 접속 가능하게 띄운다.

```bash
cd ~/cobot3_ws/src/smartfarm/monitor_ui/web
npm run dev -- --host 0.0.0.0
# 휴대폰: http://<PC_IP>:5173
```

하단 탭은 `CCTV / 수확현황 / 시세·추천`으로 구성된다. 수확현황은
`/harvester_0/vision/tomato_detections`와 매니퓰레이터 상태에서 집계한다.
도매시세는 공공데이터포털의 온라인 도매시장 거래정보 API를 사용하며,
브라우저에 인증키가 노출되지 않도록 `market_price_node`가 호출한다.

```bash
export DATA_GO_KR_SERVICE_KEY='공공데이터포털에서_발급받은_키'
ros2 launch monitor_ui ui.launch.py
```

키가 없거나 해당 날짜에 토마토 거래가 없으면 임의 가격 대신 `연동 대기`가
표시된다. 첫 연동 시 실제 API 응답의 가격 단위·필드가 신청한 명세와 같은지
확인한 뒤 운영에 사용한다.

주소창 없이 띄우려면:

```bash
google-chrome --start-fullscreen --app=http://localhost:5173
```

Isaac, 비전 노드, UI를 포함한 모든 프로세스는 `ROS_DOMAIN_ID=108`에서 직접
통신한다. 별도의 Docker/domain bridge는 사용하지 않는다. UI를 실행하는
터미널에서도 `export ROS_DOMAIN_ID=108`을 적용해야 YOLO 결과와 CCTV 토픽을
받을 수 있다.

`--moveit`으로 띄웠으면 카메라 네임스페이스가 다르다:

```bash
ros2 launch monitor_ui ui.launch.py camera_ns:=harvester_moveit
```

CAM-01은 OpenCV 디버깅창과 같은 YOLO 박스 영상
`/harvester_0/vision/annotated_image`를 표시한다. namespace 없이
`harvest_full.launch.py`를 단독 실행해 디버깅창을 띄운 경우에는 토픽이
`/vision/annotated_image`이므로 입력을 명시한다:

```bash
ros2 launch monitor_ui ui.launch.py \
  mm_front_topic:=/vision/annotated_image
```

영상 배선만 따로 확인하려면 `stream.launch.py`를 직접 띄운다.

## 화면

```
┌──────────────────────┬──────────────────────┐
│ CAM-01 MM D455/YOLO  │ CAM-02 GREENHOUSE    │
├──────────────────────┼──────────────────────┤
│ CAM-03 UNLOADING     │ CAM-04 STORAGE       │
└──────────────────────┴──────────────────────┘
      CAM-05 OVERVIEW — 격자 밖, 5 키로만
```

## 단축키

| 키 | 동작 |
|---|---|
| `1`~`4` | 해당 카메라 확대 (더블클릭도 동일) |
| `5` | 전체 조감도 |
| `0` / `ESC` | 2×2 복귀 |
| `R` | 녹화 시작·정지 |
| `F` | 브라우저 전체화면 |
| `H` | HUD 숨김 (이름·LIVE만 남김) |
| `D` | 디버그 — 수신 Hz, 로봇 좌표 |

## 녹화

REC 버튼 또는 `R`. 저장 위치는 `~/bags/monitor_YYYYMMDD_HHMMSS`, MCAP 포맷,
5분 단위 분할.

- **초당 약 5MB = 시간당 19GB.** 켜둔 채 잊으면 하룻밤에 150GB가 쌓인다.
- 정지에 1~2초 걸린다. rosbag2가 파일을 닫는 중이며, SIGKILL로 죽이면
  기록이 깨져서 SIGINT를 보내고 기다린다.
- 결과물은 **영상 파일이 아니라 토픽 기록**이다. 보려면 `ros2 bag play`로
  틀면 이 화면이 그대로 재생기가 된다(시계가 `/clock` 기준이라 시각도
  녹화 당시로 돌아간다). 정밀 분석은 Foxglove로 열면 된다.

발표용 mp4가 필요하면 녹화 대신 MJPEG를 직접 받는다:

```bash
ffmpeg -i "http://localhost:8080/stream?topic=/ui/overview&type=ros_compressed" \
  -c:v libx264 demo.mp4
```

## 구성

| 노드 | 역할 |
|---|---|
| `republish` ×5 | Isaac raw RGB → JPEG. 압축본을 서빙과 녹화가 공유한다 |
| `web_video_server` | :8080 MJPEG |
| `rosbridge_websocket` | :9090 |
| `ui_status_node` | 흩어진 상태를 `/ui/status` 하나로 5Hz JSON 집계 |
| `recorder_node` | `/recording/start`·`/stop` 서비스, `ros2 bag record` 구동 |

`/ui/status`는 `std_msgs/String`에 JSON을 담는다. 커스텀 `.msg`를 안 쓰는
이유는 UI 피드가 필드가 계속 바뀌는 자리라, 하나 추가할 때마다 인터페이스
패키지를 재빌드하는 비용이 더 크기 때문이다.

## 막혔을 때

**화면이 전부 NO SIGNAL** — Isaac에 `--cctv`가 빠졌거나 Isaac과 UI의
`ROS_DOMAIN_ID`가 108로 일치하지 않는다.
`ros2 topic list | grep cctv`로 확인한다.

**토픽은 있는데 그림이 안 나옴** — 스트림 URL에 `type=ros_compressed`가
필요하다. 없으면 web_video_server가 raw 토픽을 찾다가 프레임을 하나도
못 내보낸다(경계 헤더만 나온다).

**CAM-01만 OFFLINE** — MM을 안 띄웠거나 `camera_ns`가 다르거나 Isaac을
`--no-camera`로 실행한 경우다. 디버깅창에는 영상이 보이는데 UI만 OFFLINE이면
`ros2 topic list | grep annotated_image`로 실제 토픽 이름을 확인하고 위처럼
`mm_front_topic`을 맞춘다.

## 카메라 위치 조정

`isaacpjt/scene/monitor_cams.py`의 `eye`/`target`. 좌표는 온실·창고 치수와
`RACK_DEPTH`에서 유도하므로 씬 크기를 바꿔도 따라온다. 바꾸면 Isaac을
재시작해야 반영된다.
