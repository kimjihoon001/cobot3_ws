import { useEffect, useState } from "react";

import {
  CameraSpec,
  RobotState,
  StreamState,
  VIDEO_HOST,
  cameraHealth,
} from "../types";

const RECONNECT_DELAY_MS = 3000;

interface Props {
  cam: CameraSpec;
  hz: number;
  robot?: RobotState;
  hud: boolean;
  debug: boolean;
  focused: boolean;
  clock: string;
  onToggleFocus: () => void;
}

export default function CameraTile({
  cam,
  hz,
  robot,
  hud,
  debug,
  focused,
  clock,
  onToggleFocus,
}: Props) {
  const [stream, setStream] = useState<StreamState>("loading");
  // src를 바꿔야 <img>가 다시 붙는다. MJPEG는 한 번 끊기면 스스로 재연결하지
  // 않아서, 재시도할 때마다 이 값을 올려 URL을 새로 만든다.
  const [attempt, setAttempt] = useState(0);
  const [lastOk, setLastOk] = useState<string | null>(null);

  const markPlaying = () => {
    setStream("playing");
    setLastOk(new Date().toLocaleTimeString("ko-KR"));
  };

  // 프레임이 실제로 도착하는지는 ROS 쪽 실측 Hz가 가장 확실한 근거다.
  useEffect(() => {
    if (hz > 0) {
      markPlaying();
    }
  }, [hz]);

  useEffect(() => {
    if (stream !== "error" && stream !== "reconnecting") return;
    const timer = window.setTimeout(() => {
      setStream("reconnecting");
      setAttempt((n) => n + 1);
    }, RECONNECT_DELAY_MS);
    return () => window.clearTimeout(timer);
  }, [stream, attempt]);

  const health = cameraHealth(hz, stream);
  const live = health === "online" || health === "warning";
  // type을 안 주면 web_video_server가 raw 토픽을 찾다가 프레임을 하나도 못
  // 내보낸다. ros_compressed는 republish가 만든 JPEG를 재인코딩 없이 흘린다.
  const src =
    `${VIDEO_HOST}/stream?topic=/ui/${cam.key}` +
    `&type=ros_compressed&_=${attempt}`;

  return (
    <div
      className={`pane pane-${health}${focused ? " focused" : ""}`}
      onDoubleClick={onToggleFocus}
    >
      {/* 영상은 상태 피드와 무관하게 항상 건다. ui_status_node나 rosbridge가
          죽어도 그림은 계속 나와야 한다. */}
      <img
        className={`pane-video${cam.key === "mm_front" ? " pane-video-fill" : ""}`}
        src={src}
        alt={cam.name}
        // MJPEG 첫 프레임이 브라우저에 실제로 로드되면 rosbridge의 Hz 상태가
        // 아직 없더라도 연결 성공이다. 영상은 HTTP, 상태는 WebSocket이라
        // 어느 한쪽의 지연이 다른 쪽 표시를 가리면 안 된다.
        onLoad={markPlaying}
        onError={() => setStream("error")}
      />

      {!live && (
        <div className="pane-nosignal">
          <div className="ns-title">
            {stream === "loading" ? "CONNECTING..." : "NO SIGNAL"}
          </div>
          <div className="ns-line">
            {cam.id} {cam.name}
          </div>
          {lastOk && <div className="ns-line">LAST CONNECTION {lastOk}</div>}
          {stream === "reconnecting" && (
            <div className="ns-line blink">RECONNECTING...</div>
          )}
        </div>
      )}

      <div className="pane-top">
        <span className="pane-label">
          {cam.id} {cam.name}
        </span>
        <span className={`badge badge-${health}`}>
          ● {live ? "LIVE" : health === "error" ? "ERROR" : "OFFLINE"}
        </span>
      </div>

      {hud && (
        <div className="pane-bottom">
          <span className="pane-state">
            {robot?.state || "—"}
            {debug && robot?.x != null && (
              <span className="pane-debug">
                {" "}
                x {robot.x.toFixed(2)} y {robot.y!.toFixed(2)} θ{" "}
                {((robot.yaw! * 180) / Math.PI).toFixed(0)}°
              </span>
            )}
          </span>
          <span className="pane-meta">
            {hz > 0 && `${hz.toFixed(0)} FPS`}
            <span className="pane-time">{clock}</span>
          </span>
        </div>
      )}
    </div>
  );
}
