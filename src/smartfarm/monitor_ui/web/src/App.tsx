import { useCallback, useEffect, useRef, useState } from "react";
import ROSLIB from "roslib";

import CameraTile from "./components/CameraTile";
import BottomNav from "./components/BottomNav";
import EventTicker from "./components/EventTicker";
import HarvestPanel from "./components/HarvestPanel";
import MarketPanel from "./components/MarketPanel";
import TopStatusBar from "./components/TopStatusBar";
import { AppTab, CAMERAS, ROSBRIDGE_URL, RecordingStatus, UiStatus } from "./types";

const STORE_KEY = "monitor-ui-prefs";

/** 초 단위 시뮬 시각을 HH:MM:SS로. 녹화본을 재생해도 이 값은 그대로 맞는다. */
function formatSimTime(seconds: number): string {
  const s = Math.max(0, Math.floor(seconds));
  const pad = (n: number) => String(n).padStart(2, "0");
  return `${pad(Math.floor(s / 3600))}:${pad(Math.floor(s / 60) % 60)}:${pad(s % 60)}`;
}

function loadPrefs(): { hud: boolean; focused: string | null } {
  try {
    return { hud: true, focused: null, ...JSON.parse(localStorage.getItem(STORE_KEY) || "{}") };
  } catch {
    // 저장값이 깨져 있어도 화면은 떠야 한다.
    return { hud: true, focused: null };
  }
}

export default function App() {
  const [status, setStatus] = useState<UiStatus | null>(null);
  const [rec, setRec] = useState<RecordingStatus | null>(null);
  const [connected, setConnected] = useState(false);
  const [now, setNow] = useState(new Date());
  // 서비스 호출에 필요해서 연결을 밖으로 들고 있는다.
  const rosRef = useRef<any>(null);

  const prefs = loadPrefs();
  const [hud, setHud] = useState(prefs.hud);
  const [focused, setFocused] = useState<string | null>(prefs.focused);
  const [debug, setDebug] = useState(false);
  const [activeTab, setActiveTab] = useState<AppTab>("cctv");

  // 발표 설정만 남긴다. 로봇 상태나 감지 결과는 저장하지 않는다.
  useEffect(() => {
    localStorage.setItem(STORE_KEY, JSON.stringify({ hud, focused }));
  }, [hud, focused]);

  useEffect(() => {
    const timer = window.setInterval(() => setNow(new Date()), 1000);
    return () => window.clearInterval(timer);
  }, []);

  const toggleFullscreen = useCallback(async () => {
    try {
      if (!document.fullscreenElement) {
        await document.documentElement.requestFullscreen();
      } else {
        await document.exitFullscreen();
      }
    } catch {
      // 사용자 제스처 없이 호출되면 브라우저가 막는다. 화면은 그대로 둔다.
    }
  }, []);

  const toggleRecording = useCallback(() => {
    const ros = rosRef.current;
    if (!ros) return;
    const service = new ROSLIB.Service({
      ros,
      name: rec?.recording ? "/recording/stop" : "/recording/start",
      serviceType: "std_srvs/Trigger",
    });
    // 결과는 /recording/status로 다시 들어오므로 응답은 따로 안 본다.
    service.callService(new ROSLIB.ServiceRequest({}), () => {});
  }, [rec?.recording]);

  useEffect(() => {
    const onKey = (e: KeyboardEvent) => {
      const el = document.activeElement;
      if (el instanceof HTMLInputElement || el instanceof HTMLTextAreaElement) return;

      const key = e.key.toLowerCase();
      // 5는 격자에 없는 조감도. 전체 흐름을 보여줄 때만 꺼내 쓴다.
      const index = "12345".indexOf(key);
      if (index >= 0) setFocused(CAMERAS[index].id);
      else if (key === "0" || key === "escape") setFocused(null);
      else if (key === "f") void toggleFullscreen();
      else if (key === "h") setHud((v) => !v);
      else if (key === "d") setDebug((v) => !v);
      else if (key === "r") toggleRecording();
    };
    window.addEventListener("keydown", onKey);
    return () => window.removeEventListener("keydown", onKey);
  }, [toggleFullscreen, toggleRecording]);

  useEffect(() => {
    const ros = new ROSLIB.Ros({ url: ROSBRIDGE_URL });
    rosRef.current = ros;
    ros.on("connection", () => setConnected(true));
    // rosbridge가 죽어도 화면은 유지하고 상태만 바꾼다.
    ros.on("close", () => setConnected(false));
    ros.on("error", () => setConnected(false));

    const topic = new ROSLIB.Topic({
      ros,
      name: "/ui/status",
      messageType: "std_msgs/String",
    });
    topic.subscribe((msg: { data: string }) => {
      try {
        setStatus(JSON.parse(msg.data));
      } catch {
        // 잘린 메시지 하나 때문에 화면이 멈추지 않게 한다.
      }
    });

    const recTopic = new ROSLIB.Topic({
      ros,
      name: "/recording/status",
      messageType: "std_msgs/String",
    });
    recTopic.subscribe((msg: { data: string }) => {
      try {
        setRec(JSON.parse(msg.data));
      } catch {
        // 무시 — 다음 주기에 다시 온다.
      }
    });

    return () => {
      topic.unsubscribe();
      recTopic.unsubscribe();
      ros.close();
      rosRef.current = null;
    };
  }, []);

  // ko-KR은 "16시 45분 36초"로 뽑는다. 관제 화면엔 16:45:36이 맞다.
  const clock = now.toLocaleTimeString("sv-SE");
  const focusedCam = CAMERAS.find((c) => c.id === focused);
  // 조감도는 확대했을 때만 붙인다 — 안 볼 때 스트림 하나를 통째로 아낀다.
  const visible = CAMERAS.filter((c) => c.inGrid || c.id === focused);

  return (
    <div className="app">
      <TopStatusBar
        simTime={formatSimTime(status?.sim_time ?? 0)}
        wallDate={now.toLocaleDateString("sv-SE")}
        wallTime={clock}
        connected={connected}
        focusedName={focusedCam && `${focusedCam.id} ${focusedCam.name}`}
        recording={rec?.recording ?? false}
        recElapsed={formatSimTime(rec?.elapsed ?? 0)}
        onToggleRecording={toggleRecording}
      />

      {activeTab === "cctv" && (
        <main className={`grid${focused ? " is-focused" : ""}`}>
          {visible.map((cam) => (
            <CameraTile
              key={cam.id}
              cam={cam}
              hz={status?.cameras[cam.key] ?? 0}
              robot={cam.robot ? status?.robots[cam.robot] : undefined}
              hud={hud}
              debug={debug}
              focused={cam.id === focused}
              clock={clock}
              onToggleFocus={() =>
                setFocused((cur) => (cur === cam.id ? null : cam.id))
              }
            />
          ))}
        </main>
      )}
      {activeTab === "harvest" && <HarvestPanel metrics={status?.harvest} />}
      {activeTab === "market" && (
        <MarketPanel market={status?.market} harvest={status?.harvest} />
      )}

      <footer className="bar bar-bottom">
        <EventTicker events={status?.events ?? []} format={formatSimTime} />
        <span className="hint">
          1-4 확대 · 5 조감도 · 0 복귀 · R 녹화 · F 전체화면 · H HUD · D 디버그
        </span>
      </footer>
      <BottomNav active={activeTab} onChange={setActiveTab} />
    </div>
  );
}
