interface Props {
  simTime: string;
  wallDate: string;
  wallTime: string;
  connected: boolean;
  focusedName?: string;
  recording: boolean;
  recElapsed: string;
  onToggleRecording: () => void;
}

export default function TopStatusBar({
  simTime,
  wallDate,
  wallTime,
  connected,
  focusedName,
  recording,
  recElapsed,
  onToggleRecording,
}: Props) {
  return (
    <header className="bar bar-top">
      <span className="sys-name">SMARTFARM MONITOR</span>
      {focusedName && <span className="focus-tag">{focusedName}</span>}

      <button
        className={`rec-btn${recording ? " on" : ""}`}
        // 누른 뒤 포커스가 남으면 Enter/Space로 다시 토글된다.
        onClick={(e) => {
          e.currentTarget.blur();
          onToggleRecording();
        }}
      >
        ● {recording ? `REC ${recElapsed}` : "REC"}
      </button>

      <span className="clock">
        <span className="clock-date">{wallDate}</span>
        {wallTime}
        <span className="clock-sim">SIM {simTime}</span>
      </span>

      <span className={connected ? "sys-state ok" : "sys-state bad"}>
        {connected ? "RUNNING" : "DISCONNECTED"}
      </span>
    </header>
  );
}
