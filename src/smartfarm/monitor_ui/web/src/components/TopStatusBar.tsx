interface Props {
  simTime: string;
  wallDate: string;
  wallTime: string;
  connected: boolean;
  focusedName?: string;
}

export default function TopStatusBar({
  simTime,
  wallDate,
  wallTime,
  connected,
  focusedName,
}: Props) {
  return (
    <header className="bar bar-top">
      <span className="sys-name">SMARTFARM MONITOR</span>
      {focusedName && <span className="focus-tag">{focusedName}</span>}

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
