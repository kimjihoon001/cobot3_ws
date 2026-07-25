import { useEffect, useState } from "react";

import { UiStatus } from "../types";

const HOLD_MS = 8000;

/**
 * 최근 이벤트 한 줄만 잠깐 띄우고 사라진다. 전체 로그창을 만들지 않는 게
 * 이 화면의 원칙이라, 지나간 건 녹화본으로 돌려보는 쪽에 맡긴다.
 */
export default function EventTicker({
  events,
  format,
}: {
  events: UiStatus["events"];
  format: (t: number) => string;
}) {
  const latest = events.at(-1);
  const [shown, setShown] = useState<typeof latest>(undefined);

  useEffect(() => {
    if (!latest) return;
    setShown(latest);
    const timer = window.setTimeout(() => setShown(undefined), HOLD_MS);
    return () => window.clearTimeout(timer);
    // 같은 이벤트가 다시 오면 타이머만 다시 돈다.
  }, [latest?.t, latest?.text]);

  if (!shown) return <span className="event event-idle">—</span>;
  return (
    <span className="event">
      {format(shown.t)} {shown.src.toUpperCase()} {shown.text}
    </span>
  );
}
