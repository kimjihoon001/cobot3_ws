import { AppTab } from "../types";

const ITEMS: { id: AppTab; icon: string; label: string }[] = [
  { id: "cctv", icon: "▣", label: "CCTV" },
  { id: "harvest", icon: "●", label: "수확현황" },
  { id: "market", icon: "↗", label: "시세·추천" },
];

export default function BottomNav({
  active,
  onChange,
}: {
  active: AppTab;
  onChange: (tab: AppTab) => void;
}) {
  return (
    <nav className="bottom-nav" aria-label="화면 선택">
      {ITEMS.map((item) => (
        <button
          key={item.id}
          className={`nav-item${active === item.id ? " active" : ""}`}
          onClick={() => onChange(item.id)}
        >
          <span className="nav-icon">{item.icon}</span>
          <span>{item.label}</span>
        </button>
      ))}
    </nav>
  );
}
