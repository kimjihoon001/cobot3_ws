import { HarvestMetrics } from "../types";

const EMPTY: HarvestMetrics = {
  detected: 0,
  ripe: 0,
  spoiled: 0,
  unknown: 0,
  harvested: 0,
  failed: 0,
  state: "데이터 대기",
  quality_enabled: false,
};

export default function HarvestPanel({ metrics }: { metrics?: HarvestMetrics }) {
  const value = metrics ?? EMPTY;
  const judged = value.ripe + value.spoiled;
  const ripeRate = judged > 0 ? Math.round((value.ripe / judged) * 100) : null;

  return (
    <section className="dashboard" aria-label="수확 현황">
      <div className="section-heading">
        <div><span className="eyebrow">HARVEST STATUS</span><h1>수확 현황</h1></div>
        <span className="source-badge">ROS2 LIVE</span>
      </div>

      <div className="metric-grid">
        <article className="metric-card accent-green">
          <span>수확 완료</span><strong>{value.harvested}</strong><small>개</small>
        </article>
        <article className="metric-card">
          <span>현재 검출</span><strong>{value.detected}</strong><small>개</small>
        </article>
        <article className="metric-card accent-red">
          <span>수확 실패</span><strong>{value.failed}</strong><small>회</small>
        </article>
      </div>

      <article className="wide-card">
        <div className="card-title"><span>숙도 판정</span><b>{ripeRate === null ? "—" : `${ripeRate}% 적숙`}</b></div>
        <div className="maturity-bar">
          <span className="ripe" style={{ width: `${ripeRate ?? 0}%` }} />
        </div>
        <div className="legend-row">
          <span><i className="dot green" /> 적숙 {value.ripe}</span>
          <span><i className="dot red" /> 불량 {value.spoiled}</span>
          <span><i className="dot gray" /> 미판정 {value.unknown}</span>
        </div>
        {!value.quality_enabled && (
          <p className="notice">근거리 숙도 모델이 꺼져 있거나 판정 데이터가 아직 없습니다.</p>
        )}
      </article>

      <article className="state-card">
        <span>현재 MM 작업 상태</span><strong>{value.state || "데이터 대기"}</strong>
      </article>
    </section>
  );
}
