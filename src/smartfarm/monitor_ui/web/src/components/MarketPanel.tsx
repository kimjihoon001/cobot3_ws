import { HarvestMetrics, MarketSummary } from "../types";

function money(value: number | null | undefined): string {
  return value == null ? "—" : `${Math.round(value).toLocaleString("ko-KR")}원`;
}

function recommendation(market?: MarketSummary, harvest?: HarvestMetrics) {
  if (!market?.available) return { level: "대기", text: "도매시세 API 연동 후 추천을 시작합니다." };
  if (!harvest?.quality_enabled) return { level: "확인 필요", text: "숙도 판정 데이터를 먼저 확보해야 합니다." };
  if (harvest.ripe <= 0) return { level: "관찰", text: "현재 적숙 과실이 없어 수확을 보류합니다." };
  if (market.change_rate != null && market.change_rate >= 5)
    return { level: "출하 검토", text: "가격 상승과 적숙 과실이 함께 확인됐습니다." };
  if (market.change_rate != null && market.change_rate <= -5)
    return { level: "가격 관찰", text: "가격 하락 중이므로 품질 허용 범위에서 추이를 확인하세요." };
  return { level: "수확 가능", text: "적숙 과실을 우선 수확하고 출하 물량을 확인하세요." };
}

export default function MarketPanel({
  market,
  harvest,
}: {
  market?: MarketSummary;
  harvest?: HarvestMetrics;
}) {
  const advice = recommendation(market, harvest);
  const change = market?.change_rate;

  return (
    <section className="dashboard" aria-label="도매시세와 출하 추천">
      <div className="section-heading">
        <div><span className="eyebrow">MARKET INSIGHT</span><h1>도매시세 · 출하 추천</h1></div>
        <span className={`source-badge${market?.available ? " live" : ""}`}>
          {market?.available ? "실시간 연동" : "연동 대기"}
        </span>
      </div>

      <article className="price-hero">
        <span>{market?.product ?? "토마토"} 평균 도매가</span>
        <strong>{money(market?.average_price)}</strong>
        <div>
          <b className={change != null && change < 0 ? "down" : "up"}>
            {change == null ? "변동 데이터 수집 중" : `${change >= 0 ? "▲" : "▼"} ${Math.abs(change)}%`}
          </b>
          <small>{market?.date || "거래일 대기"}</small>
        </div>
      </article>

      <div className="market-grid">
        <article><span>최저가</span><strong>{money(market?.minimum_price)}</strong></article>
        <article><span>최고가</span><strong>{money(market?.maximum_price)}</strong></article>
        <article><span>거래량</span><strong>{market?.quantity == null ? "—" : `${market.quantity.toLocaleString("ko-KR")} kg`}</strong></article>
        <article><span>적숙 과실</span><strong>{harvest?.quality_enabled ? `${harvest.ripe}개` : "—"}</strong></article>
      </div>

      <article className="recommend-card">
        <span>오늘의 추천</span>
        <strong>{advice.level}</strong>
        <p>{advice.text}</p>
      </article>

      <p className="data-footnote">
        {market?.message || "공공데이터포털 서비스 키를 설정하면 실제 가격을 표시합니다."}
        {market?.updated_at && ` · ${market.updated_at.replace("T", " ")}`}
      </p>
    </section>
  );
}
