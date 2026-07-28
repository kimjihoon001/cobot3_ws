# -*- coding: utf-8 -*-
"""공공데이터 온라인 도매시장 거래정보를 /market/summary로 요약한다.

인증키는 소스나 브라우저에 넣지 않고 DATA_GO_KR_SERVICE_KEY 환경변수 또는
service_key ROS 파라미터로 받는다. 키가 없거나 호출이 실패하면 unavailable
상태를 발행하므로 UI가 임의 가격을 표시하지 않는다.
"""
from __future__ import annotations

import json
import os
import statistics
import urllib.parse
import urllib.request
from datetime import date, datetime, timedelta
from typing import Any

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


API_URL = "https://apis.data.go.kr/B552845/katOnline/trades"
LATCHED_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


def _rows(value: Any) -> list[dict]:
    """응답 래퍼 이름이 바뀌어도 가장 큰 레코드 배열을 찾는다."""
    candidates: list[list[dict]] = []
    if isinstance(value, list):
        records = [item for item in value if isinstance(item, dict)]
        if records:
            candidates.append(records)
        for item in value:
            candidates.extend([_rows(item)])
    elif isinstance(value, dict):
        for item in value.values():
            found = _rows(item)
            if found:
                candidates.append(found)
    return max(candidates, key=len, default=[])


def _number(record: dict, aliases: tuple[str, ...]) -> float | None:
    lowered = {str(key).lower(): value for key, value in record.items()}
    for alias in aliases:
        if alias.lower() not in lowered:
            continue
        value = lowered[alias.lower()]
        try:
            return float(str(value).replace(",", ""))
        except (TypeError, ValueError):
            pass
    return None


class MarketPriceNode(Node):
    def __init__(self) -> None:
        super().__init__("market_price_node")
        self.declare_parameter("service_key", os.environ.get(
            "DATA_GO_KR_SERVICE_KEY", ""))
        self.declare_parameter("product_keyword", "토마토")
        self.declare_parameter("subcategory_code", "")
        self.declare_parameter("refresh_sec", 300.0)
        self.declare_parameter("lookback_days", 7)
        self._pub = self.create_publisher(
            String, "/market/summary", LATCHED_QOS)
        self._last_average: float | None = None
        self.create_timer(float(self.get_parameter("refresh_sec").value), self._refresh)
        self._refresh()

    def _publish_unavailable(self, message: str) -> None:
        self._pub.publish(String(data=json.dumps({
            "available": False,
            "product": str(self.get_parameter("product_keyword").value),
            "updated_at": datetime.now().isoformat(timespec="seconds"),
            "message": message,
        }, ensure_ascii=False)))

    def _fetch_day(self, target_date: date, key: str) -> list[dict]:
        params = {
            "serviceKey": key,
            "pageNo": "1",
            "numOfRows": "1000",
            "returnType": "json",
            "cond[cfmtn_ymd::EQ]": target_date.isoformat(),
        }
        category = str(self.get_parameter("subcategory_code").value).strip()
        if category:
            params["cond[onln_whsl_mrkt_sclsf_cd::EQ]"] = category
        url = API_URL + "?" + urllib.parse.urlencode(params, safe="%")
        with urllib.request.urlopen(url, timeout=12) as response:
            return _rows(json.loads(response.read().decode("utf-8")))

    def _refresh(self) -> None:
        key = str(self.get_parameter("service_key").value).strip()
        if not key:
            self._publish_unavailable("공공데이터 API 키 설정 대기")
            return
        keyword = str(self.get_parameter("product_keyword").value).strip()
        try:
            matched: list[dict] = []
            used_date = date.today()
            for offset in range(int(self.get_parameter("lookback_days").value)):
                used_date = date.today() - timedelta(days=offset)
                records = self._fetch_day(used_date, key)
                matched = [record for record in records
                           if not keyword or keyword in json.dumps(
                               record, ensure_ascii=False)]
                if matched:
                    break
            if not matched:
                self._publish_unavailable(f"최근 거래에서 {keyword} 데이터 없음")
                return

            averages = [value for record in matched if (value := _number(record, (
                "avg_prc", "avrg_prc", "average_price", "평균가", "평균가격")))
                is not None]
            minimums = [value for record in matched if (value := _number(record, (
                "min_prc", "minimum_price", "최소가", "최저가"))) is not None]
            maximums = [value for record in matched if (value := _number(record, (
                "max_prc", "maximum_price", "최대가", "최고가"))) is not None]
            quantities = [value for record in matched if (value := _number(record, (
                "cfmtn_qty", "quantity", "확정수량", "거래량"))) is not None]
            if not averages:
                self._publish_unavailable("가격 필드 확인 필요: API 응답 명세 점검")
                return
            average = statistics.fmean(averages)
            change = None
            if self._last_average and self._last_average > 0:
                change = (average - self._last_average) / self._last_average * 100.0
            self._last_average = average
            payload = {
                "available": True,
                "product": keyword,
                "date": used_date.isoformat(),
                "average_price": round(average),
                "minimum_price": round(min(minimums)) if minimums else None,
                "maximum_price": round(max(maximums)) if maximums else None,
                "quantity": round(sum(quantities), 1) if quantities else None,
                "change_rate": round(change, 1) if change is not None else None,
                "unit": "원/kg",
                "updated_at": datetime.now().isoformat(timespec="seconds"),
                "message": "온라인 도매시장 거래정보",
            }
            self._pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))
        except Exception as exc:  # 네트워크/API 오류는 UI 상태로 바꾸고 노드는 유지한다.
            self.get_logger().warning(f"도매시세 조회 실패: {exc}")
            self._publish_unavailable(f"도매시세 조회 실패: {type(exc).__name__}")


def main() -> None:
    rclpy.init()
    node = MarketPriceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
