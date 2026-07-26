"""스쿱 릴리즈 이벤트가 위치 진단 성공 여부와 분리돼 있는지 검증한다.

2026-07-26 sim_diag_223158: `/harvester_0/scoop/release_debug`가 12093건 발행됐지만
**전부 빈 문자열**이었다. 과실/TCP 좌표 조회에 실패하면 publish 자체를 건너뛰는
구조였기 때문이다. 그 결과 IW 슬롯 배정기가 적재를 한 건도 세지 못했고, 2회
플레이스가 같은 KLT로 들어갔다.

판정 기준은 파지 '검증' 결과가 아니라 ROS가 붙인 개방 이유(reason="place")다.
과실을 제대로 집어도 검증이 실패로 뜨는 경우가 많아 검증으로 거르면 실제 적재를
놓친다.

여기서는 MMDriver를 구성하지 않고 `_publish_scoop_release`만 떼어 호출한다
(Isaac 런타임 불필요).
"""
import json

import numpy as np
import pytest


class _Pub:
    def __init__(self):
        self.messages = []

    def publish(self, value):
        self.messages.append(value)


class _Driver:
    """`_publish_scoop_release`가 실제로 건드리는 상태만 갖춘 최소 대역."""

    from mm import MMDriver
    _publish_scoop_release = MMDriver._publish_scoop_release

    def __init__(self, *, center=None, tcp=None, raises=False,
                 verified_fruit_id=7):
        self._verified_fruit_id = verified_fruit_id
        self._release_seq = 0
        self._release_pub = _Pub()
        self._center = center
        self._tcp = tcp
        self._raises = raises

    def _ripe_by_id(self, fruit_id):
        if self._raises:
            raise RuntimeError("stage 조회 실패")
        return None, self._center

    def _tcp_world(self):
        if self._raises:
            raise RuntimeError("TCP 조회 실패")
        return self._tcp


def _payloads(driver):
    return [json.loads(m) for m in driver._release_pub.messages]


def test_event_is_published_with_full_diagnostics():
    d = _Driver(center=np.array([1.0, 2.0, 3.0]), tcp=np.array([1.0, 2.0, 2.9]))
    d._publish_scoop_release("place")

    (payload,) = _payloads(d)
    assert payload["fruit_id"] == 7
    assert payload["seq"] == 1
    assert payload["lateral"] == pytest.approx(0.0, abs=1e-6)
    assert payload["fruit_map"] == [1.0, 2.0, 3.0]


def test_event_is_published_even_when_coordinates_are_missing():
    """★회귀: center/tcp가 없어도 released 이벤트는 반드시 나간다."""
    d = _Driver(center=None, tcp=None)
    d._publish_scoop_release("place")

    (payload,) = _payloads(d)
    assert payload["fruit_id"] == 7
    assert payload["seq"] == 1
    assert payload["fruit_map"] is None
    assert payload["lateral"] is None


def test_event_is_published_even_when_lookup_raises():
    d = _Driver(raises=True)
    d._publish_scoop_release("place")

    (payload,) = _payloads(d)
    assert payload["seq"] == 1
    assert payload["fruit_map"] is None


def test_sequence_number_makes_repeated_releases_distinguishable():
    """진단이 전부 null이면 페이로드가 같아져 StringPoller가 변화를 놓친다."""
    d = _Driver(center=None, tcp=None)
    d._publish_scoop_release("place")
    d._verified_fruit_id = 7          # 같은 과실 id로 한 번 더
    d._publish_scoop_release("place")

    first, second = _payloads(d)
    assert first["seq"] == 1 and second["seq"] == 2
    assert d._release_pub.messages[0] != d._release_pub.messages[1]


def test_failed_grasp_verification_still_publishes_the_release():
    """★회귀: 파지 검증이 실패로 떠도 실제로 놓았으면 이벤트가 나가야 한다.

    검증 실패는 흔하다. 이걸로 거르면 적재를 통째로 놓쳐 두 번째 플레이스가
    같은 KLT로 간다(이번 bag에서 실제로 그렇게 됐다).
    """
    d = _Driver(center=None, tcp=None, verified_fruit_id=-1)
    d._publish_scoop_release("place")

    (payload,) = _payloads(d)
    assert payload["seq"] == 1
    assert payload["fruit_id"] is None, "검증 id가 없으면 null로 둔다"


def test_approach_open_publishes_nothing():
    """파지 전 개방은 바구니 적재가 아니다 — 빈 칸을 소모하면 안 된다."""
    d = _Driver(center=None, tcp=None)
    d._publish_scoop_release("")

    assert d._release_pub.messages == []


def test_failure_recovery_open_publishes_nothing():
    """실패 시 부분 파지 해제 개방도 적재가 아니다."""
    d = _Driver(center=None, tcp=None)
    d._publish_scoop_release("abort")

    assert d._release_pub.messages == []


def test_non_place_open_discards_the_stale_verified_id():
    """개방하면 과실은 스쿱을 떠났다 — 검증 id가 다음 릴리즈로 새면 안 된다.

    파지 성공 뒤 실패 복구로 개방하면 그 과실은 바닥에 떨어진다. id를 남겨두면
    다음 릴리즈의 낙하점 진단에 엉뚱한 과실 좌표가 붙는다.
    """
    d = _Driver(center=None, tcp=None)
    d._publish_scoop_release("abort")
    assert d._verified_fruit_id == -1

    d._publish_scoop_release("place")
    (payload,) = _payloads(d)
    assert payload["fruit_id"] is None


def test_verified_id_is_consumed_once():
    """진단용 id는 한 번만 쓴다 — 다음 릴리즈에 재사용되면 안 된다."""
    d = _Driver(center=None, tcp=None)
    d._publish_scoop_release("place")
    d._publish_scoop_release("place")

    first, second = _payloads(d)
    assert first["fruit_id"] == 7
    assert second["fruit_id"] is None
