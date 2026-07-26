"""IW 빈 슬롯 배정 상태(사용 기록/초기화) 회귀 테스트.

MM이 한 적재 사이클에 2개를 놓으므로 같은 KLT를 두 번 내주면 안 되고, 하역 후
지게차가 빈 팔레트를 새로 얹으면 다시 내줄 수 있어야 한다. 초기화 기준은 시간이
아니라 WarehouseDockController가 추적하는 '데크에 결속된 팔레트 ID' 변화다.

iw.py는 Isaac 런타임 모듈을 import하므로, 여기서는 그 의존성만 stub으로 갈아끼우고
슬롯 상태 로직은 실제 코드를 그대로 실행한다. pxr이 필요한 pose 발행 경로는
호출하지 않는다.
"""
import pytest


from iw import MM_PLACE_SLOTS, IwDriver


INITIAL_IW_PALLET_PATH = "/World/IwHubCargo/Pallet_00"


class _FakeDock:
    """WarehouseDockController 대역.

    deck_pallet_path()는 실물과 같은 판별 규칙을 쓴다 — 초기 데크 조인트가 살아
    있으면 IW 적재 팔레트, 아니면 창고 팔레트. 초기 팔레트와 창고 팔레트는 ID가
    둘 다 0일 수 있어 ID만으로는 구분되지 않는다(iw_dock.py의 같은 주석 참고).
    """

    def __init__(self, pallet_id, initial_deck_joint=True):
        self.deck_pallet_id = pallet_id
        self.initial_deck_joint = initial_deck_joint

    def deck_pallet_path(self):
        if self.deck_pallet_id is None:
            return None
        if self.initial_deck_joint:
            return INITIAL_IW_PALLET_PATH
        return f"/World/Warehouse/Pallet_{self.deck_pallet_id:02d}"


class _FakePoller:
    """release_debug 메시지를 원하는 횟수만큼 흘려준다."""

    def __init__(self):
        self.pending = 0

    def emit(self):
        self.pending += 1

    def poll(self):
        if self.pending <= 0:
            return None
        self.pending -= 1
        return '{"fruit_id": 1}'


@pytest.fixture
def driver():
    node = IwDriver.__new__(IwDriver)
    node._stage = object()
    node._basket_pose_pub = object()
    node._last_basket_slot = None
    node._used_basket_slots = set()
    node._deck_pallet_id = None
    node._mm_release_poller = _FakePoller()
    node._warehouse_dock = _FakeDock(0)
    return node


def _place_into(driver, slot):
    """MM이 지금 내준 슬롯에 하나 놓은 상황을 만든다."""
    driver._last_basket_slot = slot
    driver._mm_release_poller.emit()
    driver._consume_mm_release_events()


def test_initial_deck_pallet_targets_the_iw_cargo_not_the_warehouse(driver):
    """데크 위 팔레트는 ID가 아니라 데크 조인트로 정해진다.

    초기 IW 적재 팔레트와 창고 팔레트는 ID가 둘 다 0일 수 있고, ID로만 경로를
    풀면 창고 쪽이 먼저 잡힌다. 지금은 main.py가 일반 통합 실행에서 창고 0번을
    비워 충돌을 피하지만, 씬 구성이 바뀌어 둘이 공존하게 돼도 IW 적재 팔레트를
    우선해야 한다 — 안 그러면 MM이 창고 안쪽으로 플레이스를 시도한다.
    """
    assert driver._refresh_deck_pallet() == INITIAL_IW_PALLET_PATH


def test_two_front_slots_are_the_only_candidates(driver):
    assert MM_PLACE_SLOTS == ((3, 0), (3, 1))
    assert driver._available_place_slots() == MM_PLACE_SLOTS


def test_used_slot_is_not_offered_again_before_reset(driver):
    driver._refresh_deck_pallet()
    _place_into(driver, (3, 0))

    assert driver._available_place_slots() == ((3, 1),)

    _place_into(driver, (3, 1))
    assert driver._available_place_slots() == ()


def test_release_without_an_offered_slot_consumes_nothing(driver):
    driver._last_basket_slot = None
    driver._mm_release_poller.emit()
    driver._consume_mm_release_events()

    assert driver._used_basket_slots == set()


def test_new_pallet_on_the_deck_clears_the_used_slots(driver):
    driver._refresh_deck_pallet()
    _place_into(driver, (3, 0))
    _place_into(driver, (3, 1))
    assert driver._available_place_slots() == ()

    # 지게차가 실은 팔레트를 내리고 → 빈 팔레트를 새로 얹는다.
    driver._warehouse_dock.deck_pallet_id = None
    assert driver._refresh_deck_pallet() is None
    assert driver._used_basket_slots, "내리기만 해서는 초기화하지 않는다"

    driver._warehouse_dock.deck_pallet_id = 3
    driver._warehouse_dock.initial_deck_joint = False   # 초기 팔레트는 이미 떠났다
    path = driver._refresh_deck_pallet()

    assert driver._used_basket_slots == set()
    assert driver._available_place_slots() == MM_PLACE_SLOTS
    assert path == "/World/Warehouse/Pallet_03", "새 팔레트의 KLT를 겨냥해야 한다"


def test_same_pallet_id_returning_after_removal_also_clears(driver):
    """내렸다가 같은 ID가 다시 올라와도 새 적재로 본다."""
    driver._refresh_deck_pallet()
    _place_into(driver, (3, 0))

    driver._warehouse_dock.deck_pallet_id = None
    driver._refresh_deck_pallet()
    driver._warehouse_dock.deck_pallet_id = 0
    driver._refresh_deck_pallet()

    assert driver._used_basket_slots == set()


def test_empty_deck_publishes_nothing(driver):
    driver._warehouse_dock.deck_pallet_id = None

    assert driver._refresh_deck_pallet() is None
