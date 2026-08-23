from __future__ import annotations

from dataclasses import dataclass

import pytest

from ros2_resilience.faults.stale import StaleHold


@dataclass
class Sample:
    timestamp: int
    payload: list[int]


def test_enabled_holds_previous_sample_with_timestamp_unchanged() -> None:
    hold = StaleHold[Sample]()
    first = Sample(timestamp=10, payload=[1])
    second = Sample(timestamp=20, payload=[2])

    assert hold.receive(first) == first
    hold.set_enabled(True)

    assert hold.receive(second) == first


def test_disabled_forwards_current_input_after_resume() -> None:
    hold = StaleHold[int]()
    hold.receive(1)
    hold.set_enabled(True)
    assert hold.receive(2) == 1

    hold.set_enabled(False)
    assert hold.receive(3) == 3


def test_enable_before_first_input_holds_the_first_receive() -> None:
    hold = StaleHold[int]()
    hold.set_enabled(True)

    assert hold.receive(4) == 4
    assert hold.receive(5) == 4


def test_repeated_enable_keeps_original_hold() -> None:
    hold = StaleHold[int]()
    hold.receive(1)
    hold.set_enabled(True)
    assert hold.receive(2) == 1

    hold.set_enabled(True)
    assert hold.receive(3) == 1


def test_reactivation_holds_latest_real_input_received_while_enabled() -> None:
    hold = StaleHold[int]()
    hold.receive(1)
    hold.set_enabled(True)
    assert hold.receive(2) == 1

    hold.set_enabled(False)
    hold.set_enabled(True)

    assert hold.receive(3) == 2


def test_deepcopy_isolation_for_input_and_returned_values() -> None:
    hold = StaleHold[Sample]()
    sample = Sample(timestamp=10, payload=[1])
    returned = hold.receive(sample)
    sample.payload.append(99)
    returned.payload.append(88)

    hold.set_enabled(True)
    stale = hold.receive(Sample(timestamp=20, payload=[2]))
    stale.payload.append(77)

    assert stale.timestamp == 10
    assert stale.payload == [1, 77]
    assert hold.receive(Sample(timestamp=30, payload=[3])) == Sample(10, [1])


def test_none_is_a_valid_input() -> None:
    hold = StaleHold[None]()

    assert hold.receive(None) is None
    hold.set_enabled(True)
    assert hold.receive(None) is None


@pytest.mark.parametrize("enabled", [1, 0, "true", None])
def test_set_enabled_rejects_non_boolean_values(enabled: object) -> None:
    hold = StaleHold[int]()

    with pytest.raises(TypeError, match="enabled must be a bool"):
        hold.set_enabled(enabled)  # type: ignore[arg-type]
