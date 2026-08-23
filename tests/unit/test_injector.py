from __future__ import annotations

import math
import random

import pytest

from ros2_resilience.faults.injector import FaultConfig, FaultInjector, FaultType
from ros2_resilience.faults.latency import QueueOverflowError


def config(**overrides: object) -> FaultConfig:
    values: dict[str, object] = {"enabled": True, "fault_type": FaultType.NONE}
    values.update(overrides)
    return FaultConfig(**values)  # type: ignore[arg-type]


@pytest.mark.parametrize(
    ("kwargs", "exception"),
    [
        ({"enabled": 1}, TypeError),
        ({"fault_type": "dropout"}, TypeError),
        ({"drop_probability": True}, TypeError),
        ({"drop_probability": math.nan}, ValueError),
        ({"drop_probability": 1.1}, ValueError),
        ({"delay_ns": True}, TypeError),
        ({"delay_ns": -1}, ValueError),
        ({"jitter_ns": True}, TypeError),
        ({"jitter_ns": -1}, ValueError),
        ({"seed": True}, TypeError),
        ({"seed": -1}, ValueError),
    ],
)
def test_fault_config_rejects_invalid_fields(
    kwargs: dict[str, object], exception: type[Exception]
) -> None:
    with pytest.raises(exception):
        FaultConfig(**kwargs)  # type: ignore[arg-type]


@pytest.mark.parametrize("fault_type", list(FaultType))
def test_disabled_every_type_forwards_a_snapshot_and_tracks_input(fault_type: FaultType) -> None:
    injector = FaultInjector[dict[str, list[str]]](config(enabled=False, fault_type=fault_type))
    message = {"payload": ["before"]}

    assert injector.receive(message, now_ns=0) == [{"payload": ["before"]}]
    message["payload"].append("after")
    assert injector.configure(config(fault_type=FaultType.STALE), now_ns=1) == []
    assert injector.receive({"payload": ["new"]}, now_ns=2) == [{"payload": ["before"]}]


def test_dropout_extremes_and_seeded_decisions() -> None:
    never = FaultInjector[int](config(fault_type=FaultType.DROPOUT, drop_probability=0.0))
    always = FaultInjector[int](config(fault_type=FaultType.DROPOUT, drop_probability=1.0))
    seeded = FaultInjector[int](config(fault_type=FaultType.DROPOUT, drop_probability=0.5, seed=7))

    assert [never.receive(value, now_ns=value) for value in range(2)] == [[0], [1]]
    assert [always.receive(value, now_ns=value) for value in range(2)] == [[], []]
    assert [seeded.receive(value, now_ns=value) for value in range(5)] == [[], [], [2], [], [4]]
    assert always.counters.dropped == 2


def test_disabling_latency_flushes_then_resume_accepts_new_arrivals() -> None:
    injector = FaultInjector[str](config(fault_type=FaultType.LATENCY, delay_ns=10))
    assert injector.receive("pending", now_ns=0) == []

    assert injector.configure(config(enabled=False, fault_type=FaultType.LATENCY), now_ns=1) == [
        "pending"
    ]
    assert injector.receive("immediate", now_ns=2) == ["immediate"]
    assert injector.configure(config(fault_type=FaultType.LATENCY, delay_ns=3), now_ns=3) == []
    assert injector.receive("later", now_ns=4) == []
    assert injector.poll(now_ns=7) == ["later"]


def test_mode_switch_flushes_in_original_arrival_order_before_new_mode_input() -> None:
    injector = FaultInjector[str](config(fault_type=FaultType.LATENCY, delay_ns=10))
    injector.receive("first", now_ns=0)
    injector.receive("second", now_ns=1)

    assert injector.configure(
        config(fault_type=FaultType.DROPOUT, drop_probability=0.0), now_ns=2
    ) == [
        "first",
        "second",
    ]
    assert injector.receive("after-switch", now_ns=3) == ["after-switch"]


def test_latency_reconfigure_preserves_existing_deadlines() -> None:
    injector = FaultInjector[str](config(fault_type=FaultType.LATENCY, delay_ns=10))
    assert injector.receive("old", now_ns=0) == []
    assert injector.configure(config(fault_type=FaultType.LATENCY, delay_ns=1), now_ns=1) == []
    assert injector.receive("new", now_ns=1) == []

    assert injector.poll(now_ns=2) == ["new"]
    assert injector.poll(now_ns=10) == ["old"]


def test_invalid_config_and_time_leave_pending_state_untouched() -> None:
    injector = FaultInjector[str](config(fault_type=FaultType.LATENCY, delay_ns=5))
    injector.receive("kept", now_ns=10)

    with pytest.raises(TypeError, match="config"):
        injector.configure("not-config", now_ns=11)  # type: ignore[arg-type]
    with pytest.raises(ValueError, match="backwards"):
        injector.poll(now_ns=9)
    with pytest.raises(TypeError, match="now_ns"):
        injector.configure(config(), now_ns=True)  # type: ignore[arg-type]

    assert injector.pending_count == 1
    assert injector.poll(now_ns=15) == ["kept"]


def test_stale_activation_uses_raw_arrival_from_a_previous_mode() -> None:
    injector = FaultInjector[int](config(fault_type=FaultType.DROPOUT, drop_probability=1.0))
    assert injector.receive(41, now_ns=0) == []

    assert injector.configure(config(fault_type=FaultType.STALE), now_ns=1) == []
    assert injector.receive(99, now_ns=2) == [41]


def test_disabling_and_reenabling_stale_hold_uses_latest_real_input() -> None:
    injector = FaultInjector[int](config(fault_type=FaultType.STALE))
    assert injector.receive(1, now_ns=0) == [1]
    assert injector.receive(2, now_ns=1) == [1]
    assert injector.configure(config(enabled=False, fault_type=FaultType.STALE), now_ns=2) == []
    assert injector.receive(3, now_ns=3) == [3]
    assert injector.configure(config(fault_type=FaultType.STALE), now_ns=4) == []
    assert injector.receive(4, now_ns=5) == [3]


def test_same_stale_reconfigure_keeps_original_hold() -> None:
    injector = FaultInjector[int](config(fault_type=FaultType.STALE))
    assert injector.receive(1, now_ns=0) == [1]
    assert injector.receive(2, now_ns=1) == [1]

    assert injector.configure(config(fault_type=FaultType.STALE), now_ns=2) == []
    assert injector.receive(3, now_ns=3) == [1]


def test_unrelated_reconfigure_preserves_rng_but_seed_change_resets_owned_rng() -> None:
    base = config(fault_type=FaultType.DROPOUT, drop_probability=0.5, seed=1)
    injector = FaultInjector[int](base)
    expected = random.Random(1)

    assert injector.receive(0, now_ns=0) == ([] if expected.random() < 0.5 else [0])
    assert (
        injector.configure(
            config(fault_type=FaultType.DROPOUT, drop_probability=0.5, seed=1, delay_ns=5), now_ns=1
        )
        == []
    )
    assert injector.receive(1, now_ns=2) == ([] if expected.random() < 0.5 else [1])

    assert (
        injector.configure(
            config(fault_type=FaultType.DROPOUT, drop_probability=0.5, seed=19), now_ns=3
        )
        == []
    )
    reset = random.Random(19)
    assert injector.receive(2, now_ns=4) == ([] if reset.random() < 0.5 else [2])


def test_external_rng_rejects_seed_change_without_advancing_time() -> None:
    injector = FaultInjector[int](
        config(fault_type=FaultType.DROPOUT, seed=3), rng=random.Random(3)
    )
    injector.receive(1, now_ns=10)

    with pytest.raises(ValueError, match="external rng"):
        injector.configure(config(fault_type=FaultType.DROPOUT, seed=4), now_ns=11)

    assert injector.receive(2, now_ns=10) == [2]


def test_invalid_rng_result_does_not_update_stale_state_or_clock() -> None:
    class InvalidRandom(random.Random):
        def random(self) -> float:
            return 1.0

    injector = FaultInjector[int](
        config(fault_type=FaultType.DROPOUT, drop_probability=0.5),
        rng=InvalidRandom(),
    )

    with pytest.raises(ValueError, match="within \\[0.0, 1.0\\)"):
        injector.receive(1, now_ns=10)

    assert injector.counters.received == 0
    assert injector.counters.forwarded == 0
    assert injector.counters.dropped == 0
    assert injector.configure(config(fault_type=FaultType.STALE), now_ns=0) == []
    assert injector.receive(2, now_ns=1) == [2]


def test_zero_delay_latency_is_immediate() -> None:
    injector = FaultInjector[str](config(fault_type=FaultType.LATENCY, delay_ns=0))

    assert injector.receive("now", now_ns=20) == ["now"]
    assert injector.pending_count == 0


def test_latency_overflow_is_surfaced_without_advancing_time_or_stale_state() -> None:
    injector = FaultInjector[int](config(fault_type=FaultType.LATENCY, delay_ns=5), max_pending=1)
    injector.receive(1, now_ns=10)

    with pytest.raises(QueueOverflowError, match="max_pending=1"):
        injector.receive(2, now_ns=11)

    assert injector.configure(config(fault_type=FaultType.STALE), now_ns=10) == [1]
    assert injector.receive(3, now_ns=12) == [1]
