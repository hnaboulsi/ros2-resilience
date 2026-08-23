from __future__ import annotations

import math
from collections.abc import Callable

import pytest

from ros2_resilience.faults.latency import LatencyQueue, QueueOverflowError


def constant_uniform(value: float) -> Callable[[float, float], float]:
    def uniform(_low: float, _high: float) -> float:
        return value

    return uniform


def test_zero_delay_needs_no_rng_and_preserves_arrival_order() -> None:
    def forbidden_rng(_low: float, _high: float) -> float:
        raise AssertionError("zero jitter must not call uniform")

    queue = LatencyQueue[str](delay_ns=0, uniform=forbidden_rng)
    queue.enqueue("first", now_ns=10)
    queue.enqueue("second", now_ns=10)

    assert queue.pop_ready(now_ns=10) == ["first", "second"]
    assert queue.pending_count == 0


def test_fixed_and_large_delays() -> None:
    queue = LatencyQueue[str](delay_ns=10**18, uniform=constant_uniform(0))
    queue.enqueue("late", now_ns=12)

    assert queue.pop_ready(now_ns=10**18 + 11) == []
    assert queue.pop_ready(now_ns=10**18 + 12) == ["late"]


def test_scripted_jitter_rounds_clamps_and_reorders_deadlines() -> None:
    samples = iter([2.5, -7.0, 0.49])
    calls: list[tuple[float, float]] = []

    def uniform(low: float, high: float) -> float:
        calls.append((low, high))
        return next(samples)

    queue = LatencyQueue[str](delay_ns=5, jitter_ns=7, uniform=uniform)
    queue.enqueue("first", now_ns=100)  # round(2.5) is 2, deadline 107
    queue.enqueue("second", now_ns=100)  # deadline clamps to 100
    queue.enqueue("third", now_ns=100)  # deadline 105

    assert calls == [(-7, 7), (-7, 7), (-7, 7)]
    assert queue.pop_ready(now_ns=100) == ["second"]
    assert queue.pop_ready(now_ns=105) == ["third"]
    assert queue.pop_ready(now_ns=107) == ["first"]


def test_equal_deadlines_are_stable() -> None:
    queue = LatencyQueue[str](delay_ns=10, uniform=constant_uniform(0))
    for message in ("a", "b", "c"):
        queue.enqueue(message, now_ns=0)

    assert queue.pop_ready(now_ns=10) == ["a", "b", "c"]


def test_enqueue_snapshots_message() -> None:
    queue = LatencyQueue[dict[str, list[str]]](delay_ns=1, uniform=constant_uniform(0))
    message = {"items": ["before"]}
    queue.enqueue(message, now_ns=0)
    message["items"].append("after")

    assert queue.pop_ready(now_ns=1) == [{"items": ["before"]}]


def test_overflow_does_not_drop_or_mutate_existing_queue() -> None:
    queue = LatencyQueue[str](delay_ns=1, max_pending=1, uniform=constant_uniform(0))
    queue.enqueue("kept", now_ns=1)

    with pytest.raises(QueueOverflowError, match="max_pending=1"):
        queue.enqueue("rejected", now_ns=2)

    assert queue.pending_count == 1
    assert queue.pop_ready(now_ns=2) == ["kept"]


def test_configure_only_changes_future_arrivals() -> None:
    queue = LatencyQueue[str](delay_ns=10, uniform=constant_uniform(0))
    queue.enqueue("old", now_ns=0)
    queue.configure(delay_ns=1, jitter_ns=0)
    queue.enqueue("new", now_ns=1)

    assert queue.pop_ready(now_ns=2) == ["new"]
    assert queue.pop_ready(now_ns=10) == ["old"]


def test_flush_uses_arrival_order_despite_deadline_reordering() -> None:
    samples = iter([5.0, -5.0, 0.0])
    queue = LatencyQueue[str](
        delay_ns=5,
        jitter_ns=5,
        uniform=lambda _low, _high: next(samples),
    )
    for message in ("first", "second", "third"):
        queue.enqueue(message, now_ns=20)

    assert queue.flush() == ["first", "second", "third"]
    assert queue.pending_count == 0


def test_flush_preserves_clock_history() -> None:
    queue = LatencyQueue[str](delay_ns=1, uniform=constant_uniform(0))
    queue.enqueue("pending", now_ns=10)

    assert queue.flush() == ["pending"]
    with pytest.raises(ValueError, match="backwards"):
        queue.enqueue("past", now_ns=9)


def test_failed_deepcopy_preserves_queue_and_clock() -> None:
    class Uncopyable:
        def __deepcopy__(self, memo: dict[int, object]) -> Uncopyable:
            raise RuntimeError("cannot copy")

    queue = LatencyQueue[object](delay_ns=5, uniform=constant_uniform(0))
    queue.enqueue("kept", now_ns=10)

    with pytest.raises(RuntimeError, match="cannot copy"):
        queue.enqueue(Uncopyable(), now_ns=11)

    queue.enqueue("same-time", now_ns=10)
    assert queue.pop_ready(now_ns=15) == ["kept", "same-time"]


@pytest.mark.parametrize(
    ("kwargs", "exception"),
    [
        ({"delay_ns": True}, TypeError),
        ({"delay_ns": -1}, ValueError),
        ({"jitter_ns": True}, TypeError),
        ({"jitter_ns": -1}, ValueError),
        ({"max_pending": 0}, ValueError),
        ({"max_pending": False}, TypeError),
    ],
)
def test_constructor_rejects_invalid_integer_settings(
    kwargs: dict[str, int], exception: type[Exception]
) -> None:
    settings = {"delay_ns": 0, "jitter_ns": 0, "max_pending": 1000}
    settings.update(kwargs)
    with pytest.raises(exception):
        LatencyQueue[str](uniform=constant_uniform(0), **settings)


def test_invalid_operations_do_not_advance_time_or_create_entries() -> None:
    queue = LatencyQueue[str](delay_ns=2, jitter_ns=1, uniform=constant_uniform(math.nan))

    with pytest.raises(ValueError, match="finite"):
        queue.enqueue("bad-rng", now_ns=5)
    assert queue.pending_count == 0

    queue.configure(delay_ns=2, jitter_ns=0)
    queue.enqueue("valid", now_ns=4)
    with pytest.raises(ValueError, match="backwards"):
        queue.pop_ready(now_ns=3)
    with pytest.raises(TypeError):
        queue.enqueue("bad-time", now_ns=True)
    with pytest.raises(ValueError):
        queue.enqueue("negative-time", now_ns=-1)

    assert queue.pop_ready(now_ns=6) == ["valid"]


@pytest.mark.parametrize("sample", [math.inf, -math.inf, math.nan, 3.1, "bad"])
def test_rng_sample_must_be_finite_and_inside_bounds(sample: object) -> None:
    queue = LatencyQueue[str](delay_ns=1, jitter_ns=3, uniform=lambda _low, _high: sample)  # type: ignore[arg-type]

    with pytest.raises((TypeError, ValueError)):
        queue.enqueue("message", now_ns=0)

    assert queue.pending_count == 0
    queue.configure(delay_ns=0, jitter_ns=0)
    queue.enqueue("later", now_ns=0)
    assert queue.pop_ready(now_ns=0) == ["later"]


def test_invalid_configure_is_atomic() -> None:
    queue = LatencyQueue[str](delay_ns=4, uniform=constant_uniform(0))

    with pytest.raises(ValueError):
        queue.configure(delay_ns=1, jitter_ns=-1)
    queue.enqueue("keeps-old-delay", now_ns=0)

    assert queue.pop_ready(now_ns=3) == []
    assert queue.pop_ready(now_ns=4) == ["keeps-old-delay"]
