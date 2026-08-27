from __future__ import annotations

import pytest

from ros2_resilience.health.watchdog import ViolationReason, WatchdogEvaluator


def make_watchdog(**overrides: float) -> WatchdogEvaluator:
    values: dict[str, float] = {
        "max_age_sec": 0.3,
        "expected_rate_hz": 20.0,
        "loss_window_sec": 2.0,
        "max_loss_fraction": 0.8,
    }
    values.update(overrides)
    return WatchdogEvaluator(**values)


def test_no_violation_before_any_arrival() -> None:
    watchdog = make_watchdog()
    status = watchdog.check(now_ns=0)
    assert status.violated is False
    assert status.age_sec is None


def test_healthy_stream_reports_no_violation() -> None:
    watchdog = make_watchdog()
    for t in range(0, 1_000_000_000, 50_000_000):
        watchdog.record_arrival(t)
        status = watchdog.check(t)
        assert status.violated is False


def test_stale_stream_triggers_age_violation() -> None:
    watchdog = make_watchdog(max_age_sec=0.1)
    watchdog.record_arrival(0)
    status = watchdog.check(now_ns=200_000_000)
    assert status.violated is True
    assert status.reason is ViolationReason.MESSAGE_AGE_EXCEEDED


def test_violation_latches_even_if_traffic_resumes() -> None:
    watchdog = make_watchdog(max_age_sec=0.1)
    watchdog.record_arrival(0)
    first = watchdog.check(now_ns=200_000_000)
    assert first.violated is True

    watchdog.record_arrival(210_000_000)
    second = watchdog.check(now_ns=210_000_000)
    assert second.violated is True
    assert second.reason is ViolationReason.MESSAGE_AGE_EXCEEDED


def test_reset_clears_latch_and_history() -> None:
    watchdog = make_watchdog(max_age_sec=0.1)
    watchdog.record_arrival(0)
    watchdog.check(now_ns=200_000_000)

    watchdog.reset()
    status = watchdog.check(now_ns=200_000_000)
    assert status.violated is False


def test_loss_fraction_violation_without_age_violation() -> None:
    watchdog = make_watchdog(
        max_age_sec=1.0, expected_rate_hz=20.0, loss_window_sec=1.0, max_loss_fraction=0.5
    )
    for t_ms in range(0, 1000, 200):
        watchdog.record_arrival(t_ms * 1_000_000)
    status = watchdog.check(now_ns=1_000_000_000)
    assert status.violated is True
    assert status.reason is ViolationReason.LOSS_FRACTION_EXCEEDED
    assert status.age_sec == pytest.approx(0.2, abs=1e-6)


@pytest.mark.parametrize(
    "kwargs",
    [
        {"max_age_sec": 0},
        {"max_age_sec": -1},
        {"expected_rate_hz": 0},
        {"loss_window_sec": 0},
        {"max_loss_fraction": 1.5},
        {"max_loss_fraction": -0.1},
    ],
)
def test_rejects_invalid_configuration(kwargs: dict[str, float]) -> None:
    with pytest.raises(ValueError):
        make_watchdog(**kwargs)


def test_frozen_content_triggers_age_violation_despite_steady_arrivals() -> None:
    watchdog = make_watchdog(max_age_sec=0.15)
    frozen_stamp = 0
    for t_ms in range(0, 500, 50):
        watchdog.record_arrival(t_ms * 1_000_000, stamp_ns=frozen_stamp)
        status = watchdog.check(now_ns=t_ms * 1_000_000)
        if t_ms * 1_000_000 - frozen_stamp > 150_000_000:
            assert status.violated is True
            assert status.reason is ViolationReason.MESSAGE_AGE_EXCEEDED
        else:
            assert status.violated is False


def test_default_stamp_treats_arrival_as_fresh_content() -> None:
    watchdog = make_watchdog(max_age_sec=0.1)
    watchdog.record_arrival(0)
    watchdog.record_arrival(50_000_000)
    status = watchdog.check(now_ns=50_000_000)
    assert status.violated is False
    assert status.age_sec == pytest.approx(0.0, abs=1e-9)


def test_rejects_time_moving_backwards() -> None:
    watchdog = make_watchdog()
    watchdog.record_arrival(100)
    with pytest.raises(ValueError, match="backwards"):
        watchdog.record_arrival(50)
