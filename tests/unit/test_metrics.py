from __future__ import annotations

from ros2_resilience.core import metrics
from ros2_resilience.core.events import Event, EventKind


def make_event(kind: EventKind, receipt_ns: int, **payload: object) -> Event:
    return Event(
        event_id=f"e{receipt_ns}",
        trial_id="t",
        source="s",
        kind=kind,
        receipt_ns=receipt_ns,
        payload=payload,
    )


def test_detection_latency_uses_first_violation_after_activation() -> None:
    events = [
        make_event(EventKind.WATCHDOG_VIOLATION, 90),
        make_event(EventKind.WATCHDOG_VIOLATION, 150),
        make_event(EventKind.WATCHDOG_VIOLATION, 200),
    ]
    assert metrics.detection_latency_sec(events, activation_receipt_ns=100) == (150 - 100) / 1e9


def test_detection_latency_missing_returns_none() -> None:
    assert metrics.detection_latency_sec([], activation_receipt_ns=100) is None


def test_recovery_command_latency() -> None:
    events = [make_event(EventKind.RECOVERY_PUBLISHED, 130)]
    assert metrics.recovery_command_latency_sec(events, detection_receipt_ns=100) == 30 / 1e9


def test_command_observation_latency_requires_qualifying_values() -> None:
    events = [
        make_event(EventKind.COMMAND_OBSERVED, 110, linear_x=0.5, angular_z=0.0),
        make_event(EventKind.COMMAND_OBSERVED, 140, linear_x=0.0, angular_z=0.0),
    ]
    result = metrics.command_observation_latency_sec(
        events, detection_receipt_ns=100, linear_threshold_mps=0.01, angular_threshold_rad_s=0.01
    )
    assert result == 40 / 1e9


def test_time_to_stop_returns_seconds_and_receipt_ns() -> None:
    events = [
        make_event(EventKind.RAW_ODOMETRY, 110, linear_x=0.5, angular_z=0.0),
        make_event(EventKind.RAW_ODOMETRY, 200, linear_x=0.0, angular_z=0.0),
    ]
    result = metrics.time_to_stop_sec(
        events, activation_receipt_ns=100, linear_threshold_mps=0.01, angular_threshold_rad_s=0.01
    )
    assert result == (100 / 1e9, 200)


def test_stop_hold_broken_by_velocity_spike() -> None:
    events = [
        make_event(EventKind.RAW_ODOMETRY, 100, linear_x=0.0, angular_z=0.0),
        make_event(EventKind.RAW_ODOMETRY, 150, linear_x=0.5, angular_z=0.0),
    ]
    broken_at = metrics.stop_hold_broken_at_sec(
        events,
        stop_receipt_ns=100,
        hold_deadline_ns=300,
        linear_threshold_mps=0.01,
        angular_threshold_rad_s=0.01,
        expected_period_ns=50,
    )
    assert broken_at == 50 / 1e9


def test_stop_hold_broken_by_telemetry_gap() -> None:
    events = [make_event(EventKind.RAW_ODOMETRY, 100, linear_x=0.0, angular_z=0.0)]
    broken_at = metrics.stop_hold_broken_at_sec(
        events,
        stop_receipt_ns=100,
        hold_deadline_ns=400,
        linear_threshold_mps=0.01,
        angular_threshold_rad_s=0.01,
        expected_period_ns=50,
    )
    assert broken_at == 0.0


def test_stop_hold_not_broken_when_samples_stay_below_threshold() -> None:
    events = [
        make_event(EventKind.RAW_ODOMETRY, 110, linear_x=0.0, angular_z=0.0),
        make_event(EventKind.RAW_ODOMETRY, 150, linear_x=0.0, angular_z=0.0),
        make_event(EventKind.RAW_ODOMETRY, 200, linear_x=0.0, angular_z=0.0),
    ]
    broken_at = metrics.stop_hold_broken_at_sec(
        events,
        stop_receipt_ns=100,
        hold_deadline_ns=200,
        linear_threshold_mps=0.01,
        angular_threshold_rad_s=0.01,
        expected_period_ns=50,
    )
    assert broken_at is None
