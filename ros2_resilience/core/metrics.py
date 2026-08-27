"""Duration calculations derived from collected trial events.

All functions here are pure: they take an ordered sequence of
:class:`~ros2_resilience.core.events.Event` plus explicit reference times and
return either a duration in seconds or ``None`` when the required evidence is
missing. They never raise on missing evidence; missing evidence is a FAIL
condition decided by the assertion layer, not by this module.

Every calculation here uses ``receipt_ns`` (the observer's monotonic receive
time), never a source/header timestamp, so latencies reflect this process's
observation of events and are comparable to each other regardless of which
component produced the underlying message.
"""

from __future__ import annotations

from collections.abc import Iterable

from ros2_resilience.core.events import Event, EventKind

__all__ = [
    "command_observation_latency_sec",
    "detection_latency_sec",
    "recovery_command_latency_sec",
    "stop_hold_broken_at_sec",
    "time_to_stop_sec",
]


def _first_after(events: Iterable[Event], kind: EventKind, after_ns: int) -> Event | None:
    candidates = (e for e in events if e.kind is kind and e.receipt_ns >= after_ns)
    return min(candidates, key=lambda e: e.receipt_ns, default=None)


def detection_latency_sec(events: Iterable[Event], activation_receipt_ns: int) -> float | None:
    """Time from fault activation to the first watchdog violation."""
    violation = _first_after(events, EventKind.WATCHDOG_VIOLATION, activation_receipt_ns)
    if violation is None:
        return None
    return (violation.receipt_ns - activation_receipt_ns) / 1e9


def recovery_command_latency_sec(
    events: Iterable[Event], detection_receipt_ns: int
) -> float | None:
    """Time from detection to the first recovery-command publish call."""
    published = _first_after(events, EventKind.RECOVERY_PUBLISHED, detection_receipt_ns)
    if published is None:
        return None
    return (published.receipt_ns - detection_receipt_ns) / 1e9


def command_observation_latency_sec(
    events: Iterable[Event],
    detection_receipt_ns: int,
    *,
    linear_threshold_mps: float,
    angular_threshold_rad_s: float,
) -> float | None:
    """Time from detection to the first observed qualifying zero command."""
    candidates = [
        e
        for e in events
        if e.kind is EventKind.COMMAND_OBSERVED
        and e.receipt_ns >= detection_receipt_ns
        and abs(e.payload.get("linear_x", float("inf"))) < linear_threshold_mps
        and abs(e.payload.get("angular_z", float("inf"))) < angular_threshold_rad_s
    ]
    if not candidates:
        return None
    first = min(candidates, key=lambda e: e.receipt_ns)
    return (first.receipt_ns - detection_receipt_ns) / 1e9


def time_to_stop_sec(
    events: Iterable[Event],
    activation_receipt_ns: int,
    *,
    linear_threshold_mps: float,
    angular_threshold_rad_s: float,
) -> tuple[float, int] | None:
    """Time from activation to the first qualifying raw-odometry sample.

    Returns ``(seconds, receipt_ns)`` of the qualifying sample, or ``None``
    when no raw-odometry sample at or after activation stayed under both
    thresholds.
    """
    candidates = [
        e
        for e in events
        if e.kind is EventKind.RAW_ODOMETRY
        and e.receipt_ns >= activation_receipt_ns
        and abs(e.payload.get("linear_x", float("inf"))) < linear_threshold_mps
        and abs(e.payload.get("angular_z", float("inf"))) < angular_threshold_rad_s
    ]
    if not candidates:
        return None
    first = min(candidates, key=lambda e: e.receipt_ns)
    return (first.receipt_ns - activation_receipt_ns) / 1e9, first.receipt_ns


def stop_hold_broken_at_sec(
    events: Iterable[Event],
    stop_receipt_ns: int,
    hold_deadline_ns: int,
    *,
    linear_threshold_mps: float,
    angular_threshold_rad_s: float,
    expected_period_ns: int,
) -> float | None:
    """Return seconds-since-stop of the first hold violation, or ``None``.

    A violation is either a raw-odometry sample at or above a threshold, or a
    telemetry gap between consecutive raw-odometry samples exceeding three
    expected publication periods, observed between ``stop_receipt_ns`` and
    ``hold_deadline_ns`` inclusive.
    """
    samples = sorted(
        (
            e
            for e in events
            if e.kind is EventKind.RAW_ODOMETRY
            and stop_receipt_ns <= e.receipt_ns <= hold_deadline_ns
        ),
        key=lambda e: e.receipt_ns,
    )
    max_gap_ns = 3 * expected_period_ns
    previous_ns = stop_receipt_ns
    for sample in samples:
        if sample.receipt_ns - previous_ns > max_gap_ns:
            return (previous_ns - stop_receipt_ns) / 1e9
        if (
            abs(sample.payload.get("linear_x", float("inf"))) >= linear_threshold_mps
            or abs(sample.payload.get("angular_z", float("inf"))) >= angular_threshold_rad_s
        ):
            return (sample.receipt_ns - stop_receipt_ns) / 1e9
        previous_ns = sample.receipt_ns

    if hold_deadline_ns - previous_ns > max_gap_ns:
        return (previous_ns - stop_receipt_ns) / 1e9
    return None
