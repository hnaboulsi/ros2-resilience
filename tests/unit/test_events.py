from __future__ import annotations

import pytest

from ros2_resilience.core.events import ClockDomain, Event, EventKind


def test_minimal_event_constructs() -> None:
    event = Event(
        event_id="e1", trial_id="t1", source="robot", kind=EventKind.RAW_ODOMETRY, receipt_ns=10
    )
    assert event.payload == {}
    assert event.occurrence_ns is None


def test_event_with_occurrence_time() -> None:
    event = Event(
        event_id="e1",
        trial_id="t1",
        source="robot",
        kind=EventKind.RAW_ODOMETRY,
        receipt_ns=10,
        occurrence_ns=5,
        occurrence_domain=ClockDomain.ROS,
    )
    assert event.occurrence_ns == 5
    assert event.occurrence_domain is ClockDomain.ROS


@pytest.mark.parametrize(
    "kwargs",
    [
        {"event_id": ""},
        {"trial_id": ""},
        {"source": ""},
    ],
)
def test_rejects_empty_identity_fields(kwargs: dict[str, object]) -> None:
    base = {"event_id": "e1", "trial_id": "t1", "source": "robot"}
    base.update(kwargs)
    with pytest.raises(ValueError, match="nonempty"):
        Event(kind=EventKind.RAW_ODOMETRY, receipt_ns=0, **base)


def test_rejects_non_event_kind() -> None:
    with pytest.raises(TypeError, match="EventKind"):
        Event(event_id="e1", trial_id="t1", source="robot", kind="raw_odometry", receipt_ns=0)  # type: ignore[arg-type]


def test_rejects_negative_receipt_ns() -> None:
    with pytest.raises(ValueError, match="nonnegative"):
        Event(
            event_id="e1", trial_id="t1", source="robot", kind=EventKind.RAW_ODOMETRY, receipt_ns=-1
        )


def test_rejects_occurrence_fields_set_independently() -> None:
    with pytest.raises(ValueError, match="together"):
        Event(
            event_id="e1",
            trial_id="t1",
            source="robot",
            kind=EventKind.RAW_ODOMETRY,
            receipt_ns=0,
            occurrence_ns=5,
        )
    with pytest.raises(ValueError, match="together"):
        Event(
            event_id="e1",
            trial_id="t1",
            source="robot",
            kind=EventKind.RAW_ODOMETRY,
            receipt_ns=0,
            occurrence_domain=ClockDomain.MONOTONIC,
        )
