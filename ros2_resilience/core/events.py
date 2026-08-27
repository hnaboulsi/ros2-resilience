"""Normalized, transport-independent observations collected during a trial."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import StrEnum
from typing import Any

__all__ = ["ClockDomain", "Event", "EventKind"]


class EventKind(StrEnum):
    """The kind of normalized observation an :class:`Event` carries."""

    CONFIG_APPLIED = "config_applied"
    RAW_ODOMETRY = "raw_odometry"
    FAULTED_ODOMETRY = "faulted_odometry"
    INJECTOR_COUNTERS = "injector_counters"
    WATCHDOG_VIOLATION = "watchdog_violation"
    RECOVERY_PUBLISHED = "recovery_published"
    COMMAND_OBSERVED = "command_observed"
    INFRASTRUCTURE_ERROR = "infrastructure_error"


class ClockDomain(StrEnum):
    """Which clock a timestamp on an :class:`Event` was drawn from."""

    MONOTONIC = "monotonic"
    ROS = "ros"


@dataclass(frozen=True)
class Event:
    """One normalized, immutable observation.

    ``receipt_ns`` is always the observer's monotonic receive time and is
    comparable across events. ``occurrence_ns``/``occurrence_domain`` are
    populated only for events with a meaningful source-side timestamp (for
    example a message header stamp); they are never a substitute for
    ``receipt_ns`` and must not be assumed to share a clock domain with it.
    """

    event_id: str
    trial_id: str
    source: str
    kind: EventKind
    receipt_ns: int
    payload: dict[str, Any] = field(default_factory=dict)
    occurrence_ns: int | None = None
    occurrence_domain: ClockDomain | None = None

    def __post_init__(self) -> None:
        if not self.event_id:
            raise ValueError("event_id must be nonempty")
        if not self.trial_id:
            raise ValueError("trial_id must be nonempty")
        if not self.source:
            raise ValueError("source must be nonempty")
        if not isinstance(self.kind, EventKind):
            raise TypeError("kind must be an EventKind")
        if isinstance(self.receipt_ns, bool) or not isinstance(self.receipt_ns, int):
            raise TypeError("receipt_ns must be an integer")
        if self.receipt_ns < 0:
            raise ValueError("receipt_ns must be nonnegative")
        if (self.occurrence_ns is None) != (self.occurrence_domain is None):
            raise ValueError("occurrence_ns and occurrence_domain must be set together")
        if self.occurrence_ns is not None:
            if isinstance(self.occurrence_ns, bool) or not isinstance(self.occurrence_ns, int):
                raise TypeError("occurrence_ns must be an integer")
            if self.occurrence_ns < 0:
                raise ValueError("occurrence_ns must be nonnegative")
