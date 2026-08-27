"""Timestamp-age plus trailing-window loss-fraction health detection.

The evaluator owns no clock and no subscriptions: callers report each
arrival via :meth:`record_arrival` and ask for a verdict at an explicit time
via :meth:`check`. Detection is latched: once a violation is reported it
remains reported, with its original reason, until :meth:`reset` is called
for a new trial.

Two independent signals feed the verdict, both expressed in the same clock
domain as ``now_ns`` passed to :meth:`check`:

* physical arrival count in the trailing window, driving loss-fraction
  detection (a stream that is genuinely dropping messages);
* content freshness (``stamp_ns``, typically a message's own header stamp),
  driving age detection (a stream that keeps arriving on schedule but whose
  payload has stopped advancing, as under a stale-hold or latency fault).

When ``stamp_ns`` is omitted, it defaults to ``now_ns``, treating the
arrival as fresh content -- the two signals then coincide, matching a
simple healthy or dropout-only stream.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import StrEnum

__all__ = ["ViolationReason", "WatchdogEvaluator", "WatchdogStatus"]


class ViolationReason(StrEnum):
    """Why the watchdog reported a violation."""

    MESSAGE_AGE_EXCEEDED = "message_age_exceeded"
    LOSS_FRACTION_EXCEEDED = "loss_fraction_exceeded"


@dataclass(frozen=True)
class WatchdogStatus:
    """The watchdog's verdict at one check, plus the values behind it."""

    violated: bool
    reason: ViolationReason | None
    age_sec: float | None
    loss_fraction: float | None


class WatchdogEvaluator:
    """Detect stale or under-arriving traffic on one monitored stream."""

    def __init__(
        self,
        *,
        max_age_sec: float,
        expected_rate_hz: float,
        loss_window_sec: float,
        max_loss_fraction: float,
    ) -> None:
        self._max_age_ns = self._seconds_to_ns(self._validate_positive("max_age_sec", max_age_sec))
        self._expected_rate_hz = self._validate_positive("expected_rate_hz", expected_rate_hz)
        self._loss_window_ns = self._seconds_to_ns(
            self._validate_positive("loss_window_sec", loss_window_sec)
        )
        self._max_loss_fraction = self._validate_fraction("max_loss_fraction", max_loss_fraction)
        self._arrivals: list[int] = []
        self._first_arrival_ns: int | None = None
        self._last_arrival_ns: int | None = None
        self._latest_stamp_ns: int | None = None
        self._latched = False
        self._latched_reason: ViolationReason | None = None

    def record_arrival(self, now_ns: int, *, stamp_ns: int | None = None) -> None:
        """Record a physical arrival at ``now_ns`` carrying content stamped ``stamp_ns``."""
        now = self._validate_nonnegative_int("now_ns", now_ns)
        if self._last_arrival_ns is not None and now < self._last_arrival_ns:
            raise ValueError("now_ns must not move backwards")
        effective_stamp = (
            self._validate_nonnegative_int("stamp_ns", stamp_ns) if stamp_ns is not None else now
        )
        if self._first_arrival_ns is None:
            self._first_arrival_ns = now
        self._arrivals.append(now)
        self._last_arrival_ns = now
        if self._latest_stamp_ns is None or effective_stamp > self._latest_stamp_ns:
            self._latest_stamp_ns = effective_stamp

    def reset(self) -> None:
        """Clear latched state and arrival history for a new trial."""
        self._arrivals.clear()
        self._first_arrival_ns = None
        self._last_arrival_ns = None
        self._latest_stamp_ns = None
        self._latched = False
        self._latched_reason = None

    def check(self, now_ns: int) -> WatchdogStatus:
        """Return the current verdict, latching any newly detected violation.

        The loss-fraction expectation ramps up over the first
        ``loss_window_sec`` after the first-ever arrival, so a stream that
        has only just started is not penalized for a window it has not had
        time to fill yet.
        """
        now = self._validate_nonnegative_int("now_ns", now_ns)
        window_start = now - self._loss_window_ns
        self._arrivals = [t for t in self._arrivals if t >= window_start]

        if self._last_arrival_ns is None:
            if self._latched:
                return WatchdogStatus(True, self._latched_reason, None, None)
            return WatchdogStatus(False, None, None, None)

        assert self._first_arrival_ns is not None
        assert self._latest_stamp_ns is not None
        age_sec = (now - self._latest_stamp_ns) / 1e9
        elapsed_ns = min(self._loss_window_ns, now - self._first_arrival_ns)
        expected_count = self._expected_rate_hz * (elapsed_ns / 1e9)
        loss_fraction = (
            max(0.0, 1.0 - len(self._arrivals) / expected_count) if expected_count > 0.0 else 0.0
        )

        if self._latched:
            return WatchdogStatus(True, self._latched_reason, age_sec, loss_fraction)

        reason: ViolationReason | None = None
        if now - self._latest_stamp_ns > self._max_age_ns:
            reason = ViolationReason.MESSAGE_AGE_EXCEEDED
        elif loss_fraction > self._max_loss_fraction:
            reason = ViolationReason.LOSS_FRACTION_EXCEEDED

        if reason is None:
            return WatchdogStatus(False, None, age_sec, loss_fraction)

        self._latched = True
        self._latched_reason = reason
        return WatchdogStatus(True, reason, age_sec, loss_fraction)

    @staticmethod
    def _seconds_to_ns(seconds: float) -> int:
        return int(round(seconds * 1e9))

    @staticmethod
    def _validate_positive(name: str, value: float) -> float:
        if not isinstance(value, (int, float)) or isinstance(value, bool):
            raise TypeError(f"{name} must be a number")
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f"{name} must be finite and positive")
        return float(value)

    @staticmethod
    def _validate_fraction(name: str, value: float) -> float:
        if not isinstance(value, (int, float)) or isinstance(value, bool):
            raise TypeError(f"{name} must be a number")
        if not math.isfinite(value) or not 0.0 <= value <= 1.0:
            raise ValueError(f"{name} must be finite and within [0.0, 1.0]")
        return float(value)

    @staticmethod
    def _validate_nonnegative_int(name: str, value: int) -> int:
        if isinstance(value, bool) or not isinstance(value, int):
            raise TypeError(f"{name} must be an integer")
        if value < 0:
            raise ValueError(f"{name} must be nonnegative")
        return value
