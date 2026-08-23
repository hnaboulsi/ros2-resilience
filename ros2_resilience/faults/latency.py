"""Deterministic, in-memory latency injection for message queues."""

from __future__ import annotations

import heapq
import math
from collections.abc import Callable
from copy import deepcopy
from numbers import Real

__all__ = ["LatencyQueue", "QueueOverflowError"]


class QueueOverflowError(RuntimeError):
    """Raised when an arrival would exceed a latency queue's capacity."""


class LatencyQueue[T]:
    """Hold arrivals until their configured, optionally jittered deadline.

    Timestamps are nanoseconds and must never move backwards for queue
    operations.  Configuration applies only when a message is enqueued, so a
    later configuration change cannot alter an already assigned deadline.
    """

    def __init__(
        self,
        *,
        delay_ns: int,
        jitter_ns: int = 0,
        max_pending: int = 1000,
        uniform: Callable[[float, float], float],
    ) -> None:
        self._delay_ns, self._jitter_ns = self._validate_configuration(
            delay_ns=delay_ns,
            jitter_ns=jitter_ns,
        )
        self._max_pending = self._validate_positive_int("max_pending", max_pending)
        if not callable(uniform):
            raise TypeError("uniform must be callable")

        self._uniform = uniform
        self._entries: list[tuple[int, int, T]] = []
        self._last_time_ns: int | None = None
        self._next_sequence = 0

    @property
    def pending_count(self) -> int:
        """Return the number of messages awaiting delivery."""
        return len(self._entries)

    def configure(self, *, delay_ns: int, jitter_ns: int) -> None:
        """Apply latency settings to future arrivals only."""
        delay, jitter = self._validate_configuration(
            delay_ns=delay_ns,
            jitter_ns=jitter_ns,
        )
        self._delay_ns = delay
        self._jitter_ns = jitter

    def enqueue(self, message: T, *, now_ns: int) -> None:
        """Copy and enqueue ``message`` with a deadline derived from ``now_ns``."""
        now = self._validate_time(now_ns)
        if len(self._entries) >= self._max_pending:
            raise QueueOverflowError(f"latency queue is full (max_pending={self._max_pending})")

        snapshot = deepcopy(message)
        delay = self._sample_delay()
        heapq.heappush(self._entries, (now + delay, self._next_sequence, snapshot))
        self._next_sequence += 1
        self._last_time_ns = now

    def pop_ready(self, *, now_ns: int) -> list[T]:
        """Return all messages whose deadlines are at or before ``now_ns``."""
        now = self._validate_time(now_ns)
        ready: list[T] = []
        while self._entries and self._entries[0][0] <= now:
            _, _, message = heapq.heappop(self._entries)
            ready.append(message)
        self._last_time_ns = now
        return ready

    def flush(self) -> list[T]:
        """Return and remove all pending messages in their original arrival order."""
        arrivals = [message for _, _, message in sorted(self._entries, key=lambda item: item[1])]
        self._entries.clear()
        return arrivals

    def _sample_delay(self) -> int:
        if self._jitter_ns == 0:
            return self._delay_ns

        sample = self._uniform(-self._jitter_ns, self._jitter_ns)
        if isinstance(sample, bool) or not isinstance(sample, Real):
            raise TypeError("uniform must return a finite real number")
        if not math.isfinite(sample):
            raise ValueError("uniform must return a finite value")
        if not -self._jitter_ns <= sample <= self._jitter_ns:
            raise ValueError("uniform returned a value outside the jitter range")
        return max(0, self._delay_ns + round(sample))

    def _validate_time(self, now_ns: int) -> int:
        now = self._validate_nonnegative_int("now_ns", now_ns)
        if self._last_time_ns is not None and now < self._last_time_ns:
            raise ValueError(
                f"now_ns must not move backwards (last={self._last_time_ns}, got={now})"
            )
        return now

    @classmethod
    def _validate_configuration(cls, *, delay_ns: int, jitter_ns: int) -> tuple[int, int]:
        return (
            cls._validate_nonnegative_int("delay_ns", delay_ns),
            cls._validate_nonnegative_int("jitter_ns", jitter_ns),
        )

    @staticmethod
    def _validate_nonnegative_int(name: str, value: int) -> int:
        if isinstance(value, bool) or not isinstance(value, int):
            raise TypeError(f"{name} must be an integer")
        if value < 0:
            raise ValueError(f"{name} must be nonnegative")
        return value

    @staticmethod
    def _validate_positive_int(name: str, value: int) -> int:
        if isinstance(value, bool) or not isinstance(value, int):
            raise TypeError(f"{name} must be an integer")
        if value <= 0:
            raise ValueError(f"{name} must be positive")
        return value
