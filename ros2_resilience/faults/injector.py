"""Coordinate one deterministic in-memory fault mode at a time."""

from __future__ import annotations

import math
import random
from dataclasses import dataclass
from enum import StrEnum

from ros2_resilience.faults.dropout import Dropout
from ros2_resilience.faults.latency import LatencyQueue, QueueOverflowError
from ros2_resilience.faults.stale import StaleHold

__all__ = ["FaultConfig", "FaultCounters", "FaultInjector", "FaultType"]


class FaultType(StrEnum):
    """The single fault mode selected by a :class:`FaultConfig`."""

    NONE = "none"
    DROPOUT = "dropout"
    LATENCY = "latency"
    STALE = "stale"


@dataclass(frozen=True)
class FaultConfig:
    """Validated configuration for a :class:`FaultInjector`."""

    enabled: bool = False
    fault_type: FaultType = FaultType.NONE
    drop_probability: float = 0.0
    delay_ns: int = 0
    jitter_ns: int = 0
    seed: int = 0

    def __post_init__(self) -> None:
        if not isinstance(self.enabled, bool):
            raise TypeError("enabled must be a bool")
        if not isinstance(self.fault_type, FaultType):
            raise TypeError("fault_type must be a FaultType")
        self._validate_probability(self.drop_probability)
        self._validate_nonnegative_int("delay_ns", self.delay_ns)
        self._validate_nonnegative_int("jitter_ns", self.jitter_ns)
        self._validate_nonnegative_int("seed", self.seed)

    @staticmethod
    def _validate_probability(value: float | int) -> float:
        if type(value) not in (int, float):
            raise TypeError("drop_probability must be an int or float")
        if not math.isfinite(value) or not 0.0 <= value <= 1.0:
            raise ValueError("drop_probability must be finite and within [0.0, 1.0]")
        return float(value)

    @staticmethod
    def _validate_nonnegative_int(name: str, value: int) -> int:
        if isinstance(value, bool) or not isinstance(value, int):
            raise TypeError(f"{name} must be an integer")
        if value < 0:
            raise ValueError(f"{name} must be nonnegative")
        return value


@dataclass(frozen=True)
class FaultCounters:
    """Cumulative counts for accepted inputs and emitted fault outputs."""

    received: int = 0
    forwarded: int = 0
    dropped: int = 0


class FaultInjector[T]:
    """Apply exactly one fault mode to explicit-time message arrivals.

    The injector owns no clock. Every operation receives a monotonic timestamp
    from its caller, which makes the component deterministic and ROS-free.
    """

    def __init__(
        self,
        config: FaultConfig,
        *,
        max_pending: int = 1000,
        rng: random.Random | None = None,
    ) -> None:
        self._validate_config(config)
        self._max_pending = self._validate_positive_int("max_pending", max_pending)
        if rng is not None and not isinstance(rng, random.Random):
            raise TypeError("rng must be a random.Random or None")

        self._external_rng = rng is not None
        self._rng = rng if rng is not None else random.Random(config.seed)
        self._config = config
        self._dropout = Dropout(config.drop_probability, random_value=self._rng.random)
        self._latency = LatencyQueue[T](
            delay_ns=config.delay_ns,
            jitter_ns=config.jitter_ns,
            max_pending=self._max_pending,
            uniform=self._rng.uniform,
        )
        self._stale = StaleHold[T]()
        self._stale.set_enabled(self._stale_is_active(config))
        self._last_time_ns: int | None = None
        self._received = 0
        self._forwarded = 0
        self._dropped = 0

    @property
    def config(self) -> FaultConfig:
        """Return the current immutable configuration."""
        return self._config

    @property
    def pending_count(self) -> int:
        """Return messages waiting in the latency queue."""
        return self._latency.pending_count

    @property
    def counters(self) -> FaultCounters:
        """Return an immutable snapshot of the cumulative counters."""
        return FaultCounters(
            received=self._received,
            forwarded=self._forwarded,
            dropped=self._dropped,
        )

    def receive(self, message: T, *, now_ns: int) -> list[T]:
        """Accept one raw input and return any output currently available."""
        now = self._validate_time(now_ns)
        if self._latency_is_active(self._config) and self.pending_count >= self._max_pending:
            raise QueueOverflowError(f"latency queue is full (max_pending={self._max_pending})")

        if not self._config.enabled or self._config.fault_type is FaultType.NONE:
            stale_output = self._stale.receive(message)
            outputs = [stale_output]
        elif self._config.fault_type is FaultType.DROPOUT:
            if self._dropout.should_drop():
                self._stale.receive(message)
                self._dropped += 1
                outputs = []
            else:
                stale_output = self._stale.receive(message)
                outputs = [stale_output]
        elif self._config.fault_type is FaultType.LATENCY:
            self._latency.enqueue(message, now_ns=now)
            self._stale.receive(message)
            outputs = self._latency.pop_ready(now_ns=now)
        else:
            stale_output = self._stale.receive(message)
            outputs = [stale_output]

        self._received += 1
        self._last_time_ns = now
        self._forwarded += len(outputs)
        return outputs

    def poll(self, *, now_ns: int) -> list[T]:
        """Return latency messages whose deadlines have passed by ``now_ns``."""
        now = self._validate_time(now_ns)
        outputs = (
            self._latency.pop_ready(now_ns=now) if self._latency_is_active(self._config) else []
        )
        self._last_time_ns = now
        self._forwarded += len(outputs)
        return outputs

    def configure(self, config: FaultConfig, *, now_ns: int) -> list[T]:
        """Switch configuration and flush latency on exit from that mode."""
        self._validate_config(config)
        now = self._validate_time(now_ns)
        if self._external_rng and config.seed != self._config.seed:
            raise ValueError("cannot change seed when an external rng is supplied")

        leaving_latency = self._latency_is_active(self._config) and not self._latency_is_active(
            config
        )
        flushed = self._latency.flush() if leaving_latency else []

        if not self._external_rng and config.seed != self._config.seed:
            self._rng.seed(config.seed)
        self._dropout = Dropout(config.drop_probability, random_value=self._rng.random)
        self._latency.configure(delay_ns=config.delay_ns, jitter_ns=config.jitter_ns)
        self._stale.set_enabled(self._stale_is_active(config))
        self._config = config
        self._last_time_ns = now
        self._forwarded += len(flushed)
        return flushed

    @staticmethod
    def _validate_config(config: FaultConfig) -> None:
        if not isinstance(config, FaultConfig):
            raise TypeError("config must be a FaultConfig")

    def _validate_time(self, now_ns: int) -> int:
        now = FaultConfig._validate_nonnegative_int("now_ns", now_ns)
        if self._last_time_ns is not None and now < self._last_time_ns:
            raise ValueError(
                f"now_ns must not move backwards (last={self._last_time_ns}, got={now})"
            )
        return now

    @staticmethod
    def _validate_positive_int(name: str, value: int) -> int:
        if isinstance(value, bool) or not isinstance(value, int):
            raise TypeError(f"{name} must be an integer")
        if value <= 0:
            raise ValueError(f"{name} must be positive")
        return value

    @staticmethod
    def _stale_is_active(config: FaultConfig) -> bool:
        return config.enabled and config.fault_type is FaultType.STALE

    @staticmethod
    def _latency_is_active(config: FaultConfig) -> bool:
        return config.enabled and config.fault_type is FaultType.LATENCY
