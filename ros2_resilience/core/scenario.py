"""Typed configuration for one resilience-contract scenario."""

from __future__ import annotations

from dataclasses import dataclass
from enum import StrEnum

__all__ = [
    "Expectations",
    "ExperimentTiming",
    "FaultExpectation",
    "FaultKind",
    "FaultSpec",
    "RecoveryExpectation",
    "RobotConfig",
    "RobotExpectation",
    "ScenarioConfig",
    "SystemConfig",
    "WatchdogConfig",
    "WatchdogExpectation",
]

SUPPORTED_SCHEMA_VERSION = 1
SUPPORTED_MESSAGE_TYPE = "nav_msgs/msg/Odometry"
SUPPORTED_RECOVERY_ACTION = "stop"


class FaultKind(StrEnum):
    """The fault mode a scenario applies during its activation window."""

    NONE = "none"
    DROPOUT = "dropout"
    LATENCY = "latency"
    STALE = "stale"


@dataclass(frozen=True)
class RobotConfig:
    """Kinematic robot behavior before and during a trial."""

    publish_rate_hz: float
    initial_linear_velocity_mps: float
    initial_angular_velocity_rad_s: float


@dataclass(frozen=True)
class SystemConfig:
    """Topic wiring and robot behavior for the ROS graph under test."""

    input_topic: str
    output_topic: str
    message_type: str
    command_topic: str
    robot: RobotConfig


@dataclass(frozen=True)
class ExperimentTiming:
    """Scheduling knobs independent of the fault or its evaluation."""

    seed: int
    readiness_timeout_sec: float
    healthy_baseline_sec: float
    observation_after_activation_sec: float


@dataclass(frozen=True)
class FaultSpec:
    """The fault applied after the healthy baseline, and for how long."""

    type: FaultKind
    parameters: dict[str, float]
    duration_sec: float


@dataclass(frozen=True)
class WatchdogConfig:
    """Detection thresholds for the independent watchdog observer."""

    max_age_sec: float
    expected_rate_hz: float
    loss_window_sec: float
    max_loss_fraction: float
    check_period_sec: float
    recovery: str


@dataclass(frozen=True)
class FaultExpectation:
    """Deadline for independently observable fault evidence."""

    observable: bool
    within_sec: float


@dataclass(frozen=True)
class WatchdogExpectation:
    """Expected violation reason and detection deadline."""

    violation: str
    within_sec: float


@dataclass(frozen=True)
class RecoveryExpectation:
    """Deadlines for recovery publication and for its observed effect."""

    action: str
    command_within_sec: float
    observed_within_sec: float


@dataclass(frozen=True)
class RobotExpectation:
    """Stop thresholds, deadline, and required hold duration."""

    linear_velocity_below_mps: float
    angular_velocity_below_rad_s: float
    within_sec: float
    hold_sec: float


@dataclass(frozen=True)
class Expectations:
    """Behavioral requirements evaluated against collected evidence.

    For a healthy control (``fault.type == FaultKind.NONE``), ``fault``,
    ``watchdog``, and ``recovery`` are always ``None`` and
    ``no_violation_before_fault`` is evaluated across the whole observation
    window rather than only up to activation.
    """

    no_violation_before_fault: bool
    fault: FaultExpectation | None
    watchdog: WatchdogExpectation | None
    recovery: RecoveryExpectation | None
    robot: RobotExpectation | None


@dataclass(frozen=True)
class ScenarioConfig:
    """A complete, validated resilience-contract scenario."""

    schema_version: int
    name: str
    system: SystemConfig
    experiment: ExperimentTiming
    fault: FaultSpec
    watchdog: WatchdogConfig
    expect: Expectations

    @property
    def is_healthy_control(self) -> bool:
        """Return whether this scenario applies no fault at all."""
        return self.fault.type is FaultKind.NONE
