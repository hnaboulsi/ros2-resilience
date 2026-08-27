"""Versioned, JSON-serializable trial and batch results."""

from __future__ import annotations

import statistics
from dataclasses import asdict, dataclass, field
from typing import Any

from ros2_resilience.core.assertions import AssertionResult
from ros2_resilience.core.events import Event

__all__ = ["BatchResult", "MetricSummary", "RESULTS_SCHEMA_VERSION", "TrialResult", "aggregate"]

RESULTS_SCHEMA_VERSION = 1

_METRIC_NAMES = (
    "detection_latency_sec",
    "recovery_command_latency_sec",
    "command_observation_latency_sec",
    "time_to_stop_sec",
)


@dataclass(frozen=True)
class TrialResult:
    """The complete, serializable outcome of one trial."""

    trial_id: str
    trial_index: int
    seed: int
    scenario_name: str
    lifecycle_state: str
    passed: bool
    infrastructure_error: str | None
    activation_ns: int | None
    deactivation_ns: int | None
    assertions: list[AssertionResult]
    metrics: dict[str, float | None]
    events: list[Event]

    def to_dict(self) -> dict[str, Any]:
        """Return a JSON-ready mapping for this trial."""
        return {
            "trial_id": self.trial_id,
            "trial_index": self.trial_index,
            "seed": self.seed,
            "scenario_name": self.scenario_name,
            "lifecycle_state": self.lifecycle_state,
            "passed": self.passed,
            "infrastructure_error": self.infrastructure_error,
            "activation_ns": self.activation_ns,
            "deactivation_ns": self.deactivation_ns,
            "assertions": [asdict(result) for result in self.assertions],
            "metrics": self.metrics,
            "events": [asdict(event) for event in self.events],
        }


@dataclass(frozen=True)
class MetricSummary:
    """Aggregate statistics for one metric across a batch."""

    count: int
    missing: int
    mean: float | None
    minimum: float | None
    maximum: float | None

    def to_dict(self) -> dict[str, Any]:
        """Return a JSON-ready mapping for this summary."""
        return asdict(self)


@dataclass(frozen=True)
class BatchResult:
    """A versioned batch of trial results plus their aggregates."""

    scenario_name: str
    scenario_config: dict[str, Any]
    scenario_yaml: str
    config_hash: str
    requested_trials: int
    trials: list[TrialResult] = field(default_factory=list)

    def to_dict(self) -> dict[str, Any]:
        """Return a JSON-ready mapping including aggregates and final status."""
        passed_count = sum(1 for trial in self.trials if trial.passed)
        failed_count = len(self.trials) - passed_count
        return {
            "schema_version": RESULTS_SCHEMA_VERSION,
            "scenario_name": self.scenario_name,
            "scenario_config": self.scenario_config,
            "scenario_yaml": self.scenario_yaml,
            "config_hash": self.config_hash,
            "requested_trials": self.requested_trials,
            "executed_trials": len(self.trials),
            "passed_trials": passed_count,
            "failed_trials": failed_count,
            "final_status": "pass" if passed_count == self.requested_trials else "fail",
            "aggregates": aggregate(self.trials),
            "trials": [trial.to_dict() for trial in self.trials],
        }


def aggregate(trials: list[TrialResult]) -> dict[str, dict[str, Any]]:
    """Compute count/missing/mean/min/max for each known metric across trials."""
    summaries: dict[str, dict[str, Any]] = {}
    for name in _METRIC_NAMES:
        values = [trial.metrics.get(name) for trial in trials]
        present = [v for v in values if v is not None]
        missing = len(values) - len(present)
        summary = MetricSummary(
            count=len(present),
            missing=missing,
            mean=statistics.fmean(present) if present else None,
            minimum=min(present) if present else None,
            maximum=max(present) if present else None,
        )
        summaries[name] = summary.to_dict()
    return summaries
