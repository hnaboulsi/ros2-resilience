"""Run real trials over real ROS 2 nodes and check the harness's own verdict.

These tests require a sourced ROS 2 Jazzy environment (``rclpy`` importable)
and take several real wall-clock seconds each, since nothing here is time-
accelerated. They are intentionally kept out of ``tests/unit`` (which
``pyproject.toml`` restricts pytest to by default) and are run explicitly,
for example inside the project's Docker image::

    python3 -m pytest tests/integration -q
"""

from __future__ import annotations

import copy
from pathlib import Path

import pytest
import yaml

from ros2_resilience.core.validation import parse_scenario
from ros2_resilience.ros.experiment_runner_node import run_trial

SCENARIOS_DIR = Path(__file__).resolve().parents[2] / "scenarios"


def _load(name: str, **overrides):
    with (SCENARIOS_DIR / f"{name}.yaml").open(encoding="utf-8") as handle:
        data = yaml.safe_load(handle)
    data = copy.deepcopy(data)
    for path, value in overrides.items():
        *parents, leaf = path.split(".")
        node = data
        for key in parents:
            node = node[key]
        node[leaf] = value
    return parse_scenario(data)


@pytest.mark.parametrize(
    "scenario_name",
    ["healthy_control", "dropout_burst", "delayed_odometry_stop", "stale_hold"],
)
def test_shipped_scenario_passes_for_real(scenario_name: str) -> None:
    scenario = _load(scenario_name)
    result = run_trial(
        scenario, trial_id=f"it-{scenario_name}", trial_index=0, seed=scenario.experiment.seed
    )

    assert result.infrastructure_error is None
    assert result.lifecycle_state == "finished"
    assert result.passed, [(a.type.value, a.reason) for a in result.assertions if not a.passed]


def test_impossible_deadline_produces_a_real_failure() -> None:
    """An unreachable recovery deadline must make the harness report FAIL.

    This is the negative control: it proves PASS/FAIL is driven by measured
    timing against the loaded scenario, not a hardcoded result.
    """
    scenario = _load("dropout_burst", **{"expect.recovery.observed_within_sec": 1e-9})
    result = run_trial(scenario, trial_id="it-impossible-deadline", trial_index=0, seed=99)

    assert result.infrastructure_error is None
    assert result.passed is False
    failed_types = {a.type.value for a in result.assertions if not a.passed}
    assert "recovery_command_observed" in failed_types


def test_healthy_control_reports_measurements_as_none() -> None:
    scenario = _load("healthy_control")
    result = run_trial(
        scenario, trial_id="it-healthy-metrics", trial_index=0, seed=scenario.experiment.seed
    )

    assert result.passed
    assert result.metrics == {
        "detection_latency_sec": None,
        "recovery_command_latency_sec": None,
        "command_observation_latency_sec": None,
        "time_to_stop_sec": None,
    }
