from __future__ import annotations

import json

from ros2_resilience.core.assertions import AssertionResult, AssertionType
from ros2_resilience.core.events import Event, EventKind
from ros2_resilience.core.results import RESULTS_SCHEMA_VERSION, BatchResult, TrialResult, aggregate


def make_trial(trial_index: int, *, passed: bool, detection: float | None) -> TrialResult:
    return TrialResult(
        trial_id=f"batch-{trial_index:04d}",
        trial_index=trial_index,
        seed=7 + trial_index,
        scenario_name="test_scenario",
        lifecycle_state="finished",
        passed=passed,
        infrastructure_error=None,
        activation_ns=1_000,
        deactivation_ns=2_000,
        assertions=[AssertionResult(type=AssertionType.ROBOT_STOPPED, passed=passed, reason="x")],
        metrics={"detection_latency_sec": detection, "time_to_stop_sec": 0.5},
        events=[
            Event(
                event_id="e1", trial_id="t", source="s", kind=EventKind.RAW_ODOMETRY, receipt_ns=1
            )
        ],
    )


def test_trial_to_dict_is_json_serializable() -> None:
    trial = make_trial(0, passed=True, detection=0.1)
    payload = json.dumps(trial.to_dict())
    decoded = json.loads(payload)
    assert decoded["trial_id"] == "batch-0000"
    assert decoded["assertions"][0]["type"] == "robot_stopped"
    assert decoded["metrics"]["detection_latency_sec"] == 0.1


def test_aggregate_reports_missing_and_stats() -> None:
    trials = [
        make_trial(0, passed=True, detection=0.1),
        make_trial(1, passed=False, detection=None),
    ]
    summary = aggregate(trials)
    assert summary["detection_latency_sec"]["count"] == 1
    assert summary["detection_latency_sec"]["missing"] == 1
    assert summary["detection_latency_sec"]["mean"] == 0.1
    assert summary["time_to_stop_sec"]["count"] == 2
    assert summary["time_to_stop_sec"]["mean"] == 0.5


def test_aggregate_empty_population_has_null_summary() -> None:
    summary = aggregate([])
    assert summary["detection_latency_sec"] == {
        "count": 0,
        "missing": 0,
        "mean": None,
        "minimum": None,
        "maximum": None,
    }


def test_aggregate_includes_failed_trials_when_metric_is_valid() -> None:
    trials = [make_trial(0, passed=False, detection=0.9)]
    summary = aggregate(trials)
    assert summary["detection_latency_sec"]["count"] == 1
    assert summary["detection_latency_sec"]["mean"] == 0.9


def test_batch_result_final_status_and_counts() -> None:
    trials = [make_trial(0, passed=True, detection=0.1), make_trial(1, passed=False, detection=0.2)]
    batch = BatchResult(
        scenario_name="test_scenario",
        scenario_config={"name": "test_scenario"},
        scenario_yaml="name: test_scenario\n",
        config_hash="abc123",
        requested_trials=2,
        trials=trials,
    )
    payload = batch.to_dict()
    assert payload["schema_version"] == RESULTS_SCHEMA_VERSION
    assert payload["passed_trials"] == 1
    assert payload["failed_trials"] == 1
    assert payload["final_status"] == "fail"
    assert json.dumps(payload)


def test_batch_result_all_pass_status() -> None:
    trials = [make_trial(0, passed=True, detection=0.1)]
    batch = BatchResult(
        scenario_name="s",
        scenario_config={},
        scenario_yaml="",
        config_hash="h",
        requested_trials=1,
        trials=trials,
    )
    assert batch.to_dict()["final_status"] == "pass"
