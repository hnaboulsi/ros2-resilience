from __future__ import annotations

from conftest import healthy_control_dict, scenario_dict

from ros2_resilience.core.assertions import AssertionType, Timeline, evaluate_scenario
from ros2_resilience.core.events import Event, EventKind
from ros2_resilience.core.validation import parse_scenario

SEC = 1_000_000_000


def ev(kind: EventKind, receipt_ns: int, **payload: object) -> Event:
    return Event(
        event_id=f"e{receipt_ns}-{kind}",
        trial_id="t",
        source="s",
        kind=kind,
        receipt_ns=receipt_ns,
        payload=payload,
    )


def result_map(results):
    return {r.type: r for r in results}


def test_dropout_scenario_full_pass() -> None:
    scenario = parse_scenario(scenario_dict())
    activation_ns = 10 * SEC
    timeline = Timeline(
        baseline_start_ns=activation_ns - 1 * SEC,
        activation_ns=activation_ns,
        deactivation_ns=activation_ns + 3 * SEC,
        observation_deadline_ns=activation_ns + 4 * SEC,
    )
    events = [
        ev(
            EventKind.RAW_ODOMETRY,
            activation_ns + int(0.1 * SEC),
            linear_x=0.5,
            angular_z=0.0,
            stamp_ns=1,
        ),
        ev(
            EventKind.RAW_ODOMETRY,
            activation_ns + int(0.2 * SEC),
            linear_x=0.5,
            angular_z=0.0,
            stamp_ns=2,
        ),
        ev(
            EventKind.FAULTED_ODOMETRY,
            activation_ns + int(0.1 * SEC),
            linear_x=0.5,
            angular_z=0.0,
            stamp_ns=1,
        ),
        ev(
            EventKind.INJECTOR_COUNTERS,
            activation_ns + int(0.2 * SEC),
            received=2,
            forwarded=1,
            dropped=1,
        ),
        ev(
            EventKind.WATCHDOG_VIOLATION,
            activation_ns + int(0.5 * SEC),
            reason="loss_fraction_exceeded",
            age_sec=0.1,
        ),
        ev(EventKind.RECOVERY_PUBLISHED, activation_ns + int(0.55 * SEC)),
        ev(
            EventKind.COMMAND_OBSERVED,
            activation_ns + int(0.6 * SEC),
            linear_x=0.0,
            angular_z=0.0,
        ),
        ev(
            EventKind.RAW_ODOMETRY,
            activation_ns + int(0.90 * SEC),
            linear_x=0.0,
            angular_z=0.0,
            stamp_ns=3,
        ),
        ev(
            EventKind.RAW_ODOMETRY,
            activation_ns + int(0.95 * SEC),
            linear_x=0.0,
            angular_z=0.0,
            stamp_ns=4,
        ),
        ev(
            EventKind.RAW_ODOMETRY,
            activation_ns + int(1.00 * SEC),
            linear_x=0.0,
            angular_z=0.0,
            stamp_ns=5,
        ),
        ev(
            EventKind.RAW_ODOMETRY,
            activation_ns + int(1.05 * SEC),
            linear_x=0.0,
            angular_z=0.0,
            stamp_ns=6,
        ),
        ev(
            EventKind.RAW_ODOMETRY,
            activation_ns + int(1.10 * SEC),
            linear_x=0.0,
            angular_z=0.0,
            stamp_ns=7,
        ),
        ev(
            EventKind.RAW_ODOMETRY,
            activation_ns + int(1.15 * SEC),
            linear_x=0.0,
            angular_z=0.0,
            stamp_ns=8,
        ),
    ]

    results = result_map(evaluate_scenario(scenario, events, timeline))
    assert results[AssertionType.NO_EARLY_VIOLATION].passed
    assert results[AssertionType.FAULT_OBSERVABLE].passed
    assert results[AssertionType.WATCHDOG_VIOLATION_DETECTED].passed
    assert results[AssertionType.RECOVERY_COMMAND_PUBLISHED].passed
    assert results[AssertionType.RECOVERY_COMMAND_OBSERVED].passed
    assert results[AssertionType.ROBOT_STOPPED].passed
    assert results[AssertionType.ROBOT_STOPPED_HELD].passed
    assert all(r.passed for r in results.values())


def test_dropout_fault_not_observable_without_drop_counters() -> None:
    scenario = parse_scenario(scenario_dict())
    activation_ns = 10 * SEC
    timeline = Timeline(activation_ns - SEC, activation_ns, None, activation_ns + 4 * SEC)
    events = [
        ev(EventKind.RAW_ODOMETRY, activation_ns + 100, linear_x=0.5, angular_z=0.0, stamp_ns=1)
    ]

    results = result_map(evaluate_scenario(scenario, events, timeline))
    assert results[AssertionType.FAULT_OBSERVABLE].passed is False
    assert "drop counters" in results[AssertionType.FAULT_OBSERVABLE].reason


def test_early_violation_fails_the_contract() -> None:
    scenario = parse_scenario(scenario_dict())
    activation_ns = 10 * SEC
    timeline = Timeline(activation_ns - SEC, activation_ns, None, activation_ns + 4 * SEC)
    events = [
        ev(
            EventKind.WATCHDOG_VIOLATION,
            activation_ns - int(0.5 * SEC),
            reason="loss_fraction_exceeded",
        )
    ]

    results = result_map(evaluate_scenario(scenario, events, timeline))
    assert results[AssertionType.NO_EARLY_VIOLATION].passed is False


def test_missing_evidence_fails_watchdog_and_recovery_assertions() -> None:
    scenario = parse_scenario(scenario_dict())
    activation_ns = 10 * SEC
    timeline = Timeline(activation_ns - SEC, activation_ns, None, activation_ns + 4 * SEC)

    results = result_map(evaluate_scenario(scenario, [], timeline))
    assert results[AssertionType.WATCHDOG_VIOLATION_DETECTED].passed is False
    assert results[AssertionType.RECOVERY_COMMAND_PUBLISHED].passed is False
    assert results[AssertionType.RECOVERY_COMMAND_OBSERVED].passed is False
    assert results[AssertionType.ROBOT_STOPPED].passed is False


def test_latency_fault_observable_from_matched_stamps() -> None:
    scenario = parse_scenario(
        scenario_dict(
            fault={"type": "latency", "parameters": {"delay_sec": 0.5}, "duration_sec": 3.0}
        )
    )
    activation_ns = 10 * SEC
    timeline = Timeline(activation_ns - SEC, activation_ns, None, activation_ns + 4 * SEC)
    events = [
        ev(EventKind.RAW_ODOMETRY, activation_ns, linear_x=0.5, angular_z=0.0, stamp_ns=100),
        ev(
            EventKind.FAULTED_ODOMETRY,
            activation_ns + int(0.5 * SEC),
            linear_x=0.5,
            angular_z=0.0,
            stamp_ns=100,
        ),
    ]
    results = result_map(evaluate_scenario(scenario, events, timeline))
    assert results[AssertionType.FAULT_OBSERVABLE].passed


def test_stale_fault_observable_from_held_timestamp() -> None:
    scenario = parse_scenario(
        scenario_dict(fault={"type": "stale", "parameters": {}, "duration_sec": 3.0})
    )
    activation_ns = 10 * SEC
    timeline = Timeline(activation_ns - SEC, activation_ns, None, activation_ns + 4 * SEC)
    events = [
        ev(EventKind.RAW_ODOMETRY, activation_ns, linear_x=0.5, angular_z=0.0, stamp_ns=1),
        ev(
            EventKind.RAW_ODOMETRY,
            activation_ns + int(0.1 * SEC),
            linear_x=0.5,
            angular_z=0.0,
            stamp_ns=2,
        ),
        ev(EventKind.FAULTED_ODOMETRY, activation_ns, linear_x=0.5, angular_z=0.0, stamp_ns=1),
        ev(
            EventKind.FAULTED_ODOMETRY,
            activation_ns + int(0.1 * SEC),
            linear_x=0.5,
            angular_z=0.0,
            stamp_ns=1,
        ),
    ]
    results = result_map(evaluate_scenario(scenario, events, timeline))
    assert results[AssertionType.FAULT_OBSERVABLE].passed


def test_robot_stopped_held_fails_on_late_spike() -> None:
    scenario = parse_scenario(scenario_dict())
    activation_ns = 10 * SEC
    timeline = Timeline(activation_ns - SEC, activation_ns, None, activation_ns + 4 * SEC)
    events = [
        ev(
            EventKind.RAW_ODOMETRY,
            activation_ns + int(0.1 * SEC),
            linear_x=0.0,
            angular_z=0.0,
            stamp_ns=1,
        ),
        ev(
            EventKind.RAW_ODOMETRY,
            activation_ns + int(0.2 * SEC),
            linear_x=0.5,
            angular_z=0.0,
            stamp_ns=2,
        ),
    ]
    results = result_map(evaluate_scenario(scenario, events, timeline))
    assert results[AssertionType.ROBOT_STOPPED].passed
    assert results[AssertionType.ROBOT_STOPPED_HELD].passed is False


def test_healthy_control_pass() -> None:
    scenario = parse_scenario(healthy_control_dict())
    timeline = Timeline(0, 1 * SEC, None, 4 * SEC)
    events = [
        ev(EventKind.RAW_ODOMETRY, t, linear_x=0.5, angular_z=0.0, stamp_ns=t)
        for t in range(0, 4 * SEC, int(0.1 * SEC))
    ]

    results = result_map(evaluate_scenario(scenario, events, timeline))
    assert results[AssertionType.NO_VIOLATION_DURING_OBSERVATION].passed
    assert results[AssertionType.NO_RECOVERY_COMMAND].passed


def test_healthy_control_fails_on_unexpected_violation() -> None:
    scenario = parse_scenario(healthy_control_dict())
    timeline = Timeline(0, 1 * SEC, None, 4 * SEC)
    events = [ev(EventKind.WATCHDOG_VIOLATION, 2 * SEC, reason="message_age_exceeded")]

    results = result_map(evaluate_scenario(scenario, events, timeline))
    assert results[AssertionType.NO_VIOLATION_DURING_OBSERVATION].passed is False


def test_healthy_control_fails_on_unexpected_recovery() -> None:
    scenario = parse_scenario(healthy_control_dict())
    timeline = Timeline(0, 1 * SEC, None, 4 * SEC)
    events = [ev(EventKind.RECOVERY_PUBLISHED, 2 * SEC)]

    results = result_map(evaluate_scenario(scenario, events, timeline))
    assert results[AssertionType.NO_RECOVERY_COMMAND].passed is False
