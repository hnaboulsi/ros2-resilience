from __future__ import annotations

import pytest
from conftest import healthy_control_dict, scenario_dict

from ros2_resilience.core.experiment import (
    Action,
    ActionType,
    LifecycleError,
    LifecycleState,
    TrialLifecycle,
)
from ros2_resilience.core.validation import parse_scenario

SEC = 1_000_000_000


def fault_scenario():
    return parse_scenario(
        scenario_dict(
            experiment={
                "seed": 1,
                "readiness_timeout_sec": 5.0,
                "healthy_baseline_sec": 1.0,
                "observation_after_activation_sec": 4.0,
            },
            fault={"type": "dropout", "parameters": {"drop_probability": 0.8}, "duration_sec": 2.0},
        )
    )


def healthy_scenario():
    return parse_scenario(
        healthy_control_dict(
            experiment={
                "seed": 1,
                "readiness_timeout_sec": 5.0,
                "healthy_baseline_sec": 1.0,
                "observation_after_activation_sec": 3.0,
            }
        )
    )


def test_full_fault_scenario_lifecycle() -> None:
    lifecycle = TrialLifecycle(fault_scenario())
    lifecycle.start(now_ns=0)
    assert lifecycle.state is LifecycleState.STARTING

    lifecycle.mark_ready(now_ns=100)
    assert lifecycle.state is LifecycleState.BASELINING
    assert lifecycle.baseline_start_ns == 100

    assert lifecycle.tick(now_ns=100 + int(0.5 * SEC)) is None
    assert lifecycle.state is LifecycleState.BASELINING

    action = lifecycle.tick(now_ns=100 + 1 * SEC)
    assert action == Action(ActionType.ACTIVATE_FAULT)
    assert lifecycle.state is LifecycleState.ACTIVATING

    lifecycle.on_config_applied(revision=99, now_ns=100 + 1 * SEC + 10)
    assert lifecycle.state is LifecycleState.ACTIVATING

    activation_time = 100 + 1 * SEC + 20
    lifecycle.on_config_applied(revision=lifecycle.activation_revision, now_ns=activation_time)
    assert lifecycle.state is LifecycleState.OBSERVING
    assert lifecycle.activation_ns == activation_time
    assert lifecycle.observation_deadline_ns == activation_time + 4 * SEC

    assert lifecycle.tick(now_ns=activation_time + int(1.9 * SEC)) is None

    deactivate = lifecycle.tick(now_ns=activation_time + 2 * SEC)
    assert deactivate == Action(ActionType.DEACTIVATE_FAULT)
    assert lifecycle.deactivation_requested_ns == activation_time + 2 * SEC

    assert lifecycle.tick(now_ns=activation_time + 2 * SEC + 1) is None

    finish = lifecycle.tick(now_ns=activation_time + 4 * SEC)
    assert finish == Action(ActionType.FINISH_TRIAL)
    assert lifecycle.state is LifecycleState.FINALIZING

    lifecycle.finish()
    assert lifecycle.state is LifecycleState.FINISHED


def test_healthy_control_skips_activation() -> None:
    lifecycle = TrialLifecycle(healthy_scenario())
    lifecycle.start(now_ns=0)
    lifecycle.mark_ready(now_ns=0)

    action = lifecycle.tick(now_ns=1 * SEC)
    assert action is None
    assert lifecycle.state is LifecycleState.OBSERVING
    assert lifecycle.activation_ns == 1 * SEC

    finish = lifecycle.tick(now_ns=1 * SEC + 3 * SEC)
    assert finish == Action(ActionType.FINISH_TRIAL)


def test_readiness_timeout_fails_trial() -> None:
    lifecycle = TrialLifecycle(fault_scenario())
    lifecycle.start(now_ns=0)

    action = lifecycle.tick(now_ns=6 * SEC)
    assert action is not None and action.type is ActionType.FINISH_TRIAL
    assert lifecycle.state is LifecycleState.FINALIZING
    assert lifecycle.infrastructure_error == "readiness timeout"


def test_activation_ack_timeout_fails_trial() -> None:
    lifecycle = TrialLifecycle(fault_scenario())
    lifecycle.start(now_ns=0)
    lifecycle.mark_ready(now_ns=0)
    lifecycle.tick(now_ns=1 * SEC)
    assert lifecycle.state is LifecycleState.ACTIVATING

    action = lifecycle.tick(now_ns=1 * SEC + 6 * SEC)
    assert action is not None and action.type is ActionType.FINISH_TRIAL
    assert lifecycle.infrastructure_error == "fault activation not acknowledged in time"


def test_fail_can_be_called_from_observing() -> None:
    lifecycle = TrialLifecycle(healthy_scenario())
    lifecycle.start(now_ns=0)
    lifecycle.mark_ready(now_ns=0)
    lifecycle.tick(now_ns=1 * SEC)
    assert lifecycle.state is LifecycleState.OBSERVING

    action = lifecycle.fail("service call errored", now_ns=1 * SEC + 10)
    assert action == Action(ActionType.FINISH_TRIAL, reason="service call errored")
    assert lifecycle.state is LifecycleState.FINALIZING


def test_fail_rejected_after_finalizing() -> None:
    lifecycle = TrialLifecycle(healthy_scenario())
    lifecycle.start(now_ns=0)
    lifecycle.fail("boom", now_ns=1)
    with pytest.raises(LifecycleError):
        lifecycle.fail("again", now_ns=2)


@pytest.mark.parametrize(
    "method,args",
    [
        ("mark_ready", {"now_ns": 0}),
        ("finish", {}),
    ],
)
def test_methods_reject_wrong_state(method: str, args: dict[str, int]) -> None:
    lifecycle = TrialLifecycle(healthy_scenario())
    with pytest.raises(LifecycleError):
        getattr(lifecycle, method)(**args)
