from __future__ import annotations

import copy

import pytest
from conftest import healthy_control_dict, scenario_dict

from ros2_resilience.core.scenario import FaultKind
from ros2_resilience.core.validation import ScenarioValidationError, parse_scenario


def test_valid_dropout_scenario_parses() -> None:
    scenario = parse_scenario(scenario_dict())
    assert scenario.fault.type is FaultKind.DROPOUT
    assert scenario.fault.parameters == {"drop_probability": 0.8}
    assert scenario.expect.fault is not None
    assert scenario.expect.watchdog is not None
    assert scenario.expect.recovery is not None
    assert scenario.expect.robot is not None


def test_valid_healthy_control_parses() -> None:
    scenario = parse_scenario(healthy_control_dict())
    assert scenario.is_healthy_control
    assert scenario.expect.fault is None
    assert scenario.expect.watchdog is None
    assert scenario.expect.recovery is None


@pytest.mark.parametrize(
    "mutate",
    [
        lambda d: d.update(schema_version=2),
        lambda d: d.update(unexpected_top_level_key=1),
        lambda d: d["system"].update(message_type="geometry_msgs/msg/Twist"),
        lambda d: d["system"].pop("input_topic"),
        lambda d: d["system"]["robot"].update(publish_rate_hz=0),
        lambda d: d["experiment"].update(seed=-1),
        lambda d: d["fault"].update(type="not_a_fault"),
        lambda d: d["fault"]["parameters"].update(unexpected="x"),
        lambda d: d["fault"]["parameters"].pop("drop_probability"),
        lambda d: d["watchdog"].update(max_loss_fraction=1.5),
        lambda d: d["watchdog"].update(recovery="reverse"),
        lambda d: d["expect"]["recovery"].update(action="reverse"),
    ],
)
def test_invalid_variants_are_rejected(mutate) -> None:
    data = copy.deepcopy(scenario_dict())
    mutate(data)
    with pytest.raises(ScenarioValidationError):
        parse_scenario(data)


def test_healthy_control_rejects_fault_expectation_present() -> None:
    data = healthy_control_dict()
    data["expect"]["fault"] = {"observable": True, "within_sec": 1.0}
    with pytest.raises(ScenarioValidationError, match="must be absent"):
        parse_scenario(data)


def test_non_healthy_control_requires_fault_expectation() -> None:
    data = scenario_dict()
    data["expect"]["fault"] = None
    with pytest.raises(ScenarioValidationError):
        parse_scenario(data)


def test_deadline_exceeding_observation_window_is_rejected() -> None:
    data = scenario_dict()
    data["expect"]["robot"]["hold_sec"] = 100.0
    with pytest.raises(ScenarioValidationError, match="observation_after_activation_sec"):
        parse_scenario(data)


def test_max_age_below_one_publication_period_is_rejected() -> None:
    data = scenario_dict()
    data["watchdog"]["max_age_sec"] = 0.001
    with pytest.raises(ScenarioValidationError, match="expected publication period"):
        parse_scenario(data)


def test_latency_scenario_requires_delay_sec() -> None:
    data = scenario_dict(
        fault={"type": "latency", "parameters": {"jitter_sec": 0.1}, "duration_sec": 3.0}
    )
    with pytest.raises(ScenarioValidationError, match="missing required keys"):
        parse_scenario(data)


def test_stale_scenario_rejects_parameters() -> None:
    data = scenario_dict(
        fault={"type": "stale", "parameters": {"drop_probability": 0.1}, "duration_sec": 3.0}
    )
    with pytest.raises(ScenarioValidationError, match="unsupported keys"):
        parse_scenario(data)


def test_top_level_must_be_a_mapping() -> None:
    with pytest.raises(ScenarioValidationError, match="mapping"):
        parse_scenario(["not", "a", "mapping"])
