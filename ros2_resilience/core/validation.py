"""Load and strictly validate resilience-contract YAML into a ScenarioConfig."""

from __future__ import annotations

import math
from pathlib import Path
from typing import Any

import yaml

from ros2_resilience.core.scenario import (
    SUPPORTED_MESSAGE_TYPE,
    SUPPORTED_RECOVERY_ACTION,
    SUPPORTED_SCHEMA_VERSION,
    Expectations,
    ExperimentTiming,
    FaultExpectation,
    FaultKind,
    FaultSpec,
    RecoveryExpectation,
    RobotConfig,
    RobotExpectation,
    ScenarioConfig,
    SystemConfig,
    WatchdogConfig,
    WatchdogExpectation,
)

__all__ = ["ScenarioValidationError", "load_scenario", "parse_scenario"]

_FAULT_PARAMETERS: dict[FaultKind, frozenset[str]] = {
    FaultKind.NONE: frozenset(),
    FaultKind.DROPOUT: frozenset({"drop_probability"}),
    FaultKind.LATENCY: frozenset({"delay_sec", "jitter_sec"}),
    FaultKind.STALE: frozenset(),
}
_REQUIRED_FAULT_PARAMETERS: dict[FaultKind, frozenset[str]] = {
    FaultKind.NONE: frozenset(),
    FaultKind.DROPOUT: frozenset({"drop_probability"}),
    FaultKind.LATENCY: frozenset({"delay_sec"}),
    FaultKind.STALE: frozenset(),
}


class ScenarioValidationError(ValueError):
    """Raised when scenario YAML fails schema or consistency validation."""


def load_scenario(path: str | Path) -> ScenarioConfig:
    """Read a YAML file at ``path`` and return a validated ScenarioConfig."""
    text = Path(path).read_text(encoding="utf-8")
    try:
        data = yaml.safe_load(text)
    except yaml.YAMLError as exc:
        raise ScenarioValidationError(f"invalid YAML in {path}: {exc}") from exc
    return parse_scenario(data)


def parse_scenario(data: Any) -> ScenarioConfig:
    """Validate a decoded YAML mapping and return a ScenarioConfig."""
    root = _require_mapping(data, "scenario")
    _reject_unknown_keys(
        root,
        "scenario",
        {"schema_version", "name", "system", "experiment", "fault", "watchdog", "expect"},
    )

    schema_version = _require_int(root, "schema_version", "scenario")
    if schema_version != SUPPORTED_SCHEMA_VERSION:
        raise ScenarioValidationError(
            f"unsupported schema_version {schema_version}; expected {SUPPORTED_SCHEMA_VERSION}"
        )
    name = _require_nonempty_str(root, "name", "scenario")

    system = _parse_system(_require_mapping(root.get("system"), "system"))
    experiment = _parse_experiment(_require_mapping(root.get("experiment"), "experiment"))
    fault = _parse_fault(_require_mapping(root.get("fault"), "fault"))
    watchdog = _parse_watchdog(_require_mapping(root.get("watchdog"), "watchdog"))
    expect = _parse_expect(_require_mapping(root.get("expect"), "expect"), fault.type)

    scenario = ScenarioConfig(
        schema_version=schema_version,
        name=name,
        system=system,
        experiment=experiment,
        fault=fault,
        watchdog=watchdog,
        expect=expect,
    )
    _validate_cross_field_consistency(scenario)
    return scenario


def _parse_system(section: dict[str, Any]) -> SystemConfig:
    _reject_unknown_keys(
        section,
        "system",
        {"input_topic", "output_topic", "message_type", "command_topic", "robot"},
    )
    message_type = _require_nonempty_str(section, "message_type", "system")
    if message_type != SUPPORTED_MESSAGE_TYPE:
        raise ScenarioValidationError(
            f"system.message_type must be {SUPPORTED_MESSAGE_TYPE!r}, got {message_type!r}"
        )

    robot_section = _require_mapping(section.get("robot"), "system.robot")
    _reject_unknown_keys(
        robot_section,
        "system.robot",
        {"publish_rate_hz", "initial_linear_velocity_mps", "initial_angular_velocity_rad_s"},
    )
    robot = RobotConfig(
        publish_rate_hz=_require_positive_float(robot_section, "publish_rate_hz", "system.robot"),
        initial_linear_velocity_mps=_require_float(
            robot_section, "initial_linear_velocity_mps", "system.robot"
        ),
        initial_angular_velocity_rad_s=_require_float(
            robot_section, "initial_angular_velocity_rad_s", "system.robot"
        ),
    )

    return SystemConfig(
        input_topic=_require_nonempty_str(section, "input_topic", "system"),
        output_topic=_require_nonempty_str(section, "output_topic", "system"),
        message_type=message_type,
        command_topic=_require_nonempty_str(section, "command_topic", "system"),
        robot=robot,
    )


def _parse_experiment(section: dict[str, Any]) -> ExperimentTiming:
    _reject_unknown_keys(
        section,
        "experiment",
        {
            "seed",
            "readiness_timeout_sec",
            "healthy_baseline_sec",
            "observation_after_activation_sec",
        },
    )
    return ExperimentTiming(
        seed=_require_nonnegative_int(section, "seed", "experiment"),
        readiness_timeout_sec=_require_positive_float(
            section, "readiness_timeout_sec", "experiment"
        ),
        healthy_baseline_sec=_require_positive_float(section, "healthy_baseline_sec", "experiment"),
        observation_after_activation_sec=_require_positive_float(
            section, "observation_after_activation_sec", "experiment"
        ),
    )


def _parse_fault(section: dict[str, Any]) -> FaultSpec:
    _reject_unknown_keys(section, "fault", {"type", "parameters", "duration_sec"})
    type_str = _require_nonempty_str(section, "type", "fault")
    try:
        fault_type = FaultKind(type_str)
    except ValueError as exc:
        allowed = ", ".join(kind.value for kind in FaultKind)
        raise ScenarioValidationError(
            f"fault.type must be one of [{allowed}], got {type_str!r}"
        ) from exc

    parameters_raw = section.get("parameters", {})
    parameters_section = _require_mapping(parameters_raw, "fault.parameters")
    allowed_keys = _FAULT_PARAMETERS[fault_type]
    unknown = set(parameters_section) - allowed_keys
    if unknown:
        raise ScenarioValidationError(
            f"fault.parameters has unsupported keys for type {fault_type.value!r}: "
            f"{sorted(unknown)}"
        )
    missing = _REQUIRED_FAULT_PARAMETERS[fault_type] - set(parameters_section)
    if missing:
        raise ScenarioValidationError(
            f"fault.parameters is missing required keys for type {fault_type.value!r}: "
            f"{sorted(missing)}"
        )

    parameters: dict[str, float] = {}
    if "drop_probability" in parameters_section:
        parameters["drop_probability"] = _require_probability(
            parameters_section, "drop_probability", "fault.parameters"
        )
    if "delay_sec" in parameters_section:
        parameters["delay_sec"] = _require_nonnegative_float(
            parameters_section, "delay_sec", "fault.parameters"
        )
    if "jitter_sec" in parameters_section:
        parameters["jitter_sec"] = _require_nonnegative_float(
            parameters_section, "jitter_sec", "fault.parameters"
        )

    duration_sec = (
        0.0
        if fault_type is FaultKind.NONE
        else _require_positive_float(section, "duration_sec", "fault")
    )
    if fault_type is FaultKind.NONE and "duration_sec" in section and section["duration_sec"] != 0:
        raise ScenarioValidationError("fault.duration_sec must be 0 when fault.type is 'none'")

    return FaultSpec(type=fault_type, parameters=parameters, duration_sec=duration_sec)


def _parse_watchdog(section: dict[str, Any]) -> WatchdogConfig:
    _reject_unknown_keys(
        section,
        "watchdog",
        {
            "max_age_sec",
            "expected_rate_hz",
            "loss_window_sec",
            "max_loss_fraction",
            "check_period_sec",
            "recovery",
        },
    )
    recovery = _require_nonempty_str(section, "recovery", "watchdog")
    if recovery != SUPPORTED_RECOVERY_ACTION:
        raise ScenarioValidationError(
            f"watchdog.recovery must be {SUPPORTED_RECOVERY_ACTION!r}, got {recovery!r}"
        )
    return WatchdogConfig(
        max_age_sec=_require_positive_float(section, "max_age_sec", "watchdog"),
        expected_rate_hz=_require_positive_float(section, "expected_rate_hz", "watchdog"),
        loss_window_sec=_require_positive_float(section, "loss_window_sec", "watchdog"),
        max_loss_fraction=_require_probability(section, "max_loss_fraction", "watchdog"),
        check_period_sec=_require_positive_float(section, "check_period_sec", "watchdog"),
        recovery=recovery,
    )


def _parse_expect(section: dict[str, Any], fault_type: FaultKind) -> Expectations:
    _reject_unknown_keys(
        section, "expect", {"no_violation_before_fault", "fault", "watchdog", "recovery", "robot"}
    )
    no_violation_before_fault = _require_bool(section, "no_violation_before_fault", "expect")

    if fault_type is FaultKind.NONE:
        for key in ("fault", "watchdog", "recovery"):
            if key in section and section[key] is not None:
                raise ScenarioValidationError(
                    f"expect.{key} must be absent for a healthy-control scenario (fault.type: none)"
                )
        robot = None
        if "robot" in section and section["robot"] is not None:
            robot = _parse_robot_expectation(_require_mapping(section["robot"], "expect.robot"))
        return Expectations(
            no_violation_before_fault=no_violation_before_fault,
            fault=None,
            watchdog=None,
            recovery=None,
            robot=robot,
        )

    fault_section = _require_mapping(section.get("fault"), "expect.fault")
    _reject_unknown_keys(fault_section, "expect.fault", {"observable", "within_sec"})
    fault_expectation = FaultExpectation(
        observable=_require_bool(fault_section, "observable", "expect.fault"),
        within_sec=_require_positive_float(fault_section, "within_sec", "expect.fault"),
    )

    watchdog_section = _require_mapping(section.get("watchdog"), "expect.watchdog")
    _reject_unknown_keys(watchdog_section, "expect.watchdog", {"violation", "within_sec"})
    watchdog_expectation = WatchdogExpectation(
        violation=_require_nonempty_str(watchdog_section, "violation", "expect.watchdog"),
        within_sec=_require_positive_float(watchdog_section, "within_sec", "expect.watchdog"),
    )

    recovery_section = _require_mapping(section.get("recovery"), "expect.recovery")
    _reject_unknown_keys(
        recovery_section, "expect.recovery", {"action", "command_within_sec", "observed_within_sec"}
    )
    recovery_action = _require_nonempty_str(recovery_section, "action", "expect.recovery")
    if recovery_action != SUPPORTED_RECOVERY_ACTION:
        raise ScenarioValidationError(
            f"expect.recovery.action must be {SUPPORTED_RECOVERY_ACTION!r}, got {recovery_action!r}"
        )
    recovery_expectation = RecoveryExpectation(
        action=recovery_action,
        command_within_sec=_require_positive_float(
            recovery_section, "command_within_sec", "expect.recovery"
        ),
        observed_within_sec=_require_positive_float(
            recovery_section, "observed_within_sec", "expect.recovery"
        ),
    )

    robot_section = _require_mapping(section.get("robot"), "expect.robot")
    robot_expectation = _parse_robot_expectation(robot_section)

    return Expectations(
        no_violation_before_fault=no_violation_before_fault,
        fault=fault_expectation,
        watchdog=watchdog_expectation,
        recovery=recovery_expectation,
        robot=robot_expectation,
    )


def _parse_robot_expectation(section: dict[str, Any]) -> RobotExpectation:
    _reject_unknown_keys(
        section,
        "expect.robot",
        {"linear_velocity_below_mps", "angular_velocity_below_rad_s", "within_sec", "hold_sec"},
    )
    return RobotExpectation(
        linear_velocity_below_mps=_require_positive_float(
            section, "linear_velocity_below_mps", "expect.robot"
        ),
        angular_velocity_below_rad_s=_require_positive_float(
            section, "angular_velocity_below_rad_s", "expect.robot"
        ),
        within_sec=_require_positive_float(section, "within_sec", "expect.robot"),
        hold_sec=_require_positive_float(section, "hold_sec", "expect.robot"),
    )


def _validate_cross_field_consistency(scenario: ScenarioConfig) -> None:
    observation = scenario.experiment.observation_after_activation_sec
    deadlines: list[tuple[str, float]] = [("fault.duration_sec", scenario.fault.duration_sec)]

    if scenario.expect.fault is not None:
        deadlines.append(("expect.fault.within_sec", scenario.expect.fault.within_sec))
    if scenario.expect.watchdog is not None:
        deadlines.append(("expect.watchdog.within_sec", scenario.expect.watchdog.within_sec))
    if scenario.expect.recovery is not None:
        deadlines.append(
            ("expect.recovery.observed_within_sec", scenario.expect.recovery.observed_within_sec)
        )
    if scenario.expect.robot is not None:
        deadlines.append(
            (
                "expect.robot.within_sec + expect.robot.hold_sec",
                scenario.expect.robot.within_sec + scenario.expect.robot.hold_sec,
            )
        )

    for label, deadline in deadlines:
        if deadline > observation:
            raise ScenarioValidationError(
                f"{label} ({deadline}s) exceeds experiment.observation_after_activation_sec "
                f"({observation}s)"
            )

    expected_period = 1.0 / scenario.watchdog.expected_rate_hz
    if scenario.watchdog.max_age_sec < expected_period:
        raise ScenarioValidationError(
            "watchdog.max_age_sec must be at least one expected publication period "
            f"(1/expected_rate_hz = {expected_period}s)"
        )


def _require_mapping(value: Any, field: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise ScenarioValidationError(f"{field} must be a mapping")
    return value


def _reject_unknown_keys(section: dict[str, Any], field: str, allowed: set[str]) -> None:
    unknown = set(section) - allowed
    if unknown:
        raise ScenarioValidationError(f"{field} has unsupported keys: {sorted(unknown)}")


def _require_key(section: dict[str, Any], key: str, field: str) -> Any:
    if key not in section:
        raise ScenarioValidationError(f"{field}.{key} is required")
    return section[key]


def _require_nonempty_str(section: dict[str, Any], key: str, field: str) -> str:
    value = _require_key(section, key, field)
    if not isinstance(value, str) or not value.strip():
        raise ScenarioValidationError(f"{field}.{key} must be a nonempty string")
    return value


def _require_bool(section: dict[str, Any], key: str, field: str) -> bool:
    value = _require_key(section, key, field)
    if not isinstance(value, bool):
        raise ScenarioValidationError(f"{field}.{key} must be a bool")
    return value


def _require_int(section: dict[str, Any], key: str, field: str) -> int:
    value = _require_key(section, key, field)
    if isinstance(value, bool) or not isinstance(value, int):
        raise ScenarioValidationError(f"{field}.{key} must be an integer")
    return value


def _require_nonnegative_int(section: dict[str, Any], key: str, field: str) -> int:
    value = _require_int(section, key, field)
    if value < 0:
        raise ScenarioValidationError(f"{field}.{key} must be nonnegative")
    return value


def _require_float(section: dict[str, Any], key: str, field: str) -> float:
    value = _require_key(section, key, field)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ScenarioValidationError(f"{field}.{key} must be a number")
    value = float(value)
    if not math.isfinite(value):
        raise ScenarioValidationError(f"{field}.{key} must be finite")
    return value


def _require_positive_float(section: dict[str, Any], key: str, field: str) -> float:
    value = _require_float(section, key, field)
    if value <= 0.0:
        raise ScenarioValidationError(f"{field}.{key} must be positive")
    return value


def _require_nonnegative_float(section: dict[str, Any], key: str, field: str) -> float:
    value = _require_float(section, key, field)
    if value < 0.0:
        raise ScenarioValidationError(f"{field}.{key} must be nonnegative")
    return value


def _require_probability(section: dict[str, Any], key: str, field: str) -> float:
    value = _require_float(section, key, field)
    if not 0.0 <= value <= 1.0:
        raise ScenarioValidationError(f"{field}.{key} must be within [0.0, 1.0]")
    return value
