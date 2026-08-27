"""Evaluate a scenario's behavioral requirements against collected evidence."""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass
from enum import StrEnum

from ros2_resilience.core import metrics
from ros2_resilience.core.events import Event, EventKind
from ros2_resilience.core.scenario import FaultKind, ScenarioConfig

__all__ = ["AssertionResult", "AssertionType", "Timeline", "evaluate_scenario"]


class AssertionType(StrEnum):
    """The fixed vocabulary of behavioral assertions this evaluator supports."""

    NO_EARLY_VIOLATION = "no_early_violation"
    FAULT_OBSERVABLE = "fault_observable"
    WATCHDOG_VIOLATION_DETECTED = "watchdog_violation_detected"
    RECOVERY_COMMAND_PUBLISHED = "recovery_command_published"
    RECOVERY_COMMAND_OBSERVED = "recovery_command_observed"
    ROBOT_STOPPED = "robot_stopped"
    ROBOT_STOPPED_HELD = "robot_stopped_held"
    NO_VIOLATION_DURING_OBSERVATION = "no_violation_during_observation"
    NO_RECOVERY_COMMAND = "no_recovery_command"


@dataclass(frozen=True)
class AssertionResult:
    """The outcome of evaluating one assertion against collected evidence."""

    type: AssertionType
    passed: bool
    reason: str
    observed_value: float | None = None
    threshold: float | None = None


@dataclass(frozen=True)
class Timeline:
    """The receipt-time anchors a trial's lifecycle produced.

    ``activation_ns`` is the confirmed fault-activation time for fault
    scenarios, or the end of the baseline for healthy controls.
    """

    baseline_start_ns: int
    activation_ns: int
    deactivation_ns: int | None
    observation_deadline_ns: int


def evaluate_scenario(
    scenario: ScenarioConfig,
    events: Sequence[Event],
    timeline: Timeline,
) -> list[AssertionResult]:
    """Evaluate every required assertion for ``scenario`` and return results."""
    if scenario.is_healthy_control:
        return _evaluate_healthy_control(scenario, events, timeline)
    return _evaluate_fault_scenario(scenario, events, timeline)


def _evaluate_healthy_control(
    scenario: ScenarioConfig,
    events: Sequence[Event],
    timeline: Timeline,
) -> list[AssertionResult]:
    results = [_no_violation_during_observation(events, timeline)]
    results.append(_no_recovery_command(events, timeline))
    if scenario.expect.robot is not None:
        results.append(
            _robot_stopped(scenario, events, timeline, activation_ns=timeline.activation_ns)
        )
    return results


def _evaluate_fault_scenario(
    scenario: ScenarioConfig,
    events: Sequence[Event],
    timeline: Timeline,
) -> list[AssertionResult]:
    assert scenario.expect.fault is not None
    assert scenario.expect.watchdog is not None
    assert scenario.expect.recovery is not None

    results = [_no_early_violation(events, timeline)]
    results.append(_fault_observable(scenario, events, timeline))

    watchdog_result = _watchdog_violation_detected(scenario, events, timeline)
    results.append(watchdog_result)

    detection_ns = _first_violation_receipt_ns(events, timeline.activation_ns)
    results.append(_recovery_command_published(scenario, events, timeline, detection_ns))
    results.append(_recovery_command_observed(scenario, events, timeline, detection_ns))

    if scenario.expect.robot is not None:
        results.append(
            _robot_stopped(scenario, events, timeline, activation_ns=timeline.activation_ns)
        )
        results.append(_robot_stopped_held(scenario, events, timeline))

    return results


def _first_violation_receipt_ns(events: Sequence[Event], after_ns: int) -> int | None:
    violations = [
        e for e in events if e.kind is EventKind.WATCHDOG_VIOLATION and e.receipt_ns >= after_ns
    ]
    if not violations:
        return None
    return min(e.receipt_ns for e in violations)


def _no_early_violation(events: Sequence[Event], timeline: Timeline) -> AssertionResult:
    early = [
        e
        for e in events
        if e.kind is EventKind.WATCHDOG_VIOLATION and e.receipt_ns < timeline.activation_ns
    ]
    if early:
        return AssertionResult(
            type=AssertionType.NO_EARLY_VIOLATION,
            passed=False,
            reason=f"watchdog violation observed before activation: {early[0].payload}",
        )
    return AssertionResult(
        type=AssertionType.NO_EARLY_VIOLATION, passed=True, reason="no violation before activation"
    )


def _no_violation_during_observation(
    events: Sequence[Event], timeline: Timeline
) -> AssertionResult:
    violations = [
        e
        for e in events
        if e.kind is EventKind.WATCHDOG_VIOLATION
        and timeline.baseline_start_ns <= e.receipt_ns <= timeline.observation_deadline_ns
    ]
    if violations:
        return AssertionResult(
            type=AssertionType.NO_VIOLATION_DURING_OBSERVATION,
            passed=False,
            reason=f"unexpected watchdog violation in healthy control: {violations[0].payload}",
        )
    return AssertionResult(
        type=AssertionType.NO_VIOLATION_DURING_OBSERVATION,
        passed=True,
        reason="no watchdog violation observed during the trial",
    )


def _no_recovery_command(events: Sequence[Event], timeline: Timeline) -> AssertionResult:
    published = [
        e
        for e in events
        if e.kind is EventKind.RECOVERY_PUBLISHED
        and timeline.baseline_start_ns <= e.receipt_ns <= timeline.observation_deadline_ns
    ]
    if published:
        return AssertionResult(
            type=AssertionType.NO_RECOVERY_COMMAND,
            passed=False,
            reason="unexpected recovery command published in healthy control",
        )
    return AssertionResult(
        type=AssertionType.NO_RECOVERY_COMMAND, passed=True, reason="no recovery command published"
    )


def _fault_observable(
    scenario: ScenarioConfig, events: Sequence[Event], timeline: Timeline
) -> AssertionResult:
    assert scenario.expect.fault is not None
    deadline_ns = timeline.activation_ns + _sec_to_ns(scenario.expect.fault.within_sec)
    window = [e for e in events if timeline.activation_ns <= e.receipt_ns <= deadline_ns]

    if scenario.fault.type is FaultKind.DROPOUT:
        passed, reason = _dropout_observable(window)
    elif scenario.fault.type is FaultKind.LATENCY:
        passed, reason = _latency_observable(events, window, scenario)
    elif scenario.fault.type is FaultKind.STALE:
        passed, reason = _stale_observable(window)
    else:  # pragma: no cover - unreachable for fault scenarios
        passed, reason = False, "unsupported fault type"

    return AssertionResult(type=AssertionType.FAULT_OBSERVABLE, passed=passed, reason=reason)


def _dropout_observable(window: list[Event]) -> tuple[bool, str]:
    raw = [e for e in window if e.kind is EventKind.RAW_ODOMETRY]
    faulted = [e for e in window if e.kind is EventKind.FAULTED_ODOMETRY]
    counters = [e for e in window if e.kind is EventKind.INJECTOR_COUNTERS]
    if not raw:
        return False, "no raw odometry observed after activation"
    if not counters or counters[-1].payload.get("dropped", 0) <= 0:
        return False, "injector drop counters did not advance"
    if len(faulted) >= len(raw):
        return False, "no deficit observed on the faulted stream"
    return True, (
        f"raw traffic advanced ({len(raw)} samples), injector dropped "
        f"{counters[-1].payload.get('dropped')}, faulted stream deficit observed"
    )


def _latency_observable(
    all_events: Sequence[Event], window: list[Event], scenario: ScenarioConfig
) -> tuple[bool, str]:
    delay_sec = scenario.fault.parameters.get("delay_sec", 0.0)
    tolerance_sec = max(scenario.watchdog.check_period_sec * 5, 0.01)
    threshold_ns = max(int((delay_sec - tolerance_sec) * 1e9), 0)
    raw_by_stamp = {
        e.payload.get("stamp_ns"): e for e in all_events if e.kind is EventKind.RAW_ODOMETRY
    }
    for faulted in window:
        if faulted.kind is not EventKind.FAULTED_ODOMETRY:
            continue
        raw = raw_by_stamp.get(faulted.payload.get("stamp_ns"))
        if raw is None:
            continue
        gap_ns = faulted.receipt_ns - raw.receipt_ns
        if gap_ns >= threshold_ns:
            return (
                True,
                f"matched sample delayed by {gap_ns / 1e9:.3f}s (>= {threshold_ns / 1e9:.3f}s)",
            )
    return False, "no matched raw/output pair showed the configured delay"


def _stale_observable(window: list[Event]) -> tuple[bool, str]:
    faulted = sorted(
        (e for e in window if e.kind is EventKind.FAULTED_ODOMETRY), key=lambda e: e.receipt_ns
    )
    raw = [e for e in window if e.kind is EventKind.RAW_ODOMETRY]
    if len(faulted) < 2:
        return False, "insufficient faulted-stream samples"
    faulted_stamps = {e.payload.get("stamp_ns") for e in faulted}
    if len(faulted_stamps) > 1:
        return False, "faulted-stream timestamp did not remain held"
    raw_stamps = {e.payload.get("stamp_ns") for e in raw}
    if len(raw_stamps) < 2:
        return False, "raw stream did not advance while faulted stream was held"
    return True, "faulted-stream timestamp held while raw stream continued to advance"


def _watchdog_violation_detected(
    scenario: ScenarioConfig, events: Sequence[Event], timeline: Timeline
) -> AssertionResult:
    assert scenario.expect.watchdog is not None
    deadline_ns = timeline.activation_ns + _sec_to_ns(scenario.expect.watchdog.within_sec)
    matches = [
        e
        for e in events
        if e.kind is EventKind.WATCHDOG_VIOLATION
        and timeline.activation_ns <= e.receipt_ns <= deadline_ns
        and e.payload.get("reason") == scenario.expect.watchdog.violation
    ]
    if not matches:
        return AssertionResult(
            type=AssertionType.WATCHDOG_VIOLATION_DETECTED,
            passed=False,
            reason=(
                f"no watchdog violation with reason={scenario.expect.watchdog.violation!r} "
                f"within {scenario.expect.watchdog.within_sec}s of activation"
            ),
        )
    observed_sec = (min(e.receipt_ns for e in matches) - timeline.activation_ns) / 1e9
    return AssertionResult(
        type=AssertionType.WATCHDOG_VIOLATION_DETECTED,
        passed=True,
        reason="watchdog detected the expected violation",
        observed_value=observed_sec,
        threshold=scenario.expect.watchdog.within_sec,
    )


def _recovery_command_published(
    scenario: ScenarioConfig, events: Sequence[Event], timeline: Timeline, detection_ns: int | None
) -> AssertionResult:
    assert scenario.expect.recovery is not None
    if detection_ns is None:
        return AssertionResult(
            type=AssertionType.RECOVERY_COMMAND_PUBLISHED,
            passed=False,
            reason="missing evidence: no detection time available",
        )
    latency = metrics.recovery_command_latency_sec(events, detection_ns)
    if latency is None:
        return AssertionResult(
            type=AssertionType.RECOVERY_COMMAND_PUBLISHED,
            passed=False,
            reason="no recovery command was published after detection",
        )
    passed = latency <= scenario.expect.recovery.command_within_sec
    return AssertionResult(
        type=AssertionType.RECOVERY_COMMAND_PUBLISHED,
        passed=passed,
        reason="recovery command published within deadline"
        if passed
        else "recovery command published too late",
        observed_value=latency,
        threshold=scenario.expect.recovery.command_within_sec,
    )


def _recovery_command_observed(
    scenario: ScenarioConfig, events: Sequence[Event], timeline: Timeline, detection_ns: int | None
) -> AssertionResult:
    assert scenario.expect.recovery is not None
    assert scenario.expect.robot is not None
    if detection_ns is None:
        return AssertionResult(
            type=AssertionType.RECOVERY_COMMAND_OBSERVED,
            passed=False,
            reason="missing evidence: no detection time available",
        )
    latency = metrics.command_observation_latency_sec(
        events,
        detection_ns,
        linear_threshold_mps=scenario.expect.robot.linear_velocity_below_mps,
        angular_threshold_rad_s=scenario.expect.robot.angular_velocity_below_rad_s,
    )
    if latency is None:
        return AssertionResult(
            type=AssertionType.RECOVERY_COMMAND_OBSERVED,
            passed=False,
            reason="no qualifying zero command was observed after detection",
        )
    passed = latency <= scenario.expect.recovery.observed_within_sec
    return AssertionResult(
        type=AssertionType.RECOVERY_COMMAND_OBSERVED,
        passed=passed,
        reason="qualifying command observed within deadline"
        if passed
        else "command observed too late",
        observed_value=latency,
        threshold=scenario.expect.recovery.observed_within_sec,
    )


def _robot_stopped(
    scenario: ScenarioConfig, events: Sequence[Event], timeline: Timeline, *, activation_ns: int
) -> AssertionResult:
    assert scenario.expect.robot is not None
    result = metrics.time_to_stop_sec(
        events,
        activation_ns,
        linear_threshold_mps=scenario.expect.robot.linear_velocity_below_mps,
        angular_threshold_rad_s=scenario.expect.robot.angular_velocity_below_rad_s,
    )
    if result is None:
        return AssertionResult(
            type=AssertionType.ROBOT_STOPPED,
            passed=False,
            reason="robot never reported a qualifying stopped state",
        )
    observed_sec, _ = result
    passed = observed_sec <= scenario.expect.robot.within_sec
    return AssertionResult(
        type=AssertionType.ROBOT_STOPPED,
        passed=passed,
        reason="robot stopped within deadline" if passed else "robot stopped too late",
        observed_value=observed_sec,
        threshold=scenario.expect.robot.within_sec,
    )


def _robot_stopped_held(
    scenario: ScenarioConfig, events: Sequence[Event], timeline: Timeline
) -> AssertionResult:
    assert scenario.expect.robot is not None
    result = metrics.time_to_stop_sec(
        events,
        timeline.activation_ns,
        linear_threshold_mps=scenario.expect.robot.linear_velocity_below_mps,
        angular_threshold_rad_s=scenario.expect.robot.angular_velocity_below_rad_s,
    )
    if result is None:
        return AssertionResult(
            type=AssertionType.ROBOT_STOPPED_HELD,
            passed=False,
            reason="missing evidence: robot never reported a qualifying stopped state",
        )
    _, stop_receipt_ns = result
    hold_deadline_ns = stop_receipt_ns + _sec_to_ns(scenario.expect.robot.hold_sec)
    expected_period_ns = int(round(1e9 / scenario.system.robot.publish_rate_hz))
    broken_at = metrics.stop_hold_broken_at_sec(
        events,
        stop_receipt_ns,
        hold_deadline_ns,
        linear_threshold_mps=scenario.expect.robot.linear_velocity_below_mps,
        angular_threshold_rad_s=scenario.expect.robot.angular_velocity_below_rad_s,
        expected_period_ns=expected_period_ns,
    )
    if broken_at is not None:
        return AssertionResult(
            type=AssertionType.ROBOT_STOPPED_HELD,
            passed=False,
            reason=f"stopped state broke {broken_at:.3f}s into the hold interval",
            observed_value=broken_at,
            threshold=scenario.expect.robot.hold_sec,
        )
    return AssertionResult(
        type=AssertionType.ROBOT_STOPPED_HELD,
        passed=True,
        reason="stopped state held for the required interval",
        threshold=scenario.expect.robot.hold_sec,
    )


def _sec_to_ns(seconds: float) -> int:
    return int(round(seconds * 1e9))
