"""Orchestrate one resilience trial (or a repeated batch) over real ROS 2 nodes.

Every trial gets a fresh :class:`rclpy.Context`, fresh robot/injector/watchdog
nodes, and topics namespaced by trial id, so trials do not share ROS graph
state even though they run sequentially in one process. This is a documented
Phase-1 simplification of the design's "fresh OS process per trial" goal:
process-level isolation was judged out of scope for the time available, but
graph-level isolation (fresh context, fresh nodes, fresh topics, fresh pure
watchdog/injector/robot state) is real and is what this module provides.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import sys
import time
from dataclasses import asdict
from pathlib import Path

import rclpy
from diagnostic_msgs.msg import DiagnosticStatus
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSPresetProfiles
from std_msgs.msg import Int64

from ros2_resilience.core import metrics
from ros2_resilience.core.assertions import Timeline, evaluate_scenario
from ros2_resilience.core.batch import derive_seed, generate_batch_id
from ros2_resilience.core.batch import trial_id as make_trial_id
from ros2_resilience.core.events import ClockDomain, Event, EventKind
from ros2_resilience.core.experiment import ActionType, LifecycleState, TrialLifecycle
from ros2_resilience.core.results import BatchResult, TrialResult
from ros2_resilience.core.scenario import FaultKind, ScenarioConfig
from ros2_resilience.core.validation import load_scenario
from ros2_resilience.faults.injector import FaultConfig, FaultType
from ros2_resilience.ros.fault_injector_node import FaultInjectorNode
from ros2_resilience.ros.robot_node import RobotNode
from ros2_resilience.ros.watchdog_node import WatchdogNode

__all__ = ["main", "run_batch", "run_trial"]

_READY_POLL_SEC = 0.02
_TICK_POLL_SEC = 0.02
_DRAIN_SEC = 0.2


def _ns_topic(namespace: str, topic: str) -> str:
    return f"{namespace}/{topic.lstrip('/')}"


def _topics(scenario: ScenarioConfig, namespace: str) -> dict[str, str]:
    return {
        "input": _ns_topic(namespace, scenario.system.input_topic),
        "output": _ns_topic(namespace, scenario.system.output_topic),
        "command": _ns_topic(namespace, scenario.system.command_topic),
        "config_applied": _ns_topic(namespace, "/fault_injector/config_applied"),
        "counters": _ns_topic(namespace, "/fault_injector/counters"),
        "watchdog_status": _ns_topic(namespace, "/watchdog/status"),
        "recovery_published": _ns_topic(namespace, "/watchdog/recovery_published"),
    }


def _stamp_ns(stamp) -> int:
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def _safe_float(value: str | None) -> float | None:
    try:
        return float(value) if value is not None else None
    except ValueError:
        return None


def _safe_int(value: str | None) -> int | None:
    try:
        return int(value) if value is not None else None
    except ValueError:
        return None


class _ObserverNode(Node):
    """Subscribe to every topic that carries trial evidence and normalize it."""

    def __init__(
        self, *, topics: dict[str, str], trial_id: str, on_event, on_config_applied, **kwargs
    ) -> None:
        super().__init__("observer", **kwargs)
        self._trial_id = trial_id
        self._on_event = on_event
        self._on_config_applied = on_config_applied
        self._event_seq = 0
        self._seen_raw = False
        self._seen_faulted = False

        qos = QoSPresetProfiles.SENSOR_DATA.value
        self.create_subscription(Odometry, topics["input"], self._on_raw, qos)
        self.create_subscription(Odometry, topics["output"], self._on_faulted, qos)
        self.create_subscription(Twist, topics["command"], self._on_command, 10)
        self.create_subscription(DiagnosticStatus, topics["watchdog_status"], self._on_watchdog, 10)
        self.create_subscription(DiagnosticStatus, topics["counters"], self._on_counters, 10)
        self.create_subscription(
            DiagnosticStatus, topics["recovery_published"], self._on_recovery, 10
        )
        self.create_subscription(Int64, topics["config_applied"], self._on_config_applied_msg, 10)

    def is_ready(self) -> bool:
        """Return whether both the raw and faulted odometry streams are flowing."""
        return self._seen_raw and self._seen_faulted

    def _emit(self, kind: EventKind, payload: dict, occurrence_ns: int | None = None) -> int:
        now = time.monotonic_ns()
        self._event_seq += 1
        self._on_event(
            Event(
                event_id=f"{self._trial_id}-{self._event_seq}",
                trial_id=self._trial_id,
                source="observer",
                kind=kind,
                receipt_ns=now,
                payload=payload,
                occurrence_ns=occurrence_ns,
                occurrence_domain=ClockDomain.ROS if occurrence_ns is not None else None,
            )
        )
        return now

    def _on_raw(self, msg: Odometry) -> None:
        self._seen_raw = True
        stamp_ns = _stamp_ns(msg.header.stamp)
        self._emit(
            EventKind.RAW_ODOMETRY,
            {
                "linear_x": msg.twist.twist.linear.x,
                "angular_z": msg.twist.twist.angular.z,
                "stamp_ns": stamp_ns,
            },
            occurrence_ns=stamp_ns,
        )

    def _on_faulted(self, msg: Odometry) -> None:
        self._seen_faulted = True
        stamp_ns = _stamp_ns(msg.header.stamp)
        self._emit(
            EventKind.FAULTED_ODOMETRY,
            {
                "linear_x": msg.twist.twist.linear.x,
                "angular_z": msg.twist.twist.angular.z,
                "stamp_ns": stamp_ns,
            },
            occurrence_ns=stamp_ns,
        )

    def _on_command(self, msg: Twist) -> None:
        self._emit(
            EventKind.COMMAND_OBSERVED, {"linear_x": msg.linear.x, "angular_z": msg.angular.z}
        )

    def _on_watchdog(self, msg: DiagnosticStatus) -> None:
        values = {kv.key: kv.value for kv in msg.values}
        self._emit(
            EventKind.WATCHDOG_VIOLATION,
            {
                "reason": msg.message,
                "age_sec": _safe_float(values.get("age_sec")),
                "loss_fraction": _safe_float(values.get("loss_fraction")),
            },
        )

    def _on_counters(self, msg: DiagnosticStatus) -> None:
        values = {kv.key: kv.value for kv in msg.values}
        self._emit(
            EventKind.INJECTOR_COUNTERS,
            {
                "received": _safe_int(values.get("received")),
                "forwarded": _safe_int(values.get("forwarded")),
                "dropped": _safe_int(values.get("dropped")),
            },
        )

    def _on_recovery(self, msg: DiagnosticStatus) -> None:
        del msg
        self._emit(EventKind.RECOVERY_PUBLISHED, {})

    def _on_config_applied_msg(self, msg: Int64) -> None:
        now = self._emit(EventKind.CONFIG_APPLIED, {"revision": msg.data})
        self._on_config_applied(msg.data, now)


def _build_fault_config(scenario: ScenarioConfig, seed: int) -> FaultConfig:
    fault = scenario.fault
    if fault.type is FaultKind.NONE:
        return FaultConfig(enabled=False, fault_type=FaultType.NONE, seed=seed)
    if fault.type is FaultKind.DROPOUT:
        return FaultConfig(
            enabled=True,
            fault_type=FaultType.DROPOUT,
            drop_probability=fault.parameters["drop_probability"],
            seed=seed,
        )
    if fault.type is FaultKind.LATENCY:
        return FaultConfig(
            enabled=True,
            fault_type=FaultType.LATENCY,
            delay_ns=int(round(fault.parameters["delay_sec"] * 1e9)),
            jitter_ns=int(round(fault.parameters.get("jitter_sec", 0.0) * 1e9)),
            seed=seed,
        )
    if fault.type is FaultKind.STALE:
        return FaultConfig(enabled=True, fault_type=FaultType.STALE, seed=seed)
    raise ValueError(f"unsupported fault type {fault.type!r}")  # pragma: no cover


def _first_violation_ns(events: list[Event], after_ns: int) -> int | None:
    violations = [
        e.receipt_ns
        for e in events
        if e.kind is EventKind.WATCHDOG_VIOLATION and e.receipt_ns >= after_ns
    ]
    return min(violations) if violations else None


def _empty_metrics() -> dict[str, float | None]:
    return {
        "detection_latency_sec": None,
        "recovery_command_latency_sec": None,
        "command_observation_latency_sec": None,
        "time_to_stop_sec": None,
    }


def _compute_metrics(
    scenario: ScenarioConfig, events: list[Event], timeline: Timeline
) -> dict[str, float | None]:
    result = _empty_metrics()
    if scenario.is_healthy_control:
        return result

    result["detection_latency_sec"] = metrics.detection_latency_sec(events, timeline.activation_ns)
    detection_ns = _first_violation_ns(events, timeline.activation_ns)
    if detection_ns is not None:
        result["recovery_command_latency_sec"] = metrics.recovery_command_latency_sec(
            events, detection_ns
        )
        if scenario.expect.robot is not None:
            result["command_observation_latency_sec"] = metrics.command_observation_latency_sec(
                events,
                detection_ns,
                linear_threshold_mps=scenario.expect.robot.linear_velocity_below_mps,
                angular_threshold_rad_s=scenario.expect.robot.angular_velocity_below_rad_s,
            )
    if scenario.expect.robot is not None:
        stop = metrics.time_to_stop_sec(
            events,
            timeline.activation_ns,
            linear_threshold_mps=scenario.expect.robot.linear_velocity_below_mps,
            angular_threshold_rad_s=scenario.expect.robot.angular_velocity_below_rad_s,
        )
        result["time_to_stop_sec"] = stop[0] if stop is not None else None
    return result


def run_trial(
    scenario: ScenarioConfig, *, trial_id: str, trial_index: int, seed: int
) -> TrialResult:
    """Run one real, ROS-backed trial of ``scenario`` and return its result."""
    namespace = f"/t{trial_index:04d}"
    topics = _topics(scenario, namespace)
    context = rclpy.Context()
    rclpy.init(context=context)

    lifecycle = TrialLifecycle(scenario)
    events: list[Event] = []

    robot_node = RobotNode(
        namespace=namespace,
        context=context,
        parameter_overrides=[
            Parameter("output_topic", value=topics["input"]),
            Parameter("command_topic", value=topics["command"]),
            Parameter("publish_rate_hz", value=scenario.system.robot.publish_rate_hz),
            Parameter(
                "initial_linear_velocity_mps",
                value=scenario.system.robot.initial_linear_velocity_mps,
            ),
            Parameter(
                "initial_angular_velocity_rad_s",
                value=scenario.system.robot.initial_angular_velocity_rad_s,
            ),
        ],
    )
    injector_node = FaultInjectorNode(
        namespace=namespace,
        context=context,
        parameter_overrides=[
            Parameter("input_topic", value=topics["input"]),
            Parameter("output_topic", value=topics["output"]),
            Parameter("config_applied_topic", value=topics["config_applied"]),
            Parameter("counters_topic", value=topics["counters"]),
        ],
    )
    watchdog_node = WatchdogNode(
        namespace=namespace,
        context=context,
        parameter_overrides=[
            Parameter("input_topic", value=topics["output"]),
            Parameter("command_topic", value=topics["command"]),
            Parameter("status_topic", value=topics["watchdog_status"]),
            Parameter("recovery_topic", value=topics["recovery_published"]),
            Parameter("max_age_sec", value=scenario.watchdog.max_age_sec),
            Parameter("expected_rate_hz", value=scenario.watchdog.expected_rate_hz),
            Parameter("loss_window_sec", value=scenario.watchdog.loss_window_sec),
            Parameter("max_loss_fraction", value=scenario.watchdog.max_loss_fraction),
            Parameter("check_period_sec", value=scenario.watchdog.check_period_sec),
        ],
    )
    observer_node = _ObserverNode(
        topics=topics,
        trial_id=trial_id,
        on_event=events.append,
        on_config_applied=lambda revision, now_ns: lifecycle.on_config_applied(
            revision, now_ns=now_ns
        ),
        namespace=namespace,
        context=context,
    )

    nodes = (robot_node, injector_node, watchdog_node, observer_node)
    executor = SingleThreadedExecutor(context=context)
    for node in nodes:
        executor.add_node(node)

    try:
        lifecycle.start(now_ns=time.monotonic_ns())
        readiness_deadline_ns = time.monotonic_ns() + int(
            scenario.experiment.readiness_timeout_sec * 1e9
        )
        while lifecycle.state is LifecycleState.STARTING:
            executor.spin_once(timeout_sec=_READY_POLL_SEC)
            if observer_node.is_ready():
                lifecycle.mark_ready(now_ns=time.monotonic_ns())
                break
            if time.monotonic_ns() >= readiness_deadline_ns:
                lifecycle.fail("readiness timeout", now_ns=time.monotonic_ns())
                break

        while lifecycle.state not in (LifecycleState.FINALIZING, LifecycleState.FINISHED):
            executor.spin_once(timeout_sec=_TICK_POLL_SEC)
            action = lifecycle.tick(now_ns=time.monotonic_ns())
            if action is None:
                continue
            if action.type is ActionType.ACTIVATE_FAULT:
                injector_node.apply_config(
                    _build_fault_config(scenario, seed), revision=lifecycle.activation_revision
                )
            elif action.type is ActionType.DEACTIVATE_FAULT:
                injector_node.apply_config(
                    FaultConfig(enabled=False, fault_type=FaultType.NONE, seed=seed),
                    revision=lifecycle.activation_revision + 1,
                )
            elif action.type is ActionType.FINISH_TRIAL:
                break

        drain_deadline_ns = time.monotonic_ns() + int(_DRAIN_SEC * 1e9)
        while time.monotonic_ns() < drain_deadline_ns:
            executor.spin_once(timeout_sec=_TICK_POLL_SEC)

        if lifecycle.state is LifecycleState.FINALIZING:
            lifecycle.finish()
    finally:
        for node in nodes:
            executor.remove_node(node)
            node.destroy_node()
        executor.shutdown()
        context.try_shutdown()

    activation_ns = lifecycle.activation_ns
    if lifecycle.infrastructure_error is not None or activation_ns is None:
        assertions = []
        passed = False
    else:
        timeline = Timeline(
            baseline_start_ns=lifecycle.baseline_start_ns or 0,
            activation_ns=activation_ns,
            deactivation_ns=lifecycle.deactivation_requested_ns,
            observation_deadline_ns=lifecycle.observation_deadline_ns or 0,
        )
        assertions = evaluate_scenario(scenario, events, timeline)
        passed = all(result.passed for result in assertions)

    metrics_dict = (
        _compute_metrics(
            scenario,
            events,
            Timeline(
                baseline_start_ns=lifecycle.baseline_start_ns or 0,
                activation_ns=activation_ns or 0,
                deactivation_ns=lifecycle.deactivation_requested_ns,
                observation_deadline_ns=lifecycle.observation_deadline_ns or 0,
            ),
        )
        if activation_ns is not None
        else _empty_metrics()
    )

    return TrialResult(
        trial_id=trial_id,
        trial_index=trial_index,
        seed=seed,
        scenario_name=scenario.name,
        lifecycle_state=lifecycle.state.value,
        passed=passed,
        infrastructure_error=lifecycle.infrastructure_error,
        activation_ns=activation_ns,
        deactivation_ns=lifecycle.deactivation_requested_ns,
        assertions=assertions,
        metrics=metrics_dict,
        events=events,
    )


def run_batch(scenario: ScenarioConfig, scenario_yaml: str, *, repeat: int) -> BatchResult:
    """Run ``repeat`` fresh trials of ``scenario`` and return the batch result."""
    batch_id = generate_batch_id()
    config_hash = hashlib.sha256(scenario_yaml.encode("utf-8")).hexdigest()
    batch = BatchResult(
        scenario_name=scenario.name,
        scenario_config=asdict(scenario),
        scenario_yaml=scenario_yaml,
        config_hash=config_hash,
        requested_trials=repeat,
        trials=[],
    )
    for index in range(repeat):
        trial_id = make_trial_id(batch_id, index)
        seed = derive_seed(scenario.experiment.seed, index)
        try:
            result = run_trial(scenario, trial_id=trial_id, trial_index=index, seed=seed)
        except Exception as exc:  # noqa: BLE001 - convert to an explicit FAIL record
            result = TrialResult(
                trial_id=trial_id,
                trial_index=index,
                seed=seed,
                scenario_name=scenario.name,
                lifecycle_state="finished",
                passed=False,
                infrastructure_error=f"unhandled exception: {exc!r}",
                activation_ns=None,
                deactivation_ns=None,
                assertions=[],
                metrics=_empty_metrics(),
                events=[],
            )
        batch.trials.append(result)
    return batch


def _write_json_atomically(path: Path, payload: dict) -> None:
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    tmp.replace(path)


def main(argv: list[str] | None = None) -> None:
    """CLI entry point: ``run_scenario --scenario PATH --repeat N --output PATH``."""
    parser = argparse.ArgumentParser(prog="run_scenario")
    parser.add_argument("--scenario", required=True, type=Path)
    parser.add_argument("--repeat", type=int, default=1)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--overwrite", action="store_true")
    args = parser.parse_args(argv)

    if args.repeat < 1:
        print("error: --repeat must be at least 1", file=sys.stderr)
        sys.exit(2)
    if args.output.exists() and not args.overwrite:
        print(f"error: {args.output} already exists (use --overwrite)", file=sys.stderr)
        sys.exit(2)

    try:
        scenario_yaml = args.scenario.read_text(encoding="utf-8")
        scenario = load_scenario(args.scenario)
    except Exception as exc:  # noqa: BLE001 - report and exit with a config error code
        print(f"error: {exc}", file=sys.stderr)
        sys.exit(2)

    batch = run_batch(scenario, scenario_yaml, repeat=args.repeat)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    _write_json_atomically(args.output, batch.to_dict())

    if any(trial.infrastructure_error is not None for trial in batch.trials):
        sys.exit(2)
    sys.exit(0 if all(trial.passed for trial in batch.trials) else 1)


if __name__ == "__main__":
    main()
