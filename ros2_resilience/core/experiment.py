"""Pure trial lifecycle state machine.

This module owns no clock, no ROS handles, and performs no I/O. It consumes
monotonic clock ticks and explicit notifications, and returns concrete
:class:`Action` values describing what the ROS adapter layer should do next
(activate the fault, deactivate it, or finish the trial). All events observed
during any lifecycle state are still recorded by the caller and evaluated as
evidence at the end; this state machine only decides *when* things happen; it
never erases or re-evaluates evidence itself.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import StrEnum

from ros2_resilience.core.scenario import ScenarioConfig

__all__ = [
    "Action",
    "ActionType",
    "LifecycleError",
    "LifecycleState",
    "TrialLifecycle",
]

_ACTIVATION_REVISION = 1


class LifecycleState(StrEnum):
    """One state in the trial's lifecycle."""

    CREATED = "created"
    STARTING = "starting"
    BASELINING = "baselining"
    ACTIVATING = "activating"
    OBSERVING = "observing"
    FINALIZING = "finalizing"
    FINISHED = "finished"


class ActionType(StrEnum):
    """A concrete instruction for the ROS adapter layer to carry out."""

    ACTIVATE_FAULT = "activate_fault"
    DEACTIVATE_FAULT = "deactivate_fault"
    FINISH_TRIAL = "finish_trial"


@dataclass(frozen=True)
class Action:
    """One instruction returned by the lifecycle for the caller to perform."""

    type: ActionType
    reason: str = ""


class LifecycleError(ValueError):
    """Raised when a lifecycle method is called from an invalid state."""


class TrialLifecycle:
    """Drives one trial through CREATED -> ... -> FINISHED."""

    def __init__(self, scenario: ScenarioConfig) -> None:
        self._scenario = scenario
        self._state = LifecycleState.CREATED
        self._starting_since_ns: int | None = None
        self.baseline_start_ns: int | None = None
        self.activation_requested_ns: int | None = None
        self.activation_ns: int | None = None
        self.deactivation_requested_ns: int | None = None
        self.observation_deadline_ns: int | None = None
        self.infrastructure_error: str | None = None

    @property
    def state(self) -> LifecycleState:
        """Return the current lifecycle state."""
        return self._state

    @property
    def activation_revision(self) -> int:
        """Return the config revision this trial's activation waits for."""
        return _ACTIVATION_REVISION

    def start(self, *, now_ns: int) -> None:
        """Move CREATED -> STARTING."""
        self._require_state(LifecycleState.CREATED, "start")
        self._state = LifecycleState.STARTING
        self._starting_since_ns = now_ns

    def mark_ready(self, *, now_ns: int) -> None:
        """Move STARTING -> BASELINING once the graph is confirmed ready."""
        self._require_state(LifecycleState.STARTING, "mark_ready")
        self._state = LifecycleState.BASELINING
        self.baseline_start_ns = now_ns

    def on_config_applied(self, revision: int, *, now_ns: int) -> None:
        """Advance ACTIVATING -> OBSERVING when the matching revision applies."""
        if self._state is not LifecycleState.ACTIVATING or revision != self.activation_revision:
            return
        self._state = LifecycleState.OBSERVING
        self.activation_ns = now_ns
        self.observation_deadline_ns = now_ns + self._seconds_to_ns(
            self._scenario.experiment.observation_after_activation_sec
        )

    def fail(self, message: str, *, now_ns: int) -> Action:
        """Record an infrastructure failure and move straight to FINALIZING."""
        if self._state in (LifecycleState.FINALIZING, LifecycleState.FINISHED):
            raise LifecycleError(f"cannot fail from state {self._state.value}")
        del now_ns
        self.infrastructure_error = message
        self._state = LifecycleState.FINALIZING
        return Action(ActionType.FINISH_TRIAL, reason=message)

    def tick(self, *, now_ns: int) -> Action | None:
        """Advance time-driven transitions and return the next action, if any."""
        if self._state is LifecycleState.STARTING:
            return self._tick_starting(now_ns)
        if self._state is LifecycleState.BASELINING:
            return self._tick_baselining(now_ns)
        if self._state is LifecycleState.ACTIVATING:
            return self._tick_activating(now_ns)
        if self._state is LifecycleState.OBSERVING:
            return self._tick_observing(now_ns)
        return None

    def finish(self) -> None:
        """Move FINALIZING -> FINISHED."""
        self._require_state(LifecycleState.FINALIZING, "finish")
        self._state = LifecycleState.FINISHED

    def _tick_starting(self, now_ns: int) -> Action | None:
        assert self._starting_since_ns is not None
        elapsed_ns = now_ns - self._starting_since_ns
        if elapsed_ns >= self._seconds_to_ns(self._scenario.experiment.readiness_timeout_sec):
            return self.fail("readiness timeout", now_ns=now_ns)
        return None

    def _tick_baselining(self, now_ns: int) -> Action | None:
        assert self.baseline_start_ns is not None
        elapsed_ns = now_ns - self.baseline_start_ns
        if elapsed_ns < self._seconds_to_ns(self._scenario.experiment.healthy_baseline_sec):
            return None

        if self._scenario.is_healthy_control:
            self._state = LifecycleState.OBSERVING
            self.activation_ns = now_ns
            self.observation_deadline_ns = now_ns + self._seconds_to_ns(
                self._scenario.experiment.observation_after_activation_sec
            )
            return None

        self._state = LifecycleState.ACTIVATING
        self.activation_requested_ns = now_ns
        return Action(ActionType.ACTIVATE_FAULT)

    def _tick_activating(self, now_ns: int) -> Action | None:
        assert self.activation_requested_ns is not None
        elapsed_ns = now_ns - self.activation_requested_ns
        if elapsed_ns >= self._seconds_to_ns(self._scenario.experiment.readiness_timeout_sec):
            return self.fail("fault activation not acknowledged in time", now_ns=now_ns)
        return None

    def _tick_observing(self, now_ns: int) -> Action | None:
        assert self.observation_deadline_ns is not None
        if (
            not self._scenario.is_healthy_control
            and self.deactivation_requested_ns is None
            and self.activation_ns is not None
            and now_ns - self.activation_ns
            >= self._seconds_to_ns(self._scenario.fault.duration_sec)
        ):
            self.deactivation_requested_ns = now_ns
            return Action(ActionType.DEACTIVATE_FAULT)

        if now_ns >= self.observation_deadline_ns:
            self._state = LifecycleState.FINALIZING
            return Action(ActionType.FINISH_TRIAL)
        return None

    def _require_state(self, expected: LifecycleState, method: str) -> None:
        if self._state is not expected:
            raise LifecycleError(f"cannot call {method}() from state {self._state.value}")

    @staticmethod
    def _seconds_to_ns(seconds: float) -> int:
        return int(round(seconds * 1e9))
