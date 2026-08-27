from __future__ import annotations

from typing import Any


def scenario_dict(**overrides: Any) -> dict[str, Any]:
    """Return a minimal valid dropout scenario dict, with deep overrides."""
    base: dict[str, Any] = {
        "schema_version": 1,
        "name": "test_dropout",
        "system": {
            "input_topic": "/odom_raw",
            "output_topic": "/odom",
            "message_type": "nav_msgs/msg/Odometry",
            "command_topic": "/cmd_vel",
            "robot": {
                "publish_rate_hz": 20.0,
                "initial_linear_velocity_mps": 0.5,
                "initial_angular_velocity_rad_s": 0.0,
            },
        },
        "experiment": {
            "seed": 7,
            "readiness_timeout_sec": 10.0,
            "healthy_baseline_sec": 1.0,
            "observation_after_activation_sec": 4.0,
        },
        "fault": {
            "type": "dropout",
            "parameters": {"drop_probability": 0.8},
            "duration_sec": 3.0,
        },
        "watchdog": {
            "max_age_sec": 0.3,
            "expected_rate_hz": 20.0,
            "loss_window_sec": 2.0,
            "max_loss_fraction": 0.5,
            "check_period_sec": 0.01,
            "recovery": "stop",
        },
        "expect": {
            "no_violation_before_fault": True,
            "fault": {"observable": True, "within_sec": 1.5},
            "watchdog": {"violation": "loss_fraction_exceeded", "within_sec": 0.8},
            "recovery": {"action": "stop", "command_within_sec": 0.1, "observed_within_sec": 0.3},
            "robot": {
                "linear_velocity_below_mps": 0.01,
                "angular_velocity_below_rad_s": 0.01,
                "within_sec": 1.0,
                "hold_sec": 0.25,
            },
        },
    }
    result = dict(base)
    result.update(overrides)
    return result


def healthy_control_dict(**overrides: Any) -> dict[str, Any]:
    """Return a minimal valid healthy-control scenario dict.

    Top-level sections given in ``overrides`` fully replace the default for
    that key (no recursive merging), matching :func:`scenario_dict`.
    """
    base = scenario_dict(
        name="test_healthy_control",
        fault={"type": "none", "parameters": {}, "duration_sec": 0},
        expect={
            "no_violation_before_fault": True,
            "fault": None,
            "watchdog": None,
            "recovery": None,
            "robot": None,
        },
    )
    result = dict(base)
    result.update(overrides)
    return result
