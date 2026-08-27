from __future__ import annotations

import math

import pytest

from ros2_resilience.robot.model import RobotModel


def test_straight_line_motion_integrates_x() -> None:
    robot = RobotModel(initial_linear_x=1.0, initial_angular_z=0.0)
    pose = robot.step(2.0)
    assert pose.x == pytest.approx(2.0)
    assert pose.y == pytest.approx(0.0)
    assert pose.theta == pytest.approx(0.0)


def test_pure_rotation_changes_only_theta() -> None:
    robot = RobotModel(initial_linear_x=0.0, initial_angular_z=math.pi / 2)
    pose = robot.step(1.0)
    assert pose.x == pytest.approx(0.0)
    assert pose.y == pytest.approx(0.0)
    assert pose.theta == pytest.approx(math.pi / 2)


def test_arc_motion_uses_exact_kinematic_solution() -> None:
    robot = RobotModel(initial_linear_x=1.0, initial_angular_z=1.0)
    pose = robot.step(math.pi / 2)
    assert pose.x == pytest.approx(1.0, abs=1e-9)
    assert pose.y == pytest.approx(1.0, abs=1e-9)
    assert pose.theta == pytest.approx(math.pi / 2)


def test_zero_command_stops_immediately_next_step() -> None:
    robot = RobotModel(initial_linear_x=2.0, initial_angular_z=0.5)
    robot.step(1.0)
    robot.set_command(linear_x=0.0, angular_z=0.0)
    before = robot.pose
    after = robot.step(5.0)
    assert after == before


def test_set_command_takes_effect_on_next_step_only() -> None:
    robot = RobotModel()
    robot.set_command(linear_x=1.0, angular_z=0.0)
    assert robot.velocity.linear_x == 1.0
    pose = robot.step(1.0)
    assert pose.x == pytest.approx(1.0)


def test_theta_wraps_to_plus_minus_pi() -> None:
    robot = RobotModel(initial_linear_x=0.0, initial_angular_z=math.pi)
    pose = robot.step(1.5)
    assert -math.pi <= pose.theta <= math.pi


@pytest.mark.parametrize(
    "kwargs", [{"initial_linear_x": float("nan")}, {"initial_angular_z": float("inf")}]
)
def test_rejects_non_finite_initial_velocity(kwargs: dict[str, float]) -> None:
    with pytest.raises(ValueError, match="finite"):
        RobotModel(**kwargs)


def test_rejects_negative_dt() -> None:
    robot = RobotModel()
    with pytest.raises(ValueError, match="nonnegative"):
        robot.step(-1.0)


def test_rejects_non_finite_command() -> None:
    robot = RobotModel()
    with pytest.raises(ValueError, match="finite"):
        robot.set_command(linear_x=float("nan"), angular_z=0.0)
