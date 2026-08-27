"""A planar kinematic robot with persistent velocity and immediate stopping."""

from __future__ import annotations

import math
from dataclasses import dataclass

__all__ = ["Pose", "RobotModel", "Velocity"]


@dataclass(frozen=True)
class Pose:
    """Planar robot pose."""

    x: float
    y: float
    theta: float


@dataclass(frozen=True)
class Velocity:
    """Commanded planar velocity."""

    linear_x: float
    angular_z: float


class RobotModel:
    """Integrate a persistent velocity command into a planar pose.

    A new command takes effect immediately and holds until replaced; there is
    no acceleration ramp, so setting the zero command stops the robot exactly
    at the next integration step. The model owns no clock: callers advance
    time explicitly via :meth:`step`.
    """

    def __init__(self, *, initial_linear_x: float = 0.0, initial_angular_z: float = 0.0) -> None:
        self._validate_finite("initial_linear_x", initial_linear_x)
        self._validate_finite("initial_angular_z", initial_angular_z)
        self._pose = Pose(x=0.0, y=0.0, theta=0.0)
        self._velocity = Velocity(linear_x=initial_linear_x, angular_z=initial_angular_z)

    @property
    def pose(self) -> Pose:
        """Return the current pose."""
        return self._pose

    @property
    def velocity(self) -> Velocity:
        """Return the currently commanded velocity."""
        return self._velocity

    def set_command(self, *, linear_x: float, angular_z: float) -> None:
        """Replace the commanded velocity, effective immediately."""
        self._validate_finite("linear_x", linear_x)
        self._validate_finite("angular_z", angular_z)
        self._velocity = Velocity(linear_x=linear_x, angular_z=angular_z)

    def step(self, dt_sec: float) -> Pose:
        """Integrate the current command for ``dt_sec`` seconds and return the pose."""
        if not isinstance(dt_sec, (int, float)) or isinstance(dt_sec, bool):
            raise TypeError("dt_sec must be a number")
        if not math.isfinite(dt_sec) or dt_sec < 0.0:
            raise ValueError("dt_sec must be finite and nonnegative")

        v, omega = self._velocity.linear_x, self._velocity.angular_z
        theta = self._pose.theta
        if omega == 0.0:
            new_x = self._pose.x + v * dt_sec * math.cos(theta)
            new_y = self._pose.y + v * dt_sec * math.sin(theta)
            new_theta = theta
        else:
            new_theta = theta + omega * dt_sec
            new_x = self._pose.x + (v / omega) * (math.sin(new_theta) - math.sin(theta))
            new_y = self._pose.y - (v / omega) * (math.cos(new_theta) - math.cos(theta))

        self._pose = Pose(x=new_x, y=new_y, theta=_wrap_angle(new_theta))
        return self._pose

    @staticmethod
    def _validate_finite(name: str, value: float) -> None:
        if not isinstance(value, (int, float)) or isinstance(value, bool):
            raise TypeError(f"{name} must be a number")
        if not math.isfinite(value):
            raise ValueError(f"{name} must be finite")


def _wrap_angle(theta: float) -> float:
    return math.atan2(math.sin(theta), math.cos(theta))
