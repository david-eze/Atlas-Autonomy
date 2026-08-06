"""Differential-drive kinematics and odometry integration.

Pure-python, no ROS imports so the core maths can be unit-tested
without a running ROS 2 environment.

Reference model (standard two-wheel differential drive, idealised
no-slip):

    v     = r / 2 * (wL + wR)
    omega = r / b * (wR - wL)

where ``r`` is the wheel radius, ``b`` the wheel separation and
``wL``/``wR`` the left/right wheel angular velocities.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Optional, Tuple


@dataclass
class DifferentialDriveConfig:
    """Fixed geometric parameters of the drive train."""

    wheel_radius: float
    wheel_separation: float


@dataclass
class WheelState:
    """Measured wheel angular positions and velocities."""

    left_position: float
    right_position: float
    left_velocity: float
    right_velocity: float


@dataclass
class Pose2D:
    """Planar pose in the odometry frame."""

    x: float
    y: float
    theta: float


class DifferentialDriveOdometry:
    """Integrates wheel encoder deltas into a 2D pose estimate.

    Uses the exact circular-arc integration rather than a first-order
    Euler approximation so that pure rotations (one wheel forward, one
    backward) do not drift away from straight-line integration error.
    """

    def __init__(
        self,
        config: DifferentialDriveConfig,
        initial_pose: Optional[Pose2D] = None,
    ) -> None:
        self._config = config
        self._pose = initial_pose if initial_pose is not None else Pose2D(0.0, 0.0, 0.0)
        self._last_wheels: Optional[WheelState] = None

    @property
    def pose(self) -> Pose2D:
        return self._pose

    @property
    def last_wheel_state(self) -> Optional[WheelState]:
        return self._last_wheels

    def reset(self, pose: Pose2D = Pose2D(0.0, 0.0, 0.0)) -> None:
        self._pose = pose
        self._last_wheels = None

    def reset_latest(self, wheels: WheelState) -> None:
        """Reset integration origin without touching the pose estimate."""
        self._last_wheels = wheels

    def update(self, wheels: WheelState) -> Pose2D:
        """Advances the pose estimate by the encoder delta since the
        previous call. The first call after construction or reset only
        stores the reference wheel state."""
        if self._last_wheels is None:
            self._last_wheels = wheels
            return self._pose

        r = self._config.wheel_radius
        b = self._config.wheel_separation

        d_left = (wheels.left_position - self._last_wheels.left_position) * r
        d_right = (wheels.right_position - self._last_wheels.right_position) * r
        self._last_wheels = wheels

        d_center = (d_left + d_right) / 2.0
        d_theta = (d_right - d_left) / b

        if abs(d_theta) > 1e-9:
            # Exact circular-arc odometry: the robot travels along an arc
            # of radius d_center / d_theta.
            radius = d_center / d_theta
            dx = radius * (math.sin(self._pose.theta + d_theta) - math.sin(self._pose.theta))
            dy = radius * (-math.cos(self._pose.theta + d_theta) + math.cos(self._pose.theta))
        else:
            # Straight-line motion.
            dx = d_center * math.cos(self._pose.theta)
            dy = d_center * math.sin(self._pose.theta)

        self._pose.x += dx
        self._pose.y += dy
        self._pose.theta = math.atan2(math.sin(self._pose.theta + d_theta),
                                      math.cos(self._pose.theta + d_theta))
        return self._pose

    @staticmethod
    def wheel_velocities_from_twist(
        config: DifferentialDriveConfig,
        linear: float,
        angular: float,
    ) -> Tuple[float, float]:
        """Inverse kinematics: robot twist -> left/right wheel angular velocity."""
        r = config.wheel_radius
        b = config.wheel_separation
        left = (2.0 * linear - angular * b) / (2.0 * r)
        right = (2.0 * linear + angular * b) / (2.0 * r)
        return left, right

    @staticmethod
    def twist_from_wheel_velocities(
        config: DifferentialDriveConfig,
        left_velocity: float,
        right_velocity: float,
    ) -> Tuple[float, float]:
        """Forward kinematics: wheel angular velocities -> (linear, angular)."""
        r = config.wheel_radius
        b = config.wheel_separation
        linear = r * (right_velocity + left_velocity) / 2.0
        angular = r * (right_velocity - left_velocity) / b
        return linear, angular
