import math

import pytest

from robot_sensors.differential_drive import (
    DifferentialDriveConfig,
    DifferentialDriveOdometry,
    Pose2D,
    WheelState,
)

CONFIG = DifferentialDriveConfig(wheel_radius=0.1, wheel_separation=0.32)


def test_forward_kinematics_straight_line():
    """Both wheels at the same speed -> pure translation, no rotation."""
    linear, angular = DifferentialDriveOdometry.twist_from_wheel_velocities(
        CONFIG, 10.0, 10.0)
    assert linear == pytest.approx(1.0)
    assert angular == pytest.approx(0.0)


def test_forward_kinematics_rotation_in_place():
    """Equal and opposite wheel speeds -> pure rotation."""
    linear, angular = DifferentialDriveOdometry.twist_from_wheel_velocities(
        CONFIG, -10.0, 10.0)
    assert linear == pytest.approx(0.0)
    assert angular == pytest.approx(0.1 * 20.0 / 0.32)


def test_inverse_kinematics_round_trip():
    """Forward then inverse kinematics returns the original twist."""
    config = CONFIG
    for linear, angular in [(0.5, 0.2), (1.0, 0.0), (-0.3, 1.4)]:
        left, right = DifferentialDriveOdometry.wheel_velocities_from_twist(
            config, linear, angular)
        lin2, ang2 = DifferentialDriveOdometry.twist_from_wheel_velocities(
            config, left, right)
        assert lin2 == pytest.approx(linear, abs=1e-9)
        assert ang2 == pytest.approx(angular, abs=1e-9)


def test_odometry_straight_line():
    """Advancing both wheels equally moves the robot forward along the
    initial heading without drift."""
    odom = DifferentialDriveOdometry(CONFIG)
    odom.update(WheelState(0.0, 0.0, 0.0, 0.0))
    odom.update(WheelState(1.0, 1.0, 0.0, 0.0))
    pose = odom.update(WheelState(2.0, 2.0, 0.0, 0.0))
    assert pose.x == pytest.approx(0.2)
    assert pose.y == pytest.approx(0.0)
    assert pose.theta == pytest.approx(0.0)


def test_odometry_rotation_in_place():
    """One wheel forward, one backward -> pure rotation about the centre."""
    odom = DifferentialDriveOdometry(CONFIG)
    odom.update(WheelState(0.0, 0.0, 0.0, 0.0))
    odom.update(WheelState(1.0, -1.0, 0.0, 0.0))
    pose = odom.update(WheelState(2.0, -2.0, 0.0, 0.0))
    assert pose.theta == pytest.approx(2 * 0.2 / 0.32)
    assert pose.x == pytest.approx(0.0, abs=1e-9)
    assert pose.y == pytest.approx(0.0, abs=1e-9)


def test_odometry_heading_wraps():
    """Heading is normalised to (-pi, pi]."""
    odom = DifferentialDriveOdometry(CONFIG)
    odom.update(WheelState(0.0, 0.0, 0.0, 0.0))
    odom.update(WheelState(0.0, math.pi))
    pose = odom.update(WheelState(0.0, 2.0 * math.pi))
    assert -math.pi <= pose.theta <= math.pi


def test_reset_clears_integration():
    odom = DifferentialDriveOdometry(CONFIG)
    odom.update(WheelState(0.0, 0.0, 0.0, 0.0))
    odom.update(WheelState(10.0, 10.0, 0.0, 0.0))
    odom.reset(Pose2D(5.0, 5.0, 1.0))
    pose = odom.update(WheelState(10.0, 10.0, 0.0, 0.0))
    assert pose.x == pytest.approx(5.0)
    assert pose.y == pytest.approx(5.0)
    assert pose.theta == pytest.approx(1.0)
