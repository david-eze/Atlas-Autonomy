"""Unit tests for differential drive kinematics and odometry calculations."""

import math
import unittest


def forward_kinematics(left_vel: float, right_vel: float, wheel_separation: float, wheel_radius: float):
    """Compute linear and angular velocity from wheel speeds."""
    v_left = left_vel * wheel_radius
    v_right = right_vel * wheel_radius
    linear_v = (v_right + v_left) / 2.0
    angular_w = (v_right - v_left) / wheel_separation
    return linear_v, angular_w


def inverse_kinematics(linear_v: float, angular_w: float, wheel_separation: float, wheel_radius: float):
    """Compute left and right wheel velocities from linear and angular commands."""
    v_right = linear_v + (angular_w * wheel_separation / 2.0)
    v_left = linear_v - (angular_w * wheel_separation / 2.0)
    left_vel = v_left / wheel_radius
    right_vel = v_right / wheel_radius
    return left_vel, right_vel


class TestKinematics(unittest.TestCase):
    def setUp(self):
        self.wheel_separation = 0.36
        self.wheel_radius = 0.08

    def test_straight_motion(self):
        v, w = forward_kinematics(10.0, 10.0, self.wheel_separation, self.wheel_radius)
        self.assertAlmostEqual(v, 0.8)
        self.assertAlmostEqual(w, 0.0)

    def test_pure_rotation(self):
        v, w = forward_kinematics(-10.0, 10.0, self.wheel_separation, self.wheel_radius)
        self.assertAlmostEqual(v, 0.0)
        self.assertAlmostEqual(w, 1.6 / 0.36)

    def test_inverse_roundtrip(self):
        target_v = 0.5
        target_w = 0.2
        l_vel, r_vel = inverse_kinematics(target_v, target_w, self.wheel_separation, self.wheel_radius)
        calc_v, calc_w = forward_kinematics(l_vel, r_vel, self.wheel_separation, self.wheel_radius)
        self.assertAlmostEqual(target_v, calc_v)
        self.assertAlmostEqual(target_w, calc_w)


if __name__ == '__main__':
    unittest.main()
