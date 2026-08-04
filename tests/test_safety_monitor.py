"""Unit tests for safety monitor override and collision clearance logic."""

import unittest


def evaluate_safety(min_obstacle_distance: float, cmd_linear_vel: float, safety_threshold: float = 0.30):
    """Safety decision engine: overrides command if obstacle is within safety threshold."""
    if min_obstacle_distance < safety_threshold and cmd_linear_vel > 0:
        return True, 0.0
    return False, cmd_linear_vel


class TestSafetyMonitor(unittest.TestCase):
    def test_safe_operation(self):
        overridden, safe_v = evaluate_safety(min_obstacle_distance=1.2, cmd_linear_vel=0.5)
        self.assertFalse(overridden)
        self.assertEqual(safe_v, 0.5)

    def test_emergency_stop_triggered(self):
        overridden, safe_v = evaluate_safety(min_obstacle_distance=0.15, cmd_linear_vel=0.4)
        self.assertTrue(overridden)
        self.assertEqual(safe_v, 0.0)

    def test_reversing_near_obstacle_allowed(self):
        overridden, safe_v = evaluate_safety(min_obstacle_distance=0.20, cmd_linear_vel=-0.2)
        self.assertFalse(overridden)
        self.assertEqual(safe_v, -0.2)


if __name__ == '__main__':
    unittest.main()
