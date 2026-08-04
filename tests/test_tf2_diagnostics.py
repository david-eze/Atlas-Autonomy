"""Unit tests for TF2 tree diagnostic checker logic."""

import unittest

REQUIRED_EDGES = [
    ('map', 'odom'),
    ('odom', 'base_link'),
    ('base_link', 'laser_link'),
    ('base_link', 'camera_link'),
    ('base_link', 'imu_link'),
]


def check_tf_tree_health(existing_edges, reachable_frames):
    """Check if all required edges exist and all sensors are reachable from map."""
    missing_edges = [edge for edge in REQUIRED_EDGES if edge not in existing_edges]
    sensor_leaves = ['laser_link', 'camera_link', 'imu_link']
    unreachable = [s for s in sensor_leaves if s not in reachable_frames]

    is_healthy = (len(missing_edges) == 0) and (len(unreachable) == 0)
    return is_healthy, missing_edges, unreachable


class TestTf2Diagnostics(unittest.TestCase):
    def test_healthy_tree(self):
        edges = set(REQUIRED_EDGES)
        reachable = {'laser_link', 'camera_link', 'imu_link'}
        healthy, missing, unreach = check_tf_tree_health(edges, reachable)
        self.assertTrue(healthy)
        self.assertEqual(len(missing), 0)
        self.assertEqual(len(unreach), 0)

    def test_missing_edge_detected(self):
        edges = {('map', 'odom'), ('base_link', 'laser_link')}
        reachable = {'laser_link'}
        healthy, missing, unreach = check_tf_tree_health(edges, reachable)
        self.assertFalse(healthy)
        self.assertIn(('odom', 'base_link'), missing)
        self.assertIn('camera_link', unreach)


if __name__ == '__main__':
    unittest.main()
