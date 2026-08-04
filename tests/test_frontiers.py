"""Unit tests for frontier detection and multi-criterion scoring logic."""

import math
import unittest


def detect_frontiers(grid, width, height):
    """Find frontier cells (free cells adjacent to at least one unknown cell)."""
    frontiers = []
    # Grid values: -1 = unknown, 0 = free, 100 = occupied
    for y in range(1, height - 1):
        for x in range(1, width - 1):
            if grid[y][x] == 0:
                has_unknown = False
                for dy in [-1, 0, 1]:
                    for dx in [-1, 0, 1]:
                        if grid[y + dy][x + dx] == -1:
                            has_unknown = True
                            break
                    if has_unknown:
                        break
                if has_unknown:
                    frontiers.append((x, y))
    return frontiers


def score_frontier(frontier, robot_pose, info_gain=10, dist_weight=1.0, info_weight=2.0):
    """Multi-criterion score: higher is better."""
    dist = math.hypot(frontier[0] - robot_pose[0], frontier[1] - robot_pose[1])
    score = (info_weight * info_gain) - (dist_weight * dist)
    return score


class TestFrontiers(unittest.TestCase):
    def test_frontier_detection(self):
        grid = [
            [0, 0, -1, -1, -1],
            [0, 0, -1, -1, -1],
            [0, 0, -1, -1, -1],
            [0, 0, -1, -1, -1],
            [0, 0, -1, -1, -1],
        ]
        frontiers = detect_frontiers(grid, 5, 5)
        self.assertIn((1, 1), frontiers)
        self.assertIn((1, 2), frontiers)

    def test_frontier_scoring_prefers_closer(self):
        f_close = (2, 2)
        f_far = (10, 10)
        robot_pose = (0, 0)

        s_close = score_frontier(f_close, robot_pose)
        s_far = score_frontier(f_far, robot_pose)

        self.assertGreater(s_close, s_far)


if __name__ == '__main__':
    unittest.main()
