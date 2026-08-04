"""Unit tests for TF2 coordinate transformations and matrix math."""

import math
import unittest


def transform_point_2d(px: float, py: float, tx: float, ty: float, theta: float):
    """Transform point (px, py) by translation (tx, ty) and rotation theta."""
    cos_t = math.cos(theta)
    sin_t = math.sin(theta)
    rx = px * cos_t - py * sin_t + tx
    ry = px * sin_t + py * cos_t + ty
    return rx, ry


class TestTF2Transforms(unittest.TestCase):
    def test_translation_only(self):
        rx, ry = transform_point_2d(1.0, 0.0, 5.0, 2.0, 0.0)
        self.assertAlmostEqual(rx, 6.0)
        self.assertAlmostEqual(ry, 2.0)

    def test_rotation_only(self):
        rx, ry = transform_point_2d(1.0, 0.0, 0.0, 0.0, math.pi / 2.0)
        self.assertAlmostEqual(rx, 0.0)
        self.assertAlmostEqual(ry, 1.0)

    def test_combined_transform(self):
        rx, ry = transform_point_2d(1.0, 1.0, 2.0, 3.0, math.pi)
        self.assertAlmostEqual(rx, 1.0)
        self.assertAlmostEqual(ry, 2.0)


if __name__ == '__main__':
    unittest.main()
