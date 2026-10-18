"""bench_geometry.py: the shared sign, start and stop distances."""
import math
import unittest

import bench_geometry as geo


class GeometryTests(unittest.TestCase):
    def test_sign_tuple(self):
        """in: geo.SIGN
        exp: (35.1, 2.92, 3.08)
        bug: the sign is copied into two files with two different values."""
        self.assertEqual(geo.SIGN, (35.1, 2.92, 3.08))

    def test_start_position(self):
        """in: geo.START[:3]
        exp: (15.12, 3.84, 7.5)
        bug: the runner keeps a start that differs from the unified model start."""
        self.assertEqual(geo.START[:3], (15.12, 3.84, 7.5))

    def test_start_yaw(self):
        """in: geo.START[3]
        exp: 2*pi - 0.046 rad (to 12 places)
        bug: yaw is given in degrees, or the 0.046 offset is lost."""
        self.assertAlmostEqual(geo.START[3], 2 * math.pi - 0.046, places=12)

    def test_standoff(self):
        """in: geo.STANDOFF
        exp: 3.148 (554*1.5/(0.55*480) = 3.14773), 3 places
        bug: the stop distance formula uses a wrong constant."""
        self.assertAlmostEqual(geo.STANDOFF, 3.148, places=3)

    def test_slam_standoff(self):
        """in: geo.SLAM_STANDOFF
        exp: 3.0
        bug: the probe stop distance is confused with the controller stop distance."""
        self.assertEqual(geo.SLAM_STANDOFF, 3.0)

    def test_bearing_from_start_points_at_sign(self):
        """in: geo.bearing_deg(15.12, 3.84)
        exp: 357.36 deg (+-0.01); it also equals the start yaw in degrees (+-0.05)
        bug: bearing uses (x, y) in the wrong order or forgets the modulo 360."""
        b = geo.bearing_deg(geo.START[0], geo.START[1])
        self.assertAlmostEqual(b, 357.36, delta=0.01)
        self.assertAlmostEqual(b, math.degrees(geo.START[3]), delta=0.05)

    def test_angle_diff_wraps(self):
        """in: angle_diff(10, 350), (350, 10), (0, 180), (5, 5)
        exp: 20, 20, 180, 0
        bug: a difference across the 0/360 line gives 340 instead of 20."""
        self.assertAlmostEqual(geo.angle_diff(10, 350), 20.0)
        self.assertAlmostEqual(geo.angle_diff(350, 10), 20.0)
        self.assertAlmostEqual(geo.angle_diff(0, 180), 180.0)
        self.assertAlmostEqual(geo.angle_diff(5, 5), 0.0)


if __name__ == "__main__":
    unittest.main()
