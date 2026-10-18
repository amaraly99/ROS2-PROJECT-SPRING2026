"""The cruise math of SLAM_RUNNER_MIGRATION_PLAN.html and the shipped YAML files."""
import math
import unittest

import bench_geometry as geo
import yaml

from ._fakes import ROOT

VX = 2.0
LAM = 8.0
CYCLES = 2


def distance_available():
    return (geo.SIGN[0] - geo.SLAM_STANDOFF) - geo.START[0]


def yaw_gain(vx, lam):
    w = 2 * math.pi * vx / lam
    k = 1.5                       # rad/s, the P gain of the probe yaw loop
    return 1.0 / math.sqrt(1.0 + (w / k) ** 2)


class CruiseMath(unittest.TestCase):
    def test_distance_available(self):
        """in: (SIGN x - SLAM_STANDOFF) - START x
        exp: 16.98 m
        bug: the standoff of the controllers (3.15) is used for the probe (3.0)."""
        self.assertAlmostEqual(distance_available(), 16.98, places=6)

    def test_r1_no_overshoot(self):
        """in: N = 2 cycles of 8.0 m
        exp: 2 * 8.0 <= D - 0.5 (16.0 <= 16.48)
        bug: the cruise ends beyond the goal and the drone passes the sign."""
        self.assertLessEqual(CYCLES * LAM, distance_available() - 0.5)

    def test_r2_yaw_gain(self):
        """in: yaw_gain(vx 2.0, lambda 8.0) = 1 / sqrt(1 + (2*pi*2.0/8.0/1.5)**2)
        exp: about 0.69 and >= 0.6
        bug: the yaw sweep is attenuated more than the rule allows."""
        g = yaw_gain(VX, LAM)
        self.assertAlmostEqual(g, 0.69, places=2)
        self.assertGreaterEqual(g, 0.6)

    def test_three_cycles_do_not_fit(self):
        """in: N = 3 with the shortest wavelength that R2 allows (2*pi*vx/2.0 = 6.28 m)
        exp: 3 * 6.28 = 18.85 > D - 0.5 = 16.48, so N = 3 is empty
        bug: the choice of N = 2 is not forced by the rules."""
        lam_min = 2 * math.pi * VX / 2.0
        self.assertGreater(3 * lam_min, distance_available() - 0.5)
        self.assertLessEqual(2 * lam_min, distance_available() - 0.5)

    def test_default_probe_overshoots(self):
        """in: the default probe 3 x 10.5 m
        exp: 31.5 m > D (16.98) by far
        bug: nobody sees why the default probe is not used."""
        self.assertGreater(3 * 10.5, distance_available())


class ShippedYaml(unittest.TestCase):
    def test_slam_bench_yaml_cruise_defaults(self):
        """in: config/hil/slam_bench.yaml, section slam
        exp: cycles 1, wavelength_m 13.0, vx 2.0, camera_yaw_offset_deg 80, init_axis 'path' (wider view, 2026-10-18)
        bug: the shipped file flies the default probe that overshoots the sign, or faces the plain view."""
        cfg = yaml.safe_load((ROOT / "config" / "hil" / "slam_bench.yaml").read_text())["slam"]
        self.assertEqual((cfg["cycles"], cfg["wavelength_m"], cfg["vx"], cfg["camera_yaw_offset_deg"], cfg["init_axis"]),
                         (1, 13.0, 2.0, 80, "path"))


if __name__ == "__main__":
    unittest.main()
