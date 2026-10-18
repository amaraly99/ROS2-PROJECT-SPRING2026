"""model_check.py: read the .slx files of the repo, compare with the registry and with MATLAB."""
import math
import tempfile
import unittest
from pathlib import Path

import bench_geometry as geo
from hilbench import model_check as mc

from ._fakes import ROOT

MATLAB = ROOT / "matlab"

REGISTRY_TEXT = """\
models:
  mono:   {file: matlab/hil_closed_loop_unified.slx,      right_cam: off, depth_port: off}
  rgbd:   {file: matlab/hil_closed_loop_unified_rgbd.slx, right_cam: off, depth_port: on}
  stereo:
    "0.11": {file: matlab/hil_closed_loop_unified_stereo_011.slx, right_cam: on, right_y: -0.11, depth_port: off}
    "0.36": {file: matlab/hil_closed_loop_baseline_036.slx,       right_cam: on, right_y: -0.36, depth_port: off}
"""


class ReadSlx(unittest.TestCase):
    def test_repo_base_model(self):
        """in: read_slx(matlab/hil_closed_loop.slx), the real repo file
        exp: y_ic 20.0, z_ic 5.0, x_ic 0.0 (missing IC gives 0.0), fx_left 554, right_cam 'off'
        bug: the integrator initial conditions are read from the wrong block, or a missing IC raises."""
        d = mc.read_slx(MATLAB / "hil_closed_loop.slx")
        self.assertEqual((d["y_ic"], d["z_ic"], d["x_ic"]), (20.0, 5.0, 0.0))
        self.assertEqual(d["fx_left"], 554)
        self.assertEqual(d["right_cam"], "off")

    def test_baseline_036(self):
        """in: read_slx(matlab/hil_closed_loop_baseline_036.slx)
        exp: right_y -0.36 (the right camera block is still Commented=on in the file, so right_cam is 'off':
             set_stereo(1) switches it on at run time)
        bug: the stereo baseline sign is lost (+0.36), or the mount is read from the left camera."""
        d = mc.read_slx(MATLAB / "hil_closed_loop_baseline_036.slx")
        self.assertAlmostEqual(d["right_y"], -0.36)
        self.assertEqual(d["right_cam"], "off")

    def test_baseline_011_and_061(self):
        """in: read_slx of baseline_011 and baseline_061
        exp: right_y -0.11 and -0.61
        bug: every baseline file gives the same right_y."""
        self.assertAlmostEqual(mc.read_slx(MATLAB / "hil_closed_loop_baseline_011.slx")["right_y"], -0.11)
        self.assertAlmostEqual(mc.read_slx(MATLAB / "hil_closed_loop_baseline_061.slx")["right_y"], -0.61)

    def test_baseline_rgbd(self):
        """in: read_slx(matlab/hil_closed_loop_baseline_RGBD.slx)
        exp: depth_port 'on'
        bug: DepthOutportEnabled is not found in the XML."""
        self.assertEqual(mc.read_slx(MATLAB / "hil_closed_loop_baseline_RGBD.slx")["depth_port"], "on")


class Expected(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self._tmp.cleanup)
        self.reg_path = Path(self._tmp.name) / "models.yaml"
        self.reg_path.write_text(REGISTRY_TEXT)
        self.reg = mc.load_registry(self.reg_path)

    def test_mono_start_values_come_from_geometry(self):
        """in: expected('mono') from a registry file
        exp: x_ic 15.12, y_ic 3.84, z_ic 7.5, yaw_ic 2*pi-0.046, fx_left 554, right_cam 'off', depth_port 'off'
        bug: the start is typed again in the registry and drifts from bench_geometry.START."""
        e = mc.expected("mono", registry=self.reg)
        self.assertEqual((e["x_ic"], e["y_ic"], e["z_ic"]), geo.START[:3])
        self.assertAlmostEqual(e["yaw_ic"], geo.START[3], places=9)
        self.assertEqual(e["fx_left"], 554)
        self.assertEqual((e["right_cam"], e["depth_port"]), ("off", "off"))

    def test_stereo_baseline_and_rgbd(self):
        """in: expected('stereo', 0.36), expected('stereo', 0.11), expected('rgbd')
        exp: right_y -0.36, -0.11; right_cam 'on'; depth_port 'on' for rgbd
        bug: the baseline key '0.36' (text in YAML) is not matched with the float 0.36."""
        self.assertAlmostEqual(mc.expected("stereo", 0.36, registry=self.reg)["right_y"], -0.36)
        self.assertAlmostEqual(mc.expected("stereo", 0.11, registry=self.reg)["right_y"], -0.11)
        self.assertEqual(mc.expected("stereo", 0.36, registry=self.reg)["right_cam"], "on")
        self.assertEqual(mc.expected("rgbd", registry=self.reg)["depth_port"], "on")

    def test_unknown_baseline_raises(self):
        """in: expected('stereo', 0.20) (not in the registry)
        exp: KeyError or ValueError
        bug: an unknown baseline gives an empty expectation, and check_live compares nothing."""
        with self.assertRaises((KeyError, ValueError)):
            mc.expected("stereo", 0.20, registry=self.reg)

    def test_repo_registry_file(self):
        """in: load_registry() default path config/hil/models.yaml of the repo; expected('stereo', 0.36)
        exp: right_y -0.36; expected('mono')['y_ic'] == 3.84
        bug: the registry file is missing or has another layout than CONTRACT section 3."""
        reg = mc.load_registry()
        self.assertAlmostEqual(mc.expected("stereo", 0.36, registry=reg)["right_y"], -0.36)
        self.assertEqual(mc.expected("mono", registry=reg)["y_ic"], 3.84)
        for base in (0.11, 0.36, 0.42, 0.54, 0.61):
            self.assertAlmostEqual(mc.expected("stereo", base, registry=reg)["right_y"], -base)


class CheckLive(unittest.TestCase):
    @staticmethod
    def matching_info(mode="mono", base=None):
        exp = mc.expected(mode, base)
        return {k: v for k, v in exp.items() if k != "file"}, exp

    def test_equal_gives_no_mismatch(self):
        """in: check_live(info, expected) with info copied from expected
        exp: []
        bug: a text field (right_cam) or float compare gives a false mismatch."""
        info, exp = self.matching_info()
        self.assertEqual(mc.check_live(info, exp), [])

    def test_tolerance_1e_6(self):
        """in: y_ic differs by 1e-9, then by 1e-3
        exp: [] for 1e-9; one line naming y_ic for 1e-3
        bug: the tolerance is exact equality (float noise) or is too wide."""
        info, exp = self.matching_info()
        info["y_ic"] = exp["y_ic"] + 1e-9
        self.assertEqual(mc.check_live(info, exp), [])
        info["y_ic"] = exp["y_ic"] + 1e-3
        out = mc.check_live(info, exp)
        self.assertEqual(len(out), 1)
        self.assertIn("y_ic", out[0])

    def test_text_mismatch(self):
        """in: right_cam 'on' in MATLAB, 'off' expected
        exp: one line that names right_cam
        bug: a stereo model is accepted for a mono session."""
        info, exp = self.matching_info()
        info["right_cam"] = "on"
        out = mc.check_live(info, exp)
        self.assertEqual(len(out), 1)
        self.assertIn("right_cam", out[0])

    def test_repo_base_is_not_the_unified_model(self):
        """in: the repo base model (read_slx of matlab/hil_closed_loop.slx) against expected('mono')
        exp: mismatch lines name y_ic and z_ic (20 vs 3.84, 5 vs 7.5)
        bug: the check passes on the old start, so a wrong model could fly the whole campaign."""
        info = mc.read_slx(MATLAB / "hil_closed_loop.slx")
        out = mc.check_live(info, mc.expected("mono"))
        self.assertTrue(any("y_ic" in line for line in out), out)
        self.assertTrue(any("z_ic" in line for line in out), out)

    def test_math_sanity_of_yaw(self):
        """in: expected('mono')['yaw_ic']
        exp: 2*pi - 0.046 = 6.2372 (+-1e-4)
        bug: yaw is stored in degrees."""
        self.assertAlmostEqual(mc.expected("mono")["yaw_ic"], 2 * math.pi - 0.046, delta=1e-4)


class CheckFiles(unittest.TestCase):
    def test_repo_registry_has_no_file_problem(self):
        """in: check_files() with the repo registry (unified .slx files are not in git)
        exp: [] (entries whose file does not exist are skipped)
        bug: a missing unified file is reported as a mismatch and blocks the check."""
        self.assertEqual(mc.check_files(), [])

    def test_wrong_start_in_a_real_file_is_reported(self):
        """in: registry entry mono -> matlab/hil_closed_loop.slx, the old base model (start y 20, z 5)
        exp: lines name y_ic and z_ic; no line names fx_left (that value is right)
        bug: check_files compares only the focal length, or reports right values as wrong.
        (2026-10-18: the baseline .slx files now hold the unified start, so the old base model is the wrong-start file.)"""
        with tempfile.TemporaryDirectory() as tmp:
            p = Path(tmp) / "models.yaml"
            p.write_text("models:\n  mono: {file: matlab/hil_closed_loop.slx, right_cam: off, depth_port: off}\n  stereo: {}\n")
            out = mc.check_files(registry=mc.load_registry(p), ws_root=ROOT)
        joined = "\n".join(out)
        self.assertIn("y_ic", joined)
        self.assertIn("z_ic", joined)
        self.assertNotIn("fx_left", joined)

    def test_stereo_file_right_camera_stored_off_is_fine(self):
        """in: the repo registry entry stereo 0.61 -> matlab/hil_closed_loop_baseline_061.slx (right camera commented out)
        exp: no line names right_cam
        bug: every stereo file is reported, because set_stereo(1) turns the right camera on only at load."""
        out = [l for l in mc.check_files() if "061" in l]
        self.assertNotIn("right_cam", "\n".join(out))

if __name__ == "__main__":
    unittest.main()
