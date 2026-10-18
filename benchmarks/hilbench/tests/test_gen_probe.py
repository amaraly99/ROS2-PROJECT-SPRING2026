"""scripts/gen_probe.py: slam_traj_probe_unified.m differs from slam_traj_probe.m in exactly 2 lines."""
import hashlib
import re
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

from ._fakes import ROOT

SCRIPT = ROOT / "scripts" / "gen_probe.py"
PAIRS = {"mono": ("slam_traj_probe.m", "slam_traj_probe_unified.m"),
         "rgbd": ("slam_traj_probe_rgbd.m", "slam_traj_probe_rgbd_unified.m")}


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


class GenProbe(unittest.TestCase):
    def generated(self, which="mono"):
        """Text of the generated copy. Runs the script with --out DIR (a temp folder) when it has --out;
        else reads the committed generated file in matlab/."""
        self.assertTrue(SCRIPT.exists(), "scripts/gen_probe.py is missing")
        orig_name, gen_name = PAIRS[which]
        help_text = subprocess.run([sys.executable, str(SCRIPT), "--help"], capture_output=True, text=True).stdout
        if "--out" in help_text:
            with tempfile.TemporaryDirectory() as tmp:
                r = subprocess.run([sys.executable, str(SCRIPT), "--out", tmp], capture_output=True, text=True)
                self.assertEqual(r.returncode, 0, r.stdout + r.stderr)
                return (Path(tmp) / gen_name).read_text()
        r = subprocess.run([sys.executable, str(SCRIPT)], capture_output=True, text=True)
        self.assertEqual(r.returncode, 0, r.stdout + r.stderr)
        return (ROOT / "matlab" / gen_name).read_text()

    def test_exactly_two_lines_differ(self):
        """in: run gen_probe.py; compare its output with matlab/slam_traj_probe.m line by line
        exp: same number of lines; exactly 2 lines differ: the MODEL_NAME line and the target line
        bug: the copy is rewritten (more than 2 lines change) or one of the two literals is left old."""
        gen = self.generated().splitlines()
        orig = (ROOT / "matlab" / PAIRS["mono"][0]).read_text().splitlines()
        self.assertEqual(len(gen), len(orig))
        changed = [(a, b) for a, b in zip(orig, gen) if a != b]
        self.assertEqual(len(changed), 2, changed)
        olds = " ".join(a for a, _ in changed)
        self.assertIn("MODEL_NAME", olds)
        self.assertIn("target", olds)

    def test_exactly_two_lines_differ_rgbd(self):
        """in: the same for matlab/slam_traj_probe_rgbd.m
        exp: same number of lines; exactly 2 lines differ (MODEL_NAME and target)
        bug: the RGB-D copy needs a third edit and nobody notices."""
        gen = self.generated("rgbd").splitlines()
        orig = (ROOT / "matlab" / PAIRS["rgbd"][0]).read_text().splitlines()
        self.assertEqual(len(gen), len(orig))
        changed = [(a, b) for a, b in zip(orig, gen) if a != b]
        self.assertEqual(len(changed), 2, changed)

    def test_new_literals(self):
        """in: the generated text
        exp: the MODEL_NAME line names the unified model (the literal 'hil_closed_loop_unified' or getenv('HIL_MODEL'));
             a target line holds 35.1, 2.92 and 3.08; no line still holds [35.5; 23.7; 3.2] (the old sign)
        bug: the unified probe still aims at the old sign or asks Simulink about the old model."""
        text = self.generated()
        model = [l for l in text.splitlines() if re.match(r"\s*MODEL_NAME\s*=", l)]
        self.assertEqual(len(model), 1, model)
        self.assertTrue("hil_closed_loop_unified" in model[0] or "getenv('HIL_MODEL')" in model[0], model)
        self.assertNotIn("'hil_closed_loop';", model[0])
        target = [l for l in text.splitlines() if l.strip().startswith("target") and "=" in l and "[" in l]
        self.assertTrue(any("35.1" in l and "2.92" in l and "3.08" in l for l in target), target)
        self.assertNotIn("[35.5; 23.7; 3.2]", text)

    def test_original_file_is_untouched(self):
        """in: sha256 of matlab/slam_traj_probe.m and slam_traj_probe_rgbd.m before and after running gen_probe.py
        exp: equal
        bug: the generator edits the original probe that the controller bench still uses."""
        before = {k: sha(ROOT / "matlab" / v[0]) for k, v in PAIRS.items()}
        self.generated()
        self.assertEqual({k: sha(ROOT / "matlab" / v[0]) for k, v in PAIRS.items()}, before)

    def test_original_probe_still_has_old_literals(self):
        """in: matlab/slam_traj_probe.m
        exp: still has MODEL_NAME = 'hil_closed_loop'; and target = [35.5; 23.7; 3.2];
        bug: somebody edited the original instead of the copy."""
        text = (ROOT / "matlab" / PAIRS["mono"][0]).read_text()
        self.assertIn("MODEL_NAME = 'hil_closed_loop';", text)
        self.assertIn("target = [35.5; 23.7; 3.2];", text)

    def test_script_reads_sign_from_geometry(self):
        """in: the text of scripts/gen_probe.py
        exp: it mentions bench_geometry (the sign comes from there)
        bug: the sign is typed again in the generator and drifts from bench_geometry.SIGN."""
        self.assertIn("bench_geometry", SCRIPT.read_text())


if __name__ == "__main__":
    unittest.main()
