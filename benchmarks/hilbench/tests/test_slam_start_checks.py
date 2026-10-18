"""SLAM start checks: missing SLAM builds, fatal sidecar lines, the gate timeout, and the
`<slam> <tag>` command form of slam_runner.py. Written after the run of 2026-10-18 01:53, where
ORB-SLAM2/3 could not start ('Package not found') and every run waited 2 minutes for nothing."""
import importlib.util
import tempfile
import types
import unittest
from pathlib import Path
from unittest import mock

from hilbench.kinds import slam as k

from ._fakes import ROOT

MONO_STACKS = ("unified_orbslam2_mono", "unified_orbslam3_mono", "unified_ov2slam_fast_mono", "unified_ov2slam_accurate_mono")


def load_runner():
    spec = importlib.util.spec_from_file_location("slam_runner", ROOT / "benchmarks" / "slam_runner.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


class MissingBuild(unittest.TestCase):
    def test_empty_worktree_reports_the_orbslam2_overlay(self):
        """in: missing_build_files('unified_orbslam2_mono', <empty temp dir>)
        exp: the list contains '/workspace/src/orbslam2/install_stereo/setup.bash'
        bug: a worktree without the (untracked) ORB-SLAM2 build passes preflight and every run waits for nothing."""
        with tempfile.TemporaryDirectory() as tmp:
            self.assertIn("/workspace/src/orbslam2/install_stereo/setup.bash", k.missing_build_files("unified_orbslam2_mono", tmp))

    def test_empty_worktree_reports_the_orbslam3_vocabulary(self):
        """in: missing_build_files('unified_orbslam3_mono', <empty temp dir>)
        exp: the list contains '/workspace/src/orbslam3/install/setup.bash' and the ORBvoc.txt.bin path
        bug: only the overlay is checked; a missing vocabulary file still stalls the run."""
        with tempfile.TemporaryDirectory() as tmp:
            gone = k.missing_build_files("unified_orbslam3_mono", tmp)
        self.assertIn("/workspace/src/orbslam3/install/setup.bash", gone)
        self.assertTrue(any(g.endswith("Vocabulary/ORBvoc.txt.bin") for g in gone), gone)

    def test_files_present_give_no_problem(self):
        """in: temp worktree that holds every path the orbslam2 stack names
        exp: []
        bug: the check reports a path that exists (wrong /workspace -> worktree mapping)."""
        with tempfile.TemporaryDirectory() as tmp:
            for w in k.missing_build_files("unified_orbslam2_mono", tmp):
                p = Path(tmp) / w[len("/workspace/"):]
                p.parent.mkdir(parents=True, exist_ok=True)
                p.touch()
            self.assertEqual(k.missing_build_files("unified_orbslam2_mono", tmp), [])

    def test_this_worktree_has_every_mono_build(self):
        """in: missing_build_files(<each mono stack>) on this repository
        exp: [] for all four
        bug: the SLAM builds were never copied into this worktree (the 2026-10-18 failure)."""
        for s in MONO_STACKS:
            self.assertEqual(k.missing_build_files(s, ROOT), [], s)


class SidecarFatal(unittest.TestCase):
    def _log(self, text):
        f = tempfile.NamedTemporaryFile("w", suffix=".log", delete=False)
        f.write(text)
        f.close()
        self.addCleanup(Path(f.name).unlink)
        return f.name

    def test_package_not_found_is_fatal(self):
        """in: slam_sidecar.log with the line "Package 'orbslam' not found" (repeated)
        exp: sidecar_fatal_line returns "Package 'orbslam' not found"
        bug: the SLAM-ready step waits its full time on a sidecar that can never start."""
        self.assertEqual(k.sidecar_fatal_line(self._log("Package 'orbslam' not found\n" * 3)), "Package 'orbslam' not found")

    def test_missing_shared_library_is_fatal(self):
        """in: a log line 'mono: error while loading shared libraries: libg2o.so: cannot open shared object file'
        exp: that line is returned
        bug: a missing library (wrong ld_prefix) is not seen until the timeout."""
        line = "mono: error while loading shared libraries: libg2o.so: cannot open shared object file"
        self.assertEqual(k.sidecar_fatal_line(self._log(line + "\n")), line)

    def test_normal_log_and_missing_file_are_not_fatal(self):
        """in: a log with ordinary start lines; and a path that does not exist
        exp: None for both
        bug: a slow but healthy start, or a log that is not written yet, is reported as fatal."""
        self.assertIsNone(k.sidecar_fatal_line(self._log("Loading ORB Vocabulary...\nVocabulary loaded!\n")))
        self.assertIsNone(k.sidecar_fatal_line("/nonexistent/slam_sidecar.log"))


class GateTimeout(unittest.TestCase):
    def test_bench_gets_a_gate_timeout_longer_than_the_kind_waits(self):
        """in: boot() of the SLAM kind with a fake BenchProcess that records its env
        exp: env RECORD_GATE_TIMEOUT == '440' (200 s frames + 180 s SLAM ready + 60 s)
        bug: the bench closes after its default 120 s while the kind still waits for SLAM to start."""
        seen = {}

        class FakeBench:
            def __init__(self, argv, log, parse, in_container, container, env):
                seen.update(env)
                self.gate_wait, self.run_rel, self.lines, self.error = True, "bags/x", [], None

            def alive(self):
                return True

        kind = object.__new__(k.SlamKind)
        kind.settings = types.SimpleNamespace(host="h", dds="fastrtps", ros_domain_id=0, tag="t", boot_timeout_s=5.0)
        kind.services = types.SimpleNamespace(session=types.SimpleNamespace(folder=Path(tempfile.gettempdir())),
                                              container=None, ws_root=Path(tempfile.gettempdir()))
        kind.cfg, kind.mode = {}, "mono"
        kind.arms = {"orbslam2": {"stack": {"mono": "unified_orbslam2_mono"}, "slam_type": "orbslam2", "ready_signal": "timing_csv"}}
        trial = types.SimpleNamespace(k=1, meta={}, run_dir=None, bench=None, abort_requested=lambda: False,
                                      step=lambda *a, **kw: None,
                                      gate=types.SimpleNamespace(arm=lambda: None, host_path=Path("/tmp/.gate_x_1")))
        with mock.patch.object(k, "BenchProcess", FakeBench):
            kind.boot(trial, "orbslam2")
        self.assertEqual(seen.get("RECORD_GATE_TIMEOUT"), "440")


class CommandForm(unittest.TestCase):
    def setUp(self):
        self.r = load_runner()

    def test_slam_and_tag(self):
        """in: positional_to_options(['orbslam2', 'unified_t1', '--runs', '3'])
        exp: ['--tag', 'unified_t1', '--arms', 'orbslam2', '--runs', '3']
        bug: the SLAM name or the tag is dropped or swapped."""
        self.assertEqual(self.r.positional_to_options(["orbslam2", "unified_t1", "--runs", "3"]),
                         ["--tag", "unified_t1", "--arms", "orbslam2", "--runs", "3"])

    def test_several_slams(self):
        """in: ['orbslam2,ov2_fast', 't']
        exp: ['--tag', 't', '--arms', 'orbslam2,ov2_fast']
        bug: a comma list is split into two positional words."""
        self.assertEqual(self.r.positional_to_options(["orbslam2,ov2_fast", "t"]), ["--tag", "t", "--arms", "orbslam2,ov2_fast"])

    def test_all_means_no_arms_option(self):
        """in: ['all', 't']
        exp: ['--tag', 't']
        bug: 'all' is passed on as an arm name and rejected."""
        self.assertEqual(self.r.positional_to_options(["all", "t"]), ["--tag", "t"])

    def test_old_form_unchanged(self):
        """in: ['--tag', 't', '--arms', 'orbslam3']
        exp: the same list
        bug: the documented option form stops working."""
        self.assertEqual(self.r.positional_to_options(["--tag", "t", "--arms", "orbslam3"]), ["--tag", "t", "--arms", "orbslam3"])

    def test_unknown_slam_exits_2(self):
        """in: ['orbslam9', 't']
        exp: SystemExit with code 2
        bug: a typo starts a session with no arms or a wrong arm."""
        with mock.patch("sys.stderr"):
            with self.assertRaises(SystemExit) as cm:
                self.r.positional_to_options(["orbslam9", "t"])
        self.assertEqual(cm.exception.code, 2)


if __name__ == "__main__":
    unittest.main()
