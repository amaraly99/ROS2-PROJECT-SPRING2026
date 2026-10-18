"""kinds/slam.py: settings, conditions (arms), model key, asks, tracking parse, pose count."""
import contextlib
import shutil
import tempfile
import types
import unittest
from pathlib import Path
from unittest import mock

import yaml
from hilbench import factory
from hilbench.trial import BackendFailed, Outcome, TrialResult

from ._fakes import FIXTURES, ROOT, kind_defaults, make_services, make_settings, no_external_commands


def kind_cls():
    return factory.kind_class("slam")


def settings_for(**cfg):
    cls = kind_cls()
    return make_settings(kind="slam", kind_cfg=kind_defaults(cls, **cfg))


def make_kind(tmp, ws_root=None, **cfg):
    sv = make_services(tmp)
    if ws_root is not None:
        sv.ws_root = Path(ws_root)
    return kind_cls()(settings_for(**cfg), sv)


def problems(**cfg):
    return kind_cls().validate(settings_for(**cfg))


def find(obj, names):
    """First attribute of obj that exists, from a list of possible names (None if none)."""
    for n in names:
        if hasattr(obj, n):
            return getattr(obj, n)
    return None


class Base(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.tmp = Path(self._tmp.name)
        self.addCleanup(self._tmp.cleanup)


class Validate(Base):
    def test_registered_under_slam(self):
        """in: factory.kind_class('slam')
        exp: a RunKind subclass named 'slam'
        bug: the kind module is not imported by kinds.load_all()."""
        self.assertEqual(kind_cls().name, "slam")

    def test_defaults(self):
        """in: defaults of fields()
        exp: vx 2.0, cycles 2, wavelength_m 8.0, yaw_amp_deg 0, left_image 'rgb', right_image 'rgb', min_poses 20,
             probe_timeout_s 400, end_pad 3
        bug: a default differs from the approved cruise (2 cycles of 8.0 m) or the old run_matrix values."""
        d = kind_defaults(kind_cls())
        self.assertEqual((d["vx"], d["cycles"], d["wavelength_m"], d["yaw_amp_deg"]), (2.0, 2, 8.0, 0))
        self.assertEqual((d["left_image"], d["right_image"], d["min_poses"]), ("rgb", "rgb", 20))
        self.assertEqual((d["probe_timeout_s"], d["end_pad"]), (400, 3))

    def test_camera_mode_is_required_with_three_options(self):
        """in: the FieldSpec camera_mode
        exp: required True; options mono, stereo, rgbd
        bug: a session can start without a camera mode."""
        spec = {f.name: f for f in kind_cls().fields()}["camera_mode"]
        self.assertTrue(spec.required)
        self.assertEqual(sorted(spec.options), ["mono", "rgbd", "stereo"])

    def test_valid_mono(self):
        """in: camera_mode mono, no baseline
        exp: []
        bug: the simplest valid session is rejected."""
        self.assertEqual(problems(camera_mode="mono"), [])

    def test_valid_stereo_036(self):
        """in: camera_mode stereo, baseline 0.36
        exp: []
        bug: a registered baseline is rejected."""
        self.assertEqual(problems(camera_mode="stereo", stereo_baseline_m=0.36), [])

    def test_valid_rgbd(self):
        """in: camera_mode rgbd
        exp: []
        bug: rgbd needs a baseline."""
        self.assertEqual(problems(camera_mode="rgbd"), [])

    def test_camera_mode_missing(self):
        """in: camera_mode None
        exp: a problem that names camera_mode
        bug: a session without a mode starts and picks a model by chance."""
        self.assertTrue(any("camera_mode" in p for p in problems(camera_mode=None)))

    def test_camera_mode_unknown(self):
        """in: camera_mode 'depth'
        exp: a problem that names camera_mode
        bug: any text is accepted as a mode."""
        self.assertTrue(any("camera_mode" in p for p in problems(camera_mode="depth")))

    def test_stereo_without_baseline(self):
        """in: camera_mode stereo, stereo_baseline_m None
        exp: a problem that names stereo_baseline_m
        bug: the stereo model is not chosen and model_check cannot compare."""
        self.assertTrue(any("stereo_baseline_m" in p for p in problems(camera_mode="stereo")))

    def test_baseline_with_mono(self):
        """in: camera_mode mono, stereo_baseline_m 0.36
        exp: a problem that names stereo_baseline_m
        bug: a stray baseline is ignored and the operator thinks it was used."""
        self.assertTrue(any("stereo_baseline_m" in p
                            for p in problems(camera_mode="mono", stereo_baseline_m=0.36)))

    def test_baseline_not_registered(self):
        """in: camera_mode stereo, baseline 0.20 (allowed: 0.11, 0.36, 0.42, 0.54, 0.61)
        exp: a problem that names stereo_baseline_m
        bug: a baseline without a model file passes."""
        self.assertTrue(any("stereo_baseline_m" in p
                            for p in problems(camera_mode="stereo", stereo_baseline_m=0.20)))

    def test_all_five_baselines_valid(self):
        """in: stereo with 0.11, 0.36, 0.42, 0.54, 0.61
        exp: [] for each
        bug: one of the five registered baselines is missing from the list."""
        for b in (0.11, 0.36, 0.42, 0.54, 0.61):
            self.assertEqual(problems(camera_mode="stereo", stereo_baseline_m=b), [], b)

    def test_fractional_cycles(self):
        """in: cycles 2.5
        exp: a problem that names cycles
        bug: the probe gets half a cycle and the sinusoids end with a velocity step."""
        self.assertTrue(any("cycles" in p for p in problems(camera_mode="mono", cycles=2.5)))

    def test_zero_cycles(self):
        """in: cycles 0
        exp: a problem that names cycles
        bug: a flight without a cruise leg is accepted."""
        self.assertTrue(any("cycles" in p for p in problems(camera_mode="mono", cycles=0)))


class Conditions(Base):
    def ws_with_arms(self):
        ws = self.tmp / "ws"
        shutil.copytree(FIXTURES / "slam_arms", ws / "config" / "hil" / "slam_arms")
        return ws

    @contextlib.contextmanager
    def arms_from(self, ws):
        """The kind must read arm files under services.ws_root. If kinds/slam.py keeps its own ARMS_DIR
        constant (built from the real repo), point that constant at the temp tree for the test."""
        import hilbench.kinds.slam as mod
        if hasattr(mod, "ARMS_DIR"):
            with mock.patch.object(mod, "ARMS_DIR", Path(ws) / "config" / "hil" / "slam_arms"):
                yield
        else:
            yield

    def conditions(self, **cfg):
        ws = self.ws_with_arms()
        with self.arms_from(ws):
            return make_kind(self.tmp, ws_root=ws, **cfg).conditions()

    def test_arms_filtered_by_camera_mode_mono(self):
        """in: arm files arm_both (mono, stereo), arm_mono (mono), arm_stereo (stereo); camera_mode mono
        exp: conditions() == ['arm_both', 'arm_mono'] (any order)
        bug: an arm that does not support the session camera mode is flown."""
        self.assertEqual(sorted(self.conditions(camera_mode="mono")), ["arm_both", "arm_mono"])

    def test_arms_filtered_by_camera_mode_stereo(self):
        """in: the same three arm files; camera_mode stereo, baseline 0.36
        exp: ['arm_both', 'arm_stereo']
        bug: the filter uses the file name instead of the keys of `stack`."""
        self.assertEqual(sorted(self.conditions(camera_mode="stereo", stereo_baseline_m=0.36)),
                         ["arm_both", "arm_stereo"])

    def test_no_arm_supports_rgbd(self):
        """in: the three arm files; camera_mode rgbd
        exp: [] (the session then has nothing to fly; preflight must stop it)
        bug: the filter returns every arm when none matches."""
        self.assertEqual(self.conditions(camera_mode="rgbd"), [])

    def test_real_arms_directory(self):
        """in: the real config/hil/slam_arms/*.yaml with the repo as ws_root, camera_mode mono, then stereo 0.36
        exp: conditions() equals the arms whose file has 'mono' (or 'stereo') among the keys of `stack` (computed here
             with yaml); at least one arm each
        bug: arm files in the repo are not read, or the filter disagrees with the file layout of CONTRACT section 4."""
        arms_dir = ROOT / "config" / "hil" / "slam_arms"
        loaded = [yaml.safe_load(p.read_text()) for p in sorted(arms_dir.glob("*.yaml"))]
        self.assertTrue(loaded, "no arm file in config/hil/slam_arms/")
        for mode, kw in (("mono", {}), ("stereo", {"stereo_baseline_m": 0.36})):
            expected = sorted(a["name"] for a in loaded if mode in a["stack"])
            self.assertTrue(expected, mode)
            kind = make_kind(self.tmp, ws_root=ROOT, camera_mode=mode, **kw)
            self.assertEqual(sorted(kind.conditions()), expected, mode)

    def test_arm_list_in_settings_restricts_arms(self):
        """in: arms ['arm_mono'] with camera_mode mono and the three arm files
        exp: ['arm_mono']
        bug: the `arms` setting is ignored."""
        self.assertEqual(list(self.conditions(camera_mode="mono", arms=["arm_mono"])), ["arm_mono"])


class ModelKeyAndAsks(Base):
    def test_model_key_stereo(self):
        """in: camera_mode stereo, baseline 0.36
        exp: ('stereo', 0.36)
        bug: the baseline is dropped and model_check compares against the wrong model."""
        self.assertEqual(make_kind(self.tmp, camera_mode="stereo", stereo_baseline_m=0.36).model_key(),
                         ("stereo", 0.36))

    def test_model_key_mono_and_rgbd(self):
        """in: camera_mode mono, then rgbd
        exp: ('mono', None) and ('rgbd', None)
        bug: a baseline is attached to a mono or rgbd model key."""
        self.assertEqual(make_kind(self.tmp, camera_mode="mono").model_key(), ("mono", None))
        self.assertEqual(make_kind(self.tmp, camera_mode="rgbd").model_key(), ("rgbd", None))

    def test_asks_on(self):
        """in: asks_on(BACKEND_FAILED), asks_on(RIG_FAILED), asks_on(VALID)
        exp: False, True, False
        bug: a SLAM backend that never initialises stops the session and hides the number."""
        kind = make_kind(self.tmp, camera_mode="mono")
        self.assertFalse(kind.asks_on(Outcome.BACKEND_FAILED))
        self.assertTrue(kind.asks_on(Outcome.RIG_FAILED))
        self.assertFalse(kind.asks_on(Outcome.VALID))

    def test_default_start_is_shared_start(self):
        """in: start_spec()
        exp: the shared start (15.12, 3.84, 7.5), yaw tolerance 5 deg
        bug: the SLAM kind uses a start of its own."""
        spec = make_kind(self.tmp, camera_mode="mono").start_spec()
        self.assertEqual((spec.x, spec.y, spec.z), (15.12, 3.84, 7.5))
        self.assertEqual(spec.yaw_tol_deg, 5.0)

    def test_class_attributes(self):
        """in: summary_script, columns
        exp: summary_script 'benchmarks/slam_session_summary.py'; first two columns 'run' and 'arm'
        bug: the session summary calls the controller script."""
        cls = kind_cls()
        self.assertEqual(cls.summary_script, "benchmarks/slam_session_summary.py")
        self.assertEqual(tuple(cls.columns[:2]), ("run", "arm"))


def completed(stdout="", returncode=0):
    return types.SimpleNamespace(stdout=stdout, stderr="", returncode=returncode)


class TrackingParse(Base):
    """The old bug: `ros2 topic echo --once` prints '2' then '---'; stripping non-digits gave '2---'.

    The kind reads the topic through services.container.exec_cmd; the fake returns the text.
    """

    def read(self, stdout, patience=0):
        kind = make_kind(self.tmp, camera_mode="mono")
        kind.services.container.exec_cmd.return_value = completed(stdout)
        trial = mock.MagicMock()
        trial.abort_requested.return_value = False
        return kind.check_tracking(trial, {"init_patience_s": patience})

    def test_state_two_with_yaml_separator(self):
        """in: the echo prints '2\\n---\\n'
        exp: check_tracking returns 2 (an int)
        bug: the old '2---' bug: the dashes stay in the value and a healthy backend fails."""
        self.assertEqual(self.read("2\n---\n"), 2)

    def test_empty_text_is_no_answer(self):
        """in: the echo prints '' (its window closed before a message), patience 0 s
        exp: None (no answer, not 'not initialised')
        bug: an empty read is counted as tracking state 0 and a healthy ORB-SLAM3 fails."""
        self.assertIsNone(self.read(""))

    def test_other_states(self):
        """in: the echo prints '1\\n---\\n', then '0\\n---\\n' (patience 0 s)
        exp: 1 and 0 (a real 'not ready' value is returned as it is)
        bug: only state 2 is understood and a real 'not ready' reading is lost."""
        self.assertEqual(self.read("1\n---\n"), 1)
        self.assertEqual(self.read("0\n---\n"), 0)

    def test_noise_only(self):
        """in: the echo prints 'WARNING: topic does not appear to be published yet' (no digits)
        exp: None
        bug: a warning is read as a state."""
        self.assertIsNone(self.read("WARNING: topic does not appear to be published yet\n"))

    def test_retries_empty_answer_inside_patience(self):
        """in: first read '' then '2\\n---\\n'; patience 20 s
        exp: returns 2 after a second read
        bug: one empty read fails a healthy backend (it failed ORB-SLAM3 once in four)."""
        kind = make_kind(self.tmp, camera_mode="mono")
        kind.services.container.exec_cmd.side_effect = [completed(""), completed("2\n---\n")]
        trial = mock.MagicMock()
        trial.abort_requested.return_value = False
        with mock.patch("time.sleep"):
            self.assertEqual(kind.check_tracking(trial, {"init_patience_s": 20}), 2)
        self.assertEqual(kind.services.container.exec_cmd.call_count, 2)


class FlyWithFakes(Base):
    """kind.fly() with a fake supervisor (probe protocol), a fake container and a fake arrival script."""

    def setUp(self):
        super().setUp()
        self.run_dir = self.tmp / "bags" / "run1"
        (self.run_dir / "bag").mkdir(parents=True)
        (self.run_dir / "bag" / "metadata.yaml").write_text("x: 1\n")        # the bag counts as closed

    def make(self, tracking_stdout="2\n---\n"):
        kind = make_kind(self.tmp, camera_mode="mono", min_poses=20)
        kind.arms = {"a": {"init_patience_s": 0}}
        sup = kind.services.supervisor
        sup.status_calls = 0
        sup.status.return_value = {"sim": "running", "t": 1.0, "x": 20.0, "y": 3.0, "z": 7.0, "yaw": 0.0}

        def send(cmd):
            self.sent.append(cmd)
            if cmd.startswith("probe_status"):
                return ("probe_status state=done invalid=0 maxtime_hit=0 dist_to_target_m=3.0 "
                        "end_x=32.1 end_y=2.9 end_z=3.1")
            return "ok"
        self.sent = []
        sup.send.side_effect = send
        kind.services.container.exec_cmd.return_value = completed(tracking_stdout)
        trial = mock.MagicMock()
        trial.run_dir, trial.meta, trial.state = self.run_dir, {}, {}
        trial.abort_requested.return_value = False
        return kind, trial

    def fly(self, kind, trial, poses="25\n"):
        proc = mock.MagicMock()
        proc.poll.return_value = 0
        proc.communicate.return_value = ("ARRIVED\n", None)
        with mock.patch("subprocess.Popen", return_value=proc), \
                mock.patch("subprocess.run", return_value=completed(poses)), \
                mock.patch("time.sleep"):
            return kind.fly(trial, "a")

    def test_tracking_not_two_is_backend_failed_and_sim_stopped(self):
        """in: probe init done, /slam/tracking_state reads 1
        exp: BackendFailed whose reason says 'did not initialise'; supervisor.stop() called; probe_mission never sent
        bug: a backend that never initialises is flown on, scored over a short path, or counted as a rig fault."""
        kind, trial = self.make(tracking_stdout="1\n---\n")
        with self.assertRaises(BackendFailed) as cm:
            self.fly(kind, trial)
        self.assertIn("did not initialise", cm.exception.reason)
        kind.services.supervisor.stop.assert_called()
        self.assertFalse([c for c in self.sent if c.startswith("probe_mission")])

    def test_probe_commands_carry_the_cruise_settings(self):
        """in: a good flight with the default settings (vx 2.0, cycles 2, wavelength 8.0, yaw amp 0)
        exp: the supervisor got 'probe_init' and then 'probe_mission', each with the numbers 2, 2, 8, 0
        bug: the approved cruise (2 cycles of 8.0 m) is not what MATLAB receives."""
        kind, trial = self.make()
        self.fly(kind, trial)
        cmds = [c for c in self.sent if c.startswith(("probe_init", "probe_mission"))]
        self.assertEqual([c.split()[0] for c in cmds], ["probe_init", "probe_mission"])
        for c in cmds:
            self.assertEqual([float(x) for x in c.split()[1:]], [2.0, 2.0, 8.0, 0.0])

    def test_fewer_than_min_poses_is_backend_failed(self):
        """in: tracking 2, probe done, count_poses prints 5; min_poses 20
        exp: BackendFailed whose reason mentions 'pose'
        bug: a trial with no map is recorded as a valid run with a bad score."""
        kind, trial = self.make()
        with self.assertRaises(BackendFailed) as cm:
            self.fly(kind, trial, poses="5\n")
        self.assertIn("pose", cm.exception.reason.lower())

    def test_enough_poses_completes(self):
        """in: tracking 2, probe done, count_poses prints 25; min_poses 20
        exp: fly() returns without an exception
        bug: the pose comparison is reversed or off by one at 20."""
        kind, trial = self.make()
        self.fly(kind, trial, poses="25\n")

    def test_exactly_min_poses_completes(self):
        """in: count_poses prints 20; min_poses 20
        exp: no exception (20 is enough: 'fewer than min_poses' fails)
        bug: the comparison uses <= and rejects exactly 20."""
        kind, trial = self.make()
        self.fly(kind, trial, poses="20\n")


class SlamScoring(Base):
    def test_pace_metric_is_heartbeat(self):
        """in: pace_metric of a result with metrics {'heartbeat_hz': 15.5}
        exp: 15.5
        bug: the SLAM kind has no heartbeat figure, so the pace guard is silently off."""
        kind = make_kind(self.tmp, camera_mode="mono")
        res = TrialResult(Outcome.VALID, metrics={"heartbeat_hz": 15.5})
        self.assertEqual(kind.pace_metric(res), 15.5)


if __name__ == "__main__":
    unittest.main()
