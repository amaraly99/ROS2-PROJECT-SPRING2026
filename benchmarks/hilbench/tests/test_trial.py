"""trial.py: the ladder, the outcome mapping, the start check, named()."""
import tempfile
import types
import unittest
from pathlib import Path

from hilbench.trial import BackendFailed, Outcome, RigFailed, Trial

from ._fakes import (START_STATUS, FakeKind, FakeScreen, bearing, make_services, make_settings,
                     no_external_commands)


class TrialBase(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.tmp = Path(self._tmp.name)
        self.addCleanup(self._tmp.cleanup)
        self.run_dir = self.tmp / "bags" / "run1"
        self.run_dir.mkdir(parents=True)

    def make(self, script=None, hold_status=None, links_ok=True):
        self.services = make_services(self.tmp, hold_status)
        self.kind = FakeKind(make_settings(), self.services, run_dir=self.run_dir)
        self.kind.script = dict(script or {})
        self.screen = self.services.screen
        self.keys = types.SimpleNamespace(abort=False, pause=False)
        self.trial = Trial(k=1, n_total=1, cond="A", settings=make_settings(), services=self.services,
                           kind=self.kind, screen=self.screen, keys=self.keys)
        self.links_ok = links_ok
        return self.trial

    def run_trial(self, **kw):
        trial = self.make(**kw)
        with no_external_commands(self.links_ok) as ext:
            res = trial.run()
        self.blocked = ext.blocked
        return res


class OutcomeMapping(TrialBase):
    def test_success_runs_the_ladder_in_order(self):
        """in: a fake kind that raises nothing
        exp: VALID; kind calls == [matlab_setup, boot, go, fly, stop, score]; supervisor.hold called once
        bug: a step is skipped or runs in another order."""
        res = self.run_trial()
        self.assertEqual(res.outcome, Outcome.VALID)
        self.assertEqual(self.kind.calls, ["matlab_setup", "boot", "go", "fly", "stop", "score"])
        self.services.supervisor.hold.assert_called_once()

    def test_backend_failed_in_fly(self):
        """in: kind.fly raises BackendFailed('SLAM did not initialise')
        exp: BACKEND_FAILED; reason contains 'did not initialise'; kind.stop was called
        bug: a failed system under test is counted as a rig fault, or the bench is left running."""
        res = self.run_trial(script={"fly": BackendFailed("SLAM did not initialise")})
        self.assertEqual(res.outcome, Outcome.BACKEND_FAILED)
        self.assertIn("did not initialise", res.reason)
        self.assertIn("stop", self.kind.calls)

    def test_rig_failed_in_boot(self):
        """in: kind.boot raises RigFailed('boot', 'no gate')
        exp: RIG_FAILED; reason starts with 'boot' and contains 'no gate'; go, fly never called
        bug: a rig fault is not reported with its step name, or the ladder goes on."""
        res = self.run_trial(script={"boot": RigFailed("boot", "no gate")})
        self.assertEqual(res.outcome, Outcome.RIG_FAILED)
        self.assertTrue(res.reason.startswith("boot"), res.reason)
        self.assertIn("no gate", res.reason)
        self.assertNotIn("go", self.kind.calls)
        self.assertNotIn("fly", self.kind.calls)

    def test_value_error_in_go_becomes_rig_failed_with_step_name(self):
        """in: kind.go raises ValueError('boom')
        exp: RIG_FAILED; reason starts with 'go' and contains 'ValueError' and 'boom'
        bug: an unexpected exception kills the session instead of failing one run."""
        res = self.run_trial(script={"go": ValueError("boom")})
        self.assertEqual(res.outcome, Outcome.RIG_FAILED)
        self.assertTrue(res.reason.startswith("go"), res.reason)
        self.assertIn("ValueError", res.reason)
        self.assertIn("boom", res.reason)

    def test_stop_called_after_failure_in_go(self):
        """in: kind.go raises ValueError after boot succeeded
        exp: kind.stop called exactly once
        bug: the bench and the sampler keep running into the next run."""
        self.run_trial(script={"go": ValueError("boom")})
        self.assertEqual(self.kind.calls.count("stop"), 1)

    def test_stop_called_after_rig_failure_in_fly(self):
        """in: kind.fly raises RigFailed('fly', 'sim froze')
        exp: RIG_FAILED; kind.stop called once; score not called
        bug: a rig failure during the flight skips the cleanup."""
        res = self.run_trial(script={"fly": RigFailed("fly", "sim froze")})
        self.assertEqual(res.outcome, Outcome.RIG_FAILED)
        self.assertEqual(self.kind.calls.count("stop"), 1)
        self.assertNotIn("score", self.kind.calls)

    def test_hold_failure_is_rig_failed(self):
        """in: supervisor.hold raises RigFailed('start_hold', 'frozen')
        exp: RIG_FAILED; reason starts with 'start_hold'; boot never called
        bug: the bench boots although the simulation is not held."""
        trial = self.make()
        self.services.supervisor.hold.side_effect = RigFailed("start_hold", "frozen")
        with no_external_commands():
            res = trial.run()
        self.assertEqual(res.outcome, Outcome.RIG_FAILED)
        self.assertTrue(res.reason.startswith("start_hold"), res.reason)
        self.assertNotIn("boot", self.kind.calls)

    def test_wifi_failure_is_rig_failed_before_any_matlab_call(self):
        """in: net.check_links reports ok=False
        exp: RIG_FAILED; reason starts with 'wi' (wifi / Wi-Fi); matlab_setup never called
        bug: a run starts on a weak link and its pace is not comparable."""
        res = self.run_trial(links_ok=False)
        self.assertEqual(res.outcome, Outcome.RIG_FAILED)
        self.assertTrue(res.reason.lower().startswith("wi"), res.reason)
        self.assertNotIn("matlab_setup", self.kind.calls)

    def test_no_docker_sudo_ssh_in_these_tests(self):
        """in: a full successful trial with subprocess.run blocked
        exp: the list of blocked dangerous commands holds only chown (or nothing)
        bug: the ladder calls docker, ssh or sudo for something other than the run folder owner."""
        self.run_trial()
        for cmd in self.blocked:
            self.assertIn("chown", cmd)

    def test_failed_step_is_shown_as_fail_on_the_screen(self):
        """in: kind.go raises ValueError
        exp: screen.step got ('go','run') and then ('go','fail')
        bug: the operator sees no failed step."""
        self.run_trial(script={"go": ValueError("boom")})
        events = [(n, s) for n, s, _ in self.screen.steps]
        self.assertIn(("go", "run"), events)
        self.assertIn(("go", "fail"), events)

    def test_meta_txt_gets_stop_facts_and_start_pose(self):
        """in: a successful trial; kind.stop returns {'stopfact': 1}
        exp: run_dir/meta.txt has 'stopfact=1' and 'start_x=15.12'
        bug: facts of the run are lost, or the start pose is not recorded."""
        self.run_trial()
        text = (self.run_dir / "meta.txt").read_text()
        self.assertIn("stopfact=1", text)
        self.assertIn("start_x=15.12", text)


class StartCheck(TrialBase):
    def status_with_yaw_error(self, err):
        st = dict(START_STATUS)
        st["yaw"] = (bearing(st["x"], st["y"]) + err) % 360.0
        return st

    def test_ten_degrees_off_fails(self):
        """in: MATLAB pose yaw = bearing + 10 deg
        exp: RIG_FAILED; reason starts with 'start check'; boot never called
        bug: a drone that does not face the sign is flown."""
        res = self.run_trial(hold_status=self.status_with_yaw_error(10.0))
        self.assertEqual(res.outcome, Outcome.RIG_FAILED)
        self.assertTrue(res.reason.startswith("start check"), res.reason)
        self.assertNotIn("boot", self.kind.calls)

    def test_zero_error_passes(self):
        """in: MATLAB pose yaw = bearing
        exp: VALID; boot called
        bug: the check rejects a correct start."""
        res = self.run_trial(hold_status=self.status_with_yaw_error(0.0))
        self.assertEqual(res.outcome, Outcome.VALID)
        self.assertIn("boot", self.kind.calls)

    def test_error_inside_tolerance_passes_and_wraps_over_360(self):
        """in: yaw 4 deg left of the bearing, and 4 deg right (the bearing is 357.36, so 361.36 wraps to 1.36)
        exp: both pass (limit is 5 deg and the difference wraps)
        bug: angle difference is not wrapped: 1.36 vs 357.36 deg is read as 356 deg."""
        self.assertEqual(self.run_trial(hold_status=self.status_with_yaw_error(-4.0)).outcome, Outcome.VALID)
        self.assertEqual(self.run_trial(hold_status=self.status_with_yaw_error(4.0)).outcome, Outcome.VALID)

    def test_missing_pose_in_status_fails(self):
        """in: hold() status without x, y, yaw
        exp: RIG_FAILED with reason starting 'start check'
        bug: a MATLAB without the new pose fields passes the check silently."""
        res = self.run_trial(hold_status={"state": "held", "t": 0.25})
        self.assertEqual(res.outcome, Outcome.RIG_FAILED)
        self.assertTrue(res.reason.startswith("start check"), res.reason)

    def test_kind_start_spec_changes_the_check(self):
        """in: kind.start_spec() returns the same pose with yaw_tol_deg 20 and the yaw is 10 deg off
        exp: VALID
        bug: the 5 deg tolerance is hard-coded instead of read from StartSpec."""
        from hilbench.trial import StartSpec
        trial = self.make(hold_status=self.status_with_yaw_error(10.0))
        base = self.kind.start_spec()
        self.kind.start_spec = lambda: StartSpec(base.x, base.y, base.z, base.yaw_rad, yaw_tol_deg=20.0)
        with no_external_commands():
            self.assertEqual(trial.run().outcome, Outcome.VALID)


class NamedStep(TrialBase):
    def test_named_success_marks_done(self):
        """in: with trial.named('x'): pass
        exp: screen steps ('x','run') then ('x','done')
        bug: a finished step stays 'run' on the screen."""
        trial = self.make()
        with trial.named("x"):
            pass
        self.assertEqual([(n, s) for n, s, _ in self.screen.steps], [("x", "run"), ("x", "done")])

    def test_named_other_exception_becomes_rig_failed(self):
        """in: with trial.named('x'): raise ValueError('boom')
        exp: RigFailed with step 'x'; text has 'ValueError' and 'boom'
        bug: the raw exception escapes and the run has no step name."""
        trial = self.make()
        with self.assertRaises(RigFailed) as cm:
            with trial.named("x"):
                raise ValueError("boom")
        self.assertEqual(cm.exception.step, "x")
        self.assertIn("ValueError", str(cm.exception))
        self.assertIn("boom", str(cm.exception))

    def test_named_rig_failed_passes_through(self):
        """in: with trial.named('x'): raise RigFailed('y', 'why')
        exp: the same exception object, step 'y' unchanged
        bug: the inner step name is overwritten by the outer name."""
        trial = self.make()
        err = RigFailed("y", "why")
        with self.assertRaises(RigFailed) as cm:
            with trial.named("x"):
                raise err
        self.assertIs(cm.exception, err)

    def test_named_backend_failed_passes_through(self):
        """in: with trial.named('x'): raise BackendFailed('no map')
        exp: BackendFailed with reason 'no map'
        bug: a backend failure is converted into a rig failure."""
        trial = self.make()
        with self.assertRaises(BackendFailed) as cm:
            with trial.named("x"):
                raise BackendFailed("no map")
        self.assertEqual(cm.exception.reason, "no map")

    def test_named_failure_marks_step_fail(self):
        """in: with trial.named('x'): raise ValueError
        exp: screen steps end with ('x','fail')
        bug: the screen shows a failed step as running forever."""
        trial = self.make()
        with self.assertRaises(RigFailed):
            with trial.named("x"):
                raise ValueError("boom")
        self.assertEqual(self.screen.steps[-1][:2], ("x", "fail"))


class ExceptionTypes(unittest.TestCase):
    def test_rig_failed_fields(self):
        """in: RigFailed('boot', 'no gate')
        exp: step 'boot', why 'no gate', str 'boot: no gate'
        bug: the step name is lost from the message."""
        e = RigFailed("boot", "no gate")
        self.assertEqual((e.step, e.why, str(e)), ("boot", "no gate", "boot: no gate"))

    def test_trial_result_valid_property(self):
        """in: TrialResult(VALID), TrialResult(BACKEND_FAILED), TrialResult(RIG_FAILED)
        exp: valid True, False, False
        bug: a backend failure counts as valid in the mean."""
        from hilbench.trial import TrialResult
        self.assertTrue(TrialResult(Outcome.VALID).valid)
        self.assertFalse(TrialResult(Outcome.BACKEND_FAILED).valid)
        self.assertFalse(TrialResult(Outcome.RIG_FAILED).valid)


if __name__ == "__main__":
    unittest.main()
