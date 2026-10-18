"""kinds/controller.py: the controller kind must behave like benchmarks/ctrl_runner.py."""
import json
import shutil
import tempfile
import types
import unittest
from pathlib import Path
from unittest import mock

import bench_geometry as geo
from hilbench import factory
from hilbench.trial import Outcome, StartSpec, TrialResult

from ._fakes import FIXTURES, kind_defaults, make_services, make_settings, no_external_commands


def kind_cls():
    return factory.kind_class("controller")


def make_kind(tmp, **cfg):
    cls = kind_cls()
    return cls(make_settings(kind="controller", kind_cfg=kind_defaults(cls, **cfg)), make_services(tmp))


class ControllerBase(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.tmp = Path(self._tmp.name)
        self.addCleanup(self._tmp.cleanup)


class Settings(ControllerBase):
    def test_registered_under_controller(self):
        """in: factory.kind_class('controller')
        exp: a RunKind subclass whose name is 'controller'
        bug: the kind module is not imported by kinds.load_all()."""
        self.assertEqual(kind_cls().name, "controller")

    def test_field_defaults(self):
        """in: the defaults of fields()
        exp: controller 'proportional', camera_pub 'off', camera_pub_hz 20, flight_s 150
        bug: a default differs from the approved protocol of 2026-10-17."""
        d = kind_defaults(kind_cls())
        self.assertEqual((d["controller"], d["camera_pub"], d["camera_pub_hz"], d["flight_s"]),
                         ("proportional", "off", 20, 150))

    def test_validate_default_is_clean(self):
        """in: validate(settings) with the default kind_cfg
        exp: []
        bug: the defaults fail their own check."""
        cls = kind_cls()
        self.assertEqual(cls.validate(make_settings(kind="controller", kind_cfg=kind_defaults(cls))), [])

    def test_validate_rejects_unknown_controller(self):
        """in: kind_cfg controller 'foo'
        exp: a problem that names 'controller' and 'foo'
        bug: a typo reaches controller_hil_bench.sh and fails after the Wi-Fi gate."""
        cls = kind_cls()
        probs = cls.validate(make_settings(kind="controller", kind_cfg=kind_defaults(cls, controller="foo")))
        self.assertTrue(any("foo" in p for p in probs), probs)

    def test_validate_accepts_the_four_controllers(self):
        """in: controller proportional, ibvs, h_vs, pbvs
        exp: no problem for each
        bug: h_vs or pbvs is missing from the allowed names."""
        cls = kind_cls()
        for name in ("proportional", "ibvs", "h_vs", "pbvs"):
            self.assertEqual(cls.validate(make_settings(kind="controller", kind_cfg=kind_defaults(cls, controller=name))),
                             [], name)

    def test_validate_rejects_bad_camera_pub(self):
        """in: camera_pub 'maybe'
        exp: a problem (directly, or the option list of the field lists only off/on/ab)
        bug: camera_pub accepts any text."""
        cls = kind_cls()
        probs = cls.validate(make_settings(kind="controller", kind_cfg=kind_defaults(cls, camera_pub="maybe")))
        spec = {f.name: f for f in cls.fields()}["camera_pub"]
        self.assertTrue(probs or (spec.options and "maybe" not in spec.options))
        self.assertEqual(sorted(spec.options or ()), ["ab", "off", "on"])

    def test_class_attributes(self):
        """in: summary_script and bench_kill_names
        exp: summary_script 'benchmarks/ctrl_session_summary.py'; kill names contain oracle_detector_node,
             hil_servo_node, visp_servo_node, h_vs_servo_node, visp_pbvs_node, ros2 bag record
        bug: a controller node survives a run and publishes /cmd_vel into the next hold."""
        cls = kind_cls()
        self.assertEqual(cls.summary_script, "benchmarks/ctrl_session_summary.py")
        for name in ("oracle_detector_node", "hil_servo_node", "visp_servo_node", "h_vs_servo_node",
                     "visp_pbvs_node", "ros2 bag record"):
            self.assertIn(name, cls.bench_kill_names)

    def test_default_start_spec(self):
        """in: kind.start_spec()
        exp: StartSpec at geo.START with yaw tolerance 5 deg
        bug: the controller kind keeps its own start pose."""
        spec = make_kind(self.tmp).start_spec()
        self.assertEqual((spec.x, spec.y, spec.z, spec.yaw_rad), geo.START)
        self.assertEqual(spec.yaw_tol_deg, 5.0)


class Conditions(ControllerBase):
    def test_ab_alternates_off_on(self):
        """in: camera_pub 'ab'
        exp: ['off', 'on']
        bug: the old ab protocol (OFF, ON, OFF, ON) is lost."""
        self.assertEqual(make_kind(self.tmp, camera_pub="ab").conditions(), ["off", "on"])

    def test_off_only(self):
        """in: camera_pub 'off'
        exp: ['off']
        bug: a single-condition session also runs the ON condition."""
        self.assertEqual(make_kind(self.tmp, camera_pub="off").conditions(), ["off"])

    def test_on_only(self):
        """in: camera_pub 'on'
        exp: ['on']
        bug: 'on' is treated as 'ab'."""
        self.assertEqual(make_kind(self.tmp, camera_pub="on").conditions(), ["on"])


class MatlabSetup(ControllerBase):
    def test_off_calls_pub_off(self):
        """in: matlab_setup(trial, 'off')
        exp: supervisor.pub_off() called once; pub_on never called
        bug: the camera publisher keeps running in the OFF condition."""
        kind = make_kind(self.tmp)
        kind.matlab_setup(mock.MagicMock(), "off")
        kind.services.supervisor.pub_off.assert_called_once_with()
        kind.services.supervisor.pub_on.assert_not_called()

    def test_on_calls_pub_on_with_rate(self):
        """in: matlab_setup(trial, 'on') with camera_pub_hz 20
        exp: supervisor.pub_on called once with 20.0
        bug: the rate is not passed and MATLAB uses its own default."""
        kind = make_kind(self.tmp)
        kind.matlab_setup(mock.MagicMock(), "on")
        kind.services.supervisor.pub_on.assert_called_once_with(20.0)
        kind.services.supervisor.pub_off.assert_not_called()

    def test_on_uses_configured_rate(self):
        """in: camera_pub_hz 10, cond 'on'
        exp: pub_on(10.0)
        bug: the 20 Hz default is hard-coded."""
        kind = make_kind(self.tmp, camera_pub_hz=10)
        kind.matlab_setup(mock.MagicMock(), "on")
        kind.services.supervisor.pub_on.assert_called_once_with(10.0)


class RunDirMixin:
    def run_dir_with(self, sequence=None, valid=True, reason="ok", metrics=True):
        d = self.tmp / "run"
        d.mkdir(exist_ok=True)
        chk = json.loads((FIXTURES / "run_check.json").read_text())
        if sequence is not None:
            chk["sequence"] = sequence
        chk["valid"], chk["reason"] = valid, reason
        (d / "run_check.json").write_text(json.dumps(chk))
        if metrics:
            shutil.copy(FIXTURES / "metrics.csv", d / "metrics.csv")
        return d

class Results(RunDirMixin, ControllerBase):
    def test_read_results_from_fixture(self):
        """in: fixtures/run_check.json (valid, APPROACHING -> REACHED, 0 gaps, rx 0.0281) + metrics.csv
        exp: valid True, sequence ['APPROACHING','REACHED'], hb_gaps 0, rx ~0.02812, settling_time_s ~17.418,
             steady_state_err_m ~0.1955, path_efficiency ~0.99275
        bug: results are read from the wrong key, or metrics.csv rows are not merged."""
        r = make_kind(self.tmp).read_results(self.run_dir_with())
        self.assertTrue(r["valid"])
        self.assertEqual(r["sequence"], ["APPROACHING", "REACHED"])
        self.assertEqual(r["hb_gaps"], 0)
        self.assertAlmostEqual(float(r["rx"]), 0.02812133, places=6)
        self.assertAlmostEqual(float(r["settling_time_s"]), 17.41825, places=4)
        self.assertAlmostEqual(float(r["steady_state_err_m"]), 0.195473, places=5)
        self.assertAlmostEqual(float(r["path_efficiency"]), 0.992751, places=5)

    def test_read_results_without_run_check(self):
        """in: a run folder with no run_check.json
        exp: valid False and the reason mentions 'run_check'
        bug: a crashed scorer counts as a valid run."""
        d = self.tmp / "empty"
        d.mkdir()
        r = make_kind(self.tmp).read_results(d)
        self.assertFalse(r["valid"])
        self.assertIn("run_check", r["reason"])

    def test_pace_metric_is_heartbeat_rate(self):
        """in: pace_metric of a result built from the fixture (/sim/heartbeat rate_hz 16.16672730214488)
        exp: 16.1667 (+-1e-4)
        bug: the pace guard reads /cmd_vel (20 Hz) instead of /sim/heartbeat."""
        kind = make_kind(self.tmp)
        res = TrialResult(Outcome.VALID, metrics=kind.read_results(self.run_dir_with()))
        self.assertAlmostEqual(kind.pace_metric(res), 16.16672730, delta=1e-4)

    def test_pace_metric_none_without_data(self):
        """in: pace_metric of a result with empty metrics
        exp: None
        bug: a missing figure becomes 0.0 and the guard stops the session."""
        self.assertIsNone(make_kind(self.tmp).pace_metric(TrialResult(Outcome.RIG_FAILED)))

    def test_table_row_shape_and_cells(self):
        """in: table_row(1, 'off', result from fixture)
        exp: len == len(columns) == 11; cells '1', 'off'; contains '17.4' (settling), '0.195' (ss err),
             '0.993' (path eff), '0.03' (rx)
        bug: the table loses a column or formats a number differently from the old runner."""
        kind = make_kind(self.tmp)
        res = TrialResult(Outcome.VALID, sequence=["APPROACHING", "REACHED"],
                          metrics=kind.read_results(self.run_dir_with()))
        row = kind.table_row(1, "off", res)
        self.assertEqual(len(kind.columns), 11)
        self.assertEqual(len(row), len(kind.columns))
        self.assertEqual(row[:2], ["1", "off"])
        for cell in ("17.4", "0.195", "0.993", "0.03"):
            self.assertIn(cell, row)


class Scoring(RunDirMixin, ControllerBase):
    def score(self, **kw):
        d = self.run_dir_with(**kw)
        kind = make_kind(self.tmp)
        trial = mock.MagicMock()
        trial.run_dir, trial.meta = d, {}
        with no_external_commands():
            return kind.score(trial)

    def test_searching_approaching_reached_is_valid(self):
        """in: FSM sequence SEARCHING, APPROACHING, REACHED
        exp: VALID
        bug: the long path (target found after a search) is rejected."""
        res = self.score(sequence=["SEARCHING", "APPROACHING", "REACHED"])
        self.assertEqual(res.outcome, Outcome.VALID)
        self.assertEqual(res.sequence, ["SEARCHING", "APPROACHING", "REACHED"])

    def test_approaching_reached_is_valid(self):
        """in: FSM sequence APPROACHING, REACHED
        exp: VALID
        bug: the usual path from the shared start is rejected."""
        self.assertEqual(self.score(sequence=["APPROACHING", "REACHED"]).outcome, Outcome.VALID)

    def test_approaching_alone_is_backend_failed(self):
        """in: FSM sequence APPROACHING (never REACHED); run_check valid false
        exp: BACKEND_FAILED
        bug: a controller that never reaches the target is counted as a good run."""
        res = self.score(sequence=["APPROACHING"], valid=False, reason="FSM never reached REACHED")
        self.assertEqual(res.outcome, Outcome.BACKEND_FAILED)
        self.assertIn("REACHED", res.reason)

    def test_missing_run_check_is_not_valid(self):
        """in: scorers produce no run_check.json
        exp: outcome is not VALID
        bug: a missing check file is read as valid."""
        d = self.tmp / "run"
        d.mkdir()
        kind = make_kind(self.tmp)
        trial = mock.MagicMock()
        trial.run_dir, trial.meta = d, {}
        with no_external_commands():
            res = kind.score(trial)
        self.assertNotEqual(res.outcome, Outcome.VALID)


class Asking(ControllerBase):
    def test_every_invalid_run_asks(self):
        """in: asks_on(BACKEND_FAILED), asks_on(RIG_FAILED), asks_on(VALID)
        exp: True, True, False (approved protocol: every invalid run asks)
        bug: the base-class default (ask only on RIG_FAILED) is inherited and a backend failure is skipped silently."""
        kind = make_kind(self.tmp)
        self.assertTrue(kind.asks_on(Outcome.BACKEND_FAILED))
        self.assertTrue(kind.asks_on(Outcome.RIG_FAILED))
        self.assertFalse(kind.asks_on(Outcome.VALID))


class ParseLine(ControllerBase):
    def test_fsm_state_line_is_stored(self):
        """in: bench output line '[INFO] [fsm]: State: SEARCHING -> APPROACHING'
        exp: the word 'APPROACHING' is stored in bench.extra (a value of the dict)
        bug: the flight panel shows no FSM state."""
        kind = make_kind(self.tmp)
        bench = types.SimpleNamespace(extra={}, lines=[], run_rel=None, gate_wait=False, recording=False, error=None)
        kind.parse_line("[INFO] [fsm]: State: SEARCHING -> APPROACHING", bench)
        self.assertIn("APPROACHING", bench.extra.values())


if __name__ == "__main__":
    unittest.main()
