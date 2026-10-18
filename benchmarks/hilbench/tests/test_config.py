"""config.py: defaults, YAML, command line, the bare-off bug, validation, snapshot."""
import contextlib
import dataclasses
import hashlib
import io
import tempfile
import unittest
from pathlib import Path

from hilbench import config
from hilbench.config import COMMON_FIELDS

from ._fakes import FIXTURES, FakeKind, make_settings


class ConfigBase(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.tmp = Path(self._tmp.name)
        self.addCleanup(self._tmp.cleanup)

    def yaml(self, text, name="bench.yaml"):
        p = self.tmp / name
        p.write_text(text)
        return str(p)

    def load(self, argv, path=None):
        return config.load(argv, FakeKind, config_path=path or str(self.tmp / "absent.yaml"))

    def load_exit(self, argv, path=None):
        err = io.StringIO()
        with contextlib.redirect_stderr(err):
            with self.assertRaises(SystemExit) as cm:
                self.load(argv, path)
        return cm.exception.code, err.getvalue()


class DefaultsAndMerge(ConfigBase):
    def test_defaults_without_yaml(self):
        """in: argv ['--tag','t'], no YAML file
        exp: runs 10, cooldown 10.0, pace_guard 'stop', ratio 0.85, host 192.168.1.201, port 55556, wait 180.0,
             hold 45.0, dds fastrtps, domain 0, wifi 90.0, windows check True, boot 60.0, cores none/none,
             governor leave, buffers leave, kind 'fake', config_path None, overrides ()
        bug: a built-in default drifts from config/hil/ctrl_bench.yaml (for example runs 3 instead of 10)."""
        s = self.load(["--tag", "t"])
        self.assertEqual((s.runs, s.cooldown_s, s.pace_guard, s.pace_min_ratio), (10, 10.0, "stop", 0.85))
        self.assertEqual((s.host, s.port, s.windows_user), ("192.168.1.201", 55556, "homie"))
        self.assertEqual((s.wait_s, s.hold_timeout_s, s.boot_timeout_s), (180.0, 45.0, 60.0))
        self.assertEqual((s.dds, s.ros_domain_id, s.wifi_min_pct, s.check_windows_wifi), ("fastrtps", 0, 90.0, True))
        self.assertEqual((s.controller_cores, s.runner_cores, s.cpu_governor, s.net_buffers),
                         ("none", "none", "leave", "leave"))
        self.assertEqual((s.kind, s.tag, s.config_path, tuple(s.overrides), s.dry_run),
                         ("fake", "t", None, (), False))

    def test_kind_defaults_in_kind_cfg(self):
        """in: argv ['--tag','t'], no YAML
        exp: kind_cfg == {'mode':'a','count':3,'camera':'off','label':'x'}
        bug: kind defaults are lost, or kind fields are stored as attributes of Settings."""
        s = self.load(["--tag", "t"])
        self.assertEqual(s.kind_cfg, {"mode": "a", "count": 3, "camera": "off", "label": "x"})
        self.assertFalse(hasattr(s, "mode"))

    def test_yaml_overrides_default(self):
        """in: fixtures/sample_bench.yaml (runs 4, port 6000, dds cyclonedds, wifi 80, cores '3', fake.mode b, count 5)
        exp: those values; config_path is that file; overrides ()
        bug: the YAML file is ignored, or a section is read from the wrong place."""
        path = str(FIXTURES / "sample_bench.yaml")
        s = self.load([], path)
        self.assertEqual((s.tag, s.runs, s.cooldown_s, s.pace_guard), ("from_yaml_unified", 4, 2.0, "warn"))
        self.assertEqual((s.port, s.dds, s.wifi_min_pct, s.controller_cores), (6000, "cyclonedds", 80.0, "3"))
        self.assertEqual((s.kind_cfg["mode"], s.kind_cfg["count"]), ("b", 5))
        self.assertEqual(s.config_path, path)
        self.assertEqual(tuple(s.overrides), ())

    def test_cli_overrides_yaml_and_lands_in_overrides(self):
        """in: YAML runs 4 + argv '--runs 7 --port 7000'
        exp: runs 7, port 7000, overrides contain 'runs' and 'port', not 'dds'
        bug: the YAML wins over the command line, or overrides stays empty."""
        s = self.load(["--runs", "7", "--port", "7000"], str(FIXTURES / "sample_bench.yaml"))
        self.assertEqual((s.runs, s.port), (7, 7000))
        self.assertIn("runs", s.overrides)
        self.assertIn("port", s.overrides)
        self.assertNotIn("dds", s.overrides)

    def test_cli_equal_to_yaml_is_not_an_override(self):
        """in: YAML runs 4 + argv '--runs 4'
        exp: runs 4 and 'runs' not in overrides
        bug: overrides lists names whose command-line value equals the YAML value."""
        s = self.load(["--runs", "4"], str(FIXTURES / "sample_bench.yaml"))
        self.assertEqual(s.runs, 4)
        self.assertNotIn("runs", s.overrides)

    def test_cli_wins_over_default_without_yaml(self):
        """in: argv '--tag t --runs 2' with no YAML
        exp: runs 2
        bug: the command line is read only when a YAML file exists."""
        self.assertEqual(self.load(["--tag", "t", "--runs", "2"]).runs, 2)

    def test_cli_flag_names(self):
        """in: argv '--matlab-wait 30 --hold-timeout 20 --domain 5 --wifi-min 70 --boot-timeout 33 --dry-run'
        exp: wait_s 30.0, hold_timeout_s 20.0, ros_domain_id 5, wifi_min_pct 70.0, boot_timeout_s 33.0, dry_run True
        bug: a renamed flag (cli=) is not wired to its field, or --dry-run is lost."""
        s = self.load(["--tag", "t", "--matlab-wait", "30", "--hold-timeout", "20", "--domain", "5",
                       "--wifi-min", "70", "--boot-timeout", "33", "--dry-run"])
        self.assertEqual((s.wait_s, s.hold_timeout_s, s.ros_domain_id, s.wifi_min_pct, s.boot_timeout_s),
                         (30.0, 20.0, 5, 70.0, 33.0))
        self.assertTrue(s.dry_run)

    def test_kind_flag_overrides_kind_cfg(self):
        """in: argv '--count 9 --mode b'
        exp: kind_cfg count 9 (int), mode 'b'; 'count' in overrides
        bug: kind fields are not on the command line, or the cast is skipped (count '9' as text)."""
        s = self.load(["--tag", "t", "--count", "9", "--mode", "b"])
        self.assertEqual((s.kind_cfg["count"], s.kind_cfg["mode"]), (9, "b"))
        self.assertIsInstance(s.kind_cfg["count"], int)
        self.assertIn("count", s.overrides)

    def test_config_flag_beats_config_path_argument(self):
        """in: argv '--config <sample_bench.yaml>' and config_path=<other file with runs 99>
        exp: runs 4 (from --config)
        bug: the config_path argument wins over the --config option."""
        other = self.yaml("session:\n  tag: x\n  runs: 99\n", "other.yaml")
        s = config.load(["--config", str(FIXTURES / "sample_bench.yaml")], FakeKind, config_path=other)
        self.assertEqual(s.runs, 4)

    def test_default_yaml_path_missing_gives_none(self):
        """in: no --config, no config_path, kind 'fake' (config/hil/fake_bench.yaml does not exist)
        exp: config_path None
        bug: a missing default file crashes instead of being logged as a deviation."""
        s = config.load(["--tag", "t"], FakeKind)
        self.assertIsNone(s.config_path)

    def test_parser_has_no_defaults(self):
        """in: build_parser('fake', FakeKind.fields()).parse_args([])
        exp: every parsed value is None or False
        bug: a parser default hides 'given on the command line' (the overrides list is then wrong)."""
        ns = config.build_parser("fake", FakeKind.fields()).parse_args([])
        self.assertTrue(vars(ns))
        for key, val in vars(ns).items():
            self.assertIn(val, (None, False), key)


class BareWordBug(ConfigBase):
    def test_bare_off_common_text_field(self):
        """in: YAML 'pace_guard: off' (PyYAML reads False)
        exp: pace_guard == 'off' (text)
        bug 1: bare off becomes False and the guard mode is wrong."""
        s = self.load(["--tag", "t"], self.yaml("session:\n  pace_guard: off\n"))
        self.assertEqual(s.pace_guard, "off")

    def test_bare_off_and_on_kind_text_field(self):
        """in: YAML 'fake: {camera: off}' then '{camera: on}'
        exp: kind_cfg camera 'off' then 'on'
        bug 1: camera_pub off in the YAML becomes the boolean False."""
        a = self.load(["--tag", "t"], self.yaml("fake:\n  camera: off\n", "a.yaml"))
        b = self.load(["--tag", "t"], self.yaml("fake:\n  camera: on\n", "b.yaml"))
        self.assertEqual((a.kind_cfg["camera"], b.kind_cfg["camera"]), ("off", "on"))

    def test_bool_field_stays_bool(self):
        """in: YAML 'check_windows_wifi: false'
        exp: check_windows_wifi is False (a bool, not the text 'off')
        bug: the bool fix is applied to a bool field and turns it into text."""
        s = self.load(["--tag", "t"], self.yaml("network:\n  check_windows_wifi: false\n"))
        self.assertIs(s.check_windows_wifi, False)


class Validation(ConfigBase):
    def test_empty_tag_exits_2(self):
        """in: argv '--tag ""'
        exp: SystemExit code 2, stderr names 'tag'
        bug: the runner starts a session with an empty name."""
        code, err = self.load_exit(["--tag", ""])
        self.assertEqual(code, 2)
        self.assertIn("tag", err)

    def test_missing_tag_exits_2(self):
        """in: no --tag and no tag in the YAML
        exp: SystemExit code 2
        bug: the runner invents a tag."""
        code, _ = self.load_exit([])
        self.assertEqual(code, 2)

    def test_controller_cores_empty_string_exits_2(self):
        """in: YAML 'controller_cores: \"\"'
        exp: SystemExit 2, stderr names controller_cores
        bug: an empty value is silently read as 'no pinning'."""
        code, err = self.load_exit(["--tag", "t"], self.yaml('pi:\n  controller_cores: ""\n'))
        self.assertEqual(code, 2)
        self.assertIn("controller_cores", err)

    def test_controller_cores_null_exits_2(self):
        """in: YAML 'controller_cores:' (null)
        exp: SystemExit 2
        bug: None is turned into 'none' by a normaliser."""
        code, _ = self.load_exit(["--tag", "t"], self.yaml("pi:\n  controller_cores:\n"))
        self.assertEqual(code, 2)

    def test_core_lists_are_valid(self):
        """in: controller_cores '3', '2,3', '0-2', 'none'; runner_cores '0-2'
        exp: validate() has no problem about cores
        bug: a valid core list is rejected."""
        for spec in ("3", "2,3", "0-2", "none"):
            probs = config.validate(make_settings(controller_cores=spec, runner_cores="0-2"))
            self.assertEqual([p for p in probs if "cores" in p], [], spec)

    def test_controller_cores_problems(self):
        """in: validate() with controller_cores '', None, 'abc', '3-', 'NONE-ish'
        exp: each gives a problem that names controller_cores
        bug: validate lets a bad pin through and taskset fails later."""
        for bad in ("", None, "abc", "3-", "x,2"):
            probs = config.validate(make_settings(controller_cores=bad))
            self.assertTrue(any("controller_cores" in p for p in probs), repr(bad))

    def test_runner_cores_problems(self):
        """in: validate() with runner_cores '' and 'abc'
        exp: a problem that names runner_cores
        bug: runner_cores is not checked like controller_cores."""
        for bad in ("", "abc"):
            probs = config.validate(make_settings(runner_cores=bad))
            self.assertTrue(any("runner_cores" in p for p in probs), repr(bad))

    def test_wifi_min_pct_range(self):
        """in: validate() with wifi_min_pct -1, 101, 0, 100
        exp: problems for -1 and 101; none for 0 and 100
        bug: the range check is off by one or missing."""
        for bad in (-1.0, 101.0):
            self.assertTrue(any("wifi_min_pct" in p for p in config.validate(make_settings(wifi_min_pct=bad))), bad)
        for ok in (0.0, 100.0):
            self.assertEqual(config.validate(make_settings(wifi_min_pct=ok)), [], ok)

    def test_text_outside_options(self):
        """in: validate() with pace_guard 'bogus', dds 'zenoh', cpu_governor 'turbo', net_buffers 'big'
        exp: a problem that names each field
        bug: the options of a text field are not enforced."""
        for name, val in (("pace_guard", "bogus"), ("dds", "zenoh"), ("cpu_governor", "turbo"), ("net_buffers", "big")):
            probs = config.validate(make_settings(**{name: val}))
            self.assertTrue(any(name in p for p in probs), name)

    def test_valid_settings_have_no_problem(self):
        """in: validate(make_settings()) (all defaults, tag set)
        exp: []
        bug: a false problem blocks every session."""
        self.assertEqual(config.validate(make_settings()), [])

    def test_unknown_option_in_yaml_exits_2(self):
        """in: YAML 'network: {dds: bogus}'
        exp: SystemExit 2
        bug: an unknown option value reaches MATLAB or Docker."""
        code, err = self.load_exit(["--tag", "t"], self.yaml("network:\n  dds: bogus\n"))
        self.assertEqual(code, 2)
        self.assertIn("dds", err)

    def test_unknown_option_on_command_line_exits_2(self):
        """in: argv '--dds bogus'
        exp: SystemExit 2
        bug: the command line skips the options check."""
        code, _ = self.load_exit(["--tag", "t", "--dds", "bogus"])
        self.assertEqual(code, 2)

    def test_unknown_kind_option_exits_2(self):
        """in: argv '--mode zzz' (options a, b)
        exp: SystemExit 2
        bug: a kind field ignores its options."""
        code, _ = self.load_exit(["--tag", "t", "--mode", "zzz"])
        self.assertEqual(code, 2)

    def test_kind_problem_and_common_problem_printed_together(self):
        """in: argv '--tag \"\" --count -1' (common problem + kind problem)
        exp: exit 2; stderr holds both 'tag' and 'count must not be negative'
        bug: load stops at the first problem and the operator fixes them one at a time."""
        code, err = self.load_exit(["--tag", "", "--count", "-1"])
        self.assertEqual(code, 2)
        self.assertIn("tag", err)
        self.assertIn("count must not be negative", err)


class Snapshot(ConfigBase):
    def test_snapshot_with_yaml_has_sha256(self):
        """in: as_snapshot(load with fixtures/sample_bench.yaml)
        exp: has every common field name, kind_cfg, overrides, config_path; one value is the file's sha256
        bug: the snapshot misses a field, or the YAML hash is not stored."""
        path = FIXTURES / "sample_bench.yaml"
        snap = config.as_snapshot(self.load([], str(path)))
        for f in COMMON_FIELDS:
            self.assertIn(f.name, snap)
        for key in ("kind_cfg", "overrides", "config_path"):
            self.assertIn(key, snap)
        self.assertEqual(snap["kind_cfg"]["mode"], "b")
        digest = hashlib.sha256(path.read_bytes()).hexdigest()
        self.assertIn(digest, [v for v in snap.values() if isinstance(v, str)])

    def test_snapshot_without_yaml_has_no_hash(self):
        """in: as_snapshot(load with no YAML file)
        exp: config_path None; the sha256 entry is None
        bug: a hash of an empty or missing file is invented."""
        snap = config.as_snapshot(self.load(["--tag", "t"]))
        self.assertIsNone(snap["config_path"])
        sha = [v for k, v in snap.items() if "sha" in k.lower()]
        self.assertEqual(sha, [None])

    def test_snapshot_is_json_ready(self):
        """in: json.dumps(as_snapshot(...), default=None)
        exp: no TypeError (only str, numbers, bool, None, list, dict)
        bug: the snapshot holds a tuple of non-JSON objects and session_settings.json cannot be written."""
        import json
        snap = config.as_snapshot(self.load([], str(FIXTURES / "sample_bench.yaml")))
        json.dumps(snap)


if __name__ == "__main__":
    unittest.main()
