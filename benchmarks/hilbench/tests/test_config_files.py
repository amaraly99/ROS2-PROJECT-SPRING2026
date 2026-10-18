"""The shipped YAML files follow CONTRACT.md sections 2 to 4."""
import unittest

import yaml

from ._fakes import ROOT

CFG = ROOT / "config" / "hil"
TEXT_KEYS = {"session": ("pace_guard",), "network": ("dds",),
             "pi": ("controller_cores", "runner_cores", "cpu_governor", "net_buffers")}
SLAM_TYPES = {"orbslam2", "orbslam3", "ov2slam", "rtabmap"}
READY = {"timing_csv", "handshake_ack"}


def load(name):
    return yaml.safe_load((CFG / name).read_text())


class SettingsFiles(unittest.TestCase):
    def check_file(self, name, kind_section):
        data = load(name)
        for section in ("session", "matlab", "network", "pi", kind_section):
            self.assertIn(section, data, f"{name}: section {section}")
        for section, keys in TEXT_KEYS.items():
            for key in keys:
                if key in data[section]:
                    self.assertIsInstance(data[section][key], str, f"{name}: {section}.{key} must be a quoted word")
        self.assertIsInstance(data["session"]["tag"], str)
        return data

    def test_controller_bench_yaml(self):
        """in: config/hil/controller_bench.yaml
        exp: sections session, matlab, network, pi, controller; word values are text (quoted); camera_pub is text
        bug 1: an unquoted off becomes the boolean False when the file is read."""
        data = self.check_file("controller_bench.yaml", "controller")
        self.assertIsInstance(data["controller"]["camera_pub"], str)

    def test_slam_bench_yaml(self):
        """in: config/hil/slam_bench.yaml
        exp: sections session, matlab, network, pi, slam; word values are text
        bug 1: an unquoted word becomes a boolean."""
        self.check_file("slam_bench.yaml", "slam")

    def test_old_controller_yaml_still_there(self):
        """in: config/hil/ctrl_bench.yaml
        exp: the file exists (the running campaign keeps it)
        bug: the new work deletes or renames a file of the running campaign."""
        self.assertTrue((CFG / "ctrl_bench.yaml").exists())


class ModelsRegistry(unittest.TestCase):
    def test_models_yaml_layout(self):
        """in: config/hil/models.yaml
        exp: models.mono and models.rgbd have file + depth_port; models.stereo has the five baselines
             0.11 0.36 0.42 0.54 0.61, each with right_cam on and right_y == -baseline; all files under matlab/ and .slx
        bug: the registry layout differs from CONTRACT section 3 and model_check cannot read it."""
        models = load("models.yaml")["models"]
        self.assertTrue(models["mono"]["file"].startswith("matlab/") and models["mono"]["file"].endswith(".slx"))
        self.assertEqual(models["rgbd"]["depth_port"], "on")
        self.assertEqual(models["mono"]["depth_port"], "off")
        self.assertEqual(sorted(str(k) for k in models["stereo"]), ["0.11", "0.36", "0.42", "0.54", "0.61"])
        for key, entry in models["stereo"].items():
            self.assertAlmostEqual(entry["right_y"], -float(key))
            self.assertEqual(entry["right_cam"], "on")
            self.assertTrue(entry["file"].endswith(".slx"))


class ArmFiles(unittest.TestCase):
    def arms(self):
        files = sorted((CFG / "slam_arms").glob("*.yaml"))
        self.assertTrue(files, "no arm file in config/hil/slam_arms/")
        return files

    def test_arm_file_layout(self):
        """in: every config/hil/slam_arms/*.yaml
        exp: name equals the file stem; slam_type in orbslam2/orbslam3/ov2slam/rtabmap; ready_signal in
             timing_csv/handshake_ack; init_patience_s a number; stack a non-empty dict of mono/stereo/rgbd -> text
        bug: an arm file does not follow CONTRACT section 4 and the kind drops it or crashes."""
        for f in self.arms():
            a = yaml.safe_load(f.read_text())
            self.assertEqual(a["name"], f.stem, f.name)
            self.assertIn(a["slam_type"], SLAM_TYPES, f.name)
            self.assertIn(a["ready_signal"], READY, f.name)
            self.assertIsInstance(a["init_patience_s"], (int, float), f.name)
            self.assertTrue(a["stack"], f.name)
            for mode, stack in a["stack"].items():
                self.assertIn(mode, ("mono", "stereo", "rgbd"), f.name)
                self.assertIsInstance(stack, str, f.name)

    def test_stack_gates(self):
        """in: the stack file named by every arm
        exp: exists; init_gate.enabled false; slam.startup_delay_sec 2; network.dds fastrtps;
             network.matlab_host_ip 192.168.1.201
        bug: an arm is flown with the init gate on, so the probe misses its init leg (the old 'zero poses' fault)."""
        for f in self.arms():
            a = yaml.safe_load(f.read_text())
            for mode, stack in a["stack"].items():
                p = CFG / "stack" / f"{stack}.yaml"
                self.assertTrue(p.exists(), f"{f.name}: {p} missing")
                s = yaml.safe_load(p.read_text())
                self.assertIs(s["init_gate"]["enabled"], False, p.name)
                self.assertEqual(s["slam"]["startup_delay_sec"], 2, p.name)
                self.assertEqual(s["network"]["dds"], "fastrtps", p.name)
                self.assertEqual(str(s["network"]["matlab_host_ip"]), "192.168.1.201", p.name)


if __name__ == "__main__":
    unittest.main()
