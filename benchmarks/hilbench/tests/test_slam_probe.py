"""kinds/slam_probe.py: the probe commands, built on Supervisor.send()."""
import inspect
import unittest

import hilbench.kinds.slam_probe as sp
from hilbench.trial import RigFailed


class FakeSend:
    """Stands in for the Supervisor: records commands, answers from a list (last answer repeats)."""

    def __init__(self, replies):
        self.replies, self.sent = list(replies), []
        self.status_calls = 0

    def send(self, cmd):
        self.sent.append(cmd)
        return self.replies.pop(0) if len(self.replies) > 1 else self.replies[0]


def probe_class():
    for name, obj in vars(sp).items():
        if inspect.isclass(obj) and obj.__module__ == sp.__name__ and "probe" in name.lower():
            return obj
    raise AssertionError("kinds/slam_probe.py needs a class with 'Probe' in its name (contract question)")


def make(replies, vx=2.0, cycles=2, lam=8.0, yaw=0.0):
    """Design card: Probe(sup).probe(phase, vx, cycles, lam, yaw). Built version: ProbeClient(sup, vx, ...).init()."""
    sup = FakeSend(replies)
    cls = probe_class()
    try:
        return cls(sup), sup
    except TypeError:
        return cls(sup, vx, cycles, lam, yaw), sup


def start(obj, phase, vx, cycles, lam, yaw):
    if hasattr(obj, "probe"):
        return obj.probe(phase, vx, cycles, lam, yaw)
    return getattr(obj, "init" if phase == "init" else "mission")()


def wait(obj, timeout_s):
    fn = getattr(obj, "wait", None) or getattr(obj, "wait_done")
    return fn(timeout_s)


RUNNING = "probe_status state=running invalid=0 maxtime_hit=0 dist_to_target_m=nan end_x=nan end_y=nan end_z=nan"
DONE_OK = "probe_status state=done invalid=0 maxtime_hit=0 dist_to_target_m=3.01 end_x=32.1 end_y=2.9 end_z=3.1"
DONE_BAD = "probe_status state=done invalid=1 maxtime_hit=0 dist_to_target_m=nan end_x=nan end_y=nan end_z=nan"


class ProbeCommands(unittest.TestCase):
    def test_init_command_text(self):
        """in: probe('init', 2.0, 2, 8.0, 0.0)
        exp: one send() with the word 'probe_init' and four numbers in order 2, 2, 8, 0 (read back as floats)
        bug: the numbers are swapped (cycles and wavelength) and MATLAB flies the default cruise."""
        p, sup = make(["ok"])
        start(p, "init", 2.0, 2, 8.0, 0.0)
        parts = sup.sent[0].split()
        self.assertEqual(parts[0], "probe_init")
        self.assertEqual([float(x) for x in parts[1:]], [2.0, 2.0, 8.0, 0.0])

    def test_mission_command_text(self):
        """in: probe('mission', 2.0, 2, 8.0, 5.0)
        exp: 'probe_mission' with 2, 2, 8, 5
        bug: the phase word is ignored and init is sent twice."""
        p, sup = make(["ok"], yaw=5.0)
        start(p, "mission", 2.0, 2, 8.0, 5.0)
        parts = sup.sent[0].split()
        self.assertEqual(parts[0], "probe_mission")
        self.assertEqual([float(x) for x in parts[1:]], [2.0, 2.0, 8.0, 5.0])

    def test_unknown_phase(self):
        """in: probe('fly', 2.0, 2, 8.0, 0.0)
        exp: ValueError (or RigFailed), and nothing is sent
        bug: a typo in the phase sends a command MATLAB does not know."""
        p, sup = make(["ok"])
        if not hasattr(p, "probe"):
            self.skipTest("this API has init() and mission(), no phase argument")
        with self.assertRaises((ValueError, RigFailed)):
            p.probe("fly", 2.0, 2, 8.0, 0.0)
        self.assertEqual(sup.sent, [])


class ProbeStatus(unittest.TestCase):
    def test_status_running(self):
        """in: reply 'probe_status state=running invalid=0 ...'
        exp: status() -> dict with state 'running'; sent command is 'probe_status'
        bug: the status prefix 'probe_status' breaks the parser."""
        p, sup = make([RUNNING])
        st = p.status()
        self.assertEqual(sup.sent, ["probe_status"])
        self.assertEqual(st["state"], "running")

    def test_status_done_numbers(self):
        """in: reply 'state=done invalid=0 dist_to_target_m=3.01 end_x=32.1'
        exp: state 'done'; dist_to_target_m 3.01 and end_x 32.1 as floats
        bug: numbers stay text."""
        p, _ = make([DONE_OK])
        st = p.status()
        self.assertEqual(st["state"], "done")
        self.assertAlmostEqual(float(st["dist_to_target_m"]), 3.01)
        self.assertAlmostEqual(float(st["end_x"]), 32.1)


class ProbeWait(unittest.TestCase):
    def test_wait_returns_when_done(self):
        """in: replies running, running, done (invalid=0)
        exp: wait(20) returns a dict with state 'done' and dist_to_target_m 3.01
        bug: wait returns on the first 'running' answer."""
        p, sup = make([RUNNING, RUNNING, DONE_OK])
        st = wait(p, 20)
        self.assertEqual(st["state"], "done")
        self.assertAlmostEqual(float(st["dist_to_target_m"]), 3.01)
        self.assertGreaterEqual(len(sup.sent), 3)

    def test_wait_invalid_raises_rig_failed(self):
        """in: reply state=done invalid=1 (the simulation froze)
        exp: RigFailed
        bug: a frozen probe is scored as a SLAM result."""
        p, _ = make([RUNNING, DONE_BAD])
        with self.assertRaises(RigFailed):
            wait(p, 20)

    def test_wait_timeout_raises_rig_failed(self):
        """in: state stays running; wait(0.5)
        exp: RigFailed
        bug: wait never ends and the session hangs."""
        p, _ = make([RUNNING])
        with self.assertRaises(RigFailed):
            wait(p, 0.5)


class WideViewCommands(unittest.TestCase):
    def test_wide_init_sends_six_numbers(self):
        """in: ProbeClient(sup, 2.0, 1, 13.0, 0.0, 80, 1).init()
        exp: 'probe_init 2 1 13 0 80 1'
        bug: the camera offset or the init axis never reaches MATLAB, and it flies the plain forward view."""
        sup = FakeSend(["ok"])
        sp.ProbeClient(sup, 2.0, 1, 13.0, 0.0, 80, 1).init()
        self.assertEqual(sup.sent, ["probe_init 2 1 13 0 80 1"])

    def test_wide_turn_command(self):
        """in: ProbeClient(sup, 2.0, 1, 13.0, 0.0, 80, 1).turn()
        exp: 'probe_turn 2 1 13 0 80 1'
        bug: the turn uses another word or drops the offset."""
        sup = FakeSend(["ok"])
        sp.ProbeClient(sup, 2.0, 1, 13.0, 0.0, 80, 1).turn()
        self.assertEqual(sup.sent, ["probe_turn 2 1 13 0 80 1"])

    def test_plain_probe_keeps_four_numbers_and_refuses_turn(self):
        """in: ProbeClient(sup, 2.0, 2, 8.0, 0.0).mission(), then .turn()
        exp: 'probe_mission 2 2 8 0'; turn() raises ValueError and sends nothing more
        bug: the old probe gets 6 numbers (an old supervisor answers bad_args), or a turn runs without an offset."""
        sup = FakeSend(["ok"])
        p = sp.ProbeClient(sup, 2.0, 2, 8.0, 0.0)
        p.mission()
        with self.assertRaises(ValueError):
            p.turn()
        self.assertEqual(sup.sent, ["probe_mission 2 2 8 0"])


if __name__ == "__main__":
    unittest.main()
