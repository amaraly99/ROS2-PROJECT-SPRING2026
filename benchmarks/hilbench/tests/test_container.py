"""container.py: BenchProcess default parse rules (host-side process, no Docker)."""
import tempfile
import time
import unittest
from pathlib import Path

from hilbench.container import BenchProcess

from ._fakes import no_external_commands


class BenchProcessTests(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.tmp = Path(self._tmp.name)
        self.addCleanup(self._tmp.cleanup)
        self.seen = []

    def parse(self, line, bench):
        # record what the default rules have already set when the kind's parser runs
        self.seen.append((line, bench.run_rel, bench.gate_wait))

    def feed(self, text, wait_for=None):
        """Run `printf` on the host with the given text; wait until the reader has finished."""
        log = self.tmp / "bench.log"
        with no_external_commands() as ext:
            bp = BenchProcess(["printf", "%s", text], log, self.parse, in_container=False)
            deadline = time.monotonic() + 5
            while time.monotonic() < deadline:
                if not bp.alive() and (wait_for is None or wait_for(bp)):
                    break
                time.sleep(0.05)
            time.sleep(0.1)
        self.assertEqual(ext.blocked, [])
        return bp, log

    def test_run_dir_line(self):
        """in: output line 'Run dir: bags/ctrl_proportional_N1_20261017_154029'
        exp: bench.run_rel == 'bags/ctrl_proportional_N1_20261017_154029'
        bug: the run folder is not found and the kind cannot read the results."""
        bp, _ = self.feed("noise\nRun dir: bags/ctrl_proportional_N1_20261017_154029\n", lambda b: b.run_rel)
        self.assertEqual(bp.run_rel, "bags/ctrl_proportional_N1_20261017_154029")

    def test_gate_wait_line(self):
        """in: output line '[bench] waiting for record gate: /workspace/bags/.gate_s_1'
        exp: gate_wait True
        bug: the boot step never sees the bench ready and times out."""
        bp, _ = self.feed("[bench] waiting for record gate: /workspace/bags/.gate_s_1\n", lambda b: b.gate_wait)
        self.assertTrue(bp.gate_wait)

    def test_recording_line(self):
        """in: output line '[bench] recording started'
        exp: recording True
        bug: only a line that starts exactly with '[bench] recording' counts, so a later prefix is missed."""
        bp, _ = self.feed("[bench] recording started\n", lambda b: b.recording)
        self.assertTrue(bp.recording)

    def test_recording_needs_line_start(self):
        """in: output line 'info: [bench] recording x' (the tag is not at the line start)
        exp: recording stays False
        bug: any line that contains the words sets recording."""
        bp, _ = self.feed("info: [bench] recording x\n")
        self.assertFalse(bp.recording)

    def test_error_line(self):
        """in: output lines '[bench] ERROR: no bag' and '[bench] ERROR: second'
        exp: error == '[bench] ERROR: no bag' (the first one)
        bug: the last error hides the first cause."""
        bp, _ = self.feed("[bench] ERROR: no bag\n[bench] ERROR: second\n", lambda b: b.error)
        time.sleep(0.2)
        self.assertEqual(bp.error, "[bench] ERROR: no bag")

    def test_error_needs_bench_prefix(self):
        """in: output line 'ERROR: something else' (no [bench] prefix)
        exp: error is None
        bug: noise from other tools is reported as a bench error."""
        bp, _ = self.feed("ERROR: something else\n")
        self.assertIsNone(bp.error)

    def test_lines_log_and_kind_parser(self):
        """in: three lines 'a', 'Run dir: bags/x', 'c'
        exp: log file holds the 3 lines; bp.lines holds them; the kind's parse_line is called 3 times and
             already sees run_rel set on line 2 (default rules run first)
        bug: lines are lost from the log, or the kind's parser runs before the default rules."""
        bp, log = self.feed("a\nRun dir: bags/x\nc\n", lambda b: len(b.lines) >= 3)
        self.assertEqual(log.read_text().splitlines(), ["a", "Run dir: bags/x", "c"])
        self.assertEqual(list(bp.lines), ["a", "Run dir: bags/x", "c"])
        self.assertEqual([s[0] for s in self.seen], ["a", "Run dir: bags/x", "c"])
        self.assertEqual(self.seen[1][1], "bags/x")

    def test_extra_is_a_dict_and_alive_ends(self):
        """in: a process that prints one line and exits
        exp: bp.extra == {} (a dict the kind may fill); alive() is False after exit
        bug: the kind's parser has no place to store state, or alive() stays True."""
        bp, _ = self.feed("x\n")
        self.assertEqual(bp.extra, {})
        self.assertFalse(bp.alive())

    def test_stop_kills_a_long_process_on_the_host(self):
        """in: a 'sleep 30' process, stop(timeout=1)
        exp: alive() False within 6 s; no docker or sudo command started
        bug: stop() waits for a docker pkill even for a host process, or leaves the process running."""
        log = self.tmp / "sleep.log"
        with no_external_commands() as ext:
            bp = BenchProcess(["sleep", "30"], log, lambda l, b: None, in_container=False)
            t0 = time.monotonic()
            bp.stop(timeout=1)
        self.assertFalse(bp.alive())
        self.assertLess(time.monotonic() - t0, 8)
        self.assertEqual(ext.blocked, [])


if __name__ == "__main__":
    unittest.main()
