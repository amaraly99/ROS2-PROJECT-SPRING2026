"""gate.py: the record gate file and the recorder subscription count."""
import shutil
import tempfile
import unittest
from pathlib import Path

from hilbench.gate import RecordGate

from ._fakes import FIXTURES


class GateTests(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.tmp = Path(self._tmp.name)
        self.addCleanup(self._tmp.cleanup)
        self.run_dir = self.tmp / "bags" / "run1"
        self.run_dir.mkdir(parents=True)

    def gate(self, n=7):
        return RecordGate(self.tmp, "sess", 3, n)

    def test_paths(self):
        """in: RecordGate(ws_root, 'sess', k=3, n_topics=7)
        exp: host_path == ws_root/bags/.gate_sess_3; container_path == '/workspace/bags/.gate_sess_3'
        bug: the bench script waits for a file name the runner never creates."""
        g = self.gate()
        self.assertEqual(Path(g.host_path), self.tmp / "bags" / ".gate_sess_3")
        self.assertEqual(str(g.container_path), "/workspace/bags/.gate_sess_3")

    def test_arm_open_close(self):
        """in: arm(), open(), close() in turn, with an old gate file present before arm()
        exp: after arm no file; after open the file exists; after close no file
        bug: a stale gate file from a crashed run lets the recorder start at once."""
        g = self.gate()
        Path(g.host_path).parent.mkdir(exist_ok=True)
        Path(g.host_path).touch()
        g.arm()
        self.assertFalse(Path(g.host_path).exists())
        g.open()
        self.assertTrue(Path(g.host_path).exists())
        g.close()
        self.assertFalse(Path(g.host_path).exists())

    def test_arm_creates_bags_folder(self):
        """in: arm() when ws_root/bags does not exist
        exp: the folder exists after arm()
        bug: open() fails with FileNotFoundError on a fresh checkout."""
        g = self.gate()
        shutil.rmtree(self.tmp / "bags")
        g.arm()
        self.assertTrue((self.tmp / "bags").is_dir())

    def test_close_without_file_is_quiet(self):
        """in: close() when no gate file exists
        exp: no exception
        bug: cleanup after a failed boot raises FileNotFoundError."""
        self.gate().close()

    def test_wait_recorder_counts_seven(self):
        """in: bag_record.log with 7 'Subscribed to topic' lines (real shape, fixtures/bag_record.log)
        exp: returns 7 at once
        bug: the count includes 'Overriding subscription profile' lines (it would give more than 7)."""
        shutil.copy(FIXTURES / "bag_record.log", self.run_dir / "bag_record.log")
        self.assertEqual(self.gate(7).wait_recorder(self.run_dir, timeout=2.0, poll_s=0.05), 7)

    def test_wait_recorder_timeout_returns_count(self):
        """in: log with 3 'Subscribed to topic' lines, n_topics 7, timeout 0.4 s
        exp: returns 3 (the kind raises RigFailed itself)
        bug: wait_recorder raises, or returns 0 / 7 when only 3 topics are subscribed."""
        lines = "".join(f"[INFO] [rosbag2_recorder]: Subscribed to topic '/t{i}'\n" for i in range(3))
        (self.run_dir / "bag_record.log").write_text("[WARN] noise\n" + lines)
        self.assertEqual(self.gate(7).wait_recorder(self.run_dir, timeout=0.4, poll_s=0.05), 3)

    def test_wait_recorder_without_log_returns_zero(self):
        """in: no bag_record.log, timeout 0.3 s
        exp: 0
        bug: a missing log file raises."""
        self.assertEqual(self.gate(7).wait_recorder(self.run_dir, timeout=0.3, poll_s=0.05), 0)

    def test_wait_recorder_sees_lines_that_arrive_late(self):
        """in: the log gets its 7 lines 0.3 s after the call starts (written by a thread)
        exp: returns 7 within the timeout
        bug: the log is read once instead of polled."""
        import threading
        import time

        def later():
            time.sleep(0.3)
            shutil.copy(FIXTURES / "bag_record.log", self.run_dir / "bag_record.log")
        t = threading.Thread(target=later)
        t.start()
        self.assertEqual(self.gate(7).wait_recorder(self.run_dir, timeout=3.0, poll_s=0.05), 7)
        t.join()

    def test_wait_recorder_honours_n_topics(self):
        """in: n_topics 5, log with 7 lines
        exp: returns a number >= 5 at once
        bug: the threshold 7 is hard-coded; a kind with more topics would pass too early."""
        shutil.copy(FIXTURES / "bag_record.log", self.run_dir / "bag_record.log")
        self.assertGreaterEqual(self.gate(5).wait_recorder(self.run_dir, timeout=1.0, poll_s=0.05), 5)


if __name__ == "__main__":
    unittest.main()
