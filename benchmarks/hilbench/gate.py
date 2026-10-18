"""gate.py — the record gate: the bag opens only when this file exists, so t = 0 is the same moment in every run."""
import time
from pathlib import Path


class RecordGate:
    def __init__(self, ws_root, session_name, k, n_topics):
        name = f".gate_{session_name}_{k}"
        self.host_path = Path(ws_root) / "bags" / name
        self.container_path = f"/workspace/bags/{name}"     # passed to the bench as RECORD_GATE
        self.n_topics = n_topics

    def arm(self):
        """Make bags/ if needed and remove an old gate file."""
        self.host_path.parent.mkdir(exist_ok=True)
        self.host_path.unlink(missing_ok=True)

    def open(self):
        self.host_path.touch()

    def close(self):
        self.host_path.unlink(missing_ok=True)

    def wait_recorder(self, run_dir, timeout=15.0, poll_s=0.3, on_poll=None):
        """Count 'Subscribed to topic' lines of the recorder until n_topics. Returns the count.

        The plug-in raises the failure when the count is too low. on_poll(count) is called every poll."""
        t_end, subs = time.monotonic() + timeout, 0
        rec_log = Path(run_dir) / "bag_record.log"
        while time.monotonic() < t_end:
            subs = rec_log.read_text(errors="replace").count("Subscribed to topic") if rec_log.exists() else 0
            if on_poll:
                on_poll(subs)
            if subs >= self.n_topics:
                break
            time.sleep(poll_s)
        return subs
