"""overhead.py — two meters. OverheadMeter: what the runner itself costs. PaceGuard: does the simulation slow down."""
import math
import resource
import time
from pathlib import Path


def mem_available_mb():
    """MemAvailable of the Pi in MB, or None."""
    try:
        for line in Path("/proc/meminfo").read_text().splitlines():
            if line.startswith("MemAvailable:"):
                return round(int(line.split()[1]) / 1024, 1)
    except (OSError, ValueError, IndexError):
        pass
    return None


def _own_status_mb(name):
    """VmRSS / VmHWM of this process, MB."""
    try:
        for line in Path("/proc/self/status").read_text().splitlines():
            if line.startswith(name + ":"):
                return round(int(line.split()[1]) / 1024, 1)
    except (OSError, ValueError, IndexError):
        pass
    return None


class OverheadMeter:
    """What the runner itself costs during one flight: its own CPU (this process + the finished
    children it started, e.g. `iw`), in % of one core; the whole Pi's CPU from /proc/stat, in %
    of all cores; the number of 'status' calls sent to MATLAB; its memory; and the lowest free
    memory of the Pi. The CPU of the process under test is measured elsewhere and never includes the runner."""

    @staticmethod
    def _own_cpu_s():
        s, c = resource.getrusage(resource.RUSAGE_SELF), resource.getrusage(resource.RUSAGE_CHILDREN)
        return s.ru_utime + s.ru_stime + c.ru_utime + c.ru_stime

    @staticmethod
    def _proc_stat():
        v = [int(x) for x in open("/proc/stat").readline().split()[1:]]
        return sum(v), v[3] + v[4]          # total jiffies, idle + iowait

    def __init__(self, status_counter=None):
        self.status_counter = status_counter
        self.min_mem_avail_mb = None
        self.t0, self.cpu0, self.stat0 = time.monotonic(), self._own_cpu_s(), self._proc_stat()

    def sample(self):
        """Call about once per second: keeps the lowest MemAvailable seen."""
        m = mem_available_mb()
        if m is not None and (self.min_mem_avail_mb is None or m < self.min_mem_avail_mb):
            self.min_mem_avail_mb = m

    def result(self):
        if self.min_mem_avail_mb is None:
            self.sample()
        wall = max(time.monotonic() - self.t0, 1e-6)
        total, idle = self._proc_stat()
        d_total, d_idle = total - self.stat0[0], idle - self.stat0[1]
        calls = self.status_counter() if self.status_counter else 0
        return {"flight_wall_s": round(wall, 2),
                "runner_cpu_pct_core": round(100.0 * (self._own_cpu_s() - self.cpu0) / wall, 2),
                "runner_status_calls": calls,
                "pi_cpu_total_pct": round(100.0 * (1.0 - d_idle / d_total), 2) if d_total > 0 else float("nan"),
                "runner_rss_mb": _own_status_mb("VmRSS"),
                "runner_hwm_mb": _own_status_mb("VmHWM"),
                "pi_mem_available_min_mb": self.min_mem_avail_mb}


class PaceGuard:
    """Does the simulation slow down? Run 1 sets the baseline heartbeat rate; a later run
    below min_ratio x baseline is 'degraded'. A missing rate (None, nan) is recorded and never degrades."""

    def __init__(self, mode="stop", min_ratio=0.85):
        self.mode, self.min_ratio = mode, min_ratio
        self.baseline_hz = None
        self.history = []                  # (run, hb_hz, ratio)

    def record(self, k, hb_hz):
        ok = hb_hz is not None and math.isfinite(hb_hz) and hb_hz > 0
        if not ok:
            self.history.append((k, hb_hz, None))
            return
        if self.baseline_hz is None:
            self.baseline_hz = hb_hz
        self.history.append((k, hb_hz, hb_hz / self.baseline_hz))

    def check(self):
        """('ok' | 'degraded', text) for the last recorded run. Mode off is always ok."""
        if self.mode == "off" or not self.history:
            return "ok", ""
        k, hz, ratio = self.history[-1]
        if ratio is None:
            return "ok", f"run {k}: no heartbeat rate recorded"
        txt = f"run {k}: heartbeat {hz:.1f} Hz is {ratio:.2f} of run 1 ({self.baseline_hz:.1f} Hz); limit {self.min_ratio:.2f}"
        return ("ok" if ratio >= self.min_ratio else "degraded"), txt
