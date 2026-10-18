"""pi_state.py — what the Pi does to itself, and what it records about itself.

With every setting at its default ('none' / 'leave') nothing is changed; the state is still recorded.
"""
import os
import shutil
from pathlib import Path

from .util import run

HIL_NET_BUFFERS = {   # the values run_stack_hil.sh sets for the MATLAB -> Pi stream
    "net.core.rmem_max": 16777216, "net.core.rmem_default": 16777216,
    "net.core.wmem_max": 16777216, "net.core.wmem_default": 16777216,
    "net.ipv4.ipfrag_high_thresh": 33554432, "net.ipv4.ipfrag_low_thresh": 25165824,
    "net.ipv4.ipfrag_time": 3,
}


def parse_cores(spec):
    out = set()
    for part in str(spec).split(","):
        a, _, b = part.partition("-")
        out.update(range(int(a), int(b or a) + 1))
    return out


def governors():
    return {p.parent.parent.name: p.read_text().strip()
            for p in sorted(Path("/sys/devices/system/cpu").glob("cpu[0-9]*/cpufreq/scaling_governor"))}


def net_buffers():
    return {k: Path("/proc/sys/" + k.replace(".", "/")).read_text().strip() for k in HIL_NET_BUFFERS}


def _meminfo():
    """/proc/meminfo as {name: kB}."""
    out = {}
    try:
        for line in Path("/proc/meminfo").read_text().splitlines():
            name, _, rest = line.partition(":")
            out[name] = int(rest.split()[0])
    except (OSError, ValueError, IndexError):
        pass
    return out


def mem_available_mb():
    """MemAvailable in MB, or None."""
    v = _meminfo().get("MemAvailable")
    return None if v is None else round(v / 1024, 1)


def pi_state():
    """CPU temperature, the firmware throttle flags (vcgencmd get_throttled) and memory (MB)."""
    st = {}
    try:
        st["cpu_temp_c"] = round(int(Path("/sys/class/thermal/thermal_zone0/temp").read_text()) / 1000, 1)
    except (OSError, ValueError):
        st["cpu_temp_c"] = None
    p = run(["vcgencmd", "get_throttled"], timeout=5) if shutil.which("vcgencmd") else None
    st["throttled"] = p.stdout.strip().split("=")[-1] if p and p.returncode == 0 else None
    mi = _meminfo()

    def mb(key):
        return round(mi[key] / 1024, 1) if key in mi else None
    st["mem_total_mb"] = mb("MemTotal")
    st["mem_available_mb"] = mb("MemAvailable")
    st["swap_used_mb"] = round((mi["SwapTotal"] - mi["SwapFree"]) / 1024, 1) if "SwapTotal" in mi and "SwapFree" in mi else None
    return st


def _pids_of(exe_name):
    """Process ids of a live process (container processes are visible from the host)."""
    if not exe_name:
        return []
    return run(["pgrep", "-x", exe_name[:15]], timeout=5).stdout.split()


def _status_field(pid, name):
    try:
        for line in Path(f"/proc/{pid}/status").read_text().splitlines():
            if line.startswith(name + ":"):
                return line.split(":", 1)[1].strip()
    except OSError:
        pass
    return None


def affinity_of(exe_name):
    """Cpus_allowed_list of the live process, or 'not found'."""
    for pid in _pids_of(exe_name):
        v = _status_field(pid, "Cpus_allowed_list")
        if v is not None:
            return v
    return "not found"


def process_memory(exe_name):
    """{'rss_mb', 'hwm_mb'} of the live process (VmRSS now, VmHWM = peak kept by the kernel), or None."""
    for pid in _pids_of(exe_name):
        rss, hwm = _status_field(pid, "VmRSS"), _status_field(pid, "VmHWM")
        if rss is not None and hwm is not None:
            try:
                return {"rss_mb": round(int(rss.split()[0]) / 1024, 1), "hwm_mb": round(int(hwm.split()[0]) / 1024, 1)}
            except (ValueError, IndexError):
                continue
    return None


class PiSettings:
    """Applies the Pi settings of a session and puts them back at the end."""

    def __init__(self, settings):
        self.s = settings
        self._governor_before = None
        self._net_before = None
        self.applied = {}

    def apply(self):
        s, applied = self.s, {}
        if s.runner_cores != "none":
            os.sched_setaffinity(0, parse_cores(s.runner_cores))     # this process; its children inherit it
            applied["runner_cores"] = sorted(os.sched_getaffinity(0))
        if s.cpu_governor == "performance":
            self._governor_before = governors()
            for cpu in self._governor_before:
                run(["sudo", "-n", "sh", "-c", f"echo performance > /sys/devices/system/cpu/{cpu}/cpufreq/scaling_governor"],
                    timeout=10, check=True)
            applied["cpu_governor"] = governors()
        if s.net_buffers == "hil":
            self._net_before = net_buffers()
            run(["sudo", "-n", "sysctl", "-q", *[f"{k}={v}" for k, v in HIL_NET_BUFFERS.items()]], timeout=10, check=True)
            applied["net_buffers"] = net_buffers()
        self.applied = applied
        return applied

    def restore(self):
        """Write the remembered values back. Returns what is set now (for the log)."""
        now = {}
        if self._governor_before:
            for cpu, gov in self._governor_before.items():
                run(["sudo", "-n", "sh", "-c", f"echo {gov} > /sys/devices/system/cpu/{cpu}/cpufreq/scaling_governor"], timeout=10)
            now["cpu_governor"] = governors()
            self._governor_before = None
        if self._net_before:
            run(["sudo", "-n", "sysctl", "-q", *[f"{k}={v}" for k, v in self._net_before.items()]], timeout=10)
            now["net_buffers"] = net_buffers()
            self._net_before = None
        return now

    def snapshot(self):
        """The Pi state at the start of the session."""
        return {**pi_state(), "cpu_governor": governors(), "net_buffers": net_buffers()}
