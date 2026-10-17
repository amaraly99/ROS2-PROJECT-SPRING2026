#!/usr/bin/env python3
"""ctrl_runner.py — automated N-run HIL controller benchmark, live in a rich terminal.

    ./run_controller_benchmark.sh                          # everything from config/hil/ctrl_bench.yaml
    ./run_controller_benchmark.sh --runs 10 --camera-pub off   # a command-line option wins over the file
    ./run_controller_benchmark.sh --dry-run                # preflight + status only
Guide: benchmarks/CONTROLLER_BENCHMARK.md

Protocol (approved by Amar Aly, 2026-10-17; see docs/controller-benchmark-experiment-setup.html):
  preflight  Wi-Fi name + live strength (iw on the Pi, netsh on Windows) >= --wifi-min %,
             MATLAB supervisor answers (else: "paste hil_session_start", poll 2 s for exactly 180 s),
             container from this worktree, git SHA, disk.
  per run    camera publisher off|on -> start_hold -> held -> start check (yaw within 5 deg of the
             bearing to the target) -> boot the bench (RECORD_GATE) -> open gate -> recorder
             subscribed -> resume (stopwatch t=0) -> flight --flight s -> stop -> cool-down ->
             score (plot_controller_hil.py) -> check (ctrl_run_check.py).
  validity   FSM after resume: SEARCHING->APPROACHING->REACHED or APPROACHING->REACHED. Otherwise
             not valid. Never repeated automatically: the runner asks (n = next run, a = abort).
Keys during a session: p = pause after this run, a = abort, l = show/hide the bench log.
"""
import argparse
import csv
import datetime as dt
import math
import os
import re
import resource
import select
import shutil
import subprocess
import sys
import termios
import threading
import time
import tty
from collections import deque
from pathlib import Path

from rich.console import Console, Group
from rich.live import Live
from rich.panel import Panel
from rich.progress_bar import ProgressBar
from rich.prompt import Prompt
from rich.table import Table
from rich.text import Text

sys.path.insert(0, str(Path(__file__).resolve().parent))
import hil_supervisor_client as sup  # noqa: E402

WS = Path(__file__).resolve().parent.parent
CONTAINER = "ros2_perception_stack"
TARGET = (35.1, 2.92, 3.08)                  # frozen stop sign, SIM_PARAMS.md
START = (15.12, 3.84, 7.5)                    # shared start, SIM_PARAMS.md
# Stop distance the scorer uses (pinhole camera: fy * target height / box height at stop).
# Imported, not copied, so the runner and the scorer can never disagree.
from bench_geometry import STANDOFF  # noqa: E402
SCORER_PY = os.environ.get("SCORER_PYTHON", "python3")   # needs rosbags + numpy + matplotlib
SPARK = "▁▂▃▄▅▆▇█"
console = Console()


# ─────────────────────────── small helpers ───────────────────────────
def now_ns():
    return time.time_ns()


def run(cmd, timeout=30, check=False):
    p = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
    if check and p.returncode != 0:
        raise RuntimeError(f"{' '.join(cmd)} -> {p.returncode}: {p.stderr.strip()[:300]}")
    return p


def pct_from_dbm(dbm):
    """Windows WLAN_SIGNAL_QUALITY scale: -100 dBm -> 0 %, -50 dBm and better -> 100 %."""
    return max(0.0, min(100.0, 2.0 * (dbm + 100.0)))


def bearing_deg(x, y):
    return math.degrees(math.atan2(TARGET[1] - y, TARGET[0] - x)) % 360.0


def angle_diff(a, b):
    return abs((a - b + 180.0) % 360.0 - 180.0)


def spark(values, lo=None, hi=None):
    if not values:
        return ""
    lo = min(values) if lo is None else lo
    hi = max(values) if hi is None else hi
    span = (hi - lo) or 1.0
    return "".join(SPARK[min(7, max(0, int((v - lo) / span * 7.999)))] for v in values)


# ─────────────────────────── runner overhead ───────────────────────────
class OverheadMeter:
    """What the runner itself costs during one flight (approved by Amar Aly, 2026-10-17):
    its own CPU (this process + the finished children it started, e.g. `iw`), in % of
    one core; the whole Pi's CPU from /proc/stat, in % of all cores; and the number of
    'status' calls sent to MATLAB. The controller's own CPU (cpu.csv) is measured per
    process inside the container and never includes the runner."""

    @staticmethod
    def _own_cpu_s():
        s, c = resource.getrusage(resource.RUSAGE_SELF), resource.getrusage(resource.RUSAGE_CHILDREN)
        return s.ru_utime + s.ru_stime + c.ru_utime + c.ru_stime

    @staticmethod
    def _proc_stat():
        v = [int(x) for x in open("/proc/stat").readline().split()[1:]]
        return sum(v), v[3] + v[4]          # total jiffies, idle + iowait

    def __init__(self):
        self.status_calls = 0
        self.t0, self.cpu0, self.stat0 = time.monotonic(), self._own_cpu_s(), self._proc_stat()

    def result(self):
        wall = max(time.monotonic() - self.t0, 1e-6)
        total, idle = self._proc_stat()
        d_total, d_idle = total - self.stat0[0], idle - self.stat0[1]
        return {"flight_wall_s": round(wall, 2),
                "runner_cpu_pct_core": round(100.0 * (self._own_cpu_s() - self.cpu0) / wall, 2),
                "runner_status_calls": self.status_calls,
                "pi_cpu_total_pct": round(100.0 * (1.0 - d_idle / d_total), 2) if d_total > 0 else float("nan")}


# ─────────────────────────── Pi state + settings helpers ───────────────────────────
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


def pi_state():
    """CPU temperature and the firmware throttle flags (vcgencmd get_throttled)."""
    st = {}
    try:
        st["cpu_temp_c"] = round(int(Path("/sys/class/thermal/thermal_zone0/temp").read_text()) / 1000, 1)
    except (OSError, ValueError):
        st["cpu_temp_c"] = None
    p = run(["vcgencmd", "get_throttled"], timeout=5) if shutil.which("vcgencmd") else None
    st["throttled"] = p.stdout.strip().split("=")[-1] if p and p.returncode == 0 else None
    return st


def controller_affinity(controller):
    """Cpus_allowed_list of the live controller process (container processes are visible from the host)."""
    exe = {"proportional": "hil_servo_node", "ibvs": "visp_servo_node", "h_vs": "h_vs_servo_node",
           "pbvs": "visp_pbvs_node"}[controller]
    for pid in run(["pgrep", "-x", exe[:15]], timeout=5).stdout.split():
        try:
            for line in Path(f"/proc/{pid}/status").read_text().splitlines():
                if line.startswith("Cpus_allowed_list:"):
                    return line.split(":", 1)[1].strip()
        except OSError:
            continue
    return "not found"


# ─────────────────────────── Wi-Fi ───────────────────────────
def wifi_pi(iface="wlan0"):
    """Live link from `iw dev <iface> link`: (ssid, dBm, %)."""
    p = run(["iw", "dev", iface, "link"], timeout=5)
    ssid = re.search(r"SSID: (.+)", p.stdout)
    sig = re.search(r"signal: (-?\d+) dBm", p.stdout)
    if not (ssid and sig):
        return None, None, None
    dbm = float(sig.group(1))
    return ssid.group(1).strip(), dbm, pct_from_dbm(dbm)


def wifi_windows(ssh_target):
    """Live link from `netsh wlan show interfaces` over SSH: (ssid, %, dBm or None)."""
    p = run(["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=6", ssh_target,
             "netsh wlan show interfaces"], timeout=15)
    if p.returncode != 0:
        return None, None, None, (p.stderr.strip().splitlines() or ["ssh failed"])[-1]
    ssid = re.search(r"^\s*SSID\s*:\s*(.+)$", p.stdout, re.M)
    sigp = re.search(r"^\s*Signal\s*:\s*(\d+)\s*%", p.stdout, re.M)
    rssi = re.search(r"^\s*Rssi\s*:\s*(-?\d+)", p.stdout, re.M)
    if not (ssid and sigp):
        return None, None, None, "netsh gave no SSID/Signal (wired, or Wi-Fi off?)"
    return ssid.group(1).strip(), float(sigp.group(1)), (float(rssi.group(1)) if rssi else None), ""


class WifiSampler(threading.Thread):
    """1 Hz receive rate of the Pi's Wi-Fi into <run_dir>/wifi.csv."""

    def __init__(self, path, iface="wlan0"):
        super().__init__(daemon=True)
        self.path, self.iface = Path(path), iface
        self.stop_evt = threading.Event()
        self.rates = deque(maxlen=60)
        self.error = None

    def _rx(self):
        return int(Path(f"/sys/class/net/{self.iface}/statistics/rx_bytes").read_text())

    def run(self):
        try:
            self._run()
        except Exception as e:          # never die silently: the runner shows rx "?" and logs this
            self.error = f"{e.__class__.__name__}: {e}"

    def _run(self):
        with open(self.path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t_wall_ns", "rx_MBps", "signal_dbm"])
            last_t, last_b = time.monotonic(), self._rx()
            while not self.stop_evt.wait(1.0):
                t, b = time.monotonic(), self._rx()
                rate = (b - last_b) / max(t - last_t, 1e-6) / 1e6
                last_t, last_b = t, b
                _, dbm, _ = wifi_pi(self.iface)
                self.rates.append(rate)
                w.writerow([now_ns(), f"{rate:.4f}", "" if dbm is None else int(dbm)])
                f.flush()

    def stop(self):
        self.stop_evt.set()
        self.join(timeout=3)


# ─────────────────────────── keys ───────────────────────────
class Keys(threading.Thread):
    def __init__(self):
        super().__init__(daemon=True)
        self.pause = False
        self.abort = False
        self.show_log = False
        self._fd = sys.stdin.fileno() if sys.stdin.isatty() else None
        self._old = termios.tcgetattr(self._fd) if self._fd is not None else None
        self._stop = threading.Event()

    def run(self):
        if self._fd is None:
            return
        tty.setcbreak(self._fd)
        while not self._stop.is_set():
            r, _, _ = select.select([sys.stdin], [], [], 0.2)
            if r:
                ch = sys.stdin.read(1).lower()
                if ch == "p":
                    self.pause = not self.pause
                elif ch == "a":
                    self.abort = True
                elif ch == "l":
                    self.show_log = not self.show_log

    def restore(self):
        self._stop.set()
        if self._old is not None:
            termios.tcsetattr(self._fd, termios.TCSADRAIN, self._old)

    def suspend(self):
        if self._old is not None:
            termios.tcsetattr(self._fd, termios.TCSADRAIN, self._old)

    def resume_cbreak(self):
        if self._fd is not None:
            tty.setcbreak(self._fd)


# ─────────────────────────── the bench process ───────────────────────────
class Bench:
    """controller_hil_bench.sh inside the container, with its output parsed live."""

    STATE_RE = re.compile(r"State: (\w+) -> (\w+)")

    def __init__(self, controller, run_num, env, console_log):
        self.lines = deque(maxlen=400)
        self.run_rel = None
        self.gate_wait = False
        self.recording = False
        self.fsm = "—"
        self.error = None
        self.log = open(console_log, "w")
        envs = []
        for k, v in env.items():
            envs += ["-e", f"{k}={v}"]
        cmd = ["sudo", "-n", "docker", "exec", *envs, CONTAINER, "bash", "-lc",
               f"cd /workspace && ./benchmarks/controller_hil_bench.sh {controller} {run_num} 0 true"]
        self.proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                     stdin=subprocess.DEVNULL, text=True, bufsize=1)
        self.reader = threading.Thread(target=self._read, daemon=True)
        self.reader.start()

    def _read(self):
        for line in self.proc.stdout:
            line = line.rstrip("\n")
            self.log.write(line + "\n")
            self.log.flush()
            self.lines.append(line)
            m = re.search(r"Run dir: (bags/\S+)", line)
            if m:
                self.run_rel = m.group(1)
            if "waiting for record gate" in line:
                self.gate_wait = True
            if line.startswith("[bench] recording"):
                self.recording = True
            if "ERROR" in line and line.startswith("[bench]"):
                self.error = line
            s = self.STATE_RE.search(line)
            if s:
                self.fsm = s.group(2)

    def alive(self):
        return self.proc.poll() is None

    def stop(self, timeout=25):
        run(["sudo", "-n", "docker", "exec", CONTAINER, "pkill", "-INT", "-f", "controller_hil_bench.sh"], timeout=10)
        t_end = time.monotonic() + timeout
        while self.alive() and time.monotonic() < t_end:
            time.sleep(0.3)
        if self.alive():
            self.proc.kill()
        # the bench's own cleanup stops the launch through its pipe; make sure no
        # controller is left publishing /cmd_vel into the next run's hold
        for name in ("oracle_detector_node", "hil_servo_node", "visp_servo_node", "h_vs_servo_node",
                     "visp_pbvs_node", "ros2 bag record"):
            run(["sudo", "-n", "docker", "exec", CONTAINER, "pkill", "-INT", "-f", name], timeout=10)
        time.sleep(1.0)
        self.reader.join(timeout=3)
        self.log.close()


# ─────────────────────────── the runner ───────────────────────────
class Runner:
    def __init__(self, a):
        self.a = a
        self.keys = Keys()
        stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.session = WS / "reports" / f"ctrl_{a.tag}_{stamp}"
        self.session.mkdir(parents=True, exist_ok=True)
        self.slog = open(self.session / "session.log", "a")
        self.links = {}          # name -> (ok, text)
        self.steps = []          # (name, status, detail)
        self.rows = []           # finished runs
        self.flight = None       # live flight dict
        self.bench = None
        self.banner = ""
        sha = run(["git", "-C", str(WS), "rev-parse", "--short", "HEAD"]).stdout.strip()
        dirty = run(["git", "-C", str(WS), "status", "--porcelain", "--untracked-files=no"]).stdout.strip()
        self.git = sha + ("+dirty" if dirty else "")
        self.branch = run(["git", "-C", str(WS), "branch", "--show-current"]).stdout.strip()
        self.say(f"session {self.session.name} · {self.branch} @ {self.git} · args {vars(a)}")

    # ── logging ──
    def say(self, msg):
        line = f"{dt.datetime.now().strftime('%H:%M:%S')} {msg}"
        self.slog.write(line + "\n")
        self.slog.flush()

    def deviation(self, msg):
        with open(self.session / "deviations.log", "a") as f:
            f.write(f"{dt.datetime.now().isoformat(timespec='seconds')} {msg}\n")
        self.say("DEVIATION " + msg)

    def ssh_target(self):
        return f"{self.a.win_user}@{self.a.host}"

    # ── screen ──
    def render(self):
        hdr = Text.assemble(("ctrl_runner", "bold cyan"), f" · {self.a.controller} · tag {self.a.tag} · ",
                            (f"{self.branch} @ {self.git}", "dim"), " · Fast DDS")
        lk = Table.grid(padding=(0, 2))
        for name, (ok, txt) in self.links.items():
            mark = {True: "[green]✔[/]", False: "[red]✘[/]", None: "[yellow]…[/]"}[ok]
            lk.add_row(mark, f"[bold]{name}[/]", txt)
        st = Table.grid(padding=(0, 1))
        for name, status, detail in self.steps:
            mark = {"done": "[green]✔[/]", "run": "[cyan]▶[/]", "fail": "[red]✘[/]", "wait": "[dim]·[/]"}[status]
            st.add_row(mark, name, f"[dim]{detail}[/]")
        parts = [hdr, Panel(lk, title="links", border_style="dim")]
        if self.banner:
            parts.append(Panel(Text(self.banner, style="bold yellow"), border_style="yellow"))
        if self.steps:
            parts.append(Panel(st, title=self.run_title, border_style="cyan"))
        if self.flight:
            f = self.flight
            bar = ProgressBar(total=f["total"], completed=min(f["elapsed"], f["total"]), width=48)
            ft = Table.grid(padding=(0, 2))
            ft.add_row(bar, f"{f['elapsed']:5.0f} / {f['total']:.0f} s")
            ft.add_row(f"sim [green]{f.get('sim', '?')}[/] t={f.get('t', '?')}   FSM [yellow]{f.get('fsm', '—')}[/]",
                       f"camera pub {f.get('pub', '?')}")
            ft.add_row(f"pose x={f.get('x', '?')} y={f.get('y', '?')} z={f.get('z', '?')} yaw={f.get('yaw', '?')}°",
                       f"Wi-Fi rx {f.get('rx', '?')} MB/s")
            ft.add_row(f"distance to sign [bold]{f.get('dist', '?')}[/] m  {spark(f.get('dhist', []), STANDOFF - 1, None)}",
                       f"target {STANDOFF:.2f} m")
            parts.append(Panel(ft, title="flight", border_style="green"))
        rt = Table(show_edge=False, header_style="bold")
        for c in ("run", "camera", "valid", "sequence / reason", "settling s", "ss err m", "overshoot m",
                  "path eff", "RMS cmd m/s", "hb gaps>0.2s", "rx MB/s"):
            rt.add_column(c)
        for r in self.rows:
            rt.add_row(*r)
        parts.append(Panel(rt, title="runs", border_style="dim"))
        if self.keys.show_log and self.bench:
            tail = "\n".join(list(self.bench.lines)[-12:])
            parts.append(Panel(Text(tail, style="dim"), title="bench log (l to hide)", border_style="dim"))
        flags = []
        if self.keys.pause:
            flags.append("[yellow]PAUSE after this run[/]")
        if self.keys.abort:
            flags.append("[red]ABORT requested[/]")
        parts.append(Text.from_markup("  ".join(flags) + "   [dim]p pause after this run · a abort · l bench log[/]"))
        return Group(*parts)

    def step(self, name, status, detail=""):
        for i, (n, _, _) in enumerate(self.steps):
            if n == name:
                self.steps[i] = (name, status, detail)
                break
        else:
            self.steps.append((name, status, detail))
        self.say(f"step {name}: {status} {detail}")

    # ── preflight ──
    def check_wifi(self, live):
        ssid, dbm, pct = wifi_pi()
        ok = pct is not None and pct >= self.a.wifi_min
        self.links["Pi Wi-Fi"] = (ok, f"{ssid or '?'} · {dbm if dbm is not None else '?'} dBm · {pct if pct is not None else '?'} % (iw, live link)")
        live.update(self.render())
        if self.a.skip_windows_wifi:
            self.links["Windows Wi-Fi"] = (None, "not checked (--skip-windows-wifi)")
            win_ok = True
        else:
            wssid, wpct, wrssi, err = wifi_windows(self.ssh_target())
            win_ok = wpct is not None and wpct >= self.a.wifi_min
            txt = f"{wssid} · {wpct:.0f} %" + (f" · {wrssi:.0f} dBm" if wrssi is not None else "") + " (netsh)" if wpct is not None else f"not readable: {err}"
            self.links["Windows Wi-Fi"] = (win_ok, txt)
        live.update(self.render())
        info = {"wifi_pi_ssid": ssid, "wifi_pi_dbm": dbm, "wifi_pi_pct": pct}
        if not self.a.skip_windows_wifi:
            info.update({"wifi_win_ssid": wssid, "wifi_win_pct": wpct, "wifi_win_rssi_dbm": wrssi})
        return ok and win_ok, info

    def wait_matlab(self, live):
        try:
            st = sup.status(self.a.host, self.a.port, timeout=3)
            self.links["MATLAB supervisor"] = (True, f"{self.a.host}:{self.a.port} · state={st.get('state')} sim={st.get('sim')}")
            live.update(self.render())
            return True
        except sup.SupervisorError:
            pass
        t_end = time.monotonic() + self.a.matlab_wait
        while time.monotonic() < t_end:
            left = t_end - time.monotonic()
            self.banner = f"Paste in MATLAB:   hil_session_start        (waiting {left:4.0f} s)"
            self.links["MATLAB supervisor"] = (None, f"{self.a.host}:{self.a.port} · no answer yet")
            live.update(self.render())
            time.sleep(2.0)
            try:
                st = sup.status(self.a.host, self.a.port, timeout=3)
                self.banner = ""
                self.links["MATLAB supervisor"] = (True, f"{self.a.host}:{self.a.port} · state={st.get('state')} sim={st.get('sim')}")
                live.update(self.render())
                return True
            except sup.SupervisorError:
                continue
        self.banner = f"MATLAB did not answer within {self.a.matlab_wait:.0f} s — stopped."
        self.links["MATLAB supervisor"] = (False, f"{self.a.host}:{self.a.port} · no answer in {self.a.matlab_wait:.0f} s")
        live.update(self.render())
        return False

    def check_container(self, live):
        p = run(["sudo", "-n", "docker", "inspect", "-f", "{{.State.Running}} {{range .Mounts}}{{if eq .Destination \"/workspace\"}}{{.Source}}{{end}}{{end}}", CONTAINER])
        if p.returncode == 0:
            running, _, src = p.stdout.strip().partition(" ")
            if running == "true" and Path(src).resolve() == WS:
                self.links["container"] = (True, f"{CONTAINER} · /workspace = {src}")
                live.update(self.render())
                return True
            if running == "true":
                self.links["container"] = (False, f"{CONTAINER} runs from {src}, not {WS} — stop it yourself, then rerun")
                live.update(self.render())
                return False
        self.links["container"] = (None, "starting (start_container.sh: create + apt + colcon build)…")
        live.update(self.render())
        proc = subprocess.Popen(["bash", str(WS / "start_container.sh")], stdout=subprocess.PIPE,
                                stderr=subprocess.STDOUT, text=True)
        with open(self.session / "start_container.log", "w") as f:
            for line in proc.stdout:
                f.write(line)
                self.links["container"] = (None, "starting… " + line.strip()[:90])
                live.update(self.render())
        ok = proc.wait() == 0
        self.links["container"] = (ok, f"{CONTAINER} started" if ok else "start_container.sh failed — see start_container.log")
        live.update(self.render())
        return ok

    def preflight(self, live):
        free = shutil.disk_usage(WS).free / 1e9
        self.links["disk"] = (free > 5, f"{free:.1f} GB free")
        self.links["git"] = ("+dirty" not in self.git, f"{self.branch} @ {self.git}" + ("  (uncommitted changes in tracked files)" if "+dirty" in self.git else ""))
        if "+dirty" in self.git:
            self.deviation(f"session started on a dirty tree ({self.git})")
        live.update(self.render())
        wifi_ok, _ = self.check_wifi(live)
        if not wifi_ok:
            self.banner = f"Wi-Fi below {self.a.wifi_min:.0f} % (or not readable) — stopped before any run."
            live.update(self.render())
            return False
        if not self.wait_matlab(live):
            return False
        if self.a.dry_run:
            return True
        return self.check_container(live)

    # ── Pi settings (ctrl_bench.yaml: pi.*) ──
    def apply_pi_settings(self, live):
        applied = {}
        self._governor_before = None
        if self.a.runner_cores != "none":
            cores = parse_cores(self.a.runner_cores)
            os.sched_setaffinity(0, cores)           # this process; children (iw, scorer) inherit it
            applied["runner_cores"] = sorted(os.sched_getaffinity(0))
        if self.a.cpu_governor == "performance":
            self._governor_before = governors()
            for cpu in self._governor_before:
                run(["sudo", "-n", "sh", "-c", f"echo performance > /sys/devices/system/cpu/{cpu}/cpufreq/scaling_governor"],
                    timeout=10, check=True)
            applied["cpu_governor"] = governors()
        self._net_before = None
        if self.a.net_buffers == "hil":
            self._net_before = net_buffers()
            run(["sudo", "-n", "sysctl", "-q", *[f"{k}={v}" for k, v in HIL_NET_BUFFERS.items()]], timeout=10, check=True)
            applied["net_buffers"] = net_buffers()
        self.links["Pi settings"] = (True, f"controller cores {self.a.controller_cores} · runner cores {self.a.runner_cores} · "
                                           f"governor {self.a.cpu_governor} · net buffers {self.a.net_buffers}")
        live.update(self.render())
        self.say(f"Pi settings applied: {applied or 'none (all leave/none)'}")
        return applied

    def restore_pi_settings(self):
        before = getattr(self, "_governor_before", None)
        if before:
            for cpu, gov in before.items():
                run(["sudo", "-n", "sh", "-c", f"echo {gov} > /sys/devices/system/cpu/{cpu}/cpufreq/scaling_governor"], timeout=10)
            self.say(f"CPU governor restored: {governors()}")
        net_before = getattr(self, "_net_before", None)
        if net_before:
            run(["sudo", "-n", "sysctl", "-q", *[f"{k}={v}" for k, v in net_before.items()]], timeout=10)
            self.say(f"net buffers restored: {net_buffers()}")

    def snapshot(self, applied):
        """Freeze what this session used: settings file, the bench config files, git state,
        container image, and the Pi state. Written to <session>/settings/."""
        import json
        sdir = self.session / "settings"
        sdir.mkdir(exist_ok=True)
        if self.a.config_found:
            shutil.copy(self.a.config, sdir / Path(self.a.config).name)
        for rel in ("config/hil/bench_fsm.yaml", f"config/hil/bench_{self.a.controller}.yaml", "config/hil/bench_oracle.yaml",
                    "config/hil/fastrtps_hil.xml", "config/hil/fastrtps_matlab.xml", "config/hil/bench_bag_qos.yaml",
                    "benchmarks/bench_geometry.py", "matlab/hil_session_start.m"):
            if (WS / rel).exists():
                shutil.copy(WS / rel, sdir / Path(rel).name)
        (sdir / "git_diff.patch").write_text(run(["git", "-C", str(WS), "diff", "HEAD"], timeout=30).stdout)
        img = run(["sudo", "-n", "docker", "inspect", "-f", "{{.Image}} {{.Created}}", CONTAINER], timeout=15).stdout.strip()
        effective = {k: getattr(self.a, k) for k in SETTINGS}
        (sdir / "session_settings.json").write_text(json.dumps({
            "settings_file": str(self.a.config) if self.a.config_found else None,
            "command_line_overrides": self.a.overridden,
            "effective": effective,
            "applied_pi_settings": applied,
            "git": {"branch": self.branch, "commit": self.git, "diff_file": "git_diff.patch"},
            "container": {"name": CONTAINER, "image_and_created": img},
            "pi_state_at_start": {**pi_state(), "cpu_governor": governors(), "net_buffers": net_buffers()},
        }, indent=2, default=str))
        self.say(f"settings snapshot -> {sdir}")

    # ── one run ──
    def one_run(self, live, k, n_total, cam):
        self.run_title = f"run {k}/{n_total} · camera publisher {cam.upper()}"
        self.steps, self.flight = [], None
        for name in ("Wi-Fi check", "camera publisher", "start_hold", "start check", "boot bench",
                     "recorder", "resume", "flight", "stop", "cool-down", "score + check"):
            self.step(name, "wait")
        live.update(self.render())
        meta = {"camera_pub": cam, "protocol": "SIM_PARAMS start_hold -> start check -> boot -> resume",
                "runner_session": self.session.name, "flight_s": self.a.flight}

        def fail(step, why):
            self.step(step, "fail", why)
            live.update(self.render())
            return {"valid": False, "reason": f"{step}: {why}", "meta": meta}

        # 1. Wi-Fi
        self.step("Wi-Fi check", "run")
        live.update(self.render())
        ok, info = self.check_wifi(live)
        meta.update(info)
        if not ok:
            return fail("Wi-Fi check", f"below {self.a.wifi_min:.0f} % or not readable")
        self.step("Wi-Fi check", "done", f"Pi {info['wifi_pi_pct']:.0f} %")

        # 2. camera publisher
        self.step("camera publisher", "run")
        live.update(self.render())
        try:
            sup.send("pub_stop" if cam == "off" else f"pub_start {self.a.pub_hz:g}", self.a.host, self.a.port)
        except sup.SupervisorError as e:
            return fail("camera publisher", str(e))
        self.step("camera publisher", "done", cam)

        # 3. start_hold -> held
        self.step("start_hold", "run")
        live.update(self.render())
        try:
            sup.send("start_hold", self.a.host, self.a.port)
        except sup.SupervisorError as e:
            return fail("start_hold", str(e))
        t_end, st = time.monotonic() + self.a.hold_timeout, {}
        while time.monotonic() < t_end:
            time.sleep(1.0)
            try:
                st = sup.status(self.a.host, self.a.port)
            except sup.SupervisorError:
                continue
            self.step("start_hold", "run", f"state={st.get('state')} sim={st.get('sim')} t={st.get('t')}")
            live.update(self.render())
            if st.get("state") in ("held", "frozen"):
                break
        if st.get("state") != "held":
            return fail("start_hold", f"not held within {self.a.hold_timeout:.0f} s (state={st.get('state')})")
        self.step("start_hold", "done", f"held at sim t={st.get('t')}")

        # 4. start check
        self.step("start check", "run")
        if not all(k in st for k in ("x", "y", "yaw")):
            return fail("start check", "status has no pose (yaw/x/y) — new read_cmdvel_live_interp.m on the MATLAB path?")
        brg = bearing_deg(st["x"], st["y"])
        dyaw = angle_diff(st["yaw"], brg)
        meta.update({"start_x": st["x"], "start_y": st["y"], "start_z": st.get("z"), "start_yaw_deg": st["yaw"],
                     "start_bearing_deg": round(brg, 2), "start_yaw_err_deg": round(dyaw, 2)})
        if dyaw > 5.0:
            return fail("start check", f"yaw {st['yaw']:.1f}° is {dyaw:.1f}° off the bearing {brg:.1f}° (limit 5°)")
        self.step("start check", "done", f"yaw err {dyaw:.2f}° · x={st['x']:.2f} y={st['y']:.2f} z={st.get('z')}")

        # 5. boot the bench (records only after the gate opens)
        gate_host = WS / "bags" / f".gate_{self.session.name}_{k}"
        gate_host.parent.mkdir(exist_ok=True)
        gate_host.unlink(missing_ok=True)
        env = {"MATLAB_HOST_IP": self.a.host, "DDS": self.a.dds, "ROS_DOMAIN_ID": str(self.a.domain),
               "RUN_TAG": f"{self.a.tag}_{cam}", "RECORD_GATE": f"/workspace/bags/{gate_host.name}",
               "CTRL_CORE": self.a.controller_cores}
        self.step("boot bench", "run")
        live.update(self.render())
        self.bench = Bench(self.a.controller, k, env, self.session / f"run{k}_bench_console.log")
        t_end = time.monotonic() + self.a.boot_timeout
        while time.monotonic() < t_end and not self.bench.gate_wait and self.bench.alive():
            self.step("boot bench", "run", (self.bench.lines[-1] if self.bench.lines else "")[:90])
            live.update(self.render())
            time.sleep(0.5)
        if not self.bench.gate_wait:
            why = self.bench.error or ("bench exited" if not self.bench.alive() else f"no gate wait within {self.a.boot_timeout:.0f} s")
            self.bench.stop()
            return fail("boot bench", why)
        run_dir = WS / self.bench.run_rel
        meta["run_dir"] = str(run_dir)
        meta["controller_affinity"] = controller_affinity(self.a.controller)
        meta["controller_cores_setting"] = self.a.controller_cores
        self.step("boot bench", "done", f"{self.bench.run_rel} · nodes up")

        # 6. open the gate, wait until the recorder has subscribed to all 7 topics
        self.step("recorder", "run")
        gate_host.touch()
        wifi_host = self.session / f"run{k}_wifi.csv"      # the run dir belongs to root (container)
        sampler = WifiSampler(wifi_host)
        sampler.start()
        t_end, subs = time.monotonic() + 15, 0
        while time.monotonic() < t_end:
            rec_log = run_dir / "bag_record.log"
            subs = rec_log.read_text(errors="replace").count("Subscribed to topic") if rec_log.exists() else 0
            self.step("recorder", "run", f"subscribed {subs}/7")
            live.update(self.render())
            if subs >= 7:
                break
            time.sleep(0.3)
        if subs < 7:
            sampler.stop()
            self.bench.stop()
            return fail("recorder", f"recorder subscribed to {subs}/7 topics within 15 s")
        self.step("recorder", "done", "7/7 topics")

        # 7. resume — the stopwatch starts here
        self.step("resume", "run")
        sent = now_ns()
        try:
            sup.send("resume", self.a.host, self.a.port)
        except sup.SupervisorError as e:
            sampler.stop()
            self.bench.stop()
            return fail("resume", str(e))
        meta.update({"resume_sent_ns": sent, "resume_wall_ns": now_ns()})
        meta_host = self.session / f"run{k}_meta.txt"       # on disk now: a crash cannot lose t=0
        meta_host.write_text("".join(f"{key}={val}\n" for key, val in meta.items()))
        self.step("resume", "done", "t = 0")

        # 8. flight
        self.step("flight", "run")
        t_start = time.monotonic()
        self.flight = {"total": self.a.flight, "elapsed": 0.0, "dhist": [], "pub": cam}
        meter = OverheadMeter()
        meta.update({f"{k}_before": v for k, v in pi_state().items()})
        while True:
            el = time.monotonic() - t_start
            self.flight["elapsed"] = el
            if el >= self.a.flight or self.keys.abort:
                break
            try:
                meter.status_calls += 1
                st = sup.status(self.a.host, self.a.port, timeout=2)
                d = math.dist((st.get("x", 0), st.get("y", 0), st.get("z", 0)), TARGET) if "x" in st else None
                self.flight.update({"sim": st.get("sim"), "t": st.get("t"), "x": st.get("x"), "y": st.get("y"),
                                    "z": st.get("z"), "yaw": st.get("yaw"),
                                    "dist": f"{d:.2f}" if d is not None else "?"})
                if d is not None:
                    self.flight["dhist"] = (self.flight["dhist"] + [d])[-40:]
            except sup.SupervisorError:
                self.flight["sim"] = "no status"
            self.flight["fsm"] = self.bench.fsm
            self.flight["rx"] = f"{sampler.rates[-1]:.2f}" if sampler.rates else "?"
            live.update(self.render())
            time.sleep(1.0)
        oh = meter.result()
        meta.update(oh)
        meta.update({f"{k}_after": v for k, v in pi_state().items()})
        self.step("flight", "done" if not self.keys.abort else "fail",
                  f"{min(el, self.a.flight):.0f} s · runner {oh['runner_cpu_pct_core']} % of a core · "
                  f"Pi total {oh['pi_cpu_total_pct']} % · {oh['runner_status_calls']} status calls")

        # 9. stop
        self.step("stop", "run")
        live.update(self.render())
        sampler.stop()
        self.bench.stop()
        gate_host.unlink(missing_ok=True)
        run(["sudo", "-n", "chown", "-R", f"{os.getuid()}:{os.getgid()}", str(run_dir)], timeout=30, check=True)
        if sampler.error:
            meta["wifi_sampler_error"] = sampler.error
        with open(run_dir / "meta.txt", "a") as f:
            for key, val in meta.items():
                f.write(f"{key}={val}\n")
        if wifi_host.exists():
            shutil.copy(wifi_host, run_dir / "wifi.csv")
        self.step("stop", "done", "bag closed · nodes stopped · folder owned by " + os.environ.get("USER", "?"))

        # 10. cool-down
        for s in range(int(self.a.cooldown), 0, -1):
            self.step("cool-down", "run", f"{s} s")
            live.update(self.render())
            time.sleep(1.0)
        self.step("cool-down", "done")

        # 11. score + check
        self.step("score + check", "run")
        live.update(self.render())
        sc = run([SCORER_PY, str(WS / "benchmarks" / "plot_controller_hil.py"), str(run_dir)], timeout=300)
        ck = run([SCORER_PY, str(WS / "benchmarks" / "ctrl_run_check.py"), str(run_dir)], timeout=300)
        (run_dir / "score.log").write_text(sc.stdout + sc.stderr + "\n" + ck.stdout + ck.stderr)
        res = read_results(run_dir)
        res["meta"] = meta
        self.step("score + check", "done" if res["valid"] else "fail", res["reason"])
        return res

    # ── session ──
    def conditions(self):
        if self.a.camera_pub == "ab":
            return [c for _ in range(self.a.runs) for c in ("off", "on")]
        return [self.a.camera_pub] * self.a.runs

    def go(self):
        self.keys.start()
        self.run_title = ""
        try:
            with Live(self.render(), console=console, refresh_per_second=4, screen=False) as live:
                if not self.preflight(live):
                    return 1
                if self.a.dry_run:
                    self.snapshot(applied={})
                    self.banner = "Dry run: preflight passed, no command that changes MATLAB was sent."
                    live.update(self.render())
                    return 0
                applied = self.apply_pi_settings(live)
                self.snapshot(applied)
                conds = self.conditions()
                csv_path = self.session / "runs.csv"
                with open(csv_path, "w", newline="") as f:
                    csv.writer(f).writerow(["run", "camera_pub", "valid", "reason", "sequence", "run_dir"])
                for k, cam in enumerate(conds, 1):
                    if self.keys.abort:
                        break
                    try:
                        res = self.one_run(live, k, len(conds), cam)
                    except Exception as e:
                        import traceback
                        self.say("RUNNER ERROR in run %d:\n%s" % (k, traceback.format_exc()))
                        if self.bench and self.bench.alive():
                            self.bench.stop()
                        res = {"valid": False, "reason": f"runner error: {e.__class__.__name__}: {e}"[:160],
                               "sequence": [], "meta": {"camera_pub": cam}}
                    self.rows.append(table_row(k, cam, res))
                    with open(csv_path, "a", newline="") as f:
                        csv.writer(f).writerow([k, cam, res["valid"], res["reason"], " -> ".join(res.get("sequence", [])),
                                                res["meta"].get("run_dir", "")])
                    live.update(self.render())
                    if self.keys.abort:
                        break
                    if not res["valid"] and k < len(conds):
                        live.stop()
                        self.keys.suspend()
                        ans = Prompt.ask(f"[red]Run {k} is not valid[/] ({res['reason']}). n = next run, a = abort",
                                         choices=["n", "a"], default="n")
                        self.keys.resume_cbreak()
                        live.start()
                        if ans == "a":
                            self.keys.abort = True
                            break
                    if self.keys.pause and k < len(conds):
                        live.stop()
                        self.keys.suspend()
                        Prompt.ask("[yellow]Paused.[/] Press Enter for the next run", default="")
                        self.keys.pause = False
                        self.keys.resume_cbreak()
                        live.start()
                try:
                    sup.send("stop", self.a.host, self.a.port)
                except sup.SupervisorError:
                    pass
                self.steps, self.flight = [], None
                self.banner = "Session done — summarising…"
                live.update(self.render())
                sm = run([SCORER_PY, str(WS / "benchmarks" / "ctrl_session_summary.py"), str(self.session)], timeout=300)
                (self.session / "summary.log").write_text(sm.stdout + sm.stderr)
                self.banner = f"Session done. Summary: {self.session / 'summary.md'}"
                live.update(self.render())
            console.print(Text((self.session / "summary.md").read_text() if (self.session / "summary.md").exists() else sm.stdout))
            return 0
        finally:
            if self.bench and self.bench.alive():
                self.bench.stop()
            self.restore_pi_settings()
            if not self.a.dry_run:
                try:
                    sup.send("stop", self.a.host, self.a.port)
                    self.say("sent stop to MATLAB")
                except sup.SupervisorError as e:
                    self.say(f"could not send stop to MATLAB: {e}")
            self.keys.restore()
            self.say("session end")


def read_results(run_dir):
    import json
    out = {"valid": False, "reason": "no run_check.json", "sequence": []}
    chk = run_dir / "run_check.json"
    if chk.exists():
        c = json.loads(chk.read_text())
        out.update({"valid": c["valid"], "reason": c["reason"], "sequence": c["sequence"],
                    "hb_gaps": c["timing"]["/sim/heartbeat"]["gaps_over_0.2s"],
                    "rx": c["wifi"]["rx_mean_MBps"]})
    met = run_dir / "metrics.csv"
    if met.exists():
        for row in csv.reader(open(met)):
            if len(row) == 2 and row[0] != "metric":
                out[row[0]] = row[1]
    return out


def table_row(k, cam, r):
    def num(key, fmt="{:.2f}"):
        try:
            return fmt.format(float(r[key]))
        except (KeyError, ValueError):
            return "—"
    seq = " → ".join(r.get("sequence", [])) or r.get("reason", "")
    return [str(k), cam, "[green]✔[/]" if r["valid"] else "[red]✘[/]", seq if r["valid"] else f"{seq} [dim]({r['reason']})[/]",
            num("settling_time_s", "{:.1f}"), num("steady_state_err_m", "{:.3f}"), num("overshoot_m", "{:.3f}"),
            num("path_efficiency", "{:.3f}"), num("rms_cmd_speed"), str(r.get("hb_gaps", "—")), num("rx")]


CONFIG_DEFAULT = WS / "config" / "hil" / "ctrl_bench.yaml"

# command-line option -> (section, key) in ctrl_bench.yaml, and its built-in fallback
SETTINGS = {
    "controller": ("session", "controller", "proportional"),
    "runs": ("session", "runs", 3),
    "camera_pub": ("session", "camera_pub", "ab"),
    "pub_hz": ("session", "camera_pub_hz", 20.0),
    "flight": ("session", "flight_s", 150.0),
    "cooldown": ("session", "cooldown_s", 10.0),
    "tag": ("session", "tag", "P_unified_N3"),
    "host": ("matlab", "host", sup.DEFAULT_HOST),
    "port": ("matlab", "port", sup.DEFAULT_PORT),
    "win_user": ("matlab", "windows_user", "homie"),
    "matlab_wait": ("matlab", "wait_s", 180.0),
    "hold_timeout": ("matlab", "hold_timeout_s", 45.0),
    "dds": ("network", "dds", "fastrtps"),
    "domain": ("network", "ros_domain_id", 0),
    "wifi_min": ("network", "wifi_min_pct", 90.0),
    "check_windows_wifi": ("network", "check_windows_wifi", True),
    "boot_timeout": ("pi", "boot_timeout_s", 60.0),
    "controller_cores": ("pi", "controller_cores", "none"),
    "runner_cores": ("pi", "runner_cores", "none"),
    "cpu_governor": ("pi", "cpu_governor", "leave"),
    "net_buffers": ("pi", "net_buffers", "leave"),
}


def main():
    import yaml
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("controller", nargs="?", choices=["proportional", "ibvs", "h_vs", "pbvs"],
                    help="default: session.controller in the settings file")
    ap.add_argument("--config", default=str(CONFIG_DEFAULT), help="settings file (default %(default)s)")
    ap.add_argument("--runs", type=int, help="runs per camera condition")
    ap.add_argument("--camera-pub", choices=["off", "on", "ab"], help="MATLAB camera publisher; ab = alternate OFF, ON")
    ap.add_argument("--pub-hz", type=float)
    ap.add_argument("--flight", type=float, help="seconds of flight after resume")
    ap.add_argument("--cooldown", type=float)
    ap.add_argument("--tag")
    ap.add_argument("--host")
    ap.add_argument("--port", type=int)
    ap.add_argument("--win-user")
    ap.add_argument("--matlab-wait", type=float, help="seconds to wait for the supervisor")
    ap.add_argument("--hold-timeout", type=float)
    ap.add_argument("--dds", choices=["fastrtps", "cyclonedds"])
    ap.add_argument("--domain", type=int)
    ap.add_argument("--wifi-min", type=float, help="minimum Wi-Fi %% (Windows scale) on both machines")
    ap.add_argument("--skip-windows-wifi", action="store_true", help="do not read Windows Wi-Fi (logged as a deviation)")
    ap.add_argument("--boot-timeout", type=float)
    ap.add_argument("--controller-cores", help='e.g. "3" (taskset) or none')
    ap.add_argument("--runner-cores", help='e.g. "0-2" or none')
    ap.add_argument("--cpu-governor", choices=["leave", "performance"])
    ap.add_argument("--net-buffers", choices=["leave", "hil"])
    ap.add_argument("--dry-run", action="store_true", help="preflight + status only; sends nothing that changes MATLAB")
    a = ap.parse_args()

    cfg_path = Path(a.config)
    cfg = yaml.safe_load(cfg_path.read_text()) if cfg_path.exists() else {}
    a.config_found = cfg_path.exists()
    a.overridden = []
    for name, (section, key, fallback) in SETTINGS.items():
        cli = getattr(a, name, None)
        if cli is not None:
            if (cfg.get(section) or {}).get(key) not in (None, cli):
                a.overridden.append(name)
            continue
        setattr(a, name, (cfg.get(section) or {}).get(key, fallback))
    if a.skip_windows_wifi:
        a.check_windows_wifi = False
    a.skip_windows_wifi = not a.check_windows_wifi
    for name in ("controller_cores", "runner_cores"):
        v = getattr(a, name)
        setattr(a, name, "none" if v in (None, "", "none", "None") else str(v))
    r = Runner(a)
    if not a.config_found:
        r.deviation(f"settings file {cfg_path} not found — built-in defaults used")
    if a.overridden:
        r.say("command line overrides the settings file for: " + ", ".join(a.overridden))
    if a.skip_windows_wifi:
        r.deviation("Windows Wi-Fi not checked (network.check_windows_wifi false or --skip-windows-wifi)")
    return r.go()


if __name__ == "__main__":
    sys.exit(main())
