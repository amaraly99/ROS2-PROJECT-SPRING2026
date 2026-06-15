#!/usr/bin/env python3
"""
OV2SLAM EuRoC benchmark runner
================================

Config-driven multi-sequence benchmarking for the ROS2 OV2SLAM node,
keeping the same terminal UI style as ~/ORB_SLAM2/benchmark_euroc.py.

Features
--------
* Reads OV2_BENCHMARK.yaml (or built-in defaults)
* Benchmarks EuRoC bags in stereo or mono mode
* Selects trajectory via 'paper' or 'best' policy (from BENCH.sh)
* Inline /proc-based process + per-thread CPU monitor (no external deps)
* Runs evo_ape + evo_traj and writes per-run / per-sequence summaries

Per run produces (inside results/<UTC+4_stamp>/<seq_name>/run_NN/):
  * trajectory.tum
  * ape_traj.png            (from evo_ape --save_plot)
  * traj_xy.png             (from evo_traj --save_plot)
  * <seq>_evo.txt           (raw evo output)
  * ov2slam.log             (mirrored node stdout)
  * bag_play.log
  * gt.tum                  (validated + normalised ground truth)
  * ov2slam_timings.csv     (timer means written by the node)
  * ov2slam_pid_info.json
  * ov2slam_process_cpu.csv
  * ov2slam_thread_cpu.csv
  * ov2slam_thread_cpu_summary.csv
  * ov2slam_cpu_usage.png
  * ov2slam_threads_cpu_bar.png
  * run_meta.json

After all sequences:
  * <seq_name>/run_summary.json
  * experiment_summary.csv

Usage
-----
  python3 ov2_benchmark.py
  python3 ov2_benchmark.py --config OV2_BENCHMARK.yaml
  python3 ov2_benchmark.py --config OV2_BENCHMARK.yaml --sequence MH_01_easy
  python3 ov2_benchmark.py --mode stereo --profile accurate --runs 3
"""

from __future__ import annotations

import argparse
import csv
import json
import os
import re
import shlex
import shutil
import signal
import subprocess
import sys
import threading
import time
from collections import Counter, defaultdict
from datetime import datetime, timedelta, timezone
from pathlib import Path

try:
    import yaml
except ImportError:
    print("ERROR: PyYAML required.  pip3 install pyyaml")
    sys.exit(1)


# ─── paths ────────────────────────────────────────────────────────────────────

SCRIPT_PATH = Path(__file__).resolve()
SCRIPT_DIR = SCRIPT_PATH.parent
LOCAL_WORKSPACE_ROOT = SCRIPT_DIR.parent
UTC_PLUS_4 = timezone(timedelta(hours=4))

# ─── OV2SLAM trajectory candidates (from BENCH.sh) ────────────────────────────

BEST_TRAJECTORY_CANDIDATES: list[tuple[str, str]] = [
    (
        "ov2slam_fullba_kfs_traj.txt",
        "final optimized keyframe trajectory after full BA; best final estimate when do_full_ba=1",
    ),
    (
        "ov2slam_kfs_traj.txt",
        "optimized keyframe trajectory after local BA / loop closure; primary fallback",
    ),
    (
        "ov2slam_traj.txt",
        "raw frame trajectory; fallback only when optimized keyframe outputs are absent",
    ),
]

PAPER_TRAJECTORY_CANDIDATES: list[tuple[str, str]] = [
    (
        "ov2slam_kfs_traj.txt",
        "paper-style choice: keyframe trajectory after online local BA / loop closure, "
        "without end-of-sequence full BA",
    ),
    (
        "ov2slam_traj.txt",
        "paper-style fallback: raw frame trajectory when keyframe output is missing",
    ),
    (
        "ov2slam_fullba_kfs_traj.txt",
        "last-resort fallback: full BA result exists, but this is an offline refinement "
        "beyond the paper's fully-online setting",
    ),
]

UNSUPPORTED_EVO_TRAJECTORIES = [
    "ov2slam_full_traj_wlc.txt",
    "ov2slam_full_traj_wlc_opt.txt",
]

# ─── ANSI colours (identical to benchmark_euroc.py) ───────────────────────────


class C:
    RESET = "\033[0m"
    BOLD = "\033[1m"
    DIM = "\033[2m"
    RED = "\033[91m"
    GREEN = "\033[92m"
    YELLOW = "\033[93m"
    BLUE = "\033[94m"
    MAGENTA = "\033[95m"
    CYAN = "\033[96m"
    WHITE = "\033[97m"


# ─── global state ─────────────────────────────────────────────────────────────

_log_fd = None
_all_procs: list[subprocess.Popen] = []


# ─── terminal output helpers (faithful to benchmark_euroc.py) ─────────────────


def _w(text: str) -> None:
    sys.stdout.write(text)
    sys.stdout.flush()
    if _log_fd:
        _log_fd.write(text)
        _log_fd.flush()


def msg(color: str, icon: str, label: str, detail: str = "") -> None:
    suffix = f"  {C.DIM}{detail}" if detail else ""
    _w(f"  {icon}  {C.BOLD}{color}{label}{C.RESET}{suffix}\n")


def ok(label: str, detail: str = "") -> None:
    msg(C.GREEN, f"{C.GREEN}✓{C.RESET}", label, detail)


def fail(label: str, detail: str = "") -> None:
    msg(C.RED, f"{C.RED}✗{C.RESET}", label, detail)


def step(label: str, detail: str = "") -> None:
    msg(C.CYAN, f"{C.CYAN}▶{C.RESET}", label, detail)


def stop(label: str, detail: str = "") -> None:
    msg(C.YELLOW, f"{C.YELLOW}■{C.RESET}", label, detail)


def section(title: str) -> None:
    pad = max(0, 56 - len(title))
    _w(f"\n{C.BOLD}{C.WHITE}  ── {title} {C.DIM}{'─' * pad}{C.RESET}\n")


def banner(title: str) -> None:
    bar = "═" * 66
    _w(f"\n{C.BOLD}{C.BLUE}╔{bar}╗\n")
    _w(f"{C.BOLD}{C.BLUE}║  {C.WHITE}{title:<64}{C.BLUE}║\n")
    _w(f"{C.BOLD}{C.BLUE}╚{bar}╝{C.RESET}\n")


def seq_banner(seq_name: str, idx: int, total: int) -> None:
    _w(f"\n{C.BOLD}{C.MAGENTA}  {'━' * 66}\n")
    _w(f"  Sequence {idx}/{total}: {C.WHITE}{seq_name}{C.RESET}\n")
    _w(f"{C.BOLD}{C.MAGENTA}  {'━' * 66}{C.RESET}\n")


def run_banner(run_num: int, n_runs: int) -> None:
    _w(f"\n{C.BOLD}{C.CYAN}  {'─' * 66}\n")
    _w(f"  Run {run_num}/{n_runs}\n")
    _w(f"  {'─' * 66}{C.RESET}\n")


def launch_line(line: str) -> None:
    s = line.rstrip()
    if not s:
        return
    if "ERROR" in s or "FATAL" in s or "error" in s:
        colour = C.RED
    elif "WARN" in s or "warn" in s:
        colour = C.YELLOW
    else:
        colour = C.DIM + C.WHITE
    _w(f"  {C.MAGENTA}│{C.RESET}  {colour}{s}{C.RESET}\n")


def evo_line(line: str) -> None:
    s = line.rstrip()
    if s:
        _w(f"  {C.CYAN}│{C.RESET}  {C.CYAN}{s}{C.RESET}\n")


def q(value: object) -> str:
    return shlex.quote(str(value))


# ─── signal / cleanup ─────────────────────────────────────────────────────────


def _reset_state() -> None:
    global _all_procs
    _all_procs = []


def _sigint_proc(proc: subprocess.Popen | None) -> None:  # type: ignore[type-arg]
    if proc is None:
        return
    try:
        os.killpg(proc.pid, signal.SIGINT)
    except Exception:
        pass


def _wait_for_proc(proc: subprocess.Popen | None, timeout_sec: float) -> bool:  # type: ignore[type-arg]
    if proc is None:
        return True
    deadline = time.time() + max(0.0, timeout_sec)
    while time.time() < deadline:
        if proc.poll() is not None:
            return True
        time.sleep(0.2)
    return proc.poll() is not None


def nuke(reason: str = "") -> None:
    if reason:
        _w(f"\n  {C.YELLOW}⚡  {C.BOLD}{reason}{C.RESET}\n")
    for proc in _all_procs:
        try:
            os.killpg(proc.pid, signal.SIGKILL)
        except Exception:
            pass
    for pat in [
        "ros2 bag play",
        "ov2slam_node",
        "ros2 run ov2slam",
    ]:
        subprocess.call(["pkill", "-9", "-f", pat], stderr=subprocess.DEVNULL)
    _w(f"  {C.YELLOW}■  All processes killed.{C.RESET}\n")


def _sigint(signum: int, frame: object) -> None:  # type: ignore[type-arg]
    nuke("CTRL+C — killing everything")
    if _log_fd:
        try:
            _log_fd.close()
        except Exception:
            pass
    sys.exit(130)


def in_container() -> bool:
    return Path("/.dockerenv").exists()


# ─── path helpers ─────────────────────────────────────────────────────────────


def resolve_path(root: Path, raw_path: str | Path) -> Path:
    p = Path(raw_path)
    if p.is_absolute():
        return p
    return (root / p).resolve()


# ─── shell / subprocess helpers ───────────────────────────────────────────────


def build_shell_script(cmd: str, setup_scripts: list[str], ws: Path) -> str:
    lines = [
        "set -e",
        'source_safe(){ if [ -f "$1" ]; then set +u; . "$1"; set -u; fi; }',
    ]
    opencv_lib = ws / "opencv" / "build" / "lib"
    if opencv_lib.is_dir():
        lines.append(
            f"export LD_LIBRARY_PATH={q(opencv_lib)}:${{LD_LIBRARY_PATH:-}}"
        )
    lines.append("export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp")
    for script in setup_scripts:
        path = Path(script)
        if not path.is_absolute():
            path = resolve_path(ws, script)
        lines.append(f"source_safe {q(path)}")
    lines.append(cmd)
    return "\n".join(lines)


def _bg(
    cmd: str,
    setup_scripts: list[str],
    ws: Path,
    log_path: Path | None = None,
    line_fn=None,
    cwd: Path | None = None,
) -> subprocess.Popen:  # type: ignore[type-arg]
    script = build_shell_script(cmd, setup_scripts, ws)
    proc_cwd = str(cwd) if cwd else None
    if line_fn is not None:
        log_fd = open(log_path, "w", buffering=1) if log_path else None
        proc = subprocess.Popen(
            ["bash", "-lc", script],
            cwd=proc_cwd,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
            start_new_session=True,
        )

        def _worker() -> None:
            assert proc.stdout is not None
            for line in proc.stdout:
                if log_fd:
                    log_fd.write(line)
                    log_fd.flush()
                line_fn(line)
            if log_fd:
                log_fd.close()

        threading.Thread(target=_worker, daemon=True).start()
    else:
        out = open(log_path, "w", buffering=1) if log_path else subprocess.DEVNULL
        proc = subprocess.Popen(
            ["bash", "-lc", script],
            cwd=proc_cwd,
            stdout=out,
            stderr=out,
            start_new_session=True,
        )
    _all_procs.append(proc)
    return proc


def _run(
    cmd: str,
    setup_scripts: list[str],
    ws: Path,
    cwd: Path | None = None,
    capture_output: bool = True,
) -> subprocess.CompletedProcess:  # type: ignore[type-arg]
    return subprocess.run(
        ["bash", "-lc", build_shell_script(cmd, setup_scripts, ws)],
        cwd=str(cwd) if cwd else None,
        capture_output=capture_output,
        text=True,
    )


# ─── config ───────────────────────────────────────────────────────────────────

DEFAULT_CONFIG: dict = {
    "experiment": {
        "mode": "stereo",
        "profile": "accurate",
        "trajectory_policy": "paper",
    },
    "dataset": {
        "workspace_root": "",
        "dataset_root": "",
        "sequences": ["MH_01_easy"],
        "seq_range_start": "",
        "seq_range_end": "",
    },
    "ov2slam": {
        "params_file": "",
        "node_name": "/ov2slam_node",
        "node_wait_timeout": 45,
        "cores": "",
        "setup_scripts": [
            "/opt/ros/jazzy/setup.bash",
        ],
    },
    "playback": {
        "bag_rate": 1.0,
        "bag_timeout": 1200,
        "startup_settle_sec": 15,
        "auto_exit_timeout": 600,
        "final_clock_sec": 1450000000,
        "final_clock_nanosec": 0,
        "clock_start_timeout": 10,
    },
    "evaluation": {
        "n_runs": 3,
        "retries": 3,
        "results_dir": "",
        "results_name": "",
        "log_dir": "",
        "t_max_diff": 0.1,
        "monitor_sample_sec": 0.05,
        "inter_run_delay_sec": 10,
        "sigint_timeout": 30,
    },
}


def load_config(path: Path | None) -> dict:
    import copy

    cfg: dict = {}
    if path and path.exists():
        with open(path) as fh:
            cfg = yaml.safe_load(fh) or {}
    elif path:
        fail("Config file not found", str(path))
        sys.exit(1)

    merged = copy.deepcopy(DEFAULT_CONFIG)
    for section_key, defaults in DEFAULT_CONFIG.items():
        if section_key not in cfg:
            continue
        if isinstance(defaults, dict) and isinstance(cfg[section_key], dict):
            merged[section_key].update(cfg[section_key])
        else:
            merged[section_key] = cfg[section_key]
    return merged


# ─── sequence discovery ───────────────────────────────────────────────────────


def find_sequences(cfg: dict, ws: Path) -> list[tuple[str, Path, Path]]:
    dataset_cfg = cfg["dataset"]
    workspace_root = _resolve_workspace(cfg, ws)

    dataset_root_raw = dataset_cfg.get("dataset_root", "")
    dataset_root = (
        Path(dataset_root_raw).expanduser().resolve()
        if dataset_root_raw
        else workspace_root / "datasets" / "euroc"
    )

    seqs_cfg = dataset_cfg.get("sequences", [])
    seq_range_start = str(dataset_cfg.get("seq_range_start", "") or "")
    seq_range_end = str(dataset_cfg.get("seq_range_end", "") or "")

    # Discover sequences (dirs containing at least one .db3)
    discovered: list[Path] = []
    if dataset_root.exists():
        for entry in sorted(dataset_root.iterdir()):
            if entry.is_dir() and list(entry.glob("*.db3")):
                discovered.append(entry)

    if not discovered:
        fail("No EuRoC bags found in dataset_root", str(dataset_root))
        sys.exit(1)

    by_name = {p.name: p for p in discovered}
    ordered = [p.name for p in discovered]

    # Select
    if seqs_cfg and seqs_cfg != "all":
        names: list[str] = list(seqs_cfg)
        for name in names:
            if name not in by_name:
                fail(f"Unknown sequence: {name}", f"not found under {dataset_root}")
                sys.exit(1)
        selected_dirs = [by_name[name] for name in names]
    elif seq_range_start and seq_range_end:
        if seq_range_start not in by_name or seq_range_end not in by_name:
            fail(
                "Sequence range endpoints not found",
                f"{seq_range_start} → {seq_range_end}",
            )
            sys.exit(1)
        start_idx = ordered.index(seq_range_start)
        end_idx = ordered.index(seq_range_end)
        if start_idx > end_idx:
            fail("seq_range_start appears after seq_range_end")
            sys.exit(1)
        selected_dirs = [by_name[n] for n in ordered[start_idx : end_idx + 1]]
    else:
        selected_dirs = discovered

    results: list[tuple[str, Path, Path]] = []
    for seq_dir in selected_dirs:
        seq_name = seq_dir.name

        # Bag target
        db3_files = sorted(seq_dir.glob("*.db3"))
        if not db3_files:
            stop(f"Skipping {seq_name}", "no .db3 file found")
            continue
        if len(db3_files) == 1:
            bag_path: Path = db3_files[0]
        elif (seq_dir / "metadata.yaml").is_file():
            bag_path = seq_dir
        else:
            stop(f"Skipping {seq_name}", "split bag without metadata.yaml")
            continue

        # Ground truth
        gt_path: Path | None = None
        if (seq_dir / "gt.tum").is_file():
            gt_path = seq_dir / "gt.tum"
        elif (seq_dir / f"{seq_name}.txt").is_file():
            gt_path = seq_dir / f"{seq_name}.txt"
        else:
            txt_candidates = [
                p for p in sorted(seq_dir.glob("*.txt")) if p.name != "metadata.txt"
            ]
            if len(txt_candidates) == 1:
                gt_path = txt_candidates[0]

        if gt_path is None:
            stop(f"Skipping {seq_name}", "no ground truth file found")
            continue

        results.append((seq_name, bag_path, gt_path))

    return results


# ─── mode / profile ───────────────────────────────────────────────────────────


def _resolve_workspace(cfg: dict, ws: Path) -> Path:
    raw = str(cfg.get("dataset", {}).get("workspace_root", "") or "")
    return Path(raw).expanduser().resolve() if raw else ws


def mode_profile(cfg: dict, ws: Path) -> dict:
    exp_cfg = cfg.get("experiment", {})
    ov2_cfg = cfg.get("ov2slam", {})
    workspace_root = _resolve_workspace(cfg, ws)

    mode = str(exp_cfg.get("mode", "stereo")).strip().lower()
    if mode not in {"mono", "stereo"}:
        fail("Unsupported experiment.mode", f"{mode!r}  (expected mono or stereo)")
        sys.exit(1)

    profile = str(exp_cfg.get("profile", "accurate")).strip().lower()
    if profile not in {"accurate", "average", "fast"}:
        fail(
            "Unsupported experiment.profile",
            f"{profile!r}  (expected accurate, average, or fast)",
        )
        sys.exit(1)

    trajectory_policy = str(exp_cfg.get("trajectory_policy", "paper")).strip().lower()
    if trajectory_policy not in {"paper", "best"}:
        fail(
            "Unsupported trajectory_policy",
            f"{trajectory_policy!r}  (expected paper or best)",
        )
        sys.exit(1)

    setup_scripts: list[str] = list(
        ov2_cfg.get("setup_scripts", ["/opt/ros/jazzy/setup.bash"])
    )
    # Auto-append workspace install/setup.bash if present and not already listed
    install_setup = workspace_root / "install" / "setup.bash"
    if install_setup.is_file() and str(install_setup) not in setup_scripts:
        setup_scripts.append(str(install_setup))

    params_file_raw = str(ov2_cfg.get("params_file", "") or "")
    if params_file_raw:
        params_file = Path(params_file_raw).expanduser().resolve()
    else:
        params_file = (
            workspace_root
            / "src"
            / "ov2slam_ros"
            / "parameters_files"
            / profile
            / "euroc"
            / f"euroc_{mode}.yaml"
        )

    node_name = str(ov2_cfg.get("node_name", "/ov2slam_node"))
    node_wait_timeout = float(ov2_cfg.get("node_wait_timeout", 45))
    cores = str(ov2_cfg.get("cores", "") or "").strip()

    return {
        "mode": mode,
        "profile": profile,
        "trajectory_policy": trajectory_policy,
        "setup_scripts": setup_scripts,
        "params_file": params_file,
        "node_name": node_name,
        "node_wait_timeout": node_wait_timeout,
        "cores": cores,
        "workspace_root": workspace_root,
        "scale_correction": (mode == "mono"),
    }


# ─── preflight ────────────────────────────────────────────────────────────────


def preflight_checks(mode_cfg: dict, ws: Path, dataset_root: Path) -> None:
    section("Preflight")
    ok("Workspace root", str(mode_cfg["workspace_root"]))
    ok("Dataset root", str(dataset_root))
    ok("Params file", str(mode_cfg["params_file"]))

    if not mode_cfg["params_file"].is_file():
        fail("Params file not found", str(mode_cfg["params_file"]))
        sys.exit(1)

    if not dataset_root.is_dir():
        fail("Dataset root not found", str(dataset_root))
        sys.exit(1)

    for script in mode_cfg["setup_scripts"]:
        path = Path(script)
        if path.exists():
            ok("Setup script", str(path))
        else:
            stop("Setup script missing", str(path))

    required_commands = ["ros2", "evo_ape", "evo_traj", "timeout"]
    if mode_cfg["cores"]:
        required_commands.append("taskset")

    found_commands: dict[str, str] = {}
    missing_commands: list[str] = []
    for command_name in required_commands:
        check = _run(
            f"command -v {q(command_name)}",
            mode_cfg["setup_scripts"],
            mode_cfg["workspace_root"],
        )
        path_str = (check.stdout or "").strip()
        if check.returncode == 0 and path_str:
            found_commands[command_name] = path_str
        else:
            missing_commands.append(command_name)

    if missing_commands:
        for command_name, path_str in found_commands.items():
            ok(f"Required command: {command_name}", path_str)
        fail(
            "Required commands missing",
            f"missing={', '.join(missing_commands)}"
            + (
                f"  found={', '.join(f'{n}={p}' for n, p in found_commands.items())}"
                if found_commands
                else ""
            ),
        )
        sys.exit(1)

    ok(
        "Required commands",
        ", ".join(f"{n}={p}" for n, p in found_commands.items()),
    )

    matplotlib_check = _run(
        'python3 -c "import matplotlib, numpy"',
        mode_cfg["setup_scripts"],
        mode_cfg["workspace_root"],
    )
    if matplotlib_check.returncode != 0:
        fail(
            "Python dependency missing",
            "matplotlib and numpy are required for CPU plot generation",
        )
        sys.exit(1)
    ok("Python dependencies", "matplotlib, numpy")


# ─── /proc-based process + thread CPU monitor ─────────────────────────────────
#
# Algorithm faithfully replicates BENCH.sh's embedded Python inline monitor
# and ~/ORB_SLAM2/tools/monitor_cpu_orb.py.
#
# Key points:
#   1. Given launcher_pid (the setsid bash process that ran ros2 run …)
#   2. Walk /proc to build a parent→children map, DFS from launcher_pid
#   3. Among descendants, look for ov2slam_node via /proc/<pid>/comm and
#      readlink /proc/<pid>/exe  (exact match, preferred)
#   4. Fall back to checking /proc/<pid>/cmdline tokens for ov2slam_node
#   5. After finding actual_pid, read /proc/<actual_pid>/task/<tid>/stat
#      for per-thread CPU: delta_ticks / (delta_wall_sec × SC_CLK_TCK) × 100
# ─────────────────────────────────────────────────────────────────────────────


def _pid_exists(pid: int) -> bool:
    return Path(f"/proc/{pid}").exists()


def _read_comm(path: Path) -> str:
    try:
        return path.read_text().strip()
    except Exception:
        return ""


def _read_cmdline(pid: int) -> str:
    try:
        raw = Path(f"/proc/{pid}/cmdline").read_bytes()
        return raw.replace(b"\x00", b" ").decode("utf-8", "replace").strip()
    except Exception:
        return ""


def _read_exe(pid: int) -> str:
    try:
        return os.path.basename(os.readlink(f"/proc/{pid}/exe"))
    except Exception:
        return ""


def _parse_stat(path: Path) -> tuple[int, int, int]:
    """Return (utime+stime, last_cpu_core, ppid)."""
    raw = path.read_text().strip()
    rparen = raw.rfind(")")
    rest = raw[rparen + 2 :].split()
    utime = int(rest[11])
    stime = int(rest[12])
    processor = int(rest[36])
    ppid = int(rest[1])
    return utime + stime, processor, ppid


def _collect_descendants(root_pid: int) -> list[int]:
    children: dict[int, list[int]] = defaultdict(list)
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit():
            continue
        try:
            _total, _cpu, ppid = _parse_stat(entry / "stat")
        except Exception:
            continue
        children[ppid].append(int(entry.name))
    out: list[int] = []
    stack = [root_pid]
    seen: set[int] = set()
    while stack:
        pid = stack.pop()
        if pid in seen:
            continue
        seen.add(pid)
        for child in children.get(pid, []):
            out.append(child)
            stack.append(child)
    return out


def _cmdline_has_ov2slam_node(cmdline: str) -> bool:
    for tok in [t for t in cmdline.split() if t]:
        if os.path.basename(tok) == "ov2slam_node":
            return True
    return False


def _is_exact_ov2slam_process(pid: int) -> bool:
    comm = _read_comm(Path(f"/proc/{pid}/comm"))
    exe = _read_exe(pid)
    return comm == "ov2slam_node" or exe == "ov2slam_node"


def _is_wrapped_ov2slam_process(pid: int) -> bool:
    if _is_exact_ov2slam_process(pid):
        return True
    return _cmdline_has_ov2slam_node(_read_cmdline(pid))


def _find_best_target_pid(root_pid: int) -> tuple[int | None, str]:
    if not _pid_exists(root_pid):
        return None, "launcher_gone"
    descendants = _collect_descendants(root_pid)

    for pid in descendants:
        if _pid_exists(pid) and _is_exact_ov2slam_process(pid):
            return pid, "resolved_descendant_exact"

    if _pid_exists(root_pid) and _is_exact_ov2slam_process(root_pid):
        return root_pid, "resolved_root_exact"

    for pid in descendants:
        if _pid_exists(pid) and _is_wrapped_ov2slam_process(pid):
            return pid, "resolved_descendant_wrapped"

    if _pid_exists(root_pid) and _is_wrapped_ov2slam_process(root_pid):
        root_comm = _read_comm(Path(f"/proc/{root_pid}/comm"))
        root_exe = _read_exe(root_pid)
        if root_comm == "ov2slam_node" or root_exe == "ov2slam_node":
            return root_pid, "resolved_root_wrapped"

    return None, "not_found_under_launcher"


def _resolve_actual_pid(
    root_pid: int,
    timeout_s: float = 30.0,
    prefer_exact_grace_s: float = 12.0,
) -> tuple[int | None, str]:
    """
    Wait up to timeout_s for ov2slam_node to appear under launcher_pid.
    Prefer exact matches; fall back to wrapped matches after prefer_exact_grace_s.
    Mirrors the resolve_actual_pid in BENCH.sh / monitor_cpu_orb.py.
    """
    deadline = time.monotonic() + timeout_s
    exact_deadline = min(deadline, time.monotonic() + prefer_exact_grace_s)
    fallback_pid: int | None = None
    fallback_resolution = ""

    while time.monotonic() < deadline:
        pid, resolution = _find_best_target_pid(root_pid)
        if pid is not None:
            if resolution.endswith("_exact"):
                return pid, resolution
            fallback_pid = pid
            fallback_resolution = resolution
            if time.monotonic() >= exact_deadline and _pid_exists(fallback_pid):
                return fallback_pid, f"{fallback_resolution}_after_exact_wait"
        elif resolution == "launcher_gone":
            return None, "launcher_gone"
        time.sleep(0.2)

    if fallback_pid is not None and _pid_exists(fallback_pid):
        return fallback_pid, f"{fallback_resolution}_timeout_fallback"
    if _pid_exists(root_pid):
        return None, "not_found_under_launcher"
    return None, "not_found"


class OV2SlamMonitor:
    """
    Background process + per-thread CPU sampler for ov2slam_node.

    Replicates the embedded Python monitor from BENCH.sh and
    ~/ORB_SLAM2/tools/monitor_cpu_orb.py, packaged as a threading.Thread
    so it can be started/stopped from the main run loop.

    Outputs written to run_dir/:
      ov2slam_pid_info.json
      ov2slam_process_cpu.csv
      ov2slam_thread_cpu.csv
      ov2slam_thread_cpu_summary.csv
    """

    def __init__(
        self,
        launcher_pid: int,
        run_dir: Path,
        sample_sec: float = 0.05,
    ) -> None:
        self.launcher_pid = launcher_pid
        self.run_dir = run_dir
        self.sample_sec = sample_sec
        self._stop_event = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self.hz: int = os.sysconf(os.sysconf_names["SC_CLK_TCK"])

    # ── file paths ──────────────────────────────────────────────────────────

    @property
    def info_json(self) -> Path:
        return self.run_dir / "ov2slam_pid_info.json"

    @property
    def process_csv(self) -> Path:
        return self.run_dir / "ov2slam_process_cpu.csv"

    @property
    def thread_csv(self) -> Path:
        return self.run_dir / "ov2slam_thread_cpu.csv"

    @property
    def summary_csv(self) -> Path:
        return self.run_dir / "ov2slam_thread_cpu_summary.csv"

    # ── lifecycle ───────────────────────────────────────────────────────────

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        self._thread.join(timeout=10.0)

    # ── internals ───────────────────────────────────────────────────────────

    def _write_empty_outputs(self, resolution: str) -> None:
        info = {
            "launcher_pid": self.launcher_pid,
            "ov2slam_pid": None,
            "resolution": resolution,
        }
        self.info_json.write_text(json.dumps(info, indent=2) + "\n")
        self.process_csv.write_text(
            "wall_time_unix,elapsed_s,launcher_pid,ov2slam_pid,last_cpu,process_cpu_percent\n"
        )
        self.thread_csv.write_text(
            "wall_time_unix,elapsed_s,launcher_pid,ov2slam_pid,tid,comm,last_cpu,thread_cpu_percent\n"
        )
        self.summary_csv.write_text(
            "tid,comm,samples,mean_cpu_percent,max_cpu_percent,dominant_core,last_cpu\n"
        )

    def _run(self) -> None:
        actual_pid, resolution = _resolve_actual_pid(self.launcher_pid)

        info: dict = {
            "launcher_pid": self.launcher_pid,
            "ov2slam_pid": actual_pid,
            "resolution": resolution,
        }
        if actual_pid is not None and _pid_exists(actual_pid):
            info["ov2slam_comm"] = _read_comm(Path(f"/proc/{actual_pid}/comm"))
            info["ov2slam_exe"] = _read_exe(actual_pid)
            info["ov2slam_cmdline"] = _read_cmdline(actual_pid)
        self.info_json.write_text(json.dumps(info, indent=2) + "\n")

        if actual_pid is None or not _pid_exists(actual_pid):
            self._write_empty_outputs(resolution)
            return

        summary: dict[int, dict] = {}

        with (
            self.process_csv.open("w", newline="") as proc_f,
            self.thread_csv.open("w", newline="") as thr_f,
        ):
            proc_writer = csv.writer(proc_f)
            thr_writer = csv.writer(thr_f)
            proc_writer.writerow(
                [
                    "wall_time_unix",
                    "elapsed_s",
                    "launcher_pid",
                    "ov2slam_pid",
                    "last_cpu",
                    "process_cpu_percent",
                ]
            )
            thr_writer.writerow(
                [
                    "wall_time_unix",
                    "elapsed_s",
                    "launcher_pid",
                    "ov2slam_pid",
                    "tid",
                    "comm",
                    "last_cpu",
                    "thread_cpu_percent",
                ]
            )

            start_wall = time.time()
            prev_sample_wall = time.monotonic()
            prev_proc_total, _prev_proc_cpu, _ = _parse_stat(
                Path(f"/proc/{actual_pid}/stat")
            )
            prev_threads: dict[int, tuple[int, int]] = {}
            task_dir = Path(f"/proc/{actual_pid}/task")
            if task_dir.exists():
                for task in task_dir.iterdir():
                    try:
                        tid = int(task.name)
                        total, cpu, _ = _parse_stat(task / "stat")
                        prev_threads[tid] = (total, cpu)
                    except Exception:
                        continue

            while _pid_exists(actual_pid) and not self._stop_event.is_set():
                time.sleep(self.sample_sec)
                now_wall = time.time()
                now_mono = time.monotonic()
                delta_t = now_mono - prev_sample_wall
                if delta_t <= 0:
                    continue
                prev_sample_wall = now_mono

                try:
                    proc_total, proc_cpu, _ = _parse_stat(
                        Path(f"/proc/{actual_pid}/stat")
                    )
                except Exception:
                    break

                proc_cpu_pct = (
                    100.0 * (proc_total - prev_proc_total) / (delta_t * self.hz)
                )
                prev_proc_total = proc_total
                elapsed_s = now_wall - start_wall
                proc_writer.writerow(
                    [
                        f"{now_wall:.3f}",
                        f"{elapsed_s:.3f}",
                        self.launcher_pid,
                        actual_pid,
                        proc_cpu,
                        f"{proc_cpu_pct:.3f}",
                    ]
                )
                proc_f.flush()

                current_threads: dict[int, tuple[int, int]] = {}
                if not task_dir.exists():
                    break
                for task in task_dir.iterdir():
                    try:
                        tid = int(task.name)
                        total, last_cpu, _ = _parse_stat(task / "stat")
                        comm = _read_comm(task / "comm")
                    except Exception:
                        continue

                    prev_total, _prev_cpu = prev_threads.get(tid, (total, last_cpu))
                    thread_cpu_pct = (
                        100.0 * (total - prev_total) / (delta_t * self.hz)
                    )
                    thr_writer.writerow(
                        [
                            f"{now_wall:.3f}",
                            f"{elapsed_s:.3f}",
                            self.launcher_pid,
                            actual_pid,
                            tid,
                            comm,
                            last_cpu,
                            f"{thread_cpu_pct:.3f}",
                        ]
                    )
                    current_threads[tid] = (total, last_cpu)

                    stats = summary.setdefault(
                        tid,
                        {
                            "comm": comm,
                            "samples": 0,
                            "sum_cpu": 0.0,
                            "max_cpu": 0.0,
                            "core_counts": Counter(),
                            "last_cpu": last_cpu,
                        },
                    )
                    stats["comm"] = comm
                    stats["samples"] += 1
                    stats["sum_cpu"] += thread_cpu_pct
                    stats["max_cpu"] = max(stats["max_cpu"], thread_cpu_pct)
                    stats["core_counts"][last_cpu] += 1
                    stats["last_cpu"] = last_cpu

                thr_f.flush()
                prev_threads = current_threads

        with self.summary_csv.open("w", newline="") as sf:
            writer = csv.writer(sf)
            writer.writerow(
                [
                    "tid",
                    "comm",
                    "samples",
                    "mean_cpu_percent",
                    "max_cpu_percent",
                    "dominant_core",
                    "last_cpu",
                ]
            )
            for tid, st in sorted(
                summary.items(), key=lambda item: (-item[1]["sum_cpu"], item[0])
            ):
                dominant_core = (
                    st["core_counts"].most_common(1)[0][0] if st["core_counts"] else ""
                )
                mean_cpu = st["sum_cpu"] / st["samples"] if st["samples"] else 0.0
                writer.writerow(
                    [
                        tid,
                        st["comm"],
                        st["samples"],
                        f"{mean_cpu:.3f}",
                        f"{st['max_cpu']:.3f}",
                        dominant_core,
                        st["last_cpu"],
                    ]
                )


# ─── CPU plots (from BENCH.sh generate_cpu_plots) ─────────────────────────────


def generate_cpu_plots(run_dir: Path, seq_name: str) -> bool:
    process_csv = run_dir / "ov2slam_process_cpu.csv"
    thread_summary_csv = run_dir / "ov2slam_thread_cpu_summary.csv"
    info_json = run_dir / "ov2slam_pid_info.json"
    process_plot = run_dir / "ov2slam_cpu_usage.png"
    thread_bar_plot = run_dir / "ov2slam_threads_cpu_bar.png"

    if not process_csv.exists() or not thread_summary_csv.exists():
        return False

    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception:
        return False

    info: dict = {}
    if info_json.exists():
        try:
            info = json.loads(info_json.read_text())
        except Exception:
            pass

    process_rows: list[tuple[float, float]] = []
    with process_csv.open() as fh:
        for row in csv.DictReader(fh):
            try:
                process_rows.append(
                    (float(row["elapsed_s"]), float(row["process_cpu_percent"]))
                )
            except Exception:
                continue

    if not process_rows:
        return False

    thread_rows: list[tuple[str, int, float, float, str]] = []
    with thread_summary_csv.open() as fh:
        for row in csv.DictReader(fh):
            try:
                thread_rows.append(
                    (
                        row["comm"],
                        int(row["tid"]),
                        float(row["mean_cpu_percent"]),
                        float(row["max_cpu_percent"]),
                        row["dominant_core"],
                    )
                )
            except Exception:
                continue
    thread_rows.sort(key=lambda item: (-item[2], item[1]))

    title = seq_name
    if info.get("ov2slam_pid") is not None:
        title = f"{seq_name} OV2SLAM PID {info['ov2slam_pid']}"

    fig, ax = plt.subplots(figsize=(8, 4.5), dpi=160)
    ax.plot(
        [r[0] for r in process_rows],
        [r[1] for r in process_rows],
        color="#2c7fb8",
        linewidth=1.5,
    )
    ax.set_xlabel("time since monitor start (s)")
    ax.set_ylabel("OV2SLAM CPU usage (%)")
    ax.set_ylim(0, 400)
    ax.set_title(title)
    ax.grid(True, alpha=0.35)
    fig.tight_layout()
    fig.savefig(process_plot, bbox_inches="tight")
    plt.close(fig)

    if thread_rows:
        labels: list[str] = []
        values: list[float] = []
        colors: list = []
        palette = plt.get_cmap("tab20")
        for idx, (comm, tid, mean_cpu, _max_cpu, dominant_core) in enumerate(
            thread_rows
        ):
            core_label = dominant_core if dominant_core != "" else "?"
            labels.append(f"{comm} [{tid}] @ CPU {core_label}")
            values.append(mean_cpu)
            try:
                color_idx = int(core_label) % 20
            except Exception:
                color_idx = idx % 20
            colors.append(palette(color_idx))

        width = max(10, 0.65 * len(labels) + 3)
        fig, ax = plt.subplots(figsize=(width, 5.5), dpi=160)
        x = list(range(len(labels)))
        bars = ax.bar(x, values, color=colors)
        ax.set_xticks(x)
        ax.set_xticklabels(labels, rotation=45, ha="right")
        ax.set_ylabel("mean CPU usage (%)")
        ax.set_ylim(0, 400)
        ax.set_title(f"{seq_name} OV2SLAM thread CPU usage")
        ax.grid(True, axis="y", alpha=0.35)
        ax.bar_label(
            bars,
            labels=[f"{v:.1f}" for v in values],
            padding=3,
            fontsize=8,
            rotation=90,
        )
        fig.tight_layout()
        fig.savefig(thread_bar_plot, bbox_inches="tight")
        plt.close(fig)

    return True


# ─── ground truth ─────────────────────────────────────────────────────────────


def prepare_ground_truth(seq_name: str, gt_source: Path, run_dir: Path) -> Path:
    """Validate TUM format and write a clean copy to run_dir/gt.tum."""
    gt_out = run_dir / "gt.tum"
    rows: list[str] = []
    for lineno, raw in enumerate(
        gt_source.read_text(encoding="utf-8").splitlines(), start=1
    ):
        line = raw.strip()
        if not line or line.startswith("#"):
            continue
        fields = line.replace(",", " ").split()
        if len(fields) != 8:
            raise ValueError(
                f"{gt_source}:{lineno}: expected 8 columns, got {len(fields)}"
            )
        for field in fields:
            float(field)  # validate numeric
        rows.append(" ".join(fields))
    if not rows:
        raise ValueError(f"No pose rows found in {gt_source}")
    gt_out.write_text("\n".join(rows) + "\n", encoding="utf-8")
    return gt_out


# ─── trajectory selection ─────────────────────────────────────────────────────


def select_estimate_trajectory(
    run_dir: Path, policy: str
) -> tuple[Path, str, str]:
    candidates = (
        PAPER_TRAJECTORY_CANDIDATES if policy == "paper" else BEST_TRAJECTORY_CANDIDATES
    )
    for name, reason in candidates:
        path = run_dir / name
        if path.is_file() and path.stat().st_size > 64:
            return path, name, reason
    found_unsupported = [
        name
        for name in UNSUPPORTED_EVO_TRAJECTORIES
        if (run_dir / name).exists()
    ]
    if found_unsupported:
        raise RuntimeError(
            "Only unsupported loop-closure full trajectories found: "
            + ", ".join(found_unsupported)
            + ".  Column 1 is a frame index, not a ROS timestamp — "
            "evo_ape tum cannot use these."
        )
    raise RuntimeError(f"No supported OV2SLAM trajectory found in {run_dir}")


# ─── OV2SLAM timings CSV ──────────────────────────────────────────────────────


def load_ov2_timings(path: Path) -> dict[str, float]:
    if not path.exists():
        return {}
    timings: dict[str, float] = {}
    with path.open(newline="", encoding="utf-8") as fh:
        for row in csv.DictReader(fh):
            key = (row.get("timer") or "").strip()
            try:
                value = float(row.get("mean_ms", ""))
            except Exception:
                continue
            if key:
                timings[key] = value
    return timings


# ─── EVO evaluation (style from benchmark_euroc.py) ──────────────────────────


def parse_evo_stats(text: str) -> dict:
    stats: dict = {}
    for metric in ("rmse", "mean", "median", "std", "max", "min", "sse"):
        match = re.search(
            rf"^\s*{metric}\s+([\d.eE+\-]+)", text, re.MULTILINE
        )
        if match:
            stats[metric] = float(match.group(1))
    match = re.search(r"Compared\s+(\d+)\s+absolute pose pairs", text)
    if match:
        stats["n_matched"] = int(match.group(1))
    else:
        match = re.search(r"Found\s+(\d+)\s+of max\.", text)
        if match:
            stats["n_matched"] = int(match.group(1))
    return stats


def _count_poses(tum_path: Path) -> int:
    try:
        return sum(
            1
            for line in tum_path.read_text().splitlines()
            if line.strip() and not line.startswith("#")
        )
    except Exception:
        return 0


def run_evo(
    seq_name: str,
    tum_path: Path,
    gt_path: Path,
    eval_cfg: dict,
    mode_cfg: dict,
    run_dir: Path,
) -> dict:
    t_max = float(eval_cfg.get("t_max_diff", 0.1))
    ape_plot = run_dir / "ape_traj.png"
    traj_plot = run_dir / "traj_xy.png"
    total_poses = _count_poses(tum_path)

    ape_flags = "-a -s" if mode_cfg["scale_correction"] else "-a"
    traj_flags = "-a --correct_scale" if mode_cfg["scale_correction"] else "-a"

    section(f"evo_ape  [{seq_name}]")
    step(
        "evo_ape",
        f"gt={gt_path.name}  est={tum_path.name}  "
        f"total_poses={total_poses}  mode={mode_cfg['mode']}",
    )

    ape_cmd = (
        f"evo_ape tum {q(gt_path)} {q(tum_path)} "
        f"{ape_flags} --t_max_diff {t_max} --verbose "
        f"--plot_mode xy --save_plot {q(ape_plot)}"
    )
    evo_txt = run_dir / f"{seq_name}_evo.txt"
    evo_log_fd = open(evo_txt, "w", buffering=1)
    raw_out = ""
    proc = subprocess.Popen(
        ["bash", "-lc", ape_cmd],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
    )
    assert proc.stdout is not None
    for line in proc.stdout:
        evo_log_fd.write(line)
        evo_log_fd.flush()
        evo_line(line)
        raw_out += line
    proc.wait()
    evo_log_fd.close()

    stats = parse_evo_stats(raw_out)
    stats["sequence"] = seq_name
    stats["mode"] = mode_cfg["mode"]
    stats["total_poses"] = total_poses

    n_matched = stats.get("n_matched", 0)
    match_ratio = (n_matched / total_poses) if total_poses > 0 else 0.0
    stats["match_ratio"] = round(match_ratio, 4)

    if proc.returncode == 0:
        rmse_str = f"{stats.get('rmse', float('nan')):.4f} m"
        match_str = (
            f"{n_matched}/{total_poses} matched ({match_ratio * 100:.1f}%)"
        )
        ok("evo_ape", f"RMSE = {rmse_str}  |  {match_str}")
        if ape_plot.exists():
            ok("APE plot", ape_plot.name)
    else:
        fail("evo_ape", f"rc={proc.returncode}")

    step("evo_traj", f"trajectory XY overlay → {traj_plot.name}")
    traj_cmd = (
        f"evo_traj tum {q(tum_path)} --ref {q(gt_path)} "
        f"{traj_flags} --plot_mode xy --save_plot {q(traj_plot)}"
    )
    result = subprocess.run(
        ["bash", "-lc", traj_cmd], capture_output=True, text=True
    )
    if result.returncode == 0:
        ok("evo_traj", str(traj_plot))
    else:
        fail("evo_traj", result.stderr.strip()[:160])

    return stats


# ─── helpers ──────────────────────────────────────────────────────────────────


def _mean_csv_column(path: Path, column: str) -> float | None:
    if not path.exists():
        return None
    import statistics

    values: list[float] = []
    with path.open(newline="", encoding="utf-8") as fh:
        for row in csv.DictReader(fh):
            try:
                values.append(float(row[column]))
            except Exception:
                continue
    return statistics.fmean(values) if values else None


def _wait_for_ros2_node(
    node_name: str, setup_scripts: list[str], ws: Path, timeout_s: float
) -> bool:
    expected = node_name.lstrip("/")
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        result = _run("ros2 node list", setup_scripts, ws)
        if result.returncode == 0:
            for raw in result.stdout.splitlines():
                if raw.strip().lstrip("/") == expected:
                    return True
        time.sleep(1.0)
    return False


def _wait_for_clock_message(
    setup_scripts: list[str], ws: Path, timeout_s: float
) -> bool:
    result = _run(
        f"timeout {timeout_s:.0f}s ros2 topic echo --once /clock > /dev/null 2>&1",
        setup_scripts,
        ws,
    )
    return result.returncode == 0


def _publish_final_clock(
    setup_scripts: list[str], ws: Path, sec: int, nanosec: int
) -> None:
    _run(
        f"ros2 topic pub --once /clock rosgraph_msgs/msg/Clock "
        f'"{{clock: {{sec: {sec}, nanosec: {nanosec}}}}}\" > /dev/null 2>&1 || true',
        setup_scripts,
        ws,
    )


def _wait_for_pid_exit(pid: int, timeout_s: float) -> bool:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if not _pid_exists(pid):
            return True
        time.sleep(1.0)
    return not _pid_exists(pid)


def _reset_run_dir(run_dir: Path) -> None:
    run_dir.mkdir(parents=True, exist_ok=True)
    for pattern in (
        "ov2slam.log",
        "bag_play.log",
        "gt.tum",
        "trajectory.tum",
        "ov2slam*.txt",
        "ov2slam*.csv",
        "*_evo.txt",
        "*_evo.zip",
        "*.png",
        "run_meta.json",
        "core.*",
    ):
        for path in run_dir.glob(pattern):
            if path.is_file():
                path.unlink()


# ─── run one sequence attempt ─────────────────────────────────────────────────


def run_sequence(
    seq_name: str,
    bag_path: Path,
    gt_source: Path,
    cfg: dict,
    ws: Path,
    mode_cfg: dict,
    run_dir: Path,
    run_num: int,
    n_runs: int,
) -> dict | None:
    _reset_state()
    signal.signal(signal.SIGINT, _sigint)
    signal.signal(signal.SIGTERM, _sigint)

    playback_cfg = cfg.get("playback", {})
    eval_cfg = cfg.get("evaluation", {})
    setup_scripts = mode_cfg["setup_scripts"]
    workspace_root: Path = mode_cfg["workspace_root"]

    bag_rate = float(playback_cfg.get("bag_rate", 1.0))
    bag_timeout = float(playback_cfg.get("bag_timeout", 1200))
    startup_settle_sec = float(playback_cfg.get("startup_settle_sec", 15))
    auto_exit_timeout = float(playback_cfg.get("auto_exit_timeout", 600))
    final_clock_sec = int(playback_cfg.get("final_clock_sec", 1450000000))
    final_clock_nanosec = int(playback_cfg.get("final_clock_nanosec", 0))
    clock_start_timeout = float(playback_cfg.get("clock_start_timeout", 10))
    sigint_timeout = float(eval_cfg.get("sigint_timeout", 30))
    monitor_sample_sec = float(eval_cfg.get("monitor_sample_sec", 0.05))

    node_name = mode_cfg["node_name"]
    node_wait_timeout = float(mode_cfg.get("node_wait_timeout", 45))
    trajectory_policy = mode_cfg["trajectory_policy"]

    _reset_run_dir(run_dir)

    def logf(name: str) -> Path:
        return run_dir / f"{name}.log"

    trajectory_out = run_dir / "trajectory.tum"
    warnings: list[str] = []
    seq_error = ""
    seq_traj_name = ""
    seq_traj_reason = ""
    seq_gt_verdict = ""
    gt_file: Path = run_dir / "gt.tum"   # placeholder; set properly below

    ov2_proc: subprocess.Popen | None = None  # type: ignore[type-arg]
    bag_proc: subprocess.Popen | None = None  # type: ignore[type-arg]
    monitor: OV2SlamMonitor | None = None
    wall_start: float = time.time()

    # ── Ground truth ──────────────────────────────────────────────────────────
    section(f"Ground Truth  [{seq_name}  run {run_num}/{n_runs}]")
    try:
        gt_file = prepare_ground_truth(seq_name, gt_source, run_dir)
        seq_gt_verdict = (
            f"{gt_source.name} is already valid TUM format "
            "(timestamp tx ty tz qx qy qz qw); normalized copy written to gt.tum"
        )
        ok("Ground truth", f"{gt_source.name} → gt.tum")
    except Exception as exc:
        seq_error = f"Ground truth preparation failed: {exc}"
        fail("Ground truth", seq_error)

    # ── OV2SLAM launch ────────────────────────────────────────────────────────
    if not seq_error:
        section(f"OV2SLAM Launch  [{seq_name}  run {run_num}/{n_runs}]")
        ov2_tokens = [
            "ros2",
            "run",
            "ov2slam",
            "ov2slam_node",
            str(mode_cfg["params_file"]),
            "--ros-args",
            "-p",
            "use_sim_time:=true",
        ]
        if mode_cfg["cores"]:
            ov2_tokens = ["taskset", "-c", mode_cfg["cores"]] + ov2_tokens
            step("Core pinning", mode_cfg["cores"])
        ov2_cmd = " ".join(q(tok) for tok in ov2_tokens)
        step("ros2 run ov2slam ov2slam_node", f"params={mode_cfg['params_file'].name}")
        ov2_proc = _bg(
            ov2_cmd,
            setup_scripts,
            workspace_root,
            logf("ov2slam"),
            line_fn=launch_line,
            cwd=run_dir,
        )
        ok("OV2SLAM launched", f"pid={ov2_proc.pid}")

        # ── CPU monitor ───────────────────────────────────────────────────────
        section(f"CPU Monitor  [{seq_name}  run {run_num}/{n_runs}]")
        step(
            "OV2SlamMonitor",
            f"sample={monitor_sample_sec:.2f}s  launcher_pid={ov2_proc.pid}",
        )
        monitor = OV2SlamMonitor(ov2_proc.pid, run_dir, monitor_sample_sec)
        monitor.start()
        ok("CPU monitor", f"launcher pid={ov2_proc.pid}")

        # ── Wait for node ─────────────────────────────────────────────────────
        section(f"Waiting for Node  [{seq_name}  run {run_num}/{n_runs}]")
        step("ros2 node list", f"waiting for {node_name}  (timeout {node_wait_timeout:.0f}s)")
        if not _wait_for_ros2_node(node_name, setup_scripts, workspace_root, node_wait_timeout):
            seq_error = (
                f"OV2SLAM node '{node_name}' did not appear within {node_wait_timeout}s."
            )
            fail("Node check", seq_error)
        elif ov2_proc.poll() is not None:
            seq_error = "OV2SLAM exited before bag playback started."
            fail("OV2SLAM", seq_error)
        else:
            ok("Node ready", node_name)
            _w(
                f"\n  {C.DIM}  sleeping {startup_settle_sec:.1f} s "
                f"for node to settle…{C.RESET}\n"
            )
            time.sleep(startup_settle_sec)
            if ov2_proc.poll() is not None:
                seq_error = (
                    f"OV2SLAM exited during the {startup_settle_sec}s "
                    "startup settle window."
                )
                fail("OV2SLAM", seq_error)

    # ── Bag playback ──────────────────────────────────────────────────────────
    if not seq_error:
        section(f"Bag Playback  [{seq_name}  run {run_num}/{n_runs}]")
        step("ros2 bag play", f"{bag_path.name}  rate={bag_rate}×")
        wall_start = time.time()
        bag_cmd = f"ros2 bag play {q(bag_path)} --clock --rate {bag_rate}"
        bag_proc = _bg(
            bag_cmd,
            setup_scripts,
            workspace_root,
            logf("bag_play"),
        )
        ok("Bag player", "running")

        time.sleep(1.0)
        if bag_proc.poll() is not None:
            seq_error = "ros2 bag play exited before playback was confirmed."
            fail("Bag player", seq_error)
            bag_proc = None
        else:
            # ── Verify /clock ─────────────────────────────────────────────────
            section(f"Verifying /clock  [{seq_name}  run {run_num}/{n_runs}]")
            step("/clock", f"waiting up to {clock_start_timeout:.0f}s")
            if not _wait_for_clock_message(
                setup_scripts, workspace_root, clock_start_timeout
            ):
                seq_error = (
                    f"ros2 bag play did not publish /clock within "
                    f"{clock_start_timeout}s."
                )
                fail("/clock", seq_error)
                _sigint_proc(bag_proc)
                _wait_for_proc(bag_proc, 2.0)
                bag_proc = None
            else:
                ok("/clock detected", "bag playback confirmed")

    # ── Wait for bag to finish ────────────────────────────────────────────────
    bag_rc: int | None = None
    if not seq_error and bag_proc is not None:
        section(f"Waiting for Bag  [{seq_name}  run {run_num}/{n_runs}]")
        deadline_bag = time.monotonic() + bag_timeout
        bag_rc_raw: int | None = None
        while time.monotonic() < deadline_bag:
            bag_rc_raw = bag_proc.poll()
            if bag_rc_raw is not None:
                break
            if ov2_proc is not None and ov2_proc.poll() is not None:
                _sigint_proc(bag_proc)
                _wait_for_proc(bag_proc, 2.0)
                bag_proc = None
                seq_error = (
                    "OV2SLAM exited while bag playback was still running. "
                    "Check ov2slam.log."
                )
                fail("OV2SLAM mid-run", seq_error)
                break
            time.sleep(0.5)

        if not seq_error and bag_proc is not None and bag_rc_raw is None:
            warnings.append(
                f"Bag playback timed out after {bag_timeout}s and was killed."
            )
            stop("Bag player", f"timed out after {bag_timeout}s")
            _sigint_proc(bag_proc)
            _wait_for_proc(bag_proc, 2.0)
            bag_proc = None
            seq_error = f"Bag playback timed out after {bag_timeout}s."
        elif not seq_error and bag_rc_raw is not None:
            bag_rc = bag_rc_raw
            bag_proc = None
            if bag_rc != 0:
                warnings.append(
                    f"ros2 bag play exited with rc={bag_rc}; "
                    "attempting final flush anyway."
                )
                stop("Bag player", f"rc={bag_rc}")
            else:
                ok("Bag player", "finished")

    # ── Final clock + auto-exit ───────────────────────────────────────────────
    if not seq_error:
        section(f"Final Clock + Shutdown  [{seq_name}  run {run_num}/{n_runs}]")
        step("Publishing final /clock", f"t={final_clock_sec}s")
        _publish_final_clock(
            setup_scripts, workspace_root, final_clock_sec, final_clock_nanosec
        )
        ok("Final /clock published")

        step(
            "Waiting for OV2SLAM auto-exit",
            f"timeout {auto_exit_timeout}s",
        )
        if ov2_proc is not None and ov2_proc.poll() is None:
            if _wait_for_pid_exit(ov2_proc.pid, auto_exit_timeout):
                try:
                    ov2_proc.wait(timeout=2.0)
                except Exception:
                    pass
                ok("OV2SLAM auto-exited")
            else:
                warnings.append(
                    "OV2SLAM did not auto-exit after final /clock; SIGINT fallback used."
                )
                stop("Auto-exit timeout", "sending SIGINT to OV2SLAM")
                _sigint_proc(ov2_proc)
                if not _wait_for_proc(ov2_proc, sigint_timeout):
                    try:
                        os.killpg(ov2_proc.pid, signal.SIGKILL)
                    except Exception:
                        pass

        if ov2_proc is not None and ov2_proc.poll() is None:
            seq_error = "OV2SLAM did not exit cleanly after final flush."
            fail("OV2SLAM", seq_error)

    # ── Cleanup ───────────────────────────────────────────────────────────────
    if bag_proc is not None and bag_proc.poll() is None:
        _sigint_proc(bag_proc)
        _wait_for_proc(bag_proc, 2.0)
    if ov2_proc is not None and ov2_proc.poll() is None:
        _sigint_proc(ov2_proc)
        if not _wait_for_proc(ov2_proc, sigint_timeout):
            try:
                os.killpg(ov2_proc.pid, signal.SIGKILL)
            except Exception:
                pass

    if monitor is not None:
        step("Stopping CPU monitor", "waiting for final flush…")
        monitor.stop()
        ok("CPU monitor", "stopped")

    nuke("Run complete — shutting down ROS2 pipeline")

    # ── CPU plots ─────────────────────────────────────────────────────────────
    section(f"CPU Plots  [{seq_name}  run {run_num}/{n_runs}]")
    if generate_cpu_plots(run_dir, seq_name):
        ok("CPU process plot", "ov2slam_cpu_usage.png")
        ok("CPU thread bar", "ov2slam_threads_cpu_bar.png")
    else:
        stop("CPU plots", "skipped (no data or matplotlib missing)")
        warnings.append("CPU monitor plots were not generated.")

    # ── Trajectory selection ──────────────────────────────────────────────────
    if not seq_error:
        section(f"Trajectory Selection  [{seq_name}  run {run_num}/{n_runs}]")
        try:
            traj_path, seq_traj_name, seq_traj_reason = select_estimate_trajectory(
                run_dir, trajectory_policy
            )
            shutil.copy2(traj_path, trajectory_out)
            n_poses = _count_poses(trajectory_out)
            ok("Trajectory", f"{seq_traj_name}  ({n_poses} poses)")
            step("Reason", seq_traj_reason)
        except Exception as exc:
            seq_error = str(exc)
            fail("Trajectory selection", seq_error)

    # ── EVO evaluation ────────────────────────────────────────────────────────
    evo_stats: dict = {}
    if not seq_error:
        try:
            evo_stats = run_evo(
                seq_name,
                trajectory_out,
                gt_file,
                eval_cfg,
                mode_cfg,
                run_dir,
            )
        except Exception as exc:
            seq_error = f"evo evaluation failed: {exc}"
            fail("evo_ape", seq_error)

    # ── needs_rerun evaluation ────────────────────────────────────────────────
    # A run is flagged for rerun if:
    #   • RMSE > 0.20 m  (trajectory too inaccurate to be a useful data point)
    #   • trajectory_poses < 10% of GT poses  (nearly full tracking failure)
    # Flagged runs are NOT automatically retried — reruns happen via n_runs.
    gt_poses = _count_poses(gt_file)
    traj_poses = _count_poses(trajectory_out) if trajectory_out.exists() else 0
    rmse_val = evo_stats.get("rmse")

    needs_rerun = False
    needs_rerun_reason = ""
    if not seq_error:
        if rmse_val is not None and rmse_val > 0.20:
            needs_rerun = True
            needs_rerun_reason = (
                f"RMSE {rmse_val:.4f} m > 0.20 m threshold"
            )
        elif gt_poses > 0 and traj_poses < 0.10 * gt_poses:
            needs_rerun = True
            needs_rerun_reason = (
                f"trajectory poses {traj_poses} < 10% of GT poses {gt_poses} "
                f"({traj_poses / gt_poses * 100:.1f}%)"
            )

    if needs_rerun:
        _w(
            f"\n  {C.YELLOW}⚠  NEEDS RERUN — {needs_rerun_reason}{C.RESET}\n"
        )

    # ── run_meta.json ─────────────────────────────────────────────────────────
    wall_time_sec = max(0.0, time.time() - wall_start)
    timing_avgs = load_ov2_timings(run_dir / "ov2slam_timings.csv")
    cpu_avg = _mean_csv_column(
        run_dir / "ov2slam_process_cpu.csv", "process_cpu_percent"
    )

    run_meta: dict = {
        "sequence": seq_name,
        "run": run_num,
        "mode": mode_cfg["mode"],
        "profile": mode_cfg["profile"],
        "trajectory_policy": trajectory_policy,
        "success": not bool(seq_error),
        "needs_rerun": needs_rerun,
        "needs_rerun_reason": needs_rerun_reason,
        "rmse": rmse_val,
        "match_ratio": evo_stats.get("match_ratio"),
        "n_matched": evo_stats.get("n_matched"),
        "traj_used": seq_traj_name,
        "traj_reason": seq_traj_reason,
        "trajectory_poses": traj_poses if trajectory_out.exists() else None,
        "gt_poses": gt_poses,
        "wall_time_sec": round(wall_time_sec, 3),
        "cpu_total_avg_pct": cpu_avg,
        "timing_avg_ms": timing_avgs,
        "warnings": " || ".join(warnings) if warnings else "",
        "error": seq_error,
        "gt_verdict": seq_gt_verdict,
        "bag_rate": float(playback_cfg.get("bag_rate", 1.0)),
        "bag_exit_code": bag_rc,
        "artifacts": {
            "gt_tum": "gt.tum" if (run_dir / "gt.tum").exists() else None,
            "trajectory_tum": (
                "trajectory.tum" if trajectory_out.exists() else None
            ),
            "ape_traj_png": (
                "ape_traj.png" if (run_dir / "ape_traj.png").exists() else None
            ),
            "traj_xy_png": (
                "traj_xy.png" if (run_dir / "traj_xy.png").exists() else None
            ),
            "ov2slam_log": (
                "ov2slam.log" if (run_dir / "ov2slam.log").exists() else None
            ),
            "timings_csv": (
                "ov2slam_timings.csv"
                if (run_dir / "ov2slam_timings.csv").exists()
                else None
            ),
            "pid_info_json": (
                "ov2slam_pid_info.json"
                if (run_dir / "ov2slam_pid_info.json").exists()
                else None
            ),
            "process_cpu_csv": (
                "ov2slam_process_cpu.csv"
                if (run_dir / "ov2slam_process_cpu.csv").exists()
                else None
            ),
            "thread_cpu_csv": (
                "ov2slam_thread_cpu.csv"
                if (run_dir / "ov2slam_thread_cpu.csv").exists()
                else None
            ),
            "thread_cpu_summary_csv": (
                "ov2slam_thread_cpu_summary.csv"
                if (run_dir / "ov2slam_thread_cpu_summary.csv").exists()
                else None
            ),
            "cpu_usage_png": (
                "ov2slam_cpu_usage.png"
                if (run_dir / "ov2slam_cpu_usage.png").exists()
                else None
            ),
            "threads_cpu_bar_png": (
                "ov2slam_threads_cpu_bar.png"
                if (run_dir / "ov2slam_threads_cpu_bar.png").exists()
                else None
            ),
        },
    }

    with open(run_dir / "run_meta.json", "w") as fh:
        json.dump(run_meta, fh, indent=2)

    if seq_error:
        fail(f"Run {run_num} failed", seq_error)
        return None

    return run_meta


# ─── summaries ────────────────────────────────────────────────────────────────


def _safe_mean(values: list) -> float:
    clean = [v for v in values if v is not None and not (isinstance(v, float) and v != v)]
    if not clean:
        return float("nan")
    return sum(clean) / len(clean)


def _safe_std(values: list) -> float:
    clean = [v for v in values if v is not None and not (isinstance(v, float) and v != v)]
    if len(clean) < 2:
        return 0.0
    mean = _safe_mean(clean)
    return (sum((v - mean) ** 2 for v in clean) / len(clean)) ** 0.5


def write_seq_summary(
    seq_name: str,
    mode: str,
    stats_list: list[dict | None],
    seq_dir: Path,
) -> dict:
    valid = [s for s in stats_list if s is not None and s.get("rmse") is not None]
    clean = [s for s in valid if not s.get("needs_rerun")]
    flagged = [s for s in valid if s.get("needs_rerun")]
    # For aggregate metrics, prefer clean runs; fall back to all valid if none clean.
    selected = clean if clean else valid

    section(f"Run Summary  [{seq_name}]")
    hdr = (
        f"  {'Run':>5} {'RMSE (m)':>10} {'CPU %':>10} "
        f"{'Wall (s)':>10} {'Poses':>8}  Trajectory"
    )
    _w(f"{C.BOLD}{C.WHITE}{hdr}{C.RESET}\n  {'─' * 78}\n")

    for stats in stats_list:
        if stats is None:
            _w(f"  {'?':>5} {C.RED}{'FAILED':>10}{C.RESET}\n")
            continue
        run_id = stats.get("run", "?")
        rmse = stats.get("rmse")
        if rmse is None:
            _w(f"  {run_id:>5} {C.RED}{'FAILED':>10}{C.RESET}\n")
            continue
        cpu_pct = stats.get("cpu_total_avg_pct") or float("nan")
        wall = stats.get("wall_time_sec", float("nan"))
        poses = stats.get("trajectory_poses", 0) or 0
        traj_name = stats.get("traj_used", "-") or "-"
        flag = (
            f"  {C.YELLOW}⚠ RERUN{C.RESET}  {C.DIM}{stats.get('needs_rerun_reason', '')}{C.RESET}"
            if stats.get("needs_rerun")
            else ""
        )
        color = C.GREEN if rmse < 0.05 else C.YELLOW if rmse < 0.10 else C.RED
        _w(
            f"  {run_id:>5} {color}{rmse:>10.4f}{C.RESET} {cpu_pct:>10.2f}"
            f" {wall:>10.2f} {poses:>8}  {traj_name}{flag}\n"
        )

    rmse_all = _safe_mean([s["rmse"] for s in valid])
    rmse_std_all = _safe_std([s["rmse"] for s in valid])
    rmse_clean = _safe_mean([s["rmse"] for s in clean])
    rmse_std_clean = _safe_std([s["rmse"] for s in clean])
    cpu_avg = _safe_mean(
        [s.get("cpu_total_avg_pct") for s in selected if s.get("cpu_total_avg_pct") is not None]
    )
    wall_avg = _safe_mean([s.get("wall_time_sec", float("nan")) for s in selected])

    color_all = C.GREEN if rmse_all < 0.05 else C.YELLOW if rmse_all < 0.10 else C.RED
    _w(f"  {'─' * 78}\n")
    _w(
        f"  {'all':>5} {color_all}{rmse_all:>10.4f}{C.RESET} {cpu_avg:>10.2f}"
        f" {wall_avg:>10.2f}           "
        f"clean={len(clean)}/{len(valid)}  flagged={len(flagged)}\n"
    )
    if clean:
        color_clean = C.GREEN if rmse_clean < 0.05 else C.YELLOW if rmse_clean < 0.10 else C.RED
        _w(
            f"  {'clean':>5} {color_clean}{rmse_clean:>10.4f}{C.RESET}  "
            f"(±{rmse_std_clean:.4f})\n"
        )

    summary = {
        "sequence": seq_name,
        "mode": mode,
        "n_runs_total": len(stats_list),
        "n_runs_success": len(valid),
        "n_runs_clean": len(clean),
        "n_runs_flagged": len(flagged),
        "clean_selection_fallback_used": bool(valid and not clean),
        "rmse_avg_all": rmse_all,
        "rmse_std_all": rmse_std_all,
        "rmse_avg_clean": rmse_clean,
        "rmse_std_clean": rmse_std_clean,
        "cpu_total_avg_pct": cpu_avg,
        "wall_time_avg_sec": wall_avg,
        "runs": [s for s in stats_list if s is not None],
    }

    seq_dir.mkdir(parents=True, exist_ok=True)
    with open(seq_dir / "run_summary.json", "w") as fh:
        json.dump(summary, fh, indent=2)
    return summary


def write_summary(seq_summaries: list[dict], results_root: Path) -> None:
    if not seq_summaries:
        return
    results_root.mkdir(parents=True, exist_ok=True)
    csv_path = results_root / "experiment_summary.csv"

    fieldnames = [
        "sequence",
        "mode",
        "n_runs_total",
        "n_runs_success",
        "n_runs_clean",
        "n_runs_flagged",
        "clean_selection_fallback_used",
        "rmse_avg_all",
        "rmse_std_all",
        "rmse_avg_clean",
        "rmse_std_clean",
        "cpu_total_avg_pct",
        "wall_time_avg_sec",
    ]
    with open(csv_path, "w", newline="") as fh:
        writer = csv.DictWriter(fh, fieldnames=fieldnames, extrasaction="ignore")
        writer.writeheader()
        for row in seq_summaries:
            writer.writerow(row)

    section("Experiment Summary")
    hdr = (
        f"  {'Sequence':<22} {'Mode':<8} {'Runs':>12} "
        f"{'RMSE all':>10} {'RMSE clean':>12} {'CPU %':>8} {'Wall (s)':>10}"
    )
    _w(f"{C.BOLD}{C.WHITE}{hdr}{C.RESET}\n  {'─' * 88}\n")
    for row in seq_summaries:
        rmse_all = row.get("rmse_avg_all", float("nan"))
        rmse_clean = row.get("rmse_avg_clean", float("nan"))
        cpu_pct = row.get("cpu_total_avg_pct", float("nan"))
        wall_avg = row.get("wall_time_avg_sec", float("nan"))
        flagged = row.get("n_runs_flagged", 0)
        color = C.GREEN if rmse_all < 0.05 else C.YELLOW if rmse_all < 0.10 else C.RED
        color_c = C.GREEN if rmse_clean < 0.05 else C.YELLOW if rmse_clean < 0.10 else C.RED
        flag_str = f"  {C.YELLOW}⚠{flagged}{C.RESET}" if flagged else ""
        _w(
            f"  {row['sequence']:<22} {row['mode']:<8} "
            f"{row['n_runs_clean']:>2}c/{row['n_runs_success']:>2}ok/{row['n_runs_total']:<3} "
            f"{color}{rmse_all:>10.4f}{C.RESET} {color_c}{rmse_clean:>12.4f}{C.RESET}"
            f" {cpu_pct:>8.2f} {wall_avg:>10.2f}{flag_str}\n"
        )
    _w(f"\n  {C.DIM}CSV → {csv_path}{C.RESET}\n")


# ─── argument parsing ─────────────────────────────────────────────────────────


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="OV2SLAM EuRoC multi-sequence benchmark runner.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            "Examples:\n"
            "  python3 ov2_benchmark.py\n"
            "  python3 ov2_benchmark.py --config OV2_BENCHMARK.yaml\n"
            "  python3 ov2_benchmark.py --sequence MH_01_easy --runs 1\n"
        ),
    )
    parser.add_argument(
        "--config",
        type=Path,
        default=None,
        help="Path to OV2_BENCHMARK.yaml.  If omitted uses built-in defaults.",
    )
    parser.add_argument(
        "--sequence",
        default=None,
        help="Run only this sequence (overrides config dataset.sequences).",
    )
    parser.add_argument(
        "--mode",
        default=None,
        choices=["mono", "stereo"],
        help="Sensor mode (overrides config experiment.mode).",
    )
    parser.add_argument(
        "--profile",
        default=None,
        choices=["accurate", "average", "fast"],
        help="OV2SLAM speed profile (overrides config experiment.profile).",
    )
    parser.add_argument(
        "--trajectory-policy",
        default=None,
        choices=["paper", "best"],
        dest="trajectory_policy",
        help="Trajectory selection policy (overrides config experiment.trajectory_policy).",
    )
    parser.add_argument(
        "--runs",
        type=int,
        default=None,
        help="Number of runs per sequence (overrides config evaluation.n_runs).",
    )
    parser.add_argument(
        "--results-name",
        default=None,
        dest="results_name",
        help="Results subdirectory name (overrides config evaluation.results_name).",
    )
    return parser.parse_args()


# ─── main ─────────────────────────────────────────────────────────────────────


def main() -> None:
    args = parse_args()
    ws = LOCAL_WORKSPACE_ROOT

    cfg = load_config(args.config)

    # CLI overrides
    if args.sequence:
        cfg["dataset"]["sequences"] = [args.sequence]
    if args.mode:
        cfg["experiment"]["mode"] = args.mode
    if args.profile:
        cfg["experiment"]["profile"] = args.profile
    if args.trajectory_policy:
        cfg["experiment"]["trajectory_policy"] = args.trajectory_policy
    if args.runs is not None:
        cfg["evaluation"]["n_runs"] = args.runs
    if args.results_name:
        cfg["evaluation"]["results_name"] = args.results_name

    eval_cfg = cfg.get("evaluation", {})
    playback_cfg = cfg.get("playback", {})
    mode_cfg = mode_profile(cfg, ws)
    workspace_root: Path = mode_cfg["workspace_root"]

    n_runs = int(eval_cfg.get("n_runs", 3))
    inter_run_delay_sec = float(eval_cfg.get("inter_run_delay_sec", 10))

    # Resolve dataset root
    dataset_root_raw = str(cfg.get("dataset", {}).get("dataset_root", "") or "")
    dataset_root = (
        Path(dataset_root_raw).expanduser().resolve()
        if dataset_root_raw
        else workspace_root / "datasets" / "euroc"
    )

    # Resolve log / results dirs
    log_dir_raw = str(eval_cfg.get("log_dir", "") or "")
    log_dir = (
        Path(log_dir_raw).expanduser().resolve()
        if log_dir_raw
        else workspace_root / "run_logs"
    )
    results_dir_raw = str(eval_cfg.get("results_dir", "") or "")
    results_dir = (
        Path(results_dir_raw).expanduser().resolve()
        if results_dir_raw
        else workspace_root / "results" / "ov2slam_benchmark"
    )
    results_name_raw = str(eval_cfg.get("results_name", "") or "")
    results_name = (
        results_name_raw
        if results_name_raw
        else datetime.now(UTC_PLUS_4).strftime("%Y%m%d_%H%M%S")
    )

    results_root = results_dir / results_name
    results_root.mkdir(parents=True, exist_ok=True)
    log_dir.mkdir(parents=True, exist_ok=True)

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    master_log = log_dir / f"terminal_{stamp}_ov2_benchmark.log"

    global _log_fd
    _log_fd = open(master_log, "w", buffering=1)
    _w(f"  {C.DIM}master log  : {master_log}{C.RESET}\n")

    signal.signal(signal.SIGINT, _sigint)
    signal.signal(signal.SIGTERM, _sigint)

    try:
        preflight_checks(mode_cfg, ws, dataset_root)
        sequences = find_sequences(cfg, ws)
        if not sequences:
            fail("No valid sequences found — check your dataset paths.")
            sys.exit(1)

        banner(
            f"OV2SLAM EuRoC Benchmark  "
            f"({mode_cfg['mode'].upper()} · {mode_cfg['profile'].upper()} · "
            f"{len(sequences)} seq × {n_runs} run)"
        )
        _w(f"  {C.DIM}config      : {args.config or 'built-in defaults'}\n")
        _w(f"  {C.DIM}mode        : {mode_cfg['mode']}\n")
        _w(f"  {C.DIM}profile     : {mode_cfg['profile']}\n")
        _w(f"  {C.DIM}traj policy : {mode_cfg['trajectory_policy']}\n")
        _w(f"  {C.DIM}params      : {mode_cfg['params_file']}\n")
        _w(f"  {C.DIM}log dir     : {log_dir}\n")
        _w(f"  {C.DIM}results     : {results_root}\n")
        _w(f"  {C.DIM}bag rate    : {playback_cfg.get('bag_rate', 1.0)}×\n")
        _w(f"  {C.DIM}runs        : {n_runs}  (order: seq1_run1, seq2_run1 … seq1_run2, seq2_run2 …)\n")
        _w(f"  {C.DIM}rerun flag  : RMSE > 0.20 m  OR  poses < 10%% of GT\n")
        _w(f"  {C.DIM}sequences   : {[s[0] for s in sequences]}{C.RESET}\n")

        # Save run config snapshot
        run_config = {
            "config_file": str(args.config or "built-in defaults"),
            "workspace_root": str(workspace_root),
            "dataset_root": str(dataset_root),
            "results_root": str(results_root),
            "mode": mode_cfg["mode"],
            "profile": mode_cfg["profile"],
            "trajectory_policy": mode_cfg["trajectory_policy"],
            "params_file": str(mode_cfg["params_file"]),
            "n_runs": n_runs,
            "rerun_rmse_threshold": 0.20,
            "rerun_pose_coverage_threshold": 0.10,
            "selected_sequences": [s[0] for s in sequences],
            "bag_play_rate": float(playback_cfg.get("bag_rate", 1.0)),
            "evo_t_max_diff": float(eval_cfg.get("t_max_diff", 0.1)),
        }
        (results_root / "run_config.json").write_text(
            json.dumps(run_config, indent=2) + "\n", encoding="utf-8"
        )
        if args.config and args.config.exists():
            shutil.copy2(args.config, results_root / "benchmark_config_used.yaml")
        if mode_cfg["params_file"].is_file():
            shutil.copy2(
                mode_cfg["params_file"], results_root / "params_used.yaml"
            )

        seq_run_stats_map: dict[str, list[dict | None]] = {
            s[0]: [] for s in sequences
        }
        seq_summaries: list[dict] = []
        total_global = n_runs * len(sequences)
        global_run = 0
        overall_success = True

        # Running order: all sequences for run 1, then all sequences for run 2, …
        # i.e.  seq1_run1 → seq2_run1 → … → seq1_run2 → seq2_run2 → …
        for run_num in range(1, n_runs + 1):
            for seq_name, bag_path, gt_source in sequences:
                global_run += 1
                seq_banner(seq_name, global_run, total_global)
                run_banner(run_num, n_runs)

                seq_dir = results_root / seq_name
                run_dir = seq_dir / f"run_{run_num:02d}"

                # Single attempt — no automatic retries.
                # Runs flagged needs_rerun are informational only; reruns happen
                # naturally via the n_runs outer loop.
                stats: dict | None = None
                try:
                    stats = run_sequence(
                        seq_name,
                        bag_path,
                        gt_source,
                        cfg,
                        ws,
                        mode_cfg,
                        run_dir,
                        run_num,
                        n_runs,
                    )
                except Exception as exc:
                    fail("Unexpected error", str(exc))
                    stats = None

                seq_run_stats_map[seq_name].append(stats)
                if stats is None:
                    overall_success = False

                if global_run < total_global and inter_run_delay_sec > 0:
                    _w(
                        f"\n  {C.DIM}Cooling down {inter_run_delay_sec:.1f} s "
                        f"before next run…{C.RESET}\n"
                    )
                    time.sleep(inter_run_delay_sec)

        # ── Per-sequence summaries ─────────────────────────────────────────
        for seq_name, _, _ in sequences:
            seq_dir = results_root / seq_name
            stats_list = seq_run_stats_map[seq_name]
            seq_summary = write_seq_summary(
                seq_name, mode_cfg["mode"], stats_list, seq_dir
            )
            if seq_summary:
                seq_summaries.append(seq_summary)

        write_summary(seq_summaries, results_root)

        _w(
            f"\n  {C.BOLD}"
            f"{C.GREEN if overall_success else C.RED}"
            f"{'All sequences complete.' if overall_success else 'Some sequences failed.'}"
            f"{C.RESET}\n"
        )
        _w(f"  {C.DIM}Results  : {results_root}\n")
        _w(f"  {C.DIM}Logs     : {log_dir}\n\n")

        sys.exit(0 if overall_success else 1)

    finally:
        try:
            if _log_fd:
                _log_fd.close()
        except Exception:
            pass


if __name__ == "__main__":
    main()
