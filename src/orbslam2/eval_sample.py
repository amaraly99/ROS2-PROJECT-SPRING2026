#!/usr/bin/env python3
"""
RTAB-Map multi-sequence evaluation runner
==========================================

Reads experiment_config.yaml (or legacy defaults) and runs the full RTAB-Map
F2M stereo pipeline for every selected sequence, with configurable retries.

Per run produces (inside results/<UTC+4_stamp>/<seq_name>/run_NN/):
  • trajectory.tum
  • ape_traj.png         – APE-coloured trajectory
  • traj_xy.png          – GT vs estimated overlay
  • ape_stats.json
  • timing_bag/          – ros2 bag of /odom_info + /rtabmap/info
  • odom_timing.csv      – per-frame odometry timings
  • slam_timing.csv      – per-node SLAM timings
  • timing_summary.csv   – avg/std/min/max per metric
  • cpu_usage.csv        – per-thread CPU samples
  • logs/                – per-process log files

After all sequences:
  • <seq_name>/run_summary.json       – avg/std RMSE + avg timings across retries
  • cpu_per_sequence.png              – total CPU bar chart per sequence
  • cpu_per_thread.png                – avg CPU per thread across all sequences
  • experiment_summary.csv           – all sequences × all run-average metrics

Launch order (per run)
-----------------------
  1.  euroc_caminfo_pub.py            (background)
  2.  rectify_node  cam0              (background)
  3.  rectify_node  cam1              (background)
  4.  ros2 launch euroc_offline_f2m.launch.py   ← live output
  5.  sleep 2 s
  6.  ros2 bag record /odom_info /rtabmap/info   (background)
  7.  monitor_cpu.py                  (background)
  8.  ros2 bag play <sequence>  --clock
  9.  mapPath_to_tum.py               (background)
  10. wait for bag → SIGINT timing bag + CPU monitor → nuke everything
  11. postprocess.py --run-dir        (timing CSVs)
  12. evo_ape  + evo_traj             (PNG plots + stats)

Usage
-----
  # Config-driven (recommended):
  python3 /workspace/eval.py --config /workspace/experiment_config.yaml

  # Single-sequence override:
  python3 /workspace/eval.py --config /workspace/experiment_config.yaml \\
          --sequence MH_01_easy

  # Legacy (no config — uses MH_01_easy with built-in defaults):
  python3 /workspace/eval.py

CTRL+C  →  SIGKILL every launched process and all ros2 children.
"""

import argparse
import csv
import json
import os
import re
import shlex
import signal
import subprocess
import sys
import tempfile
import threading
import time
from pathlib import Path

try:
    import yaml
except ImportError:
    print("ERROR: PyYAML required.  pip3 install pyyaml")
    sys.exit(1)

try:
    import numpy as np
except ImportError:
    np = None  # graceful degradation for std computation

# ── Paths ──────────────────────────────────────────────────────────────────────
SCRIPT_PATH       = Path(__file__).resolve()
HOST_WS           = SCRIPT_PATH.parent
CONTAINER_WS      = Path("/workspace")
DEFAULT_CONTAINER = "rtabmap_jazzy"

UTC4_OFFSET = 4 * 3600  # seconds east of UTC

# ── ANSI colours ───────────────────────────────────────────────────────────────
class C:
    RESET   = "\033[0m"
    BOLD    = "\033[1m"
    DIM     = "\033[2m"
    RED     = "\033[91m"
    GREEN   = "\033[92m"
    YELLOW  = "\033[93m"
    BLUE    = "\033[94m"
    MAGENTA = "\033[95m"
    CYAN    = "\033[96m"
    WHITE   = "\033[97m"


# ── Terminal + log tee ────────────────────────────────────────────────────────
_log_fd = None


def _w(text: str) -> None:
    sys.stdout.write(text)
    sys.stdout.flush()
    if _log_fd:
        _log_fd.write(text)
        _log_fd.flush()


def msg(color, icon, label, detail=""):
    d = f"  {C.DIM}{detail}" if detail else ""
    _w(f"  {icon}  {C.BOLD}{color}{label}{C.RESET}{d}\n")

def ok(label, detail=""):   msg(C.GREEN,   f"{C.GREEN}✓{C.RESET}",  label, detail)
def fail(label, detail=""): msg(C.RED,     f"{C.RED}✗{C.RESET}",    label, detail)
def step(label, detail=""): msg(C.CYAN,    f"{C.CYAN}▶{C.RESET}",   label, detail)
def stop(label, detail=""): msg(C.YELLOW,  f"{C.YELLOW}■{C.RESET}", label, detail)

def section(title):
    pad = max(0, 56 - len(title))
    _w(f"\n{C.BOLD}{C.WHITE}  ── {title} {C.DIM}{'─' * pad}{C.RESET}\n")

def banner(title):
    bar = "═" * 66
    _w(f"\n{C.BOLD}{C.BLUE}╔{bar}╗\n")
    _w(f"{C.BOLD}{C.BLUE}║  {C.WHITE}{title:<64}{C.BLUE}║\n")
    _w(f"{C.BOLD}{C.BLUE}╚{bar}╝{C.RESET}\n")

def seq_banner(seq_name, idx, total):
    _w(f"\n{C.BOLD}{C.MAGENTA}  {'━'*66}\n")
    _w(f"  Sequence {idx}/{total}: {C.WHITE}{seq_name}{C.RESET}\n")
    _w(f"{C.BOLD}{C.MAGENTA}  {'━'*66}{C.RESET}\n")

def run_banner(run_num, n_retries):
    _w(f"\n{C.BOLD}{C.CYAN}  {'─'*66}\n")
    _w(f"  Run {run_num}/{n_retries}\n")
    _w(f"  {'─'*66}{C.RESET}\n")

def launch_line(line):
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

def evo_line(line):
    s = line.rstrip()
    if s:
        _w(f"  {C.CYAN}│{C.RESET}  {C.CYAN}{s}{C.RESET}\n")


# ── Process management ─────────────────────────────────────────────────────────
_all_procs: list = []
_abort_event = threading.Event()


def _reset_state():
    global _all_procs, _abort_event
    _all_procs = []
    _abort_event = threading.Event()


def ros(cmd):
    return f"source /opt/ros/jazzy/setup.bash && {cmd}"


def q(v):
    return shlex.quote(str(v))


def _bg(cmd, log_path=None, line_fn=None):
    """Launch cmd via bash in a new session; optionally stream through line_fn."""
    if line_fn is not None:
        log_fd = open(log_path, "w", buffering=1) if log_path else None
        proc = subprocess.Popen(
            ["bash", "-lc", ros(cmd)],
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            text=True, bufsize=1,
            start_new_session=True,
        )
        def _worker():
            for line in proc.stdout:
                if log_fd:
                    log_fd.write(line); log_fd.flush()
                line_fn(line)
            if log_fd:
                log_fd.close()
        threading.Thread(target=_worker, daemon=True).start()
    else:
        out = open(log_path, "w", buffering=1) if log_path else subprocess.DEVNULL
        proc = subprocess.Popen(
            ["bash", "-lc", ros(cmd)],
            stdout=out, stderr=out,
            start_new_session=True,
        )
    _all_procs.append(proc)
    return proc


def _sigint_proc(proc):
    """Send SIGINT to a process group, ignoring errors."""
    try:
        os.killpg(proc.pid, signal.SIGINT)
    except Exception:
        pass


def nuke(reason=""):
    if reason:
        _w(f"\n  {C.YELLOW}⚡  {C.BOLD}{reason}{C.RESET}\n")
    for proc in _all_procs:
        try:
            os.killpg(proc.pid, signal.SIGKILL)
        except Exception:
            pass
    for pat in ["ros2", "rtabmap", "stereo_odometry", "rosbag2",
                "mapPath_to_tum", "euroc_caminfo_pub", "rectify_node",
                "monitor_cpu"]:
        subprocess.call(["pkill", "-9", "-f", pat], stderr=subprocess.DEVNULL)
    _w(f"  {C.YELLOW}■  All processes killed.{C.RESET}\n")


def _sigint(signum, frame):
    nuke("CTRL+C — killing everything")
    if _log_fd:
        try: _log_fd.close()
        except Exception: pass
    sys.exit(130)


def watch_traj_proc(traj_proc):
    """Abort the whole run if the trajectory recorder exits unexpectedly."""
    def _watch():
        traj_proc.wait()
        if not _abort_event.is_set():
            _abort_event.set()
            fail("Trajectory recorder exited unexpectedly",
                 f"rc={traj_proc.returncode} — check traj_*.log")
            nuke("Trajectory recorder died — aborting")
            if _log_fd:
                try: _log_fd.close()
                except Exception: pass
            os.kill(os.getpid(), signal.SIGTERM)
    threading.Thread(target=_watch, daemon=True).start()


# ── Config helpers ────────────────────────────────────────────────────────────

_LEGACY_DEFAULT_CONFIG = {
    "dataset": {
        "base_path": str(CONTAINER_WS),
        "sequences_dir": ".",
        "gt_dir": ".",
        "sequences": ["MH_01_easy"],
    },
    "evaluation": {
        "n_retries": 1,
        "bag_rate": 1.0,
        "t_max_diff": 0.01,
        "results_dir": str(CONTAINER_WS / "results"),
        "log_dir":     str(CONTAINER_WS / "run_logs"),
    },
    "rtabmap_params": {},
}


def load_config(path: Path) -> dict:
    with open(path) as f:
        cfg = yaml.safe_load(f)
    for section_key, defaults in _LEGACY_DEFAULT_CONFIG.items():
        if section_key not in cfg:
            cfg[section_key] = defaults
        elif isinstance(defaults, dict):
            for k, v in defaults.items():
                cfg[section_key].setdefault(k, v)
    return cfg


def write_exp_params(rtabmap_params: dict, out_path: Path):
    """Write a ROS2 node-parameter YAML that overrides common_params in the launch file."""
    if not rtabmap_params:
        out_path.write_text("")
        return
    doc = {
        "stereo_odometry": {"ros__parameters": dict(rtabmap_params)},
        "rtabmap":         {"ros__parameters": dict(rtabmap_params)},
    }
    with open(out_path, "w") as f:
        yaml.dump(doc, f, default_flow_style=False)


def find_sequences(cfg: dict) -> list:
    """Return [(name, bag_path, gt_path), ...] according to the config."""
    base     = Path(cfg["dataset"]["base_path"])
    seq_dir  = base / cfg["dataset"].get("sequences_dir", ".")
    gt_dir   = base / cfg["dataset"].get("gt_dir",       ".")
    seqs_cfg = cfg["dataset"].get("sequences", "all")

    if seqs_cfg == "all":
        names = []
        if seq_dir.exists():
            for p in sorted(seq_dir.iterdir()):
                if p.suffix == ".db3":
                    names.append(p.stem)
                elif p.is_dir() and (p / "metadata.yaml").exists():
                    names.append(p.name)
        if not names:
            fail("No bags found in sequences_dir", str(seq_dir))
    else:
        names = list(seqs_cfg)

    result = []
    for name in names:
        bag = seq_dir / f"{name}.db3"
        if not bag.exists():
            bag = seq_dir / name
        gt = gt_dir / f"{name}.txt"
        if not bag.exists():
            stop(f"Skipping {name}", f"bag not found: {bag}")
            continue
        if not gt.exists():
            stop(f"Skipping {name}", f"ground truth not found: {gt}")
            continue
        result.append((name, bag, gt))
    return result


# ── evo evaluation + plot generation ─────────────────────────────────────────

RERUN_MATCH_THRESHOLD = 0.10  # flag as needing rerun if <10% of poses matched GT


def parse_evo_stats(text: str) -> dict:
    stats = {}
    for metric in ("rmse", "mean", "median", "std", "max", "min", "sse"):
        m = re.search(rf"^\s*{metric}\s+([\d.eE+\-]+)", text, re.MULTILINE)
        if m:
            stats[metric] = float(m.group(1))
    # matched pairs from verbose line: "Compared 261 absolute pose pairs."
    m = re.search(r"Compared\s+(\d+)\s+absolute pose pairs", text)
    if m:
        stats["n_matched"] = int(m.group(1))
    else:
        # fallback: "Found 261 of max. 264 possible matching timestamps"
        m = re.search(r"Found\s+(\d+)\s+of max\.", text)
        if m:
            stats["n_matched"] = int(m.group(1))
    return stats


def _count_poses(tum_path: Path) -> int:
    try:
        return sum(
            1 for ln in tum_path.read_text().splitlines()
            if ln.strip() and not ln.startswith("#")
        )
    except Exception:
        return 0


def run_evo(seq_name, tum_path, gt_path, eval_cfg, run_dir: Path, log_dir: Path, stamp):
    """
    Run evo_ape (APE + trajectory-coloured map) and evo_traj (XY overlay).
    All plots saved as PNG inside run_dir.
    Returns a stats dict with needs_rerun=True if <10% of poses matched GT.
    """
    run_dir.mkdir(parents=True, exist_ok=True)

    t_max      = eval_cfg.get("t_max_diff", 0.01)
    match_thr  = float(eval_cfg.get("rerun_match_threshold", RERUN_MATCH_THRESHOLD))
    ape_plot   = run_dir / "ape_traj.png"
    traj_plot  = run_dir / "traj_xy.png"
    stats_file = run_dir / "ape_stats.json"

    total_poses = _count_poses(tum_path)

    # ── evo_ape: per-pose error coloured onto the trajectory ──────────────────
    section(f"evo_ape  [{seq_name}]")
    step("evo_ape", f"gt={gt_path.name}  est={tum_path.name}  total_poses={total_poses}")

    ape_cmd = (
        f"evo_ape tum {q(gt_path)} {q(tum_path)} "
        f"-a -s --t_max_diff {t_max} --verbose "
        f"--plot_mode xy --save_plot {q(ape_plot)}"
    )
    evo_log = log_dir / f"evo_{seq_name}_{stamp}.log"
    evo_log_fd = open(evo_log, "w", buffering=1)
    raw_out = ""
    proc = subprocess.Popen(
        ["bash", "-lc", ape_cmd],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        text=True, bufsize=1,
    )
    for line in proc.stdout:
        evo_log_fd.write(line); evo_log_fd.flush()
        evo_line(line)
        raw_out += line
    proc.wait()
    evo_log_fd.close()

    stats = parse_evo_stats(raw_out)
    stats["sequence"]     = seq_name
    stats["total_poses"]  = total_poses

    # Match ratio: how many of our poses actually paired with GT
    n_matched = stats.get("n_matched", 0)
    match_ratio = (n_matched / total_poses) if total_poses > 0 else 0.0
    stats["match_ratio"]  = round(match_ratio, 4)
    stats["needs_rerun"]  = match_ratio < match_thr

    with open(stats_file, "w") as f:
        json.dump(stats, f, indent=2)

    if proc.returncode == 0:
        rmse_str   = f"{stats.get('rmse', float('nan')):.4f} m"
        match_str  = f"{n_matched}/{total_poses} matched ({match_ratio*100:.1f}%)"
        if stats["needs_rerun"]:
            _w(f"\n  {C.RED}⚠  NEEDS RERUN — only {match_ratio*100:.1f}% of poses "
               f"matched GT (threshold {match_thr*100:.0f}%){C.RESET}\n")
            fail("evo_ape", f"RMSE = {rmse_str}  |  {match_str}  ← FLAGGED")
            # Append to the experiment-level flagged runs log
            flagged_log = run_dir.parent.parent / "flagged_runs.txt"
            run_label = run_dir.name  # e.g. run_01
            with open(flagged_log, "a") as fl:
                fl.write(
                    f"{seq_name}/{run_label}  "
                    f"match={match_ratio*100:.1f}%  "
                    f"poses={total_poses}  "
                    f"RMSE={stats.get('rmse', float('nan')):.4f}m\n"
                )
        else:
            ok("evo_ape", f"RMSE = {rmse_str}  |  {match_str}")
    else:
        fail("evo_ape", f"rc={proc.returncode}")

    # ── evo_traj: GT (dashed) vs estimated (solid), XY plane ─────────────────
    step("evo_traj", f"trajectory XY overlay → {traj_plot.name}")
    traj_cmd = (
        f"evo_traj tum {q(tum_path)} --ref {q(gt_path)} "
        f"-a --correct_scale --plot_mode xy --save_plot {q(traj_plot)}"
    )
    r = subprocess.run(["bash", "-lc", traj_cmd], capture_output=True, text=True)
    if r.returncode == 0:
        ok("evo_traj", str(traj_plot))
    else:
        fail("evo_traj", r.stderr.strip()[:120])

    return stats


# ── Single-sequence, single-run pipeline ─────────────────────────────────────

def run_sequence(seq_name, bag_path, gt_path, cfg, ws, stamp,
                 run_dir: Path, run_num: int, n_retries: int):
    """
    Run the full pipeline for one sequence × one retry.
    Returns stats dict or None on failure.
    """
    _reset_state()
    signal.signal(signal.SIGINT,  _sigint)
    signal.signal(signal.SIGTERM, _sigint)

    eval_cfg = cfg.get("evaluation", {})
    bag_rate = float(eval_cfg.get("bag_rate", 1.0))

    log_dir = run_dir / "logs"
    log_dir.mkdir(parents=True, exist_ok=True)
    run_dir.mkdir(parents=True, exist_ok=True)

    def logf(name):
        return log_dir / f"{name}.log"

    traj_out = run_dir / "trajectory.tum"

    section(f"Pre-launch: Camera Info + Rectifiers  [{seq_name}  run {run_num}/{n_retries}]")

    step("euroc_caminfo_pub", "/cam0|1/image_raw → /cam0|1/camera_info")
    _bg(f"python3 {q(ws / 'euroc_caminfo_pub.py')}", logf("caminfo"))
    ok("euroc_caminfo_pub")

    step("rectify_node  cam0")
    _bg(
        "ros2 run image_proc rectify_node --ros-args "
        "-r __node:=rectify_cam0 -p use_sim_time:=true "
        "-r image:=/cam0/image_raw -r camera_info:=/cam0/camera_info "
        "-r image_rect:=/cam0/image_rect",
        logf("rectify_cam0"),
    )
    ok("rectify_node  cam0")

    step("rectify_node  cam1")
    _bg(
        "ros2 run image_proc rectify_node --ros-args "
        "-r __node:=rectify_cam1 -p use_sim_time:=true "
        "-r image:=/cam1/image_raw -r camera_info:=/cam1/camera_info "
        "-r image_rect:=/cam1/image_rect",
        logf("rectify_cam1"),
    )
    ok("rectify_node  cam1")

    section(f"ros2 launch  [{seq_name}  run {run_num}/{n_retries}]")
    step("ros2 launch", str(ws / "euroc_offline_f2m.launch.py"))
    launch_proc = _bg(
        f"ros2 launch {q(ws / 'euroc_offline_f2m.launch.py')}",
        logf("launch"),
        line_fn=launch_line,
    )

    _w(f"\n  {C.DIM}  sleeping 2 s for launch to settle…{C.RESET}\n")
    time.sleep(2)

    if launch_proc.poll() is not None:
        fail("Launch exited early", f"rc={launch_proc.returncode}")
        nuke("Launch crashed — aborting run")
        return None
    ok("Launch running")

    # ── Timing bag recorder ────────────────────────────────────────────────────
    timing_bag_dir = run_dir / "timing_bag"
    section(f"Timing Bag Recorder  [{seq_name}  run {run_num}/{n_retries}]")
    step("ros2 bag record", "/odom_info  /rtabmap/info")
    timing_bag_proc = _bg(
        f"ros2 bag record /odom_info /rtabmap/info -o {q(timing_bag_dir)}",
        logf("timing_bag"),
    )
    ok("Timing bag recorder", "running")

    # ── CPU monitor ────────────────────────────────────────────────────────────
    cpu_csv = run_dir / "cpu_usage.csv"
    step("monitor_cpu", f"→ {cpu_csv.name}")
    cpu_proc = _bg(
        f"python3 {q(ws / 'monitor_cpu.py')} {q(cpu_csv)} 1.0",
        logf("cpu_monitor"),
    )
    ok("CPU monitor", "running")

    # ── Bag playback ───────────────────────────────────────────────────────────
    section(f"Bag Playback  [{seq_name}  run {run_num}/{n_retries}]")
    step("ros2 bag play", f"{bag_path.name}  rate={bag_rate}×")
    bag_proc = _bg(
        f"ros2 bag play {q(bag_path)} --clock --rate {bag_rate}",
        logf("bag"),
    )
    ok("Bag player", "running")

    # ── Trajectory recorder ────────────────────────────────────────────────────
    section(f"Trajectory Recorder  [{seq_name}  run {run_num}/{n_retries}]")
    step("mapPath_to_tum", f"→ {traj_out.name}")
    traj_proc = _bg(
        f"python3 {q(ws / 'mapPath_to_tum.py')} "
        f"--source topic --topic /mapData "
        f"--discovery-timeout 120 --msg-timeout 120 --idle-timeout 0 "
        f"--out {q(traj_out)}",
        logf("traj"),
    )
    ok("Trajectory recorder", "listening on /mapData")
    watch_traj_proc(traj_proc)

    # ── Wait for bag to finish ─────────────────────────────────────────────────
    section(f"Waiting for bag to finish  [{seq_name}  run {run_num}/{n_retries}]")
    while bag_proc.poll() is None:
        if _abort_event.is_set():
            return None
        if launch_proc.poll() is not None:
            fail("Launch died mid-run", f"rc={launch_proc.returncode}")
            nuke("Launch crashed")
            return None
        time.sleep(1)

    if _abort_event.is_set():
        return None

    ok("Bag player", f"finished  rc={bag_proc.returncode}")
    _abort_event.set()

    # Gracefully stop timing bag and CPU monitor so files are finalised cleanly
    step("Stopping timing bag recorder", "SIGINT → wait 2 s")
    _sigint_proc(timing_bag_proc)
    _sigint_proc(cpu_proc)
    time.sleep(2)

    nuke("Bag done — shutting down ROS2 pipeline")

    # ── Post-process timing bag ────────────────────────────────────────────────
    section(f"Post-processing timings  [{seq_name}  run {run_num}/{n_retries}]")
    step("postprocess", f"{run_dir.name}")
    pp_cmd = (
        f"python3 {q(ws / 'postprocess.py')} --run-dir {q(run_dir)}"
    )
    pp = subprocess.run(
        ["bash", "-lc", ros(pp_cmd)],
        capture_output=True, text=True,
    )
    for line in (pp.stdout + pp.stderr).splitlines():
        if line.strip():
            _w(f"  {C.DIM}{line}{C.RESET}\n")
    if pp.returncode == 0:
        ok("Timing post-processing", "done")
    else:
        stop("Timing post-processing failed", f"rc={pp.returncode}")

    # ── Evaluate ──────────────────────────────────────────────────────────────
    if not traj_out.exists() or traj_out.stat().st_size == 0:
        fail("Trajectory file missing or empty", str(traj_out))
        return None

    n_poses = sum(
        1 for ln in traj_out.read_text().splitlines()
        if ln.strip() and not ln.startswith("#")
    )
    ok("Trajectory", f"{n_poses} poses  ·  {traj_out.name}")

    eval_log_dir = run_dir / "logs"
    stats = run_evo(seq_name, traj_out, gt_path, eval_cfg,
                    run_dir=run_dir, log_dir=eval_log_dir, stamp=stamp)

    # Attach run metadata to stats
    stats["run"] = run_num
    stats["cpu_csv"] = str(cpu_csv)

    # Load timing summary into stats if available
    timing_summary = run_dir / "timing_summary.csv"
    if timing_summary.exists():
        try:
            with open(timing_summary) as f:
                for row in csv.DictReader(f):
                    col = f"timing_{row['metric'].replace('/', '_')}_avg_ms"
                    stats[col] = float(row["avg_ms"])
        except Exception:
            pass

    return stats


# ── Per-sequence summary ───────────────────────────────────────────────────────

def _safe_mean(vals):
    vals = [v for v in vals if v is not None and not (isinstance(v, float) and v != v)]
    if not vals:
        return float("nan")
    return sum(vals) / len(vals)


def _safe_std(vals):
    vals = [v for v in vals if v is not None and not (isinstance(v, float) and v != v)]
    if len(vals) < 2:
        return 0.0
    mean = _safe_mean(vals)
    return (sum((v - mean) ** 2 for v in vals) / len(vals)) ** 0.5


def write_seq_summary(seq_name: str, stats_list: list, seq_dir: Path):
    """Write run_summary.json and print a per-run table for one sequence."""
    if not stats_list:
        return

    rmses = [s.get("rmse") for s in stats_list]
    avg_rmse = _safe_mean(rmses)
    std_rmse = _safe_std(rmses)

    section(f"Retry Summary  [{seq_name}]")
    hdr = f"  {'Run':>5} {'RMSE (m)':>10} {'Mean (m)':>10} {'Max (m)':>10}"
    _w(f"{C.BOLD}{C.WHITE}{hdr}{C.RESET}\n  {'─'*40}\n")
    for s in stats_list:
        rmse  = s.get("rmse", float("nan"))
        mean  = s.get("mean", float("nan"))
        mx    = s.get("max",  float("nan"))
        mr    = s.get("match_ratio", float("nan"))
        flag  = f"  {C.RED}⚠ RERUN{C.RESET}" if s.get("needs_rerun") else ""
        color = C.GREEN if rmse < 0.05 else C.YELLOW if rmse < 0.10 else C.RED
        _w(f"  {s.get('run', '?'):>5} {color}{rmse:>10.4f}{C.RESET}"
           f" {mean:>10.4f} {mx:>10.4f}  match={mr*100:.1f}%{flag}\n")
    color = C.GREEN if avg_rmse < 0.05 else C.YELLOW if avg_rmse < 0.10 else C.RED
    _w(f"  {'─'*40}\n")
    _w(f"  {'avg':>5} {color}{avg_rmse:>10.4f}{C.RESET}"
       f"   ±{std_rmse:.4f}\n")

    summary = {
        "sequence":   seq_name,
        "n_runs":     len(stats_list),
        "rmse_avg":   avg_rmse,
        "rmse_std":   std_rmse,
        "runs":       stats_list,
    }
    seq_dir.mkdir(parents=True, exist_ok=True)
    with open(seq_dir / "run_summary.json", "w") as f:
        json.dump(summary, f, indent=2)


# ── Experiment summary CSV ─────────────────────────────────────────────────────

def write_summary(seq_summaries: list, results_root: Path):
    """Write experiment_summary.csv across all sequences."""
    if not seq_summaries:
        return
    results_root.mkdir(parents=True, exist_ok=True)
    csv_path = results_root / "experiment_summary.csv"

    # Collect all column names
    base_cols = ["sequence", "n_runs", "rmse_avg", "rmse_std"]
    timing_cols = sorted({
        k for s in seq_summaries for k in s.keys()
        if k.startswith("timing_")
    })
    cpu_cols = ["cpu_total_avg_pct"]
    all_cols = base_cols + timing_cols + cpu_cols

    with open(csv_path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=all_cols, extrasaction="ignore")
        w.writeheader()
        for s in seq_summaries:
            w.writerow(s)

    section("Experiment Summary")
    hdr = f"  {'Sequence':<22} {'Runs':>5} {'RMSE avg':>10} {'RMSE std':>10}"
    _w(f"{C.BOLD}{C.WHITE}{hdr}{C.RESET}\n  {'─'*52}\n")
    for s in seq_summaries:
        rmse = s.get("rmse_avg", float("nan"))
        std  = s.get("rmse_std", 0.0)
        runs = s.get("n_runs", "?")
        color = C.GREEN if rmse < 0.05 else C.YELLOW if rmse < 0.10 else C.RED
        _w(f"  {s['sequence']:<22} {runs:>5} {color}{rmse:>10.4f}{C.RESET}"
           f" {std:>10.4f}\n")
    _w(f"\n  {C.DIM}CSV → {csv_path}{C.RESET}\n")


# ── Host → container bridge ────────────────────────────────────────────────────

def in_container():
    return Path("/.dockerenv").exists()


def _to_container(path: str) -> str:
    p = Path(path)
    if p.is_absolute():
        try:
            return str(CONTAINER_WS / p.relative_to(HOST_WS))
        except ValueError:
            return str(p)
    return str(CONTAINER_WS / p)


def run_from_host(args) -> None:
    cmd = [
        "sudo", "docker", "exec", "-it", args.container,
        "python3", str(CONTAINER_WS / SCRIPT_PATH.name),
        "--inside-container",
    ]
    if args.config:
        cmd += ["--config", _to_container(str(args.config))]
    if args.sequence:
        cmd += ["--sequence", args.sequence]
    proc = subprocess.Popen(cmd)
    try:
        sys.exit(proc.wait())
    except KeyboardInterrupt:
        try:
            proc.send_signal(signal.SIGINT)
            proc.wait(timeout=10)
        except Exception:
            proc.kill()
        sys.exit(130)


# ── Entry point ───────────────────────────────────────────────────────────────

def parse_args():
    ws = CONTAINER_WS if in_container() else HOST_WS
    p = argparse.ArgumentParser(
        description="RTAB-Map multi-sequence evaluation runner.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument("--inside-container", action="store_true", help=argparse.SUPPRESS)
    p.add_argument("--container", default=DEFAULT_CONTAINER,
                   help="Docker container name (host-side only)")
    p.add_argument("--config", type=Path, default=None,
                   help="Path to experiment_config.yaml.  "
                        "If omitted, uses legacy MH_01_easy defaults.")
    p.add_argument("--sequence", default=None,
                   help="Run only this sequence (overrides config sequences list).")
    args = p.parse_args()
    args.workspace = CONTAINER_WS if (in_container() or args.inside_container) else HOST_WS
    return args


def main():
    args = parse_args()
    ws = args.workspace

    # ── Load config ───────────────────────────────────────────────────────────
    if args.config and args.config.exists():
        cfg = load_config(args.config)
    else:
        if args.config:
            fail("Config file not found", str(args.config))
            sys.exit(1)
        cfg = _LEGACY_DEFAULT_CONFIG.copy()
        cfg["dataset"]["base_path"] = str(ws)

    if args.sequence:
        cfg["dataset"]["sequences"] = [args.sequence]

    eval_cfg    = cfg.get("evaluation", {})
    log_dir     = Path(eval_cfg.get("log_dir",     str(ws / "run_logs")))
    results_dir = Path(eval_cfg.get("results_dir", str(ws / "results")))
    n_retries   = int(eval_cfg.get("n_retries", 1))
    log_dir.mkdir(parents=True, exist_ok=True)

    # UTC+4 timestamp for the experiment folder
    utc4_stamp  = time.strftime("%Y%m%d_%H%M%S", time.gmtime(time.time() + UTC4_OFFSET))
    results_root = results_dir / utc4_stamp
    results_root.mkdir(parents=True, exist_ok=True)

    stamp = time.strftime("%Y%m%d_%H%M%S")
    master_log = log_dir / f"terminal_{stamp}.log"

    global _log_fd
    _log_fd = open(master_log, "w", buffering=1)
    _w(f"  {C.DIM}master log  : {master_log}{C.RESET}\n")

    signal.signal(signal.SIGINT,  _sigint)
    signal.signal(signal.SIGTERM, _sigint)

    exp_params_file = None
    try:
        if not (in_container() or args.inside_container):
            run_from_host(args)
            return

        # ── Write experiment params file for the launch file ──────────────────
        rtabmap_params  = cfg.get("rtabmap_params", {})
        exp_params_file = Path(tempfile.mktemp(suffix="_rtabmap_exp.yaml"))
        write_exp_params(rtabmap_params, exp_params_file)
        os.environ["RTABMAP_EXP_PARAMS"] = str(exp_params_file)

        # ── Discover sequences ────────────────────────────────────────────────
        sequences = find_sequences(cfg)
        if not sequences:
            fail("No valid sequences found — check your config dataset paths.")
            sys.exit(1)

        banner(
            f"RTAB-Map F2M Benchmark  "
            f"({len(sequences)} seq × {n_retries} retr)"
        )
        _w(f"  {C.DIM}config      : {args.config or 'legacy defaults'}\n")
        _w(f"  {C.DIM}log dir     : {log_dir}\n")
        _w(f"  {C.DIM}results     : {results_root}\n")
        _w(f"  {C.DIM}bag rate    : {eval_cfg.get('bag_rate', 1.0)}×\n")
        _w(f"  {C.DIM}retries     : {n_retries}\n")
        _w(f"  {C.DIM}sequences   : {[s[0] for s in sequences]}{C.RESET}\n")

        # ── Main loop ─────────────────────────────────────────────────────────
        # Outer loop: retries.  Inner loop: sequences.
        # e.g. n_retries=2, seqs=[SEQ1,SEQ2] → SEQ1 SEQ2 SEQ1 SEQ2
        seq_summaries: list[dict] = []
        all_cpu_csvs:  list[Path] = []

        # Accumulate per-sequence run stats keyed by seq_name
        seq_run_stats_map: dict[str, list[dict]] = {s[0]: [] for s in sequences}

        total_runs = n_retries * len(sequences)
        global_run = 0

        for run_num in range(1, n_retries + 1):
            for seq_idx, (seq_name, bag_path, gt_path) in enumerate(sequences, 1):
                global_run += 1
                seq_banner(seq_name, global_run, total_runs)
                run_banner(run_num, n_retries)

                seq_dir = results_root / seq_name
                run_dir = seq_dir / f"run_{run_num:02d}"

                stats = run_sequence(
                    seq_name, bag_path, gt_path, cfg, ws, stamp,
                    run_dir=run_dir,
                    run_num=run_num,
                    n_retries=n_retries,
                )

                if stats:
                    seq_run_stats_map[seq_name].append(stats)
                    cpu_path = Path(stats.get("cpu_csv", ""))
                    if cpu_path.exists():
                        all_cpu_csvs.append(cpu_path)
                else:
                    seq_run_stats_map[seq_name].append(
                        {"sequence": seq_name, "run": run_num}
                    )

                # Cooldown between runs (not after the very last one)
                if global_run < total_runs:
                    _w(f"\n  {C.DIM}Cooling down 5 s before next run…{C.RESET}\n")
                    time.sleep(5)

        # ── Per-sequence summaries (written after all retries are done) ────────
        for seq_name, bag_path, gt_path in sequences:
            seq_dir       = results_root / seq_name
            seq_run_stats = seq_run_stats_map[seq_name]
            write_seq_summary(seq_name, seq_run_stats, seq_dir)

            rmses = [s.get("rmse") for s in seq_run_stats if s.get("rmse") is not None]
            seq_row: dict = {
                "sequence": seq_name,
                "n_runs":   len(seq_run_stats),
                "rmse_avg": _safe_mean(rmses),
                "rmse_std": _safe_std(rmses),
            }
            timing_keys = {k for s in seq_run_stats for k in s if k.startswith("timing_")}
            for tk in timing_keys:
                vals = [s[tk] for s in seq_run_stats if tk in s]
                seq_row[tk] = _safe_mean(vals)

            seq_summaries.append(seq_row)

        # ── Aggregate CPU charts ──────────────────────────────────────────────
        if all_cpu_csvs:
            section("Aggregate CPU Charts")
            step("postprocess --aggregate-cpu", str(results_root))
            cpu_arg = " ".join(q(str(p)) for p in all_cpu_csvs)
            agg_cmd = (
                f"python3 {q(ws / 'postprocess.py')} "
                f"--aggregate-cpu "
                f"--results-root {q(results_root)} "
                f"--cpu-csvs {cpu_arg}"
            )
            agg = subprocess.run(
                ["bash", "-lc", ros(agg_cmd)],
                capture_output=True, text=True,
            )
            for line in (agg.stdout + agg.stderr).splitlines():
                if line.strip():
                    _w(f"  {C.DIM}{line}{C.RESET}\n")
            if agg.returncode == 0:
                ok("CPU charts", str(results_root))
            else:
                stop("CPU chart generation failed", f"rc={agg.returncode}")

        # ── Experiment summary ────────────────────────────────────────────────
        write_summary(seq_summaries, results_root)

        _w(f"\n  {C.BOLD}{C.GREEN}All sequences complete.{C.RESET}\n")
        _w(f"  {C.DIM}Results  : {results_root}\n")
        _w(f"  {C.DIM}Logs     : {log_dir}\n\n")

    finally:
        try:
            _log_fd.close()
        except Exception:
            pass
        if exp_params_file and exp_params_file.exists():
            exp_params_file.unlink(missing_ok=True)


if __name__ == "__main__":
    main()
