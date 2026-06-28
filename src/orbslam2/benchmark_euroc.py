#!/usr/bin/env python3
"""
ORB-SLAM2 EuRoC benchmark runner
================================

Config-driven multi-sequence benchmarking for the ROS2 ORB-SLAM2 wrappers in
this repository, keeping the same terminal UI style as eval_sample.py.

Features
--------
* Reads experiment_configuration.yaml (or built-in defaults)
* Benchmarks EuRoC bags in either mono or stereo mode
* Automatically applies the right EuRoC preprocessing pipeline per mode:
  - mono   : /cam0/image_raw -> /camera
  - stereo : rectified /camera/left + /camera/right
* Runs evo_ape + evo_traj and writes per-run / per-sequence summaries

Per run produces (inside results/<UTC+4_stamp>/<seq_name>/run_NN/):
  * trajectory.tum
  * ape_traj.png        (canonical convenience copy)
  * traj_xy.png         (canonical convenience copy)
  * ape_stats.json
  * run_meta.json
  * logs/

After all sequences:
  * <seq_name>/run_summary.json
  * experiment_summary.csv

Usage
-----
  python3 benchmark_euroc.py --config experiment_configuration.yaml
  python3 benchmark_euroc.py --config experiment_configuration.yaml --sequence MH_01_easy
  python3 benchmark_euroc.py
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
from datetime import datetime, timedelta, timezone
from pathlib import Path

try:
    import yaml
except ImportError:
    print("ERROR: PyYAML required. pip3 install pyyaml")
    sys.exit(1)


SCRIPT_PATH = Path(__file__).resolve()
HOST_WS = SCRIPT_PATH.parent
CONTAINER_WS = Path("/workspace")
DEFAULT_CONTAINER = "orbslam2_container"
UTC_PLUS_4 = timezone(timedelta(hours=4))
RERUN_MATCH_THRESHOLD = 0.10
SUPPORTED_BAG_SUFFIXES = (".db3", ".mcap")


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


_log_fd = None
_all_procs: list[subprocess.Popen] = []


def _w(text: str) -> None:
    sys.stdout.write(text)
    sys.stdout.flush()
    if _log_fd:
        _log_fd.write(text)
        _log_fd.flush()


def msg(color, icon, label, detail=""):
    suffix = f"  {C.DIM}{detail}" if detail else ""
    _w(f"  {icon}  {C.BOLD}{color}{label}{C.RESET}{suffix}\n")


def ok(label, detail=""):
    msg(C.GREEN, f"{C.GREEN}✓{C.RESET}", label, detail)


def fail(label, detail=""):
    msg(C.RED, f"{C.RED}✗{C.RESET}", label, detail)


def step(label, detail=""):
    msg(C.CYAN, f"{C.CYAN}▶{C.RESET}", label, detail)


def stop(label, detail=""):
    msg(C.YELLOW, f"{C.YELLOW}■{C.RESET}", label, detail)


def section(title):
    pad = max(0, 56 - len(title))
    _w(f"\n{C.BOLD}{C.WHITE}  ── {title} {C.DIM}{'─' * pad}{C.RESET}\n")


def banner(title):
    bar = "═" * 66
    _w(f"\n{C.BOLD}{C.BLUE}╔{bar}╗\n")
    _w(f"{C.BOLD}{C.BLUE}║  {C.WHITE}{title:<64}{C.BLUE}║\n")
    _w(f"{C.BOLD}{C.BLUE}╚{bar}╝{C.RESET}\n")


def seq_banner(seq_name, idx, total):
    _w(f"\n{C.BOLD}{C.MAGENTA}  {'━' * 66}\n")
    _w(f"  Sequence {idx}/{total}: {C.WHITE}{seq_name}{C.RESET}\n")
    _w(f"{C.BOLD}{C.MAGENTA}  {'━' * 66}{C.RESET}\n")


def run_banner(run_num, n_retries):
    _w(f"\n{C.BOLD}{C.CYAN}  {'─' * 66}\n")
    _w(f"  Run {run_num}/{n_retries}\n")
    _w(f"  {'─' * 66}{C.RESET}\n")


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


def q(value) -> str:
    return shlex.quote(str(value))


def _reset_state():
    global _all_procs
    _all_procs = []


def _sigint_proc(proc: subprocess.Popen | None):
    if proc is None:
        return
    try:
        os.killpg(proc.pid, signal.SIGINT)
    except Exception:
        pass


def _wait_for_proc(proc: subprocess.Popen | None, timeout_sec: float) -> bool:
    if proc is None:
        return True
    deadline = time.time() + max(0.0, timeout_sec)
    while time.time() < deadline:
        if proc.poll() is not None:
            return True
        time.sleep(0.2)
    return proc.poll() is not None


def nuke(reason=""):
    if reason:
        _w(f"\n  {C.YELLOW}⚡  {C.BOLD}{reason}{C.RESET}\n")
    for proc in _all_procs:
        try:
            os.killpg(proc.pid, signal.SIGKILL)
        except Exception:
            pass
    for pat in [
        "ros2 bag play",
        "ros2 run orbslam mono",
        "ros2 run orbslam stereo",
        "euroc_rectify.py",
        "euroc_mono_relay.py",
        "ros2",
        "orbslam",
    ]:
        subprocess.call(["pkill", "-9", "-f", pat], stderr=subprocess.DEVNULL)
    _w(f"  {C.YELLOW}■  All processes killed.{C.RESET}\n")


def _sigint(signum, frame):  # type: ignore[unused-argument]
    nuke("CTRL+C — killing everything")
    if _log_fd:
        try:
            _log_fd.close()
        except Exception:
            pass
    sys.exit(130)


def in_container() -> bool:
    return Path("/.dockerenv").exists()


def _to_container(path: str) -> str:
    p = Path(path)
    if p.is_absolute():
        try:
            return str(CONTAINER_WS / p.relative_to(HOST_WS))
        except ValueError:
            return str(p)
    return str(CONTAINER_WS / p)


def resolve_path(root: Path, raw_path: str | Path) -> Path:
    p = Path(raw_path)
    if p.is_absolute():
        return p
    return (root / p).resolve()


def resolve_under(base: Path, raw_path: str | Path) -> Path:
    p = Path(raw_path)
    if p.is_absolute():
        return p
    return (base / p).resolve()


def build_shell_script(cmd: str, setup_scripts: list[str], ws: Path) -> str:
    lines = [
        "set -e",
        'source_safe(){ if [ -f "$1" ]; then set +u; . "$1"; set -u; fi; }',
        f'export LD_LIBRARY_PATH={q(str(ws / "lib"))}:/usr/local/lib:${{LD_LIBRARY_PATH:-}}',
        f'export LIBRARY_PATH={q(str(ws / "lib"))}:/usr/local/lib:${{LIBRARY_PATH:-}}',
    ]
    for script in setup_scripts:
        path = Path(script)
        if not path.is_absolute():
            path = resolve_path(ws, script)
        lines.append(f"source_safe {q(path)}")
    lines.append(cmd)
    return "\n".join(lines)


def _bg(cmd: str, setup_scripts: list[str], ws: Path, log_path=None, line_fn=None, cwd: Path | None = None):
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

        def _worker():
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


def _run(cmd: str, setup_scripts: list[str], ws: Path, cwd: Path | None = None, capture_output=True):
    return subprocess.run(
        ["bash", "-lc", build_shell_script(cmd, setup_scripts, ws)],
        cwd=str(cwd) if cwd else None,
        capture_output=capture_output,
        text=True,
    )


DEFAULT_CONFIG = {
    "experiment": {
        "mode": "stereo",
    },
    "dataset": {
        "base_path": ".",
        "sequences_dir": ".",
        "gt_dir": ".",
        "sequences": ["MH_01_easy"],
    },
    "orbslam": {
        "package": "orbslam",
        "vocabulary_path": "Vocabulary/ORBvoc.txt",
        "mono_settings_path": "ros2-ORB_SLAM2/src/monocular/EuRoC.yaml",
        "stereo_settings_path": "ros2-ORB_SLAM2/src/stereo/EuRoC.yaml",
        "setup_scripts": [
            "/opt/ros/jazzy/setup.bash",
            "/root/ws/install/setup.bash",
            "install_stereo/setup.bash",
            "install_stereo/setup.sh",
        ],
    },
    "playback": {
        "bag_rate": 1.0,
        "launch_settle_sec": 15.0,
        "post_bag_wait_sec": 2.0,
        "shutdown_grace_sec": 6.0,
        "cooldown_sec": 5.0,
    },
    "evaluation": {
        "n_retries": 1,
        "t_max_diff": 0.01,
        "rerun_match_threshold": RERUN_MATCH_THRESHOLD,
        "results_dir": "results",
        "log_dir": "run_logs",
        "cpu_sample_sec": 0.05,
    },
}


def load_config(path: Path | None) -> dict:
    cfg = {}
    if path and path.exists():
        with open(path) as f:
            cfg = yaml.safe_load(f) or {}
    elif path:
        fail("Config file not found", str(path))
        sys.exit(1)

    merged = json.loads(json.dumps(DEFAULT_CONFIG))
    for section_key, defaults in DEFAULT_CONFIG.items():
        if section_key not in cfg:
            continue
        if isinstance(defaults, dict) and isinstance(cfg[section_key], dict):
            merged[section_key].update(cfg[section_key])
        else:
            merged[section_key] = cfg[section_key]
    return merged


def find_sequences(cfg: dict, ws: Path) -> list[tuple[str, Path, Path]]:
    dataset_cfg = cfg["dataset"]
    base = resolve_path(ws, dataset_cfg.get("base_path", "."))
    seq_dir = resolve_under(base, dataset_cfg.get("sequences_dir", "."))
    gt_dir = resolve_under(base, dataset_cfg.get("gt_dir", "."))
    seqs_cfg = dataset_cfg.get("sequences", "all")

    if seqs_cfg == "all":
        names = []
        if seq_dir.exists():
            for entry in sorted(seq_dir.iterdir()):
                if entry.is_file() and entry.suffix.lower() in SUPPORTED_BAG_SUFFIXES:
                    names.append(entry.stem)
                elif entry.is_dir() and any(
                    child.is_file() and child.suffix.lower() in SUPPORTED_BAG_SUFFIXES
                    for child in entry.iterdir()
                ):
                    names.append(entry.name)
        if not names:
            fail("No bags found in sequences_dir", str(seq_dir))
    else:
        names = list(seqs_cfg)

    results: list[tuple[str, Path, Path]] = []
    for name in names:
        bag = seq_dir / f"{name}.db3"
        if not bag.exists():
            bag = seq_dir / f"{name}.mcap"
        if not bag.exists():
            bag = seq_dir / name
        gt = gt_dir / f"{name}.txt"
        if not gt.exists():
            gt = gt_dir / f"{name}.tum"
        if not gt.exists():
            gt = gt_dir / name / "gt.tum"
        if not bag.exists():
            stop(f"Skipping {name}", f"bag not found: {bag}")
            continue
        if not gt.exists():
            stop(f"Skipping {name}", f"ground truth not found: {gt}")
            continue
        results.append((name, bag, gt))
    return results


def mode_profile(cfg: dict, ws: Path) -> dict:
    mode = str(cfg.get("experiment", {}).get("mode", "stereo")).strip().lower()
    if mode not in {"mono", "stereo"}:
        fail("Unsupported experiment.mode", f"{mode} (expected mono or stereo)")
        sys.exit(1)

    orb_cfg = cfg.get("orbslam", {})
    setup_scripts = list(orb_cfg.get("setup_scripts", []))
    package = str(orb_cfg.get("package", "orbslam"))
    vocabulary_path = resolve_path(ws, orb_cfg.get("vocabulary_path", "Vocabulary/ORBvoc.txt"))

    if mode == "mono":
        settings_path = resolve_path(
            ws, orb_cfg.get("mono_settings_path", "ros2-ORB_SLAM2/src/monocular/EuRoC.yaml")
        )
        helper_script = resolve_path(ws, "tools/euroc_mono_relay.py")
        return {
            "mode": mode,
            "package": package,
            "setup_scripts": setup_scripts,
            "vocabulary_path": vocabulary_path,
            "settings_path": settings_path,
            "helper_script": helper_script,
            "cpu_monitor_script": resolve_path(ws, "tools/monitor_cpu_orb.py"),
            "postprocess_script": resolve_path(ws, "tools/postprocess_orb.py"),
            "helper_label": "euroc_mono_relay",
            "helper_detail": "/cam0/image_raw → /camera",
            "trajectory_native": "KeyFrameTrajectory.txt",
            "scale_correction": True,
        }

    settings_path = resolve_path(
        ws, orb_cfg.get("stereo_settings_path", "ros2-ORB_SLAM2/src/stereo/EuRoC.yaml")
    )
    helper_script = resolve_path(ws, "tools/euroc_rectify.py")
    return {
        "mode": mode,
        "package": package,
        "setup_scripts": setup_scripts,
        "vocabulary_path": vocabulary_path,
        "settings_path": settings_path,
        "helper_script": helper_script,
        "cpu_monitor_script": resolve_path(ws, "tools/monitor_cpu_orb.py"),
        "postprocess_script": resolve_path(ws, "tools/postprocess_orb.py"),
        "helper_label": "euroc_rectify",
        "helper_detail": "/cam0|1/image_raw → /camera/left|right",
        "trajectory_native": "Stereo_KeyFrameTrajectory.txt",
        "scale_correction": False,
    }


def preflight_checks(mode_cfg: dict, ws: Path):
    section("Preflight")
    for label, path in [
        ("Vocabulary", mode_cfg["vocabulary_path"]),
        ("Settings", mode_cfg["settings_path"]),
        ("Helper script", mode_cfg["helper_script"]),
        ("CPU monitor", mode_cfg["cpu_monitor_script"]),
        ("Postprocess script", mode_cfg["postprocess_script"]),
    ]:
        if Path(path).exists():
            ok(label, str(path))
        else:
            fail(label, f"missing: {path}")
            sys.exit(1)

    for script in mode_cfg["setup_scripts"]:
        path = Path(script)
        if not path.is_absolute():
            path = resolve_path(ws, script)
        if path.exists():
            ok("Setup script", str(path))
        else:
            stop("Setup script missing", str(path))

    required_commands = ["ros2", "evo_ape", "evo_traj"]
    found_commands = {}
    missing_commands = []
    for command_name in required_commands:
        check = _run(f"command -v {q(command_name)}", mode_cfg["setup_scripts"], ws)
        path = (check.stdout or "").strip()
        if check.returncode == 0 and path:
            found_commands[command_name] = path
        else:
            missing_commands.append(command_name)

    if missing_commands:
        for command_name, path in found_commands.items():
            ok(f"Required command: {command_name}", path)
        fail(
            "Required commands missing",
            f"missing={', '.join(missing_commands)}"
            + (
                f"  found={', '.join(f'{name}={path}' for name, path in found_commands.items())}"
                if found_commands else ""
            ),
        )
        sys.exit(1)

    ok(
        "Required commands",
        ", ".join(f"{name}={path}" for name, path in found_commands.items()),
    )

    matplotlib_check = _run('python3 -c "import matplotlib"', mode_cfg["setup_scripts"], ws)
    if matplotlib_check.returncode != 0:
        fail("Python dependency missing", "matplotlib is required for benchmark post-processing")
        sys.exit(1)
    ok("Python dependencies", "matplotlib")

    rmw = os.environ.get("RMW_IMPLEMENTATION", "")
    ok("DDS / RMW", rmw if rmw else "rmw_cyclonedds_cpp (ROS2 default)")


def bootstrap_environment(mode_cfg: dict, ws: Path):
    section("Environment Bootstrap")
    setup_scripts = mode_cfg["setup_scripts"]
    step("Sourcing setup scripts", "ROS2 + ORB-SLAM2 overlay")
    bootstrap = _run(
        'echo "ROS_DISTRO=${ROS_DISTRO:-unset}" && command -v ros2 && command -v python3',
        setup_scripts,
        ws,
    )
    if bootstrap.stdout:
        for line in bootstrap.stdout.splitlines():
            evo_line(line)
    if bootstrap.stderr:
        for line in bootstrap.stderr.splitlines():
            evo_line(line)
    if bootstrap.returncode != 0:
        fail("Environment bootstrap", f"rc={bootstrap.returncode}")
        sys.exit(1)
    ok("Environment bootstrap", "setup scripts sourced successfully")


def parse_evo_stats(text: str) -> dict:
    stats = {}
    for metric in ("rmse", "mean", "median", "std", "max", "min", "sse"):
        match = re.search(rf"^\s*{metric}\s+([\d.eE+\-]+)", text, re.MULTILINE)
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


def materialize_plot_alias(target: Path, suffixes: list[str]) -> Path | None:
    if target.exists():
        return target
    stem = target.stem
    for suffix in suffixes:
        candidate = target.with_name(f"{stem}{suffix}")
        if candidate.exists():
            shutil.copy2(candidate, target)
            return target
    return None


def run_evo(seq_name: str, tum_path: Path, gt_path: Path, eval_cfg: dict, mode_cfg: dict, run_dir: Path, log_dir: Path, stamp: str):
    run_dir.mkdir(parents=True, exist_ok=True)
    t_max = float(eval_cfg.get("t_max_diff", 0.01))
    match_thr = float(eval_cfg.get("rerun_match_threshold", RERUN_MATCH_THRESHOLD))
    ape_plot = run_dir / "ape_traj.png"
    traj_plot = run_dir / "traj_xy.png"
    stats_file = run_dir / "ape_stats.json"
    total_poses = _count_poses(tum_path)

    ape_flags = "-a -s" if mode_cfg["scale_correction"] else "-a"
    traj_flags = "-a --correct_scale" if mode_cfg["scale_correction"] else "-a"

    section(f"evo_ape  [{seq_name}]")
    step(
        "evo_ape",
        f"gt={gt_path.name}  est={tum_path.name}  total_poses={total_poses}  mode={mode_cfg['mode']}",
    )

    ape_cmd = (
        f"evo_ape tum {q(gt_path)} {q(tum_path)} "
        f"{ape_flags} --t_max_diff {t_max} --verbose "
        f"--plot_mode xy --save_plot {q(ape_plot)}"
    )
    evo_log = log_dir / f"evo_{seq_name}_{stamp}.log"
    evo_log_fd = open(evo_log, "w", buffering=1)
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
    stats["needs_rerun"] = match_ratio < match_thr

    with open(stats_file, "w") as f:
        json.dump(stats, f, indent=2)

    canonical_ape_plot = materialize_plot_alias(ape_plot, ["_map.png", "_raw.png"])

    if proc.returncode == 0:
        rmse_str = f"{stats.get('rmse', float('nan')):.4f} m"
        match_str = f"{n_matched}/{total_poses} matched ({match_ratio * 100:.1f}%)"
        if stats["needs_rerun"]:
            _w(
                f"\n  {C.RED}⚠  NEEDS RERUN — only {match_ratio * 100:.1f}% of poses "
                f"matched GT (threshold {match_thr * 100:.0f}%){C.RESET}\n"
            )
            fail("evo_ape", f"RMSE = {rmse_str}  |  {match_str}  ← FLAGGED")
            flagged_log = run_dir.parent.parent / "flagged_runs.txt"
            with open(flagged_log, "a") as fl:
                fl.write(
                    f"{seq_name}/{run_dir.name}  mode={mode_cfg['mode']}  "
                    f"match={match_ratio * 100:.1f}%  poses={total_poses}  "
                    f"RMSE={stats.get('rmse', float('nan')):.4f}m\n"
                )
        else:
            ok("evo_ape", f"RMSE = {rmse_str}  |  {match_str}")
            if canonical_ape_plot:
                ok("APE plot", canonical_ape_plot.name)
    else:
        fail("evo_ape", f"rc={proc.returncode}")

    step("evo_traj", f"trajectory XY overlay → {traj_plot.name}")
    traj_cmd = (
        f"evo_traj tum {q(tum_path)} --ref {q(gt_path)} "
        f"{traj_flags} --plot_mode xy --save_plot {q(traj_plot)}"
    )
    result = subprocess.run(["bash", "-lc", traj_cmd], capture_output=True, text=True)
    if result.returncode == 0:
        canonical_traj_plot = materialize_plot_alias(
            traj_plot,
            ["_trajectories.png", "_xyz.png", "_rpy.png", "_speeds.png"],
        )
        ok("evo_traj", str(canonical_traj_plot or traj_plot))
    else:
        fail("evo_traj", result.stderr.strip()[:160])

    return stats


def cleanup_stale_trajectories(mode_cfg: dict, ws: Path, run_dir: Path):
    for path in [
        run_dir / mode_cfg["trajectory_native"],
        run_dir / "trajectory.tum",
        ws / mode_cfg["trajectory_native"],
    ]:
        try:
            path.unlink()
        except FileNotFoundError:
            pass


def locate_trajectory(mode_cfg: dict, ws: Path, run_dir: Path) -> Path | None:
    for candidate in [
        run_dir / mode_cfg["trajectory_native"],
        ws / mode_cfg["trajectory_native"],
    ]:
        if candidate.exists() and candidate.stat().st_size > 0:
            return candidate
    return None


def run_sequence(
    seq_name: str,
    bag_path: Path,
    gt_path: Path,
    cfg: dict,
    ws: Path,
    mode_cfg: dict,
    stamp: str,
    run_dir: Path,
    run_num: int,
    n_retries: int,
):
    _reset_state()
    signal.signal(signal.SIGINT, _sigint)
    signal.signal(signal.SIGTERM, _sigint)

    playback_cfg = cfg.get("playback", {})
    eval_cfg = cfg.get("evaluation", {})
    setup_scripts = mode_cfg["setup_scripts"]
    bag_rate = float(playback_cfg.get("bag_rate", 1.0))
    settle_sec = float(playback_cfg.get("launch_settle_sec", 2.0))
    post_bag_wait_sec = float(playback_cfg.get("post_bag_wait_sec", 2.0))
    shutdown_grace_sec = float(playback_cfg.get("shutdown_grace_sec", 6.0))
    cpu_sample_sec = float(eval_cfg.get("cpu_sample_sec", 0.05))

    log_dir = run_dir / "logs"
    log_dir.mkdir(parents=True, exist_ok=True)
    run_dir.mkdir(parents=True, exist_ok=True)
    cleanup_stale_trajectories(mode_cfg, ws, run_dir)

    def logf(name: str) -> Path:
        return log_dir / f"{name}.log"

    trajectory_out = run_dir / "trajectory.tum"
    timing_csv = run_dir / "timing_events.csv"
    cpu_info_json = run_dir / "orbslam_pid_info.json"
    cpu_process_csv = run_dir / "orbslam_process_cpu.csv"
    cpu_thread_csv = run_dir / "orbslam_thread_cpu.csv"
    cpu_thread_summary_csv = run_dir / "orbslam_thread_cpu_summary.csv"

    section(f"Pre-launch: EuRoC Input Bridge  [{seq_name}  run {run_num}/{n_retries}]")
    step(mode_cfg["helper_label"], mode_cfg["helper_detail"])
    helper_proc = _bg(
        f"python3 {q(mode_cfg['helper_script'])}",
        setup_scripts,
        ws,
        logf("bridge"),
    )
    ok(mode_cfg["helper_label"])

    section(f"ORB-SLAM2 Launch  [{seq_name}  run {run_num}/{n_retries}]")
    step(
        "ros2 run orbslam",
        f"{mode_cfg['mode']}  settings={mode_cfg['settings_path'].name}",
    )
    launch_proc = _bg(
        " ".join(
            [
                "env",
                f"ORB_BENCH_TIMING_CSV={q(timing_csv)}",
                "ros2",
                "run",
                q(mode_cfg["package"]),
                q(mode_cfg["mode"]),
                q(mode_cfg["vocabulary_path"]),
                q(mode_cfg["settings_path"]),
            ]
        ),
        setup_scripts,
        ws,
        logf("orbslam"),
        line_fn=launch_line,
        cwd=run_dir,
    )

    _w(f"\n  {C.DIM}  sleeping {settle_sec:.1f} s for launch to settle…{C.RESET}\n")
    time.sleep(settle_sec)

    if helper_proc.poll() is not None:
        fail(mode_cfg["helper_label"], f"exited early rc={helper_proc.returncode}")
        nuke("Input bridge crashed — aborting run")
        return None
    ok("Input bridge running")

    if launch_proc.poll() is not None:
        fail("ORB-SLAM2 exited early", f"rc={launch_proc.returncode}")
        nuke("ORB-SLAM2 crashed — aborting run")
        return None
    ok("ORB-SLAM2 running")

    section(f"CPU Monitor  [{seq_name}  run {run_num}/{n_retries}]")
    step("monitor_cpu_orb", f"{mode_cfg['mode']}  sample={cpu_sample_sec:.2f} s")
    cpu_monitor_proc = _bg(
        " ".join(
            [
                "python3",
                q(mode_cfg["cpu_monitor_script"]),
                "--launcher-pid",
                str(launch_proc.pid),
                "--info-json",
                q(cpu_info_json),
                "--process-csv",
                q(cpu_process_csv),
                "--thread-csv",
                q(cpu_thread_csv),
                "--summary-csv",
                q(cpu_thread_summary_csv),
                "--sample-sec",
                str(cpu_sample_sec),
                "--target-exec",
                q(mode_cfg["mode"]),
            ]
        ),
        setup_scripts,
        ws,
        logf("cpu_monitor"),
    )
    ok("CPU monitor", f"launcher pid={launch_proc.pid}")

    section(f"Bag Playback  [{seq_name}  run {run_num}/{n_retries}]")
    step("ros2 bag play", f"{bag_path.name}  rate={bag_rate}×")
    wall_start = time.time()
    bag_proc = _bg(
        f"ros2 bag play {q(bag_path)} --clock --rate {bag_rate}",
        setup_scripts,
        ws,
        logf("bag"),
    )
    ok("Bag player", "running")

    section(f"Waiting for bag to finish  [{seq_name}  run {run_num}/{n_retries}]")
    while bag_proc.poll() is None:
        if helper_proc.poll() is not None:
            fail(mode_cfg["helper_label"], f"died mid-run rc={helper_proc.returncode}")
            nuke("Input bridge crashed")
            return None
        if launch_proc.poll() is not None:
            fail("ORB-SLAM2 died mid-run", f"rc={launch_proc.returncode}")
            nuke("ORB-SLAM2 crashed")
            return None
        time.sleep(1)

    bag_rc = bag_proc.returncode
    if bag_rc == 0:
        ok("Bag player", "finished")
    else:
        fail("Bag player", f"rc={bag_rc}")

    if post_bag_wait_sec > 0:
        _w(f"\n  {C.DIM}  waiting {post_bag_wait_sec:.1f} s for final frames…{C.RESET}\n")
        time.sleep(post_bag_wait_sec)

    step("Stopping ORB-SLAM2", f"SIGINT → wait {shutdown_grace_sec:.1f} s")
    _sigint_proc(launch_proc)
    orbslam_stopped = _wait_for_proc(launch_proc, max(0.5, shutdown_grace_sec - 1.0))
    _sigint_proc(helper_proc)
    helper_stopped = _wait_for_proc(helper_proc, 1.0)

    still_running = []
    if not orbslam_stopped:
        still_running.append("ORB-SLAM2")
    if not helper_stopped:
        still_running.append(mode_cfg["helper_label"])
    if still_running:
        stop("Graceful shutdown timeout", ", ".join(still_running))

    step("Stopping CPU monitor", "SIGINT → wait 3.0 s")
    _sigint_proc(cpu_monitor_proc)
    cpu_monitor_stopped = _wait_for_proc(cpu_monitor_proc, 3.0)
    if cpu_monitor_stopped:
        ok("CPU monitor", "stopped")
    else:
        stop("CPU monitor timeout", "will be force-killed")

    nuke("Bag done — shutting down ROS2 pipeline")

    trajectory_native = locate_trajectory(mode_cfg, ws, run_dir)
    if not trajectory_native:
        fail("Trajectory file missing or empty", mode_cfg["trajectory_native"])
        return None

    shutil.copy2(trajectory_native, trajectory_out)
    n_poses = _count_poses(trajectory_out)
    wall_time_sec = max(0.0, time.time() - wall_start)
    pose_rate_hz = (n_poses / wall_time_sec) if wall_time_sec > 0 else 0.0

    ok("Trajectory", f"{n_poses} poses  ·  {trajectory_out.name}")

    stats = run_evo(
        seq_name,
        trajectory_out,
        gt_path,
        eval_cfg,
        mode_cfg,
        run_dir=run_dir,
        log_dir=log_dir,
        stamp=stamp,
    )
    stats["run"] = run_num
    stats["mode"] = mode_cfg["mode"]
    stats["wall_time_sec"] = round(wall_time_sec, 3)
    stats["trajectory_poses"] = n_poses
    stats["trajectory_rate_hz"] = round(pose_rate_hz, 3)
    stats["bag_rate"] = bag_rate
    stats["bag_exit_code"] = bag_rc
    stats["timing_events_csv"] = timing_csv.name

    section(f"Post-process  [{seq_name}  run {run_num}/{n_retries}]")
    step("postprocess_orb", "timing summaries, CPU plots, thread validation")
    postprocess = _run(
        f"python3 {q(mode_cfg['postprocess_script'])} --run-dir {q(run_dir)}",
        setup_scripts,
        ws,
        cwd=run_dir,
    )
    if postprocess.stdout:
        for line in postprocess.stdout.splitlines():
            evo_line(line)
    if postprocess.stderr:
        for line in postprocess.stderr.splitlines():
            evo_line(line)
    if postprocess.returncode != 0:
        fail("postprocess_orb", f"rc={postprocess.returncode}")
        raise RuntimeError(
            f"Per-run post-processing failed for {seq_name} {run_dir.name}. "
            f"See {logf('cpu_monitor')} and {run_dir}."
        )
    ok("Post-process", "timing + CPU + thread validation complete")

    timing_means = load_timing_summary(run_dir / "timing_summary.csv")
    cpu_summary = load_json_file(run_dir / "cpu_summary.json")
    thread_report = load_json_file(run_dir / "thread_identity_report.json")
    stats["frontend_full_tracking_avg_ms"] = timing_means.get("frontend/full_tracking")
    stats["cpu_total_avg_pct"] = cpu_summary.get("avg_pct")
    stats["cpu_summary"] = cpu_summary
    stats["timing_avg_ms_by_category"] = timing_means
    stats["thread_identity"] = thread_report

    with open(run_dir / "run_meta.json", "w") as f:
        json.dump(stats, f, indent=2)
    stats["artifacts"] = build_artifact_manifest(run_dir)
    with open(run_dir / "run_meta.json", "w") as f:
        json.dump(stats, f, indent=2)

    return stats


def _safe_mean(values):
    clean = [v for v in values if v is not None and not (isinstance(v, float) and v != v)]
    if not clean:
        return float("nan")
    return sum(clean) / len(clean)


def _safe_std(values):
    clean = [v for v in values if v is not None and not (isinstance(v, float) and v != v)]
    if len(clean) < 2:
        return 0.0
    mean = _safe_mean(clean)
    return (sum((v - mean) ** 2 for v in clean) / len(clean)) ** 0.5


def normalize_metric_key(category: str) -> str:
    out = []
    for char in category:
        if char.isalnum():
            out.append(char.lower())
        else:
            out.append("_")
    key = "".join(out)
    while "__" in key:
        key = key.replace("__", "_")
    return key.strip("_")


def load_json_file(path: Path) -> dict:
    if not path.exists():
        return {}
    try:
        return json.loads(path.read_text())
    except Exception:
        return {}


def load_timing_summary(path: Path) -> dict[str, float]:
    if not path.exists():
        return {}
    timing_means: dict[str, float] = {}
    with path.open(newline="") as handle:
        for row in csv.DictReader(handle):
            category = (row.get("category") or "").strip()
            if not category:
                continue
            try:
                timing_means[category] = float(row["avg_ms"])
            except Exception:
                continue
    return timing_means


def select_clean_runs(valid_runs: list[dict]) -> list[dict]:
    clean = [run for run in valid_runs if not run.get("needs_rerun")]
    return clean if clean else valid_runs


def metric_values(stats_list: list[dict], key: str) -> list[float]:
    values = []
    for stats in stats_list:
        value = stats.get(key)
        if value is None:
            continue
        if isinstance(value, float) and value != value:
            continue
        values.append(value)
    return values


def timing_category_values(stats_list: list[dict], category: str) -> list[float]:
    values = []
    for stats in stats_list:
        timing_map = stats.get("timing_avg_ms_by_category", {}) or {}
        value = timing_map.get(category)
        if value is None:
            continue
        if isinstance(value, float) and value != value:
            continue
        values.append(value)
    return values


def build_artifact_manifest(run_dir: Path) -> dict[str, str]:
    artifacts = {}
    for key, name in [
        ("trajectory_tum", "trajectory.tum"),
        ("ape_traj_png", "ape_traj.png"),
        ("traj_xy_png", "traj_xy.png"),
        ("ape_stats_json", "ape_stats.json"),
        ("timing_events_csv", "timing_events.csv"),
        ("timing_summary_csv", "timing_summary.csv"),
        ("orbslam_process_cpu_csv", "orbslam_process_cpu.csv"),
        ("orbslam_thread_cpu_csv", "orbslam_thread_cpu.csv"),
        ("orbslam_thread_cpu_summary_csv", "orbslam_thread_cpu_summary.csv"),
        ("orbslam_pid_info_json", "orbslam_pid_info.json"),
        ("cpu_summary_json", "cpu_summary.json"),
        ("thread_identity_report_json", "thread_identity_report.json"),
        ("orbslam_cpu_usage_png", "orbslam_cpu_usage.png"),
        ("orbslam_threads_cpu_bar_png", "orbslam_threads_cpu_bar.png"),
        ("run_meta_json", "run_meta.json"),
    ]:
        path = run_dir / name
        if path.exists():
            artifacts[key] = name
    return artifacts


def write_seq_summary(seq_name: str, mode: str, stats_list: list[dict], seq_dir: Path):
    valid = [s for s in stats_list if s.get("rmse") is not None]
    clean = [s for s in valid if not s.get("needs_rerun")]
    selected = select_clean_runs(valid)
    categories = sorted(
        {
            category
            for stats in valid
            for category in (stats.get("timing_avg_ms_by_category", {}) or {}).keys()
        }
    )

    section(f"Retry Summary  [{seq_name}]")
    hdr = (
        f"  {'Run':>5} {'RMSE (m)':>10} {'FE (ms)':>10} {'CPU %':>10} "
        f"{'Wall (s)':>10} {'Poses':>8} {'Rate (Hz)':>10}"
    )
    _w(f"{C.BOLD}{C.WHITE}{hdr}{C.RESET}\n  {'─' * 86}\n")
    for stats in stats_list:
        run_id = stats.get("run", "?")
        rmse = stats.get("rmse")
        if rmse is None:
            _w(f"  {run_id:>5} {C.RED}{'FAILED':>10}{C.RESET}\n")
            continue
        wall = stats.get("wall_time_sec", float("nan"))
        poses = stats.get("trajectory_poses", 0)
        rate = stats.get("trajectory_rate_hz", float("nan"))
        fe_ms = stats.get("frontend_full_tracking_avg_ms", float("nan"))
        cpu_pct = stats.get("cpu_total_avg_pct", float("nan"))
        match_ratio = stats.get("match_ratio", float("nan"))
        flag = f"  {C.RED}⚠ RERUN{C.RESET}" if stats.get("needs_rerun") else ""
        color = C.GREEN if rmse < 0.05 else C.YELLOW if rmse < 0.10 else C.RED
        _w(
            f"  {run_id:>5} {color}{rmse:>10.4f}{C.RESET} {fe_ms:>10.3f} {cpu_pct:>10.2f}"
            f" {wall:>10.2f} {poses:>8} {rate:>10.2f}  match={match_ratio * 100:.1f}%{flag}\n"
        )

    rmse_all = _safe_mean(metric_values(valid, "rmse"))
    rmse_std_all = _safe_std(metric_values(valid, "rmse"))
    rmse_clean = _safe_mean(metric_values(clean, "rmse"))
    rmse_std_clean = _safe_std(metric_values(clean, "rmse"))
    fe_all = _safe_mean(metric_values(valid, "frontend_full_tracking_avg_ms"))
    fe_clean = _safe_mean(metric_values(clean, "frontend_full_tracking_avg_ms"))
    cpu_selected = _safe_mean(metric_values(selected, "cpu_total_avg_pct"))
    color = C.GREEN if rmse_all < 0.05 else C.YELLOW if rmse_all < 0.10 else C.RED
    _w(f"  {'─' * 86}\n")
    _w(
        f"  {'all':>5} {color}{rmse_all:>10.4f}{C.RESET} {fe_all:>10.3f}"
        f" {cpu_selected:>10.2f}  clean_runs={len(clean):>2}/{len(valid):<2}\n"
    )
    if clean:
        clean_color = C.GREEN if rmse_clean < 0.05 else C.YELLOW if rmse_clean < 0.10 else C.RED
        _w(f"  {'clean':>5} {clean_color}{rmse_clean:>10.4f}{C.RESET} {fe_clean:>10.3f}\n")

    runs_payload = []
    for stats in stats_list:
        entry = dict(stats)
        run_num = int(stats.get("run", 0) or 0)
        run_dir = seq_dir / f"run_{run_num:02d}" if run_num > 0 else seq_dir
        artifacts = {}
        for key, name in (entry.get("artifacts", {}) or {}).items():
            artifacts[key] = f"{run_dir.name}/{name}"
        entry["artifacts"] = artifacts
        runs_payload.append(entry)

    summary = {
        "sequence": seq_name,
        "mode": mode,
        "n_runs_total": len(stats_list),
        "n_runs_completed": len(valid),
        "n_runs_clean": len(clean),
        "n_runs_flagged": len([s for s in valid if s.get("needs_rerun")]),
        "clean_selection_fallback_used": bool(valid and not clean),
        "rmse_avg_all": rmse_all,
        "rmse_std_all": rmse_std_all,
        "rmse_avg_clean": rmse_clean,
        "rmse_std_clean": rmse_std_clean,
        "wall_time_avg_sec": _safe_mean(metric_values(selected, "wall_time_sec")),
        "trajectory_poses_avg": _safe_mean(metric_values(selected, "trajectory_poses")),
        "trajectory_rate_avg_hz": _safe_mean(metric_values(selected, "trajectory_rate_hz")),
        "match_ratio_avg": _safe_mean(metric_values(selected, "match_ratio")),
        "cpu_total_avg_pct": cpu_selected,
        "frontend_full_tracking_avg_all_ms": fe_all,
        "frontend_full_tracking_avg_clean_ms": fe_clean,
        "timing_categories_observed": categories,
        "runs": runs_payload,
    }

    for category in categories:
        metric_key = f"timing_{normalize_metric_key(category)}"
        summary[f"{metric_key}_avg_ms"] = _safe_mean(timing_category_values(selected, category))
        summary[f"{metric_key}_avg_all_ms"] = _safe_mean(timing_category_values(valid, category))
        summary[f"{metric_key}_avg_clean_ms"] = _safe_mean(timing_category_values(clean, category))

    seq_dir.mkdir(parents=True, exist_ok=True)
    with open(seq_dir / "run_summary.json", "w") as f:
        json.dump(summary, f, indent=2)
    return summary


def write_summary(seq_summaries: list[dict], results_root: Path):
    if not seq_summaries:
        return
    results_root.mkdir(parents=True, exist_ok=True)
    csv_path = results_root / "experiment_summary.csv"
    base_fieldnames = [
        "sequence",
        "mode",
        "n_runs_total",
        "n_runs_completed",
        "n_runs_clean",
        "n_runs_flagged",
        "clean_selection_fallback_used",
        "rmse_avg_all",
        "rmse_std_all",
        "rmse_avg_clean",
        "rmse_std_clean",
        "wall_time_avg_sec",
        "trajectory_poses_avg",
        "trajectory_rate_avg_hz",
        "match_ratio_avg",
        "cpu_total_avg_pct",
        "frontend_full_tracking_avg_all_ms",
        "frontend_full_tracking_avg_clean_ms",
    ]
    dynamic_fieldnames = sorted(
        {
            key
            for row in seq_summaries
            for key in row.keys()
            if key.startswith("timing_") and key not in {"timing_categories_observed"}
        }
    )
    fieldnames = base_fieldnames + dynamic_fieldnames
    with open(csv_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames, extrasaction="ignore")
        writer.writeheader()
        for row in seq_summaries:
            writer.writerow(row)

    section("Experiment Summary")
    hdr = (
        f"  {'Sequence':<22} {'Mode':<8} {'Runs':>9} {'Flag':>5} "
        f"{'RMSE all':>10} {'RMSE clean':>12} {'FE clean':>10} {'CPU %':>10}"
    )
    _w(f"{C.BOLD}{C.WHITE}{hdr}{C.RESET}\n  {'─' * 64}\n")
    for row in seq_summaries:
        rmse_all = row.get("rmse_avg_all", float("nan"))
        rmse_clean = row.get("rmse_avg_clean", float("nan"))
        fe_clean = row.get("frontend_full_tracking_avg_clean_ms", float("nan"))
        cpu_pct = row.get("cpu_total_avg_pct", float("nan"))
        color = C.GREEN if rmse_all < 0.05 else C.YELLOW if rmse_all < 0.10 else C.RED
        _w(
            f"  {row['sequence']:<22} {row['mode']:<8} "
            f"{row['n_runs_clean']:>2}/{row['n_runs_total']:<6} {row['n_runs_flagged']:>5}"
            f" {color}{rmse_all:>10.4f}{C.RESET} {rmse_clean:>12.4f}"
            f" {fe_clean:>10.3f} {cpu_pct:>10.2f}\n"
        )
    _w(f"\n  {C.DIM}CSV → {csv_path}{C.RESET}\n")


def run_from_host(args) -> None:
    cmd = [
        "sudo",
        "docker",
        "exec",
        "-it",
        args.container,
        "python3",
        str(CONTAINER_WS / SCRIPT_PATH.name),
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


def parse_args():
    parser = argparse.ArgumentParser(
        description="ORB-SLAM2 EuRoC multi-sequence benchmark runner.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--inside-container", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument("--container", default=DEFAULT_CONTAINER, help="Docker container name (host-side only)")
    parser.add_argument(
        "--config",
        type=Path,
        default=None,
        help="Path to experiment_configuration.yaml. If omitted, uses built-in defaults.",
    )
    parser.add_argument(
        "--sequence",
        default=None,
        help="Run only this sequence (overrides config dataset.sequences).",
    )
    args = parser.parse_args()
    args.workspace = CONTAINER_WS if (in_container() or args.inside_container) else HOST_WS
    return args


def main():
    args = parse_args()
    ws = args.workspace

    cfg = load_config(args.config)
    if args.sequence:
        cfg["dataset"]["sequences"] = [args.sequence]

    eval_cfg = cfg.get("evaluation", {})
    playback_cfg = cfg.get("playback", {})
    mode_cfg = mode_profile(cfg, ws)

    log_dir = resolve_path(ws, eval_cfg.get("log_dir", "run_logs"))
    results_dir = resolve_path(ws, eval_cfg.get("results_dir", "results"))
    n_retries = int(eval_cfg.get("n_retries", 1))
    log_dir.mkdir(parents=True, exist_ok=True)

    utc4_stamp = datetime.now(UTC_PLUS_4).strftime("%Y%m%d_%H%M%S")
    results_root = results_dir / utc4_stamp
    results_root.mkdir(parents=True, exist_ok=True)

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    master_log = log_dir / f"terminal_{stamp}.log"

    global _log_fd
    _log_fd = open(master_log, "w", buffering=1)
    _w(f"  {C.DIM}master log  : {master_log}{C.RESET}\n")

    signal.signal(signal.SIGINT, _sigint)
    signal.signal(signal.SIGTERM, _sigint)

    try:
        if not (in_container() or args.inside_container):
            run_from_host(args)
            return

        bootstrap_environment(mode_cfg, ws)
        preflight_checks(mode_cfg, ws)

        sequences = find_sequences(cfg, ws)
        if not sequences:
            fail("No valid sequences found — check your dataset paths.")
            sys.exit(1)

        banner(
            f"ORB-SLAM2 EuRoC Benchmark  "
            f"({mode_cfg['mode'].upper()} · {len(sequences)} seq × {n_retries} retr)"
        )
        _w(f"  {C.DIM}config      : {args.config or 'built-in defaults'}\n")
        _w(f"  {C.DIM}mode        : {mode_cfg['mode']}\n")
        _w(f"  {C.DIM}vocabulary  : {mode_cfg['vocabulary_path']}\n")
        _w(f"  {C.DIM}settings    : {mode_cfg['settings_path']}\n")
        _w(f"  {C.DIM}log dir     : {log_dir}\n")
        _w(f"  {C.DIM}results     : {results_root}\n")
        _w(f"  {C.DIM}bag rate    : {playback_cfg.get('bag_rate', 1.0)}×\n")
        _w(f"  {C.DIM}retries     : {n_retries}\n")
        _w(f"  {C.DIM}sequences   : {[s[0] for s in sequences]}{C.RESET}\n")

        seq_run_stats_map: dict[str, list[dict]] = {seq_name: [] for seq_name, _, _ in sequences}
        seq_summaries: list[dict] = []
        total_runs = n_retries * len(sequences)
        global_run = 0
        cooldown_sec = float(playback_cfg.get("cooldown_sec", 5.0))

        for run_num in range(1, n_retries + 1):
            for seq_name, bag_path, gt_path in sequences:
                global_run += 1
                seq_banner(seq_name, global_run, total_runs)
                run_banner(run_num, n_retries)

                seq_dir = results_root / seq_name
                run_dir = seq_dir / f"run_{run_num:02d}"
                stats = run_sequence(
                    seq_name,
                    bag_path,
                    gt_path,
                    cfg,
                    ws,
                    mode_cfg,
                    stamp,
                    run_dir,
                    run_num,
                    n_retries,
                )
                if stats:
                    seq_run_stats_map[seq_name].append(stats)
                else:
                    seq_run_stats_map[seq_name].append(
                        {"sequence": seq_name, "mode": mode_cfg["mode"], "run": run_num}
                    )

                if global_run < total_runs and cooldown_sec > 0:
                    _w(f"\n  {C.DIM}Cooling down {cooldown_sec:.1f} s before next run…{C.RESET}\n")
                    time.sleep(cooldown_sec)

        for seq_name, _, _ in sequences:
            seq_dir = results_root / seq_name
            stats_list = seq_run_stats_map[seq_name]
            seq_summary = write_seq_summary(seq_name, mode_cfg["mode"], stats_list, seq_dir)
            if seq_summary:
                seq_summaries.append(seq_summary)

        write_summary(seq_summaries, results_root)

        section("Aggregate Post-process")
        step("postprocess_orb", "root plots, trajectory copies, markdown gallery")
        aggregate = _run(
            f"python3 {q(mode_cfg['postprocess_script'])} --aggregate --results-root {q(results_root)}",
            mode_cfg["setup_scripts"],
            ws,
        )
        if aggregate.stdout:
            for line in aggregate.stdout.splitlines():
                evo_line(line)
        if aggregate.stderr:
            for line in aggregate.stderr.splitlines():
                evo_line(line)
        if aggregate.returncode != 0:
            fail("aggregate postprocess", f"rc={aggregate.returncode}")
            raise RuntimeError(f"Aggregate post-processing failed for {results_root}")
        ok("Aggregate postprocess", "plots + markdown gallery complete")

        _w(f"\n  {C.BOLD}{C.GREEN}All sequences complete.{C.RESET}\n")
        _w(f"  {C.DIM}Results  : {results_root}\n")
        _w(f"  {C.DIM}Logs     : {log_dir}\n\n")
    finally:
        try:
            if _log_fd:
                _log_fd.close()
        except Exception:
            pass


if __name__ == "__main__":
    main()
