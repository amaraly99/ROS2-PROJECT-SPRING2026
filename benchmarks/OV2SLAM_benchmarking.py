#!/usr/bin/env python3

"""Standalone OV2SLAM benchmark runner with ORB_SLAM2-style UI and aggregation.

Loads experiment_configuration.yaml, launches OV2SLAM, plays EuRoC bags, monitors
/proc CPU usage, evaluates the produced trajectory with evo, and writes ORB-style
run/experiment summaries.

Lifecycle model (the important part)
------------------------------------
OV2SLAM detects the end of the image stream on its own, writes its trajectory
files, and exits cleanly. The benchmarker therefore does NOT treat OV2SLAM
exiting as a crash. A run is a SUCCESS when a valid trajectory file is produced;
it is a FAILURE only when no usable trajectory exists (a real crash) or a setup
step (node start, bag playback, /clock) fails. This keeps the runner simple and
free of the false "crash during bag playback" reports that plagued earlier
versions.
"""

from __future__ import annotations

import argparse
import copy
import csv
import json
import os
import re
import shlex
import shutil
import subprocess
import sys
import time
from pathlib import Path
from typing import Any

from benchmark_common import (
    BagInfo,
    Console,
    ProcessMonitor,
    RESULTS_TIMEZONE,
    ShellContext,
    coerce_bool,
    count_tum_poses,
    datetime_compact,
    detect_ros_setup_files,
    discover_sequences,
    ensure_list,
    generate_aggregate_outputs,
    generate_cpu_plots,
    mean_csv_column,
    prepare_ground_truth,
    resolve_bag_target,
    resolve_dir,
    resolve_file,
    run_capture,
    run_evo_and_plots,
    sequence_key,
    select_sequences,
    terminate_process_group,
    wait_for_clock_message,
    wait_for_pid_exit,
    wait_for_ros2_node,
    launch_background,
    launch_background_mirrored,
)

try:
    import yaml
except ImportError:
    yaml = None  # type: ignore[assignment]


SCRIPT_PATH = Path(__file__).resolve()
SCRIPT_DIR = SCRIPT_PATH.parent
LOCAL_WORKSPACE_ROOT = SCRIPT_DIR.parent
DEFAULT_WORKSPACE_ROOT = Path("/workspace") if Path("/workspace").is_dir() else LOCAL_WORKSPACE_ROOT
DEFAULT_CONFIG_PATH = SCRIPT_DIR / "experiment_configuration.yaml"
DEFAULT_DATASET_ROOT = DEFAULT_WORKSPACE_ROOT / "datasets" / "euroc"
DEFAULT_OUTPUT_ROOT = DEFAULT_WORKSPACE_ROOT / "results" / "ov2slam_benchmark_statistical"
PLOT_NAMED_THREAD_ROLES = SCRIPT_DIR / "plot_named_thread_roles.py"

DEFAULT_MODE = "stereo"
DEFAULT_PROFILE = "accurate"
DEFAULT_TRAJECTORY_POLICY = "paper"
DEFAULT_RUNS = 1
DEFAULT_RETRIES = 0

# Trajectory file preference, by policy. Each entry is (filename, why-it-was-chosen).
BEST_TRAJECTORY_CANDIDATES = [
    ("ov2slam_fullba_kfs_traj.txt", "final optimized keyframe trajectory after full BA; best final estimate when do_full_ba=1"),
    ("ov2slam_kfs_traj.txt", "optimized keyframe trajectory after local BA / loop closure; primary fallback"),
    ("ov2slam_traj.txt", "raw frame trajectory; fallback only when optimized keyframe outputs are absent"),
]
PAPER_TRAJECTORY_CANDIDATES = [
    ("ov2slam_kfs_traj.txt", "paper-style choice: keyframe trajectory after online local BA / loop closure, without end-of-sequence full BA"),
    ("ov2slam_traj.txt", "paper-style fallback: raw frame trajectory when keyframe output is missing"),
    ("ov2slam_fullba_kfs_traj.txt", "last-resort fallback: full BA result (offline refinement beyond the paper's fully-online setting)"),
]
UNSUPPORTED_EVO_TRAJECTORIES = [
    "ov2slam_full_traj_wlc.txt",
    "ov2slam_full_traj_wlc_opt.txt",
]

DEFAULT_CONFIG: dict[str, Any] = {
    "experiment": {
        "mode": DEFAULT_MODE,
        "profile": DEFAULT_PROFILE,
        "trajectory_policy": DEFAULT_TRAJECTORY_POLICY,
    },
    "dataset": {
        "workspace_root": "",
        "dataset_root": "",
        "sequences": [],
        "seq_range_start": "",
        "seq_range_end": "",
    },
    "ov2slam": {
        "params_file": "",
        "node_name": "/ov2slam_node",
        "cores": "",
        "topic_left": "",
        "topic_right": "",
        "node_wait_timeout": 45,
    },
    "playback": {
        "bag_rate": 1.0,
        "bag_timeout": 1200,
        "use_bag_qos_overrides": False,
        "startup_settle_sec": 2,
        "auto_exit_timeout": 600,
        "clock_start_timeout": 10,
        "sigint_timeout": 30,
    },
    "evaluation": {
        "n_runs": DEFAULT_RUNS,
        "retries": DEFAULT_RETRIES,
        "results_dir": "",
        "results_name": "",
        "log_dir": "",
        "t_max_diff": 0.1,
        "monitor_sample_sec": 0.05,
        "inter_run_delay_sec": 10,
    },
}

console = Console()


def die(message: str) -> None:
    console.fail(message)
    raise SystemExit(1)


# ──────────────────────────────────────────────────────────────────────────
# CLI + configuration loading
# ──────────────────────────────────────────────────────────────────────────
def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        prog=SCRIPT_PATH.name,
        description="Standalone OV2SLAM benchmark runner (ORB_SLAM2-style UI and aggregation).",
    )
    parser.add_argument("--config", default=str(DEFAULT_CONFIG_PATH))
    parser.add_argument("--mode", choices=["mono", "stereo"])
    parser.add_argument("--profile", choices=["fast", "average", "accurate"])
    parser.add_argument("--sequence", action="append", dest="sequences", default=[])
    parser.add_argument("--runs", type=int)
    parser.add_argument("--trajectory-policy", choices=["paper", "best"])
    parser.add_argument("--params")
    parser.add_argument("--cores")
    parser.add_argument("--workspace-root")
    parser.add_argument("--dataset-root")
    parser.add_argument("--output-root")
    parser.add_argument("--results-name")
    parser.add_argument("--retries", type=int)
    parser.add_argument("--engine-retries", type=int, help="Backward-compatible alias for --retries.")
    parser.add_argument("--all", action="store_true", help="Benchmark all discovered sequences.")
    return parser.parse_args(argv)


def _merge_section(defaults: dict[str, Any], overrides: dict[str, Any]) -> dict[str, Any]:
    merged = copy.deepcopy(defaults)
    for key, value in overrides.items():
        if isinstance(merged.get(key), dict) and isinstance(value, dict):
            merged[key] = _merge_section(merged[key], value)
        else:
            merged[key] = value
    return merged


def _normalize_flat_config(flat: dict[str, Any]) -> dict[str, Any]:
    """Map legacy BENCH.yaml flat keys into the nested experiment layout."""
    cfg = copy.deepcopy(DEFAULT_CONFIG)
    flat_to_nested = {
        "camera": ("experiment", "mode"),
        "speed": ("experiment", "profile"),
        "trajectory_policy": ("experiment", "trajectory_policy"),
        "workspace_root": ("dataset", "workspace_root"),
        "dataset_root": ("dataset", "dataset_root"),
        "seq_range_start": ("dataset", "seq_range_start"),
        "seq_range_end": ("dataset", "seq_range_end"),
        "params_file": ("ov2slam", "params_file"),
        "node_name": ("ov2slam", "node_name"),
        "ov2slam_cores": ("ov2slam", "cores"),
        "node_wait_timeout": ("ov2slam", "node_wait_timeout"),
        "bag_play_rate": ("playback", "bag_rate"),
        "bag_timeout": ("playback", "bag_timeout"),
        "startup_settle_sec": ("playback", "startup_settle_sec"),
        "auto_exit_timeout": ("playback", "auto_exit_timeout"),
        "sigint_timeout": ("playback", "sigint_timeout"),
        "output_root": ("evaluation", "results_dir"),
        "results_name": ("evaluation", "results_name"),
        "retries": ("evaluation", "retries"),
        "monitor_sample_sec": ("evaluation", "monitor_sample_sec"),
        "inter_run_delay_sec": ("evaluation", "inter_run_delay_sec"),
    }
    for flat_key, (sec, key) in flat_to_nested.items():
        if flat.get(flat_key) not in (None, ""):
            cfg[sec][key] = flat[flat_key]
    if flat.get("sequences") is not None:
        cfg["dataset"]["sequences"] = ensure_list(flat["sequences"])
    return cfg


def load_config(args: argparse.Namespace) -> dict[str, Any]:
    config_path = resolve_file(args.config)
    if not config_path.exists():
        if args.config != str(DEFAULT_CONFIG_PATH):
            die(f"Config file not found: {config_path}")
        return copy.deepcopy(DEFAULT_CONFIG)
    if yaml is None:
        die("PyYAML required for config. Install with: pip install pyyaml")
    raw = yaml.safe_load(config_path.read_text(encoding="utf-8")) or {}
    if not isinstance(raw, dict):
        die(f"Config root must be a mapping: {config_path}")
    if {"experiment", "dataset", "ov2slam"} & raw.keys():
        merged = copy.deepcopy(DEFAULT_CONFIG)
        for sec, defaults in DEFAULT_CONFIG.items():
            if isinstance(raw.get(sec), dict):
                merged[sec] = _merge_section(defaults, raw[sec])
            elif sec in raw:
                merged[sec] = raw[sec]
        return merged
    return _normalize_flat_config(raw)


def apply_cli_overrides(cfg: dict[str, Any], args: argparse.Namespace) -> dict[str, Any]:
    out = copy.deepcopy(cfg)
    if args.mode:
        out["experiment"]["mode"] = args.mode
    if args.profile:
        out["experiment"]["profile"] = args.profile
    if args.trajectory_policy:
        out["experiment"]["trajectory_policy"] = args.trajectory_policy
    if args.workspace_root:
        out["dataset"]["workspace_root"] = args.workspace_root
    if args.dataset_root:
        out["dataset"]["dataset_root"] = args.dataset_root
    if args.output_root:
        out["evaluation"]["results_dir"] = args.output_root
    if args.results_name:
        out["evaluation"]["results_name"] = args.results_name
    if args.params:
        out["ov2slam"]["params_file"] = args.params
    if args.cores is not None:
        out["ov2slam"]["cores"] = args.cores
    if args.runs is not None:
        out["evaluation"]["n_runs"] = args.runs
    retries = args.retries if args.retries is not None else args.engine_retries
    if retries is not None:
        out["evaluation"]["retries"] = retries
    if args.all or args.sequences:
        out["dataset"]["seq_range_start"] = ""
        out["dataset"]["seq_range_end"] = ""
        out["dataset"]["sequences"] = args.sequences if args.sequences else []
    return out


def section(cfg: dict[str, Any], name: str) -> dict[str, Any]:
    value = cfg.get(name, {})
    return value if isinstance(value, dict) else {}


def resolve_workspace(cfg: dict[str, Any], cli_override: str | None) -> Path:
    if cli_override:
        return resolve_dir(cli_override)
    raw = str(section(cfg, "dataset").get("workspace_root", "") or "")
    return resolve_dir(raw) if raw else DEFAULT_WORKSPACE_ROOT


def build_shell_context(workspace_root: Path) -> ShellContext:
    ld_candidates = []
    opencv_lib = workspace_root / "opencv" / "build" / "lib"
    if opencv_lib.is_dir():
        ld_candidates.append(opencv_lib)
    return ShellContext(
        workspace_root=workspace_root,
        setup_files=detect_ros_setup_files(workspace_root),
        env_vars={
            "RMW_IMPLEMENTATION": "rmw_cyclonedds_cpp",
            "PYTHONHOME": "",  # prevent host pyenv from bleeding into container subprocesses
        },
        ld_library_prepend=ld_candidates,
    )


# ──────────────────────────────────────────────────────────────────────────
# Preflight + params
# ──────────────────────────────────────────────────────────────────────────
def preflight(shell_ctx: ShellContext, dataset_root: Path, params_file: Path, cores: str) -> None:
    console.section("Preflight")
    console.ok("Workspace root", str(shell_ctx.workspace_root))
    console.ok("Dataset root", str(dataset_root))
    console.ok("Params file", str(params_file))

    required = ["python3", "ros2", "evo_ape", "timeout"]
    if cores:
        required.append("taskset")
    missing = [
        name for name in required
        if run_capture(shell_ctx, f"command -v {shlex.quote(name)}").returncode != 0
    ]
    if missing:
        die(f"Missing required commands in sourced environment: {', '.join(missing)}")
    if not dataset_root.is_dir():
        die(f"Dataset root not found: {dataset_root}")
    if not params_file.is_file():
        die(f"Params file not found: {params_file}")
    try:
        import matplotlib  # noqa: F401
        import numpy  # noqa: F401
    except Exception:
        die("python3 modules 'matplotlib' and 'numpy' are required for plot generation.")
    console.ok("Required commands", ", ".join(required))
    rmw = shell_ctx.env_vars.get("RMW_IMPLEMENTATION") or os.environ.get("RMW_IMPLEMENTATION") or ""
    console.ok("DDS / RMW", rmw if rmw else "rmw_fastrtps_cpp (ROS2 default)")


def _replace_or_append_ov2_param(text: str, key: str, value: str) -> str:
    pattern = re.compile(rf"^(\s*{re.escape(key)}\s*:\s*).*$", re.MULTILINE)
    if pattern.search(text):
        return pattern.sub(rf"\1{value}", text, count=1)
    suffix = "" if text.endswith("\n") else "\n"
    return f"{text}{suffix}{key}: {value}\n"


def prepare_runtime_params_file(
    base_params_file: Path,
    results_root: Path,
    mode: str,
    topic_left: str,
    topic_right: str,
) -> Path:
    """Return the params file to launch with. Only rewrites it when topic overrides are set."""
    if not topic_left and not topic_right:
        return base_params_file
    if mode == "stereo" and (not topic_left or not topic_right):
        die("OV2SLAM stereo mode requires both ov2slam.topic_left and ov2slam.topic_right.")
    if mode == "mono" and not topic_left:
        die("OV2SLAM mono mode requires ov2slam.topic_left.")

    updated = base_params_file.read_text(encoding="utf-8")
    if topic_left:
        updated = _replace_or_append_ov2_param(updated, "Camera.topic_left", topic_left)
    if topic_right:
        updated = _replace_or_append_ov2_param(updated, "Camera.topic_right", topic_right)
    runtime_params_file = results_root / "params_runtime.yaml"
    runtime_params_file.write_text(updated, encoding="utf-8")
    return runtime_params_file


def write_bag_qos_overrides(run_dir: Path, topic_left: str, topic_right: str, mode: str) -> Path | None:
    topics = [t for t in ([topic_left] if mode == "mono" else [topic_left, topic_right]) if t]
    if not topics:
        return None
    lines: list[str] = []
    for topic in topics:
        lines += [f'"{topic}":', "  reliability: reliable", "  history: keep_last", "  depth: 10"]
    path = run_dir / "bag_qos_overrides.yaml"
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return path


# ──────────────────────────────────────────────────────────────────────────
# Per-run artifacts (trajectory selection, CPU plots, run metadata)
# ──────────────────────────────────────────────────────────────────────────
def select_trajectory(run_dir: Path, trajectory_policy: str) -> tuple[Path, str, str]:
    candidates = PAPER_TRAJECTORY_CANDIDATES if trajectory_policy == "paper" else BEST_TRAJECTORY_CANDIDATES
    for name, reason in candidates:
        path = run_dir / name
        if path.is_file() and path.stat().st_size > 64:
            return path, name, reason
    unsupported = [name for name in UNSUPPORTED_EVO_TRAJECTORIES if (run_dir / name).exists()]
    if unsupported:
        raise RuntimeError(
            "Only unsupported loop-closure trajectories were found: "
            + ", ".join(unsupported)
            + " (synthetic frame indices in column 1, not timestamps)."
        )
    raise RuntimeError(f"No valid OV2SLAM trajectory was produced in {run_dir} (OV2SLAM likely crashed).")


def reset_run_dir(run_dir: Path) -> None:
    run_dir.mkdir(parents=True, exist_ok=True)
    for pattern in (
        "ov2slam.log", "bag_play.log", "gt.tum", "trajectory.tum",
        "ov2slam*.txt", "ov2slam*.csv", "*_evo.txt", "*_evo.zip", "*.png",
        "run_meta.json", "core.*",
    ):
        for path in run_dir.glob(pattern):
            if path.is_file():
                path.unlink()


def load_ov2_timings(path: Path) -> dict[str, float]:
    if not path.exists():
        return {}
    timings: dict[str, float] = {}
    with path.open(newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            key = (row.get("timer") or "").strip()
            try:
                value = float(row.get("mean_ms", ""))
            except (TypeError, ValueError):
                continue
            if key:
                timings[key] = value
    return timings


def cpu_plot_title(seq_name: str, run_dir: Path) -> str:
    info_path = run_dir / "ov2slam_pid_info.json"
    if info_path.is_file():
        try:
            pid = json.loads(info_path.read_text(encoding="utf-8")).get("ov2slam_pid")
            if pid is not None:
                return f"{seq_name} OV2SLAM PID {pid}"
        except Exception:
            pass
    return f"{seq_name} OV2SLAM"


def generate_named_thread_role_plots(run_dir: Path, seq_name: str, workspace_root: Path) -> bool:
    """Invoke plot_named_thread_roles.py to break CPU usage down by named SLAM thread."""
    if not PLOT_NAMED_THREAD_ROLES.is_file():
        return False
    thread_csv = run_dir / "ov2slam_thread_cpu.csv"
    pid_info = run_dir / "ov2slam_pid_info.json"
    if not thread_csv.is_file() or not pid_info.is_file():
        return False
    env = os.environ.copy()
    env.update({
        "THREAD_ROLE_SEQUENCE_DIR": str(run_dir),
        "THREAD_ROLE_THREAD_CSV": str(thread_csv),
        "THREAD_ROLE_PID_INFO_JSON": str(pid_info),
        "THREAD_ROLE_OUTPUT_PNG": str(run_dir / "ov2slam_named_thread_roles_aggregated.png"),
        "THREAD_ROLE_SUMMARY_CSV": str(run_dir / "ov2slam_named_thread_roles_aggregated.csv"),
        "THREAD_ROLE_FIGURE_TITLE": f"{seq_name} OV2SLAM named threads",
    })
    result = subprocess.run(
        [sys.executable, str(PLOT_NAMED_THREAD_ROLES)],
        cwd=str(workspace_root), env=env, capture_output=True, text=True, check=False, timeout=120,
    )
    return result.returncode == 0


def generate_all_cpu_artifacts(run_dir: Path, seq_name: str, workspace_root: Path) -> None:
    process_csv = run_dir / "ov2slam_process_cpu.csv"
    thread_summary_csv = run_dir / "ov2slam_thread_cpu_summary.csv"
    if process_csv.is_file() and thread_summary_csv.is_file():
        generate_cpu_plots(
            process_csv, thread_summary_csv,
            run_dir / "ov2slam_cpu_usage.png", run_dir / "ov2slam_threads_cpu_bar.png",
            cpu_plot_title(seq_name, run_dir),
        )
    generate_named_thread_role_plots(run_dir, seq_name, workspace_root)


def make_run_meta(
    *, seq_name: str, run_name: str, mode: str, profile: str, trajectory_policy: str,
    success: bool, rmse: float | None, traj_used: str, traj_reason: str,
    warnings: list[str], error: str, gt_verdict: str, run_dir: Path,
) -> dict[str, Any]:
    cpu_csv = run_dir / "ov2slam_process_cpu.csv"
    timing_csv = run_dir / "ov2slam_timings.csv"

    def artifact(name: str) -> str | None:
        return name if (run_dir / name).exists() else None

    return {
        "sequence": seq_name,
        "run_name": run_name,
        "mode": mode,
        "profile": profile,
        "trajectory_policy": trajectory_policy,
        "success": success,
        "rmse": rmse,
        "traj_used": traj_used,
        "traj_reason": traj_reason,
        "warnings": " || ".join(warnings),
        "error": error,
        "gt_verdict": gt_verdict,
        "cpu_total_avg_pct": mean_csv_column(cpu_csv, "process_cpu_percent"),
        "trajectory_poses": count_tum_poses(run_dir / "trajectory.tum"),
        "timing_avg_ms": load_ov2_timings(timing_csv),
        "artifacts": {
            "gt_tum": artifact("gt.tum"),
            "trajectory_tum": artifact("trajectory.tum"),
            "ape_traj_png": artifact("ape_traj.png"),
            "traj_xy_png": artifact("traj_xy.png"),
            "timings_csv": artifact("ov2slam_timings.csv"),
            "process_cpu_csv": artifact("ov2slam_process_cpu.csv"),
            "thread_cpu_csv": artifact("ov2slam_thread_cpu.csv"),
            "thread_cpu_summary_csv": artifact("ov2slam_thread_cpu_summary.csv"),
            "named_thread_roles_png": artifact("ov2slam_named_thread_roles_aggregated.png"),
            "named_thread_roles_csv": artifact("ov2slam_named_thread_roles_aggregated.csv"),
        },
    }


# ──────────────────────────────────────────────────────────────────────────
# The run lifecycle
# ──────────────────────────────────────────────────────────────────────────
def run_sequence_attempt(
    *, shell_ctx: ShellContext, seq_dir: Path, run_dir: Path, mode: str, profile: str,
    params_file: Path, topic_left: str, topic_right: str, trajectory_policy: str, cores: str,
    node_name: str, node_wait_timeout: float, bag_timeout: float, auto_exit_timeout: float,
    sigint_timeout: float, startup_settle_sec: float, bag_play_rate: float,
    use_bag_qos_overrides: bool, monitor_sample_sec: float, evo_t_max_diff: float,
    clock_start_timeout: float,
) -> dict[str, Any]:
    seq_name = sequence_key(seq_dir)
    warnings: list[str] = []
    node_proc: subprocess.Popen[str] | None = None
    bag_proc: subprocess.Popen[str] | None = None
    monitor: ProcessMonitor | None = None
    gt_file: Path | None = None
    gt_verdict = ""
    traj_name = ""
    traj_reason = ""
    success = False
    error = ""
    rmse: float | None = None

    try:
        reset_run_dir(run_dir)
        bag_info: BagInfo = resolve_bag_target(seq_dir)
        bag_qos = write_bag_qos_overrides(run_dir, topic_left, topic_right, mode) if use_bag_qos_overrides else None

        # 1) Ground truth (optional — a missing GT only skips evo, never fails the run).
        try:
            gt_file, gt_source, gt_verdict = prepare_ground_truth(seq_dir, run_dir)
            console.step("Ground truth", gt_source)
        except (FileNotFoundError, ValueError) as exc:
            gt_verdict = f"GT unavailable: {exc}"
            warnings.append(gt_verdict)
            console.warn("Ground truth", gt_verdict)

        # 2) Launch OV2SLAM and start CPU monitoring.
        #
        # Two deliberate choices keep OV2SLAM from shutting down early:
        #
        #  (a) No use_sim_time:=true. OV2SLAM's automatic end-of-sequence detection
        #      (ov2slam.cpp) exits when no image arrives for 100 * cam_delay, where
        #      cam_delay is the node-clock gap between the last two processed frames.
        #      Wall-clock time keeps cam_delay ~= the real frame interval; the sim
        #      clock is discrete and makes it collapse. Trajectory timestamps come
        #      from image headers, so this does not affect accuracy.
        #
        #  (b) Plain file redirect for the node log (launch_background), NOT a PTY
        #      mirror. OV2SLAM prints a multi-line timing block every frame (~400
        #      lines/s). Pumping that through Python line-by-line and onto the
        #      terminal burns a core; on this 4-core SBC it starves OV2SLAM's image
        #      ingestion thread, its internal image queue drains, and the heuristic
        #      above then trips a premature shutdown mid-sequence (partial trajectory,
        #      bad RMSE). The full node log is still written to ov2slam.log.
        ov_cmd = ["ros2", "run", "ov2slam", "ov2slam_node", str(params_file)]
        if cores:
            ov_cmd = ["taskset", "-c", cores] + ov_cmd
            console.step("Core pinning", cores)
        node_proc = launch_background(
            shell_ctx, " ".join(shlex.quote(t) for t in ov_cmd),
            run_dir, run_dir / "ov2slam.log",
        )
        monitor = ProcessMonitor(
            node_proc.pid, run_dir, "ov2slam",
            exact_process_names={"ov2slam_node"}, wrapped_cmd_tokens={"ov2slam_node"},
            sample_sec=monitor_sample_sec,
        )
        monitor.start()
        console.step("OV2SLAM", f"pid={node_proc.pid}")

        if not wait_for_ros2_node(shell_ctx, node_name, node_wait_timeout):
            raise RuntimeError(f"OV2SLAM node '{node_name}' did not appear within {node_wait_timeout:.0f}s.")
        settle_deadline = time.monotonic() + startup_settle_sec
        while time.monotonic() < settle_deadline:
            if node_proc.poll() is not None:
                raise RuntimeError("OV2SLAM exited during startup. Check ov2slam.log.")
            time.sleep(0.25)

        # 3) Play the bag and confirm it is publishing /clock.
        bag_cmd = ["ros2", "bag", "play", str(bag_info.target), "--clock", "--rate", str(bag_play_rate)]
        if bag_qos is not None:
            bag_cmd += ["--qos-profile-overrides-path", str(bag_qos)]
        bag_proc = launch_background(
            shell_ctx, " ".join(shlex.quote(t) for t in bag_cmd), run_dir, run_dir / "bag_play.log",
        )
        console.step("Bag playback", bag_info.description)
        time.sleep(1.0)
        if bag_proc.poll() is not None:
            raise RuntimeError("ros2 bag play exited before playback was confirmed.")
        if not wait_for_clock_message(shell_ctx, clock_start_timeout):
            raise RuntimeError(f"ros2 bag play did not publish /clock within {clock_start_timeout:.0f}s.")

        # 4) Wait for the bag to finish. OV2SLAM may self-exit at end-of-stream BEFORE the
        #    bag player stops (that is normal, not a crash): in that case stop feeding the
        #    bag and move straight to evaluation.
        deadline = time.monotonic() + bag_timeout
        while bag_proc.poll() is None:
            if node_proc.poll() is not None:
                warnings.append("OV2SLAM exited during playback (end-of-stream self-shutdown); stopping bag.")
                terminate_process_group(bag_proc.pid, 5.0)
                break
            if time.monotonic() > deadline:
                terminate_process_group(bag_proc.pid, 5.0)
                raise RuntimeError(f"Bag playback timed out after {bag_timeout:.0f}s.")
            time.sleep(0.5)
        console.step("Playback done", "all frames delivered")

        # 5) Let OV2SLAM flush its trajectory and exit. Once the bag stops it no
        #    longer receives images and shuts itself down (writing all trajectory
        #    files). If it is still alive after auto_exit_timeout, SIGINT triggers
        #    the same flush. We do NOT publish a final /clock here: without
        #    use_sim_time OV2SLAM ignores /clock, and `ros2 topic pub --once` can
        #    block indefinitely, wedging the whole run.
        if node_proc.poll() is None:
            if not wait_for_pid_exit(node_proc.pid, auto_exit_timeout):
                warnings.append("OV2SLAM did not self-exit after playback; sending SIGINT to flush trajectory.")
                terminate_process_group(node_proc.pid, sigint_timeout)
        console.step("OV2SLAM stopped", "trajectory flushed")

        # 6) A produced trajectory == success. Evaluate it with evo when GT is available.
        trajectory_path, traj_name, traj_reason = select_trajectory(run_dir, trajectory_policy)
        shutil.copy2(trajectory_path, run_dir / "trajectory.tum")
        console.step("Trajectory", traj_name)
        if gt_file is not None:
            evo = run_evo_and_plots(
                shell_ctx=shell_ctx, gt_file=gt_file, est_file=run_dir / "trajectory.tum",
                out_dir=run_dir, seq_name=seq_name, t_max_diff=evo_t_max_diff,
                correct_scale=(mode == "mono"), est_label="OV2SLAM",
            )
            rmse = evo.rmse
            console.ok("APE RMSE", f"{rmse:.4f} m")
        else:
            warnings.append("Skipped evo evaluation because no ground truth was available.")
        success = True

    except Exception as exc:
        error = str(exc)
        console.fail("Attempt failed", error)
    finally:
        for proc in (bag_proc, node_proc):
            if proc is not None and proc.poll() is None:
                terminate_process_group(proc.pid, sigint_timeout)
        if monitor is not None:
            monitor.stop()
            try:
                generate_all_cpu_artifacts(run_dir, seq_name, shell_ctx.workspace_root)
            except Exception:
                pass

    return make_run_meta(
        seq_name=seq_name, run_name=run_dir.name, mode=mode, profile=profile,
        trajectory_policy=trajectory_policy, success=success, rmse=rmse,
        traj_used=traj_name, traj_reason=traj_reason, warnings=warnings,
        error=error, gt_verdict=gt_verdict, run_dir=run_dir,
    )


def run_sequence_with_retries(*, retries: int, inter_run_delay_sec: float, **attempt_kwargs: Any) -> dict[str, Any]:
    seq_dir: Path = attempt_kwargs["seq_dir"]
    run_dir: Path = attempt_kwargs["run_dir"]
    total_attempts = retries + 1
    last_meta: dict[str, Any] | None = None
    for attempt in range(1, total_attempts + 1):
        if attempt > 1:
            console.warn("Retry", f"{sequence_key(seq_dir)}/{run_dir.name} attempt {attempt}/{total_attempts}")
        last_meta = run_sequence_attempt(**attempt_kwargs)
        if last_meta["success"]:
            return last_meta
        if attempt < total_attempts and inter_run_delay_sec > 0:
            time.sleep(inter_run_delay_sec)
    assert last_meta is not None
    return last_meta


# ──────────────────────────────────────────────────────────────────────────
# main
# ──────────────────────────────────────────────────────────────────────────
def main(argv: list[str]) -> int:
    args = parse_args(argv)
    cfg = apply_cli_overrides(load_config(args), args)

    exp_cfg = section(cfg, "experiment")
    dataset_cfg = section(cfg, "dataset")
    ov2_cfg = section(cfg, "ov2slam")
    playback_cfg = section(cfg, "playback")
    eval_cfg = section(cfg, "evaluation")

    workspace_root = resolve_workspace(cfg, args.workspace_root)
    dataset_root_raw = str(dataset_cfg.get("dataset_root", "") or "")
    dataset_root = resolve_dir(dataset_root_raw) if dataset_root_raw else DEFAULT_DATASET_ROOT

    mode = str(exp_cfg.get("mode", DEFAULT_MODE)).strip().lower()
    if mode not in {"mono", "stereo"}:
        die(f"experiment.mode must be mono or stereo, got {mode!r}")
    profile = str(exp_cfg.get("profile", DEFAULT_PROFILE)).strip().lower()
    if profile not in {"accurate", "average", "fast"}:
        die(f"experiment.profile must be accurate, average, or fast, got {profile!r}")
    trajectory_policy = str(exp_cfg.get("trajectory_policy", DEFAULT_TRAJECTORY_POLICY))

    params_value = str(ov2_cfg.get("params_file", "") or "")
    params_file = (
        resolve_file(params_value) if params_value
        else workspace_root / "src" / "ov2slam_ros" / "parameters_files" / profile / "euroc" / f"euroc_{mode}.yaml"
    )

    runs = int(eval_cfg.get("n_runs", DEFAULT_RUNS))
    retries = int(eval_cfg.get("retries", DEFAULT_RETRIES))
    cores = str(ov2_cfg.get("cores", "") or "")
    node_name = str(ov2_cfg.get("node_name", "/ov2slam_node"))
    topic_left = str(ov2_cfg.get("topic_left", "") or "").strip()
    topic_right = str(ov2_cfg.get("topic_right", "") or "").strip()
    node_wait_timeout = float(ov2_cfg.get("node_wait_timeout", 45))

    results_dir_raw = str(eval_cfg.get("results_dir", "") or "")
    output_root = resolve_dir(results_dir_raw) if results_dir_raw else DEFAULT_OUTPUT_ROOT
    results_name = str(eval_cfg.get("results_name", "") or "") or datetime_compact()
    log_dir_raw = str(eval_cfg.get("log_dir", "") or "")
    log_dir = resolve_dir(log_dir_raw) if log_dir_raw else workspace_root / "run_logs"

    bag_play_rate = float(playback_cfg.get("bag_rate", 1.0))
    bag_timeout = float(playback_cfg.get("bag_timeout", 1200))
    use_bag_qos_overrides = coerce_bool(playback_cfg.get("use_bag_qos_overrides", False), False)
    auto_exit_timeout = float(playback_cfg.get("auto_exit_timeout", 600))
    sigint_timeout = float(playback_cfg.get("sigint_timeout", 30))
    startup_settle_sec = float(playback_cfg.get("startup_settle_sec", 2))
    clock_start_timeout = float(playback_cfg.get("clock_start_timeout", 10))

    inter_run_delay_sec = float(eval_cfg.get("inter_run_delay_sec", 10))
    monitor_sample_sec = float(eval_cfg.get("monitor_sample_sec", 0.05))
    evo_t_max_diff = float(eval_cfg.get("t_max_diff", 0.1))

    configured_sequences = [str(x) for x in ensure_list(dataset_cfg.get("sequences", [])) if str(x)]
    seq_range_start = str(dataset_cfg.get("seq_range_start", "") or "")
    seq_range_end = str(dataset_cfg.get("seq_range_end", "") or "")

    output_root.mkdir(parents=True, exist_ok=True)
    results_root = output_root / results_name
    results_root.mkdir(parents=True, exist_ok=True)
    log_file = log_dir / f"terminal_{datetime_compact()}_ov2_benchmarking.log"
    console.set_log_file(log_file)
    console.banner("OV2SLAM Benchmark")

    shell_ctx = build_shell_context(workspace_root)
    runtime_params_file = prepare_runtime_params_file(params_file, results_root, mode, topic_left, topic_right)
    preflight(shell_ctx, dataset_root, runtime_params_file, cores)

    discovered = discover_sequences(dataset_root)
    if not discovered:
        die(f"No sequences found under {dataset_root}")
    selected = select_sequences(discovered, configured_sequences, seq_range_start, seq_range_end)
    if not selected:
        die("Sequence selection produced an empty set.")

    source_config = resolve_file(args.config)
    if source_config.exists():
        shutil.copy2(source_config, results_root / "benchmark_config_used.yaml")
    (results_root / "params_used.yaml").write_text(runtime_params_file.read_text(encoding="utf-8"), encoding="utf-8")
    run_config = {
        "config_file": str(source_config if source_config.exists() else DEFAULT_CONFIG_PATH),
        "workspace_root": str(workspace_root),
        "dataset_root": str(dataset_root),
        "results_root": str(results_root),
        "results_timezone": RESULTS_TIMEZONE,
        "params_file": str(runtime_params_file),
        "mode": mode,
        "profile": profile,
        "topic_left": topic_left,
        "topic_right": topic_right,
        "trajectory_policy": trajectory_policy,
        "cores": cores,
        "n_runs": runs,
        "retries": retries,
        "selected_sequences": [sequence_key(p) for p in selected],
        "bag_play_rate": bag_play_rate,
        "use_bag_qos_overrides": use_bag_qos_overrides,
        "evo_t_max_diff": evo_t_max_diff,
        "monitor_sample_sec": monitor_sample_sec,
    }
    (results_root / "run_config.json").write_text(json.dumps(run_config, indent=2) + "\n", encoding="utf-8")

    console.section("Configuration")
    console.ok("Config file", str(source_config if source_config.exists() else DEFAULT_CONFIG_PATH))
    console.ok("Results root", str(results_root))
    console.ok("Mode", mode)
    console.ok("Profile", profile)
    if topic_left:
        console.ok("Topic left", topic_left)
    if topic_right:
        console.ok("Topic right", topic_right)
    console.ok("Runs per sequence", str(runs))
    console.ok("Engine retries", str(retries))
    console.ok("Trajectory policy", trajectory_policy)
    console.ok("Bag QoS overrides", "enabled" if use_bag_qos_overrides else "disabled")
    _rmw = shell_ctx.env_vars.get("RMW_IMPLEMENTATION") or os.environ.get("RMW_IMPLEMENTATION") or ""
    console.ok("DDS / RMW", _rmw if _rmw else "rmw_fastrtps_cpp (ROS2 default)")
    console.ok("Sequences", ", ".join(sequence_key(p) for p in selected))

    overall_success = True
    total_sequences = len(selected)
    for seq_idx, seq_dir in enumerate(selected, start=1):
        seq_name = sequence_key(seq_dir)
        console.seq_banner(seq_name, seq_idx, total_sequences)
        for run_idx in range(1, runs + 1):
            console.run_banner(run_idx, runs)
            run_name = f"run_{run_idx:02d}"
            run_dir = results_root / seq_name / run_name
            meta = run_sequence_with_retries(
                retries=retries,
                inter_run_delay_sec=inter_run_delay_sec,
                shell_ctx=shell_ctx,
                seq_dir=seq_dir,
                run_dir=run_dir,
                mode=mode,
                profile=profile,
                params_file=runtime_params_file,
                topic_left=topic_left,
                topic_right=topic_right,
                trajectory_policy=trajectory_policy,
                cores=cores,
                node_name=node_name,
                node_wait_timeout=node_wait_timeout,
                bag_timeout=bag_timeout,
                auto_exit_timeout=auto_exit_timeout,
                sigint_timeout=sigint_timeout,
                startup_settle_sec=startup_settle_sec,
                bag_play_rate=bag_play_rate,
                use_bag_qos_overrides=use_bag_qos_overrides,
                monitor_sample_sec=monitor_sample_sec,
                evo_t_max_diff=evo_t_max_diff,
                clock_start_timeout=clock_start_timeout,
            )
            (run_dir / "run_meta.json").write_text(json.dumps(meta, indent=2) + "\n", encoding="utf-8")
            if meta["success"]:
                rmse_txt = f"{meta['rmse']:.4f} m" if isinstance(meta.get("rmse"), (int, float)) else "n/a"
                console.ok("Run complete", f"{seq_name}/{run_name} rmse={rmse_txt}")
            else:
                console.fail("Run failed", f"{seq_name}/{run_name} {meta['error']}")
                overall_success = False
            # Keep going: one failed run must not abort the whole benchmark.
            if inter_run_delay_sec > 0 and run_idx < runs:
                time.sleep(inter_run_delay_sec)

    generate_aggregate_outputs(results_root)
    console.section("Outputs")
    console.ok("Experiment summary", str(results_root / "experiment_summary.csv"))
    console.ok("Polling report", str(results_root / "polling_results.md"))
    console.ok("Terminal log", str(log_file))
    return 0 if overall_success else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
