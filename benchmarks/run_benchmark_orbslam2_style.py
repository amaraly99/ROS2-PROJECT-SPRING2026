#!/usr/bin/env python3

"""Standalone ORB_SLAM2-style benchmark runner for OV2SLAM.

This script no longer delegates execution to BENCH.sh. It is the benchmark
engine: it loads an experiment config, launches OV2SLAM directly, plays EuRoC
bags, evaluates with evo, and writes run/experiment summaries itself.
"""

from __future__ import annotations

import argparse
import json
import os
import shlex
import shutil
import sys
import time
from pathlib import Path
from typing import Any

from benchmark_common import (
    EUROC_SEQUENCES,
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
    load_top_level_yaml,
    mean_csv_column,
    prepare_ground_truth,
    publish_final_clock,
    resolve_bag_target,
    resolve_dir,
    resolve_file,
    run_capture,
    run_evo_and_plots,
    select_sequences,
    terminate_process_group,
    wait_for_clock_message,
    wait_for_pid_exit,
    wait_for_ros2_node,
    launch_background,
    launch_background_mirrored,
)


SCRIPT_PATH = Path(__file__).resolve()
SCRIPT_DIR = SCRIPT_PATH.parent
LOCAL_WORKSPACE_ROOT = SCRIPT_DIR.parent
DEFAULT_WORKSPACE_ROOT = Path("/workspace") if Path("/workspace").is_dir() else LOCAL_WORKSPACE_ROOT
DEFAULT_CONFIG_PATH = SCRIPT_DIR / "OV2SLAM_BENCHMARK.yaml"
DEFAULT_DATASET_ROOT = DEFAULT_WORKSPACE_ROOT / "datasets" / "euroc"
DEFAULT_OUTPUT_ROOT = DEFAULT_WORKSPACE_ROOT / "results" / "ov2slam_orbslam2_style"
DEFAULT_MODE = "stereo"
DEFAULT_PROFILE = "accurate"
DEFAULT_TRAJECTORY_POLICY = "paper"
DEFAULT_RUNS = 1
DEFAULT_RETRIES = 0
FINAL_CLOCK_SEC = 1450000000
FINAL_CLOCK_NANOSEC = 0

BEST_TRAJECTORY_CANDIDATES = [
    (
        "ov2slam_fullba_kfs_traj.txt",
        "final optimized keyframe trajectory after full BA; best final estimate when do_full_ba=1",
    ),
    (
        "ov2slam_kfs_traj.txt",
        "optimized keyframe trajectory after local BA / loop closure; primary fallback",
    ),
    ("ov2slam_traj.txt", "raw frame trajectory; fallback only when optimized keyframe outputs are absent"),
]

PAPER_TRAJECTORY_CANDIDATES = [
    (
        "ov2slam_kfs_traj.txt",
        "paper-style choice: keyframe trajectory after online local BA / loop closure, without end-of-sequence full BA",
    ),
    ("ov2slam_traj.txt", "paper-style fallback: raw frame trajectory when keyframe output is missing"),
    (
        "ov2slam_fullba_kfs_traj.txt",
        "last-resort fallback: full BA result exists, but this is an offline refinement beyond the paper's fully-online setting",
    ),
]

UNSUPPORTED_EVO_TRAJECTORIES = [
    "ov2slam_full_traj_wlc.txt",
    "ov2slam_full_traj_wlc_opt.txt",
]


console = Console()


def die(message: str) -> None:
    console.fail(message)
    raise SystemExit(1)


def load_ov2_timings(path: Path) -> dict[str, float]:
    if not path.exists():
        return {}
    import csv

    timings: dict[str, float] = {}
    with path.open(newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            key = (row.get("timer") or "").strip()
            try:
                value = float(row.get("mean_ms", ""))
            except Exception:
                continue
            if key:
                timings[key] = value
    return timings


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        prog=SCRIPT_PATH.name,
        description="Standalone ORB_SLAM2-style benchmark runner for OV2SLAM.",
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


def config_value(config: dict[str, Any], *keys: str, default: Any = None) -> Any:
    for key in keys:
        if key in config and config[key] not in (None, ""):
            return config[key]
    return default


def load_config(args: argparse.Namespace) -> dict[str, Any]:
    config_path = resolve_file(args.config)
    if config_path.exists():
        return load_top_level_yaml(config_path)
    if args.config != str(DEFAULT_CONFIG_PATH):
        die(f"Config file not found: {config_path}")
    return {}


def build_shell_context(workspace_root: Path) -> ShellContext:
    ld_candidates = []
    opencv_lib = workspace_root / "opencv" / "build" / "lib"
    if opencv_lib.is_dir():
        ld_candidates.append(opencv_lib)
    return ShellContext(
        workspace_root=workspace_root,
        setup_files=detect_ros_setup_files(workspace_root),
        env_vars={"RMW_IMPLEMENTATION": "rmw_cyclonedds_cpp"},
        ld_library_prepend=ld_candidates,
    )


def preflight(shell_ctx: ShellContext, dataset_root: Path, params_file: Path, cores: str) -> None:
    console.section("Preflight")
    console.ok("Workspace root", str(shell_ctx.workspace_root))
    console.ok("Dataset root", str(dataset_root))
    console.ok("Params file", str(params_file))

    required = ["python3", "ros2", "evo_ape", "timeout"]
    if cores:
        required.append("taskset")
    missing: list[str] = []
    for command_name in required:
        probe = run_capture(shell_ctx, f"command -v {shlex.quote(command_name)}")
        if probe.returncode != 0:
            missing.append(command_name)
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


def select_trajectory(run_dir: Path, trajectory_policy: str) -> tuple[Path, str, str]:
    candidates = PAPER_TRAJECTORY_CANDIDATES if trajectory_policy == "paper" else BEST_TRAJECTORY_CANDIDATES
    for name, reason in candidates:
        path = run_dir / name
        if path.is_file() and path.stat().st_size > 64:
            return path, name, reason
    found_unsupported = [name for name in UNSUPPORTED_EVO_TRAJECTORIES if (run_dir / name).exists()]
    if found_unsupported:
        raise RuntimeError(
            "Only unsupported loop-closure full trajectories were found: "
            + ", ".join(found_unsupported)
            + ". These files use synthetic frame indices in column 1, not timestamps."
        )
    raise RuntimeError(f"No supported OV2SLAM trajectory found in {run_dir}")


def reset_run_dir(run_dir: Path) -> None:
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


def make_run_meta(
    seq_name: str,
    run_name: str,
    mode: str,
    profile: str,
    trajectory_policy: str,
    success: bool,
    rmse: float | None,
    traj_used: str,
    traj_reason: str,
    warnings: list[str],
    error: str,
    gt_verdict: str,
    run_dir: Path,
) -> dict[str, Any]:
    cpu_csv = run_dir / "ov2slam_process_cpu.csv"
    timing_csv = run_dir / "ov2slam_timings.csv"
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
            "gt_tum": "gt.tum" if (run_dir / "gt.tum").exists() else None,
            "trajectory_tum": "trajectory.tum" if (run_dir / "trajectory.tum").exists() else None,
            "ape_traj_png": "ape_traj.png" if (run_dir / "ape_traj.png").exists() else None,
            "traj_xy_png": "traj_xy.png" if (run_dir / "traj_xy.png").exists() else None,
            "timings_csv": "ov2slam_timings.csv" if timing_csv.exists() else None,
            "process_cpu_csv": "ov2slam_process_cpu.csv" if cpu_csv.exists() else None,
            "thread_cpu_csv": "ov2slam_thread_cpu.csv" if (run_dir / "ov2slam_thread_cpu.csv").exists() else None,
            "thread_cpu_summary_csv": (
                "ov2slam_thread_cpu_summary.csv" if (run_dir / "ov2slam_thread_cpu_summary.csv").exists() else None
            ),
        },
    }


def run_sequence_attempt(
    shell_ctx: ShellContext,
    seq_dir: Path,
    run_dir: Path,
    mode: str,
    profile: str,
    params_file: Path,
    trajectory_policy: str,
    cores: str,
    node_name: str,
    node_wait_timeout: float,
    bag_timeout: float,
    auto_exit_timeout: float,
    sigint_timeout: float,
    startup_settle_sec: float,
    bag_play_rate: float,
    monitor_sample_sec: float,
    evo_t_max_diff: float,
) -> dict[str, Any]:
    seq_name = seq_dir.name
    warnings: list[str] = []
    node_proc = None
    bag_proc = None
    monitor = None
    gt_verdict = ""
    traj_name = ""
    traj_reason = ""
    try:
        reset_run_dir(run_dir)
        bag_info: BagInfo = resolve_bag_target(seq_dir)
        gt_file, gt_source, gt_verdict = prepare_ground_truth(seq_dir, run_dir)
        console.step("Ground truth", gt_source)

        ov_cmd = [
            "ros2",
            "run",
            "ov2slam",
            "ov2slam_node",
            str(params_file),
            "--ros-args",
            "-p",
            "use_sim_time:=true",
        ]
        if cores:
            ov_cmd = ["taskset", "-c", cores] + ov_cmd
            console.step("Core pinning", cores)
        node_proc = launch_background_mirrored(
            shell_ctx,
            " ".join(shlex.quote(token) for token in ov_cmd),
            run_dir,
            run_dir / "ov2slam.log",
        )
        monitor = ProcessMonitor(
            node_proc.pid,
            run_dir,
            "ov2slam",
            exact_process_names={"ov2slam_node"},
            wrapped_cmd_tokens={"ov2slam_node"},
            sample_sec=monitor_sample_sec,
        )
        monitor.start()
        console.step("OV2SLAM", f"pid={node_proc.pid}")
        if not wait_for_ros2_node(shell_ctx, node_name, node_wait_timeout):
            raise RuntimeError(f"OV2SLAM node '{node_name}' did not appear within {node_wait_timeout}s.")
        time.sleep(startup_settle_sec)
        if node_proc.poll() is not None:
            raise RuntimeError("OV2SLAM exited during startup settle.")

        bag_cmd = [
            "ros2",
            "bag",
            "play",
            str(bag_info.target),
            "--clock",
            "--rate",
            str(bag_play_rate),
        ]
        bag_proc = launch_background(
            shell_ctx,
            " ".join(shlex.quote(token) for token in bag_cmd),
            run_dir,
            run_dir / "bag_play.log",
        )
        console.step("Bag playback", bag_info.description)
        time.sleep(1.0)
        if bag_proc.poll() is not None:
            raise RuntimeError("ros2 bag play exited before playback was confirmed.")
        if not wait_for_clock_message(shell_ctx, 10.0):
            terminate_process_group(bag_proc.pid, 2.0)
            raise RuntimeError("ros2 bag play did not publish /clock within 10s.")

        deadline = time.monotonic() + bag_timeout
        bag_rc: int | None = None
        while time.monotonic() < deadline:
            bag_rc = bag_proc.poll()
            if bag_rc is not None:
                break
            if node_proc.poll() is not None:
                terminate_process_group(bag_proc.pid, 2.0)
                bag_proc = None
                raise RuntimeError("OV2SLAM exited while bag playback was still running. Check ov2slam.log.")
            time.sleep(0.5)

        if bag_proc is not None and bag_rc is None:
            warnings.append(f"Bag playback timed out after {bag_timeout}s and was killed.")
            terminate_process_group(bag_proc.pid, 2.0)
            bag_proc = None
            publish_final_clock(shell_ctx, FINAL_CLOCK_SEC, FINAL_CLOCK_NANOSEC)
            if node_proc is not None and node_proc.poll() is None:
                terminate_process_group(node_proc.pid, sigint_timeout)
            raise RuntimeError(f"Bag playback timed out after {bag_timeout}s.")
        bag_proc = None

        if bag_rc != 0:
            warnings.append(f"ros2 bag play exited with rc={bag_rc}; attempting final flush anyway.")
        publish_final_clock(shell_ctx, FINAL_CLOCK_SEC, FINAL_CLOCK_NANOSEC)
        console.step("Final clock", f"{FINAL_CLOCK_SEC}s")

        if node_proc.poll() is None:
            if wait_for_pid_exit(node_proc.pid, auto_exit_timeout):
                node_proc.wait(timeout=1.0)
            else:
                warnings.append("OV2SLAM did not auto-exit after final /clock; SIGINT fallback used.")
                terminate_process_group(node_proc.pid, sigint_timeout)
        if node_proc.poll() is None:
            raise RuntimeError("OV2SLAM did not exit cleanly after final flush.")

        trajectory_path, traj_name, traj_reason = select_trajectory(run_dir, trajectory_policy)
        shutil.copy2(trajectory_path, run_dir / "trajectory.tum")
        evo = run_evo_and_plots(
            shell_ctx=shell_ctx,
            gt_file=gt_file,
            est_file=run_dir / "trajectory.tum",
            out_dir=run_dir,
            seq_name=seq_name,
            t_max_diff=evo_t_max_diff,
            correct_scale=(mode == "mono"),
            est_label="OV2SLAM",
        )
        meta = make_run_meta(
            seq_name=seq_name,
            run_name=run_dir.name,
            mode=mode,
            profile=profile,
            trajectory_policy=trajectory_policy,
            success=True,
            rmse=evo.rmse,
            traj_used=traj_name,
            traj_reason=traj_reason,
            warnings=warnings,
            error="",
            gt_verdict=gt_verdict,
            run_dir=run_dir,
        )
        return meta
    except Exception as exc:
        return make_run_meta(
            seq_name=seq_name,
            run_name=run_dir.name,
            mode=mode,
            profile=profile,
            trajectory_policy=trajectory_policy,
            success=False,
            rmse=None,
            traj_used=traj_name,
            traj_reason=traj_reason,
            warnings=warnings,
            error=str(exc),
            gt_verdict=gt_verdict,
            run_dir=run_dir,
        )
    finally:
        if bag_proc is not None and bag_proc.poll() is None:
            terminate_process_group(bag_proc.pid, 2.0)
        if node_proc is not None and node_proc.poll() is None:
            terminate_process_group(node_proc.pid, sigint_timeout)
        if monitor is not None:
            monitor.stop()
            generate_cpu_plots(
                run_dir / "ov2slam_process_cpu.csv",
                run_dir / "ov2slam_thread_cpu_summary.csv",
                run_dir / "ov2slam_cpu_usage.png",
                run_dir / "ov2slam_threads_cpu_bar.png",
                f"{seq_name} OV2SLAM",
            )


def run_sequence_with_retries(
    shell_ctx: ShellContext,
    seq_dir: Path,
    run_dir: Path,
    mode: str,
    profile: str,
    params_file: Path,
    trajectory_policy: str,
    cores: str,
    retries: int,
    node_name: str,
    node_wait_timeout: float,
    bag_timeout: float,
    auto_exit_timeout: float,
    sigint_timeout: float,
    startup_settle_sec: float,
    bag_play_rate: float,
    monitor_sample_sec: float,
    evo_t_max_diff: float,
    inter_run_delay_sec: float,
) -> dict[str, Any]:
    total_attempts = retries + 1
    last_meta: dict[str, Any] | None = None
    for attempt in range(1, total_attempts + 1):
        if attempt > 1:
            console.warn("Retry", f"{seq_dir.name} attempt {attempt}/{total_attempts}")
        last_meta = run_sequence_attempt(
            shell_ctx=shell_ctx,
            seq_dir=seq_dir,
            run_dir=run_dir,
            mode=mode,
            profile=profile,
            params_file=params_file,
            trajectory_policy=trajectory_policy,
            cores=cores,
            node_name=node_name,
            node_wait_timeout=node_wait_timeout,
            bag_timeout=bag_timeout,
            auto_exit_timeout=auto_exit_timeout,
            sigint_timeout=sigint_timeout,
            startup_settle_sec=startup_settle_sec,
            bag_play_rate=bag_play_rate,
            monitor_sample_sec=monitor_sample_sec,
            evo_t_max_diff=evo_t_max_diff,
        )
        if last_meta["success"]:
            return last_meta
        if attempt < total_attempts and inter_run_delay_sec > 0:
            time.sleep(inter_run_delay_sec)
    assert last_meta is not None
    return last_meta


def main(argv: list[str]) -> int:
    args = parse_args(argv)
    config = load_config(args)

    workspace_root = resolve_dir(config_value(config, "workspace_root", default=args.workspace_root or DEFAULT_WORKSPACE_ROOT))
    dataset_root = resolve_dir(config_value(config, "dataset_root", default=args.dataset_root or DEFAULT_DATASET_ROOT))
    output_root = resolve_dir(config_value(config, "output_root", default=args.output_root or DEFAULT_OUTPUT_ROOT))
    mode = config_value(config, "mode", "camera", default=args.mode or DEFAULT_MODE)
    profile = config_value(config, "profile", "speed", default=args.profile or DEFAULT_PROFILE)
    params_value = args.params if args.params else config_value(config, "params", "params_file", default="")
    params_file = resolve_file(params_value) if params_value else (
        workspace_root / "src" / "ov2slam_ros" / "parameters_files" / profile / "euroc" / f"euroc_{mode}.yaml"
    )
    trajectory_policy = args.trajectory_policy or config_value(config, "trajectory_policy", default=DEFAULT_TRAJECTORY_POLICY)
    runs = args.runs if args.runs is not None else int(config_value(config, "runs", default=DEFAULT_RUNS))
    retries = (
        args.retries
        if args.retries is not None
        else args.engine_retries
        if args.engine_retries is not None
        else int(config_value(config, "retries", "engine_retries", default=DEFAULT_RETRIES))
    )
    cores = args.cores if args.cores is not None else str(config_value(config, "cores", "ov2slam_cores", default=""))
    results_name = args.results_name or str(config_value(config, "results_name", default="")) or datetime_compact()
    node_name = str(config_value(config, "node_name", default="/ov2slam_node"))
    node_wait_timeout = float(config_value(config, "node_wait_timeout", default=45))
    bag_timeout = float(config_value(config, "bag_timeout", default=1200))
    auto_exit_timeout = float(config_value(config, "auto_exit_timeout", default=600))
    sigint_timeout = float(config_value(config, "sigint_timeout", default=30))
    startup_settle_sec = float(config_value(config, "startup_settle_sec", default=2))
    inter_run_delay_sec = float(config_value(config, "inter_run_delay_sec", default=10))
    monitor_sample_sec = float(config_value(config, "monitor_sample_sec", default=0.05))
    bag_play_rate = float(config_value(config, "bag_play_rate", default=1.0))
    evo_t_max_diff = float(config_value(config, "evo_t_max_diff", default=0.1))

    configured_sequences = [str(item) for item in ensure_list(config_value(config, "sequences", default=[])) if str(item)]
    seq_range_start = str(config_value(config, "seq_range_start", default=""))
    seq_range_end = str(config_value(config, "seq_range_end", default=""))
    if args.all:
        configured_sequences = []
        seq_range_start = ""
        seq_range_end = ""
    if args.sequences:
        configured_sequences = args.sequences
        seq_range_start = ""
        seq_range_end = ""

    output_root.mkdir(parents=True, exist_ok=True)
    results_root = output_root / results_name
    results_root.mkdir(parents=True, exist_ok=True)
    log_file = workspace_root / "run_logs" / f"terminal_{datetime_compact()}_ov2_standalone.log"
    console.set_log_file(log_file)
    console.banner("OV2SLAM Standalone Benchmark")

    shell_ctx = build_shell_context(workspace_root)
    preflight(shell_ctx, dataset_root, params_file, cores)

    discovered = discover_sequences(dataset_root)
    if not discovered:
        die(f"No sequences found under {dataset_root}")
    selected = select_sequences(discovered, configured_sequences, seq_range_start, seq_range_end)
    if not selected:
        die("Sequence selection produced an empty set.")

    config_used_path = results_root / "benchmark_config_used.yaml"
    source_config = resolve_file(args.config)
    if source_config.exists():
        shutil.copy2(source_config, config_used_path)
    (results_root / "params_used.yaml").write_text(params_file.read_text(encoding="utf-8"), encoding="utf-8")
    run_config = {
        "config_file": str(source_config if source_config.exists() else DEFAULT_CONFIG_PATH),
        "workspace_root": str(workspace_root),
        "dataset_root": str(dataset_root),
        "results_root": str(results_root),
        "results_timezone": RESULTS_TIMEZONE,
        "params_file": str(params_file),
        "mode": mode,
        "profile": profile,
        "trajectory_policy": trajectory_policy,
        "cores": cores,
        "runs": runs,
        "retries": retries,
        "selected_sequences": [path.name for path in selected],
        "bag_play_rate": bag_play_rate,
        "evo_t_max_diff": evo_t_max_diff,
    }
    (results_root / "run_config.json").write_text(json.dumps(run_config, indent=2) + "\n", encoding="utf-8")

    console.section("Configuration")
    console.ok("Config file", str(source_config if source_config.exists() else DEFAULT_CONFIG_PATH))
    console.ok("Results root", str(results_root))
    console.ok("Mode", mode)
    console.ok("Profile", profile)
    console.ok("Runs per sequence", str(runs))
    console.ok("Trajectory policy", trajectory_policy)
    console.ok("Sequences", ", ".join(path.name for path in selected))

    all_run_metas: list[dict[str, Any]] = []
    overall_success = True
    total_sequences = len(selected)
    for seq_idx, seq_dir in enumerate(selected, start=1):
        console.seq_banner(seq_dir.name, seq_idx, total_sequences)
        for run_idx in range(1, runs + 1):
            console.run_banner(run_idx, runs)
            run_name = f"run_{run_idx:02d}"
            run_dir = results_root / seq_dir.name / run_name
            meta = run_sequence_with_retries(
                shell_ctx=shell_ctx,
                seq_dir=seq_dir,
                run_dir=run_dir,
                mode=mode,
                profile=profile,
                params_file=params_file,
                trajectory_policy=trajectory_policy,
                cores=cores,
                retries=retries,
                node_name=node_name,
                node_wait_timeout=node_wait_timeout,
                bag_timeout=bag_timeout,
                auto_exit_timeout=auto_exit_timeout,
                sigint_timeout=sigint_timeout,
                startup_settle_sec=startup_settle_sec,
                bag_play_rate=bag_play_rate,
                monitor_sample_sec=monitor_sample_sec,
                evo_t_max_diff=evo_t_max_diff,
                inter_run_delay_sec=inter_run_delay_sec,
            )
            (run_dir / "run_meta.json").write_text(json.dumps(meta, indent=2) + "\n", encoding="utf-8")
            all_run_metas.append(meta)
            if meta["success"]:
                console.ok("Run complete", f"{seq_dir.name}/{run_name} rmse={meta['rmse']}")
            else:
                console.fail("Run failed", f"{seq_dir.name}/{run_name} {meta['error']}")
                overall_success = False
                break
            if inter_run_delay_sec > 0 and run_idx < runs:
                time.sleep(inter_run_delay_sec)
        if not overall_success:
            break

    generate_aggregate_outputs(results_root)
    console.section("Outputs")
    console.ok("Experiment summary", str(results_root / "experiment_summary.csv"))
    console.ok("Polling report", str(results_root / "polling_results.md"))
    console.ok("Terminal log", str(log_file))
    return 0 if overall_success else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
