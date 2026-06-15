#!/usr/bin/env python3

"""Standalone ORB_SLAM2-style benchmark runner for ORB_SLAM3_ROS2."""

from __future__ import annotations

import argparse
import json
import shlex
import shutil
import sys
import tarfile
import time
from pathlib import Path
from typing import Any

from benchmark_common import (
    BagInfo,
    Console,
    EUROC_SEQUENCES,
    RESULTS_TIMEZONE,
    ProcessMonitor,
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
)


SCRIPT_PATH = Path(__file__).resolve()
SCRIPT_DIR = SCRIPT_PATH.parent
LOCAL_WORKSPACE_ROOT = SCRIPT_DIR.parent
DEFAULT_WORKSPACE_ROOT = Path("/workspace") if Path("/workspace").is_dir() else LOCAL_WORKSPACE_ROOT
DEFAULT_CONFIG_PATH = SCRIPT_DIR / "ORBSLAM3_ROS2_BENCHMARK.yaml"
DEFAULT_DATASET_ROOT = DEFAULT_WORKSPACE_ROOT / "datasets" / "euroc"
DEFAULT_OUTPUT_ROOT = DEFAULT_WORKSPACE_ROOT / "results" / "orbslam3_ros2_orbslam2_style"
DEFAULT_MODE = "stereo-inertial"
DEFAULT_RUNS = 1
DEFAULT_RETRIES = 0

DEFAULT_SETTINGS_BY_MODE = {
    "mono": "src/ORB_SLAM3_ROS2/config/monocular/EuRoC.yaml",
    "stereo": "src/ORB_SLAM3_ROS2/config/stereo/EuRoC.yaml",
    "stereo-inertial": "src/ORB_SLAM3_ROS2/config/stereo-inertial/EuRoC.yaml",
}

DEFAULT_REMAPS_BY_MODE = {
    "mono": ["/cam0/image_raw:=/camera"],
    "stereo": ["/cam0/image_raw:=/camera/left", "/cam1/image_raw:=/camera/right"],
    "stereo-inertial": [
        "/cam0/image_raw:=/camera/left",
        "/cam1/image_raw:=/camera/right",
        "/imu0:=/imu",
    ],
}


console = Console()


def die(message: str) -> None:
    console.fail(message)
    raise SystemExit(1)


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        prog=SCRIPT_PATH.name,
        description="Standalone ORB_SLAM2-style benchmark runner for ORB_SLAM3_ROS2.",
    )
    parser.add_argument("--config", default=str(DEFAULT_CONFIG_PATH))
    parser.add_argument("--mode", choices=["mono", "stereo", "stereo-inertial", "rgbd"])
    parser.add_argument("--sequence", action="append", dest="sequences", default=[])
    parser.add_argument("--runs", type=int)
    parser.add_argument("--workspace-root")
    parser.add_argument("--dataset-root")
    parser.add_argument("--output-root")
    parser.add_argument("--results-name")
    parser.add_argument("--settings")
    parser.add_argument("--vocabulary")
    parser.add_argument("--cores")
    parser.add_argument("--retries", type=int)
    parser.add_argument("--all", action="store_true")
    parser.add_argument("--rectify", action="store_true")
    parser.add_argument("--equalize", action="store_true")
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
    return ShellContext(
        workspace_root=workspace_root,
        setup_files=detect_ros_setup_files(workspace_root),
        env_vars={},
        ld_library_prepend=[],
    )


def resolve_settings_file(workspace_root: Path, mode: str, configured: str) -> Path:
    if configured:
        return resolve_file(configured)
    relative = DEFAULT_SETTINGS_BY_MODE.get(mode)
    if relative is None:
        die(f"Mode '{mode}' requires an explicit settings_file in the config or via --settings.")
    return resolve_file(workspace_root / relative)


def resolve_vocabulary_file(workspace_root: Path, configured: str) -> Path:
    vocab = resolve_file(configured) if configured else resolve_file(workspace_root / "src/ORB_SLAM3_ROS2/vocabulary/ORBvoc.txt")
    if vocab.is_file():
        return vocab
    archive = vocab.with_suffix(vocab.suffix + ".tar.gz") if vocab.suffix else Path(str(vocab) + ".tar.gz")
    if not archive.is_file():
        archive = resolve_file(workspace_root / "src/ORB_SLAM3_ROS2/vocabulary/ORBvoc.txt.tar.gz")
    if archive.is_file():
        archive.parent.mkdir(parents=True, exist_ok=True)
        with tarfile.open(archive, "r:gz") as handle:
            handle.extractall(path=archive.parent)
        if vocab.is_file():
            return vocab
        extracted = archive.parent / "ORBvoc.txt"
        if extracted.is_file():
            return extracted
    die(f"Vocabulary file not found: {vocab}")


def preflight(shell_ctx: ShellContext, dataset_root: Path, settings_file: Path, vocabulary_file: Path, cores: str) -> None:
    console.section("Preflight")
    console.ok("Workspace root", str(shell_ctx.workspace_root))
    console.ok("Dataset root", str(dataset_root))
    console.ok("Settings file", str(settings_file))
    console.ok("Vocabulary file", str(vocabulary_file))

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
    if not settings_file.is_file():
        die(f"Settings file not found: {settings_file}")
    if not vocabulary_file.is_file():
        die(f"Vocabulary file not found: {vocabulary_file}")
    try:
        import matplotlib  # noqa: F401
        import numpy  # noqa: F401
    except Exception:
        die("python3 modules 'matplotlib' and 'numpy' are required for plot generation.")
    console.ok("Required commands", ", ".join(required))


def reset_run_dir(run_dir: Path) -> None:
    run_dir.mkdir(parents=True, exist_ok=True)
    for pattern in (
        "orbslam3.log",
        "bag_play.log",
        "gt.tum",
        "trajectory.tum",
        "KeyFrameTrajectory.txt",
        "orbslam3*.csv",
        "orbslam3*.png",
        "*_evo.txt",
        "*_evo.zip",
        "run_meta.json",
        "core.*",
    ):
        for path in run_dir.glob(pattern):
            if path.is_file():
                path.unlink()


def trajectory_candidate(run_dir: Path) -> tuple[Path, str, str]:
    path = run_dir / "KeyFrameTrajectory.txt"
    if path.is_file() and path.stat().st_size > 64:
        return path, path.name, "ORB_SLAM3 keyframe trajectory written during node shutdown"
    raise RuntimeError(f"Expected ORB_SLAM3 trajectory file was not produced: {path}")


def default_remaps_for_mode(mode: str) -> list[str]:
    return list(DEFAULT_REMAPS_BY_MODE.get(mode, []))


def make_run_meta(
    seq_name: str,
    run_name: str,
    mode: str,
    success: bool,
    rmse: float | None,
    traj_used: str,
    traj_reason: str,
    warnings: list[str],
    error: str,
    gt_verdict: str,
    run_dir: Path,
) -> dict[str, Any]:
    cpu_csv = run_dir / "orbslam3_process_cpu.csv"
    return {
        "sequence": seq_name,
        "run_name": run_name,
        "mode": mode,
        "success": success,
        "rmse": rmse,
        "traj_used": traj_used,
        "traj_reason": traj_reason,
        "warnings": " || ".join(warnings),
        "error": error,
        "gt_verdict": gt_verdict,
        "cpu_total_avg_pct": mean_csv_column(cpu_csv, "process_cpu_percent"),
        "trajectory_poses": count_tum_poses(run_dir / "trajectory.tum"),
        "artifacts": {
            "gt_tum": "gt.tum" if (run_dir / "gt.tum").exists() else None,
            "trajectory_tum": "trajectory.tum" if (run_dir / "trajectory.tum").exists() else None,
            "ape_traj_png": "ape_traj.png" if (run_dir / "ape_traj.png").exists() else None,
            "traj_xy_png": "traj_xy.png" if (run_dir / "traj_xy.png").exists() else None,
            "process_cpu_csv": "orbslam3_process_cpu.csv" if cpu_csv.exists() else None,
            "thread_cpu_csv": "orbslam3_thread_cpu.csv" if (run_dir / "orbslam3_thread_cpu.csv").exists() else None,
            "thread_cpu_summary_csv": (
                "orbslam3_thread_cpu_summary.csv" if (run_dir / "orbslam3_thread_cpu_summary.csv").exists() else None
            ),
        },
    }


def build_orbslam3_command(mode: str, vocabulary_file: Path, settings_file: Path, rectify: bool, equalize: bool) -> list[str]:
    cmd = ["ros2", "run", "orbslam3", mode, str(vocabulary_file), str(settings_file)]
    if mode == "stereo":
        cmd.append("true" if rectify else "false")
    elif mode == "stereo-inertial":
        cmd.append("true" if rectify else "false")
        cmd.append("true" if equalize else "false")
    return cmd


def run_sequence_attempt(
    shell_ctx: ShellContext,
    seq_dir: Path,
    run_dir: Path,
    mode: str,
    vocabulary_file: Path,
    settings_file: Path,
    rectify: bool,
    equalize: bool,
    remaps: list[str],
    cores: str,
    node_name: str,
    node_wait_timeout: float,
    bag_timeout: float,
    sigint_timeout: float,
    startup_settle_sec: float,
    post_bag_settle_sec: float,
    bag_play_rate: float,
    monitor_sample_sec: float,
    evo_t_max_diff: float,
    use_scale_correction: bool,
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

        node_cmd = build_orbslam3_command(mode, vocabulary_file, settings_file, rectify, equalize)
        if cores:
            node_cmd = ["taskset", "-c", cores] + node_cmd
            console.step("Core pinning", cores)
        node_proc = launch_background(
            shell_ctx,
            " ".join(shlex.quote(token) for token in node_cmd),
            run_dir,
            run_dir / "orbslam3.log",
        )
        wrapped_tokens = {mode}
        monitor = ProcessMonitor(
            node_proc.pid,
            run_dir,
            "orbslam3",
            exact_process_names={mode},
            wrapped_cmd_tokens=wrapped_tokens,
            sample_sec=monitor_sample_sec,
        )
        monitor.start()
        console.step("ORB_SLAM3", f"pid={node_proc.pid}")
        if not wait_for_ros2_node(shell_ctx, node_name, node_wait_timeout):
            raise RuntimeError(f"ORB_SLAM3 node '{node_name}' did not appear within {node_wait_timeout}s.")
        time.sleep(startup_settle_sec)
        if node_proc.poll() is not None:
            raise RuntimeError("ORB_SLAM3 exited during startup settle.")

        bag_cmd = ["ros2", "bag", "play", str(bag_info.target), "--clock", "--rate", str(bag_play_rate)]
        for remap in remaps:
            bag_cmd.extend(["--remap", remap])
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

        try:
            bag_rc = bag_proc.wait(timeout=bag_timeout)
        except Exception:
            warnings.append(f"Bag playback timed out after {bag_timeout}s and was killed.")
            terminate_process_group(bag_proc.pid, 2.0)
            bag_proc = None
            raise RuntimeError(f"Bag playback timed out after {bag_timeout}s.")
        bag_proc = None
        if bag_rc != 0:
            warnings.append(f"ros2 bag play exited with rc={bag_rc}; continuing to shutdown ORB_SLAM3.")

        if node_proc.poll() is None:
            time.sleep(post_bag_settle_sec)
            terminate_process_group(node_proc.pid, sigint_timeout)
        if node_proc.poll() is None:
            raise RuntimeError("ORB_SLAM3 did not exit cleanly after SIGINT.")

        trajectory_path, traj_name, traj_reason = trajectory_candidate(run_dir)
        shutil.copy2(trajectory_path, run_dir / "trajectory.tum")
        evo = run_evo_and_plots(
            shell_ctx=shell_ctx,
            gt_file=gt_file,
            est_file=run_dir / "trajectory.tum",
            out_dir=run_dir,
            seq_name=seq_name,
            t_max_diff=evo_t_max_diff,
            correct_scale=use_scale_correction,
            est_label="ORB_SLAM3",
        )
        return make_run_meta(
            seq_name=seq_name,
            run_name=run_dir.name,
            mode=mode,
            success=True,
            rmse=evo.rmse,
            traj_used=traj_name,
            traj_reason=traj_reason,
            warnings=warnings,
            error="",
            gt_verdict=gt_verdict,
            run_dir=run_dir,
        )
    except Exception as exc:
        return make_run_meta(
            seq_name=seq_name,
            run_name=run_dir.name,
            mode=mode,
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
                run_dir / "orbslam3_process_cpu.csv",
                run_dir / "orbslam3_thread_cpu_summary.csv",
                run_dir / "orbslam3_cpu_usage.png",
                run_dir / "orbslam3_threads_cpu_bar.png",
                f"{seq_name} ORB_SLAM3",
            )


def run_sequence_with_retries(
    shell_ctx: ShellContext,
    seq_dir: Path,
    run_dir: Path,
    mode: str,
    vocabulary_file: Path,
    settings_file: Path,
    rectify: bool,
    equalize: bool,
    remaps: list[str],
    cores: str,
    retries: int,
    node_name: str,
    node_wait_timeout: float,
    bag_timeout: float,
    sigint_timeout: float,
    startup_settle_sec: float,
    post_bag_settle_sec: float,
    bag_play_rate: float,
    monitor_sample_sec: float,
    evo_t_max_diff: float,
    use_scale_correction: bool,
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
            vocabulary_file=vocabulary_file,
            settings_file=settings_file,
            rectify=rectify,
            equalize=equalize,
            remaps=remaps,
            cores=cores,
            node_name=node_name,
            node_wait_timeout=node_wait_timeout,
            bag_timeout=bag_timeout,
            sigint_timeout=sigint_timeout,
            startup_settle_sec=startup_settle_sec,
            post_bag_settle_sec=post_bag_settle_sec,
            bag_play_rate=bag_play_rate,
            monitor_sample_sec=monitor_sample_sec,
            evo_t_max_diff=evo_t_max_diff,
            use_scale_correction=use_scale_correction,
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
    mode = args.mode or str(config_value(config, "mode", default=DEFAULT_MODE))
    settings_file = resolve_settings_file(workspace_root, mode, args.settings or str(config_value(config, "settings_file", default="")))
    vocabulary_file = resolve_vocabulary_file(
        workspace_root,
        args.vocabulary or str(config_value(config, "vocabulary_file", default="")),
    )
    runs = args.runs if args.runs is not None else int(config_value(config, "runs", default=DEFAULT_RUNS))
    retries = args.retries if args.retries is not None else int(config_value(config, "retries", default=DEFAULT_RETRIES))
    cores = args.cores if args.cores is not None else str(config_value(config, "cores", "orbslam3_cores", default=""))
    results_name = args.results_name or str(config_value(config, "results_name", default="")) or datetime_compact()
    rectify = args.rectify or coerce_bool(config_value(config, "rectify", default=False), default=False)
    equalize = args.equalize or coerce_bool(config_value(config, "equalize", default=False), default=False)
    remaps = [str(item) for item in ensure_list(config_value(config, "bag_remaps", default=[])) if str(item)]
    if not remaps:
        remaps = default_remaps_for_mode(mode)
    node_name = str(config_value(config, "node_name", default="/ORB_SLAM3_ROS2"))
    node_wait_timeout = float(config_value(config, "node_wait_timeout", default=45))
    bag_timeout = float(config_value(config, "bag_timeout", default=1200))
    sigint_timeout = float(config_value(config, "sigint_timeout", default=30))
    startup_settle_sec = float(config_value(config, "startup_settle_sec", default=2))
    post_bag_settle_sec = float(config_value(config, "post_bag_settle_sec", default=5))
    inter_run_delay_sec = float(config_value(config, "inter_run_delay_sec", default=10))
    monitor_sample_sec = float(config_value(config, "monitor_sample_sec", default=0.05))
    bag_play_rate = float(config_value(config, "bag_play_rate", default=1.0))
    evo_t_max_diff = float(config_value(config, "evo_t_max_diff", default=0.1))
    use_scale_correction = coerce_bool(
        config_value(config, "use_scale_correction", default=(mode == "mono")),
        default=(mode == "mono"),
    )

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
    log_file = workspace_root / "run_logs" / f"terminal_{datetime_compact()}_orbslam3_standalone.log"
    console.set_log_file(log_file)
    console.banner("ORB_SLAM3_ROS2 Standalone Benchmark")

    shell_ctx = build_shell_context(workspace_root)
    preflight(shell_ctx, dataset_root, settings_file, vocabulary_file, cores)

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
    run_config = {
        "config_file": str(source_config if source_config.exists() else DEFAULT_CONFIG_PATH),
        "workspace_root": str(workspace_root),
        "dataset_root": str(dataset_root),
        "results_root": str(results_root),
        "results_timezone": RESULTS_TIMEZONE,
        "mode": mode,
        "settings_file": str(settings_file),
        "vocabulary_file": str(vocabulary_file),
        "rectify": rectify,
        "equalize": equalize,
        "bag_remaps": remaps,
        "cores": cores,
        "runs": runs,
        "retries": retries,
        "selected_sequences": [path.name for path in selected],
        "bag_play_rate": bag_play_rate,
        "evo_t_max_diff": evo_t_max_diff,
        "use_scale_correction": use_scale_correction,
    }
    (results_root / "run_config.json").write_text(json.dumps(run_config, indent=2) + "\n", encoding="utf-8")
    (results_root / "settings_used.yaml").write_text(settings_file.read_text(encoding="utf-8"), encoding="utf-8")

    console.section("Configuration")
    console.ok("Config file", str(source_config if source_config.exists() else DEFAULT_CONFIG_PATH))
    console.ok("Results root", str(results_root))
    console.ok("Mode", mode)
    console.ok("Settings file", str(settings_file))
    console.ok("Vocabulary file", str(vocabulary_file))
    console.ok("Runs per sequence", str(runs))
    console.ok("Sequences", ", ".join(path.name for path in selected))

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
                vocabulary_file=vocabulary_file,
                settings_file=settings_file,
                rectify=rectify,
                equalize=equalize,
                remaps=remaps,
                cores=cores,
                retries=retries,
                node_name=node_name,
                node_wait_timeout=node_wait_timeout,
                bag_timeout=bag_timeout,
                sigint_timeout=sigint_timeout,
                startup_settle_sec=startup_settle_sec,
                post_bag_settle_sec=post_bag_settle_sec,
                bag_play_rate=bag_play_rate,
                monitor_sample_sec=monitor_sample_sec,
                evo_t_max_diff=evo_t_max_diff,
                use_scale_correction=use_scale_correction,
                inter_run_delay_sec=inter_run_delay_sec,
            )
            (run_dir / "run_meta.json").write_text(json.dumps(meta, indent=2) + "\n", encoding="utf-8")
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
