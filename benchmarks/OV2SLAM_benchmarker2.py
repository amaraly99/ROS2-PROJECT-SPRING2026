#!/usr/bin/env python3

"""OV2SLAM benchmark runner (v2) — fresh, minimal, correct.

Runs OV2SLAM exactly the way a bare manual session does — and that is the whole
point:

    ros2 run ov2slam ov2slam_node <params>     # no use_sim_time, plain log file
    ros2 bag play <bag> --clock --rate 1.0      # ALWAYS real time (rate 1.0)
    evo_ape tum gt.tum trajectory.tum --align   # evaluate

then adds light CPU monitoring, evo evaluation and aggregation around it.

Why v1 produced broken stereo trajectories
-------------------------------------------
OV2SLAM subscribes to the camera topics with `SensorDataQoS().keep_last(10)`,
i.e. BEST_EFFORT, depth 10. v1 mirrored OV2SLAM's ~400 line/s stdout through a
Python PTY pump and sampled /proc for every thread at 20 Hz, flushing two CSVs
each sample. On a 4-core SBC that starves OV2SLAM's rclcpp executor thread; the
10-deep best-effort image queue overflows and DDS silently drops ~half the
frames -> tracking diverges (RMSE 1-2 m) and OV2SLAM's end-of-stream heuristic
fires early. Run the very same node/bag with no pump and a light monitor and it
delivers all frames and ~0.038 m on MH_01.

So v2:
  * launches the node with a plain file redirect (no PTY pump),
  * does NOT pass use_sim_time (trajectory stamps come from image headers),
  * samples CPU in memory at ~1 Hz and writes the CSVs once at the end,
  * keeps bag rate at 1.0 and instead WAITS for OV2SLAM to finish: accurate
    stereo runs ~0.6x real time on this box, so after the bag ends OV2SLAM still
    has a large backlog to drain. We wait for it to self-exit (it does, once the
    backlog is empty) before evaluating, and only SIGINT as a last resort.
  * verifies the trajectory actually spans the whole sequence; a short
    trajectory means frames were dropped -> the run failed and is retried.

Supports all four EuRoC profiles: accurate/fast x stereo/mono.
"""

from __future__ import annotations

import argparse
import json
import os
import shlex
import shutil
import threading
import time
from collections import Counter
from pathlib import Path
from typing import Any

from benchmark_common import (
    Console,
    ShellContext,
    collect_descendants,
    count_tum_poses,
    datetime_compact,
    detect_ros_setup_files,
    discover_sequences,
    generate_aggregate_outputs,
    generate_cpu_plots,
    launch_background,
    mean_csv_column,
    parse_stat,
    pid_exists,
    prepare_ground_truth,
    read_cmdline,
    read_comm,
    read_exe,
    resolve_bag_target,
    resolve_dir,
    resolve_file,
    run_evo_and_plots,
    select_sequences,
    sequence_key,
    terminate_process_group,
    wait_for_ros2_node,
)

try:
    import yaml
except ImportError:
    yaml = None  # type: ignore[assignment]


SCRIPT_DIR = Path(__file__).resolve().parent
WORKSPACE_ROOT = Path("/workspace") if Path("/workspace").is_dir() else SCRIPT_DIR.parent
DATASET_ROOT = WORKSPACE_ROOT / "datasets" / "euroc"
PARAMS_ROOT = WORKSPACE_ROOT / "src" / "ov2slam_ros" / "parameters_files"
OUTPUT_ROOT = WORKSPACE_ROOT / "results" / "ov2slam_benchmark_v2"
LOG_DIR = WORKSPACE_ROOT / "run_logs"
CONFIG_PATH = SCRIPT_DIR / "experiment_configuration.yaml"

NODE_NAME = "ov2slam_node"
BAG_RATE = 1.0  # MANDATORY: OV2SLAM is always benchmarked at real time.

ALL_COMBOS = [("accurate", "stereo"), ("accurate", "mono"), ("fast", "stereo"), ("fast", "mono")]
DEFAULT_SEQUENCES = [
    "MH_01_easy", "MH_02_easy", "MH_03_medium", "MH_04_difficult", "MH_05_difficult",
    "V1_01_easy", "V1_02_medium", "V1_03_difficult", "V2_01_easy", "V2_02_medium",
]

# Paper-style trajectory preference: online keyframe trajectory, no offline full BA.
TRAJECTORY_CANDIDATES = ["ov2slam_kfs_traj.txt", "ov2slam_traj.txt", "ov2slam_fullba_kfs_traj.txt"]

console = Console()


# ──────────────────────────────────────────────────────────────────────────
# Lightweight CPU monitor
#
# Reads /proc once per `sample_sec`, accumulates everything in memory and writes
# the CSVs once on stop(). No per-sample disk flush, no stdout pumping: the goal
# is to observe OV2SLAM without stealing CPU from its rclcpp executor (which would
# make the best-effort image subscription drop frames).
# ──────────────────────────────────────────────────────────────────────────
class CpuMonitor:
    def __init__(self, launcher_pid: int, run_dir: Path, sample_sec: float = 1.0) -> None:
        self.launcher_pid = launcher_pid
        self.run_dir = run_dir
        self.sample_sec = sample_sec
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self.hz = os.sysconf(os.sysconf_names["SC_CLK_TCK"])
        self.process_csv = run_dir / "ov2slam_process_cpu.csv"
        self.thread_summary_csv = run_dir / "ov2slam_thread_cpu_summary.csv"
        self.info_path = run_dir / "ov2slam_pid_info.json"

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        self._thread.join(timeout=15)

    def _resolve_node_pid(self, timeout_s: float = 30.0) -> int | None:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline and not self._stop.is_set():
            for pid in [self.launcher_pid, *collect_descendants(self.launcher_pid)]:
                if read_comm(Path(f"/proc/{pid}/comm")) == NODE_NAME:
                    return pid
            time.sleep(0.5)
        return None

    def _run(self) -> None:
        pid = self._resolve_node_pid()
        if pid is None:
            return
        self.info_path.write_text(
            json.dumps({
                "launcher_pid": self.launcher_pid,
                "ov2slam_pid": pid,
                "ov2slam_comm": read_comm(Path(f"/proc/{pid}/comm")),
                "ov2slam_exe": read_exe(pid),
                "ov2slam_cmdline": read_cmdline(pid),
            }, indent=2) + "\n",
            encoding="utf-8",
        )

        start = time.monotonic()
        prev_t = start
        prev_proc, _, _ = parse_stat(Path(f"/proc/{pid}/stat"))
        prev_threads: dict[int, int] = {}
        task_dir = Path(f"/proc/{pid}/task")
        for task in task_dir.iterdir():
            try:
                prev_threads[int(task.name)] = parse_stat(task / "stat")[0]
            except Exception:
                continue

        proc_series: list[tuple[float, float]] = []                 # (elapsed_s, cpu%)
        thr: dict[int, dict[str, Any]] = {}                         # tid -> accumulators

        while pid_exists(pid) and not self._stop.is_set():
            time.sleep(self.sample_sec)
            now = time.monotonic()
            dt = now - prev_t
            prev_t = now
            if dt <= 0:
                continue
            try:
                proc_total = parse_stat(Path(f"/proc/{pid}/stat"))[0]
            except Exception:
                break
            proc_series.append((now - start, 100.0 * (proc_total - prev_proc) / (dt * self.hz)))
            prev_proc = proc_total

            if not task_dir.exists():
                break
            current: dict[int, int] = {}
            for task in task_dir.iterdir():
                try:
                    tid = int(task.name)
                    total, last_cpu, _ = parse_stat(task / "stat")
                    comm = read_comm(task / "comm")
                except Exception:
                    continue
                pct = 100.0 * (total - prev_threads.get(tid, total)) / (dt * self.hz)
                current[tid] = total
                acc = thr.setdefault(tid, {"comm": comm, "n": 0, "sum": 0.0, "max": 0.0, "cores": Counter()})
                acc["comm"] = comm
                acc["n"] += 1
                acc["sum"] += pct
                acc["max"] = max(acc["max"], pct)
                acc["cores"][last_cpu] += 1
            prev_threads = current

        self._write_csvs(pid, proc_series, thr)

    def _write_csvs(self, pid: int, proc_series: list[tuple[float, float]], thr: dict[int, dict[str, Any]]) -> None:
        import csv

        with self.process_csv.open("w", newline="", encoding="utf-8") as fh:
            w = csv.writer(fh)
            w.writerow(["elapsed_s", "ov2slam_pid", "process_cpu_percent"])
            for elapsed, pct in proc_series:
                w.writerow([f"{elapsed:.3f}", pid, f"{pct:.3f}"])

        with self.thread_summary_csv.open("w", newline="", encoding="utf-8") as fh:
            w = csv.writer(fh)
            w.writerow(["tid", "comm", "samples", "mean_cpu_percent", "max_cpu_percent", "dominant_core"])
            for tid, acc in sorted(thr.items(), key=lambda kv: -kv[1]["sum"]):
                core = acc["cores"].most_common(1)[0][0] if acc["cores"] else ""
                mean = acc["sum"] / acc["n"] if acc["n"] else 0.0
                w.writerow([tid, acc["comm"], acc["n"], f"{mean:.3f}", f"{acc['max']:.3f}", core])


# ──────────────────────────────────────────────────────────────────────────
# Helpers
# ──────────────────────────────────────────────────────────────────────────
def build_shell_context() -> ShellContext:
    ld = []
    opencv_lib = WORKSPACE_ROOT / "opencv" / "build" / "lib"
    if opencv_lib.is_dir():
        ld.append(opencv_lib)
    return ShellContext(
        workspace_root=WORKSPACE_ROOT,
        setup_files=detect_ros_setup_files(WORKSPACE_ROOT),
        env_vars={"RMW_IMPLEMENTATION": "rmw_cyclonedds_cpp", "PYTHONHOME": ""},
        ld_library_prepend=ld,
    )


def params_for(profile: str, mode: str) -> Path:
    return PARAMS_ROOT / profile / "euroc" / f"euroc_{mode}.yaml"


def tum_last_timestamp(path: Path) -> float | None:
    if not path.is_file():
        return None
    last = None
    for line in path.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if line and not line.startswith("#"):
            try:
                last = float(line.split()[0])
            except (ValueError, IndexError):
                continue
    return last


def tum_first_timestamp(path: Path) -> float | None:
    if not path.is_file():
        return None
    for line in path.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if line and not line.startswith("#"):
            try:
                return float(line.split()[0])
            except (ValueError, IndexError):
                continue
    return None


def gt_coverage_pct(gt_file: Path, est_file: Path) -> float | None:
    """Fraction of the ground-truth *timeline* the estimate actually spans.

    EuRoC GT is dense (~200 Hz) while the OV2SLAM estimate is sparse keyframes, so
    a pose-count ratio is meaningless (it would read ~2% even for a perfect run).
    Instead we measure temporal coverage: a run that initialises late or loses
    tracking early spans only a fraction of the GT span and is flagged degenerate.
    """
    gt_lo, gt_hi = tum_first_timestamp(gt_file), tum_last_timestamp(gt_file)
    est_lo, est_hi = tum_first_timestamp(est_file), tum_last_timestamp(est_file)
    if None in (gt_lo, gt_hi, est_lo, est_hi) or gt_hi <= gt_lo:
        return None
    overlap = max(0.0, min(est_hi, gt_hi) - max(est_lo, gt_lo))
    return 100.0 * overlap / (gt_hi - gt_lo)


def select_trajectory(run_dir: Path) -> tuple[Path, str]:
    for name in TRAJECTORY_CANDIDATES:
        path = run_dir / name
        if path.is_file() and path.stat().st_size > 64:
            return path, name
    raise RuntimeError("OV2SLAM produced no usable trajectory (it likely crashed).")


def reset_run_dir(run_dir: Path) -> None:
    run_dir.mkdir(parents=True, exist_ok=True)
    for pattern in ("ov2slam*", "bag*.log", "gt.tum", "trajectory.tum", "*.png", "*_evo.*", "run_meta.json"):
        for path in run_dir.glob(pattern):
            if path.is_file():
                path.unlink()


def check_socket_buffer() -> None:
    """Warn loudly if net.core.rmem_max is too small for cyclonedds image streams.

    A single EuRoC frame is ~361 KB; the default 208 KB buffer drops frames, which
    desyncs the stereo pair ("Throw img -- Sync error") and makes OV2SLAM exit early.
    """
    try:
        rmem = int(Path("/proc/sys/net/core/rmem_max").read_text().strip())
    except Exception:
        return
    if rmem < 2_000_000:
        console.warn("DDS socket buffer", f"net.core.rmem_max={rmem} is too small — cyclonedds WILL drop camera frames.")
        console.warn("Fix", "sudo sysctl -w net.core.rmem_max=16777216 (also in start_container.sh / /etc/sysctl.d)")
    else:
        console.ok("DDS socket buffer", f"net.core.rmem_max={rmem} (OK)")


def kill_stray_nodes(shell_ctx: ShellContext) -> None:
    """Make sure no OV2SLAM/bag process from a previous run is still on the graph."""
    for pattern in ("ov2slam_node", "ros2 bag play", "ros2 run ov2slam"):
        os.system(f"pkill -9 -f {shlex.quote(pattern)} >/dev/null 2>&1")
    time.sleep(1.0)


def resolve_node_pid(launcher_pid: int) -> int | None:
    """Find the actual ov2slam_node process under the `ros2 run` launcher."""
    for pid in [launcher_pid, *collect_descendants(launcher_pid)]:
        if read_comm(Path(f"/proc/{pid}/comm")) == NODE_NAME:
            return pid
    return None


def proc_state(pid: int) -> str:
    """Single-char process state from /proc/<pid>/stat ('Z' == zombie/finished)."""
    try:
        raw = Path(f"/proc/{pid}/stat").read_text(encoding="utf-8")
        return raw[raw.rfind(")") + 2]
    except Exception:
        return ""  # gone


def wait_for_node_exit(launcher_pid: int, timeout: float) -> bool:
    """Wait until OV2SLAM's node process drains its backlog and self-exits.

    We watch the actual ov2slam_node process, NOT the `ros2 run` launcher wrapper.
    The wrapper does NOT exit when the node self-terminates (it lingers / leaves the
    node as a zombie), so waiting on the launcher PID always burns the full timeout.
    The node, by contrast, disappears (or goes 'Z') the instant it finishes writing
    its trajectory — which is the real "drain complete" signal.
    """
    node_pid = resolve_node_pid(launcher_pid)
    if node_pid is None:
        return True  # node already gone -> draining is done
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if not pid_exists(node_pid) or proc_state(node_pid) == "Z":
            return True
        time.sleep(0.5)
    return False


# ──────────────────────────────────────────────────────────────────────────
# One run
# ──────────────────────────────────────────────────────────────────────────
def run_once(
    shell_ctx: ShellContext,
    seq_dir: Path,
    run_dir: Path,
    profile: str,
    mode: str,
    params_file: Path,
    cfg: dict[str, Any],
) -> dict[str, Any]:
    seq_name = sequence_key(seq_dir)
    warnings: list[str] = []
    node_proc = bag_proc = monitor = None
    gt_file: Path | None = None
    gt_verdict = ""
    traj_used = ""
    rmse: float | None = None
    gt_coverage: float | None = None
    success = False
    error = ""
    traj_end = seq_end = None

    node_wait = float(cfg["node_wait_timeout"])
    settle = float(cfg["startup_settle_sec"])
    bag_timeout = float(cfg["bag_timeout"])
    drain_timeout = float(cfg["drain_timeout"])
    sigint_timeout = float(cfg["sigint_timeout"])
    sample_sec = float(cfg["monitor_sample_sec"])
    t_max_diff = float(cfg["t_max_diff"])
    cores = str(cfg["cores"]).strip()
    complete_tol = float(cfg["completeness_tolerance_sec"])
    min_coverage = float(cfg["min_gt_coverage_pct"])

    try:
        reset_run_dir(run_dir)
        bag = resolve_bag_target(seq_dir)
        try:
            gt_file, gt_src, gt_verdict = prepare_ground_truth(seq_dir, run_dir)
            seq_end = tum_last_timestamp(gt_file)
            console.step("Ground truth", gt_src)
        except (FileNotFoundError, ValueError) as exc:
            gt_verdict = f"GT unavailable: {exc}"
            warnings.append(gt_verdict)
            console.warn("Ground truth", gt_verdict)

        # 1) Launch OV2SLAM — plain file redirect, NO use_sim_time, NO PTY pump.
        ov_cmd = ["ros2", "run", "ov2slam", "ov2slam_node", str(params_file)]
        if cores:
            ov_cmd = ["taskset", "-c", cores] + ov_cmd
            console.step("Core pinning", cores)
        node_proc = launch_background(
            shell_ctx, " ".join(shlex.quote(t) for t in ov_cmd), run_dir, run_dir / "ov2slam.log"
        )
        console.step("OV2SLAM", f"pid={node_proc.pid}")
        if not wait_for_ros2_node(shell_ctx, NODE_NAME, node_wait):
            raise RuntimeError(f"OV2SLAM node did not appear within {node_wait:.0f}s.")
        deadline = time.monotonic() + settle
        while time.monotonic() < deadline:
            if node_proc.poll() is not None:
                raise RuntimeError("OV2SLAM exited during startup. Check ov2slam.log.")
            time.sleep(0.25)

        # 2) Light CPU monitor (optional — it competes with OV2SLAM's executor).
        if cfg["monitor_enabled"]:
            monitor = CpuMonitor(node_proc.pid, run_dir, sample_sec=sample_sec)
            monitor.start()

        # 3) Play the bag at real time.
        bag_cmd = ["ros2", "bag", "play", str(bag.target), "--clock", "--rate", str(BAG_RATE)]
        bag_proc = launch_background(
            shell_ctx, " ".join(shlex.quote(t) for t in bag_cmd), run_dir, run_dir / "bag_play.log"
        )
        console.step("Bag playback", f"{bag.description}  @ rate {BAG_RATE}")
        time.sleep(1.0)
        if bag_proc.poll() is not None:
            raise RuntimeError("ros2 bag play exited before playback started.")

        # 4) Wait for the bag to finish. If OV2SLAM dies here it crashed/early-exited.
        bag_deadline = time.monotonic() + bag_timeout
        while bag_proc.poll() is None:
            if node_proc.poll() is not None:
                warnings.append("OV2SLAM exited during playback (likely dropped frames / crash).")
                terminate_process_group(bag_proc.pid, 5.0)
                break
            if time.monotonic() > bag_deadline:
                terminate_process_group(bag_proc.pid, 5.0)
                raise RuntimeError(f"Bag playback timed out after {bag_timeout:.0f}s.")
            time.sleep(0.5)
        console.step("Playback done", "frames published; draining OV2SLAM backlog")

        # 5) Drain. OV2SLAM runs slower than real time, so after the bag ends it
        #    keeps processing its queued frames, writes its trajectory, then self-
        #    exits. We watch the actual node process (not node_proc — that's the
        #    `ros2 run` launcher wrapper, which lingers after the node exits and
        #    would make us burn the full drain_timeout every run).
        if wait_for_node_exit(node_proc.pid, drain_timeout):
            console.step("OV2SLAM stopped", "self-exited after draining")
        else:
            warnings.append(f"OV2SLAM still running after {drain_timeout:.0f}s; SIGINT to flush.")
        # The launcher wrapper lingers regardless; stop it now that the node is done.
        if node_proc.poll() is None:
            terminate_process_group(node_proc.pid, sigint_timeout)

        # 6) Stop monitor, build CPU plots.
        monitor.stop()
        monitor = None
        if (run_dir / "ov2slam_process_cpu.csv").is_file() and (run_dir / "ov2slam_thread_cpu_summary.csv").is_file():
            try:
                generate_cpu_plots(
                    run_dir / "ov2slam_process_cpu.csv", run_dir / "ov2slam_thread_cpu_summary.csv",
                    run_dir / "ov2slam_cpu_usage.png", run_dir / "ov2slam_threads_cpu_bar.png",
                    f"{seq_name} {profile} {mode} OV2SLAM",
                )
            except Exception:
                pass

        # 7) Trajectory + completeness gate (a short trajectory == dropped frames == failure).
        traj_path, traj_used = select_trajectory(run_dir)
        shutil.copy2(traj_path, run_dir / "trajectory.tum")
        traj_end = tum_last_timestamp(run_dir / "ov2slam_traj.txt") or tum_last_timestamp(run_dir / "trajectory.tum")
        console.step("Trajectory", traj_used)
        if seq_end is not None and traj_end is not None:
            missing = seq_end - traj_end
            if missing > complete_tol:
                raise RuntimeError(
                    f"Trajectory ends {missing:.1f}s before the sequence end "
                    f"(frames were dropped); treating run as failed."
                )

        # 8) Evaluate.
        if gt_file is not None:
            evo = run_evo_and_plots(
                shell_ctx=shell_ctx, gt_file=gt_file, est_file=run_dir / "trajectory.tum",
                out_dir=run_dir, seq_name=seq_name, t_max_diff=t_max_diff,
                correct_scale=(mode == "mono"), est_label="OV2SLAM",
            )
            rmse = evo.rmse
            console.ok("APE RMSE", f"{rmse:.4f} m")

            # Degeneracy gate: how much of the GT timeline did the estimate cover?
            gt_coverage = gt_coverage_pct(gt_file, run_dir / "trajectory.tum")
            if gt_coverage is None:
                warnings.append("Could not compute GT coverage.")
            elif gt_coverage >= min_coverage:
                console.ok("GT covered", f"{gt_coverage:.1f}%")
            else:
                console.warn("GT covered", f"{gt_coverage:.1f}% < {min_coverage:.0f}% — degenerate, rerunning")
                rmse = None  # degenerate run must not pollute the averages
                raise RuntimeError(
                    f"GT coverage {gt_coverage:.1f}% < {min_coverage:.0f}%; degenerate run."
                )
        else:
            warnings.append("No ground truth; skipped evo.")
        success = True

    except Exception as exc:
        error = str(exc)
        console.fail("Run error", error)
    finally:
        if monitor is not None:
            monitor.stop()
        for proc in (bag_proc, node_proc):
            if proc is not None and proc.poll() is None:
                terminate_process_group(proc.pid, sigint_timeout)

    return {
        "sequence": seq_name,
        "run_name": run_dir.name,
        "profile": profile,
        "mode": mode,
        "success": success,
        "rmse": rmse,
        "gt_coverage_pct": gt_coverage,
        "traj_used": traj_used,
        "trajectory_poses": count_tum_poses(run_dir / "trajectory.tum"),
        "cpu_total_avg_pct": mean_csv_column(run_dir / "ov2slam_process_cpu.csv", "process_cpu_percent"),
        "trajectory_end_ts": traj_end,
        "sequence_end_ts": seq_end,
        "warnings": " || ".join(warnings),
        "error": error,
        "gt_verdict": gt_verdict,
    }


def run_with_retries(shell_ctx, seq_dir, run_dir, profile, mode, params_file, cfg) -> dict[str, Any]:
    retries = int(cfg["retries"])
    delay = float(cfg["inter_run_delay_sec"])
    meta = None
    for attempt in range(1, retries + 2):
        if attempt > 1:
            console.warn("Retry", f"{sequence_key(seq_dir)}/{run_dir.name} attempt {attempt}/{retries + 1}")
            kill_stray_nodes(shell_ctx)
        meta = run_once(shell_ctx, seq_dir, run_dir, profile, mode, params_file, cfg)
        if meta["success"]:
            return meta
        if attempt <= retries and delay > 0:
            time.sleep(delay)
    return meta  # type: ignore[return-value]


# ──────────────────────────────────────────────────────────────────────────
# Config + CLI
# ──────────────────────────────────────────────────────────────────────────
DEFAULTS: dict[str, Any] = {
    "node_wait_timeout": 45,
    "startup_settle_sec": 10,
    "bag_timeout": 1200,
    "drain_timeout": 600,            # OV2SLAM runs slower than real time; give it room to finish.
    "sigint_timeout": 30,
    "monitor_sample_sec": 2.0,       # light: in-memory, written once at the end.
    "monitor_enabled": True,
    "t_max_diff": 0.1,
    "cores": "",                     # taskset spec for OV2SLAM, e.g. "0-2"; empty = all cores.
    "completeness_tolerance_sec": 8.0,
    "min_gt_coverage_pct": 80.0,     # run covering < this %% of the GT timeline is degenerate -> failed + rerun.
    "inter_run_delay_sec": 8,
    "retries": 2,
    "n_runs": 1,
}


def load_runtime_cfg() -> dict[str, Any]:
    # Only a few stable knobs come from the shared yaml. Monitor settings are NOT
    # taken from it (the v1 file carries a 0.25s rate that is too aggressive here).
    cfg = dict(DEFAULTS)
    from_yaml = {"node_wait_timeout", "startup_settle_sec", "bag_timeout", "sigint_timeout",
                 "t_max_diff", "inter_run_delay_sec", "retries", "n_runs"}
    if yaml is not None and CONFIG_PATH.is_file():
        raw = yaml.safe_load(CONFIG_PATH.read_text(encoding="utf-8")) or {}
        for section in ("playback", "evaluation", "ov2slam"):
            block = raw.get(section) or {}
            for key in from_yaml:
                if key in block:
                    cfg[key] = block[key]
    return cfg


def parse_args(argv: list[str]) -> argparse.Namespace:
    p = argparse.ArgumentParser(description="OV2SLAM benchmark runner v2 (real-time, all 4 modes).")
    p.add_argument("--profile", choices=["accurate", "fast", "average"])
    p.add_argument("--mode", choices=["stereo", "mono"])
    p.add_argument("--all-modes", action="store_true", help="Run accurate+fast x stereo+mono.")
    p.add_argument("--sequence", action="append", dest="sequences", default=[])
    p.add_argument("--runs", type=int)
    p.add_argument("--retries", type=int)
    p.add_argument("--min-gt-coverage", type=float, help="Min %% of GT timespan a run must cover or it's failed + rerun (default 80).")
    p.add_argument("--cores", help='taskset CPU list for OV2SLAM, e.g. "0-2".')
    p.add_argument("--no-monitor", action="store_true", help="Disable CPU monitoring (max headroom for OV2SLAM).")
    p.add_argument("--monitor-sample", type=float, help="CPU monitor sample period in seconds (default 2.0).")
    p.add_argument("--results-name")
    return p.parse_args(argv)


def main(argv: list[str]) -> int:
    args = parse_args(argv)
    cfg = load_runtime_cfg()
    if args.runs is not None:
        cfg["n_runs"] = args.runs
    if args.retries is not None:
        cfg["retries"] = args.retries
    if args.min_gt_coverage is not None:
        cfg["min_gt_coverage_pct"] = args.min_gt_coverage
    if args.cores is not None:
        cfg["cores"] = args.cores
    if args.no_monitor:
        cfg["monitor_enabled"] = False
    if args.monitor_sample is not None:
        cfg["monitor_sample_sec"] = args.monitor_sample

    if args.all_modes:
        combos = ALL_COMBOS
    else:
        combos = [(args.profile or "accurate", args.mode or "stereo")]

    sequences = args.sequences or DEFAULT_SEQUENCES
    discovered = discover_sequences(DATASET_ROOT)
    if not discovered:
        console.fail("No sequences", str(DATASET_ROOT))
        return 1
    selected = select_sequences(discovered, sequences, "", "")
    if not selected:
        console.fail("No matching sequences", ", ".join(sequences))
        return 1

    results_name = args.results_name or datetime_compact()
    log_file = LOG_DIR / f"terminal_{datetime_compact()}_ov2_benchmarker2.log"
    console.set_log_file(log_file)
    console.banner("OV2SLAM Benchmark v2")

    shell_ctx = build_shell_context()
    check_socket_buffer()
    n_runs = int(cfg["n_runs"])
    overall_ok = True

    for profile, mode in combos:
        params_file = params_for(profile, mode)
        if not params_file.is_file():
            console.fail("Missing params", str(params_file))
            overall_ok = False
            continue
        combo_root = OUTPUT_ROOT / results_name / f"{profile}_{mode}"
        combo_root.mkdir(parents=True, exist_ok=True)

        console.section(f"{profile} / {mode}")
        console.ok("Params", str(params_file))
        console.ok("Sequences", ", ".join(sequence_key(s) for s in selected))
        console.ok("Runs/seq", str(n_runs))
        console.ok("Bag rate", f"{BAG_RATE} (real time)")
        console.ok("Results", str(combo_root))

        for si, seq_dir in enumerate(selected, 1):
            seq_name = sequence_key(seq_dir)
            console.seq_banner(f"{profile}/{mode} · {seq_name}", si, len(selected))
            for ri in range(1, n_runs + 1):
                console.run_banner(ri, n_runs)
                run_dir = combo_root / seq_name / f"run_{ri:02d}"
                kill_stray_nodes(shell_ctx)
                meta = run_with_retries(shell_ctx, seq_dir, run_dir, profile, mode, params_file, cfg)
                (run_dir / "run_meta.json").write_text(json.dumps(meta, indent=2) + "\n", encoding="utf-8")
                if meta["success"]:
                    rmse = meta["rmse"]
                    console.ok("Run complete", f"{seq_name}/run_{ri:02d}  rmse={rmse:.4f} m" if isinstance(rmse, float) else f"{seq_name}/run_{ri:02d}")
                else:
                    console.fail("Run failed", f"{seq_name}/run_{ri:02d}  {meta['error']}")
                    overall_ok = False
                if cfg["inter_run_delay_sec"] and ri < n_runs:
                    time.sleep(float(cfg["inter_run_delay_sec"]))

        try:
            generate_aggregate_outputs(combo_root)
        except Exception as exc:
            console.warn("Aggregate", str(exc))
        console.ok("Summary", str(combo_root / "experiment_summary.csv"))

    console.section("Done")
    console.ok("Terminal log", str(log_file))
    return 0 if overall_ok else 1


if __name__ == "__main__":
    raise SystemExit(main(__import__("sys").argv[1:]))
