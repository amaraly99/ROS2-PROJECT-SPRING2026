#!/usr/bin/env python3
"""
Background ORB-SLAM2 CPU sampler.

Resolves the actual `mono` / `stereo` executable launched underneath `ros2 run`
and samples both process-level and per-thread CPU usage via `/proc`.
"""

from __future__ import annotations

import argparse
import csv
import json
import os
import signal
import sys
import time
from collections import Counter, defaultdict
from pathlib import Path


RUNNING = True


def _stop(_signum, _frame):
    global RUNNING
    RUNNING = False


signal.signal(signal.SIGTERM, _stop)
signal.signal(signal.SIGINT, _stop)


def pid_exists(pid: int) -> bool:
    return Path(f"/proc/{pid}").exists()


def read_comm(path: Path) -> str:
    try:
        return path.read_text().strip()
    except Exception:
        return ""


def read_cmdline(pid: int) -> str:
    try:
        raw = Path(f"/proc/{pid}/cmdline").read_bytes()
        return raw.replace(b"\x00", b" ").decode("utf-8", "replace").strip()
    except Exception:
        return ""


def read_exe(pid: int) -> str:
    try:
        return os.path.basename(os.readlink(f"/proc/{pid}/exe"))
    except Exception:
        return ""


def parse_stat(path: Path):
    raw = path.read_text().strip()
    rparen = raw.rfind(")")
    rest = raw[rparen + 2 :].split()
    utime = int(rest[11])
    stime = int(rest[12])
    processor = int(rest[36])
    ppid = int(rest[1])
    return utime + stime, processor, ppid


def collect_descendants(root_pid: int):
    children = defaultdict(list)
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit():
            continue
        stat_path = entry / "stat"
        try:
            _total, _cpu, ppid = parse_stat(stat_path)
        except Exception:
            continue
        children[ppid].append(int(entry.name))

    out = []
    stack = [root_pid]
    seen = set()
    while stack:
        pid = stack.pop()
        if pid in seen:
            continue
        seen.add(pid)
        for child in children.get(pid, []):
            out.append(child)
            stack.append(child)
    return out


def cmdline_has_target(cmdline: str, target_exec: str) -> bool:
    for token in [tok for tok in cmdline.split() if tok]:
        if os.path.basename(token) == target_exec:
            return True
    return False


def is_exact_target_process(pid: int, target_exec: str) -> bool:
    comm = read_comm(Path(f"/proc/{pid}/comm"))
    exe = read_exe(pid)
    return comm == target_exec or exe == target_exec


def is_wrapped_target_process(pid: int, target_exec: str) -> bool:
    if is_exact_target_process(pid, target_exec):
        return True
    return cmdline_has_target(read_cmdline(pid), target_exec)


def find_best_target_pid(root_pid: int, target_exec: str):
    if not pid_exists(root_pid):
        return None, "launcher_gone"

    descendants = collect_descendants(root_pid)

    for pid in descendants:
        if pid_exists(pid) and is_exact_target_process(pid, target_exec):
            return pid, "resolved_descendant_exact"

    if pid_exists(root_pid) and is_exact_target_process(root_pid, target_exec):
        return root_pid, "resolved_root_exact"

    for pid in descendants:
        if pid_exists(pid) and is_wrapped_target_process(pid, target_exec):
            return pid, "resolved_descendant_wrapped"

    if pid_exists(root_pid) and is_wrapped_target_process(root_pid, target_exec):
        return root_pid, "resolved_root_wrapped"

    return None, "not_found_under_launcher"


def resolution_is_exact(resolution: str) -> bool:
    return resolution.endswith("_exact")


def resolve_actual_pid(root_pid: int, target_exec: str, timeout_s: float = 30.0, prefer_exact_grace_s: float = 12.0):
    deadline = time.monotonic() + timeout_s
    exact_deadline = min(deadline, time.monotonic() + prefer_exact_grace_s)
    fallback_pid = None
    fallback_resolution = ""

    while time.monotonic() < deadline and RUNNING:
        pid, resolution = find_best_target_pid(root_pid, target_exec)
        if pid is not None:
            if resolution_is_exact(resolution):
                return pid, resolution
            fallback_pid = pid
            fallback_resolution = resolution
            if time.monotonic() >= exact_deadline and pid_exists(fallback_pid):
                return fallback_pid, f"{fallback_resolution}_after_exact_wait"
        elif resolution == "launcher_gone":
            return None, "launcher_gone"
        time.sleep(0.2)

    if fallback_pid is not None and pid_exists(fallback_pid):
        return fallback_pid, f"{fallback_resolution}_timeout_fallback"
    if pid_exists(root_pid):
        return None, "not_found_under_launcher"
    return None, "not_found"


def write_info(info_path: Path, launcher_pid: int, orbslam_pid: int | None, resolution: str, target_exec: str):
    info = {
        "launcher_pid": launcher_pid,
        "orbslam_pid": orbslam_pid,
        "target_exec": target_exec,
        "resolution": resolution,
    }
    if orbslam_pid is not None and pid_exists(orbslam_pid):
        info["orbslam_comm"] = read_comm(Path(f"/proc/{orbslam_pid}/comm"))
        info["orbslam_exe"] = read_exe(orbslam_pid)
        info["orbslam_cmdline"] = read_cmdline(orbslam_pid)
    info_path.write_text(json.dumps(info, indent=2) + "\n")


def prime_monitor_state(pid: int):
    proc_total, proc_cpu, _ = parse_stat(Path(f"/proc/{pid}/stat"))
    task_dir = Path(f"/proc/{pid}/task")
    prev_threads = {}
    if task_dir.exists():
        for task in task_dir.iterdir():
            tid = int(task.name)
            total, cpu, _ = parse_stat(task / "stat")
            prev_threads[tid] = (total, cpu)
    return proc_total, proc_cpu, task_dir, prev_threads


def write_headers(process_csv: Path, thread_csv: Path, summary_csv: Path):
    process_csv.parent.mkdir(parents=True, exist_ok=True)
    process_csv.write_text("wall_time_unix,elapsed_s,orbslam_pid,last_cpu,process_cpu_percent\n")
    thread_csv.write_text("wall_time_unix,elapsed_s,orbslam_pid,tid,comm,last_cpu,thread_cpu_percent\n")
    summary_csv.write_text("tid,comm,samples,mean_cpu_percent,max_cpu_percent,dominant_core,last_cpu\n")


def run_monitor(args) -> int:
    hz = os.sysconf(os.sysconf_names["SC_CLK_TCK"])
    launcher_pid = args.launcher_pid
    info_path = args.info_json
    process_csv = args.process_csv
    thread_csv = args.thread_csv
    summary_csv = args.summary_csv
    target_exec = args.target_exec

    actual_pid, resolution = resolve_actual_pid(launcher_pid, target_exec)
    write_info(info_path, launcher_pid, actual_pid, resolution, target_exec)

    if actual_pid is None or not pid_exists(actual_pid):
        write_headers(process_csv, thread_csv, summary_csv)
        return 0

    process_csv.parent.mkdir(parents=True, exist_ok=True)
    summary = {}

    with process_csv.open("w", newline="") as proc_f, thread_csv.open("w", newline="") as thr_f:
        proc_writer = csv.writer(proc_f)
        thr_writer = csv.writer(thr_f)
        proc_writer.writerow(["wall_time_unix", "elapsed_s", "orbslam_pid", "last_cpu", "process_cpu_percent"])
        thr_writer.writerow(["wall_time_unix", "elapsed_s", "orbslam_pid", "tid", "comm", "last_cpu", "thread_cpu_percent"])

        start_wall = time.time()
        prev_sample_wall = time.monotonic()
        prev_proc_total, _prev_proc_cpu, task_dir, prev_threads = prime_monitor_state(actual_pid)

        while RUNNING:
            if not pid_exists(actual_pid):
                next_pid, next_resolution = find_best_target_pid(launcher_pid, target_exec)
                if next_pid is None or not pid_exists(next_pid):
                    break
                actual_pid = next_pid
                resolution = next_resolution
                write_info(info_path, launcher_pid, actual_pid, resolution, target_exec)
                prev_sample_wall = time.monotonic()
                prev_proc_total, _prev_proc_cpu, task_dir, prev_threads = prime_monitor_state(actual_pid)
                continue

            next_pid, next_resolution = find_best_target_pid(launcher_pid, target_exec)
            if next_pid is not None and resolution_is_exact(next_resolution):
                if next_pid != actual_pid:
                    actual_pid = next_pid
                    resolution = next_resolution
                    write_info(info_path, launcher_pid, actual_pid, resolution, target_exec)
                    prev_sample_wall = time.monotonic()
                    prev_proc_total, _prev_proc_cpu, task_dir, prev_threads = prime_monitor_state(actual_pid)
                    continue
                if resolution != next_resolution:
                    resolution = next_resolution
                    write_info(info_path, launcher_pid, actual_pid, resolution, target_exec)

            time.sleep(args.sample_sec)
            now_wall = time.time()
            now_mono = time.monotonic()
            delta_t = now_mono - prev_sample_wall
            if delta_t <= 0:
                continue
            prev_sample_wall = now_mono

            try:
                proc_total, proc_cpu, _ = parse_stat(Path(f"/proc/{actual_pid}/stat"))
            except Exception:
                break

            proc_cpu_pct = 100.0 * (proc_total - prev_proc_total) / (delta_t * hz)
            prev_proc_total = proc_total

            elapsed_s = now_wall - start_wall
            proc_writer.writerow([f"{now_wall:.3f}", f"{elapsed_s:.3f}", actual_pid, proc_cpu, f"{proc_cpu_pct:.3f}"])
            proc_f.flush()

            current_threads = {}
            if not task_dir.exists():
                break

            for task in task_dir.iterdir():
                try:
                    tid = int(task.name)
                    total, last_cpu, _ = parse_stat(task / "stat")
                    comm = read_comm(task / "comm")
                except Exception:
                    continue

                prev_total, _prev_cpu = prev_threads.get(tid, (total, last_cpu))
                thread_cpu_pct = 100.0 * (total - prev_total) / (delta_t * hz)
                thr_writer.writerow([f"{now_wall:.3f}", f"{elapsed_s:.3f}", actual_pid, tid, comm, last_cpu, f"{thread_cpu_pct:.3f}"])
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

    with summary_csv.open("w", newline="") as summary_f:
        writer = csv.writer(summary_f)
        writer.writerow(["tid", "comm", "samples", "mean_cpu_percent", "max_cpu_percent", "dominant_core", "last_cpu"])
        for tid, stats in sorted(summary.items(), key=lambda item: (-item[1]["sum_cpu"], item[0])):
            dominant_core = ""
            if stats["core_counts"]:
                dominant_core = stats["core_counts"].most_common(1)[0][0]
            mean_cpu = stats["sum_cpu"] / stats["samples"] if stats["samples"] else 0.0
            writer.writerow([tid, stats["comm"], stats["samples"], f"{mean_cpu:.3f}", f"{stats['max_cpu']:.3f}", dominant_core, stats["last_cpu"]])

    return 0


def parse_args():
    parser = argparse.ArgumentParser(description="Monitor ORB-SLAM2 process and per-thread CPU usage")
    parser.add_argument("--launcher-pid", type=int, required=True)
    parser.add_argument("--info-json", type=Path, required=True)
    parser.add_argument("--process-csv", type=Path, required=True)
    parser.add_argument("--thread-csv", type=Path, required=True)
    parser.add_argument("--summary-csv", type=Path, required=True)
    parser.add_argument("--sample-sec", type=float, default=0.05)
    parser.add_argument("--target-exec", required=True)
    return parser.parse_args()


def main() -> int:
    return run_monitor(parse_args())


if __name__ == "__main__":
    raise SystemExit(main())
