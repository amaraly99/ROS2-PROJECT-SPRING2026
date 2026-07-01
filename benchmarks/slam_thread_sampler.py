#!/usr/bin/env python3
"""
slam_thread_sampler.py — per-thread CPU sampler for a SLAM sidecar container.

Usage:
    python3 slam_thread_sampler.py <container> <out_csv> [interval_sec]

Output CSV columns:  elapsed_s, thread_name, cpu_percent
  - One row per active thread per sample interval.
  - cpu_percent is % of one core (ORBGBA can spike >100% when GBA fires).
  - Thread names from /proc/1/task/<tid>/comm inside the container.
    After `exec` in the entrypoint, PID 1 = the SLAM binary itself.
  - Single docker exec per sample (batch read) to keep overhead low.
  - Self-terminates when the container exits.

Methodology matches monitor_cpu_orb.py from the EuRoC benchmarking harness.
"""

import csv
import subprocess
import sys
import time

CONTAINER = sys.argv[1] if len(sys.argv) > 1 else None
OUT_CSV   = sys.argv[2] if len(sys.argv) > 2 else None
INTERVAL  = float(sys.argv[3]) if len(sys.argv) > 3 else 2.0

if not CONTAINER or not OUT_CSV:
    sys.exit("Usage: slam_thread_sampler.py <container> <out_csv> [interval_sec]")

CLK_TCK = 100  # Hz — standard Linux aarch64 default


def container_running() -> bool:
    r = subprocess.run(
        ["sudo", "docker", "inspect", CONTAINER],
        capture_output=True
    )
    return r.returncode == 0


def batch_read_threads() -> dict:
    """One docker exec → {tid: (comm, utime+stime)} for all /proc/1/task/ entries."""
    r = subprocess.run(
        ["sudo", "docker", "exec", CONTAINER, "bash", "-c",
         "for d in /proc/1/task/*/; do "
         "tid=\"${d%/}\"; tid=\"${tid##*/}\"; "
         "comm=$(cat \"${d}comm\" 2>/dev/null || echo '?'); "
         "stat=$(cat \"${d}stat\" 2>/dev/null || echo ''); "
         "echo \"${tid}|${comm}|${stat}\"; "
         "done"],
        capture_output=True, text=True, timeout=6
    )
    if r.returncode != 0 or not r.stdout.strip():
        return {}
    result = {}
    for line in r.stdout.splitlines():
        parts = line.split("|", 2)
        if len(parts) < 3:
            continue
        tid, comm, stat = parts
        fields = stat.split()
        if len(fields) < 15:
            continue
        try:
            ticks = int(fields[13]) + int(fields[14])   # utime + stime
        except (ValueError, IndexError):
            continue
        result[tid] = (comm.strip(), ticks)
    return result


start_time = time.monotonic()
prev_sample: dict = {}
prev_time: float  = 0.0

with open(OUT_CSV, "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(["elapsed_s", "thread_name", "cpu_percent"])
    f.flush()

    # Warm-up sample (no output — just establishes baseline ticks).
    if container_running():
        prev_sample = batch_read_threads()
        prev_time   = time.monotonic()
    time.sleep(INTERVAL)

    while container_running():
        now    = time.monotonic()
        sample = batch_read_threads()
        if sample and prev_sample:
            dt = now - prev_time
            for tid, (comm, ticks) in sample.items():
                if tid in prev_sample:
                    _, prev_ticks = prev_sample[tid]
                    dticks = ticks - prev_ticks
                    pct = 100.0 * dticks / (CLK_TCK * dt) if dt > 0 else 0.0
                    writer.writerow([
                        f"{now - start_time:.3f}",
                        comm,
                        f"{pct:.2f}",
                    ])
            f.flush()
        prev_sample = sample
        prev_time   = now
        time.sleep(INTERVAL)
