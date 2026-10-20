#!/usr/bin/env bash
# ─────────────────────────────────────────────────────────────────
# slam_cpu_sampler.sh — sample a SLAM sidecar container's CPU / memory /
# thread count into a CSV until the container disappears.
#
#   slam_cpu_sampler.sh <container> <out_csv> [interval_sec]
#
# Output CSV columns (the first four are unchanged, so every reader keeps working):
#   elapsed_s, cpu_percent, mem_mb, threads, rss_mb, hwm_mb
#
# cpu_percent is the container's whole-process CPU% from `docker stats`
# (multi-threaded -> can exceed 100% of one core).
#
# Memory (2026-10-19): the Pi kernel boots with cgroup_disable=memory, so
# `docker stats` reports 0 for memory. Memory is therefore read on the host from
# /proc for every process of the container (`docker top`):
#   mem_mb = sum of PSS (smaps_rollup; shared pages are split between processes)
#   rss_mb = sum of VmRSS
#   hwm_mb = sum of VmHWM (the peak RSS of each process, so peaks between samples count)
# threads is the container PID count from `docker stats`.
# Self-terminates when the container is removed (run_stack_hil.sh stop).
# ─────────────────────────────────────────────────────────────────
set -u

CONTAINER="${1:?usage: slam_cpu_sampler.sh <container> <out_csv> [interval_sec]}"
OUT="${2:?usage: slam_cpu_sampler.sh <container> <out_csv> [interval_sec]}"
INTERVAL="${3:-2}"

echo "elapsed_s,cpu_percent,mem_mb,threads,rss_mb,hwm_mb" > "$OUT"
start=$(date +%s.%N)

while sudo docker inspect "$CONTAINER" >/dev/null 2>&1; do
    line=$(sudo docker stats "$CONTAINER" --no-stream \
        --format '{{.CPUPerc}}|{{.PIDs}}' 2>/dev/null) || break
    if [[ -z "$line" ]]; then
        sleep "$INTERVAL"
        continue
    fi
    cpu=$(echo "$line"  | cut -d'|' -f1 | tr -d '% ')
    pids=$(echo "$line" | cut -d'|' -f2 | tr -d ' ')

    # Host PIDs of every process in the container.
    hostpids=$(sudo docker top "$CONTAINER" -eo pid 2>/dev/null | awk 'NR>1 {print $1}')
    pss=0; rss=0; hwm=0
    if [[ -n "$hostpids" ]]; then
        rollups=(); statuses=()
        for p in $hostpids; do rollups+=("/proc/$p/smaps_rollup"); statuses+=("/proc/$p/status"); done
        pss=$(sudo awk '/^Pss:/ {s += $2} END {printf "%.2f", s / 1024}' "${rollups[@]}" 2>/dev/null)
        read -r rss hwm < <(sudo awk '/^VmRSS:/ {r += $2} /^VmHWM:/ {h += $2} END {printf "%.2f %.2f", r / 1024, h / 1024}' "${statuses[@]}" 2>/dev/null)
    fi

    now=$(date +%s.%N)
    elapsed=$(awk -v a="$now" -v b="$start" 'BEGIN {printf "%.3f", a - b}')
    echo "${elapsed},${cpu},${pss:-0},${pids},${rss:-0},${hwm:-0}" >> "$OUT"
    sleep "$INTERVAL"
done
