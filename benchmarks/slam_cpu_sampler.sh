#!/usr/bin/env bash
# ─────────────────────────────────────────────────────────────────
# slam_cpu_sampler.sh — sample a SLAM sidecar container's CPU / memory /
# thread count into a CSV until the container disappears.
#
#   slam_cpu_sampler.sh <container> <out_csv> [interval_sec]
#
# Output CSV columns (matches OV2SLAM's ov2slam_process_cpu.csv methodology,
# extended with mem + thread count for the HIL utilisation table):
#   elapsed_s, cpu_percent, mem_mb, threads
#
# `cpu_percent` is the container's whole-process CPU% as reported by
# `docker stats` (multi-threaded → can exceed 100% of one core, e.g. 250%
# across 4 cores). `threads` is the container PID count (≈ live threads for a
# single-process SLAM sidecar). Self-terminates when the container is removed
# (run_stack_hil.sh stop), so no explicit kill is strictly required.
# ─────────────────────────────────────────────────────────────────
set -u

CONTAINER="${1:?usage: slam_cpu_sampler.sh <container> <out_csv> [interval_sec]}"
OUT="${2:?usage: slam_cpu_sampler.sh <container> <out_csv> [interval_sec]}"
INTERVAL="${3:-2}"

echo "elapsed_s,cpu_percent,mem_mb,threads" > "$OUT"
start=$(date +%s.%N)

while sudo docker inspect "$CONTAINER" >/dev/null 2>&1; do
    line=$(sudo docker stats "$CONTAINER" --no-stream \
        --format '{{.CPUPerc}}|{{.MemUsage}}|{{.PIDs}}' 2>/dev/null) || break
    if [[ -z "$line" ]]; then
        sleep "$INTERVAL"
        continue
    fi

    cpu=$(echo "$line"  | cut -d'|' -f1 | tr -d '% ')
    memraw=$(echo "$line" | cut -d'|' -f2 | awk '{print $1}')   # e.g. 123.4MiB
    pids=$(echo "$line" | cut -d'|' -f3 | tr -d ' ')

    # Normalise docker's human memory units to MB.
    memmb=$(echo "$memraw" | awk '
        /GiB/ {gsub(/GiB/,""); printf "%.2f", $1*1024; next}
        /MiB/ {gsub(/MiB/,""); printf "%.2f", $1;       next}
        /KiB/ {gsub(/KiB/,""); printf "%.4f", $1/1024;  next}
        /GB/  {gsub(/GB/,"");  printf "%.2f", $1*1000;  next}
        /MB/  {gsub(/MB/,"");  printf "%.2f", $1;       next}
        /kB/  {gsub(/kB/,"");  printf "%.4f", $1/1000;  next}
                              {gsub(/[A-Za-z]/,""); printf "%.2f", $1}')

    now=$(date +%s.%N)
    elapsed=$(awk -v a="$now" -v b="$start" 'BEGIN{printf "%.3f", a-b}')

    echo "${elapsed},${cpu},${memmb},${pids}" >> "$OUT"
    sleep "$INTERVAL"
done
