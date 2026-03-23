#!/bin/bash
# ─── Resource Contention Profiling via perf stat ────────────────────────────
#
# Attaches `perf stat` to running pipeline processes and measures:
#   - L1/L2/LLC cache miss rates
#   - Context switches, CPU migrations
#
# Run this script under different pipeline conditions to compare:
#   Condition A: ovcam + yolo_producer only (SLAM stopped)
#   Condition B: ovcam + SLAM only (YOLO stopped)
#   Condition C: ovcam + yolo + SLAM (no ViSP)
#   Condition D: full pipeline (all 4 processes)
#
# Usage:
#   bash benchmarks/contention_bench.sh --condition A --label "yolo_alone"
#   bash benchmarks/contention_bench.sh --condition D --label "full_pipeline"
#   bash benchmarks/contention_bench.sh --duration 60  (default)
#
# Prerequisites:
#   - The pipeline must already be running (use ./run_stack.sh)
#   - perf must be installed: sudo apt install linux-perf
#   - May need: echo -1 | sudo tee /proc/sys/kernel/perf_event_paranoid
#
# Output: benchmarks/results/contention/<label>_perf.txt
#         benchmarks/results/contention/summary.csv
# ─────────────────────────────────────────────────────────────────────────────

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
RESULTS_DIR="$SCRIPT_DIR/results/contention"
mkdir -p "$RESULTS_DIR"

# ── Defaults ─────────────────────────────────────────────────────────────────
DURATION=60
CONDITION="D"
LABEL=""
QUIET=0

while [[ $# -gt 0 ]]; do
    case $1 in
        --condition) CONDITION="$2"; shift 2 ;;
        --label)     LABEL="$2"; shift 2 ;;
        --duration)  DURATION="$2"; shift 2 ;;
        --quiet)     QUIET=1; shift ;;
        *) echo "Unknown arg: $1"; exit 1 ;;
    esac
done

[[ -z "$LABEL" ]] && LABEL="condition_${CONDITION}"
OUT_FILE="$RESULTS_DIR/${LABEL}_perf.txt"
SUMMARY_CSV="$RESULTS_DIR/summary.csv"

log() { [[ $QUIET -eq 0 ]] && echo "[contention_bench] $*"; }

# ── perf availability check ───────────────────────────────────────────────────
if ! command -v perf &>/dev/null; then
    echo "ERROR: perf not found. Install with: sudo apt install linux-perf"
    exit 1
fi

PARANOID=$(cat /proc/sys/kernel/perf_event_paranoid 2>/dev/null || echo "3")
if [[ "$PARANOID" -gt 1 ]]; then
    echo "WARNING: perf_event_paranoid=$PARANOID — perf may fail to attach."
    echo "         Fix: echo 1 | sudo tee /proc/sys/kernel/perf_event_paranoid"
fi

# ── Find PIDs ─────────────────────────────────────────────────────────────────
find_pid() {
    local pattern="$1"
    pgrep -f "$pattern" 2>/dev/null | head -1
}

OVCAM_PID=$(find_pid "ovcam_producer")
YOLO_PID=$(find_pid  "yolo_producer.py")
SLAM_PID=$(find_pid  "ov2slam_node")
VISP_PID=$(find_pid  "visp_servo_node")

log "Found PIDs:"
log "  ovcam_producer : ${OVCAM_PID:-NOT RUNNING}"
log "  yolo_producer  : ${YOLO_PID:-NOT RUNNING}"
log "  ov2slam_node   : ${SLAM_PID:-NOT RUNNING}"
log "  visp_servo_node: ${VISP_PID:-NOT RUNNING}"

# ── Build PID list based on condition ────────────────────────────────────────
PID_LIST=""
case "$CONDITION" in
    A)  # YOLO alone (ovcam + yolo)
        [[ -n "$YOLO_PID"  ]] && PID_LIST="$YOLO_PID"
        [[ -n "$OVCAM_PID" ]] && PID_LIST="$PID_LIST $OVCAM_PID"
        ;;
    B)  # SLAM alone (ovcam + slam)
        [[ -n "$SLAM_PID"  ]] && PID_LIST="$SLAM_PID"
        [[ -n "$OVCAM_PID" ]] && PID_LIST="$PID_LIST $OVCAM_PID"
        ;;
    C)  # YOLO + SLAM (no ViSP)
        [[ -n "$YOLO_PID"  ]] && PID_LIST="$YOLO_PID"
        [[ -n "$SLAM_PID"  ]] && PID_LIST="$PID_LIST $SLAM_PID"
        [[ -n "$OVCAM_PID" ]] && PID_LIST="$PID_LIST $OVCAM_PID"
        ;;
    D)  # Full pipeline
        [[ -n "$YOLO_PID"  ]] && PID_LIST="$YOLO_PID"
        [[ -n "$SLAM_PID"  ]] && PID_LIST="$PID_LIST $SLAM_PID"
        [[ -n "$VISP_PID"  ]] && PID_LIST="$PID_LIST $VISP_PID"
        [[ -n "$OVCAM_PID" ]] && PID_LIST="$PID_LIST $OVCAM_PID"
        ;;
    *)
        echo "Unknown condition '$CONDITION'. Use A, B, C, or D."
        exit 1 ;;
esac

PID_LIST=$(echo "$PID_LIST" | tr ' ' '\n' | sort -u | tr '\n' ' ')

if [[ -z "${PID_LIST// /}" ]]; then
    echo "ERROR: No matching processes found for condition $CONDITION."
    echo "       Make sure the pipeline is running: ./run_stack.sh"
    exit 1
fi

# Build perf -p flags
PERF_P_FLAGS=""
for pid in $PID_LIST; do
    PERF_P_FLAGS="$PERF_P_FLAGS -p $pid"
done

log "Profiling PIDs: $PID_LIST"
log "Duration: ${DURATION}s  →  $OUT_FILE"

# ── Run perf stat ─────────────────────────────────────────────────────────────
EVENTS="cache-misses,cache-references,L1-dcache-load-misses,L1-dcache-loads,\
LLC-load-misses,LLC-loads,LLC-store-misses,LLC-stores,\
context-switches,cpu-migrations,instructions,cycles"

# perf stat writes to stderr; capture both stdout and stderr
perf stat \
    -e "$EVENTS" \
    $PERF_P_FLAGS \
    --no-aggr \
    -o "$OUT_FILE" \
    sleep "$DURATION" 2>&1 | tee -a "$OUT_FILE"

log "perf output saved to: $OUT_FILE"

# ── Parse and append to summary CSV ──────────────────────────────────────────
if [[ ! -f "$SUMMARY_CSV" ]]; then
    echo "label,condition,duration_s,\
cache_miss_pct,LLC_load_miss_pct,LLC_store_miss_pct,\
ctx_switches,cpu_migrations,instructions_B,cycles_B,\
pid_list" > "$SUMMARY_CSV"
fi

# Extract key metrics from perf output using awk
parse_metric() {
    local metric="$1"
    grep -i "$metric" "$OUT_FILE" 2>/dev/null \
        | awk '{gsub(",","",$1); print $1}' | head -1 || echo "N/A"
}

CACHE_REFS=$(parse_metric "cache-references")
CACHE_MISS=$(parse_metric "cache-misses")
LLC_LOADS=$(parse_metric "LLC-loads")
LLC_LOAD_MISS=$(parse_metric "LLC-load-misses")
LLC_STORES=$(parse_metric "LLC-stores")
LLC_STORE_MISS=$(parse_metric "LLC-store-misses")
CTX_SW=$(parse_metric "context-switches")
CPU_MIG=$(parse_metric "cpu-migrations")
INSTRUCTIONS=$(parse_metric "instructions")
CYCLES=$(parse_metric "cycles")

# Compute percentages (bash integer arithmetic is insufficient; use awk)
CACHE_MISS_PCT=$(awk -v m="$CACHE_MISS" -v r="$CACHE_REFS" \
    'BEGIN{if(r+0>0) printf "%.2f", m*100/r; else print "N/A"}')
LLC_LOAD_PCT=$(awk -v m="$LLC_LOAD_MISS" -v r="$LLC_LOADS" \
    'BEGIN{if(r+0>0) printf "%.2f", m*100/r; else print "N/A"}')
LLC_STORE_PCT=$(awk -v m="$LLC_STORE_MISS" -v r="$LLC_STORES" \
    'BEGIN{if(r+0>0) printf "%.2f", m*100/r; else print "N/A"}')

PID_TAG=$(echo "$PID_LIST" | tr ' ' '+' | sed 's/^+//;s/+$//')
echo "$LABEL,$CONDITION,$DURATION,\
$CACHE_MISS_PCT,$LLC_LOAD_PCT,$LLC_STORE_PCT,\
$CTX_SW,$CPU_MIG,$INSTRUCTIONS,$CYCLES,\
$PID_TAG" >> "$SUMMARY_CSV"

log ""
log "─── Summary ────────────────────────────────────────────────"
log "  Label         : $LABEL"
log "  Cache miss    : ${CACHE_MISS_PCT}%  (cache-miss / cache-ref)"
log "  LLC load miss : ${LLC_LOAD_PCT}%"
log "  LLC store miss: ${LLC_STORE_PCT}%"
log "  Ctx switches  : $CTX_SW"
log "  CPU migrations: $CPU_MIG"
log "─────────────────────────────────────────────────────────────"
log "Appended to: $SUMMARY_CSV"
log ""
log "Suggested run order:"
log "  1. Start: ./run_stack.sh --no-slam  →  bash benchmarks/contention_bench.sh --condition A --label yolo_alone"
log "  2. Start: SLAM only (no YOLO shm)   →  bash benchmarks/contention_bench.sh --condition B --label slam_alone"
log "  3. Start: ./run_stack.sh            →  bash benchmarks/contention_bench.sh --condition C --label yolo_plus_slam"
log "  4. Full pipeline running            →  bash benchmarks/contention_bench.sh --condition D --label full_pipeline"
