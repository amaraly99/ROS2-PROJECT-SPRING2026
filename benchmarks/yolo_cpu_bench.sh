#!/bin/bash
# ─── YOLO CPU Benchmark — All Model Sizes ───────────────────────────────────
#
# Benchmarks YOLO26 n/s/m/l/xl running on ARM CPU (Ultralytics PyTorch)
# and compares against the NPU baseline.
#
# For each model size:
#   - Runs yolo_cpu_producer.py in benchmark mode (no SHM write)
#   - Records fps, inference latency (mean, P50, P95) to CSV
#   - Also collects CPU/RAM usage via pidstat
#
# Usage:
#   bash benchmarks/yolo_cpu_bench.sh                  # all sizes
#   bash benchmarks/yolo_cpu_bench.sh --sizes "n s"    # subset
#   bash benchmarks/yolo_cpu_bench.sh --frames 100     # fewer frames
#
# Prerequisites:
#   - ovcam_producer must be running (camera SHM active)
#   - pip install ultralytics torch torchvision
#   - YOLO26 .pt model files in PROJECT_DIR/models/
#     Download: python3 -c "from ultralytics import YOLO; YOLO('yolo26n.pt')"
#     (Ultralytics auto-downloads on first use)
#
# Output:
#   benchmarks/results/yolo_cpu/yolo26<size>_cpu_timings.csv   — per-frame timing
#   benchmarks/results/yolo_cpu/summary.csv                    — all sizes comparison
# ─────────────────────────────────────────────────────────────────────────────

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
RESULTS_DIR="$SCRIPT_DIR/results/yolo_cpu"
PRODUCER="$PROJECT_DIR/src/yolo_producer/yolo_cpu_producer.py"
MODELS_DIR="$PROJECT_DIR/models"

mkdir -p "$RESULTS_DIR"

# ── Defaults ─────────────────────────────────────────────────────────────────
SIZES="n s m l"   # xl excluded by default (likely too slow on RPi5 CPU)
FRAMES=200
IMG_SIZE=640
CONF=0.25

while [[ $# -gt 0 ]]; do
    case $1 in
        --sizes)    SIZES="$2"; shift 2 ;;
        --frames)   FRAMES="$2"; shift 2 ;;
        --img-size) IMG_SIZE="$2"; shift 2 ;;
        --conf)     CONF="$2"; shift 2 ;;
        *) echo "Unknown arg: $1"; exit 1 ;;
    esac
done

# ── Ensure ovcam SHM is available ────────────────────────────────────────────
if [[ ! -e /dev/shm/ovcam_frames ]]; then
    echo "ERROR: /dev/shm/ovcam_frames not found."
    echo "       Start ovcam_producer first: ./run_stack.sh --no-slam"
    echo "       (You can stop YOLO/SLAM — only ovcam_producer is needed)"
    exit 1
fi

# ── Summary CSV header ────────────────────────────────────────────────────────
SUMMARY="$RESULTS_DIR/summary.csv"
if [[ ! -f "$SUMMARY" ]]; then
    echo "model,device,frames,avg_fps,mean_infer_ms,mean_total_ms,\
peak_rss_mb,cpu_pct_avg,npu_baseline_ms,speedup_vs_npu" > "$SUMMARY"
fi

NPU_BASELINE_FPS=32.1
NPU_BASELINE_INFER_MS=25.1

log() { echo "[yolo_cpu_bench] $*"; }

# ── Run one size ──────────────────────────────────────────────────────────────
run_size() {
    local SIZE="$1"
    local MODEL_NAME="yolo26${SIZE}.pt"
    local MODEL_PATH="$MODELS_DIR/$MODEL_NAME"
    local CSV_OUT="$RESULTS_DIR/yolo26${SIZE}_cpu_timings.csv"
    local LOG_OUT="$RESULTS_DIR/yolo26${SIZE}_cpu_run.log"

    log "──────────────────────────────────────────────────────"
    log "Model: $MODEL_NAME  (frames=$FRAMES, imgsz=$IMG_SIZE)"

    # Download model if not present (ultralytics auto-download on first predict)
    # We trigger it via a quick Python check:
    if [[ ! -f "$MODEL_PATH" ]]; then
        log "Model not found at $MODEL_PATH — triggering Ultralytics auto-download..."
        python3 -c "from ultralytics import YOLO; YOLO('yolo26${SIZE}.pt')" \
            && cp ~/ultralytics/models/yolo26${SIZE}.pt "$MODEL_PATH" 2>/dev/null || true
        # If auto-download puts it in cwd or ~/.cache, find it
        CANDIDATE=$(find ~ -name "$MODEL_NAME" -newer "$RESULTS_DIR" 2>/dev/null | head -1 || true)
        if [[ -n "$CANDIDATE" ]] && [[ ! -f "$MODEL_PATH" ]]; then
            cp "$CANDIDATE" "$MODEL_PATH"
        fi
        # If still not found, just let yolo_cpu_producer use Ultralytics default cache
        if [[ ! -f "$MODEL_PATH" ]]; then
            MODEL_PATH="yolo26${SIZE}.pt"  # Let Ultralytics download to default cache
        fi
    fi

    # Start pidstat in background to capture CPU/RAM usage
    PIDSTAT_LOG="$RESULTS_DIR/yolo26${SIZE}_pidstat.log"
    pidstat -r -u -h 2 > "$PIDSTAT_LOG" &
    PIDSTAT_PID=$!

    # Run benchmark
    python3 "$PRODUCER" \
        --model "$MODEL_PATH" \
        --benchmark \
        --frames "$FRAMES" \
        --img-size "$IMG_SIZE" \
        --conf "$CONF" \
        --csv "$CSV_OUT" \
        2>&1 | tee "$LOG_OUT"

    # Stop pidstat
    kill "$PIDSTAT_PID" 2>/dev/null || true

    # Parse results from log
    AVG_FPS=$(grep "Avg FPS" "$LOG_OUT" | awk '{print $NF}' | head -1 || echo "N/A")
    MEAN_INFER=$(grep "Inference" "$LOG_OUT" | awk '{print $3}' | head -1 || echo "N/A")
    MEAN_TOTAL=$(grep "Total E2E" "$LOG_OUT" | awk '{print $3}' | head -1 || echo "N/A")

    # Parse peak RSS from pidstat (RSS in kB → MB)
    PEAK_RSS=$(awk 'NR>3 && /python/{rss=$8; if(rss>max) max=rss} END{printf "%.1f", max/1024}' \
               "$PIDSTAT_LOG" 2>/dev/null || echo "N/A")
    CPU_AVG=$(awk 'NR>3 && /python/{s+=$7; n++} END{if(n>0) printf "%.1f", s/n; else print "N/A"}' \
              "$PIDSTAT_LOG" 2>/dev/null || echo "N/A")

    # Compute speedup vs NPU
    SPEEDUP=$(awk -v infer="$MEAN_INFER" -v npu="$NPU_BASELINE_INFER_MS" \
        'BEGIN{if(infer+0>0 && npu+0>0) printf "%.2f", npu/infer; else print "N/A"}' \
        2>/dev/null || echo "N/A")

    echo "yolo26${SIZE},cpu,$FRAMES,$AVG_FPS,$MEAN_INFER,$MEAN_TOTAL,\
$PEAK_RSS,$CPU_AVG,$NPU_BASELINE_INFER_MS,$SPEEDUP" >> "$SUMMARY"

    log "Done: fps=$AVG_FPS  infer=${MEAN_INFER}ms  total=${MEAN_TOTAL}ms"
    log "      RSS=${PEAK_RSS}MB  CPU=${CPU_AVG}%"
    log "      vs NPU (${NPU_BASELINE_INFER_MS}ms): speedup=$SPEEDUP"
}

# ── Run all sizes ─────────────────────────────────────────────────────────────
log "Starting YOLO CPU benchmark — sizes: [$SIZES]"
log "Results → $RESULTS_DIR"
log ""

for SIZE in $SIZES; do
    run_size "$SIZE"
done

log ""
log "═══════════════════════════════════════════════════════"
log "All sizes complete. Summary:"
column -t -s',' "$SUMMARY"
log ""
log "NPU baseline: ${NPU_BASELINE_FPS} fps / ${NPU_BASELINE_INFER_MS} ms inference"
log ""
log "Next step: run E2E latency probe for each config:"
log "  Start yolo_cpu_producer --model yolo26n.pt (no --benchmark)"
log "  Then: python3 benchmarks/e2e_latency_probe.py --config yolo_cpu_n --duration 60"
