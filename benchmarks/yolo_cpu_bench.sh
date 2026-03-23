#!/bin/bash
# ─── YOLO CPU/ONNX Benchmark — All Model Sizes ──────────────────────────────
#
# Benchmarks YOLO26 n/s/m/l on ARM CPU using two backends:
#   1. PyTorch FP32  — default Ultralytics .pt model
#   2. ONNX Runtime  — exported .onnx, uses ARM-optimised ORT kernels (~2.5× faster)
#
# Compares both against the Hailo NPU baseline (yolo26n INT8 = 25.1 ms / 32 fps).
#
# Camera SHM is NOT required: script auto-detects ovcam and falls back to
# --synthetic (random frames of camera dimensions) when the camera is not running.
#
# Usage:
#   bash benchmarks/yolo_cpu_bench.sh                      # n s m l, PyTorch only
#   bash benchmarks/yolo_cpu_bench.sh --onnx               # also benchmark ONNX
#   bash benchmarks/yolo_cpu_bench.sh --sizes "n s" --onnx # subset + ONNX
#   bash benchmarks/yolo_cpu_bench.sh --frames 50          # fewer frames (faster)
#
# Prerequisites:
#   pip install ultralytics torch onnxruntime onnx
#   YOLO26 .pt weights in PROJECT_DIR/models/
#     (auto-downloaded on first run)
#
# Output:
#   benchmarks/results/yolo_cpu/yolo26<size>_cpu_timings.csv  — per-frame timing
#   benchmarks/results/yolo_cpu/yolo26<size>_onnx_timings.csv — ONNX per-frame
#   benchmarks/results/yolo_cpu/summary.csv                   — full comparison
# ─────────────────────────────────────────────────────────────────────────────

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
RESULTS_DIR="$SCRIPT_DIR/results/yolo_cpu"
PRODUCER="$PROJECT_DIR/src/yolo_producer/yolo_cpu_producer.py"
MODELS_DIR="$PROJECT_DIR/models"

mkdir -p "$RESULTS_DIR"

# ── Logging ───────────────────────────────────────────────────────────────────
log() { echo "[yolo_cpu_bench] $*"; }

# ── Defaults ─────────────────────────────────────────────────────────────────
SIZES="n s m l"
FRAMES=100          # 100 frames by default (200 for final paper runs)
IMG_SIZE=640
CONF=0.25
BENCH_ONNX=0        # set via --onnx flag

while [[ $# -gt 0 ]]; do
    case $1 in
        --sizes)    SIZES="$2"; shift 2 ;;
        --frames)   FRAMES="$2"; shift 2 ;;
        --img-size) IMG_SIZE="$2"; shift 2 ;;
        --conf)     CONF="$2"; shift 2 ;;
        --onnx)     BENCH_ONNX=1; shift ;;
        *) echo "Unknown arg: $1"; exit 1 ;;
    esac
done

# ── Camera SHM: prefer live frames, fall back to synthetic ───────────────────
USE_SYNTHETIC=0
if [[ ! -e /dev/shm/ovcam_frames ]]; then
    log "ovcam_frames SHM not found — using --synthetic (random frames, no camera needed)"
    USE_SYNTHETIC=1
else
    log "ovcam_frames SHM found — reading live camera frames"
fi

# ── NPU baseline (measured: Session 3) ───────────────────────────────────────
NPU_BASELINE_FPS=32.1
NPU_BASELINE_INFER_MS=25.1

# ── Summary CSV ───────────────────────────────────────────────────────────────
SUMMARY="$RESULTS_DIR/summary.csv"
if [[ ! -f "$SUMMARY" ]]; then
    echo "model,backend,frames,avg_fps,mean_infer_ms,mean_total_ms,"\
"peak_rss_mb,cpu_pct_avg,npu_baseline_ms,slowdown_vs_npu" > "$SUMMARY"
fi

# ── Helper: ensure .pt model is present ──────────────────────────────────────
ensure_pt_model() {
    local SIZE="$1"
    local MODEL_NAME="yolo26${SIZE}.pt"
    local MODEL_PATH="$MODELS_DIR/$MODEL_NAME"

    if [[ ! -f "$MODEL_PATH" ]]; then
        log "Downloading $MODEL_NAME via Ultralytics..."
        python3 -c "
from ultralytics import YOLO
import shutil, glob, os
m = YOLO('yolo26${SIZE}.pt')
candidates = glob.glob(os.path.expanduser('~/.cache/ultralytics/**/$MODEL_NAME'),
                       recursive=True)
if candidates:
    shutil.copy(candidates[0], '$MODEL_PATH')
    print('Saved to $MODEL_PATH')
else:
    print('WARNING: could not locate downloaded model')
" 2>&1 | grep -E "Saved|WARNING|Error"
    fi

    if [[ ! -f "$MODEL_PATH" ]]; then
        log "ERROR: $MODEL_PATH not found. Skipping size $SIZE."
        return 1
    fi
    return 0
}

# ── Helper: export .onnx if not already present ───────────────────────────────
ensure_onnx_model() {
    local SIZE="$1"
    local ONNX_PATH="$MODELS_DIR/yolo26${SIZE}.onnx"
    local PT_PATH="$MODELS_DIR/yolo26${SIZE}.pt"

    if [[ ! -f "$ONNX_PATH" ]]; then
        log "Exporting yolo26${SIZE}.onnx from ${PT_PATH}..."
        python3 -c "
from ultralytics import YOLO
import os
m = YOLO('$PT_PATH')
path = m.export(format='onnx', imgsz=$IMG_SIZE, simplify=True)
print(f'Exported: {path}  ({os.path.getsize(path)//1024}KB)')
" 2>&1 | grep -E "Exported|Error"
    fi

    if [[ ! -f "$ONNX_PATH" ]]; then
        log "ERROR: ONNX export failed for size $SIZE. Skipping ONNX benchmark."
        return 1
    fi
    return 0
}

# ── Helper: start pidstat monitoring ─────────────────────────────────────────
start_pidstat() {
    local LOG="$1"
    if command -v pidstat &>/dev/null; then
        pidstat -r -u -h 2 > "$LOG" &
        echo $!
    else
        echo "pidstat_unavailable" > "$LOG"
        echo ""
    fi
}

# ── Helper: parse pidstat output ─────────────────────────────────────────────
parse_pidstat() {
    local LOG="$1"
    local PEAK_RSS CPU_AVG
    PEAK_RSS=$(awk 'NR>3 && /python/{rss=$8; if(rss+0>max+0) max=rss} END{
        if(max+0>0) printf "%.1f", max/1024; else print "N/A"}' "$LOG" 2>/dev/null || echo "N/A")
    CPU_AVG=$(awk 'NR>3 && /python/{s+=$7; n++} END{
        if(n>0) printf "%.1f", s/n; else print "N/A"}' "$LOG" 2>/dev/null || echo "N/A")
    echo "$PEAK_RSS $CPU_AVG"
}

# ── Run benchmark for one model + backend ─────────────────────────────────────
run_benchmark() {
    local MODEL_PATH="$1"
    local BACKEND_LABEL="$2"   # e.g. "PyTorch-CPU-FP32" or "ONNX-Runtime-CPU"
    local SIZE="$3"
    local CSV_SUFFIX="$4"      # "cpu" or "onnx"

    local CSV_OUT="$RESULTS_DIR/yolo26${SIZE}_${CSV_SUFFIX}_timings.csv"
    local LOG_OUT="$RESULTS_DIR/yolo26${SIZE}_${CSV_SUFFIX}_run.log"
    local PIDSTAT_LOG="$RESULTS_DIR/yolo26${SIZE}_${CSV_SUFFIX}_pidstat.log"

    log "  Backend: $BACKEND_LABEL  frames=$FRAMES  imgsz=$IMG_SIZE"

    PIDSTAT_PID=$(start_pidstat "$PIDSTAT_LOG")

    PRODUCER_ARGS=(
        --model "$MODEL_PATH"
        --benchmark
        --frames "$FRAMES"
        --img-size "$IMG_SIZE"
        --conf "$CONF"
        --csv "$CSV_OUT"
    )
    [[ "$USE_SYNTHETIC" -eq 1 ]] && PRODUCER_ARGS+=(--synthetic)

    python3 "$PRODUCER" "${PRODUCER_ARGS[@]}" 2>&1 | tee "$LOG_OUT"

    [[ -n "$PIDSTAT_PID" ]] && kill "$PIDSTAT_PID" 2>/dev/null || true

    # Parse summary from log
    # Log format: "  Avg FPS      : 2.2"  /  "  Inference    : 448.5 ms/frame"
    local AVG_FPS MEAN_INFER MEAN_TOTAL
    AVG_FPS=$(grep    "Avg FPS"   "$LOG_OUT" | awk '{print $NF}' | head -1 || echo "N/A")
    MEAN_INFER=$(grep "Inference" "$LOG_OUT" | awk '{print $4}'  | head -1 || echo "N/A")
    MEAN_TOTAL=$(grep "Total E2E" "$LOG_OUT" | awk '{print $4}'  | head -1 || echo "N/A")

    read PEAK_RSS CPU_AVG <<< "$(parse_pidstat "$PIDSTAT_LOG")"

    local SLOWDOWN
    SLOWDOWN=$(awk -v infer="$MEAN_INFER" -v npu="$NPU_BASELINE_INFER_MS" \
        'BEGIN{if(infer+0>0 && npu+0>0) printf "%.1fx", infer/npu; else print "N/A"}')

    echo "yolo26${SIZE},$BACKEND_LABEL,$FRAMES,$AVG_FPS,$MEAN_INFER,$MEAN_TOTAL,"\
"$PEAK_RSS,$CPU_AVG,$NPU_BASELINE_INFER_MS,$SLOWDOWN" >> "$SUMMARY"

    log "  Done: fps=$AVG_FPS  infer=${MEAN_INFER}ms  total=${MEAN_TOTAL}ms  slowdown=${SLOWDOWN}"
    log "  RSS=${PEAK_RSS}MB  CPU=${CPU_AVG}%"
}

# ── Main loop ─────────────────────────────────────────────────────────────────
log "YOLO CPU/ONNX Benchmark — sizes: [$SIZES]  ONNX: $([[ $BENCH_ONNX -eq 1 ]] && echo yes || echo no)"
log "Results → $RESULTS_DIR"
log ""

for SIZE in $SIZES; do
    log "══════════════════════════════════════════════════════"
    log "Model: yolo26${SIZE}  (frames=$FRAMES, imgsz=$IMG_SIZE)"

    ensure_pt_model "$SIZE" || continue

    run_benchmark "$MODELS_DIR/yolo26${SIZE}.pt" "PyTorch-CPU-FP32" "$SIZE" "cpu"

    if [[ "$BENCH_ONNX" -eq 1 ]]; then
        ensure_onnx_model "$SIZE" && \
            run_benchmark "$MODELS_DIR/yolo26${SIZE}.onnx" "ONNX-Runtime-CPU" "$SIZE" "onnx"
    fi
    log ""
done

log ""
log "═══════════════════════════════════════════════════════"
log "Complete. Full comparison:"
log ""
column -t -s',' "$SUMMARY"
log ""
log "NPU baseline: ${NPU_BASELINE_FPS} fps  /  ${NPU_BASELINE_INFER_MS} ms inference"
log ""
log "Next steps:"
log "  1. Run E2E latency with each config:"
log "     python3 src/yolo_producer/yolo_cpu_producer.py --model models/yolo26n.pt"
log "     python3 benchmarks/e2e_latency_probe.py --config yolo_cpu_n --duration 60"
log "  2. Run resource contention profiling:"
log "     bash benchmarks/contention_bench.sh --condition D --label full_pipeline"
