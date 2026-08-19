#!/usr/bin/env bash
# run_coco_sweep.sh — COCO accuracy + latency sweep, 9 YOLO configs x 2 backends.
#
# Accuracy is the primary output and is pinning-independent. Latency is secondary:
# runs are DELIBERATELY UNPINNED so CPU numbers are comparable to the published
# standalone table (whose pinning is unrecorded). Do not add taskset without
# re-baselining, see Finding: single-core pinning throttles ONNX ~1.7x.
#
# Thresholds:
#   0.001  standard mAP, comparable to published tables
#   0.20   the deployed operating point (hil_servo_params min_confidence,
#          registry default, and the v8/v11 baked HEF score threshold)
#
# For v8/v11 on NPU the 0.001 run is the CENSORING EXPERIMENT: the HEF bakes a
# 0.200 on-device floor, so 0.001 and 0.20 must come out IDENTICAL. That equality
# is the measurement, not a bug.
set -uo pipefail

REPO="$(cd "$(dirname "$0")/.." && pwd)"
OUT="$REPO/benchmarks/results/accuracy/coco"
IMGS="$HOME/datasets/coco/val2017"
ANN="$HOME/datasets/coco/annotations/instances_val2017.json"
LOG="$OUT/sweep.log"
mkdir -p "$OUT"

run() {  # backend model conf limit
    local backend=$1 model=$2 conf=$3 limit=$4
    local tag="${model}_${backend}_conf${conf}"
    if ls "$OUT"/coco_accuracy_${tag}_*.csv >/dev/null 2>&1; then
        echo "[skip] $tag (already done)" | tee -a "$LOG"; return 0
    fi
    echo "" | tee -a "$LOG"
    echo "######## $tag  limit=${limit:-full}  $(date +%H:%M:%S) ########" | tee -a "$LOG"
    local args=(--backend "$backend" --model "$model" --repo-root "$REPO"
                --coco-images "$IMGS" --coco-ann "$ANN" --conf "$conf"
                --threads 2 --inter-threads 1 --warmup 5 --out-dir "$OUT")
    [ "$limit" -gt 0 ] && args+=(--limit "$limit")
    python3 -u "$REPO/benchmarks/coco_accuracy_bench.py" "${args[@]}" 2>&1 \
        | grep -viE --line-buffered "W:onnxruntime|GPU device discovery" | tee -a "$LOG" \
        | grep -iE --line-buffered "mAP50-95 |stop sign:|latency  :|images    :|!!|mode=" || true
    sleep 5   # let the Hailo VDevice fully release between HEF swaps
}

echo "===== COCO sweep started $(date) =====" | tee -a "$LOG"

# ---- NPU: all 9, both thresholds, FULL val2017 (5000) ----
for m in yolo26n yolo26s yolo26m yolov8n yolov8s yolov8m yolov11n yolov11s yolov11m; do
    for c in 0.001 0.20; do run npu "$m" "$c" 0; done
done

# ---- CPU nano: both thresholds, 1000-image subset ----
for m in yolo26n yolov8n yolov11n; do
    for c in 0.001 0.20; do run cpu "$m" "$c" 1000; done
done

# ---- CPU small + medium: deployed threshold only, 1000-image subset ----
for m in yolo26s yolov8s yolov11s yolo26m yolov8m yolov11m; do
    run cpu "$m" 0.20 1000
done

echo "" | tee -a "$LOG"
echo "===== COCO sweep finished $(date) =====" | tee -a "$LOG"
