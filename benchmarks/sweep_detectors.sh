#!/bin/bash
# sweep_detectors.sh — full detector-axis sweep through the HIL loop.
#
# Fixed: --slam-mode fast, --controller proportional, 3 runs per config.
# Sweeps (20 configs):
#   9 YOLO models (yolov8/yolov11/yolo26 × n/s/m) on the NPU (HEF)
#   9 YOLO models on the CPU (onnxruntime)
#   1 OpenCV MOG2 background-subtraction detector
#   1 OWL-ViT open-vocabulary VLM detector (longer window — very low rate)
#
# Usage:
#   ./benchmarks/sweep_detectors.sh [--only <substr>] [--runs N] [--duration S]
#       [--output-dir DIR] [--no-slam]
#
#   --only yolo26      run only configs whose label contains the substring
#   --no-slam          run the whole sweep without OV2SLAM background load
#
# Requires the MATLAB supervisor (benchmarks/matlab/hil_run_supervisor.m)
# listening on ${MATLAB_HOST_IP}:55556, or you will be prompted to restart
# Simulink between runs.

set -u
cd "$(dirname "$0")/.."

RUNS=3; DURATION=60; VLM_DURATION=180
OUTPUT_DIR="benchmarks/results/hil"
ONLY=""; SLAM_MODE="fast"

while [[ $# -gt 0 ]]; do
    case "$1" in
        --only)       ONLY="$2"; shift 2 ;;
        --runs)       RUNS="$2"; shift 2 ;;
        --duration)   DURATION="$2"; shift 2 ;;
        --output-dir) OUTPUT_DIR="$2"; shift 2 ;;
        --no-slam)    SLAM_MODE="no-slam"; shift ;;
        *) echo "Unknown arg: $1" >&2; exit 1 ;;
    esac
done

# config tuples: label  detector  model  backend  duration
CONFIGS=()
for M in yolov8n yolov8s yolov8m yolov11n yolov11s yolov11m yolo26n yolo26s yolo26m; do
    CONFIGS+=("${M}_npu yolo $M npu $DURATION")
done
for M in yolov8n yolov8s yolov8m yolov11n yolov11s yolov11m yolo26n yolo26s yolo26m; do
    CONFIGS+=("${M}_cpu yolo $M cpu $DURATION")
done
CONFIGS+=("opencv_mog2 opencv - cpu $DURATION")
CONFIGS+=("vlm_owlvit vlm - cpu $VLM_DURATION")

# filter + ETA
SELECTED=()
TOTAL_S=0
for C in "${CONFIGS[@]}"; do
    read -r LABEL _ _ _ DUR <<< "$C"
    [[ -n "$ONLY" && "$LABEL" != *"$ONLY"* ]] && continue
    SELECTED+=("$C")
    TOTAL_S=$((TOTAL_S + RUNS * (45 + 35 + DUR + 15)))   # frames-wait + warmup + data + teardown
done
[[ ${#SELECTED[@]} -gt 0 ]] || { echo "no configs match --only '$ONLY'" >&2; exit 1; }

echo "[sweep] ${#SELECTED[@]} configs × ${RUNS} runs, slam=${SLAM_MODE}"
echo "[sweep] estimated time: ~$((TOTAL_S / 60)) min"
echo "[sweep] output: ${OUTPUT_DIR}"

FAILED=()
for C in "${SELECTED[@]}"; do
    read -r LABEL DETECTOR MODEL BACKEND DUR <<< "$C"
    echo ""
    echo "[sweep] ━━━ ${LABEL} (detector=${DETECTOR} model=${MODEL} backend=${BACKEND}) ━━━"
    ARGS=(--detector "$DETECTOR" --backend "$BACKEND" --slam-mode "$SLAM_MODE"
          --duration "$DUR" --runs "$RUNS"
          --label "${LABEL}_${SLAM_MODE}" --output-dir "$OUTPUT_DIR")
    [[ "$MODEL" != "-" ]] && ARGS+=(--model "$MODEL")
    if ! ./benchmarks/run_hil_experiments.sh "${ARGS[@]}"; then
        FAILED+=("$LABEL")
        echo "[sweep] CONFIG FAILED: $LABEL — continuing"
    fi
done

echo ""
echo "[sweep] complete. summaries:"
ls -1 "${OUTPUT_DIR}"/*_summary.json 2>/dev/null
[[ ${#FAILED[@]} -eq 0 ]] || { echo "[sweep] FAILED configs: ${FAILED[*]}" >&2; exit 1; }
