#!/bin/bash
# sweep_detector_e2e.sh — detector-axis system sweep into a FRESH DATED folder.
#
# Same inner loop as run_hil_experiments.sh (full stack, swap only the detector),
# but every summary now also carries:
#   detector_latency_ms : detector-only stages S3-S7 + det_path (S3-S8)
#   mission             : reached_success_rate, time_to_reached_s,
#                         approach_duration_s, n_reacquire, n_lost
#
# Output: benchmarks/results/detector_sweep_<YYYY-MM-DD>/  (auto-dated, final data)
#
# Usage:
#   ./benchmarks/sweep_detector_e2e.sh [--working-only] [--only SUBSTR]
#       [--runs N] [--duration S] [--no-slam]
#
#   --working-only   OBSOLETE (kept for compatibility): skips the 6 v8/v11 NPU
#                    configs. They were once believed miscalibrated (old Finding 9),
#                    but the real bug was the fixed-stride NMS-by-class parser —
#                    fixed 2026-07-11 in yolo_producer.postprocess_ondevice_nms.
#                    All 20 configs produce real detections now; don't use this flag.
#   --only SUBSTR    further filter by label substring (chunking across MATLAB sessions)
#
# Start the MATLAB supervisor before each chunk; restart MATLAB between chunks.

set -u
cd "$(dirname "$0")/.."
WS="$(pwd)"

RUNS=3; DURATION=60; VLM_DURATION=180
OUTPUT_DIR="benchmarks/results/detector_sweep_$(date +%F)"
ONLY=""; SLAM_MODE="fast"; WORKING_ONLY=false

while [[ $# -gt 0 ]]; do
    case "$1" in
        --only)         ONLY="$2"; shift 2 ;;
        --runs)         RUNS="$2"; shift 2 ;;
        --duration)     DURATION="$2"; shift 2 ;;
        --output-dir)   OUTPUT_DIR="$2"; shift 2 ;;
        --no-slam)      SLAM_MODE="no-slam"; shift ;;
        --working-only) WORKING_ONLY=true; shift ;;
        *) echo "Unknown arg: $1" >&2; exit 1 ;;
    esac
done

mkdir -p "$OUTPUT_DIR"

# label  detector  model  backend  duration
CONFIGS=()
for M in yolov8n yolov8s yolov8m yolov11n yolov11s yolov11m yolo26n yolo26s yolo26m; do
    CONFIGS+=("${M}_npu yolo $M npu $DURATION")
done
for M in yolov8n yolov8s yolov8m yolov11n yolov11s yolov11m yolo26n yolo26s yolo26m; do
    CONFIGS+=("${M}_cpu yolo $M cpu $DURATION")
done
CONFIGS+=("opencv_mog2 opencv - cpu $DURATION")
CONFIGS+=("vlm_owlvit vlm - cpu $VLM_DURATION")

# the 6 NPU configs known broken until the HEFs are recompiled (Finding 9)
BROKEN_NPU="yolov8n_npu yolov8s_npu yolov8m_npu yolov11n_npu yolov11s_npu yolov11m_npu"
is_broken() { [[ " $BROKEN_NPU " == *" $1 "* ]]; }

SELECTED=(); TOTAL_S=0
for C in "${CONFIGS[@]}"; do
    read -r LABEL _ _ _ DUR <<< "$C"
    [[ -n "$ONLY" && "$LABEL" != *"$ONLY"* ]] && continue
    if [[ "$WORKING_ONLY" == "true" ]] && is_broken "$LABEL"; then continue; fi
    SELECTED+=("$C")
    TOTAL_S=$((TOTAL_S + RUNS * (45 + 35 + DUR + 20)))
done
[[ ${#SELECTED[@]} -gt 0 ]] || { echo "no configs match" >&2; exit 1; }

# manifest — provenance for the final dataset
{
    echo "detector sweep — $(date -Is)"
    echo "git: $(git rev-parse HEAD 2>/dev/null)"
    echo "slam_mode=$SLAM_MODE runs=$RUNS duration=$DURATION working_only=$WORKING_ONLY only='$ONLY'"
    echo "camera intrinsics (Simulink): fx=fy=1200 cx=320 cy=240 640x480"
    echo "conf threshold: 0.20 (producer + visp min_confidence)"
    echo "NOTE: v8/v11 NPU HEFs miscalibrated (person-only) until recompiled — Finding 9"
    echo "configs this run:"
    for C in "${SELECTED[@]}"; do read -r L _ <<< "$C"; echo "  - $L"; done
} >> "${OUTPUT_DIR}/MANIFEST.txt"

echo "[sweep] ${#SELECTED[@]} configs × ${RUNS} runs → ${OUTPUT_DIR}"
echo "[sweep] estimated: ~$((TOTAL_S / 60)) min"

FAILED=()
for C in "${SELECTED[@]}"; do
    read -r LABEL DETECTOR MODEL BACKEND DUR <<< "$C"
    echo ""
    echo "[sweep] ━━━ ${LABEL} (detector=${DETECTOR} model=${MODEL} backend=${BACKEND}) ━━━"
    ARGS=(--detector "$DETECTOR" --backend "$BACKEND" --slam-mode "$SLAM_MODE"
          --conf 0.20 --duration "$DUR" --runs "$RUNS"
          --label "${LABEL}_${SLAM_MODE}" --output-dir "$OUTPUT_DIR")
    [[ "$MODEL" != "-" ]] && ARGS+=(--model "$MODEL")
    if ! ./benchmarks/run_hil_experiments.sh "${ARGS[@]}"; then
        FAILED+=("$LABEL")
        echo "[sweep] CONFIG FAILED: $LABEL — continuing"
    fi
done

echo ""
echo "[sweep] complete → ${OUTPUT_DIR}"
ls -1 "${OUTPUT_DIR}"/*_summary.json 2>/dev/null
[[ ${#FAILED[@]} -eq 0 ]] || { echo "[sweep] FAILED: ${FAILED[*]}" >&2; exit 1; }
