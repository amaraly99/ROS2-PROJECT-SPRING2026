#!/usr/bin/env bash
# run_coco_sweep.sh — COCO accuracy + latency sweep, 9 YOLO configs x 2 backends.
#
#   usage: run_coco_sweep.sh {controlled|deployed}
#
# PROFILES — why there are two
# ----------------------------
# The v1 sweep (2026-08-19, archived under paper_data/accuracy/coco_v1_superseded)
# ran every config at whatever postprocess its own code path happened to use.
# Those paths were never reconciled: YOLO26/NPU got host NMS at IoU 0.45 while
# the v8/v11 HEFs ran on-device NMS at a baked 0.70, the 64-detection cap applied
# to the NPU paths but not the CPU ones, and only the NPU paths truncated box
# coordinates with int(). Every one of those biases lands on YOLO26/NPU, which is
# exactly the cell where v1 reported YOLO26 losing. See that directory's README.
#
#   controlled  IoU 0.70, max_det 300, float coords.
#               One postprocess for all nine models on both backends. This is
#               the arm that answers "which detector is more accurate" and the
#               only arm comparable to published mAP tables (Ultralytics' own
#               numbers also use NMS IoU 0.7 and COCO evaluates at maxDets=100).
#
#   deployed    IoU 0.45, max_det 64, int coords.
#               The robot's actual postprocess, applied EQUALLY to all nine
#               models rather than to some of them. Answers "what does the robot
#               really achieve". Not comparable to published tables.
#
# The gap between the two profiles is itself a result: it is the paper's thesis
# -- component benchmarks mispredicting deployed behaviour -- appearing in the
# postprocess configuration rather than in throughput.
#
# THRESHOLDS
#   0.001  standard mAP, comparable to published tables
#   0.20   the deployed operating point (hil_servo_params min_confidence,
#          registry default, and the v8/v11 baked HEF score threshold)
#
# For v8/v11 on NPU the 0.001 run is the CENSORING EXPERIMENT: the HEF bakes a
# 0.200 on-device floor, so 0.001 and 0.20 must come out IDENTICAL. That equality
# is the measurement, not a bug. It is an equality between two runs through the
# same path, so it survived the v1 defect and must survive the correction too --
# if a profile change breaks it, the change is wrong.
#
# IMAGE BUDGET
#   NPU is fast enough for the full 5000. CPU medium is ~1.4 s/img, so CPU uses a
#   1000-image subset. Because that mismatch confounds every NPU-vs-CPU claim,
#   the controlled profile ALSO runs all nine NPU configs at 1000 images, giving
#   a matched-subset comparison. Always report the budget.
#
# Runs are DELIBERATELY UNPINNED so CPU latency stays comparable to the published
# standalone table, whose pinning is unrecorded. Do not add taskset without
# re-baselining, see Finding: single-core pinning throttles ONNX ~1.7x.
set -uo pipefail

PROFILE="${1:-}"
case "$PROFILE" in
    controlled|deployed) ;;
    *) echo "usage: $0 {controlled|deployed}" >&2; exit 2 ;;
esac

REPO="$(cd "$(dirname "$0")/.." && pwd)"
OUT="$REPO/benchmarks/results/accuracy/coco/v2_${PROFILE}"
IMGS="$HOME/datasets/coco/val2017"
ANN="$HOME/datasets/coco/annotations/instances_val2017.json"
LOG="$OUT/sweep.log"
mkdir -p "$OUT"

# Has this exact config already produced a CSV?
#
# The bench names files coco_accuracy_<model>_<backend>_conf<C>_<profile>[_n<N>]
# _<host>_<stamp>.csv, where <C> is Python's float repr ("0.20" -> "0.2"). An
# earlier version of this check globbed conf* and ended the tag with _*, so
# conf0.001 satisfied the conf0.20 check and an _n1000 subset run satisfied a
# full-val2017 check. It skipped 13 of 40 configs and still printed "finished".
# Match the threshold exactly, and test the _n<N> segment explicitly.
already_done() {  # backend model conf_tag limit
    local backend=$1 model=$2 conf_tag=$3 limit=$4
    local base="coco_accuracy_${model}_${backend}_conf${conf_tag}_${PROFILE}"
    local f bn rest
    for f in "$OUT/${base}"_*.csv; do
        [ -e "$f" ] || continue
        bn=$(basename "$f"); rest=${bn#"${base}_"}
        if [ "$limit" -gt 0 ]; then
            [[ "$rest" == n${limit}_* ]] && return 0
        else
            [[ "$rest" == n[0-9]* ]] || return 0
        fi
    done
    return 1
}

run() {  # backend model conf limit
    local backend=$1 model=$2 conf=$3 limit=$4
    # Normalise exactly as argparse does, so the check matches the filename.
    local conf_tag; conf_tag=$(python3 -c "print(float('$conf'))")
    if already_done "$backend" "$model" "$conf_tag" "$limit"; then
        echo "[skip] ${model}/${backend}/conf${conf_tag}/${PROFILE}/${limit:-full} (already done)" | tee -a "$LOG"
        return 0
    fi
    echo "" | tee -a "$LOG"
    echo "######## ${model} ${backend} conf${conf} ${PROFILE} limit=${limit:-full}  $(date +%H:%M:%S) ########" | tee -a "$LOG"
    local args=(--backend "$backend" --model "$model" --repo-root "$REPO"
                --coco-images "$IMGS" --coco-ann "$ANN" --conf "$conf"
                --profile "$PROFILE"
                --threads 2 --inter-threads 1 --warmup 5 --out-dir "$OUT")
    [ "$limit" -gt 0 ] && args+=(--limit "$limit")
    python3 -u "$REPO/benchmarks/coco_accuracy_bench.py" "${args[@]}" 2>&1 \
        | grep -viE --line-buffered "W:onnxruntime|GPU device discovery" | tee -a "$LOG" \
        | grep -iE --line-buffered "mAP50-95 |stop sign:|latency  :|images    :|!!|mode=|profile=" || true
    sleep 5   # let the Hailo VDevice fully release between HEF swaps
}

ALL="yolo26n yolo26s yolo26m yolov8n yolov8s yolov8m yolov11n yolov11s yolov11m"
NANO="yolo26n yolov8n yolov11n"
BIG="yolo26s yolov8s yolov11s yolo26m yolov8m yolov11m"

echo "===== COCO sweep v2 [$PROFILE] started $(date) =====" | tee -a "$LOG"

# Cheapest and most informative first, so the YOLO26 question is answered early
# and the multi-hour CPU configs trail.

if [ "$PROFILE" = "controlled" ]; then
    # 1. NPU, both thresholds, full val2017 — the headline accuracy table.
    for m in $ALL; do for c in 0.001 0.20; do run npu "$m" "$c" 0; done; done

    # 2. NPU at 1000 images — matched subset for the NPU-vs-CPU comparison.
    for m in $ALL; do run npu "$m" 0.20 1000; done

    # 3. CPU nano, both thresholds.
    for m in $NANO; do for c in 0.001 0.20; do run cpu "$m" "$c" 1000; done; done

    # 4. EXTERNAL VALIDATION ANCHOR. yolo26n CPU FP32 at standard mAP over the
    #    full val2017 should land near Ultralytics' published ~0.40. If it does,
    #    the harness is validated against a reference outside this project; if it
    #    does not, nothing else here should be trusted.
    run cpu yolo26n 0.001 0

    # 5. CPU small + medium at the deployed threshold — the long pole (~1.6 h).
    for m in $BIG; do run cpu "$m" 0.20 1000; done
else
    # deployed: the operating point only. Same postprocess for all nine models.
    for m in $ALL; do run npu "$m" 0.20 0; done
    for m in $ALL; do run cpu "$m" 0.20 1000; done
fi

echo "" | tee -a "$LOG"
echo "===== COCO sweep v2 [$PROFILE] finished $(date) =====" | tee -a "$LOG"
