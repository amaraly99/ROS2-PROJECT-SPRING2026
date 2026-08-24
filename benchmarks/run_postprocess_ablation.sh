#!/usr/bin/env bash
# run_postprocess_ablation.sh — attribute the v1 YOLO26/NPU accuracy deficit to
# specific postprocess artifacts, before spending hours on a corrected sweep.
#
# WHY
# ---
# The v1 COCO sweep reported YOLO26 losing to v8/v11 on the NPU. But the four
# backend x family postprocess paths were never controlled against each other,
# and three separate biases all land on YOLO26/NPU:
#
#   NMS IoU        YOLO26/NPU ran host NMS at 0.45; the v8/v11 HEFs bake 0.70.
#   Detection cap  Both NPU paths cap at YOLO_MAX_DETS=64; neither CPU path does.
#                  It only BINDS on YOLO26 (~35 det/img) because on-device
#                  censoring already leaves v8/v11 at ~7 det/img.
#   Box coords     Both NPU paths truncate with int() (YOLO26 twice: once in
#                  postprocess, once in the rescale loop). CPU keeps floats.
#
# This measures each factor's worth in mAP points so the corrected sweep can be
# interpreted, and so the paper can state the size of the artifact rather than
# just asserting one existed.
#
# DESIGN
#   yolo26n / NPU   full 2x2x2 factorial: iou {0.45,0.70} x max_det {64,300}
#                                         x coords {int,float}          = 8 runs
#   yolov8n / NPU   coords {int,float} x max_det {64,300}               = 4 runs
#                   (iou is not a factor: its NMS runs on-device at a baked 0.70
#                    and the host cannot change it -- that non-effect is itself
#                    worth showing empirically rather than assuming)
#   yolo26n / CPU   FP32 anchor, controlled postprocess                 = 1 run
#
# 500 images at conf=0.001. conf is low on purpose: the artifacts act on the tail
# of the precision/recall curve, which the deployed 0.20 threshold hides.
#
# Each cell writes to its own directory because the bench's filename tag encodes
# the profile but not iou/max_det/coords, so same-model cells would otherwise
# collide.
set -uo pipefail

REPO="$(cd "$(dirname "$0")/.." && pwd)"
OUT="$REPO/benchmarks/results/accuracy/coco/ablation"
IMGS="$HOME/datasets/coco/val2017"
ANN="$HOME/datasets/coco/annotations/instances_val2017.json"
LOG="$OUT/ablation.log"
N=500
CONF=0.001
mkdir -p "$OUT"

cell() {  # backend model iou max_det coords
    local backend=$1 model=$2 iou=$3 max_det=$4 coords=$5
    local name="${model}_${backend}_iou${iou}_det${max_det}_${coords}"
    local dir="$OUT/$name"
    if compgen -G "$dir/coco_accuracy_*.csv" >/dev/null 2>&1; then
        echo "[skip] $name" | tee -a "$LOG"; return 0
    fi
    mkdir -p "$dir"
    echo "" | tee -a "$LOG"
    echo "######## $name  $(date +%H:%M:%S) ########" | tee -a "$LOG"
    local args=(--backend "$backend" --model "$model" --repo-root "$REPO"
                --coco-images "$IMGS" --coco-ann "$ANN" --conf "$CONF"
                --limit "$N" --iou "$iou" --max-det "$max_det"
                --threads 2 --inter-threads 1 --warmup 5 --out-dir "$dir")
    [ "$coords" = "float" ] && args+=(--float-coords)
    python3 -u "$REPO/benchmarks/coco_accuracy_bench.py" "${args[@]}" 2>&1 \
        | grep -viE --line-buffered "W:onnxruntime|GPU device discovery" | tee -a "$LOG" \
        | grep -iE --line-buffered "mAP50-95 |stop sign:|latency  :|!!|profile=" || true
    sleep 5
}

echo "===== postprocess ablation started $(date) =====" | tee -a "$LOG"

# yolo26n / NPU — full factorial
for iou in 0.45 0.70; do
  for det in 64 300; do
    for co in int float; do
      cell npu yolo26n "$iou" "$det" "$co"
    done
  done
done

# yolov8n / NPU — coords x cap (iou held at the baked 0.70 for honesty)
for det in 64 300; do
  for co in int float; do
    cell npu yolov8n 0.70 "$det" "$co"
  done
done

# yolo26n / CPU FP32 — the un-quantized anchor, controlled postprocess
cell cpu yolo26n 0.70 300 float

echo "" | tee -a "$LOG"
echo "===== postprocess ablation finished $(date) =====" | tee -a "$LOG"
echo "" | tee -a "$LOG"
echo "--- summary ---" | tee -a "$LOG"
python3 - "$OUT" <<'PY' 2>&1 | tee -a "$LOG"
import csv, glob, os, sys
rows = []
for f in sorted(glob.glob(os.path.join(sys.argv[1], "*", "coco_accuracy_*.csv"))):
    with open(f) as fh:
        for r in csv.DictReader(fh):
            rows.append(r)
if not rows:
    print("no results"); raise SystemExit
hdr = f"{'model':9} {'be':4} {'iou':5} {'maxdet':>6} {'coords':6} {'mAP50-95':>9} {'mAP75':>7} {'stopAP':>7} {'dets':>7}"
print(hdr); print("-" * len(hdr))
for r in rows:
    print(f"{r['model']:9} {r['backend']:4} {r['iou']:5} {r['max_det']:>6} "
          f"{r['coord_mode']:6} {float(r['mAP50_95']):>9.4f} {float(r['mAP75']):>7.4f} "
          f"{float(r['stopsign_AP50_95']):>7.4f} {int(r['n_detections']):>7}")
PY
