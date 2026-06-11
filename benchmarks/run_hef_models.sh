#!/bin/bash
# ─── HEF model sweep — functional check + benchmark for all models/hef/*.hef ──
#
# Camera-free. For every HEF it runs benchmarks/hef_bench.py (the producer's real
# postprocess) → single-stream FPS, per-stage timing, detection sanity on a static
# image, and an annotated jpg. Then prints one comparison table + writes a CSV.
#
# Each model runs in its own subprocess that CLEANLY releases the VDevice, with a
# settle delay between models. Do NOT interleave `hailortcli benchmark` here — a
# second process grabbing the VDevice while the CIM is mid-release wedges the
# HAILO10H SoC off the PCIe bus (needs a reboot). Run hailortcli separately if you
# want the pipelined NPU-streaming ceiling.
#
# Usage:
#   bash benchmarks/run_hef_models.sh [IMAGE] [ITERS]
# Defaults: IMAGE=/tmp/bus.jpg  ITERS=60

set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
BENCH="$SCRIPT_DIR/hef_bench.py"

IMAGE="${1:-/tmp/bus.jpg}"
ITERS="${2:-60}"
WARMUP=8
SETTLE=2          # seconds between models — let the NPU fully release

TS=$(date +%Y%m%d_%H%M%S)
OUT="$SCRIPT_DIR/results/hef"
mkdir -p "$OUT"
TIMING_CSV="$OUT/hef_timing_${TS}.csv"

if [ ! -f "$IMAGE" ] && [ "$IMAGE" = "/tmp/bus.jpg" ]; then
    echo "test image missing — fetching COCO sample (bus.jpg)..."
    curl -sSL --max-time 20 -o "$IMAGE" https://ultralytics.com/images/bus.jpg || true
fi
if [ ! -f "$IMAGE" ]; then
    echo "ERROR: test image not found: $IMAGE"; exit 1
fi

# Reference (known-good) first, then the 9 new ones.
HEFS=( "$PROJECT_DIR/models/yolo26n_10h.hef" )
for f in "$PROJECT_DIR"/models/hef/*.hef; do HEFS+=( "$f" ); done

echo "═══════════════════════════════════════════════════════════════════"
echo " HEF model sweep — ${#HEFS[@]} models   image=$IMAGE   iters=$ITERS"
echo "═══════════════════════════════════════════════════════════════════"

for hef in "${HEFS[@]}"; do
    name="$(basename "${hef%.hef}")"
    echo ""
    python3 "$BENCH" --hef "$hef" --image "$IMAGE" \
        --iters "$ITERS" --warmup "$WARMUP" \
        --csv "$TIMING_CSV" --annotate "$OUT/${name}.jpg" 2>&1 \
        | grep -vE "Lost communication|VDevice is released|CHECK failed"
    sleep "$SETTLE"
done

# ── Final aggregate table ────────────────────────────────────────────────────
echo ""
echo "═══════════════════════════════════════════════════════════════════"
echo " SUMMARY"
echo "═══════════════════════════════════════════════════════════════════"
python3 - "$TIMING_CSV" <<'PY'
import csv, sys
rows = list(csv.DictReader(open(sys.argv[1])))
print(f"  {'model':<14}{'MB':>6}{'status':>11}{'pp_mode':>20}{'infer':>8}{'e2e':>8}{'FPS':>7}{'dets':>6}  detections")
print("  " + "-" * 104)
for r in rows:
    def num(k, f="{:.1f}"):
        v = r.get(k, "")
        return f.format(float(v)) if v else "-"
    print(f"  {r['model']:<14}{r['size_mb']:>6}{r['status']:>11}{r['pp_mode']:>20}"
          f"{num('inference_ms'):>8}{num('e2e_ms'):>8}{num('fps'):>7}{r['n_dets']:>6}  {r['detections']}")
print()
print(f"  timing CSV : {sys.argv[1]}")
print(f"  annotated  : benchmarks/results/hef/<model>.jpg")
PY
