#!/bin/bash
# ─── YOLO FP32 vs FP16 Per-Operation Breakdown ─────────────────
# Runs yolo_producer twice (FP32 then FP16), collects per-frame
# timing CSVs, and prints a comparison table.
#
# Prerequisites: ovcam_producer must be running (camera SHM active).
#
# Usage:
#   bash benchmarks/yolo_fp_breakdown.sh [NUM_FRAMES]
#
# Default: 500 frames per run.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
PRODUCER="$PROJECT_DIR/src/yolo_producer/yolo_producer.py"
HEF="$PROJECT_DIR/models/yolo26n_10h.hef"

NUM_FRAMES="${1:-500}"
OUT_DIR="$SCRIPT_DIR/results"
mkdir -p "$OUT_DIR"

TIMESTAMP=$(date +%Y%m%d_%H%M%S)
FP32_CSV="$OUT_DIR/fp32_breakdown_${TIMESTAMP}.csv"
FP16_CSV="$OUT_DIR/fp16_breakdown_${TIMESTAMP}.csv"

echo "═══════════════════════════════════════════════════════════"
echo " YOLO FP32 vs FP16 Per-Operation Breakdown"
echo " Frames per run: $NUM_FRAMES"
echo "═══════════════════════════════════════════════════════════"
echo ""

# ── Run FP32 ──────────────────────────────────────────────────
echo "▶ Running FP32 ($NUM_FRAMES frames)..."
python3 "$PRODUCER" --hef "$HEF" --no-image \
    --csv "$FP32_CSV" --max-frames "$NUM_FRAMES" 2>&1 | tail -5
echo "  → $FP32_CSV"
echo ""

# ── Run FP16 ──────────────────────────────────────────────────
echo "▶ Running FP16 ($NUM_FRAMES frames)..."
python3 "$PRODUCER" --hef "$HEF" --no-image --fp16 \
    --csv "$FP16_CSV" --max-frames "$NUM_FRAMES" 2>&1 | tail -5
echo "  → $FP16_CSV"
echo ""

# ── Print comparison table ────────────────────────────────────
echo "═══════════════════════════════════════════════════════════"
echo " Results (mean over $NUM_FRAMES frames)"
echo "═══════════════════════════════════════════════════════════"

python3 -c "
import csv, sys

def load_means(path):
    with open(path) as f:
        reader = csv.DictReader(f)
        rows = list(reader)
    if not rows:
        print(f'ERROR: {path} is empty', file=sys.stderr)
        sys.exit(1)
    # Skip first 10 frames (warmup)
    rows = rows[10:]
    cols = ['preprocess_ms', 'inference_ms', 'prefilter_ms', 'sigmoid_ms',
            'bbox_extract_ms', 'grid_lookup_ms', 'coord_rescale_ms',
            'decode_ms', 'nms_ms', 'total_pp_ms', 'n_dets']
    means = {}
    for c in cols:
        vals = [float(r[c]) for r in rows]
        means[c] = sum(vals) / len(vals)
    means['_n'] = len(rows)
    return means

fp32 = load_means('$FP32_CSV')
fp16 = load_means('$FP16_CSV')

print(f'  Warmup: 10 frames skipped, {fp32[\"_n\"]} frames averaged')
print()
fmt = '  {:<22s}  {:>8s}  {:>8s}  {:>8s}'
print(fmt.format('Operation', 'FP32', 'FP16', 'Ratio'))
print('  ' + '-' * 52)

ops = [
    ('Pre-filter (logits)',  'prefilter_ms'),
    ('Sigmoid + argmax',     'sigmoid_ms'),
    ('Bbox extract',         'bbox_extract_ms'),
    ('Grid lookup',          'grid_lookup_ms'),
    ('Coord rescale',        'coord_rescale_ms'),
    ('Decode (total)',       'decode_ms'),
    ('NMS (C++ internal)',   'nms_ms'),
    ('Total post-process',   'total_pp_ms'),
    ('',                     None),
    ('Preprocess',           'preprocess_ms'),
    ('NPU inference',        'inference_ms'),
    ('Avg detections',       'n_dets'),
]

for label, key in ops:
    if key is None:
        print()
        continue
    v32 = fp32[key]
    v16 = fp16[key]
    if key == 'n_dets':
        print(f'  {label:<22s}  {v32:>8.1f}  {v16:>8.1f}  {\"\":>8s}')
    elif v32 > 0.001:
        ratio = v16 / v32
        print(f'  {label:<22s}  {v32:>7.3f}ms  {v16:>7.3f}ms  {ratio:>6.1f}x')
    else:
        print(f'  {label:<22s}  {v32:>7.3f}ms  {v16:>7.3f}ms  {\"n/a\":>8s}')
"

echo ""
echo "CSV files saved in: $OUT_DIR/"
