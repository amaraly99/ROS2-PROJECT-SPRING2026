#!/bin/bash
# ─── YOLO Throughput vs Resolution Tradeoff ─────────────────────
# Runs yolo_producer at 3 effective resolutions by downsampling
# the source image before inference. The HEF is fixed at 640x640
# so NPU inference time stays constant — this measures detection
# quality degradation vs effective resolution.
#
# Prerequisites: ovcam_producer must be running (camera SHM active).
#
# Usage:
#   bash benchmarks/yolo_resolution_bench.sh [NUM_FRAMES]
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

echo "═══════════════════════════════════════════════════════════"
echo " Throughput vs Resolution Tradeoff"
echo " Frames per run: $NUM_FRAMES"
echo " Camera: 640x480, HEF: 640x640 (fixed)"
echo "═══════════════════════════════════════════════════════════"
echo ""

# Downsample factors → effective resolutions
# 1.0 → 640x480 (native)
# 1.54 → ~416x312
# 2.0 → 320x240
FACTORS="1.0 1.54 2.0"
LABELS="640x480 416x312 320x240"

CSV_FILES=""
for factor in $FACTORS; do
    if [ "$factor" = "1.0" ]; then
        label="640x480"
    elif [ "$factor" = "1.54" ]; then
        label="416x312"
    else
        label="320x240"
    fi

    CSV="$OUT_DIR/res_${label}_${TIMESTAMP}.csv"
    echo "▶ Effective resolution ${label} (downsample=${factor}x) — $NUM_FRAMES frames..."
    python3 "$PRODUCER" --hef "$HEF" --no-image \
        --downsample "$factor" \
        --csv "$CSV" --max-frames "$NUM_FRAMES" 2>&1 | tail -3
    echo "  → $CSV"
    echo ""

    if [ -z "$CSV_FILES" ]; then
        CSV_FILES="$CSV"
    else
        CSV_FILES="$CSV_FILES,$CSV"
    fi
done

# ── Print comparison table ────────────────────────────────────
echo "═══════════════════════════════════════════════════════════"
echo " Results (mean over $NUM_FRAMES frames, 10-frame warmup)"
echo "═══════════════════════════════════════════════════════════"

python3 -c "
import csv, sys

def load_stats(path):
    with open(path) as f:
        reader = csv.DictReader(f)
        rows = list(reader)
    rows = rows[10:]  # skip warmup
    n_dets = [float(r['n_dets']) for r in rows]
    inference = [float(r['inference_ms']) for r in rows]
    preprocess = [float(r['preprocess_ms']) for r in rows]
    total_pp = [float(r['total_pp_ms']) for r in rows]
    total = [p + i + pp for p, i, pp in zip(preprocess, inference, total_pp)]
    return {
        'n_dets_mean': sum(n_dets) / len(n_dets),
        'n_dets_std': (sum((x - sum(n_dets)/len(n_dets))**2 for x in n_dets) / len(n_dets))**0.5,
        'inference_ms': sum(inference) / len(inference),
        'preprocess_ms': sum(preprocess) / len(preprocess),
        'total_pp_ms': sum(total_pp) / len(total_pp),
        'total_ms': sum(total) / len(total),
        'fps': 1000.0 / (sum(total) / len(total)),
        'n_frames': len(rows),
        # Frames with zero detections
        'zero_det_pct': sum(1 for x in n_dets if x == 0) / len(n_dets) * 100,
    }

csvs = '$CSV_FILES'.split(',')
labels = ['640x480', '416x312', '320x240']

results = []
for path, label in zip(csvs, labels):
    s = load_stats(path)
    results.append((label, s))

hdr = '  {:<22s}' + '  {:>12s}' * len(results)
print(hdr.format('Metric', *[r[0] for r in results]))
print('  ' + '-' * (22 + 14 * len(results)))

rows_out = [
    ('Preprocess',      'preprocess_ms', 'ms'),
    ('NPU inference',   'inference_ms',  'ms'),
    ('CPU postprocess',  'total_pp_ms',  'ms'),
    ('Total pipeline',   'total_ms',     'ms'),
    ('FPS',              'fps',          'f1'),
    ('Avg detections',   'n_dets_mean',  'f1'),
    ('Det std dev',      'n_dets_std',   'f1'),
    ('Zero-det frames',  'zero_det_pct', 'pct'),
]

for label, key, fmt in rows_out:
    vals = []
    for name, s in results:
        v = s[key]
        if fmt == 'ms':
            vals.append(f'{v:.2f}ms')
        elif fmt == 'pct':
            vals.append(f'{v:.1f}%')
        else:
            vals.append(f'{v:.1f}')
    row_fmt = '  {:<22s}' + '  {:>12s}' * len(vals)
    print(row_fmt.format(label, *vals))
"

echo ""
echo "To generate plots: python3 benchmarks/plot_resolution.py $OUT_DIR/res_*_${TIMESTAMP}.csv"
echo "CSV files saved in: $OUT_DIR/"
