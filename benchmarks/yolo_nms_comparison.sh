#!/bin/bash
# ─── YOLO On-Device NMS vs CPU Post-Processing Comparison ──────
# Compares yolo26n (CPU postprocess) vs yolov8m/yolov11m (on-device NMS).
#
# Prerequisites: ovcam_producer must be running (camera SHM active).
#
# Usage:
#   bash benchmarks/yolo_nms_comparison.sh [NUM_FRAMES]
#
# Default: 500 frames per run.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
PRODUCER="$PROJECT_DIR/src/yolo_producer/yolo_producer.py"

YOLO26N_HEF="$PROJECT_DIR/models/yolo26n_10h.hef"
YOLOV8M_HEF="/usr/share/hailo-models/yolov8m_h10.hef"
YOLOV11M_HEF="/usr/share/hailo-models/yolov11m_h10.hef"

NUM_FRAMES="${1:-500}"
OUT_DIR="$SCRIPT_DIR/results"
mkdir -p "$OUT_DIR"

TIMESTAMP=$(date +%Y%m%d_%H%M%S)

echo "═══════════════════════════════════════════════════════════"
echo " On-Device NMS vs CPU Post-Processing Comparison"
echo " Frames per run: $NUM_FRAMES"
echo "═══════════════════════════════════════════════════════════"
echo ""

# ── Run yolo26n (CPU postprocess) ─────────────────────────────
CSV_26N="$OUT_DIR/nms_yolo26n_${TIMESTAMP}.csv"
echo "▶ yolo26n (4.6MB, CPU postprocess) — $NUM_FRAMES frames..."
python3 "$PRODUCER" --hef "$YOLO26N_HEF" --no-image \
    --csv "$CSV_26N" --max-frames "$NUM_FRAMES" 2>&1 | tail -5
echo "  → $CSV_26N"
echo ""

# ── Run yolov8m (on-device NMS) ───────────────────────────────
CSV_V8M="$OUT_DIR/nms_yolov8m_${TIMESTAMP}.csv"
if [ -f "$YOLOV8M_HEF" ]; then
    echo "▶ yolov8m (21MB, on-device NMS) — $NUM_FRAMES frames..."
    python3 "$PRODUCER" --hef "$YOLOV8M_HEF" --no-image --ondevice-nms \
        --csv "$CSV_V8M" --max-frames "$NUM_FRAMES" 2>&1 | tail -5
    echo "  → $CSV_V8M"
else
    echo "⚠ yolov8m HEF not found at $YOLOV8M_HEF, skipping"
    CSV_V8M=""
fi
echo ""

# ── Run yolov11m (on-device NMS) ──────────────────────────────
CSV_V11M="$OUT_DIR/nms_yolov11m_${TIMESTAMP}.csv"
if [ -f "$YOLOV11M_HEF" ]; then
    echo "▶ yolov11m (28MB, on-device NMS) — $NUM_FRAMES frames..."
    python3 "$PRODUCER" --hef "$YOLOV11M_HEF" --no-image --ondevice-nms \
        --csv "$CSV_V11M" --max-frames "$NUM_FRAMES" 2>&1 | tail -5
    echo "  → $CSV_V11M"
else
    echo "⚠ yolov11m HEF not found at $YOLOV11M_HEF, skipping"
    CSV_V11M=""
fi
echo ""

# ── Print comparison table ────────────────────────────────────
echo "═══════════════════════════════════════════════════════════"
echo " Results (mean over $NUM_FRAMES frames, 10-frame warmup)"
echo "═══════════════════════════════════════════════════════════"

python3 -c "
import csv, sys, os

def load_means(path):
    if not path or not os.path.exists(path):
        return None
    with open(path) as f:
        reader = csv.DictReader(f)
        rows = list(reader)
    if len(rows) < 20:
        print(f'WARNING: {path} has only {len(rows)} rows', file=sys.stderr)
        return None
    rows = rows[10:]  # skip warmup
    cols = ['preprocess_ms', 'inference_ms', 'total_pp_ms', 'n_dets']
    means = {}
    for c in cols:
        vals = [float(r[c]) for r in rows]
        means[c] = sum(vals) / len(vals)
    # Total pipeline = preprocess + inference + postprocess
    means['total_ms'] = means['preprocess_ms'] + means['inference_ms'] + means['total_pp_ms']
    means['fps'] = 1000.0 / means['total_ms'] if means['total_ms'] > 0 else 0
    means['_n'] = len(rows)
    return means

models = [
    ('yolo26n (CPU PP)',   '$CSV_26N'),
    ('yolov8m (NPU NMS)',  '$CSV_V8M'),
    ('yolov11m (NPU NMS)', '$CSV_V11M'),
]

results = []
for name, path in models:
    m = load_means(path)
    if m is not None:
        results.append((name, m))

if not results:
    print('ERROR: No valid results')
    sys.exit(1)

hdr = '  {:<22s}' + '  {:>12s}' * len(results)
print(hdr.format('Metric', *[r[0] for r in results]))
print('  ' + '-' * (22 + 14 * len(results)))

rows_out = [
    ('Preprocess',      'preprocess_ms', 'ms'),
    ('NPU inference',   'inference_ms',  'ms'),
    ('CPU postprocess',  'total_pp_ms',  'ms'),
    ('Total pipeline',   'total_ms',     'ms'),
    ('FPS',              'fps',          ''),
    ('Avg detections',   'n_dets',       ''),
]

for label, key, unit in rows_out:
    vals = []
    for name, m in results:
        v = m[key]
        if unit == 'ms':
            vals.append(f'{v:.2f}ms')
        elif key == 'fps':
            vals.append(f'{v:.1f}')
        else:
            vals.append(f'{v:.1f}')
    fmt = '  {:<22s}' + '  {:>12s}' * len(vals)
    print(fmt.format(label, *vals))
"

echo ""
echo "CSV files saved in: $OUT_DIR/"
