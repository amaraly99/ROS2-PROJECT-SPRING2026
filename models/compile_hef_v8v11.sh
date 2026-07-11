#!/usr/bin/env bash
# compile_hef_v8v11.sh
# --------------------
# ██ OBSOLETE (2026-07-11) — DO NOT RUN. The HEFs were never miscalibrated: the
# ██ "person-only" symptom was a fixed-stride parser bug in yolo_producer.py's
# ██ postprocess_ondevice_nms (HAILO NMS BY CLASS output is variable-length).
# ██ Parser fixed; official Model Zoo v5.1.0 hailo10h HEFs installed in models/hef/
# ██ (June-2 originals in models/hef/original_june02_hefs/ — they work too).
# ██ Kept only as a reference for how a DFC recompile would be done.
#
# Recompiles the SIX defective yolov8/yolov11 HEFs with a PROPER COCO calibration
# set, mirroring what produced the working yolo26 HEFs (see ~/vision_ws/compile_hef.sh).
#
# WHY: the existing models/hef/yolov8{n,s,m}.hef and yolov11{n,s,m}.hef were INT8
# quantized with an inadequate calibration set. Result: only the 'person' class
# survived; stop sign / bus / dog / etc. quantize to ~zero score and never fire on
# the NPU. Proven June 23 2026: yolov8n CPU-ONNX detects a stop sign @0.938, but the
# NPU HEF emits nothing even at NMS threshold 0.01. The fix is recompilation with
# real COCO val2017 calibration. (Finding 9.)
#
# WHERE TO RUN: an x86 host with the Hailo Dataflow Compiler / Model Zoo installed.
#   The Raspberry Pi CANNOT run the DFC — it only runs HailoRT inference.
#
# PREREQUISITES (on the x86 DFC host):
#   - Hailo Model Zoo (hailomz) + DFC v5.x for your HailoRT version, venv active
#   - COCO val2017 images for calibration (reuse the SAME set the yolo26 compile used:
#       ~/yolo26_hailo/data/coco/val2017). ~1000 images is plenty.
#   - The source ONNX models (already in this repo): models/onnx/<id>/<id>.onnx
#
# USAGE:
#   source ~/hailo-apps/venv_hailo_apps/bin/activate   # your DFC venv
#   CALIB_DIR=~/yolo26_hailo/data/coco/val2017 bash models/compile_hef_v8v11.sh
#
# After it finishes, copy the new HEFs back to the Pi's models/hef/ and re-run the
# 6 NPU configs (Phase 4 of the finish guide).

set -euo pipefail

# ── config ────────────────────────────────────────────────────────────────────
HW_ARCH="${HW_ARCH:-hailo10h}"                 # MUST match the device (Hailo-10H, 40 TOPS)
CALIB_DIR="${CALIB_DIR:-$HOME/yolo26_hailo/data/coco/val2017}"
REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"  # the ROS2 project root
ONNX_ROOT="$REPO_ROOT/models/onnx"
OUT_DIR="$REPO_ROOT/models/hef"
WORK_DIR="${WORK_DIR:-$HOME/v8v11_recompile}"

# Map: HEF basename  ->  hailomz model name  ->  onnx subdir
#   (hailomz model names per Hailo Model Zoo; verify with `hailomz info | grep -i yolo`.
#    Some MZ versions name v11 'yolov11n', older ones 'yolo11n' — adjust if needed.)
declare -A MZ_NAME=(
  [yolov8n]=yolov8n  [yolov8s]=yolov8s  [yolov8m]=yolov8m
  [yolov11n]=yolov11n [yolov11s]=yolov11s [yolov11m]=yolov11m
)
declare -A ONNX_SUBDIR=(
  [yolov8n]=yolov8n  [yolov8s]=yolov8s  [yolov8m]=yolov8m
  [yolov11n]=yolo11n [yolov11s]=yolo11s [yolov11m]=yolo11m
)

# ── sanity ────────────────────────────────────────────────────────────────────
command -v hailomz >/dev/null 2>&1 || { echo "ERROR: hailomz not on PATH — activate the DFC venv."; exit 1; }
[ -d "$CALIB_DIR" ] || { echo "ERROR: calib dir not found: $CALIB_DIR"; exit 1; }
n_calib=$(find "$CALIB_DIR" -maxdepth 1 -iname '*.jpg' | wc -l)
echo "Calibration images: $n_calib in $CALIB_DIR"
[ "$n_calib" -ge 64 ] || { echo "ERROR: need >=64 calib images (have $n_calib)."; exit 1; }
mkdir -p "$WORK_DIR" "$OUT_DIR"

# ── compile loop ──────────────────────────────────────────────────────────────
for HEF in yolov8n yolov8s yolov8m yolov11n yolov11s yolov11m; do
  mz="${MZ_NAME[$HEF]}"
  onnx="$ONNX_ROOT/${ONNX_SUBDIR[$HEF]}/${ONNX_SUBDIR[$HEF]}.onnx"
  echo ""
  echo "════════════════════════════════════════════════════════════"
  echo "  Compiling $HEF  (mz=$mz, arch=$HW_ARCH)"
  echo "  onnx: $onnx"
  echo "════════════════════════════════════════════════════════════"
  [ -f "$onnx" ] || { echo "  WARN: onnx missing ($onnx) — letting hailomz fetch its own"; onnx=""; }

  cd "$WORK_DIR"
  # hailomz compile bakes the model's on-device NMS postprocess (NMS BY CLASS, 80
  # classes) — the same single-output format our postprocess_ondevice_nms expects.
  # --calib-path forces real COCO calibration (THE fix).
  if [ -n "$onnx" ]; then
    hailomz compile "$mz" --hw-arch "$HW_ARCH" --calib-path "$CALIB_DIR" --ckpt "$onnx"
  else
    hailomz compile "$mz" --hw-arch "$HW_ARCH" --calib-path "$CALIB_DIR"
  fi

  # hailomz writes <mz>.hef into the CWD
  produced="$WORK_DIR/${mz}.hef"
  [ -f "$produced" ] || produced="$(find "$WORK_DIR" -name "${mz}.hef" | sort | tail -1)"
  [ -f "$produced" ] || { echo "  ERROR: no HEF produced for $mz — check DFC log above"; exit 1; }

  cp -f "$produced" "$OUT_DIR/${HEF}.hef"
  echo "  ✓ $OUT_DIR/${HEF}.hef"

  # quick sanity: confirm 80-class NMS-by-class output is present
  hailortcli parse-hef "$OUT_DIR/${HEF}.hef" 2>/dev/null \
    | grep -iE "nms|classes: 80|Score threshold" | head -3 || true
done

echo ""
echo "All 6 HEFs recompiled into $OUT_DIR/"
echo "Next: copy models/hef/ back to the Pi, then verify each detects a stop sign"
echo "      (see the finish guide, Phase 4)."
