#!/usr/bin/env bash
# fetch_models.sh — obtain every model the detector sweep needs.
#
# The model binaries are ~700 MB and are NOT committed. This script reproduces
# models/hef/ and models/onnx/ exactly as the paper used them. Run from anywhere:
#
#   bash models/fetch_models.sh              # everything
#   bash models/fetch_models.sh hef          # NPU only (enough to run the stack)
#   bash models/fetch_models.sh onnx         # CPU baselines only
#
# Provenance of each group:
#
#   v8/v11 HEFs   Official Hailo Model Zoo v5.1.0 hailo10h builds, downloaded
#                 below and checksum-verified. These are the ones the paper used
#                 (verified byte-identical). They bake a 0.20 NMS score threshold
#                 into the on-device op, which is why every config in
#                 model_registry.json runs at conf 0.20 — see the note there.
#
#   yolo26 HEFs   Custom-compiled with the Hailo Dataflow Compiler against COCO
#                 val2017 calibration; there is no public Model Zoo entry for
#                 YOLO26 on hailo10h. Published as assets on the release tag.
#                 Set MODELS_RELEASE_URL to fetch them, or compile your own on an
#                 x86 DFC host (the Pi cannot run the DFC — only HailoRT inference).
#
#   ONNX          Exported from the Ultralytics .pt weights at 640x640. Requires
#                 the `ultralytics` package. Note the weights are AGPL-3.0 — see
#                 THIRD_PARTY_LICENSES.md.

set -euo pipefail

MODELS_DIR="$(cd "$(dirname "$0")" && pwd)"
MZ_BASE="${MZ_BASE:-https://hailo-model-zoo.s3.eu-west-2.amazonaws.com/ModelZoo/Compiled/v5.1.0/hailo10h}"
MODELS_RELEASE_URL="${MODELS_RELEASE_URL:-}"
WHAT="${1:-all}"

# sha256 of every artifact as used in the paper.
declare -A SHA=(
  [yolov8n.hef]=ac0938bbea8811b0e10acff45002e251f6206243d12f6bd29aa26d0dd1c48793
  [yolov8s.hef]=3cc9ddd06061a3c1f7db8bb99fd48004d9e01ce89effcd02edec93e93d7f3907
  [yolov8m.hef]=861d6ece3aea7685d6b245abd5864d9b3498ce23aa1968f3835a591835da5b57
  [yolov11n.hef]=24f6a0018e010be016e4e75be9c510b1d91927e635278918e5c003bc545bdc4a
  [yolov11s.hef]=0085e22da9a31dc29ade6c80ab08efd4590ed31e9c176a26dc4d003e065c8a22
  [yolov11m.hef]=17903c551faf97a975d8b98d78534576df11264c8898b57c90a4416ccee90310
  [yolo26n.hef]=61ace06ad27b97c1436037747a7543b09b5a7776845f8967234ec943c34e2bd2
  [yolo26s.hef]=419c8ad2a6074456bf1e93cd53d011f94a3b7edbea99747a54fce4f97fd40df0
  [yolo26m.hef]=27c0d03f11772e2908f2dd6317843076666b02bb430dd5b699e37f7db823e533
)

MZ_HEFS=(yolov8n yolov8s yolov8m yolov11n yolov11s yolov11m)
YOLO26_HEFS=(yolo26n yolo26s yolo26m)

# id -> onnx subdirectory (on-disk naming is inconsistent across families:
# yolov11n.hef but yolo11n.onnx — model_registry.json is the source of truth).
declare -A ONNX_SUBDIR=(
  [yolov8n]=yolov8n  [yolov8s]=yolov8s  [yolov8m]=yolov8m
  [yolov11n]=yolo11n [yolov11s]=yolo11s [yolov11m]=yolo11m
  [yolo26n]=yolo26n  [yolo26s]=yolo26s  [yolo26m]=yolo26m
)

verify() {  # path
    local f="$1" want="${SHA[$(basename "$f")]:-}"
    [ -n "$want" ] || return 0
    local got
    got="$(sha256sum "$f" | cut -d' ' -f1)"
    if [ "$got" != "$want" ]; then
        echo "  ✗ CHECKSUM MISMATCH: $(basename "$f")"
        echo "     expected $want"
        echo "     got      $got"
        echo "     This is NOT the artifact the paper used. Delete it and retry."
        return 1
    fi
    echo "  ✓ $(basename "$f") (sha256 ok)"
}

fetch_hef() {
    mkdir -p "$MODELS_DIR/hef"

    echo "── Hailo Model Zoo HEFs (v8/v11) ───────────────────────────────"
    for m in "${MZ_HEFS[@]}"; do
        local out="$MODELS_DIR/hef/$m.hef"
        if [ -f "$out" ] && verify "$out" 2>/dev/null; then continue; fi
        echo "  ↓ $m.hef"
        curl -fL --retry 3 -o "$out" "$MZ_BASE/$m.hef"
        verify "$out"
    done

    echo "── YOLO26 HEFs (custom DFC build) ──────────────────────────────"
    if [ -z "$MODELS_RELEASE_URL" ]; then
        echo "  ! MODELS_RELEASE_URL is not set, so the YOLO26 HEFs cannot be fetched."
        echo "    Get them from the release assets:"
        echo "      MODELS_RELEASE_URL=https://github.com/<owner>/<repo>/releases/download/<tag> \\"
        echo "        bash models/fetch_models.sh hef"
        echo "    Or compile them yourself on an x86 Hailo DFC host."
        echo "    The v8/v11 models above are enough to run the stack and 12 of the"
        echo "    20 sweep configs; the 8 yolo26 configs need these."
        return 0
    fi
    for m in "${YOLO26_HEFS[@]}"; do
        local out="$MODELS_DIR/hef/$m.hef"
        if [ -f "$out" ] && verify "$out" 2>/dev/null; then continue; fi
        echo "  ↓ $m.hef"
        curl -fL --retry 3 -o "$out" "$MODELS_RELEASE_URL/$m.hef"
        verify "$out"
    done
}

fetch_onnx() {
    echo "── ONNX exports (CPU baselines) ────────────────────────────────"
    if ! python3 -c 'import ultralytics' 2>/dev/null; then
        echo "  ! ultralytics is not installed — skipping ONNX export."
        echo "    pip install ultralytics, then: bash models/fetch_models.sh onnx"
        return 0
    fi
    for id in "${!ONNX_SUBDIR[@]}"; do
        local sub="${ONNX_SUBDIR[$id]}"
        local out="$MODELS_DIR/onnx/$sub/$sub.onnx"
        [ -f "$out" ] && { echo "  ✓ $sub.onnx (present)"; continue; }
        echo "  ↓ exporting $sub.onnx"
        mkdir -p "$MODELS_DIR/onnx/$sub"
        python3 - "$sub" "$MODELS_DIR/onnx/$sub" <<'PY'
import shutil, sys
from pathlib import Path
from ultralytics import YOLO
name, dest = sys.argv[1], Path(sys.argv[2])
m = YOLO(f"{name}.pt")
p = Path(m.export(format="onnx", imgsz=640, opset=11, simplify=True))
shutil.move(str(p), dest / f"{name}.onnx")
PY
    done
    echo "  Note: the *_nms_config.json files next to each ONNX are DFC compile"
    echo "  metadata, not needed for CPU inference."
}

case "$WHAT" in
    hef)  fetch_hef ;;
    onnx) fetch_onnx ;;
    all)  fetch_hef; fetch_onnx ;;
    *)    echo "usage: $0 [all|hef|onnx]" >&2; exit 2 ;;
esac

echo
echo "Done. models/model_registry.json maps every logical id to these paths."
