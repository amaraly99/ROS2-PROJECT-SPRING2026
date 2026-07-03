#!/usr/bin/env bash
# Reads config/hil/startup_config.yaml's `slam:` key and builds ONLY that one
# SLAM sidecar, instead of building every SLAM every time. Dispatches to the
# existing build path for each -- doesn't reinvent either one.
set -euo pipefail

WS="$(cd "$(dirname "$0")/.." && pwd)"
CFG="${WS}/config/hil/startup_config.yaml"

[[ -f "$CFG" ]] || { echo "ERROR: $CFG not found" >&2; exit 1; }

SLAM=$(grep -E '^\s*slam:' "$CFG" | head -1 \
  | sed -E 's/^\s*slam:\s*//; s/\s*#.*$//; s/["'"'"']//g' | xargs)

case "$SLAM" in
  ov2slam)
    echo "[test_slam_build] Building OV2SLAM only (colcon --packages-select ov2slam)..."
    "${WS}/run_stack_hil.sh" build ov2slam
    ;;
  orbslam2)
    echo "[test_slam_build] Building ORB-SLAM2 (src/orbslam2/start_container)..."
    (cd "${WS}/src/orbslam2" && ./start_container)
    ;;
  none|"")
    echo "[test_slam_build] slam: none -- skipping, nothing built."
    ;;
  *)
    echo "ERROR: unknown slam '${SLAM}' in ${CFG} (expected ov2slam | orbslam2 | none)" >&2
    exit 1
    ;;
esac

echo "[test_slam_build] Done."
