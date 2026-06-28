#!/usr/bin/env bash

set -euo pipefail

if [[ $# -ne 2 ]]; then
  echo "Usage: $0 <ground_truth_tum.txt> <trajectory_tum.txt>" >&2
  exit 1
fi

GT_PATH="$1"
TRAJ_PATH="$2"

if [[ ! -f "$GT_PATH" ]]; then
  echo "Ground truth file not found: $GT_PATH" >&2
  exit 1
fi

if [[ ! -f "$TRAJ_PATH" ]]; then
  echo "Trajectory file not found: $TRAJ_PATH" >&2
  exit 1
fi

if ! command -v evo_ape >/dev/null 2>&1; then
  echo "evo_ape is not in PATH." >&2
  exit 1
fi

if ! command -v evo_rpe >/dev/null 2>&1; then
  echo "evo_rpe is not in PATH." >&2
  exit 1
fi

if ! command -v evo_traj >/dev/null 2>&1; then
  echo "evo_traj is not in PATH." >&2
  exit 1
fi

BASE_NAME="$(basename "$TRAJ_PATH")"
BASE_NAME="${BASE_NAME%.*}"

echo "Generating APE trajectory plots for:"
echo "  GT:   $GT_PATH"
echo "  Traj: $TRAJ_PATH"

evo_ape tum "$GT_PATH" "$TRAJ_PATH" -a -s -p -v --plot_mode xy \
  --save_plot "${BASE_NAME}_ape_xy.png"

evo_ape tum "$GT_PATH" "$TRAJ_PATH" -a -s -p -v --plot_mode xyz \
  --save_plot "${BASE_NAME}_ape_xyz.png"

echo "Saved plots:"
echo "  ${BASE_NAME}_ape_xy.png"
echo "  ${BASE_NAME}_ape_xyz.png"
