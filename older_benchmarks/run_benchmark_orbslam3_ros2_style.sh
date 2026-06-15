#!/usr/bin/env bash

set -euo pipefail

SCRIPT_PATH="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)/$(basename -- "${BASH_SOURCE[0]}")"
SCRIPT_DIR="$(cd -- "$(dirname -- "$SCRIPT_PATH")" && pwd)"

exec python3 "$SCRIPT_DIR/run_benchmark_orbslam3_ros2_style.py" "$@"
