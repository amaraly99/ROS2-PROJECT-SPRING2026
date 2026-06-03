#!/usr/bin/env bash

# Thin launcher for the standalone Python benchmark runner.
# This no longer delegates to BENCH.sh.

set -euo pipefail

SCRIPT_PATH="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)/$(basename -- "${BASH_SOURCE[0]}")"
SCRIPT_DIR="$(cd -- "$(dirname -- "$SCRIPT_PATH")" && pwd)"

exec python3 "$SCRIPT_DIR/run_benchmark_orbslam2_style.py" "$@"
