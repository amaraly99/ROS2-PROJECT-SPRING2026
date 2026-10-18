#!/bin/bash
# ─────────────────────────────────────────────────────────────────────────────
# run_controller_benchmark2.sh — the HIL controller benchmark on the shared harness (hilbench).
#
#   ./run_controller_benchmark2.sh                       # all settings from config/hil/controller_bench.yaml
#   ./run_controller_benchmark2.sh --dry-run             # checks only (Wi-Fi, MATLAB)
#   ./run_controller_benchmark2.sh --runs 10 --camera-pub off   # a command-line option wins over the file
#   ./run_controller_benchmark2.sh --help                # every option
#
# Before: in MATLAB on Windows ->  cd <repo>\matlab ; hil_session_start
# Guide:  benchmarks/hilbench/README.md
# The old runner (run_controller_benchmark.sh) stays until the campaign ends.
# ─────────────────────────────────────────────────────────────────────────────
# One-time setup: python3 -m venv ~/.venvs/hilrun && ~/.venvs/hilrun/bin/pip install rich pyyaml
VENV_PY="${HILRUN_PYTHON:-$HOME/.venvs/hilrun/bin/python}"
[[ -x "$VENV_PY" ]] || { echo "missing $VENV_PY — run: python3 -m venv ~/.venvs/hilrun && ~/.venvs/hilrun/bin/pip install rich pyyaml" >&2; exit 1; }
exec "$VENV_PY" "$(cd "$(dirname "$0")" && pwd)/benchmarks/ctrl_runner2.py" "$@"
