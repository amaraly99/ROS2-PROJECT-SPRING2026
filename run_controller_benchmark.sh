#!/bin/bash
# ─────────────────────────────────────────────────────────────────────────────
# run_controller_benchmark.sh — THE way to run the HIL controller benchmark.
#
#   ./run_controller_benchmark.sh                       # all settings from config/hil/ctrl_bench.yaml
#   ./run_controller_benchmark.sh --dry-run             # checks only (Wi-Fi, MATLAB, container)
#   ./run_controller_benchmark.sh --runs 10 --camera-pub off   # a command-line option wins over the file
#   ./run_controller_benchmark.sh --help                # every option
#
# Before: in MATLAB on Windows ->  cd <repo>\matlab ; hil_session_start
# Guide:  benchmarks/CONTROLLER_BENCHMARK.md
# ─────────────────────────────────────────────────────────────────────────────
exec "$(cd "$(dirname "$0")" && pwd)/benchmarks/ctrl_runner" "$@"
