#!/usr/bin/env python3
# ─────────────────────────────────────────────────────────────────────────────
# eval_slam_hil.py — compatibility shim.
#
# The evaluator is now the `slam_eval` package (benchmarks/slam_eval/), split
# per-SLAM with a shared base class, and it computes ATE with evo (the same tool
# the offline ORB-SLAM2 / OV2SLAM benchmarks used). evo + its deps live in a
# dedicated venv (benchmarks/.evo_venv), so this shim just re-execs there.
#
# Usage is unchanged:
#     python3 benchmarks/eval_slam_hil.py bags/run_<config>_<stamp> [--slam <type>]
#
# (Or call the package directly: benchmarks/.evo_venv/bin/python -m slam_eval <run>)
# ─────────────────────────────────────────────────────────────────────────────
import os
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
VENV_PY = HERE / ".evo_venv" / "bin" / "python"

if not VENV_PY.exists():
    sys.exit(
        "evo venv missing at benchmarks/.evo_venv — create it once:\n"
        "  python3 -m venv benchmarks/.evo_venv\n"
        "  benchmarks/.evo_venv/bin/pip install evo rosbags matplotlib numpy"
    )

# Put benchmarks/ on the path so `-m slam_eval` imports, without changing cwd
# (so relative run-dir args like bags/run_... still resolve against the caller).
env = dict(os.environ)
env["PYTHONPATH"] = str(HERE) + os.pathsep + env.get("PYTHONPATH", "")
sys.exit(subprocess.call([str(VENV_PY), "-m", "slam_eval", *sys.argv[1:]], env=env))
