#!/usr/bin/env python3
"""ctrl_runner2.py — the HIL controller benchmark on the shared harness (hilbench, kind "controller").

    ./run_controller_benchmark2.sh                          # everything from config/hil/controller_bench.yaml
    ./run_controller_benchmark2.sh ibvs --runs 10           # a controller name first, as in ctrl_runner.py
    ./run_controller_benchmark2.sh --dry-run                # preflight + status only
Keys during a session: p = pause after this run, a = abort, l = show/hide the bench log.
The old runner ctrl_runner.py stays until the campaign ends.
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from hilbench.factory import main  # noqa: E402

CONTROLLERS = ("proportional", "ibvs", "h_vs", "pbvs")

if __name__ == "__main__":
    args = sys.argv[1:]
    if args and args[0] in CONTROLLERS:      # old style: the controller as the first word
        args = ["--controller", args[0]] + args[1:]
    sys.exit(main("controller", args))
