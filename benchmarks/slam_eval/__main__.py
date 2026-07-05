#!/usr/bin/env python3
# __main__.py — dispatch a run dir to the right per-SLAM evaluator.
#
#   benchmarks/.evo_venv/bin/python -m slam_eval <run_dir> [--slam <type>]
#
# The SLAM backend is taken from --slam, else from the run's meta.txt (slam_type=).
# To add a backend: write its subclass (see base.py) and register it below.
import sys
from pathlib import Path

from .orbslam2 import Orbslam2Evaluator
from .ov2slam import Ov2slamEvaluator

SLAM_REGISTRY = {
    "orbslam2": Orbslam2Evaluator,
    "ov2slam": Ov2slamEvaluator,
}


def _meta_slam_type(rundir: Path) -> str:
    meta = rundir / "meta.txt"
    if meta.exists():
        for line in meta.read_text().splitlines():
            if line.startswith("slam_type="):
                return line.split("=", 1)[1].strip()
    return ""


def main():
    argv = sys.argv[1:]
    if not argv:
        sys.exit("usage: python -m slam_eval <run_dir> [--slam <type>]")
    rundir = Path(argv[0])
    if not rundir.exists():
        sys.exit(f"run dir not found: {rundir}")

    slam_type = ""
    if "--slam" in argv:
        i = argv.index("--slam")
        if i + 1 < len(argv):
            slam_type = argv[i + 1]
    if not slam_type:
        slam_type = _meta_slam_type(rundir)
    if not slam_type:
        sys.exit("could not determine slam_type — pass --slam <type> or ensure "
                 "meta.txt has a slam_type= line")

    cls = SLAM_REGISTRY.get(slam_type)
    if cls is None:
        sys.exit(f"no evaluator for slam_type='{slam_type}'. Known: {sorted(SLAM_REGISTRY)}")
    cls(rundir).run()


if __name__ == "__main__":
    main()
