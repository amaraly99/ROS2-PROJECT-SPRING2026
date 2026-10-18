"""util.py — small helpers that every module needs. Nothing else lives here.

It also holds the two exception classes of a trial (trial.py re-exports them), so that a
module below trial.py (supervisor.py) can raise them without importing upwards.
"""
import os
import subprocess
import time
from pathlib import Path

WS_ROOT = Path(__file__).resolve().parents[2]            # the repository root
SCORER_PY = os.environ.get("SCORER_PYTHON", "python3")   # needs rosbags + numpy + matplotlib


class RigFailed(Exception):
    """The harness did not do what it claims. The run is RIG_FAILED; the runner asks n / a."""

    def __init__(self, step, why):
        super().__init__(f"{step}: {why}")
        self.step = step
        self.why = why


class BackendFailed(Exception):
    """The system under test failed. The run is BACKEND_FAILED."""

    def __init__(self, reason):
        super().__init__(reason)
        self.reason = reason


def now_ns():
    return time.time_ns()


def run(cmd, timeout=30, check=False):
    p = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
    if check and p.returncode != 0:
        raise RuntimeError(f"{' '.join(cmd)} -> {p.returncode}: {p.stderr.strip()[:300]}")
    return p


def angle_diff(a, b):
    """Smallest absolute difference of two angles, degrees."""
    return abs((a - b + 180.0) % 360.0 - 180.0)
