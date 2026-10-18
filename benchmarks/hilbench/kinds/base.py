"""kinds/base.py — the contract between the core and an experiment (CONTRACT, fixed).

The core (session.py, trial.py, ui.py, summary.py) calls only the slots below and never
branches on a kind's name. A kind is built with the frozen Settings and a Services toolbox.
"""
import abc
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Optional

import bench_geometry as geo

from ..trial import Outcome, StartSpec, TrialResult


@dataclass
class Services:
    """The generic parts a kind receives. Each one has a single job and no kind name."""
    supervisor: Any      # supervisor.Supervisor
    container: Any       # container.Container
    screen: Any          # ui.Screen
    session: Any         # session.Session: .say(msg), .deviation(msg), .folder (Path), .name
    ws_root: Path        # the repository root (the container mounts it as /workspace)


class RunKind(abc.ABC):
    """Fill every abstract slot. Defaults are given where one is sensible."""

    name: str = ""                 # registry key, e.g. "controller"
    columns: tuple = ()            # run-table columns (first two are always "run" and the condition)
    summary_script: str = ""       # repo-relative path of the session summary script
    bench_kill_names: tuple = ()   # process names to pkill -INT in the container after a run
    exe_name: Optional[str] = None # process under test, for affinity_of / process_memory

    def __init__(self, settings, services):
        self.settings = settings
        self.services = services
        self.cfg = settings.kind_cfg   # this kind's own values

    # ── settings ──
    @classmethod
    @abc.abstractmethod
    def fields(cls):
        """list[config.FieldSpec] of this kind's own settings (YAML section = cls.name)."""

    @classmethod
    def validate(cls, settings):
        """list[str] of problems with this kind's own fields (settings.kind_cfg)."""
        return []

    # ── schedule ──
    @abc.abstractmethod
    def conditions(self):
        """list of conditions the schedule rotates through (round robin, settings.runs each)."""

    def condition_label(self, cond):
        """Short text for the screen and runs.csv."""
        return str(cond)

    def start_spec(self):
        x, y, z, yaw = geo.START
        return StartSpec(x, y, z, yaw)

    def model_key(self):
        """(camera_mode, baseline_m or None) for model_check, or None to skip the model check."""
        return None

    # ── session ──
    def preflight(self, session):
        """Extra checks of this kind. Raise trial.RigFailed to stop the session."""

    def snapshot_files(self):
        """Repo-relative files copied into <session>/settings/."""
        return []

    # ── one trial ──
    @abc.abstractmethod
    def matlab_setup(self, trial, cond):
        """Supervisor calls before the hold (e.g. camera publisher on/off)."""

    @abc.abstractmethod
    def boot(self, trial, cond):
        """Start the Pi side of one flight; nothing recorded. Set trial.bench and trial.run_dir."""

    @abc.abstractmethod
    def go(self, trial, cond):
        """Open the gate, wait for the recorder, resume — in this kind's order."""

    @abc.abstractmethod
    def fly(self, trial, cond):
        """The flight. May raise trial.BackendFailed or trial.RigFailed."""

    @abc.abstractmethod
    def stop(self, trial):
        """Stop samplers and the bench. Return a dict of facts for meta.txt."""

    @abc.abstractmethod
    def score(self, trial):
        """Run the scorers for trial.run_dir and return a TrialResult."""

    # ── results ──
    @abc.abstractmethod
    def read_results(self, run_dir):
        """dict read from this kind's own result files in run_dir."""

    @abc.abstractmethod
    def table_row(self, k, cond, res):
        """list[str], one cell per entry of self.columns (res is a TrialResult)."""

    def pace_metric(self, res):
        """Heartbeat rate (Hz) of the run, or None. Default: run_check.json timing."""
        return (res.metrics or {}).get("heartbeat_hz")

    def asks_on(self, outcome):
        """True: the runner asks 'n = next run, a = abort' after this outcome."""
        return outcome == Outcome.RIG_FAILED

    # ── screen and bench output ──
    def flight_rows(self, flight):
        """list[tuple[str, str]] for the flight panel."""
        return []

    def parse_line(self, line, bench):
        """Update bench state from one output line (run_rel, gate_wait, recording, error, extra)."""
