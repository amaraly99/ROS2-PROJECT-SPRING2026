"""summary.py — the files every session has. Reading a run's own result files is the plug-in's job."""
import csv
from pathlib import Path

from .util import SCORER_PY, WS_ROOT, run


def format_row(k, cond_label, res, kind):
    """The cells of one row of the run table: the plug-in builds them, one per column."""
    cells = list(kind.table_row(k, cond_label, res))
    n = len(kind.columns)
    return (cells + ["—"] * n)[:n] if n else cells


def append_runs_csv(path, row):
    with open(path, "a", newline="") as f:
        csv.writer(f).writerow(row)


def finalize(session_folder, kind, python=SCORER_PY):
    """Run the plug-in's summary script on the session folder. Writes summary.log. Returns summary.md or None."""
    folder = Path(session_folder)
    if not kind.summary_script:
        return None
    sm = run([python, str(WS_ROOT / kind.summary_script), str(folder)], timeout=300)
    (folder / "summary.log").write_text(sm.stdout + sm.stderr)
    md = folder / "summary.md"
    return md if md.exists() else None
