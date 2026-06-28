#!/usr/bin/env python3
"""
Polling report: 10×10 markdown grid with traj_xy + RMSE per cell, then summary tables.
Usage:
    python3 tools/polling_report.py                          # latest batch
    python3 tools/polling_report.py 20260604_110230          # specific batch
    python3 tools/polling_report.py --all                    # all batches
Output: results/<batch>/polling_report_<HH_MM>.md  (or repo root if not writable)
"""

from __future__ import annotations

import json
import statistics
import sys
from dataclasses import dataclass
from datetime import datetime, timedelta, timezone
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
RESULTS_DIR = ROOT / "results"
GT_DIR = ROOT / "datasets" / "gt"
GT_MATCH_MAX_DIFF_S = 0.1

UTC_PLUS_4 = timezone(timedelta(hours=4))

SEQUENCES = [
    "MH_01_easy",
    "MH_02_easy",
    "MH_03_medium",
    "MH_04_difficult",
    "MH_05_difficult",
    "V1_01_easy",
    "V1_02_medium",
    "V1_03_difficult",
    "V2_01_easy",
    "V2_02_medium",
]

IMG_WIDTH = 140


@dataclass
class RunCell:
    rmse: float | None = None
    needs_rerun: bool = False
    traj_xy: Path | None = None
    gt_coverage_pct: float | None = None
    total_poses: int = 0


def _read_json(path: Path) -> dict | None:
    if not path.exists():
        return None
    try:
        return json.loads(path.read_text())
    except Exception:
        return None


def load_tum_xyz(path: Path) -> tuple[list[float], list[tuple[float, float, float]]]:
    """Return sorted timestamps and xyz tuples from a TUM file."""
    stamps: list[float] = []
    xyz: list[tuple[float, float, float]] = []
    if not path.is_file():
        return stamps, xyz
    for line in path.read_text().splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        parts = line.split()
        if len(parts) < 4:
            continue
        stamps.append(float(parts[0]))
        xyz.append((float(parts[1]), float(parts[2]), float(parts[3])))
    if not stamps:
        return stamps, xyz
    order = sorted(range(len(stamps)), key=stamps.__getitem__)
    return [stamps[i] for i in order], [xyz[i] for i in order]


def path_length_xyz(xyz: list[tuple[float, float, float]]) -> float:
    if len(xyz) < 2:
        return 0.0
    total = 0.0
    for a, b in zip(xyz, xyz[1:]):
        dx = b[0] - a[0]
        dy = b[1] - a[1]
        dz = b[2] - a[2]
        total += (dx * dx + dy * dy + dz * dz) ** 0.5
    return total


def match_timestamp_indices(gt_ts: list[float], est_ts: list[float], max_diff_s: float) -> list[tuple[int, int]]:
    """Greedy timestamp association (same idea as select_trajectory_examples.py)."""
    matches: list[tuple[int, int]] = []
    i = 0
    j = 0
    while i < len(gt_ts) and j < len(est_ts):
        delta = est_ts[j] - gt_ts[i]
        if abs(delta) <= max_diff_s:
            best_i = i
            best_abs = abs(delta)
            k = i + 1
            while k < len(gt_ts) and gt_ts[k] <= est_ts[j] + max_diff_s:
                candidate = abs(est_ts[j] - gt_ts[k])
                if candidate < best_abs:
                    best_abs = candidate
                    best_i = k
                k += 1
            matches.append((best_i, j))
            i = best_i + 1
            j += 1
        elif gt_ts[i] < est_ts[j] - max_diff_s:
            i += 1
        else:
            j += 1
    return matches


def gt_path_coverage_pct(gt_path: Path, est_path: Path, max_diff_s: float = GT_MATCH_MAX_DIFF_S) -> float | None:
    """
    Fraction of GT path arc-length that is covered by time-aligned estimate poses.
    Low values usually mean the run died early or produced a very short trajectory.
    """
    gt_ts, gt_xyz = load_tum_xyz(gt_path)
    est_ts, est_xyz = load_tum_xyz(est_path)
    if len(gt_ts) < 2 or len(est_ts) < 2:
        return None
    matches = match_timestamp_indices(gt_ts, est_ts, max_diff_s)
    if len(matches) < 2:
        return 0.0
    gt_total = path_length_xyz(gt_xyz)
    if gt_total <= 0.0:
        return None
    gt_idx = sorted({gi for gi, _ in matches})
    gt_covered = path_length_xyz([gt_xyz[i] for i in gt_idx])
    return 100.0 * gt_covered / gt_total


def find_traj_xy(run_dir: Path, meta: dict | None) -> Path | None:
    if meta:
        rel = meta.get("artifacts", {}).get("traj_xy_png") or meta.get("traj_xy_png")
        if rel:
            candidate = run_dir / rel
            if candidate.is_file():
                return candidate
    direct = run_dir / "traj_xy.png"
    return direct if direct.is_file() else None


def load_batch(batch_dir: Path) -> dict[str, dict[int, RunCell]]:
    """Return {sequence: {run_number: RunCell}}."""
    data: dict[str, dict[int, RunCell]] = {seq: {} for seq in SEQUENCES}
    for seq in SEQUENCES:
        seq_dir = batch_dir / seq
        if not seq_dir.is_dir():
            continue
        for run_dir in sorted(seq_dir.glob("run_*")):
            if not run_dir.is_dir():
                continue
            try:
                run_num = int(run_dir.name.split("_")[1])
            except (IndexError, ValueError):
                continue
            meta = _read_json(run_dir / "run_meta.json")
            ape = _read_json(run_dir / "ape_stats.json")
            rmse = None
            needs_rerun = False
            total_poses = 0
            for src in (meta, ape):
                if src:
                    if "rmse" in src and rmse is None:
                        rmse = float(src["rmse"])
                    if "needs_rerun" in src:
                        needs_rerun = bool(src["needs_rerun"])
                    if "total_poses" in src:
                        total_poses = int(src["total_poses"])
            tum = run_dir / "trajectory.tum"
            gt_file = GT_DIR / f"{seq}.txt"
            gt_cov = None
            if tum.is_file() and gt_file.is_file():
                gt_cov = gt_path_coverage_pct(gt_file, tum)
            data[seq][run_num] = RunCell(
                rmse=rmse,
                needs_rerun=needs_rerun,
                traj_xy=find_traj_xy(run_dir, meta),
                gt_coverage_pct=gt_cov,
                total_poses=total_poses,
            )
    return data


def detect_max_runs(data: dict[str, dict[int, RunCell]]) -> int:
    runs: set[int] = set()
    for seq_runs in data.values():
        runs.update(seq_runs.keys())
    return max(runs) if runs else 0


def _seq_numeric_stats(vals: list[float]) -> tuple[float | None, float | None, float | None, int]:
    if not vals:
        return None, None, None, 0
    mean = statistics.mean(vals)
    median = statistics.median(vals)
    std = statistics.pstdev(vals) if len(vals) > 1 else 0.0
    return mean, median, std, len(vals)


def seq_stats(cells: dict[int, RunCell]) -> tuple[float | None, float | None, float | None, int]:
    vals = [c.rmse for c in cells.values() if c.rmse is not None]
    return _seq_numeric_stats(vals)


def seq_coverage_stats(cells: dict[int, RunCell]) -> tuple[float | None, float | None, float | None, int]:
    vals = [c.gt_coverage_pct for c in cells.values() if c.gt_coverage_pct is not None]
    return _seq_numeric_stats(vals)


def coverage_style(pct: float | None) -> str:
    if pct is None:
        return ""
    if pct < 50.0:
        return ' style="color:#c0392b;font-weight:bold"'
    if pct < 90.0:
        return ' style="color:#d35400"'
    return ""


def rel_link(report_path: Path, image_path: Path) -> str:
    """POSIX relative path from the report file to the image."""
    import os

    return os.path.relpath(image_path.resolve(), report_path.parent.resolve()).replace("\\", "/")


def visual_cell(cell: RunCell | None, report_path: Path) -> str:
    if cell is None or (cell.rmse is None and cell.traj_xy is None):
        return "<div align=\"center\">n/a</div>"

    parts: list[str] = ['<div align="center">']
    if cell.traj_xy and cell.traj_xy.is_file():
        href = rel_link(report_path, cell.traj_xy.resolve())
        alt = cell.traj_xy.parent.name
        parts.append(f'<img src="{href}" width="{IMG_WIDTH}" alt="{alt}"/>')
    else:
        parts.append("<i>no traj_xy</i>")

    if cell.rmse is not None:
        warn = " ⚠" if cell.needs_rerun else ""
        parts.append(f"<br/><b>{cell.rmse:.4f} m</b>{warn}")
    else:
        parts.append("<br/>RMSE n/a")

    if cell.gt_coverage_pct is not None:
        style = coverage_style(cell.gt_coverage_pct)
        parts.append(f'<br/><span{style}>GT path: {cell.gt_coverage_pct:.1f}%</span>')
        if cell.total_poses:
            parts.append(f"<br/><small>{cell.total_poses} poses</small>")
    else:
        parts.append("<br/>GT path: n/a")

    parts.append("</div>")
    return "".join(parts)


def rmse_only_cell(cell: RunCell | None) -> str:
    if cell is None or cell.rmse is None:
        return "n/a"
    marker = " ⚠" if cell.needs_rerun else ""
    return f"{cell.rmse:.4f}{marker}"


def coverage_only_cell(cell: RunCell | None) -> str:
    if cell is None or cell.gt_coverage_pct is None:
        return "n/a"
    return f"{cell.gt_coverage_pct:.1f}%"


def build_report(batch_dir: Path, report_path: Path) -> str:
    data = load_batch(batch_dir)
    max_runs = detect_max_runs(data)
    if max_runs == 0:
        return f"# Polling Report — {batch_dir.name}\n\nNo runs found.\n"

    run_cols = list(range(1, max_runs + 1))
    batch_name = batch_dir.name
    now = datetime.now(UTC_PLUS_4).strftime("%Y-%m-%d %H:%M UTC+4")

    lines: list[str] = [
        f"# Polling Report — {batch_name}",
        "",
        f"Generated: {now}  ",
        f"Batch: `{batch_name}`  ",
        f"Sequences: {len(SEQUENCES)} · Runs per sequence: up to {max_runs}  ",
        "",
        "Each cell: **traj_xy**, **APE RMSE**, and **GT path covered %** (arc-length of GT trajectory matched in time).  ",
        "Low GT path % (&lt;90% orange, &lt;50% red) usually means the run died early or barely moved.  ",
        "⚠ = `needs_rerun` in run_meta.json",
        "",
        "## Trajectory grid (sequence × run)",
        "",
        "<!-- HTML in table cells for image + caption; open in VS Code / GitHub -->",
        "",
    ]

    header_cols = ["Sequence"] + [f"Run {r}" for r in run_cols]
    lines.append("| " + " | ".join(header_cols) + " |")
    lines.append("| " + " | ".join(["---"] * len(header_cols)) + " |")

    for seq in SEQUENCES:
        row = [f"**{seq}**"]
        for r in run_cols:
            row.append(visual_cell(data[seq].get(r), report_path))
        lines.append("| " + " | ".join(row) + " |")

    lines.append("")

    # ── GT path coverage summary ─────────────────────────────────────────────
    lines.append("## GT path coverage summary (sequence × run)")
    lines.append("")
    cov_header = ["Sequence"] + [f"Run {r}" for r in run_cols] + ["Mean", "Median", "Std", "Valid"]
    lines.append("| " + " | ".join(cov_header) + " |")
    lines.append("| " + " | ".join(["---"] * len(cov_header)) + " |")
    for seq in SEQUENCES:
        cov_mean, cov_median, cov_std, cov_n = seq_coverage_stats(data[seq])
        row = [seq]
        for r in run_cols:
            row.append(coverage_only_cell(data[seq].get(r)))
        row.append(f"{cov_mean:.1f}%" if cov_mean is not None else "n/a")
        row.append(f"{cov_median:.1f}%" if cov_median is not None else "n/a")
        row.append(f"{cov_std:.1f}%" if cov_std is not None else "n/a")
        row.append(str(cov_n))
        lines.append("| " + " | ".join(row) + " |")
    lines.append("")

    # ── Mean / std summary (numeric table) ───────────────────────────────────
    lines.append("## RMSE summary (mean, median, std across runs)")
    lines.append("")
    summary_header = ["Sequence"] + [f"Run {r}" for r in run_cols] + ["Mean", "Median", "Std", "Valid"]
    lines.append("| " + " | ".join(summary_header) + " |")
    lines.append("| " + " | ".join(["---"] * len(summary_header)) + " |")

    for seq in SEQUENCES:
        mean, median, std, n_valid = seq_stats(data[seq])
        row = [seq]
        for r in run_cols:
            row.append(rmse_only_cell(data[seq].get(r)))
        row.append(f"{mean:.4f}" if mean is not None else "n/a")
        row.append(f"{median:.4f}" if median is not None else "n/a")
        row.append(f"{std:.4f}" if std is not None else "n/a")
        row.append(str(n_valid))
        lines.append("| " + " | ".join(row) + " |")

    lines.append("")

    # ── Per-run column summary ───────────────────────────────────────────────
    lines.append("## Per-run column summary (mean / median RMSE across sequences)")
    lines.append("")
    run_header = ["Metric"] + [f"Run {r}" for r in run_cols]
    lines.append("| " + " | ".join(run_header) + " |")
    lines.append("| " + " | ".join(["---"] * len(run_header)) + " |")

    col_means: list[float | None] = []
    col_medians: list[float | None] = []
    col_valids: list[int] = []
    for r in run_cols:
        vals = [
            data[seq][r].rmse
            for seq in SEQUENCES
            if r in data[seq] and data[seq][r].rmse is not None
        ]
        if vals:
            col_means.append(statistics.mean(vals))
            col_medians.append(statistics.median(vals))
        else:
            col_means.append(None)
            col_medians.append(None)
        col_valids.append(len(vals))

    lines.append(
        "| Mean RMSE | "
        + " | ".join(f"{v:.4f}" if v is not None else "n/a" for v in col_means)
        + " |"
    )
    lines.append(
        "| Median RMSE | "
        + " | ".join(f"{v:.4f}" if v is not None else "n/a" for v in col_medians)
        + " |"
    )
    lines.append("| Valid seqs | " + " | ".join(str(v) for v in col_valids) + " |")
    lines.append("")

    # ── Best run per sequence ────────────────────────────────────────────────
    lines.append("## Best run per sequence")
    lines.append("")
    lines.append("| Sequence | Best Run | RMSE | GT path % |")
    lines.append("| --- | --- | --- | --- |")
    for seq in SEQUENCES:
        valid = {r: c.rmse for r, c in data[seq].items() if c.rmse is not None}
        if valid:
            best_r = min(valid, key=valid.__getitem__)
            cov = data[seq][best_r].gt_coverage_pct
            cov_str = f"{cov:.1f}%" if cov is not None else "n/a"
            lines.append(f"| {seq} | run_{best_r:02d} | {valid[best_r]:.4f} | {cov_str} |")
        else:
            lines.append(f"| {seq} | — | n/a | n/a |")
    lines.append("")

    return "\n".join(lines)


def find_latest_batch() -> Path:
    batches = sorted(
        [d for d in RESULTS_DIR.iterdir() if d.is_dir() and not d.name.startswith(".")],
        key=lambda d: d.name,
    )
    if not batches:
        raise FileNotFoundError(f"No batch folders found under {RESULTS_DIR}")
    return batches[-1]


def pick_report_path(batch_dir: Path) -> Path:
    now = datetime.now(UTC_PLUS_4)
    time_tag = now.strftime("%H_%M")
    for candidate in (batch_dir, ROOT):
        try:
            test = candidate / f".write_test_{time_tag}"
            test.write_text("")
            test.unlink()
            return candidate / f"polling_report_{time_tag}.md"
        except PermissionError:
            continue
    raise PermissionError(f"Cannot write report to {batch_dir} or {ROOT}")


def main() -> None:
    args = sys.argv[1:]

    if "--all" in args:
        batches = sorted(d for d in RESULTS_DIR.iterdir() if d.is_dir())
    elif args and not args[0].startswith("--"):
        batch_name = args[0]
        batches = [RESULTS_DIR / batch_name]
        if not batches[0].is_dir():
            print(f"ERROR: batch '{batch_name}' not found under {RESULTS_DIR}")
            sys.exit(1)
    else:
        batches = [find_latest_batch()]

    for batch_dir in batches:
        out = pick_report_path(batch_dir)
        content = build_report(batch_dir, out)
        out.write_text(content)
        print(f"Saved: {out}")
        print(content)


if __name__ == "__main__":
    main()
