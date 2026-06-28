#!/usr/bin/env python3
from __future__ import annotations

import csv
import json
import shutil
from dataclasses import dataclass
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
HOME = Path.home()

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

GT_ROOT = ROOT / "datasets" / "gt"
ORB2_RESULTS_ROOT = ROOT / "results" / "20260530_232822"
RTAB_RESULTS_ROOT = HOME / "RTAB_Docker" / "results"
RTAB_BATCH_BY_SEQUENCE = {
    "MH_01_easy": "20260525_012112",
    "MH_05_difficult": "20260525_074355",
    "V1_02_medium": "20260525_074355",
    "V1_03_difficult": "20260525_074355",
}
RTAB_DEFAULT_BATCH = "20260525_013838"

OUT_DIR = ROOT / "images"
SUMMARY_CSV = OUT_DIR / "trajectory_selection_summary.csv"
SUMMARY_JSON = OUT_DIR / "trajectory_selection_summary.json"


@dataclass
class RunCandidate:
    algorithm: str
    sequence: str
    batch_id: str
    run: int
    rmse_m: float
    gt_coverage_ratio: float
    length_match_ratio: float
    trajectory_image: Path

    @property
    def gt_coverage_pct(self) -> float:
        return self.gt_coverage_ratio * 100.0

    @property
    def length_match_pct(self) -> float:
        return self.length_match_ratio * 100.0


def load_tum(path: Path) -> np.ndarray:
    rows: list[list[float]] = []
    for line in path.read_text().splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        parts = line.split()
        if len(parts) < 8:
            continue
        rows.append([float(value) for value in parts[:8]])
    if not rows:
        return np.empty((0, 8), dtype=float)
    array = np.asarray(rows, dtype=float)
    order = np.argsort(array[:, 0])
    return array[order]


def path_length(points_xyz: np.ndarray) -> float:
    if len(points_xyz) < 2:
        return 0.0
    return float(np.linalg.norm(np.diff(points_xyz, axis=0), axis=1).sum())


def match_indices(gt_ts: np.ndarray, est_ts: np.ndarray, max_diff_s: float = 0.1) -> list[tuple[int, int]]:
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


def trajectory_metrics(gt_path: Path, est_path: Path) -> tuple[float, float]:
    gt = load_tum(gt_path)
    est = load_tum(est_path)
    if len(gt) < 2 or len(est) < 2:
        return 0.0, 0.0

    matches = match_indices(gt[:, 0], est[:, 0])
    if len(matches) < 2:
        return 0.0, 0.0

    gt_idx = np.asarray([gt_i for gt_i, _ in matches], dtype=int)
    est_idx = np.asarray([est_i for _, est_i in matches], dtype=int)

    gt_total_len = path_length(gt[:, 1:4])
    gt_covered_len = path_length(gt[gt_idx, 1:4])
    est_covered_len = path_length(est[est_idx, 1:4])

    gt_coverage_ratio = gt_covered_len / gt_total_len if gt_total_len else 0.0
    if gt_covered_len <= 0.0 or est_covered_len <= 0.0:
        length_match_ratio = 0.0
    else:
        length_match_ratio = min(gt_covered_len, est_covered_len) / max(gt_covered_len, est_covered_len)
    return gt_coverage_ratio, length_match_ratio


def load_orb2_candidates(sequence: str) -> list[RunCandidate]:
    seq_dir = ORB2_RESULTS_ROOT / sequence
    payload = json.loads((seq_dir / "run_summary.json").read_text())
    gt_path = GT_ROOT / f"{sequence}.txt"
    candidates: list[RunCandidate] = []
    for run in payload["runs"]:
        run_id = int(run["run"])
        run_dir = seq_dir / f"run_{run_id:02d}"
        coverage_ratio, length_match_ratio = trajectory_metrics(gt_path, run_dir / "trajectory.tum")
        candidates.append(
            RunCandidate(
                algorithm="ORB-SLAM2",
                sequence=sequence,
                batch_id="20260530_232822",
                run=run_id,
                rmse_m=float(run["rmse"]),
                gt_coverage_ratio=coverage_ratio,
                length_match_ratio=length_match_ratio,
                trajectory_image=run_dir / "traj_xy_trajectories.png",
            )
        )
    return candidates


def load_rtab_candidates(sequence: str) -> list[RunCandidate]:
    batch_id = RTAB_BATCH_BY_SEQUENCE.get(sequence, RTAB_DEFAULT_BATCH)
    seq_dir = RTAB_RESULTS_ROOT / batch_id / sequence
    payload = json.loads((seq_dir / "run_summary.json").read_text())
    gt_path = GT_ROOT / f"{sequence}.txt"
    candidates: list[RunCandidate] = []
    for run in payload["runs"]:
        run_id = int(run["run"])
        run_dir = seq_dir / f"run_{run_id:02d}"
        coverage_ratio, length_match_ratio = trajectory_metrics(gt_path, run_dir / "trajectory.tum")
        candidates.append(
            RunCandidate(
                algorithm="RTAB-Map",
                sequence=sequence,
                batch_id=batch_id,
                run=run_id,
                rmse_m=float(run["rmse"]),
                gt_coverage_ratio=coverage_ratio,
                length_match_ratio=length_match_ratio,
                trajectory_image=run_dir / "traj_xy_trajectories.png",
            )
        )
    return candidates


def choose_candidate(candidates: list[RunCandidate]) -> tuple[RunCandidate, str]:
    eligible = [candidate for candidate in candidates if candidate.gt_coverage_ratio >= 0.90]
    if eligible:
        selected = sorted(
            eligible,
            key=lambda candidate: (
                candidate.rmse_m,
                -candidate.length_match_ratio,
                -candidate.gt_coverage_ratio,
                candidate.run,
            ),
        )[0]
        return selected, "coverage>=90_lowest_rmse"

    selected = sorted(
        candidates,
        key=lambda candidate: (
            -candidate.gt_coverage_ratio,
            -candidate.length_match_ratio,
            candidate.rmse_m,
            candidate.run,
        ),
    )[0]
    return selected, "highest_coverage_fallback"


def copy_selected_image(candidate: RunCandidate) -> str:
    stem = "orbslam2" if candidate.algorithm == "ORB-SLAM2" else "rtabmap"
    destination = OUT_DIR / f"{stem}_{candidate.sequence}_trajectory_xy.png"
    destination.parent.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(candidate.trajectory_image, destination)
    return str(destination.relative_to(ROOT))


def main() -> None:
    rows: list[dict[str, object]] = []
    for sequence in SEQUENCES:
        for loader in (load_orb2_candidates, load_rtab_candidates):
            candidates = loader(sequence)
            selected, reason = choose_candidate(candidates)
            copied_image = copy_selected_image(selected)
            rows.append(
                {
                    "algorithm": selected.algorithm,
                    "sequence": selected.sequence,
                    "batch_id": selected.batch_id,
                    "selected_run": selected.run,
                    "selection_reason": reason,
                    "rmse_m": round(selected.rmse_m, 6),
                    "gt_coverage_pct": round(selected.gt_coverage_pct, 2),
                    "length_match_pct": round(selected.length_match_pct, 2),
                    "source_image": str(selected.trajectory_image),
                    "copied_image": copied_image,
                }
            )

    with SUMMARY_CSV.open("w", newline="") as handle:
        writer = csv.DictWriter(
            handle,
            fieldnames=[
                "algorithm",
                "sequence",
                "batch_id",
                "selected_run",
                "selection_reason",
                "rmse_m",
                "gt_coverage_pct",
                "length_match_pct",
                "source_image",
                "copied_image",
            ],
        )
        writer.writeheader()
        writer.writerows(rows)

    SUMMARY_JSON.write_text(json.dumps(rows, indent=2) + "\n")


if __name__ == "__main__":
    main()
