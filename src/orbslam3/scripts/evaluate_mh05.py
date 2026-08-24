#!/usr/bin/env python3

"""
Evaluate MH05 trajectories against the canonical gt.tum ground truth.

This helper treats keyframe APE as the primary metric and camera APE as
secondary. It can consume RunMetadata_*.txt files emitted by the wrapper or
trajectory files directly.
"""

import argparse
import subprocess
import sys
from pathlib import Path


def parse_metadata(path: Path):
    result = {}
    for line in path.read_text().splitlines():
        if ":" not in line:
            continue
        key, value = line.split(":", 1)
        result[key.strip()] = value.strip()
    return result


def resolve_run(path: Path):
    if path.name.startswith("RunMetadata_"):
        metadata = parse_metadata(path)
        keyframe = Path(metadata.get("keyframe_trajectory_file", ""))
        camera = Path(metadata.get("camera_trajectory_file", ""))
        if not keyframe.is_absolute():
            keyframe = path.parent / keyframe
        if not camera.is_absolute():
            camera = path.parent / camera
        label = metadata.get("bag_label") or path.stem
        return {
            "label": label,
            "metadata": path,
            "keyframe": keyframe,
            "camera": camera,
        }

    if path.name.startswith("KeyFrameTrajectory_"):
        suffix = path.name.replace("KeyFrameTrajectory_", "")
        return {
            "label": path.stem,
            "metadata": None,
            "keyframe": path,
            "camera": path.parent / f"CameraTrajectory_{suffix}",
        }

    if path.name.startswith("CameraTrajectory_"):
        suffix = path.name.replace("CameraTrajectory_", "")
        return {
            "label": path.stem,
            "metadata": None,
            "keyframe": path.parent / f"KeyFrameTrajectory_{suffix}",
            "camera": path,
        }

    raise ValueError(f"Unsupported run reference: {path}")


def run_evo(gt_path: Path, trajectory_path: Path, label: str, secondary=False):
    kind = "camera" if secondary else "keyframe"
    print(f"\n=== {label} ({kind}) ===")
    command = ["evo_ape", "tum", str(gt_path), str(trajectory_path), "-va", "--align"]
    completed = subprocess.run(command, check=False, text=True, capture_output=True)
    if completed.stdout:
        print(completed.stdout.rstrip())
    if completed.stderr:
        print(completed.stderr.rstrip(), file=sys.stderr)
    return completed.returncode


def main():
    parser = argparse.ArgumentParser(description="Evaluate MH05 trajectories with evo_ape")
    parser.add_argument(
        "runs",
        nargs="*",
        help="RunMetadata_*.txt or trajectory files. If omitted, the latest RunMetadata file is used.",
    )
    parser.add_argument(
        "--repo-root",
        default=str(Path(__file__).resolve().parents[2]),
        help="Repository root used to resolve the canonical gt.tum path.",
    )
    args = parser.parse_args()

    repo_root = Path(args.repo_root).resolve()
    gt_path = repo_root / "datasets" / "euroc" / "MH_05_difficult" / "gt.tum"
    if not gt_path.exists():
        raise FileNotFoundError(f"Canonical MH05 ground truth missing: {gt_path}")

    run_paths = [Path(run).resolve() for run in args.runs]
    if not run_paths:
        metadata_candidates = sorted(repo_root.glob("RunMetadata_*.txt"))
        if not metadata_candidates:
            raise FileNotFoundError("No RunMetadata_*.txt files found and no runs were provided.")
        run_paths = [metadata_candidates[-1]]

    overall_rc = 0
    for run_path in run_paths:
        run = resolve_run(run_path)

        if not run["keyframe"].exists():
            raise FileNotFoundError(f"Missing keyframe trajectory: {run['keyframe']}")

        overall_rc |= run_evo(gt_path, run["keyframe"], run["label"], secondary=False)

        if run["camera"].exists():
            overall_rc |= run_evo(gt_path, run["camera"], run["label"], secondary=True)

    raise SystemExit(overall_rc)


if __name__ == "__main__":
    main()
