#!/usr/bin/env python3
"""Build 10x10 (sequence x run) trajectory-overlay grids for OV2SLAM vs ground truth.

Two composite figures are produced, each a single image (no LaTeX cell squashing):
  * stereo: GT + OV2SLAM Accurate Stereo + OV2SLAM Fast Stereo
  * mono  : GT + OV2SLAM Accurate Mono   + OV2SLAM Fast Mono

Every subplot uses equal aspect ratio (so the real sequence X:Y shape is preserved,
not squashed), a light grid, and the whole figure shares one legend. Estimates are
Umeyama-aligned to GT before plotting: SE(3) for stereo, Sim(3) (with scale) for
mono, matching how the APE RMSEs were computed.
"""
from __future__ import annotations

import math
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

REPO = Path(__file__).resolve().parent.parent
OUT_DIR = REPO / "SLAM_Images"

SEQUENCES = [
    "MH_01_easy", "MH_02_easy", "MH_03_medium", "MH_04_difficult", "MH_05_difficult",
    "V1_01_easy", "V1_02_medium", "V1_03_difficult", "V2_01_easy", "V2_02_medium",
]
RUNS = list(range(1, 11))
T_MAX_DIFF = 0.1

GT_COLOR = "#7f7f7f"
ACC_COLOR = "#1f77b4"   # blue
FAST_COLOR = "#d62728"  # red

# Each mode: (accurate source, fast source, align-with-scale?)
#   base/<sequence>/run_NN/<fname>
MODES = {
    "stereo": {
        "title": "OV$^2$SLAM Stereo vs. Ground Truth",
        "use_scale": False,
        "acc": {
            "base": REPO / "results/ov2slam_benchmark_statistical/20260605_204652",
            "fname": "ov2slam_kfs_traj.txt", "label": "Accurate Stereo",
        },
        "fast": {
            "base": REPO / "results/ov2slam_benchmark_statistical/20260610_005350",
            "fname": "ov2slam_kfs_traj.txt", "label": "Fast Stereo",
        },
    },
    "mono": {
        "title": "OV$^2$SLAM Mono vs. Ground Truth",
        "use_scale": True,
        "acc": {
            "base": REPO / "results/ov2slam_benchmark_v2/20260612_234141/accurate_mono",
            "fname": "trajectory.tum", "label": "Accurate Mono",
        },
        "fast": {
            "base": REPO / "results/ov2slam_benchmark_v2/20260613_102424/fast_mono",
            "fname": "trajectory.tum", "label": "Fast Mono",
        },
    },
}


def load_tum_xyz(path: Path):
    rows = []
    for raw in path.read_text(encoding="utf-8").splitlines():
        line = raw.strip()
        if not line or line.startswith("#"):
            continue
        f = line.replace(",", " ").split()
        if len(f) < 8:
            continue
        rows.append((float(f[0]), float(f[1]), float(f[2]), float(f[3])))
    if not rows:
        raise ValueError(f"No TUM poses in {path}")
    return rows


def associate(gt_rows, est_rows, max_diff):
    gt_pts, est_pts = [], []
    j = 0
    for gt_ts, gx, gy, gz in gt_rows:
        while j < len(est_rows) and est_rows[j][0] < gt_ts - max_diff:
            j += 1
        if j >= len(est_rows):
            break
        best_idx, best_dt, k = None, math.inf, j
        while k < len(est_rows) and est_rows[k][0] <= gt_ts + max_diff:
            dt = abs(est_rows[k][0] - gt_ts)
            if dt < best_dt:
                best_dt, best_idx = dt, k
            k += 1
        if best_idx is None:
            continue
        _, ex, ey, ez = est_rows[best_idx]
        gt_pts.append([gx, gy, gz])
        est_pts.append([ex, ey, ez])
        j = best_idx + 1
    return np.asarray(gt_pts), np.asarray(est_pts)


def umeyama(src, dst, with_scale):
    """Align src -> dst. Returns (scale, R, t)."""
    src_mean, dst_mean = src.mean(0), dst.mean(0)
    src_d, dst_d = src - src_mean, dst - dst_mean
    cov = (dst_d.T @ src_d) / src.shape[0]
    u, s, vt = np.linalg.svd(cov)
    corr = np.eye(3)
    if np.linalg.det(u) * np.linalg.det(vt) < 0:
        corr[-1, -1] = -1.0
    R = u @ corr @ vt
    if with_scale:
        var = np.mean(np.sum(src_d * src_d, axis=1))
        scale = float(np.sum(s * np.diag(corr)) / var) if var > 0 else 1.0
    else:
        scale = 1.0
    t = dst_mean - scale * (R @ src_mean)
    return scale, R, t


def aligned_xy(gt_file: Path, est_file: Path, with_scale: bool):
    gt_rows = load_tum_xyz(gt_file)
    est_rows = load_tum_xyz(est_file)
    gt_assoc, est_assoc = associate(gt_rows, est_rows, T_MAX_DIFF)
    gt_xyz = np.asarray([[x, y, z] for _, x, y, z in gt_rows])
    est_xyz = np.asarray([[x, y, z] for _, x, y, z in est_rows])
    if len(gt_assoc) >= 3:
        scale, R, t = umeyama(est_assoc, gt_assoc, with_scale)
        est_xyz = (scale * (R @ est_xyz.T)).T + t
    return gt_xyz, est_xyz


def build_grid(mode_key: str, cfg: dict) -> None:
    nrows, ncols = len(SEQUENCES), len(RUNS)
    fig, axes = plt.subplots(nrows, ncols, figsize=(1.7 * ncols, 1.7 * nrows), dpi=150)

    for r, seq in enumerate(SEQUENCES):
        for c, run in enumerate(RUNS):
            ax = axes[r][c]
            ax.set_aspect("equal", adjustable="box")
            ax.grid(True, alpha=0.3, linewidth=0.4)
            ax.tick_params(left=False, bottom=False, labelleft=False, labelbottom=False)
            run_dir = f"run_{run:02d}"

            gt_file = cfg["acc"]["base"] / seq / run_dir / "gt.tum"
            if not gt_file.is_file():
                gt_file = cfg["fast"]["base"] / seq / run_dir / "gt.tum"
            gt_drawn = False
            for src, color in ((cfg["acc"], ACC_COLOR), (cfg["fast"], FAST_COLOR)):
                est_file = src["base"] / seq / run_dir / src["fname"]
                if not est_file.is_file() or not gt_file.is_file():
                    continue
                try:
                    gt_xyz, est_xyz = aligned_xy(gt_file, est_file, cfg["use_scale"])
                except Exception:
                    continue
                if not gt_drawn:
                    ax.plot(gt_xyz[:, 0], gt_xyz[:, 1], "--", color=GT_COLOR,
                            linewidth=1.0, label="Ground Truth")
                    gt_drawn = True
                ax.plot(est_xyz[:, 0], est_xyz[:, 1], color=color, linewidth=0.9,
                        label=src["label"])

            if r == 0:
                ax.set_title(f"Run {run}", fontsize=9)
            if c == 0:
                ax.set_ylabel(seq, fontsize=9, rotation=90, labelpad=6)

    # One shared legend (dedupe handles/labels across all subplots).
    handles, labels = [], []
    for axrow in axes:
        for ax in axrow:
            for h, l in zip(*ax.get_legend_handles_labels()):
                if l not in labels:
                    handles.append(h)
                    labels.append(l)
    fig.legend(handles, labels, loc="upper center", ncol=3, fontsize=12,
               frameon=True, bbox_to_anchor=(0.5, 1.0))
    fig.suptitle(cfg["title"], fontsize=15, y=1.015)
    fig.tight_layout(rect=(0, 0, 1, 0.985))

    OUT_DIR.mkdir(parents=True, exist_ok=True)
    for ext in ("png", "pdf"):
        out = OUT_DIR / f"traj_overlay_grid_{mode_key}.{ext}"
        fig.savefig(out, bbox_inches="tight")
        print(f"wrote {out}")
    plt.close(fig)


def main() -> None:
    for mode_key, cfg in MODES.items():
        build_grid(mode_key, cfg)


if __name__ == "__main__":
    main()
