#!/usr/bin/env python3
"""slam_session_summary.py — summary of one SLAM session (slam_runner.py), per arm.

    python3 benchmarks/slam_session_summary.py reports/slam_<tag>_<stamp>

Reads <session>/runs.csv and, per run, slam_metrics.csv, run_check.json and meta.txt. For each arm:
valid runs k/n, backend failures (the SLAM did not initialise, or built no map) and rig failures, then
mean ± std over the VALID runs of the accuracy, initialisation, cost, message-timing and Wi-Fi measures.
Writes summary.csv and summary.md into the session dir.
"""
import csv
import json
import math
import statistics as stats
import sys
from pathlib import Path

ACCURACY = [("ape_rmse", "ATE RMSE (m)"), ("ape_median", "ATE median (m)"), ("ape_max", "ATE max (m)"),
            ("umeyama_scale", "scale (Sim3 fit)"), ("scale_drift_pct", "scale drift (%)"),
            ("gt_path_m", "scored path (m)"), ("err_per_m_pct", "error per metre (%)"),
            ("path_ratio_pct", "path length ratio (%)"), ("d_target_m", "end distance to sign (m)"),
            ("n_poses", "poses scored")]
INIT = [("init_time_s", "init time (s)"), ("init_dist_m", "init distance (m)"), ("harness_delay_s", "harness delay (s)"),
        ("ready_margin_s", "ready margin (s)")]
COST = [("fe_ms_mean", "front end (ms, mean)"), ("fe_ms_p95", "front end (ms, p95)"), ("input_rate_hz", "input rate (Hz)"),
        ("capacity_hz", "capacity (Hz)"), ("pose_rate_hz", "pose rate (Hz)"), ("poses_per_frame_pct", "poses per frame (%)"),
        ("cpu_mean_pct", "SLAM CPU mean (% of a core)"), ("cpu_max_pct", "SLAM CPU max (% of a core)"),
        ("mem_max_mb", "SLAM memory max (MiB, PSS)"), ("mem_mean_mb", "SLAM memory mean (MiB, PSS)"),
        ("mem_peak_mb", "SLAM memory peak (MiB, sum of VmHWM)")]
TIMING = [("/sim/heartbeat", "rate_hz", "heartbeat rate (Hz)"),
          ("/sim/heartbeat", "gap_p95_ms", "heartbeat gap p95 (ms)"),
          ("/sim/heartbeat", "gap_max_ms", "heartbeat gap max (ms)"),
          ("/sim/heartbeat", "gaps_over_0.2s", "heartbeat gaps > 0.2 s (count)"),
          ("/sim/drone_pose", "rate_hz", "drone pose rate (Hz)"),
          ("/slam/pose", "rate_hz", "/slam/pose rate (Hz)")]


def mean_std(vals):
    vals = [v for v in vals if v is not None and not math.isnan(v)]
    if not vals:
        return float("nan"), float("nan"), 0
    return stats.fmean(vals), (stats.stdev(vals) if len(vals) > 1 else 0.0), len(vals)


def to_float(text):
    try:
        return float(text)
    except (TypeError, ValueError):
        return None


def load_run(run_dir):
    """All numbers of one run in one dict (metric name -> float)."""
    run_dir = Path(run_dir)
    out = {}
    m = run_dir / "slam_metrics.csv"
    if m.exists():
        for row in csv.reader(open(m)):
            if len(row) == 2 and row[0] != "metric":
                v = to_float(row[1])
                if v is not None:
                    out[row[0]] = v
    meta = run_dir / "meta.txt"
    if meta.exists():
        for line in meta.read_text().splitlines():
            k, _, v = line.partition("=")
            if k in ("runner_cpu_pct_core", "pi_cpu_total_pct", "runner_status_calls", "probe_sec", "slam_poses"):
                f = to_float(v)
                if f is not None:
                    out[k] = f
    c = run_dir / "run_check.json"
    if c.exists():
        chk = json.loads(c.read_text())
        for topic, key, _ in TIMING:
            v = chk.get("timing", {}).get(topic, {}).get(key)
            out[f"{topic}:{key}"] = float("nan") if v is None else float(v)
        out["wifi_rx_mean_MBps"] = float(chk.get("wifi", {}).get("rx_mean_MBps", float("nan")))
    return out


def session_facts(runs):
    """camera mode and baseline from the first run that has a meta.txt."""
    for r in runs:
        meta = Path(r["run_dir"]) / "meta.txt" if r.get("run_dir") else None
        if meta and meta.exists():
            facts = dict(line.partition("=")[::2] for line in meta.read_text().splitlines())
            base = facts.get("stereo_baseline_m", "None")
            return facts.get("camera_mode", "?") + ("" if base in ("None", "") else f" {base} m")
    return "?"


def main():
    if len(sys.argv) < 2:
        sys.exit("usage: slam_session_summary.py <session_dir>")
    session = Path(sys.argv[1])
    runs = list(csv.DictReader(open(session / "runs.csv")))
    arms = list(dict.fromkeys(r["arm"] for r in runs))
    data = {a: [] for a in arms}
    n_valid = {a: 0 for a in arms}
    n_all = {a: 0 for a in arms}
    n_backend = {a: 0 for a in arms}
    n_rig = {a: 0 for a in arms}
    notes = {a: [] for a in arms}
    for r in runs:
        a = r["arm"]
        n_all[a] += 1
        outcome = r.get("outcome", "")
        if outcome == "valid" and r["run_dir"]:
            n_valid[a] += 1
            data[a].append(load_run(r["run_dir"]))
        elif outcome == "backend_failed":
            n_backend[a] += 1
            notes[a].append(f"run {r['run']}: backend failed - {r['reason']}")
        else:
            n_rig[a] += 1
            notes[a].append(f"run {r['run']}: rig failed - {r['reason']}")

    rows = [("valid runs", {a: f"{n_valid[a]}/{n_all[a]}" for a in arms}),
            ("backend failures (no init, no map)", {a: str(n_backend[a]) for a in arms}),
            ("rig failures (not comparable)", {a: str(n_rig[a]) for a in arms})]
    keys = []
    for group in (ACCURACY, INIT, COST):
        keys += list(group)
    keys += [(f"{t}:{k}", label) for t, k, label in TIMING]
    keys += [("wifi_rx_mean_MBps", "Wi-Fi receive rate on the Pi (MB/s)"), ("probe_sec", "probe flight (s)"),
             ("runner_cpu_pct_core", "runner's own CPU (% of a core)"),
             ("pi_cpu_total_pct", "whole Pi CPU during flight (% of all cores)"),
             ("runner_status_calls", "status calls to MATLAB per flight")]
    csv_rows = []
    for key, label in keys:
        cells = {}
        for a in arms:
            mu, sd, n = mean_std([d.get(key) for d in data[a]])
            cells[a] = "—" if n == 0 else f"{mu:.3f} ± {sd:.3f}"
            csv_rows.append([key, a, mu, sd, n])
        if any(c != "—" for c in cells.values()):
            rows.append((label, cells))
    for label, vals in rows[:3]:
        for a in arms:
            csv_rows.append([label, a, vals[a], "", ""])

    with open(session / "summary.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["metric", "arm", "mean", "std", "n_valid"])
        w.writerows(csv_rows)

    head = "| measure | " + " | ".join(arms) + " |"
    sep = "|---|" + "---:|" * len(arms)
    lines = [f"# {session.name}", "", f"Camera mode: {session_facts(runs)}. Mean ± std over the valid runs of each arm.", "",
             head, sep]
    for label, cells in rows:
        lines.append(f"| {label} | " + " | ".join(cells[a] for a in arms) + " |")
    if any(notes.values()):
        lines += ["", "## Runs that are not valid", ""]
        for a in arms:
            for n in notes[a]:
                lines.append(f"- {a}: {n}")
    (session / "summary.md").write_text("\n".join(lines) + "\n")
    print("\n".join(lines))
    return 0


if __name__ == "__main__":
    sys.exit(main())
