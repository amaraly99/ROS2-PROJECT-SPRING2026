#!/usr/bin/env python3
"""ctrl_session_summary.py — summary of one ctrl_runner session, camera publisher OFF vs ON.

    python3 benchmarks/ctrl_session_summary.py reports/ctrl_<tag>_<stamp>

Reads <session>/runs.csv and, per run, metrics.csv + run_check.json. For each camera
condition: valid runs k/n, then mean ± std over the VALID runs of the controller
metrics, the message timing at the Pi and the Wi-Fi receive rate. Writes
summary.csv and summary.md into the session dir.
"""
import csv
import json
import math
import statistics as stats
import sys
from pathlib import Path

CONTROL = [("settling_time_s", "settling time (s)"), ("steady_state_err_m", "steady-state error (m)"),
           ("overshoot_m", "overshoot (m)"), ("IAE", "IAE (m·s)"), ("path_efficiency", "path efficiency"),
           ("rms_cmd_speed", "RMS command speed (m/s)"), ("total_variation", "total variation (m/s)"),
           ("cpu_mean_pct", "controller CPU (% of a core)"), ("mem_rss_mb", "controller memory (MB)")]
TIMING = [("/sim/heartbeat", "gap_p95_ms", "heartbeat gap p95 (ms)"),
          ("/sim/heartbeat", "gap_max_ms", "heartbeat gap max (ms)"),
          ("/sim/heartbeat", "gaps_over_0.2s", "heartbeat gaps > 0.2 s (count)"),
          ("/sim/drone_pose", "rate_hz", "drone pose rate (Hz)"),
          ("/sim/drone_pose", "gap_p95_ms", "drone pose gap p95 (ms)"),
          ("/cmd_vel", "rate_hz", "cmd_vel rate (Hz)")]


def mean_std(vals):
    vals = [v for v in vals if v is not None and not math.isnan(v)]
    if not vals:
        return float("nan"), float("nan"), 0
    return stats.fmean(vals), (stats.stdev(vals) if len(vals) > 1 else 0.0), len(vals)


def load_run(run_dir):
    run_dir = Path(run_dir)
    out = {}
    m = run_dir / "metrics.csv"
    if m.exists():
        for row in csv.reader(open(m)):
            if len(row) == 2 and row[0] != "metric":
                try:
                    out[row[0]] = float(row[1])
                except ValueError:
                    pass
    meta = run_dir / "meta.txt"
    if meta.exists():
        for line in meta.read_text().splitlines():
            k, _, v = line.partition("=")
            if k in ("runner_cpu_pct_core", "pi_cpu_total_pct", "runner_status_calls"):
                try:
                    out[k] = float(v)
                except ValueError:
                    pass
    c = run_dir / "run_check.json"
    if c.exists():
        chk = json.loads(c.read_text())
        for topic, key, _ in TIMING:
            out[f"{topic}:{key}"] = float(chk.get("timing", {}).get(topic, {}).get(key, float("nan")))
        out["wifi_rx_mean_MBps"] = float(chk.get("wifi", {}).get("rx_mean_MBps", float("nan")))
    return out


def main():
    if len(sys.argv) < 2:
        sys.exit("usage: ctrl_session_summary.py <session_dir>")
    session = Path(sys.argv[1])
    runs = list(csv.DictReader(open(session / "runs.csv")))
    conds = sorted({r["camera_pub"] for r in runs}, key=lambda c: ("off", "on").index(c) if c in ("off", "on") else 9)
    data = {c: [] for c in conds}
    count = {c: [0, 0] for c in conds}
    for r in runs:
        count[r["camera_pub"]][1] += 1
        if r["valid"] == "True" and r["run_dir"]:
            count[r["camera_pub"]][0] += 1
            data[r["camera_pub"]].append(load_run(r["run_dir"]))

    rows = [("valid runs", {c: f"{count[c][0]}/{count[c][1]}" for c in conds})]
    keys = [(k, label) for k, label in CONTROL] + [(f"{t}:{k}", label) for t, k, label in TIMING] + \
           [("wifi_rx_mean_MBps", "Wi-Fi receive rate on the Pi (MB/s)"),
            ("runner_cpu_pct_core", "runner's own CPU (% of a core)"),
            ("pi_cpu_total_pct", "whole Pi CPU during flight (% of all cores)"),
            ("runner_status_calls", "status calls to MATLAB per flight")]
    csv_rows = []
    for key, label in keys:
        cells = {}
        for c in conds:
            mu, sd, n = mean_std([d.get(key) for d in data[c]])
            cells[c] = "—" if n == 0 else f"{mu:.3f} ± {sd:.3f}"
            csv_rows.append([key, c, mu, sd, n])
        rows.append((label, cells))

    with open(session / "summary.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["metric", "camera_pub", "mean", "std", "n_valid"])
        w.writerows(csv_rows)

    head = "| measure | " + " | ".join(f"camera {c.upper()}" for c in conds) + " |"
    sep = "|---|" + "---:|" * len(conds)
    lines = [f"# {session.name}", "", "Mean ± std over the valid runs of each condition.", "", head, sep]
    for label, cells in rows:
        lines.append(f"| {label} | " + " | ".join(cells[c] for c in conds) + " |")
    (session / "summary.md").write_text("\n".join(lines) + "\n")
    print("\n".join(lines))
    return 0


if __name__ == "__main__":
    sys.exit(main())
