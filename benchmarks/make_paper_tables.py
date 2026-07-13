#!/usr/bin/env python3
"""make_paper_tables.py — aggregate HIL runs into the paper's latency and
placement tables (LaTeX-ready rows).

Produces three artifacts under --out-dir:
  table_1a_latency.tex    Table tab:latency_results (baseline Config A, 4 stages + E2E)
  table_1b_perstage.tex   per-stage x per-config P50 (Configs A/B/C/D) + /cmd_vel Hz
  table_placement.tex     Table tab:placement rows (Det.lat / /cmd_vel Hz / Centroid RMS)

Each configuration is identified by a run *label* (the --label passed to
run_hil_experiments.sh). Runs are discovered as {dir}/{label}_run*_telemetry.csv
across every --results-dir. Stage grouping reuses latency_stage_report so the
join logic is identical to the per-run reports. See benchmarks/LATENCY_STAGES.md.

Config → label mapping (defaults match the existing artifacts):
  A  yolo26n_npu_fast   (baseline; existing)
  B  yolo26n_cpu_fast   (CPU infer;  existing, detector sweep)
  C  placement_C        (deferred HIL run)
  D  placement_D        (deferred HIL run)
Override any with --labelA/--labelB/--labelC/--labelD.

No fresh runs, no ROS2, no MATLAB — pure post-processing of collected CSV/JSON.
"""

import argparse
import glob
import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from latency_stage_report import (  # noqa: E402
    _read_csv, _dist, paper_stage_series, PAPER_STAGE_LABELS)


# ── discovery + aggregation ──────────────────────────────────────────────────
def find_runs(dirs, label):
    """Sorted run prefixes (…/{label}_run{N}) that have producer telemetry."""
    prefixes = {}
    for d in dirs:
        for p in sorted(glob.glob(os.path.join(d, f"{label}_run*_telemetry.csv"))):
            pref = p[: -len("_telemetry.csv")]
            prefixes[os.path.basename(pref)] = pref  # de-dup by basename
    return [prefixes[k] for k in sorted(prefixes)]


def pooled_stages(prefixes):
    """Pool paper_stage_series across runs → {stage_key: dist or None}."""
    acc = {k: [] for k, _ in PAPER_STAGE_LABELS}
    for pref in prefixes:
        s = paper_stage_series(_read_csv(pref + "_telemetry.csv"),
                               _read_csv(pref + ".csv"),
                               _read_csv(pref + "_cam_stamps.csv"))
        for k in acc:
            acc[k].extend(s.get(k, []))
    return {k: _dist(v) for k, v in acc.items()}


def centroid_rms(prefixes, states=None):
    """Per-run centroid RMS (px): sqrt(mean(err^2)) per run, then mean±std.

    states: optional set of state names to restrict to (e.g. tracking only).
    Returns dict or None if no visp telemetry present.
    """
    per_run = []
    total = 0
    for pref in prefixes:
        errs = []
        for r in _read_csv(pref + "_visp.csv"):
            if states and (r.get("state") or "").upper() not in states:
                continue
            try:
                errs.append(float(r["centroid_err_px"]))
            except (KeyError, ValueError):
                pass
        if errs:
            per_run.append(math.sqrt(sum(e * e for e in errs) / len(errs)))
            total += len(errs)
    if not per_run:
        return None
    m = sum(per_run) / len(per_run)
    sd = math.sqrt(sum((x - m) ** 2 for x in per_run) / len(per_run)) if len(per_run) > 1 else 0.0
    return {"mean": m, "std": sd, "n_runs": len(per_run), "n_rows": total}


def read_summary(dirs, label):
    for d in dirs:
        p = os.path.join(d, f"{label}_summary.json")
        if os.path.exists(p):
            with open(p) as f:
                return json.load(f)
    return None


def cmdvel_hz(summary):
    """(mean, std) of /cmd_vel Hz from a run summary, or None."""
    if summary and isinstance(summary.get("cmd_vel_hz"), dict):
        return summary["cmd_vel_hz"].get("mean"), summary["cmd_vel_hz"].get("std")
    return None


def aggregate(dirs, label, states=None):
    prefixes = find_runs(dirs, label)
    return {
        "label": label,
        "prefixes": prefixes,
        "n_runs": len(prefixes),
        "stages": pooled_stages(prefixes) if prefixes else {},
        "centroid": centroid_rms(prefixes, states=states) if prefixes else None,
        "summary": read_summary(dirs, label),
    }


# ── LaTeX emitters ───────────────────────────────────────────────────────────
def _f(x, nd=2):
    return "---" if x is None else f"{x:.{nd}f}"


def emit_1a(agg):
    """tab:latency_results body — Config A baseline, 4 stages + E2E."""
    st = agg["stages"]
    lines = []
    for key, label in PAPER_STAGE_LABELS:
        d = st.get(key)
        pad = f"{label:<17}"
        if key == "end_to_end":
            lines.append("\\hline")
        if d is None:
            lines.append(f"{pad} & --- & --- & --- \\\\")
        else:
            lines.append(f"{pad} & {_f(d['mean_ms'])} & {_f(d['p50_ms'])} "
                         f"& {_f(d['p95_ms'])} \\\\")
    return "\n".join(lines)


def emit_1b(configs):
    """per-stage P50 (ms) per config + /cmd_vel Hz. Rows = configs."""
    order = ["frame_acq_jitter", "object_detection", "detection_publish",
             "control", "end_to_end"]
    rows = []
    for cfg, agg in configs:
        st = agg["stages"]
        cells = [_f(st[k]["p50_ms"]) if st.get(k) else "---" for k in order]
        cv = cmdvel_hz(agg["summary"])
        cv_s = "---" if not cv or cv[0] is None else f"{cv[0]:.2f}$\\pm${cv[1]:.2f}"
        rows.append(f"{cfg} & " + " & ".join(cells) + f" & {cv_s} \\\\")
    header = ("\\textbf{Cfg} & \\textbf{Frame Acq.} & \\textbf{Obj.\\ Det.} & "
              "\\textbf{Det.\\ Pub.} & \\textbf{Control} & \\textbf{E2E} & "
              "\\textbf{/cmd\\_vel Hz} \\\\")
    return header, "\n".join(rows)


PLACEMENT_DESC = {
    "A": "NPU infer., SLAM pinned (cores 2--3)",
    "B": "CPU infer., SLAM pinned",
    "C": "NPU infer., SLAM shared core",
    "D": "all processes, single core",
}


def emit_placement(configs):
    """tab:placement rows: Det.lat (ms) / /cmd_vel Hz / Centroid RMS (px)."""
    rows = []
    for cfg, agg in configs:
        st = agg["stages"]
        det = st.get("object_detection")
        det_s = _f(det["mean_ms"], 1) if det else "---"
        cv = cmdvel_hz(agg["summary"])
        cv_s = "---" if not cv or cv[0] is None else f"{cv[0]:.2f}$\\pm${cv[1]:.2f}"
        c = agg["centroid"]
        c_s = "---" if not c else f"{c['mean']:.1f}$\\pm${c['std']:.1f}"
        rows.append(f"{cfg} & {PLACEMENT_DESC[cfg]:<36} & {det_s} & {cv_s} & {c_s} \\\\")
    return "\n".join(rows)


# ── main ─────────────────────────────────────────────────────────────────────
def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--results-dir", action="append", default=None,
                    help="dir(s) holding {label}_run*_*.csv (repeatable)")
    ap.add_argument("--labelA", default="yolo26n_npu_fast")
    ap.add_argument("--labelB", default="yolo26n_cpu_fast")
    ap.add_argument("--labelC", default="placement_C")
    ap.add_argument("--labelD", default="placement_D")
    ap.add_argument("--out-dir", default="benchmarks/results/paper_tables")
    # Centroid RMS is measured during APPROACHING only. Pooling every state instead
    # makes the metric a function of how much of the fixed recording window each run
    # spent parked at the goal: once REACHED, the drone is stationary and centred, so
    # its error collapses to ~10-18 px, and REACHED's share of the samples varies by
    # config (23% for the 16 Hz NPU run vs 44% for the 4.3 Hz CPU run). That ratio is
    # an artifact of run length versus reach time, not of tracking quality, and it
    # silently flatters whichever config idled at the target longest. Pass
    # --centroid-states '' to restore the old pooled behaviour.
    ap.add_argument("--centroid-states", default="APPROACHING",
                    help="comma-separated servo states to measure centroid RMS over "
                         "(default: APPROACHING). Empty string pools all states.")
    args = ap.parse_args()

    dirs = args.results_dir or [
        "benchmarks/results/hil",
        "benchmarks/results/detector_sweep_2026-06-25",
    ]
    dirs = [d for d in dirs if os.path.isdir(d)]
    os.makedirs(args.out_dir, exist_ok=True)

    states = ([s.strip().upper() for s in args.centroid_states.split(",")]
              if args.centroid_states else None)

    aggs = {c: aggregate(dirs, lbl, states=states) for c, lbl in
            (("A", args.labelA), ("B", args.labelB),
             ("C", args.labelC), ("D", args.labelD))}
    configs = [(c, aggs[c]) for c in ("A", "B", "C", "D")]

    # console summary
    print("Discovered runs:")
    for c, a in configs:
        cen = a["centroid"]
        cen_s = f"{cen['mean']:.1f}px x{cen['n_runs']}" if cen else "no centroid"
        print(f"  Config {c}: {a['label']:20} {a['n_runs']} run(s)  [{cen_s}]")
    print()

    # 1a — baseline (Config A)
    body_1a = emit_1a(aggs["A"])
    print("── Table 1a (tab:latency_results) ──")
    print(body_1a, "\n")

    # 1b — per stage x config
    hdr_1b, body_1b = emit_1b(configs)
    print("── Table 1b (per-stage x per-config, P50 ms) ──")
    print(hdr_1b)
    print(body_1b, "\n")

    # placement — C/D novel rows (A/B cross-referenced)
    body_pl = emit_placement(configs)
    print("── Table placement (tab:placement) ──")
    print(body_pl, "\n")

    for name, text in (("table_1a_latency.tex", body_1a),
                       ("table_1b_perstage.tex", hdr_1b + "\n\\hline\n" + body_1b),
                       ("table_placement.tex", body_pl)):
        with open(os.path.join(args.out_dir, name), "w") as f:
            f.write(text + "\n")
    print(f"→ wrote LaTeX fragments to {args.out_dir}/")


if __name__ == "__main__":
    main()
