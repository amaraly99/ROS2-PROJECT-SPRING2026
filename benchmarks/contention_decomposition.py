#!/usr/bin/env python3
"""
contention_decomposition.py — what channel couples the detector to its co-tenants?

The detector sweep shows a CPU-resident detector losing 20-34 % of its standalone
throughput inside the closed loop. The manuscript previously attributed that to
the detector and the SLAM solver "competing for the Cortex-A76 cores", which is
wrong: run_stack_hil.sh pins the detector to cores 0-1 and OV2SLAM to cores 2-3,
so they never contend for a core. This isolates the channel that is actually
responsible, using synthetic co-tenants instead of the full stack so it needs no
simulator, no SLAM build and no wired-LAN rig.

Conditions, detector always pinned to cores 0-1:

    unpinned    free to use all four cores, no co-tenant  -- affinity reference
    idle        pinned, no co-tenant                      -- baseline
    cpu         two ALU spinners on cores 2-3, L1-resident working set
    mem         two streaming loads on cores 2-3, 64 MB buffers (>> 2 MB L3)

`cpu` occupies the other cores without generating memory traffic; `mem`
saturates the memory path without needing the cores the detector uses. The
difference between them is the answer.

    python3 benchmarks/contention_decomposition.py \\
        --models yolo26n yolo26m --reps 3 \\
        --out benchmarks/paper_data/contention

Writes one CSV row per (model, condition, rep) plus a summary to stdout.
"""

import argparse
import csv
import json
import os
import subprocess
import sys
import tempfile
import time

HOG = r'''
import sys, time, numpy as np
mode, secs = sys.argv[1], float(sys.argv[2])
end = time.monotonic() + secs
if mode == "mem":
    a = np.ones(8 * 1024 * 1024, dtype=np.float64)   # 64 MB, >> 2 MB shared L3
    b = np.empty_like(a)
    while time.monotonic() < end:
        np.copyto(b, a); b += 1.0
else:
    x = np.ones(256, dtype=np.float64)               # 2 KB, fits L1
    while time.monotonic() < end:
        for _ in range(2000):
            x = x * 1.0000001 + 0.0000001
'''

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def bench(model, frames, warmup, pin, jpath):
    cmd = []
    if pin:
        cmd += ["taskset", "-c", pin]
    cmd += [sys.executable, os.path.join(REPO, "benchmarks", "standalone_yolo_benchmarker.py"),
            "--backend", "cpu", "--model_id", model, "--threads", "2",
            "--inter_threads", "1", "--frames", str(frames), "--warmup", str(warmup),
            "--conf", "0.2", "--image", os.path.join(REPO, "benchmarks/assets/bus.jpg"),
            "--json", jpath]
    subprocess.run(cmd, cwd=REPO, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=True)
    d = json.load(open(jpath))
    return d["latency_s"]["inference"]["mean"] * 1e3


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--models", nargs="+", default=["yolo26n"])
    ap.add_argument("--reps", type=int, default=3)
    ap.add_argument("--frames", type=int, default=40)
    ap.add_argument("--warmup", type=int, default=5)
    ap.add_argument("--hog-seconds", type=float, default=600)
    ap.add_argument("--out", default=os.path.join(REPO, "benchmarks/paper_data/contention"))
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)

    tmp = tempfile.mkdtemp()
    hogpy = os.path.join(tmp, "hog.py")
    open(hogpy, "w").write(HOG)

    rows = []
    for model in a.models:
        # medium variants cost ~1.4 s/frame; keep them to a workable frame count
        frames = a.frames if model.endswith("n") or model.endswith("s") else max(8, a.frames // 4)
        warmup = a.warmup if frames >= 20 else 3
        for rep in range(1, a.reps + 1):
            for cond in ("unpinned", "idle", "cpu", "mem"):
                procs = []
                if cond in ("cpu", "mem"):
                    for core in ("2", "3"):
                        procs.append(subprocess.Popen(
                            ["taskset", "-c", core, sys.executable, hogpy, cond,
                             str(a.hog_seconds)],
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL))
                    time.sleep(3)          # let the co-tenants reach steady state
                pin = None if cond == "unpinned" else "0,1"
                jp = os.path.join(tmp, f"{model}_{cond}_{rep}.json")
                try:
                    ms = bench(model, frames, warmup, pin, jp)
                finally:
                    for p in procs:
                        p.terminate()
                    for p in procs:
                        p.wait()
                    if procs:
                        time.sleep(1)
                rows.append({"model": model, "condition": cond, "rep": rep,
                             "frames": frames, "inference_ms": round(ms, 2)})
                print(f"  {model:9s} {cond:9s} rep{rep}  {ms:8.1f} ms", flush=True)

    csv_path = os.path.join(a.out, "contention_decomposition.csv")
    with open(csv_path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    print(f"\nwrote {csv_path}")

    print(f"\n{'model':9s} {'condition':10s} {'mean ms':>9s} {'std':>6s} {'vs idle':>9s}")
    for model in a.models:
        base = [r["inference_ms"] for r in rows if r["model"] == model and r["condition"] == "idle"]
        b = sum(base) / len(base)
        for cond in ("unpinned", "idle", "cpu", "mem"):
            v = [r["inference_ms"] for r in rows if r["model"] == model and r["condition"] == cond]
            m = sum(v) / len(v)
            sd = (sum((x - m) ** 2 for x in v) / len(v)) ** 0.5 if len(v) > 1 else 0.0
            print(f"{model:9s} {cond:10s} {m:9.1f} {sd:6.1f} {100*(m-b)/b:+8.1f}%")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
