#!/usr/bin/env python3
"""
make_detector_tables.py — emit the standalone-detector LaTeX table from committed data.

    tables/table_detectors.tex        tab:detectors (Table 2)

Every YOLO row is derived here from committed measurement artifacts, so a `diff`
against the committed fragment is a real check that the table matches the data:

    NPU rows   benchmarks/paper_data/hef/hef_timing_*.csv   (hef_bench.py,
               80 synchronous iterations / 8 warmup, HailoRT 5.1.1)
    CPU rows   benchmarks/paper_data/standalone/bench_<id>_cpu_t2.json
               (standalone_yolo_benchmarker.py, ONNX Runtime, intra_op=2,
               inter_op=1, spinning disabled)

Two rows are NOT emitted, because no standalone measurement of them is archived
anywhere in the repository: OpenCV MOG2 and OWL-ViT. The draft carries ~12 ms /
~80 fps and ~1700 ms / ~0.6 fps for these, and the ~1.7 s OWL-ViT figure is
load-bearing for the manuscript's "~2.2x its standalone cost under contention"
claim. Until they are measured, this script emits a visible placeholder rather
than silently reproducing unsourced numbers.

    python3 benchmarks/make_detector_tables.py [--repo-root .] [--out-dir DIR]
"""

import argparse
import glob
import json
import os

DISPLAY = {"yolo26n": "YOLO26n", "yolo26s": "YOLO26s", "yolo26m": "YOLO26m",
           "yolov8n": "YOLOv8n", "yolov8s": "YOLOv8s", "yolov8m": "YOLOv8m",
           "yolov11n": "YOLOv11n", "yolov11s": "YOLOv11s", "yolov11m": "YOLOv11m"}

# Order the table presents: YOLO26 n/s/m, then v8 n/s/m, then v11 n/s/m.
ORDER = ["yolo26n", "yolo26s", "yolo26m",
         "yolov8n", "yolov8s", "yolov8m",
         "yolov11n", "yolov11s", "yolov11m"]


def load_npu(repo):
    """model -> (inference_ms, e2e_ms, fps) from the HEF bench CSV.

    Deliberately keyed on the bare model id. The CSV also carries a
    `yolo26n_10h` row -- the self-compiled DFC 5.2.0 HEF that has been retired in
    favour of the Hailo Model Zoo build. Selecting it here is what put a
    24.7/26.0/38.5 row in the draft for a model the rest of the paper never runs;
    the deployed artifact (models/hef/yolo26n.hef) measures 27.2/28.5/35.1, which
    is what the in-loop sweep also sees (S5 P50 27.58 +/- 0.02 ms).
    """
    import csv
    pats = sorted(glob.glob(os.path.join(
        repo, "benchmarks", "paper_data", "hef", "hef_timing_*.csv")))
    if not pats:
        raise SystemExit("no benchmarks/paper_data/hef/hef_timing_*.csv found")
    out = {}
    with open(pats[-1]) as f:
        for r in csv.DictReader(f):
            if r["model"] in DISPLAY:
                out[r["model"]] = (float(r["inference_ms"]),
                                   float(r["e2e_ms"]), float(r["fps"]))
    return out


def load_cpu(repo):
    """model -> (inference_ms, e2e_ms, fps, frames, conf) from the standalone JSONs."""
    out = {}
    pat = os.path.join(repo, "benchmarks", "paper_data", "standalone",
                       "bench_*_cpu_t2.json")
    for p in sorted(glob.glob(pat)):
        d = json.load(open(p))
        # The glob also matches bench_owlvit_cpu_t2.json, which is written by a
        # different harness with a different schema; it is loaded by load_vlm().
        if d.get("config", {}).get("model_id") not in DISPLAY:
            continue
        c, lat = d["config"], d["latency_s"]
        if c["intra_threads"] != 2 or c["inter_threads"] != 1:
            raise SystemExit(f"{p}: expected intra=2/inter=1, got "
                             f"{c['intra_threads']}/{c['inter_threads']}")
        out[c["model_id"]] = (lat["inference"]["mean"] * 1e3,
                              lat["end_to_end"]["mean"] * 1e3,
                              d["fps"]["mean"], c["frames"], c["conf"])
    return out


def load_vlm(repo):
    """(end_to_end_ms, fps) for OWL-ViT, or None if not measured.

    Produced by benchmarks/standalone_vlm_benchmark.py, which drives the same
    OwlViTPredictor the pipeline uses, on one still image with warm-up discarded
    -- the same protocol as the YOLO CPU rows, at the same 2 threads.
    """
    p = os.path.join(repo, "benchmarks", "paper_data", "standalone",
                     "bench_owlvit_cpu_t2.json")
    if not os.path.exists(p):
        return None
    d = json.load(open(p))
    return d["latency_ms"]["end_to_end"]["mean"], d["fps"]["mean"]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--repo-root", default=".")
    ap.add_argument("--out-dir", default=None,
                    help="default: <repo>/benchmarks/paper_data/tables")
    args = ap.parse_args()
    repo = os.path.abspath(args.repo_root)
    outdir = args.out_dir or os.path.join(repo, "benchmarks", "paper_data", "tables")
    os.makedirs(outdir, exist_ok=True)

    npu, cpu = load_npu(repo), load_cpu(repo)
    missing = [m for m in ORDER if m not in npu or m not in cpu]
    if missing:
        raise SystemExit(f"missing measurements for: {missing}")

    frames = {m: cpu[m][3] for m in ORDER}
    confs = sorted({cpu[m][4] for m in ORDER})

    L = [
        "% Standalone detector benchmarks (tab:detectors).",
        "% NPU rows: hef_bench.py, 80 synchronous iterations / 8 warmup, Hailo-10H +",
        "%   HailoRT 5.1.1, throughput = 1/E2E, single-stream synchronous.",
        "%   Source: benchmarks/paper_data/hef/hef_timing_*.csv",
        "% CPU rows: standalone_yolo_benchmarker.py, ONNX Runtime, intra_op=2, inter_op=1,",
        "%   spinning disabled, 640x640 network input, no SLAM co-load.",
        "%   Source: benchmarks/paper_data/standalone/bench_<id>_cpu_t2.json",
        f"%   Measured frames per row: {', '.join(f'{DISPLAY[m]} {frames[m]}' for m in ORDER)}",
        f"%   conf thresholds present in these runs: {confs}",
        "% GENERATED -- regenerate rather than hand-edit.",
        r"\textbf{Model} & \textbf{Backend} & \textbf{Inf.\ (ms)} & \textbf{E2E (ms)} & \textbf{fps} \\",
        r"\hline",
        r"\multicolumn{5}{|l|}{\textit{YOLO-family --- Hailo-10H NPU (INT8 HEF, synchronous)}} \\",
        r"\hline",
    ]
    for m in ORDER:
        i, e, f = npu[m]
        L.append(f"{DISPLAY[m]:8s} & Hailo-10H  & {i:.1f}  & {e:.1f}  & {f:.1f} \\\\")
    L += [
        r"\hline",
        r"\multicolumn{5}{|l|}{\textit{YOLO-family --- ARM CPU (Cortex-A76, ONNX Runtime)}} \\",
        r"\hline",
    ]
    for m in ORDER:
        i, e, f = cpu[m][0], cpu[m][1], cpu[m][2]
        L.append(f"{DISPLAY[m]:8s} & ONNX-CPU  & {i:.1f}  & {e:.1f}  & {f:.1f} \\\\")
    L += [r"\hline",
          r"\multicolumn{5}{|l|}{\textit{Non-YOLO --- ARM CPU (Cortex-A76)}} \\",
          r"\hline"]

    # OpenCV MOG2 has no standalone row by construction, not by omission:
    # background subtraction estimates its model from a moving sequence, so the
    # single-image protocol behind every other row does not define a comparable
    # figure for it. Its in-loop cost is reported in tab:detector_e2e instead.
    L.append(r"OpenCV MOG2     & ARM CPU & ---  & \multicolumn{2}{c|}{\textit{see text}} \\")

    vlm = load_vlm(repo)
    if vlm:
        e, f = vlm
        L.append(r"OWL-ViT base-32~\cite{minderer2022owlvit} & ARM CPU & --- & "
                 f"{e:.0f} & {f:.2f} \\\\")
    else:
        L.append(r"% OWL-ViT: no measurement archived -- run standalone_vlm_benchmark.py")
    L.append(r"\hline")

    p = os.path.join(outdir, "table_detectors.tex")
    with open(p, "w") as f:
        f.write("\n".join(L) + "\n")
    print(f"wrote {p}")
    print(f"  NPU  yolo26n -> {npu['yolo26n'][2]:.1f} fps "
          f"(Model Zoo HEF; the retired yolo26n_10h build reads 38.5 and is not used)")
    print(f"  CPU  yolo26n -> {cpu['yolo26n'][0]:.1f} ms inference")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
