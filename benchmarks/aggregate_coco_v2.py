#!/usr/bin/env python3
"""
aggregate_coco_v2.py — turn the v2 COCO sweep CSVs into the tables the paper needs.

Reads benchmarks/results/accuracy/coco/v2_{controlled,deployed}/*.csv and emits,
to benchmarks/paper_data/accuracy/:

    coco_v2_all_runs.csv        every run, one row, all postprocess columns
    COCO_ACCURACY_RESULTS.md    narrative report (BRIEF_A structure)
    tables/table_coco_*.tex     LaTeX fragments

Every cross-backend comparison it prints is guarded: it refuses to compare two
runs that did not use the same image budget and the same postprocess profile.
That guard exists because the v1 sweep compared NPU-at-5000 against CPU-at-1000
under four different postprocess configurations, and nothing in the output said
so.

    python3 benchmarks/aggregate_coco_v2.py [--repo-root .]
"""

import argparse
import csv
import glob
import os
from collections import defaultdict

FAMILIES = {"yolo26": "YOLO26", "yolov8": "YOLOv8", "yolov11": "YOLOv11"}
SCALES = ["n", "s", "m"]


def family_scale(model):
    for pref, nice in FAMILIES.items():
        if model.startswith(pref):
            return nice, model[len(pref):]
    return model, ""


def load(repo):
    rows = []
    pat = os.path.join(repo, "benchmarks", "results", "accuracy", "coco",
                       "v2_*", "coco_accuracy_*.csv")
    for f in sorted(glob.glob(pat)):
        with open(f) as fh:
            for r in csv.DictReader(fh):
                for k in ("mAP50_95", "mAP50", "mAP75", "stopsign_AP50_95",
                          "stopsign_AP50", "inference_ms", "total_ms", "conf", "iou"):
                    if r.get(k) not in (None, "", "None"):
                        r[k] = float(r[k])
                for k in ("images", "n_detections", "max_det"):
                    if r.get(k) not in (None, "", "None"):
                        r[k] = int(r[k])
                r["_src"] = os.path.basename(f)
                rows.append(r)
    return rows


def key(r):
    return (r["model"], r["backend"], r["conf"], r["profile"], r["images"])


def comparable(a, b):
    """Two runs may be compared only if budget and postprocess match."""
    return (a["images"] == b["images"] and a["profile"] == b["profile"]
            and a["max_det"] == b["max_det"] and a["coord_mode"] == b["coord_mode"]
            and a["conf"] == b["conf"])


def tbl(rows, profile, backend, conf, images=None):
    """model -> row, for one cell of the design."""
    out = {}
    for r in rows:
        if (r["profile"] == profile and r["backend"] == backend
                and abs(r["conf"] - conf) < 1e-9
                and (images is None or r["images"] == images)):
            out[r["model"]] = r
    return out


def fmt_matrix(sel, field, width=9, prec=4):
    lines = []
    hdr = f"{'':8}" + "".join(f"{FAMILIES[f]:>{width}}" for f in FAMILIES)
    lines.append(hdr)
    for s in SCALES:
        cells = ""
        for fam in FAMILIES:
            r = sel.get(f"{fam}{s}")
            cells += f"{r[field]:>{width}.{prec}f}" if r else f"{'--':>{width}}"
        lines.append(f"{s:8}" + cells)
    return "\n".join(lines)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--repo-root", default=os.path.join(os.path.dirname(__file__), ".."))
    args = ap.parse_args()
    repo = os.path.abspath(args.repo_root)

    rows = load(repo)
    if not rows:
        print("no v2 CSVs found — run benchmarks/run_coco_sweep.sh first")
        return 1

    outdir = os.path.join(repo, "benchmarks", "paper_data", "accuracy")
    os.makedirs(os.path.join(outdir, "tables"), exist_ok=True)

    # ---- combined CSV ----------------------------------------------------
    cols = [c for c in rows[0].keys() if c != "_src"] + ["_src"]
    comb = os.path.join(outdir, "coco_v2_all_runs.csv")
    with open(comb, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=cols)
        w.writeheader()
        for r in sorted(rows, key=lambda r: (r["profile"], r["backend"], r["model"], r["conf"])):
            w.writerow(r)
    print(f"wrote {comb}  ({len(rows)} runs)")

    # ---- console summary -------------------------------------------------
    print(f"\n{'='*72}\nCOCO v2 — runs by cell\n{'='*72}")
    cells = defaultdict(int)
    for r in rows:
        cells[(r["profile"], r["backend"], r["conf"], r["images"])] += 1
    for k in sorted(cells):
        print(f"  profile={k[0]:11} backend={k[1]:4} conf={k[2]:<6} images={k[3]:<5} n={cells[k]}")

    for prof in ("controlled", "deployed"):
        for conf in (0.001, 0.20):
            sel = tbl(rows, prof, "npu", conf)
            if not sel:
                continue
            print(f"\n--- {prof} / NPU / conf={conf} — mAP50-95 ---")
            print(fmt_matrix(sel, "mAP50_95"))
            print(f"--- {prof} / NPU / conf={conf} — inference ms ---")
            print(fmt_matrix(sel, "inference_ms", prec=2))

    # ---- censoring check -------------------------------------------------
    print(f"\n{'='*72}\nCENSORING CHECK — v8/v11 NPU must be identical across thresholds\n{'='*72}")
    for prof in ("controlled", "deployed"):
        a = tbl(rows, prof, "npu", 0.001)
        b = tbl(rows, prof, "npu", 0.20)
        if not (a and b):
            continue
        for m in sorted(set(a) & set(b)):
            same = abs(a[m]["mAP50_95"] - b[m]["mAP50_95"]) < 1e-9
            expect = "identical" if not m.startswith("yolo26") else "should differ"
            ok = same if expect == "identical" else not same
            print(f"  {prof:11} {m:9} {a[m]['mAP50_95']:.4f} vs {b[m]['mAP50_95']:.4f}  "
                  f"({expect}) {'OK' if ok else '!! UNEXPECTED'}")

    # ---- quantization delta, guarded -------------------------------------
    print(f"\n{'='*72}\nQUANTIZATION — NPU INT8 vs CPU FP32 (matched budget + postprocess only)\n{'='*72}")
    by = defaultdict(dict)
    for r in rows:
        by[(r["model"], r["profile"], r["conf"], r["images"])][r["backend"]] = r
    any_pair = False
    for k in sorted(by):
        pair = by[k]
        if "npu" in pair and "cpu" in pair and comparable(pair["npu"], pair["cpu"]):
            any_pair = True
            n, c = pair["npu"]["mAP50_95"], pair["cpu"]["mAP50_95"]
            print(f"  {k[0]:9} {k[1]:11} conf={k[2]:<6} n={k[3]:<5} "
                  f"CPU {c:.4f} -> NPU {n:.4f}  {100*(n-c)/c:+6.1f}%")
    if not any_pair:
        print("  none yet — needs matched-budget NPU and CPU runs in the same profile")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
