#!/usr/bin/env python3
"""
aggregate_coco_v2.py — turn the v2 COCO sweep CSVs into the tables the paper needs.

Reads benchmarks/results/accuracy/coco/v2_{controlled,deployed}/*.csv and emits,
to benchmarks/paper_data/accuracy/:

    coco_v2_all_runs.csv        every run, one row, all postprocess columns
    tables/table_coco_*.tex     LaTeX fragments

COCO_ACCURACY_RESULTS.md sits in the same directory but is NOT written here; it
is maintained by hand and can fall behind this script's output.

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


def _run_csvs(repo):
    """Locate the per-run sweep CSVs.

    Preferred source is the raw sweep output under benchmarks/results/, which is
    gitignored and so only exists on the machine that ran the sweep. A fresh
    checkout has the same runs committed under benchmarks/paper_data/, so fall
    back to those: the two sets mirror each other exactly (v2_controlled 40 +
    v2_controlled_matched_head 3 + v2_deployed 18 = 61 runs), only the filenames
    differ. results/ wins when both are present, so on the authoring machine this
    returns exactly what it always did.
    """
    raw = sorted(glob.glob(os.path.join(
        repo, "benchmarks", "results", "accuracy", "coco",
        "v2_*", "coco_accuracy_*.csv")))
    if raw:
        return raw
    return sorted(glob.glob(os.path.join(
        repo, "benchmarks", "paper_data", "accuracy",
        "coco_v2_*", "coco_accuracy_*.csv")))


def load(repo):
    rows = []
    for f in _run_csvs(repo):
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


# ── LaTeX fragments ──────────────────────────────────────────────────────────
# Published COCO val2017 mAP50-95, Sapkota & Karkee, arXiv:2510.09653 Table 3.
# YOLO26n is quoted at its END-TO-END value (40.3), not the one-to-many 39.8,
# because our CPU export is the e2e/NMS-free head (model_registry.json:
# onnx_layout "yolo26_e2e", output shape (1,300,6)). Comparing our e2e export
# against their one-to-many number would be an apples-to-oranges delta.
PUBLISHED = {"yolov8n": (0.373, ""), "yolov11n": (0.395, ""),
             "yolo26n": (0.403, " (e2e)")}

DISPLAY = {"yolo26n": "YOLO26n", "yolo26s": "YOLO26s", "yolo26m": "YOLO26m",
           "yolov8n": "YOLOv8n", "yolov8s": "YOLOv8s", "yolov8m": "YOLOv8m",
           "yolov11n": "YOLOv11n", "yolov11s": "YOLOv11s", "yolov11m": "YOLOv11m"}


def _b(val, on, prec=4):
    """Bold a value when `on`."""
    s = f"{val:.{prec}f}"
    return f"\\textbf{{{s}}}" if on else s


def _pick(rows, backend, conf, images, layout=None, model=None):
    """One run, or None. `layout` filters onnx_layout exactly when given."""
    hits = [r for r in rows
            if r["backend"] == backend and abs(r["conf"] - conf) < 1e-9
            and r["profile"] == "controlled" and r["images"] == images
            and (model is None or r["model"] == model)
            and (layout is None or (r.get("onnx_layout") or "") == layout)]
    if len(hits) > 1:
        raise SystemExit(f"ambiguous cell: {backend} conf={conf} n={images} "
                         f"layout={layout!r} model={model} -> {len(hits)} runs")
    return hits[0] if hits else None


def emit_tables(rows, outdir):
    """Write the LaTeX fragments the paper \\input{}s.

    Every value is derived from coco_v2_all_runs.csv here, so `diff` against the
    committed fragments is a real check that the tables match the data. Cells are
    looked up by (backend, conf, images, onnx_layout) and a duplicate match is a
    hard error rather than a silent first-match.
    """
    tdir = os.path.join(outdir, "tables")
    os.makedirs(tdir, exist_ok=True)
    written = []

    def write(name, text):
        p = os.path.join(tdir, name)
        with open(p, "w") as f:
            f.write(text)
        written.append(name)

    nano = ["yolo26n", "yolov8n", "yolov11n"]
    groups = [["yolo26n", "yolov8n", "yolov11n"],
              ["yolo26s", "yolov8s", "yolov11s"],
              ["yolo26m", "yolov8m", "yolov11m"]]

    # ── Table 3: component ranking vs deployed ranking ──────────────────────
    # Both columns must share an image budget. The v1 sweep's headline error was
    # ranking a 1000-image CPU column against a 5000-image NPU column, so the
    # budget is chosen explicitly and a mismatch is fatal.
    # layout="" pins the component column to each model's DEFAULT export, which is
    # the configuration published mAP tables are computed on -- for YOLO26 that is
    # the e2e/NMS-free head. The raw-tensor YOLO26 exports live in the same pool
    # (they are the architecture-matched arm of the quantization table) and would
    # otherwise make this lookup ambiguous.
    for budget in (5000, 1000):
        comp = {m: _pick(rows, "cpu", 0.001, budget, layout="", model=m) for m in nano}
        if all(comp.values()):
            break
    else:
        raise SystemExit("no image budget has CPU FP32 conf=0.001 for all three nano models")
    depl = {m: _pick(rows, "npu", 0.2, budget, model=m) for m in nano}
    missing = [m for m in nano if not depl[m]]
    if missing:
        raise SystemExit(f"deployed cells missing at n={budget}: {missing} "
                         f"(refusing to rank across different image budgets)")

    crank = {m: i + 1 for i, m in enumerate(
        sorted(nano, key=lambda m: -comp[m]["mAP50_95"]))}
    drank = {m: i + 1 for i, m in enumerate(
        sorted(nano, key=lambda m: -depl[m]["mAP50_95"]))}
    lines = [
        "% The thesis result: component benchmark and deployed configuration rank the",
        "% same models in OPPOSITE orders, on the same hardware.",
        f"%   Component = CPU FP32, conf=0.001 (standard mAP), {budget}-image val2017",
        f"%   Deployed  = NPU INT8, conf=0.20, {budget}-image val2017, controlled postprocess",
        "% Both columns share one image budget -- see emit_tables() in aggregate_coco_v2.py.",
        "% Source: benchmarks/paper_data/accuracy/coco_v2_all_runs.csv",
        "% GENERATED -- regenerate rather than hand-edit.",
        (r"\textbf{Model} & \multicolumn{2}{c|}{\textbf{Component (CPU FP32)}} "
         r"& \multicolumn{2}{c}{\textbf{Deployed (NPU INT8)}} \\"),
        r"         & mAP50-95 & rank & mAP50-95 & rank \\",
        r"\hline",
    ]
    for m in sorted(nano, key=lambda m: crank[m]):
        cr, dr = crank[m], drank[m]
        lines.append(
            f"{DISPLAY[m]:8s} & {_b(comp[m]['mAP50_95'], cr == 1)} & "
            f"{'\\textbf{1}' if cr == 1 else cr} & "
            f"{_b(depl[m]['mAP50_95'], dr == 1)} & "
            f"{'\\textbf{1}' if dr == 1 else dr} \\\\")
    write("table_coco_benchmark_vs_deployed.tex", "\n".join(lines) + "\n")

    # ── Validation against published mAP ────────────────────────────────────
    lines = [
        "% Harness validation: our CPU FP32 pipeline against published COCO mAP50-95.",
        "% Published values: Sapkota & Karkee, arXiv:2510.09653, Table 3.",
        "% YOLO26n is compared against its END-TO-END figure (40.3), because our CPU",
        "% export is the e2e/NMS-free head -- see PUBLISHED in aggregate_coco_v2.py.",
        f"% Ours: ARM CPU, FP32 ONNX, conf=0.001, {budget}-image val2017, controlled postprocess.",
        "% GENERATED -- regenerate rather than hand-edit.",
        r"\textbf{Model} & \textbf{Published} & \textbf{This work} & \textbf{$\Delta$ (\%)} \\",
        r"\hline",
    ]
    for m in nano:
        pub, note = PUBLISHED[m]
        ours = comp[m]["mAP50_95"]
        d = 100.0 * (ours - pub) / pub
        lines.append(f"{DISPLAY[m]:8s} & {pub:.3f}{note} & {ours:.4f} & ${d:+.1f}$ \\\\")
    write("table_coco_validation.tex", "\n".join(lines) + "\n")

    # ── Table 4: deployed accuracy, all nine ────────────────────────────────
    lines = [
        "% Detector accuracy at the deployed operating point.",
        "% NPU INT8, conf=0.20, full COCO val2017 (5000 images), controlled postprocess",
        "% (NMS IoU 0.70, max_det 300, float box coords) applied identically to all nine.",
        "% Source: benchmarks/paper_data/accuracy/coco_v2_all_runs.csv",
        "% ALL VALUES GENERATED FROM THE CSV, NOT TRANSCRIBED -- regenerate rather than hand-edit.",
        "% NOTE: YOLOv8/v11 NPU values are CENSORED LOWER BOUNDS -- their HEFs bake a",
        "% 0.200 on-device score floor, so standard mAP is structurally unobtainable.",
        "% Latency is deliberately omitted: tab:detectors already reports it from a",
        "% different harness, and printing both invites a spurious inconsistency.",
        r"\textbf{Model} & \textbf{mAP50-95} & \textbf{mAP50} & \textbf{Stop sign AP} \\",
        r"\hline",
    ]
    for g in groups:
        sel = {m: _pick(rows, "npu", 0.2, 5000, model=m) for m in g}
        if not all(sel.values()):
            raise SystemExit(f"deployed table missing NPU conf=0.20 n=5000 for {g}")
        best = {f: max(sel[m][f] for m in g)
                for f in ("mAP50_95", "mAP50", "stopsign_AP50_95")}
        for m in g:
            r = sel[m]
            lines.append(
                f"{DISPLAY[m]:8s} & {_b(r['mAP50_95'], r['mAP50_95'] == best['mAP50_95'])}"
                f" & {_b(r['mAP50'], r['mAP50'] == best['mAP50'])}"
                f" & {_b(r['stopsign_AP50_95'], r['stopsign_AP50_95'] == best['stopsign_AP50_95'])} \\\\")
        lines.append(r"\hline")
    write("table_coco_deployed.tex", "\n".join(lines) + "\n")

    # ── Table 5: INT8 quantization penalty, architecture-matched ────────────
    # YOLO26's CPU arm must be the raw-tensor (one2many) export so it runs the
    # SAME head as its HEF; its default export is the e2e head and comparing
    # against that would conflate quantization with an architecture change.
    lines = [
        "% INT8 quantization penalty, architecture- and budget-matched.",
        "% conf=0.20, 1000-image subset both backends, controlled postprocess.",
        "% YOLO26 CPU uses the one2many raw-tensor export (benchmarks/export_yolo26_raw_tensor.py)",
        "% so it runs the SAME head as its HEF -- precision is the only variable.",
        "% Source: benchmarks/paper_data/accuracy/coco_v2_all_runs.csv",
        "% GENERATED -- regenerate rather than hand-edit.",
        r"\textbf{Model} & \textbf{CPU FP32} & \textbf{NPU INT8} & \textbf{$\Delta$ (\%)} \\",
        r"\hline",
    ]
    fam_order = [["yolo26n", "yolo26s", "yolo26m"],
                 ["yolov8n", "yolov8s", "yolov8m"],
                 ["yolov11n", "yolov11s", "yolov11m"]]
    for i, fam in enumerate(fam_order):
        for m in fam:
            layout = "raw_tensor" if m.startswith("yolo26") else ""
            cpu = _pick(rows, "cpu", 0.2, 1000, layout=layout, model=m)
            npu = _pick(rows, "npu", 0.2, 1000, model=m)
            if not (cpu and npu):
                raise SystemExit(f"quantization table missing a cell for {m}")
            d = 100.0 * (npu["mAP50_95"] - cpu["mAP50_95"]) / cpu["mAP50_95"]
            lines.append(f"{DISPLAY[m]:8s} & {cpu['mAP50_95']:.4f} & "
                         f"{npu['mAP50_95']:.4f} & ${d:.1f}$ \\\\")
        if i < len(fam_order) - 1:
            lines.append(r"\hline")
    write("table_coco_quantization.tex", "\n".join(lines) + "\n")

    print(f"\nwrote {len(written)} LaTeX fragments to {tdir}/")
    for n in written:
        print(f"  {n}")


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
    # Union of every row's keys, in first-seen order. Taking the schema from
    # rows[0] alone breaks whenever the first file happens to lack a column that
    # later files carry (e.g. onnx_layout, which only the yolo26 CPU runs set) —
    # and which file sorts first depends on which directory the runs were read
    # from.
    cols = []
    for r in rows:
        for c in r:
            if c != "_src" and c not in cols:
                cols.append(c)
    cols.append("_src")
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

    # ---- LaTeX fragments the paper \input{}s -----------------------------
    emit_tables(rows, outdir)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
