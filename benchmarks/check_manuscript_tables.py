#!/usr/bin/env python3
"""
check_manuscript_tables.py — does the manuscript still match the generated tables?

verify_tables.sh proves the LaTeX fragments under benchmarks/paper_data/ match the
archived measurements. It says nothing about the paper, because access.tex carries
no \\input{} -- every table is an inline literal. The two are independent copies and
can drift silently, which is how Table 2's YOLO26n NPU row came to report a HEF the
pipeline no longer loads, and how the ranking table came to compare a 1000-image
column against a 5000-image one.

This closes that gap: for each table with a generator, it pulls the inline body out
of access.tex by \\label, normalises both sides to their data rows, and diffs.

    python3 benchmarks/check_manuscript_tables.py [--tex PATH] [--repo-root .]

Exit status is non-zero if any table has drifted.
"""

import argparse
import os
import re
import sys

# label in access.tex -> generated fragment, relative to the repo root
PAIRS = [
    ("tab:detectors",        "benchmarks/paper_data/tables/table_detectors.tex"),
    ("tab:latency_results",  "benchmarks/paper_data/tables/table_1a_latency.tex"),
    ("tab:placement_stages", "benchmarks/paper_data/tables/table_placement_stages_paper.tex"),
    ("tab:acc_reversal",     "benchmarks/paper_data/accuracy/tables/table_coco_benchmark_vs_deployed.tex"),
    ("tab:acc_deployed",     "benchmarks/paper_data/accuracy/tables/table_coco_deployed.tex"),
    ("tab:acc_quant",        "benchmarks/paper_data/accuracy/tables/table_coco_quantization.tex"),
]

# Tables the paper prints that no committed script produces yet. Listed so the
# report states the gap rather than silently covering only part of the paper.
UNGENERATED = [
    ("tab:detector_e2e", "detector sweep: data committed under paper_data/detector_sweep/, no emitter"),
]


def rows(text):
    """Numeric data rows of a LaTeX table body, normalised for comparison.

    Header and rule rows are excluded on purpose. The manuscript and the
    generators lay headers out differently (line breaks, column-spec details) and
    those differences are cosmetic; what must not drift is the numbers. A row
    counts when it has a '&', terminates with '\\\\', and contains a digit
    that survives stripping LaTeX control sequences. Requiring a decimal (rather
    than any digit) is what keeps column headings such as "mAP50-95" out.
    """
    out = []
    for line in text.split("\n"):
        line = line.strip()
        if not line or line.startswith("%"):
            continue
        line = re.sub(r"\\hline", "", line).strip()
        if "&" not in line or not line.endswith("\\\\"):
            continue
        # strip control sequences before looking for digits, so \multicolumn{2}
        # and \textbf do not make a header row look numeric
        bare = re.sub(r"\\[a-zA-Z]+", "", line)
        if not re.search(r"\d+\.\d+", bare):
            continue
        out.append(re.sub(r"\s+", " ", line))
    return out


def inline_body(tex, label):
    """The tabular body of the table carrying `label` in the manuscript."""
    i = tex.find("\\label{%s}" % label)
    if i < 0:
        return None
    b = tex.find("\\begin{tabular}", i)
    e = tex.find("\\end{tabular}", b)
    if b < 0 or e < 0:
        return None
    return tex[tex.find("\n", b):e]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--repo-root", default=".")
    ap.add_argument("--tex", default="paper/draft_3/access.tex")
    args = ap.parse_args()
    repo = os.path.abspath(args.repo_root)
    tex_path = args.tex if os.path.isabs(args.tex) else os.path.join(repo, args.tex)
    if not os.path.exists(tex_path):
        print(f"manuscript not found: {tex_path}")
        return 0                      # not an error: the .tex is deliberately untracked
    tex = open(tex_path).read()

    drift = 0
    for label, frag in PAIRS:
        fp = os.path.join(repo, frag)
        body = inline_body(tex, label)
        if body is None:
            print(f"  {label:22s} NOT FOUND in manuscript"); drift += 1; continue
        if not os.path.exists(fp):
            print(f"  {label:22s} fragment missing ({frag})"); drift += 1; continue
        paper_rows, gen_rows = rows(body), rows(open(fp).read())
        # The contract is one-directional: every row the generator produces from
        # the archived data must appear in the paper. Rows the paper carries that
        # no generator emits are reported, not failed -- some are deliberate
        # (the non-YOLO detectors have no archived standalone measurement).
        missing = [r for r in gen_rows if r not in paper_rows]
        extra = [r for r in paper_rows if r not in gen_rows]
        if not missing:
            note = f", {len(extra)} ungenerated row(s) in paper" if extra else ""
            print(f"  {label:22s} MATCHES ({len(gen_rows)} rows{note})")
            for r in extra[:4]:
                print(f"      ungenerated: {r[:96]}")
            continue
        drift += 1
        print(f"  {label:22s} DRIFTED ({len(missing)} row(s) differ)")
        for r in missing[:6]:
            print(f"      data  says : {r[:96]}")
        for r in extra[:6]:
            print(f"      paper says : {r[:96]}")

    for label, why in UNGENERATED:
        print(f"  {label:22s} no generator -- {why}")

    print()
    print("manuscript matches the generated tables."
          if not drift else
          f"{drift} table(s) drifted from the data. Regenerate, then reconcile access.tex.")
    return 1 if drift else 0


if __name__ == "__main__":
    sys.exit(main())
