#!/usr/bin/env python3
"""Aggregate the ALICIA stereo and mono per-trial results.csv files into one
compact per-algorithm CSV each -- one row per algorithm, mean-aggregated over
its valid (status=="ok") trials.

Method (as specified by the user, not the alternative concat-then-group
design): read each results.csv separately -- this project's own convention is
one algorithm ("arm") per results directory -- aggregate its numeric columns
right there, emit exactly one row per file, then union all per-file rows into
one DataFrame per modality. Numeric columns are found by dtype
(`select_dtypes(include="number")`), not a hardcoded list, with a single named
exception: `trial` is numeric but is a row index, not a metric, and is
excluded from aggregation on purpose.

ov2_fast_clahe is hardcoded excluded, both of its directories (the original
attempt and the CLAHE_REP2 replication). Root-caused this session to a
reproducible OV2SLAM segfault (a SlamManager::reset() / LoopCloser::run()
race under use_fast:1), confirmed identical in plain `fast` and `fast_clahe`
logs alike, unrelated to CLAHE. Declared dead by the user; not a real
algorithm comparison point.

A file whose every trial aborted (e.g. ov2_fast mono, 0/10) still produces a
row: `valid[col].mean()` on an empty/all-NaN selection returns NaN on its own,
and pd.DataFrame(list_of_row_dicts) unions differing key sets across rows and
NaN-fills any row missing a key a richer row has. No special-casing needed for
either case; both fall out of ordinary pandas behavior.
"""
import io
import subprocess
from pathlib import Path

import pandas as pd

PI_HOST = "amaraly@192.168.1.60"
PI_RESULTS_DIR = "~/ROS2-slam-hil/results"

EXCLUDED_ARMS = {"ov2_fast_clahe"}
NON_METRIC_NUMERIC = {"trial"}

OUTPUT_DIR = Path(
    r"C:\Users\homie\AppData\Local\Temp\claude\c--Users-homie-Desktop-ROS2-slam-hil"
    r"\986ea6ff-4b02-4436-a30f-c6955aafb71a\scratchpad"
)


def list_result_dirs(prefix):
    """Directory names on the Pi under results/ starting with `prefix`."""
    completed = subprocess.run(
        ["ssh", PI_HOST, f"cd {PI_RESULTS_DIR} && ls -d {prefix}*/ 2>/dev/null"],
        capture_output=True, text=True, check=True,
    )
    return [d.strip().rstrip("/") for d in completed.stdout.splitlines() if d.strip()]


def read_remote_csv(dirname):
    """Fetch one results.csv over SSH straight into a DataFrame, no local cache file."""
    remote_path = f"{PI_RESULTS_DIR}/{dirname}/results.csv"
    completed = subprocess.run(
        ["ssh", PI_HOST, f"cat {remote_path}"],
        capture_output=True, text=True, check=True,
    )
    return pd.read_csv(io.StringIO(completed.stdout))


def aggregate_one_file(df, source_dirname):
    """One algorithm's results.csv -> one row dict, or None if excluded/empty."""
    if df.empty:
        print(f"  SKIP  {source_dirname}: empty results.csv")
        return None

    arm = df["arm"].iloc[0]
    if arm in EXCLUDED_ARMS:
        print(f"  SKIP  {source_dirname}: arm={arm!r} is hardcoded excluded (dead)")
        return None

    valid = df[df["status"] == "ok"]

    numeric_cols = [
        c for c in df.select_dtypes(include="number").columns
        if c not in NON_METRIC_NUMERIC
    ]

    row = {
        "arm": arm,
        "config": df["config"].iloc[0],
        "scene": df["scene"].iloc[0],
    }
    for col in numeric_cols:
        row[col] = valid[col].mean()
    row["n_ok"] = int(len(valid))
    row["n_total"] = int(len(df))

    print(f"  OK    {source_dirname}: arm={arm}  n_ok={row['n_ok']}/{row['n_total']}")
    return row


def build_aggregate_csv(prefix, output_name):
    print(f"=== {prefix}* ===")
    rows = []
    for dirname in list_result_dirs(prefix):
        df = read_remote_csv(dirname)
        row = aggregate_one_file(df, dirname)
        if row is not None:
            rows.append(row)

    result_df = pd.DataFrame(rows)
    out_path = OUTPUT_DIR / output_name
    result_df.to_csv(out_path, index=False)
    print(f"  wrote {out_path}  ({len(result_df)} algorithm rows, "
          f"{len(result_df.columns)} columns)")
    print()
    return result_df


def main():
    build_aggregate_csv("ALICIA_matrix_", "alicia_stereo_aggregate.csv")
    build_aggregate_csv("ALICIA_MONO_matrix_", "alicia_mono_aggregate.csv")


if __name__ == "__main__":
    main()
