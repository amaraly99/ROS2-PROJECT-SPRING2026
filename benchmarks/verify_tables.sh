#!/usr/bin/env bash
# verify_tables.sh — regenerate every paper table from committed data and diff.
#
# The contract: each table the paper \input{}s is produced by a committed script
# from committed data, so regenerating it must reproduce the committed fragment
# byte for byte. An empty diff is the guarantee that the printed numbers match
# the archived measurements.
#
#   bash benchmarks/verify_tables.sh          # verify, no writes to the repo
#   bash benchmarks/verify_tables.sh --update # accept new output into paper_data
#
# Exit status is non-zero if any table fails to reproduce.

set -uo pipefail
REPO="$(cd "$(dirname "$0")/.." && pwd)"
cd "$REPO"
UPDATE=0
[[ "${1:-}" == "--update" ]] && UPDATE=1

TMP="$(mktemp -d)"
trap 'rm -rf "$TMP"' EXIT
COMMITTED_SWEEP="benchmarks/paper_data/tables"
COMMITTED_ACC="benchmarks/paper_data/accuracy/tables"
fail=0; pass=0

check() {  # <label> <generated-file> <committed-file>
    local label="$1" gen="$2" com="$3"
    if [[ ! -f "$gen" ]]; then
        printf '  %-38s NOT GENERATED\n' "$label"; fail=$((fail+1)); return
    fi
    if [[ ! -f "$com" ]]; then
        printf '  %-38s no committed copy (new)\n' "$label"
        [[ $UPDATE -eq 1 ]] && cp "$gen" "$com" && printf '      -> added\n'
        return
    fi
    if diff -q "$gen" "$com" >/dev/null; then
        printf '  %-38s OK\n' "$label"; pass=$((pass+1))
    else
        printf '  %-38s DIFFERS\n' "$label"
        diff "$com" "$gen" | sed 's/^/      /' | head -20
        if [[ $UPDATE -eq 1 ]]; then cp "$gen" "$com"; printf '      -> updated\n'
        else fail=$((fail+1)); fi
    fi
}

echo "== HIL sweep + placement tables (make_paper_tables.py) =="
python3 benchmarks/make_paper_tables.py \
    --results-dir benchmarks/paper_data/detector_sweep \
    --results-dir benchmarks/paper_data/placement \
    --out-dir "$TMP/sweep" >/dev/null 2>&1
for f in table_1a_latency.tex table_1b_perstage.tex \
         table_1b_perstage_spread.tex table_placement.tex; do
    check "$f" "$TMP/sweep/$f" "$COMMITTED_SWEEP/$f"
done

echo "== Standalone detector table (make_detector_tables.py) =="
python3 benchmarks/make_detector_tables.py --repo-root . \
    --out-dir "$TMP/det" >/dev/null 2>&1
check "table_detectors.tex" "$TMP/det/table_detectors.tex" \
      "$COMMITTED_SWEEP/table_detectors.tex"

echo "== COCO accuracy tables (aggregate_coco_v2.py) =="
# Run against a scratch root whose only input is the COMMITTED paper_data mirror,
# so this also proves a fresh checkout can regenerate them.
mkdir -p "$TMP/acc/benchmarks/paper_data/accuracy"
for d in coco_v2_controlled coco_v2_deployed coco_v2_matched_head; do
    ln -s "$REPO/benchmarks/paper_data/accuracy/$d" \
          "$TMP/acc/benchmarks/paper_data/accuracy/$d"
done
python3 benchmarks/aggregate_coco_v2.py --repo-root "$TMP/acc" >/dev/null 2>&1
for f in table_coco_benchmark_vs_deployed.tex table_coco_validation.tex \
         table_coco_deployed.tex table_coco_quantization.tex; do
    check "$f" "$TMP/acc/benchmarks/paper_data/accuracy/tables/$f" \
          "$COMMITTED_ACC/$f"
done

echo
if [[ $fail -eq 0 ]]; then
    echo "All $pass table(s) reproduce from committed data."
else
    echo "$fail table(s) failed to reproduce, $pass OK."
    echo "Re-run with --update once you have confirmed the new values are correct."
fi
exit $(( fail > 0 ))
