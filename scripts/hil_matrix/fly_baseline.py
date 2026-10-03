#!/usr/bin/env python3
"""fly_baseline.py -- run all 5 stereo arms for one baseline, in order.

Thin driver over run_baseline_matrix.py: no new orchestration logic, just the
fixed 5-arm sequence and the BASELINE0XX run-tag convention already used by
the 0.36/0.42/0.54/0.61 baselines, so one command replaces typing out 5 by
hand. FAIL FAST, same policy as run_matrix.py itself: the first arm that
exits non-zero stops the whole sequence, nothing after it runs, nothing is
retried.

    py scripts\\hil_matrix\\fly_baseline.py --baseline 0.11 --dry-run
    py scripts\\hil_matrix\\fly_baseline.py --baseline 0.11

Each arm still gets its own resolved-config printout, its own log under
scripts/hil_matrix/logs/, and its own results directory -- this wrapper only
sequences them, it does not change what any individual arm does.
"""

import argparse
import subprocess
import sys
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
RUNNER = SCRIPT_DIR / "run_baseline_matrix.py"

# Fixed 5-arm stereo protocol. RTAB-Map alone needs --trials forced to 10 --
# its config defaults to 3 (see run_baseline_matrix.py / HANDOFF Part H).
ARMS = [
    ("only_ov2_stereo_oracle_nopin",      {}),
    ("only_ov2_stereo_oracle_fast_nopin", {}),
    ("only_orbslam2_stereo_oracle_nopin", {}),
    ("only_orbslam3_stereo_oracle_nopin", {}),
    ("only_rtabmap_stereo_oracle_nopin",  {"trials": 10}),
]


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--baseline", type=float, required=True,
                     help="meters, e.g. 0.11 -- forwarded to every arm")
    ap.add_argument("--dry-run", action="store_true",
                     help="forwarded to every arm; nothing is executed anywhere")
    ap.add_argument("--run-tag", default=None,
                     help="default: BASELINE0XX from --baseline (0.11 -> BASELINE011)")
    ap.add_argument("--transport", choices=["wifi", "ethernet"], default=None,
                     help="forwarded to every arm as run_baseline_matrix.py's own "
                          "--transport (see there for the resolved addresses and the "
                          "eth0 carrier check)")
    args = ap.parse_args()

    code = f"{round(args.baseline * 100):03d}"
    run_tag = args.run_tag or f"BASELINE{code}"

    print(f"== fly_baseline: {args.baseline} m, run_tag={run_tag}, "
          f"{len(ARMS)} arms {'[DRY RUN]' if args.dry_run else ''} ==", flush=True)

    for i, (config, extra) in enumerate(ARMS, 1):
        cmd = ["py", str(RUNNER),
               "--config", config,
               "--baseline", str(args.baseline),
               "--run-tag", run_tag]
        if "trials" in extra:
            cmd += ["--trials", str(extra["trials"])]
        if args.transport:
            cmd += ["--transport", args.transport]
        if args.dry_run:
            cmd.append("--dry-run")

        print(f"\n== [{i}/{len(ARMS)}] {config} ==", flush=True)
        print("$ " + " ".join(cmd), flush=True)
        rc = subprocess.run(cmd).returncode
        if rc != 0:
            print(f"\nFAIL  {config} exited {rc} -- stopping here. "
                  f"Arm(s) after this one were NOT run.", file=sys.stderr, flush=True)
            return rc

    print(f"\n== all {len(ARMS)} arms complete for baseline {args.baseline} m "
          f"(run_tag={run_tag}) ==", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
