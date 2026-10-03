#!/usr/bin/env bash
# RTAB-Map repeat at n=10 for all three baselines.
#
# WHY THIS EXISTS: run_baseline_sweep.sh omitted the --trials 10 override for
# RTAB-Map. only_rtabmap_stereo_oracle_nopin.yaml carries `trials: 3` as its file
# default (every other arm's config defaults to 10), so RTAB came out n=3 at all
# three baselines while the other four arms were n=10. That makes RTAB
# non-comparable to its own peers and to the ALICIA RTAB n=10 figure.
#
# Same swap -> verify -> sync -> run sequence as the sweep, so the physical
# baseline (.slx camera mount) and the calibrated baseline (five files on the Pi)
# agree for each run. A mismatch is silent, see FIX-027.
#
# Run tag is BASELINE<TAG>_RTAB10 so these are unambiguously distinguishable from
# the n=3 runs they supersede.

set -u

REPO="/c/Users/homie/Desktop/ROS2-slam-hil"
MATLAB_DIR="$REPO/matlab"
PI="amaraly@192.168.1.60"
ARM="only_rtabmap_stereo_oracle_nopin"

BASELINES=("0.36:036" "0.42:042" "0.54:054")

STAMP=$(date +%Y%m%d_%H%M%S)
LOG="$REPO/scripts/hil_matrix/logs/rtab_repeat_${STAMP}.log"

exec > >(tee -a "$LOG") 2>&1

echo "================================================================"
echo "RTAB-MAP REPEAT (n=10) started $(date)"
echo "log: $LOG"
echo "================================================================"

for entry in "${BASELINES[@]}"; do
  B="${entry%%:*}"
  TAG="${entry##*:}"

  echo ""
  echo "################################################################"
  echo "### RTAB n=10 @ BASELINE ${B} m   start $(date)"
  echo "################################################################"

  SRC="$MATLAB_DIR/hil_closed_loop_baseline_${TAG}.slx"
  if [[ ! -f "$SRC" ]]; then
    echo "FATAL: $SRC missing, skipping baseline $B"
    continue
  fi

  running=$(powershell -Command "@(Get-Process -EA SilentlyContinue | Where-Object { \$_.Name -match '^(matlab|AutoVrtlEnv)' }).Count" | tr -d '\r')
  if [[ "$running" != "0" ]]; then
    echo "  MATLAB still running ($running proc) -- killing before model swap"
    powershell -Command "Get-Process -EA SilentlyContinue | Where-Object { \$_.Name -match '^(matlab|AutoVrtlEnv|MathWorksCrashReporter)' } | Stop-Process -Force -EA SilentlyContinue"
    sleep 5
  fi
  cp -f "$SRC" "$MATLAB_DIR/hil_closed_loop.slx"
  echo "  swapped model <- hil_closed_loop_baseline_${TAG}.slx"

  TMPX=$(mktemp -d)
  unzip -o -q "$MATLAB_DIR/hil_closed_loop.slx" -d "$TMPX"
  FOUND=$(grep -ho 'tmountOffset">\[0.2, -[0-9.]*' "$TMPX/simulink/systems/"*.xml 2>/dev/null | head -1 | grep -o '\-[0-9.]*$')
  rm -rf "$TMPX"
  if [[ "$FOUND" != "-$B" ]]; then
    echo "FATAL: .slx encodes '$FOUND', expected '-$B' -- ABORTING this baseline"
    continue
  fi
  echo "  verified .slx camera offset = $FOUND"

  if ! ssh "$PI" "python3 ~/sync_baseline.py $B"; then
    echo "FATAL: calib sync failed for $B -- ABORTING this baseline"
    continue
  fi

  echo ""
  echo "### running $ARM --trials 10 @ baseline $B"
  ( cd "$REPO" && py scripts/hil_matrix/run_matrix.py --config "$ARM" --trials 10 --run-tag "BASELINE${TAG}_RTAB10" )
  echo "### RTAB n=10 @ baseline $B  exit=$?  end $(date)"
done

echo ""
echo "================================================================"
echo "RTAB-MAP REPEAT finished $(date)"
echo "================================================================"
