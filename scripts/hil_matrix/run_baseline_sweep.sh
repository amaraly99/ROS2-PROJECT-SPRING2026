#!/usr/bin/env bash
# Baseline sweep: run the full 5-arm stereo protocol at 0.36, 0.42 and 0.54 m.
#
# For each baseline this does, IN ORDER:
#   1. swap matlab/hil_closed_loop.slx to that baseline's model variant
#   2. verify the swapped .slx actually encodes that baseline (extract + grep)
#   3. sync all five baseline-encoding files on the Pi (calibs + ORB bf/b + RTAB)
#   4. run the five stereo arms, 10 trials each, unpinned, sequentially
#
# Step 2 and 3 exist because physical baseline (.slx camera mount) and calibrated
# baseline (the five files) must agree. A mismatch is silent: SLAM scales every
# depth by the calibrated value with no error raised. Same class as FIX-027.
#
# The stereo protocol (arms, configs) is the ALICIA one, verified from the
# snapshotted configs of results/ALICIA_matrix_*: all five _nopin variants,
# n=10, fastrtps/WiFi, five separate single-arm invocations.

set -u

REPO="/c/Users/homie/Desktop/ROS2-slam-hil"
MATLAB_DIR="$REPO/matlab"
PI="amaraly@192.168.1.60"

ARMS=(
  only_ov2_stereo_oracle_nopin
  only_ov2_stereo_oracle_fast_nopin
  only_orbslam2_stereo_oracle_nopin
  only_orbslam3_stereo_oracle_nopin
  only_rtabmap_stereo_oracle_nopin
)

BASELINES=("0.36:036" "0.42:042" "0.54:054")

STAMP=$(date +%Y%m%d_%H%M%S)
LOG="$REPO/scripts/hil_matrix/logs/baseline_sweep_${STAMP}.log"

exec > >(tee -a "$LOG") 2>&1

echo "================================================================"
echo "BASELINE SWEEP started $(date)"
echo "log: $LOG"
echo "================================================================"

for entry in "${BASELINES[@]}"; do
  B="${entry%%:*}"     # 0.36
  TAG="${entry##*:}"   # 036

  echo ""
  echo "################################################################"
  echo "### BASELINE ${B} m   (tag ${TAG})   start $(date)"
  echo "################################################################"

  # --- 1. swap the model -------------------------------------------------
  SRC="$MATLAB_DIR/hil_closed_loop_baseline_${TAG}.slx"
  if [[ ! -f "$SRC" ]]; then
    echo "FATAL: $SRC missing, skipping baseline $B"
    continue
  fi
  # never swap while MATLAB holds the model open
  running=$(powershell -Command "@(Get-Process -EA SilentlyContinue | Where-Object { \$_.Name -match '^(matlab|AutoVrtlEnv)' }).Count" | tr -d '\r')
  if [[ "$running" != "0" ]]; then
    echo "  MATLAB still running ($running proc) -- killing before model swap"
    powershell -Command "Get-Process -EA SilentlyContinue | Where-Object { \$_.Name -match '^(matlab|AutoVrtlEnv|MathWorksCrashReporter)' } | Stop-Process -Force -EA SilentlyContinue"
    sleep 5
  fi
  cp -f "$SRC" "$MATLAB_DIR/hil_closed_loop.slx"
  echo "  swapped model <- hil_closed_loop_baseline_${TAG}.slx"

  # --- 2. verify the .slx really encodes this baseline -------------------
  TMPX=$(mktemp -d)
  unzip -o -q "$MATLAB_DIR/hil_closed_loop.slx" -d "$TMPX"
  FOUND=$(grep -ho 'tmountOffset">\[0.2, -[0-9.]*' "$TMPX/simulink/systems/"*.xml 2>/dev/null | head -1 | grep -o '\-[0-9.]*$')
  rm -rf "$TMPX"
  if [[ "$FOUND" != "-$B" ]]; then
    echo "FATAL: .slx encodes '$FOUND', expected '-$B' -- ABORTING this baseline"
    continue
  fi
  echo "  verified .slx camera offset = $FOUND"

  # --- 3. sync the five calib files on the Pi ---------------------------
  if ! ssh "$PI" "python3 ~/sync_baseline.py $B"; then
    echo "FATAL: calib sync failed for $B -- ABORTING this baseline"
    continue
  fi

  # --- 4. run the five stereo arms --------------------------------------
  for cfg in "${ARMS[@]}"; do
    echo ""
    echo "--------------------------------------------------------------"
    echo "### ARM $cfg  @ baseline $B   start $(date)"
    echo "--------------------------------------------------------------"
    ( cd "$REPO" && py scripts/hil_matrix/run_matrix.py --config "$cfg" --run-tag "BASELINE${TAG}" )
    echo "### ARM $cfg  @ baseline $B   exit=$?  end $(date)"
  done

  echo "### BASELINE ${B} m COMPLETE $(date)"
done

echo ""
echo "================================================================"
echo "BASELINE SWEEP finished $(date)"
echo "================================================================"
