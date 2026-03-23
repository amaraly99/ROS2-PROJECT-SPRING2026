#!/bin/bash
# ─────────────────────────────────────────────────────────────────
# run_benchmark.sh — Run OV2SLAM on EuRoC sequences and collect metrics
#
# Runs INSIDE the Docker container. Launches OV2SLAM + euroc_publisher,
# waits for completion, then evaluates with evo.
#
# Usage (from inside container):
#   bash /workspace/benchmarks/run_benchmark.sh [config_name] [num_runs]
#
# Example:
#   bash /workspace/benchmarks/run_benchmark.sh baseline 3
#   bash /workspace/benchmarks/run_benchmark.sh solver_tuned 1
# ─────────────────────────────────────────────────────────────────

set -eo pipefail
# NOTE: -u (nounset) is intentionally omitted — sourcing ROS2 setup.bash
# references variables that may be unset (e.g. AMENT_TRACE_SETUP_FILES).

CONFIG_NAME="${1:-baseline}"
NUM_RUNS="${2:-1}"

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
RESULTS_DIR="$SCRIPT_DIR/results/${CONFIG_NAME}_$(date +%Y%m%d_%H%M%S)"

# EuRoC sequences (flat layout — no mav0/ wrapper)
EUROC_DIR="$PROJECT_DIR/datasets"
SEQUENCES=("MH01-Easy" "MH05-Hard")

# OV2SLAM config — select based on CONFIG_NAME
PARAMS_DIR="$PROJECT_DIR/src/ov2slam_ros/parameters_files/fast/euroc"
case "$CONFIG_NAME" in
    *solver_tuned*)  SLAM_CONFIG="$PARAMS_DIR/euroc_mono_solver_tuned.yaml" ;;
    *fe_combined*)   SLAM_CONFIG="$PARAMS_DIR/euroc_mono_fe_combined.yaml" ;;
    *fe_klt2*)       SLAM_CONFIG="$PARAMS_DIR/euroc_mono_fe_klt2.yaml" ;;
    *fe_maxdist65*)  SLAM_CONFIG="$PARAMS_DIR/euroc_mono_fe_maxdist65.yaml" ;;
    *)               SLAM_CONFIG="$PARAMS_DIR/euroc_mono.yaml" ;;
esac

# Source ROS2
source /opt/ros/jazzy/setup.bash
source /workspace/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export LD_LIBRARY_PATH=/workspace/opencv/build/lib:${LD_LIBRARY_PATH:-}

mkdir -p "$RESULTS_DIR"

echo "═══════════════════════════════════════════════════"
echo " OV2SLAM Benchmark: $CONFIG_NAME"
echo " Runs per sequence: $NUM_RUNS"
echo " Results dir: $RESULTS_DIR"
echo "═══════════════════════════════════════════════════"

# Check prerequisites
command -v evo_ape >/dev/null 2>&1 || echo "[WARN] evo not installed — install with: pip3 install evo"

# Check for EuRoC datasets
MISSING=0
for SEQ in "${SEQUENCES[@]}"; do
    if [ ! -d "$EUROC_DIR/$SEQ/cam0/data" ]; then
        echo "[WARN] Missing: $EUROC_DIR/$SEQ"
        MISSING=$((MISSING+1))
    fi
done
if [ "$MISSING" -eq "${#SEQUENCES[@]}" ]; then
    echo "[ERROR] No EuRoC sequences found. Download them first."
    exit 1
fi

# Save config snapshot
cp "$SLAM_CONFIG" "$RESULTS_DIR/config_snapshot.yaml"

# ── Helper: run one benchmark ─────────────────────────────────
run_one() {
    local SEQ="$1"
    local RUN="$2"
    local RUN_DIR="$3"
    local SEQ_DIR="$EUROC_DIR/$SEQ"

    mkdir -p "$RUN_DIR"
    cd "$RUN_DIR"

    echo "  Starting OV2SLAM..."

    # Launch OV2SLAM in background
    ros2 run ov2slam ov2slam_node "$SLAM_CONFIG" \
        > "$RUN_DIR/ov2slam_stdout.log" 2>&1 &
    local SLAM_PID=$!

    # Wait for SLAM to initialize
    sleep 3

    echo "  Starting EuRoC publisher (rate=1.0x)..."

    # Launch image publisher — it exits when done
    python3 "$SCRIPT_DIR/euroc_publisher.py" "$SEQ_DIR" --rate 1.0 \
        > "$RUN_DIR/publisher.log" 2>&1
    local PUB_EXIT=$?

    echo "  Publisher finished (exit=$PUB_EXIT). Waiting for SLAM to flush..."

    # Give SLAM a few seconds to process remaining frames and write results
    sleep 5

    # Send SIGINT to SLAM (triggers writeResults + CSV export)
    kill -INT "$SLAM_PID" 2>/dev/null || true

    # Wait up to 30s for graceful shutdown
    local WAIT=0
    while kill -0 "$SLAM_PID" 2>/dev/null && [ "$WAIT" -lt 30 ]; do
        sleep 1
        WAIT=$((WAIT+1))
    done
    kill -9 "$SLAM_PID" 2>/dev/null || true
    wait "$SLAM_PID" 2>/dev/null || true

    echo "  SLAM exited after ${WAIT}s."

    # Collect output files (OV2SLAM writes to CWD)
    for f in ov2slam_traj.txt ov2slam_traj_kitti.txt ov2slam_kfs_traj.txt \
             ov2slam_full_traj_wlc.txt ov2slam_full_traj_wlc_opt.txt \
             ov2slam_timings.csv; do
        if [ -f "$RUN_DIR/$f" ]; then
            echo "    Found: $f"
        fi
    done

    # Run evo evaluation
    local GT_FILE="$SEQ_DIR/state_groundtruth_estimate0/data.csv"
    local TRAJ_FILE="$RUN_DIR/ov2slam_traj.txt"

    if [ -f "$TRAJ_FILE" ] && [ -f "$GT_FILE" ]; then
        if command -v evo_ape >/dev/null 2>&1; then
            echo "  Evaluating ATE..."
            evo_ape euroc "$GT_FILE" "$TRAJ_FILE" \
                --align --correct_scale \
                --save_results "$RUN_DIR/ate_results.zip" \
                > "$RUN_DIR/ate_summary.txt" 2>&1 || true

            echo "  Evaluating RPE..."
            evo_rpe euroc "$GT_FILE" "$TRAJ_FILE" \
                --align \
                --save_results "$RUN_DIR/rpe_results.zip" \
                > "$RUN_DIR/rpe_summary.txt" 2>&1 || true
        else
            echo "  [SKIP] evo not installed — skipping ATE/RPE evaluation"
        fi
    else
        echo "  [WARN] Trajectory or ground truth not found — skipping evaluation"
        [ ! -f "$TRAJ_FILE" ] && echo "    Missing: $TRAJ_FILE"
        [ ! -f "$GT_FILE" ] && echo "    Missing: $GT_FILE"
    fi

    cd "$PROJECT_DIR"
}

# ── Main benchmark loop ───────────────────────────────────────
for SEQ in "${SEQUENCES[@]}"; do
    SEQ_DIR="$EUROC_DIR/$SEQ"
    if [ ! -d "$SEQ_DIR/cam0/data" ]; then
        echo ""
        echo "─── Sequence: $SEQ — SKIPPED (cam0/data not found) ───"
        continue
    fi

    echo ""
    echo "─── Sequence: $SEQ ───"

    for RUN in $(seq 1 "$NUM_RUNS"); do
        RUN_DIR="$RESULTS_DIR/${SEQ}/run_${RUN}"
        echo "  Run $RUN/$NUM_RUNS..."
        run_one "$SEQ" "$RUN" "$RUN_DIR"
        echo "  Done."
    done
done

echo ""
echo "═══════════════════════════════════════════════════"
echo " Benchmark complete. Results in: $RESULTS_DIR"
echo "═══════════════════════════════════════════════════"

# ── Generate summary CSV ──────────────────────────────────────
echo "config,sequence,run,ate_rmse,rpe_rmse,fe_mean_ms,ba_mean_ms,klt_mean_ms,total_frames" \
    > "$RESULTS_DIR/summary.csv"

for SEQ in "${SEQUENCES[@]}"; do
    for RUN in $(seq 1 "$NUM_RUNS"); do
        RUN_DIR="$RESULTS_DIR/${SEQ}/run_${RUN}"
        ATE="N/A"; RPE="N/A"; FE="N/A"; BA="N/A"; KLT="N/A"; FRAMES="N/A"

        # Parse ATE
        if [ -f "$RUN_DIR/ate_summary.txt" ]; then
            ATE=$(grep -oP 'rmse\s+\K[0-9.]+' "$RUN_DIR/ate_summary.txt" 2>/dev/null || echo "N/A")
        fi

        # Parse RPE
        if [ -f "$RUN_DIR/rpe_summary.txt" ]; then
            RPE=$(grep -oP 'rmse\s+\K[0-9.]+' "$RUN_DIR/rpe_summary.txt" 2>/dev/null || echo "N/A")
        fi

        # Parse timings CSV
        if [ -f "$RUN_DIR/ov2slam_timings.csv" ]; then
            FE=$(awk -F, '$1=="0.Full-Front_End"{printf "%.2f", $2}' "$RUN_DIR/ov2slam_timings.csv" 2>/dev/null || echo "N/A")
            BA=$(awk -F, '$1=="2.BA_Optimize"{printf "%.2f", $2}' "$RUN_DIR/ov2slam_timings.csv" 2>/dev/null || echo "N/A")
            KLT=$(awk -F, '$1=="2.FE_TM_KLT-Tracking"{printf "%.2f", $2}' "$RUN_DIR/ov2slam_timings.csv" 2>/dev/null || echo "N/A")
        fi

        # Count frames from trajectory
        if [ -f "$RUN_DIR/ov2slam_traj.txt" ]; then
            FRAMES=$(wc -l < "$RUN_DIR/ov2slam_traj.txt" 2>/dev/null || echo "N/A")
        fi

        echo "$CONFIG_NAME,$SEQ,$RUN,$ATE,$RPE,$FE,$BA,$KLT,$FRAMES" >> "$RESULTS_DIR/summary.csv"
    done
done

echo ""
echo "Summary CSV:"
cat "$RESULTS_DIR/summary.csv"
echo ""
echo "Summary written to: $RESULTS_DIR/summary.csv"
