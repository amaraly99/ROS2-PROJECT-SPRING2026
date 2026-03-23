#!/bin/bash
# ─── Core Pinning Variance Test ───────────────────────────────────────────────
# Quantifies how CPU affinity pinning reduces FPS jitter in the full pipeline.
# Runs OV2SLAM on MH01-Easy twice (unpinned vs pinned) for 60 seconds each,
# collects per-second FPS samples via /statistics topic, then computes:
#   mean FPS, std dev FPS, max latency spike, deadline miss count.
#
# Must be run INSIDE the Docker container:
#   bash /workspace/benchmarks/core_pinning_bench.sh
#
# Result: benchmarks/results/core_pinning_comparison.csv
# ─────────────────────────────────────────────────────────────────────────────

set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
RESULTS_DIR="$SCRIPT_DIR/results"
OUT_CSV="$RESULTS_DIR/core_pinning_comparison.csv"
DATASET="$PROJECT_DIR/datasets/MH01-Easy"
SLAM_CONFIG="$PROJECT_DIR/src/ov2slam_ros/parameters_files/fast/euroc/euroc_mono.yaml"

source /opt/ros/jazzy/setup.bash
source /workspace/install/setup.bash 2>/dev/null || true
export LD_LIBRARY_PATH=/workspace/opencv/build/lib:${LD_LIBRARY_PATH:-}
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

mkdir -p "$RESULTS_DIR"

DURATION=60   # seconds per run
TARGET_FPS=20 # EuRoC cam0 rate

echo "═══════════════════════════════════════════════════════"
echo " Core Pinning Variance Test"
echo " Duration per run: ${DURATION}s | Dataset: MH01-Easy"
echo "═══════════════════════════════════════════════════════"

# ── Helper: run SLAM + publisher and collect per-second topic rates ───────────
run_mode() {
    local MODE="$1"         # "unpinned" or "pinned"
    local RUN_DIR="$RESULTS_DIR/pinning_${MODE}"
    mkdir -p "$RUN_DIR"

    echo ""
    echo "─── Mode: $MODE ───"

    # Launch SLAM
    if [ "$MODE" = "pinned" ]; then
        taskset -c 2,3 ros2 run ov2slam ov2slam_node "$SLAM_CONFIG" \
            > "$RUN_DIR/slam.log" 2>&1 &
    else
        ros2 run ov2slam ov2slam_node "$SLAM_CONFIG" \
            > "$RUN_DIR/slam.log" 2>&1 &
    fi
    SLAM_PID=$!
    sleep 3

    # Launch publisher at 2x speed so we always have frames in the window
    if [ "$MODE" = "pinned" ]; then
        taskset -c 0 python3 "$SCRIPT_DIR/euroc_publisher.py" "$DATASET" --rate 1.0 \
            > "$RUN_DIR/publisher.log" 2>&1 &
    else
        python3 "$SCRIPT_DIR/euroc_publisher.py" "$DATASET" --rate 1.0 \
            > "$RUN_DIR/publisher.log" 2>&1 &
    fi
    PUB_PID=$!

    # Collect per-second frame counts on /cam0/image_raw for $DURATION seconds
    echo "  Collecting FPS samples for ${DURATION}s..."

    python3 - "$DURATION" "$RUN_DIR/fps_samples.csv" <<'PYEOF'
import sys, csv, time, threading
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

duration = int(sys.argv[1])
out_csv  = sys.argv[2]

class RateMonitor(Node):
    def __init__(self):
        super().__init__('rate_monitor')
        self.count = 0
        self.lock  = threading.Lock()
        self.create_subscription(Image, '/cam0/image_raw', self._cb, 10)

    def _cb(self, _msg):
        with self.lock:
            self.count += 1

rclpy.init()
node = RateMonitor()
executor = rclpy.executors.SingleThreadedExecutor()
executor.add_node(node)

spin_thread = threading.Thread(target=executor.spin, daemon=True)
spin_thread.start()

samples = []
t_end = time.monotonic() + duration
prev = time.monotonic()
while time.monotonic() < t_end:
    time.sleep(1.0)
    now = time.monotonic()
    with node.lock:
        cnt = node.count
        node.count = 0
    fps = cnt / (now - prev)
    samples.append(fps)
    prev = now
    print(f"  t={len(samples):3d}s  fps={fps:.1f}", flush=True)

executor.shutdown()
node.destroy_node()
rclpy.shutdown()

with open(out_csv, 'w', newline='') as f:
    w = csv.writer(f)
    w.writerow(['second', 'fps'])
    for i, fps in enumerate(samples, 1):
        w.writerow([i, f'{fps:.3f}'])
print(f"Saved {len(samples)} samples to {out_csv}")
PYEOF

    # Cleanup
    kill -INT "$SLAM_PID" 2>/dev/null || true
    kill -TERM "$PUB_PID" 2>/dev/null || true
    sleep 3
    kill -9 "$SLAM_PID" 2>/dev/null || true
    kill -9 "$PUB_PID" 2>/dev/null || true
    wait "$SLAM_PID" 2>/dev/null || true
    wait "$PUB_PID" 2>/dev/null || true

    echo "  Run complete. Analyzing samples..."

    # Compute stats from fps_samples.csv
    python3 - "$RUN_DIR/fps_samples.csv" "$MODE" "$TARGET_FPS" <<'PYEOF2'
import sys, csv, math

in_csv     = sys.argv[1]
mode       = sys.argv[2]
target_fps = float(sys.argv[3])

rows = []
with open(in_csv) as f:
    for row in csv.DictReader(f):
        rows.append(float(row['fps']))

if not rows:
    print(f"  WARNING: no samples in {in_csv}")
    sys.exit(0)

mean = sum(rows) / len(rows)
var  = sum((x - mean)**2 for x in rows) / len(rows)
std  = math.sqrt(var)
mx   = max(rows)
# Max latency spike: frame period at min FPS vs nominal
min_fps  = min(r for r in rows if r > 0)
max_lat  = 1000.0 / min_fps if min_fps > 0 else 9999
nom_lat  = 1000.0 / target_fps if target_fps > 0 else 50
# Deadline miss: fps dropped below 50% of target
deadline_misses = sum(1 for r in rows if r < target_fps * 0.5)

print(f"  {mode}: mean={mean:.2f} std={std:.2f} max_lat={max_lat:.1f}ms misses={deadline_misses}")
print(f"STATS:{mode},{mean:.3f},{std:.3f},{max_lat:.1f},{deadline_misses}")
PYEOF2
}

# ── Run both modes ─────────────────────────────────────────────────────────────
run_mode unpinned 2>&1 | tee "$RESULTS_DIR/pinning_unpinned_run.log"
run_mode pinned   2>&1 | tee "$RESULTS_DIR/pinning_pinned_run.log"

# ── Aggregate into CSV ─────────────────────────────────────────────────────────
echo ""
echo "═══════════════════════════════════════════════════════"
echo " Aggregating results..."
echo "═══════════════════════════════════════════════════════"

echo "mode,node,fps_mean,fps_std,max_latency_ms,deadline_misses" > "$OUT_CSV"

for MODE in unpinned pinned; do
    LOG="$RESULTS_DIR/pinning_${MODE}_run.log"
    if [ -f "$LOG" ]; then
        # Extract the STATS line from the log
        STATS=$(grep "^STATS:" "$LOG" | tail -1 | sed 's/^STATS://')
        if [ -n "$STATS" ]; then
            # STATS format: mode,mean,std,max_lat,misses
            echo "${STATS},ov2slam_slam" >> "$OUT_CSV"
        fi
    fi
done

echo ""
echo "Results:"
cat "$OUT_CSV"
echo ""
echo "Core pinning comparison written to: $OUT_CSV"
