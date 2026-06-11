#!/bin/bash
# run_experiments.sh — sweep the YOLO experimental matrix on a SINGLE branch.
#
# Loops { placement config C1..C4 } x { backend } x { model } and, for each cell:
#   1. brings up the stack with the config's taskset core masks (camera, bridges,
#      OV2SLAM) — pinning via the shell, the proven-correct method (run_stack.sh),
#      NOT the launch-file prefix,
#   2. runs a live N-frame test through yolo_shm_producer (the measured component),
#      which writes 7-stamp per-frame telemetry to a per-cell CSV,
#   3. aggregates that CSV into a per-cell JSON latency-breakdown,
#   4. tears the stack down cleanly (so the Hailo VDevice is released) and advances.
#
# Everything is parameterized — no per-config branches:
#   ./run_experiments.sh                                  # full default matrix
#   ./run_experiments.sh --backends npu --configs "C1 C3" --models "yolov8n yolo26n"
#   ./run_experiments.sh --frames 200 --no-slam           # quick, SLAM-free
#
# Placement configs (4-core Pi5) — isolation -> contention ladder:
#   C1 Full-isolation : cam0  bridges0  YOLO=1       SLAM=2,3     (no core sharing)
#   C2 Shared-light   : cam0  bridges0  YOLO=1       SLAM=1,2,3   (SLAM touches YOLO's core)
#   C3 Co-schedule    : cam0  bridges0  YOLO=1,2,3   SLAM=1,2,3   (heavy overlap)
#   C4 Saturation     : cam0  bridges0  YOLO=0,1     SLAM=0,1     (everything fights)

set -u
cd "$(dirname "$0")/.."
WS="$(pwd)"

# ── defaults (all overridable) ─────────────────────────────────────────────────
MODELS="yolov8n yolov8s yolov8m yolov11n yolov11s yolov11m yolo26n yolo26s yolo26m"
BACKENDS="npu cpu"
CONFIGS="C1 C2 C3 C4"
FRAMES=500
CONF=0.25
IOU=0.45
RUN_SLAM=1
OUTDIR="${WS}/benchmarks/results/experiments"
CONTAINER="ros2_perception_stack"
CALIB="/workspace/camera_calib/ov5647_ov2slam.yaml"

while [[ $# -gt 0 ]]; do
    case "$1" in
        --models)   MODELS="$2";   shift 2 ;;
        --backends) BACKENDS="$2"; shift 2 ;;
        --configs)  CONFIGS="$2";  shift 2 ;;
        --frames)   FRAMES="$2";   shift 2 ;;
        --conf)     CONF="$2";     shift 2 ;;
        --iou)      IOU="$2";      shift 2 ;;
        --outdir)   OUTDIR="$2";   shift 2 ;;
        --no-slam)  RUN_SLAM=0;    shift   ;;
        *) echo "unknown arg: $1" >&2; exit 2 ;;
    esac
done

log()  { echo "[exp] $*"; }
sep()  { echo "────────────────────────────────────────────────────────────"; }
mkdir -p "$OUTDIR"

# ── core masks for a config -> globals CAM/BRIDGE/YOLO/SLAM + YOLO_NCORES ──────
config_cores() {
    case "$1" in
        C1) CAM=0; BRIDGE=0; YOLO=1;     SLAM=2,3   ;;
        C2) CAM=0; BRIDGE=0; YOLO=1;     SLAM=1,2,3 ;;
        C3) CAM=0; BRIDGE=0; YOLO=1,2,3; SLAM=1,2,3 ;;
        C4) CAM=0; BRIDGE=0; YOLO=0,1;   SLAM=0,1   ;;
        *) echo "[exp] unknown config '$1'" >&2; return 1 ;;
    esac
    # CPU thread count = number of cores in the YOLO mask (so onnxruntime uses
    # exactly the cores it is pinned to — see Layer 3 thread-bounding rationale).
    YOLO_NCORES=$(awk -F',' '{print NF}' <<<"$YOLO")
}

# ── teardown: kill everything + free shm so the next cell starts clean ─────────
teardown() {
    sudo docker kill "$CONTAINER" 2>/dev/null || true
    sudo docker rm   "$CONTAINER" 2>/dev/null || true
    sudo pkill -f yolo_shm_producer 2>/dev/null || true
    sudo pkill -f yolo_producer     2>/dev/null || true
    sudo pkill -f ovcam_producer    2>/dev/null || true
    sudo rm -f /dev/shm/ovcam_frames /dev/shm/yolo_shm \
               /dev/shm/sem.ovcam_ready /dev/shm/sem.yolo_ready 2>/dev/null || true
}
trap teardown EXIT

# ── run a Docker ROS2 node pinned to cores (mirrors run_stack.sh docker_node) ──
docker_node() {  # cores pkg node logfile [args...]
    local cores="$1" pkg="$2" node="$3" logfile="$4"; shift 4
    sudo docker exec -d "$CONTAINER" bash -lc "
        source /opt/ros/jazzy/setup.bash
        source /workspace/install/setup.bash
        export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
        export LD_LIBRARY_PATH=/workspace/opencv/build/lib:\${LD_LIBRARY_PATH:-}
        taskset -c ${cores} ros2 run ${pkg} ${node} $* > ${logfile} 2>&1
    "
}

# ── one matrix cell ────────────────────────────────────────────────────────────
run_cell() {
    local config="$1" backend="$2" model="$3"
    config_cores "$config" || return 1
    local threads=1
    [[ "$backend" == "cpu" ]] && threads="$YOLO_NCORES"

    local tag="${config}_${backend}_${model}"
    local csv="${OUTDIR}/${tag}.csv"
    local json="${OUTDIR}/${tag}.json"
    local plog="${OUTDIR}/${tag}.producer.log"

    sep
    log "CELL  config=$config backend=$backend model=$model | "\
"YOLO=$YOLO SLAM=$SLAM threads=$threads frames=$FRAMES"

    teardown; sleep 1

    # 1. container
    sudo docker run -d --entrypoint "" --name "$CONTAINER" \
        --net=host --ipc=host --privileged \
        --device=/dev/hailo0:/dev/hailo0 \
        -v "${WS}:/workspace" -v /tmp/.X11-unix:/tmp/.X11-unix \
        -e DISPLAY=:0 "$CONTAINER" sleep infinity >/dev/null
    sleep 1

    # 2. camera producer (host)
    taskset -c "$CAM" "${WS}/src/ovcam_producer/build/ovcam_producer" \
        --width 640 --height 480 --fps 60 > /tmp/ovcam_producer.log 2>&1 &
    for _ in $(seq 1 10); do [[ -e /dev/shm/ovcam_frames ]] && break; sleep 0.5; done
    if [[ ! -e /dev/shm/ovcam_frames ]]; then
        log "  ERROR: ovcam_producer failed (see /tmp/ovcam_producer.log) — skipping cell"
        return 1
    fi

    # 3. ovcam_bridge (feeds SLAM) + 4. OV2SLAM (the contention load)
    docker_node "$BRIDGE" ovcam_bridge ovcam_bridge_node /tmp/ovcam_bridge.log
    if [[ "$RUN_SLAM" == "1" ]]; then
        sudo docker exec -d "$CONTAINER" bash -lc "
            source /opt/ros/jazzy/setup.bash
            source /workspace/install/setup.bash
            export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
            export LD_LIBRARY_PATH=/workspace/opencv/build/lib:\${LD_LIBRARY_PATH:-}
            taskset -c ${SLAM} ros2 run ov2slam ov2slam_node ${CALIB} \
                > /tmp/ov2slam.log 2>&1"
        sleep 3   # let SLAM spin up so contention is present during measurement
    fi

    # 5. measured run: yolo_shm_producer (host), backgrounded so we can attach the
    #    yolo_bridge once /yolo_shm exists, then wait for the N-frame run to finish.
    taskset -c "$YOLO" python3 "${WS}/src/yolo_producer/yolo_shm_producer.py" \
        --backend "$backend" --model_id "$model" --threads "$threads" \
        --conf "$CONF" --iou "$IOU" --no-image \
        --max-frames "$FRAMES" --telemetry_csv "$csv" > "$plog" 2>&1 &
    local prod_pid=$!

    for _ in $(seq 1 40); do [[ -e /dev/shm/yolo_shm ]] && break; sleep 0.5; done
    docker_node "$BRIDGE" yolo_bridge yolo_bridge_node /tmp/yolo_bridge.log

    log "  measuring ${FRAMES} frames (pid $prod_pid)…"
    wait "$prod_pid" || log "  WARNING: producer exited non-zero (see $plog)"

    # 6. aggregate the per-frame telemetry CSV -> per-cell JSON breakdown
    if [[ -s "$csv" ]]; then
        python3 - "$csv" "$json" "$config" "$backend" "$model" "$threads" "$YOLO" "$SLAM" <<'PY'
import sys, csv, json
csvp, jsonp, cfg, backend, model, threads, ycores, scores = sys.argv[1:9]
rows = list(csv.DictReader(open(csvp)))
def stats(col):
    v = [float(r[col]) for r in rows if r.get(col) not in (None, "")]
    v = [x for x in v if x > 0] or v
    if not v: return {"mean": 0, "p50": 0, "p95": 0, "n": 0}
    s = sorted(v); n = len(s)
    p = lambda q: s[max(0, min(n - 1, round(q * (n - 1))))]
    return {"mean": sum(v) / n, "p50": p(.5), "p95": p(.95), "n": n}
stages = ["capture_read_ms", "preprocess_ms", "inference_ms",
          "nms_ms", "shmwrite_ms", "e2e_capture_to_commit_ms"]
out = {"config": cfg, "backend": backend, "model": model,
       "cpu_threads": int(threads), "yolo_cores": ycores, "slam_cores": scores,
       "frames": len(rows), "stages_ms": {k: stats(k) for k in stages}}
e = out["stages_ms"]["e2e_capture_to_commit_ms"]["mean"]
out["fps_capture_to_commit"] = round(1000.0 / e, 2) if e > 0 else 0.0
json.dump(out, open(jsonp, "w"), indent=2)
print(f"[exp]   aggregated {len(rows)} frames -> {jsonp} "
      f"(e2e {e:.1f}ms, {out['fps_capture_to_commit']} FPS)")
PY
    else
        log "  ERROR: no telemetry CSV produced (see $plog)"
    fi

    teardown; sleep 2   # settle: release Hailo VDevice before next cell
}

# ── sweep ──────────────────────────────────────────────────────────────────────
log "matrix: configs=[$CONFIGS] backends=[$BACKENDS] models=[$MODELS] frames=$FRAMES slam=$RUN_SLAM"
log "output: $OUTDIR"
command -v taskset >/dev/null || { echo "[exp] need taskset (util-linux)"; exit 1; }
[[ -x "${WS}/src/ovcam_producer/build/ovcam_producer" ]] \
    || { echo "[exp] ovcam_producer not built"; exit 1; }

for config in $CONFIGS; do
    for backend in $BACKENDS; do
        for model in $MODELS; do
            run_cell "$config" "$backend" "$model" \
                || log "cell ${config}/${backend}/${model} failed — continuing"
        done
    done
done

sep
log "sweep complete. Per-cell JSON + CSV in: $OUTDIR"
log "collect with:  python3 -c \"import json,glob;[print(json.load(open(f))) for f in glob.glob('$OUTDIR/*.json')]\""
