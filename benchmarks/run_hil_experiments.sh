#!/bin/bash
# run_hil_experiments.sh — run the full HIL stack for one configuration,
# collect per-stage latency + rate data for N runs, aggregate, tear down.
#
# Designed as the inner loop of benchmarks/sweep_detectors.sh, but usable
# standalone. SLAM is only LAUNCHED here (unchanged binary/config) — no SLAM
# code or configuration is modified by this script.
#
# Usage:
#   ./benchmarks/run_hil_experiments.sh \
#       --detector yolo --model yolo26n --backend npu \
#       --slam-mode fast --duration 60 --runs 3 \
#       --label config_A_yolo26n --output-dir benchmarks/results/hil/
#
# Arguments:
#   --detector yolo|opencv|vlm        (default: yolo)
#   --model <registry id>             (default: yolo26n; ignored unless yolo)
#   --backend npu|cpu                 (default: npu;     ignored unless yolo)
#   --conf <float>                    (default: 0.20)
#   --slam-mode fast|no-slam          (default: fast — current hil_sim_ov2slam.yaml;
#                                      'accurate' intentionally unsupported until a
#                                      dedicated config exists — SLAM is Amar's domain)
#   --controller proportional         (default; 'hybrid' is deferred — errors out)
#   --duration <s>                    (default: 60)   measurement window per run
#   --warmup <s>                      (default: 35)   stack warm-up before probing
#   --runs <int>                      (default: 3)
#   --label <string>                  (required) output filename prefix
#   --output-dir <path>               (default: benchmarks/results/hil/)
#   --no-matlab-ctl                   skip MATLAB supervisor TCP control entirely
#                                     (sim assumed already running continuously)
#
# MATLAB coordination (drone reset per run):
#   If the supervisor (benchmarks/matlab/hil_run_supervisor.m) is listening on
#   ${MATLAB_HOST_IP}:${MATLAB_CTL_PORT} (default 192.168.56.1:55556), this
#   script sends "start" before each run (the supervisor [re]starts the
#   Simulink model — a restart re-applies initial conditions, resetting the
#   drone pose) and "stop" after. If the TCP connect fails, it falls back to
#   prompting you to restart Simulink manually (or proceeds without prompting
#   when NONINTERACTIVE=1).
#
# Per-run outputs in ${OUTPUT_DIR}:
#   <label>_run<N>.csv               probe rows (det/cmd/camera latencies)
#   <label>_run<N>_camera.csv        per-frame camera arrivals
#   <label>_run<N>_stats.json        machine-readable probe summary
#   <label>_run<N>_telemetry.csv     producer 7-stamp telemetry
#   <label>_run<N>_cam_stamps.csv    sim_camera_bridge stamp log
#   <label>_run<N>_stages.csv        per-stage latency table
#   <label>_run<N>_stack.log         run_stack_hil.sh + producer logs
#   <label>_summary.json             cross-run aggregate (written at the end)

set -u
cd "$(dirname "$0")/.."
WS="$(pwd)"

DETECTOR="yolo"; MODEL="yolo26n"; BACKEND="npu"; CONF="0.20"
SLAM_MODE="fast"; CONTROLLER="proportional"
DURATION=60; WARMUP=35; RUNS=3; LABEL=""; OUTPUT_DIR="benchmarks/results/hil"
MATLAB_CTL=true
MATLAB_HOST_IP="${MATLAB_HOST_IP:-192.168.56.1}"
MATLAB_CTL_PORT="${MATLAB_CTL_PORT:-55556}"
DDS="${DDS:-fastrtps}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

while [[ $# -gt 0 ]]; do
    case "$1" in
        --detector)   DETECTOR="$2"; shift 2 ;;
        --model)      MODEL="$2"; shift 2 ;;
        --backend)    BACKEND="$2"; shift 2 ;;
        --conf)       CONF="$2"; shift 2 ;;
        --slam-mode)  SLAM_MODE="$2"; shift 2 ;;
        --controller) CONTROLLER="$2"; shift 2 ;;
        --duration)   DURATION="$2"; shift 2 ;;
        --warmup)     WARMUP="$2"; shift 2 ;;
        --runs)       RUNS="$2"; shift 2 ;;
        --label)      LABEL="$2"; shift 2 ;;
        --output-dir) OUTPUT_DIR="$2"; shift 2 ;;
        --no-matlab-ctl) MATLAB_CTL=false; shift ;;
        *) echo "Unknown arg: $1" >&2; exit 1 ;;
    esac
done

log() { echo "[hil-exp] $*"; }
die() { echo "[hil-exp] ERROR: $*" >&2; exit 1; }

[[ -n "$LABEL" ]] || die "--label is required"
case "$CONTROLLER" in
    proportional) ;;
    hybrid) die "--controller hybrid is not implemented yet (deferred)" ;;
    *) die "unknown controller '$CONTROLLER'" ;;
esac
case "$SLAM_MODE" in
    fast|no-slam) ;;
    accurate) die "--slam-mode accurate needs a dedicated OV2SLAM config (Amar's domain) — unsupported here" ;;
    *) die "unknown slam-mode '$SLAM_MODE'" ;;
esac
mkdir -p "$OUTPUT_DIR"

# DDS env for in-container ROS2 commands — mirrors run_stack_hil.sh
case "$DDS" in
    cyclonedds) RMW="rmw_cyclonedds_cpp"; DDS_ENV="CYCLONEDDS_URI=file:///tmp/cyclonedds_hil.resolved.xml" ;;
    fastrtps)   RMW="rmw_fastrtps_cpp";   DDS_ENV="FASTRTPS_DEFAULT_PROFILES_FILE=/tmp/fastrtps_hil.resolved.xml" ;;
    *) die "unknown DDS '$DDS'" ;;
esac

in_container() {  # run a command inside the stack container with ROS2 sourced
    sudo docker exec ros2_perception_stack bash -lc "
        source /opt/ros/jazzy/setup.bash
        source /workspace/install/setup.bash
        export RMW_IMPLEMENTATION=${RMW}
        export ROS_DOMAIN_ID=${ROS_DOMAIN_ID}
        export ${DDS_ENV}
        $1
    "
}

matlab_send() {  # send a control word to the MATLAB supervisor; returns 1 on failure
    [[ "$MATLAB_CTL" == "true" ]] || return 0
    if echo "$1" | timeout 5 nc -w 3 "$MATLAB_HOST_IP" "$MATLAB_CTL_PORT" >/dev/null 2>&1; then
        log "MATLAB supervisor: sent '$1'"
        return 0
    fi
    return 1
}

matlab_start_or_prompt() {
    if ! matlab_send "start"; then
        log "WARNING: MATLAB supervisor not reachable at ${MATLAB_HOST_IP}:${MATLAB_CTL_PORT}"
        if [[ "${NONINTERACTIVE:-0}" == "1" ]]; then
            log "NONINTERACTIVE=1 — assuming the simulation is already running"
        else
            read -r -p "[hil-exp] (Re)start the Simulink simulation manually, then press Enter... "
        fi
    fi
}

wait_for_frames() {  # poll the camera SHM write_seq until frames flow (or timeout)
    python3 - "$1" <<'PYEOF'
import mmap, os, struct, sys, time
deadline = time.monotonic() + float(sys.argv[1])
path = "/dev/shm/ovcam_frames"
last = -1
while time.monotonic() < deadline:
    try:
        with open(path, "rb") as f:
            mm = mmap.mmap(f.fileno(), 128, mmap.MAP_SHARED, mmap.PROT_READ)
            seq = struct.unpack_from("<Q", mm, 64)[0]
            mm.close()
        if last >= 0 and seq > last:
            sys.exit(0)          # advancing — frames are flowing
        last = seq
    except (FileNotFoundError, ValueError):
        pass
    time.sleep(1.0)
sys.exit(1)
PYEOF
}

STACK_ARGS=(--detector "$DETECTOR" --conf "$CONF" --stamp-log)
[[ "$DETECTOR" == "yolo" ]] && STACK_ARGS+=(--model "$MODEL" --backend "$BACKEND")
[[ "$SLAM_MODE" == "no-slam" ]] && STACK_ARGS+=(--no-slam)

log "config: detector=$DETECTOR model=$MODEL backend=$BACKEND conf=$CONF slam=$SLAM_MODE"
log "        controller=$CONTROLLER duration=${DURATION}s warmup=${WARMUP}s runs=$RUNS"
log "        label=$LABEL → $OUTPUT_DIR"

OK_RUNS=0
for N in $(seq 1 "$RUNS"); do
    log "════ run $N/$RUNS ════"
    RUN_PREFIX="${OUTPUT_DIR}/${LABEL}_run${N}"
    STACK_LOG="${RUN_PREFIX}_stack.log"
    PROBE_CSV="/tmp/hil_probe_${LABEL}_run${N}.csv"
    TELEM_CSV="/tmp/yolo_telemetry.csv"

    # 1. start the stack (creates the container + camera SHM)
    TELEMETRY_CSV="$TELEM_CSV" ./run_stack_hil.sh "${STACK_ARGS[@]}" \
        > "$STACK_LOG" 2>&1
    if [[ $? -ne 0 ]]; then
        log "STARTUP_FAILED run $N (stack launch — see $STACK_LOG)"
        ./run_stack_hil.sh stop >> "$STACK_LOG" 2>&1
        continue
    fi

    # 2. (re)start the simulation — restart resets the drone pose
    matlab_start_or_prompt

    # 3. camera frames must flow before the warm-up clock starts
    if ! wait_for_frames 45; then
        log "STARTUP_FAILED run $N (no camera frames within 45 s — is MATLAB publishing?)"
        matlab_send "stop" || true
        ./run_stack_hil.sh stop >> "$STACK_LOG" 2>&1
        continue
    fi
    log "camera frames flowing — warming up ${WARMUP}s (SLAM init + first keyframes)"
    sleep "$WARMUP"

    # 4. closed loop must be alive: a /cmd_vel message within 45 s
    if ! in_container "timeout 45 ros2 topic echo --once /cmd_vel > /dev/null 2>&1"; then
        log "STARTUP_FAILED run $N (no /cmd_vel within 45 s)"
        matlab_send "stop" || true
        ./run_stack_hil.sh stop >> "$STACK_LOG" 2>&1
        continue
    fi

    # 5. measure
    log "collecting ${DURATION}s of data..."
    in_container "python3 /workspace/benchmarks/e2e_latency_probe.py \
        --duration ${DURATION} --output ${PROBE_CSV}" >> "$STACK_LOG" 2>&1

    # 6. tear down before copying (flushes telemetry + stamp CSVs)
    matlab_send "stop" || true
    ./run_stack_hil.sh stop >> "$STACK_LOG" 2>&1
    sleep 5

    # 7. collect artifacts
    BASE="${PROBE_CSV%.csv}"
    cp -f "$PROBE_CSV"            "${RUN_PREFIX}.csv"            2>/dev/null || true
    cp -f "${BASE}_camera.csv"    "${RUN_PREFIX}_camera.csv"     2>/dev/null || true
    cp -f "${BASE}_stats.json"    "${RUN_PREFIX}_stats.json"     2>/dev/null || true
    cp -f "$TELEM_CSV"            "${RUN_PREFIX}_telemetry.csv"  2>/dev/null || true
    cp -f /tmp/sim_cam_stamps.csv "${RUN_PREFIX}_cam_stamps.csv" 2>/dev/null || true
    cat /tmp/hil_launch.log /tmp/yolo_producer.log >> "$STACK_LOG" 2>/dev/null || true

    if [[ ! -s "${RUN_PREFIX}.csv" ]]; then
        log "RUN_FAILED run $N (probe produced no data)"
        continue
    fi

    # 8. per-run stage table
    python3 benchmarks/latency_stage_report.py \
        --telemetry "${RUN_PREFIX}_telemetry.csv" \
        --probe "${RUN_PREFIX}.csv" \
        --bridge-log "${RUN_PREFIX}_cam_stamps.csv" \
        --out "${RUN_PREFIX}_stages.csv" || true

    OK_RUNS=$((OK_RUNS + 1))
    log "run $N complete ✓"
done

# ── aggregate across successful runs ──────────────────────────────────────────
log "aggregating ${OK_RUNS}/${RUNS} successful runs..."
python3 - "$OUTPUT_DIR" "$LABEL" "$RUNS" "$DETECTOR" "$MODEL" "$BACKEND" "$SLAM_MODE" <<'PYEOF'
import glob, json, math, os, sys
outdir, label, runs, detector, model, backend, slam = sys.argv[1:8]

stats = []
for p in sorted(glob.glob(os.path.join(outdir, f"{label}_run*_stats.json"))):
    with open(p) as f:
        stats.append(json.load(f))

def agg(path_fn):
    vals = [path_fn(s) for s in stats if path_fn(s)]
    if not vals:
        return {"mean": 0.0, "std": 0.0, "n": 0}
    m = sum(vals) / len(vals)
    sd = math.sqrt(sum((v - m) ** 2 for v in vals) / len(vals)) if len(vals) > 1 else 0.0
    return {"mean": round(m, 3), "std": round(sd, 3), "n": len(vals)}

summary = {
    "label": label,
    "detector": detector, "model": model, "backend": backend, "slam_mode": slam,
    "runs_requested": int(runs), "runs_ok": len(stats),
    "cmd_vel_hz":  agg(lambda s: s["cmd_vel"]["hz_mean"]),
    "det_hz":      agg(lambda s: s["det"]["hz_mean"]),
    "camera_hz":   agg(lambda s: s["sim_camera"]["hz_mean"]),
    "ovcam_hz":    agg(lambda s: s["ovcam"]["hz_mean"]),
    "e2e_det_to_cmd_ms_mean": agg(lambda s: s["e2e_det_to_cmd_ms"]["mean"]),
    "e2e_det_to_cmd_ms_p95":  agg(lambda s: s["e2e_det_to_cmd_ms"]["p95"]),
    "cam_to_det_ms_mean":     agg(lambda s: s["cam_to_det_ms"]["mean"]),
    "cam_to_det_ms_p95":      agg(lambda s: s["cam_to_det_ms"]["p95"]),
}
out = os.path.join(outdir, f"{label}_summary.json")
with open(out, "w") as f:
    json.dump(summary, f, indent=2)
print(json.dumps(summary, indent=2))
print(f"→ {out}")
PYEOF

[[ "$OK_RUNS" -gt 0 ]] || die "all $RUNS runs failed"
log "done: $OK_RUNS/$RUNS runs ok"
