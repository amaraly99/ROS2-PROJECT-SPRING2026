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
#   --placement A|B|C|D               workload-placement preset (paper tab:placement).
#                                     Sets backend + SLAM/detector/node core affinity:
#                                       A  npu, slam 2,3 · detector 1 · nodes 0-3 (baseline)
#                                       B  cpu, slam 2,3 · detector 1 · nodes 0-3 (CPU infer)
#                                       C  npu, slam 0   · detector 1 · nodes 0   (SLAM shares core 0)
#                                       D  npu, slam 0   · detector 0 · nodes 0   (all on core 0)
#   --slam-cores <list>               taskset CPU list for OV2SLAM       (default 2,3)
#   --detector-core <core>            taskset CPU for the host producer  (default 1)
#   --node-cores <list>               taskset CPU list for the ROS nodes (default 0-3)
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
# Placement / core-affinity (empty = use run_stack_hil.sh defaults = baseline Config A)
PLACEMENT=""; SLAM_CORES=""; DETECTOR_CORE=""; NODE_CORES=""
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
        --placement)     PLACEMENT="$2"; shift 2 ;;
        --slam-cores)    SLAM_CORES="$2"; shift 2 ;;
        --detector-core) DETECTOR_CORE="$2"; shift 2 ;;
        --node-cores)    NODE_CORES="$2"; shift 2 ;;
        *) echo "Unknown arg: $1" >&2; exit 1 ;;
    esac
done

# Expand a --placement preset into backend + core-affinity knobs. Individual
# --slam-cores/--detector-core/--node-cores set on the command line win over the
# preset (only fill the ones the preset didn't already receive explicitly).
if [[ -n "$PLACEMENT" ]]; then
    case "$PLACEMENT" in
        A) BACKEND="npu"; : "${SLAM_CORES:=2,3}"; : "${DETECTOR_CORE:=1}"; : "${NODE_CORES:=0-3}" ;;
        B) BACKEND="cpu"; : "${SLAM_CORES:=2,3}"; : "${DETECTOR_CORE:=1}"; : "${NODE_CORES:=0-3}" ;;
        C) BACKEND="npu"; : "${SLAM_CORES:=0}";   : "${DETECTOR_CORE:=1}"; : "${NODE_CORES:=0}" ;;
        D) BACKEND="npu"; : "${SLAM_CORES:=0}";   : "${DETECTOR_CORE:=0}"; : "${NODE_CORES:=0}" ;;
        *) echo "unknown --placement '$PLACEMENT' (want A|B|C|D)" >&2; exit 1 ;;
    esac
fi

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
# Core-affinity passthrough (placement sweep). Only forwarded when set, so the
# default invocation reproduces the baseline layout via run_stack_hil.sh defaults.
[[ -n "$SLAM_CORES"    ]] && STACK_ARGS+=(--slam-cores "$SLAM_CORES")
[[ -n "$DETECTOR_CORE" ]] && STACK_ARGS+=(--detector-core "$DETECTOR_CORE")
[[ -n "$NODE_CORES"    ]] && STACK_ARGS+=(--node-cores "$NODE_CORES")

log "config: detector=$DETECTOR model=$MODEL backend=$BACKEND conf=$CONF slam=$SLAM_MODE"
log "        controller=$CONTROLLER duration=${DURATION}s warmup=${WARMUP}s runs=$RUNS"
[[ -n "$PLACEMENT$SLAM_CORES$DETECTOR_CORE$NODE_CORES" ]] && \
    log "        placement=${PLACEMENT:-custom} slam_cores=${SLAM_CORES:-default} detector_core=${DETECTOR_CORE:-default} node_cores=${NODE_CORES:-default}"
log "        label=$LABEL → $OUTPUT_DIR"

OK_RUNS=0
for N in $(seq 1 "$RUNS"); do
    log "════ run $N/$RUNS ════"
    RUN_PREFIX="${OUTPUT_DIR}/${LABEL}_run${N}"
    STACK_LOG="${RUN_PREFIX}_stack.log"
    PROBE_CSV="/tmp/hil_probe_${LABEL}_run${N}.csv"
    TELEM_CSV="/tmp/yolo_telemetry.csv"

    # clear stale per-run scratch so a failed/short run can't copy previous data
    sudo rm -f /tmp/visp_telemetry.csv /tmp/sim_cam_stamps.csv "$TELEM_CSV" 2>/dev/null || true

    # 1. (re)start the simulation FIRST — restart resets the drone pose.
    #    Order matters: the Windows-side frame publisher keeps streaming the
    #    previous run's final frame (drone parked at the sign) after a model
    #    stop, so starting the stack first lets visp_servo see that stale
    #    end-scene and latch a bogus REACHED on /visp/state before the reset.
    matlab_start_or_prompt

    # 2. start the stack (creates the container + camera SHM). The drone just
    #    hovers at its initial condition until /cmd_vel starts flowing, so the
    #    ~20 s of stack boot costs nothing mission-wise.
    TELEMETRY_CSV="$TELEM_CSV" ./run_stack_hil.sh "${STACK_ARGS[@]}" \
        > "$STACK_LOG" 2>&1
    if [[ $? -ne 0 ]]; then
        log "STARTUP_FAILED run $N (stack launch — see $STACK_LOG)"
        ./run_stack_hil.sh stop >> "$STACK_LOG" 2>&1
        continue
    fi

    # 3. camera frames must flow before the warm-up clock starts
    if ! wait_for_frames 45; then
        log "STARTUP_FAILED run $N (no camera frames within 45 s — is MATLAB publishing?)"
        matlab_send "stop" || true
        ./run_stack_hil.sh stop >> "$STACK_LOG" 2>&1
        continue
    fi
    # 3b. start the mission-state monitor NOW (drone just reset → t0 ≈ mission
    #     start). It captures the SEARCHING→APPROACHING→REACHED timeline for the
    #     time-to-REACHED metric, spanning warmup + measurement. Writes the JSON
    #     incrementally so it survives a hard kill.
    MISSION_JSON="/tmp/mission_${LABEL}_run${N}.json"
    sudo rm -f "$MISSION_JSON"
    MON_DUR=$(( WARMUP + 45 + DURATION + 60 ))
    in_container "python3 /workspace/benchmarks/mission_monitor.py \
        --output ${MISSION_JSON} --duration ${MON_DUR}" >> "$STACK_LOG" 2>&1 &
    MON_PID=$!

    # 3c. start the /cmd_vel liveness probe NOW too, in parallel with warm-up —
    # not after it. /cmd_vel is plain volatile QoS (no transient_local replay),
    # and this profile's SIMPLE discovery only re-announces every ~20s
    # (config/hil/fastrtps_hil.xml), so a freshly-spawned `ros2 topic echo`
    # process can burn most of a 45s post-warmup window just completing DDS
    # discovery — producing a false STARTUP_FAILED even though cmd_vel is
    # already flowing (confirmed via mission_monitor reaching APPROACHING on
    # runs this previously flagged as failed). Starting it alongside warmup
    # gives discovery the full WARMUP+45s to complete instead.
    CMDVEL_TIMEOUT=$((WARMUP + 45))
    in_container "timeout ${CMDVEL_TIMEOUT} ros2 topic echo --once /cmd_vel > /dev/null 2>&1" &
    CMDVEL_PID=$!

    log "camera frames flowing — warming up ${WARMUP}s (SLAM init + first keyframes)"
    sleep "$WARMUP"

    # 4. closed loop must be alive: a /cmd_vel message within the probe window
    if ! wait "$CMDVEL_PID"; then
        log "STARTUP_FAILED run $N (no /cmd_vel within ${CMDVEL_TIMEOUT} s)"
        kill "$MON_PID" 2>/dev/null || true
        sudo docker exec ros2_perception_stack pkill -f mission_monitor.py 2>/dev/null || true
        matlab_send "stop" || true
        ./run_stack_hil.sh stop >> "$STACK_LOG" 2>&1
        continue
    fi

    # 5. measure
    log "collecting ${DURATION}s of data..."
    in_container "python3 /workspace/benchmarks/e2e_latency_probe.py \
        --duration ${DURATION} --output ${PROBE_CSV}" >> "$STACK_LOG" 2>&1

    # 6. tear down before copying (flushes telemetry + stamp CSVs)
    kill "$MON_PID" 2>/dev/null || true
    sudo docker exec ros2_perception_stack pkill -TERM -f mission_monitor.py 2>/dev/null || true
    sleep 1
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
    cp -f /tmp/visp_telemetry.csv "${RUN_PREFIX}_visp.csv"       2>/dev/null || true
    cp -f "$MISSION_JSON"         "${RUN_PREFIX}_mission.json"   2>/dev/null || true
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

import csv as _csv
def _frozen_frame_ratio(telem_path):
    """Return unique_hashes/total_frames from frame_hash column; None if missing."""
    if not os.path.exists(telem_path):
        return None
    with open(telem_path) as f:
        rows = list(_csv.DictReader(f))
    if not rows or "frame_hash" not in rows[0]:
        return None
    hashes = [r["frame_hash"] for r in rows if r.get("frame_hash")]
    return len(set(hashes)) / len(hashes) if hashes else None

frozen_ratios = []
for n in range(1, int(runs) + 1):
    r = _frozen_frame_ratio(os.path.join(outdir, f"{label}_run{n}_telemetry.csv"))
    if r is not None:
        frozen_ratios.append(r)
frozen_ratio_mean = round(sum(frozen_ratios) / len(frozen_ratios), 4) if frozen_ratios else None
frozen_flag = frozen_ratio_mean is not None and frozen_ratio_mean < 0.05

# ── detector-only latency stages (S3-S7) aggregated from per-run stage tables ──
def _mean_std(vals):
    vals = [v for v in vals if v is not None]
    if not vals:
        return {"mean": None, "std": None, "n": 0}
    m = sum(vals) / len(vals)
    sd = math.sqrt(sum((v - m) ** 2 for v in vals) / len(vals)) if len(vals) > 1 else 0.0
    return {"mean": round(m, 3), "std": round(sd, 3), "n": len(vals)}

# stage code -> output key (only the detector-involved stages)
_STAGE_KEYS = {"S3": "s3_frame_age", "S4": "s4_preprocess", "S5": "s5_inference",
               "S6": "s6_nms", "S7": "s7_shm_write", "S3-S8": "det_path"}
_stage_runs = {k: [] for k in _STAGE_KEYS.values()}
_stage_runs["detector_compute"] = []
for n in range(1, int(runs) + 1):
    sp = os.path.join(outdir, f"{label}_run{n}_stages.csv")
    if not os.path.exists(sp):
        continue
    by_code = {}
    with open(sp) as f:
        for row in _csv.DictReader(f):
            try:
                by_code[row["stage"]] = float(row["mean_ms"])
            except (KeyError, ValueError):
                pass
    for code, key in _STAGE_KEYS.items():
        if code in by_code:
            _stage_runs[key].append(by_code[code])
    # detector compute = preprocess + inference + nms + shm write
    comp = [by_code.get(c) for c in ("S4", "S5", "S6", "S7")]
    if all(c is not None for c in comp):
        _stage_runs["detector_compute"].append(sum(comp))
detector_latency_ms = {k: _mean_std(v) for k, v in _stage_runs.items()}

# ── mission behaviour metrics aggregated from per-run mission JSONs ────────────
_missions = []
for n in range(1, int(runs) + 1):
    mp = os.path.join(outdir, f"{label}_run{n}_mission.json")
    if os.path.exists(mp):
        try:
            with open(mp) as f:
                _missions.append(json.load(f))
        except (ValueError, OSError):
            pass
if _missions:
    n_reached = sum(1 for m in _missions if m.get("reached"))
    mission = {
        "runs": len(_missions),
        "reached_success_rate": round(n_reached / len(_missions), 3),
        "time_to_reached_s":    _mean_std([m.get("time_to_reached_s") for m in _missions]),
        "approach_duration_s":  _mean_std([m.get("approach_duration_s") for m in _missions]),
        "time_to_first_approach_s": _mean_std([m.get("time_to_first_approach_s") for m in _missions]),
        "n_reacquire":          _mean_std([m.get("n_reacquire") for m in _missions]),
        "n_lost":               _mean_std([m.get("n_lost") for m in _missions]),
    }
else:
    mission = {"runs": 0, "reached_success_rate": None}

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
    "detector_latency_ms": detector_latency_ms,
    "mission": mission,
    "unique_frame_ratio": frozen_ratio_mean,
    "frozen_camera_flag": frozen_flag,
}
out = os.path.join(outdir, f"{label}_summary.json")
with open(out, "w") as f:
    json.dump(summary, f, indent=2)
if frozen_flag:
    print(f"[sweep] WARNING: {label} unique_frame_ratio={frozen_ratio_mean:.3f} < 0.05 — FROZEN CAMERA (discard run)", flush=True)
print(json.dumps(summary, indent=2))
print(f"→ {out}")
PYEOF

[[ "$OK_RUNS" -gt 0 ]] || die "all $RUNS runs failed"
log "done: $OK_RUNS/$RUNS runs ok"
