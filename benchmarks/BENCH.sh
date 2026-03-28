#!/usr/bin/env bash

# OV2SLAM EuRoC Benchmark Runner for ROS2
#
# Assumptions:
# 1. ROS2 is sourced in the shell that launches this script.
# 2. `ros2 run ov2slam ov2slam_node PARAMS.yaml` starts the node as `/ov2slam_node`.
# 3. EuRoC sequence directories contain at least one `.db3` bag file and one ground-truth
#    file: either `SEQ.txt` or `gt.tum`.
# 4. Ground-truth files in this repo already match TUM pose format:
#    `timestamp tx ty tz qx qy qz qw`.
# 5. Single-file ROS2 bags can be played directly from the `.db3` path. Split bags require
#    `metadata.yaml` so `ros2 bag play` can discover all segments.
# 6. The timestamped OV2SLAM trajectories suitable for `evo_ape tum` are:
#    `ov2slam_fullba_kfs_traj.txt`, `ov2slam_kfs_traj.txt`, `ov2slam_traj.txt`.
#    The older `ov2slam_full_traj_wlc*.txt` files are not used for evo here because this
#    codebase writes frame indices there, not timestamps.
#
# Default output tree:
#   <output-root>/<timestamp>/
#   ├── benchmark.log
#   ├── params_used.yaml
#   ├── run_config.json
#   ├── summary.txt
#   ├── summary.json
#   └── <SEQ_NAME>/
#       ├── ov2slam.log
#       ├── bag_play.log
#       ├── gt.tum
#       ├── ov2slam_traj.txt
#       ├── ov2slam_kfs_traj.txt
#       ├── ov2slam_fullba_kfs_traj.txt
#       ├── ov2slam_timings.csv
#       ├── <SEQ>_evo.txt
#       ├── <SEQ>_ape_xy.png
#       └── <SEQ>_evo.zip

set -uo pipefail

SCRIPT_PATH="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)/$(basename -- "${BASH_SOURCE[0]}")"
SCRIPT_DIR="$(cd -- "$(dirname -- "$SCRIPT_PATH")" && pwd)"
LOCAL_WORKSPACE_ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd)"
if [[ -d /workspace ]]; then
  DEFAULT_WORKSPACE_ROOT="/workspace"
else
  DEFAULT_WORKSPACE_ROOT="$LOCAL_WORKSPACE_ROOT"
fi

BAG_PLAY_RATE="1.0"
FINAL_CLOCK_SEC="1450000000"
FINAL_CLOCK_NANOSEC="0"
DEFAULT_NODE_NAME="/ov2slam_node"
RESULTS_TIMEZONE="Asia/Dubai"
OV2SLAM_STARTUP_SETTLE_SEC="2"
CLOCK_START_TIMEOUT="10"
SEQUENCE_RETRIES="2"
INTER_RUN_DELAY_SEC="10"
MONITOR_SAMPLE_SEC="1.0"

declare -a BEST_TRAJECTORY_CANDIDATES=(
  "ov2slam_fullba_kfs_traj.txt|final optimized keyframe trajectory after full BA; best final estimate when do_full_ba=1"
  "ov2slam_kfs_traj.txt|optimized keyframe trajectory after local BA / loop closure; primary fallback"
  "ov2slam_traj.txt|raw frame trajectory; fallback only when optimized keyframe outputs are absent"
)

declare -a PAPER_TRAJECTORY_CANDIDATES=(
  "ov2slam_kfs_traj.txt|paper-style choice: keyframe trajectory after online local BA / loop closure, without end-of-sequence full BA"
  "ov2slam_traj.txt|paper-style fallback: raw frame trajectory when keyframe output is missing"
  "ov2slam_fullba_kfs_traj.txt|last-resort fallback: full BA result exists, but this is an offline refinement beyond the paper's fully-online setting"
)

declare -a UNSUPPORTED_EVO_TRAJECTORIES=(
  "ov2slam_full_traj_wlc.txt"
  "ov2slam_full_traj_wlc_opt.txt"
)

LOG_FILE=""
LAST_ERROR=""
STOP_AFTER_CURRENT=0
CURRENT_OV_PID=""
CURRENT_BAG_PID=""
CURRENT_MONITOR_PID=""
CONFIG_FILE=""

declare -a RESULT_SEQS=()
declare -a RESULT_STATUS=()
declare -a RESULT_RMSE=()
declare -a RESULT_TRAJ=()
declare -a RESULT_REASON=()
declare -a RESULT_WARNINGS=()
declare -a RESULT_ERROR=()
declare -a RESULT_GT_VERDICT=()

SEQ_RESULT_STATUS=""
SEQ_RESULT_RMSE=""
SEQ_RESULT_TRAJ=""
SEQ_RESULT_REASON=""
SEQ_RESULT_WARNINGS=""
SEQ_RESULT_ERROR=""
SEQ_RESULT_GT_VERDICT=""

WORKSPACE_ROOT=""
DATASET_ROOT=""
OUTPUT_BASE=""
RESULTS_ROOT=""
RESULTS_NAME=""
PARAMS_FILE=""
REQUESTED_CAMERA="stereo"
REQUESTED_SPEED="accurate"
TRAJECTORY_POLICY="paper"
NODE_NAME="$DEFAULT_NODE_NAME"
NODE_WAIT_TIMEOUT="45"
BAG_TIMEOUT="1200"
AUTO_EXIT_TIMEOUT="600"
SIGINT_TIMEOUT="30"
OUTPUT_ROOT_EXPLICIT=0
OV2SLAM_CORES=""

declare -a SELECTED_SEQUENCES=()
declare -a REQUESTED_SEQUENCES=()
SEQ_RANGE_START=""
SEQ_RANGE_END=""

json_escape() {
  local s="${1:-}"
  s=${s//\\/\\\\}
  s=${s//\"/\\\"}
  s=${s//$'\n'/\\n}
  s=${s//$'\r'/\\r}
  s=${s//$'\t'/\\t}
  printf '%s' "$s"
}

json_string() {
  printf '"%s"' "$(json_escape "${1:-}")"
}

timestamp() {
  TZ="$RESULTS_TIMEZONE" date +"%H:%M:%S"
}

datetime_full() {
  TZ="$RESULTS_TIMEZONE" date +"%Y-%m-%d %H:%M:%S %Z"
}

datetime_compact() {
  TZ="$RESULTS_TIMEZONE" date +"%Y%m%d_%H%M%S"
}

emit_log() {
  local level="$1"
  shift
  local msg="$*"
  local line
  printf -v line "%s %-7s %s\n" "$(timestamp)" "$level" "$msg"
  printf '%s' "$line"
  if [[ -n "$LOG_FILE" ]]; then
    printf '%s' "$line" >>"$LOG_FILE"
  fi
}

log_info() { emit_log "INFO" "$*"; }
log_warn() { emit_log "WARNING" "$*"; }
log_error() { emit_log "ERROR" "$*"; }
log_debug() { emit_log "DEBUG" "$*"; }

log_blank() {
  printf '\n'
  if [[ -n "$LOG_FILE" ]]; then
    printf '\n' >>"$LOG_FILE"
  fi
}

die() {
  LAST_ERROR="$*"
  log_error "$LAST_ERROR"
  exit 1
}

usage() {
  cat <<EOF
Usage:
  BENCH.sh [config.yaml]
  BENCH.sh --config config.yaml

Options:
  --config PATH
  -h, --help

Examples:
  ./BENCH.sh
  ./BENCH.sh benchmarks/BENCH.yaml

Default config path:
  $SCRIPT_DIR/BENCH.yaml
EOF
}

cleanup_active_processes() {
  if [[ -n "$CURRENT_BAG_PID" ]] && kill -0 "$CURRENT_BAG_PID" 2>/dev/null; then
    kill -KILL -- "-$CURRENT_BAG_PID" 2>/dev/null || true
    wait "$CURRENT_BAG_PID" 2>/dev/null || true
  fi
  if [[ -n "$CURRENT_OV_PID" ]] && kill -0 "$CURRENT_OV_PID" 2>/dev/null; then
    kill -KILL -- "-$CURRENT_OV_PID" 2>/dev/null || true
    wait "$CURRENT_OV_PID" 2>/dev/null || true
  fi
  if [[ -n "$CURRENT_MONITOR_PID" ]] && kill -0 "$CURRENT_MONITOR_PID" 2>/dev/null; then
    kill -TERM "$CURRENT_MONITOR_PID" 2>/dev/null || true
    wait "$CURRENT_MONITOR_PID" 2>/dev/null || true
  fi
}

trap cleanup_active_processes EXIT
trap 'log_warn "Ctrl+C received. Finishing the current sequence, then stopping."; STOP_AFTER_CURRENT=1; trap - INT' INT

parse_args() {
  local positional_config=""
  while (($# > 0)); do
    case "$1" in
      --config)
        [[ $# -ge 2 ]] || die "--config requires a value."
        CONFIG_FILE="$2"
        shift 2
        ;;
      -h|--help)
        usage
        exit 0
        ;;
      *)
        if [[ -z "$positional_config" && "$1" != --* ]]; then
          positional_config="$1"
          shift
        else
          die "Unknown argument: $1"
        fi
        ;;
    esac
  done

  if [[ -n "$CONFIG_FILE" && -n "$positional_config" ]]; then
    die "Use either --config PATH or a single positional config path, not both."
  fi

  if [[ -z "$CONFIG_FILE" ]]; then
    CONFIG_FILE="${positional_config:-$SCRIPT_DIR/BENCH.yaml}"
  fi
}

load_config_file() {
  [[ -f "$CONFIG_FILE" ]] || die "Config file missing: $CONFIG_FILE"
  CONFIG_FILE="$(cd -- "$(dirname -- "$CONFIG_FILE")" 2>/dev/null && pwd)/$(basename -- "$CONFIG_FILE")"

  REQUESTED_SEQUENCES=()
  local config_lines=""
  if ! config_lines="$(python3 - "$CONFIG_FILE" <<'PY'
import re
import sys
from pathlib import Path


def strip_comment(text: str) -> str:
    out = []
    in_single = False
    in_double = False
    for ch in text:
        if ch == "'" and not in_double:
            in_single = not in_single
            out.append(ch)
            continue
        if ch == '"' and not in_single:
            in_double = not in_double
            out.append(ch)
            continue
        if ch == "#" and not in_single and not in_double:
            break
        out.append(ch)
    return "".join(out).rstrip()


def unquote(value: str) -> str:
    value = value.strip()
    if value in ("null", "~"):
        return ""
    if len(value) >= 2 and value[0] == value[-1] and value[0] in ("'", '"'):
        return value[1:-1]
    return value


def split_inline_list(value: str):
    inner = value.strip()[1:-1].strip()
    if not inner:
        return []
    items = []
    buf = []
    in_single = False
    in_double = False
    for ch in inner:
        if ch == "'" and not in_double:
            in_single = not in_single
            buf.append(ch)
            continue
        if ch == '"' and not in_single:
            in_double = not in_double
            buf.append(ch)
            continue
        if ch == "," and not in_single and not in_double:
            items.append(unquote("".join(buf)))
            buf = []
            continue
        buf.append(ch)
    if buf:
        items.append(unquote("".join(buf)))
    return [item for item in items if item != ""]


path = Path(sys.argv[1])
current_list_key = None
for lineno, raw in enumerate(path.read_text().splitlines(), start=1):
    line = strip_comment(raw)
    if not line.strip():
        continue
    if re.match(r"^\s*-\s+", line):
        if current_list_key is None:
            raise SystemExit(f"{path}:{lineno}: list item without a preceding key")
        item = re.sub(r"^\s*-\s*", "", line).strip()
        print(f"LIST\t{current_list_key}\t{unquote(item)}")
        continue

    current_list_key = None
    stripped = line.strip()
    match = re.match(r"^([A-Za-z0-9_]+)\s*:\s*(.*)$", stripped)
    if not match:
        raise SystemExit(f"{path}:{lineno}: unsupported YAML syntax: {raw}")
    key, value = match.groups()
    value = value.strip()
    if value == "":
        current_list_key = key
        continue
    if value.startswith("[") and value.endswith("]"):
        for item in split_inline_list(value):
            print(f"LIST\t{key}\t{item}")
    else:
        print(f"SCALAR\t{key}\t{unquote(value)}")
PY
  )"; then
    die "Failed to parse config file: $CONFIG_FILE"
  fi

  while IFS=$'\t' read -r kind key value; do
    case "$kind:$key" in
      SCALAR:workspace_root) WORKSPACE_ROOT="$value" ;;
      SCALAR:dataset_root) DATASET_ROOT="$value" ;;
      SCALAR:output_root) OUTPUT_BASE="$value"; [[ -n "$value" ]] && OUTPUT_ROOT_EXPLICIT=1 || OUTPUT_ROOT_EXPLICIT=0 ;;
      SCALAR:results_name) RESULTS_NAME="$value" ;;
      SCALAR:params_file) PARAMS_FILE="$value" ;;
      SCALAR:camera) REQUESTED_CAMERA="$value" ;;
      SCALAR:speed) REQUESTED_SPEED="$value" ;;
      SCALAR:trajectory_policy) TRAJECTORY_POLICY="$value" ;;
      LIST:sequences) REQUESTED_SEQUENCES+=("$value") ;;
      SCALAR:seq_range_start) SEQ_RANGE_START="$value" ;;
      SCALAR:seq_range_end) SEQ_RANGE_END="$value" ;;
      SCALAR:node_name) NODE_NAME="$value" ;;
      SCALAR:ov2slam_cores) OV2SLAM_CORES="$value" ;;
      SCALAR:retries) SEQUENCE_RETRIES="$value" ;;
      SCALAR:node_wait_timeout) NODE_WAIT_TIMEOUT="$value" ;;
      SCALAR:bag_timeout) BAG_TIMEOUT="$value" ;;
      SCALAR:auto_exit_timeout) AUTO_EXIT_TIMEOUT="$value" ;;
      SCALAR:sigint_timeout) SIGINT_TIMEOUT="$value" ;;
      SCALAR:startup_settle_sec) OV2SLAM_STARTUP_SETTLE_SEC="$value" ;;
      SCALAR:inter_run_delay_sec) INTER_RUN_DELAY_SEC="$value" ;;
      SCALAR:monitor_sample_sec) MONITOR_SAMPLE_SEC="$value" ;;
      *)
        die "Unsupported config entry: $key"
        ;;
    esac
  done <<<"$config_lines"
}

validate_config_values() {
  case "$REQUESTED_CAMERA" in
    stereo|mono) ;;
    *) die "Config error: camera must be stereo or mono." ;;
  esac

  case "$REQUESTED_SPEED" in
    accurate|average|fast) ;;
    *) die "Config error: speed must be accurate, average, or fast." ;;
  esac

  case "$TRAJECTORY_POLICY" in
    paper|best) ;;
    *) die "Config error: trajectory_policy must be paper or best." ;;
  esac

  if ((${#REQUESTED_SEQUENCES[@]} > 0)) && [[ -n "$SEQ_RANGE_START" ]]; then
    die "Config error: use either sequences or seq_range_start/seq_range_end, not both."
  fi

  if [[ -n "$SEQ_RANGE_START" || -n "$SEQ_RANGE_END" ]]; then
    [[ -n "$SEQ_RANGE_START" && -n "$SEQ_RANGE_END" ]] || die "Config error: seq_range_start and seq_range_end must be provided together."
  fi

  if [[ -n "$RESULTS_NAME" ]]; then
    [[ "$RESULTS_NAME" != */* ]] || die "Config error: results_name must be a single directory name, not a path."
    [[ "$RESULTS_NAME" != "." && "$RESULTS_NAME" != ".." ]] || die "Config error: results_name cannot be '.' or '..'."
  fi

  [[ "$SEQUENCE_RETRIES" =~ ^[0-9]+$ ]] || die "Config error: retries must be a non-negative integer."
  [[ "$NODE_WAIT_TIMEOUT" =~ ^[0-9]+$ ]] || die "Config error: node_wait_timeout must be an integer."
  [[ "$BAG_TIMEOUT" =~ ^[0-9]+$ ]] || die "Config error: bag_timeout must be an integer."
  [[ "$AUTO_EXIT_TIMEOUT" =~ ^[0-9]+$ ]] || die "Config error: auto_exit_timeout must be an integer."
  [[ "$SIGINT_TIMEOUT" =~ ^[0-9]+$ ]] || die "Config error: sigint_timeout must be an integer."
  [[ "$OV2SLAM_STARTUP_SETTLE_SEC" =~ ^[0-9]+$ ]] || die "Config error: startup_settle_sec must be an integer."
  [[ "$INTER_RUN_DELAY_SEC" =~ ^[0-9]+$ ]] || die "Config error: inter_run_delay_sec must be an integer."
  [[ "$MONITOR_SAMPLE_SEC" =~ ^[0-9]+([.][0-9]+)?$ ]] || die "Config error: monitor_sample_sec must be a number."
}

resolve_paths() {
  if [[ -z "$WORKSPACE_ROOT" ]]; then
    WORKSPACE_ROOT="$DEFAULT_WORKSPACE_ROOT"
  fi
  WORKSPACE_ROOT="$(cd -- "$WORKSPACE_ROOT" 2>/dev/null && pwd)" || die "Workspace root missing: $WORKSPACE_ROOT"

  if [[ -z "$DATASET_ROOT" ]]; then
    DATASET_ROOT="$WORKSPACE_ROOT/datasets/euroc"
  fi
  if [[ -e "$DATASET_ROOT" ]]; then
    DATASET_ROOT="$(cd -- "$DATASET_ROOT" 2>/dev/null && pwd)"
  fi

  if [[ -z "$OUTPUT_BASE" ]]; then
    OUTPUT_BASE="$WORKSPACE_ROOT/results/ov2slam_benchmark"
  fi

  if [[ -z "$PARAMS_FILE" ]]; then
    PARAMS_FILE="$WORKSPACE_ROOT/src/ov2slam_ros/parameters_files/$REQUESTED_SPEED/euroc/euroc_${REQUESTED_CAMERA}.yaml"
  fi
  if [[ -e "$PARAMS_FILE" ]]; then
    PARAMS_FILE="$(cd -- "$(dirname -- "$PARAMS_FILE")" && pwd)/$(basename -- "$PARAMS_FILE")"
  fi
}

infer_params_metadata() {
  EFFECTIVE_CAMERA="unknown"
  EFFECTIVE_SPEED="custom"

  if [[ -f "$PARAMS_FILE" ]]; then
    if grep -Eq '^[[:space:]]*mono:[[:space:]]*1[[:space:]]*$' "$PARAMS_FILE" && ! grep -Eq '^[[:space:]]*stereo:[[:space:]]*1[[:space:]]*$' "$PARAMS_FILE"; then
      EFFECTIVE_CAMERA="mono"
    elif grep -Eq '^[[:space:]]*stereo:[[:space:]]*1[[:space:]]*$' "$PARAMS_FILE" && ! grep -Eq '^[[:space:]]*mono:[[:space:]]*1[[:space:]]*$' "$PARAMS_FILE"; then
      EFFECTIVE_CAMERA="stereo"
    fi

    case "$PARAMS_FILE" in
      */parameters_files/accurate/*) EFFECTIVE_SPEED="accurate" ;;
      */parameters_files/average/*) EFFECTIVE_SPEED="average" ;;
      */parameters_files/fast/*) EFFECTIVE_SPEED="fast" ;;
    esac
  fi
}

create_results_root() {
  local run_name="$1"
  local requested="$OUTPUT_BASE/$run_name"

  if ((OUTPUT_ROOT_EXPLICIT)); then
    mkdir -p -- "$requested" || die "Could not create output directory: $requested"
    RESULTS_ROOT="$requested"
    return
  fi

  local candidate
  for candidate in "$requested" "/tmp/ov2slam_benchmark/$run_name"; do
    if mkdir -p -- "$candidate" 2>/dev/null; then
      RESULTS_ROOT="$candidate"
      return
    fi
  done

  die "Could not create a writable output directory under $OUTPUT_BASE or /tmp/ov2slam_benchmark."
}

preflight() {
  local errors=0

  command -v ros2 >/dev/null 2>&1 || { log_error "PREFLIGHT FAIL: 'ros2' not found. Source your ROS2 setup.bash first."; errors=1; }
  command -v evo_ape >/dev/null 2>&1 || { log_error "PREFLIGHT FAIL: 'evo_ape' not found. Install with: pip install evo"; errors=1; }
  command -v python3 >/dev/null 2>&1 || { log_error "PREFLIGHT FAIL: 'python3' not found. It is required for the 2D PNG plot."; errors=1; }
  command -v setsid >/dev/null 2>&1 || { log_error "PREFLIGHT FAIL: 'setsid' not found. It is required for process-group cleanup."; errors=1; }
  command -v timeout >/dev/null 2>&1 || { log_error "PREFLIGHT FAIL: 'timeout' not found. It is required for bag playback verification."; errors=1; }
  if [[ -n "$OV2SLAM_CORES" ]]; then
    command -v taskset >/dev/null 2>&1 || { log_error "PREFLIGHT FAIL: 'taskset' not found. It is required for --ov2slam-cores."; errors=1; }
    [[ "$OV2SLAM_CORES" =~ ^[0-9]+([,-][0-9]+)*$ ]] || { log_error "PREFLIGHT FAIL: --ov2slam-cores must be a CPU list like 2 or 2,3 or 0-3."; errors=1; }
  fi
  [[ -d "$DATASET_ROOT" ]] || { log_error "PREFLIGHT FAIL: Dataset root missing: $DATASET_ROOT"; errors=1; }
  [[ -f "$PARAMS_FILE" ]] || { log_error "PREFLIGHT FAIL: Params file missing: $PARAMS_FILE"; errors=1; }

  if ! python3 - <<'PY' >/dev/null 2>&1
import matplotlib, numpy
PY
  then
    log_error "PREFLIGHT FAIL: python3 modules 'matplotlib' and 'numpy' are required for PNG plot generation."
    errors=1
  fi

  ((errors == 0)) || exit 1
}

discover_sequences() {
  DISCOVERED_SEQUENCES=()
  local entry
  while IFS= read -r entry; do
    shopt -s nullglob
    local db3_files=("$entry"/*.db3)
    shopt -u nullglob
    if ((${#db3_files[@]} > 0)); then
      DISCOVERED_SEQUENCES+=("$entry")
    fi
  done < <(find "$DATASET_ROOT" -mindepth 1 -maxdepth 1 -type d | sort)
}

select_sequences() {
  SELECTED_SEQUENCES=()
  local -A by_name=()
  local ordered_names=()
  local seq_dir seq_name

  for seq_dir in "${DISCOVERED_SEQUENCES[@]}"; do
    seq_name="$(basename -- "$seq_dir")"
    by_name["$seq_name"]="$seq_dir"
    ordered_names+=("$seq_name")
  done

  if ((${#REQUESTED_SEQUENCES[@]} > 0)); then
    local name
    for name in "${REQUESTED_SEQUENCES[@]}"; do
      [[ -n "${by_name[$name]:-}" ]] || die "Unknown sequence: $name"
      SELECTED_SEQUENCES+=("${by_name[$name]}")
    done
    return
  fi

  if [[ -n "$SEQ_RANGE_START" ]]; then
    [[ -n "${by_name[$SEQ_RANGE_START]:-}" ]] || die "Unknown sequence range start: $SEQ_RANGE_START"
    [[ -n "${by_name[$SEQ_RANGE_END]:-}" ]] || die "Unknown sequence range end: $SEQ_RANGE_END"

    local start_idx=-1 end_idx=-1 i
    for i in "${!ordered_names[@]}"; do
      [[ "${ordered_names[$i]}" == "$SEQ_RANGE_START" ]] && start_idx="$i"
      [[ "${ordered_names[$i]}" == "$SEQ_RANGE_END" ]] && end_idx="$i"
    done
    ((start_idx >= 0 && end_idx >= 0)) || die "Could not resolve sequence range."
    ((start_idx <= end_idx)) || die "Invalid sequence range: $SEQ_RANGE_START appears after $SEQ_RANGE_END."

    for ((i=start_idx; i<=end_idx; i++)); do
      SELECTED_SEQUENCES+=("${by_name[${ordered_names[$i]}]}")
    done
    return
  fi

  SELECTED_SEQUENCES=("${DISCOVERED_SEQUENCES[@]}")
}

resolve_bag_target() {
  local seq_dir="$1"
  shopt -s nullglob
  local db3_files=("$seq_dir"/*.db3)
  shopt -u nullglob

  ((${#db3_files[@]} > 0)) || {
    LAST_ERROR="No .db3 file found in $seq_dir"
    return 1
  }

  if ((${#db3_files[@]} == 1)); then
    BAG_TARGET="${db3_files[0]}"
    if [[ -f "$seq_dir/metadata.yaml" ]]; then
      BAG_DESC="$(basename -- "$BAG_TARGET")  (single .db3; metadata also present)"
    else
      BAG_DESC="$(basename -- "$BAG_TARGET")  (single .db3)"
    fi
    return 0
  fi

  if [[ -f "$seq_dir/metadata.yaml" ]]; then
    BAG_TARGET="$seq_dir"
    BAG_DESC="$(basename -- "$seq_dir")/  (${#db3_files[@]} segments via metadata.yaml)"
    return 0
  fi

  LAST_ERROR=$(
    cat <<EOF
Split bag detected in $seq_dir but metadata.yaml is missing.
  Found ${#db3_files[@]} .db3 files.
  Fix with: ros2 bag reindex $seq_dir
EOF
  )
  return 1
}

validate_and_copy_tum() {
  local src="$1"
  local dest="$2"
  if ! awk '
    BEGIN {
      found = 0
      num = "^[-+]?[0-9]*\\.?[0-9]+([eE][-+]?[0-9]+)?$"
    }
    /^[[:space:]]*$/ { next }
    /^[[:space:]]*#/ { next }
    {
      gsub(/,/, " ")
      n = split($0, a, /[[:space:]]+/)
      if (n != 8) {
        printf("Expected 8 columns on line %d, got %d\n", NR, n) > "/dev/stderr"
        exit 2
      }
      for (i = 1; i <= 8; i++) {
        if (a[i] !~ num) {
          printf("Non-numeric value on line %d: %s\n", NR, a[i]) > "/dev/stderr"
          exit 3
        }
      }
      printf("%s %s %s %s %s %s %s %s\n", a[1], a[2], a[3], a[4], a[5], a[6], a[7], a[8])
      found = 1
    }
    END {
      if (!found) {
        print "No pose rows found" > "/dev/stderr"
        exit 4
      }
    }
  ' "$src" >"$dest"; then
    LAST_ERROR="Ground truth validation failed for $src"
    rm -f -- "$dest"
    return 1
  fi
  return 0
}

prepare_ground_truth() {
  local seq_dir="$1"
  local out_dir="$2"
  local seq_name
  seq_name="$(basename -- "$seq_dir")"

  local source=""
  if [[ -f "$seq_dir/gt.tum" ]]; then
    source="$seq_dir/gt.tum"
  elif [[ -f "$seq_dir/$seq_name.txt" ]]; then
    source="$seq_dir/$seq_name.txt"
  else
    shopt -s nullglob
    local txt_files=("$seq_dir"/*.txt)
    shopt -u nullglob
    local filtered=()
    local path
    for path in "${txt_files[@]}"; do
      [[ "$(basename -- "$path")" == "metadata.txt" ]] && continue
      filtered+=("$path")
    done

    if ((${#filtered[@]} == 0)); then
      LAST_ERROR="No ground truth found in $seq_dir. Expected gt.tum or $seq_name.txt."
      return 1
    fi
    if ((${#filtered[@]} > 1)); then
      LAST_ERROR="Multiple candidate ground-truth files found in $seq_dir."
      return 1
    fi
    source="${filtered[0]}"
  fi

  GT_FILE="$out_dir/gt.tum"
  if ! validate_and_copy_tum "$source" "$GT_FILE"; then
    return 1
  fi

  GT_VERDICT="$(basename -- "$source") is already valid TUM format (timestamp tx ty tz qx qy qz qw); normalized copy written to gt.tum"
  GT_SOURCE_BASENAME="$(basename -- "$source")"
  return 0
}

wait_for_ros2_node() {
  local node_name="$1"
  local timeout_s="$2"
  local expected="${node_name#/}"
  local deadline=$((SECONDS + timeout_s))
  local output line

  while ((SECONDS < deadline)); do
    if output="$(ros2 node list 2>/dev/null)"; then
      while IFS= read -r line; do
        [[ "${line#/}" == "$expected" ]] && return 0
      done <<<"$output"
    fi
    sleep 1
  done
  return 1
}

wait_for_clock_message() {
  local timeout_s="$1"
  timeout "${timeout_s}s" ros2 topic echo --once /clock >/dev/null 2>&1
}

publish_final_clock() {
  ros2 topic pub --once /clock rosgraph_msgs/msg/Clock \
    "{clock: {sec: ${FINAL_CLOCK_SEC}, nanosec: ${FINAL_CLOCK_NANOSEC}}}" \
    >/dev/null 2>&1 || true
}

wait_for_pid_exit() {
  local pid="$1"
  local timeout_s="$2"
  local deadline=$((SECONDS + timeout_s))
  while kill -0 "$pid" 2>/dev/null; do
    ((SECONDS < deadline)) || return 1
    sleep 1
  done
  return 0
}

start_ov2slam_monitor() {
  local launcher_pid="$1"
  local out_dir="$2"
  local info_path="$out_dir/ov2slam_pid_info.json"
  local process_csv="$out_dir/ov2slam_process_cpu.csv"
  local thread_csv="$out_dir/ov2slam_thread_cpu.csv"
  local summary_csv="$out_dir/ov2slam_thread_cpu_summary.csv"

  python3 - "$launcher_pid" "$info_path" "$process_csv" "$thread_csv" "$summary_csv" "$MONITOR_SAMPLE_SEC" <<'PY' &
import csv
import json
import os
import sys
import time
from collections import Counter, defaultdict
from pathlib import Path


launcher_pid = int(sys.argv[1])
info_path = Path(sys.argv[2])
process_csv = Path(sys.argv[3])
thread_csv = Path(sys.argv[4])
summary_csv = Path(sys.argv[5])
sample_sec = float(sys.argv[6])
hz = os.sysconf(os.sysconf_names["SC_CLK_TCK"])


def pid_exists(pid: int) -> bool:
    return Path(f"/proc/{pid}").exists()


def read_comm(path: Path) -> str:
    try:
        return path.read_text().strip()
    except Exception:
        return ""


def read_cmdline(pid: int) -> str:
    try:
        raw = Path(f"/proc/{pid}/cmdline").read_bytes()
        return raw.replace(b"\x00", b" ").decode("utf-8", "replace").strip()
    except Exception:
        return ""


def read_exe(pid: int) -> str:
    try:
        return os.path.basename(os.readlink(f"/proc/{pid}/exe"))
    except Exception:
        return ""


def parse_stat(path: Path):
    raw = path.read_text().strip()
    rparen = raw.rfind(")")
    rest = raw[rparen + 2 :].split()
    utime = int(rest[11])
    stime = int(rest[12])
    processor = int(rest[36])
    ppid = int(rest[1])
    return utime + stime, processor, ppid


def collect_descendants(root_pid: int):
    children = defaultdict(list)
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit():
            continue
        stat_path = entry / "stat"
        try:
            _total, _cpu, ppid = parse_stat(stat_path)
        except Exception:
            continue
        children[ppid].append(int(entry.name))

    out = []
    stack = [root_pid]
    seen = set()
    while stack:
        pid = stack.pop()
        if pid in seen:
            continue
        seen.add(pid)
        for child in children.get(pid, []):
            out.append(child)
            stack.append(child)
    return out


def cmdline_has_ov2slam_node(cmdline: str) -> bool:
    tokens = [tok for tok in cmdline.split() if tok]
    for tok in tokens:
        base = os.path.basename(tok)
        if base == "ov2slam_node":
            return True
    return False


def is_exact_ov2slam_process(pid: int) -> bool:
    comm = read_comm(Path(f"/proc/{pid}/comm"))
    exe = read_exe(pid)
    return comm == "ov2slam_node" or exe == "ov2slam_node"


def is_wrapped_ov2slam_process(pid: int) -> bool:
    if is_exact_ov2slam_process(pid):
        return True
    cmdline = read_cmdline(pid)
    return cmdline_has_ov2slam_node(cmdline)


def resolve_actual_pid(root_pid: int, timeout_s: float = 30.0):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if not pid_exists(root_pid):
            return None, "launcher_gone"
        descendants = collect_descendants(root_pid)

        for pid in descendants:
            if pid_exists(pid) and is_exact_ov2slam_process(pid):
                return pid, "resolved_descendant_exact"

        if pid_exists(root_pid) and is_exact_ov2slam_process(root_pid):
            return root_pid, "resolved_root_exact"

        for pid in descendants:
            if pid_exists(pid) and is_wrapped_ov2slam_process(pid):
                return pid, "resolved_descendant_wrapped"

        if pid_exists(root_pid) and is_wrapped_ov2slam_process(root_pid):
            root_comm = read_comm(Path(f"/proc/{root_pid}/comm"))
            root_exe = read_exe(root_pid)
            if root_comm == "ov2slam_node" or root_exe == "ov2slam_node":
                return root_pid, "resolved_root_wrapped"

        time.sleep(0.2)
    if pid_exists(root_pid):
        return None, "not_found_under_launcher"
    return None, "not_found"


actual_pid, resolution = resolve_actual_pid(launcher_pid)
info = {
    "launcher_pid": launcher_pid,
    "ov2slam_pid": actual_pid,
    "resolution": resolution,
}
if actual_pid is not None and pid_exists(actual_pid):
    info["ov2slam_comm"] = read_comm(Path(f"/proc/{actual_pid}/comm"))
    info["ov2slam_exe"] = read_exe(actual_pid)
    info["ov2slam_cmdline"] = read_cmdline(actual_pid)
info_path.write_text(json.dumps(info, indent=2) + "\n")

if actual_pid is None or not pid_exists(actual_pid):
    process_csv.write_text("wall_time_unix,elapsed_s,launcher_pid,ov2slam_pid,last_cpu,process_cpu_percent\n")
    thread_csv.write_text("wall_time_unix,elapsed_s,launcher_pid,ov2slam_pid,tid,comm,last_cpu,thread_cpu_percent\n")
    summary_csv.write_text("tid,comm,samples,mean_cpu_percent,max_cpu_percent,dominant_core,last_cpu\n")
    sys.exit(0)

process_csv.parent.mkdir(parents=True, exist_ok=True)
with process_csv.open("w", newline="") as proc_f, thread_csv.open("w", newline="") as thr_f:
    proc_writer = csv.writer(proc_f)
    thr_writer = csv.writer(thr_f)
    proc_writer.writerow(["wall_time_unix", "elapsed_s", "launcher_pid", "ov2slam_pid", "last_cpu", "process_cpu_percent"])
    thr_writer.writerow(["wall_time_unix", "elapsed_s", "launcher_pid", "ov2slam_pid", "tid", "comm", "last_cpu", "thread_cpu_percent"])

    start_wall = time.time()
    prev_sample_wall = time.monotonic()
    prev_proc_total, prev_proc_cpu, _ = parse_stat(Path(f"/proc/{actual_pid}/stat"))
    prev_threads = {}
    task_dir = Path(f"/proc/{actual_pid}/task")
    for task in task_dir.iterdir():
        tid = int(task.name)
        total, cpu, _ = parse_stat(task / "stat")
        prev_threads[tid] = (total, cpu)

    summary = {}
    while pid_exists(actual_pid):
        time.sleep(sample_sec)
        now_wall = time.time()
        now_mono = time.monotonic()
        delta_t = now_mono - prev_sample_wall
        if delta_t <= 0:
            continue
        prev_sample_wall = now_mono

        try:
            proc_total, proc_cpu, _ = parse_stat(Path(f"/proc/{actual_pid}/stat"))
        except Exception:
            break

        proc_cpu_pct = 100.0 * (proc_total - prev_proc_total) / (delta_t * hz)
        prev_proc_total = proc_total

        elapsed_s = now_wall - start_wall
        proc_writer.writerow([f"{now_wall:.3f}", f"{elapsed_s:.3f}", launcher_pid, actual_pid, proc_cpu, f"{proc_cpu_pct:.3f}"])
        proc_f.flush()

        current_threads = {}
        if not task_dir.exists():
            break
        for task in task_dir.iterdir():
            try:
                tid = int(task.name)
                total, last_cpu, _ = parse_stat(task / "stat")
                comm = read_comm(task / "comm")
            except Exception:
                continue

            prev_total, _prev_cpu = prev_threads.get(tid, (total, last_cpu))
            thread_cpu_pct = 100.0 * (total - prev_total) / (delta_t * hz)
            thr_writer.writerow([f"{now_wall:.3f}", f"{elapsed_s:.3f}", launcher_pid, actual_pid, tid, comm, last_cpu, f"{thread_cpu_pct:.3f}"])
            current_threads[tid] = (total, last_cpu)

            stats = summary.setdefault(
                tid,
                {
                    "comm": comm,
                    "samples": 0,
                    "sum_cpu": 0.0,
                    "max_cpu": 0.0,
                    "core_counts": Counter(),
                    "last_cpu": last_cpu,
                },
            )
            stats["comm"] = comm
            stats["samples"] += 1
            stats["sum_cpu"] += thread_cpu_pct
            stats["max_cpu"] = max(stats["max_cpu"], thread_cpu_pct)
            stats["core_counts"][last_cpu] += 1
            stats["last_cpu"] = last_cpu

        thr_f.flush()
        prev_threads = current_threads

with summary_csv.open("w", newline="") as summary_f:
    writer = csv.writer(summary_f)
    writer.writerow(["tid", "comm", "samples", "mean_cpu_percent", "max_cpu_percent", "dominant_core", "last_cpu"])
    for tid, stats in sorted(summary.items(), key=lambda item: (-item[1]["sum_cpu"], item[0])):
        dominant_core = ""
        if stats["core_counts"]:
            dominant_core = stats["core_counts"].most_common(1)[0][0]
        mean_cpu = stats["sum_cpu"] / stats["samples"] if stats["samples"] else 0.0
        writer.writerow([tid, stats["comm"], stats["samples"], f"{mean_cpu:.3f}", f"{stats['max_cpu']:.3f}", dominant_core, stats["last_cpu"]])
PY
  CURRENT_MONITOR_PID=$!
  MONITOR_PID="$CURRENT_MONITOR_PID"
}

wait_for_monitor_exit() {
  local monitor_pid="$1"
  [[ -n "$monitor_pid" ]] || return 0
  if kill -0 "$monitor_pid" 2>/dev/null; then
    if ! wait_for_pid_exit "$monitor_pid" 5; then
      kill -TERM "$monitor_pid" 2>/dev/null || true
      wait_for_pid_exit "$monitor_pid" 2 || true
      kill -KILL "$monitor_pid" 2>/dev/null || true
    fi
  fi
  wait "$monitor_pid" 2>/dev/null || true
  CURRENT_MONITOR_PID=""
}

inter_run_cleanup_pause() {
  local label="$1"
  cleanup_active_processes
  CURRENT_BAG_PID=""
  CURRENT_OV_PID=""
  CURRENT_MONITOR_PID=""
  if ((INTER_RUN_DELAY_SEC > 0)); then
    log_info "  Inter-run cleanup complete for $label. Waiting ${INTER_RUN_DELAY_SEC}s before the next start ..."
    sleep "$INTER_RUN_DELAY_SEC"
  fi
}

terminate_process_group() {
  local pid="$1"
  local label="$2"
  local sigint_timeout="$3"
  local allow_missing="$4"

  if ! kill -0 "$pid" 2>/dev/null; then
    return 0
  fi

  if [[ "$allow_missing" == "1" ]]; then
    log_warn "  Sending SIGINT to $label; final outputs may be incomplete."
  else
    log_info "  Sending SIGINT to $label."
  fi

  kill -INT -- "-$pid" 2>/dev/null || true
  if wait_for_pid_exit "$pid" "$sigint_timeout"; then
    wait "$pid" 2>/dev/null || true
    return 0
  fi

  log_warn "  $label did not exit after SIGINT within ${sigint_timeout}s; sending SIGKILL."
  kill -KILL -- "-$pid" 2>/dev/null || true
  wait_for_pid_exit "$pid" 10 || true
  wait "$pid" 2>/dev/null || true
}

select_estimate_trajectory() {
  local out_dir="$1"
  local policy="$2"
  local -a candidates=()
  if [[ "$policy" == "paper" ]]; then
    candidates=("${PAPER_TRAJECTORY_CANDIDATES[@]}")
  else
    candidates=("${BEST_TRAJECTORY_CANDIDATES[@]}")
  fi

  local entry name reason path
  for entry in "${candidates[@]}"; do
    name="${entry%%|*}"
    reason="${entry#*|}"
    path="$out_dir/$name"
    if [[ -f "$path" ]]; then
      local size
      size="$(wc -c <"$path" 2>/dev/null || printf '0')"
      if [[ "$size" =~ ^[0-9]+$ ]] && ((size > 64)); then
        EST_FILE="$path"
        EST_TRAJ_NAME="$name"
        EST_TRAJ_REASON="$reason"
        return 0
      fi
    fi
  done

  local found_unsupported=()
  for name in "${UNSUPPORTED_EVO_TRAJECTORIES[@]}"; do
    [[ -f "$out_dir/$name" ]] && found_unsupported+=("$name")
  done
  if ((${#found_unsupported[@]} > 0)); then
    LAST_ERROR="Only unsupported loop-closure full trajectories were found: ${found_unsupported[*]}. These files use synthetic frame indices in column 1, not timestamps, so they are not valid inputs for evo_ape tum against EuRoC GT."
    return 1
  fi

  local present_txt=()
  shopt -s nullglob
  present_txt=("$out_dir"/*.txt)
  shopt -u nullglob
  LAST_ERROR="No supported OV2SLAM trajectory found in $out_dir."
  return 1
}

generate_eval_plots() {
  local gt_file="$1"
  local est_file="$2"
  local out_dir="$3"
  local seq_name="$4"
  local ape_plot_path="$out_dir/${seq_name}_ape_xy.png"
  local rpe_plot_path="$out_dir/${seq_name}_rpe_trans.png"

  if ! python3 - "$gt_file" "$est_file" "$ape_plot_path" "$rpe_plot_path" "$seq_name" <<'PY'
import math
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def load_tum_xyz(path: Path):
    rows = []
    for raw_line in path.read_text().splitlines():
        line = raw_line.strip()
        if not line or line.startswith("#"):
            continue
        fields = line.replace(",", " ").split()
        if len(fields) < 8:
            continue
        rows.append((float(fields[0]), float(fields[1]), float(fields[2]), float(fields[3])))
    if not rows:
        raise ValueError(f"No TUM poses found in {path}")
    return rows


def associate_xyz(gt_rows, est_rows, max_diff=0.1):
    gt_points = []
    est_points = []
    assoc_times = []
    j = 0
    for gt_ts, gx, gy, gz in gt_rows:
        while j < len(est_rows) and est_rows[j][0] < gt_ts - max_diff:
            j += 1
        if j >= len(est_rows):
            break
        best_idx = None
        best_dt = math.inf
        k = j
        while k < len(est_rows) and est_rows[k][0] <= gt_ts + max_diff:
            dt = abs(est_rows[k][0] - gt_ts)
            if dt < best_dt:
                best_dt = dt
                best_idx = k
            k += 1
        if best_idx is None:
            continue
        _, ex, ey, ez = est_rows[best_idx]
        gt_points.append([gx, gy, gz])
        est_points.append([ex, ey, ez])
        assoc_times.append(gt_ts)
        j = best_idx + 1
    if len(gt_points) < 3:
        raise ValueError(f"Only {len(gt_points)} timestamp associations found; need at least 3 for alignment.")
    return (
        np.asarray(assoc_times, dtype=float),
        np.asarray(gt_points, dtype=float),
        np.asarray(est_points, dtype=float),
    )


def umeyama_similarity(src: np.ndarray, dst: np.ndarray):
    if src.shape != dst.shape:
        raise ValueError(f"Alignment shape mismatch: {src.shape} vs {dst.shape}")
    if src.shape[0] < 3:
        raise ValueError("Need at least 3 points for alignment.")

    src_mean = src.mean(axis=0)
    dst_mean = dst.mean(axis=0)
    src_demean = src - src_mean
    dst_demean = dst - dst_mean
    covariance = (dst_demean.T @ src_demean) / src.shape[0]
    u, singular_values, vt = np.linalg.svd(covariance)
    correction = np.eye(src.shape[1])
    if np.linalg.det(u) * np.linalg.det(vt) < 0:
        correction[-1, -1] = -1.0
    rotation = u @ correction @ vt
    src_var = np.mean(np.sum(src_demean * src_demean, axis=1))
    if src_var <= 0:
        raise ValueError("Source variance is zero; cannot align trajectory.")
    scale = float(np.sum(singular_values * np.diag(correction)) / src_var)
    translation = dst_mean - scale * (rotation @ src_mean)
    return scale, rotation, translation


gt_file = Path(sys.argv[1])
est_file = Path(sys.argv[2])
ape_plot_path = Path(sys.argv[3])
rpe_plot_path = Path(sys.argv[4])
seq_name = sys.argv[5]

gt_rows = load_tum_xyz(gt_file)
est_rows = load_tum_xyz(est_file)
assoc_times, gt_assoc, est_assoc = associate_xyz(gt_rows, est_rows, max_diff=0.1)
scale, rotation, translation = umeyama_similarity(est_assoc, gt_assoc)

gt_xyz = np.asarray([[x, y, z] for _, x, y, z in gt_rows], dtype=float)
est_xyz = np.asarray([[x, y, z] for _, x, y, z in est_rows], dtype=float)
est_aligned = (scale * (rotation @ est_xyz.T)).T + translation
est_assoc_aligned = (scale * (rotation @ est_assoc.T)).T + translation

fig, ax = plt.subplots(figsize=(6, 6), dpi=160)
ax.plot(gt_xyz[:, 0], gt_xyz[:, 1], "--", color="#7f7f7f", linewidth=1.5, label="GT")
ax.plot(est_aligned[:, 0], est_aligned[:, 1], color="#57c7b6", linewidth=1.6, label="OV2SLAM")
ax.set_xlabel("x (m)")
ax.set_ylabel("y (m)")
ax.set_title(seq_name)
ax.set_aspect("equal", adjustable="box")
ax.grid(True, alpha=0.35)
ax.legend(loc="best")
fig.tight_layout()
fig.savefig(ape_plot_path, bbox_inches="tight")
plt.close(fig)

if len(assoc_times) >= 2:
    rel_time = assoc_times[1:] - assoc_times[0]
    gt_rel = gt_assoc[1:] - gt_assoc[:-1]
    est_rel = est_assoc_aligned[1:] - est_assoc_aligned[:-1]
    rpe_trans = np.linalg.norm(gt_rel - est_rel, axis=1)

    fig, ax = plt.subplots(figsize=(7, 4), dpi=160)
    ax.plot(rel_time, rpe_trans, color="#e67e22", linewidth=1.3)
    ax.set_xlabel("time since start (s)")
    ax.set_ylabel("translation RPE (m)")
    ax.set_title(f"{seq_name} RPE")
    ax.grid(True, alpha=0.35)
    fig.tight_layout()
    fig.savefig(rpe_plot_path, bbox_inches="tight")
    plt.close(fig)
PY
  then
    LAST_ERROR="APE/RPE PNG plot generation failed for $seq_name"
    return 1
  fi

  APE_PLOT_PATH="$ape_plot_path"
  RPE_PLOT_PATH="$rpe_plot_path"
  return 0
}

generate_cpu_plots() {
  local out_dir="$1"
  local seq_name="$2"
  local process_csv="$out_dir/ov2slam_process_cpu.csv"
  local thread_summary_csv="$out_dir/ov2slam_thread_cpu_summary.csv"
  local info_json="$out_dir/ov2slam_pid_info.json"
  local process_plot="$out_dir/ov2slam_cpu_usage.png"
  local thread_bar_plot="$out_dir/ov2slam_threads_cpu_bar.png"

  [[ -f "$process_csv" ]] || return 1
  [[ -f "$thread_summary_csv" ]] || return 1

  if ! python3 - "$process_csv" "$thread_summary_csv" "$info_json" "$process_plot" "$thread_bar_plot" "$seq_name" <<'PY'
import csv
import json
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


process_csv = Path(sys.argv[1])
thread_summary_csv = Path(sys.argv[2])
info_json = Path(sys.argv[3])
process_plot = Path(sys.argv[4])
thread_bar_plot = Path(sys.argv[5])
seq_name = sys.argv[6]

info = {}
if info_json.exists():
    try:
        info = json.loads(info_json.read_text())
    except Exception:
        info = {}

process_rows = []
with process_csv.open() as f:
    reader = csv.DictReader(f)
    for row in reader:
        try:
            process_rows.append((float(row["elapsed_s"]), float(row["process_cpu_percent"])))
        except Exception:
            continue

if not process_rows:
    raise SystemExit(1)

thread_rows = []
with thread_summary_csv.open() as f:
    reader = csv.DictReader(f)
    for row in reader:
        try:
            thread_rows.append(
                (
                    row["comm"],
                    int(row["tid"]),
                    float(row["mean_cpu_percent"]),
                    float(row["max_cpu_percent"]),
                    row["dominant_core"],
                )
            )
        except Exception:
            continue

thread_rows.sort(key=lambda item: (-item[2], item[1]))

fig, ax = plt.subplots(figsize=(8, 4.5), dpi=160)
times = [row[0] for row in process_rows]
cpu = [row[1] for row in process_rows]
title = seq_name
if info.get("ov2slam_pid") is not None:
    title = f"{seq_name} OV2SLAM PID {info['ov2slam_pid']}"
ax.plot(times, cpu, color="#2c7fb8", linewidth=1.5)
ax.set_xlabel("time since monitor start (s)")
ax.set_ylabel("OV2SLAM CPU usage (%)")
ax.set_ylim(0, 400)
ax.set_title(title)
ax.grid(True, alpha=0.35)
fig.tight_layout()
fig.savefig(process_plot, bbox_inches="tight")
plt.close(fig)

if thread_rows:
    labels = []
    values = []
    colors = []
    palette = plt.get_cmap("tab20")
    for idx, (comm, tid, mean_cpu, _max_cpu, dominant_core) in enumerate(thread_rows):
        core_label = dominant_core if dominant_core != "" else "?"
        labels.append(f"{comm} [{tid}] @ CPU {core_label}")
        values.append(mean_cpu)
        try:
            color_idx = int(core_label) % 20
        except Exception:
            color_idx = idx % 20
        colors.append(palette(color_idx))

    height = max(4.5, 0.35 * len(labels) + 1.5)
    fig, ax = plt.subplots(figsize=(10, height), dpi=160)
    y = list(range(len(labels)))
    ax.barh(y, values, color=colors)
    ax.set_yticks(y)
    ax.set_yticklabels(labels)
    ax.invert_yaxis()
    ax.set_xlabel("mean CPU usage (%)")
    ax.set_xlim(0, 400)
    ax.set_title(f"{seq_name} thread CPU usage")
    ax.grid(True, axis="x", alpha=0.35)
    fig.tight_layout()
    fig.savefig(thread_bar_plot, bbox_inches="tight")
    plt.close(fig)
PY
  then
    LAST_ERROR="OV2SLAM CPU plot generation failed for $seq_name"
    return 1
  fi

  CPU_PLOT_PATH="$process_plot"
  THREAD_BAR_PLOT_PATH="$thread_bar_plot"
  return 0
}

run_evo() {
  local gt_file="$1"
  local est_file="$2"
  local out_dir="$3"
  local seq_name="$4"
  local zip_path="$out_dir/${seq_name}_evo.zip"
  local txt_path="$out_dir/${seq_name}_evo.txt"
  local output rc

  local -a cmd=(
    evo_ape
    tum
    "$gt_file"
    "$est_file"
    --verbose
    --align
    --t_max_diff
    0.1
    --save_results
    "$zip_path"
  )
  local scale_mode="disabled"
  if [[ "$EFFECTIVE_CAMERA" == "mono" ]]; then
    cmd+=(--correct_scale)
    scale_mode="enabled"
  fi

  log_info "  EVO  <- $(basename -- "$est_file") vs gt.tum"
  log_info "  EVO scale correction <- $scale_mode"
  log_debug "  CMD: ${cmd[*]}"

  output="$(MPLBACKEND=Agg "${cmd[@]}" 2>&1)"
  rc=$?
  printf '%s\n' "$output" >"$txt_path"

  if ((rc != 0)); then
    LAST_ERROR="evo_ape failed. See $txt_path."
    return 1
  fi

  EVO_RMSE="$(printf '%s\n' "$output" | awk '/^[[:space:]]*rmse[[:space:]]/ {print $2; exit}')"
  if [[ -z "$EVO_RMSE" ]]; then
    LAST_ERROR="Could not parse RMSE from evo output. See $txt_path."
    return 1
  fi

  if generate_eval_plots "$gt_file" "$est_file" "$out_dir" "$seq_name"; then
    log_info "  APE  <- $(basename -- "$APE_PLOT_PATH")  (2D xy PNG)"
    log_info "  RPE  <- $(basename -- "$RPE_PLOT_PATH")  (translation PNG)"
    printf '\nGenerated 2D XY APE plot: %s\n' "$(basename -- "$APE_PLOT_PATH")" >>"$txt_path"
    printf 'Generated translational RPE plot: %s\n' "$(basename -- "$RPE_PLOT_PATH")" >>"$txt_path"
  else
    log_warn "  APE/RPE plot generation failed; metrics are still valid."
    printf '\nAPE/RPE plot generation failed.\n' >>"$txt_path"
  fi

  return 0
}

append_result() {
  RESULT_SEQS+=("$1")
  RESULT_STATUS+=("$2")
  RESULT_RMSE+=("$3")
  RESULT_TRAJ+=("$4")
  RESULT_REASON+=("$5")
  RESULT_WARNINGS+=("$6")
  RESULT_ERROR+=("$7")
  RESULT_GT_VERDICT+=("$8")
}

reset_current_sequence_result() {
  SEQ_RESULT_STATUS=""
  SEQ_RESULT_RMSE=""
  SEQ_RESULT_TRAJ=""
  SEQ_RESULT_REASON=""
  SEQ_RESULT_WARNINGS=""
  SEQ_RESULT_ERROR=""
  SEQ_RESULT_GT_VERDICT=""
}

append_current_sequence_result() {
  local seq_name="$1"
  append_result \
    "$seq_name" \
    "$SEQ_RESULT_STATUS" \
    "$SEQ_RESULT_RMSE" \
    "$SEQ_RESULT_TRAJ" \
    "$SEQ_RESULT_REASON" \
    "$SEQ_RESULT_WARNINGS" \
    "$SEQ_RESULT_ERROR" \
    "$SEQ_RESULT_GT_VERDICT"
}

reset_sequence_output_dir() {
  local out_dir="$1"
  mkdir -p -- "$out_dir" || return 1
  rm -f -- \
    "$out_dir"/ov2slam.log \
    "$out_dir"/bag_play.log \
    "$out_dir"/gt.tum \
    "$out_dir"/ov2slam_pid_info.json \
    "$out_dir"/ov2slam*.txt \
    "$out_dir"/ov2slam*.csv \
    "$out_dir"/*_evo.txt \
    "$out_dir"/*_evo.zip \
    "$out_dir"/*_ape_xy.png \
    "$out_dir"/*_rpe_trans.png \
    "$out_dir"/ov2slam_cpu_usage.png \
    "$out_dir"/ov2slam_threads_cpu_bar.png \
    "$out_dir"/core.*
}

run_sequence_attempt() {
  local seq_dir="$1"
  local attempt="$2"
  local total_attempts="$3"
  local seq_name
  seq_name="$(basename -- "$seq_dir")"
  local out_dir="$RESULTS_ROOT/$seq_name"
  mkdir -p -- "$out_dir" || { LAST_ERROR="Could not create output dir: $out_dir"; return 1; }
  reset_current_sequence_result

  local ov_log="$out_dir/ov2slam.log"
  local bag_log="$out_dir/bag_play.log"
  local seq_error=""
  local seq_rmse=""
  local seq_traj=""
  local seq_reason=""
  local seq_gt_verdict=""
  local warnings=()

  local ov_pid=""
  local bag_pid=""
  local monitor_pid=""

  log_blank
  log_info "========================================================================"
  log_info "SEQ : $seq_name"
  log_info "OUT : $out_dir"
  log_info "ATTEMPT : ${attempt}/${total_attempts}"
  log_info "========================================================================"

  if ! resolve_bag_target "$seq_dir"; then
    seq_error="$LAST_ERROR"
    goto_sequence_cleanup=1
  else
    log_info "  BAG  <- $BAG_DESC"
  fi

  if [[ -z "$seq_error" ]]; then
    if ! prepare_ground_truth "$seq_dir" "$out_dir"; then
      seq_error="$LAST_ERROR"
    else
      seq_gt_verdict="$GT_VERDICT"
      log_info "  GT   <- $GT_SOURCE_BASENAME  (already valid TUM format)"
    fi
  fi

  if [[ -z "$seq_error" ]]; then
    local -a ov_cmd=(
      ros2
      run
      ov2slam
      ov2slam_node
      "$PARAMS_FILE"
      --ros-args
      -p
      use_sim_time:=true
    )
    if [[ -n "$OV2SLAM_CORES" ]]; then
      ov_cmd=(taskset -c "$OV2SLAM_CORES" "${ov_cmd[@]}")
      log_info "  OV2SLAM core pinning <- $OV2SLAM_CORES"
    fi
    local ov_cmd_str=""
    printf -v ov_cmd_str '%q ' "${ov_cmd[@]}"
    log_info "  Launching OV2SLAM ..."
    log_debug "  CMD: ${ov_cmd[*]}"
    setsid bash -lc "cd $(printf '%q' "$out_dir") && exec ${ov_cmd_str}" >"$ov_log" 2>&1 &
    ov_pid=$!
    CURRENT_OV_PID="$ov_pid"
    log_info "  OV2SLAM PID $ov_pid"
    start_ov2slam_monitor "$ov_pid" "$out_dir"
    monitor_pid="$MONITOR_PID"
    log_info "  Monitor PID $monitor_pid"

    log_info "  Waiting for node $NODE_NAME (timeout ${NODE_WAIT_TIMEOUT}s) ..."
    if ! wait_for_ros2_node "$NODE_NAME" "$NODE_WAIT_TIMEOUT"; then
      seq_error="OV2SLAM node '$NODE_NAME' did not appear in ros2 node list within ${NODE_WAIT_TIMEOUT}s. Check $ov_log."
    elif ! kill -0 "$ov_pid" 2>/dev/null; then
      wait "$ov_pid" 2>/dev/null || true
      seq_error="OV2SLAM exited before bag playback started. Check $ov_log."
    else
      log_info "  Letting OV2SLAM settle for ${OV2SLAM_STARTUP_SETTLE_SEC}s before bag playback ..."
      sleep "$OV2SLAM_STARTUP_SETTLE_SEC"
      if ! kill -0 "$ov_pid" 2>/dev/null; then
        wait "$ov_pid" 2>/dev/null || true
        seq_error="OV2SLAM exited during the ${OV2SLAM_STARTUP_SETTLE_SEC}s startup settle window. Check $ov_log."
      fi
    fi
  fi

  if [[ -z "$seq_error" ]]; then
    local -a bag_cmd=(
      ros2
      bag
      play
      "$BAG_TARGET"
      --clock
      --rate
      "$BAG_PLAY_RATE"
    )
    log_info "  Playing bag at ${BAG_PLAY_RATE}x ..."
    log_debug "  CMD: ${bag_cmd[*]}"
    setsid "${bag_cmd[@]}" >"$bag_log" 2>&1 &
    bag_pid=$!
    CURRENT_BAG_PID="$bag_pid"

    sleep 1
    if ! kill -0 "$bag_pid" 2>/dev/null; then
      wait "$bag_pid" 2>/dev/null || true
      seq_error="ros2 bag play exited before playback was confirmed. Check $bag_log."
    else
      log_info "  Verifying that bag playback is actively publishing /clock (timeout ${CLOCK_START_TIMEOUT}s) ..."
      if ! wait_for_clock_message "$CLOCK_START_TIMEOUT"; then
        kill -KILL -- "-$bag_pid" 2>/dev/null || true
        wait "$bag_pid" 2>/dev/null || true
        CURRENT_BAG_PID=""
        bag_pid=""
        seq_error="ros2 bag play did not publish /clock within ${CLOCK_START_TIMEOUT}s. Playback may be idle or misconfigured. Check $bag_log."
      else
        log_info "  /clock detected; bag playback confirmed."
      fi
    fi

    local bag_rc=0
    if [[ -z "$seq_error" ]]; then
      if wait_for_pid_exit "$bag_pid" "$BAG_TIMEOUT"; then
        wait "$bag_pid"
        bag_rc=$?
      else
        warnings+=("Bag playback timed out after ${BAG_TIMEOUT}s and was killed.")
        kill -KILL -- "-$bag_pid" 2>/dev/null || true
        wait "$bag_pid" 2>/dev/null || true
        bag_rc=124
        publish_final_clock
        if [[ -n "$ov_pid" ]] && kill -0 "$ov_pid" 2>/dev/null; then
          terminate_process_group "$ov_pid" "OV2SLAM" "$SIGINT_TIMEOUT" "1"
        fi
        seq_error="Bag playback timed out after ${BAG_TIMEOUT}s."
      fi
      CURRENT_BAG_PID=""
      bag_pid=""
    fi

    if [[ -z "$seq_error" ]]; then
      if ((bag_rc != 0)); then
        warnings+=("ros2 bag play exited with rc=${bag_rc}; attempting final flush anyway.")
        log_warn "  ros2 bag play exited with rc=${bag_rc}; attempting final flush anyway."
      else
        log_info "  Bag playback finished (rc=${bag_rc})."
      fi

      if ! kill -0 "$ov_pid" 2>/dev/null; then
        wait "$ov_pid" 2>/dev/null || true
        seq_error="OV2SLAM exited before final flush. Check $ov_log."
      fi
    fi
  fi

  if [[ -z "$seq_error" ]]; then
    log_info "  Publishing final /clock at t=${FINAL_CLOCK_SEC}s ..."
    publish_final_clock

    log_info "  Waiting for OV2SLAM to auto-exit and write trajectories (timeout ${AUTO_EXIT_TIMEOUT}s) ..."
    if wait_for_pid_exit "$ov_pid" "$AUTO_EXIT_TIMEOUT"; then
      wait "$ov_pid"
      local ov_rc=$?
      log_info "  OV2SLAM exited naturally (rc=${ov_rc})."
      CURRENT_OV_PID=""
      ov_pid=""
    else
      warnings+=("OV2SLAM did not auto-exit after final /clock; SIGINT fallback used.")
      terminate_process_group "$ov_pid" "OV2SLAM" "$SIGINT_TIMEOUT" "1"
      CURRENT_OV_PID=""
      ov_pid=""
    fi
  fi

  if [[ -z "$seq_error" ]]; then
    if ! select_estimate_trajectory "$out_dir" "$TRAJECTORY_POLICY"; then
      seq_error="$LAST_ERROR"
    else
      seq_traj="$EST_TRAJ_NAME"
      seq_reason="$EST_TRAJ_REASON"
      log_info "  TRAJ <- $seq_traj"
    fi
  fi

  if [[ -z "$seq_error" ]]; then
    if ! run_evo "$GT_FILE" "$EST_FILE" "$out_dir" "$seq_name"; then
      seq_error="$LAST_ERROR"
    else
      seq_rmse="$EVO_RMSE"
      log_info "  Sequence complete. ATE RMSE = $seq_rmse"
    fi
  fi

  if [[ -n "$bag_pid" ]] && kill -0 "$bag_pid" 2>/dev/null; then
    kill -KILL -- "-$bag_pid" 2>/dev/null || true
    wait "$bag_pid" 2>/dev/null || true
  fi
  if [[ -n "$ov_pid" ]] && kill -0 "$ov_pid" 2>/dev/null; then
    terminate_process_group "$ov_pid" "OV2SLAM (cleanup)" 10 "1"
  fi
  wait_for_monitor_exit "$monitor_pid"
  CURRENT_BAG_PID=""
  CURRENT_OV_PID=""
  CURRENT_MONITOR_PID=""

  if generate_cpu_plots "$out_dir" "$seq_name"; then
    log_info "  CPU  <- $(basename -- "$CPU_PLOT_PATH")"
    log_info "  CPU  <- $(basename -- "$THREAD_BAR_PLOT_PATH")"
  else
    warnings+=("OV2SLAM CPU monitor plots were not generated.")
  fi

  local warnings_joined=""
  if ((${#warnings[@]} > 0)); then
    warnings_joined="$(printf '%s || ' "${warnings[@]}")"
    warnings_joined="${warnings_joined% || }"
  fi

  if [[ -n "$seq_error" ]]; then
    log_error "  SEQUENCE FAILED [$seq_name]: $seq_error"
    SEQ_RESULT_STATUS="FAIL"
    SEQ_RESULT_RMSE=""
    SEQ_RESULT_TRAJ="$seq_traj"
    SEQ_RESULT_REASON="$seq_reason"
    SEQ_RESULT_WARNINGS="$warnings_joined"
    SEQ_RESULT_ERROR="$seq_error"
    SEQ_RESULT_GT_VERDICT="$seq_gt_verdict"
    LAST_ERROR="$seq_error"
    return 1
  fi

  SEQ_RESULT_STATUS="OK"
  SEQ_RESULT_RMSE="$seq_rmse"
  SEQ_RESULT_TRAJ="$seq_traj"
  SEQ_RESULT_REASON="$seq_reason"
  SEQ_RESULT_WARNINGS="$warnings_joined"
  SEQ_RESULT_ERROR=""
  SEQ_RESULT_GT_VERDICT="$seq_gt_verdict"
  return 0
}

run_sequence() {
  local seq_dir="$1"
  local seq_name
  seq_name="$(basename -- "$seq_dir")"
  local out_dir="$RESULTS_ROOT/$seq_name"
  local total_attempts=$((SEQUENCE_RETRIES + 1))
  local attempt

  for ((attempt=1; attempt<=total_attempts; attempt++)); do
    reset_sequence_output_dir "$out_dir" || {
      LAST_ERROR="Could not reset output dir: $out_dir"
      reset_current_sequence_result
      SEQ_RESULT_STATUS="FAIL"
      SEQ_RESULT_ERROR="$LAST_ERROR"
      append_current_sequence_result "$seq_name"
      return 1
    }

    if run_sequence_attempt "$seq_dir" "$attempt" "$total_attempts"; then
      append_current_sequence_result "$seq_name"
      return 0
    fi

    if ((attempt < total_attempts)); then
      log_warn "  Attempt ${attempt}/${total_attempts} failed for $seq_name: ${SEQ_RESULT_ERROR}"
      inter_run_cleanup_pause "$seq_name retry"
    fi
  done

  append_current_sequence_result "$seq_name"
  return 1
}

write_run_config() {
  local cfg="$RESULTS_ROOT/run_config.json"
  {
    printf '{\n'
    printf '  "config_file": %s,\n' "$(json_string "$CONFIG_FILE")"
    printf '  "workspace_root": %s,\n' "$(json_string "$WORKSPACE_ROOT")"
    printf '  "dataset_root": %s,\n' "$(json_string "$DATASET_ROOT")"
    printf '  "results_root": %s,\n' "$(json_string "$RESULTS_ROOT")"
    printf '  "results_timezone": %s,\n' "$(json_string "$RESULTS_TIMEZONE")"
    printf '  "results_name": %s,\n' "$(json_string "$(basename -- "$RESULTS_ROOT")")"
    printf '  "params_file": %s,\n' "$(json_string "$PARAMS_FILE")"
    printf '  "requested_camera": %s,\n' "$(json_string "$REQUESTED_CAMERA")"
    printf '  "requested_speed": %s,\n' "$(json_string "$REQUESTED_SPEED")"
    printf '  "trajectory_policy": %s,\n' "$(json_string "$TRAJECTORY_POLICY")"
    printf '  "effective_camera": %s,\n' "$(json_string "$EFFECTIVE_CAMERA")"
    printf '  "effective_speed": %s,\n' "$(json_string "$EFFECTIVE_SPEED")"
    printf '  "node_name": %s,\n' "$(json_string "$NODE_NAME")"
    printf '  "ov2slam_cores": %s,\n' "$(json_string "$OV2SLAM_CORES")"
    printf '  "sequence_retries": %s,\n' "$(json_string "$SEQUENCE_RETRIES")"
    printf '  "bag_rate": %s,\n' "$(json_string "$BAG_PLAY_RATE")"
    printf '  "evo_correct_scale": %s,\n' "$([[ "$EFFECTIVE_CAMERA" == "mono" ]] && printf 'true' || printf 'false')"
    printf '  "selected_sequences": ['
    local i
    for i in "${!SELECTED_SEQUENCES[@]}"; do
      ((i > 0)) && printf ', '
      printf '%s' "$(json_string "$(basename -- "${SELECTED_SEQUENCES[$i]}")")"
    done
    printf '],\n'
    printf '  "gt_verdict": %s,\n' "$(json_string "EuRoC *.txt files are already valid TUM pose files; normalized to gt.tum.")"
    printf '  "trajectory_candidates": [\n'
    local -a candidates=()
    if [[ "$TRAJECTORY_POLICY" == "paper" ]]; then
      candidates=("${PAPER_TRAJECTORY_CANDIDATES[@]}")
    else
      candidates=("${BEST_TRAJECTORY_CANDIDATES[@]}")
    fi
    for i in "${!candidates[@]}"; do
      local name="${candidates[$i]%%|*}"
      local reason="${candidates[$i]#*|}"
      printf '    {"file": %s, "reason": %s}' "$(json_string "$name")" "$(json_string "$reason")"
      ((i + 1 < ${#candidates[@]})) && printf ','
      printf '\n'
    done
    printf '  ],\n'
    printf '  "excluded_from_evo": ['
    for i in "${!UNSUPPORTED_EVO_TRAJECTORIES[@]}"; do
      ((i > 0)) && printf ', '
      printf '%s' "$(json_string "${UNSUPPORTED_EVO_TRAJECTORIES[$i]}")"
    done
    printf ']\n'
    printf '}\n'
  } >"$cfg"
}

write_summary() {
  local summary_txt="$RESULTS_ROOT/summary.txt"
  local summary_json="$RESULTS_ROOT/summary.json"
  local line

  {
    printf 'OV2SLAM EuRoC Benchmark Summary\n'
    printf 'Date        : %s\n' "$(datetime_full)"
    printf 'Timezone    : %s\n' "$RESULTS_TIMEZONE"
    printf 'Config file : %s\n' "$CONFIG_FILE"
    printf 'Dataset root: %s\n' "$DATASET_ROOT"
    printf 'Params file : %s\n' "$PARAMS_FILE"
    printf 'Camera mode : %s\n' "$EFFECTIVE_CAMERA"
    printf 'Speed mode  : %s\n' "$EFFECTIVE_SPEED"
    printf 'Traj policy : %s\n' "$TRAJECTORY_POLICY"
    printf 'OV2SLAM CPUs: %s\n' "${OV2SLAM_CORES:-all}"
    printf 'Retries     : %s\n' "$SEQUENCE_RETRIES"
    printf 'Bag rate    : %sx\n' "$BAG_PLAY_RATE"
    printf 'GT verdict  : EuRoC *.txt files in this repo are already valid TUM pose format; the runner validates and normalizes them into gt.tum.\n'
    printf 'Trajectory  : `paper` prefers ov2slam_kfs_traj.txt; `best` prefers ov2slam_fullba_kfs_traj.txt.\n'
    printf 'Evo scale   : `--correct_scale` is applied only for mono runs.\n'
    printf 'Excluded    : ov2slam_full_traj_wlc*.txt are not used for evo here because column 1 is a frame index, not a timestamp.\n'
    printf '%s\n' '----------------------------------------------------------------------------------------'
    printf '%-18s %-6s %-12s %-28s\n' "Sequence" "Status" "RMSE (m)" "Trajectory"
    printf '%s\n' '----------------------------------------------------------------------------------------'

    local i
    for i in "${!RESULT_SEQS[@]}"; do
      local rmse_display="-"
      [[ -n "${RESULT_RMSE[$i]}" ]] && rmse_display="${RESULT_RMSE[$i]}"
      printf '%-18s %-6s %-12s %-28s\n' \
        "${RESULT_SEQS[$i]}" \
        "${RESULT_STATUS[$i]}" \
        "$rmse_display" \
        "${RESULT_TRAJ[$i]:--}"
      [[ -n "${RESULT_REASON[$i]}" ]] && printf '  trajectory reason: %s\n' "${RESULT_REASON[$i]}"
      [[ -n "${RESULT_WARNINGS[$i]}" ]] && printf '  warning: %s\n' "${RESULT_WARNINGS[$i]}"
      [[ -n "${RESULT_ERROR[$i]}" ]] && printf '  error: %s\n' "${RESULT_ERROR[$i]}"
    done
    printf '%s\n' '----------------------------------------------------------------------------------------'
  } | tee "$summary_txt"

  {
    printf '[\n'
    local i
    for i in "${!RESULT_SEQS[@]}"; do
      printf '  {\n'
      printf '    "seq": %s,\n' "$(json_string "${RESULT_SEQS[$i]}")"
      printf '    "success": %s,\n' "$([[ "${RESULT_STATUS[$i]}" == "OK" ]] && printf 'true' || printf 'false')"
      if [[ -n "${RESULT_RMSE[$i]}" ]]; then
        printf '    "rmse": %s,\n' "${RESULT_RMSE[$i]}"
      else
        printf '    "rmse": null,\n'
      fi
      printf '    "traj_used": %s,\n' "$(json_string "${RESULT_TRAJ[$i]}")"
      printf '    "traj_reason": %s,\n' "$(json_string "${RESULT_REASON[$i]}")"
      printf '    "warnings": %s,\n' "$(json_string "${RESULT_WARNINGS[$i]}")"
      printf '    "error": %s,\n' "$(json_string "${RESULT_ERROR[$i]}")"
      printf '    "gt_verdict": %s\n' "$(json_string "${RESULT_GT_VERDICT[$i]}")"
      printf '  }'
      ((i + 1 < ${#RESULT_SEQS[@]})) && printf ','
      printf '\n'
    done
    printf ']\n'
  } >"$summary_json"
}

main() {
  parse_args "$@"
  load_config_file
  validate_config_values
  resolve_paths
  infer_params_metadata
  preflight

  local run_name
  if [[ -n "$RESULTS_NAME" ]]; then
    run_name="$RESULTS_NAME"
  else
    run_name="$(datetime_compact)"
  fi
  create_results_root "$run_name"
  LOG_FILE="$RESULTS_ROOT/benchmark.log"
  : >"$LOG_FILE"

  log_info "Workspace root : $WORKSPACE_ROOT"
  log_info "Dataset root   : $DATASET_ROOT"
  log_info "Results root   : $RESULTS_ROOT"
  log_info "Results name   : $(basename -- "$RESULTS_ROOT")"
  log_info "Results TZ     : $RESULTS_TIMEZONE"
  log_info "Config file    : $CONFIG_FILE"
  log_info "Params file    : $PARAMS_FILE"
  log_info "Camera mode    : $EFFECTIVE_CAMERA"
  log_info "Speed mode     : $EFFECTIVE_SPEED"
  log_info "Traj policy    : $TRAJECTORY_POLICY"
  log_info "Retries        : $SEQUENCE_RETRIES"

  discover_sequences
  ((${#DISCOVERED_SEQUENCES[@]} > 0)) || die "No sequences found under $DATASET_ROOT"

  select_sequences
  ((${#SELECTED_SEQUENCES[@]} > 0)) || die "Sequence selection produced an empty set."

  cp -- "$PARAMS_FILE" "$RESULTS_ROOT/params_used.yaml" || die "Could not copy params file into results root."
  cp -- "$CONFIG_FILE" "$RESULTS_ROOT/bench_config_used.yaml" || die "Could not copy benchmark config into results root."
  write_run_config

  local names=()
  local seq_dir
  for seq_dir in "${SELECTED_SEQUENCES[@]}"; do
    names+=("$(basename -- "$seq_dir")")
  done
  log_info "Sequences (${#SELECTED_SEQUENCES[@]}): [${names[*]}]"

  local overall_success=0
  for seq_dir in "${SELECTED_SEQUENCES[@]}"; do
    if ((STOP_AFTER_CURRENT)); then
      log_warn "Stopping before starting the next sequence."
      break
    fi

    if ! run_sequence "$seq_dir"; then
      overall_success=1
      log_error "Stopping after the first sequence error."
      break
    fi

    inter_run_cleanup_pause "$(basename -- "$seq_dir")"
  done

  write_summary
  return "$overall_success"
}

main "$@"
