#!/usr/bin/env python3
"""
parse_stack.py — read a nested HIL stack config (config/hil/stack/<name>.yaml)
and emit flat, shell-safe KEY=value lines for run_stack_hil.sh to source.

Usage:
    eval "$(python3 scripts/parse_stack.py config/hil/stack/full_ov2slam.yaml --emit-env)"
    python3 scripts/parse_stack.py <cfg> --get slam.cpu      # single value (debug)

Design:
  - The bash side stays in charge; this script only flattens YAML.
  - Every emitted value is shlex-quoted so values with spaces (SLAM_COMMAND) and
    special chars survive `eval` intact.
  - Topic wiring is explicit: SLAM_REMAPS is pre-assembled into a ready-to-use
    `-r a:=b -r c:=d` string from the slam.remap list.
"""
import argparse
import shlex
import sys

try:
    import yaml
except ImportError:
    sys.exit("parse_stack.py: PyYAML not found — `pip install pyyaml`")


def get(d, path, default=None):
    """Dotted-path lookup: get(cfg, 'slam.topics.image_in')."""
    cur = d
    for key in path.split("."):
        if not isinstance(cur, dict) or key not in cur:
            return default
        cur = cur[key]
    return cur


def as_bool(v, default=False):
    if v is None:
        return default
    return str(v).strip().lower() in ("true", "1", "yes", "on")


def emit(pairs):
    """Print KEY=<shlex-quoted value> lines."""
    out = []
    for k, v in pairs:
        if v is None:
            v = ""
        if isinstance(v, bool):
            v = "true" if v else "false"
        out.append(f"{k}={shlex.quote(str(v))}")
    print("\n".join(out))


def build_remaps(remap_list):
    """['a:=b', 'c:=d'] -> '-r a:=b -r c:=d' (empty string if none)."""
    if not remap_list:
        return ""
    return " ".join(f"-r {item}" for item in remap_list)


def emit_env(cfg):
    pairs = [
        # ── run mode (benchmark = record a bag for RMSE; scout = live/demo) ──
        ("MODE", get(cfg, "mode", "scout")),

        # ── network ──
        ("MATLAB_HOST_IP", get(cfg, "network.matlab_host_ip")),
        ("PI_INTERFACE",   get(cfg, "network.interface")),
        ("DDS",            get(cfg, "network.dds", "cyclonedds")),
        ("ROS_DOMAIN_ID",  get(cfg, "network.ros_domain_id", 0)),

        # ── detector (runs in main container; host_cpu is for yolo_producer) ──
        ("DETECTOR",          get(cfg, "detector.type", "yolo")),
        ("DETECTOR_CPU",      get(cfg, "detector.cpu", "")),
        ("DETECTOR_HOST_CPU", get(cfg, "detector.host_cpu", "")),
        ("DETECTOR_IMAGE_IN", get(cfg, "detector.topics.image_in", "/ovcam/image_raw")),
        ("DETECTOR_OUT",      get(cfg, "detector.topics.out", "/yolo/detections")),
        ("YOLO_HEF",          get(cfg, "detector.yolo.hef", "")),
        ("YOLO_CONF",         get(cfg, "detector.yolo.confidence", "")),

        # ── controller (runs in main container) ──
        ("CONTROLLER",            get(cfg, "controller.type", "proportional")),
        ("CONTROLLER_CPU",        get(cfg, "controller.cpu", "")),
        ("CONTROLLER_GAINS_FILE", get(cfg, "controller.gains_file", "")),
        ("CONTROLLER_DET_IN",     get(cfg, "controller.topics.detections_in", "/yolo/detections")),
        ("CONTROLLER_CMD_OUT",    get(cfg, "controller.topics.cmd_out", "/cmd_vel")),
        ("CONTROLLER_USE_SLAM_DEPTH", as_bool(get(cfg, "controller.use_slam_depth"), False)),
        ("CONTROLLER_USE_SLAM_POSE",  as_bool(get(cfg, "controller.use_slam_pose"),  False)),

        # ── slam (separate sidecar container) ──
        ("SLAM_ENABLED",   as_bool(get(cfg, "slam.enabled"), False)),
        ("SLAM_TYPE",      get(cfg, "slam.type", "none")),
        ("SLAM_IMAGE",     get(cfg, "slam.image", "")),
        ("SLAM_CONTAINER", get(cfg, "slam.container_name", "")),
        ("SLAM_CPU",       get(cfg, "slam.cpu", "")),
        ("SLAM_DELAY",     get(cfg, "slam.startup_delay_sec", 0)),
        ("SLAM_RESTART",   get(cfg, "slam.restart", "no")),
        ("SLAM_COMMAND",        get(cfg, "slam.command", "")),
        ("SLAM_SETUP_OVERLAY",  get(cfg, "slam.setup_overlay", "")),
        ("SLAM_LD_PREFIX",      get(cfg, "slam.ld_prefix", "")),
        ("SLAM_IMAGE_IN",  get(cfg, "slam.topics.image_in", "/ovcam/image_raw")),
        ("SLAM_POSE_OUT",  get(cfg, "slam.topics.pose_out", "/slam/pose")),
        ("SLAM_REMAPS",    build_remaps(get(cfg, "slam.remap", []))),

        # ── init_gate (SLAM-warmup gate, opt-in, ORB-SLAM2 only) ──
        ("INIT_GATE_ENABLED", as_bool(get(cfg, "init_gate.enabled"), False)),
    ]
    emit(pairs)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("config")
    ap.add_argument("--emit-env", action="store_true",
                    help="print flat KEY=value lines for bash to source")
    ap.add_argument("--get", metavar="DOTTED.PATH",
                    help="print a single value (debug)")
    args = ap.parse_args()

    try:
        with open(args.config) as f:
            cfg = yaml.safe_load(f) or {}
    except FileNotFoundError:
        sys.exit(f"parse_stack.py: config not found: {args.config}")
    except yaml.YAMLError as e:
        sys.exit(f"parse_stack.py: YAML error in {args.config}: {e}")

    if args.get:
        val = get(cfg, args.get)
        print("" if val is None else val)
    else:  # default to emit-env
        emit_env(cfg)


if __name__ == "__main__":
    main()
