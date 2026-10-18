"""kinds/slam.py — the SLAM benchmark as a plug-in (plan: benchmarks/SLAM_RUNNER_MIGRATION_PLAN.html).

Ported from scripts/hil_matrix/run_matrix.py (start_stack, run_trial, preflight). The shared steps
(Wi-Fi, start_hold, start check, cool-down) are in trial.py; the Pi starts MATLAB's probe, it never
moves the drone itself.

  one session = one camera mode (and, for stereo, one baseline) = one Simulink model.
  conditions  = the arms (config/hil/slam_arms/<arm>.yaml), round robin.
  per run     camera publisher on -> start_hold -> held -> start check -> boot the stack on the Pi host
              (benchmarks/slam_hil_bench.sh, RECORD_GATE) -> resume -> frames flow -> SLAM consumes frames
              -> open gate -> recorder subscribed -> flight (probe_init, tracking_state == 2, probe_mission,
              recorded arrival, >= min_poses non-zero /slam/pose) -> stop -> score (slam_score_run.py).
  outcome     VALID | BACKEND_FAILED (did not initialise, too few poses) | RIG_FAILED (the harness).
"""
import csv
import json
import math
import re
import shutil
import subprocess
import time
from pathlib import Path

import yaml
from rich.markup import escape

import bench_geometry as geo

from ..config import FieldSpec
from ..container import BenchProcess
from ..factory import register
from ..net import WifiSampler
from ..overhead import OverheadMeter
from ..supervisor import SupervisorError
from ..trial import BackendFailed, Outcome, RigFailed, TrialResult
from ..ui import spark
from ..util import SCORER_PY, WS_ROOT, run
from .base import RunKind
from .slam_probe import ProbeClient

CAMERA_MODES = ("mono", "stereo", "rgbd")
BASELINES = (0.11, 0.36, 0.42, 0.54, 0.61)
ARMS_DIR = WS_ROOT / "config" / "hil" / "slam_arms"      # for messages; the code reads <ws_root>/config/hil/slam_arms
STACK_DIR = WS_ROOT / "config" / "hil" / "stack"
BENCH_SCRIPT = "slam_hil_bench.sh"
FRAMES_WAIT_S = 200.0       # camera bridge frame counter must climb within this
RIGHT_FRAME_TEXT = "[sim_camera_bridge_right]: sim_camera_bridge: frame"
SLAM_READY_WAIT_S = 180.0   # the SLAM sidecar must consume frames within this
HIL_LAUNCH_LOG = Path("/tmp/hil_launch.log")        # written by run_stack_hil.sh (ros2 launch output)
RMW = {"fastrtps": "rmw_fastrtps_cpp", "cyclonedds": "rmw_cyclonedds_cpp"}
PROBE_COPIES = ("matlab/slam_traj_probe_unified.m", "matlab/slam_traj_probe_rgbd_unified.m")


# ── arm files (CONTRACT.md section 4) ──
def arm_names(ws_root=WS_ROOT):
    """Names of all arm files under <ws_root>/config/hil/slam_arms, sorted."""
    return sorted(p.stem for p in (Path(ws_root) / "config" / "hil" / "slam_arms").glob("*.yaml"))


def load_arm(name, ws_root=WS_ROOT):
    """The arm file as a dict. camera_modes = the keys of `stack`. KeyError when the file is missing."""
    path = Path(ws_root) / "config" / "hil" / "slam_arms" / f"{name}.yaml"
    if not path.is_file():
        raise KeyError(f"arm {name!r}: {path} not found (known: {', '.join(arm_names(ws_root)) or 'none'})")
    arm = yaml.safe_load(path.read_text()) or {}
    arm.setdefault("name", name)
    arm["stack"] = dict(arm.get("stack") or {})
    arm["camera_modes"] = list(arm["stack"])
    arm.setdefault("ready_signal", "timing_csv")
    arm.setdefault("init_patience_s", 9)
    return arm


def check_stack_config(stack_name, dds, matlab_host):
    """Problems of one unified stack config (the gates of preflight() in run_matrix.py, plus DDS, IP, pinning)."""
    path = STACK_DIR / f"{stack_name}.yaml"
    if not path.is_file():
        return [f"{path} not found"]
    cfg = yaml.safe_load(path.read_text()) or {}
    out = []
    # init_gate.enabled=true overrides --hold-fsm and launches the controller, which fights the probe.
    if (cfg.get("init_gate") or {}).get("enabled") is not False:
        out.append(f"{stack_name}: init_gate.enabled must be false")
    # The startup delay must be the same for every arm, or the comparison is unfair.
    if (cfg.get("slam") or {}).get("startup_delay_sec") != 2:
        out.append(f"{stack_name}: slam.startup_delay_sec must be 2")
    net = cfg.get("network") or {}
    if net.get("dds") != dds:
        out.append(f"{stack_name}: network.dds is {net.get('dds')!r}, the session uses {dds!r}")
    if net.get("matlab_host_ip") != matlab_host:
        out.append(f"{stack_name}: network.matlab_host_ip is {net.get('matlab_host_ip')!r}, the session uses {matlab_host!r}")
    if (cfg.get("slam") or {}).get("cpu") not in (None, ""):
        out.append(f"{stack_name}: slam.cpu is pinned ({cfg['slam']['cpu']!r}); the session uses no pinning")
    return out


def missing_build_files(stack_name, ws_root=WS_ROOT):
    """Files under /workspace that the SLAM sidecar of one stack config needs, but this worktree lacks.

    The SLAM builds (e.g. src/orbslam2/install_stereo, src/orbslam3/install) are not tracked by git,
    so a new worktree does not have them. Checked here: slam.setup_overlay, every /workspace/... part
    of slam.ld_prefix, and every /workspace/... word of slam.command.
    """
    path = STACK_DIR / f"{stack_name}.yaml"
    if not path.is_file():
        return []
    slam = (yaml.safe_load(path.read_text()) or {}).get("slam") or {}
    want = []
    if slam.get("setup_overlay"):
        want.append(str(slam["setup_overlay"]))
    want += [d for d in str(slam.get("ld_prefix") or "").split(":") if d.startswith("/workspace/")]
    want += re.findall(r"/workspace/[^\s'\";]+", str(slam.get("command") or ""))
    out = []
    for w in dict.fromkeys(want):                      # keep order, drop repeats
        host = Path(ws_root) / w[len("/workspace/"):]
        if not host.exists():
            out.append(w)
    return out


# Lines in slam_sidecar.log that mean the SLAM process can never start: stop at once, do not wait.
SIDECAR_FATAL = (re.compile(r"Package '[^']+' not found"), re.compile(r"No executable found"),
                 re.compile(r"error while loading shared libraries"))


def sidecar_fatal_line(log_path):
    """The first fatal line of the SLAM sidecar log, or None."""
    try:
        text = Path(log_path).read_text(errors="replace")
    except OSError:
        return None
    for line in text.splitlines():
        if any(rx.search(line) for rx in SIDECAR_FATAL):
            return line.strip()
    return None


def parse_tracking_state(text):
    """Output of `ros2 topic echo /slam/tracking_state --once --field data` -> int, or None for 'no answer'.

    The echo prints the value, then a YAML '---' line. The first line that is only an integer is the state;
    digits inside a warning do not count. (The old code stripped non-digits and read '2---'.)"""
    for line in (text or "").splitlines():
        m = re.fullmatch(r"\s*(-?\d+)\s*", line)
        if m:
            return int(m.group(1))
    return None


# ── command-line casts ──
def to_ramp(v):
    """None / '' / 'none' -> None (old wide view); 0|1|'0'|'1'|'true'|'false' -> 0 or 1."""
    if v is None or str(v).strip().lower() in ("", "none", "null"):
        return None
    t = str(v).strip().lower()
    if t in ("1", "true", "yes", "on"):
        return 1
    if t in ("0", "false", "no", "off"):
        return 0
    raise ValueError(f"yaw_ramp must be 0, 1 or none, got {v!r}")


def to_arms(v):
    """'a,b' or ['a','b'] or None -> list of arm names ([] = every arm that supports the camera mode)."""
    if v is None:
        return []
    if isinstance(v, str):
        return [x.strip() for x in v.split(",") if x.strip()]
    return [str(x) for x in v]


def _count_lines(path):
    try:
        with open(path, "rb") as f:
            return sum(1 for _ in f)
    except OSError:
        return 0


def _count_text(path, text):
    try:
        return Path(path).read_text(errors="replace").count(text)
    except OSError:
        return 0


@register("slam")
class SlamKind(RunKind):
    columns = ("run", "arm", "outcome", "init", "poses", "ATE RMSE", "path", "scale", "rx")
    summary_script = "benchmarks/slam_session_summary.py"
    # The bench is a host process: the runner's kill names are run on the host, where a name such as
    # "ros2 bag record" would match docker client command lines. stop() cleans the container itself.
    bench_kill_names = ()
    # optional attributes the core reads (see trial.py, session.py)
    n_topics = 5          # a recorder of an SLAM run subscribed to /sim/drone_pose, /sim/heartbeat, /sim/target_pose,
    #                       /slam/pose and /tf in the one proven run; /slam/tracking_state comes on top (6 when all exist)
    min_topics = 4        # fewer than this = the recorder is broken (the three MATLAB topics and /slam/pose)
    cond_key = "arm"
    step_labels = {"setup": "camera publisher", "boot": "boot stack",
                   "go": ("resume", "frames", "SLAM ready", "recorder"),
                   "fly": ("probe init", "tracking", "probe mission", "arrival", "poses"),
                   "score": "score"}

    def __init__(self, settings, services):
        super().__init__(settings, services)
        c = self.cfg
        self.mode = c["camera_mode"]
        self.baseline = c["stereo_baseline_m"] if self.mode == "stereo" else None
        self.ws = Path(services.ws_root)
        self.arms = {name: load_arm(name, self.ws) for name in self._arm_list()}
        label = self.mode + (f" {self.baseline:g} m" if self.baseline is not None else "")
        self.title = f"slam {label}"
        self.probe = ProbeClient(services.supervisor, c["vx"], c["cycles"], c["wavelength_m"], c["yaw_amp_deg"],
                                 c["camera_yaw_offset_deg"], 1 if c["init_axis"] == "path" else 0, c["yaw_ramp"])

    # ── settings ──
    @classmethod
    def fields(cls):
        return [
            FieldSpec("camera_mode", "slam", None, options=CAMERA_MODES, required=True,
                      help="one per session; must match the Simulink model loaded in MATLAB"),
            FieldSpec("stereo_baseline_m", "slam", None, cast=float, cli="--baseline",
                      help="stereo only: 0.11 | 0.36 | 0.42 | 0.54 | 0.61"),
            FieldSpec("arms", "slam", [], cast=to_arms, help="comma list of arm names (default: every arm that supports the mode)"),
            FieldSpec("vx", "slam", 2.0, cast=float, help="probe forward speed, m/s (SIX_VX_OVERRIDE)"),
            FieldSpec("cycles", "slam", 2, cast=float, help="whole cycles of the cruise (SIX_CYCLES_OVERRIDE)"),
            FieldSpec("wavelength_m", "slam", 8.0, cast=float, help="metres per cycle (SIX_LAMBDA_OVERRIDE)"),
            FieldSpec("yaw_amp_deg", "slam", 0.0, cast=float, help="yaw sweep amplitude (YAW_AMP_OVERRIDE)"),
            FieldSpec("camera_yaw_offset_deg", "slam", 0.0, cast=float,
                      help="wider view: camera heading = bearing to the sign + this, deg (0 = face the sign)"),
            FieldSpec("yaw_ramp", "slam", None, cast=to_ramp,
                      help="end-facing: 1 = camera ramps from the offset to 0 over the cruise; 0 = no ramp (yaw_amp_deg is the sweep); none = old wide view"),
            FieldSpec("init_axis", "slam", "lateral", options=("lateral", "path"),
                      help="init leg: lateral = -y (old), path = +x (sideways for a turned camera)"),
            FieldSpec("left_image", "slam", "rgb", options=("rgb", "mono"), help="image type MATLAB sends, left eye"),
            FieldSpec("right_image", "slam", "rgb", options=("rgb", "mono"), help="image type MATLAB sends, right eye"),
            FieldSpec("min_poses", "slam", 20, cast=int, help="fewer non-zero /slam/pose = no map"),
            FieldSpec("probe_timeout_s", "slam", 400.0, cast=float, help="longest wait for one probe phase, s"),
            FieldSpec("end_pad", "slam", 3, cast=int, help="scoring window padding, samples"),
        ]

    @classmethod
    def validate(cls, settings):
        c, out = settings.kind_cfg, []
        mode = c.get("camera_mode")
        if mode is None:
            out.append("slam.camera_mode is required: mono, stereo or rgbd")
            return out
        if mode not in CAMERA_MODES:
            out.append(f"slam.camera_mode {mode!r} is not one of {', '.join(CAMERA_MODES)}")
            return out
        b = c.get("stereo_baseline_m")
        if mode == "stereo":
            if b is None:
                out.append("slam.stereo_baseline_m is required for stereo: " + ", ".join(f"{x:g}" for x in BASELINES))
            elif not any(abs(b - x) < 1e-9 for x in BASELINES):
                out.append(f"slam.stereo_baseline_m {b:g} is not one of " + ", ".join(f"{x:g}" for x in BASELINES))
        elif b is not None:
            out.append(f"slam.stereo_baseline_m {b:g} is given but camera_mode is {mode}: a baseline is only for stereo")
        cyc = c.get("cycles")
        if cyc is None or cyc < 1 or abs(cyc - round(cyc)) > 1e-9:
            out.append(f"slam.cycles {cyc!r} must be a whole number of 1 or more")
        for key in ("vx", "wavelength_m", "probe_timeout_s"):
            if c.get(key) is None or c[key] <= 0:
                out.append(f"slam.{key} must be more than 0")
        if c.get("min_poses") is None or c["min_poses"] < 1:
            out.append("slam.min_poses must be 1 or more")
        if c.get("end_pad") is None or c["end_pad"] < 0:
            out.append("slam.end_pad must be 0 or more")
        names = to_arms(c.get("arms"))
        if not names:
            names = [n for n in arm_names() if mode in load_arm(n)["camera_modes"]]
            if not names:
                out.append(f"no arm file in {ARMS_DIR} supports camera_mode {mode}")
        for n in names:
            try:
                arm = load_arm(n)
            except KeyError as e:
                out.append(str(e.args[0]))
                continue
            if mode not in arm["camera_modes"]:
                out.append(f"arm {n} does not support camera_mode {mode} (it supports: {', '.join(arm['camera_modes'])})")
                continue
            if not (STACK_DIR / f"{arm['stack'][mode]}.yaml").is_file():
                out.append(f"arm {n}: stack config {arm['stack'][mode]}.yaml not found in {STACK_DIR}")
        return out

    def _arm_list(self):
        names = to_arms(self.cfg["arms"])
        if names:
            return names
        return [n for n in arm_names(self.ws) if self.mode in load_arm(n, self.ws)["camera_modes"]]

    # ── schedule ──
    def conditions(self):
        return list(self.arms)

    def condition_label(self, cond):
        return str(cond)

    def run_title(self, cond):
        return f"{cond} · {self.mode}"

    def model_key(self):
        return (self.mode, self.baseline)

    def snapshot_files(self):
        out = ["config/hil/slam_bench.yaml", "benchmarks/slam_hil_bench.sh"]
        for name, arm in self.arms.items():
            out += [f"config/hil/slam_arms/{name}.yaml", f"config/hil/stack/{arm['stack'][self.mode]}.yaml"]
        out += list(PROBE_COPIES)
        return out

    # ── session ──
    def preflight(self, session):
        """Arm gates, the probe copies, MATLAB's waiting loop, and the image types MATLAB sends."""
        s, c = self.settings, self.cfg
        for name, arm in self.arms.items():
            stack = arm["stack"][self.mode]
            bad = check_stack_config(stack, s.dds, s.host)
            if bad:
                raise RigFailed("preflight", "; ".join(bad))
            gone = missing_build_files(stack, self.services.ws_root)
            if gone:
                raise RigFailed("preflight", f"arm {name}: the SLAM build is missing in this worktree: {', '.join(gone)}. "
                                "git does not track SLAM builds: build them here, or copy them from the worktree "
                                "that has them (see benchmarks/SLAM_BENCHMARK.md, 'SLAM builds')")
            session.say(f"arm {name}: stack {stack} ok (init_gate false, startup_delay_sec 2, {s.dds}, {s.host}, no pinning, build present)")
        for rel in PROBE_COPIES:
            if not (WS_ROOT / rel).is_file():
                raise RigFailed("preflight", f"{rel} is missing: run scripts/gen_probe.py and copy it to the MATLAB PC")
        sup = self.services.supervisor
        try:
            info = sup.model_info()
        except SupervisorError as e:
            raise RigFailed("preflight", f"model_info: {e}") from e
        for eye in ("left_image", "right_image"):
            if info.get(eye) != c[eye]:
                raise RigFailed("preflight", f"MATLAB sends {eye}={info.get(eye)} but slam.{eye} is {c[eye]}. "
                                f"In MATLAB: hil_session_start_slam('{self.mode}', "
                                f"{'[]' if self.baseline is None else format(self.baseline, 'g')}, '{c['left_image']}', '{c['right_image']}')")
        # A bare probe_init is refused with bad_args when the waiting loop runs, with no_loop when it does not.
        try:
            sup.send("probe_init")
        except SupervisorError as e:
            if "no_loop" in str(e):
                raise RigFailed("preflight", "MATLAB is not in the probe waiting loop. Paste: "
                                f"hil_session_start_slam('{self.mode}')") from e
            if "bad_args" not in str(e):
                raise RigFailed("preflight", f"probe_init test: {e}") from e
        else:
            raise RigFailed("preflight", "MATLAB accepted a probe_init without arguments: old hil_run_supervisor.m?")
        if self.probe.wide:
            # wider view: a supervisor from before 2026-10-18 does not know probe_turn (and refuses 6 numbers)
            try:
                sup.send("probe_turn")
            except SupervisorError as e:
                if "bad_args" not in str(e):
                    raise RigFailed("preflight", f"MATLAB does not know probe_turn ({e}): restart MATLAB so it loads "
                                    f"the new hil_run_supervisor.m, then paste hil_session_start_slam('{self.mode}')") from e
            else:
                raise RigFailed("preflight", "MATLAB accepted a probe_turn without arguments: old hil_run_supervisor.m?")
        session.say(f"preflight: MATLAB model {info.get('model')}, probe waiting loop answers")

    # ── one trial ──
    def matlab_setup(self, trial, cond):
        c = self.cfg
        trial.meta.update({"slam_arm": cond, "camera_mode": self.mode, "stereo_baseline_m": self.baseline,
                           "vx": c["vx"], "cycles": c["cycles"], "wavelength_m": c["wavelength_m"],
                           "yaw_amp_deg": c["yaw_amp_deg"], "camera_yaw_offset_deg": c["camera_yaw_offset_deg"], "yaw_ramp": c["yaw_ramp"],
                           "init_axis": c["init_axis"], "left_image": c["left_image"], "right_image": c["right_image"],
                           "min_poses": c["min_poses"], "end_pad": c["end_pad"],
                           "slam_standoff_m": geo.SLAM_STANDOFF, "controller_standoff_m": round(geo.STANDOFF, 3),
                           "protocol": "start_hold -> start check -> boot -> resume -> ready -> gate -> probe_init -> "
                                       "tracking_state -> probe_mission -> wait-arrival -> min_poses"})
        if self.probe.wide:
            # wider view: no images until the camera has turned (SLAM must never see a pure rotation)
            self.services.supervisor.pub_off()
            trial.step("camera publisher", "done", "off until the turn")
        else:
            self.services.supervisor.pub_on(20)
            trial.step("camera publisher", "done", "20 Hz")

    def parse_line(self, line, bench):
        if "handshake ACKed" in line:
            bench.extra["handshake"] = True

    def boot(self, trial, cond):
        s, ses, c = self.settings, self.services.session, self.cfg
        arm = self.arms[cond]
        trial.gate.arm()
        env = {"MATLAB_HOST_IP": s.host, "DDS": s.dds, "ROS_DOMAIN_ID": str(s.ros_domain_id), "RUN_TAG": s.tag,
               "RECORD_GATE": str(trial.gate.host_path), "CAMERA_MODE": self.mode,
               # the gate opens only after frames (max FRAMES_WAIT_S) and SLAM ready (max SLAM_READY_WAIT_S)
               "RECORD_GATE_TIMEOUT": str(int(FRAMES_WAIT_S + SLAM_READY_WAIT_S + 60))}
        argv = [f"./benchmarks/{BENCH_SCRIPT}", cond, str(trial.k)]
        bench = BenchProcess(argv, ses.folder / f"run{trial.k}_bench_console.log", self.parse_line,
                             in_container=False, container=self.services.container, env=env)
        trial.bench = bench
        t_end = time.monotonic() + s.boot_timeout_s
        while time.monotonic() < t_end and not bench.gate_wait and bench.alive():
            trial.step("boot stack", "run", (bench.lines[-1] if bench.lines else "")[:90])
            if trial.abort_requested():
                break
            time.sleep(0.5)
        if not bench.gate_wait:
            why = bench.error or ("bench exited" if not bench.alive() else f"no gate wait within {s.boot_timeout_s:.0f} s")
            self._stop_bench(bench)
            raise RigFailed("boot stack", why)
        if not bench.run_rel:
            self._stop_bench(bench)
            raise RigFailed("boot stack", "the bench printed no 'Run dir:' line")
        trial.run_dir = self.services.ws_root / bench.run_rel
        trial.meta.update({"run_dir": str(trial.run_dir), "stack_config": arm["stack"][self.mode],
                           "slam_type": arm.get("slam_type"), "ready_signal": arm["ready_signal"],
                           "pinning": "none", "probe": "slam_traj_probe_rgbd_unified" if self.mode == "rgbd" else "slam_traj_probe_unified"})
        trial.step("boot stack", "done", f"{bench.run_rel} · stack up, waiting for the gate")

    def _stop_bench(self, bench):
        # SIGTERM to the bench script: it stops the stack (run_stack_hil.sh stop) and exits.
        bench.stop(timeout=90, kill_names=self.bench_kill_names, signal_cmd=["pkill", "-TERM", "-f", BENCH_SCRIPT])

    def _sidecar_ok(self, trial):
        """A SLAM sidecar that cannot start is a rig fault: say so at once, not after the timeout."""
        line = sidecar_fatal_line(trial.run_dir / "slam_sidecar.log")
        if line:
            raise RigFailed("SLAM ready", f"SLAM sidecar cannot start: {line}")

    def _check_abort(self, trial, step):
        if trial.abort_requested():
            raise RigFailed(step, "aborted by the operator")

    def _bench_alive(self, trial, step):
        if trial.bench is not None and not trial.bench.alive():
            raise RigFailed(step, trial.bench.error or "the bench script exited")

    def go(self, trial, cond):
        """resume (drone still) -> frames flow -> SLAM consumes them -> gate -> recorder -> 2 s settle."""
        ses, sup, arm = self.services.session, self.services.supervisor, self.arms[cond]
        trial.step("resume", "run")
        try:
            sent, ok = sup.resume()
        except SupervisorError as e:
            raise RigFailed("resume", str(e)) from e
        trial.meta.update({"resume_sent_ns": sent, "resume_wall_ns": ok})
        meta_host = ses.folder / f"run{trial.k}_meta.txt"       # on disk now: a crash cannot lose t = 0
        meta_host.write_text("".join(f"{key}={val}\n" for key, val in trial.meta.items()))
        trial.step("resume", "done", "sim runs, drone still")

        if self.probe.wide:
            # wider view: hover and turn the camera with the publisher off, then start the images
            trial.step("resume", "run", f"turning the camera {self.cfg['camera_yaw_offset_deg']:g} deg")
            self.probe.turn()
            st = self.probe.wait_done(60.0, abort=trial.abort_requested, what="probe turn")
            if st.get("invalid"):
                raise RigFailed("resume", "the camera turn was invalid (sim froze)")
            try:
                sup.pub_on(20)
            except SupervisorError as e:
                raise RigFailed("resume", f"pub_on after the turn: {e}") from e
            trial.step("resume", "done", "camera turned, images on")

        # frames: a container that is up is not a pipeline that delivers; the bridge counter must climb
        trial.step("frames", "run")
        t_end, a, b = time.monotonic() + FRAMES_WAIT_S, 0, 0
        while True:
            self._check_abort(trial, "frames")
            self._bench_alive(trial, "frames")
            a = _count_text(HIL_LAUNCH_LOG, "sim_camera_bridge: frame")
            time.sleep(2.0)
            b = _count_text(HIL_LAUNCH_LOG, "sim_camera_bridge: frame")
            trial.step("frames", "run", f"bridge frames {a} -> {b}")
            if b > a and b > 5:
                break
            if time.monotonic() > t_end:
                raise RigFailed("frames", f"sim_camera_bridge frame count never climbed ({FRAMES_WAIT_S:.0f} s)")
        if self.mode == "stereo":
            # the right eye has its own bridge; an empty right eye ruins every stereo run
            ra = _count_text(HIL_LAUNCH_LOG, RIGHT_FRAME_TEXT)
            time.sleep(3.0)
            rb = _count_text(HIL_LAUNCH_LOG, RIGHT_FRAME_TEXT)
            if rb <= ra:
                raise RigFailed("frames", "right-eye bridge frame count does not climb: restart MATLAB "
                                          "(supervisor with the STEREO_ON pub_start), then check Pi Wi-Fi and UDP buffers")
            b = f"{b}, right {ra} -> {rb}"
        trial.step("frames", "done", f"bridge frames {a} -> {b}")

        # SLAM ready: the sidecar must consume frames, or the probe misses the sideways init leg
        trial.step("SLAM ready", "run")
        if arm["ready_signal"] == "handshake_ack":
            # ORB-SLAM3 writes its timing CSV at shutdown, so wait for the handshake ACK instead
            log = trial.run_dir / "slam_sidecar.log"
            t_end = time.monotonic() + SLAM_READY_WAIT_S
            while _count_text(log, "handshake ACKed") == 0:
                self._check_abort(trial, "SLAM ready")
                self._bench_alive(trial, "SLAM ready")
                self._sidecar_ok(trial)
                if time.monotonic() > t_end:
                    raise RigFailed("SLAM ready", f"SLAM backend never ACKed the handshake ({SLAM_READY_WAIT_S:.0f} s)")
                time.sleep(3.0)
            trial.step("SLAM ready", "done", "handshake ACKed")
        else:
            files = [trial.run_dir / "ov2slam_timing_events.csv", trial.run_dir / "timing_events.csv"]
            t_end = time.monotonic() + SLAM_READY_WAIT_S
            while True:
                self._check_abort(trial, "SLAM ready")
                self._bench_alive(trial, "SLAM ready")
                self._sidecar_ok(trial)
                n1 = sum(_count_lines(f) for f in files)
                time.sleep(3.0)
                n2 = sum(_count_lines(f) for f in files)
                trial.step("SLAM ready", "run", f"timing events {n1} -> {n2}")
                if n2 > n1 and n2 > 10:
                    break
                if time.monotonic() > t_end:
                    raise RigFailed("SLAM ready", f"SLAM backend never started consuming frames ({SLAM_READY_WAIT_S:.0f} s)")
            trial.step("SLAM ready", "done", f"consuming frames ({n1} -> {n2} events)")

        # gate and recorder
        trial.step("recorder", "run")
        trial.gate.open()
        sampler = WifiSampler(ses.folder / f"run{trial.k}_wifi.csv")     # the run dir belongs to root (container)
        trial.state["sampler"] = sampler
        sampler.start()
        n = trial.gate.n_topics
        subs = trial.gate.wait_recorder(trial.run_dir, timeout=15.0,
                                        on_poll=lambda cnt: trial.step("recorder", "run", f"subscribed {cnt}/{n}"))
        if subs < self.min_topics:
            raise RigFailed("recorder", f"recorder subscribed to {subs} topics within 15 s (needs {self.min_topics})")
        if subs < n:
            ses.deviation(f"run {trial.k}: recorder subscribed to {subs}/{n} topics")
        trial.step("recorder", "done", f"{subs} topics")
        time.sleep(2.0)                          # let the front end settle before the init leg

    def check_tracking(self, trial, arm, on_try=None):
        """Read /slam/tracking_state from the live topic until it is 2 or the arm's patience is over.

        The bag cannot be read yet (metadata.yaml is written when recording stops). An empty answer means
        'no answer' (the echo closed before a message), not 'not initialised': it is retried inside the patience.
        Returns the last integer read, or None."""
        patience = arm["init_patience_s"]
        patience = patience.get(self.mode, 9) if isinstance(patience, dict) else patience
        s = self.settings
        script = ("source /opt/ros/jazzy/setup.bash 2>/dev/null; "
                  "timeout 10 ros2 topic echo /slam/tracking_state --once --field data 2>/dev/null")
        env = {"RMW_IMPLEMENTATION": RMW[s.dds], "ROS_DOMAIN_ID": str(s.ros_domain_id)}
        t0, state, n = time.monotonic(), None, 0
        while True:
            n += 1
            p = self.services.container.exec_cmd(["bash", "-lc", script], env=env, timeout=25)
            got = parse_tracking_state(p.stdout)
            if got is not None:
                state = got
                if state == 2:
                    return state
            if on_try:
                on_try(n, state)
            if time.monotonic() - t0 >= patience:
                return state
            self._check_abort(trial, "tracking")
            time.sleep(3.0)

    def fly(self, trial, cond):
        s, c, sup = self.settings, self.cfg, self.services.supervisor
        arm, probe = self.arms[cond], self.probe
        ws = self.services.ws_root
        meter = OverheadMeter(lambda: sup.status_calls)
        t_start = time.monotonic()
        fl = trial.flight = {"total": 90.0, "elapsed": 0.0, "dhist": [], "arm": cond, "phase": "probe init",
                             "tracking": "—", "poses": "—"}

        def tick(st=None, el=None):
            fl["elapsed"] = time.monotonic() - t_start
            if st is not None:
                fl["probe"] = st.get("state")
            try:
                ps = sup.status(timeout=2)
                d = math.dist((ps.get("x", 0), ps.get("y", 0), ps.get("z", 0)), geo.SIGN) if "x" in ps else None
                fl.update({"sim": ps.get("sim"), "t": ps.get("t"), "x": ps.get("x"), "y": ps.get("y"), "z": ps.get("z"),
                           "yaw": ps.get("yaw"), "dist": f"{d:.2f}" if d is not None else "?"})
                if d is not None:
                    fl["dhist"] = (fl["dhist"] + [d])[-40:]
            except SupervisorError:
                fl["sim"] = "no status"
            sampler = trial.state.get("sampler")
            fl["rx"] = f"{sampler.rates[-1]:.2f}" if sampler is not None and sampler.rates else "?"
            meter.sample()
            if fl["elapsed"] > fl["total"]:
                fl["total"] = fl["elapsed"] + 30.0

        # 1. init leg (3 m sideways)
        trial.step("probe init", "run")
        probe.init()
        probe.wait_done(c["probe_timeout_s"], on_poll=tick, abort=trial.abort_requested, what="probe init")
        trial.step("probe init", "done", f"{fl['elapsed']:.0f} s")

        # 2. is SLAM tracking now? (not 2 -> the run ends here, as a result; nothing flies on)
        fl["phase"] = "tracking"
        trial.step("tracking", "run")

        def on_try(n, state):
            trial.step("tracking", "run", f"read {n}: tracking_state={state if state is not None else 'no answer'}")
            fl["tracking"] = "no answer" if state is None else str(state)
            tick()
        state = self.check_tracking(trial, arm, on_try)
        trial.meta["tracking_state"] = state
        fl["tracking"] = "—" if state is None else str(state)
        if state != 2:
            try:
                sup.stop()                       # nothing is measured after this: stop the sim now
            except SupervisorError:
                pass
            raise BackendFailed(f"did not initialise during the init cycle (tracking_state={state}) — mission not flown")
        trial.step("tracking", "done", "tracking_state = 2")

        # 3. mission (cruise + approach)
        fl["phase"] = "probe mission"
        trial.step("probe mission", "run")
        probe.mission()
        st = probe.wait_done(c["probe_timeout_s"], on_poll=tick, abort=trial.abort_requested, what="probe mission")
        dur = time.monotonic() - t_start
        trial.meta.update({"probe_sec": round(dur, 1), "dist_to_target_m": st.get("dist_to_target_m"),
                           "maxtime_hit": st.get("maxtime_hit")})
        if st.get("maxtime_hit"):
            raise RigFailed("probe mission", "a leg hit the MAXTIME cap — path shape off-nominal")
        trial.step("probe mission", "done", f"{dur:.0f} s · {st.get('dist_to_target_m')} m from the sign")

        # 4. the recorded /sim/drone_pose must hold at the standoff: otherwise the bag would end mid-flight
        fl["phase"] = "arrival"
        trial.step("arrival", "run")
        max_wait = max(90, int(5.0 * dur) + 150)
        script = "run_stack_hil_rgbd.sh" if self.mode == "rgbd" else "run_stack_hil.sh"
        t_arr = time.monotonic()
        proc = subprocess.Popen(["bash", "-lc", f"cd {ws} && ./{script} wait-arrival {max_wait}"],
                                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL, text=True)
        while proc.poll() is None:
            if time.monotonic() - t_arr > max_wait + 15 or trial.abort_requested():
                proc.kill()
                break
            tick()
            trial.step("arrival", "run", f"waiting for the recorded arrival ({time.monotonic() - t_arr:.0f} / {max_wait} s)")
            time.sleep(1.0)
        out = proc.communicate()[0] or ""
        trial.meta["wait_arrival_s"] = round(time.monotonic() - t_arr, 1)
        if "ARRIVED" not in out:
            raise RigFailed("arrival", f"recorded /sim/drone_pose never held within the standoff within {max_wait} s "
                                       "(the bag would be truncated mid-flight)")
        trial.step("arrival", "done", f"{trial.meta['wait_arrival_s']:.0f} s")

        # 5. the map: close the recorder (a bag is readable only after that), count non-zero /slam/pose
        fl["phase"] = "poses"
        trial.step("poses", "run", "closing the recorder")
        if not self._close_recorder(trial):
            raise RigFailed("poses", "the bag was not closed within 40 s (no metadata.yaml)")
        trial.meta.update(meter.result())
        try:
            n = self.count_poses(trial)
        finally:
            fl["poses"] = str(trial.meta.get("slam_poses", "—"))
        trial.step("poses", "done", f"{n} non-zero /slam/pose")

    def _close_recorder(self, trial, wait_s=40.0):
        """SIGINT to ros2 bag record in the container, then wait for metadata.yaml (the footer is written)."""
        self.services.container.exec_cmd(["bash", "-c", "pkill -INT -f '[r]os2 bag record' || true"], timeout=15)
        meta = Path(trial.run_dir) / "bag" / "metadata.yaml"
        t_end = time.monotonic() + wait_s
        while time.monotonic() < t_end:
            if meta.exists():
                time.sleep(1.0)
                return True
            time.sleep(0.5)
        return False

    def count_poses(self, trial):
        """Non-zero /slam/pose in the closed bag (scripts/hil_matrix/count_poses.py). Returns the number.

        Fewer than min_poses = no map = BackendFailed. The bag must be closed first (_close_recorder)."""
        p = run([SCORER_PY, str(self.services.ws_root / "scripts" / "hil_matrix" / "count_poses.py"), str(trial.run_dir)],
                timeout=180)
        lines = (p.stdout or "").strip().splitlines()
        try:
            n = int(lines[-1])
        except (ValueError, IndexError) as e:
            raise RigFailed("poses", f"count_poses.py gave no number: {(p.stderr or p.stdout)[-200:]}") from e
        trial.meta["slam_poses"] = n
        if n < self.cfg["min_poses"]:
            raise BackendFailed(f"only {n} non-zero poses (need {self.cfg['min_poses']}) — no map recorded")
        return n

    def stop(self, trial):
        facts = {}
        sampler = trial.state.get("sampler")
        if sampler is not None:
            sampler.stop()
            if sampler.error:
                facts["wifi_sampler_error"] = sampler.error
        if trial.bench is not None:
            self._stop_bench(trial.bench)
        try:        # the recorder is closed by now; this only catches a bench that died early
            self.services.container.exec_cmd(["bash", "-c", "pkill -INT -f '[r]os2 bag record' || true"], timeout=15)
        except Exception:
            pass
        trial.gate.close()
        trial.chown_run_dir()
        if trial.run_dir is not None and Path(trial.run_dir).exists():
            wifi_host = self.services.session.folder / f"run{trial.k}_wifi.csv"
            if wifi_host.exists():
                shutil.copy(wifi_host, Path(trial.run_dir) / "wifi.csv")
            if HIL_LAUNCH_LOG.exists():
                try:
                    shutil.copy(HIL_LAUNCH_LOG, Path(trial.run_dir) / "hil_launch.log")
                except OSError:
                    pass
        return facts

    def score(self, trial):
        ws = self.services.ws_root
        sc = run([SCORER_PY, str(ws / "benchmarks" / "slam_score_run.py"), str(trial.run_dir),
                  "--end-pad", str(self.cfg["end_pad"])], timeout=900)
        (trial.run_dir / "score.log").write_text(sc.stdout + sc.stderr)
        res = self.read_results(trial.run_dir)
        if not res or "outcome" not in res:
            raise RigFailed("score", "no run_check.json (the scorer failed, see score.log)")
        outcome = Outcome(res["outcome"])
        return TrialResult(outcome, res["reason"], sequence=[], meta=trial.meta, run_dir=trial.run_dir, metrics=res)

    # ── results ──
    def read_results(self, run_dir):
        run_dir = Path(run_dir)
        out = {}
        chk = run_dir / "run_check.json"
        if chk.exists():
            c = json.loads(chk.read_text())
            hb = c.get("timing", {}).get("/sim/heartbeat", {})
            hz = hb.get("rate_hz")
            out.update({"valid": c.get("valid"), "outcome": c.get("outcome"), "reason": c.get("reason", ""),
                        "hb_gaps": hb.get("gaps_over_0.2s"),
                        "heartbeat_hz": hz if isinstance(hz, (int, float)) and math.isfinite(hz) else None,
                        "rx": c.get("wifi", {}).get("rx_mean_MBps")})
        met = run_dir / "slam_metrics.csv"
        if met.exists():
            with open(met) as f:
                for row in csv.reader(f):
                    if len(row) == 2 and row[0] != "metric":
                        out[row[0]] = row[1]
        return out

    def table_row(self, k, cond, res):
        r, meta = res.metrics or {}, res.meta or {}

        def num(key, fmt="{:.2f}"):
            try:
                return fmt.format(float(r[key]))
            except (KeyError, ValueError, TypeError):
                return "—"
        if res.valid:
            outcome = "[green]valid[/]"
        elif res.outcome == Outcome.BACKEND_FAILED:
            outcome = f"[yellow]backend failed[/] [dim]({escape(res.reason)})[/]"
        else:
            outcome = f"[red]rig failed[/] [dim]({escape(res.reason)})[/]"
        ts = meta.get("tracking_state")
        if ts is None:
            init = "—"
        elif ts == 2:
            init = "[green]✔[/] " + (num("init_time_s", "{:.1f} s") if "init_time_s" in r else "")
        else:
            init = f"[red]✘[/] state={ts}"
        poses = r.get("n_poses") or meta.get("slam_poses") or "—"
        try:
            poses = str(int(float(poses)))
        except (ValueError, TypeError):
            poses = str(poses)
        return [str(k), str(cond), outcome, init.strip(), poses, num("ape_rmse", "{:.3f} m"),
                num("gt_path_m", "{:.1f} m"), num("umeyama_scale", "{:.3f}"), num("rx")]

    def asks_on(self, outcome):
        return outcome == Outcome.RIG_FAILED        # a backend failure is a result, not a reason to ask

    # ── screen ──
    def flight_rows(self, f):
        return [(f"sim [green]{f.get('sim', '?')}[/] t={f.get('t', '?')}   phase [yellow]{f.get('phase', '—')}[/]"
                 f" · probe {f.get('probe', '—')}", f"arm {f.get('arm', '?')}"),
                (f"pose x={f.get('x', '?')} y={f.get('y', '?')} z={f.get('z', '?')} yaw={f.get('yaw', '?')}°",
                 f"Wi-Fi rx {f.get('rx', '?')} MB/s"),
                (f"distance to sign [bold]{f.get('dist', '?')}[/] m  {spark(f.get('dhist', []), geo.SLAM_STANDOFF - 1, None)}",
                 f"tracking_state {f.get('tracking', '—')} · poses {f.get('poses', '—')}")]
