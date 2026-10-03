#!/usr/bin/env python3
# run_matrix_rgbd.py: GENERATED RGB-D copy of run_matrix.py (2026-09-27). Differences:
#   model hil_closed_loop -> hil_closed_loop_baseline_RGBD; ./run_stack_hil.sh -> ./run_stack_hil_rgbd.sh;
#   set_rgbd(1) + hil_ros_init_LT_rgbd + sim_camera_publisher_timer_LT_rgbd + slam_traj_probe_rgbd;
#   a stereo arm is a hard error. Regenerate with the sed in HANDOFF.md A5 if run_matrix.py changes.
"""run_matrix.py -- SLAM benchmark matrix, Windows side. Run this yourself.

    py scripts\\hil_matrix\\run_matrix.py --config default --dry-run
    py scripts\\hil_matrix\\run_matrix.py --config default --doctor
    py scripts\\hil_matrix\\run_matrix.py --config default
    py scripts\\hil_matrix\\run_matrix.py --config default --only orbslam3 --trials 10

THREE MODES
    --dry-run   Walks the ENTIRE run, printing every command it would issue, in
                order, and executes none of them. No MATLAB, no containers, no
                flight, nothing written anywhere except this script's own log.
                Read-only checks DO still run, against the real Pi and the real
                host, so the plan is validated against the actual rig rather
                than an imagined one. Values a dry run cannot know are printed
                as "[dry] assuming: X" -- those assumptions are declared in the
                source, never silently chosen.
    --doctor    Runs every precondition check for real and flies nothing.
    (neither)   Flies the matrix.

CONFIGURATION
    --config <name> reads config/hil/matrix/<name>.yaml, matching the project's
    existing `run_stack_hil.sh --config <name>` convention. Everything in that
    file can be overridden on the command line, so the YAML is the reproducible
    record and the flags are for one-offs. The resolved configuration, after
    merging, is printed at the top of every run and saved with the results.

WHY THIS EXISTS
    So you do not have to take my word for any number. Two properties are
    deliberate and you should hold the script to them:

    1. EVERY command sent to the Pi is printed before it runs, in a form you can
       paste into your own terminal. Same for every MATLAB statement. All of it
       is also written to <outdir>/run.log on the Pi side and run_<stamp>.log
       locally. If a result exists, the exact sequence that produced it is on
       disk.
    2. FAIL FAST. No retries, no fallbacks, no "attempt 2". The moment any step
       does not do what it claims, the run stops at that step and says which one.
       A retry loop hides the difference between "flaky" and "broken", and a
       matrix that quietly retried is a matrix you cannot audit.

    Nothing here computes a metric. Numbers come from aggregate_matrix.py on the
    Pi, from the recorded bags. This script only flies trials and records what it
    did.

RUN ORDER is round robin (A1,B1,C1, A2,B2,C2, ...) so session drift -- thermal,
MATLAB state, Pi load -- spreads evenly across arms instead of landing entirely
on whichever backend runs last.

REQUIREMENTS
    py -c "import matlab.engine"      must succeed
    ssh amaraly@192.168.137.10 true   must succeed without a password
"""

import argparse
import atexit
import json
import re
import shlex
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

import yaml

SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parent.parent
CONFIG_DIR = REPO_ROOT / "config" / "hil" / "matrix"

# Defaults, overridden by config/hil/matrix/<name>.yaml, overridden again by
# command-line flags. Keep this table as the single source of what a run can be
# told to do: anything absent here is not configurable, by design.
DEFAULTS = {
    "trials": 3,
    "vx": 2.0,
    "end_pad": 3,
    "run_tag": "",
    "calib": "camera_calib/hil_sim_ov2slam_accurate.yaml",
    "pi": "amaraly@192.168.137.10",
    "pi_repo": "~/ROS2-PROJECT-SPRING2026",
    "matlab_dir": str(REPO_ROOT / "matlab"),
    "desktop": True,
    "probe_timeout": 400,
    "min_free_mb": 8000,
    "min_poses": 20,
    "pacing_lo": 0.85,
    "pacing_hi": 1.15,
    "respawn_tol_m": 2.0,
    # Image type of the colour stream: "mono" sends mono8 (RGBD_COLOUR_MONO=1),
    # "rgb" sends bgr8 (RGBD_COLOUR_MONO=0), "" leaves RGBD_COLOUR_MONO as set in
    # the environment (switch on demand). SLAM receives gray either way; keep
    # "rgb" whenever the YOLO detector runs.
    "left_image": "",
    # Scene is recorded per run and becomes a grouping column in results.csv.
    # A second environment is a new row here, not a code change.
    "arms": [
        {"name": "orbslam2",     "config": "orbslam2_probe",          "scene": "parking_lot_far"},
        {"name": "ov2_fast",     "config": "ov2slam_oracle_fast",     "scene": "parking_lot_far"},
        {"name": "ov2_accurate", "config": "ov2slam_oracle_accurate", "scene": "parking_lot_far"},
        {"name": "orbslam3",     "config": "orbslam3_probe",          "scene": "parking_lot_far"},
    ],
}

# Set from --dry-run. Every side-effecting call checks it; read-only checks
# ignore it deliberately, so a dry run is validated against the real rig.
DRY = False


# ══════════════════════════════════════════════════════════════════════════
#  Tracing.  Everything the script does goes through here, so the log is
#  complete by construction rather than by remembering to add print()s.
# ══════════════════════════════════════════════════════════════════════════

class Log:
    def __init__(self, path):
        self.fh = open(path, "w", encoding="utf-8", buffering=1)
        self.path = path

    def __call__(self, msg, prefix=""):
        line = f"[{datetime.now():%H:%M:%S}] {prefix}{msg}"
        print(line, flush=True)
        self.fh.write(line + "\n")

    def cmd(self, kind, text):
        """Echo a command in a form the user can paste and reproduce."""
        self(f"$ {text}", prefix=f"{kind} ")


LOG = None  # set in main()


class StepFailed(Exception):
    """The HARNESS did not do what it claims. Stops the whole run.

    Simulink refusing to start, pacing collapse, the drone not respawning, ssh
    dying: none of these say anything about a SLAM backend, and continuing past
    one produces trials that are not comparable to the ones before it.
    """

    def __init__(self, step, detail):
        super().__init__(f"{step}: {detail}")
        self.step = step
        self.detail = detail


class TrialAborted(Exception):
    """The BACKEND did not do its job. Ends this trial only; the matrix goes on.

    A backend that fails to initialise inside the init leg is a RESULT, not a
    harness fault -- it is exactly the reliability number the benchmark exists
    to measure. Aborting the trial (rather than flying a mission with no map,
    which would be scored over a shorter path and flatter that arm) is the
    correct handling; aborting the whole matrix would throw away every other
    arm's data to record one arm's failure.

    Recorded in the manifest with its reason so reliability is computable.
    """

    def __init__(self, reason):
        super().__init__(reason)
        self.reason = reason


class step:
    """Context manager that names a step, times it, and attributes failures.

    The point is that a traceback is never the user-facing error. Whatever
    goes wrong, the last line printed is `FAIL  <step name>: <why>`, so the
    question "where did it break" is answered without reading Python.
    """

    def __init__(self, name):
        self.name = name

    def __enter__(self):
        LOG(f"STEP  {self.name}")
        self.t0 = time.time()
        return self

    def __exit__(self, exc_type, exc, tb):
        dt = time.time() - self.t0
        if exc_type is None:
            LOG(f"OK    {self.name}  ({dt:.1f}s)")
            return False
        if isinstance(exc, TrialAborted):
            LOG(f"ABORT {self.name}: {exc.reason}")
            return False
        if isinstance(exc, StepFailed):
            return False  # already attributed to a step, let it through
        detail = str(exc).replace("\n", " | ")[:400]
        raise StepFailed(self.name, f"{exc_type.__name__}: {detail}") from exc


def fail(step_name, detail):
    raise StepFailed(step_name, detail)


def nap(seconds, why=""):
    """time.sleep that a dry run skips.

    Waits are most of a trial's wall clock, so honouring them in dry-run would
    make the plan take as long as the flight it is standing in for.
    """
    if DRY:
        LOG(f"      [dry] would wait {seconds:.0f}s{(' -- ' + why) if why else ''}")
        return
    time.sleep(seconds)


def load_config(name):
    """Merge DEFAULTS <- config/hil/matrix/<name>.yaml. Unknown keys are fatal.

    Silently ignoring a misspelled key is how a run ends up not doing what the
    file says it does, which is the exact failure this whole script exists to
    prevent.
    """
    cfg = json.loads(json.dumps(DEFAULTS))   # deep copy
    if not name:
        return cfg, None
    path = Path(name)
    if not path.is_file():
        # Accept "default", "default.yaml" and a full path alike. Without the
        # suffix strip, the natural thing to type -- the actual filename --
        # becomes "<name>.yaml.yaml" and fails.
        stem = name[:-5] if name.endswith(".yaml") else (
            name[:-4] if name.endswith(".yml") else name)
        path = CONFIG_DIR / f"{stem}.yaml"
    if not path.is_file():
        avail = sorted(p.stem for p in CONFIG_DIR.glob("*.yaml")) if CONFIG_DIR.is_dir() else []
        sys.exit(f"config {name!r} not found at {path}. "
                 f"Available in {CONFIG_DIR}: {', '.join(avail) or '(none)'}")
    loaded = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    unknown = set(loaded) - set(DEFAULTS)
    if unknown:
        sys.exit(f"{path}: unknown key(s) {sorted(unknown)}. "
                 f"Known keys: {', '.join(sorted(DEFAULTS))}")
    for arm in loaded.get("arms") or []:
        missing = {"name", "config", "scene"} - set(arm)
        if missing:
            sys.exit(f"{path}: arm {arm} is missing {sorted(missing)}")
    cfg.update(loaded)
    return cfg, path


# ══════════════════════════════════════════════════════════════════════════
#  Pi over ssh.  subprocess, not paramiko: the printed line IS the command,
#  so anything this script did to the Pi you can redo by hand.
# ══════════════════════════════════════════════════════════════════════════

class Pi:
    def __init__(self, host, repo, log):
        self.host = host
        self.repo = repo
        self.log = log

    def __call__(self, cmd, timeout=60, check=True, reads=False, dry_value="",
                 log_lines=20):
        """Run `cmd` on the Pi.

        reads=True marks a command as read-only, so a dry run still executes it
        and the printed plan is checked against the real machine. Everything
        else is skipped in dry-run and returns dry_value, which is the value the
        plan ASSUMES -- stated in the source rather than hidden.
        """
        remote = f"cd {self.repo} && {cmd}"
        if DRY and not reads:
            self.log.cmd("pi  ", f"[dry] WOULD RUN: ssh {self.host} {shlex.quote(remote)}")
            if dry_value:
                self.log(f"      [dry] assuming: {dry_value!r}")
            return dry_value
        # -n redirects ssh's stdin from /dev/null. Without it ssh inherits the
        # console's stdin and, called repeatedly from an interactive terminal,
        # can swallow input and block -- which is why the PowerShell version ran
        # fine unattended but hung when launched by hand from a VS Code terminal.
        argv = ["ssh", "-n", "-o", "ConnectTimeout=8", "-o", "BatchMode=yes",
                self.host, remote]
        self.log.cmd("pi  ", f"ssh {self.host} {shlex.quote(remote)}")
        try:
            p = subprocess.run(argv, capture_output=True, text=True,
                               timeout=timeout)
        except subprocess.TimeoutExpired:
            fail("pi-ssh", f"timed out after {timeout}s: {cmd}")
        out = (p.stdout or "").strip()
        err = (p.stderr or "").strip()
        if err:
            for line in err.splitlines():
                self.log(f"      pi-stderr: {line}")
        if check and p.returncode != 0:
            fail("pi-ssh", f"exit {p.returncode}: {cmd}\n{err or out}")
        if out:
            # Truncation is for chatty probe commands, NOT for results. Capping
            # at 20 lines silently cut the aggregation report after its first
            # arm block, so a healthy 3-arm run looked like it had produced one
            # arm -- the data was complete on the Pi the whole time. Anything
            # dropped is now stated rather than vanishing, and callers that
            # print a deliverable pass log_lines=None.
            lines = out.splitlines()
            shown = lines if log_lines is None else lines[:log_lines]
            for line in shown:
                self.log(f"      {line}")
            if len(lines) > len(shown):
                self.log(f"      ... {len(lines) - len(shown)} more line(s) not shown "
                         f"(full output in the command's own log on the Pi)")
        return out

    def num(self, cmd, default=0, dry_value=0, **kw):
        """First integer in the output, or `default` if there is none.

        Empty means "no answer", not "zero" -- keep those distinguishable at
        the call site.
        """
        if DRY and not kw.get("reads"):
            self.log.cmd("pi  ", f"[dry] WOULD RUN: {cmd}")
            self.log(f"      [dry] assuming: {dry_value}")
            return dry_value
        out = self(cmd, check=False, **kw)
        m = re.search(r"(-?\d+)", out or "")
        return int(m.group(1)) if m else default

    def copy_to(self, local, remote_rel):
        dest = f"{self.host}:{self.repo}/{remote_rel}"
        if DRY:
            self.log.cmd("scp ", f"[dry] WOULD COPY: scp {local} {dest}")
            return
        self.log.cmd("scp ", f"scp {local} {dest}")
        p = subprocess.run(["scp", "-q", str(local), dest],
                           capture_output=True, text=True, timeout=120)
        if p.returncode != 0:
            fail("scp", f"{local} -> {dest}: {p.stderr.strip()}")


# ══════════════════════════════════════════════════════════════════════════
#  Windows host hygiene
# ══════════════════════════════════════════════════════════════════════════

# Two per-trial process leaks once put 47 orphans and 17 GB beyond reach, at
# which point Simulink stopped starting entirely.
#   matlab*      : matches MATLAB, MATLABWindow and matlabwindowhelper (5
#                  orphans x up to 600 MB observed). The old PowerShell version
#                  used Get-Process with an exact name, so it killed "matlab"
#                  and left every helper behind.
#   AutoVrtlEnv  : the Unreal executable behind the Simulation 3D blocks. It is
#                  spawned per trial and does NOT die with MATLAB. ~500 MB each.
# Left alone these produce a monotonic pacing collapse (0.769 -> 0.140 over ~10
# trials) that looks like a SLAM result and is not.
#
# MathWorksServiceHost is deliberately NOT killed. It is the licensing daemon
# and it is a SINGLETON: measured at exactly one instance (plus one -Monitor)
# after three consecutive MATLAB launches, so it is not part of the leak. It
# not stopping with MATLAB is a documented MathWorks behaviour, not our bug.
# Killing it every trial only forces the licensing handshake to be redone and
# slows MATLAB startup. MathWorksCrashReporter IS killed -- it is a modal
# dialog that blocks the next launch.
KILL_PATTERN = r"^(matlab|AutoVrtlEnv|MathWorksCrashReporter)"
# What preflight COUNTS as evidence of a dirty host, which is broader.
LEAK_PATTERN = r"^(matlab|AutoVrtlEnv|MathWorksCrashReporter)"


def ps(script, timeout=120, reads=False):
    """Run a PowerShell snippet for the few things Python has no clean API for.

    reads=True means the snippet only observes, so a dry run still runs it.
    """
    if DRY and not reads:
        LOG.cmd("win ", f"[dry] WOULD RUN: powershell -Command {script.strip()}")
        return ""
    LOG.cmd("win ", f"powershell -Command {script.strip()}")
    p = subprocess.run(["powershell", "-NoProfile", "-NonInteractive",
                        "-Command", script],
                       capture_output=True, text=True, timeout=timeout)
    if p.returncode != 0:
        fail("powershell", (p.stderr or p.stdout).strip()[:400])
    return (p.stdout or "").strip()


def leftover_procs():
    out = ps(f"@(Get-Process -EA SilentlyContinue | "
             f"Where-Object {{ $_.Name -match '{LEAK_PATTERN}' }}).Count", reads=True)
    return int(out or 0)


def kill_leftovers():
    ps(f"Get-Process -EA SilentlyContinue | "
       f"Where-Object {{ $_.Name -match '{KILL_PATTERN}' }} | "
       f"Stop-Process -Force -EA SilentlyContinue")
    nap(6, "process teardown")


def wait_matlab_gone(timeout=60):
    """Block until MATLAB has actually finished exiting.

    eng.quit() returns before the process is gone: with the desktop up,
    MATLAB + MATLABWindow take 10-20 s to tear down (measured 17 processes
    still listed immediately after quit, 2 singletons remaining 40 s later).
    Starting the next trial during that window means two MATLABs briefly
    compete for RAM and the free-RAM reading is wrong. Wait, then force-kill
    only what refuses to go.
    """
    if DRY:
        LOG("      [dry] would wait for MATLAB to exit (measured ~20s)")
        return True
    t0 = time.time()
    while time.time() - t0 < timeout:
        if leftover_procs() == 0:
            LOG(f"      MATLAB exited cleanly ({time.time() - t0:.0f}s)")
            return True
        time.sleep(3)
    LOG(f"      MATLAB still up after {timeout}s -- force-killing the remainder")
    kill_leftovers()
    return False


def free_mb():
    out = ps("[int]((Get-CimInstance Win32_OperatingSystem).FreePhysicalMemory/1KB)",
             reads=True)
    return int(out or 0)


def list_autosaves(matlab_dir):
    return sorted(f.name for f in Path(matlab_dir).glob("*.slx.autosave"))


def clear_autosaves(matlab_dir):
    """Delete stale .slx.autosave files.

    Root cause of the 2026-08-06 blocker: the orchestrator force-killed a dirty
    MATLAB every trial, so Simulink left a .slx.autosave behind. With one
    present, `set_param(...,'SimulationCommand','start')` threw NOTHING, status
    stayed `stopped`, and sim time never advanced -- 20 consecutive trials died
    with "Simulink failed to start".

    This script also fixes the cause (quit_matlab below closes models without
    saving and exits gracefully), but a stale file from any earlier session, a
    crash, or a Ctrl-C would still poison the next run. Sweep regardless.
    """
    killed = []
    for f in Path(matlab_dir).glob("*.slx.autosave"):
        killed.append(f.name)
        if DRY:
            LOG(f"      [dry] WOULD DELETE {f}")
        else:
            f.unlink()
    return killed


# ══════════════════════════════════════════════════════════════════════════
#  MATLAB via matlab.engine
#
#  Not COM. COM hands MATLAB errors back as ordinary text, which is why the
#  PowerShell version had to regex '^\s*(\?\?\?|Error using|Error:|...)' and
#  why a failing SimulationCommand was invisible. matlab.engine raises a real
#  MatlabExecutionError, and supports background=True so the blocking probe
#  call can actually be given a timeout.
# ══════════════════════════════════════════════════════════════════════════

_ENG = None


def start_matlab(matlab_dir, desktop):
    global _ENG
    opts = f"-sd \"{matlab_dir}\"" + (" -desktop" if desktop else "")
    if DRY:
        LOG.cmd("mat ", f"[dry] WOULD START: matlab.engine.start_matlab({opts!r})")
        return None
    import matlab.engine
    LOG.cmd("mat ", f"matlab.engine.start_matlab({opts!r})")
    _ENG = matlab.engine.start_matlab(opts)
    return _ENG


def m(cmd, nargout=0, timeout=None, dry_value=None):
    """Run a MATLAB statement. Raises on MATLAB error, unlike COM."""
    if DRY:
        LOG.cmd("mat ", f"[dry] WOULD RUN: {cmd}")
        return dry_value
    import matlab.engine
    LOG.cmd("mat ", cmd)
    try:
        if timeout is None:
            return _ENG.eval(cmd, nargout=nargout)
        fut = _ENG.eval(cmd, nargout=nargout, background=True)
        t0 = time.time()
        while not fut.done():
            if time.time() - t0 > timeout:
                fut.cancel()
                fail("matlab", f"'{cmd}' exceeded {timeout}s -- cancelled")
            time.sleep(0.5)
        return fut.result()
    except matlab.engine.MatlabExecutionError as e:
        fail("matlab", f"{cmd}\n{e}")
    except matlab.engine.EngineError as e:
        fail("matlab", f"engine died running '{cmd}': {e}")


def mval(expr, dry_value=0):
    """Evaluate a MATLAB expression to a Python scalar."""
    if DRY:
        LOG.cmd("mat ", f"[dry] WOULD READ: {expr}")
        LOG(f"      [dry] assuming: {dry_value!r}")
        return dry_value
    # NOT "__v": MATLAB identifiers may not start with an underscore, and the
    # engine rejects the whole statement with "Invalid text character", which
    # reads like an encoding problem rather than a naming one.
    m(f"hilrun_val = {expr};")
    return _ENG.workspace["hilrun_val"]


def quit_matlab():
    """Close models WITHOUT saving, then exit cleanly.

    This is the fix for the autosave blocker. The old orchestrator did
    Stop-Process -Force on a MATLAB holding a dirty model; Simulink's response
    to that is to leave a .slx.autosave, which then prevents the model from
    starting at all. Closing the model first means there is no dirty state to
    autosave and no prompt to block the exit.
    """
    global _ENG
    if DRY:
        LOG.cmd("mat ", "[dry] WOULD QUIT: bdclose('all') then eng.quit()")
        return
    if _ENG is None:
        return
    try:
        m("try, set_param('hil_closed_loop_baseline_RGBD','SimulationCommand','stop'); end")
        m("try, stop(timerfindall); delete(timerfindall); end")
        m("try, bdclose('all'); end")   # 'all' + no save => no autosave
        LOG.cmd("mat ", "eng.quit()")
        _ENG.quit()
    except Exception as e:
        LOG(f"      graceful quit failed ({e}) -- falling back to kill")
    _ENG = None


# ══════════════════════════════════════════════════════════════════════════
#  Simulink lifecycle
# ══════════════════════════════════════════════════════════════════════════

def rmw_for(dds):
    """Map a matrix config's `dds:` arm field (fastrtps | cyclonedds, same
    words used by config/hil/stack/*.yaml's own `network.dds`) to the RMW
    implementation string MATLAB and the Pi both need. One place to say it,
    instead of the same fixed text typed twice — a stack config that needs
    fastrtps (stereo does; see config/hil/stack/ov2slam_stereo.yaml's own
    comment on why) used to get silently forced to cyclonedds here no matter
    what its own config said. Missing/absent `dds:` keeps today's behavior
    (cyclonedds) for every existing arm that doesn't set it.
    """
    return "rmw_fastrtps_cpp" if dds == "fastrtps" else "rmw_cyclonedds_cpp"


def restart_matlab(args):
    """Full process restart per trial. Mandatory, not hygiene.

    Sim pacing halves with every in-session Simulink restart (measured 24.5 ->
    68 -> 138 s for the same displacement-gated probe), so trials 2 and 3 would
    fly a different profile than trial 1 and would not be comparable.

    Does NOT start the stack itself -- that's the separate start_stack(dds)
    call, kept as its own step (see run_trial()) so dds can be threaded in
    from the current arm without this function needing to know about arms.
    """
    quit_matlab()
    wait_matlab_gone()          # eng.quit() returns before the process is gone
    kill_leftovers()            # AutoVrtlEnv survives MATLAB's own exit
    gone = clear_autosaves(args.matlab_dir)
    if gone:
        LOG(f"      cleared stale autosave: {', '.join(gone)}")
    start_matlab(args.matlab_dir, args.desktop)


def start_stack(dds="cyclonedds", stereo=False, left_image=""):
    """stereo=False (default) touches nothing new -- every existing mono arm
    behaves exactly as before. stereo=True calls set_stereo(1) before
    hil_ros_init_LT, per set_stereo.m's own documented call order. Without
    this, the right-eye camera block stays commented out (mono is the
    model's saved default) and the right-eye topic never publishes at all --
    the exact silent failure this project chased down: right-eye relay
    starts and waits correctly, MATLAB just never sends it anything. First
    compile after the toggle rebuilds the accelerator target (1-2 min, per
    set_stereo.m's own comment) -- the existing 240s start timeout below
    already has headroom for that, not extended further without evidence
    it's actually too short.
    """
    m("clear all")
    m(f"setenv('RMW_IMPLEMENTATION','{rmw_for(dds)}'); setenv('ROS_DOMAIN_ID','0')")
    if stereo:
        fail("rgbd", "stereo: true arm given to run_matrix_rgbd.py (RGB-D is left camera only)")
    m("set_rgbd(1)", timeout=180)
    m("run hil_ros_init_LT_rgbd", timeout=180)
    m("load_system('hil_closed_loop_baseline_RGBD'); set_param('hil_closed_loop_baseline_RGBD','StopTime','inf')")
    m("try, read_cmdvel_live_interp; catch, end")
    m("set_param('hil_closed_loop_baseline_RGBD','SimulationCommand','start')", timeout=240)
    nap(5)
    status = mval("string(get_param('hil_closed_loop_baseline_RGBD','SimulationStatus'))",
                  dry_value="running")
    if "running" not in str(status):
        # This is the failure mode the stale .slx.autosave produced: start
        # throws nothing, status stays 'stopped'. Preflight already sweeps
        # autosaves, so if you land here the next move is to open the model in
        # the GUI and press Run -- the dialog shows errors the API swallows.
        fail("simulink-start",
             f"SimulationStatus={status!r}, expected 'running'. Open "
             f"hil_closed_loop_baseline_RGBD in the MATLAB GUI and press Run to see the error.")
    if left_image not in ("", "rgb", "mono"):
        fail("image-type", f"left_image={left_image!r}; expected 'rgb', 'mono' or ''")
    if left_image:
        m(f"setenv('RGBD_COLOUR_MONO','{1 if left_image == 'mono' else 0}')")
    m("run sim_camera_publisher_timer_LT_rgbd", timeout=120)
    nap(2)
    if int(mval("numel(timerfindall('Name','sim_cam_pub_timer_LT'))", dry_value=1)) == 0:
        fail("camera-timer", "sim_cam_pub_timer_LT is not running")


def check_pacing(args):
    """Assert Simulink is holding real time, as sim-time advance per wall second.

    The session that produced this project's worst data degraded from 0.44 to
    0.13 without any error being raised.

    NOT measured as drone displacement under a commanded velocity: that
    conflates command latency, accelerator-target rebuilds and actual pacing,
    and it rejected four healthy trials. Readings are bimodal -- healthy starts
    land at 0.87-0.94, and low readings right after load are the accelerator
    build and JIT, not degradation. So warm up, then resample.
    """
    if DRY:
        # Cannot be simulated: the ratio is wall-clock over a real 10 s window,
        # and with waits skipped the denominator would be ~0. Say so instead of
        # printing an invented pass.
        LOG.cmd("mat ", "[dry] WOULD SAMPLE: get_param('hil_closed_loop_baseline_RGBD',"
                        "'SimulationTime') twice, 10s apart, up to 3 times")
        LOG(f"      [dry] would require ratio in [{args.pacing_lo}, {args.pacing_hi}]")
        return float('nan')
    nap(12)
    ratio = 0.0
    for i in range(1, 4):
        t1 = float(mval("get_param('hil_closed_loop_baseline_RGBD','SimulationTime')"))
        w0 = time.time()
        nap(10)
        t2 = float(mval("get_param('hil_closed_loop_baseline_RGBD','SimulationTime')"))
        ratio = (t2 - t1) / (time.time() - w0)
        if args.pacing_lo <= ratio <= args.pacing_hi:
            return ratio
        LOG(f"      pacing sample {i}: {ratio:.3f} -- resampling")
    fail("pacing", f"ratio {ratio:.3f} after 3 samples "
                   f"(want {args.pacing_lo}-{args.pacing_hi}) -- "
                   f"Simulink is not keeping real time, trial not comparable")


def reset_sim(args):
    m("set_param('hil_closed_loop_baseline_RGBD','SimulationCommand','stop')")
    m("assignin('base','sim_cmdvel',zeros(5,1));"
      "assignin('base','sim_cmdvel_ver',int32(evalin('base','sim_cmdvel_ver'))+1);")
    nap(2)
    m("set_param('hil_closed_loop_baseline_RGBD','SimulationCommand','start')", timeout=240)
    nap(3)
    # The probe has no return-to-home; it assumes it starts at spawn. Without
    # this, trial N+1 silently starts where trial N parked and flies a
    # completely different path.
    away = float(mval("norm(sim_pose(1:3)-[0;20;5])", dry_value=0.0))
    if away > args.respawn_tol_m:
        fail("respawn", f"drone is {away:.2f} m from spawn after sim restart "
                        f"(tolerance {args.respawn_tol_m} m)")


# ══════════════════════════════════════════════════════════════════════════
#  Preflight
# ══════════════════════════════════════════════════════════════════════════

def preflight(pi, arms, args, mutate=True):
    """Check the rig. With mutate=False (doctor) it only reports.

    Doctor must not kill your open MATLAB or delete your files just because you
    asked it whether the rig is healthy. It reports the same findings and tells
    you what a real run would do about them.
    """
    for a in arms:
        # init_gate.enabled=true SILENTLY OVERRIDES --hold-fsm: the gate flies
        # its own warmup, declares SLAM_READY, then launches the
        # detector+controller stack, which fights the MATLAB probe for cmd_vel.
        # Observed once: drone ended 18.8 m PAST the sign.
        # awk, not python -c: nested quotes in a python one-liner do not survive
        # the ssh -> bash quoting chain.
        g = pi("awk '/^init_gate:/{f=1} f&&/^  enabled:/{print $2; exit}' "
               f"config/hil/stack/{a['config']}.yaml", reads=True).strip()
        LOG(f"      {a['config']:<26} init_gate.enabled = {g}")
        if g != "false":
            fail("preflight", f"{a['config']}: init_gate.enabled={g!r}, must be false "
                              f"(it overrides --hold-fsm and launches the controller)")

        # Startup delay must be IDENTICAL across arms or the comparison is
        # unfair: a backend that launches later misses more of the sideways init
        # leg and is then scored over a shorter path. Measured with ORB-SLAM2 at
        # 8 s vs OV2SLAM at 1 s -- ORB-SLAM2's first frame landed at 15.0 s,
        # after the init leg had finished, and it did not initialise until
        # 19.7 m into the mission.
        d = pi(f"grep -E '^ *startup_delay_sec:' config/hil/stack/{a['config']}.yaml "
               f"| grep -oE '[0-9]+'", reads=True).strip()
        LOG(f"      {a['config']:<26} startup_delay_sec = {d}")
        if d != "2":
            fail("preflight", f"{a['config']}: startup_delay_sec={d}, must be 2 "
                              f"(unified across all arms)")

    n = leftover_procs()
    LOG(f"      host  {n} MathWorks/Unreal process(es) present")
    if n and mutate:
        LOG(f"      clearing {n} leftover process(es)")
        kill_leftovers()
    elif n:
        LOG(f"      (a real run would kill these {n} process(es) before trial 1)")

    auto = list_autosaves(args.matlab_dir)
    LOG(f"      host  stale autosaves: {', '.join(auto) if auto else 'none'}")
    if auto and mutate:
        clear_autosaves(args.matlab_dir)
        LOG("      cleared")
    elif auto:
        LOG("      (a real run would delete these -- they stop Simulink starting)")

    # In doctor mode a blocker is reported and the checks continue, so one run
    # tells you everything that is wrong. In a real run the first one stops it.
    problems = []

    def blocker(msg):
        if mutate:
            fail("preflight", msg)
        problems.append(msg)
        LOG(f"      BLOCKER {msg}")

    mb = free_mb()
    LOG(f"      host  free RAM {mb:,} MB")
    if mb < args.min_free_mb:
        blocker(f"only {mb} MB free (need {args.min_free_mb}+). Simulink will not hold real time "
                f"below this" + (f"; note {n} MathWorks/Unreal process(es) are still "
                                 f"up and a real run clears those first" if n else
                                 ". Reboot before running"))

    up = pi('sudo docker ps --format "{{.Names}}"', reads=True).strip()
    LOG(f"      pi    containers up: {up.splitlines() if up else 'none'}")
    if up:
        blocker(f"Pi still has containers up: {up.splitlines()} "
                f"-- run ./run_stack_hil_rgbd.sh stop first")

    dirty = pi("git status --porcelain camera_calib/ | head -5",
               check=False, reads=True).strip()
    if dirty:
        LOG(f"      WARNING camera_calib/ is dirty: {dirty}")

    return problems


# ══════════════════════════════════════════════════════════════════════════
#  One trial
# ══════════════════════════════════════════════════════════════════════════

def run_trial(pi, arm, t, args, run_rel):
    label = f"{arm['name']}_t{t}"
    LOG("")
    LOG(f"---- {label} " + "-" * 46)

    with step(f"{label} / restart MATLAB"):
        restart_matlab(args)
    with step(f"{label} / start Simulink + camera"):
        start_stack(arm.get("dds", "cyclonedds"), arm.get("stereo", False), args.left_image)
    with step(f"{label} / verify pacing"):
        pacing = check_pacing(args)
        LOG(f"      pacing {pacing:.3f}")
    with step(f"{label} / respawn drone"):
        reset_sim(args)

    with step(f"{label} / launch Pi stack"):
        pi(f"if [ -f {args.calib}.matrixbak ]; then "
           f"cp {args.calib}.matrixbak {args.calib}; fi")
        tag = f" --run-tag {args.run_tag}_{arm['name']}" if args.run_tag else ""
        pi(f"nohup ./run_stack_hil_rgbd.sh --config {arm['config']} --mode benchmark "
           f"--hold-fsm{tag} > /tmp/hil_matrix_up.log 2>&1 &")

    with step(f"{label} / wait for camera frames to flow"):
        # A container that is 'up' is not the same as a pipeline that is
        # delivering. The counter must CLIMB.
        ok = False
        for _ in range(40):
            nap(3)
            # dry_value pairs are chosen to satisfy the gate below, so a dry run
            # walks the same single code path instead of branching around it.
            a = pi.num("grep -c 'sim_camera_bridge: frame' /tmp/hil_launch.log 2>/dev/null | tail -1",
                       dry_value=0)
            nap(2)
            b = pi.num("grep -c 'sim_camera_bridge: frame' /tmp/hil_launch.log 2>/dev/null | tail -1",
                       dry_value=10)
            if b > a and b > 5:
                LOG(f"      bridge frames {a} -> {b}")
                ok = True
                break
        if not ok:
            fail("camera-frames", "sim_camera_bridge frame count never climbed")

    with step(f"{label} / wait for SLAM to consume frames"):
        # A climbing bridge counter only proves the BRIDGE is alive. The SLAM
        # sidecar starts later (startup_delay_sec, and ORB-SLAM2 spends a long
        # time loading its vocabulary). Firing the probe before it is consuming
        # means it MISSES THE SIDEWAYS INIT LEG -- the one maneuver built to
        # initialise it. Measured on run_orbslam2_probe_20260802_154843: init leg
        # ran 6.5-12.0 s, first frame arrived at 15.0 s, and it then needed until
        # 31.7 s / 15.1 m to bootstrap off degenerate forward motion.
        #
        # The signal differs by backend. OV2SLAM and ORB-SLAM2 append to a
        # per-frame timing CSV as they track, so a growing file proves
        # consumption. ORB-SLAM3 (Mechazo11 wrapper) accumulates in memory and
        # writes at shutdown, so that file never grows and this gate would time
        # out on a healthy node. For it, wait for the handshake ACK instead --
        # that proves the C++ node cleared `initialized_`, which is what actually
        # unblocks frame processing.
        bag_rel = "bags/$(ls -1t bags/ | head -1)"
        ok = False
        for _ in range(40):
            if arm["name"].startswith("orbslam3"):
                nap(3)
                if pi.num(f"grep -c 'handshake ACKed' {bag_rel}/slam_sidecar.log "
                          f"2>/dev/null | tail -1", dry_value=1) > 0:
                    LOG("      SLAM ready (handshake ACKed)")
                    ok = True
                    break
            else:
                a = pi.num(f"cat {bag_rel}/ov2slam_timing_events.csv "
                           f"{bag_rel}/timing_events.csv 2>/dev/null | wc -l",
                           dry_value=0)
                nap(3)
                b = pi.num(f"cat {bag_rel}/ov2slam_timing_events.csv "
                           f"{bag_rel}/timing_events.csv 2>/dev/null | wc -l",
                           dry_value=20)
                if b > a and b > 10:
                    LOG(f"      SLAM consuming frames ({a} -> {b} events)")
                    ok = True
                    break
        if not ok:
            pi("./run_stack_hil_rgbd.sh stop", check=False)
            fail("slam-readiness", "SLAM backend never started consuming frames")
    nap(2)   # let the frontend settle before the init leg

    t0 = time.time()
    with step(f"{label} / PHASE 0 init cycle"):
        m(f"SIX_VX_OVERRIDE = {args.vx}; YAW_AMP_OVERRIDE = 0;")
        m("PROBE_PHASE_OVERRIDE = 'init';")
        m("run slam_traj_probe_rgbd", timeout=args.probe_timeout)

    with step(f"{label} / verify SLAM initialised"):
        # Ask the LIVE topic, not the bag. count_poses.py opens the bag with
        # rosbags.AnyReader, which needs metadata.yaml -- only written when
        # recording STOPS. Reading a still-open bag mid-trial always returns 0,
        # which failed ORB-SLAM3 four times in a row on a backend that works.
        #
        # /slam/tracking_state is the stack's backend-agnostic contract
        # (2 == OK, tracking). Every backend publishes it, so no per-arm cases.
        #
        # EMPTY means "no answer" (the echo's window closed before a message
        # landed), NOT "not initialised". Conflating those failed a healthy
        # ORB-SLAM3 on 1 attempt in 4 and would make a genuine init failure
        # indistinguishable from query flakiness on the very arm we are
        # trying to diagnose. Only a real non-2 value counts as failure.
        # This is a read retry, not a trial retry -- the trial still fails
        # fast once the budget below runs out.
        #
        # init_patience_sec (opt-in per arm, default 9s -- matches this
        # loop's old fixed shape of one read + 3 retries, 3s apart, so every
        # existing arm's behavior is unchanged unless its config sets a
        # bigger number): total wall-clock budget to keep re-reading a real
        # but not-yet-"2" answer, instead of accepting the FIRST real
        # reading as final. Found live: stereo's nb3dkps_ genuinely climbs
        # past its ready threshold, but gradually -- confirmed via the SLAM
        # sidecar's own console log in one real trial, climbing
        # 0->1->3->7->13->50 and staying above 30 -- well past the ~9s the
        # old fixed shape allowed. Default stays small because mono/
        # ORB-SLAM3 arms may deliberately want the fast-fail behavior (see
        # e.g. only_ov2_fast.yaml's own comment: some arms are EXPECTED to
        # abort every trial, and a big default budget would add real
        # wall-clock cost x10 trials to a config built around fast,
        # single-trial failure observation).
        state = None
        arm_rmw = rmw_for(arm.get("dds", "cyclonedds"))
        patience_sec = arm.get("init_patience_sec", 9)
        t0 = time.time()
        s = 0
        while True:
            s += 1
            raw = pi("timeout 15 sudo docker exec ros2_perception_stack bash -lc "
                     "'source /opt/ros/jazzy/setup.bash 2>/dev/null; "
                     f"export RMW_IMPLEMENTATION={arm_rmw}; "
                     "timeout 10 ros2 topic echo /slam/tracking_state --once "
                     "--field data 2>/dev/null' 2>/dev/null", check=False,
                     dry_value="2")
            # `ros2 topic echo --once` prints the value then a YAML "---"
            # separator. Take the first integer; stripping non-digits kept the
            # dashes and gave "2---", which never equals "2" and also failed a
            # healthy backend.
            mt = re.search(r"(\d+)", raw or "")
            if mt:
                state = mt.group(1)
                if state == "2":
                    break
                LOG(f"      tracking_state read {s} = {state} (not ready yet) -- retrying")
            else:
                LOG(f"      tracking_state read {s} returned nothing -- retrying")
            # Same time budget covers BOTH a real "not ready" reading and an
            # empty one -- an empty reading must not be able to loop forever
            # just because it never happens to hit the state=="2" break above.
            if (time.time() - t0) >= patience_sec:
                break
            nap(3)

        ok = (state == "2")
        verdict = "YES" if ok else "NO"
        LOG(f"      SLAM initialised during init cycle: {verdict} (tracking_state={state})")
        # Echo the verdict into the MATLAB console too -- no silent treatment.
        m(f"disp('  [orchestrator] INIT CYCLE COMPLETE -- SLAM initialised: "
          f"{verdict} (tracking_state={state})')")
        if not ok:
            # KILL THE SIMULATOR IMMEDIATELY.
            #
            # Stopping the Pi stack alone left Simulink RUNNING and the drone
            # still flying until the next trial happened to restart MATLAB.
            # That wastes wall clock, leaves the drone parked away from spawn,
            # and keeps an Unreal/AutoVrtlEnv process alive against a failure
            # that is already decided. Nothing more is measured after the init
            # cycle fails, so nothing should still be moving.
            m("try, set_param('hil_closed_loop_baseline_RGBD','SimulationCommand','stop'); end")
            m("clear PROBE_PHASE_OVERRIDE")
            quit_matlab()
            wait_matlab_gone()
            kill_leftovers()
            LOG("      simulator stopped and MATLAB shut down")
            pi("./run_stack_hil_rgbd.sh stop", check=False)
            # Abort THIS TRIAL, not the matrix. A backend that has not
            # initialised by the end of the 3 m sideways leg would otherwise be
            # scored over a shorter path than every other arm, which flatters
            # it -- so the mission is not flown. But the failure is a property
            # of the backend, so it is recorded and the matrix continues.
            raise TrialAborted(f"{arm['name']} did not initialise during the init "
                               f"cycle (tracking_state={state}) -- mission not flown")

    with step(f"{label} / PHASE 1 mission"):
        m("PROBE_PHASE_OVERRIDE = 'mission';")
        m("run slam_traj_probe_rgbd", timeout=args.probe_timeout)
        m("clear PROBE_PHASE_OVERRIDE")
        # In dry-run the probe did not actually block, so t0 measures nothing.
        # Use a representative flight time so the drain figure printed below is
        # the one a real run would use.
        dur = 44.0 if DRY else time.time() - t0
        LOG(f"      probe returned after {dur:.1f}s{' [dry: representative]' if DRY else ''}")

    with step(f"{label} / drain and stop"):
        # Was a fixed sleep (0.45*dur+6) calibrated to a ~13.8 Hz delivery rate.
        # That assumption breaks at any other delivery rate (e.g. a wider
        # stereo baseline slowing frame delivery), truncating the bag
        # mid-flight while `dist_to_target_m` still reports a normal arrival --
        # the mission finished, only the recording didn't catch up. Replaced
        # with a condition-based wait: poll the actual RECORDED
        # /sim/drone_pose stream (not MATLAB's own fast internal sim_pose)
        # until it holds within the mission's own standoff distance for 10
        # consecutive samples, same shape as aggregate_matrix.py's own
        # "3 samples past GT speed < 0.10 m/s" window-end rule.
        # First real use (ORB-SLAM2 stereo, 0.61m baseline) hit this ceiling:
        # recorded arrival needed >120s wall-clock against a 17.6s probe
        # duration, ORB2's front-end is far heavier than OV2SLAM's (~45ms vs
        # ~10ms/frame per earlier session timing), so its recording backlog is
        # proportionally worse. A bigger ceiling costs nothing for a
        # fast-arriving backend, the script exits the moment it actually
        # arrives, so there's no downside to being generous here.
        max_wait = max(90, int(5.0 * dur) + 150)
        LOG(f"      waiting for recorded arrival (max {max_wait}s)")
        arrival = pi(f"./run_stack_hil_rgbd.sh wait-arrival {max_wait}",
                     timeout=max_wait + 15, check=False, dry_value="ARRIVED")
        if "ARRIVED" not in arrival:
            pi("./run_stack_hil_rgbd.sh stop", timeout=180, check=False)
            fail("trial-arrival-timeout",
                 f"recorded /sim/drone_pose never held within standoff for "
                 f"10 consecutive samples within {max_wait}s -- bag would be "
                 f"truncated mid-flight, same failure mode a silent 'ok' used "
                 f"to hide")
        pi("./run_stack_hil_rgbd.sh stop", timeout=180)

    with step(f"{label} / validate trial"):
        invalid = mval("double(init_result.invalid)", dry_value=0)
        dist = float(mval("double(init_result.dist_to_target_m)", dry_value=3.16))
        maxt = mval("double(init_result.maxtime_hit)", dry_value=0)
        nap(3)
        bag = pi("ls -1t bags/ | head -1",
                 dry_value=f"run_{arm['config']}_<stamp>").strip()
        pi(f"cp /tmp/hil_matrix_up.log {run_rel}/logs/{label}_stack.log 2>/dev/null",
           check=False)

        if float(invalid) != 0:
            fail("trial-invalid", "probe reported invalid=true (sim froze)")
        if float(maxt) != 0:
            fail("trial-maxtime", "leg hit the MAXTIME cap -- path shape off-nominal")

        # A bag with zero non-zero /slam/pose entries is a FAILED trial, not a
        # valid one with a bad score. The readiness gate proves frames are being
        # CONSUMED; it cannot prove a map was ever BUILT. Observed on ov2_fast at
        # startup_delay_sec 2: gate passed, frames flowed, zero poses produced --
        # the intermittent VOLATILE-QoS race. Without this check it records as
        # status=ok and silently poisons the mean.
        nposes = pi.num(f"python3 {run_rel}/count_poses.py bags/{bag}", default=-1,
                        dry_value=args.min_poses)
        LOG(f"      non-zero SLAM poses: {nposes}")
        if nposes < args.min_poses:
            # Also a trial-level outcome, not a harness fault: whatever the
            # cause, this trial has no map to score. The reason string keeps it
            # distinguishable from an init-cycle failure in the manifest, since
            # this one can also be the intermittent VOLATILE-QoS race rather
            # than the backend's fault.
            raise TrialAborted(f"only {nposes} non-zero poses (need "
                               f"{args.min_poses}) -- no map recorded")

        LOG(f"      bag={bag} dist={dist:.2f} m")

    return dict(arm=arm["name"], config=arm["config"], scene=arm["scene"],
                trial=t, bag=bag, vx=args.vx,
                dist_to_target_m=dist, probe_sec=round(dur, 1),
                pacing_mps=round(pacing, 3), status="ok")


# ══════════════════════════════════════════════════════════════════════════

def main():
    global LOG, DRY
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", default="",
                    help="config name (config/hil/matrix/<name>.yaml) or a path. "
                         "Defines arms, trials, thresholds and hosts. Flags below "
                         "override individual keys")
    ap.add_argument("--dry-run", action="store_true",
                    help="print every command the run would issue, in order, and "
                         "execute none of them. Read-only checks still run")
    ap.add_argument("--doctor", action="store_true",
                    help="run every precondition check for real and exit. Flies "
                         "nothing, starts no container, changes nothing")
    ap.add_argument("--only", default="", help="run just this arm, e.g. --only orbslam3")
    # Every flag below defaults to None so "not given" is distinguishable from
    # "given the same value the YAML has". Only non-None flags override the file.
    ap.add_argument("--trials", type=int)
    ap.add_argument("--vx", type=float)
    ap.add_argument("--run-tag")
    ap.add_argument("--end-pad", type=int, help="passed to aggregate_matrix.py")
    ap.add_argument("--pi")
    ap.add_argument("--pi-repo")
    ap.add_argument("--matlab-dir")
    ap.add_argument("--probe-timeout", type=int)
    ap.add_argument("--min-free-mb", type=int)
    ap.add_argument("--desktop", action="store_true", default=None,
                    help="launch MATLAB with its desktop (default: on, matches how "
                         "every recorded result so far was produced)")
    ap.add_argument("--headless", dest="desktop", action="store_false",
                    help="launch MATLAB without the desktop. NOT how existing "
                         "results were produced; do not mix the two in one matrix")
    args = ap.parse_args()

    DRY = args.dry_run
    if args.dry_run and args.doctor:
        sys.exit("--dry-run and --doctor are different modes; pick one")

    # Layer the configuration: DEFAULTS <- YAML <- command-line flags.
    cfg, cfg_path = load_config(args.config)
    for key, val in vars(args).items():
        if key in cfg and val is not None:
            cfg[key] = val
    arms_cfg = cfg.pop("arms")
    for key, val in cfg.items():
        setattr(args, key, val)

    arms = [a for a in arms_cfg if not args.only or a["name"] == args.only]
    if not arms:
        sys.exit(f"--only {args.only!r} matched no arm. "
                 f"Known: {', '.join(a['name'] for a in arms_cfg)}")

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    run_rel = f"results/matrix_{stamp}"
    log_dir = SCRIPT_DIR / "logs"
    log_dir.mkdir(exist_ok=True)
    local_log = log_dir / f"run_{stamp}.log"
    LOG = Log(local_log)

    mode = "DRY RUN" if DRY else ("DOCTOR" if args.doctor else "LIVE")
    LOG(f"== SLAM BENCHMARK MATRIX [{mode}] " + "=" * 32)
    LOG(f"      log      {local_log}")
    LOG(f"      config   {cfg_path or '(built-in defaults, no --config given)'}")
    LOG(f"      pi       {args.pi}:{args.pi_repo}")
    LOG(f"      matlab   {args.matlab_dir} ({'desktop' if args.desktop else 'headless'})")
    LOG(f"      matrix   {len(arms)} arm(s) x {args.trials} trial(s) @ vx={args.vx}, round robin")
    LOG(f"      arms     {', '.join(a['name'] for a in arms)}")
    LOG(f"      outdir   {run_rel}")
    LOG(f"      policy   FAIL FAST -- no retries; first bad step stops the run")
    # Print the RESOLVED config, not the file, so what is recorded is what ran.
    LOG("      resolved " + json.dumps(
        {k: v for k, v in sorted(cfg.items()) if k not in ("pi", "pi_repo", "matlab_dir")}))
    if DRY:
        LOG("      NOTE     dry run: nothing below is executed. Lines marked "
            "WOULD RUN / WOULD COPY / WOULD DELETE are the plan.")

    pi = Pi(args.pi, args.pi_repo, LOG)
    atexit.register(quit_matlab)

    rows = []
    try:
        LOG("")
        with step("PREFLIGHT"):
            # Dry-run reports every blocker like doctor does, rather than
            # stopping at the first: the point is to see the whole plan.
            problems = preflight(pi, arms, args, mutate=not (args.doctor or DRY))

        if args.doctor:
            # Start a fresh engine. Does NOT kill anything -- an already open
            # MATLAB of yours is left alone, this is an extra process that quits.
            with step("DOCTOR / MATLAB engine reachable"):
                try:
                    start_matlab(args.matlab_dir, args.desktop)
                    LOG(f"      MATLAB {mval('string(version)')}")
                    LOG(f"      pwd    {mval('string(pwd)')}")
                    quit_matlab()
                    # Prove doctor cleans up after itself rather than claiming it.
                    wait_matlab_gone()
                except StepFailed as e:
                    problems.append(f"MATLAB engine: {e.detail}")
                    LOG(f"      BLOCKER MATLAB engine: {e.detail}")

            LOG("")
            if problems:
                LOG(f"== DOCTOR: {len(problems)} BLOCKER(S) ==")
                for p in problems:
                    LOG(f"   - {p}")
                LOG("   Nothing was flown, killed, or deleted.")
                return 1
            LOG("== DOCTOR: OK, rig is ready ==")
            LOG("   Unverified until a real trial: Simulink pacing, probe execution, "
                "SLAM init. Those need a flight.")
            return 0

        with step("SETUP / stage files on the Pi"):
            pi(f"mkdir -p {run_rel}/logs {run_rel}/configs")
            for a in arms:
                pi(f"cp config/hil/stack/{a['config']}.yaml {run_rel}/configs/")
            pi(f"cp camera_calib/hil_sim_ov2slam_*.yaml {run_rel}/configs/ 2>/dev/null; "
               f"git rev-parse HEAD > {run_rel}/git_commit.txt; "
               f"git status --short > {run_rel}/git_status.txt", check=False)
            pi(f"cp -n {args.calib} {args.calib}.matrixbak")
            pi.copy_to(SCRIPT_DIR / "count_poses.py", f"{run_rel}/")

        # Round robin: A1,B1,C1, A2,B2,C2, ...
        for t in range(1, args.trials + 1):
            for arm in arms:
                try:
                    rows.append(run_trial(pi, arm, t, args, run_rel))
                except TrialAborted as e:
                    # Recorded, not retried, and NOT fatal to the matrix. This
                    # row is the reliability datum for that arm.
                    LOG(f"      recorded as aborted; continuing with the matrix")
                    rows.append(dict(arm=arm["name"], config=arm["config"],
                                     scene=arm["scene"], trial=t, bag="",
                                     vx=args.vx, dist_to_target_m=None,
                                     probe_sec=None, pacing_mps=None,
                                     status=f"aborted: {e.reason}"))
                    try:
                        pi("./run_stack_hil_rgbd.sh stop", check=False, timeout=180)
                    except StepFailed:
                        pass

    except StepFailed as e:
        LOG("")
        LOG(f"FAIL  {e.step}: {e.detail}")
        LOG(f"      {len(rows)} trial(s) completed before this. Nothing was retried.")
        LOG(f"      full trace: {local_log}")
        return 2
    except KeyboardInterrupt:
        LOG("")
        LOG("ABORTED by Ctrl-C")
        return 130
    finally:
        # Doctor promises to change nothing. It stages no calib backup, starts
        # no container, and must not kill processes it did not start -- its own
        # engine is quit gracefully above and that is all it owns.
        if not args.doctor:
            # Never leave the repo calib patched, even on a hard abort.
            try:
                pi(f"if [ -f {args.calib}.matrixbak ]; then "
                   f"cp {args.calib}.matrixbak {args.calib}; "
                   f"rm -f {args.calib}.matrixbak; fi", check=False)
                pi("./run_stack_hil_rgbd.sh stop", check=False, timeout=180)
                LOG("      calib restored, Pi stack stopped")
            except Exception as e:
                LOG(f"      WARNING cleanup failed: {e}")
            quit_matlab()
            kill_leftovers()
        else:
            quit_matlab()

    with step("MANIFEST + AGGREGATE"):
        manifest = dict(stamp=stamp, vx=args.vx, trials=args.trials,
                        end_pad_samples=args.end_pad, runs=rows)
        tmp = log_dir / f"manifest_{stamp}.json"
        if not DRY:
            tmp.write_text(json.dumps(manifest, indent=2), encoding="utf-8")
        pi.copy_to(tmp, f"{run_rel}/manifest.json")
        pi.copy_to(SCRIPT_DIR / "aggregate_matrix.py", f"{run_rel}/")
        pi.copy_to(local_log, f"{run_rel}/run.log")

    LOG("")
    LOG("== AGGREGATING " + "=" * 44)
    # log_lines=None: this is the deliverable. Never truncate it.
    report = pi(f"python3 {run_rel}/aggregate_matrix.py {run_rel} "
                f"--end-pad {args.end_pad}", timeout=900, log_lines=None)
    LOG("")
    if DRY:
        LOG(f"== DRY RUN COMPLETE: plan for {len(rows)} trial(s) printed above ==")
        LOG("   Nothing was executed. No MATLAB, no containers, no flight, no bag.")
        LOG("   Re-run without --dry-run to fly it.")
        return 0
    ok_n = sum(1 for r in rows if r["status"] == "ok")
    LOG(f"== DONE: {ok_n}/{len(rows)} trials valid -> {run_rel}")
    if ok_n != len(rows):
        LOG("   aborted trials (backend produced no map; recorded, not retried):")
        for r in rows:
            if r["status"] != "ok":
                LOG(f"     {r['arm']} t{r['trial']}: {r['status']}")
        LOG("   These are reliability data, not harness faults. They are in "
            "manifest.json.")
    LOG(f"   numbers above were computed on the Pi by aggregate_matrix.py, "
        f"from the bags, not by this script")
    return 0


if __name__ == "__main__":
    sys.exit(main())
