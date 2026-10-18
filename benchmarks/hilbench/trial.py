"""trial.py — one trial = one fixed ladder of named steps; a kind fills five of them.

CONTRACT (fixed): Outcome, TrialResult, StartSpec, RigFailed, BackendFailed, and the
attributes of Trial that a kind may use (listed on the class). RigFailed and BackendFailed
are defined in util.py (so that supervisor.py can raise RigFailed) and re-exported here.

Ladder of Trial.run():
   1 wifi           net.check_links
   2 matlab_setup   kind.matlab_setup
   3 hold           supervisor.hold
   4 start check    kind.start_spec() vs the MATLAB pose (yaw within tolerance of the bearing)
   5 boot           kind.boot  (sets trial.bench, trial.run_dir)
   6 go             kind.go    (gate + resume, in the kind's order)
   7 fly            kind.fly
   8 stop           kind.stop; chown the run folder; append meta.txt
   9 cool-down      settings.cooldown_s
  10 score          kind.score -> TrialResult
"""
import contextlib
import math
import os
import time
import enum
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

import bench_geometry as geo

from . import net
from .gate import RecordGate
from .util import BackendFailed, RigFailed, angle_diff, run  # noqa: F401  (re-exported)


class Outcome(str, enum.Enum):
    VALID = "valid"
    BACKEND_FAILED = "backend_failed"   # the thing under test failed: a result, counted
    RIG_FAILED = "rig_failed"           # the harness did not do what it says: not comparable


@dataclass
class TrialResult:
    outcome: Outcome
    reason: str = "ok"
    sequence: list = field(default_factory=list)     # e.g. FSM states; kinds may leave it empty
    meta: dict = field(default_factory=dict)
    run_dir: Optional[Path] = None
    metrics: dict = field(default_factory=dict)      # what kind.read_results() returned

    @property
    def valid(self):
        return self.outcome == Outcome.VALID


@dataclass(frozen=True)
class StartSpec:
    """Expected start pose. Default = bench_geometry.START, yaw within 5 deg of the bearing to SIGN."""
    x: float
    y: float
    z: float
    yaw_rad: float
    yaw_tol_deg: float = 5.0
    pos_tol_m: float = 0.5


# The ladder: slot -> default step name(s) on the screen. A kind may rename them with an
# optional class attribute step_labels = {slot: "name" or ("name 1", "name 2")}.
_STEP_SLOTS = (("wifi", "Wi-Fi check"), ("setup", "matlab setup"), ("hold", "start_hold"),
               ("start_check", "start check"), ("boot", "boot"), ("go", "go"), ("fly", "flight"),
               ("stop", "stop"), ("cooldown", "cool-down"), ("score", "score"))


class Trial:
    """One trial. Attributes a kind may read or set (CONTRACT):

        k, n_total, cond     run number (1-based), runs in the session, this run's condition
        settings, services   frozen Settings; kinds.base.Services
        meta                 dict -> run_dir/meta.txt (kinds add keys)
        run_dir              Path of the bags/... run folder (the kind sets it in boot())
        bench                container.BenchProcess or None (the kind sets it in boot())
        gate                 gate.RecordGate for this run (created by the Trial)
        flight               dict the screen shows in the flight panel (kinds update it)
        step(name, status, detail="")   status in wait | run | done | fail
        abort_requested()    True when the operator pressed a

    Also: state (a dict a kind may use to keep its own objects between its slots),
    chown_run_dir() (give the run folder back to the user).
    Optional class attributes of a kind that the Trial reads: n_topics (topics the recorder
    must subscribe, default 7), step_labels (see above), run_title(cond) (text on the steps panel).
    """

    def __init__(self, k, n_total, cond, settings, services, kind, screen, keys):
        self.k, self.n_total, self.cond = k, n_total, cond
        self.settings, self.services, self.kind = settings, services, kind
        self.screen, self.keys = screen, keys
        self.meta = {}
        self.run_dir = None
        self.bench = None
        self.state = {}
        self.gate = RecordGate(services.ws_root, services.session.name, k, getattr(kind, "n_topics", 7))
        self._running = None        # the step that started last and is not finished
        self._booted = False
        self._stopped = False
        screen.log_lines = lambda: list(self.bench.lines) if self.bench else None

    # ── what a kind may use ──
    @property
    def flight(self):
        return self.screen.flight

    @flight.setter
    def flight(self, value):
        self.screen.flight = value

    def step(self, name, status, detail=""):
        if status == "run":
            self._running = name
        elif name == self._running:
            self._running = None
        self.screen.step(name, status, detail)

    def abort_requested(self):
        return bool(self.keys.abort)

    def chown_run_dir(self):
        """The bench runs as root in the container: give the run folder back to the user."""
        if self.run_dir is not None and Path(self.run_dir).exists():
            run(["sudo", "-n", "chown", "-R", f"{os.getuid()}:{os.getgid()}", str(self.run_dir)], timeout=30, check=True)

    def _status_of(self, name):
        for n, st, detail in self.screen.steps:
            if n == name:
                return st, detail
        return None, ""

    @contextlib.contextmanager
    def named(self, step):
        """One step of the ladder. RigFailed and BackendFailed pass through; any other exception
        becomes RigFailed("<step>", "<Type>: <msg>"). On success the step is 'done'."""
        self.step(step, "run")
        try:
            yield
        except BackendFailed as e:
            self.step(self._running or step, "fail", e.reason)
            raise
        except RigFailed as e:
            self.step(e.step, "fail", e.why)
            raise
        except Exception as e:
            name = self._running or step
            self.step(name, "fail", f"{e.__class__.__name__}: {e}")
            raise RigFailed(name, f"{e.__class__.__name__}: {e}") from e
        status, detail = self._status_of(step)
        if status == "run":
            self.step(step, "done", detail)

    # ── the ladder ──
    def _names(self, slot):
        default = dict(_STEP_SLOTS)[slot]
        v = getattr(self.kind, "step_labels", {}).get(slot, default)
        return [v] if isinstance(v, str) else list(v)

    def run(self):
        kind, scr = self.kind, self.screen
        title = getattr(kind, "run_title", None)
        scr.run_title = f"run {self.k}/{self.n_total} · " + (title(self.cond) if title else kind.condition_label(self.cond))
        scr.steps, scr.flight = [], None
        for slot, _ in _STEP_SLOTS:
            for n in self._names(slot):
                self.step(n, "wait")
        self.meta["runner_session"] = self.services.session.name
        try:
            res = self._ladder()
        except BackendFailed as e:
            res = TrialResult(Outcome.BACKEND_FAILED, e.reason, meta=dict(self.meta), run_dir=self.run_dir)
        except RigFailed as e:
            res = TrialResult(Outcome.RIG_FAILED, f"{e.step}: {e.why}", meta=dict(self.meta), run_dir=self.run_dir)
        except Exception as e:
            res = TrialResult(Outcome.RIG_FAILED, f"{self._running or '?'}: {e.__class__.__name__}: {e}"[:200],
                              meta=dict(self.meta), run_dir=self.run_dir)
        self._cleanup()
        return res

    def _ladder(self):
        s, kind = self.settings, self.kind
        wifi = self._names("wifi")[0]
        with self.named(wifi):
            rep = net.check_links(s)
            for name, (ok, txt) in rep.lines.items():
                self.screen.set_link(name, ok, txt)
            self.meta.update(rep.info)
            if not rep.ok:
                raise RigFailed(wifi, f"below {s.wifi_min_pct:.0f} % or not readable")
            pct = rep.info.get("wifi_pi_pct")
            self.step(wifi, "done", f"Pi {pct:.0f} %" if pct is not None else "")

        with self.named(self._names("setup")[0]):
            kind.matlab_setup(self, self.cond)

        hold = self._names("hold")[0]
        with self.named(hold):
            st = self.services.supervisor.hold(
                s.hold_timeout_s,
                on_status=lambda t: self.step(hold, "run", f"state={t.get('state')} sim={t.get('sim')} t={t.get('t')}"))
            self.step(hold, "done", f"held at sim t={st.get('t')}")

        with self.named(self._names("start_check")[0]):
            self._start_check(st)

        with self.named(self._names("boot")[0]):
            kind.boot(self, self.cond)
        self._booted = True

        with self.named(self._names("go")[0]):
            kind.go(self, self.cond)

        with self.named(self._names("fly")[0]):
            kind.fly(self, self.cond)

        self._stop()

        cool = self._names("cooldown")[0]
        for sec in range(int(s.cooldown_s), 0, -1):
            self.step(cool, "run", f"{sec} s")
            time.sleep(1.0)
        self.step(cool, "done")

        sc = self._names("score")[0]
        with self.named(sc):
            res = kind.score(self)
            self.step(sc, "done" if res.valid else "fail", res.reason)
        if not res.meta:
            res.meta = dict(self.meta)
        if res.run_dir is None:
            res.run_dir = self.run_dir
        return res

    def _start_check(self, st):
        """The MATLAB pose must match kind.start_spec(): yaw within the tolerance of the bearing to the sign."""
        name = self._names("start_check")[0]
        spec = self.kind.start_spec()
        if not all(key in st for key in ("x", "y", "yaw")):
            raise RigFailed(name, "status has no pose (yaw/x/y) — new read_cmdvel_live_interp.m on the MATLAB path?")
        brg = math.degrees(math.atan2(geo.SIGN[1] - st["y"], geo.SIGN[0] - st["x"])) % 360.0
        dyaw = angle_diff(st["yaw"], brg)
        pos_err = math.dist((st["x"], st["y"], st.get("z", spec.z)), (spec.x, spec.y, spec.z))
        self.meta.update({"start_x": st["x"], "start_y": st["y"], "start_z": st.get("z"), "start_yaw_deg": st["yaw"],
                          "start_bearing_deg": round(brg, 2), "start_yaw_err_deg": round(dyaw, 2),
                          "start_pos_err_m": round(pos_err, 3)})
        if dyaw > spec.yaw_tol_deg:
            raise RigFailed(name, f"yaw {st['yaw']:.1f}° is {dyaw:.1f}° off the bearing {brg:.1f}° (limit {spec.yaw_tol_deg:g}°)")
        if pos_err > spec.pos_tol_m:
            raise RigFailed(name, f"start is {pos_err:.2f} m from ({spec.x:.2f}, {spec.y:.2f}, {spec.z:.2f}) (limit {spec.pos_tol_m:g} m)")
        self.step(name, "done", f"yaw err {dyaw:.2f}° · x={st['x']:.2f} y={st['y']:.2f} z={st.get('z')}")

    def _stop(self):
        """kind.stop, then give the run folder to the user and append meta.txt."""
        if self._stopped:
            return
        self._stopped = True
        name = self._names("stop")[0]
        with self.named(name):
            self.meta.update(self.kind.stop(self) or {})
            if self.run_dir is not None and Path(self.run_dir).exists():
                self.chown_run_dir()
                with open(Path(self.run_dir) / "meta.txt", "a") as f:
                    for key, val in self.meta.items():
                        f.write(f"{key}={val}\n")
            self.step(name, "done", "bag closed · nodes stopped · folder owned by " + os.environ.get("USER", "?"))

    def _cleanup(self):
        """After a failure: the bench is always stopped, also when the ladder broke before the stop step."""
        if self._booted and not self._stopped:
            try:
                self._stop()
            except Exception as e:
                self.services.session.say(f"stop after a failure also failed: {e.__class__.__name__}: {e}")
        if self.bench is not None and self.bench.alive():
            self.bench.stop(kill_names=self.kind.bench_kill_names)
        self.gate.close()
