"""session.py — one session: the folder, the log, the schedule, the loop over trials,
the questions to the operator and the clean exit. It never asks which experiment it runs:
it calls the slots of the RunKind it was given."""
import csv
import datetime as dt
import json
import shutil
import sys
import traceback
from pathlib import Path

from rich.console import Console
from rich.live import Live
from rich.markup import escape
from rich.text import Text

from . import config, model_check, net, summary, ui
from .container import Container
from .overhead import PaceGuard
from .pi_state import PiSettings
from .supervisor import Supervisor, SupervisorError
from .trial import Outcome, RigFailed, Trial
from .util import SCORER_PY, WS_ROOT, run

console = Console()

# files every session freezes into <session>/settings/ (the plug-in adds its own)
COMMON_SNAPSHOT_FILES = ("config/hil/fastrtps_hil.xml", "config/hil/fastrtps_matlab.xml", "config/hil/models.yaml",
                         "benchmarks/bench_geometry.py", "matlab/hil_session_start.m")


def _services_cls(kind_cls):
    """Services lives next to RunKind (kinds/base.py). Found through the class, so that the
    session does not import a module above it."""
    for c in kind_cls.__mro__:
        if c.__name__ == "RunKind":
            return sys.modules[c.__module__].Services
    raise TypeError(f"{kind_cls.__name__} is not a RunKind")


class Session:
    def __init__(self, settings, kind_cls):
        self.settings = settings
        self.ws = WS_ROOT
        stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.name = f"{settings.kind}_{settings.tag}_{stamp}"
        self.folder = self.ws / "reports" / self.name
        self.folder.mkdir(parents=True, exist_ok=True)
        self.slog = open(self.folder / "session.log", "a")
        sha = run(["git", "-C", str(self.ws), "rev-parse", "--short", "HEAD"]).stdout.strip()
        dirty = run(["git", "-C", str(self.ws), "status", "--porcelain", "--untracked-files=no"]).stdout.strip()
        self.git = sha + ("+dirty" if dirty else "")
        self.branch = run(["git", "-C", str(self.ws), "branch", "--show-current"]).stdout.strip()
        self.rows = []
        self.trial = None
        self.pace = PaceGuard(settings.pace_guard, settings.pace_min_ratio)
        self.pi = PiSettings(settings)
        self.keys = ui.Keys()
        self.supervisor = Supervisor(settings.host, settings.port)
        self.container = Container(ws_root=self.ws)
        self.screen = ui.Screen("")
        self.screen.keys = self.keys
        self.screen.on_step = lambda name, status, detail: self.say(f"step {name}: {status} {detail}")
        services = _services_cls(kind_cls)(self.supervisor, self.container, self.screen, self, self.ws)
        self.kind = kind_cls(settings, services)
        self.screen.columns = self.kind.columns
        self.screen.flight_rows = self.kind.flight_rows
        title = getattr(self.kind, "title", self.kind.name)
        dds = {"fastrtps": "Fast DDS", "cyclonedds": "Cyclone DDS"}.get(settings.dds, settings.dds)
        self.screen.header_text = (f"[bold cyan]hilbench[/] · {escape(str(title))} · tag {escape(str(settings.tag))} · "
                                   f"[dim]{escape(self.branch)} @ {self.git}[/] · {dds}")
        self.say(f"session {self.name} · {self.branch} @ {self.git} · settings {config.as_snapshot(settings)}")
        if settings.config_path is None:
            self.deviation(f"settings file for {settings.kind} not found — built-in defaults used")
        if settings.overrides:
            self.say("command line overrides the settings file for: " + ", ".join(settings.overrides))
        if not settings.check_windows_wifi:
            self.deviation("Windows Wi-Fi not checked (network.check_windows_wifi false or --skip-windows-wifi)")

    # ── logging ──
    def say(self, msg):
        self.slog.write(f"{dt.datetime.now().strftime('%H:%M:%S')} {msg}\n")
        self.slog.flush()

    def deviation(self, msg):
        with open(self.folder / "deviations.log", "a") as f:
            f.write(f"{dt.datetime.now().isoformat(timespec='seconds')} {msg}\n")
        self.say("DEVIATION " + msg)

    # ── schedule ──
    def schedule(self):
        """[(k, condition)], numbered from 1: the conditions rotate, `runs` times."""
        conds = self.kind.conditions()
        return list(enumerate([c for _ in range(self.settings.runs) for c in conds], 1))

    # ── preflight ──
    def preflight(self, live=None):
        s, scr = self.settings, self.screen
        free = shutil.disk_usage(self.ws).free / 1e9
        scr.set_link("disk", free > 5, f"{free:.1f} GB free")
        dirty = "+dirty" in self.git
        scr.set_link("git", not dirty, f"{self.branch} @ {self.git}" + ("  (uncommitted changes in tracked files)" if dirty else ""))
        if dirty:
            self.deviation(f"session started on a dirty tree ({self.git})")
        rep = net.check_links(s)
        for name, (ok, txt) in rep.lines.items():
            scr.set_link(name, ok, txt)
        if not rep.ok:
            scr.banner = f"Wi-Fi below {s.wifi_min_pct:.0f} % (or not readable) — stopped before any run."
            return False
        if not self._wait_matlab():
            return False
        if s.dry_run:
            return True
        if not self._check_model():
            return False
        if not self.container.ensure_running(scr, self.folder / "start_container.log"):
            return False
        try:
            self.kind.preflight(self)
        except RigFailed as e:
            scr.banner = f"{e.step}: {e.why} — stopped before any run."
            self.say(f"preflight: {e}")
            return False
        return True

    def _wait_matlab(self):
        s, scr, where = self.settings, self.screen, f"{self.settings.host}:{self.settings.port}"

        def tick(left):
            scr.banner = f"Paste in MATLAB:   hil_session_start        (waiting {left:4.0f} s)"
            scr.set_link("MATLAB supervisor", None, f"{where} · no answer yet")
        if self.supervisor.wait_matlab(s.wait_s, on_tick=tick):
            st = self.supervisor.last_status
            scr.banner = ""
            scr.set_link("MATLAB supervisor", True, f"{where} · state={st.get('state')} sim={st.get('sim')}")
            return True
        scr.banner = f"MATLAB did not answer within {s.wait_s:.0f} s — stopped."
        scr.set_link("MATLAB supervisor", False, f"{where} · no answer in {s.wait_s:.0f} s")
        return False

    def _check_model(self):
        """The model MATLAB has loaded must match config/hil/models.yaml (the Pi is the truth)."""
        key = self.kind.model_key()
        if key is None:
            return True
        scr = self.screen
        try:
            exp = model_check.expected(*key)
            live_info = self.supervisor.model_info()
        except (KeyError, SupervisorError) as e:
            scr.set_link("model", False, f"{e}")
            scr.banner = f"Model check not possible: {e}"
            return False
        bad = model_check.check_live(live_info, exp)
        bad += [m for m in model_check.check_files(ws_root=self.ws) if m.startswith(exp["file"] + ":")]
        if bad:
            for line in bad:
                self.say("model check: " + line)
            scr.set_link("model", False, f"{live_info.get('model')}: {len(bad)} difference(s), first: {bad[0]}")
            scr.banner = "Wrong model in MATLAB — " + "; ".join(bad)[:300]
            return False
        scr.set_link("model", True, f"{live_info.get('model')} matches models.yaml ({key[0]})")
        return True

    # ── Pi settings and snapshot ──
    def snapshot(self, applied):
        """Freeze what this session used: settings file, config files, git state, container image, Pi state."""
        s = self.settings
        sdir = self.folder / "settings"
        sdir.mkdir(exist_ok=True)
        if s.config_path:
            shutil.copy(s.config_path, sdir / Path(s.config_path).name)
        for rel in list(self.kind.snapshot_files()) + list(COMMON_SNAPSHOT_FILES):
            if (self.ws / rel).exists():
                shutil.copy(self.ws / rel, sdir / Path(rel).name)
        (sdir / "git_diff.patch").write_text(run(["git", "-C", str(self.ws), "diff", "HEAD"], timeout=30).stdout)
        (sdir / "session_settings.json").write_text(json.dumps({
            "settings_file": s.config_path,
            "command_line_overrides": list(s.overrides),
            "effective": config.as_snapshot(s),
            "applied_pi_settings": applied,
            "git": {"branch": self.branch, "commit": self.git, "diff_file": "git_diff.patch"},
            "container": {"name": self.container.name, "image_and_created": self.container.image_info()},
            "pi_state_at_start": self.pi.snapshot(),
        }, indent=2, default=str))
        self.say(f"settings snapshot -> {sdir}")

    # ── the loop ──
    def _run_trial(self, k, n, cond):
        self.trial = Trial(k, n, cond, self.settings, self.kind.services, self.kind, self.screen, self.keys)
        try:
            return self.trial.run()
        except Exception as e:      # Trial.run catches everything itself; this is the last net
            self.say("RUNNER ERROR in run %d:\n%s" % (k, traceback.format_exc()))
            from .trial import TrialResult
            return TrialResult(Outcome.RIG_FAILED, f"runner error: {e.__class__.__name__}: {e}"[:160], meta={"condition": str(cond)})

    def run(self):
        s, scr, kind, keys = self.settings, self.screen, self.kind, self.keys
        keys.start()
        stop_note = ""
        try:
            with Live(get_renderable=scr.render, console=console, refresh_per_second=4, screen=False) as live:
                if not self.preflight(live):
                    return 1
                if s.dry_run:
                    self.snapshot(applied={})
                    scr.banner = "Dry run: preflight passed, no command that changes MATLAB was sent."
                    return 0
                applied = self.pi.apply()
                scr.set_link("Pi settings", True, f"pinned cores {s.controller_cores} · runner cores {s.runner_cores} · "
                                                  f"governor {s.cpu_governor} · net buffers {s.net_buffers}")
                self.say(f"Pi settings applied: {applied or 'none (all leave/none)'}")
                self.snapshot(applied)
                sched = self.schedule()
                csv_path = self.folder / "runs.csv"
                with open(csv_path, "w", newline="") as f:
                    csv.writer(f).writerow(["run", getattr(kind, "cond_key", "condition"), "valid", "reason",
                                            "sequence", "run_dir", "outcome"])
                for k, cond in sched:
                    if keys.abort:
                        break
                    res = self._run_trial(k, len(sched), cond)
                    scr.rows.append(summary.format_row(k, cond, res, kind))
                    summary.append_runs_csv(csv_path, [k, kind.condition_label(cond), res.valid, res.reason,
                                                       " -> ".join(res.sequence),
                                                       res.run_dir or res.meta.get("run_dir", ""), res.outcome.value])
                    if keys.abort:
                        break
                    if kind.asks_on(res.outcome) and k < len(sched):
                        if ui.ask_next_or_abort(live, keys, k, res.reason) == "a":
                            keys.abort = True
                            break
                    self.pace.record(k, kind.pace_metric(res))
                    state, text = self.pace.check()
                    if state == "degraded":
                        self.say("pace: " + text)
                        if s.pace_guard == "stop":
                            ratio = self.pace.history[-1][2]
                            stop_note = f"Pace fell to {ratio:.2f} of run 1. Paste hil_session_start in MATLAB, then start the next session."
                            scr.banner = stop_note
                            self.deviation("session stopped by the pace guard: " + text)
                            break
                        self.deviation("pace fell (mode warn, session goes on): " + text)
                    if keys.pause and k < len(sched):
                        ui.wait_for_enter(live, keys)
                try:
                    self.supervisor.stop()
                except SupervisorError:
                    pass
                scr.steps, scr.flight = [], None
                scr.banner = (stop_note + "  " if stop_note else "") + "Session done — summarising…"
                md = summary.finalize(self.folder, kind, SCORER_PY)
                scr.banner = (stop_note + "  " if stop_note else "") + f"Session done. Summary: {self.folder / 'summary.md'}"
            console.print(Text(md.read_text() if md else ""))
            return 0
        finally:
            if self.trial and self.trial.bench is not None and self.trial.bench.alive():
                self.trial.bench.stop(kill_names=kind.bench_kill_names)
            restored = self.pi.restore()
            if restored:
                self.say(f"Pi settings restored: {restored}")
            if not s.dry_run:
                try:
                    self.supervisor.stop()
                    self.say("sent stop to MATLAB")
                except SupervisorError as e:
                    self.say(f"could not send stop to MATLAB: {e}")
            keys.restore()
            self.say("session end")
