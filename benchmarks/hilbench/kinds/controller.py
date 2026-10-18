"""kinds/controller.py — the HIL controller benchmark as a plug-in (protocol approved 2026-10-17).

Ported from benchmarks/ctrl_runner.py (one_run steps 2, 5, 6, 7, 8, 11). The shared steps
(Wi-Fi, start_hold, start check, cool-down) are in trial.py.

  camera publisher off|on -> boot the bench (RECORD_GATE) -> open gate -> recorder subscribed to
  all 7 topics -> resume (stopwatch t = 0) -> flight flight_s -> stop -> score (plot_controller_hil.py)
  -> check (ctrl_run_check.py).
  Valid: FSM after resume is SEARCHING->APPROACHING->REACHED or APPROACHING->REACHED. Anything else
  is BACKEND_FAILED. The runner never repeats a run; it asks (n = next run, a = abort).
"""
import csv
import json
import math
import re
import shutil
import time
from pathlib import Path

from rich.markup import escape

import bench_geometry as geo

from ..config import FieldSpec
from ..container import BenchProcess
from ..factory import register
from ..net import WifiSampler
from ..overhead import OverheadMeter
from ..pi_state import affinity_of, pi_state, process_memory
from ..supervisor import SupervisorError
from ..trial import Outcome, RigFailed, TrialResult
from ..ui import spark
from ..util import SCORER_PY, run
from .base import RunKind

CONTROLLERS = ("proportional", "ibvs", "h_vs", "pbvs")
EXE = {"proportional": "hil_servo_node", "ibvs": "visp_servo_node", "h_vs": "h_vs_servo_node", "pbvs": "visp_pbvs_node"}
STATE_RE = re.compile(r"State: (\w+) -> (\w+)")
BENCH_SCRIPT = "controller_hil_bench.sh"


@register("controller")
class ControllerKind(RunKind):
    columns = ("run", "camera", "valid", "sequence / reason", "settling s", "ss err m", "overshoot m",
               "path eff", "RMS cmd m/s", "hb gaps>0.2s", "rx MB/s")
    summary_script = "benchmarks/ctrl_session_summary.py"
    bench_kill_names = ("oracle_detector_node", "hil_servo_node", "visp_servo_node", "h_vs_servo_node",
                        "visp_pbvs_node", "ros2 bag record")
    # optional attributes the core reads (see trial.py, session.py)
    n_topics = 7
    cond_key = "camera_pub"           # name of the condition column in runs.csv (the summary script reads it)
    step_labels = {"setup": "camera publisher", "boot": "boot bench", "go": ("recorder", "resume"),
                   "score": "score + check"}

    def __init__(self, settings, services):
        super().__init__(settings, services)
        self.exe_name = EXE[self.cfg["controller"]]
        self.title = f"controller {self.cfg['controller']}"

    # ── settings ──
    @classmethod
    def fields(cls):
        return [
            FieldSpec("controller", "controller", "proportional", options=CONTROLLERS, help="controller under test"),
            FieldSpec("camera_pub", "controller", "off", options=("off", "on", "ab"),
                      help="MATLAB camera publisher; ab = alternate off, on"),
            FieldSpec("camera_pub_hz", "controller", 20.0, cast=float, cli="--pub-hz", help="publisher rate when on, Hz"),
            FieldSpec("flight_s", "controller", 150.0, cast=float, cli="--flight", help="seconds of flight after resume"),
        ]

    @classmethod
    def validate(cls, settings):
        cfg, out = settings.kind_cfg, []
        if cfg.get("controller") not in CONTROLLERS:
            out.append(f"controller.controller: {cfg.get('controller')!r} is not one of {', '.join(CONTROLLERS)}")
        if cfg.get("camera_pub") not in ("off", "on", "ab"):
            out.append(f"controller.camera_pub: {cfg.get('camera_pub')!r} is not one of off, on, ab")
        if cfg.get("flight_s") is None or cfg["flight_s"] <= 0:
            out.append("controller.flight_s must be more than 0")
        return out

    # ── schedule ──
    def conditions(self):
        return ["off", "on"] if self.cfg["camera_pub"] == "ab" else [self.cfg["camera_pub"]]

    def run_title(self, cond):
        return f"camera publisher {cond.upper()}"

    def snapshot_files(self):
        return ["config/hil/bench_fsm.yaml", f"config/hil/bench_{self.cfg['controller']}.yaml",
                "config/hil/bench_oracle.yaml", "config/hil/bench_bag_qos.yaml"]

    # ── one trial ──
    def matlab_setup(self, trial, cond):
        trial.meta.update({"camera_pub": cond, "protocol": "SIM_PARAMS start_hold -> start check -> boot -> resume",
                           "flight_s": self.cfg["flight_s"]})
        sup = self.services.supervisor
        if cond == "off":
            sup.pub_off()
        else:
            sup.pub_on(self.cfg["camera_pub_hz"])
        trial.step("camera publisher", "done", cond)

    def parse_line(self, line, bench):
        m = STATE_RE.search(line)
        if m:
            bench.extra["fsm"] = m.group(2)

    def boot(self, trial, cond):
        s, ses = self.settings, self.services.session
        trial.gate.arm()
        env = {"MATLAB_HOST_IP": s.host, "DDS": s.dds, "ROS_DOMAIN_ID": str(s.ros_domain_id),
               "RUN_TAG": f"{s.tag}_{cond}", "RECORD_GATE": trial.gate.container_path,
               "CTRL_CORE": s.controller_cores}
        argv = [f"./benchmarks/{BENCH_SCRIPT}", self.cfg["controller"], str(trial.k), "0", "true"]
        bench = BenchProcess(argv, ses.folder / f"run{trial.k}_bench_console.log", self.parse_line,
                             container=self.services.container, env=env)
        trial.bench = bench
        t_end = time.monotonic() + s.boot_timeout_s
        while time.monotonic() < t_end and not bench.gate_wait and bench.alive():
            trial.step("boot bench", "run", (bench.lines[-1] if bench.lines else "")[:90])
            time.sleep(0.5)
        if not bench.gate_wait:
            why = bench.error or ("bench exited" if not bench.alive() else f"no gate wait within {s.boot_timeout_s:.0f} s")
            self._stop_bench(bench)
            raise RigFailed("boot bench", why)
        if not bench.run_rel:
            self._stop_bench(bench)
            raise RigFailed("boot bench", "the bench printed no 'Run dir:' line")
        trial.run_dir = self.services.ws_root / bench.run_rel
        trial.meta["run_dir"] = str(trial.run_dir)
        trial.meta["controller_affinity"] = affinity_of(self.exe_name)
        trial.meta["controller_cores_setting"] = s.controller_cores
        trial.step("boot bench", "done", f"{bench.run_rel} · nodes up")

    def _stop_bench(self, bench):
        bench.stop(kill_names=self.bench_kill_names, signal_cmd=["pkill", "-INT", "-f", BENCH_SCRIPT])

    def go(self, trial, cond):
        """Open the gate, wait until the recorder has subscribed to all topics, then resume (t = 0)."""
        ses, sup = self.services.session, self.services.supervisor
        trial.step("recorder", "run")
        trial.gate.open()
        sampler = WifiSampler(ses.folder / f"run{trial.k}_wifi.csv")     # the run dir belongs to root (container)
        trial.state["sampler"] = sampler
        sampler.start()
        n = trial.gate.n_topics
        subs = trial.gate.wait_recorder(trial.run_dir, timeout=15.0,
                                        on_poll=lambda c: trial.step("recorder", "run", f"subscribed {c}/{n}"))
        if subs < n:
            raise RigFailed("recorder", f"recorder subscribed to {subs}/{n} topics within 15 s")
        trial.step("recorder", "done", f"{n}/{n} topics")

        trial.step("resume", "run")
        try:
            sent, ok = sup.resume()
        except SupervisorError as e:
            raise RigFailed("resume", str(e)) from e
        trial.meta.update({"resume_sent_ns": sent, "resume_wall_ns": ok})
        meta_host = ses.folder / f"run{trial.k}_meta.txt"       # on disk now: a crash cannot lose t = 0
        meta_host.write_text("".join(f"{key}={val}\n" for key, val in trial.meta.items()))
        trial.step("resume", "done", "t = 0")

    def fly(self, trial, cond):
        sup, flight_s = self.services.supervisor, self.cfg["flight_s"]
        sampler = trial.state["sampler"]
        trial.step("flight", "run")
        t_start = time.monotonic()
        trial.flight = {"total": flight_s, "elapsed": 0.0, "dhist": [], "pub": cond}
        fl = trial.flight
        calls0 = sup.status_calls
        meter = OverheadMeter(lambda: sup.status_calls - calls0)    # status calls of the flight only
        trial.meta.update({f"{key}_before": v for key, v in pi_state().items()})
        while True:
            el = time.monotonic() - t_start
            fl["elapsed"] = el
            if el >= flight_s or trial.abort_requested():
                break
            try:
                st = sup.status(timeout=2)
                d = math.dist((st.get("x", 0), st.get("y", 0), st.get("z", 0)), geo.SIGN) if "x" in st else None
                fl.update({"sim": st.get("sim"), "t": st.get("t"), "x": st.get("x"), "y": st.get("y"),
                           "z": st.get("z"), "yaw": st.get("yaw"), "dist": f"{d:.2f}" if d is not None else "?"})
                if d is not None:
                    fl["dhist"] = (fl["dhist"] + [d])[-40:]
            except SupervisorError:
                fl["sim"] = "no status"
            fl["fsm"] = trial.bench.extra.get("fsm", "—")
            fl["rx"] = f"{sampler.rates[-1]:.2f}" if sampler.rates else "?"
            meter.sample()
            time.sleep(1.0)
        oh = meter.result()
        trial.meta.update(oh)
        trial.meta.update({f"{key}_after": v for key, v in pi_state().items()})
        mem = process_memory(self.exe_name)
        if mem:
            trial.meta.update({"controller_rss_mb": mem["rss_mb"], "controller_hwm_mb": mem["hwm_mb"]})
        trial.step("flight", "done" if not trial.abort_requested() else "fail",
                   f"{min(el, flight_s):.0f} s · runner {oh['runner_cpu_pct_core']} % of a core · "
                   f"Pi total {oh['pi_cpu_total_pct']} % · {oh['runner_status_calls']} status calls")

    def stop(self, trial):
        facts = {}
        sampler = trial.state.get("sampler")
        if sampler is not None:
            sampler.stop()
            if sampler.error:
                facts["wifi_sampler_error"] = sampler.error
        if trial.bench is not None:
            self._stop_bench(trial.bench)
        trial.gate.close()
        trial.chown_run_dir()                       # the Wi-Fi csv is copied into the run folder next
        wifi_host = self.services.session.folder / f"run{trial.k}_wifi.csv"
        if trial.run_dir is not None and wifi_host.exists():
            shutil.copy(wifi_host, trial.run_dir / "wifi.csv")
        return facts

    def score(self, trial):
        ws = self.services.ws_root
        sc = run([SCORER_PY, str(ws / "benchmarks" / "plot_controller_hil.py"), str(trial.run_dir)], timeout=300)
        ck = run([SCORER_PY, str(ws / "benchmarks" / "ctrl_run_check.py"), str(trial.run_dir)], timeout=300)
        (trial.run_dir / "score.log").write_text(sc.stdout + sc.stderr + "\n" + ck.stdout + ck.stderr)
        res = self.read_results(trial.run_dir)
        if not (trial.run_dir / "run_check.json").exists():     # a scorer failed: the harness, not the controller
            return TrialResult(Outcome.RIG_FAILED, "score + check: no run_check.json (see score.log)", meta=trial.meta,
                               run_dir=trial.run_dir, metrics=res)
        outcome = Outcome.VALID if res["valid"] else Outcome.BACKEND_FAILED   # an invalid FSM sequence is a result
        return TrialResult(outcome, res["reason"], sequence=res["sequence"], meta=trial.meta,
                           run_dir=trial.run_dir, metrics=res)

    # ── results ──
    def read_results(self, run_dir):
        run_dir = Path(run_dir)
        out = {"valid": False, "reason": "no run_check.json", "sequence": []}
        chk = run_dir / "run_check.json"
        if chk.exists():
            c = json.loads(chk.read_text())
            hb = c.get("timing", {}).get("/sim/heartbeat", {})
            hz = hb.get("rate_hz")
            out.update({"valid": c["valid"], "reason": c["reason"], "sequence": c["sequence"],
                        "hb_gaps": hb.get("gaps_over_0.2s"),
                        "heartbeat_hz": hz if isinstance(hz, (int, float)) and math.isfinite(hz) else None,
                        "rx": c.get("wifi", {}).get("rx_mean_MBps")})
        met = run_dir / "metrics.csv"
        if met.exists():
            with open(met) as f:
                for row in csv.reader(f):
                    if len(row) == 2 and row[0] != "metric":
                        out[row[0]] = row[1]
        return out

    def table_row(self, k, cond, res):
        r = res.metrics or {}

        def num(key, fmt="{:.2f}"):
            try:
                return fmt.format(float(r[key]))
            except (KeyError, ValueError, TypeError):
                return "—"
        seq = " → ".join(res.sequence) or res.reason
        shown = escape(seq) if res.valid else f"{escape(seq)} [dim]({escape(res.reason)})[/]"
        return [str(k), str(cond), "[green]✔[/]" if res.valid else "[red]✘[/]", shown,
                num("settling_time_s", "{:.1f}"), num("steady_state_err_m", "{:.3f}"), num("overshoot_m", "{:.3f}"),
                num("path_efficiency", "{:.3f}"), num("rms_cmd_speed"), str(r.get("hb_gaps", "—")), num("rx")]

    def pace_metric(self, res):
        return (res.metrics or {}).get("heartbeat_hz")      # /sim/heartbeat rate_hz of run_check.json

    def asks_on(self, outcome):
        return outcome != Outcome.VALID                     # approved protocol: any invalid run

    # ── screen ──
    def flight_rows(self, f):
        return [(f"sim [green]{f.get('sim', '?')}[/] t={f.get('t', '?')}   FSM [yellow]{f.get('fsm', '—')}[/]",
                 f"camera pub {f.get('pub', '?')}"),
                (f"pose x={f.get('x', '?')} y={f.get('y', '?')} z={f.get('z', '?')} yaw={f.get('yaw', '?')}°",
                 f"Wi-Fi rx {f.get('rx', '?')} MB/s"),
                (f"distance to sign [bold]{f.get('dist', '?')}[/] m  {spark(f.get('dhist', []), geo.STANDOFF - 1, None)}",
                 f"target {geo.STANDOFF:.2f} m")]
