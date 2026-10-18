"""Shared helpers for the hilbench tests: fake kind, fake services, fake TCP supervisor.

Nothing here talks to Docker, ssh, MATLAB or the lab network.
"""
import contextlib
import math
import socket
import subprocess
import threading
import types
from pathlib import Path
from unittest import mock

import bench_geometry as geo
from hilbench.config import COMMON_FIELDS, FieldSpec, Settings
from hilbench.kinds.base import RunKind, Services
from hilbench.trial import Outcome, TrialResult

HERE = Path(__file__).resolve().parent
FIXTURES = HERE / "fixtures"
ROOT = HERE.parents[2]            # tests -> hilbench -> benchmarks -> repository root
BENCH = ROOT / "benchmarks"


def make_settings(kind="fake", kind_cfg=None, **over):
    """A Settings object built from the COMMON_FIELDS defaults (no YAML, no command line)."""
    vals = {f.name: f.default for f in COMMON_FIELDS}
    vals.update(tag="t_unified", cooldown_s=0.0)
    vals.update(over)
    return Settings(kind=kind, kind_cfg=dict(kind_cfg or {}), **vals)


def kind_defaults(kind_cls, **over):
    """kind_cfg with every default of kind_cls.fields(), then the given overrides."""
    cfg = {f.name: f.default for f in kind_cls.fields()}
    cfg.update(over)
    return cfg


def bearing(x, y):
    """Bearing in degrees from (x, y) to the sign, computed here from the geometry tuple."""
    return math.degrees(math.atan2(geo.SIGN[1] - y, geo.SIGN[0] - x)) % 360.0


START_STATUS = {"state": "held", "sim": "running", "t": 0.25, "x": geo.START[0], "y": geo.START[1],
                "z": geo.START[2], "yaw": bearing(geo.START[0], geo.START[1])}


class FakeScreen(mock.MagicMock):
    """Records every step(name, status, detail) call in .steps."""

    def __init__(self, *a, **k):
        super().__init__(*a, **k)
        self.steps = []

    def step(self, name, status, detail=""):
        self.steps.append((name, status, detail))


class FakeKind(RunKind):
    """A minimal complete RunKind. `script` maps a method name to an exception to raise there."""
    name = "fake"
    columns = ("run", "cond")
    n_topics = 7

    @classmethod
    def fields(cls):
        return [FieldSpec("mode", "fake", "a", options=("a", "b")),
                FieldSpec("count", "fake", 3, cast=int),
                FieldSpec("camera", "fake", "off", options=("off", "on", "ab")),
                FieldSpec("label", "fake", "x")]

    @classmethod
    def validate(cls, settings):
        if settings.kind_cfg.get("count", 0) < 0:
            return ["fake: count must not be negative"]
        return []

    def __init__(self, settings, services, conds=("A",), run_dir=None):
        super().__init__(settings, services)
        self.conds = list(conds)
        self.run_dir = run_dir
        self.script = {}
        self.calls = []

    def _do(self, name):
        self.calls.append(name)
        exc = self.script.get(name)
        if exc is not None:
            raise exc

    def conditions(self):
        return list(self.conds)

    def matlab_setup(self, trial, cond):
        self._do("matlab_setup")

    def boot(self, trial, cond):
        self._do("boot")
        trial.run_dir = self.run_dir
        trial.bench = mock.MagicMock()

    def go(self, trial, cond):
        self._do("go")

    def fly(self, trial, cond):
        self._do("fly")

    def stop(self, trial):
        self._do("stop")
        return {"stopfact": 1}

    def score(self, trial):
        self._do("score")
        return TrialResult(Outcome.VALID, "ok", run_dir=trial.run_dir)

    def read_results(self, run_dir):
        return {}

    def table_row(self, k, cond, res):
        return [str(k), str(cond)]


def make_services(tmp, hold_status=None):
    """Services with a mock supervisor whose hold() returns hold_status."""
    sup = mock.MagicMock()
    sup.hold.return_value = dict(START_STATUS if hold_status is None else hold_status)
    sup.status_calls = 0
    session = types.SimpleNamespace(name="sess", folder=Path(tmp), say=lambda m: None,
                                    deviation=lambda m: None)
    return Services(supervisor=sup, container=mock.MagicMock(), screen=FakeScreen(),
                    session=session, ws_root=Path(tmp))


@contextlib.contextmanager
def no_external_commands(links_ok=True):
    """Block docker, sudo, ssh and friends. Records the blocked commands in .blocked.

    Patches subprocess.run (every other command returns success with empty output) and
    net.check_links (a healthy link report, or a failing one).
    """
    state = types.SimpleNamespace(blocked=[], seen=[])
    real_run = subprocess.run
    danger = {"sudo", "docker", "ssh", "scp", "sysctl", "matlab"}

    def fake_run(cmd, *a, **k):
        args = [cmd] if isinstance(cmd, str) else list(cmd)
        state.seen.append(args)
        if args and Path(str(args[0])).name in danger:
            state.blocked.append(args)
        return subprocess.CompletedProcess(args, 0, stdout="", stderr="")

    report = types.SimpleNamespace(ok=links_ok, lines={"Pi Wi-Fi": (links_ok, "fake")},
                                   info={"wifi_pi_ssid": "fake", "wifi_pi_dbm": -40.0, "wifi_pi_pct": 100.0})
    with contextlib.ExitStack() as st:
        st.enter_context(mock.patch("subprocess.run", fake_run))
        st.enter_context(mock.patch("hilbench.net.check_links", return_value=report))
        # in case the ladder imported the name directly
        st.enter_context(mock.patch("hilbench.trial.check_links", return_value=report, create=True))
        state.real_run = real_run
        yield state


class FakeSupervisorServer:
    """A TCP server on 127.0.0.1 (free port). One command line in, one reply line out.

    handler(cmd) -> reply text, or None to close without any reply.
    """

    def __init__(self, handler):
        self.handler = handler
        self.received = []
        self._srv = socket.socket()
        self._srv.bind(("127.0.0.1", 0))
        self._srv.listen(8)
        self._srv.settimeout(0.2)
        self.port = self._srv.getsockname()[1]
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _loop(self):
        while not self._stop.is_set():
            try:
                conn, _ = self._srv.accept()
            except socket.timeout:
                continue
            except OSError:
                return
            with conn:
                conn.settimeout(2.0)
                buf = b""
                try:
                    while not buf.endswith(b"\n"):
                        chunk = conn.recv(4096)
                        if not chunk:
                            break
                        buf += chunk
                except OSError:
                    continue
                cmd = buf.decode().strip()
                self.received.append(cmd)
                reply = self.handler(cmd)
                if reply is not None:
                    conn.sendall((reply + "\n").encode())

    def close(self):
        self._stop.set()
        self._thread.join(timeout=2)
        self._srv.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()


def closed_port():
    """A TCP port on 127.0.0.1 where nothing listens."""
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    return port
