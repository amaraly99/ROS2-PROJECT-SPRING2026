"""supervisor.py — the Pi's only way to talk to MATLAB: one TCP line out, one line back.

It carries only the protocol every experiment shares. Commands of one experiment live in
that experiment's plug-in and are built on send().
"""
import socket
import time

from .util import RigFailed, now_ns


class SupervisorError(RuntimeError):
    """The supervisor did not answer, or answered with an error."""


def _parse_kv(line, prefix):
    """'<prefix> k=v k=v ...' -> dict. Numbers become floats where they parse; words stay text."""
    if not line.startswith(prefix):
        raise SupervisorError(f"not a {prefix} line: {line!r}")
    out = {}
    for tok in line.split()[1:]:
        if "=" not in tok:
            continue
        k, v = tok.split("=", 1)
        try:
            out[k] = float(v)
        except ValueError:
            out[k] = v
    return out


def parse_status(line):
    """'status sim=running t=1.25 state=held ... yaw=357.36 x=15.12 ...' -> dict."""
    return _parse_kv(line, "status")


class Supervisor:
    def __init__(self, host, port, timeout_s=5.0):
        self.host, self.port, self.timeout_s = host, port, timeout_s
        self.status_calls = 0      # the overhead meter reads this
        self.last_status = {}

    def send(self, cmd, timeout=None):
        """Send one command line, return the reply line (stripped). Raises SupervisorError."""
        timeout = self.timeout_s if timeout is None else timeout
        try:
            with socket.create_connection((self.host, self.port), timeout=timeout) as s:
                s.settimeout(timeout)
                s.sendall((cmd.strip() + "\n").encode())
                buf = b""
                while not buf.endswith(b"\n"):
                    chunk = s.recv(4096)
                    if not chunk:
                        break
                    buf += chunk
        except OSError as e:
            raise SupervisorError(f"no answer from {self.host}:{self.port} ({e.__class__.__name__}: {e})") from e
        reply = buf.decode(errors="replace").strip()
        if not reply:
            raise SupervisorError(f"empty reply to '{cmd}'")
        if reply.startswith("error"):
            raise SupervisorError(f"'{cmd}' -> {reply}")
        return reply

    def status(self, timeout=None):
        self.status_calls += 1
        self.last_status = parse_status(self.send("status", timeout))
        return self.last_status

    def wait_matlab(self, wait_s, on_tick=None, poll_s=2.0):
        """True when MATLAB answers within wait_s. on_tick(seconds_left) is called before each wait."""
        try:
            self.status(timeout=3)
            return True
        except SupervisorError:
            pass
        t_end = time.monotonic() + wait_s
        while time.monotonic() < t_end:
            if on_tick:
                on_tick(t_end - time.monotonic())
            time.sleep(poll_s)
            try:
                self.status(timeout=3)
                return True
            except SupervisorError:
                continue
        return False

    def hold(self, timeout_s, poll_s=1.0, on_status=None):
        """start_hold, then poll until state=held. Returns the status dict.

        'frozen' or a timeout raises RigFailed("start_hold", ...). on_status(status) is called
        after every poll that answered (the screen shows it)."""
        try:
            self.send("start_hold")
        except SupervisorError as e:
            raise RigFailed("start_hold", str(e)) from e
        t_end, st = time.monotonic() + timeout_s, {}
        while time.monotonic() < t_end:
            time.sleep(poll_s)
            try:
                st = self.status()
            except SupervisorError:
                continue
            if on_status:
                on_status(st)
            if st.get("state") in ("held", "frozen"):
                break
        if st.get("state") != "held":
            raise RigFailed("start_hold", f"not held within {timeout_s:.0f} s (state={st.get('state')})")
        return st

    def resume(self):
        """Send resume. Returns (ns before the send, ns after the reply): the cross-check of t = 0."""
        sent = now_ns()
        self.send("resume")
        return sent, now_ns()

    def stop(self):
        self.send("stop")

    def pub_on(self, hz):
        self.send(f"pub_start {hz:g}")

    def pub_off(self):
        self.send("pub_stop")

    def model_info(self):
        """What MATLAB has loaded: model, start values, focal lengths, right camera, depth port."""
        return _parse_kv(self.send("model_info"), "model_info")
