"""container.py — the Docker side: is the right container up, and the process that runs one flight inside it."""
import os
import re
import shlex
import subprocess
import threading
import time
from collections import deque
from pathlib import Path

from .util import WS_ROOT, run


class Container:
    def __init__(self, name="ros2_perception_stack", ws_root=WS_ROOT):
        self.name = name
        self.ws_root = Path(ws_root)

    def ensure_running(self, screen, log_path):
        """True when the container runs with /workspace = this repository. Starts it if it is not running."""
        p = run(["sudo", "-n", "docker", "inspect", "-f",
                 "{{.State.Running}} {{range .Mounts}}{{if eq .Destination \"/workspace\"}}{{.Source}}{{end}}{{end}}",
                 self.name])
        if p.returncode == 0:
            running, _, src = p.stdout.strip().partition(" ")
            if running == "true" and Path(src).resolve() == self.ws_root.resolve():
                screen.set_link("container", True, f"{self.name} · /workspace = {src}")
                return True
            if running == "true":
                screen.set_link("container", False,
                                f"{self.name} runs from {src}, not {self.ws_root} — stop it yourself, then rerun")
                return False
        screen.set_link("container", None, "starting (start_container.sh: create + apt + colcon build)…")
        proc = subprocess.Popen(["bash", str(self.ws_root / "start_container.sh")], stdout=subprocess.PIPE,
                                stderr=subprocess.STDOUT, text=True)
        with open(log_path, "w") as f:
            for line in proc.stdout:
                f.write(line)
                screen.set_link("container", None, "starting… " + line.strip()[:90])
        ok = proc.wait() == 0
        screen.set_link("container", ok, f"{self.name} started" if ok else "start_container.sh failed — see start_container.log")
        return ok

    def exec_cmd(self, args, env=None, timeout=30):
        envs = []
        for k, v in (env or {}).items():
            envs += ["-e", f"{k}={v}"]
        return run(["sudo", "-n", "docker", "exec", *envs, self.name, *args], timeout=timeout)

    def pkill(self, names):
        """pkill -INT -f for every name, inside the container."""
        for name in names:
            self.exec_cmd(["pkill", "-INT", "-f", name], timeout=10)

    def image_info(self):
        """'<image id> <created>' of the container, for the settings snapshot."""
        return run(["sudo", "-n", "docker", "inspect", "-f", "{{.Image}} {{.Created}}", self.name], timeout=15).stdout.strip()


class BenchProcess:
    """A bench script inside the container (or on the host), with its output parsed live.

    Every line goes to the log file, to `lines`, and to parse_line(line, self) of the plug-in.
    """

    def __init__(self, argv, console_log, parse_line, in_container=True, container=None, env=None):
        self.lines = deque(maxlen=400)
        self.run_rel = None
        self.gate_wait = False
        self.recording = False
        self.error = None
        self.extra = {}
        self.parse_line = parse_line
        self.in_container = in_container
        self.container = container
        self._stopped = False
        self.log = open(console_log, "w")
        if in_container:
            if container is None:
                raise ValueError("in_container needs a Container")
            envs = []
            for k, v in (env or {}).items():
                envs += ["-e", f"{k}={v}"]
            cmd = ["sudo", "-n", "docker", "exec", *envs, container.name, "bash", "-lc",
                   "cd /workspace && " + shlex.join(argv)]
            cwd, penv = None, None
        else:
            cmd = list(argv)
            cwd = str(container.ws_root) if container else None
            penv = {**os.environ, **{k: str(v) for k, v in (env or {}).items()}}
        self.proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                     stdin=subprocess.DEVNULL, text=True, bufsize=1, cwd=cwd, env=penv)
        self.reader = threading.Thread(target=self._read, daemon=True)
        self.reader.start()

    def _read(self):
        for line in self.proc.stdout:
            line = line.rstrip("\n")
            self.log.write(line + "\n")
            self.log.flush()
            self.lines.append(line)
            m = re.search(r"Run dir: (bags/\S+)", line)
            if m:
                self.run_rel = m.group(1)
            if "waiting for record gate" in line:
                self.gate_wait = True
            if line.startswith("[bench] recording"):
                self.recording = True
            if "ERROR" in line and line.startswith("[bench]") and self.error is None:
                self.error = line      # the first one is the cause
            if self.parse_line:
                self.parse_line(line, self)

    def _kill(self, cmd_args):
        if self.in_container:
            self.container.exec_cmd(cmd_args, timeout=10)
        else:
            run(cmd_args, timeout=10)

    def alive(self):
        return self.proc.poll() is None

    def stop(self, timeout=25, kill_names=(), signal_cmd=None):
        """Stop the bench. Safe to call twice.

        signal_cmd: argument list that asks the bench script to end (run in the container, or on
        the host), e.g. ["pkill", "-INT", "-f", "<script>"]. Then wait; kill the process if it stays.
        kill_names: process names to pkill -INT afterwards, so none is left publishing into the next run."""
        if self._stopped:
            return
        self._stopped = True
        if signal_cmd:
            self._kill(list(signal_cmd))
        t_end = time.monotonic() + timeout
        while self.alive() and time.monotonic() < t_end:
            time.sleep(0.3)
        if self.alive():
            self.proc.kill()
        for name in kill_names:
            self._kill(["pkill", "-INT", "-f", name])
        time.sleep(1.0)
        self.reader.join(timeout=3)
        self.log.close()
