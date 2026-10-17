#!/usr/bin/env python3
"""Client for the MATLAB HIL run supervisor (matlab/hil_run_supervisor.m).

The supervisor listens on TCP port 55556 on the Windows host. One connection
carries one command line and returns one reply line:

    start | start_hold | resume | stop | pub_stop | pub_start [hz]  -> "ok" | "error <msg>"
    status -> "status sim=<..> t=<s> state=<..> pub=<on|off> timers=<n> hz=<..>
               fresh=<n> repeat=<n> drop=<n> [yaw=<deg> x=<m> y=<m> z=<m>]"

CLI:
    python3 benchmarks/hil_supervisor_client.py [--host H] [--port P] status
    python3 benchmarks/hil_supervisor_client.py start_hold
    python3 benchmarks/hil_supervisor_client.py pub_start 20
"""
import argparse
import os
import socket
import sys

DEFAULT_HOST = os.environ.get("MATLAB_HOST_IP", "192.168.1.201")
DEFAULT_PORT = int(os.environ.get("MATLAB_CTL_PORT", "55556"))
COMMANDS = ("status", "start", "start_hold", "resume", "stop", "pub_stop", "pub_start")


class SupervisorError(RuntimeError):
    """The supervisor did not answer, or answered with an error."""


def send(cmd, host=DEFAULT_HOST, port=DEFAULT_PORT, timeout=5.0):
    """Send one command line, return the reply line (stripped). Raises SupervisorError."""
    try:
        with socket.create_connection((host, port), timeout=timeout) as s:
            s.settimeout(timeout)
            s.sendall((cmd.strip() + "\n").encode())
            buf = b""
            while not buf.endswith(b"\n"):
                chunk = s.recv(4096)
                if not chunk:
                    break
                buf += chunk
    except OSError as e:
        raise SupervisorError(f"no answer from {host}:{port} ({e.__class__.__name__}: {e})") from e
    reply = buf.decode(errors="replace").strip()
    if not reply:
        raise SupervisorError(f"empty reply to '{cmd}'")
    if reply.startswith("error"):
        raise SupervisorError(f"'{cmd}' -> {reply}")
    return reply


def parse_status(line):
    """'status sim=running t=1.25 state=held ... yaw=357.36 x=15.12 ...' -> dict.

    Numbers become floats where they parse; '?' and words stay strings.
    """
    if not line.startswith("status"):
        raise SupervisorError(f"not a status line: {line!r}")
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


def status(host=DEFAULT_HOST, port=DEFAULT_PORT, timeout=5.0):
    return parse_status(send("status", host, port, timeout))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default=DEFAULT_HOST)
    ap.add_argument("--port", type=int, default=DEFAULT_PORT)
    ap.add_argument("--timeout", type=float, default=5.0)
    ap.add_argument("command", choices=COMMANDS)
    ap.add_argument("arg", nargs="?", help="rate in Hz for pub_start")
    a = ap.parse_args()
    cmd = a.command + (f" {a.arg}" if a.arg else "")
    try:
        reply = send(cmd, a.host, a.port, a.timeout)
    except SupervisorError as e:
        print(f"ERROR {e}", file=sys.stderr)
        return 1
    print(reply)
    if a.command == "status":
        for k, v in parse_status(reply).items():
            print(f"  {k:8s} {v}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
