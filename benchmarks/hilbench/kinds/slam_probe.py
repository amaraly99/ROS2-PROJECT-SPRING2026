"""kinds/slam_probe.py — the commands that start the SLAM trajectory probe in MATLAB.

Built on the public Supervisor.send(). The Pi never moves the drone: it sets a flag in MATLAB,
and the waiting loop of hil_session_start_slam(...) flies the (generated, unchanged) probe on
the MATLAB main thread. Commands (matlab/hil_run_supervisor.m, CONTRACT.md section 5):

    probe_init    <vx> <cycles> <lambda_m> <yaw_amp_deg>      reply: ok | error busy | error no_loop
    probe_mission <vx> <cycles> <lambda_m> <yaw_amp_deg>
    probe_turn    <...4 numbers...> <cam_yaw_offset_deg> <init_along_path>   (wider view; init and mission
                  also carry the 2 extra numbers then, and MATLAB runs slam_traj_probe_wide.m)
    probe_status  -> probe_status state=idle|running|done invalid=0|1 maxtime_hit=<n>
                     dist_to_target_m=<m|nan> end_x=<m> end_y=<m> end_z=<m>
"""
import time

from ..supervisor import SupervisorError
from ..trial import RigFailed


def parse_probe_status(line):
    """'probe_status state=done invalid=0 ...' -> dict. Numbers become floats, 'nan' stays a float."""
    if not line.startswith("probe_status"):
        raise SupervisorError(f"not a probe_status line: {line!r}")
    out = {}
    for tok in line.split()[1:]:
        if "=" not in tok:
            continue
        key, val = tok.split("=", 1)
        try:
            out[key] = float(val)
        except ValueError:
            out[key] = val
    return out


class ProbeClient:
    """The probe commands over a Supervisor (or anything with send(cmd) -> reply line)."""

    def __init__(self, supervisor, vx, cycles, wavelength_m, yaw_amp_deg, cam_yaw_offset_deg=0.0, init_along_path=0, yaw_ramp=None):
        self.sup = supervisor
        self.args = f"{vx:g} {int(cycles)} {wavelength_m:g} {yaw_amp_deg:g}"
        self.wide = bool(cam_yaw_offset_deg or init_along_path or yaw_ramp is not None)
        if self.wide:   # wider view: MATLAB runs slam_traj_probe_wide.m (6 numbers)
            self.args += f" {cam_yaw_offset_deg:g} {int(bool(init_along_path))}"
            if yaw_ramp is not None:   # end-facing: 7th number, 1 = ramp the camera from the offset to 0 over the cruise
                self.args += f" {int(bool(yaw_ramp))}"

    def turn(self):
        """Wider view only: hover and turn the camera to (bearing to the sign + offset). Returns at once."""
        if not self.wide:
            raise ValueError("turn() needs cam_yaw_offset_deg or init_along_path")
        return self._request("probe_turn")

    def init(self):
        """Phase 0: the sideways init leg. Returns at once; the probe runs in MATLAB."""
        return self._request("probe_init")

    def mission(self):
        """Phase 1: cruise and approach to 3 m before the sign."""
        return self._request("probe_mission")

    def _request(self, name):
        try:
            return self.sup.send(f"{name} {self.args}")
        except SupervisorError as e:
            raise RigFailed(name, str(e)) from e

    def status(self):
        return parse_probe_status(self.sup.send("probe_status"))

    def wait_done(self, timeout_s, poll_s=1.0, on_poll=None, abort=None, idle_grace_s=10.0, what="probe"):
        """Poll probe_status each poll_s until state=done. Returns the last status dict.

        Raises RigFailed when the probe reports invalid=1 (the sim froze), does not finish within timeout_s,
        when MATLAB stays 'idle' after a request for idle_grace_s (the request was lost), or when abort() is true.
        on_poll(status, elapsed_s) is called after every answer."""
        t0 = time.monotonic()
        st, idle_since = {}, None
        while True:
            el = time.monotonic() - t0
            if abort is not None and abort():
                raise RigFailed("probe", "aborted by the operator")
            if el > timeout_s:
                raise RigFailed("probe", f"not finished within {timeout_s:.0f} s (state={st.get('state')})")
            try:
                st = self.status()
            except SupervisorError:
                time.sleep(poll_s)
                continue
            if on_poll:
                on_poll(st, el)
            state = st.get("state")
            if state == "done":
                if st.get("invalid"):
                    raise RigFailed(what, "probe reported invalid=1 (the sim froze)")
                return st
            if state == "idle":
                idle_since = idle_since if idle_since is not None else time.monotonic()
                if time.monotonic() - idle_since > idle_grace_s:
                    raise RigFailed("probe", "MATLAB stayed idle after the request (is the waiting loop running?)")
            else:
                idle_since = None
            time.sleep(poll_s)
