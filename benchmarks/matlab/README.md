# MATLAB-side HIL automation

The MATLAB/Simulink sim host is the authoritative source of camera frames and drone
state; the Pi runs the perception stack and closes the loop over ROS 2. The scripts that
run on the sim host live in [`matlab/`](../../matlab/) at the repo root — this page only
covers how the Pi drives them during a sweep.

## Per experiment session (on the sim host)

Run these in order, in one MATLAB session. Each step must be confirmed working before the
next; the supervisor in particular starts **last**, once frames are already streaming.

1. `clear all`, then the three `setenv(...)` ROS 2 environment lines.
2. `hil_ros_init_LT` — wait for `=== LT Init complete ===`.
3. Open and **run** `hil_closed_loop.slx` — wait for `latest_frame` to appear in the workspace.
4. `sim_camera_publisher_timer_LT` — confirm ~20 Hz is arriving on the Pi *first*.
5. `hil_run_supervisor` — returns to the prompt and keeps polling. Stop it with
   `stop_hil_supervisor`.

The supervisor listens on TCP port **55556**: `start` stops the model if running and
restarts it, re-applying every initial condition so the drone returns to its starting
pose; `stop` halts it. It is a non-blocking state machine on a 0.2 s timer — the callback
must never block, because it can fire re-entrantly from inside the Simulink 3D engine's
own step, and a blocking wait there deadlocks the sim.

## Driving it from the Pi

```bash
./benchmarks/sweep_detectors.sh                                    # full 20-config sweep
./benchmarks/run_hil_experiments.sh --label smoke --runs 1 --duration 30
```

The Pi sends `start` before each run and `stop` after it.

**Firewall:** the first time MATLAB opens the port, Windows asks to allow it — allow on
the private / host-only network. Set `MATLAB_HOST_IP` on the Pi to the sim host's address
(default `192.168.56.1`, the host-only adapter).

Quick connectivity check from the Pi:

```bash
echo start | nc -w 3 "$MATLAB_HOST_IP" 55556   # prints "ok" and starts the sim
echo stop  | nc -w 3 "$MATLAB_HOST_IP" 55556   # prints "ok" and stops it
```

## Without the supervisor

Everything still works: the Pi falls back to prompting
`(Re)start the Simulink simulation manually, then press Enter...` between runs. With
`NONINTERACTIVE=1` it skips the prompt and assumes the sim is already streaming — fine for
a smoke test, but there is no per-run drone reset, so trajectories are not comparable
across configs.
