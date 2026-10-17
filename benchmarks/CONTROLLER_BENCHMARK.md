# Controller benchmark (HIL) — how to find it and run it

**One command on the Pi:** `./run_controller_benchmark.sh`
**All settings in one file:** [`config/hil/ctrl_bench.yaml`](../config/hil/ctrl_bench.yaml)
**One command in MATLAB first:** `hil_session_start`

The benchmark flies one controller (proportional, IBVS, h_vs or PBVS) against the
MATLAB/Simulink drone, N times, from the same start pose. It records every flight, checks
that each run is valid, scores it, and writes a summary. The detector is the *oracle* (a
perfect bounding box from the true poses), so only the control law differs between controllers.

Approved setup and protocol: Amar Aly, 2026-10-17. The original June design is in
[`CONTROLLER_BENCH_PLAN.md`](CONTROLLER_BENCH_PLAN.md).

---

## 1. Run it (three steps)

**Step 1 — Windows, in MATLAB R2026a (once per MATLAB session):**

```
cd C:\Users\homie\Desktop\ROS2-sim-unified-start\matlab
hil_session_start
```

Wait for `[session] READY`. Then leave MATLAB alone.

**Step 2 — Pi, a terminal in the repo (a real terminal, so the keys work):**

```bash
cd ~/ROS2-sim-unified-start
./run_controller_benchmark.sh --dry-run      # checks only: Wi-Fi, MATLAB, settings
./run_controller_benchmark.sh                # the real session
```

**Step 3 — read the result:** the terminal prints the summary at the end, and the path of
`reports/ctrl_<tag>_<stamp>/summary.md`.

Keys during a session: `p` pause after this run · `a` abort (finished runs stay) · `l` show the bench log.
If a run is not valid, the runner asks: `n` next run or `a` abort. It never repeats a run by itself.

---

## 2. Where everything is

| What | File |
|---|---|
| **Entry point** | `run_controller_benchmark.sh` (repo root) → `benchmarks/ctrl_runner` → `benchmarks/ctrl_runner.py` |
| **All run settings** | `config/hil/ctrl_bench.yaml` |
| MATLAB session setup | `matlab/hil_session_start.m` (model `hil_closed_loop_unified`, start-pose check, Fast DDS profile) |
| MATLAB remote control | `matlab/hil_run_supervisor.m` (TCP 55556: `status`, `start_hold`, `resume`, `stop`, `pub_start`, `pub_stop`) |
| Pi → MATLAB client | `benchmarks/hil_supervisor_client.py` (also by hand: `python3 benchmarks/hil_supervisor_client.py status`) |
| One flight (inside the container) | `benchmarks/controller_hil_bench.sh` → `benchmarks/controller_bench.launch.py` |
| Controller / FSM / oracle parameters | `config/hil/bench_<controller>.yaml`, `config/hil/bench_fsm.yaml`, `config/hil/bench_oracle.yaml` |
| Scoring of one run | `benchmarks/plot_controller_hil.py` → `metrics.csv` + figures |
| Validity + message timing of one run | `benchmarks/ctrl_run_check.py` → `run_check.json` |
| Summary of a session | `benchmarks/ctrl_session_summary.py` → `summary.md`, `summary.csv` |
| Stop distance (3.15 m) | `benchmarks/bench_geometry.py` |
| Shared simulator facts (start pose, target, cameras) | `SIM_PARAMS.md` |

---

## 3. The settings file

Every value below lives in `config/hil/ctrl_bench.yaml`. A command-line option wins over the file
(`./run_controller_benchmark.sh --help` lists them). Each session copies the file it used into
`reports/ctrl_<tag>_<stamp>/settings/`.

| Section | Key | Default | Meaning |
|---|---|---|---|
| session | `controller` | proportional | proportional · ibvs · h_vs · pbvs |
| session | `runs` | 3 | runs **per camera condition** (`ab` gives 2 × runs) |
| session | `camera_pub` | ab | MATLAB camera publisher: off · on · ab (alternate OFF, ON) |
| session | `flight_s` | 150 | seconds of flight after `resume` |
| session | `cooldown_s` | 10 | pause after each run |
| session | `tag` | P_unified_N3 | session name |
| matlab | `host`, `port` | 192.168.1.201, 55556 | where the supervisor listens |
| matlab | `wait_s` | 180 | wait this long for MATLAB, then stop |
| network | `dds`, `ros_domain_id` | fastrtps, 0 | must match MATLAB |
| network | `wifi_min_pct` | 90 | minimum live Wi-Fi on the Pi **and** on Windows (90 % = −55 dBm) |
| pi | `controller_cores` | none | `3` pins the controller to core 3; none = the OS decides |
| pi | `runner_cores` | none | `0-2` keeps the runner and its samplers on cores 0–2 |
| pi | `cpu_governor` | leave | `performance` = fixed CPU speed during the session, restored after |
| pi | `net_buffers` | leave | `hil` = the kernel network buffers of `run_stack_hil.sh` |

Examples:

```bash
./run_controller_benchmark.sh --runs 10 --camera-pub off --tag P_offN10
./run_controller_benchmark.sh --controller-cores 3 --runner-cores 0-2 --cpu-governor performance
./run_controller_benchmark.sh ibvs --tag IBVS_N3
```

---

## 4. What one run does

```
 Wi-Fi check (Pi iw + Windows netsh, both >= 90 %)
   -> camera publisher off|on
   -> start_hold            MATLAB restarts the sim and holds it at the start pose (sim t = 0.25 s)
   -> start check           yaw within 5 deg of the bearing to the sign
   -> boot the bench        oracle + controller in the container; nothing is recorded yet
   -> open the record gate  the bag recorder subscribes to its 7 topics
   -> resume                the drone may move
   -> flight 150 s          live view: FSM state, pose, distance to the sign, Wi-Fi
   -> stop                  bag closed, nodes stopped, run folder owned by the user
   -> cool-down 10 s
   -> score + check         metrics.csv, run_check.json
```

**Stopwatch (t = 0):** the first `/sim/heartbeat` after the hold, i.e. the first message in which
the simulator's own clock has moved past the held value. The time MATLAB answered `resume` is kept
as a cross-check (`resume_ok_minus_t0_s` in `metrics.csv`).

**Valid run:** from t = 0 to the end, the FSM states (repeats joined) are exactly
`SEARCHING -> APPROACHING -> REACHED` or `APPROACHING -> REACHED`. Anything else is not valid.

---

## 5. What it writes

```
reports/ctrl_<tag>_<stamp>/            one session
  session.log                          every step, with time stamps
  runs.csv                             one row per run: valid, reason, FSM sequence, run folder
  summary.md / summary.csv             mean ± std over the valid runs, per camera condition
  deviations.log                       anything that broke the protocol (dirty git tree, skipped checks, ...)
  settings/                            the settings file, the bench config files, git diff, session_settings.json
  run<k>_bench_console.log             the bench output of run k

bags/ctrl_<controller>_N<k>_<stamp>/   one run
  bag/                                 rosbag: /cmd_vel /sim/drone_pose /sim/target_pose /sim/pitch_angle
                                       /sim/heartbeat /yolo/detections /bench/state
  meta.txt                             DDS, camera, Wi-Fi, start pose, resume times, CPU affinity,
                                       temperature + throttle flags before/after, runner overhead
  metrics.csv                          settling time, steady-state error, overshoot, IAE/ITAE/ISE, path
                                       efficiency, RMS command speed, total variation, CPU, memory
  run_check.json                       valid?, FSM sequence, message rates and gaps, Wi-Fi load
  cpu.csv, wifi.csv, fig_*.png
```

---

## 6. If something goes wrong

| What you see | What to do |
|---|---|
| "Paste in MATLAB: hil_session_start" and a countdown | MATLAB is not ready. Run step 1. The runner waits 180 s. |
| `MATLAB supervisor ✘ … no answer` with MATLAB up | Windows firewall: rule "HIL supervisor 55556" must allow the Pi. |
| `Pi Wi-Fi ✘` or `Windows Wi-Fi ✘` | signal below 90 %; move closer, or check the network name in the links panel. |
| `start check ✘` | the model does not start at the shared pose. `hil_session_start` must use `hil_closed_loop_unified`. |
| `container ✘ … runs from another worktree` | stop that container yourself, then run again. |
| a run is not valid | it stays in the data and in `runs.csv`; the summary counts it as k/n. |

---

## 7. Facts to know before you compare numbers

- Results from different setups do not mix. The June N = 10 table used CycloneDDS, another start
  pose and a manual Ctrl-C window; this setup uses Fast DDS, the shared 20 m start and a fixed 150 s window.
- With `camera_pub: on`, MATLAB publishes the images, but nothing on the Pi subscribes in this
  bench, so Fast DDS probably sends nothing over the Wi-Fi (not yet verified).
- The controller is **not** pinned unless `pi.controller_cores` is set.
- MATLAB's heartbeat measured 16.2 Hz on 2026-10-17 (nominal 20 Hz).
