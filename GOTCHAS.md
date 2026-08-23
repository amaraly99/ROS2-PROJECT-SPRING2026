# GOTCHAS — Things That Have Burned Us Before

A running list of traps in this HIL benchmarking pipeline. Update whenever something bites.

---

## 1. `ls | sort | tail` gives ALPHABETICAL order, not chronological

`ibvs` < `proportional` alphabetically, so `ls bags/ | sort | tail -5` always shows
proportional runs as "latest" even if an ibvs run is newer.

**Always use:** `ls -lt ~/ROS2-PROJECT-SPRING2026/bags/ | head -10`

---

## 2. Bag directory timestamps are UTC; host `ls` shows UTC+4

The container runs UTC. Bag names (e.g. `ctrl_ibvs_N1_20260615_203327`) are UTC.
`ls -lt` on the host shows UTC+4, so a file named `203327` (20:33 UTC) shows as
`00:33` the next day on the host. Don't mix the two when reasoning about "which is newer."

---

## 3. `PI_INTERFACE` auto-detection can return `wlan` instead of `wlan0`

Inside the container (especially as root), `ip route get` may return a truncated
interface name. CycloneDDS immediately fails with `wlan: does not match an available interface`.
Both nodes die in < 1s, but pgrep may find stale processes from earlier runs and
falsely report success — recording starts against a dead launch.

**Always set both env vars before running:**
```bash
export PI_INTERFACE=wlan0
export MATLAB_HOST_IP=192.168.137.1
```

---

## 4. Stale node processes from a failed/aborted run fool the pgrep health check

If a previous run left orphaned `oracle_detector_node` / `visp_servo_node` /
`hil_servo_node` processes alive, the 6s health check finds them and declares
"Nodes confirmed running" even though the current launch already crashed.
The script now kills stale nodes at startup (`pkill -f oracle_detector_node` etc.),
but if you ever bypass the script, kill manually first:
```bash
pkill -f oracle_detector_node; pkill -f visp_servo_node; pkill -f hil_servo_node
```

---

## 5. REACHED at startup is EXPECTED — do NOT Ctrl-C

When MATLAB Simulink is paused between runs, it still publishes the last drone pose
(drone already at dist=3.4m from previous run). The oracle immediately publishes a
large bbox, the FSM transitions SEARCHING→APPROACHING→REACHED in ~130ms.

**Do NOT abort.** Wait for `[bench] recording` then do Stop+Run in MATLAB. The
`have_fresh_sim_` guard now holds the FSM in SEARCHING until sim_t < 2.0s is seen.
After MATLAB Stop+Run, heartbeat drops → `handle_sim_restart()` fires → FSM resets.

---

## 6. `LAUNCH_PID` captures the `tee` PID, not the `ros2 launch` PID

```bash
ros2 launch ... 2>&1 | tee launch.log &
LAUNCH_PID=$!          # ← this is tee's PID
```
`kill "$LAUNCH_PID"` only kills `tee`. The actual `ros2 launch` process keeps running
as an orphan. Use `pgrep -f oracle_detector_node` / `pgrep -f visp_servo_node` to
find and check the real node PIDs.

---

## 7. Always check git SHA before assuming the bench script version

Old script at `26023f3` used `docker exec` to spawn nodes, causing a DDS file path
mismatch (`/tmp/cyclonedds_hil.resolved.xml` not visible from inside a nested exec).
The current script runs everything in one shell inside the container. If runs fail
with CycloneDDS errors, check `meta.txt → git_sha` and compare against the current
HEAD.

---

## 8. `colcon build` must run INSIDE the container

The install tree at `/workspace/install/` is built for the container's ARM64 Linux
environment. Never build on the host and expect it to work on the Pi container.

```bash
ssh amaraly@192.168.137.10
./enter_container.sh
cd /workspace
colcon build --packages-select servo_core visp_servo hil_servo --symlink-install
```

---

## 9. DDS config requires `envsubst` from `gettext-base`

The raw `config/hil/cyclonedds_hil.xml` has `${MATLAB_HOST_IP}` and `${PI_INTERFACE}`
placeholders. `envsubst` materialises them to `/tmp/cyclonedds_hil.resolved.xml`.
If `gettext-base` is missing the bench script dies early with a clear error.

---

## 10. Pre-clamp vs post-clamp velocities in logs

`max_linear` is applied in `publish_cmd_vel()` AFTER the controller output is logged.
IBVS logs showing `vx=182 m/s` are the raw ViSP output — the drone actually moves at
≤3.0 m/s. Don't treat raw log values as actual commanded velocities.

---

## 11. `have_fresh_sim_` guard — how it clears

Set to `false` on node startup. Cleared to `true` in two ways:
- `on_heartbeat()`: when `sim_t < fresh_sim_threshold_sec_` (2.0s) is observed
- `handle_sim_restart()`: fires on mid-run MATLAB Stop+Run (heartbeat drop > 0.5s)

If the FSM appears stuck in SEARCHING and detections are arriving, check whether
`have_fresh_sim_` is still false (log will show `Fresh sim confirmed` when it clears).

---

## 12. Bench script `set -u` — source ROS2 scripts with `set +u`

The script uses `set -u` (error on unbound variables). ROS2 setup scripts have
unbound variables internally. The bench script wraps them:
```bash
set +u
source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"
set -u
```
If you add more `source` calls, wrap them the same way or the script will die silently.

---

## 13. CPU sampler logged 0.0 for %cpu AND %mem — wrong PID  [FIXED 2026-06-16]

**Old bug:** the inline sampler did `CTRL_PID=$(pgrep -f "$NODE_EXEC" | head -1)`
then `ps -o %cpu=,%mem= -p $CTRL_PID`. Two faults: `head -1` grabbed a
launch-wrapper / zombie PID (the `%mem=0.0` was the tell — a live node always has
nonzero RSS), and `ps %cpu` is a LIFETIME-AVERAGE, not instantaneous.

**Fix:** replaced with `benchmarks/cpu_sampler.sh`, which:
- resolves the PID by matching `/proc/<pid>/comm` (the real executable name), NOT
  the full cmdline — this excludes shells/wrappers whose argv merely *contains* the
  node name (the sampler itself, `ros2 run`, an ssh command). `pgrep -f` alone will
  match those; the `ros2` launcher has `comm=ros2`, so it's filtered out.
- skips zombies (state Z) and zero-RSS processes.
- computes INSTANTANEOUS %cpu from `/proc/<pid>/stat` utime+stime deltas over the
  sample interval, normalised to one core (top convention).

Validated: `yes` → ~100%, idle binary → 0%, real `visp_servo_node` → picks the node
(comm=visp_servo_node) not the `ros2` wrapper, RSS 27 MB, cpu 0–1% while idle-spinning.
NOTE: comm is truncated to 15 chars by the kernel — `visp_servo_node` is exactly 15,
fine; longer exec names would need the truncation-prefix match (already handled).

## 14. Analyze on the Pi with the pyenv Python, NOT `sudo python3`

`sudo python3` is system Python and has none of the deps (`rosbags`, `matplotlib`).
The packages live in the pyenv install. Over non-interactive SSH the shims aren't on
PATH, so call it by full path:
```bash
~/.pyenv/versions/3.11.15/bin/python3 benchmarks/plot_controller_hil.py <rundir>
```
Also: run dirs are created root-owned inside the container — `sudo chmod -R 777 <dir>`
before the script tries to write metrics.csv/PNGs into them.

## 15. `np.trapz` removed in NumPy 2.0 → use `np.trapezoid`

The Pi has NumPy 2.4. `plot_controller_hil.py` now falls back via
`getattr(np, 'trapezoid', getattr(np, 'trapz', None))` so it works on both 1.x and 2.x.

---

## 16. SSH is passwordless; container entry is via `./enter_container.sh`

```bash
ssh amaraly@192.168.137.10     # passwordless — key deployed
./enter_container.sh          # enters ros2_perception_stack
# now at /workspace inside the container
```
Do not try to `docker exec` nodes manually — use the bench script which handles
DDS env setup correctly.

---

## 17. `docker exec` over non-interactive SSH needs `sudo`

For one-off inspection from a remote (non-interactive) SSH session, `docker exec`
hits `permission denied ... /var/run/docker.sock`. Use sudo:
```bash
ssh amaraly@192.168.137.10 "sudo docker exec ros2_perception_stack bash -c \
  'source /opt/ros/jazzy/setup.bash && source /workspace/install/setup.bash && \
   timeout 5 ros2 topic echo /bench/state --field data | head'"
```
Interactive `./enter_container.sh` does NOT need sudo (user is in the right group
for an interactive login shell; the non-interactive socket access differs).

---

## 18. `have_fresh_sim_` / `benchmark_mode` — the FSM "armed" flag (CRITICAL)

`have_fresh_sim_` answers "am I cleared to engage?". Mapping is easy to invert:
- `true`  = GO. FSM searches/locks/approaches. (fresh sim confirmed, OR scout mode)
- `false` = WAIT. FSM frozen in SEARCHING. **This is the stuck state.**

The drone freezing in SEARCHING for minutes WITH the target in clear view = the
guard is `false` and never got cleared. Symptom in /rosout:
`[SEARCHING/...] ... vx=+0.000 vy=+0.000 vz=+0.000 wz=+0.000` with valid detections.

`benchmark_mode` (ROS param, default true) sets the boot value:
- `benchmark_mode=true`  → `have_fresh_sim_=false` at boot → needs Stop→Run (or
  sim_t<2.0) to flip true. Permanently stuck if nodes start mid-sim (sim_t high).
- `benchmark_mode=false` (SCOUT) → `have_fresh_sim_=true` at boot → engages
  immediately, NO Stop→Run.

4th arg of the bench script: `./controller_hil_bench.sh <ctrl> [run] [dur] [mode]`.
Stop→Run is ONLY a benchmarking need (plant reset to identical ICs + flip guard).
Real scouting must NOT require it — use `benchmark_mode=false`.

A previous "3s fallback timer" that force-cleared the guard was REMOVED — it was a
band-aid. benchmark_mode is the single, clean mechanism. Don't reintroduce the hack.

---

## 19. Drone starts at yaw=π (facing AWAY from target) — search must full-rotate

The Simulink ICs now start the drone at yaw=π (intentional, so search is visible).
The old search only swept ±60° around the start yaw → with yaw=π and target at
world-bearing ~14°, it NEVER pointed at the target → infinite spin, no acquisition.

Fix in place: `SearchStep::FULL_ROTATE` is the FIRST search step (spins a full 360°
at search_spin_speed=0.5 rad/s for search_full_rotate_sec=13s) before the ±60°
refinement. If you change the start yaw or target position, the full-rotate makes
acquisition independent of starting heading — keep it.

---

## 20. Freeze-on-detect: don't keep yawing once the sign is seen

In SEARCHING, if the FSM keeps issuing the search yaw command while detections are
arriving, it sweeps the sign back out of FOV before detection #2 (lockon_consec=2)
→ consecutive_dets_ resets forever → never locks. `build_command()` SEARCHING case
freezes (pitch-return only, no yaw) once `consecutive_dets_ >= 1 && |ex_norm| <
lockon_ex_tol_`. Don't remove the freeze — it's what lets the 2nd detection land.

---

## 21. Oracle is omniscient by default — pose-staleness gate has a hole

`oracle_detector_node.py` projects the target from `/sim/drone_pose` +
`/sim/target_pose` — pure math, sees through walls and at any range. Two gates added:
- `pose_stale_sec=0.3` — suppress if drone-pose age > 0.3s.
  ⚠️ HOLE: MATLAB's publish timer runs on wall-clock and republishes the SAME stale
  pose at 20Hz during a sim pause, so age may never exceed 0.3s. This gate only
  catches true publish GAPS, NOT paused-but-republishing. The real instant-REACHED
  protection is `benchmark_mode` + sim_t<2.0 guard, not this gate.
- `max_detection_range_m=0.0` (OFF by default) — set >0 to require the drone to
  physically approach before the target is "detectable" (realism / demo QoL).

FOV gating (`Zc<=0.1` behind-camera, off-screen u/v bounds) always worked. Zc =
target depth along camera forward axis (>0 ahead, <0 behind, ~0 = 90° to side).