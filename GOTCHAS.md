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

## 3. `ip` command is NOT available inside the container

`ip route get ...` will silently fail or error inside `ros2_perception_stack`.
**Always set manually before running the bench script:**
```bash
export PI_INTERFACE=wlan0
export MATLAB_HOST_IP=192.168.1.201
```
The bench script auto-detects interface via `ip` if `PI_INTERFACE` is unset — this
silently produces an empty string and the script dies on the `envsubst` step.

---

## 4. REACHED at startup is EXPECTED — do NOT Ctrl-C

When MATLAB Simulink is paused between runs, it still publishes the last drone pose
(drone already at dist=3.4m from previous run). The oracle immediately publishes a
large bbox, the FSM transitions SEARCHING→APPROACHING→REACHED in ~130ms.

**Do NOT abort.** Wait for `[bench] recording` then do Stop+Run in MATLAB. The
`have_fresh_sim_` guard now holds the FSM in SEARCHING until sim_t < 2.0s is seen.
After MATLAB Stop+Run, heartbeat drops → `handle_sim_restart()` fires → FSM resets.

---

## 5. `LAUNCH_PID` captures the `tee` PID, not the `ros2 launch` PID

```bash
ros2 launch ... 2>&1 | tee launch.log &
LAUNCH_PID=$!          # ← this is tee's PID
```
`kill "$LAUNCH_PID"` only kills `tee`. The actual `ros2 launch` process keeps running
as an orphan. Use `pgrep -f oracle_detector_node` / `pgrep -f visp_servo_node` to
find and check the real node PIDs.

---

## 6. Always check git SHA before assuming the bench script version

Old script at `26023f3` used `docker exec` to spawn nodes, causing a DDS file path
mismatch (`/tmp/cyclonedds_hil.resolved.xml` not visible from inside a nested exec).
The current script runs everything in one shell inside the container. If runs fail
with CycloneDDS errors, check `meta.txt → git_sha` and compare against the current
HEAD.

---

## 7. `colcon build` must run INSIDE the container

The install tree at `/workspace/install/` is built for the container's ARM64 Linux
environment. Never build on the host and expect it to work on the Pi container.

```bash
ssh amaraly@192.168.1.60
./enter_container.sh
cd /workspace
colcon build --packages-select servo_core visp_servo hil_servo --symlink-install
```

---

## 8. DDS config requires `envsubst` from `gettext-base`

The raw `config/hil/cyclonedds_hil.xml` has `${MATLAB_HOST_IP}` and `${PI_INTERFACE}`
placeholders. `envsubst` materialises them to `/tmp/cyclonedds_hil.resolved.xml`.
If `gettext-base` is missing the bench script dies early with a clear error.

---

## 9. Pre-clamp vs post-clamp velocities in logs

`max_linear` is applied in `publish_cmd_vel()` AFTER the controller output is logged.
IBVS logs showing `vx=182 m/s` are the raw ViSP output — the drone actually moves at
≤3.0 m/s. Don't treat raw log values as actual commanded velocities.

---

## 10. `have_fresh_sim_` guard — how it clears

Set to `false` on node startup. Cleared to `true` in two ways:
- `on_heartbeat()`: when `sim_t < fresh_sim_threshold_sec_` (2.0s) is observed
- `handle_sim_restart()`: fires on mid-run MATLAB Stop+Run (heartbeat drop > 0.5s)

If the FSM appears stuck in SEARCHING and detections are arriving, check whether
`have_fresh_sim_` is still false (log will show `Fresh sim confirmed` when it clears).

---

## 11. Bench script `set -u` — source ROS2 scripts with `set +u` 

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

## 12. SSH is passwordless; container entry is via `./enter_container.sh`

```bash
ssh amaraly@192.168.1.60     # passwordless — key deployed
./enter_container.sh          # enters ros2_perception_stack
# now at /workspace inside the container
```
Do not try to `docker exec` nodes manually — use the bench script which handles
DDS env setup correctly.