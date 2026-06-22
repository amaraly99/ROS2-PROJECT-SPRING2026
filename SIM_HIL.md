# sim-hil Branch — Hardware-In-the-Loop Simulation

## What this branch does

Replaces the physical OV5647 camera with a synthetic Unreal Engine scene
streamed from a MATLAB/Simulink Windows host. The full
**YOLO → OV2SLAM → hil_servo → /cmd_vel** pipeline runs on real RPi5 hardware.
Only two things change vs. main:

1. The image source is a ROS topic from MATLAB instead of `ovcam_producer`
   reading the OV5647 over libcamera.
2. The servo controller (`hil_servo_node`) uses proportional gains tuned for
   the sim camera geometry (`bench_fsm.yaml` + `bench_proportional.yaml`).

The OV2SLAM, yolo_producer, yolo_bridge, and ovcam_bridge binaries are
byte-for-byte identical between `main` and `sim-hil`. Behaviour changes
come from launch-file parameter overrides, not source edits.

---

## Architecture

```
MATLAB/Simulink (Windows)
  Unreal scene → ROS2 Image publisher → /sim/camera/image_raw
                                        (bgr8, 640×480, ~20 Hz,
                                         BEST_EFFORT+VOLATILE)
                                              ▼
┌──── RPi5  Docker (--net=host --ipc=host --privileged) ─────────────────┐
│                                                                        │
│  sim_camera_bridge    subs /sim/camera/image_raw                       │
│  (C++ node, Core 0)   BGR8 → NV12 → writes /ovcam_frames POSIX SHM    │
│                         (matches ovcam_producer wire format exactly)   │
│                               ▼                                        │
│        ┌──────────────────────┴─────────────────────────┐              │
│        ▼                                                ▼              │
│  ovcam_bridge                                   yolo_producer          │
│  /ovcam/image_raw mono8                         (host, Core 1, Hailo)  │
│        ▼                                                ▼              │
│  ov2slam_node (Cores 2,3)                       yolo_bridge            │
│  /vo_pose, /point_cloud                         /yolo/detections       │
│        └──────────────────────┬─────────────────────────┘              │
│                               ▼                                        │
│                   hil_servo_node (Core 0)                              │
│                   target_class = "stop sign"   (min_conf 0.20)         │
│                   params: bench_fsm.yaml + bench_proportional.yaml     │
│                               ▼                                        │
│                            /cmd_vel                                    │
└────────────────────────────────────────────────────────────────────────┘
                               ▼
          MATLAB Simulink drone dynamics → Unreal scene update
```

**Drone Simulink ICs:** x=0, y=15, z=10 m, yaw=0, pitch=0.
**Stop sign world position** (from `/sim/target_pose`): (35.5, 23.7, 3.2, π).

---

## Prerequisites

### Both machines

| Requirement | Detail |
|---|---|
| ROS2 distro | Jazzy (Pi container + MATLAB toolbox) |
| DDS | CycloneDDS — `rmw_cyclonedds_cpp` on both sides |
| `ROS_DOMAIN_ID` | `0` on both sides |
| Network | Same L2 subnet — no NAT between Pi `eth0` and Windows NIC |

### RPi5

| Requirement | Detail |
|---|---|
| Docker image | `ros2_perception_stack` built from this repo's `Dockerfile` |
| Hailo HAT | `/dev/hailo0` must be present and powered |
| Tools | `taskset` (`util-linux`), `envsubst` (`gettext-base`) |
| CycloneDDS RMW | `rmw_cyclonedds_cpp` installed inside the Docker image |

### MATLAB / Windows

| Requirement | Detail |
|---|---|
| MATLAB | R2024a or later (CycloneDDS RMW support added R2024a) |
| ROS Toolbox | Installed and licensed |
| CycloneDDS RMW | Bundled with MATLAB R2024a+ — no separate install needed |

---

## Network setup (one-time, per machine)

### 1 — Find your IPs

**On the Pi:**
```bash
ip addr show eth0 | grep 'inet '
# example: inet 192.168.1.100/24
```

**On Windows (PowerShell):**
```powershell
Get-NetIPAddress -AddressFamily IPv4 | Select-Object IPAddress, InterfaceAlias
# Note the IP on the NIC connected to the same switch as the Pi (typically "Ethernet", not "Wi-Fi")
```

### 2 — Open Windows Firewall for CycloneDDS

CycloneDDS uses UDP 7400–7500 for discovery and data. Run once in an elevated PowerShell:

```powershell
New-NetFirewallRule -DisplayName "CycloneDDS ROS2 HIL" `
    -Direction Inbound -Protocol UDP `
    -LocalPort 7400-7500 -Action Allow
```

> **Troubleshooting:** If MATLAB and Pi still cannot discover each other after this,
> also open UDP 49152–65535 (ephemeral range), or temporarily disable the firewall
> on the active NIC profile to isolate whether firewall is the cause.

### 3 — Verify connectivity

```bash
# From the Pi
ping <WINDOWS_IP>
```
```powershell
# From Windows PowerShell
ping <PI_IP>
```
Both must succeed before proceeding.

---

## MATLAB startup sequence

Run in order every session. Do not skip or reorder.

### Step 1 — Set DDS environment (before anything else)

In the MATLAB Command Window immediately after opening MATLAB:

```matlab
clear all
setenv('RMW_IMPLEMENTATION','rmw_cyclonedds_cpp')
setenv('ROS_DOMAIN_ID','0')
```

> **`clear all` is only safe here.** Never run it again after Step 2 — it destroys the
> ROS context and requires a full MATLAB restart.
>
> These env vars must be set **before** Step 2 calls `ros2node()`. The ROS context is
> created at first node creation and cannot be reconfigured without restarting MATLAB.

### Step 2 — Initialise ROS

```matlab
run hil_ros_init_LT
```

Watch for `=== LT Init complete ===`. This script:
- Reads `RMW_IMPLEMENTATION` from the environment set in Step 1.
- Calls `ros2node('/matlab_bridge')`, pauses 3 s for CycloneDDS peer discovery.
- Creates all publishers (`/sim/camera/image_raw`, `/sim/pitch_angle`,
  `/sim/drone_pose`, `/sim/heartbeat`, `/sim/target_pose`) and subscribers
  (`/cmd_vel`, `/vo_pose`).
- Starts the 20 Hz state publisher timer (`pitch_pub_timer_LT`).

> **If peer discovery fails** (`ros2('topic','list')` returns no topics or only `/rosout`):
> 1. Confirm the Pi stack is running first — discovery is mutual; MATLAB alone won't show Pi topics.
> 2. Verify Windows Firewall rule is active (`Get-NetFirewallRule -DisplayName "CycloneDDS*"`).
> 3. Check `getenv('RMW_IMPLEMENTATION')` returns `rmw_cyclonedds_cpp` (not empty string).
> 4. If the Pi and Windows have multiple NICs, see the **Force MATLAB to a specific NIC** note in
>    the CycloneDDS section below.

### Step 3 — Open the Simulink model

Open `hil_closed_loop.slx`. Do **not** press Run yet.

### Step 4 — Pre-register the MATLAB Function block

```matlab
read_cmdvel_live_interp
```

This prints an error (wrong number of arguments) — **that is expected and safe to ignore.**
Running it once forces MATLAB to compile and register the `read_cmdvel_live_interp` function.
Without this step, pressing Run on the Simulink model produces a block-level compile error.

### Step 5 — Run the Simulink model

Press **Run** on `hil_closed_loop.slx`. Set simulation pacing to 1 s wall clock (real time).
Unreal Engine launches and `latest_frame` starts populating. Confirm with:

```matlab
whos latest_frame   % expect: 480x640x3  uint8
```

If the RPi5 stack is already running, the drone begins moving as soon as it receives frames.
If not, the drone sits idle — that is normal.

### Step 6 — Start the camera publisher

Once `latest_frame` is confirmed in the workspace:

```matlab
run sim_camera_publisher_timer_LT
```

Edit `PUBLISH_HZ` at the top of that script before running (default 20 Hz). This sends
bgr8 frames at the set rate. The loop is now closed.

### Step 7 — Live view *(optional)*

```matlab
run live_camera_view_LT
```

Single-panel 5 Hz viewer showing `latest_frame`. Reads from local workspace — no extra
network traffic. Close the figure or call `stop(live_view_timer_LT)` to stop.

---

**Startup summary**

| # | Command | Gate |
|---|---|---|
| 1 | `clear all` + 2× `setenv(...)` | Fresh MATLAB session only — before `ros2node()` |
| 2 | `run hil_ros_init_LT` | Must see `=== LT Init complete ===` |
| 3 | Open `hil_closed_loop.slx` | Do not Run yet |
| 4 | `read_cmdvel_live_interp` *(expect error — ignore)* | Fixes Simulink block compile error |
| 5 | Run `hil_closed_loop.slx` | `latest_frame` must appear as 480×640×3 uint8 |
| 6 | `run sim_camera_publisher_timer_LT` | Loop closes; Pi starts receiving frames |
| 7 | `run live_camera_view_LT` *(optional)* | — |

---

## MATLAB topics and workspace variables

### Published by MATLAB → RPi5

| Topic | Type | QoS | Rate | Purpose |
|---|---|---|---|---|
| `/sim/camera/image_raw` | `sensor_msgs/Image` | BEST_EFFORT + VOLATILE + Depth 5 | 20 Hz | bgr8 frames from Unreal (RGB→BGR swapped before publish; checksum-deduped) |
| `/sim/pitch_angle` | `std_msgs/Float64` | RELIABLE + VOLATILE | 20 Hz | Drone pitch (rad) — `hil_servo_node` state feedback |
| `/sim/drone_pose` | `std_msgs/Float64MultiArray` | RELIABLE + VOLATILE | 20 Hz | `[x, y, z, pitch, yaw]` from Simulink integrators |
| `/sim/heartbeat` | `std_msgs/Float64` | RELIABLE + VOLATILE | 20 Hz | Simulink sim time — used as image timestamp and `hil_servo_node` liveness watchdog |
| `/sim/target_pose` | `std_msgs/Float64MultiArray` | RELIABLE + TRANSIENT_LOCAL | 1 Hz | Fixed stop sign world coords `[35.5, 23.7, 3.2, π]` |

### Subscribed by MATLAB ← RPi5

| Topic | Type | Workspace variable | Purpose |
|---|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | `sim_vx/vy/vz/wy/wz`; batch: `sim_cmdvel [5×1]`, `sim_cmdvel_ver` | `hil_servo_node` velocity commands → Simulink drone dynamics |
| `/vo_pose` | `geometry_msgs/PoseStamped` | `sim_vo_pose [x;y;z;qx;qy;qz;qw]`, `sim_vo_pose_count` | OV2SLAM output — logged for SLAM experiment |

### Camera publish rate guide

| Rate | Bandwidth | Notes |
|---|---|---|
| 14 Hz | ~12.9 MB/s | Safe starting point — try first |
| 20 Hz | ~18.4 MB/s | Target; confirm no sim lag at 14 Hz before stepping up |
| 30 Hz | ~27.6 MB/s | Only try if 20 Hz shows zero lag. Simulink fixed-step is 50 ms so unique frames cap at 20 Hz; above 20 Hz the dedup checksum skips resends. |

Coordinate convention (Simulink-confirmed):

| Axis | Positive direction |
|---|---|
| `linear.x` | forward |
| `linear.y` | LEFT |
| `linear.z` | UP |
| `angular.y` | nose down (pitch rate) |
| `angular.z` | yaw left |

---

## Running on the RPi5

### Start (CycloneDDS — primary)

```bash
DDS=cyclonedds MATLAB_HOST_IP=192.168.1.42 ./run_stack_hil.sh
```

Replace `192.168.1.42` with your actual Windows IP.

**Optional flags** (append after the script name):

```bash
# Skip OV2SLAM — faster startup; use for DDS and controller testing only
DDS=cyclonedds MATLAB_HOST_IP=192.168.1.42 ./run_stack_hil.sh --no-slam

# Enable debug image published back to MATLAB (+921 KB/frame extra traffic)
DDS=cyclonedds MATLAB_HOST_IP=192.168.1.42 ./run_stack_hil.sh --debug-image
```

### What the script does (in order)

| Step | Action |
|---|---|
| Pre-flight | Checks `taskset`, `envsubst`, `/dev/hailo0`. Falls back to default `MATLAB_HOST_IP=192.168.56.1` with a warning if unset. |
| IP detection | `ip route get $MATLAB_HOST_IP` → auto-detects `PI_LOCAL_IP` and `PI_INTERFACE`. |
| Kernel tuning | Sets `net.core.rmem_max=16 MB`, `net.ipv4.ipfrag_high_thresh=32 MB` (idempotent). 640×480 bgr8 fragments into ~660 UDP datagrams at MTU 1500; default kernel settings cap receive rate near 13 Hz. |
| DDS profile | `envsubst` substitutes `${MATLAB_HOST_IP}` and `${PI_INTERFACE}` in `config/hil/cyclonedds_hil.xml` → `/tmp/cyclonedds_hil.resolved.xml`. |
| Cleanup | Kills any stale Docker container and SHM files from prior runs. |
| Docker | `docker run -d --net=host --ipc=host --privileged` mounts workspace at `/workspace`. |
| Launch | `ros2 launch sim_camera_bridge hil_simulation.launch.py slam:=false` starts `sim_camera_bridge`, `ovcam_bridge`, `yolo_bridge`, `hil_servo_node` inside Docker. |
| SHM wait | Waits up to 10 s for `/dev/shm/ovcam_frames`; fixes permissions to 0666. |
| yolo_producer | `taskset -c 1 python3 -u yolo_producer.py --conf 0.20 --no-image` on the host. |
| OV2SLAM watchdog | After 15 s delay, starts `ov2slam_node` on cores 2–3 inside Docker, auto-restarting on exit. |

### Stop

```bash
./run_stack_hil.sh stop
```

Kills the Docker container, `yolo_producer`, and clears `/dev/shm/ovcam_frames`,
`/dev/shm/yolo_shm`, and both semaphores.

### Rate check

```bash
./run_stack_hil.sh hz /sim/camera/image_raw
./run_stack_hil.sh hz /yolo/detections
./run_stack_hil.sh hz /cmd_vel
```

Runs `ros2 topic hz` inside the container with the CycloneDDS profile already loaded.

---

## CycloneDDS configuration details

### RPi5 profile (`config/hil/cyclonedds_hil.xml`)

`run_stack_hil.sh` substitutes the variables at launch via `envsubst`:

```xml
<CycloneDDS>
  <Domain id="any">
    <General>
      <AllowMulticast>false</AllowMulticast>
      <Interfaces>
        <NetworkInterface name="${PI_INTERFACE}" priority="default" multicast="false" />
      </Interfaces>
    </General>
    <Discovery>
      <Peers>
        <Peer Address="${MATLAB_HOST_IP}"/>
        <Peer Address="localhost"/>
      </Peers>
      <ParticipantIndex>auto</ParticipantIndex>
    </Discovery>
  </Domain>
</CycloneDDS>
```

| Setting | Why |
|---|---|
| `AllowMulticast=false` | Prevents DDS traffic leaking onto `wlan0` and `docker0` |
| `NetworkInterface` → `${PI_INTERFACE}` | Locks DDS to the physical NIC toward Windows (`eth0` typically) |
| `<Peer Address="${MATLAB_HOST_IP}"/>` | Unicast discovery to Windows host — Pi sends SPDP directly to MATLAB's IP |
| `<Peer Address="localhost"/>` | Lets `ros2 topic list` on the Pi itself see nodes inside Docker |

### MATLAB / Windows — no XML file needed (default case)

Because the Pi sends unicast discovery directly to `MATLAB_HOST_IP`, MATLAB's CycloneDDS
receives those packets and responds without needing a custom XML profile. The only requirements
on the Windows side are the env vars in Step 1 and the firewall rule above.

### Force MATLAB to a specific NIC (multi-NIC machines)

If the Windows machine has both Ethernet and Wi-Fi, set `CYCLONEDDS_URI` in Step 1 to pin
MATLAB to the correct interface. Replace the `NetworkInterface name` and `Peer Address` with
your actual values:

```matlab
setenv('CYCLONEDDS_URI', ['<CycloneDDS><Domain><General>' ...
    '<Interfaces><NetworkInterface name="Ethernet"/></Interfaces>' ...
    '</General><Discovery><Peers>' ...
    '<Peer Address="192.168.1.100"/>' ...   % PI_IP
    '</Peers></Discovery></Domain></CycloneDDS>'])
```

Set this in Step 1, before `run hil_ros_init_LT`.

---

## Verifying the stack

After `run_stack_hil.sh` completes and MATLAB is publishing frames, open a separate Pi terminal:

```bash
# All three should approximately match MATLAB publish rate (~20 Hz)
ros2 topic hz /sim/camera/image_raw
ros2 topic hz /ovcam/image_raw
ros2 topic hz /yolo/detections

# Nonzero when a stop sign is in frame
ros2 topic echo /cmd_vel --once

# Detection stream
ros2 topic echo /yolo/detections --field detections | head -20
```

**Expected log lines** (`/tmp/hil_launch.log`):
```
[sim_camera_bridge]: ready — subscribed to /sim/camera/image_raw, writing /ovcam_frames SHM (640x480 NV12, 4 slots)
[hil_servo_node]: started — tracking 'stop sign' (conf >= 0.20)
```

**Expected yolo_producer output** (`/tmp/yolo_producer.log`):
```
[hailo] input: 640x640x3
```

**Confirm loop is closed** in MATLAB:
```matlab
sim_vx   % nonzero when stop sign is detected and drone is approaching
sim_cmdvel_ver   % counter increments with every /cmd_vel message received
```

---

## FastDDS fallback

Use only if MATLAB is older than R2024a or `rmw_cyclonedds_cpp` is unavailable.

### MATLAB Step 1 (FastDDS)

```matlab
clear all
setenv('RMW_IMPLEMENTATION','rmw_fastrtps_cpp')
setenv('ROS_DOMAIN_ID','0')
setenv('FASTRTPS_DEFAULT_PROFILES_FILE','C:\path\to\your\fastrtps_matlab.xml')
```

> The template is at `config/hil/fastrtps_matlab.xml`. Copy it locally and replace
> `${MATLAB_HOST_IP}` with your Windows IP and `${PI_ADDR}` with your Pi's IP before use.

Steps 2–7 are identical to the CycloneDDS sequence.

### Pi (FastDDS)

```bash
DDS=fastrtps MATLAB_HOST_IP=192.168.1.42 ./run_stack_hil.sh
```

The FastDDS profile (`config/hil/fastrtps_hil.xml`) uses two UDPv4 transports:
`eth0` for cross-machine frames and `127.0.0.1` for intra-Pi node discovery.

---

## Log files

| File | Content |
|---|---|
| `/tmp/hil_launch.log` | `ros2 launch` output — `sim_camera_bridge`, `ovcam_bridge`, `yolo_bridge`, `hil_servo_node` |
| `/tmp/yolo_producer.log` | Host `yolo_producer.py` — per-stage timing, FPS, Hailo NPU init status |
| `/tmp/ov2slam_hil.log` | OV2SLAM watchdog — restart events, SLAM init and tracking status |

---

## Known limitations

- **OV2SLAM on synthetic textures.** Unreal textures sometimes lack KLT-trackable
  features that real-world footage has. If SLAM fails to initialise, `hil_servo_node`
  falls back to bbox-based depth estimate (`known_target_height = 1.5 m` in
  `bench_fsm.yaml`). The control demo still works.
- **Frame rate ceiling = MATLAB publish rate.** Real OV5647 publishes at 60 Hz on
  main; HIL is bottlenecked by Simulink's timer (~20 Hz typical). Publishing above
  20 Hz sends duplicate frames that the dedup checksum skips.
- **Unicast-only, same L2 required.** `AllowMulticast=false` on the Pi means both
  machines must be on the same Ethernet switch (or VLAN). Cross-subnet routing will
  not work without explicit firewall holes on both ends for the DDS UDP port ranges.
- **OV2SLAM deferred 15 s.** The watchdog deliberately waits 15 s before launching
  OV2SLAM so the drone has time to begin motion (parallax required for init). If
  SLAM does not initialise after the drone starts moving, check `/tmp/ov2slam_hil.log`.
- **Docker SHM permissions.** `sim_camera_bridge` runs as root inside Docker and
  creates SHM with 0644. `run_stack_hil.sh` automatically fixes this to 0666 so the
  host-user `yolo_producer` can access it. If you restart Docker manually, re-run
  the `chmod` step or use `./run_stack_hil.sh stop && ./run_stack_hil.sh`.

---

## Files on this branch

| Path | Purpose |
|---|---|
| `src/sim_camera_bridge/` | C++ ROS2 package — SHM filler replacing `ovcam_producer` in HIL. Includes `ovcam_shm.hpp` from the producer package to guarantee binary compatibility. |
| `src/sim_camera_bridge/launch/hil_simulation.launch.py` | ROS2 launch file for the in-Docker node group. |
| `config/hil/bench_fsm.yaml` | Shared FSM, filter, and camera geometry parameters (applied to both TS1 `visp_servo` and TS2 `hil_servo` — same machine gives a fair A/B). |
| `config/hil/bench_proportional.yaml` | TS2 controller gains (`hil_servo` proportional law — `k_fwd`, `k_lat`, `k_vz`). Pass after `bench_fsm.yaml`. |
| `config/hil/bench_ibvs.yaml` | TS1 IBVS gains (`visp_servo` — for benchmarking only, not default HIL). |
| `config/hil/bench_oracle.yaml` | Oracle controller params (ground-truth pose feedback). |
| `config/hil/bench_h_vs.yaml` | Hybrid visual servo params. |
| `config/hil/hil_servo_params.yaml` | Legacy ViSP param file (`visp_servo_node`-scoped) — superseded by `bench_fsm.yaml` + `bench_proportional.yaml` for the default stack. |
| `config/hil/cyclonedds_hil.xml` | CycloneDDS profile (default) — `${MATLAB_HOST_IP}` and `${PI_INTERFACE}` substituted at launch via `envsubst`. |
| `config/hil/fastrtps_hil.xml` | FastDDS profile (alternate) — dual-transport (`eth0` + loopback). |
| `config/hil/fastrtps_matlab.xml` | FastDDS profile template for the MATLAB Windows side — edit with real IPs before use. |
| `config/hil/bag_qos_overrides.yaml` | QoS overrides for `ros2 bag play` replay. |
| `run_stack_hil.sh` | Bash wrapper — Docker + DDS env + `ros2 launch` + native `yolo_producer` + OV2SLAM watchdog. |
| `SIM_HIL.md` | This file. |
| `matlab/hil_ros_init_LT.m` | MATLAB session init — reads `RMW_IMPLEMENTATION` from env, creates `ros2node`, all publishers/subscribers, 20 Hz state timer. Run once per session. |
| `matlab/sim_camera_publisher_timer_LT.m` | Rate-configurable timer publisher — reads `latest_frame`, RGB→BGR, sparse checksum dedup, sends `sensor_msgs/Image` to `/sim/camera/image_raw`. |
| `matlab/live_camera_view_LT.m` | Optional 5 Hz live viewer — shows `latest_frame` in a single figure panel. No network traffic. |
| `matlab/read_cmdvel_live_interp.m` | MATLAB Function block in `hil_closed_loop.slx` — reads `sim_cmdvel` (version-cached), applies safety clamps `[±2, ±2, ±1, ±1, ±1]` m/s, writes pose back for state publisher. |
| `matlab/hil_closed_loop.slx` | Simulink model — Unreal Engine drone dynamics, `Simulation 3D Camera` block (640×480 RGB), `read_cmdvel_live_interp` MATLAB Function block. |
| `camera_calib/hil_sim_ov2slam.yaml` | OV2SLAM calibration YAML for the sim camera (fx=fy=554 px, cx=320, cy=240). |

## Files NOT modified

`git diff main..sim-hil --stat` should show only additions.
`yolo_producer`, `yolo_bridge`, `ovcam_bridge`, `ov2slam`, and `run_stack.sh` are untouched.
