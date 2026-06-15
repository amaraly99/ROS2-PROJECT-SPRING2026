# OV2SLAM Benchmark Investigation Report
**Date:** 2026-06-12  
**Platform:** Raspberry Pi 5, ROS2 Jazzy, Docker (ros2_perception_stack), cyclonedds

---

## Problem

OV2SLAM was throwing a large number of images and exiting far earlier than expected during benchmarking. Stereo accurate mode on MH_01_easy was processing only ~321 of 3682 frames, producing RMSE of ~0.082 m (expected: ~0.04 m). After a Pi reboot, the issue became catastrophically worse — even manual tmux runs reproduced the failure.

---

## Root Cause

**`net.core.rmem_max` too small for cyclonedds UDP image transport.**

| Parameter | Value |
|---|---|
| Default `net.core.rmem_max` | ~208 KB |
| Single EuRoC camera frame (752×480 mono8) | ~361 KB |

cyclonedds (the RMW used in the container) routes all topics — including camera images — over UDP sockets, even on loopback. When a single frame exceeds the socket receive buffer, the kernel silently drops it.

Dropped frames cause stereo pair desynchronisation. OV2SLAM's stereo guard (`|t_left − t_right| > 0.015 s`) rejects the unpaired frame, logging "Throw img -- Sync error". With enough dropped frames:
- OV2SLAM's auto-exit heuristic (`now - last_img_time > 100 * cam_delay`) trips prematurely.
- The node self-terminates before the sequence ends.

`net.core.rmem_max` resets to the kernel default on every reboot — this is why the problem reappeared after the Pi restart despite having worked before.

**The original benchmarker masked this** because the v1 `Ov2slamCrashWatchdog` classified OV2SLAM's early self-exit as a "crash" and reported a generic failure, making it look like a software bug rather than a transport issue.

---

## Controlled Experiment

Three conditions tested on MH_01_easy, 60-second window:

| Condition | Frames received | Images thrown |
|---|---|---|
| cyclonedds + rmem_max = 208 KB (default) | 776 | 268 |
| fastrtps + rmem_max = 208 KB | 1168 | 0 |
| cyclonedds + rmem_max = 16 MB (fix) | 1173 | 0 |

fastrtps uses shared memory on loopback (immune to socket buffer limits). cyclonedds with a large buffer matches fastrtps performance exactly. This confirmed the buffer size — not the RMW implementation itself — was the bottleneck.

---

## Fix Applied

### 1. Raise socket buffer (immediate, host)
```bash
sudo sysctl -w net.core.rmem_max=16777216
sudo sysctl -w net.core.rmem_default=16777216
```

### 2. Persist across reboots
Created `/etc/sysctl.d/60-cyclonedds-ros2.conf` on the Pi host:
```
net.core.rmem_max = 16777216
net.core.rmem_default = 16777216
```

### 3. Re-apply on every container start
Added to `start_container.sh` (before `docker run`):
```bash
log_step "Raising net.core.rmem_max for cyclonedds image streams (16 MB)..."
sudo sysctl -w net.core.rmem_max=16777216 >/dev/null
sudo sysctl -w net.core.rmem_default=16777216 >/dev/null
```

### 4. Startup check in benchmarker
`OV2SLAM_benchmarker2.py` now calls `check_socket_buffer()` at startup — warns loudly and prints the fix command if `rmem_max < 2 MB`.

---

## Benchmarker Rewrite (OV2SLAM_benchmarker2.py)

The original benchmarker had several architectural problems that compounded the transport issue:

- False crash detection (`Ov2slamCrashWatchdog` treated normal self-exit as crash)
- PTY pump consuming ~400 lines/s of subprocess stdout on a 4-core SBC
- `publish_final_clock` orphan subprocess that blocked for 13+ minutes
- `use_sim_time:=true` making OV2SLAM's node clock jittery → unreliable `cam_delay`
- CPU monitor sampling at 0.05 s (20 Hz) on a Cortex-A76

The rewrite (`OV2SLAM_benchmarker2.py`) addresses all of these:
- Success criterion: valid trajectory produced (not watchdog heuristic)
- `BAG_RATE = 1.0` hardcoded
- No `use_sim_time`
- Plain file-redirect subprocess log (no PTY pump)
- `publish_final_clock` removed entirely
- CPU monitor samples at 2.0 s by default
- Completeness gate: fails a run if trajectory ends >8 s before sequence end
- `--all-modes` flag for accurate+fast × stereo+mono in one invocation

---

## Validation

End-to-end run of accurate-stereo on MH_01_easy with fix applied:

```
Mode:     accurate-stereo
Sequence: MH_01_easy
Result:   SUCCESS
RMSE:     0.0428 m   (reference: ~0.04 m)
Frames:   3682 / 3682  (full trajectory)
Throws:   0
Loops:    4
EXIT:     0
```

---

## Remaining

The other three modes (accurate-mono, fast-stereo, fast-mono) have not yet been run through benchmarker2. With the socket buffer fix in place, they are expected to behave correctly.
