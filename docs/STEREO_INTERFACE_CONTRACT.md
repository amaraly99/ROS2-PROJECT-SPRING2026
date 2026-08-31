# Stereo HIL — interface contract

**Status:** Pi-side spec complete; Pi-side code not yet written; MATLAB side not yet built.
Verified against the source 2026-08-24 — encoding and bridge-parameter claims corrected.
**Owner:** Ahmed (Simulink) · **Consumer:** OV2SLAM and any other stereo SLAM node.
**Baseline:** 0.11 m (EuRoC-matched), configurable — see [Baseline](#baseline-one-setting-two-files).

This is the complete list of what the MATLAB/Simulink/Unreal side must publish for stereo
SLAM to work on the Pi. Everything on the Pi side is specified against this contract.

## Why stereo at all

Most SLAM systems assume stereo; the rig is mono-only today, which the advisor flagged as a
limitation. Stereo also removes the monocular scale ambiguity. It changes **only the SLAM
axis** — see [What stereo does not change](#what-stereo-does-not-change).

---

## 1. Topics

| Topic | Type | Notes |
|---|---|---|
| `/sim/camera/image_raw` | `sensor_msgs/msg/Image` | The **existing** mono camera. **NOT renamed.** This is the left eye. |
| `/sim/camera/right/image_raw` | `sensor_msgs/msg/Image` | New — the only addition. |

> **Corrected 2026-08-25.** This document previously specified renaming the left topic to
> `/sim/camera/left/image_raw` for symmetry. That was wrong and would have broken the running
> system. `/sim/camera/image_raw` is hardcoded in `visp_servo_node.cpp:398` and appears in
> `run_stack_hil.sh` (3x), `scripts/record_bag.sh:82`, `config/hil/bag_qos_overrides.yaml:26`
> and `hil_simulation.launch.py:71` — and **every existing rosbag replays against it**, so a
> rename would have silently invalidated the recorded-bag workflow too.
>
> Adding only the right eye is also the better experiment: the left eye stays *bit-identical* to
> what produced every published mono result, so mono-vs-stereo differs in exactly one variable.

## 2. Encoding and geometry

| Property | Required value | Why |
|---|---|---|
| `encoding` | **`bgr8`** | What the mono path already declares (`hil_ros_init_LT.m:41`) and packs (`sim_camera_publisher_timer_LT.m:115-117`). `sim_camera_bridge` calls `cv_bridge::toCvCopy(msg, "rgb8")`, which *converts from the declared encoding* — so `bgr8` in, RGB out, correctly. **Declare what you actually pack.** |
| Width | **640** (must be a multiple of 32) | The bridge throws at startup otherwise — it mirrors `ovcam_producer`'s `(W + 31) & ~31` stride rule. |
| Height | **480** | Matches the ring and every existing result. |
| Both eyes | **Identical** resolution, FOV, orientation **and channel order** | The pair must be rectified by construction. Mismatched channel order between eyes degrades stereo matching silently — no error, just worse depth. |

## 3. QoS — must match exactly

```
Reliability : BEST_EFFORT
History     : KEEP_LAST, depth 5
Durability  : VOLATILE
```

`sim_camera_bridge` subscribes BEST_EFFORT. A RELIABLE publisher will **fail the QoS handshake
and deliver nothing** — it does not silently degrade. RELIABLE also causes backpressure and
rate collapse on 921 KB frames, which is why the mono path chose BEST_EFFORT originally.

### Bandwidth — stereo doubles it

Mono at 20 Hz is 18.4 MB/s (~150 Mbit/s). **Stereo is ~37 MB/s (~295 Mbit/s).** The publisher
already carries an idle-throttle guard because sustained throughput with no consumer once drove
MATLAB's ROS2 layer to out-of-memory (`sim_camera_publisher_timer_LT.m:56-64`); doubling the load
makes that failure mode more likely. **Stereo HIL requires wired Ethernet** — Wi-Fi cannot carry
it. Dropping to `PUBLISH_HZ = 15` for stereo is an acceptable, reportable constraint if needed.

## 4. Timestamps — the hard requirement

**Both images of a pair must carry the identical `header.stamp`, taken from the simulation
clock at render time.**

This is the single most likely thing to get wrong, and it fails silently. Stereo SLAM matches
left to right by timestamp; if the two eyes carry different stamps, the matcher pairs frames
that do not correspond and depth is garbage — with no error message.

Do **not** stamp each publisher independently with "now". Render both eyes in the same
simulation timestep, take one timestamp, and apply it to both messages.

**This is largely already solved on the MATLAB side.** The camera is not published by a Simulink
ROS block — it is published by a **MATLAB timer** (`sim_camera_publisher_timer_LT.m`) that reads
the rendered frame from the base-workspace variable `latest_frame`, and it already stamps from
`sim_heartbeat`, the *simulation* clock, not wall-clock (`sim_camera_publisher_timer_LT.m:135-141`).
Publishing both eyes inside one timer tick from a **single** `sim_heartbeat` read gives them a
shared stamp by construction. One timer, one clock read, two sends — never a second timer.

### Why the Pi side needed changing for this

The SHM ring originally **discarded** the source timestamp: `sim_camera_bridge` stamped each
slot with the Pi's `CLOCK_MONOTONIC` *at write time*, and `ovcam_bridge` republished that as
`header.stamp`. Two independent bridge instances would therefore have stamped left and right
with **their own arrival times**, and stereo would have synchronised on network jitter rather
than capture time.

The fix carries the source stamp through the ring in the 12 spare bytes of `SlotHeader._pad`
(`uint64_t t_src_ns`). This is ABI-compatible: `sizeof(SlotHeader)` stays 64 bytes and existing
consumers ignore the field, so the mono path and every published result stay valid.

## 5. Baseline — one setting, two files

The baseline appears in exactly two places and **they must always match**:

| Where | What it means |
|---|---|
| Simulink camera block offset | Where the cameras physically **are** |
| `body_T_cam1` in `camera_calib/hil_sim_ov2slam_stereo.yaml` | Where SLAM **believes** they are |

If they disagree, SLAM's scale is silently wrong — every distance comes out proportionally off,
with no error. Change both together.

Current value **0.11 m**, matching EuRoC so the in-loop geometry is consistent with the existing
standalone EuRoC SLAM tables. Both are plain parameters; no rebuild is needed to change it.

Right camera = left camera offset **+0.11 m along the body lateral axis**, identical
orientation, no rotation.

## 6. Calibration

Already written: `camera_calib/hil_sim_ov2slam_stereo.yaml`.

Both eyes are the same ideal pinhole, verified against the running Simulink model:

```
Focal length  : [1200, 1200] px    (HFOV 29.9deg = 2*atan(320/1200))
Optical center: [320, 240]
Image size    : [480, 640]
Distortion    : radial [0,0], tangential [0,0]
```

> **Focal length corrected 2026-08-25 — this said 554, and 554 was wrong by 2.17x.**
> 554 was never measured; it was derived from an *assumption* that the block was 60deg HFOV
> (320/tan(30deg)). The block was never opened to check. It renders at fx = 1200 — a narrow
> ~30deg lens. Verified two independent ways: a warp test (correlation 0.94 warped vs 0.41 raw)
> and stereo triangulation against simulator ground-truth depth (fx = 1236, within 3% of 1200).
> Use **1200**, the block's actual setting — not 1236, which is a noisy measurement of it.
>
> **Do not "fix" the Simulink block to match the old value.** The block produced every published
> result and is the ground truth here; the config files were the description, and the description
> was wrong. Correct the descriptions.
>
> Detector results are unaffected — a detector consumes pixels and never sees focal length — as
> is the EuRoC SLAM benchmark, which uses a real dataset with its own calibration. What is
> affected: HIL SLAM runs, where wrong intrinsics distort reconstructed trajectory shape and
> inflate ATE in a way Umeyama alignment cannot remove, and any stated metric stand-off.

`bdo_stereo_rect: 0` — two ideal pinholes with pure lateral offset and no relative rotation are
**already rectified** (R = I, D = 0). Rectifying again would only add interpolation error.

> The mono config carried EuRoC values in its right-camera block (fxr 457.587, non-zero
> distortion, a rotated `body_T_cam1`). Those were inert in mono mode but are actively wrong in
> stereo. The stereo config replaces them; do not copy them back.

---

## Pi-side pipeline (for reference)

```
MATLAB  /sim/camera/image_raw        -> sim_camera_bridge(left)  -> /ovcam_frames       -> ovcam_bridge(left)  -> /ovcam/image_raw        --+
   (unchanged — all defaults)                                                                                                              +-> OV2SLAM (stereo)
MATLAB  /sim/camera/right/image_raw  -> sim_camera_bridge(right) -> /ovcam_frames_right -> ovcam_bridge(right) -> /ovcam/right/image_raw  --+
   (the only new chain)

                                        /ovcam_frames is ALSO consumed by yolo_producer (detector stays mono, left eye)
```

**The entire left chain keeps its current defaults — no parameters set, nothing renamed.** Only
the right chain is new and parameterised. Besides being the smaller diff, this keeps
`benchmarks/e2e_latency_probe.py` working untouched in stereo mode: it subscribes to
`/ovcam/image_raw` (`e2e_latency_probe.py:130`) and matches detector output against that topic's
`header.stamp`. A rename would have made it match nothing and silently report
`t_cam_stamp_ns = 0` rather than fail. Because it keeps working, the mono-vs-stereo detector
latency delta comes straight out of the stereo SLAM run with the same tool and no
reconfiguration — see [What stereo does not change](#what-stereo-does-not-change).

Stereo runs **two instances of each existing node** — no new node type is needed. Parameter
status, verified against the source:

| Node | Has today | Still to add |
|---|---|---|
| `sim_camera_bridge` | `input_topic`, `shm_name`, `sem_name`, `width`, `height`, `slots` (`sim_camera_bridge_node.cpp:35-40`) | `use_source_stamp` |
| `ovcam_bridge` | `shm_name`, `sem_name`, `frame_id` (`ovcam_bridge_node.cpp:34-36`) | **`output_topic`** — currently absent, the publish topic is fixed |

Until `output_topic` lands, two `ovcam_bridge` instances would both publish to the same topic.
The MATLAB side can be built and verified independently in the meantime.

The two `sim_camera_bridge` instances **must** get distinct `shm_name` *and* `sem_name`, or they
write into the same shared-memory ring and the two eyes overwrite each other's slots:

| Instance | input_topic | shm_name | sem_name | ovcam_bridge output_topic |
|---|---|---|---|---|
| left | `/sim/camera/image_raw` *(default)* | `/ovcam_frames` *(default)* | `/ovcam_ready` *(default)* | `/ovcam/image_raw` *(default)* |
| right | `/sim/camera/right/image_raw` | `/ovcam_frames_right` | `/ovcam_ready_right` | `/ovcam/right/image_raw` |

Left is entirely defaults; every value that has to be set is on the right instance.

## What stereo does not change

- **The detector.** It consumes one image — the left eye — so detections are **identical by
  construction**. The detector sweep is not repeated. Left-eye-only is also the recognised
  convention: KITTI evaluates 2D detection on the left colour image, and Stereo R-CNN reports
  left AP2D consistently higher than right or joint-stereo AP2D. The stereo-detection
  literature targets *3D* boxes, not better 2D ones.
- **The controller.** IBVS servos on a 2D bbox and uses bbox size as its depth proxy; it has no
  input for metric depth. Consuming stereo depth would require a different control law
  (Controller C, deferred).

Detector **latency** may still change, because a second stream adds a second bridge, a second
NV12 conversion, double the inbound DDS traffic and a heavier SLAM front-end, all on the same
four cores. That cost is measured for free from the stereo SLAM sweep's own detector-rate
telemetry rather than by re-running the sweep.

## Acceptance test

Before trusting any stereo result:

1. `ros2 topic hz /ovcam/image_raw` and `/ovcam/right/image_raw` — both at the expected rate.
2. Left and right `header.stamp` **match per pair** (not merely close).
3. OV2SLAM reaches stereo initialisation and publishes `/vo_pose`.
4. **Re-run one mono config and confirm it is unchanged** — proof the `_pad` change and the
   topic rename broke nothing.
