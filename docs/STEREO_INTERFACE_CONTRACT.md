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
| `/sim/camera/left/image_raw` | `sensor_msgs/msg/Image` | The **existing** mono camera, renamed. All prior mono results remain the left-eye baseline. |
| `/sim/camera/right/image_raw` | `sensor_msgs/msg/Image` | New. |

Renaming rather than adding a third topic keeps mono and stereo runs comparable: the left eye
is bit-identical to what every published mono result already used.

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

Both eyes are the same ideal pinhole, confirmed from the Simulink 3D Camera block's Parameters
tab:

```
Focal length  : [554, 554] px      (HFOV 60deg -> fx = 320 / tan(30deg) ~= 554)
Optical center: [320, 240]
Image size    : [480, 640]
Distortion    : radial [0,0], tangential [0,0]
```

`bdo_stereo_rect: 0` — two ideal pinholes with pure lateral offset and no relative rotation are
**already rectified** (R = I, D = 0). Rectifying again would only add interpolation error.

> The mono config carried EuRoC values in its right-camera block (fxr 457.587, non-zero
> distortion, a rotated `body_T_cam1`). Those were inert in mono mode but are actively wrong in
> stereo. The stereo config replaces them; do not copy them back.

---

## Pi-side pipeline (for reference)

```
MATLAB  /sim/camera/left/image_raw  -> sim_camera_bridge(left)  -> /ovcam_frames_left  -> ovcam_bridge(left)  -> /ovcam/left/image_raw  --+
                                                                                                                                          +-> OV2SLAM (stereo)
MATLAB  /sim/camera/right/image_raw -> sim_camera_bridge(right) -> /ovcam_frames_right -> ovcam_bridge(right) -> /ovcam/right/image_raw --+

                                        /ovcam_frames_left is ALSO consumed by yolo_producer (detector stays mono)
```

Stereo runs **two instances of each existing node** — no new node type is needed. Parameter
status, verified against the source:

| Node | Has today | Still to add |
|---|---|---|
| `sim_camera_bridge` | `input_topic`, `shm_name`, `sem_name`, `width`, `height`, `slots` (`sim_camera_bridge_node.cpp:35-40`) | `use_source_stamp` |
| `ovcam_bridge` | `shm_name`, `sem_name`, `frame_id` (`ovcam_bridge_node.cpp:34-36`) | **`output_topic`** — currently absent, the publish topic is fixed |

Until `output_topic` lands, two `ovcam_bridge` instances would both publish to the same topic.
The MATLAB side can be built and verified independently in the meantime.

| Instance | shm_name | sem_name |
|---|---|---|
| left | `/ovcam_frames_left` | `/ovcam_left_ready` |
| right | `/ovcam_frames_right` | `/ovcam_right_ready` |

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

1. `ros2 topic hz /ovcam/left/image_raw` and `/ovcam/right/image_raw` — both at the expected rate.
2. Left and right `header.stamp` **match per pair** (not merely close).
3. OV2SLAM reaches stereo initialisation and publishes `/vo_pose`.
4. **Re-run one mono config and confirm it is unchanged** — proof the `_pad` change and the
   topic rename broke nothing.
