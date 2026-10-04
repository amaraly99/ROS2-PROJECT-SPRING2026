# SIM_PARAMS: shared simulator parameters (2026-10-04)

One Simulink model, run by both rigs. Values marked **[Amar to confirm]** are read from Amar's files
on benchmarks/slam-hil @ 4490660 but cover his SLAM paths, which only he can confirm. Values marked
**[open]** are not decided yet.

## Model

| Item | Value | Source |
|---|---|---|
| Model file | `matlab/hil_closed_loop.slx` from 4490660 (sha256 a2357d70208a15c9075eaf34fe18fa0c22ec8034317a17957bb4a63fd496242b) with the start ICs below applied by `matlab/apply_sim_params.m`; new sha256 **[recorded when saved]** | |
| MATLAB | R2026a Update 4 (26.1.0.3312084) | every model's `metadata/mwcorePropertiesReleaseInfo.xml` |
| Scene | Large parking lot, `/Game/Maps/USCityBlock`, scene Ts 1/20 s | Simulation 3D Scene Configuration |
| Solver | ode1, fixed step 1/20 s, accelerator, pacing on | `configSet0.xml`, `blockdiagram.xml` |
| StopTime | **[open]** `inf` (Amar) or `500` (Ahmed) | |
| Frame convention | ISO 8855 on Quadrotor1; x forward, y left, z up; yaw positive to the left | camera blocks; `bench_fsm.yaml:10-12` |
| cmd_vel frame | world frame: linear.x/y/z integrate straight into world x/y/z, no rotation by yaw | `read_cmdvel_live_interp.m:49`, root wiring |

## Cameras

All cameras: pinhole, no distortion, skew 0, sample time inherited (-1), mounted on Quadrotor1.

| Camera | Block | Active | Resolution | fx, fy | cx, cy | HFOV / VFOV | Mount translation (m, body: fwd, left, up) | Mount rotation (deg) | Rate | Encoding | ROS topic | Consumer |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| Left (mono, and left eye of stereo) | Simulation 3D Camera | always | 640 x 480 | 554, 554 | 320, 240 | 60.0 / 46.8 deg | [0.2, 0, 0.2] | [0, 5, 0] (5 deg down) | 20 Hz | `bgr8` (default) or `mono8` **[Amar to confirm per run]** | `/sim/camera/image_raw` | detector (via sim_camera_bridge), controller (via detections), SLAM (mono, and left eye) |
| Right (stereo only) | Simulation 3D Camera Right | only after `set_stereo(1)`; commented out in every saved file | 640 x 480 | 554, 554 | 320, 240 | 60.0 / 46.8 deg | [0.2, -B, 0.2], B = 0.11 m in this model **[Amar to confirm the baseline set: 0.11, 0.36, 0.42, 0.54, 0.61]** | [0, 5, 0] | 20 Hz, paired with left, same stamp | same as left **[Amar to confirm]** | `/sim/camera/right/image_raw` | SLAM (stereo) |
| Depth (RGB-D only) | Simulation 3D Camera, depth port | only in `hil_closed_loop_baseline_RGBD.slx` | 640 x 480 | 554, 554 (same camera as left) | 320, 240 | 60.0 / 46.8 deg | same as left | same as left | 20 Hz | `16UC1`, millimetres | `/sim/camera/depth/image_raw` | SLAM (RGB-D) **[Amar to confirm]** |

HFOV = 2 atan(320 / 554) = 60.0 deg; VFOV = 2 atan(240 / 554) = 46.8 deg.

## Start pose and target

| Item | Value |
|---|---|
| Target | the stop sign at (35.1, 2.92, 3.08) m (fitted from detector boxes, frozen; DEVIATIONS 17.1) |
| Not the target | `/sim/target_pose` constant (35.5, 23.7, 3.2, pi): a different sign is very likely there (INFERRED, one run) |
| Start, x y z | (5.13, 4.30, 10): 30.0 m horizontal from the target, on the same line as the earlier 45 m start |
| Start, yaw | `2*pi-0.0460` rad (357.36 deg): pointed straight at the target |
| At the start | the target is at image column 320 and about 17 px wide (30 m); the other sign is 5.2 deg outside the 60 deg view |
| Pitch, roll | 0, 0 |
| Start protocol | held start: supervisor `start_hold`, check that the start points within 5 deg of the target, boot the stack, `resume` |
| Standoff | FSM `target_bbox_ratio` 0.55 / `hold_bbox_ratio` 0.50 of image height (`bench_fsm.yaml:39-40`) stop the drone about 2.2 m from the sign at fx 554 (about 4.7 m at fx 1200). **[Amar to confirm: keep, or scale to 0.254 / 0.231 for about 4.7 m]** |

## Transport

| Item | Value |
|---|---|
| RMW | `rmw_fastrtps_cpp` (Fast DDS) **[Amar to confirm: his `hil_ros_init_LT.m:19` takes it from the environment]** |
| ROS_DOMAIN_ID | 0 |
| Fast DDS profile | UDPv4 only, 8 MB send and listen buffers, built-in transports off (Ahmed's host; not committed) **[Amar to confirm his]** |
| Image QoS | best effort, volatile, depth 5 |
| Pi link | Ethernet, `eth0` |
