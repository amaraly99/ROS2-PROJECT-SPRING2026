# SIM_PARAMS: shared simulator parameters (2026-10-05)

One Simulink model, run by both rigs. This sheet describes the model that is live on the Windows HIL host.
Values marked **[Amar to confirm]** cover his SLAM paths, which only he can confirm. Values marked
**[provisional]** are in use but not final.

## Model

| Item | Value | Source |
|---|---|---|
| Model file | `HIL\hil_closed_loop.slx` on the Windows host, sha256 `a38a664c4600d21ed118e3ece5eaf6c461369153f31e342ad86ca9d7f5b0c3c6` | Windows read-back, 2026-10-04 |
| How it was built | built by the committed default of `matlab/apply_sim_params.m` (no start argument). The chain starts at Amar's `matlab/hil_closed_loop.slx` from 4490660 (sha256 a2357d70208a15c9075eaf34fe18fa0c22ec8034317a17957bb4a63fd496242b); each start in the history below was built from the one before it. The script sets only the camera FocalLength and the four start ICs, and asserts cx, cy and image size | `apply_sim_params.m` |
| Hash note | every .slx save writes a new UUID and timestamp, so a rebuild gives the same parameters but a different sha256. Check parameters, not only the hash | |
| MATLAB | R2026a Update 4 (26.1.0.3312084) | `metadata/mwcorePropertiesReleaseInfo.xml` |
| Solver | ode1, fixed step 1/20 s, accelerator, pacing on | `configSet0.xml`, `blockdiagram.xml` |
| StopTime | `inf` | Windows read-back |
| Frame convention | ISO 8855 on Quadrotor1; x forward, y left, z up; yaw positive to the left | camera blocks; `bench_fsm.yaml:10-12` |
| cmd_vel frame | world frame: linear.x/y/z integrate straight into world x/y/z, no rotation by yaw | `read_cmdvel_live_interp.m:49`, root wiring |

## Scene

| Item | Value | Source |
|---|---|---|
| Scene | Large parking lot (Scene source: Default Scenes). Unreal loads `/Game/Maps/LargeParkingLot` in `AutoVrtlEnv.exe`, Unreal Engine 5.4.4. The block's hidden `ScenePath` (`/Game/Maps/USCityBlock`) is not used | Simulation 3D Scene Configuration; Unreal log |
| Scene sample time | 1/20 s | Simulation 3D Scene Configuration |
| Weather | override off (`EnableWeather` off), so the scene's built-in sky and sun are used. The block's sun, cloud, fog and rain values do not apply | Simulation 3D Scene Configuration |
| Sky over time | clouds move with sim time and are identical across runs at equal sim time | Pi-3 render comparison, 2026-10-05 |
| Unreal lifetime | Unreal relaunches and reloads the map on every sim start, and shuts down on stop | Unreal logs, one map load per run |
| Random seed | none in the model | |

## Cameras

All cameras: pinhole, no distortion, skew 0, sample time inherited (-1), mounted on Quadrotor1.

| Camera | Block | Active | Resolution | fx, fy | cx, cy | HFOV / VFOV | Mount translation (m, body: fwd, left, up) | Mount rotation (deg) | Rate | Encoding | ROS topic | Consumer |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| Left (mono, and left eye of stereo) | Simulation 3D Camera | always | 640 x 480 | 554, 554 | 320, 240 | 60.0 / 46.8 deg | [0.2, 0, 0.2] | [0, 5, 0] (pitch 5 deg down) | 20 Hz | `bgr8` (default) or `mono8` **[Amar to confirm per run]** | `/sim/camera/image_raw` | detector (via sim_camera_bridge), controller (via detections), SLAM (mono, and left eye) |
| Right (stereo only) | Simulation 3D Camera Right | no: commented out in the shared model (mono). Only after `set_stereo(1)` | 640 x 480 | 554, 554 | 320, 240 | 60.0 / 46.8 deg | [0.2, -B, 0.2], B = 0.11 m in this model **[Amar to confirm the baseline set: 0.11, 0.36, 0.42, 0.54, 0.61]** | [0, 5, 0] | 20 Hz, paired with left, same stamp | same as left **[Amar to confirm]** | `/sim/camera/right/image_raw` | SLAM (stereo) |
| Depth (RGB-D only) | Simulation 3D Camera, depth port | only in `hil_closed_loop_baseline_RGBD.slx` | 640 x 480 | 554, 554 (same camera as left) | 320, 240 | 60.0 / 46.8 deg | same as left | same as left | 20 Hz | `16UC1`, millimetres | `/sim/camera/depth/image_raw` | SLAM (RGB-D) **[Amar to confirm]** |

HFOV = 2 atan(320 / 554) = 60.0 deg; VFOV = 2 atan(240 / 554) = 46.8 deg.

## Start pose and target

| Item | Value |
|---|---|
| Target | the frozen stop sign at (35.1, 2.92, 3.08) m (fitted from detector boxes; DEVIATIONS 17.1) |
| `/sim/target_pose` | publishes `[35.1, 2.92, 3.08, pi]` (reliable, transient local), from `matlab/hil_ros_init_LT.m` |
| Readers that treat `/sim/target_pose` as the goal | `src/oracle_detector/oracle_detector/oracle_detector_node.py`, `scripts/hil_matrix/wait_for_arrival.py`, `benchmarks/plot_controller_hil.py`, `benchmarks/compare_controllers.py` |
| Not the target | the old constant (35.5, 23.7, 3.2) is a second sign. From this start it is about 47 deg off-axis, outside the 30 deg half field of view |
| Start, x y z | (15.12, 3.84, 7.5): 20.0 m horizontal from the target, on the same bearing as the earlier 30 m and 45 m starts |
| Start, yaw | `2*pi-0.0460` rad (357.36 deg): pointed straight at the target |
| Pitch, roll | 0, 0 |
| At the start | the target is at image column 320, about 25 x 25 px; the camera looks down at it at about 13.1 deg |
| Start protocol | held start: supervisor `start_hold`, check that the start points within 5 deg of the target, boot the stack, `resume` |
| Standoff **[provisional]** | window 1.6 to 2.8 m from the sign at fx 554 with the shared FSM ratios, `target_bbox_ratio` 0.55 / `hold_bbox_ratio` 0.50 of image height (`bench_fsm.yaml:39-40`). The smoke run reached 2.13 m |

## Start history

| Start | Model sha256 | Result | Backup on the Windows host |
|---|---|---|---|
| 30 m: (5.13, 4.30, 10) | b8ec8250... | failed: the nano detectors do not see the sign at 30 m with fx 554 | `HIL\hil_closed_loop_fx554_30m_b8ec8250.slx` |
| 20 m at z 10: (15.12, 3.84, 10) | d0cdc6e1... | failed the start check: yolov11n on the NPU scored 0.31 against a 0.35 bar; look-down 19.8 deg | `HIL\hil_closed_loop_fx554_20m_z10_d0cdc6e1.slx` |
| 20 m at z 7.5: (15.12, 3.84, 7.5) | a38a664c... | passed | live model |

The older fx 1200 model is kept as `HIL\hil_closed_loop_fx1200_45m_946390cf.slx`.

## Transport

| Item | Value |
|---|---|
| RMW | `rmw_fastrtps_cpp` (Fast DDS) **[Amar to confirm: his `hil_ros_init_LT.m:19` takes it from the environment; the Windows copy sets it]** |
| ROS_DOMAIN_ID | 0 |
| Fast DDS profile | UDPv4 only, 8 MB send and listen buffers, built-in transports off (Ahmed's host; not committed) **[Amar to confirm his]** |
| Image QoS | best effort, volatile, depth 5 |
| Pi link | Ethernet, `eth0` |
| Supervisor | TCP port 55556 on the Windows host: `status`, `start`, `start_hold`, `resume`, `stop`, `pub_start`, `pub_stop` |
