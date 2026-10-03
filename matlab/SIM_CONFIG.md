# Windows-side simulator configuration (Ahmed's sim host)

Pinned 2026-10-03 from Ahmed's MATLAB HIL folder on the Windows sim host. Every value
below was read from the .slx XML or the .m/.xml files, not from a MATLAB session.

## Model: matlab/hil_closed_loop.slx

- sha256 946390cf28e80c2078835b513d97a3ab334ef19b1cf3afd4a7f9678a3706dd66
- Saved in R2026a Update 4 (26.1.0.3312084), last saved 2026-09-27 04:42 UTC
- Solver ode1, fixed step 1/20 s, Accelerator mode, pacing on, stop time 500 s
- Scene: Large parking lot, scene sample time 1/20 s

## Cameras

ISO 8855 mount on Quadrotor1, 640x480, no distortion, skew 0, sample time inherited.

| Block | State | fx, fy | cx, cy | HFOV (deg) | Translation (m) | Rotation (deg) |
|---|---|---|---|---|---|---|
| Simulation 3D Camera | active | 1200, 1200 | 320, 240 | 29.9 | [0.2, 0, 0.2] | [0, 5, 0] |
| Simulation 3D Camera Right | commented out | 1200, 1200 | 320, 240 | 29.9 | [0.2, -0.11, 0.2] | [0, 5, 0] |

fx is a property of each model file. Other models in the repo use other values (554 in
Amar's stereo models) and are not described here.

- Left image: `/sim/camera/image_raw`, encoding `bgr8`, sent by `hil_publish_frame.m`.
- Right image: no publisher in this commit. `set_stereo(1)` alone does not publish it.

The camera publisher timer is created by `hil_pub_control('start', hz)`, with period
`1 / hz` (`hil_pub_control.m:40`). It is started either by
`sim_camera_publisher_timer_LT.m:35`, `hil_pub_control('start', PUBLISH_HZ);`, or by the
supervisor's `pub_start` command at `hil_run_supervisor.m:261`,
`hil_pub_control('start', hz);`, where `hz` comes from the command argument
(`hil_run_supervisor.m:254`, `hz = str2double(strtrim(cmd(10:end)));`).

## Initial conditions

x -10, y 5, z 10 m, yaw 2*pi rad, pitch 0 (integrator initial conditions in the model).

The copies of this model on exp/detector-accuracy and feat/stereo-hil, and the comment at
`hil_run_supervisor.m:31`, give (-15, 10, 10). Those are the earlier 2026-09-23 pilot values.

## Target pose

Published on `/sim/target_pose`: [35.5, 23.7, 3.2, pi] (`hil_ros_init_LT.m:123`).

This constant is not the position of the stop sign the detectors see. The Pi-side scorer
uses a fitted sign position of about (35.1, 2.92, 3.08), 20.8 m away in y. Do not use the
published constant as the sign position.

## ROS 2 and network

- RMW `rmw_fastrtps_cpp` (`hil_ros_init_LT.m:19`), `ROS_DOMAIN_ID` 0 (`hil_ros_init_LT.m:20`).
- Fast DDS profile used on this host (not committed): UDPv4 transport, 8 MB send and listen
  socket buffers, built-in transports off.
  sha256 073112d192180febefa87119817248fdf77dd3cd37db8d064c78551495613724.
  Whether a given session loads this profile is not recorded here.
- Supervisor TCP port 55556 (`hil_run_supervisor.m:81`).

## Supervisor

The header of `hil_run_supervisor.m` still says "v3 (PROPOSED 2026-09-23, NOT YET APPLIED)"
while the code implements v3 (`start_hold`, `resume`, pose in `status`). The Pi-side caller
of `start_hold` is not in this commit.

## File hashes (sha256)

```
946390cf28e80c2078835b513d97a3ab334ef19b1cf3afd4a7f9678a3706dd66  matlab/hil_closed_loop.slx
c02d8ce547e889c261079bfd8cf77e05faf20a5796f7faa6aa2503b4e1c55317  matlab/hil_ros_init_LT.m
edec97966fe4a9e12cb3498653998714aae5c3fdb74be76ed978ca1a369c5503  matlab/hil_run_supervisor.m
e82bbc8fb3868a8ec149a34cc063b826cbc09131c41547e3c96d4ce5d0188102  matlab/sim_camera_publisher_timer_LT.m
4b54aacf20b3b0b17d56e38e0be1896e5cc4cafd451f6170f90c86bc0f29a989  matlab/hil_pub_control.m
f9bebda2b8f4f07445c2b909829acb6222b88ad3b8508f17ff32c1dde99f1632  matlab/hil_publish_frame.m
68ca8fd1f9514e6fc8156784102801851f503d75aca5e44cf572a5919c407a78  matlab/set_stereo.m
239f0acf1db58b3d699177d21df43a2bd8caa85be9cd242acc42dba821603434  matlab/live_camera_view_stereo_LT.m
4d4496b2493589c652c4446594f30df3d917d7fcddb1ae90291f9c02ac1a35ca  matlab/check_camera_fov.m
f332bead249b789cb9c873539353d2449264ea829e0ed0ce5039caca066cc9e4  matlab/check_stereo_geometry.m
```
