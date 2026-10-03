# `hil_closed_loop_baseline_061.slx` is NOT a 0.61 m model

**Status (2026-10-03):** this file is byte-identical to `hil_closed_loop_baseline_054.slx`.
Its right-camera offset is `tmountOffset = [0.2, -0.54, 0.2]`, i.e. **0.54 m**.
Do not use it for a 0.61 m run.

## What happened

- The 0.61 m stereo runs (the ALICIA batch, 2026-09-13/14) were flown with the live
  `hil_closed_loop.slx` set to 0.61 m.
- Before the 0.36/0.42/0.54 sweep (2026-09-16/17) that live model was changed, and
  the 0.61 m version was never saved or committed.
- A later file named `_061.slx` was created from the 0.54 m model by mistake.
- Searched and absent (HANDOFF_2026-09-19.md:59-65): every commit of the model on every
  branch, both git stashes, dangling git objects, every `.slx` under the Windows user
  profile, OneDrive, File History. **Not searched:** Windows Previous Versions (needs admin).

## What survived

- **Calibration side of 0.61 m:** five tracked `*.baseline061.bak` files on the Pi
  (`camera_calib/hil_sim_ov2slam_stereo{,_fast}.yaml`, `src/orbslam2/.../stereo/hil_sim.yaml`,
  `src/orbslam3/.../Stereo/HIL_SIM.yaml`, `src/rtabmap_docker/hil_stereo_bridge.py`).
  `python3 scripts/hil_matrix/sync_baseline.py 0.61` produces the same values.
- **Results and bags:** `results/stereo/alicia_baseline_061_wifi_*` and the
  `bags/*baseline061*` runs on the Pi. The data is real; only the exact model file is gone.

## How to rebuild a real 0.61 m model

1. Open any baseline model in the MATLAB GUI (never hand-edit the `.slx` XML).
2. Set the right camera's `tmountOffset` and `mountPoint` y-value to `-0.61`
   (the baseline models differ only in these two fields, HANDOFF_2026-09-19.md:67-70).
3. Save as `hil_closed_loop_baseline_061.slx`, replacing this mislabeled copy, and commit it.
4. Verify: unzip the `.slx` and grep `tmountOffset` for `-0.61`.
5. Before any 0.61 m run, also sync the Pi calibs: `python3 ~/sync_baseline.py 0.61`.

A rebuilt model reproduces the 0.61 m *condition*, not the original file. It is only
equivalent if nothing else in the model changed since 2026-09-14 (not verified).

## Verified baseline models in this directory

| File | Offset found | Status |
|---|---|---|
| `hil_closed_loop_baseline_011.slx` | -0.11 | genuine |
| `hil_closed_loop_baseline_036.slx` | -0.36 | genuine |
| `hil_closed_loop_baseline_042.slx` | -0.42 | genuine |
| `hil_closed_loop_baseline_054.slx` | -0.54 | genuine |
| `hil_closed_loop_baseline_061.slx` | -0.54 | **mislabeled, copy of _054** |
| `hil_closed_loop_baseline_RGBD.slx` | -0.11 | RGB-D model (left camera + depth) |
