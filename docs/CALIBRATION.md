# Camera calibration

The OV5647 was calibrated at 640×480 with a checkerboard. Result lives in
[`camera_calib/ov5647_ov2slam.yaml`](../camera_calib/ov5647_ov2slam.yaml) and is read by both
OV²SLAM and `visp_servo` (the servo needs `fx, fy, cx, cy` to build the interaction matrix).

| Parameter | Value |
|---|---|
| Resolution | 640 × 480 |
| fx | 543.098 |
| fy | 539.702 |
| cx | 310.687 |
| cy | 222.928 |
| k1 | −0.00984 |
| k2 | +0.07654 |
| p1 | −0.00247 |
| p2 | +0.00278 |
| RMS reprojection error | 0.974 px |

## Recalibrating

Necessary if the lens or the capture resolution changes — the intrinsics do **not** scale trivially
with resolution, and a wrong `cx`/`cy` biases the servo's centering error directly.

```bash
cd camera_calib
python3 capture.py       # ~30 checkerboard shots at varied angles and distances
python3 calibrate.py     # prints the intrinsics + RMS error
```

Then update `ov5647_ov2slam.yaml`. An RMS much above ~1 px usually means the checkerboard shots were
too similar to each other — vary the angle and distance more.
