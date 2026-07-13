# Onboard Vision Framework — Detection · SLAM · Visual Servoing

A modular ROS 2 framework for evaluating onboard robot vision pipelines **end to end** on edge
hardware. The detector, the SLAM backend, and the controller can each be swapped with a single
argument, and the framework measures what changes across the *whole* pipeline — per-stage latency,
control-loop rate, and closed-loop mission outcome — rather than benchmarking each component in
isolation.

> Raspberry Pi 5 · Hailo-10H NPU · OV5647 CSI camera · ROS 2 Jazzy · OV²SLAM · ViSP

This is the research artifact for *"Onboard Vision for Small Robots: Detection, SLAM, and Visual
Servoing"* (IEEE Access, 2026). The dataset and the scripts that regenerate every table in the paper
are in [`benchmarks/paper_data/`](benchmarks/paper_data/).

## Architecture

Hardware-touching processes (libcamera, Hailo NPU) run **natively on the host**. Everything ROS 2
runs **in Docker**. They communicate through POSIX shared memory, not DDS — a seqlock ring buffer
moves 921 KB frames ~235× faster than DDS at the median.

```
┌──────────────────── HOST (Raspberry Pi 5) ─────────────────────────┐
│  Core 0 — ovcam_producer   libcamera → /ovcam_frames (POSIX shm)   │
│  Core 1 — yolo_producer    /ovcam_frames → Hailo NPU → /yolo_shm   │
│                                                                     │
│  ┌────────────────── DOCKER (ROS 2 Jazzy) ──────────────────────┐  │
│  │  Core 0,1 — ovcam_bridge   /ovcam_frames → /ovcam/image_raw  │  │
│  │  Core 0,1 — yolo_bridge    /yolo_shm     → /yolo/detections  │  │
│  │  Core 2,3 — ov2slam        /ovcam/image_raw → /vo_pose       │  │
│  │                                             → /point_cloud    │  │
│  │  Core 0,1 — visp_servo     detections + pose → /cmd_vel       │  │
│  └───────────────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────────────┘
```

The detector stage is pluggable: YOLOv8/v11/YOLO26 (n/s/m) on the NPU or on the CPU via ONNX
Runtime, OpenCV MOG2 background subtraction, or OWL-ViT. Selected at launch with
`--detector`/`--model`/`--backend`; see [`models/model_registry.json`](models/model_registry.json).

## Requirements

| | |
|---|---|
| Board | Raspberry Pi 5 (Cortex-A76 ×4, 16 GB) |
| Host OS | **Raspberry Pi OS Trixie (64-bit)** — required; the Hailo-10H drivers and libcamera integration depend on it |
| Accelerator | Hailo-10H AI HAT+ 2, passed through as `/dev/hailo0` |
| Camera | OV5647 CSI @ 640×480, 30 fps |
| Docker | Engine ≥ 24 |

```bash
sudo apt install -y libcamera-dev cmake g++ pkg-config          # host-side
pip3 install hailort-*.whl --break-system-packages              # from the Hailo Developer Zone
bash models/fetch_models.sh                                     # models are not committed
```

## Quick start

Build once, then bring the stack up:

```bash
docker build -t ros2_perception_stack .
./start_container.sh

sudo docker exec ros2_perception_stack bash -lc "
  source /opt/ros/jazzy/setup.bash && cd /workspace
  colcon build --packages-select yolo_msgs ovcam_bridge yolo_bridge visp_servo --symlink-install"

# OV²SLAM (first time only — builds vendored Ceres/Sophus/obindex2 too)
sudo docker exec ros2_perception_stack bash -lc "
  cd /workspace/src/ov2slam_ros/Thirdparty && ./build_thirdparty.sh
  cd /workspace && source /opt/ros/jazzy/setup.bash
  colcon build --packages-select ov2slam --symlink-install"

./run_stack.sh                    # live camera, all six processes, correctly pinned
./run_stack.sh stop
```

`run_stack.sh` is the supported path — it starts each process on its designated core in the right
order and cleans up the shared-memory segments on exit. Each package's README documents how to run
it standalone if you need to.

For the hardware-in-the-loop rig (MATLAB/Simulink drives a simulated drone; the Pi closes the loop):

```bash
./run_stack_hil.sh --model yolov8n --backend npu --detector yolo
```

See [`matlab/`](matlab/) for the sim-host side and [`benchmarks/matlab/README.md`](benchmarks/matlab/README.md)
for the run protocol.

## Repository map

| Path | Contents |
|---|---|
| [`src/`](src/) | The nine ROS 2 packages. Each has its own README. |
| [`benchmarks/`](benchmarks/) | Experiment harnesses, latency probes, and the paper dataset — see [`benchmarks/README.md`](benchmarks/README.md) |
| [`benchmarks/paper_data/`](benchmarks/paper_data/) | Every number in the paper, plus the data and scripts to regenerate them |
| [`matlab/`](matlab/) | Simulink HIL sim-host runbook |
| [`models/`](models/) | `model_registry.json` and `fetch_models.sh`; binaries are fetched, not committed |
| [`config/hil/`](config/hil/) | DDS profiles and servo params for the HIL rig |
| [`camera_calib/`](camera_calib/) | OV5647 intrinsics (calibrated at 640×480, RMS 0.97 px) |
| [`docs/`](docs/) | [Tuning](docs/TUNING.md) · [Troubleshooting](docs/TROUBLESHOOTING.md) · [EuRoC replay](docs/EUROC.md) · [Calibration](docs/CALIBRATION.md) · [SLAM bug-fix log](docs/BUGFIXES_2026-03.md) |

## Key results

Measured on the HIL rig, 20 detector configurations × 3 runs. Full numbers and provenance in
[`benchmarks/paper_data/README.md`](benchmarks/paper_data/README.md).

- **The NPU is 18–51× faster than the CPU on the same model and weights** (15.6 ms vs 288 ms for
  YOLOv8n), and NPU inference is near-flat across model size (15.6–39.5 ms) while CPU explodes
  (226–2120 ms).
- **`/cmd_vel` tracks the detection rate 1:1** — the controller emits one command per detection, so
  control authority is throttled directly by detector throughput.
- **Reach time is actuation-limited, not detection-limited.** In simulation time every detector that
  closes the loop arrives in ~40–42 s, across a 12× spread in detection rate. What matters is
  whether the loop closes at all, and there is a hard viability cliff near **1 Hz**.
- **A detector can be fast and still be useless.** MOG2 is the cheapest thing measured (14.9 ms) and
  cannot servo at all — it tracks motion blobs, not the target.

## License

**GPL-3.0** — see [`LICENSE`](LICENSE). This is forced by the vendored OV²SLAM, obindex2 and
ibow_lcd, which are GPL-3.0; a permissive license would not be valid for the combined work. Every
third-party component and its terms are enumerated in
[`THIRD_PARTY_LICENSES.md`](THIRD_PARTY_LICENSES.md). Note that the YOLO model weights derive from
Ultralytics and are **AGPL-3.0**; they are fetched rather than committed.

## Citation

See [`CITATION.cff`](CITATION.cff).

## Contributors

| Name | Contribution |
|---|---|
| Ahmed Dhaouadi | Detector framework (NPU/CPU/plug-in backends), ViSP visual servoing, HIL automation, latency and mission benchmarking |
| Amar Aly | OV²SLAM integration and tuning, ROS 2 environment, camera calibration |
| Mohamed Hassan | Supervision |
