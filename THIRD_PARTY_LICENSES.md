# Third-party licenses

This repository distributes third-party source code. This file records what is included, under
what terms, and why the work as a whole is **GPL-3.0**.

## Effective license of the distributed work: GPL-3.0

OV²SLAM and two of its vendored dependencies (`obindex2`, `ibow_lcd`) are GPL-3.0. Distributing
them together with our own code makes the combined work a GPL-3.0 work, and a permissive top-level
license would not be valid for it. The root [`LICENSE`](LICENSE) is therefore GPL-3.0.

This is a property of *this repository as distributed*, not a judgement about any individual file.
Our own contributions — the detector framework (`src/yolo_producer`, `src/yolo_bridge`,
`src/yolo_msgs`), the visual servoing node (`src/visp_servo`), the camera/SHM bridges
(`src/ovcam_producer`, `src/ovcam_bridge`, `src/sim_camera_bridge`), the benchmark tooling
(`benchmarks/`), and the MATLAB HIL runbook (`matlab/`) — are original work by the authors and are
released under GPL-3.0 as part of this distribution.

Every vendored dependency retains its own upstream license file, unmodified, in its own directory.

## Vendored under `src/ov2slam_ros/`

| Component | License | License file | GPL-3.0 compatible |
|---|---|---|---|
| OV²SLAM (Ferrera et al.) | **GPL-3.0** | `src/ov2slam_ros/license.txt` | — (is the GPL) |
| obindex2 | **GPL-3.0** | `Thirdparty/obindex2/LICENSE` | — (is the GPL) |
| ibow_lcd | **GPL-3.0** | `Thirdparty/ibow_lcd/LICENSE` | — (is the GPL) |
| Ceres Solver | BSD-3-Clause (some components Apache-2.0) | `Thirdparty/ceres-solver/LICENSE` | yes |
| Sophus | MIT | `Thirdparty/Sophus/LICENSE.txt` | yes |
| backward-cpp | MIT | `Thirdparty/backward-cpp/LICENSE.txt` | yes |

Apache-2.0 is one-way compatible with GPL-3.0, and BSD-3-Clause and MIT are permissive; all three
combine into a GPL-3.0 work without conflict.

### `Thirdparty/opengv` — tracked but empty, and not distributed

`src/ov2slam_ros/Thirdparty/opengv` is a **gitlink** (git mode `160000`, commit
`91f4b19c73450833a40e463ad3648aae80b3a7f3`) with **no corresponding `.gitmodules` entry** — a broken
submodule reference inherited from upstream OV²SLAM. The directory is empty on clone.

Consequently **no OpenGV code is distributed by this repository and no OpenGV license obligation
arises.** OpenGV is an optional dependency: `src/ov2slam_ros/CMakeLists.txt` uses
`find_package(opengv QUIET)` and only defines `-DUSE_OPENGV` if it is found, so this build compiles
with the OpenGV code paths disabled. If you install OpenGV yourself, it is BSD-licensed and its
terms are between you and its authors.

## External dependencies (not distributed here)

Obtained separately by the user; listed for completeness because the stack does not run without them.

| Dependency | License | Notes |
|---|---|---|
| ROS 2 Jazzy | Apache-2.0 | |
| OpenCV (+ contrib) | Apache-2.0 | built from source, see `build_opencv.sh` |
| ViSP | GPL-2.0-or-later | used by `src/visp_servo` |
| Eigen | MPL-2.0 | |
| HailoRT | Hailo proprietary EULA | NPU runtime; not redistributed |
| ONNX Runtime | MIT | CPU detector baselines |

## Model weights

The YOLO models used in the paper derive from **Ultralytics** weights, which are **AGPL-3.0**. They
are **not committed to this repository**; `models/fetch_models.sh` downloads or exports them on
demand. If you redistribute the weights or a derived model, the AGPL-3.0 terms apply to you —
including its network-use provision. The compiled Hailo `.hef` artifacts for YOLOv8/YOLOv11 are the
official Hailo Model Zoo builds and carry Hailo's Model Zoo terms; the YOLO26 `.hef` files are our
own Dataflow Compiler builds of the same Ultralytics weights and inherit AGPL-3.0.

`models/yolo26n_10h.hef` and `src/yolo_ros/yolo26n.pt` are tracked in git history (they predate this
policy) and are subject to the same AGPL-3.0 terms.
