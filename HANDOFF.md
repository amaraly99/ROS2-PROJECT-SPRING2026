# Project Handoff — ROS2 HIL Visual Servoing (2026-06-24)

**Branch:** `controller-benchmark` | **Working dir:** `c:\Users\homie\Desktop\ROS2-PROJECT-SPRING2026`
**Pi SSH:** `ssh amaraly@<pi-ip>` (wlan0 = no IPv4, always use eth0 interface in configs)
**Container image:** `ros2_perception_stack`

---

## 1. What this project is

MATLAB/Simulink HIL quadrotor (MATLAB side = simulator) connected over CycloneDDS to a Raspberry Pi 5 running ROS 2. The Pi runs a perception + visual servoing stack inside Docker. The drone approaches a target sign, guided by one of three visual servo controllers, and stops at a standoff distance $d^* \approx 3.15$ m.

Three controllers compared:
- **TS1 — ViSP IBVS** (`visp_servo`): textbook IBVS with `vpServo`, 4-corner image features, L⁺
- **TS2 — Custom Proportional** (`hil_servo`): Jacobian-free bbox centroid + height-ratio range proxy
- **TS3 — Homography-based VS** (`h_vs_servo`): Benhimane & Malis planar homography, 6-DOF twist

All three share an identical FSM (SEARCHING → APPROACHING → REACHED / REACQUIRE), safety filters, and velocity clamp (3.0 m/s). The ONLY variable is the servoing law.

---

## 2. Infrastructure changes (this session)

### `run_stack_hil.sh` — major rewrites

**YAML config system** — replaces env-var prefix method:
```bash
./run_stack_hil.sh --config ibvs       # loads config/hil/stack/ibvs.yaml
./run_stack_hil.sh --config default    # loads config/hil/stack/default.yaml
```

**Config files** in `config/hil/stack/`:
- `default.yaml` — controller: ibvs, detector: yolo, slam: false, interface: **eth0** (NOT wlan0 — wlan0 has no IPv4)
- `ibvs.yaml`, `h_vs.yaml`, `oracle.yaml`, `full.yaml`

YAML keys: `matlab_host_ip`, `dds`, `ros_domain_id`, `controller`, `detector`, `interface`, `slam`, `debug_image`

Pure-bash YAML parser — no yq/python needed. Parsing uses `${line%%:*}` / `${line#*:}` string splitting.

**Interface pinning** — `interface: eth0` in config → script reads `ip -4 addr show eth0` for `PI_LOCAL_IP` and exports `MATLAB_HOST_IP`. If not set, falls back to `ip route get` auto-detect.

**Build subcommand:**
```bash
./run_stack_hil.sh build              # build all STACK_PKGS
./run_stack_hil.sh build yolo_bridge  # build one package
./run_stack_hil.sh --build --config ibvs  # build then launch
```
Build uses a throwaway Docker container (`docker run --rm -v WS:/workspace ...`). The `install/` dir lives on the Pi host filesystem (volume mount), so builds persist across container restarts.

**Critical build flag:** always `--base-paths src --packages-up-to ${STACK_PKGS}` — bare `colcon build` from `/workspace` crashes because it scans vendored OpenCV source in `/workspace/opencv/` which uses `ocv_define_module` (not a standalone colcon package).

**STACK_PKGS:** `sim_camera_bridge ovcam_bridge yolo_bridge oracle_detector hil_servo visp_servo h_vs_servo ov2slam`

**yolo_producer** hardcoded to `/usr/bin/python3` (line ~353) — pyenv shim at `/home/amaraly/.pyenv/shims/python3` does NOT have hailo_platform or opencv. `/usr/bin/python3` has both.

---

## 3. N=10 Benchmark Results

**Date:** 2026-06-23 | **N=10 per controller** | **30 total runs** | All healthy, zero overshoot

### Mean ± std (ddof=1)

| Metric | IBVS (TS1) | Proportional (TS2) | h_vs (TS3) | Winner |
|---|---:|---:|---:|:--|
| Settling time (s) | **19.17 ± 0.97** | 34.14 ± 1.31 | 35.22 ± 1.58 | IBVS |
| Steady-state err (m) | **0.180 ± 0.014** | 0.214 ± 0.014 | 0.183 ± 0.023 | IBVS ≈ h_vs (tie) |
| Path efficiency | 0.887 ± 0.013 | **0.998 ± 0.005** | 0.986 ± 0.002 | Proportional |
| RMS cmd speed (m/s) | 2.380 ± 0.343 | 0.957 ± 0.131 | **0.840 ± 0.200** | h_vs |
| Control jerk (TV) | 36.53 ± 5.03 | 4.831 ± 0.274 | **1.692 ± 0.066** | h_vs (22× smoother than IBVS) |
| IAE | **329.0 ± 20.5** | 515.7 ± 33.8 | 514.6 ± 27.9 | IBVS |
| CPU (% of 1 core) | 0.812 ± 0.012 | **0.762 ± 0.019** | 0.849 ± 0.026 | ≈ all negligible |
| Memory RSS (MB) | 28.24 ± 0.07 | **18.52 ± 0.03** | 23.72 ± 0.04 | Proportional |

**Key insight — IBVS saturation:** RMS speed 2.38 m/s = 79% of the 3.0 m/s clamp. IBVS is the ONLY controller riding the velocity ceiling through most of the approach — explains fastest settling AND highest jerk (saturated command keeps switching direction as 4 image features compete).

**h_vs lateral parking caveat:** Oracle emits axis-aligned bboxes → `getPerspectiveTransform` returns pure affine H (H[2,2]=1 → e_v[2]=0 → no forward vel from homography). Forward velocity reuses TS2 proportional law (k_fwd=3.0). For affine H: e_v[0] = H[0,2] = (cx−cx0)/fx AND e_w[1] = H[0,2] − H[2,0] = same signal → yaw nulls image error from offset position → drone parks to the side of sign. Input degeneracy, not code bug. Would not occur with a real textured target (genuine perspective component).

**Bag folders (Pi, `~/ros2_ws/bags/`):**
- TS1 ibvs: `ctrl_ibvs_N{1..10}_20260623_19{0358,0605,0809,1104,1522,1839,2236,2513,2840,3159}`
- TS2 prop: `ctrl_proportional_N{1..10}_20260623_{193408,194341,194551,194749,195020,195221,195502,200009,200159,200349}`
- TS3 h_vs: `ctrl_h_vs_N{1..10}_20260623_{200609,201801,202133,202411,202600,202843,203159,203717,204239,204628}`

---

## 4. Files created / modified this session

### Paper: `paper.tex`
Self-contained VS benchmark section (no outer \documentclass — meant to be `\input{}` into a larger paper). Contains:
- Intro: all three as image-based; ViSP PBVS reserved as future/pose-based placeholder
- `\subsubsection{Controller architectures}` — proportional (Jacobian-free, cites chaumette2006visual, viki2024hyco, robinson2023visualservowheels), ViSP IBVS (L+, cites marchand2005visp), h_vs (between image-based and pose-based), oracle-as-SLAM-substitute explanation
- `\subsubsection{Experiment design}` — HIL, oracle, N=10, d*=3.15m, all metric definitions
- Table `tab:vs_median` — median values, image-based group + pose-based placeholder (ViSP PBVS future)
- Table `tab:vs_mean_std` — mean ± std, same grouping
- `\subsubsection{Approach behaviour}` — IBVS analysis with saturation observation (2.38 m/s = 79% clamp), Proportional analysis, h_vs analysis with both affine degeneracy caveats derived
- 3 figures: `VS_Plots/cmp_distance.png`, `cmp_trajectory.png`, `cmp_metrics.png`
- 4 `\cite{}` keys: `chaumette2006visual`, `marchand2005visp`, `viki2024hyco`, `robinson2023visualservowheels`
- **SLAM section removed** — oracle justification explains why SLAM not needed for the comparison

### Reports
- `reports/N10_all_controllers/REPORT.md` — authoritative N=10 report (full per-run table, headline table, all caveats)
- `reports/N3_20260616_rerun/REPORT.md` — marked as SUPERSEDED at top, points to N10 report

### Figures: `VS_Plots/`
- `cmp_distance.png`, `cmp_trajectory.png`, `cmp_metrics.png` — copied from `reports/N10_all_controllers/`
- These are the figures referenced in `paper.tex`

### Scripts
- `benchmarks/compare_controllers.py` — **Fig 1 rewritten** to mean ± 1σ shaded band per controller (was 30 raw overlaid lines). Uses `np.interp` onto common time grid, `np.vstack`, `mean + fill_between`.
- `benchmarks/plot_controller_hil.py` — unchanged, computes per-run `metrics.csv`

### Docs
- `docs/homography_explained.tex` — standalone compilable LaTeX (`pdflatex homography_explained.tex`). Explains homogeneous coords, transformation ladder (affine/projective), K matrix, H=K⁻¹GK, control law e_v and e_w, skew-symmetric/vex operator, body mapping, affine degeneracy (both: forward-vel and lateral-parking), paper symbol decoder table, file map. Has C++ lstlisting blocks with syntax highlighting.
- `docs/homography_explained.md` — Markdown version (same content, 11 sections)
- `docs/metrics_explained.tex` — **NEW this session**. Standalone compilable LaTeX explaining all benchmark metric equations: d* derivation from pinhole model, e(t)=d(t)−d*, IAE/ISE/ITAE with comparison table, settling time (band+hold definition), steady-state error (mean |e| last 3s), overshoot, path length L, path efficiency η, RMS command speed, total variation (jerk), CPU/memory. Every equation matches actual `plot_controller_hil.py` code with code snippets underneath.

---

## 5. Standoff and metric constants (from code)

```python
FY = 554.0          # focal length, vertical (px)
KNOWN_H = 1.5       # target physical height (m)
TARGET_BBOX_RATIO = 0.55  # bbox-height/img-height at standoff
IMG_H = 480         # image height (px)
STANDOFF = 554 * 1.5 / (0.55 * 480) ≈ 3.15 m

SETTLE_BAND = 0.30  # m — tolerance band for settling
SETTLE_HOLD = 2.0   # s — must stay in band this long
```

---

## 6. Why ViSP+YOLO failed in the oldest run_stack_hil.sh

The oldest script (commit `94b659c`) was missing kernel UDP buffer tuning added in commit `5f9f6c5` (message: "sim-hil: full HIL baseline — DDS fixed, all topics live, visp+YOLO+SLAM running").

**Without tuning:** Stock RPi5 has `rmem_max=208 KB` and `ipfrag_high_thresh=4 MB`. MATLAB publishes 921 KB raw frames (~660 UDP fragments at MTU 1500). Kernel drops fragments during reassembly → effective frame rate caps at ~13 Hz with jitter.

**With tuning (current script):** `rmem_max=16 MB`, `ipfrag_high_thresh=32 MB` → full stream through at 30+ Hz.

**Why this breaks ViSP specifically:** `servo_fsm_node` runs its control loop inside `on_detections()` callback (detection-driven, not timer-driven). Control rate = detection rate = camera frame rate. At 13 Hz, ViSP's interaction-matrix law (which linearises around small inter-frame motion) diverges — finite differences are too large, approximation breaks → oscillation/failure. The proportional controller degrades gracefully at lower rates.

**Commit `80afa9f`** also fixed a "YOLO rate halving" bug (shared-semaphore double-consumption) which would have further halved the detection rate in the old script, making things even worse.

---

## 7. Known issues / gotchas

- `wlan0` has no IPv4 on the Pi — **always use `interface: eth0`** in configs
- Bare `colcon build` from `/workspace` crashes on vendored OpenCV — **always use `--base-paths src --packages-up-to ${STACK_PKGS}`**
- pyenv `python3` shim does NOT have hailo_platform — `yolo_producer` hardcoded to `/usr/bin/python3`
- New bags created by root inside Docker → PermissionError → fix: `sudo chmod -R 777 bags/ctrl_*`
- `hil_simulation.launch.py` was rewritten last session but needs rebuild before first run: `./run_stack_hil.sh build`

---

## 8. Pending

- **ViSP PBVS** — listed as future work in paper. Tables have placeholder row. Not implemented yet.
- **Real textured target test for h_vs** — would demonstrate whether perspective component resolves lateral-parking degeneracy
- Compile and review both LaTeX docs (`homography_explained.tex`, `metrics_explained.tex`) — both should `pdflatex` cleanly

---

## 9. How to run

```bash
# Launch with ibvs config
./run_stack_hil.sh --config ibvs

# Build everything then launch
./run_stack_hil.sh --build --config ibvs

# Build only yolo_bridge
./run_stack_hil.sh build yolo_bridge

# Re-analyse a run
python3 benchmarks/plot_controller_hil.py bags/ctrl_ibvs_N1_<stamp>

# Generate comparison figures (give it all 30 run dirs)
python3 benchmarks/compare_controllers.py reports/N10_all_controllers/ bags/ctrl_ibvs_N{1..10}_* bags/ctrl_proportional_N{1..10}_* bags/ctrl_h_vs_N{1..10}_*

# Compile docs
pdflatex docs/homography_explained.tex
pdflatex docs/metrics_explained.tex
```
