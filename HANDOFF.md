# Project Handoff — ROS2 HIL Visual Servoing (2026-07-04)

**Branch:** `controller-benchmark` | **Pi SSH:** `ssh amaraly@192.168.137.10` (passwordless, key deployed)
**Pi repo:** `/home/amaraly/ROS2-PROJECT-SPRING2026/` | **Container:** `ros2_perception_stack`

> This document supersedes the 2026-07-03 handoff. See `docs/fixlog/` for individual
> applied fixes, `docs/ideas_to_check/` for open hypotheses, and **`docs/TODO.md`**
> (new today) for the reviewed action list going into the next session.

---

## 0. TL;DR for a fresh session

**The headline result: SLAM-init-gate benchmarking now works end-to-end.** A run of
`./run_stack_hil.sh --config orbslam2_eval` produces a real, clean ATE result —
**13.0 cm RMSE over a 60m-class flight (0.33% of path)**, ORB-SLAM2 tracking
essentially the whole way through. This did not work at the start of today's
session — getting here required finding and fixing four independent, real bugs
(not just one), described in full below. Read §2 before touching
`src/init_gate/` or `run_stack_hil.sh`'s gate-sequencing code again — every fix
came from a specific, reproducible symptom, not a guess.

1. **`INITIALIZER_GATE` is implemented, tested, and works.** New package
   `src/init_gate/` (6 small files, ament_python) — a standalone node that runs
   *before* the FSM/detector/controller stack launches, gives monocular ORB-SLAM2 a
   translation-only warmup (mirror-based: out for 1s, back for 1s, per axis — no
   pose feedback, no world-frame math, cannot run away), waits for
   `/slam/tracking_state==OK` (debounced ×2) or a 30s timeout, then hands off. Opt-in
   via `config/hil/stack/orbslam2_eval.yaml`'s `init_gate.enabled: true` — every
   other stack config is untouched (`INIT_GATE_ENABLED` defaults `false`).
2. **Four real bugs were found and fixed today, in order of discovery:**
   - The gate's initial pose-wait used `time.sleep()` instead of `rclpy.spin_once()`
     — subscription callbacks never fired, so it silently timed out every single run
     regardless of whether the simulator was running. **Since fully removed** — the
     final gate design doesn't read pose at all anymore (see next bug).
   - The gate's original design used ground-truth `/sim/drone_pose` to compute a
     "return to center" correction — but `/cmd_vel` is body-frame and the drone
     boots at `yaw=π`, so the world-frame correction inverted sign and the drone
     strafed left **forever** (positive feedback). **Fixed by ripping out pose
     entirely**: the gate now mirrors its own commands (same body-frame velocity,
     sign flipped, same duration) to undo each leg — this cannot invert regardless
     of yaw, because there's no frame conversion left to get wrong.
   - The gate could hand off to the normal stack **mid-displacement** (if SLAM went
     ready right after an outward leg, before its mirror-back) — fixed by making
     the mirror-back unconditional, readiness only ever checked at "center."
   - **The actual reason TS2 never approached the target after a successful gate
     run**: `benchmark_mode`'s fresh-sim guard (`have_fresh_sim_`) needs a MATLAB
     Stop→Run to prove `sim_t<2.0` — but you cannot do that after a SLAM warmup
     without destroying the tracking the gate just built. The FSM was detecting the
     target, centering, freezing at lock-on — and then sitting in `SEARCHING`
     forever. Fixed: gated runs now force `benchmark_mode:=false` (scout mode),
     which engages on first detection with no reset needed.
3. **Bag recording was moved to start earlier** (right after the SLAM sidecar,
   before the gate runs) so a **failed** gate still leaves a real bag to diagnose —
   previously it left only `meta.txt`. Side effect, now understood and partially
   patched: the bag's `/sim/drone_pose` includes Simulink's zero-initialized
   placeholder samples before the model actually starts running (34 samples, all
   exactly `(0,0,0)`, confirmed via direct bag inspection) — `eval_slam_hil.py` now
   filters these, which fixed a ~20m phantom jump in the trajectory plot. The bag
   *itself* still starts before the FSM does — see `docs/TODO.md` item 1, undecided.
4. **`eval_slam_hil.py` restyled** to match `SLAM_benchmark/images/`'s visual
   language (vertical CPU bars, plain GT-vs-SLAM line-overlay trajectory instead of
   an ATE-colored scatter) plus one new plot (front-end tracking latency per frame —
   the single-run analogue of that folder's per-sequence timing plots, since a
   single HIL flight has no "sequence" axis to plot against).
5. **`docs/TODO.md` written** (new today) — 4 items reviewed with the user:
   GT-recording-vs-FSM-start timing (open, undecided), commit sync (see §7), a
   future multi-run SLAM comparison tool (`compare_slam_hil.py`-style, needs a fresh
   well-resourced session), and paper writing.
6. **New standing rule in `CLAUDE.md`**: "ask more, don't just comply" — added
   after explicit user feedback this session. Before executing a request with a
   judgment call baked in (file placement, styling, scope, ambiguous
   interpretation), ask first.

---

## 1. What this project is

MATLAB/Simulink/Unreal (Windows) streams a synthetic camera feed + drone pose over CycloneDDS to a
Raspberry Pi 5. The Pi runs a perception + visual-servoing stack inside Docker and returns `/cmd_vel`.

Three swappable modules:
- **detector** — `yolo` (Hailo NPU) | `oracle` (synthetic bbox from sim pose, ground-truth)
- **controller** — `proportional` (TS2) | `ibvs` (TS1) | `h_vs` (TS3) | `pbvs` (TS4, paused)
- **slam** — `ov2slam` | `orbslam2` | (orbslam3 planned) — runs as Docker sidecar

Entry point: `./run_stack_hil.sh --config <name>` reads `config/hil/stack/<name>.yaml`.

**Architecture, one paragraph:** `servo_fsm_node` (`src/servo_core`) is a single shared
class that owns *all* ROS I/O for the normal flight (detections, sim topics, safety
filters, `/cmd_vel` + `/bench/state` publish) and delegates only the APPROACHING-phase
velocity computation to an injected `IServoController`. All four controllers plug
into this contract. `INITIALIZER_GATE` (new, `src/init_gate/`) is architecturally
**outside** this — a separate node, separate package, zero changes to
`servo_core`, that runs *before* the shared FSM ever launches and exits once SLAM
is warmed up.

---

## 2. INITIALIZER_GATE — design history and why it looks like it does

This was built over many rounds of proposal → critique → correction in today's
session. The end design is much simpler than the first several proposals — every
simplification was earned by a real, specific objection, not aesthetic preference.
If you're tempted to "improve" this design, re-read this section first.

### The problem it solves

ORB-SLAM2 (monocular) needs real translational parallax to initialize. The shared
FSM's `SEARCHING` sequence (`FULL_ROTATE → YAW_RIGHT_60 → YAW_LEFT_60 → YAW_CENTER →
STRAFE_RIGHT`) has three yaw-arrival steps that are pure-P, undamped, tight-tolerance
(0.05 rad) chases against a plant with real rotational lag — a prior session
confirmed via bag data these oscillate forever (570°↔726° swings, 100+s, never
converged) if the FSM is ever held in `SEARCHING` long enough to reach them. The
drone never reached `STRAFE_RIGHT` — the only step with real translation — so SLAM
never got a baseline. Documented in the previous handoff as the "Supreme Leader"
finding: **the FSM never translated, full stop.**

### Design evolution (why it's a separate node, not a flag in the shared FSM)

1. First proposal: a `slam_benchmarking_mode` flag *inside* `servo_fsm_node.cpp`,
   skip the yaw steps, do translation-only legs, "return to exact start point,"
   then hand off. **Critiqued as wrong-as-specified**: "return to exact start"
   was assumed to mean closed-loop position-arrival control — the exact same
   failure shape as the yaw bug, just on a new axis, and this codebase has no
   damping primitive to build it safely.
2. Correction: return-to-start as a **one-shot** computed correction from recorded
   ground truth (record once, compute delta once, execute once, never re-check) —
   structurally cannot oscillate, since there's no loop to oscillate in.
3. Blast-radius concern: this was the third proposed touch to the shared FSM file
   this week. Resolved by moving the whole thing to a **separate, isolated node**
   (`INITIALIZER_GATE`) that runs *before* the FSM/detector/controller stack even
   launches — not gated-inert, genuinely not started yet — which fully eliminates
   any `/cmd_vel` dual-publisher risk and touches `servo_core` not at all.
4. Cycle finalized as `center → left → center → right → center → up → center`,
   looped, absolute-ground-truth-anchored returns (not relative/dead-reckoning, so
   drift doesn't compound cycle to cycle), no "down" leg (avoids ever needing a
   floor-guard duplicate — "up" only, and the return-to-center from "up" is
   inherently a bounded descent back to the known-safe starting altitude).
5. Termination: `run_stack_hil.sh stop` is confirmed **unreachable from inside the
   gate** (host-only, `sudo docker stop`, would be the node asking to kill its own
   container mid-execution) — so on 30s timeout the gate zero-holds, logs
   `/bench/state=SLAM_INIT_FAILED`, exits — bag/sidecar left running, operator
   runs `stop` manually.
6. **Then real-world testing found what design review couldn't**: the ground-truth
   pose approach from step 4 had two implementation bugs (the `time.sleep()` spin
   bug, and the world/body-frame sign inversion causing infinite left-strafe — both
   in §0 above). **The fix for the frame bug was to delete the ground-truth
   approach entirely** and replace it with pure command-mirroring (mirror the
   `/cmd_vel` command itself, not a pose correction) — simpler than anything
   proposed during design review, and immune to the frame bug by construction
   since there's no frame conversion left at all.

### Final architecture

```
run_stack_hil.sh (init_gate.enabled=true path — every other config is untouched)

1. docker run -d ros2_perception_stack
2. ros2 launch hil_simulation.launch.py bridge_nodes:=true stack_nodes:=false
   → sim_camera_bridge + ovcam_bridge start; /ovcam/image_raw flows (SLAM needs this
     — launch file had to be split into bridge_nodes/stack_nodes groups so the
     camera feed can start without the FSM/detector/controller also starting)
3. start_slam_sidecar() (pulled forward from its normal late position)
4. start_bag_recording() (ALSO pulled forward — see §0.3, TODO item 1)
5. docker exec (BLOCKING): ros2 run init_gate init_gate_node
   → mirror cycle: left/back, right/back, up/down, looped
   → checks /slam/tracking_state at each "center"; success or 30s timeout
   → exit 0 (ready) or exit 1 (SLAM_INIT_FAILED, zero-held, logged, bag/sidecar
     left running — die() only prints + exits, never touches docker)
6. if exit 0: benchmark_mode forced false (scout) — see §0.2, this was the FSM-frozen bug
   ros2 launch hil_simulation.launch.py bridge_nodes:=false stack_nodes:=true
   → detector + controller (unmodified FSM) launch fresh, engage on first detection
```

### `src/init_gate/` — current file layout (all Python, ament_python package)

```
src/init_gate/
├── package.xml, setup.py, setup.cfg, resource/init_gate   -- standard skeleton, modeled on oracle_detector
└── init_gate/
    ├── __init__.py
    ├── params.py          -- LEG_SPEED=0.6, LEG_DURATION_SEC=1.0, READY_DEBOUNCE=2, TIMEOUT_SEC=30.0
    ├── slam_readiness.py  -- subscribes /slam/tracking_state, debounces (needs 2 consecutive OK=2)
    ├── motion.py          -- MotionCommander: run_leg() only (elapsed-time /cmd_vel, spins the node
    │                         every tick so subscription callbacks fire), zero_hold(), log()
    ├── cycle.py           -- the mirror sequence + run loop (policy only, no ROS wiring)
    └── init_gate_node.py  -- thin ROS2 Node subclass, wiring only + main()
```

Note: `pose_tracker.py` **no longer exists** — deleted today when the design moved
from ground-truth-position-correction to pure command-mirroring. If you see any
reference to it (docs, old branches, memory), it's stale.

---

## 3. Current state — what's DONE vs pending

| Area | Status | Notes |
|------|--------|-------|
| `INITIALIZER_GATE` | **DONE, tested, works** | Real ATE result obtained (§4). 4 bugs found+fixed today, see §0.2 |
| Launch-file split (`bridge_nodes`/`stack_nodes`) | **DONE** | Both default `true` — zero behavior change for any config that doesn't set them |
| `benchmark_mode` scout-mode-for-gated-runs fix | **DONE** | Was the reason TS2 never approached after a successful gate — see §0.2 |
| Bag-recording reordering | **DONE**, but see TODO item 1 | Failed gates now leave a diagnosable bag; bag still starts before FSM, not at FSM start — undecided |
| `eval_slam_hil.py` fixes + restyle | **DONE, tested** | Zero-GT filter, vertical-bar CPU chart, plain-line trajectory, new frontend-timing plot |
| First successful ORB-SLAM2 HIL benchmark | **DONE** | 13.0cm ATE RMSE, 0.33%/0.39% of full/eval path, 697 samples @ 15.9Hz — see §4 |
| `docs/TODO.md` | **DONE**, needs user review | 4 items, see file directly |
| Multi-run SLAM comparison tool | **NOT STARTED** | TODO item 3 — needs a fresh, well-resourced (Opus) session |
| GT-recording-vs-FSM-start timing | **OPEN, undecided** | TODO item 1 — three options laid out, needs a decision before implementing |
| TS4 PBVS | **PAUSED, untouched** | See `docs/fixlog/005-vx-unbounded.md`, TODO-H |
| OV2SLAM readiness signal | **STILL DOESN'T EXIST** | Confirmed again this session: `src/ov2slam_ros/src/ov2slam_node.cpp` publishes only `/slam/pose`. `init_gate.enabled` is validated ORB-SLAM2-only at boot (`run_stack_hil.sh` dies loudly if combined with any other `slam.type`) |
| Windows ↔ Pi sync | **IN SYNC** as of `16bb178`, except `eval_slam_hil.py` | See §7 |

---

## 4. First successful benchmark result (2026-07-04)

Run: `run_orbslam2_eval_20260704_013420`. Full command: `./run_stack_hil.sh --config orbslam2_eval`.

| Metric | Value |
|---|---|
| **ATE RMSE** | **0.130 m** (13.0 cm) |
| ATE RMSE as % of path | 0.33% (full, post zero-filter) / 0.39% (eval window) |
| ATE mean / max / min | 0.115 m / 0.364 m / 0.007 m |
| Full GT path length | 39.87 m (was 60.48 m before the zero-sample filter fix) |
| Eval-window path length | 33.11 m (unaffected by the filter — was already correctly windowed) |
| SLAM samples | 697 @ 15.9 Hz |
| Monocular scale factor (Umeyama) | 30.27× — expected/necessary for monocular, not a red flag |
| Front-end tracking | 26.2 ms/frame mean, 38.2 Hz, 800 frames — decays from ~50ms at init to a stable ~26ms (see `fig_slam_frontend_timing.png`) |
| CPU | 80% mean, up to 154%, 15 threads |
| Time to first SLAM pose | 15.4s after recording start (8s fixed `startup_delay_sec` + ~7.4s real init, helped by the gate's warmup) |
| SLAM tracking coverage | t=15.4s to t=59.4s — essentially the entire recorded window, no gap at the end |

Plots (`fig_slam_ate.png`, `fig_slam_trajectory.png`, `fig_slam_error_xyz.png`,
`fig_slam_thread_cpu.png`, `fig_slam_frontend_timing.png`, `traj_xy_*.png`,
`slam_metrics.csv`) are sitting at the **repo root on Windows** right now (user's
explicit choice — not committed, just local, may be cleaned up whenever). Same run's
full bag lives on the Pi at
`~/ROS2-PROJECT-SPRING2026/bags/run_orbslam2_eval_20260704_013420/`.

---

## 5. Diagnostic pattern used heavily today (reusable)

To inspect a bag's actual topic contents without a running ROS2 install on the Pi
host, spin up a throwaway container from the `ros2_perception_stack` image:

```bash
sudo docker run --rm --entrypoint '' -v "$(pwd):/workspace" ros2_perception_stack bash -lc "
    source /opt/ros/jazzy/setup.bash
    python3 -c '
import rosbag2_py
from rclpy.serialization import deserialize_message
# ... open StorageOptions(uri=..., storage_id=\"mcap\"), read_next(), deserialize per topic
'
"
```

This is how every bug in §0.2 was actually confirmed — not guessed. Specifically:
the spin-bug was found by noticing `/sim/drone_pose` had 808 messages starting at
t=+0.05s while the gate published exactly ONE `/cmd_vel` message the whole 30s
(the final zero-hold) — mechanical proof the callback never fired. The zero-sample
artifact was found by dumping the first 15 raw `/sim/drone_pose` messages directly
and seeing `[0,0,0,0,0]` repeated 34 times. Don't guess when you can read the bag.

---

## 6. Open TODOs — see `docs/TODO.md` for the reviewed version

1. **GT recording should start when FSM starts, not before** — currently starts
   before the gate even runs (deliberate, for gate-failure diagnostics), which
   means the bag includes ~15-20s of pre-FSM movement. Three options laid out in
   `docs/TODO.md`, undecided, needs your call.
2. **Commit + sync** — only `eval_slam_hil.py` is uncommitted now (everything else
   from today already landed via 7 incremental commits). One more commit+push+pull
   closes it out. See §7.
3. **Multi-run SLAM comparison tool** — `compare_slam_hil.py`-style, analogous to
   the existing `compare_controllers.py`. Motivated directly by today's finding that
   `SLAM_benchmark/images/` is an *offline, 10-sequence, multi-backend* EuRoC study
   (confirmed via its `manifest.json`) — most of its plot types (`*_by_sequence`,
   `pairwise_*`, `comparison_*_all_algorithms`) need multiple HIL runs to mean
   anything here. Flagged as needing a fresh, well-resourced (Opus) session — not
   started.
4. **Paper writing** — circle back once the above stabilizes.
5. (Still open from before, untouched today) **TS4 PBVS** (`docs/fixlog/005`),
   **SEARCHING yaw-settle oscillation** (`docs/ideas_to_check/002` — note this no
   longer blocks anything, since `INITIALIZER_GATE` sidesteps `SEARCHING` entirely
   by running before the FSM ever launches), **SLAM-depth IBVS** (fixlog 009/010).

---

## 7. Windows ↔ Pi sync

**Current state, verified fresh:** both Windows and Pi are at commit `16bb178`
("Added a INITIALIZER_GATE for online SLAM benchmarking, pending test" — the 7th of
7 incremental commits under that same message from today's session). **In sync.**

Only uncommitted difference: `benchmarks/eval_slam_hil.py` is modified identically
on both sides (today's zero-filter + restyle + new plot — I `scp`'d it to the Pi
directly to test, matches the Windows copy byte-for-byte since it's the same file).

To close this out:
```bash
# Windows
git add benchmarks/eval_slam_hil.py docs/TODO.md
git commit -m "..."
git push origin controller-benchmark

# Pi
cd ~/ROS2-PROJECT-SPRING2026
git pull --ff-only origin controller-benchmark
```

**Do NOT commit** (per explicit user instruction): the `fig_slam_*.png`/`traj_xy_*.png`/
`slam_metrics.csv` files sitting at Windows repo root — local only, `slam_metrics.csv`
is gitignored anyway (`*.csv` pattern).

**Pi-only, deliberately untouched, do not commit or revert without asking:**
- `config/hil/stack/orbslam2_eval.yaml`: `detector.cpu: "0"` / `controller.cpu: "1"`
  — the user's own manual edit. **Standing instruction: do not modify `controller_cpu`/
  `detector_cpu` in any config unless explicitly told to.**
- `src/orbslam2/build_stereo/orbslam/{mono,colcon_command_prefix_build.sh.env}` —
  expected rebuild-artifact diffs from `./start_container`, unrelated to anything
  in this handoff.

**`bags/` is gitignored on both sides** — confirmed multiple times this session via
`git ls-files --error-unmatch bags/` (fails, not tracked) and `git status --ignored`
(`!! bags/`). No git operation in the sync sequence above touches it. The only
command that would (`git clean -fdx`) does not appear anywhere in this handoff's
instructions.

---

## 8. Key gotchas from today (additions — see 2026-07-03 handoff for the rest)

- **`time.sleep()` in a ROS2 node body does NOT let subscriptions receive
  messages.** Only `rclpy.spin_once()`/`spin()`/`spin_some()` invoke callbacks. A
  node can sit next to a topic publishing 800 messages and see zero of them if it's
  just sleeping. This cost an entire debugging cycle today — check this first if a
  node "isn't receiving" something that's confirmably being published (verify via
  bag or `ros2 topic echo` from a separate process).
- **`/cmd_vel` is body-frame, `/sim/drone_pose` is world-frame, and the drone boots
  at `yaw=π`.** Any logic that reads world-frame position and issues a body-frame
  correction needs an explicit frame transform — skipping it doesn't just give a
  wrong answer, it can invert into positive feedback (this is exactly what caused
  the infinite left-strafe). When in doubt, prefer mirroring the command itself
  over recomputing a correction from pose — it's frame-invariant by construction.
- **`benchmark_mode`'s fresh-sim guard is fundamentally incompatible with any
  workflow that needs continuous SLAM tracking across a mode transition** — the
  guard's whole mechanism is "wait for a MATLAB Stop→Run reset," and resetting the
  sim teleports the drone and destroys SLAM's accumulated tracking. Any future
  gate/warmup-style mechanism that hands off to the shared FSM needs
  `benchmark_mode:=false` (scout), not `true` — this isn't optional, it's a hard
  incompatibility.
- **A "failed" run leaving no diagnostic bag is worse than a failed run** — the
  fix (start recording earlier) traded one problem for a smaller, better-understood
  one (bag includes pre-FSM movement). Prefer that trade every time: something to
  look at, even if imperfect, beats nothing to look at.
- **Read the bag before hypothesizing.** Every real bug found today (the spin bug,
  the frame-inversion bug, the zero-sample artifact) was confirmed by dumping raw
  bag contents, not by reasoning about the code in isolation. The code reasoning
  was necessary but not sufficient — it explained *why* once the symptom was
  already visible in the data.
