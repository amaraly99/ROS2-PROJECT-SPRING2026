# Project Handoff — ROS2 HIL Visual Servoing (2026-07-03)

**Branch:** `controller-benchmark` | **Pi SSH:** `ssh amaraly@192.168.1.60` (passwordless, key deployed)
**Pi repo:** `/home/amaraly/ROS2-PROJECT-SPRING2026/` | **Container:** `ros2_perception_stack`

> This document supersedes the 2026-07-02 handoff. See `docs/fixlog/` for individual applied
> fixes and `docs/ideas_to_check/` for open hypotheses investigated but not (yet) implemented.

---

> ## 🌙 LATE NIGHT SUPREME LEADER IMPORTANT
>
> **The FSM just didn't translate. That's the whole bug, in one sentence.**
>
> `gatetest2`'s drone never moved its x/y/z position at all — confirmed directly from
> `/sim/drone_pose`, pinned to the same coordinates for the entire 145+ second run. It only ever
> rotated. Here's why: the search sequence is `FULL_ROTATE → YAW_RIGHT_60 → YAW_LEFT_60 →
> YAW_CENTER → STRAFE_RIGHT`, and **`STRAFE_RIGHT` is the only step that commands real
> translation** (`linear.y`) — everything before it is rotation-only. The three `YAW_*` steps use
> a pure-proportional controller (no damping) chasing a tight 2.9° arrival tolerance against a
> drone with real rotational lag — overshoot, reverse, overshoot the other way, forever. Since
> "arrived" never becomes true, the FSM never advances past the `YAW_*` steps and **never once
> reaches `STRAFE_RIGHT`.**
>
> Zero translation → zero parallax → ORB-SLAM2 has nothing to triangulate against, no matter how
> long you let it run. It's not that the strafe was too weak or the SLAM system is flaky — **the
> strafe never executed at all.** Any redesign (`OracleFSM` or otherwise) must guarantee real,
> deliberate translation actually happens and isn't gated behind a control loop that can get stuck
> — this is the single most important fact from today's whole session. Don't re-litigate it,
> don't rediscover it the hard way again — just make sure whatever ships next actually moves the
> drone sideways/up-down before assuming SLAM should be initializing.

---

## 0. TL;DR for a fresh session

1. **The SLAM-init-gate fix (FIX-011) was implemented, tested twice, and fully reverted today.**
   It's back to exactly its pre-session state on both Windows and (source-wise) the Pi. Nothing
   from FIX-011 is live. Do not re-attempt this exact approach without reading §2–§4 below first
   — it's not that the idea was wrong, it's that it exposed a *second*, pre-existing bug that
   made it untestable, and prompted a full rethink of where this fix belongs architecturally.
2. **A new design ("OracleFSM" — a completely separate, isolated FSM, not touching the shared
   `servo_core`) was proposed, then put through a full critic+KISS review. Verdict: the core idea
   is right, but three specific parts of it are either undeliverable or mechanically impossible as
   worded.** Full critique in §5 — **read this before writing any code next session.** The user's
   own assessment at end of session: *"the idea is flawed, there is no structurally easy way to
   truly handle it without more thinking."* Next session should start with design, not
   implementation.
3. **A live landmine exists in committed code right now** (commit `7d2fb4e`):
   `config/hil/stack/orbslam2_eval.yaml` still sets `blind_until_slam_ready: true` /
   `max_blind_strafes: 2`, and the launch-arg plumbing for both is still fully wired through
   `parse_stack.py` → `run_stack_hil.sh` → `hil_simulation.launch.py` — but the FSM node
   (`servo_fsm_node.cpp`) no longer declares or reads either param, since only the two FSM source
   files were reverted. Running `orbslam2_eval` today silently passes two dead params into the
   node. **Needs cleanup before `OracleFSM` work starts on the same stack config** (see §6, TODO-Y).
4. **Windows ↔ Pi are diverged again** — Windows has committed today's work as `7d2fb4e` (1 ahead
   of `origin/controller-benchmark`, not pushed). The Pi has the equivalent file content but
   **uncommitted** (still sitting on `add7c90`). See §7.

---

## 1. What this project is

MATLAB/Simulink/Unreal (Windows) streams a synthetic camera feed + drone pose over CycloneDDS to a
Raspberry Pi 5. The Pi runs a perception + visual-servoing stack inside Docker and returns `/cmd_vel`.

Three swappable modules:
- **detector** — `yolo` (Hailo NPU) | `oracle` (synthetic bbox from sim pose, ground-truth)
- **controller** — `proportional` (TS2) | `ibvs` (TS1) | `h_vs` (TS3) | `pbvs` (TS4, paused)
- **slam** — `ov2slam` | `orbslam2` | (orbslam3 planned) — runs as Docker sidecar

Entry point: `./run_stack_hil.sh --config <name>` reads `config/hil/stack/<name>.yaml`.

**Architecture, in one paragraph** (see the dependency-map artifact built this session for the
full visual): `servo_fsm_node` (`src/servo_core`) is a single shared class that owns *all* ROS I/O
— detections, sim topics, safety filters (clamp/ramp/floor), `/cmd_vel` + `/bench/state` publish —
and delegates only the APPROACHING-phase velocity computation to an injected `IServoController`
(`servo_controller.hpp`: `ServoInputs` in, `ServoVel` out). All four controllers plug into this
same contract. The one architectural exception: `visp_servo` (TS1/IBVS) is the only controller
that touches SLAM data, and it does so by owning its own direct `/slam/pose`/`/slam/cloud`
subscriptions internally — bypassing the FSM's "owns all I/O" rule. That bypass is exactly the
seam today's SLAM-init work ran into.

---

## 2. What was attempted today (2026-07-02/03), in order

### FIX-011 — SLAM-init gate on the shared FSM (implemented → reverted)

**Goal:** `orbslam2_eval` benchmark runs (oracle + TS2 + ORB-SLAM2) were producing wildly
inconsistent SLAM trajectory quality, because the drone commits to forward-dominant `APPROACHING`
motion before ORB-SLAM2 ever gets translational parallax (monocular SLAM needs real lateral/
vertical baseline; forward motion is near-degenerate). Root cause fully diagnosed in
`docs/ideas_to_check/001-orbslam2-monocular-init-reliability.md`.

**What was built:** an opt-in `SlamGate` state machine added to `servo_fsm_node.hpp`/`.cpp`
(shared by all 4 controllers), gated by `blind_until_slam_ready`/`max_blind_strafes` params,
default off. When enabled: ignore detections and free-run the existing `SEARCHING` step sequence
(which includes one real translational leg, `STRAFE_RIGHT`) until `/slam/tracking_state` reports
OK, then force one `FULL_ROTATE` re-sweep to re-find the target, then release to normal operation.
Reviewed by critic + KISS agents per the SECOND BRAIN protocol before merging — critic verdict
`partial` (several real risks found and fixed: tracking_state topic wiring, `handle_sim_restart`
not resetting the gate, FAILED-state leaking through the watchdog), KISS verdict
`simplify-recommended` (adopted the naming fix, kept the timing-sensitive param). Full writeup:
`docs/fixlog/011-slam-init-gate.md`.

**First live test (`gatetest1`)**: gate went `SLAM_BLIND → SLAM_INIT_FAILED` after 2 blind
strafes. Traced the bag + the SLAM sidecar's `timing_events.csv` (cross-referencing two different
container clocks with a ~4h timezone offset) and found: **ORB-SLAM2 had actually initialized
18.87 seconds before the gate gave up.** Root cause: the *deployed* ORB-SLAM2 ROS2 wrapper binary
(`install_stereo/orbslam/lib/orbslam/mono`) was built **Jun 28**, but the source file containing
the `/slam/tracking_state` publisher (`monocular-slam-node.cpp`) was last edited **Jun 29** — the
publisher code existed in source but was never compiled into the running binary. Confirmed via
`strings` on the binary (absent: `slam_tracking_state`; present: `slam_pose`). **This is FIX-012**
— fixed by rebuilding via `src/orbslam2/./start_container` (rebuilds the ORB-SLAM2 core lib +
ROS2 wrapper; this is a separate build path from `./run_stack_hil.sh build`, which doesn't touch
`src/orbslam2/` at all). Verified post-rebuild the binary now contains the publisher string.
Writeup: `docs/fixlog/012-stale-orbslam2-wrapper-binary.md`. **This fix is real, independent of
FIX-011, and was NOT reverted — it stands on its own.**

**Second live test (`gatetest2`, after FIX-012)**: with the tracking-state topic now genuinely
flowing, the gate still never worked — `/bench/state` stayed on `SLAM_BLIND` and `vy` (strafe
velocity) was `0.000` for the entire 145+ second run. Traced fine-grained `/sim/drone_pose` +
`/cmd_vel` data and found a **second, pre-existing, unrelated bug**: the shared FSM's
`YAW_RIGHT_60`/`YAW_LEFT_60`/`YAW_CENTER` search steps use a pure-proportional yaw controller
(`k_search_yaw_ * err`, clamped to `max_angular_`, **no damping term**) chasing a tight arrival
tolerance (`search_yaw_arrive_tol_rad = 0.05` rad ≈ 2.9°) against a simulated drone with real
rotational lag/inertia — this produces an **undamped ~150° limit cycle that never converges**
(confirmed: yaw swung 570°↔726° repeatedly over 100+ seconds, no decay). Confirmed via a
*historical* pre-FIX-011 bag (`orbslam2_eval_39`) that this code path had genuinely never been
exercised before: `SEARCHING` always used to exit via lock-on in ~11s, *less* than `FULL_ROTATE`'s
own 13s duration — meaning the drone had never, in this project's history, stayed in `SEARCHING`
long enough to reach the `YAW_*` steps until FIX-011's gate held it open on purpose. `FULL_ROTATE`
and `STRAFE_RIGHT`, by contrast, are purely elapsed-time-based (no arrival check) and can't get
stuck — only the three `YAW_*` steps have this problem. Full writeup, evidence table, and two
candidate fixes (not implemented): `docs/ideas_to_check/002-searching-yaw-settle-oscillation.md`.

**Decision: revert FIX-011 entirely.** Per user direction — the shared FSM had already been
touched twice today (the gate, then the FIX-012 rebuild), and modifying it a third time to chase
the oscillation bug was ruled out for today. `servo_fsm_node.hpp`/`.cpp` were `git checkout --`'d
back to `add7c90` on **both** Windows and the Pi (confirmed clean diff both places), and rebuilt
on the Pi (`./run_stack_hil.sh build`, clean, all 11 packages) so the deployed binaries match the
reverted source. **The shared FSM is, right now, byte-for-byte what it was before this session.**

### OracleFSM — proposed replacement design (discussed, critiqued, not implemented)

Given the shared FSM is a monolith every controller depends on (confirmed via a dependency-map
diagram built this session — see the Artifact from earlier in the conversation, not saved to
disk), and today's bug is direct proof that changing it has blast radius across all 4 controllers,
the user proposed a **completely separate, isolated FSM class ("OracleFSM")** used only by the
`orbslam2_eval` benchmark path, never touching `servo_core`. Design evolved through discussion to:

1. Record the drone's initial position on boot.
2. Run a warmup maneuver (translation-only — lateral and/or vertical, tunable speed) to give
   ORB-SLAM2 real parallax, bounded by a hard time budget (`MAX_WAIT_INITIALIZATION_SLAM`).
3. On SLAM-ready: return to the recorded initial position, then hand off to the *existing,
   unmodified* `ProportionalController` (TS2) via the *existing, unmodified* `IServoController`
   contract — so the recorded flight is meant to look like a stock TS2 `controller_bench` run.
4. On timeout: stop the experiment.

**This was put through the full critic+KISS SECOND BRAIN protocol before any code was written.**
Critic verdict: **WRONG, as literally specified.** Three separate parts of the design as worded
cannot be built as stated:

- **"Exactly like `controller_bench`"** is not achievable — `controller_bench.launch.py` runs in
  total isolation (no camera bridge, no SLAM sidecar); any SLAM-eval run necessarily has the SLAM
  sidecar + camera bridge running alongside, which `HANDOFF.md` (this file, historically) already
  documents as a non-comparable CPU/timing environment. The honest goal is "same controller law,
  comparable-but-not-identical trajectory," not "exactly the same."
- **"Return to initial point immediately"** is a brand-new, undamped, 3-axis position-arrival
  controller, proposed the same day the project's only other arrival-controller (1-axis yaw) was
  found to oscillate forever and had to be reverted. No damping primitive exists anywhere in this
  codebase to copy from. The offset to correct is *variable*, not fixed — SLAM-init or the
  timeout can fire mid-leg of the warmup maneuver, not just after a clean cycle. Needs an explicit
  loose tolerance + damping + hard timeout fallback, none of which the design specifies.
- **"Experiment stops immediately"** is mechanically underspecified and probably can't be done
  from inside one ROS2 node: the FSM/controller node, the SLAM sidecar (separate Docker container),
  and bag recording (a separate shell-level `docker exec` subprocess with no handle back to the
  FSM) are three independent processes. A ROS node can't `docker stop` a sidecar or kill a shell
  subprocess it doesn't own. Either new cross-process signaling is needed (doesn't exist today),
  or "stop" really means "FSM zero-holds + logs + marks `/bench/state`, sidecar/bag keep running
  until manually stopped" — the design needs to say which.

KISS verdict: **simplify-recommended** — a single-direction lateral strafe (not 4-way up/down/
left/right; monocular degeneracy is specifically forward/back, one lateral leg already breaks it,
and vertical motion is a brand-new command axis with real floor-safety-guard risk for no proven
benefit); no need to defer bag recording (`eval_slam_hil.py`'s ATE window is *already* just the
temporal intersection of `/slam/pose` ∩ `/sim/drone_pose` timestamps — a warmup prelude is
excluded from the metrics for free, since SLAM publishes nothing until initialized); hold-and-log
rather than hard-teardown on timeout (matches the existing house pattern — no new IPC needed);
debounce the SLAM-ready signal (require ~2 consecutive `tracking_state==OK` messages, mirroring
the existing `lockon_consec` pattern, not just one).

Additional gaps found: **OV2SLAM has no equivalent readiness signal** (confirmed in
`slam_pose_source.hpp` code comments — it never publishes anything analogous to ORB-SLAM2's
`/slam/tracking_state`), so "any other SLAM" in the original wording should be scoped to
ORB-SLAM2 only for now. No answer yet for **how `OracleFSM` actually gets launched** (no slot
exists today in `hil_simulation.launch.py`/`parse_stack.py` for a non-`IServoController` FSM
variant — likely needs its own dedicated launch file, matching the `controller_bench.launch.py`
precedent of "special-purpose harness gets its own launch file"). Unclear whether it needs to
emit `/bench/state` in the exact same string format so `plot_controller_hil.py`/
`compare_controllers.py` work unmodified — probably yes, given the goal is plot comparability, but
not yet stated as a requirement.

**Status: paused for more design thinking. Not implemented. No code written for OracleFSM.**

---

## 3. Current state (what's DONE vs pending)

| Area | Status | Notes |
|------|--------|-------|
| Phase A — modular stack | **DONE, Pi-tested** | `run_stack_hil.sh` + `parse_stack.py` + all stack configs |
| ORB-SLAM2 sidecar | **DONE, Pi-tested** | `src/orbslam2/` + `orbslam2_fixed` Docker image |
| FIX-012 — stale wrapper binary | **DONE** | `/slam/tracking_state` now genuinely publishes; independent of FIX-011 |
| FIX-011 — SLAM-init gate | **REVERTED** | Implemented, tested twice, found a second bug, fully reverted. See §2 |
| SEARCHING yaw-settle oscillation | **OPEN, documented, unfixed** | `docs/ideas_to_check/002-*.md` — undamped ~150° limit cycle in `YAW_RIGHT_60`/`LEFT_60`/`CENTER`, blocks any future gate-style fix on the shared FSM |
| OracleFSM (isolated redesign) | **DESIGN STAGE, critiqued, paused** | See §2 — needs another design pass before implementation |
| Dead SLAM-gate wiring in `orbslam2_eval.yaml` | **OPEN — landmine** | See §0.3, §6 TODO-Y |
| ORB-SLAM2 monocular init reliability (original) | **STILL OPEN** | Underlying problem unresolved — none of today's attempts got a clean end-to-end run |
| TS4 PBVS | **PAUSED** | Diverges in APPROACHING — see `docs/fixlog/005-vx-unbounded.md`, TODO-H. Also newly noted: `CMakeLists.txt` still requires ViSP for a controller that no longer calls it (vestigial, found during today's dependency-mapping) |
| ORB-SLAM3 live integration | **PLANNED, untouched today** | Needs adapter + C++ pose-topic patch |
| Windows ↔ Pi sync | **DIVERGED AGAIN** | See §7 |

---

## 4. How to run everything

Unchanged from before — see `./run_stack_hil.sh --help` / comments at the top of the script.

```bash
./run_stack_hil.sh --config orbslam2_eval          # SLAM ATE capture — CURRENTLY HAS DEAD PARAMS, see §0.3
./run_stack_hil.sh --config ov2slam_oracle         # OV2SLAM ATE capture — unaffected by today's session
./run_stack_hil.sh build                            # rebuild all HIL stack packages (does NOT rebuild src/orbslam2/)
cd src/orbslam2 && ./start_container                 # rebuild ORB-SLAM2 core + ROS2 wrapper specifically
./run_stack_hil.sh stop
```

**Caution:** running `orbslam2_eval` right now will pass `blind_until_slam_ready:=true` and
`max_blind_strafes:=2` into `hil_servo_node`, which no longer declares those parameters (reverted
in FIX-011's rollback). This should be harmless (ROS2 typically ignores undeclared parameter
overrides rather than erroring) but is untested in this exact state and is a known loose end —
see TODO-Y.

Evaluating a bag is unchanged: `python3 benchmarks/eval_slam_hil.py bags/run_<cfg>_<stamp>`.

### Diagnostic pattern developed today (reusable)

To inspect what actually happened in a bag without a running ROS2 install on the Pi host (ROS2
only exists inside Docker), spin up a throwaway container from the `ros2_perception_stack` image
and read the bag with `rosbag2_py` + `rclpy.serialization.deserialize_message` directly:

```bash
sudo docker run --rm --entrypoint '' -v "$(pwd):/workspace" -v /tmp:/tmp ros2_perception_stack bash -lc "
    source /opt/ros/jazzy/setup.bash
    source /workspace/install/setup.bash
    python3 /tmp/your_script.py /workspace/bags/run_.../bag
"
```
Watch for container-clock timezone offsets between the SLAM sidecar's `timing_events.csv`
(wall-clock timestamps) and the bag's recorded `t` (also wall-clock, but potentially a different
container's clock/timezone) — today's session found a clean ~4-hour offset between them that had
to be corrected for before cross-referencing was valid.

---

## 5. Full critic + KISS findings on OracleFSM (read before designing further)

See §2 above for the summarized version. The two full agent transcripts aren't saved verbatim,
but every finding from both was folded into the summary in §2 and cross-checked directly against
this conversation. If picking this up fresh, re-derive by re-reading:
- `src/servo_core/include/servo_core/servo_controller.hpp` — the contract any new FSM must honor
  to reuse `ProportionalController` unmodified.
- `src/servo_core/src/servo_fsm_node.cpp` `SearchStep` handling (~lines 627-790) — the oscillation
  bug's exact shape, and the *good* pattern (`FULL_ROTATE`/`STRAFE_RIGHT`'s elapsed-time design) to
  copy for any new warmup maneuver.
- `benchmarks/controller_bench.launch.py` — the "special-purpose harness gets its own launch file"
  precedent.
- `benchmarks/eval_slam_hil.py` (~lines 240-290) — confirms the ATE window is a pure
  `/slam/pose` ∩ `/sim/drone_pose` timestamp intersection, not `/bench/state`-keyed.
- `src/visp_servo/include/visp_servo/slam_pose_source.hpp` — code comments confirming OV2SLAM
  never publishes a tracking-state-equivalent topic.

---

## 6. Open TODOs (active — resume here)

| ID | Fix/Idea | Description | Priority |
|----|-----|-------------|----------|
| **TODO-Y** (new) | landmine | Clean up `orbslam2_eval.yaml`'s dead `blind_until_slam_ready`/`max_blind_strafes` + the still-wired launch-arg plumbing in `parse_stack.py`/`run_stack_hil.sh`/`hil_simulation.launch.py`, OR confirm it's harmless as-is and leave a comment saying so. Do this **before** starting `OracleFSM` work on the same stack config, or there will be two half-wired warmup mechanisms tangled together. | High, blocks next session |
| **TODO-Z** (new) | OracleFSM redesign | Re-derive the design incorporating all critic+KISS findings (§2/§5): honest "comparable, not exact" framing vs `controller_bench`; a *specified* return-to-origin controller (loose tolerance + timeout fallback + some damping); a *specified* meaning of "stop" (hold-and-log vs. real teardown, and if the latter, what new IPC it needs); single-direction strafe not 4-way; debounced SLAM-ready detection; explicit launch-integration plan (new dedicated launch file?); explicit `/bench/state` format decision. | High — the actual next step |
| **TODO-V/W/X** (idea 002) | yaw oscillation | Still open, still unfixed. Two candidate fixes documented (max-dwell timeout fallback vs. config-only tolerance loosening), neither implemented. Only matters again if a future design puts the *shared* FSM back in `SEARCHING` for an extended period — `OracleFSM`, if it doesn't reuse `SEARCHING` at all (current plan), sidesteps this entirely. | Medium, contingent |
| **TODO-Q/R** (idea 001) | SLAM init (original) | Superseded by FIX-011/012's investigation but the *underlying* reliability problem is still fundamentally unresolved end-to-end — no clean full run has happened yet. | High, this is the actual goal |
| **TODO-H** (FIX-005) | PBVS | Paused — re-enable `wz` or diagnose why TS2 survives the same geometry without yaw. Untouched today. | Medium |
| **TODO-K** (FIX-006) | IBVS oscillation | Untouched today. | Medium |
| **TODO-L/M/N/O** (FIX-009/010) | SLAM-depth IBVS | Untouched today. | Medium/Low |

---

## 7. Windows ↔ Pi sync (read before running any git command)

**Windows**: today's work (FIX-011's non-FSM parts, FIX-012's docs, idea 002, this handoff) is
committed as `7d2fb4e "added OV2SLAM and ORBSLAM benchmarking support"`, sitting 1 commit ahead of
`origin/controller-benchmark`, **not pushed**. `servo_fsm_node.hpp`/`.cpp` are reverted and match
`add7c90` exactly (verified clean diff) — they are NOT part of what changed in `7d2fb4e` relative
to FSM behavior; the commit is everything else (docs, config, launch/parse wiring, FIX-012's
already-rebuilt binary artifacts under `src/orbslam2/`... actually verify this — the orbslam2
build artifacts were rebuilt on the **Pi**, not Windows; Windows' `7d2fb4e` does not include the
Pi's rebuilt `install_stereo`/`build_stereo` binaries, those only exist on the Pi's filesystem).

**Pi**: still sitting on `add7c90`, **uncommitted**, with working-tree changes that are the
file-level equivalent of what Windows committed as `7d2fb4e` (confirmed via `git status --short`:
same files modified — `orbslam2_eval.yaml`, `run_stack_hil.sh`, `parse_stack.py`,
`hil_simulation.launch.py`, `docs/fixlog/README.md`, plus untracked `docs/fixlog/011-*.md`,
`012-*.md`, `docs/ideas_to_check/`), **plus** the Pi-only rebuilt ORB-SLAM2 binaries under
`src/orbslam2/build_stereo/` (tracked in git, shows as modified: `colcon_command_prefix_build.sh.env`,
`mono`) and one pre-existing unrelated mode-only diff (`benchmarks/slam_cpu_sampler.sh`,
644→755 chmod, zero content change, safe to discard, noted in the 2026-07-02 handoff and still
true). `servo_fsm_node.hpp`/`.cpp` are reverted and clean on the Pi too (confirmed).

**To sync** (once TODO-Y is resolved — resolve it on Windows first, then push, so the Pi pulls a
clean state rather than needing its own separate cleanup):

```bash
# Windows — after TODO-Y cleanup, if desired
git add -A
git commit -m "..."   # or amend if TODO-Y cleanup happens before anyone else pulls 7d2fb4e
git push origin controller-benchmark

# Pi — discard local uncommitted changes (they're superseded by what's already pushed) and pull
ssh amaraly@192.168.1.60
cd ~/ROS2-PROJECT-SPRING2026
git checkout -- benchmarks/slam_cpu_sampler.sh   # discards the mode-only chmod diff, no content lost
git stash -u   # or git checkout -- <files> individually if you want to keep the rebuilt orbslam2 binaries staged separately — VERIFY before discarding, the build_stereo/orbslam/mono rebuild took real time to produce
git fetch origin
git reset --hard origin/controller-benchmark
cd src/orbslam2 && ./start_container   # rebuild orbslam2 again post-reset, since reset wipes the Pi's already-rebuilt binaries
```

> **Warning**: the Pi's `build_stereo`/`install_stereo` binaries reflect today's FIX-012 rebuild.
> A `git reset --hard` will revert those tracked files back to whatever's committed (possibly the
> pre-FIX-012 stale binary, if the rebuilt binary was never committed from the Pi side — check
> `git log -- src/orbslam2/build_stereo/orbslam/mono` before resetting). If so, **re-run
> `./start_container` on the Pi after the reset** to restore the FIX-012 fix, or you'll silently
> reintroduce the stale-binary bug.

---

## 8. SLAM integration details (unchanged from 2026-07-02, still accurate)

### Canonical topics (contract between all SLAMs and the rest of the stack)
- `/slam/pose` — `geometry_msgs/PoseStamped` — SLAM camera pose in map frame
- `/slam/cloud` — `sensor_msgs/PointCloud2` — map points (used by SlamDepthSource)
- `/slam/tracking_state` — `std_msgs/Int32` — **ORB-SLAM2 only** (confirmed today: OV2SLAM never
  publishes this). `-1=SYSTEM_NOT_READY, 0=NO_IMAGES_YET, 1=NOT_INITIALIZED, 2=OK, 3=LOST`.

### ORB-SLAM2 sidecar (`orbslam2_fixed` image)
- Code: `src/orbslam2/ros2-ORB_SLAM2/` (ROS2 wrapper) + `src/orbslam2/{include,src}` (core lib,
  vendored ORB-SLAM2 + `BenchmarkUtils.cc` for CSV timing/thread-naming instrumentation)
- **Two separate build paths, do not confuse them**: `./run_stack_hil.sh build` builds the main
  HIL stack packages (`STACK_PKGS` — does NOT include `orbslam`). `cd src/orbslam2 &&
  ./start_container` rebuilds the ORB-SLAM2 core lib + ROS2 wrapper specifically — **this is the
  one that must be re-run whenever `ros2-ORB_SLAM2/src/**` changes**, discovered the hard way
  today when the deployed binary was ~12 hours stale relative to its own source (FIX-012).
- Binary: `/workspace/src/orbslam2/install_stereo/orbslam/lib/orbslam/mono` (bind-mounted live
  from `${WS}`, no image-baked copy — editing source + rebuilding takes effect immediately)
- Startup gotchas unchanged from 2026-07-02 handoff (direct-binary invocation, `LD_LIBRARY_PATH`
  overlay, `--entrypoint ""`, Tcw size guard, headless Viewer, `startup_delay_sec` — now **8s**,
  down from 15s, see `docs/fixlog/011-slam-init-gate.md`'s "Timing update" section for why the
  original 15s rationale doesn't cleanly transfer once a blind-search-style gate is involved).

### OV2SLAM sidecar (`ros2_perception_stack` image) — unchanged, untouched today

### SLAM depth for IBVS (`SlamDepthSource`) — unchanged, untouched today

---

## 9. Phase B — ORB-SLAM3 live integration (PLANNED, untouched today)

Unchanged from 2026-07-02 handoff — see that section if picking this up. Not touched this session.

---

## 10. N=10 Controller benchmark results (unchanged, for reference)

Unchanged from 2026-07-02 handoff. `reports/N10_all_controllers/REPORT.md` has the full numbers.
Relevant new context from today: the dependency-mapping exercise reconfirmed
`controller_bench.launch.py` is deliberately, completely isolated from SLAM/camera-bridge/YOLO —
this is exactly why `OracleFSM`'s "exactly like controller_bench" framing needed correcting (§2).

---

## 11. Key file map

```
run_stack_hil.sh                              — entry point (modular, SLAM sidecar, benchmark recording, --run-tag)
scripts/parse_stack.py                        — nested YAML → flat env vars (still has dead blind_until_slam_ready/max_blind_strafes emit — TODO-Y)
config/hil/stack/orbslam2_eval.yaml           — HAS DEAD PARAMS right now, see §0.3/TODO-Y
config/hil/bench_fsm.yaml                     — shared FSM params (search_yaw_arrive_tol_rad=0.05 — the value implicated in idea 002's oscillation)

src/sim_camera_bridge/launch/hil_simulation.launch.py  — still has blind_until_slam_ready/max_blind_strafes launch args wired (TODO-Y) — no slot for a non-IServoController FSM variant like OracleFSM yet
src/servo_core/src/servo_fsm_node.cpp         — shared FSM, REVERTED to add7c90, matches pre-session state exactly
src/servo_core/include/servo_core/servo_controller.hpp — IServoController/ServoInputs/ServoVel contract — untouched, this is what OracleFSM must honor to reuse ProportionalController
src/hil_servo/                                — Proportional controller (TS2) — untouched, what OracleFSM would delegate to
src/visp_servo/include/visp_servo/slam_pose_source.hpp — code comment confirms OV2SLAM never publishes tracking_state
src/orbslam2/                                 — ORB-SLAM2 source + HIL wrapper
  ros2-ORB_SLAM2/src/monocular/monocular-slam-node.cpp — tracking_state publisher (source was fine; deployed binary was stale — FIX-012)
  start_container                             — THE build script for this package specifically (separate from run_stack_hil.sh build)

benchmarks/eval_slam_hil.py                   — ATE eval; window = /slam/pose ∩ /sim/drone_pose timestamp intersection (NOT /bench/state-keyed — this fact mattered twice today)
benchmarks/controller_bench.launch.py         — ISOLATED oracle+controller harness (N=10 benchmark, no SLAM) — the "special harness gets its own launch file" precedent for OracleFSM

docs/fixlog/                                  — ADR-style archive of APPLIED fixes
  011-slam-init-gate.md                       — the gate that was implemented, tested, and reverted today
  012-stale-orbslam2-wrapper-binary.md        — the real, standing fix from today
docs/ideas_to_check/                          — open hypotheses NOT yet implemented
  001-orbslam2-monocular-init-reliability.md  — graduated → fixlog/011, but the underlying problem is still open (011 was reverted)
  002-searching-yaw-settle-oscillation.md     — the yaw bug found today, fully documented, not fixed
```

---

## 12. Known gotchas / Pi environment (additions from today; see 2026-07-02 handoff for the rest)

- **Two build systems for two different things** — `./run_stack_hil.sh build` (main stack) vs.
  `cd src/orbslam2 && ./start_container` (ORB-SLAM2 specifically). A stale-binary bug (FIX-012)
  came directly from not realizing these are separate; if ORB-SLAM2 behavior seems to not match
  its own source, rebuild with the right script.
- **Container clocks can be offset** — the SLAM sidecar's `timing_events.csv` wall-clock and a
  bag's recorded timestamps can differ by a clean multi-hour offset (found: ~4h) if the containers
  have different `TZ` settings. Always sanity-check by converting both to human-readable
  datetimes before cross-referencing; don't assume they share an epoch reference just because
  they're both "wall clock."
- **Any new arrival/position-control logic needs damping + a timeout fallback by design, not as an
  afterthought** — this is the single biggest lesson from today, twice-confirmed (the yaw
  oscillation, and the critic's warning about `OracleFSM`'s proposed return-to-origin repeating the
  same shape). If it chases an error to a tight tolerance with pure-P against a plant with any real
  lag/inertia, assume it can oscillate forever until proven otherwise.
- **Bag-reading without a Pi-side ROS2 install**: `ros2` CLI/Python isn't installed on the bare Pi
  host, only inside the Docker image. Use a throwaway `docker run --rm` container from
  `ros2_perception_stack` (mounting the repo + `/tmp`) to run `rosbag2_py` scripts — see §4's
  reusable diagnostic pattern.
