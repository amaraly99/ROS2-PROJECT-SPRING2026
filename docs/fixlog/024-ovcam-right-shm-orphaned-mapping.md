---
id: FIX-024
title: ovcam_bridge_right silently reads a deleted/orphaned shared-memory segment -- caused by run_stack_hil.sh's cleanup never targeting the right-eye shm/sem paths
date: 2026-09-04
status: resolved  # TODO-AP implemented + verified end-to-end with a real recorded run producing genuine /slam/pose output (see Post-fix verification)
component: run_stack_hil.sh (cleanup paths) / sim_camera_bridge / ovcam_bridge (right-eye pair)
supersedes: none
critic_verdict: open_concern (real gap found, does not invalidate the fix for the diagnosed mechanism -- see below)
kiss_verdict: ok
open_todos: [TODO-AL, TODO-AM, TODO-AN, TODO-AO, TODO-AQ]
---

## Symptom

Multiple real recorded bags (4+ independent runs, different starting
conditions each time) all show literally zero `/slam/pose` messages from
OV2SLAM in stereo mode. Ruled out one by one, with direct evidence, before
reaching this root cause: not a bag/QoS recording problem (bag QoS config
checked, live `ros2 topic info --verbose` showed identical RELIABLE/VOLATILE
on both sides), not "image producer not running" (raw shared-memory bytes
read directly, both eyes' frame counters climbing steadily, ~9/sec each),
not a topic-name mismatch (calib-file-driven topic names traced end to end,
matched exactly). Narrowed specifically to: the right-eye relay process
never publishes a single message, despite being a registered, QoS-matched
publisher sitting on top of a genuinely active data source.

## Root cause

`/proc/<ovcam_bridge_right PID>/maps` shows:
```
/dev/shm/ovcam_frames_right (deleted)
```
while the working left-eye reader's mapping is clean (no `(deleted)`).
Re-checked live at time of diagnosis -- current state, not a stale one-off
read.

Mechanism, confirmed directly from `sim_camera_bridge_node.cpp`'s
`create_shm()` (not inferred): every time the picture-writer process starts,
it unconditionally does
```cpp
shm_unlink(shm_name_.c_str());
int fd = shm_open(shm_name_.c_str(), O_CREAT | O_RDWR, 0666);
```
-- wipe-and-recreate, with no coordination against a reader that might
already have the old object open. The relay (reader) process's own open
code has no `O_CREAT` and no detection/recovery if the object it originally
mapped gets deleted out from under it -- it just keeps reading from
whatever memory it already has mapped, forever, silently.

`ps -o pid,lstart` for both the right-eye writer and the right-eye reader
shows the identical start second (`23:35:36`) -- `ps`'s resolution can't
prove sub-second ordering, but same-second startup is exactly the condition
under which this race is expected to bite, so it doesn't disprove the
mechanism; it's consistent with it.

**This is symmetric at the code level**: the left-eye pair runs the exact
same unlink-and-recreate code path in `create_shm()`. Left is not immune to
the same mechanism in principle -- but see below for why right specifically
is far more exposed in practice.

**Refinement -- the actual trigger is not a tight startup race, it's a
confirmed, standing gap in `run_stack_hil.sh`'s cleanup.** Grepped the
entire script for every reference to the right-eye shm/sem paths
(`grep -n '_right\|ovcam_frames\|sem.ovcam_ready' run_stack_hil.sh`):
`_right` does not appear in any cleanup, readiness-wait, or permissions
step, anywhere in the file. Specifically:
- Both the explicit `stop` command (line ~464) and the pre-launch cleanup
  that runs before every new run (line ~637) do:
  ```bash
  sudo rm -f /dev/shm/ovcam_frames /dev/shm/yolo_shm \
             /dev/shm/sem.ovcam_ready /dev/shm/sem.yolo_ready 2>/dev/null || true
  ```
  -- left-eye and yolo paths only. `/dev/shm/ovcam_frames_right` and
  `/dev/shm/sem.ovcam_ready_right` are never named.
- The "wait for shm to appear before proceeding" gate (line ~701-707) and
  the post-launch `chmod 666` permissions fixup (line ~713) are likewise
  left-eye-only.
- The writer's own destructor (`sim_camera_bridge_node.cpp`, both eyes)
  does correctly `shm_unlink()`/`sem_unlink()` on a clean exit -- so this
  gap only bites when the writer does NOT get a clean shutdown (crash,
  `docker stop` grace period expiring into a hard kill, etc). But when it
  does, the right-eye objects have **no fallback cleanup at all**, while
  the left eye has two independent layers (its own destructor, plus this
  script's explicit `rm -f` backup).

**Live reproduction, not inferred**: checked `/dev/shm/` right now, with
zero stack running (`sudo docker ps` empty, the previously-diagnosed PIDs
gone). Result:
```
-rw-r--r--  1 root root  1843584 Sep  3 23:35 ovcam_frames_right
-rw-r--r--  1 root root       32 Sep  3 23:35 sem.ovcam_ready_right
```
`ovcam_frames`, `yolo_shm`, `sem.ovcam_ready`, `sem.yolo_ready` are all
absent -- the left-eye/yolo cleanup ran and worked. The right-eye objects
are the only ones left behind, sitting there indefinitely, exactly matching
the missing-cleanup-code finding above. This means any new run's right-eye
reader can attach to this exact leftover pair before the new run's writer
gets around to replacing it -- no sub-second race required, just whichever
of "reader's retry loop runs" or "writer's recreate runs" happens first,
against a target that's been sitting there the whole time.

**Ruled out: cross-process permissions.** Considered whether the reader
might be silently failing on `sem_timedwait()` due to a permissions
mismatch (root-owned files, requested mode 0666 but actually created as
644 per the umask -- confirmed via the `ls -la` above) rather than an
orphaned-mapping issue. Checked the launch file
(`hil_simulation.launch.py`): both `ovcam_bridge` and `ovcam_bridge_right`
are declared as `Node(...)` entries inside the same `if bridge_nodes:`
block of the same launch file as their matching writers -- meaning left
and right run in the identical process/user context as each other. Since
the left eye demonstrably works under that same context, a permissions
explanation would have to fail both eyes equally, not just the right one.
Ruled out as the differentiator; the `(deleted)` mapping remains the
direct, unambiguous evidence, and the missing-cleanup-code finding above
explains why it happens.

Why there are zero warnings anywhere: the reader's two warning-emitting
code paths ("consumer too slow, dropped frame" and "seqlock retry
exhausted") both live inside `publish_frame()`, which is only ever called
from `for fn = last_fn+1 to latest`. If the reader is stuck on an orphaned
mapping, `latest` (read via `ws_load()`) never advances from whatever value
it was frozen at the moment it got orphaned, so that loop body never
executes at all -- not a code path that was reached and suppressed, a code
path that was never reached.

## Diff

TODO-AP implemented in `run_stack_hil.sh` (874 lines total), three sites:

**1. Explicit `stop` command's cleanup (~line 464-476):**
```diff
     sudo pkill -f yolo_producer 2>/dev/null || true
-    sudo rm -f /dev/shm/ovcam_frames /dev/shm/yolo_shm \
-               /dev/shm/sem.ovcam_ready /dev/shm/sem.yolo_ready 2>/dev/null || true
+    # Right-eye paths included unconditionally -- rm -f on a nonexistent path
+    # is a silent no-op, so this is safe even for mono runs. Left-eye-only
+    # cleanup here was the root cause of FIX-024 (see docs/fixlog/024-...).
+    sudo rm -f /dev/shm/ovcam_frames /dev/shm/yolo_shm /dev/shm/ovcam_frames_right \
+               /dev/shm/sem.ovcam_ready /dev/shm/sem.yolo_ready /dev/shm/sem.ovcam_ready_right \
+               2>/dev/null || true
```

**2. Pre-launch "clean stale state" cleanup (~line 640-649):** identical
addition, same two paths, in the second (separately maintained, un-factored
-- matching this file's existing convention) copy of the same cleanup logic.

**3. Right-eye readiness gate, new, gated by `$STEREO_ON` (~line 719-733):**
```bash
if [[ "${STEREO_ON:-false}" == "true" ]]; then
    log "Waiting for /dev/shm/ovcam_frames_right to appear..."
    for i in $(seq 1 20); do
        [[ -e /dev/shm/ovcam_frames_right ]] && break
        sleep 0.5
    done
    [[ -e /dev/shm/ovcam_frames_right ]] \
        || die "sim_camera_bridge_right did not create /dev/shm/ovcam_frames_right — check /tmp/hil_launch.log"
    log "Right-eye SHM created."
fi
```
No `chmod 666` equivalent added: confirmed via the launch file's own
comments that `yolo_producer` (the only host-side, non-root consumer
needing that permissions fix) reads only the left ring by design --
detection stays monocular, never touches the right-eye shm.

Deployed to the Pi, `bash -n` clean both locally and on the Pi before and
after deployment.

## Critic verdict & concerns

**open_concern.** Independently re-verified all three diff sites live on
the Pi, confirmed `STEREO_ON` is never actually unset (`parse_stack.py`
always emits an explicit `true`/`false`, checked against both a mono and
the stereo config), confirmed no new mid-write race was introduced (the
blocking `docker stop`/`docker rm` calls complete before either `rm -f`
line runs, for both eyes symmetrically), and confirmed `--resume-fsm`
doesn't create a gap (it's mutually exclusive with the cleanup-skipping
path in a way that still leaves the right-eye shm fresh before a hold).

**The real gap**: the original fix write-up dismissed "writer crash during
a live run" as not needing a fix (TODO-AM) because the container isn't
`--restart`-policy. The critic checked this directly and found it doesn't
hold up -- `sim_camera_bridge_right` is a `ros2 launch`-spawned child
process with no `respawn=True` and no `OnProcessExit` handler in
`hil_simulation.launch.py`. If that one node dies mid-run (segfault,
OOM-kill -- plausible on a Pi 5 under stereo load) while the rest of the
container keeps running, the right-eye shm is left existing-but-abandoned,
and this fix's new readiness gate only checks *existence* once at launch,
not *liveness* during the run -- so it would not catch this. This is a
distinct failure mode from the one actually diagnosed and confirmed this
session (stale shm surviving a stop/restart boundary, confirmed via the
`/proc/<pid>/maps` "(deleted)" evidence) -- so it does not make this fix
wrong for what it targets, but the "mid-run crashes essentially don't
happen" claim was unverified and should not have been asserted as fact.
Logged as TODO-AQ below rather than folded into this fix's scope.

## KISS verdict

**ok.** Independently confirmed no simpler correct alternative exists:
checked all four function definitions in this 874-line script for an
existing "wait for a shm path" helper -- none exists, the left-eye wait
loop is itself already inline, un-factored, duplicated logic, matching how
the two `rm -f` cleanup blocks are already two separate copies of each
other in this same file. Inventing a shared helper now would be the actual
style violation; duplicating inline is this file's established convention.
Also refined the framing of the diff itself: the two `rm -f` additions
(sites 1 and 2) are what actually close the confirmed race (verified by
reading the writer/reader code directly -- the writer is idempotent against
stale files, the reader never re-opens after its first successful open);
the readiness gate (site 3) is a fail-fast diagnostic addition, not itself
load-bearing for the fix -- worth describing as "2 required + 1 defensive,"
not three equally necessary sites.

## Verification

Root-cause evidence gathered (all direct, none assumed):
1. `/proc/<right-reader PID>/maps` re-checked at diagnosis time: still shows
   `(deleted)`, right now, not historical.
2. `ps -o pid,lstart,cmd` for both right-eye processes: identical start
   second -- consistent with, does not prove, the race window.
3. `sim_camera_bridge_node.cpp` read directly: confirmed unconditional
   `shm_unlink()` + `shm_open(O_CREAT)` on every startup.
4. Raw shared-memory `write_seq` sampled directly (bypassing ROS entirely)
   on both `/dev/shm/ovcam_frames` and `/dev/shm/ovcam_frames_right`:
   both climbing steadily and in sync -- proves frames genuinely reach the
   Pi via shared memory on both eyes, isolating the failure to the
   reader-side mapping, not the producer.

5. Whole-script grep for `_right` combined with every shm/sem operation
   (cleanup, readiness-wait, chmod): confirmed zero matches -- the right
   eye has no cleanup code path at all, not just an under-tested one.
6. Live check of `/dev/shm/` with no stack running: right-eye objects
   persisted from the last run (timestamps unchanged since that run ended);
   left-eye and yolo objects were already gone. Direct, current-state proof
   of the missing-cleanup theory, not an inference from code alone.
7. Considered and ruled out a cross-process permissions explanation (see
   above) -- left and right run in the identical user/container context via
   the same launch file, so permissions can't be what makes only right fail.

Not yet done: proving the sub-second interleaving directly for the case
where a stale object does NOT already exist (no tool used so far has
sub-second process-start resolution); reproducing the same "(deleted)"
state on the LEFT eye on a run where left loses the race instead (would
confirm the code-level symmetry empirically -- lower priority now that the
missing-cleanup gap explains why right is exposed far more often in
practice than left).

### Post-fix verification (TODO-AP)

**Direct reproduction of the fix closing the exact confirmed mechanism**,
not assumed: seeded fake stale right-eye files on the Pi (`sudo touch
/dev/shm/ovcam_frames_right /dev/shm/sem.ovcam_ready_right`), simulating
precisely the "writer didn't get a clean shutdown" scenario this bug
depends on. Confirmed present via `ls -la`. Ran `./run_stack_hil.sh stop`.
Re-checked `/dev/shm/` afterward: both files gone, no `ovcam` entries
remain at all. This is the exact same check that originally caught the bug
live (`ls -la /dev/shm/ | grep -i ovcam`), now showing the opposite result.

**End-to-end confirmation, done**: user ran a real MATLAB-fed stereo session
(`bags/run_ov2slam_stereo_20260904_115958`, mode=record, git_sha=6e7bf48,
154.8s, 2420 total messages). `ros2 bag info` on the actual recording:
```
Topic: /slam/pose | Type: geometry_msgs/msg/PoseStamped | Count: 66
Topic: /tf        | Type: tf2_msgs/msg/TFMessage         | Count: 66
```
**66 `/slam/pose` messages** -- every prior recorded run this session showed
zero. Played the bag back and sampled real position values directly (not
just the count):
```
x: 0.00048691738321782245  y: -0.00022966701202708708  z: -0.0031069732749898164
x: 0.00048661247299074546  y: -0.00022877763094691355  z: -0.003108286972409334
...
```
Finite, small-magnitude, smoothly varying (consistent with SLAM
initializing near the run's starting pose) -- no NaN, no divergence, no
frozen/repeated values. This is the original symptom (`zero /slam/pose
messages, every run`) directly reversed, with real recorded data, not an
inference from code or a synthetic test. `pgrep ovcam_bridge_right` found
no live process at check time (the run had already ended and the stack was
torn down) -- so the `/proc/<pid>/maps` clean-mapping check from the
original plan was not additionally performed live, but is now lower value:
the actual downstream symptom (real pose output) is the thing that
mattered, and it's confirmed directly.

Status moved to `resolved` on this basis. TODO-AQ (mid-run crash liveness
gap) remains open as documented above -- untouched by this verification,
not blocking.

## Open TODOs

- **TODO-AP**: ~~RESOLVED~~ implemented and directly verified (see Post-fix
  verification above) -- the confirmed stop/restart-boundary mechanism is
  closed. End-to-end SLAM-pose confirmation still pending a live run
  (not a gap in this TODO, just not yet exercised).
- **TODO-AQ** (new, from critic review, active): the readiness gate added
  by TODO-AP checks shm *existence* once at launch, not *liveness* during
  the run. `sim_camera_bridge_right` has no `respawn`/`OnProcessExit`
  handling in `hil_simulation.launch.py` -- confirmed by the critic agent,
  not assumed -- so a genuine mid-run crash of that one node (segfault,
  OOM-kill) would leave the container looking healthy while the right-eye
  shm silently goes stale, uncaught by anything currently in place. Add a
  liveness check during the run (e.g. periodically confirm
  `/ovcam/right/image_raw` is still publishing, or re-check `/proc/<sim_
  camera_bridge_right PID>/maps` on an interval) if this is ever actually
  observed in practice -- not implemented now since it's unconfirmed
  whether it happens at all, only that nothing currently catches it if it
  did.
- **TODO-AL** (fast, partial mitigation, superseded by TODO-AP for the
  confirmed mechanism -- kept for reference): delay starting each relay
  (`ovcam_bridge` / `ovcam_bridge_right`) by 1-2s after its matching
  picture-writer starts -- same pattern already used for SLAM's own startup
  delay elsewhere in this stack. Shrinks the race window a lot, does not
  close it, and does nothing for a writer that restarts independently
  mid-run while its reader keeps running.
- **TODO-AM** (real fix, both eyes): make the relay self-heal -- detect that
  its mapped shared-memory segment has been orphaned/replaced (e.g. compare
  the underlying object identity, or notice `write_seq` hasn't advanced for
  far longer than the expected frame interval) and automatically re-open
  the current object. Handles the startup race AND any future writer
  restart. More code, needs real testing.
- **TODO-AN** (alternative/complementary): sequence the launch so each
  picture-writer is fully up and has already published its shared-memory
  object before its relay is allowed to start looking for it at all.
  Removes the race for normal startup; does not protect against a writer
  restarting later while the relay keeps running (TODO-AM still needed for
  that case).
- **TODO-AO**: whichever fix is chosen, apply it to BOTH `sim_camera_bridge`
  /`ovcam_bridge` (left) and `sim_camera_bridge_right`/`ovcam_bridge_right`
  (right) -- the vulnerability is symmetric, left only looked fine because
  it won the coin flip this run.

**Status as of this update**: TODO-AP is implemented, deployed, and
directly verified for the confirmed stop/restart mechanism -- see Post-fix
verification above. Remaining, not yet implemented: TODO-AQ (mid-run
liveness gap, found by critic review, not yet confirmed to happen in
practice), TODO-AM (self-healing reconnect -- would also close TODO-AQ's
gap as a side effect if implemented), TODO-AL/AN (superseded in priority by
AP for the mechanism that was actually confirmed), TODO-AO (apply to left
eye too -- lower priority since left has not exhibited this bug, code-level
symmetry only).

This entry's `status: partial` reflects: the diagnosed and confirmed bug is
fixed and directly verified; full end-to-end confirmation (a live run
producing real `/slam/pose` output) is the one remaining step, and it
requires a MATLAB-fed session this agent cannot drive alone.
