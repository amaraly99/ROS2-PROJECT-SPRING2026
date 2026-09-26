---
id: FIX-023
title: new `record` mode -- scout behavior + bag recording, without benchmark's CPU/thread profiling samplers
date: 2026-09-03
status: resolved
component: run_stack_hil.sh
supersedes: none
critic_verdict: wrong_fix (round 1) -> ok (round 2, after fix)
kiss_verdict: open_concern (round 1, same gap as critic, structure otherwise confirmed simplest) -> ok (round 2)
open_todos: [TODO-AK]
---

## Symptom / request

To check whether SLAM's pose output is sane, the direct way is recording a
bag (/slam/pose + /sim/drone_pose side by side). Bag recording only exists
gated behind `MODE == "benchmark"`, which also drags in CPU/thread usage
profiling samplers the user explicitly did not want turned on for this
("record SHOULD skip the CPU/thread tracking"). Requested: a third mode,
`record`, behaving like `scout` plus recording only.

## Investigation

Grepped every `MODE` reference in `run_stack_hil.sh` (7 total, not
assumed): per-run results-dir creation (~96), SLAM console-log persistence
(~197/238 -- already handles non-benchmark modes correctly since FIX-021,
untouched here), CPU/thread samplers (~247-262 -- must stay benchmark-only,
untouched), the actual `start_bag_recording()` gate (~296), the MODE
validator (~388-390), and the final run-summary text (two separate
`if`-blocks, ~826 and ~830 -- not one block as first assumed). Confirmed
via the same grep that `MODE` no longer controls FSM/controller startup
timing anywhere (decoupled in an earlier fix) -- `record` needs no special
handling there, it's genuinely scout's existing behavior plus recording.

## Diff

`run_stack_hil.sh`:
- Added `record` to the MODE validator, alongside `benchmark`/`scout`.
- Added one shared boolean, computed once right after MODE is finalized,
  matching this file's own existing convention for `SLAM_ENABLED`/
  `DEBUG_IMAGE_ON`:
  ```bash
  RECORDING_ON=false
  [[ "$MODE" == "benchmark" || "$MODE" == "record" ]] && RECORDING_ON=true
  ```
- Switched the results-dir-creation gate (~96), `start_bag_recording()`'s
  gate (~296), and the recording section of the final summary (~830) from
  checking `$MODE == "benchmark"` directly to checking `$RECORDING_ON`.
- CPU/thread sampler block (~247-262): completely untouched, still checks
  `$MODE == "benchmark"` specifically -- record mode never triggers it.
- Fixed two literal-string bugs surfaced by review (see below): `meta.txt`'s
  `echo "mode=benchmark"` -> `echo "mode=${MODE}"`, and the summary's
  hardcoded `"Recording (benchmark mode):"` label -> `"Recording
  (mode=${MODE}):"`.

## Critic verdict & concerns

Round 1: **wrong_fix**. Found a real bug the original diff missed: line
~104, inside the SAME block being re-gated, has a hardcoded literal `echo
"mode=benchmark"` written into `meta.txt` -- not tied to `$MODE` at all.
Left as-is, a `record`-mode run would produce `meta.txt` claiming
`mode=benchmark` while `run_config.yaml` (written two lines later, same
block, correctly using `mode: ${MODE}`) would say `mode: record` -- two
metadata files for the same run contradicting each other on the one field
that identifies what kind of run it was. Checked downstream
(`benchmarks/slam_eval/*.py`): nothing currently reads/filters on `mode=`
from `meta.txt`, so nothing breaks *today* -- but the file itself would
become a landmine for any future script or person trusting it, especially
since the code's own comment calls `meta.txt` an "eval compat shim."
Round 2 (after fixing the literal): **ok**.

## KISS verdict

Round 1: **open_concern**, independently found the identical `meta.txt` gap
plus one more (the summary's `"benchmark mode"` label text). Otherwise
confirmed the structural choices as already-simplest, not overcomplicated:
a named `RECORDING_ON` boolean matches this file's own established idiom
(`SLAM_ENABLED`, `DEBUG_IMAGE_ON` are the same pattern) rather than
repeating an inline `||` condition at 4 separate call sites; `record` as a
new MODE value (not a separate orthogonal `--record` flag layered on top)
is correct because the file's own comment already documents that MODE's
whole current purpose *is* recording control, nothing else -- a second flag
would add a second axis on top of one that's already single-purpose.
Round 2: **ok**.

## Verification

Ran the real thing end to end, all three things checked with real output:
- `./run_stack_hil.sh --config ov2slam_stereo --mode record`: no "unknown
  mode" die, log line reads `Recording (mode=record) →
  bags/run_ov2slam_stereo_.../bag` (not "benchmark").
- `meta.txt` and `run_config.yaml` in that same run directory: `mode=record`
  and `mode: record` respectively -- agree with each other now, the exact
  contradiction the critic predicted is gone.
- `/tmp/slam_cpu_sampler.pid` and `/tmp/slam_thread_sampler.pid`: neither
  exists after the run -- confirmed the CPU/thread samplers genuinely did
  not start, not just assumed from reading the untouched gate.
- Stopped cleanly via `./run_stack_hil.sh stop`.

## Open TODOs

- TODO-AK: noticed, not touched (out of scope for this request): the final
  run-summary block has a THIRD `if [[ "$MODE" == "benchmark" ]]` (~line
  826, inside the SLAM_ENABLED section) printing
  `${RUNREL}/slam_sidecar.log` as the persisted-log path -- this is stale
  relative to FIX-021, which gave scout (and now record) mode a real
  persisted log too, just at a different path (`/tmp/slam_sidecar_<name>
  .log`, not `${RUNREL}/...`). FIX-021 only updated the inline
  "SLAM sidecar started" message, not this separate end-of-run summary
  echo -- so right now this summary block still tells scout/record users
  nothing about where their SLAM log actually is. Small, easy fix, but a
  distinct pre-existing gap from a different fix; flagged rather than
  folded in here to keep this change scoped to what was actually asked for.
