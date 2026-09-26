---
id: FIX-026
title: run_matrix.py's "verify SLAM initialised" check accepts the FIRST real reading as final -- no patience for a signal that ramps up gradually (stereo)
date: 2026-09-04
status: resolved  # deployed, reviewed, verified live end-to-end
component: scripts/hil_matrix/run_matrix.py (run_trial(), "verify SLAM initialised" step)
supersedes: none
critic_verdict: open_concern (round 1, real footgun found in the config surface) -> ok in effect after collapsing to a single field per KISS's own suggestion
kiss_verdict: open_concern (round 1, same footgun, plus a simpler alternative proposed) -> adopted directly
open_todos: []
---

## Symptom

After FIX-025 was deployed and rebuilt, a live automated stereo trial still
aborted with `tracking_state=1` ("not ready"). Direct evidence ruled out
FIX-025 itself as the cause: the SLAM sidecar's own console log from that
exact trial showed the real 3D-point count (`nb3dkps_`, the value FIX-025's
threshold checks) genuinely climbing past the required threshold
(0 -> 1 -> 3 -> 7 -> 13 -> 50) and staying above it for the rest of the run.
FIX-025's logic was proven correct. The trial still failed.

## Root cause

Confirmed directly from source: the "verify SLAM initialised" step's retry
loop only retried when the topic read came back completely EMPTY (query
flakiness, the echo's window closing before a message landed). The moment
it read ANY real number -- including a legitimate `1` meaning "not ready
yet" -- it broke immediately and accepted that as the trial's final answer,
after at most ~15-19s of total elapsed time (one read + up to 3 retries,
3s apart). It was never designed to distinguish "genuinely stuck" from
"still ramping up" -- a distinction mono/ORB-SLAM3 apparently never needed,
since their own tracking-state signal (per this project's own contract)
becomes valid quickly. Stereo's own point count (via FIX-025's new path)
climbs far more gradually, and the real trial's log showed it took noticeably
longer than the ~19s the old loop allowed before giving up.

## Diff

`scripts/hil_matrix/run_matrix.py`, the "verify SLAM initialised" step,
final (post-review) form:

```python
        state = None
        arm_rmw = rmw_for(arm.get("dds", "cyclonedds"))
        patience_sec = arm.get("init_patience_sec", 9)
        t0 = time.time()
        s = 0
        while True:
            s += 1
            raw = pi("timeout 15 sudo docker exec ros2_perception_stack bash -lc "
                     "'source /opt/ros/jazzy/setup.bash 2>/dev/null; "
                     f"export RMW_IMPLEMENTATION={arm_rmw}; "
                     "timeout 10 ros2 topic echo /slam/tracking_state --once "
                     "--field data 2>/dev/null' 2>/dev/null", check=False,
                     dry_value="2")
            mt = re.search(r"(\d+)", raw or "")
            if mt:
                state = mt.group(1)
                if state == "2":
                    break
                LOG(f"      tracking_state read {s} = {state} (not ready yet) -- retrying")
            else:
                LOG(f"      tracking_state read {s} returned nothing -- retrying")
            # Same time budget covers BOTH a real "not ready" reading and an
            # empty one -- an empty reading must not be able to loop forever
            # just because it never happens to hit the state=="2" break above.
            if (time.time() - t0) >= patience_sec:
                break
            nap(3)
```

New per-arm field: `init_patience_sec` (default `9`, matching the old fixed
shape of one read + 3 retries at 3s apart -- so every existing matrix config
that doesn't set it is unaffected). Set to `60` on the new
`only_ov2_stereo_oracle.yaml` config, comfortably past the observed
threshold-crossing point with real margin.

### First draft, and what review changed about it

The first version added three separate fields (`slow_init` bool,
`init_check_retries`, `init_check_interval_sec`), gated so the retry-on-
real-but-not-ready behavior only activated if `slow_init: true` was set.
Both review agents independently found the same real footgun: setting
`slow_init: true` alone, without also remembering to set the other two
numbers, would silently fall back to the old ~9s budget -- doing almost
nothing, contrary to what the flag name implies. The simplification agent's
concrete recommendation -- collapse to one field, a total wall-clock
patience budget in seconds, whose presence/value alone determines the
behavior -- was adopted directly, replacing the three-field design shown
above with the single `init_patience_sec` field.

While rewriting to the collapsed design, caught and fixed a second real bug
in my own first draft before it ever reached review: the empty-read branch
had no time bound at all in the first version of the collapsed loop --
a persistently empty reading could have looped forever. Fixed by applying
the same `patience_sec` budget check after BOTH branches (empty or real),
not just inside the real-reading branch.

## Critic verdict & concerns

**open_concern** on the first (three-field) draft. Verified directly: byte-
for-byte identical control flow for every arm that doesn't opt in (traced
all three `state` cases -- None, "1", "2" -- against the original code);
no key-name collisions in any existing matrix config; no arm-level
allow-list in `load_config()` that would reject the new optional keys; no
external wall-clock ceiling the longer loop could blow through. The one
real objection: the fix was technically correct but incomplete as staged --
the one config that actually needed it (`only_ov2_stereo_oracle.yaml`) had
not yet been updated to set the new field, so the reported symptom would
still reproduce on the very next run without that follow-up. Addressed by
adding the field to that config in the same pass, before calling this done.

## KISS verdict

**open_concern**, converged independently on the same footgun (three knobs,
only one of which -- `slow_init` -- looks like the real toggle, but doing
nothing without the other two). Proposed the single-field collapse
described above, including working replacement code, which was adopted
directly rather than re-reviewed as a separate round -- the suggested code
was correct on inspection and directly closed the exact concern raised.
Also independently confirmed: making the longer retry the universal default
(no opt-in at all) would have a real cost, not a theoretical one --
`only_ov2_fast.yaml` runs 10 trials each *designed* to hit this exact retry
loop and fail it every time (its own header comment: "EXPECTED to abort
every trial... That abort is the finding, not a harness failure"). An
unconditional default would add real, multi-minute wall-clock cost to that
config's normal use. The opt-in mechanism itself was judged justified;
only the number of knobs implementing it was the objection.

## Verification

- `bash`/Python compile check clean (`py -m py_compile`), both after the
  first draft and after the collapse to the single-field design.
- Isolated logic check: default (`init_patience_sec` absent) case traced
  by hand for all three `state` outcomes, confirmed unchanged from the
  original loop's behavior.
- Real dry run (`--dry-run --trials 1` on `only_ov2_stereo_oracle`): config
  resolves cleanly, no parse errors, the intended commands print correctly.
- **Real live trial** (the actual test this fix exists for): ran end to
  end. `SLAM initialised during init cycle: YES (tracking_state=2)` --
  first time this ever happened for stereo, in this session or any prior
  one. The mission then actually flew (`PHASE 1 mission`, 22.2s), the stack
  drained and stopped cleanly, and the trial validated with **86 non-zero
  SLAM poses** recorded. This is the real, end-to-end proof the whole
  chain of fixes this session (FIX-024, FIX-025, FIX-026, plus the
  set_stereo/remap gaps found along the way) was aiming for.

## Open TODOs

None new from this fix specifically. Separately noted (not part of this
fix): the `init_patience_sec: 60` value on the stereo config is a generous
estimate from one observed run, not a formally measured minimum -- may need
tuning if a different scene or condition makes SLAM ramp up even more
slowly. Not tracked as a numbered TODO since it's a tuning question, not a
known defect.
