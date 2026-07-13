# OV²SLAM stability fixes (2026-03-01)

Four patches to upstream OV²SLAM that fixed NaN/Inf crashes and initialization failures on the live
mono camera. Recorded here because they are non-obvious and easy to reintroduce.

| Fix | File | Root cause |
|---|---|---|
| **A — Timestamp truncation** | `src/ov2slam_node.cpp` | The mono path used only `stamp.sec` and discarded the nanoseconds, so 97.4% of consecutive frame pairs had `dt = 0` → division by zero downstream. |
| **B — Motion model** | `include/visual_front_end.hpp` | `MotionModel` divided by `dt` with no guard for `dt ≈ 0`. Added a `dt` threshold and `allFinite()` checks. |
| **C — Pose validation** | `src/frame.cpp` | `setTwc()` accepted NaN/Inf poses unchecked, poisoning the whole map. Added `allFinite()` and a `det(R) ≈ 1` guard. |
| **D — Topic subscription** | `src/ov2slam_node.cpp` | The left topic was hardcoded, and the right-topic subscription crashed in mono mode when the topic string was empty. |

A and B are the same underlying bug seen from two sides: a zero `dt` that should never have been
possible. If SLAM starts producing NaN poses again, check the timestamp path first.
