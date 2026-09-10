# Contention decomposition — which channel couples the detector to its co-tenants?

**Run:** 2026-09-11 · `benchmarks/contention_decomposition.py --models yolo26n yolo26m --reps 3`
**Data:** `contention_decomposition.csv` (one row per model × condition × repetition)

## Why

The detector sweep shows a CPU-resident detector losing 20–34 % of its standalone
throughput inside the closed loop. The draft attributed this to the detector and
the SLAM solver "competing for the Cortex-A76 cores". That is not what the stack
does: `run_stack_hil.sh` pins the detector to cores 0–1 and OV²SLAM to cores 2–3,
so the two never contend for a core. Something else couples them, and this
isolates what.

Synthetic co-tenants are used instead of the real stack so the experiment needs no
simulator, no SLAM build and no wired-LAN rig — it runs on the Pi alone.

| Condition | Detector | Cores 2–3 |
|---|---|---|
| `unpinned` | free on all four cores | idle |
| `idle` | pinned to 0–1 | idle |
| `cpu` | pinned to 0–1 | two ALU spinners, ~2 KB working set (fits L1) |
| `mem` | pinned to 0–1 | two streaming loads, 64 MB buffers (≫ 2 MB shared L3) |

## Results (mean ± std over three repetitions, inference stage)

| model | condition | mean (ms) | std | vs `idle` |
|---|---|---|---|---|
| yolo26n | unpinned | 165.8 | 0.7 | −0.1 % |
| yolo26n | idle | 166.0 | 0.6 | — |
| yolo26n | cpu | 171.2 | 1.8 | **+3.1 %** |
| yolo26n | mem | 589.7 | 23.9 | **+255 %** |
| yolo26m | unpinned | 1427.2 | 7.1 | −0.1 % |
| yolo26m | idle | 1428.0 | 4.3 | — |
| yolo26m | cpu | 1682.4 | 44.4 | **+17.8 %** |
| yolo26m | mem | 4934.9 | 31.8 | **+246 %** |

## What this settles

**1. Core-affinity confinement costs nothing.** Restricting the detector from four
cores to two, with no co-tenant, changes inference time by −0.1 % on both models.
The 20–34 % in-loop loss is therefore not an artifact of how the stack pins
processes; it is real contention.

*(An earlier single-repetition probe suggested affinity cost 6–16 %. It did not
reproduce at n=3 with 40 frames. Treat the single-run figure as noise.)*

**2. The coupling runs through the memory path, not through core occupancy.** A
co-tenant that saturates the memory controller from cores the detector does not
use slows it by a factor of ~3.5, while a co-tenant that occupies the same cores
with L1-resident arithmetic costs 3.1 % (nano). Two orders of magnitude separate
the two channels. This is consistent with the memory-bandwidth constraint reported
for other edge NPU platforms (Kong et al., *Sci. Rep.* 2026).

## What this does NOT settle

**The `cpu` condition is not a clean control.** Its inner loop uses NumPy
expressions that allocate temporaries, so it is not purely ALU-bound; and driving
all four cores raises die temperature, so some of the +3.1 % / +17.8 % may be
frequency scaling rather than resource sharing. The `cpu`-condition numbers should
be read as "small" rather than as a measured quantity.

**The working-set explanation for the medium-tier step is not demonstrated.** The
detector sweep shows the in-loop loss stepping up from 22.7–26.0 % (nano/small) to
27.3–32.1 % (medium), which invites the explanation that a larger weight footprint
is more exposed to a contended cache. Under a *saturating* memory co-tenant both
scales slow by nearly the same factor (255 % vs 246 %), so that hypothesis is
neither confirmed nor refuted here — saturation is a different regime from SLAM's
moderate, bursty traffic. The paper reports the step without attributing it.

## Reproduce

```bash
python3 benchmarks/contention_decomposition.py --models yolo26n yolo26m --reps 3
```

~25 minutes. No root, no simulator, no SLAM.
