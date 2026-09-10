# Paper data — detector sweep & placement study

Everything the paper's tables and figures are built from, plus the raw per-run logs needed to
regenerate them. Dataset date **2026-07-13**; it supersedes all earlier sweeps, whose reach times
are invalid (see [Known artifacts](#known-artifacts-in-earlier-data)).

Model binaries are not in this repo — run `bash models/fetch_models.sh` first if you want to
re-run the experiments rather than just re-derive the tables.

## Layout

```
paper_data/
  detector_comparison.csv     aggregated: 20 detector configs, one row each
  detector_sweep/             20 configs x 3 runs — per-run logs + per-config summaries
  placement/                  configs C and D x 3 runs (A and B come from the sweep)
  tables/                     LaTeX fragments used verbatim in the paper
  figures/                    the three sweep figures
```

Per run (`{label}_run{N}…`), six files matter:

| File | Contents |
|---|---|
| `_telemetry.csv` | per-frame producer stamps S1–S7 (acquire → preprocess → infer → NMS → SHM write) |
| `{prefix}.csv` | e2e probe: detection publish (S7–S8), control (S9), `/cmd_vel` arrivals |
| `_stages.csv` | per-run stage summary |
| `_cam_stamps.csv` | pairs the MATLAB sim clock with the Pi's monotonic clock — this is what makes sim-time possible |
| `_visp.csv` | per-frame servo state + centroid error (source of mission state and Centroid-RMS) |
| `_mission.json` | derived mission outcome; `_stats.json`, `{label}_summary.json` aggregate it |

Raw camera-timing CSVs and stack logs are not included; nothing in the published pipeline reads them.

## Regenerating the tables

From the repo root, using only what is in this directory:

```bash
python3 benchmarks/make_paper_tables.py \
    --results-dir benchmarks/paper_data/detector_sweep \
    --results-dir benchmarks/paper_data/placement \
    --out-dir /tmp/tables
diff -r /tmp/tables benchmarks/paper_data/tables      # must be empty
```

That diff being empty is the guarantee: the committed tables are exactly what the committed scripts
produce from the committed data. To rebuild `detector_comparison.csv` and the figures instead:

```bash
python3 benchmarks/mission_from_visp_csv.py  --sweep benchmarks/paper_data/detector_sweep
python3 benchmarks/plot_detector_sweep.py           benchmarks/paper_data/detector_sweep
```

## Reading the numbers

**Mission time is in simulation time, never wall-clock.** The `time_to_reached_s_wall` column exists
for transparency only and must not be compared across configs. MATLAB's simulation rate drifts
between 0.78× and 0.95× of real time from run to run, which stretched *identical* missions from
43.7 s to 54.8 s of wall-clock and manufactured a ~10 s "detector effect" that does not exist. The
sim clock resets to zero at each run, so `time_to_reached_sim_s` is a drift-free mission clock;
`sim_rate` reports the drift per config.

**Reach is actuation-limited, not detection-limited.** In sim time every detector that closes the
loop arrives in ~40–42 s, whether it runs at 16.1 Hz or 1.3 Hz — a 12× spread in detection rate with
no effect on mission time. What actually matters is whether the loop closes at all, and there is a
hard viability threshold near 1 Hz: at ≥1.3 Hz every config reaches 3/3, at ≤0.51 Hz none ever does.

**`/cmd_vel` tracks the detection rate 1:1** — the controller emits one command per detection, so
control authority is throttled directly by detector throughput.

**MOG2's reach time is not meaningful.** It "arrives" in 22 s, but it is a background-subtraction
motion detector: its box is a motion blob, not the stop sign, so its bbox ratio swings wildly and
trips the distance-based termination without the drone ever approaching. Its rate (16.1 Hz) and
latency (14.9 ms — genuinely the cheapest) are real; its reach is reported as N/A. A detector can be
fast and still be useless for closed-loop servoing.

**`yolov11s_cpu` has elevated reach variance** (44.7 ± 3.9 s): one run briefly lost the sign
mid-approach and re-acquired. Real low-rate tracking fragility, not an artifact.

**Centroid RMS is measured during APPROACHING only.** Pooling every servo state instead makes the
metric a function of how much of the fixed recording window each run spent *parked at the goal*:
once REACHED, the drone is stationary and centred, so its error collapses to ~10–18 px, and
REACHED's share of the samples varies a lot by config (23% of samples for the 16 Hz NPU run versus
44% for the 4.3 Hz CPU run, which arrives with more of its window left). That ratio reflects run
length versus reach time, not tracking quality, and it silently flatters whichever config idled at
the target longest. `make_paper_tables.py` therefore defaults to `--centroid-states APPROACHING`.

**Centroid RMS does not resolve these configurations at n=3, and no claim should rest on it.**
An earlier revision of this file read the config means as showing that a higher command rate does
not buy better tracking (A at 16.16 Hz, 114.4 px, against B at 4.33 Hz, 96.7 px). Recomputing per
run shows the spread within a single configuration is as large as any difference between them:

| Cfg | per-run RMS (px) | approach duration (s) | samples |
|---|---|---|---|
| A | 118.3 / 120.7 / 104.2 | 54.8 / 55.1 / 52.7 | ~850 |
| B | 91.3 / 107.3 / 91.3 | 42.7 / 42.7 / 43.5 | ~172 |
| C | 79.5 / 99.9 / 117.3 | 42.2 / 42.0 / 44.2 | ~673 |
| D | 119.9 / 115.7 / 111.2 | 45.3 / 45.5 / 47.3 | ~714 |

Config C alone spans 38 px and fully overlaps A's range. Note also that B is sampled ~5x more
sparsely (it runs at 4.33 Hz), so its RMS is a coarser estimate of the same trajectory, and B's
approach is *shorter* in wall-clock than A's — so the tempting explanation that a slower command
rate flatters the metric is not supported either. The honest statement is that three runs cannot
separate these configurations, which is what the paper now says.

The reach result is unaffected and stands on its own: mission time is actuation-limited, and a 12x
spread in detection rate produces no measurable difference in time to target.

## Known artifacts in earlier data

Fixed in this dataset; the reason older numbers differ and must not be carried forward.

1. **Watchdog reset.** At CPU detection rates the 0.4 s "target lost" watchdog fired between every
   pair of detections and reset the lock-on and reach counters, so CPU missions could never
   complete. Every pre-2026-07-13 CPU mission is invalid.
2. **Stale-scene false REACHED.** The simulator streams the previous run's final frame (drone parked
   at the sign) until the model resets, latching a bogus REACHED at ~0.2 s in 7 of 12 NPU runs.
   Fixed with a "must have seen the sign from far away" guard.
3. **Sim-rate drift.** The reason wall-clock reach times are unusable at all.

Every run here passed a false-reach gate and a completeness gate after those fixes: 20/20 detector
configs present, n=3 each, 0 SUSPECT, 0 frozen-camera flags (`*/mission_audit.csv`).
