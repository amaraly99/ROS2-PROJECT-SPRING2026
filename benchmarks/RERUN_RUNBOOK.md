# Fresh-Batch Re-Run Runbook — 2026-07-12

Single internally-consistent dataset under the fixed parser + fixed harness.
Supersedes detector_sweep_2026-06-25 and the recovered-artifact values as the
paper source (June data stays on disk as archive only).

## Unified settings
- warmup 5 s (fast configs) — warm-up is only a guard delay before the latency
  probe; it is NOT part of any measurement. Slow configs get a longer guard
  (CPU-medium 15 s, OWL-ViT 30 s) purely so model-load doesn't trip the
  /cmd_vel startup check (window = warmup + 45 s). No metric is affected.
- data window 50 s (fast), 90 s (CPU-medium, ≥25 detections/run at ~0.3 Hz),
  180 s (OWL-ViT, ~27 detections at ~0.15 Hz, same as June).
- 3 runs per config; conf 0.20; slam fast; fresh MATLAB per chunk.

## What the clocks measure (Ahmed's timing question)
- Mission clock t0 = mission_monitor start = the moment camera frames flow
  (immediately after the drone reset). time_to_reached is measured from there;
  approach_duration (first APPROACHING → REACHED) is anchor-independent.
- The "collecting Ns of data" window is the latency probe: per-detection
  cam→det→cmd latencies. These are content-independent, valid in any mission
  phase. With 5 s warmup the window now covers approach + reach.
- The mission monitor runs ~(warmup+45+duration+60) s — reach is captured even
  if it lands after the probe window ends (matters for ~2 Hz CPU-nano configs).

## Output dirs
- Detector (20 configs): benchmarks/results/detector_sweep_2026-07-12/
- Placement C/D:         benchmarks/results/placement_2026-07-12/
- Placement A ≡ yolo26n_npu_fast, B ≡ yolo26n_cpu_fast (identical config+affinity;
  make_paper_tables.py defaults already point there). tab:latency_results
  derives from the same yolo26n_npu_fast runs. No duplicate flights.

## Chunk 0 — PILOT GATE (start of MATLAB session 1)
    cd ~/ROS2-PROJECT-SPRING2026
    ./benchmarks/run_hil_experiments.sh --detector yolo --model yolov8n --backend npu \
        --slam-mode fast --conf 0.20 --duration 50 --warmup 5 --runs 3 \
        --label yolov8n_npu_fast --output-dir benchmarks/results/detector_sweep_2026-07-12

Gate check (all 3 runs):
    for N in 1 2 3; do python3 -c "
import json;d=json.load(open('benchmarks/results/detector_sweep_2026-07-12/yolov8n_npu_fast_run$N''_mission.json'))
print('run$N', d.get('initial_state'), 'reach:', d['time_to_reached_s'], 'approach:', d['approach_duration_s'])"; done
    python3 -c "import json;d=json.load(open('benchmarks/results/detector_sweep_2026-07-12/yolov8n_npu_fast_summary.json'));print('cmd_vel_hz',d['cmd_vel_hz'])"

PASS = initial_state SEARCHING (or None), no REACHED before ~40 s, reach ≈ 45–52 s,
approach ≈ 38–44 s, cmd_vel ≈ 16.1 Hz. If reach > ~55 s: bump --duration to 60
for ALL remaining fast configs. If STARTUP_FAILED or cmd_vel unstable: bump
--warmup to 10 and re-pilot. Otherwise continue chunk 1 in the SAME session.

## Chunks (fresh MATLAB per chunk; Ctrl-C between configs is always safe —
## re-run any config that lacks a _summary.json)

Chunk 1 (same session as pilot, ~+20 min): yolo26n/s/m NPU
Chunk 2 (~26 min): yolov11n/s/m NPU + yolov8s NPU
Chunk 3 (~26 min): yolov8m NPU + opencv + placement C + placement D
Chunk 4 (~26 min): yolo26n/v11n/v8n CPU + yolo26s CPU
Chunk 5 (~22 min): yolov11s/v8s CPU + yolo26m CPU (90 s window)
Chunk 6 (~32 min): yolov11m/v8m CPU (90 s) + OWL-ViT (180 s; Ctrl-C before it
                   and mini-session it if MATLAB is dragging)

Slow-config note: CPU-medium and OWL-ViT never close the loop (detection-rate
arithmetic); their latency numbers are content-independent, so June values
could be reused if skipped — default is re-fly (chunks 5–6).

## Final build (no MATLAB)
    cd ~/ROS2-PROJECT-SPRING2026
    python3 benchmarks/plot_detector_sweep.py benchmarks/results/detector_sweep_2026-07-12
    python3 benchmarks/recompute_mission_metrics.py --sweep benchmarks/results/detector_sweep_2026-07-12
    python3 benchmarks/make_paper_tables.py \
        --results-dir benchmarks/results/detector_sweep_2026-07-12 \
        --results-dir benchmarks/results/placement_2026-07-12 \
        --centroid-states APPROACHING,REACHED \
        --out-dir benchmarks/results/paper_tables_2026-07-12

recompute_mission_metrics doubles as the batch audit: with the fixed harness it
must report zero RECOVERED/artifact rows; SUSPECT still catches MOG2
false-positive reaches. detector_comparison_fixed.csv = SINGLE SOURCE OF TRUTH.

## Hand to the Windows-side Claude
- detector_sweep_2026-07-12/detector_comparison_fixed.csv (tab:detector_e2e)
- detector_sweep_2026-07-12/mission_audit.csv (provenance)
- paper_tables_2026-07-12/table_1a_latency.tex, table_1b_perstage.tex,
  table_placement.tex (tab:latency_results, per-stage, tab:placement)
- benchmarks/results/paper_data_handoff.md (July 12 section)
Instructions: regenerate every table cell from these files only; "—" +
"did not reach within the N s window" for non-reaches; reach framing =
actuation-limited (~47–50 s for every loop-closing config); drop all
June-derived numbers and the 16–47 s spread; keep the baked-0.20-threshold
limitation paragraph; footnote placement-B centroid if approach-only.
