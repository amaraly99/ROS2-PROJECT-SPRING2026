# PAPER_IMAGES_HIL

Standalone, reviewable draft of the SLAM ablation section plus everything it
cites. Nothing here is typed by hand: every number in the tables and figures is
computed from the recorded bags.

## Layout

    section_hil_slam.tex   the draft subsection (\input-able, or compile alone)
    generate.py            regenerates ALL tables + figures from bags/
    tables/*.tex           \input-able LaTeX fragments
    figures/*.pdf          \includegraphics-able
    data/provenance.json   exactly which bag produced which row

## Regenerating

    python3 PAPER_IMAGES_HIL/generate.py

Re-run after any new trials. It rereads bags/, recomputes the statistics and
overwrites tables/ and figures/. Do not edit numbers in the .tex fragments --
they will be silently overwritten.

Arms are declared by bag glob at the top of generate.py, so the set of runs
behind the paper is explicit and auditable rather than implied.

## Definitions actually used

    APE RMSE   Sim(3)-aligned absolute pose error; scale is ESTIMATED, never
               assumed, because monocular scale is unobservable.
    epsilon    APE RMSE / path length, x100. Dimensionless (metres/metres), so
               the unit is %, NOT %/m -- arms scored over slightly different
               path lengths stay comparable. Same convention as KITTI.
    window     mission start -> arrival, IDENTICAL for every arm. The
               initialisation leg is excluded for all backends: windowing on
               each backend's own first pose makes the scored path length a
               function of when it initialised, which shortens the ruler for the
               slowest initialiser.
    delta_s    scale fitted independently on each half of the window. Detects a
               map being progressively rescaled -- a failure a single global
               scale cannot express.

## Known gaps in this draft

- Offline EuRoC subsection is an outline only. Numbers deliberately omitted
  rather than guessed; populate from results/ov2slam_benchmark_statistical/,
  ORBSLAM3_ROS2/results/orbslam_benchmark/ and Latest_FAST_Test/.
- ORB-SLAM3 has no front-end latency figure. Its wrapper accumulates per-frame
  timings in memory and writes only at shutdown, so there is no per-frame CSV to
  read. Shown as an em-dash rather than a fabricated value.
- OV2SLAM-fast initialises at 10.75 m against a 3.0 m initialisation leg, so it
  enters the mission without a map and is scored over ~26 m where the others get
  ~33 m. Its row is reported but should be treated as provisional until it
  either gets an initialisation cycle that works or is excluded.
