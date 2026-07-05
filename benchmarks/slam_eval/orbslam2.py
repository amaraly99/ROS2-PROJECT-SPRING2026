#!/usr/bin/env python3
# orbslam2.py — ORB-SLAM2 evaluator. Everything real lives in base.py; this only
# declares what's ORB-SLAM2-specific. See base.py's header for how to add a backend.
from .base import SlamEvaluator


class Orbslam2Evaluator(SlamEvaluator):
    slam_type = "orbslam2"
    label = "ORB-SLAM2"
    color = "#6a4c93"                       # paper palette (generate_paper_images.py)
    frontend_csv = "timing_events.csv"      # written by ScopedBenchmarkTimer (ORB_BENCH_TIMING_CSV)
    # frontend_category inherits base's "frontend/full_tracking"

    # Logical thread-role order from the offline study's cpu_per_thread plot.
    _ROLE_ORDER = ["ORBGBA", "ORBLocalMap", "ORBFrontEnd", "ORBLoopClose",
                   "ORBExtractL", "ORBExtractR"]

    def thread_role_order(self, names):
        # Match the offline study's cpu_per_thread plot: ORB* logical roles only,
        # in fixed order — drop DDS/runtime threads. Absent roles (e.g. ORBGBA if
        # no global BA fired during a short flight) simply don't appear.
        return [r for r in self._ROLE_ORDER if r in names]
