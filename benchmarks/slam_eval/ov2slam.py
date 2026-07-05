#!/usr/bin/env python3
# ov2slam.py — OV2SLAM evaluator. Everything real lives in base.py; this only
# declares what's OV2SLAM-specific: its timing file, and its fast/accurate mode
# (label + colour), inferred from the stack config name in meta.txt.
from .base import SlamEvaluator


class Ov2slamEvaluator(SlamEvaluator):
    slam_type = "ov2slam"
    frontend_csv = "ov2slam_timing_events.csv"   # written by Profiler::LogEvent (OV2_BENCH_TIMING_CSV)
    # frontend_category inherits base's "frontend/full_tracking"

    # The offline study benchmarked fast vs accurate as two separate algorithms;
    # colours match generate_paper_images.py's LEGACY_DATA.
    _MODE = {
        "fast":     ("OV2SLAM (fast)", "#7fb069"),
        "accurate": ("OV2SLAM (accurate)", "#2f6db3"),
    }

    def __init__(self, rundir):
        super().__init__(rundir)
        label_color = self._MODE.get(self._detect_mode(), ("OV2SLAM", "#2f6db3"))
        self.label, self.color = label_color

    def _detect_mode(self) -> str:
        # e.g. config=ov2slam_oracle_fast → "fast"; ov2slam_oracle_accurate → "accurate".
        cfg = self.meta_val("config").lower()
        if "fast" in cfg:
            return "fast"
        if "accurate" in cfg:
            return "accurate"
        return "unknown"
