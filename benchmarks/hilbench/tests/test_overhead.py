"""overhead.py: PaceGuard and OverheadMeter."""
import unittest

from hilbench.overhead import OverheadMeter, PaceGuard


class PaceGuardTests(unittest.TestCase):
    def test_degraded_below_ratio(self):
        """in: run 1 hb 16.17 Hz, run 2 hb 13.0 Hz, mode stop, limit 0.85
        exp: check() -> ('degraded', non-empty text); history[-1] is (2, 13.0, ~0.804)
        bug: the 20 percent loss of pace is not caught and the session goes on."""
        g = PaceGuard("stop", 0.85)
        g.record(1, 16.17)
        g.record(2, 13.0)
        state, text = g.check()
        self.assertEqual(state, "degraded")
        self.assertTrue(text)
        self.assertEqual(g.baseline_hz, 16.17)
        k, hb, ratio = g.history[-1]
        self.assertEqual((k, hb), (2, 13.0))
        self.assertAlmostEqual(ratio, 13.0 / 16.17, places=6)

    def test_ok_above_ratio(self):
        """in: run 1 hb 16.17, run 2 hb 15.0 (ratio 0.928)
        exp: check() -> 'ok'
        bug: a small loss of pace stops the session."""
        g = PaceGuard("stop", 0.85)
        g.record(1, 16.17)
        g.record(2, 15.0)
        self.assertEqual(g.check()[0], "ok")

    def test_limit_is_ratio_of_run_one(self):
        """in: run 1 hb 16.0; run 2 hb 13.8 (0.8625); run 3 hb 13.5 (0.844)
        exp: ok after run 2, degraded after run 3
        bug: the limit uses the previous run, or the wrong side of 0.85."""
        g = PaceGuard("stop", 0.85)
        g.record(1, 16.0)
        g.record(2, 13.8)
        self.assertEqual(g.check()[0], "ok")
        g.record(3, 13.5)
        self.assertEqual(g.check()[0], "degraded")

    def test_first_run_is_ok(self):
        """in: only run 1 recorded
        exp: 'ok'; baseline_hz == hb of run 1; history [(1, hb, 1.0)]
        bug: run 1 is compared with itself and fails on rounding."""
        g = PaceGuard("stop", 0.85)
        g.record(1, 16.17)
        self.assertEqual(g.check()[0], "ok")
        self.assertEqual(g.history, [(1, 16.17, 1.0)])

    def test_mode_off_never_degrades(self):
        """in: mode off, run 1 hb 16.17, run 2 hb 1.0
        exp: 'ok'
        bug: mode off still stops the session."""
        g = PaceGuard("off", 0.85)
        g.record(1, 16.17)
        g.record(2, 1.0)
        self.assertEqual(g.check()[0], "ok")

    def test_mode_warn_reports_degraded(self):
        """in: mode warn, run 1 hb 16.17, run 2 hb 10.0
        exp: 'degraded' (the Session decides to go on)
        bug: the guard hides the drop in warn mode so the deviation is never logged."""
        g = PaceGuard("warn", 0.85)
        g.record(1, 16.17)
        g.record(2, 10.0)
        self.assertEqual(g.check()[0], "degraded")

    def test_none_heartbeat_never_degrades(self):
        """in: run 1 hb 16.17, run 2 hb None (kind has no heartbeat figure)
        exp: 'ok'; history keeps the entry for run 2
        bug: None raises TypeError in the division."""
        g = PaceGuard("stop", 0.85)
        g.record(1, 16.17)
        g.record(2, None)
        self.assertEqual(g.check()[0], "ok")
        self.assertEqual(g.history[-1][0], 2)


class OverheadMeterTests(unittest.TestCase):
    def test_result_keys(self):
        """in: OverheadMeter(status_counter=lambda: 144); sample(); result()
        exp: keys flight_wall_s, runner_cpu_pct_core, runner_status_calls, pi_cpu_total_pct, runner_rss_mb,
             runner_hwm_mb, pi_mem_available_min_mb; runner_status_calls == 144
        bug: a figure that meta.txt and the summary need is missing, or the status counter is not read."""
        m = OverheadMeter(status_counter=lambda: 144)
        m.sample()
        r = m.result()
        for key in ("flight_wall_s", "runner_cpu_pct_core", "runner_status_calls", "pi_cpu_total_pct",
                    "runner_rss_mb", "runner_hwm_mb", "pi_mem_available_min_mb"):
            self.assertIn(key, r)
        self.assertEqual(r["runner_status_calls"], 144)

    def test_memory_figures_are_positive(self):
        """in: sample(); result() on this Pi
        exp: pi_mem_available_min_mb > 0; runner_rss_mb > 0; runner_hwm_mb >= runner_rss_mb
        bug: the lowest MemAvailable stays at its start value (inf or 0) because sample() does nothing."""
        m = OverheadMeter()
        m.sample()
        r = m.result()
        self.assertGreater(r["pi_mem_available_min_mb"], 0)
        self.assertGreater(r["runner_rss_mb"], 0)
        self.assertGreaterEqual(r["runner_hwm_mb"], r["runner_rss_mb"] - 0.01)

    def test_without_counter_status_calls_is_zero(self):
        """in: OverheadMeter() with no status_counter
        exp: runner_status_calls == 0
        bug: a missing counter raises TypeError at result()."""
        self.assertEqual(OverheadMeter().result()["runner_status_calls"], 0)


if __name__ == "__main__":
    unittest.main()
