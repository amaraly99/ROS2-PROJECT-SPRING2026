"""pi_state.py: core lists, the Pi state read, process helpers, PiSettings with defaults."""
import subprocess
import unittest

from hilbench import pi_state

from ._fakes import make_settings, no_external_commands


class ParseCores(unittest.TestCase):
    def test_list_and_range(self):
        """in: parse_cores('0-2,3'), ('3'), ('2,3'), ('0-0')
        exp: {0,1,2,3}, {3}, {2,3}, {0}
        bug: a range is read as its two ends, or 'a-b' stops at b-1."""
        self.assertEqual(pi_state.parse_cores("0-2,3"), {0, 1, 2, 3})
        self.assertEqual(pi_state.parse_cores("3"), {3})
        self.assertEqual(pi_state.parse_cores("2,3"), {2, 3})
        self.assertEqual(pi_state.parse_cores("0-0"), {0})


class PiState(unittest.TestCase):
    def test_memory_keys_from_proc_meminfo(self):
        """in: pi_state() on this Pi (reads the real /proc/meminfo)
        exp: mem_total_mb > 0, mem_available_mb > 0, mem_available_mb <= mem_total_mb, swap_used_mb >= 0
        bug: MemAvailable is read in kB but reported as MB (a 1000 x error)."""
        st = pi_state.pi_state()
        self.assertGreater(st["mem_total_mb"], 0)
        self.assertGreater(st["mem_available_mb"], 0)
        self.assertLessEqual(st["mem_available_mb"], st["mem_total_mb"])
        self.assertGreaterEqual(st["swap_used_mb"], 0)
        self.assertLess(st["mem_total_mb"], 1024 * 1024)       # below 1 TB: the unit is MB

    def test_old_keys_still_present(self):
        """in: pi_state()
        exp: keys cpu_temp_c and throttled exist (value may be None)
        bug: the new memory code drops the old temperature keys that meta.txt expects."""
        st = pi_state.pi_state()
        self.assertIn("cpu_temp_c", st)
        self.assertIn("throttled", st)

    def test_net_buffer_table(self):
        """in: HIL_NET_BUFFERS
        exp: 7 entries; net.core.rmem_max == 16777216; net.ipv4.ipfrag_time == 3
        bug: the sysctl values differ from run_stack_hil.sh."""
        self.assertEqual(len(pi_state.HIL_NET_BUFFERS), 7)
        self.assertEqual(pi_state.HIL_NET_BUFFERS["net.core.rmem_max"], 16777216)
        self.assertEqual(pi_state.HIL_NET_BUFFERS["net.ipv4.ipfrag_time"], 3)


class ProcessHelpers(unittest.TestCase):
    def test_unknown_process(self):
        """in: affinity_of and process_memory for 'no_such_exe_zz'
        exp: 'not found' and None
        bug: a missing process raises instead of being recorded as 'not found'."""
        self.assertEqual(pi_state.affinity_of("no_such_exe_zz"), "not found")
        self.assertIsNone(pi_state.process_memory("no_such_exe_zz"))

    def test_running_process(self):
        """in: a 'sleep 20' child process
        exp: affinity_of('sleep') is a core list text (not 'not found'); process_memory has rss_mb > 0 and hwm_mb >= rss_mb
        bug: VmRSS is read from the wrong line, or the HWM peak is lower than the current size."""
        p = subprocess.Popen(["sleep", "20"])
        self.addCleanup(p.wait)
        self.addCleanup(p.kill)
        import time
        time.sleep(0.2)
        self.assertNotEqual(pi_state.affinity_of("sleep"), "not found")
        mem = pi_state.process_memory("sleep")
        self.assertGreater(mem["rss_mb"], 0)
        self.assertGreaterEqual(mem["hwm_mb"], mem["rss_mb"] - 0.01)


class PiSettingsDefaults(unittest.TestCase):
    def test_option_a_changes_nothing(self):
        """in: PiSettings(make_settings()) with cores none/none, governor leave, buffers leave; apply()
        exp: {} and no sudo, sysctl or docker command is started
        bug: Option A (all 'leave') still changes the Pi or calls sudo."""
        with no_external_commands() as ext:
            applied = pi_state.PiSettings(make_settings()).apply()
            pi_state.PiSettings(make_settings()).restore()
        self.assertEqual(applied, {})
        self.assertEqual(ext.blocked, [])

    def test_snapshot_is_a_dict(self):
        """in: PiSettings(make_settings()).snapshot()
        exp: a dict
        bug: session_settings.json cannot store the Pi state."""
        self.assertIsInstance(pi_state.PiSettings(make_settings()).snapshot(), dict)


if __name__ == "__main__":
    unittest.main()
