"""net.py: percent from dBm, the two Wi-Fi readers (commands mocked), the sampler."""
import subprocess
import tempfile
import unittest
from pathlib import Path
from unittest import mock

from hilbench import net

IW_OK = """Connected to aa:bb:cc:dd:ee:ff (on wlan0)
\tSSID: {ssid}
\tfreq: 5180
\tRX: 123456 bytes (1000 packets)
\tsignal: {dbm} dBm
\ttx bitrate: 866.7 MBit/s
"""

NETSH_OK = """    Name                   : Wi-Fi
    State                  : connected
    SSID                   : Yasser5G
    BSSID                  : aa:bb:cc:dd:ee:ff
    Network type           : Infrastructure
    Signal                 : 99%
    Rssi                   : -27
"""


def fake_run(stdout="", returncode=0, stderr=""):
    def run(cmd, *a, **k):
        return subprocess.CompletedProcess(cmd, returncode, stdout=stdout, stderr=stderr)
    return run


class PctFromDbm(unittest.TestCase):
    def test_values(self):
        """in: pct_from_dbm(-55), (-30), (-120), (-100), (-75)
        exp: 90.0, 100.0 (clamp), 0.0 (clamp), 0.0, 50.0
        bug: the Windows scale 2 x (dBm + 100) is wrong, or the clamp is missing."""
        self.assertEqual(net.pct_from_dbm(-55), 90.0)
        self.assertEqual(net.pct_from_dbm(-30), 100.0)
        self.assertEqual(net.pct_from_dbm(-120), 0.0)
        self.assertEqual(net.pct_from_dbm(-100), 0.0)
        self.assertEqual(net.pct_from_dbm(-75), 50.0)


class WifiPi(unittest.TestCase):
    def test_parses_ssid_dbm_pct(self):
        """in: iw output with SSID 'Yasser5G' and signal -62 dBm
        exp: ('Yasser5G', -62.0, 76.0)
        bug: the regex reads the wrong line, or the percent is not computed."""
        with mock.patch("subprocess.run", fake_run(IW_OK.format(ssid="Yasser5G", dbm=-62))):
            self.assertEqual(net.wifi_pi(), ("Yasser5G", -62.0, 76.0))

    def test_ssid_with_spaces(self):
        """in: iw output with SSID 'My Home 5G'
        exp: ssid 'My Home 5G'
        bug: the SSID is cut at the first space."""
        with mock.patch("subprocess.run", fake_run(IW_OK.format(ssid="My Home 5G", dbm=-50))):
            self.assertEqual(net.wifi_pi()[0], "My Home 5G")

    def test_not_connected(self):
        """in: iw output 'Not connected.'
        exp: (None, None, None)
        bug: a missing link raises an exception instead of failing the check."""
        with mock.patch("subprocess.run", fake_run("Not connected.\n")):
            self.assertEqual(net.wifi_pi(), (None, None, None))


class WifiWindows(unittest.TestCase):
    def test_parses_netsh(self):
        """in: netsh output (SSID line, then BSSID line, Signal 99%, Rssi -27)
        exp: ('Yasser5G', 99.0, -27.0, falsy error)
        bug: the BSSID line is read as the SSID, or the percent sign breaks the number."""
        with mock.patch("subprocess.run", fake_run(NETSH_OK)):
            ssid, pct, dbm, err = net.wifi_windows("homie@192.168.1.201")
        self.assertEqual((ssid, pct, dbm), ("Yasser5G", 99.0, -27.0))
        self.assertFalse(err)

    def test_ssh_failure_gives_error_text(self):
        """in: ssh returns code 255 and stderr 'Connection timed out'
        exp: ssid None, pct None, dbm None, error text contains 'timed out'
        bug: an ssh failure is read as a good link or crashes."""
        with mock.patch("subprocess.run", fake_run("", 255, "ssh: Connection timed out\n")):
            ssid, pct, dbm, err = net.wifi_windows("homie@x")
        self.assertEqual((ssid, pct, dbm), (None, None, None))
        self.assertIn("timed out", err)

    def test_wired_windows_gives_error(self):
        """in: netsh prints 'There is 1 interface ... not connected' (no SSID, no Signal)
        exp: pct None and a non-empty error text
        bug: a wired PC passes the Wi-Fi gate."""
        with mock.patch("subprocess.run", fake_run("There is 1 interface on the system:\n    State : disconnected\n")):
            _, pct, _, err = net.wifi_windows("homie@x")
        self.assertIsNone(pct)
        self.assertTrue(err)


class CheckLinks(unittest.TestCase):
    @staticmethod
    def settings(**over):
        from ._fakes import make_settings
        return make_settings(**over)

    def test_ok_when_both_links_good(self):
        """in: Pi 100 %, Windows 99 %, limit 90
        exp: report.ok True; info has the Pi ssid and the Windows percent
        bug: the gate fails on a good link."""
        with mock.patch.object(net, "wifi_pi", return_value=("S", -40.0, 100.0)), \
                mock.patch.object(net, "wifi_windows", return_value=("S", 99.0, -27.0, "")):
            rep = net.check_links(self.settings())
        self.assertTrue(rep.ok)
        self.assertIn("S", [v for v in rep.info.values() if isinstance(v, str)])
        self.assertIn(99.0, rep.info.values())

    def test_fails_when_windows_below_limit(self):
        """in: Pi 100 %, Windows 80 %, limit 90
        exp: report.ok False
        bug: only the Pi link is checked."""
        with mock.patch.object(net, "wifi_pi", return_value=("S", -40.0, 100.0)), \
                mock.patch.object(net, "wifi_windows", return_value=("S", 80.0, -60.0, "")):
            self.assertFalse(net.check_links(self.settings()).ok)

    def test_windows_not_asked_when_check_disabled(self):
        """in: check_windows_wifi False
        exp: wifi_windows never called; ok True when the Pi link is good
        bug: the deviation switch still waits for ssh."""
        with mock.patch.object(net, "wifi_pi", return_value=("S", -40.0, 100.0)), \
                mock.patch.object(net, "wifi_windows") as win:
            rep = net.check_links(self.settings(check_windows_wifi=False))
        win.assert_not_called()
        self.assertTrue(rep.ok)

    def test_unreadable_pi_link_fails(self):
        """in: wifi_pi returns (None, None, None)
        exp: report.ok False
        bug: an unreadable link passes because None is not below the limit."""
        with mock.patch.object(net, "wifi_pi", return_value=(None, None, None)), \
                mock.patch.object(net, "wifi_windows", return_value=("S", 99.0, -27.0, "")):
            self.assertFalse(net.check_links(self.settings()).ok)


class Sampler(unittest.TestCase):
    def test_sampler_has_rates_and_error_attributes(self):
        """in: WifiSampler(path) created, not started
        exp: rates is empty with maxlen 60; error is None
        bug: the screen reads a missing attribute while the thread starts."""
        with tempfile.TemporaryDirectory() as tmp:
            s = net.WifiSampler(Path(tmp) / "wifi.csv")
            self.assertEqual(len(s.rates), 0)
            self.assertEqual(s.rates.maxlen, 60)
            self.assertIsNone(s.error)


if __name__ == "__main__":
    unittest.main()
