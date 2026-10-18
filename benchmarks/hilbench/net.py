"""net.py — Wi-Fi: the two live readings, the check on both machines, and the 1 Hz receive-rate recorder."""
import csv
import re
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path

from .util import now_ns, run


def pct_from_dbm(dbm):
    """Windows WLAN_SIGNAL_QUALITY scale: -100 dBm -> 0 %, -50 dBm and better -> 100 %."""
    return max(0.0, min(100.0, 2.0 * (dbm + 100.0)))


def wifi_pi(iface="wlan0"):
    """Live link from `iw dev <iface> link`: (ssid, dBm, %)."""
    p = run(["iw", "dev", iface, "link"], timeout=5)
    ssid = re.search(r"SSID: (.+)", p.stdout)
    sig = re.search(r"signal: (-?\d+) dBm", p.stdout)
    if not (ssid and sig):
        return None, None, None
    dbm = float(sig.group(1))
    return ssid.group(1).strip(), dbm, pct_from_dbm(dbm)


def wifi_windows(ssh_target):
    """Live link from `netsh wlan show interfaces` over SSH: (ssid, %, dBm or None, error text)."""
    p = run(["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=6", ssh_target,
             "netsh wlan show interfaces"], timeout=15)
    if p.returncode != 0:
        return None, None, None, (p.stderr.strip().splitlines() or ["ssh failed"])[-1]
    ssid = re.search(r"^\s*SSID\s*:\s*(.+)$", p.stdout, re.M)
    sigp = re.search(r"^\s*Signal\s*:\s*(\d+)\s*%", p.stdout, re.M)
    rssi = re.search(r"^\s*Rssi\s*:\s*(-?\d+)", p.stdout, re.M)
    if not (ssid and sigp):
        return None, None, None, "netsh gave no SSID/Signal (wired, or Wi-Fi off?)"
    return ssid.group(1).strip(), float(sigp.group(1)), (float(rssi.group(1)) if rssi else None), ""


@dataclass
class LinkReport:
    ok: bool
    lines: dict = field(default_factory=dict)    # name -> (ok, text): the screen shows these
    info: dict = field(default_factory=dict)     # facts for meta.txt


def check_links(settings):
    """Pi link, and the Windows link unless check_windows_wifi is false. ok = both >= wifi_min_pct."""
    lines, info = {}, {}
    ssid, dbm, pct = wifi_pi()
    pi_ok = pct is not None and pct >= settings.wifi_min_pct
    lines["Pi Wi-Fi"] = (pi_ok, f"{ssid or '?'} · {dbm if dbm is not None else '?'} dBm · "
                               f"{pct if pct is not None else '?'} % (iw, live link)")
    info.update({"wifi_pi_ssid": ssid, "wifi_pi_dbm": dbm, "wifi_pi_pct": pct})
    if not settings.check_windows_wifi:
        lines["Windows Wi-Fi"] = (None, "not checked (check_windows_wifi is false)")
        return LinkReport(pi_ok, lines, info)
    wssid, wpct, wrssi, err = wifi_windows(f"{settings.windows_user}@{settings.host}")
    win_ok = wpct is not None and wpct >= settings.wifi_min_pct
    if wpct is not None:
        txt = f"{wssid} · {wpct:.0f} %" + (f" · {wrssi:.0f} dBm" if wrssi is not None else "") + " (netsh)"
    else:
        txt = f"not readable: {err}"
    lines["Windows Wi-Fi"] = (win_ok, txt)
    info.update({"wifi_win_ssid": wssid, "wifi_win_pct": wpct, "wifi_win_rssi_dbm": wrssi})
    return LinkReport(pi_ok and win_ok, lines, info)


class WifiSampler(threading.Thread):
    """1 Hz receive rate of the Pi's Wi-Fi into a csv file."""

    def __init__(self, path, iface="wlan0"):
        super().__init__(daemon=True)
        self.path, self.iface = Path(path), iface
        self.stop_evt = threading.Event()
        self.rates = deque(maxlen=60)
        self.error = None

    def _rx(self):
        return int(Path(f"/sys/class/net/{self.iface}/statistics/rx_bytes").read_text())

    def run(self):
        try:
            self._run()
        except Exception as e:          # never die silently: the screen shows rx "?" and the run logs this
            self.error = f"{e.__class__.__name__}: {e}"

    def _run(self):
        with open(self.path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t_wall_ns", "rx_MBps", "signal_dbm"])
            last_t, last_b = time.monotonic(), self._rx()
            while not self.stop_evt.wait(1.0):
                t, b = time.monotonic(), self._rx()
                rate = (b - last_b) / max(t - last_t, 1e-6) / 1e6
                last_t, last_b = t, b
                _, dbm, _ = wifi_pi(self.iface)
                self.rates.append(rate)
                w.writerow([now_ns(), f"{rate:.4f}", "" if dbm is None else int(dbm)])
                f.flush()

    def stop(self):
        self.stop_evt.set()
        if self.is_alive():
            self.join(timeout=3)
