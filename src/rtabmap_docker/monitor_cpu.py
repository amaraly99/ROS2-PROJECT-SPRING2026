#!/usr/bin/env python3

import csv
import os
import signal
import sys
import time
from pathlib import Path


TARGET_PROCESSES = {"stereo_odometry", "rtabmap"}
CLK_TCK = None
STOP = False


def _read_text(path: Path) -> str | None:
    try:
        return path.read_text().strip()
    except Exception:
        return None


def _thread_times(stat_text: str) -> int:
    # /proc/<pid>/task/<tid>/stat format: the process name is wrapped in
    # parentheses and may contain spaces, so parse from the trailing fields.
    end = stat_text.rfind(")")
    fields = stat_text[end + 2 :].split()
    return int(fields[11]) + int(fields[12])


def _iter_threads():
    proc_root = Path("/proc")
    for pid_dir in proc_root.iterdir():
        if not pid_dir.name.isdigit():
            continue
        comm = _read_text(pid_dir / "comm")
        if comm not in TARGET_PROCESSES:
            continue

        pid = int(pid_dir.name)
        task_dir = pid_dir / "task"
        try:
            tids = sorted(task_dir.iterdir(), key=lambda p: int(p.name))
        except Exception:
            continue

        for tid_dir in tids:
            stat_text = _read_text(tid_dir / "stat")
            thread_name = _read_text(tid_dir / "comm")
            if not stat_text or not thread_name:
                continue
            try:
                total_ticks = _thread_times(stat_text)
            except Exception:
                continue
            yield {
                "process_name": comm,
                "pid": pid,
                "tid": int(tid_dir.name),
                "thread_name": thread_name,
                "ticks": total_ticks,
            }


def _handle_stop(signum, frame):
    global STOP
    STOP = True


def main() -> int:
    global CLK_TCK

    if len(sys.argv) < 2:
        print("usage: monitor_cpu.py OUT_CSV [INTERVAL_S]", file=sys.stderr)
        return 2

    out_csv = Path(sys.argv[1])
    interval = float(sys.argv[2]) if len(sys.argv) > 2 else 1.0
    if interval <= 0:
        raise ValueError("interval must be > 0")

    out_csv.parent.mkdir(parents=True, exist_ok=True)
    CLK_TCK = os.sysconf("SC_CLK_TCK")

    signal.signal(signal.SIGINT, _handle_stop)
    signal.signal(signal.SIGTERM, _handle_stop)

    previous: dict[tuple[int, int], int] = {}
    previous_time = time.monotonic()

    with out_csv.open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["wall_ts", "process_name", "pid", "tid", "thread_name", "cpu_pct"])
        f.flush()

        while not STOP:
            time.sleep(interval)
            now = time.monotonic()
            elapsed = max(now - previous_time, 1e-9)
            wall_ts = time.time()

            current = list(_iter_threads())
            for row in current:
                key = (row["pid"], row["tid"])
                if key not in previous:
                    continue
                delta_ticks = row["ticks"] - previous[key]
                cpu_pct = max(delta_ticks / CLK_TCK / elapsed * 100.0, 0.0)
                writer.writerow(
                    [
                        f"{wall_ts:.3f}",
                        row["process_name"],
                        row["pid"],
                        row["tid"],
                        row["thread_name"],
                        f"{cpu_pct:.2f}",
                    ]
                )

            previous = {(row["pid"], row["tid"]): row["ticks"] for row in current}
            previous_time = now
            f.flush()

    print(f"monitor_cpu: wrote {out_csv}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
