#!/usr/bin/env python3

from __future__ import annotations

import csv
import json
import math
import os
import pty
import re
import shlex
import signal
import statistics
import subprocess
import sys
import threading
import time
from collections import Counter, defaultdict
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any, Iterable
from zoneinfo import ZoneInfo


RESULTS_TIMEZONE = "Asia/Dubai"
EUROC_SEQUENCES = [
    "MH_01_easy",
    "MH_02_easy",
    "MH_03_medium",
    "MH_04_difficult",
    "MH_05_difficult",
    "V1_01_easy",
    "V1_02_medium",
    "V1_03_difficult",
    "V2_01_easy",
    "V2_02_medium",
]

C_RESET = "\033[0m"
C_BOLD = "\033[1m"
C_DIM = "\033[2m"
C_RED = "\033[91m"
C_GREEN = "\033[92m"
C_YELLOW = "\033[93m"
C_BLUE = "\033[94m"
C_MAGENTA = "\033[95m"
C_CYAN = "\033[96m"
C_WHITE = "\033[97m"


class Console:
    def __init__(self) -> None:
        self.log_file: Path | None = None

    def set_log_file(self, path: Path) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text("", encoding="utf-8")
        self.log_file = path

    def write(self, text: str) -> None:
        print(text, end="")
        if self.log_file is not None:
            with self.log_file.open("a", encoding="utf-8") as handle:
                handle.write(text)

    def msg(self, color: str, icon: str, label: str, detail: str = "") -> None:
        suffix = f"  {C_DIM}{detail}" if detail else ""
        self.write(f"  {icon}  {C_BOLD}{color}{label}{C_RESET}{suffix}{C_RESET}\n")

    def ok(self, label: str, detail: str = "") -> None:
        self.msg(C_GREEN, f"{C_GREEN}✓{C_RESET}", label, detail)

    def fail(self, label: str, detail: str = "") -> None:
        self.msg(C_RED, f"{C_RED}✗{C_RESET}", label, detail)

    def step(self, label: str, detail: str = "") -> None:
        self.msg(C_CYAN, f"{C_CYAN}▶{C_RESET}", label, detail)

    def warn(self, label: str, detail: str = "") -> None:
        self.msg(C_YELLOW, f"{C_YELLOW}■{C_RESET}", label, detail)

    def section(self, title: str) -> None:
        pad = max(0, 56 - len(title))
        self.write(f"\n{C_BOLD}{C_WHITE}  ── {title} {C_DIM}{'─' * pad}{C_RESET}\n")

    def banner(self, title: str) -> None:
        bar = "═" * 66
        self.write(
            f"\n{C_BOLD}{C_BLUE}╔{bar}╗\n"
            f"{C_BOLD}{C_BLUE}║  {C_WHITE}{title:<64}{C_BLUE}║\n"
            f"{C_BOLD}{C_BLUE}╚{bar}╝{C_RESET}\n"
        )

    def seq_banner(self, seq_name: str, idx: int, total: int) -> None:
        bar = "━" * 66
        self.write(
            f"\n{C_BOLD}{C_MAGENTA}  {bar}\n"
            f"  Sequence {idx}/{total}: {C_WHITE}{seq_name}{C_RESET}\n"
            f"{C_BOLD}{C_MAGENTA}  {bar}{C_RESET}\n"
        )

    def run_banner(self, run_idx: int, total: int) -> None:
        bar = "─" * 66
        self.write(
            f"\n{C_BOLD}{C_CYAN}  {bar}\n"
            f"  Run {run_idx}/{total}\n"
            f"  {bar}{C_RESET}\n"
        )


def datetime_compact() -> str:
    return datetime.now(ZoneInfo(RESULTS_TIMEZONE)).strftime("%Y%m%d_%H%M%S")


def resolve_dir(path_str: str | Path) -> Path:
    return Path(path_str).expanduser().resolve()


def resolve_file(path_str: str | Path) -> Path:
    return Path(path_str).expanduser().resolve()


def parse_scalar(value: str) -> Any:
    text = value.strip()
    if not text or text in {"null", "~"}:
        return ""
    if len(text) >= 2 and text[0] == text[-1] and text[0] in {"'", '"'}:
        return text[1:-1]
    lower = text.lower()
    if lower == "true":
        return True
    if lower == "false":
        return False
    if re.fullmatch(r"[-+]?[0-9]+", text):
        return int(text)
    if re.fullmatch(r"[-+]?[0-9]*\.[0-9]+", text):
        return float(text)
    return text


def strip_comment(text: str) -> str:
    out: list[str] = []
    in_single = False
    in_double = False
    for ch in text:
        if ch == "'" and not in_double:
            in_single = not in_single
            out.append(ch)
            continue
        if ch == '"' and not in_single:
            in_double = not in_double
            out.append(ch)
            continue
        if ch == "#" and not in_single and not in_double:
            break
        out.append(ch)
    return "".join(out).rstrip()


def split_inline_list(value: str) -> list[Any]:
    inner = value.strip()[1:-1].strip()
    if not inner:
        return []
    items: list[str] = []
    buf: list[str] = []
    in_single = False
    in_double = False
    for ch in inner:
        if ch == "'" and not in_double:
            in_single = not in_single
            buf.append(ch)
            continue
        if ch == '"' and not in_single:
            in_double = not in_double
            buf.append(ch)
            continue
        if ch == "," and not in_single and not in_double:
            items.append("".join(buf))
            buf = []
            continue
        buf.append(ch)
    if buf:
        items.append("".join(buf))
    return [parse_scalar(item) for item in items]


def load_top_level_yaml(path: Path) -> dict[str, Any]:
    try:
        import yaml  # type: ignore
    except Exception:
        yaml = None
    if yaml is not None:
        data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
        if not isinstance(data, dict):
            raise ValueError(f"{path} must contain a top-level mapping.")
        return data

    data: dict[str, Any] = {}
    current_list_key: str | None = None
    current_list: list[Any] | None = None
    for lineno, raw in enumerate(path.read_text(encoding="utf-8").splitlines(), start=1):
        line = strip_comment(raw)
        if not line.strip():
            continue
        if re.match(r"^\s*-\s+", line):
            if current_list_key is None or current_list is None:
                raise ValueError(f"{path}:{lineno}: list item without a preceding key")
            item = re.sub(r"^\s*-\s*", "", line).strip()
            current_list.append(parse_scalar(item))
            continue
        current_list_key = None
        current_list = None
        match = re.match(r"^\s*([A-Za-z0-9_]+)\s*:\s*(.*)$", line)
        if not match:
            raise ValueError(f"{path}:{lineno}: unsupported YAML syntax: {raw}")
        key, value = match.groups()
        value = value.strip()
        if value == "":
            data[key] = []
            current_list_key = key
            current_list = data[key]
            continue
        if value.startswith("[") and value.endswith("]"):
            data[key] = split_inline_list(value)
            continue
        data[key] = parse_scalar(value)
    return data


def ensure_list(value: Any) -> list[Any]:
    if value in (None, ""):
        return []
    if isinstance(value, list):
        return value
    return [value]


def coerce_bool(value: Any, default: bool = False) -> bool:
    if value in (None, ""):
        return default
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in {"1", "true", "yes", "on"}
    return bool(value)


def first_existing(paths: Iterable[Path]) -> Path | None:
    for path in paths:
        if path.is_file():
            return path
    return None


def detect_ros_setup_files(workspace_root: Path) -> list[Path]:
    files: list[Path] = []
    ros_setup = first_existing(
        Path(f"/opt/ros/{distro}/setup.bash")
        for distro in ("jazzy", "humble", "iron", "rolling", "foxy")
    )
    if ros_setup is not None:
        files.append(ros_setup)
    workspace_setup = workspace_root / "install" / "setup.bash"
    if workspace_setup.is_file():
        files.append(workspace_setup)
    return files


@dataclass
class ShellContext:
    workspace_root: Path
    setup_files: list[Path]
    env_vars: dict[str, str]
    ld_library_prepend: list[Path]

    def prefix(self) -> str:
        parts = ['source_safe(){ if [ -f "$1" ]; then set +u; . "$1"; set -u; fi; };']
        for setup in self.setup_files:
            parts.append(f"source_safe {shlex.quote(str(setup))};")
        for key, value in self.env_vars.items():
            parts.append(f"export {key}={shlex.quote(str(value))};")
        if self.ld_library_prepend:
            joined = ":".join(str(path) for path in self.ld_library_prepend)
            parts.append(f"export LD_LIBRARY_PATH={shlex.quote(joined)}:${{LD_LIBRARY_PATH:-}};")
        return " ".join(parts)

    def wrap(self, command: str) -> str:
        return f"{self.prefix()} {command}"


def run_capture(shell_ctx: ShellContext, command: str, cwd: Path | None = None) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        ["bash", "-lc", shell_ctx.wrap(command)],
        cwd=str(cwd) if cwd else None,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        check=False,
    )


def launch_background(shell_ctx: ShellContext, command: str, cwd: Path, log_path: Path) -> subprocess.Popen[str]:
    handle = log_path.open("w", encoding="utf-8")
    return subprocess.Popen(
        ["bash", "-lc", shell_ctx.wrap(f"exec {command}")],
        cwd=str(cwd),
        stdout=handle,
        stderr=subprocess.STDOUT,
        text=True,
        preexec_fn=os.setsid,
    )


def launch_background_mirrored(
    shell_ctx: ShellContext,
    command: str,
    cwd: Path,
    log_path: Path,
) -> subprocess.Popen[str]:
    master_fd, slave_fd = pty.openpty()
    process = subprocess.Popen(
        ["bash", "-lc", shell_ctx.wrap(f"exec {command}")],
        cwd=str(cwd),
        stdin=slave_fd,
        stdout=slave_fd,
        stderr=slave_fd,
        text=False,
        bufsize=0,
        preexec_fn=os.setsid,
    )
    os.close(slave_fd)

    def pump() -> None:
        with log_path.open("wb") as handle:
            while True:
                try:
                    chunk = os.read(master_fd, 4096)
                except OSError:
                    break
                if not chunk:
                    break
                sys.stdout.buffer.write(chunk)
                sys.stdout.buffer.flush()
                handle.write(chunk)
                handle.flush()
        try:
            os.close(master_fd)
        except OSError:
            pass

    threading.Thread(target=pump, daemon=True).start()
    return process


def wait_for_pid_exit(pid: int, timeout_s: float) -> bool:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if not Path(f"/proc/{pid}").exists():
            return True
        time.sleep(1.0)
    return not Path(f"/proc/{pid}").exists()


def terminate_process_group(pid: int, timeout_s: float) -> None:
    try:
        os.killpg(pid, signal.SIGINT)
    except ProcessLookupError:
        return
    if wait_for_pid_exit(pid, timeout_s):
        return
    try:
        os.killpg(pid, signal.SIGKILL)
    except ProcessLookupError:
        return
    wait_for_pid_exit(pid, 5.0)


def wait_for_ros2_node(shell_ctx: ShellContext, node_name: str, timeout_s: float) -> bool:
    expected = node_name.removeprefix("/")
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        result = run_capture(shell_ctx, "ros2 node list")
        if result.returncode == 0:
            for raw in result.stdout.splitlines():
                if raw.strip().removeprefix("/") == expected:
                    return True
        time.sleep(1.0)
    return False


def wait_for_clock_message(shell_ctx: ShellContext, timeout_s: float) -> bool:
    result = run_capture(shell_ctx, f"timeout {timeout_s}s ros2 topic echo --once /clock >/dev/null 2>&1")
    return result.returncode == 0


def publish_final_clock(shell_ctx: ShellContext, sec: int, nanosec: int) -> None:
    run_capture(
        shell_ctx,
        (
            "ros2 topic pub --once /clock rosgraph_msgs/msg/Clock "
            f"\"{{clock: {{sec: {sec}, nanosec: {nanosec}}}}}\" >/dev/null 2>&1 || true"
        ),
    )


def discover_sequences(dataset_root: Path) -> list[Path]:
    found: list[Path] = []
    for entry in sorted(dataset_root.iterdir()):
        if not entry.is_dir():
            continue
        if any(entry.glob("*.db3")):
            found.append(entry)
    return found


def select_sequences(
    discovered: list[Path],
    requested_sequences: list[str],
    seq_range_start: str,
    seq_range_end: str,
) -> list[Path]:
    by_name = {path.name: path for path in discovered}
    ordered = [path.name for path in discovered]
    if requested_sequences:
        missing = [name for name in requested_sequences if name not in by_name]
        if missing:
            raise ValueError(f"Unknown sequence(s): {', '.join(missing)}")
        return [by_name[name] for name in requested_sequences]
    if seq_range_start or seq_range_end:
        if not (seq_range_start and seq_range_end):
            raise ValueError("seq_range_start and seq_range_end must be provided together.")
        if seq_range_start not in by_name or seq_range_end not in by_name:
            raise ValueError("Sequence range endpoints were not found.")
        start_idx = ordered.index(seq_range_start)
        end_idx = ordered.index(seq_range_end)
        if start_idx > end_idx:
            raise ValueError("seq_range_start appears after seq_range_end.")
        return [by_name[name] for name in ordered[start_idx : end_idx + 1]]
    return discovered


@dataclass
class BagInfo:
    target: Path
    description: str


def resolve_bag_target(seq_dir: Path) -> BagInfo:
    db3_files = sorted(seq_dir.glob("*.db3"))
    if not db3_files:
        raise FileNotFoundError(f"No .db3 file found in {seq_dir}")
    if len(db3_files) == 1:
        desc = f"{db3_files[0].name}  (single .db3)"
        if (seq_dir / "metadata.yaml").is_file():
            desc = f"{db3_files[0].name}  (single .db3; metadata also present)"
        return BagInfo(db3_files[0], desc)
    if (seq_dir / "metadata.yaml").is_file():
        return BagInfo(seq_dir, f"{seq_dir.name}/  ({len(db3_files)} segments via metadata.yaml)")
    raise FileNotFoundError(
        f"Split bag detected in {seq_dir} but metadata.yaml is missing. Fix with: ros2 bag reindex {seq_dir}"
    )


def normalize_tum_file(src: Path, dst: Path) -> None:
    rows: list[str] = []
    for lineno, raw_line in enumerate(src.read_text(encoding="utf-8").splitlines(), start=1):
        line = raw_line.strip()
        if not line or line.startswith("#"):
            continue
        fields = line.replace(",", " ").split()
        if len(fields) != 8:
            raise ValueError(f"{src}:{lineno}: expected 8 columns, got {len(fields)}")
        for field in fields:
            float(field)
        rows.append(" ".join(fields))
    if not rows:
        raise ValueError(f"No pose rows found in {src}")
    dst.write_text("\n".join(rows) + "\n", encoding="utf-8")


def prepare_ground_truth(seq_dir: Path, out_dir: Path) -> tuple[Path, str, str]:
    seq_name = seq_dir.name
    candidate: Path | None = None
    if (seq_dir / "gt.tum").is_file():
        candidate = seq_dir / "gt.tum"
    elif (seq_dir / f"{seq_name}.txt").is_file():
        candidate = seq_dir / f"{seq_name}.txt"
    else:
        txt_candidates = [path for path in sorted(seq_dir.glob("*.txt")) if path.name != "metadata.txt"]
        if len(txt_candidates) == 1:
            candidate = txt_candidates[0]
        elif not txt_candidates:
            raise FileNotFoundError(f"No ground truth found in {seq_dir}")
        else:
            raise ValueError(f"Multiple candidate ground-truth files found in {seq_dir}")
    gt_file = out_dir / "gt.tum"
    normalize_tum_file(candidate, gt_file)
    verdict = (
        f"{candidate.name} is already valid TUM format "
        "(timestamp tx ty tz qx qy qz qw); normalized copy written to gt.tum"
    )
    return gt_file, candidate.name, verdict


def count_tum_poses(path: Path) -> int | None:
    if not path.exists():
        return None
    count = 0
    for raw_line in path.read_text(encoding="utf-8").splitlines():
        line = raw_line.strip()
        if line and not line.startswith("#"):
            count += 1
    return count


def mean_csv_column(path: Path, column: str) -> float | None:
    if not path.exists():
        return None
    values: list[float] = []
    with path.open(newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            try:
                values.append(float(row[column]))
            except Exception:
                continue
    return statistics.fmean(values) if values else None


def read_comm(path: Path) -> str:
    try:
        return path.read_text(encoding="utf-8").strip()
    except Exception:
        return ""


def read_cmdline(pid: int) -> str:
    try:
        raw = Path(f"/proc/{pid}/cmdline").read_bytes()
    except Exception:
        return ""
    return raw.replace(b"\x00", b" ").decode("utf-8", "replace").strip()


def read_exe(pid: int) -> str:
    try:
        return os.path.basename(os.readlink(f"/proc/{pid}/exe"))
    except Exception:
        return ""


def parse_stat(path: Path) -> tuple[int, int, int]:
    raw = path.read_text(encoding="utf-8").strip()
    rparen = raw.rfind(")")
    rest = raw[rparen + 2 :].split()
    utime = int(rest[11])
    stime = int(rest[12])
    processor = int(rest[36])
    ppid = int(rest[1])
    return utime + stime, processor, ppid


def pid_exists(pid: int) -> bool:
    return Path(f"/proc/{pid}").exists()


def collect_descendants(root_pid: int) -> list[int]:
    children: dict[int, list[int]] = defaultdict(list)
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit():
            continue
        try:
            _total, _cpu, ppid = parse_stat(entry / "stat")
        except Exception:
            continue
        children[ppid].append(int(entry.name))
    found: list[int] = []
    stack = [root_pid]
    seen: set[int] = set()
    while stack:
        pid = stack.pop()
        if pid in seen:
            continue
        seen.add(pid)
        for child in children.get(pid, []):
            found.append(child)
            stack.append(child)
    return found


class ProcessMonitor:
    def __init__(
        self,
        launcher_pid: int,
        out_dir: Path,
        prefix: str,
        exact_process_names: set[str],
        wrapped_cmd_tokens: set[str] | None,
        sample_sec: float,
    ) -> None:
        self.launcher_pid = launcher_pid
        self.out_dir = out_dir
        self.prefix = prefix
        self.exact_process_names = exact_process_names
        self.wrapped_cmd_tokens = wrapped_cmd_tokens or exact_process_names
        self.sample_sec = sample_sec
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self.hz = os.sysconf(os.sysconf_names["SC_CLK_TCK"])

    @property
    def info_path(self) -> Path:
        return self.out_dir / f"{self.prefix}_pid_info.json"

    @property
    def process_csv(self) -> Path:
        return self.out_dir / f"{self.prefix}_process_cpu.csv"

    @property
    def thread_csv(self) -> Path:
        return self.out_dir / f"{self.prefix}_thread_cpu.csv"

    @property
    def summary_csv(self) -> Path:
        return self.out_dir / f"{self.prefix}_thread_cpu_summary.csv"

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        self._thread.join(timeout=10.0)

    @property
    def pid_json_key(self) -> str:
        return f"{self.prefix}_pid"

    @property
    def comm_json_key(self) -> str:
        return f"{self.prefix}_comm"

    @property
    def exe_json_key(self) -> str:
        return f"{self.prefix}_exe"

    @property
    def cmdline_json_key(self) -> str:
        return f"{self.prefix}_cmdline"

    def _cmdline_has_wrapped_token(self, cmdline: str) -> bool:
        tokens = [tok for tok in cmdline.split() if tok]
        for token in tokens:
            if os.path.basename(token) in self.wrapped_cmd_tokens:
                return True
        return False

    def _is_exact_process(self, pid: int) -> bool:
        comm = read_comm(Path(f"/proc/{pid}/comm"))
        exe = read_exe(pid)
        return comm in self.exact_process_names or exe in self.exact_process_names

    def _is_wrapped_process(self, pid: int) -> bool:
        if self._is_exact_process(pid):
            return True
        return self._cmdline_has_wrapped_token(read_cmdline(pid))

    def _resolve_actual_pid(self, timeout_s: float = 30.0) -> tuple[int | None, str]:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if not pid_exists(self.launcher_pid):
                return None, "launcher_gone"
            descendants = collect_descendants(self.launcher_pid)

            for pid in descendants:
                if pid_exists(pid) and self._is_exact_process(pid):
                    return pid, "resolved_descendant_exact"

            if pid_exists(self.launcher_pid) and self._is_exact_process(self.launcher_pid):
                return self.launcher_pid, "resolved_root_exact"

            for pid in descendants:
                if pid_exists(pid) and self._is_wrapped_process(pid):
                    return pid, "resolved_descendant_wrapped"

            if pid_exists(self.launcher_pid) and self._is_wrapped_process(self.launcher_pid):
                root_comm = read_comm(Path(f"/proc/{self.launcher_pid}/comm"))
                root_exe = read_exe(self.launcher_pid)
                if root_comm in self.exact_process_names or root_exe in self.exact_process_names:
                    return self.launcher_pid, "resolved_root_wrapped"

            time.sleep(0.2)
        if pid_exists(self.launcher_pid):
            return None, "not_found_under_launcher"
        return None, "not_found"

    def _write_empty_outputs(self, resolution: str) -> None:
        info = {"launcher_pid": self.launcher_pid, self.pid_json_key: None, "resolution": resolution}
        self.info_path.write_text(json.dumps(info, indent=2) + "\n", encoding="utf-8")
        self.process_csv.write_text(
            f"wall_time_unix,elapsed_s,launcher_pid,{self.pid_json_key},last_cpu,process_cpu_percent\n",
            encoding="utf-8",
        )
        self.thread_csv.write_text(
            f"wall_time_unix,elapsed_s,launcher_pid,{self.pid_json_key},tid,comm,last_cpu,thread_cpu_percent\n",
            encoding="utf-8",
        )
        self.summary_csv.write_text(
            "tid,comm,samples,mean_cpu_percent,max_cpu_percent,dominant_core,last_cpu\n",
            encoding="utf-8",
        )

    def _run(self) -> None:
        actual_pid, resolution = self._resolve_actual_pid()
        if actual_pid is None or not pid_exists(actual_pid):
            self._write_empty_outputs(resolution)
            return

        info = {
            "launcher_pid": self.launcher_pid,
            self.pid_json_key: actual_pid,
            "resolution": resolution,
            self.comm_json_key: read_comm(Path(f"/proc/{actual_pid}/comm")),
            self.exe_json_key: read_exe(actual_pid),
            self.cmdline_json_key: read_cmdline(actual_pid),
        }
        self.info_path.write_text(json.dumps(info, indent=2) + "\n", encoding="utf-8")

        start_wall = time.time()
        prev_sample_wall = time.monotonic()
        prev_proc_total, prev_proc_cpu, _ = parse_stat(Path(f"/proc/{actual_pid}/stat"))
        prev_threads: dict[int, tuple[int, int]] = {}
        task_dir = Path(f"/proc/{actual_pid}/task")
        for task in task_dir.iterdir():
            tid = int(task.name)
            total, cpu, _ = parse_stat(task / "stat")
            prev_threads[tid] = (total, cpu)

        summary: dict[int, dict[str, Any]] = {}
        with self.process_csv.open("w", newline="", encoding="utf-8") as proc_f, self.thread_csv.open(
            "w",
            newline="",
            encoding="utf-8",
        ) as thr_f:
            proc_writer = csv.writer(proc_f)
            thr_writer = csv.writer(thr_f)
            proc_writer.writerow(
                ["wall_time_unix", "elapsed_s", "launcher_pid", self.pid_json_key, "last_cpu", "process_cpu_percent"]
            )
            thr_writer.writerow(
                ["wall_time_unix", "elapsed_s", "launcher_pid", self.pid_json_key, "tid", "comm", "last_cpu", "thread_cpu_percent"]
            )

            while pid_exists(actual_pid) and not self._stop.is_set():
                time.sleep(self.sample_sec)
                now_wall = time.time()
                now_mono = time.monotonic()
                delta_t = now_mono - prev_sample_wall
                if delta_t <= 0:
                    continue
                prev_sample_wall = now_mono
                try:
                    proc_total, proc_cpu, _ = parse_stat(Path(f"/proc/{actual_pid}/stat"))
                except Exception:
                    break
                proc_cpu_pct = 100.0 * (proc_total - prev_proc_total) / (delta_t * self.hz)
                prev_proc_total = proc_total
                elapsed_s = now_wall - start_wall
                proc_writer.writerow([f"{now_wall:.3f}", f"{elapsed_s:.3f}", self.launcher_pid, actual_pid, proc_cpu, f"{proc_cpu_pct:.3f}"])
                proc_f.flush()

                current_threads: dict[int, tuple[int, int]] = {}
                if not task_dir.exists():
                    break
                for task in task_dir.iterdir():
                    try:
                        tid = int(task.name)
                        total, last_cpu, _ = parse_stat(task / "stat")
                        comm = read_comm(task / "comm")
                    except Exception:
                        continue
                    prev_total, _prev_cpu = prev_threads.get(tid, (total, last_cpu))
                    thread_cpu_pct = 100.0 * (total - prev_total) / (delta_t * self.hz)
                    thr_writer.writerow(
                        [f"{now_wall:.3f}", f"{elapsed_s:.3f}", self.launcher_pid, actual_pid, tid, comm, last_cpu, f"{thread_cpu_pct:.3f}"]
                    )
                    current_threads[tid] = (total, last_cpu)
                    stats = summary.setdefault(
                        tid,
                        {
                            "comm": comm,
                            "samples": 0,
                            "sum_cpu": 0.0,
                            "max_cpu": 0.0,
                            "core_counts": Counter(),
                            "last_cpu": last_cpu,
                        },
                    )
                    stats["comm"] = comm
                    stats["samples"] += 1
                    stats["sum_cpu"] += thread_cpu_pct
                    stats["max_cpu"] = max(stats["max_cpu"], thread_cpu_pct)
                    stats["core_counts"][last_cpu] += 1
                    stats["last_cpu"] = last_cpu
                thr_f.flush()
                prev_threads = current_threads

        with self.summary_csv.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.writer(handle)
            writer.writerow(["tid", "comm", "samples", "mean_cpu_percent", "max_cpu_percent", "dominant_core", "last_cpu"])
            for tid, stats in sorted(summary.items(), key=lambda item: (-item[1]["sum_cpu"], item[0])):
                dominant_core = ""
                if stats["core_counts"]:
                    dominant_core = stats["core_counts"].most_common(1)[0][0]
                mean_cpu = stats["sum_cpu"] / stats["samples"] if stats["samples"] else 0.0
                writer.writerow(
                    [
                        tid,
                        stats["comm"],
                        stats["samples"],
                        f"{mean_cpu:.3f}",
                        f"{stats['max_cpu']:.3f}",
                        dominant_core,
                        stats["last_cpu"],
                    ]
                )


def generate_cpu_plots(process_csv: Path, thread_summary_csv: Path, out_process_png: Path, out_threads_png: Path, title: str) -> bool:
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception:
        return False

    process_rows: list[tuple[float, float]] = []
    with process_csv.open(encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            try:
                process_rows.append((float(row["elapsed_s"]), float(row["process_cpu_percent"])))
            except Exception:
                continue
    if not process_rows:
        return False

    thread_rows: list[tuple[str, int, float, float, str]] = []
    with thread_summary_csv.open(encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            try:
                thread_rows.append(
                    (
                        row["comm"],
                        int(row["tid"]),
                        float(row["mean_cpu_percent"]),
                        float(row["max_cpu_percent"]),
                        row["dominant_core"],
                    )
                )
            except Exception:
                continue
    thread_rows.sort(key=lambda item: (-item[2], item[1]))

    fig, ax = plt.subplots(figsize=(8, 4.5), dpi=160)
    ax.plot([row[0] for row in process_rows], [row[1] for row in process_rows], color="#2c7fb8", linewidth=1.5)
    ax.set_xlabel("time since monitor start (s)")
    ax.set_ylabel("process CPU usage (%)")
    ax.set_ylim(0, 400)
    ax.set_title(title)
    ax.grid(True, alpha=0.35)
    fig.tight_layout()
    fig.savefig(out_process_png, bbox_inches="tight")
    plt.close(fig)

    if thread_rows:
        labels = []
        values = []
        colors = []
        palette = plt.get_cmap("tab20")
        for idx, (comm, tid, mean_cpu, _max_cpu, dominant_core) in enumerate(thread_rows):
            labels.append(f"{comm} [{tid}] @ CPU {dominant_core or '?'}")
            values.append(mean_cpu)
            try:
                color_idx = int(dominant_core) % 20
            except Exception:
                color_idx = idx % 20
            colors.append(palette(color_idx))
        width = max(10, 0.65 * len(labels) + 3)
        fig, ax = plt.subplots(figsize=(width, 5.5), dpi=160)
        x = list(range(len(labels)))
        bars = ax.bar(x, values, color=colors)
        ax.set_xticks(x)
        ax.set_xticklabels(labels, rotation=45, ha="right")
        ax.set_ylabel("mean CPU usage (%)")
        ax.set_ylim(0, 400)
        ax.set_title(f"{title} thread CPU usage")
        ax.grid(True, axis="y", alpha=0.35)
        ax.bar_label(bars, labels=[f"{value:.1f}" for value in values], padding=3, fontsize=8, rotation=90)
        fig.tight_layout()
        fig.savefig(out_threads_png, bbox_inches="tight")
        plt.close(fig)
    return True


def load_tum_xyz(path: Path) -> list[tuple[float, float, float, float]]:
    rows: list[tuple[float, float, float, float]] = []
    for raw_line in path.read_text(encoding="utf-8").splitlines():
        line = raw_line.strip()
        if not line or line.startswith("#"):
            continue
        fields = line.replace(",", " ").split()
        if len(fields) < 8:
            continue
        rows.append((float(fields[0]), float(fields[1]), float(fields[2]), float(fields[3])))
    if not rows:
        raise ValueError(f"No TUM poses found in {path}")
    return rows


def associate_xyz(
    gt_rows: list[tuple[float, float, float, float]],
    est_rows: list[tuple[float, float, float, float]],
    max_diff: float,
) -> tuple[list[float], list[list[float]], list[list[float]]]:
    gt_points: list[list[float]] = []
    est_points: list[list[float]] = []
    assoc_times: list[float] = []
    j = 0
    for gt_ts, gx, gy, gz in gt_rows:
        while j < len(est_rows) and est_rows[j][0] < gt_ts - max_diff:
            j += 1
        if j >= len(est_rows):
            break
        best_idx: int | None = None
        best_dt = math.inf
        k = j
        while k < len(est_rows) and est_rows[k][0] <= gt_ts + max_diff:
            dt = abs(est_rows[k][0] - gt_ts)
            if dt < best_dt:
                best_dt = dt
                best_idx = k
            k += 1
        if best_idx is None:
            continue
        _, ex, ey, ez = est_rows[best_idx]
        gt_points.append([gx, gy, gz])
        est_points.append([ex, ey, ez])
        assoc_times.append(gt_ts)
        j = best_idx + 1
    if len(gt_points) < 3:
        raise ValueError(f"Only {len(gt_points)} timestamp associations found; need at least 3 for alignment.")
    return assoc_times, gt_points, est_points


def umeyama_similarity(src_points: list[list[float]], dst_points: list[list[float]]) -> tuple[float, list[list[float]], list[float]]:
    try:
        import numpy as np
    except Exception as exc:
        raise RuntimeError("numpy is required for plot generation") from exc
    src = np.asarray(src_points, dtype=float)
    dst = np.asarray(dst_points, dtype=float)
    if src.shape != dst.shape or src.shape[0] < 3:
        raise ValueError("Alignment input shape mismatch")
    src_mean = src.mean(axis=0)
    dst_mean = dst.mean(axis=0)
    src_demean = src - src_mean
    dst_demean = dst - dst_mean
    covariance = (dst_demean.T @ src_demean) / src.shape[0]
    u, singular_values, vt = np.linalg.svd(covariance)
    correction = np.eye(src.shape[1])
    if np.linalg.det(u) * np.linalg.det(vt) < 0:
        correction[-1, -1] = -1.0
    rotation = u @ correction @ vt
    src_var = np.mean(np.sum(src_demean * src_demean, axis=1))
    if src_var <= 0:
        raise ValueError("Source variance is zero; cannot align trajectory.")
    scale = float(np.sum(singular_values * np.diag(correction)) / src_var)
    translation = dst_mean - scale * (rotation @ src_mean)
    return scale, rotation.tolist(), translation.tolist()


def generate_xy_plots(
    gt_file: Path,
    est_file: Path,
    ape_plot_path: Path,
    traj_plot_path: Path,
    title: str,
    est_label: str,
    t_max_diff: float,
) -> None:
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        import numpy as np
    except Exception as exc:
        raise RuntimeError("matplotlib and numpy are required for plot generation") from exc

    gt_rows = load_tum_xyz(gt_file)
    est_rows = load_tum_xyz(est_file)
    _times, gt_assoc, est_assoc = associate_xyz(gt_rows, est_rows, max_diff=t_max_diff)
    scale, rotation_list, translation_list = umeyama_similarity(est_assoc, gt_assoc)
    rotation = np.asarray(rotation_list, dtype=float)
    translation = np.asarray(translation_list, dtype=float)
    gt_xyz = np.asarray([[x, y, z] for _, x, y, z in gt_rows], dtype=float)
    est_xyz = np.asarray([[x, y, z] for _, x, y, z in est_rows], dtype=float)
    est_aligned = (scale * (rotation @ est_xyz.T)).T + translation

    for target in (ape_plot_path, traj_plot_path):
        fig, ax = plt.subplots(figsize=(6, 6), dpi=160)
        ax.plot(gt_xyz[:, 0], gt_xyz[:, 1], "--", color="#7f7f7f", linewidth=1.5, label="GT")
        ax.plot(est_aligned[:, 0], est_aligned[:, 1], color="#57c7b6", linewidth=1.6, label=est_label)
        ax.set_xlabel("x (m)")
        ax.set_ylabel("y (m)")
        ax.set_title(title)
        ax.set_aspect("equal", adjustable="box")
        ax.grid(True, alpha=0.35)
        ax.legend(loc="best")
        fig.tight_layout()
        fig.savefig(target, bbox_inches="tight")
        plt.close(fig)


@dataclass
class EvoResult:
    rmse: float
    summary_txt: Path
    result_zip: Path
    ape_plot: Path
    traj_plot: Path


def run_evo_and_plots(
    shell_ctx: ShellContext,
    gt_file: Path,
    est_file: Path,
    out_dir: Path,
    seq_name: str,
    t_max_diff: float,
    correct_scale: bool,
    est_label: str,
) -> EvoResult:
    summary_txt = out_dir / f"{seq_name}_evo.txt"
    result_zip = out_dir / f"{seq_name}_evo.zip"
    ape_plot = out_dir / "ape_traj.png"
    traj_plot = out_dir / "traj_xy.png"

    cmd = [
        "evo_ape",
        "tum",
        shlex.quote(str(gt_file)),
        shlex.quote(str(est_file)),
        "--verbose",
        "--align",
        "--t_max_diff",
        str(t_max_diff),
        "--save_results",
        shlex.quote(str(result_zip)),
    ]
    if correct_scale:
        cmd.append("--correct_scale")
    result = run_capture(shell_ctx, " ".join(cmd))
    summary_txt.write_text(result.stdout, encoding="utf-8")
    if result.returncode != 0:
        raise RuntimeError(f"evo_ape failed for {seq_name}. See {summary_txt}")
    match = re.search(r"^\s*rmse\s+([0-9eE+.\-]+)\s*$", result.stdout, flags=re.MULTILINE)
    if match is None:
        raise RuntimeError(f"Could not parse RMSE from {summary_txt}")
    generate_xy_plots(gt_file, est_file, ape_plot, traj_plot, seq_name, est_label, t_max_diff)
    return EvoResult(float(match.group(1)), summary_txt, result_zip, ape_plot, traj_plot)


def safe_copy(src: Path | None, dst: Path) -> None:
    if src is not None and src.exists():
        dst.write_bytes(src.read_bytes())


def generate_aggregate_outputs(results_root: Path) -> None:
    sequence_dirs = sorted([path for path in results_root.iterdir() if path.is_dir()])
    run_rows: list[dict[str, Any]] = []
    for seq_dir in sequence_dirs:
        run_dirs = sorted([path for path in seq_dir.iterdir() if path.is_dir() and path.name.startswith("run_")])
        metas: list[dict[str, Any]] = []
        for run_dir in run_dirs:
            meta_path = run_dir / "run_meta.json"
            if not meta_path.exists():
                continue
            meta = json.loads(meta_path.read_text(encoding="utf-8"))
            metas.append(meta)
            run_rows.append(meta)
            for artifact_name in ("ape_traj.png", "traj_xy.png"):
                src = run_dir / artifact_name
                if src.exists():
                    dst = results_root / f"{artifact_name.rsplit('.', 1)[0]}_{seq_dir.name}_{run_dir.name}.png"
                    safe_copy(src, dst)

        rmse_vals = [meta["rmse"] for meta in metas if isinstance(meta.get("rmse"), (int, float))]
        cpu_vals = [meta["cpu_total_avg_pct"] for meta in metas if isinstance(meta.get("cpu_total_avg_pct"), (int, float))]
        summary = {
            "sequence": seq_dir.name,
            "n_runs_total": len(metas),
            "n_runs_success": sum(1 for meta in metas if meta.get("success")),
            "rmse_avg": statistics.fmean(rmse_vals) if rmse_vals else None,
            "rmse_std": statistics.pstdev(rmse_vals) if len(rmse_vals) > 1 else (0.0 if rmse_vals else None),
            "cpu_total_avg_pct": statistics.fmean(cpu_vals) if cpu_vals else None,
            "runs": metas,
        }
        (seq_dir / "run_summary.json").write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")

    with (results_root / "experiment_summary.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(
            [
                "sequence",
                "run_name",
                "success",
                "rmse",
                "traj_used",
                "cpu_total_avg_pct",
                "trajectory_poses",
            ]
        )
        for row in sorted(run_rows, key=lambda item: (item["sequence"], item["run_name"])):
            writer.writerow(
                [
                    row["sequence"],
                    row["run_name"],
                    row.get("success"),
                    row.get("rmse"),
                    row.get("traj_used"),
                    row.get("cpu_total_avg_pct"),
                    row.get("trajectory_poses"),
                ]
            )

    table_runs = sorted({row["run_name"] for row in run_rows})
    lines = [
        "# Trajectory Report",
        "",
        f"- Run folder: `{results_root}`",
        "- Plot style: estimated trajectory in blue, reference trajectory in dashed gray",
        "",
    ]
    if table_runs:
        lines.append("| " + " | ".join(name.replace("_", " ").title() for name in table_runs) + " |")
        lines.append("|" + "---|" * len(table_runs))
        for seq_dir in sequence_dirs:
            cells: list[str] = []
            for run_name in table_runs:
                run_dir = seq_dir / run_name
                img = run_dir / "traj_xy.png"
                if img.exists():
                    rel = img.relative_to(results_root).as_posix()
                    cells.append(
                        f"**{seq_dir.name}**<br><a href=\"{rel}\"><img src=\"{rel}\" "
                        f"alt=\"{seq_dir.name} {run_name} trajectory\" width=\"100%\"></a>"
                    )
                else:
                    cells.append(f"**{seq_dir.name}**<br>-")
            lines.append("| " + " | ".join(cells) + " |")
    (results_root / "polling_results.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
