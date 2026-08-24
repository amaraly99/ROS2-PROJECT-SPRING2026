#!/usr/bin/env python3

import argparse
import os
from pathlib import Path
import re
import shlex
import signal
import subprocess
import sys
import threading
import time


SCRIPT_PATH = Path(__file__).resolve()
HOST_WORKSPACE = SCRIPT_PATH.parent
CONTAINER_WORKSPACE = Path("/workspace")
DEFAULT_CONTAINER = "rtabmap_jazzy"
SUPPORTED_BAG_SUFFIXES = (".db3", ".mcap")


def in_container():
    return Path("/.dockerenv").exists()


def q(value):
    return shlex.quote(str(value))


def ros(command):
    return f"source /opt/ros/jazzy/setup.bash && {command}"


def tail(path, lines=80):
    if not path.exists():
        return ""
    data = path.read_text(errors="ignore").splitlines()
    return "\n".join(data[-lines:])


def run_capture(command):
    return subprocess.run(["bash", "-lc", command], text=True, capture_output=True)


def start_process(command):
    return subprocess.Popen(["bash", "-lc", command], start_new_session=True)


def run_live(command):
    return subprocess.run(["bash", "-lc", command], text=True)


def stop_process(proc, sig=signal.SIGTERM, timeout=5):
    if proc is None or proc.poll() is not None:
        return
    try:
        os.killpg(proc.pid, sig)
    except ProcessLookupError:
        return
    try:
        proc.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(proc.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        proc.wait(timeout=timeout)


def stop_pid(pid, sig=signal.SIGTERM, timeout=5):
    try:
        os.killpg(pid, sig)
    except ProcessLookupError:
        return
    deadline = time.time() + timeout
    while time.time() < deadline:
        try:
            os.kill(pid, 0)
        except ProcessLookupError:
            return
        time.sleep(0.1)
    try:
        os.killpg(pid, signal.SIGKILL)
    except ProcessLookupError:
        return


def workspace_path(path_str):
    path = Path(path_str)
    if path.is_absolute():
        return path
    return (HOST_WORKSPACE / path).resolve()


def host_to_container_path(path_str):
    path = workspace_path(path_str)
    try:
        rel = path.relative_to(HOST_WORKSPACE)
        return str(CONTAINER_WORKSPACE / rel)
    except ValueError:
        return str(path)


def normalize_bag_path(path_str):
    path = Path(path_str)
    if not path.exists():
        raise RuntimeError(f"Bag path not found: {path_str}")
    if path.is_dir():
        if (path / "metadata.yaml").exists():
            return path
        raise RuntimeError(f"Bag directory is missing metadata.yaml: {path}")
    if path.suffix.lower() not in SUPPORTED_BAG_SUFFIXES:
        suffixes = ", ".join(SUPPORTED_BAG_SUFFIXES)
        raise RuntimeError(f"Unsupported bag file type for {path} (expected one of: {suffixes})")
    return path


class EvalRunner:
    def __init__(self, args):
        self.args = args
        self.launch_proc = None
        self.bag_proc = None
        self.current_proc = None
        self.stream_threads = []

        self.log_dir = Path(args.log_dir)
        self.log_dir.mkdir(parents=True, exist_ok=True)
        stamp = time.strftime("%Y%m%d_%H%M%S")
        self.launch_log = self.log_dir / f"launch_{stamp}.log"
        self.bag_log = self.log_dir / f"bag_{stamp}.log"
        self.export_log = self.log_dir / f"export_{stamp}.log"
        self.evo_log = self.log_dir / f"evo_{stamp}.log"
        self.pid_file = self.log_dir / "run_euroc_f2m_eval.pids"

    def cleanup(self):
        stop_process(self.current_proc)
        stop_process(self.bag_proc)
        stop_process(self.launch_proc)
        for thread in self.stream_threads:
            thread.join(timeout=1)
        self.remove_pid_file()

    def write_pid_file(self):
        lines = [f"runner_pid={os.getpid()}"]
        if self.launch_proc is not None:
            lines.append(f"launch_pgid={self.launch_proc.pid}")
        if self.bag_proc is not None:
            lines.append(f"bag_pgid={self.bag_proc.pid}")
        if self.current_proc is not None:
            lines.append(f"current_pgid={self.current_proc.pid}")
        self.pid_file.write_text("\n".join(lines) + "\n")

    def remove_pid_file(self):
        if self.pid_file.exists():
            self.pid_file.unlink()

    def validate_inputs(self):
        self.args.bag = str(normalize_bag_path(self.args.bag))
        if not Path(self.args.gt).exists():
            raise RuntimeError(f"Ground truth file not found: {self.args.gt}")
        if not Path(self.args.launch_file).exists():
            raise RuntimeError(f"Launch file not found: {self.args.launch_file}")
        if not Path(self.args.exporter).exists():
            raise RuntimeError(f"Exporter not found: {self.args.exporter}")

    def ensure_nothing_is_running(self):
        result = run_capture(ros("ros2 node list"))
        nodes = [line.strip() for line in result.stdout.splitlines() if line.strip().startswith("/")]
        conflicts = [n for n in nodes if n in ("/rtabmap", "/stereo_odometry", "/rosbag2_player")]
        if conflicts:
            raise RuntimeError(
                "Existing nodes are still running: " + ", ".join(sorted(conflicts))
            )

    def start_launch(self):
        command = ros(f"ros2 launch {q(self.args.launch_file)}")
        self.launch_proc = subprocess.Popen(
            ["bash", "-lc", command],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
            start_new_session=True,
        )
        self.stream_threads.append(
            self.start_stream_thread(self.launch_proc, self.launch_log, echo=True)
        )
        self.write_pid_file()

    def start_stream_thread(self, proc, log_path, echo):
        def worker():
            with open(log_path, "w", encoding="utf-8", errors="ignore") as f:
                assert proc.stdout is not None
                for line in proc.stdout:
                    f.write(line)
                    f.flush()
                    if echo:
                        sys.stdout.write(line)
                        sys.stdout.flush()

        thread = threading.Thread(target=worker, daemon=True)
        thread.start()
        return thread

    def wait_for_service(self, timeout_sec=60):
        pattern = re.compile(r"/rtabmap(?:/rtabmap)?/get_map_data2$")
        deadline = time.time() + timeout_sec
        while time.time() < deadline:
            if self.launch_proc.poll() is not None:
                raise RuntimeError("Launch exited early.\n" + tail(self.launch_log))
            result = run_capture(ros("ros2 service list"))
            services = [line.strip() for line in result.stdout.splitlines()]
            if any(pattern.search(name) for name in services):
                return
            time.sleep(1)
        raise RuntimeError("Timed out waiting for get_map_data2.\n" + tail(self.launch_log))

    def play_bag(self):
        command = ros(
            f"ros2 bag play {q(self.args.bag)} --clock > {q(self.bag_log)} 2>&1"
        )
        self.bag_proc = start_process(command)
        self.write_pid_file()
        returncode = self.bag_proc.wait()
        self.bag_proc = None
        self.write_pid_file()
        if returncode != 0:
            raise RuntimeError("Bag playback failed.\n" + tail(self.bag_log))

    def export_trajectory(self):
        out_path = Path(self.args.traj_out)
        if out_path.exists():
            out_path.unlink()

        command = ros(
            f"python3 {q(self.args.exporter)} --source service --idle-timeout 0 "
            f"--out {q(self.args.traj_out)}"
        )
        self.current_proc = subprocess.Popen(
            ["bash", "-lc", command],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
            start_new_session=True,
        )
        self.stream_threads.append(
            self.start_stream_thread(self.current_proc, self.export_log, echo=True)
        )
        self.write_pid_file()
        returncode = self.current_proc.wait()
        self.current_proc = None
        self.write_pid_file()
        if returncode != 0:
            raise RuntimeError("Trajectory export failed.\n" + tail(self.export_log))
        if not out_path.exists() or out_path.stat().st_size == 0:
            raise RuntimeError("Trajectory file was not created.\n" + tail(self.export_log))

    def run_evo(self):
        exists = run_capture("command -v evo_ape")
        if exists.returncode != 0:
            print("[WARN] evo_ape is not installed in this environment.")
            return

        command = (
            f"evo_ape tum {q(self.args.gt)} {q(self.args.traj_out)} "
            f"-a -s --t_max_diff 0.01"
        )
        self.current_proc = subprocess.Popen(
            ["bash", "-lc", command],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
            start_new_session=True,
        )
        self.stream_threads.append(
            self.start_stream_thread(self.current_proc, self.evo_log, echo=True)
        )
        self.write_pid_file()
        returncode = self.current_proc.wait()
        self.current_proc = None
        self.write_pid_file()
        if returncode != 0:
            raise RuntimeError("evo_ape failed.\n" + tail(self.evo_log))

    def run(self):
        self.validate_inputs()
        self.ensure_nothing_is_running()

        print("[INFO] Launching RTAB-Map...")
        self.start_launch()
        print("[INFO] Waiting for get_map_data2...")
        self.wait_for_service()

        print(f"[INFO] Playing bag: {self.args.bag}")
        self.play_bag()

        print(f"[INFO] Waiting {self.args.settle_sec}s for final optimization...")
        time.sleep(self.args.settle_sec)

        print(f"[INFO] Exporting trajectory to {self.args.traj_out}")
        self.export_trajectory()

        print("[INFO] Stopping ROS processes...")
        self.cleanup()

        print("[INFO] Evaluating trajectory...")
        self.run_evo()

        print()
        print("[INFO] Done.")
        print(f"[INFO] Trajectory: {self.args.traj_out}")
        print(f"[INFO] Launch log: {self.launch_log}")
        print(f"[INFO] Bag log: {self.bag_log}")
        print(f"[INFO] Export log: {self.export_log}")
        if self.evo_log.exists():
            print(f"[INFO] Evo log: {self.evo_log}")


def run_inside_container(args):
    if args.cleanup:
        cleanup_from_pid_file(Path(args.log_dir) / "run_euroc_f2m_eval.pids")
        return

    runner = EvalRunner(args)

    def handle_interrupt(_signum, _frame):
        runner.cleanup()
        raise SystemExit(130)

    signal.signal(signal.SIGINT, handle_interrupt)
    signal.signal(signal.SIGTERM, handle_interrupt)

    try:
        runner.run()
    finally:
        runner.cleanup()


def run_from_host(args):
    if args.cleanup:
        command = [
            "sudo",
            "docker",
            "exec",
            "-i",
            args.container,
            "python3",
            str(CONTAINER_WORKSPACE / SCRIPT_PATH.name),
            "--inside-container",
            "--cleanup",
            "--log-dir",
            host_to_container_path(args.log_dir),
        ]
        raise SystemExit(subprocess.call(command))

    command = [
        "sudo",
        "docker",
        "exec",
        "-i",
        args.container,
        "python3",
        str(CONTAINER_WORKSPACE / SCRIPT_PATH.name),
        "--inside-container",
        "--bag",
        host_to_container_path(args.bag),
        "--gt",
        host_to_container_path(args.gt),
        "--launch-file",
        host_to_container_path(args.launch_file),
        "--exporter",
        host_to_container_path(args.exporter),
        "--traj-out",
        host_to_container_path(args.traj_out),
        "--log-dir",
        host_to_container_path(args.log_dir),
        "--settle-sec",
        str(args.settle_sec),
    ]

    proc = subprocess.Popen(command, start_new_session=True)
    try:
        raise SystemExit(proc.wait())
    except KeyboardInterrupt:
        stop_process(proc, sig=signal.SIGINT, timeout=10)
        raise SystemExit(130)


def parse_args():
    if in_container():
        default_bag = str(CONTAINER_WORKSPACE / "MH_01_easy.db3")
        default_gt = str(CONTAINER_WORKSPACE / "MH_01_easy.txt")
        default_launch = str(CONTAINER_WORKSPACE / "euroc_offline_f2m.launch.py")
        default_exporter = str(CONTAINER_WORKSPACE / "mapPath_to_tum.py")
        default_traj = str(CONTAINER_WORKSPACE / "rtabmap_mapPath.tum")
        default_logs = str(CONTAINER_WORKSPACE / "run_logs")
    else:
        default_bag = str(HOST_WORKSPACE / "MH_01_easy.db3")
        default_gt = str(HOST_WORKSPACE / "MH_01_easy.txt")
        default_launch = str(HOST_WORKSPACE / "euroc_offline_f2m.launch.py")
        default_exporter = str(HOST_WORKSPACE / "mapPath_to_tum.py")
        default_traj = str(HOST_WORKSPACE / "rtabmap_mapPath.tum")
        default_logs = str(HOST_WORKSPACE / "run_logs")

    parser = argparse.ArgumentParser(
        description="Run RTAB-Map launch, play a rosbag2 .db3/.mcap bag, export trajectory, then evaluate."
    )
    parser.add_argument("--inside-container", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument("--cleanup", action="store_true")
    parser.add_argument("--container", default=DEFAULT_CONTAINER)
    parser.add_argument("--bag", default=default_bag, help="Path to a rosbag2 bag file (.db3 or .mcap) or bag directory.")
    parser.add_argument("--gt", default=default_gt)
    parser.add_argument("--launch-file", default=default_launch)
    parser.add_argument("--exporter", default=default_exporter)
    parser.add_argument("--traj-out", default=default_traj)
    parser.add_argument("--log-dir", default=default_logs)
    parser.add_argument("--settle-sec", type=float, default=3.0)
    return parser.parse_args()


def cleanup_from_pid_file(pid_file: Path):
    if not pid_file.exists():
        print(f"[INFO] No PID file found at {pid_file}")
        return

    data = {}
    for line in pid_file.read_text().splitlines():
        if "=" in line:
            key, value = line.split("=", 1)
            data[key.strip()] = value.strip()

    for key in ("current_pgid", "bag_pgid", "launch_pgid"):
        value = data.get(key)
        if value and value.isdigit():
            print(f"[INFO] Stopping {key}={value}")
            stop_pid(int(value), sig=signal.SIGTERM, timeout=5)

    pid_file.unlink(missing_ok=True)


def main():
    args = parse_args()
    if args.inside_container or in_container():
        run_inside_container(args)
    else:
        run_from_host(args)


if __name__ == "__main__":
    main()
