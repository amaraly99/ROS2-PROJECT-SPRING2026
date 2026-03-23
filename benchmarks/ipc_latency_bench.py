#!/usr/bin/env python3
"""
IPC latency benchmark: POSIX SHM seqlock vs ROS2 DDS

Sends N messages of a specified size, measures delivery latency
(writer timestamp to reader timestamp), outputs CDF data to CSV.

Usage:
  python3 benchmarks/ipc_latency_bench.py --mode shm --n 1000 --size 921600
  python3 benchmarks/ipc_latency_bench.py --mode dds --n 1000 --size 921600

For DDS mode this script must be run inside the Docker container (rclpy required).
"""

import argparse
import ctypes
import mmap
import os
import struct
import sys
import time
import threading
import csv
import random

# ─── Args ─────────────────────────────────────────────────────────────────────
parser = argparse.ArgumentParser(description='IPC latency benchmark')
parser.add_argument('--mode', choices=['shm', 'dds'], required=True)
parser.add_argument('--n', type=int, default=1000, help='Number of messages')
parser.add_argument('--size', type=int, default=921600, help='Message size bytes (921600 = 640x480 RGB)')
parser.add_argument('--out', type=str, default=None, help='Output CSV path')
args = parser.parse_args()

N_MESSAGES = args.n
MSG_SIZE   = args.size
OUT_CSV    = args.out or f'benchmarks/results/ipc_latency_{args.mode}_{args.n}x{args.size}.csv'

os.makedirs(os.path.dirname(OUT_CSV), exist_ok=True)


# ═══════════════════════════════════════════════════════════════════════════════
# SHM Seqlock Mode
# ═══════════════════════════════════════════════════════════════════════════════

def run_shm(n_msg: int, msg_size: int) -> list:
    """
    Measure SHM seqlock delivery latency.

    Layout of the shared region (single slot, no ring buffer):
      [0:8]         seqlock counter (uint64, odd = writing)
      [8:16]        write_ts_ns (uint64)
      [16:16+size]  payload

    Writer increments seqlock to odd, writes ts + data, increments to even.
    Reader polls for even seqlock, records read_ts, computes latency.
    """
    _rt = ctypes.CDLL("librt.so.1", use_errno=True)
    _pt = ctypes.CDLL("libpthread.so.0", use_errno=True)

    O_CREAT = 0o100
    O_RDWR  = 0o2
    PROT_READ_WRITE = mmap.PROT_READ | mmap.PROT_WRITE

    SHM_NAME   = b"/ipc_bench_lat"
    SEM_NAME   = b"/ipc_bench_sem"
    HEADER_SZ  = 64    # aligned slot header
    TOTAL_SZ   = HEADER_SZ + msg_size

    # SEQLOCK layout inside header:
    #   offset 0:  uint64 seq   (odd = writing, even = ready)
    #   offset 8:  uint64 write_ts_ns
    SEQ_OFF     = 0
    WTS_OFF     = 8
    PAYLOAD_OFF = HEADER_SZ

    # Create / open SHM
    _rt.shm_open.argtypes  = [ctypes.c_char_p, ctypes.c_int, ctypes.c_uint]
    _rt.shm_open.restype   = ctypes.c_int
    _rt.shm_unlink.argtypes = [ctypes.c_char_p]
    _rt.shm_unlink.restype  = ctypes.c_int
    _rt.ftruncate = ctypes.CDLL(None).ftruncate
    _rt.ftruncate.argtypes = [ctypes.c_int, ctypes.c_long]
    _rt.ftruncate.restype  = ctypes.c_int

    _pt.sem_open.argtypes  = [ctypes.c_char_p, ctypes.c_int, ctypes.c_uint, ctypes.c_uint]
    _pt.sem_open.restype   = ctypes.c_void_p
    _pt.sem_post.argtypes  = [ctypes.c_void_p]
    _pt.sem_post.restype   = ctypes.c_int
    _pt.sem_wait.argtypes  = [ctypes.c_void_p]
    _pt.sem_wait.restype   = ctypes.c_int
    _pt.sem_unlink.argtypes = [ctypes.c_char_p]
    _pt.sem_unlink.restype  = ctypes.c_int

    _rt.shm_unlink(SHM_NAME)
    _pt.sem_unlink(SEM_NAME)

    fd = _rt.shm_open(SHM_NAME, O_CREAT | O_RDWR, 0o666)
    if fd < 0:
        raise RuntimeError(f"shm_open failed: {ctypes.get_errno()}")
    os.ftruncate(fd, TOTAL_SZ)

    mm = mmap.mmap(fd, TOTAL_SZ, mmap.MAP_SHARED, PROT_READ_WRITE)
    os.close(fd)

    # Sem: writer posts, reader waits
    sem = _pt.sem_open(SEM_NAME, O_CREAT, 0o666, 0)
    if sem is None or sem == ctypes.c_void_p(-1).value:
        raise RuntimeError("sem_open failed")

    latencies_us = []
    payload = bytes(random.getrandbits(8) for _ in range(min(msg_size, 65536)))
    # Tile payload to full size
    payload = (payload * ((msg_size // len(payload)) + 1))[:msg_size]

    # ── Reader thread ──────────────────────────────────────────────────────
    reader_done = threading.Event()
    reader_lats = []

    def reader():
        for _ in range(n_msg):
            # Wait for semaphore (writer signals delivery)
            _pt.sem_wait(sem)
            read_ts = time.time_ns()

            # Seqlock read
            while True:
                s1 = struct.unpack_from("<Q", mm, SEQ_OFF)[0]
                if s1 & 1:        # odd → writer still writing
                    continue
                write_ts = struct.unpack_from("<Q", mm, WTS_OFF)[0]
                s2 = struct.unpack_from("<Q", mm, SEQ_OFF)[0]
                if s2 == s1:      # no torn read
                    break

            lat_us = (read_ts - write_ts) / 1_000.0
            reader_lats.append(lat_us)

        reader_done.set()

    t = threading.Thread(target=reader, daemon=True)
    t.start()

    # ── Writer ─────────────────────────────────────────────────────────────
    time.sleep(0.05)   # give reader thread time to reach sem_wait
    for i in range(n_msg):
        # Seqlock write: increment to odd
        seq = struct.unpack_from("<Q", mm, SEQ_OFF)[0]
        struct.pack_into("<Q", mm, SEQ_OFF, seq + 1)   # odd = writing

        write_ts = time.time_ns()
        struct.pack_into("<Q", mm, WTS_OFF, write_ts)
        mm[PAYLOAD_OFF:PAYLOAD_OFF + msg_size] = payload

        struct.pack_into("<Q", mm, SEQ_OFF, seq + 2)   # even = ready
        _pt.sem_post(sem)                               # wake reader

        time.sleep(0.001)   # 1ms between writes ≈ 1000fps max (well above camera rate)

    reader_done.wait(timeout=30)
    t.join(timeout=30)

    # Cleanup
    mm.close()
    _rt.shm_unlink(SHM_NAME)
    _pt.sem_unlink(SEM_NAME)

    return reader_lats


# ═══════════════════════════════════════════════════════════════════════════════
# DDS / ROS2 Mode
# ═══════════════════════════════════════════════════════════════════════════════

def run_dds(n_msg: int, msg_size: int) -> list:
    """Measure ROS2 DDS delivery latency via a loopback topic."""
    try:
        import rclpy
        from rclpy.node import Node
        from std_msgs.msg import UInt8MultiArray
    except ImportError:
        print("ERROR: rclpy not available. Run this inside the Docker container.", file=sys.stderr)
        sys.exit(1)

    rclpy.init()

    latencies_us = []
    write_ts_store = {}   # seq → write_ts_ns
    received = threading.Event()
    received_count = [0]
    lock = threading.Lock()

    class LatBench(Node):
        def __init__(self):
            super().__init__('ipc_lat_bench')
            self.pub = self.create_publisher(UInt8MultiArray, '/ipc_bench/frame', 10)
            self.sub = self.create_subscription(
                UInt8MultiArray, '/ipc_bench/frame', self._cb, 10)
            self._seq = 0
            payload_chunk = bytes(random.getrandbits(8) for _ in range(min(msg_size, 65536)))
            payload_full  = (payload_chunk * ((msg_size // len(payload_chunk)) + 1))[:msg_size]
            # Pre-build as list[int] once (UInt8MultiArray.data expects list[int])
            self._payload_list = list(payload_full)

        def publish_one(self):
            msg = UInt8MultiArray()
            seq = self._seq
            write_ts = time.time_ns()
            with lock:
                write_ts_store[seq] = write_ts
            # Pack seq (4 bytes) at the front, then payload
            seq_bytes = list(struct.pack("<I", seq))
            msg.data = seq_bytes + self._payload_list[:msg_size - 4]
            self.pub.publish(msg)
            self._seq += 1

        def _cb(self, msg):
            recv_ts = time.time_ns()
            if len(msg.data) < 4:
                return
            seq = struct.unpack_from("<I", bytes(msg.data[:4]))[0]
            with lock:
                wts = write_ts_store.pop(seq, None)
            if wts is None:
                return
            lat_us = (recv_ts - wts) / 1_000.0
            latencies_us.append(lat_us)
            received_count[0] += 1
            if received_count[0] >= n_msg:
                received.set()

    node = LatBench()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    time.sleep(0.5)   # wait for DDS discovery

    for i in range(n_msg):
        node.publish_one()
        time.sleep(0.002)   # 500fps max

    received.wait(timeout=60)

    executor.shutdown()
    node.destroy_node()
    rclpy.shutdown()

    return latencies_us


# ═══════════════════════════════════════════════════════════════════════════════
# Main
# ═══════════════════════════════════════════════════════════════════════════════

print(f"IPC Latency Benchmark — mode={args.mode} n={N_MESSAGES} size={MSG_SIZE}B")
print(f"Message size: {MSG_SIZE / 1024:.1f} KB  ({MSG_SIZE / (640*480*3):.3f} × 640×480 RGB frame)")
print()

if args.mode == 'shm':
    lats = run_shm(N_MESSAGES, MSG_SIZE)
else:
    lats = run_dds(N_MESSAGES, MSG_SIZE)

if not lats:
    print("ERROR: No latency measurements collected.", file=sys.stderr)
    sys.exit(1)

lats_sorted = sorted(lats)
n = len(lats_sorted)
p50  = lats_sorted[int(0.50 * n)]
p95  = lats_sorted[int(0.95 * n)]
p99  = lats_sorted[int(0.99 * n)]
mean = sum(lats_sorted) / n

print(f"Results ({n} messages):")
print(f"  Mean:   {mean:.1f} µs")
print(f"  P50:    {p50:.1f} µs")
print(f"  P95:    {p95:.1f} µs")
print(f"  P99:    {p99:.1f} µs")
print(f"  Max:    {lats_sorted[-1]:.1f} µs")
print()

with open(OUT_CSV, 'w', newline='') as f:
    writer = csv.writer(f)
    writer.writerow(['mode', 'message_size_bytes', 'latency_us'])
    for lat in lats_sorted:
        writer.writerow([args.mode, MSG_SIZE, f'{lat:.2f}'])

print(f"CSV written to {OUT_CSV}")
