#!/usr/bin/env python3
"""
synthetic_camera_feeder.py — fill /ovcam_frames with synthetic NV12 frames.

Test utility: stands in for sim_camera_bridge/ovcam_producer so detector
producers and the SHM→ROS2 bridges can be exercised on the Pi WITHOUT the
MATLAB simulation or the physical camera. Layout is byte-identical to
ovcam_shm.hpp (ShmGlobal 128 B, SlotHeader 64 B, NV12 payload).

The pattern is a bright block orbiting a gray background — moving, so MOG2
detects it; high-contrast, so YOLO/VLM see *something* (semantics untested).

Usage:
    python3 benchmarks/synthetic_camera_feeder.py [--fps 20] [--frames 0]
            [--width 640] [--height 480] [--cleanup]
"""

import argparse
import ctypes
import mmap
import os
import signal
import struct
import sys
import time

import numpy as np

OVCAM_MAGIC = 0x4F564342
FOURCC_NV12 = 0x3231564E
GLBL_SZ = 128
SLOT_HDR = 64
O_CREAT = 0o100

_libc = ctypes.CDLL(None, use_errno=True)
_libc.sem_open.restype = ctypes.c_void_p


def main():
    ap = argparse.ArgumentParser(description="Synthetic /ovcam_frames feeder")
    ap.add_argument("--fps", type=float, default=20.0)
    ap.add_argument("--frames", type=int, default=0, help="0 = until SIGINT")
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--slots", type=int, default=4)
    ap.add_argument("--cleanup", action="store_true",
                    help="remove SHM + semaphore on exit")
    cfg = ap.parse_args()

    W, H, N = cfg.width, cfg.height, cfg.slots
    stride = W
    frame_bytes = stride * H * 3 // 2
    slot_size = (SLOT_HDR + frame_bytes + 63) & ~63
    total = GLBL_SZ + N * slot_size

    path = "/dev/shm/ovcam_frames"
    fd = os.open(path, os.O_CREAT | os.O_RDWR, 0o666)
    os.ftruncate(fd, total)
    mm = mmap.mmap(fd, total, mmap.MAP_SHARED, mmap.PROT_WRITE | mmap.PROT_READ)
    os.close(fd)
    mm[:total] = b"\x00" * total
    struct.pack_into("<IIIIQQIIII", mm, 0, OVCAM_MAGIC, 1, N, 0,
                     slot_size, frame_bytes, W, H, stride, FOURCC_NV12)

    sem = _libc.sem_open(b"/ovcam_ready", O_CREAT, 0o666, 0)
    if not sem:
        sys.exit("sem_open(/ovcam_ready) failed")

    stop = [False]
    signal.signal(signal.SIGINT, lambda *_: stop.__setitem__(0, True))
    signal.signal(signal.SIGTERM, lambda *_: stop.__setitem__(0, True))

    y = np.full((H, W), 96, np.uint8)
    uv = np.full((H // 2, W), 128, np.uint8)   # neutral chroma
    print(f"[feeder] {W}x{H} NV12 @ {cfg.fps} Hz → {path}", file=sys.stderr)

    fn = 0
    period = 1.0 / cfg.fps
    t_next = time.monotonic()
    while not stop[0] and (cfg.frames == 0 or fn < cfg.frames):
        fn += 1
        # moving bright block (orbit)
        frame_y = y.copy()
        cx = int(W / 2 + (W / 4) * np.cos(fn / 20.0))
        cy = int(H / 2 + (H / 4) * np.sin(fn / 20.0))
        frame_y[max(0, cy - 40):cy + 40, max(0, cx - 40):cx + 40] = 235

        idx = (fn - 1) % N
        off = GLBL_SZ + idx * slot_size
        struct.pack_into("<Q", mm, off, fn * 2 - 1)            # seqlock odd
        t_ns = time.clock_gettime_ns(time.CLOCK_MONOTONIC)
        struct.pack_into("<QIIIIi", mm, off + 8, t_ns, W, H, stride,
                         FOURCC_NV12, frame_bytes)
        mm[off + SLOT_HDR:off + SLOT_HDR + frame_bytes] = (
            frame_y.tobytes() + uv.tobytes())
        struct.pack_into("<Q", mm, off, fn * 2)                # seqlock even
        struct.pack_into("<Q", mm, 64, fn)                     # write_seq
        _libc.sem_post(ctypes.c_void_p(sem))

        t_next += period
        dt = t_next - time.monotonic()
        if dt > 0:
            time.sleep(dt)

    print(f"[feeder] done: {fn} frames", file=sys.stderr)
    mm.close()
    if cfg.cleanup:
        for p in (path, "/dev/shm/sem.ovcam_ready"):
            try:
                os.unlink(p)
            except FileNotFoundError:
                pass


if __name__ == "__main__":
    main()
