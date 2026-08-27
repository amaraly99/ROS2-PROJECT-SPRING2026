#!/usr/bin/env python3
"""Print the number of non-zero /slam/pose entries in a bag.

Exists as a FILE, not an inline `python3 -c`, because a one-liner has to survive
PowerShell -> ssh -> bash quoting and does not: parentheses in the python source
get interpreted by bash and the whole trial fails with
"syntax error near unexpected token `('". That bit twice today.

Used by run_matrix.ps1 to fail a trial whose SLAM never built a map. The
readiness gate proves frames are being CONSUMED; it cannot prove a map was
BUILT. Observed on ov2_fast at startup_delay_sec 2: gate passed, frames flowed,
zero poses produced (1 in 6 attempts). Without this check that bag is recorded
status=ok and silently poisons the arm's mean.

Usage: python3 count_poses.py <bag_dir>
"""
import sys
from pathlib import Path

from rosbags.highlevel import AnyReader

bag = Path(sys.argv[1])
if bag.name != 'bag':
    bag = bag / 'bag'

n = 0
with AnyReader([bag]) as reader:
    conns = [c for c in reader.connections if c.topic == '/slam/pose']
    for conn, _, raw in reader.messages(connections=conns):
        p = reader.deserialize(raw, conn.msgtype).pose.position
        if abs(p.x) + abs(p.y) + abs(p.z) > 1e-9:
            n += 1
print(n)
