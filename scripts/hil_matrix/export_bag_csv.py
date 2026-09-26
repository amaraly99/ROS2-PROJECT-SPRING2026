#!/usr/bin/env python3
"""Dump a bag's /sim/drone_pose and /slam/pose streams to plain CSV.

Standalone -- takes a single bag path directly, no manifest.json or matrix
run required. Uses the exact same rosbags.AnyReader extraction as
aggregate_matrix.py's load_bag(), so numbers match the official pipeline;
this is for inspecting/plotting/computing RMSE yourself, not a second
implementation of the metric.

    python3 export_bag_csv.py bags/run_.../  [--out-dir .]

Writes <out-dir>/<bag_name>_gt.csv   (t, x, y, z, yaw)      -- ground truth
       <out-dir>/<bag_name>_slam.csv (t, x, y, z, qx,qy,qz,qw) -- SLAM estimate

GT rows are NOT filtered -- every /sim/drone_pose message is written as-is,
including a run where the position never changes (that IS the finding, and
filtering it out would hide it). SLAM rows drop the exact-(0,0,0) placeholder
poses OV2SLAM/ORB-SLAM2 publish before they have a real map, matching
aggregate_matrix.py's own load_bag().
"""
import argparse
import csv
from pathlib import Path

from rosbags.highlevel import AnyReader


def export(bag_path, out_dir):
    bag_path = Path(bag_path)
    # Trial bags are laid out as <run>/bag/metadata.yaml (see run_stack_hil.sh /
    # aggregate_matrix.py's own load_bag(run / 'bag')). Accept either the outer
    # run directory or the inner one so this doesn't need to be remembered.
    if not (bag_path / 'metadata.yaml').exists() and (bag_path / 'bag' / 'metadata.yaml').exists():
        bag_path = bag_path / 'bag'
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    name = bag_path.name if bag_path.name != 'bag' else bag_path.parent.name

    gt_rows, slam_rows = [], []
    with AnyReader([bag_path]) as r:
        conns = [c for c in r.connections
                 if c.topic in ('/sim/drone_pose', '/slam/pose')]
        for c, ts, raw in r.messages(connections=conns):
            m, t = r.deserialize(raw, c.msgtype), ts * 1e-9
            if c.topic == '/sim/drone_pose':
                d = m.data
                if len(d) >= 5:
                    gt_rows.append((t, d[0], d[1], d[2], d[4]))
                elif len(d) >= 3:
                    gt_rows.append((t, d[0], d[1], d[2], float('nan')))
            else:
                p, q = m.pose.position, m.pose.orientation
                if p.x == p.y == p.z == 0:
                    continue
                slam_rows.append((t, p.x, p.y, p.z, q.x, q.y, q.z, q.w))

    gt_rows.sort(key=lambda row: row[0])
    slam_rows.sort(key=lambda row: row[0])

    gt_path = out_dir / f'{name}_gt.csv'
    with open(gt_path, 'w', newline='') as fh:
        w = csv.writer(fh)
        w.writerow(['t', 'x', 'y', 'z', 'yaw'])
        w.writerows(gt_rows)

    slam_path = out_dir / f'{name}_slam.csv'
    with open(slam_path, 'w', newline='') as fh:
        w = csv.writer(fh)
        w.writerow(['t', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'])
        w.writerows(slam_rows)

    print(f'GT:   {len(gt_rows)} rows -> {gt_path}')
    print(f'SLAM: {len(slam_rows)} rows -> {slam_path}')
    if gt_rows:
        uniq = len({(x, y, z) for _, x, y, z, _ in gt_rows})
        print(f'GT unique (x,y,z) positions: {uniq} / {len(gt_rows)}'
              + ('  <-- GT never moved, RMSE against this bag is meaningless'
                 if uniq <= 1 else ''))


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('bag', help='path to the bag directory (the one containing metadata.yaml)')
    ap.add_argument('--out-dir', default='.', help='directory to write the two CSVs into')
    args = ap.parse_args()
    export(args.bag, args.out_dir)


if __name__ == '__main__':
    main()
