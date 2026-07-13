#!/usr/bin/env python3
"""
Extract /sim/drone_pose from a ROS2 bag and write a TUM ground-truth file.

TUM format (one line per pose):
    timestamp  tx  ty  tz  qx  qy  qz  qw

/sim/drone_pose is std_msgs/Float64MultiArray with:
    data[0] = x (m)
    data[1] = y (m)
    data[2] = z (m)
    data[3] = pitch (rad)
    data[4] = yaw   (rad)
    roll is assumed 0 (Simulink model publishes no roll)

Timestamps come from the bag receive time (nanoseconds → decimal seconds).

Usage (inside Docker):
    python3 /workspace/scripts/bag_to_tum_gt.py \
        /workspace/bags/hil_full_20260606_002324 \
        /workspace/bags/hil_full_20260606_002324/gt_tum.txt

Or with --topic to override the default topic name.
"""

import argparse
import math
import struct
import sys


def euler_to_quat(roll: float, pitch: float, yaw: float):
    """ZYX Euler → quaternion (qx, qy, qz, qw)."""
    cy, sy = math.cos(yaw * 0.5),   math.sin(yaw * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cr, sr = math.cos(roll * 0.5),  math.sin(roll * 0.5)
    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return qx, qy, qz, qw


def decode_float64_multiarray(cdr_bytes: bytes) -> list:
    """
    Minimal CDR deserializer for std_msgs/Float64MultiArray.
    CDR layout (little-endian after the 4-byte encapsulation header):
      uint32  layout.dim.size  (0 for empty dim)
      uint32  layout.data_offset
      uint32  data.size
      float64 data[0..n-1]
    """
    # skip 4-byte CDR encapsulation header
    offset = 4

    # layout.dim array length
    dim_len = struct.unpack_from("<I", cdr_bytes, offset)[0]
    offset += 4
    # skip each MultiArrayDimension (label string + 2x uint32)
    for _ in range(dim_len):
        str_len = struct.unpack_from("<I", cdr_bytes, offset)[0]
        offset += 4 + str_len
        if str_len % 4:
            offset += 4 - (str_len % 4)  # 4-byte alignment
        offset += 8  # size + stride (2x uint32)

    # layout.data_offset
    offset += 4

    # data array
    data_len = struct.unpack_from("<I", cdr_bytes, offset)[0]
    offset += 4
    values = list(struct.unpack_from(f"<{data_len}d", cdr_bytes, offset))
    return values


def main():
    parser = argparse.ArgumentParser(description="ROS2 bag → TUM ground-truth file")
    parser.add_argument("bag_path", help="Path to the bag directory")
    parser.add_argument("output", help="Output TUM text file path")
    parser.add_argument("--topic", default="/sim/drone_pose",
                        help="Topic name (default: /sim/drone_pose)")
    args = parser.parse_args()

    try:
        import rosbag2_py
    except ImportError:
        sys.exit("rosbag2_py not found — run this script inside the Docker container")

    storage_options = rosbag2_py.StorageOptions(uri=args.bag_path, storage_id="mcap")
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )

    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    topic_types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if args.topic not in topic_types:
        available = [t for t in topic_types if "pose" in t or "sim" in t]
        sys.exit(
            f"Topic '{args.topic}' not found in bag.\n"
            f"Pose-like topics present: {available}"
        )

    filter_ = rosbag2_py.StorageFilter(topics=[args.topic])
    reader.set_filter(filter_)

    rows = []
    while reader.has_next():
        topic, data, timestamp_ns = reader.read_next()
        if topic != args.topic:
            continue
        try:
            values = decode_float64_multiarray(data)
        except Exception as e:
            print(f"[warn] Failed to decode message at t={timestamp_ns}: {e}", file=sys.stderr)
            continue
        if len(values) < 5:
            print(f"[warn] Short message ({len(values)} values) at t={timestamp_ns} — skipped",
                  file=sys.stderr)
            continue

        x, y, z = values[0], values[1], values[2]
        pitch, yaw = values[3], values[4]
        qx, qy, qz, qw = euler_to_quat(0.0, pitch, yaw)
        t_sec = timestamp_ns * 1e-9
        rows.append((t_sec, x, y, z, qx, qy, qz, qw))

    if not rows:
        sys.exit(f"No messages found on topic '{args.topic}'")

    with open(args.output, "w") as f:
        f.write("# TUM ground-truth — extracted from /sim/drone_pose\n")
        f.write("# timestamp tx ty tz qx qy qz qw\n")
        for t, tx, ty, tz, qx, qy, qz, qw in rows:
            f.write(f"{t:.9f} {tx:.6f} {ty:.6f} {tz:.6f} "
                    f"{qx:.6f} {qy:.6f} {qz:.6f} {qw:.6f}\n")

    print(f"[bag_to_tum_gt] Wrote {len(rows)} poses → {args.output}")


if __name__ == "__main__":
    main()
