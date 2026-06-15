#!/usr/bin/env python3
"""Extract all frames from an MP4 into a formatted_dataset directory."""

from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path

import cv2


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Extract every frame from a video into a formatted_dataset directory."
    )
    parser.add_argument("video", help="Path to the input MP4/video file.")
    parser.add_argument(
        "--output",
        default="formatted_dataset",
        help="Output directory to create. Default: formatted_dataset",
    )
    parser.add_argument(
        "--prefix",
        default="frame",
        help="Frame filename prefix. Default: frame",
    )
    parser.add_argument(
        "--ext",
        choices=("png", "jpg", "jpeg"),
        default="png",
        help="Image format for extracted frames. Default: png",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    video_path = Path(args.video).expanduser().resolve()
    if not video_path.is_file():
        print(f"Video not found: {video_path}", file=sys.stderr)
        return 1

    output_root = Path(args.output).expanduser().resolve()
    images_dir = output_root / "images"
    images_dir.mkdir(parents=True, exist_ok=True)

    capture = cv2.VideoCapture(str(video_path))
    if not capture.isOpened():
        print(f"Could not open video: {video_path}", file=sys.stderr)
        return 1

    fps = capture.get(cv2.CAP_PROP_FPS) or 0.0
    metadata_path = output_root / "frames.csv"

    frame_count = 0
    with metadata_path.open("w", newline="", encoding="utf-8") as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(["frame_index", "timestamp_sec", "image_path"])

        while True:
            ok, frame = capture.read()
            if not ok:
                break

            image_name = f"{args.prefix}_{frame_count:06d}.{args.ext}"
            image_path = images_dir / image_name

            if not cv2.imwrite(str(image_path), frame):
                print(f"Failed to write image: {image_path}", file=sys.stderr)
                capture.release()
                return 1

            timestamp_sec = frame_count / fps if fps > 0 else ""
            writer.writerow([frame_count, timestamp_sec, image_path.name])
            frame_count += 1

    capture.release()

    readme_path = output_root / "README.txt"
    readme_path.write_text(
        "\n".join(
            [
                "formatted_dataset",
                f"source_video={video_path}",
                f"frame_count={frame_count}",
                f"fps={fps}",
                "images_dir=images",
                "metadata=frames.csv",
            ]
        )
        + "\n",
        encoding="utf-8",
    )

    print(f"Saved {frame_count} frames to {images_dir}")
    print(f"Frame metadata written to {metadata_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
