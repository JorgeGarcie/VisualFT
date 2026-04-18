#!/usr/bin/env python3
"""Extract MCAP bag files into the standard rawdata format.

Output per bag:
    {output_dir}/{bag_name}/
    ├── frames/              # frame_XXXXXX.jpg
    ├── camera_frames.csv    # time, frame_number, image_path
    ├── tcp_pose.csv         # time, x, y, z, qx, qy, qz, qw
    └── wrench_data.csv      # time, sensor, fx, fy, fz, tx, ty, tz

Usage:
    python extract_mcap.py /path/to/data/singlep1/000_scan/
    python extract_mcap.py /path/to/data/  --all   # process all bags under data/
"""

import argparse
import csv
import os
import struct
import sys
from pathlib import Path

import numpy as np
from mcap_ros2.reader import read_ros2_messages


DEFAULT_OUTPUT = Path(__file__).resolve().parent.parent / "classifier" / "TendonClassifier" / "scripts" / "labeling" / "rawdata"


def extract_bag(bag_dir: Path, output_dir: Path):
    bag_dir = Path(bag_dir)
    # Name from grandparent: data/<name>/000_scan/ → <name>
    bag_name = bag_dir.parent.name if bag_dir.name.startswith("000_") else bag_dir.name
    out = output_dir / bag_name
    frames_dir = out / "frames"
    frames_dir.mkdir(parents=True, exist_ok=True)

    print(f"Extracting: {bag_dir}")
    print(f"Output:     {out}")

    wrench_rows = []
    pose_rows = []
    frame_rows = []
    frame_idx = 0
    t0 = None

    # Find the .mcap file inside the bag directory
    mcap_files = list(bag_dir.glob("*.mcap"))
    if not mcap_files:
        print(f"  No .mcap file found in {bag_dir}, skipping")
        return
    mcap_file = mcap_files[0]

    for msg in read_ros2_messages(str(mcap_file)):
        topic = msg.channel.topic
        t = msg.log_time  # datetime object
        if t0 is None:
            t0 = t
        t_sec = (t - t0).total_seconds()

        if topic == "/coinft/wrench":
            w = msg.ros_msg.wrench
            wrench_rows.append([
                t_sec, "coinft",
                w.force.x, w.force.y, w.force.z,
                w.torque.x, w.torque.y, w.torque.z,
            ])

        elif topic in ("/rdk/wrench", "/flexiv/wrench"):
            w = msg.ros_msg.wrench
            wrench_rows.append([
                t_sec, "flexiv",
                w.force.x, w.force.y, w.force.z,
                w.torque.x, w.torque.y, w.torque.z,
            ])

        elif topic in ("/rdk/tcp_pose", "/flexiv/tcp_pose"):
            p = msg.ros_msg.pose
            pose_rows.append([
                t_sec,
                p.position.x, p.position.y, p.position.z,
                p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w,
            ])

        elif topic == "/image_raw":
            img_msg = msg.ros_msg
            # ROS Image → raw RGB/BGR → JPEG
            h, w_px = img_msg.height, img_msg.width
            encoding = img_msg.encoding
            raw = bytes(img_msg.data)

            fname = f"frame_{frame_idx:06d}.jpg"
            fpath = frames_dir / fname

            try:
                import cv2
                if encoding in ("rgb8",):
                    arr = np.frombuffer(raw, dtype=np.uint8).reshape(h, w_px, 3)
                    arr = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
                elif encoding in ("bgr8",):
                    arr = np.frombuffer(raw, dtype=np.uint8).reshape(h, w_px, 3)
                elif encoding in ("mono8",):
                    arr = np.frombuffer(raw, dtype=np.uint8).reshape(h, w_px)
                else:
                    arr = np.frombuffer(raw, dtype=np.uint8).reshape(h, w_px, -1)
                cv2.imwrite(str(fpath), arr)
            except ImportError:
                # Fallback: save raw PNG via PIL
                from PIL import Image
                if encoding in ("rgb8",):
                    img = Image.frombytes("RGB", (w_px, h), raw)
                elif encoding in ("bgr8",):
                    arr = np.frombuffer(raw, dtype=np.uint8).reshape(h, w_px, 3)
                    img = Image.fromarray(arr[:, :, ::-1])
                else:
                    img = Image.frombytes("L", (w_px, h), raw)
                img.save(str(fpath))

            frame_rows.append([t_sec, frame_idx, f"frames/{fname}"])
            frame_idx += 1

    # Sort wrench by time (coinft and flexiv interleaved)
    wrench_rows.sort(key=lambda r: r[0])

    # Write CSVs
    with open(out / "wrench_data.csv", "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["time", "sensor", "fx", "fy", "fz", "tx", "ty", "tz"])
        writer.writerows(wrench_rows)

    with open(out / "tcp_pose.csv", "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["time", "x", "y", "z", "qx", "qy", "qz", "qw"])
        writer.writerows(pose_rows)

    with open(out / "camera_frames.csv", "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["time", "frame_number", "image_path"])
        writer.writerows(frame_rows)

    print(f"  Wrench:  {len(wrench_rows)} rows")
    print(f"  Poses:   {len(pose_rows)} rows")
    print(f"  Frames:  {frame_idx} images")
    print(f"  Done: {out}\n")


def main():
    parser = argparse.ArgumentParser(description="Extract MCAP bags to rawdata format")
    parser.add_argument("path", type=Path, help="Bag directory or parent data directory")
    parser.add_argument("--all", action="store_true",
                        help="Process all */000_scan/ bags under the given path")
    parser.add_argument("-o", "--output", type=Path, default=DEFAULT_OUTPUT,
                        help="Output directory")
    args = parser.parse_args()

    if args.all:
        # Find bags: either */000_scan/ (old) or any subdir with .mcap files (new)
        bags = sorted(args.path.glob("*/000_scan/"))
        if not bags:
            bags = sorted(
                d for d in args.path.iterdir()
                if d.is_dir() and list(d.glob("*.mcap"))
            )
        if not bags:
            print(f"No bags found under {args.path}")
            sys.exit(1)
        for bag in bags:
            extract_bag(bag, args.output)
    else:
        extract_bag(args.path, args.output)


if __name__ == "__main__":
    main()
