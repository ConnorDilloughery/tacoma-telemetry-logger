#!/usr/bin/env python3
"""
Camera Logger
-------------
Continuously records dashcam-style video for the session using the Pi
camera, with autofocus locked to infinity (appropriate for a
windshield-mounted, forward-facing camera). Records the exact wall-clock
start time to a sidecar JSON file so that a later script
(extract_event_clips.py) can convert a sensor-log timestamp into the
correct offset within the video and pull a short clip around it.

Design notes:
- Uses rpicam-vid with --codec libav to write directly to an .mp4
  container (not raw .h264). This matters for later clip extraction:
  mp4's own timestamp/index data is what lets ffmpeg seek accurately;
  raw .h264 has no reliable seek points on its own.
- Resolution/framerate default to 1536x864 @ 30fps -- enough detail to
  make sense of a clip, without file sizes exploding at the sensor's
  full 4608x2592 resolution over a whole drive.
- Focus is locked to infinity (--autofocus-mode manual --lens-position 0)
  so the video doesn't hunt for focus while driving.

Usage:
    python3 camera_logger.py --out video_<session>.mp4 [--duration 60]
"""

import argparse
import json
import subprocess
from datetime import datetime


def main():
    parser = argparse.ArgumentParser(description="Record dashcam video with a timestamped start marker.")
    parser.add_argument("--out", required=True, help="Output .mp4 path")
    parser.add_argument("--width", type=int, default=1536)
    parser.add_argument("--height", type=int, default=864)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--duration", type=float, default=None,
                         help="Seconds to record (default: run until stopped)")
    args = parser.parse_args()

    start_time = datetime.now()
    meta_path = args.out.rsplit(".", 1)[0] + "_meta.json"
    with open(meta_path, "w") as f:
        json.dump({"video_path": args.out, "start_time": start_time.isoformat()}, f)

    # -t 0 means "record until stopped" (SIGINT/SIGTERM); rpicam-vid
    # handles a clean stop on those signals and finalizes the mp4
    # properly rather than leaving it truncated/unplayable.
    timeout_ms = int(args.duration * 1000) if args.duration else 0

    cmd = [
        "rpicam-vid",
        "--width", str(args.width),
        "--height", str(args.height),
        "--framerate", str(args.fps),
        "--codec", "libav",
        "--autofocus-mode", "manual",
        "--lens-position", "0",
        "--nopreview",
        "-o", args.out,
        "-t", str(timeout_ms),
    ]

    print(f"Recording to {args.out}")
    print(f"Start time recorded: {start_time.isoformat()} (meta: {meta_path})")

    try:
        subprocess.run(cmd)
    except KeyboardInterrupt:
        pass

    print("Recording stopped.")


if __name__ == "__main__":
    main()
