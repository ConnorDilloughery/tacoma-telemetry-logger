#!/usr/bin/env python3
"""
Extract Event Clips
--------------------
Detects hard-braking events from the aligned sensor log (using the rate
of change of OBD-reported speed, which is orientation-independent --
unlike the IMU, whose mounting angle relative to the vehicle isn't
calibrated yet) and extracts a short video clip around each event from
the session's dashcam recording.

Design notes:
- Detection threshold is in mph per second of deceleration. -10 mph/s
  (~ -0.45g) is a reasonable starting point for "hard brake" -- tune
  with --threshold if it's too sensitive/not sensitive enough once you
  have real drives to compare against.
- Nearby detections (within --merge-window seconds) are merged into a
  single event, since one real brake event usually triggers several
  consecutive samples past the threshold, not just one.
- Clips are extracted with ffmpeg using -c copy (fast, no re-encoding).
  The tradeoff: copy-mode seeking snaps to the nearest keyframe, so a
  clip's actual start may be off by up to ~1 second from the requested
  time. If you need frame-accurate clips later, drop -c copy and
  re-encode instead (slower, but exact).
- Writes events.json alongside the clips, so generate_drive_report.py
  can read it back and annotate plots / list clips in the README
  without redoing detection.

Requires: pandas, ffmpeg installed on the system (sudo apt install ffmpeg)

Usage:
    python3 extract_event_clips.py --aligned drives/<session>/aligned.csv \\
        --video video_<session>.mp4 --meta video_<session>_meta.json \\
        --out-dir drives/<session>
"""

import argparse
import json
import subprocess
from datetime import datetime
from pathlib import Path

import pandas as pd


def fix_clock_jumps(df: pd.DataFrame, jump_threshold_s: float = 60.0) -> pd.DataFrame:
    """
    Shifts timestamps to remove large discontinuities (this Pi has no
    battery-backed real-time clock and can jump its clock forward by
    hours mid-session once NTP syncs -- see the matching function in
    generate_drive_report.py and ekf_fusion.py for the full
    explanation). Without this, an event genuinely detected within the
    first few minutes of a real drive could get timestamped on the
    wrong (post-jump) side of a multi-hour gap, causing
    generate_drive_report.py's red event markers to plot nowhere near
    the actual speed data.
    """
    df = df.sort_values("timestamp").reset_index(drop=True)
    dt = df["timestamp"].diff()
    jump_mask = dt.dt.total_seconds() > jump_threshold_s
    if not jump_mask.any():
        return df

    corrected = df["timestamp"].copy()
    for jump_idx in df.index[jump_mask]:
        jump_size = dt.loc[jump_idx]
        corrected.loc[jump_idx:] = corrected.loc[jump_idx:] - jump_size
        print(f"  corrected a {jump_size.total_seconds():.1f}s clock jump at row {jump_idx}")

    df["timestamp"] = corrected
    return df


def find_brake_events(df: pd.DataFrame, threshold: float, merge_window: float):
    """
    Returns a list of (event_time, peak_decel_rate) tuples.

    threshold: mph per second (negative = deceleration); events at or
               below this rate are flagged.
    merge_window: seconds; detections within this gap of each other are
                  treated as one event, keeping the point of strongest
                  deceleration as the event's timestamp.
    """
    speed_col = "obd_speed"
    if speed_col not in df.columns:
        return []

    cols = ["timestamp", speed_col]
    # Carry the session-boundary marker through if combine_sessions.py
    # added one (as imu_source_session, since IMU is the base timeline
    # in align_logs.py's merge). Its presence means this data may
    # splice together separately-recorded sessions with a real, dead
    # time gap between them -- not the same thing as a single
    # session's own clock-jump artifact. Computing a rate of change
    # across that boundary divides a real (possibly large) speed
    # change by a corrected, near-zero dt, manufacturing an impossible
    # deceleration out of two genuinely unremarkable readings on
    # either side of a real pause (confirmed: a real 59.7->1.2 mph
    # change across a real ~24-minute gap between chained sessions
    # was reported as a physically impossible "-141.6 mph/s" event).
    session_col = "imu_source_session" if "imu_source_session" in df.columns else None
    if session_col:
        cols.append(session_col)

    sub = df[cols].dropna(subset=["timestamp", speed_col]).reset_index(drop=True)
    if len(sub) < 2:
        return []

    sub["dt"] = sub["timestamp"].diff().dt.total_seconds()
    sub["dspeed"] = sub[speed_col].diff()
    # avoid divide-by-zero on duplicate/near-duplicate timestamps
    sub["decel_rate"] = sub["dspeed"] / sub["dt"].replace(0, pd.NA)

    if session_col:
        crosses_boundary = sub[session_col] != sub[session_col].shift(1)
        sub.loc[crosses_boundary, "decel_rate"] = pd.NA

    flagged = sub[sub["decel_rate"] <= threshold].reset_index(drop=True)
    if flagged.empty:
        return []

    events = []
    group_start_idx = 0
    for i in range(1, len(flagged) + 1):
        at_end = i == len(flagged)
        gap = None if at_end else (flagged.loc[i, "timestamp"] - flagged.loc[i - 1, "timestamp"]).total_seconds()
        if at_end or gap > merge_window:
            group = flagged.iloc[group_start_idx:i]
            peak_row = group.loc[group["decel_rate"].idxmin()]
            events.append((peak_row["timestamp"], peak_row["decel_rate"]))
            group_start_idx = i

    return events


def extract_clip(video_path: str, video_start: datetime, event_time: datetime,
                  before: float, after: float, out_path: str) -> bool:
    offset = (event_time - video_start).total_seconds()
    clip_start = max(0.0, offset - before)
    duration = before + after

    cmd = [
        "ffmpeg", "-y",
        "-ss", str(clip_start),
        "-i", video_path,
        "-t", str(duration),
        "-c", "copy",
        out_path,
    ]
    result = subprocess.run(cmd, capture_output=True)
    return result.returncode == 0


def main():
    parser = argparse.ArgumentParser(description="Detect hard-brake events and extract video clips around them.")
    parser.add_argument("--aligned", required=True, help="Path to the aligned session CSV")
    parser.add_argument("--video", help="Path to the session's video file (skip clip extraction if omitted)")
    parser.add_argument("--meta", help="Path to the video's _meta.json (required if --video is given)")
    parser.add_argument("--out-dir", required=True, help="Directory to write events.json and clips into")
    parser.add_argument("--threshold", type=float, default=-10.0,
                         help="Deceleration threshold in mph per second (default: -10)")
    parser.add_argument("--merge-window", type=float, default=3.0,
                         help="Seconds within which detections are merged into one event (default: 3)")
    parser.add_argument("--before", type=float, default=2.0, help="Seconds of clip before the event (default: 2)")
    parser.add_argument("--after", type=float, default=3.0, help="Seconds of clip after the event (default: 3)")
    args = parser.parse_args()

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    df = pd.read_csv(args.aligned)
    df["timestamp"] = pd.to_datetime(df["timestamp"])
    df = fix_clock_jumps(df)

    events = find_brake_events(df, args.threshold, args.merge_window)
    print(f"Detected {len(events)} hard-braking event(s)")

    video_start = None
    if args.video and args.meta:
        with open(args.meta) as f:
            meta = json.load(f)
        video_start = datetime.fromisoformat(meta["start_time"])

    events_out = []
    for i, (event_time, decel_rate) in enumerate(events, start=1):
        entry = {
            "index": i,
            "event_time": event_time.isoformat(),
            "decel_rate_mph_per_s": round(float(decel_rate), 2),
            "clip_file": None,
        }

        if video_start is not None:
            clip_name = f"brake_clip_{i}.mp4"
            clip_path = out_dir / clip_name
            success = extract_clip(args.video, video_start, event_time, args.before, args.after, str(clip_path))
            if success:
                entry["clip_file"] = clip_name
                print(f"  Event {i}: {event_time.isoformat()} ({decel_rate:.1f} mph/s) -> {clip_name}")
            else:
                print(f"  Event {i}: {event_time.isoformat()} ({decel_rate:.1f} mph/s) -> clip extraction failed")
        else:
            print(f"  Event {i}: {event_time.isoformat()} ({decel_rate:.1f} mph/s) -- no video provided")

        events_out.append(entry)

    events_path = out_dir / "events.json"
    with open(events_path, "w") as f:
        json.dump(events_out, f, indent=2)
    print(f"\nWrote {events_path}")


if __name__ == "__main__":
    main()
