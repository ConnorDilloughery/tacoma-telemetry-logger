#!/usr/bin/env python3
"""
Concatenates the raw obd/gnss/imu CSVs from several chained sessions
(each capped at MAX_SESSION_DURATION_S by ignition_watcher.py, see the
engineering log) back into one combined drive's raw sensor files.

This only makes sense for sessions from the SAME boot with no reboot
in between -- the Pi's clock is continuous across chained sessions
(only the recording script restarts, not the system itself), so their
raw timestamps already line up correctly when concatenated in order.
If a reboot happened between two of the sessions you're combining,
don't combine across that boundary -- treat them as separate drives.

Usage:
    python3 combine_sessions.py <output_session_id> <session1> <session2> ... <sessionN>

Sessions must be listed in chronological order. Each session's raw
files are expected at drives/<session>/raw/<type>_<session>.csv
(where process_drive.sh already moved them). The combined output is
written to obd_<output_session_id>.csv, gnss_<output_session_id>.csv,
and imu_<output_session_id>.csv in the current directory, ready to
run through process_drive.sh exactly like a normal single session.
"""
import sys
from pathlib import Path

import pandas as pd

SENSOR_TYPES = ["obd", "gnss", "imu"]


def main():
    if len(sys.argv) < 4:
        print(f"Usage: {sys.argv[0]} <output_session_id> <session1> <session2> ... <sessionN>")
        sys.exit(1)

    output_id = sys.argv[1]
    sessions = sys.argv[2:]
    print(f"Combining {len(sessions)} sessions into {output_id}:")
    for s in sessions:
        print(f"  - {s}")

    for sensor in SENSOR_TYPES:
        frames = []  # list of (session_id, dataframe) tuples, kept in sync
        for session in sessions:
            path = Path("drives") / session / "raw" / f"{sensor}_{session}.csv"
            if not path.exists():
                print(f"  WARNING: {path} not found -- skipping this session for {sensor}")
                continue
            df = pd.read_csv(path)
            frames.append((session, df))

        if not frames:
            print(f"  No {sensor} data found across any listed session -- skipping {sensor} output.")
            continue

        combined = pd.concat([f for _, f in frames], ignore_index=True)
        # Track which original session each row came from. This isn't
        # just bookkeeping: a rate-of-change calculation (hard-brake
        # detection, or any future derivative-based check) must never
        # be computed ACROSS a session boundary. The gap between two
        # combined sessions is real, physical dead time where nothing
        # was recorded (the vehicle sitting between chained 5-minute
        # sessions), not a mislabeled clock the way a single session's
        # own NTP-sync jump is -- fix_clock_jumps() collapsing it down
        # to a near-zero corrected dt is correct for that original
        # case, but applied here it turns a real, unremarkable speed
        # change across a real pause into an impossible instantaneous
        # deceleration (confirmed: a genuine 59.7->1.2 mph change
        # across a real ~24-minute gap was reported as "-141.6 mph/s",
        # a physically nonsensical rate, purely from this mismatch).
        # Built from the same (session, frame) pairs actually
        # concatenated above, so a skipped/missing session (handled
        # above) can never misalign this labeling with the real data.
        combined["source_session"] = sum(([s] * len(f) for s, f in frames), [])
        # Sessions are already chronological by construction, but sort
        # defensively in case of any overlap or out-of-order input.
        if "timestamp" in combined.columns:
            combined["timestamp"] = pd.to_datetime(combined["timestamp"], errors="coerce")
            combined = combined.sort_values("timestamp").reset_index(drop=True)

        out_path = Path(f"{sensor}_{output_id}.csv")
        combined.to_csv(out_path, index=False)
        print(f"  Wrote {len(combined)} rows to {out_path}")

    print(f"\nDone. Now run: ./process_drive.sh {output_id}")


if __name__ == "__main__":
    main()
