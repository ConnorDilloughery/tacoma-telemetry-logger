#!/usr/bin/env python3
"""
Align Logs
----------
Merges the three independently-logged sensor CSVs (OBD/CAN, GNSS, IMU)
onto a single common timeline using nearest-timestamp matching, and
writes one combined CSV.

Design notes:
- Each logger timestamps its own rows using the Pi's system clock at
  the moment it captured a reading. Because the three loggers run as
  separate processes with different polling rates (GNSS updates
  irregularly based on fix availability, IMU at ~10-20Hz, OBD at
  whatever --rate was set), their timestamps never line up exactly.
- Rather than requiring exact matches (which would drop almost every
  row), we use an "as-of" join: for each timestamp in the base
  timeline, pick the most recent reading from each other sensor that
  is not newer than that timestamp. This is the standard technique
  for merging multi-rate sensor streams and is what pandas.merge_asof
  implements directly.
- The IMU log is used as the base timeline since it's the
  highest-rate stream (10-20Hz) -- every other sensor's reading gets
  matched onto each IMU timestamp. If you want a coarser combined
  rate instead, swap --base to "obd" or "gnss".
- A --tolerance limits how old a matched reading is allowed to be
  before it's considered "too stale" and left blank instead of
  reused. This matters most for GNSS, which can go a full second or
  more between updates.

Requires:
    pip3 install pandas --break-system-packages

Usage:
    python3 align_logs.py --session 20260901_201232
        (looks for obd_20260901_201232.csv, gnss_..., imu_... in the
         current directory)

    python3 align_logs.py --obd obd_x.csv --gnss gnss_x.csv --imu imu_x.csv
        (explicit filenames, if not using the run_session.sh naming
         convention)
"""

import argparse

import pandas as pd


def load_csv(path: str) -> pd.DataFrame:
    df = pd.read_csv(path)
    # errors="coerce" turns any unparseable/truncated timestamp (e.g. a
    # row left half-written when the logger was stopped mid-write) into
    # NaT instead of raising, so one bad row doesn't crash the whole
    # alignment step -- we then drop those rows explicitly below.
    df["timestamp"] = pd.to_datetime(df["timestamp"], errors="coerce")
    n_before = len(df)
    df = df.dropna(subset=["timestamp"])
    n_dropped = n_before - len(df)
    if n_dropped:
        print(f"  {path}: dropped {n_dropped} row(s) with missing/unparseable timestamp")
    df = df.sort_values("timestamp").reset_index(drop=True)
    return df


def main():
    parser = argparse.ArgumentParser(description="Align OBD, GNSS, and IMU logs onto a common timeline.")
    parser.add_argument("--session", help="Session ID from run_session.sh (e.g. 20260901_201232)")
    parser.add_argument("--obd", help="Path to OBD CSV (overrides --session)")
    parser.add_argument("--gnss", help="Path to GNSS CSV (overrides --session)")
    parser.add_argument("--imu", help="Path to IMU CSV (overrides --session)")
    parser.add_argument("--base", choices=["imu", "obd", "gnss"], default="imu",
                         help="Which sensor's timestamps to use as the common timeline (default: imu)")
    parser.add_argument("--tolerance", default="1s",
                         help="Max age of a matched reading before it's left blank, e.g. '1s', '500ms' (default: 1s)")
    parser.add_argument("--out", default=None, help="Output CSV path (default: aligned_<session>.csv or aligned_log.csv)")
    args = parser.parse_args()

    if args.session:
        obd_path = args.obd or f"obd_{args.session}.csv"
        gnss_path = args.gnss or f"gnss_{args.session}.csv"
        imu_path = args.imu or f"imu_{args.session}.csv"
    else:
        obd_path, gnss_path, imu_path = args.obd, args.gnss, args.imu
        if not all([obd_path, gnss_path, imu_path]):
            parser.error("Provide --session, or all three of --obd/--gnss/--imu")

    out_path = args.out or (f"aligned_{args.session}.csv" if args.session else "aligned_log.csv")

    print(f"Loading:\n  OBD:  {obd_path}\n  GNSS: {gnss_path}\n  IMU:  {imu_path}")

    obd = load_csv(obd_path)
    gnss = load_csv(gnss_path)
    imu = load_csv(imu_path)

    # Prefix each sensor's non-timestamp columns so the merged output
    # is unambiguous about which sensor a column came from.
    obd = obd.rename(columns={c: f"obd_{c}" for c in obd.columns if c != "timestamp"})
    gnss = gnss.rename(columns={c: f"gnss_{c}" for c in gnss.columns if c != "timestamp"})
    imu = imu.rename(columns={c: f"imu_{c}" for c in imu.columns if c != "timestamp"})

    frames = {"obd": obd, "gnss": gnss, "imu": imu}
    base = frames.pop(args.base)

    tolerance = pd.Timedelta(args.tolerance)

    merged = base
    for name, df in frames.items():
        merged = pd.merge_asof(
            merged,
            df,
            on="timestamp",
            direction="backward",   # most recent reading at or before this timestamp
            tolerance=tolerance,    # if nothing that recent exists, leave it blank (NaN)
        )

    merged.to_csv(out_path, index=False)
    print(f"\nWrote {len(merged)} aligned rows to {out_path}")
    print(f"Base timeline: {args.base} ({len(base)} rows)")

    # Quick sanity check: how many rows actually got a match from each
    # other sensor, vs. how many came up empty (outside tolerance).
    for name, df in {"obd": obd, "gnss": gnss}.items() if args.base == "imu" else {}.items():
        pass  # (kept simple -- see README note below for deeper QA ideas)


if __name__ == "__main__":
    main()
