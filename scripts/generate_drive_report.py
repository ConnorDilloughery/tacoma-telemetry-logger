#!/usr/bin/env python3
"""
Generate Drive Report
-----------------------
Reads an aligned session CSV (and optional events.json from
extract_event_clips.py), computes summary stats, produces plots, and
writes a per-drive README.md suitable for browsing directly on GitHub.

Design notes:
- Distance traveled is computed with the haversine formula over
  consecutive GNSS lat/lon points -- a standard great-circle distance
  calculation, accurate enough for this purpose without needing an
  external geo library.
- Plots are saved as PNGs alongside the CSV in the same drive folder,
  and the README references them with relative paths, which is what
  makes them render inline when this folder is pushed to GitHub.
- stats.json is written separately from the README so update_index.py
  (which builds the top-level gallery across all drives) can read
  structured data instead of having to parse markdown.

Requires: pandas, matplotlib

Usage:
    python3 generate_drive_report.py --aligned drives/<session>/aligned.csv \\
        --out-dir drives/<session> --session <session> [--events drives/<session>/events.json]
"""

import argparse
import json
import math
from pathlib import Path

import pandas as pd
import numpy as np
import matplotlib
matplotlib.use("Agg")  # headless -- no display available on the Pi (or in CI)
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection


def haversine_miles(lat1, lon1, lat2, lon2):
    R = 3958.8  # Earth radius, miles
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dlambda / 2) ** 2
    return 2 * R * math.asin(math.sqrt(a))


def valid_gnss_points(df: pd.DataFrame) -> pd.DataFrame:
    """
    Filters out GNSS rows that shouldn't be trusted as real fixes:
    - missing lat/lon
    - exactly (0, 0) -- "null island", the value a GPS module/parser
      often reports before it has acquired a real satellite fix, not
      an actual location. Left unfiltered, one of these at the start
      of a log turns a normal drive's distance/route into a nonsense
      line stretching to the Gulf of Guinea.
    - a fix_quality of 0, if that column is present (0 = no fix per
      the GGA sentence spec)
    """
    cols = ["gnss_latitude", "gnss_longitude"]
    if not all(c in df.columns for c in cols):
        return df.iloc[0:0]  # empty frame, same columns

    valid = df.dropna(subset=cols).copy()
    valid = valid[~((valid["gnss_latitude"] == 0) & (valid["gnss_longitude"] == 0))]
    if "gnss_fix_quality" in valid.columns:
        valid = valid[(valid["gnss_fix_quality"].isna()) | (valid["gnss_fix_quality"] >= 1)]
    return valid


def compute_distance_miles(df: pd.DataFrame) -> float:
    points = valid_gnss_points(df)[["gnss_latitude", "gnss_longitude"]]
    if len(points) < 2:
        return 0.0
    total = 0.0
    prev = None
    for _, row in points.iterrows():
        if prev is not None:
            total += haversine_miles(prev[0], prev[1], row["gnss_latitude"], row["gnss_longitude"])
        prev = (row["gnss_latitude"], row["gnss_longitude"])
    return total


def plot_speed(df: pd.DataFrame, events: list, out_path: Path):
    fig, ax = plt.subplots(figsize=(10, 4))
    if "obd_speed" in df.columns:
        ax.plot(df["timestamp"], df["obd_speed"], label="OBD speed (mph)")
    for e in events:
        ax.axvline(pd.to_datetime(e["event_time"]), color="red", linestyle="--", alpha=0.6)
    ax.set_xlabel("Time")
    ax.set_ylabel("Speed (mph)")
    ax.set_title("Speed over time (red = hard-brake event)")
    ax.legend()
    fig.autofmt_xdate()
    fig.tight_layout()
    fig.savefig(out_path, dpi=120)
    plt.close(fig)


def plot_rpm(df: pd.DataFrame, out_path: Path):
    if "obd_rpm" not in df.columns:
        return
    fig, ax = plt.subplots(figsize=(10, 4))
    ax.plot(df["timestamp"], df["obd_rpm"], color="darkorange")
    ax.set_xlabel("Time")
    ax.set_ylabel("RPM")
    ax.set_title("Engine RPM over time")
    fig.autofmt_xdate()
    fig.tight_layout()
    fig.savefig(out_path, dpi=120)
    plt.close(fig)


def plot_route(df: pd.DataFrame, out_path: Path):
    """
    2D route map with the path colored by speed (a continuous gradient
    along the line, using the same segment-coloring technique as a 3D
    plot would, just flattened onto lat/lon) so you can see where the
    drive sped up, slowed down, or braked hard directly on the map --
    without needing a 3D view or a separate time-series plot.
    """
    if "obd_speed" in df.columns:
        points = valid_gnss_points(df)[["gnss_longitude", "gnss_latitude", "obd_speed"]].dropna().reset_index(drop=True)
    else:
        points = valid_gnss_points(df)[["gnss_longitude", "gnss_latitude"]].dropna().reset_index(drop=True)

    if len(points) < 2:
        return

    fig, ax = plt.subplots(figsize=(7, 6))

    if "obd_speed" in points.columns:
        x = points["gnss_longitude"].values
        y = points["gnss_latitude"].values
        speed = points["obd_speed"].values

        segments = np.array([x, y]).T.reshape(-1, 1, 2)
        segments = np.concatenate([segments[:-1], segments[1:]], axis=1)

        lc = LineCollection(segments, cmap="plasma", linewidths=3)
        lc.set_array(speed[:-1])
        line = ax.add_collection(lc)
        ax.autoscale()
        cbar = fig.colorbar(line, ax=ax)
        cbar.set_label("Speed (mph)")
    else:
        ax.plot(points["gnss_longitude"], points["gnss_latitude"], color="steelblue", linewidth=1.5)

    ax.scatter(points["gnss_longitude"].iloc[0], points["gnss_latitude"].iloc[0], color="green", label="Start", zorder=5)
    ax.scatter(points["gnss_longitude"].iloc[-1], points["gnss_latitude"].iloc[-1], color="red", label="End", zorder=5)
    ax.set_xlabel("Longitude")
    ax.set_ylabel("Latitude")
    ax.set_title("Route (colored by speed)")
    ax.set_aspect("equal", adjustable="datalim")
    ax.legend()
    fig.tight_layout()
    fig.savefig(out_path, dpi=120)
    plt.close(fig)


def plot_imu_accel(df: pd.DataFrame, out_path: Path):
    accel_cols = ["imu_accel_x", "imu_accel_y", "imu_accel_z"]
    if not all(c in df.columns for c in accel_cols):
        return
    fig, ax = plt.subplots(figsize=(10, 4))
    for c, label in zip(accel_cols, ["x", "y", "z"]):
        ax.plot(df["timestamp"], df[c], label=label, linewidth=0.8)
    ax.set_xlabel("Time")
    ax.set_ylabel("Linear acceleration (m/s^2)")
    ax.set_title("IMU linear acceleration")
    ax.legend()
    fig.autofmt_xdate()
    fig.tight_layout()
    fig.savefig(out_path, dpi=120)
    plt.close(fig)



def main():
    parser = argparse.ArgumentParser(description="Generate a per-drive report (stats, plots, README) from an aligned CSV.")
    parser.add_argument("--aligned", required=True, help="Path to the aligned session CSV")
    parser.add_argument("--out-dir", required=True, help="Directory to write plots, stats.json, and README.md")
    parser.add_argument("--session", required=True, help="Session ID (used in the report title)")
    parser.add_argument("--events", help="Path to events.json from extract_event_clips.py (optional)")
    args = parser.parse_args()

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    df = pd.read_csv(args.aligned)
    df["timestamp"] = pd.to_datetime(df["timestamp"])

    events = []
    if args.events and Path(args.events).exists():
        with open(args.events) as f:
            events = json.load(f)

    duration_s = (df["timestamp"].max() - df["timestamp"].min()).total_seconds()
    distance_miles = compute_distance_miles(df)
    max_speed = df["obd_speed"].max() if "obd_speed" in df.columns else None
    max_rpm = df["obd_rpm"].max() if "obd_rpm" in df.columns else None

    stats = {
        "session": args.session,
        "start_time": df["timestamp"].min().isoformat(),
        "duration_s": round(duration_s, 1),
        "distance_miles": round(distance_miles, 2),
        "max_speed_mph": None if max_speed is None or pd.isna(max_speed) else round(float(max_speed), 1),
        "max_rpm": None if max_rpm is None or pd.isna(max_rpm) else round(float(max_rpm), 0),
        "num_hard_brake_events": len(events),
    }

    with open(out_dir / "stats.json", "w") as f:
        json.dump(stats, f, indent=2)

    plot_speed(df, events, out_dir / "speed.png")
    plot_rpm(df, out_dir / "rpm.png")
    plot_route(df, out_dir / "route_map.png")
    plot_imu_accel(df, out_dir / "imu_accel.png")

    # --- README ---
    lines = [
        f"# Drive: {args.session}",
        "",
        f"- **Start time:** {stats['start_time']}",
        f"- **Duration:** {stats['duration_s']:.0f} s ({stats['duration_s']/60:.1f} min)",
        f"- **Distance:** {stats['distance_miles']:.2f} mi",
        f"- **Max speed:** {stats['max_speed_mph']} mph" if stats["max_speed_mph"] is not None else "- **Max speed:** n/a",
        f"- **Max RPM:** {stats['max_rpm']}" if stats["max_rpm"] is not None else "- **Max RPM:** n/a",
        f"- **Hard-braking events detected:** {stats['num_hard_brake_events']}",
        "",
        "## Route",
        "![route](route_map.png)" if (out_dir / "route_map.png").exists() else "_No GPS route available._",
        "",
        "## Speed",
        "![speed](speed.png)" if (out_dir / "speed.png").exists() else "_No speed data available._",
        "",
        "## RPM",
        "![rpm](rpm.png)" if (out_dir / "rpm.png").exists() else "_No RPM data available._",
        "",
        "## IMU Linear Acceleration",
        "![imu](imu_accel.png)" if (out_dir / "imu_accel.png").exists() else "_No IMU data available._",
        "",
    ]

    if events:
        lines.append("## Hard-Braking Events")
        lines.append("")
        for e in events:
            lines.append(f"### Event {e['index']} — {e['event_time']}")
            lines.append(f"Deceleration: {e['decel_rate_mph_per_s']} mph/s")
            if e.get("clip_file"):
                lines.append("")
                lines.append(f"[Watch clip]({e['clip_file']})")
            lines.append("")

    (out_dir / "README.md").write_text("\n".join(lines))
    print(f"Wrote report to {out_dir}/README.md")


if __name__ == "__main__":
    main()
