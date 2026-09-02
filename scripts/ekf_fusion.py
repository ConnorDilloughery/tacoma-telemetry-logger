#!/usr/bin/env python3
"""
EKF Sensor Fusion
------------------
Fuses GNSS position, OBD-II speed, and IMU acceleration/gyro data from
an aligned drive session into a single continuous vehicle state
estimate, using an Extended Kalman Filter (EKF).

State vector: [x, y, heading, speed]
    x, y     -- position in meters, on a local flat-ground coordinate
                system centered on the drive's first valid GPS fix
                (NOT lat/lon directly -- working in meters makes the
                filter math simpler and keeps units consistent)
    heading  -- radians, 0 = pointing along +x
    speed    -- m/s

Process model (predict step, runs every IMU sample ~10-20Hz):
    x'       = x + speed * cos(heading) * dt
    y'       = y + speed * sin(heading) * dt
    heading' = heading + yaw_rate * dt      (yaw_rate from IMU gyro)
    speed'   = speed + accel * dt           (accel from IMU)

Measurement updates (correct step, runs whenever a new reading arrives):
    - GNSS position (x, y), converted from lat/lon -- corrects drift
      in x, y directly
    - OBD speed -- corrects drift in the speed state directly

Known limitation (documented, not hidden): the IMU's mounting
orientation hasn't been calibrated against the vehicle's actual axes.
This script assumes the IMU's local X-axis is roughly the vehicle's
forward direction (for acceleration) and its Z-axis is roughly
vertical (for yaw rate). That's a reasonable starting assumption for a
box sitting flat in the cabin, but it's an approximation, not a
calibrated transform. A logical next step is an explicit IMU-to-vehicle
axis calibration (e.g. comparing IMU heading changes against GPS course
changes during turns) to correct for any real mounting misalignment.

Requires: pandas, numpy, matplotlib

Usage:
    python3 ekf_fusion.py --aligned drives/<session>/aligned.csv \\
        --out-dir drives/<session>
"""

import argparse
import math
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


MPH_TO_MS = 0.44704
DEG_TO_M_LAT = 111320.0  # meters per degree of latitude, roughly constant


def latlon_to_local_xy(lat, lon, lat0, lon0):
    """
    Flat-ground local projection: converts lat/lon into meters on a
    plane tangent to the Earth at (lat0, lon0). Accurate enough for a
    single drive's distance scale (a few miles); not meant for
    anything beyond that.
    """
    x = (lon - lon0) * DEG_TO_M_LAT * math.cos(math.radians(lat0))
    y = (lat - lat0) * DEG_TO_M_LAT
    return x, y


class EKF:
    def __init__(self, x0, y0, heading0, speed0):
        self.state = np.array([x0, y0, heading0, speed0], dtype=float)
        # Initial uncertainty: fairly unsure about everything at the start.
        self.P = np.diag([25.0, 25.0, (math.pi / 2) ** 2, 25.0])

        # Process noise: how much we expect the *model* to be wrong per
        # second, beyond what the IMU-driven prediction already
        # accounts for. Tuned loosely -- these are reasonable starting
        # values, not derived from a noise characterization of this
        # specific IMU/GPS combination.
        self.Q_base = np.diag([0.5, 0.5, 0.05, 0.5])

        # Measurement noise: how much we trust each sensor's reading.
        # GPS position noise ~ a few meters (typical consumer GNSS).
        self.R_gps = np.diag([9.0, 9.0])         # ~3m std dev
        self.R_speed = np.array([[0.5]])          # OBD speed is fairly trustworthy

    def predict(self, dt, accel, yaw_rate):
        x, y, heading, speed = self.state

        # Nonlinear state transition
        x_new = x + speed * math.cos(heading) * dt
        y_new = y + speed * math.sin(heading) * dt
        heading_new = heading + yaw_rate * dt
        speed_new = speed + accel * dt

        self.state = np.array([x_new, y_new, heading_new, speed_new])

        # Jacobian of the state transition (linearization around the
        # current state) -- this is the "Extended" part of EKF: exact
        # for a linear model, a locally-valid approximation here since
        # our motion model is nonlinear (heading-dependent).
        F = np.array([
            [1, 0, -speed * math.sin(heading) * dt, math.cos(heading) * dt],
            [0, 1,  speed * math.cos(heading) * dt, math.sin(heading) * dt],
            [0, 0, 1, 0],
            [0, 0, 0, 1],
        ])

        Q = self.Q_base * dt
        self.P = F @ self.P @ F.T + Q

    def update_gps(self, x_meas, y_meas):
        H = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
        ])
        z = np.array([x_meas, y_meas])
        self._update(z, H, self.R_gps)

    def update_speed(self, speed_meas):
        H = np.array([[0, 0, 0, 1]])
        z = np.array([speed_meas])
        self._update(z, H, self.R_speed)

    def _update(self, z, H, R):
        y = z - H @ self.state  # innovation (measurement residual)
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)  # Kalman gain
        self.state = self.state + K @ y
        self.P = (np.eye(len(self.state)) - K @ H) @ self.P


def main():
    parser = argparse.ArgumentParser(description="Fuse GNSS/OBD/IMU data with an EKF.")
    parser.add_argument("--aligned", required=True, help="Path to the aligned session CSV")
    parser.add_argument("--out-dir", required=True, help="Directory to write fused.csv and the comparison plot")
    args = parser.parse_args()

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    df = pd.read_csv(args.aligned)
    df["timestamp"] = pd.to_datetime(df["timestamp"])
    df = df.sort_values("timestamp").reset_index(drop=True)

    # Filter obviously-invalid GNSS points (same null-island / fix
    # quality logic used in generate_drive_report.py).
    has_gnss = "gnss_latitude" in df.columns and "gnss_longitude" in df.columns
    if has_gnss:
        # Same validity check as generate_drive_report.py's
        # valid_gnss_points(): exact (0,0) is the "null island"
        # placeholder, and fix_quality == 0 means "no fix" per the GGA
        # sentence spec -- a module can report this alongside a
        # nonzero-but-meaningless lat/lon while acquiring satellites,
        # so checking (0,0) alone isn't sufficient. Missing this
        # second check is exactly what let one bad early fix anchor
        # the whole session's coordinate origin on garbage data.
        invalid = (df["gnss_latitude"] == 0) & (df["gnss_longitude"] == 0)
        if "gnss_fix_quality" in df.columns:
            invalid = invalid | (df["gnss_fix_quality"] == 0)
        df.loc[invalid, ["gnss_latitude", "gnss_longitude"]] = np.nan

    # Reference point for the local flat-ground projection: first valid GPS fix.
    valid_gps = df.dropna(subset=["gnss_latitude", "gnss_longitude"]) if has_gnss else pd.DataFrame()
    if valid_gps.empty:
        print("No valid GPS fix in this session -- can't establish a reference point for fusion.")
        return
    first_gps = valid_gps.iloc[0]
    lat0, lon0 = first_gps["gnss_latitude"], first_gps["gnss_longitude"]

    # Initial state: at the origin, facing along +x arbitrarily (we
    # don't have a trustworthy initial heading yet -- it'll converge
    # quickly once GPS updates start correcting it), initial speed
    # from the first available OBD reading or 0.
    initial_speed_mph = df["obd_speed"].dropna().iloc[0] if "obd_speed" in df.columns and df["obd_speed"].notna().any() else 0.0
    ekf = EKF(0.0, 0.0, 0.0, initial_speed_mph * MPH_TO_MS)

    results = []
    prev_time = None
    prev_gps_latlon = None
    prev_obd_speed = None

    for _, row in df.iterrows():
        t = row["timestamp"]
        if prev_time is None:
            dt = 0.0
        else:
            dt = (t - prev_time).total_seconds()
            # Defensive clamp: a system clock jump (e.g. the Pi's clock
            # syncing to NTP mid-session, since it has no
            # battery-backed real-time clock and boots with an
            # arbitrary time until it gets network access) can produce
            # one enormous dt between otherwise-~0.1s-spaced rows. Left
            # unclamped, predict() would multiply speed by that huge dt
            # and teleport the position hundreds of meters in one step
            # -- exactly what happened here (a real 510-second gap
            # between two consecutive samples). Capping dt at 1 second
            # means a clock glitch costs at most one slightly-stale
            # prediction step, not a permanent, unrecoverable jump.
            if dt > 1.0 or dt < 0:
                print(f"  clock discontinuity detected at {t} (dt={dt:.1f}s) -- clamping to 0.1s")
                dt = 0.1
        prev_time = t

        # IMU-driven prediction. accel_x / gyro_z chosen per the
        # documented forward/vertical-axis assumption at the top of
        # this file.
        accel = row["imu_accel_x"] if "imu_accel_x" in row and not pd.isna(row["imu_accel_x"]) else 0.0
        yaw_rate = row["imu_gyro_z"] if "imu_gyro_z" in row and not pd.isna(row["imu_gyro_z"]) else 0.0

        # Zero-Velocity Update (ZUPT): when OBD speed says the vehicle
        # is stopped, we KNOW speed is 0 and heading isn't changing --
        # so we ignore the IMU's accel/gyro readings for this step
        # instead of integrating them. Without this, a tiny gyro bias
        # (a small nonzero reading even when perfectly still) gets
        # integrated into a slowly spinning heading estimate while
        # parked, and once heading is wrong, any residual acceleration
        # bias projects into a runaway position drift with nothing to
        # correct it if GPS updates have also gone stale. This is a
        # standard technique in inertial navigation, not a workaround
        # specific to this dataset.
        obd_speed_now = row["obd_speed"] if "obd_speed" in row and not pd.isna(row["obd_speed"]) else None
        stationary = obd_speed_now is not None and abs(obd_speed_now) < 1.0  # mph

        # OBD data going missing entirely (CAN dropout, common near a
        # drive's end when the vehicle powers down) is just as
        # dangerous as being stopped -- with no speed reference at all
        # to check against, blindly trusting raw IMU accel/gyro for
        # dead reckoning is exactly what produces runaway drift. We
        # can't assume the vehicle is stationary in this case (it might
        # not be), but we CAN stop trusting noisy IMU-only propulsion:
        # hold heading fixed and let speed decay toward 0 rather than
        # integrate unconstrained.
        obd_missing = obd_speed_now is None

        if stationary or obd_missing:
            accel = 0.0
            yaw_rate = 0.0

        if dt > 0:
            ekf.predict(dt, accel, yaw_rate)

        if stationary:
            # Pin speed to 0 with high confidence every step while
            # stopped, not just when a "new" OBD reading arrives --
            # this is what actually prevents drift from accumulating
            # during a long stop, rather than just slowing it down.
            ekf.update_speed(0.0)
        elif obd_missing:
            # No speed reference available at all. Rather than trust
            # whatever speed the filter had right before data dropped
            # (which could be genuinely moving, e.g. mid-CAN-dropout
            # while still driving), gently pull the speed estimate
            # toward 0 over time -- a soft assumption that "probably
            # slowing down/parking" is safer than "keep going at
            # whatever speed I last knew," without hard-committing to
            # a full stop the way the ZUPT branch above does.
            ekf.update_speed(ekf.state[3] * 0.9)

        # GPS correction -- only apply when this is a genuinely new fix
        # (aligned.csv forward-fills GNSS onto the IMU's faster
        # timeline via merge_asof, so most rows just repeat the last
        # fix; applying it as "new evidence" every row would make the
        # filter overconfident in stale data).
        if has_gnss and not pd.isna(row["gnss_latitude"]) and not pd.isna(row["gnss_longitude"]):
            latlon = (row["gnss_latitude"], row["gnss_longitude"])
            if latlon != prev_gps_latlon:
                x_meas, y_meas = latlon_to_local_xy(latlon[0], latlon[1], lat0, lon0)
                ekf.update_gps(x_meas, y_meas)
                prev_gps_latlon = latlon

        # OBD speed correction -- same "only if new" logic.
        if "obd_speed" in row and not pd.isna(row["obd_speed"]):
            if row["obd_speed"] != prev_obd_speed:
                ekf.update_speed(row["obd_speed"] * MPH_TO_MS)
                prev_obd_speed = row["obd_speed"]

        x, y, heading, speed = ekf.state
        results.append({
            "timestamp": t,
            "fused_x_m": x,
            "fused_y_m": y,
            "fused_heading_deg": math.degrees(heading) % 360,
            "fused_speed_mph": speed / MPH_TO_MS,
        })

    fused = pd.DataFrame(results)
    fused_path = out_dir / "fused.csv"
    fused.to_csv(fused_path, index=False)
    print(f"Wrote {len(fused)} fused rows to {fused_path}")

    # --- Comparison plot: raw GPS fixes vs. the EKF's fused path ---
    raw_points = df.dropna(subset=["gnss_latitude", "gnss_longitude"])
    raw_xy = [latlon_to_local_xy(lat, lon, lat0, lon0) for lat, lon in zip(raw_points["gnss_latitude"], raw_points["gnss_longitude"])]

    fig, ax = plt.subplots(figsize=(8, 7))
    if raw_xy:
        rx, ry = zip(*raw_xy)
        ax.scatter(rx, ry, s=15, color="gray", alpha=0.6, label="Raw GPS fixes", zorder=3)
    ax.plot(fused["fused_x_m"], fused["fused_y_m"], color="crimson", linewidth=1.5, label="EKF fused path", zorder=4)
    ax.set_xlabel("Local X (m, east)")
    ax.set_ylabel("Local Y (m, north)")
    ax.set_title("Raw GPS vs. EKF-Fused Trajectory")
    ax.set_aspect("equal", adjustable="datalim")
    ax.legend()
    fig.tight_layout()
    fig.savefig(out_dir / "ekf_comparison.png", dpi=130)
    plt.close(fig)
    print(f"Wrote {out_dir / 'ekf_comparison.png'}")


if __name__ == "__main__":
    main()
