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


def fix_clock_jumps(df: pd.DataFrame, jump_threshold_s: float = 60.0) -> pd.DataFrame:
    """
    Shifts timestamps to remove large discontinuities (see the
    matching function/docstring in generate_drive_report.py for the
    full explanation -- this Pi has no battery-backed RTC and can jump
    its clock forward by hours mid-session once NTP syncs). Run this
    once up front rather than relying solely on the dt-clamp in
    predict(): the clamp only stops a single bad step from teleporting
    the position, it doesn't fix every downstream row's timestamp
    still being wrong afterward.
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
        # Cap how much real time the KINEMATIC INTEGRATION is allowed
        # to extrapolate over in one step, even though we still use the
        # true dt to scale process noise below. A single instantaneous
        # IMU snapshot (one accel/yaw-rate reading) is a reasonable
        # stand-in for "what the vehicle was doing" over a couple of
        # seconds, but assuming it held constant for 10+ seconds is not
        # -- real driving continuously changes speed and heading, so
        # extrapolating a stale snapshot across a long real gap (e.g. an
        # I2C/IMU dropout lasting many seconds, confirmed via
        # `--> Nm jump in 10+s` messages this project has actually
        # logged) produces exactly the kind of large single-step
        # "runaway" this fusion pipeline's own tests are designed to
        # catch -- except it isn't corrupted data this time, it's a
        # genuinely bad extrapolation assumption over a real gap.
        # Capping the integration dt means the filter simply stops
        # moving the position estimate forward once a gap gets long
        # enough that the constant-velocity/constant-turn-rate
        # assumption stops being reasonable, and instead leans on
        # inflated uncertainty (using the FULL true dt for Q below) so
        # the next real GPS/OBD measurement -- now also protected by
        # the innovation gate above -- corrects it properly instead of
        # the filter confidently reporting a wild extrapolation.
        INTEGRATION_DT_CAP_S = 3.0
        integration_dt = min(dt, INTEGRATION_DT_CAP_S)

        x, y, heading, speed = self.state

        # Nonlinear state transition
        x_new = x + speed * math.cos(heading) * integration_dt
        y_new = y + speed * math.sin(heading) * integration_dt
        heading_new = heading + yaw_rate * integration_dt
        speed_new = speed + accel * integration_dt

        self.state = np.array([x_new, y_new, heading_new, speed_new])
        self._clamp_speed()

        # Jacobian of the state transition (linearization around the
        # current state) -- this is the "Extended" part of EKF: exact
        # for a linear model, a locally-valid approximation here since
        # our motion model is nonlinear (heading-dependent). Uses the
        # same capped dt as the state transition above, for consistency.
        F = np.array([
            [1, 0, -speed * math.sin(heading) * integration_dt, math.cos(heading) * integration_dt],
            [0, 1,  speed * math.cos(heading) * integration_dt, math.sin(heading) * integration_dt],
            [0, 0, 1, 0],
            [0, 0, 0, 1],
        ])

        # Process noise uses the TRUE dt, not the capped one: a longer
        # real gap should make the filter progressively less confident
        # in its own state, even though it stopped extrapolating
        # position/speed further after INTEGRATION_DT_CAP_S.
        Q = self.Q_base * dt
        self.P = F @ self.P @ F.T + Q

    def update_gps(self, x_meas, y_meas, trusted=False):
        H = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
        ])
        z = np.array([x_meas, y_meas])
        self._update(z, H, self.R_gps, trusted=trusted)

    def update_speed(self, speed_meas):
        H = np.array([[0, 0, 0, 1]])
        z = np.array([speed_meas])
        self._update(z, H, self.R_speed)

    def _update(self, z, H, R, trusted=False):
        y = z - H @ self.state  # innovation (measurement residual)
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)  # Kalman gain
        correction = K @ y

        # Sanity-check the correction itself, not just the measurement
        # that produced it: a numerically unstable Kalman gain (e.g.
        # from an ill-conditioned P matrix) can apply an enormous
        # correction directly to POSITION (state[0]/state[1]) in one
        # update -- confirmed on a real drive where a 9433m single-step
        # jump occurred at an ordinary predict-only row with completely
        # normal dt and OBD speed, meaning the position itself must
        # have already been corrupted by an earlier update's corrupted
        # correction, not by anything wrong with that row's own inputs.
        #
        # `trusted=True` (used for a GPS fix already confirmed by a
        # second, independent, mutually-consistent reading in the
        # calling code) bypasses this check entirely. Discovered why
        # this has to be optional: applying the check unconditionally
        # created a WORSE failure than the one it was meant to prevent
        # -- once real dead-reckoning drift exceeds the threshold, every
        # subsequent correction attempting to fix that drift also looks
        # "too large" relative to the increasingly-wrong internal state,
        # so the filter refuses ALL further GPS corrections forever,
        # even ones independently confirmed as internally consistent
        # with each other. On one real drive this silently let the
        # reported position diverge over 31 kilometers from the true
        # GPS track across a ~13-minute stretch, with no visible
        # single-step jump at all to flag it. A correction confirmed by
        # agreement between two independent readings is strong enough
        # evidence of correctness that it should be trusted regardless
        # of size; the magnitude check is only meant to catch a single,
        # unconfirmed, possibly-corrupted or numerically-unstable
        # correction, not to permanently veto real recovery from drift.
        MAX_POSITION_CORRECTION_M = 200.0
        position_correction_mag = math.hypot(correction[0], correction[1])
        if not trusted and position_correction_mag > MAX_POSITION_CORRECTION_M:
            print(
                f"  discarded a {position_correction_mag:.0f}m Kalman correction "
                f"(max allowed: {MAX_POSITION_CORRECTION_M:.0f}m) -- likely a "
                f"numerically unstable gain rather than a real, physically "
                f"plausible correction; keeping the pre-update state instead"
            )
            return

        self.state = self.state + correction
        self.P = (np.eye(len(self.state)) - K @ H) @ self.P
        # Keep P symmetric: floating-point roundoff accumulated over a
        # long drive (thousands of predict/update cycles) can nudge P
        # away from perfect symmetry, and an asymmetric covariance
        # matrix is a well-known way for an EKF to become numerically
        # unstable and silently diverge over time -- suspected as the
        # root cause of one real drive's speed state reaching an
        # absurd ~94,000 m/s with no corresponding bad sensor input at
        # all. Averaging P with its own transpose after every update is
        # a standard, cheap defensive measure for exactly this.
        self.P = (self.P + self.P.T) / 2
        self._clamp_speed()

    def _clamp_speed(self):
        # Final, unconditional safety net on the state itself, applied
        # after every predict AND update: whatever the root cause of a
        # divergence might be (numerical instability, a sensor
        # reading that slipped past upstream validation, anything not
        # yet anticipated), the fused output should never report a
        # speed beyond what this vehicle can physically achieve. This
        # bounds the position drift any single subsequent predict step
        # can produce, regardless of why the state got here.
        MAX_PLAUSIBLE_SPEED_MS = 60.0  # ~134 mph, generous upper bound
        self.state[3] = max(-MAX_PLAUSIBLE_SPEED_MS, min(MAX_PLAUSIBLE_SPEED_MS, self.state[3]))


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
    df = fix_clock_jumps(df)

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
        #
        # fix_quality is coerced to numeric first: a garbled/partial
        # NMEA sentence can leave a corrupted string in this column
        # (confirmed once as literal scrambled binary-looking text).
        # `== 0` against a string silently evaluates to False rather
        # than raising -- unlike the `>=` crash caught earlier in
        # generate_drive_report.py, this failure mode was SILENT, so a
        # corrupted row was never being filtered out here at all. If
        # the same corruption event also scrambled lat/lon in that
        # row, that garbage coordinate would get fed straight into
        # update_gps(), producing exactly the kind of massive
        # single-step position jump test_ekf_no_position_runaway is
        # designed to catch.
        fix_quality = pd.to_numeric(df.get("gnss_fix_quality"), errors="coerce") if "gnss_fix_quality" in df.columns else None
        lat_num = pd.to_numeric(df["gnss_latitude"], errors="coerce")
        lon_num = pd.to_numeric(df["gnss_longitude"], errors="coerce")

        invalid = (lat_num == 0) & (lon_num == 0)
        if fix_quality is not None:
            invalid = invalid | (fix_quality == 0)
        # Defense in depth: also reject anything that isn't a finite,
        # physically-plausible coordinate at all (catches corruption
        # that doesn't happen to land on exactly (0,0) or fail the
        # fix-quality check, e.g. a garbled lat/lon value directly).
        invalid = invalid | lat_num.isna() | lon_num.isna()
        invalid = invalid | (lat_num.abs() > 90) | (lon_num.abs() > 180)

        lat_num[invalid] = np.nan
        lon_num[invalid] = np.nan
        df["gnss_latitude"] = lat_num
        df["gnss_longitude"] = lon_num

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
    prev_gps_time = None
    pending_fix = None  # (x_meas, y_meas, t) for a reading that failed the gate but hasn't been ruled out yet
    prev_obd_speed = None

    for _, row in df.iterrows():
        t = row["timestamp"]
        if prev_time is None:
            dt = 0.0
        else:
            dt = (t - prev_time).total_seconds()
            # fix_clock_jumps() (called above, before this loop) already
            # finds and corrects genuine multi-second-to-hour NTP-sync
            # discontinuities at the row level, so by the time we get
            # here, a large dt is no longer expected to be clock
            # corruption. It's much more likely to be a REAL gap -- e.g.
            # the ~2s pause imu_logger.py's I2C staleness-recovery
            # routine takes to reset and re-enable the sensor, which is
            # genuine elapsed driving time, not an artifact. An earlier
            # version of this clamp treated anything over 1 second as a
            # clock glitch and shrank it to 0.1s, which quietly discarded
            # real distance traveled during every IMU recovery -- over a
            # long drive with many recoveries, that accumulated into
            # exactly the kind of large single-step "runaway" this
            # script's own dt-based safety net was meant to prevent
            # (caught by test_ekf_no_position_runaway in tests/test_pipeline.py).
            # The threshold here is now just a defensive backstop for
            # anything fix_clock_jumps' own 60s threshold might have
            # missed, not the primary correction mechanism.
            if dt > 60.0 or dt < 0:
                print(f"  unexpected large dt at {t} (dt={dt:.1f}s) -- clamping to 0.1s")
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
                # Innovation gate: format-level validation (fix_quality,
                # (0,0), numeric range) can't catch a coordinate that's
                # perfectly well-formed but physically nonsensical given
                # where the vehicle actually is -- confirmed by testing
                # with a synthetically corrupted-but-in-range point that
                # sailed straight through those checks and produced a
                # multi-thousand-km single-step jump. Instead, reject
                # any GPS fix implying a jump the vehicle couldn't
                # plausibly have made in the time elapsed since the
                # filter's last update. 100 mph is already far beyond
                # this vehicle's real capability, so a jump requiring a
                # higher implied speed than that is treated as a
                # corrupted/outlier reading and skipped rather than fed
                # into the filter.
                #
                # Elapsed time here MUST be measured since the last
                # actual GPS fix, not the generic per-row IMU dt: GPS
                # updates roughly once per second while the IMU runs at
                # ~10Hz, so aligned.csv forward-fills the same lat/lon
                # across ~10 rows before a new one appears. Using the
                # tiny row-to-row dt (~0.1s) here made every ordinary,
                # correct GPS update look 10x faster than it really was,
                # which falsely rejected real corrections and let dead-
                # reckoning drift accumulate uncorrected -- a first,
                # broken version of this gate did exactly that.
                MAX_PLAUSIBLE_SPEED_MS = 45.0  # ~100 mph, generous upper bound
                dx_gps = x_meas - ekf.state[0]
                dy_gps = y_meas - ekf.state[1]
                gps_jump_dist = math.hypot(dx_gps, dy_gps)
                elapsed_since_last_gps = (t - prev_gps_time).total_seconds() if prev_gps_time is not None else None
                implied_speed = (
                    gps_jump_dist / elapsed_since_last_gps
                    if elapsed_since_last_gps and elapsed_since_last_gps > 0
                    else 0.0  # first-ever fix: nothing to compare against, always accept
                )

                if implied_speed <= MAX_PLAUSIBLE_SPEED_MS:
                    # Ordinary case: consistent with where the filter
                    # already thinks it is. Accept immediately.
                    ekf.update_gps(x_meas, y_meas)
                    prev_gps_time = t
                    pending_fix = None
                else:
                    # This reading disagrees sharply with the filter's
                    # current (possibly drift-accumulated) estimate.
                    # Rather than reject outright -- which would also
                    # discard a LEGITIMATE large catch-up correction
                    # after a genuinely long, continuous GPS blackout
                    # (confirmed: a real 13s gap producing a 288m/22 m/s
                    # correction is normal, expected EKF behavior) --
                    # check whether it's independently confirmed by the
                    # immediately preceding rejected reading. Two
                    # readings landing close to each other (a real GPS
                    # fix, ~1s apart, shouldn't move far) is strong
                    # evidence both are real; a single, unconfirmed
                    # outlier is far more likely to be a low-quality
                    # fix right after reacquiring lock (a real, common
                    # GPS behavior) or corrupted data -- confirmed by a
                    # case where one such unconfirmed reading implied a
                    # 620 mph jump and was never repeated.
                    CONFIRM_MAX_GAP_S = 3.0
                    CONFIRM_MAX_DIST_M = 60.0
                    confirmed = False
                    if pending_fix is not None:
                        px, py, pt = pending_fix
                        pending_gap_s = (t - pt).total_seconds()
                        pending_dist = math.hypot(x_meas - px, y_meas - py)
                        if pending_gap_s <= CONFIRM_MAX_GAP_S and pending_dist <= CONFIRM_MAX_DIST_M:
                            confirmed = True

                    if confirmed:
                        print(
                            f"  accepting GPS fix at {t} after confirmation by a matching "
                            f"prior reading ({gps_jump_dist:.0f}m jump from filter, but only "
                            f"{pending_dist:.0f}m from the previous candidate)"
                        )
                        ekf.update_gps(x_meas, y_meas, trusted=True)
                        prev_gps_time = t
                        pending_fix = None
                    else:
                        print(
                            f"  rejected implausible GPS fix at {t}: {gps_jump_dist:.0f}m jump "
                            f"in {elapsed_since_last_gps:.1f}s (implied {implied_speed:.0f} m/s) -- "
                            f"holding as unconfirmed"
                        )
                        pending_fix = (x_meas, y_meas, t)
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
