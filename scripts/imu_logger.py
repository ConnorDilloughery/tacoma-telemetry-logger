#!/usr/bin/env python3
"""
BNO085 IMU Logger
------------------
Reads orientation (rotation vector / quaternion), linear acceleration,
gyroscope, and calibration status from a BNO085 9-DOF IMU over I2C,
prints live values, and logs to a timestamped CSV file.

Design notes:
- The BNO085 doesn't stream data by default -- you must explicitly
  enable each "report" you want (rotation vector, linear accel, gyro)
  before it starts producing that data. This is a one-time setup step.
- We use ROTATION_VECTOR (the chip's onboard sensor-fusion output,
  combining accel+gyro+mag into an absolute orientation quaternion)
  rather than raw accelerometer/magnetometer, since that fusion is
  exactly what we'd otherwise have to implement ourselves.
- LINEAR_ACCELERATION reports acceleration with gravity already
  subtracted, which is what you want for detecting real vehicle
  motion (braking, accelerating) rather than just measuring gravity.
- The CSV schema mirrors obd_logger.py / gnss_logger.py (a timestamp
  column plus one column per value) so all three logs can be joined
  by time later during sensor fusion.

Requires:
    pip3 install adafruit-circuitpython-bno08x adafruit-blinka --break-system-packages

Usage:
    python3 imu_logger.py [--rate 20] [--duration 60]
"""

import argparse
import csv
import time
from datetime import datetime

import board
import busio
from adafruit_bno08x import (
    BNO_REPORT_ROTATION_VECTOR,
    BNO_REPORT_LINEAR_ACCELERATION,
    BNO_REPORT_GYROSCOPE,
)
from adafruit_bno08x.i2c import BNO08X_I2C

FIELDNAMES = [
    "timestamp",
    "quat_i",
    "quat_j",
    "quat_k",
    "quat_real",
    "accel_x",
    "accel_y",
    "accel_z",
    "gyro_x",
    "gyro_y",
    "gyro_z",
]


def main():
    parser = argparse.ArgumentParser(description="Log BNO085 IMU data over I2C.")
    parser.add_argument("--rate", type=float, default=20.0, help="Polling cycles per second (default: 20)")
    parser.add_argument("--duration", type=float, default=None, help="Seconds to run (default: run forever)")
    parser.add_argument("--out", default=None, help="Output CSV path (default: auto-timestamped filename)")
    args = parser.parse_args()

    out_path = args.out or f"imu_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
    period = 1.0 / args.rate

    # Standard I2C bus on the Pi's SDA/SCL pins (same bus the GPS
    # compass is already on -- I2C addressing keeps them separate).
    i2c = busio.I2C(board.SCL, board.SDA, frequency=400000)
    bno = BNO08X_I2C(i2c)

    # Enable the reports we want. Each of these tells the chip to
    # start computing and buffering that particular output; without
    # this call, reading bno.quaternion etc. would raise an error.
    bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)
    bno.enable_feature(BNO_REPORT_LINEAR_ACCELERATION)
    bno.enable_feature(BNO_REPORT_GYROSCOPE)

    print(f"Logging to {out_path}")
    print(f"Polling at {args.rate} Hz")
    print("Press Ctrl+C to stop.\n")

    # Staleness detection: an I2C glitch or a chip entering a bad
    # internal state can make the BNO085 keep returning its LAST
    # successfully-read report indefinitely, with no exception raised
    # -- unlike a clean disconnect, this fails silently. A real,
    # moving sensor's readings change on every sample even at rest
    # (thermal/electrical noise alone guarantees that), so N identical
    # consecutive readings is itself a fault signal worth acting on,
    # not just implausible magnitude.
    STALE_THRESHOLD = 20  # ~2 seconds at 10Hz before we consider it stuck
    last_accel = None
    stale_count = 0

    start = time.monotonic()
    try:
        with open(out_path, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=FIELDNAMES)
            writer.writeheader()

            while args.duration is None or (time.monotonic() - start) < args.duration:
                cycle_start = time.monotonic()

                try:
                    quat_i, quat_j, quat_k, quat_real = bno.quaternion
                    accel_x, accel_y, accel_z = bno.linear_acceleration
                    gyro_x, gyro_y, gyro_z = bno.gyro
                except Exception as e:
                    # The BNO085 occasionally returns a stale/invalid
                    # report right after enabling a feature or if I2C
                    # timing hiccups. Skip this cycle rather than crash.
                    print(f"read error, skipping cycle: {e}")
                    time.sleep(period)
                    continue

                # Sanity-check for corrupted single-sample glitches.
                # A hand-held or vehicle-mounted IMU won't legitimately
                # see >20 rad/s (~1145 deg/s) of rotation or >4g of
                # linear acceleration; readings past that are far more
                # likely a garbled I2C packet than real motion, so we
                # drop the whole row rather than log a physically
                # implausible spike.
                GYRO_LIMIT = 20.0       # rad/s
                ACCEL_LIMIT = 40.0      # m/s^2 (~4g)
                gyro_vals = (gyro_x, gyro_y, gyro_z)
                accel_vals = (accel_x, accel_y, accel_z)
                if any(abs(v) > GYRO_LIMIT for v in gyro_vals) or any(
                    abs(v) > ACCEL_LIMIT for v in accel_vals
                ):
                    print(f"outlier reading discarded: gyro={gyro_vals} accel={accel_vals}")
                    elapsed = time.monotonic() - cycle_start
                    sleep_time = period - elapsed
                    if sleep_time > 0:
                        time.sleep(sleep_time)
                    continue

                # Staleness check: has the reading actually changed?
                if accel_vals == last_accel:
                    stale_count += 1
                else:
                    stale_count = 0
                last_accel = accel_vals

                if stale_count >= STALE_THRESHOLD:
                    print(
                        f"WARNING: IMU reading unchanged for {stale_count} consecutive "
                        f"samples (~{stale_count / args.rate:.1f}s) -- sensor may be "
                        f"stuck. Attempting to re-initialize the I2C connection."
                    )
                    try:
                        i2c.deinit()
                    except Exception:
                        pass
                    time.sleep(0.5)
                    try:
                        i2c = busio.I2C(board.SCL, board.SDA, frequency=400000)
                        bno = BNO08X_I2C(i2c)
                        bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)
                        bno.enable_feature(BNO_REPORT_LINEAR_ACCELERATION)
                        bno.enable_feature(BNO_REPORT_GYROSCOPE)
                        print("IMU re-initialized.")
                    except Exception as e:
                        print(f"IMU re-initialization failed: {e}")
                    stale_count = 0
                    last_accel = None
                    time.sleep(period)
                    continue

                row = {
                    "timestamp": datetime.now().isoformat(),
                    "quat_i": quat_i,
                    "quat_j": quat_j,
                    "quat_k": quat_k,
                    "quat_real": quat_real,
                    "accel_x": accel_x,
                    "accel_y": accel_y,
                    "accel_z": accel_z,
                    "gyro_x": gyro_x,
                    "gyro_y": gyro_y,
                    "gyro_z": gyro_z,
                }
                writer.writerow(row)
                f.flush()

                print(
                    f"quat=({quat_i:.2f},{quat_j:.2f},{quat_k:.2f},{quat_real:.2f}) "
                    f"accel=({accel_x:.2f},{accel_y:.2f},{accel_z:.2f}) "
                    f"gyro=({gyro_x:.2f},{gyro_y:.2f},{gyro_z:.2f})"
                )

                elapsed = time.monotonic() - cycle_start
                sleep_time = period - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)

    except KeyboardInterrupt:
        print("\nStopped by user.")
    finally:
        print(f"Log saved to {out_path}")


if __name__ == "__main__":
    main()
