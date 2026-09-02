#!/usr/bin/env python3
"""
GNSS NMEA Logger
----------------
Reads raw NMEA sentences from a serial GPS module (e.g. Beitian BN-880),
parses the position (GGA) and speed/course (RMC) sentences using pynmea2,
prints live values, and logs to a timestamped CSV file.

Design notes:
- We only care about GGA (position/altitude/fix quality) and RMC
  (speed/course/date) sentences. GSA/GSV sentences (satellite detail)
  are ignored here to keep the logger focused on what we'll actually
  fuse later.
- Serial data can be noisy, especially right after power-on, so every
  parse is wrapped in a try/except. A bad line is skipped, not fatal.
- The CSV schema mirrors obd_logger.py's pattern (a timestamp column
  plus one column per value) so the two logs can be joined by time
  later during sensor fusion.

Requires:
    pip3 install pyserial pynmea2 --break-system-packages

Usage:
    python3 gnss_logger.py [--port /dev/serial0] [--baud 9600] [--duration 60]
"""

import argparse
import csv
import time
from datetime import datetime

import serial
import pynmea2

FIELDNAMES = [
    "timestamp",
    "fix_quality",
    "num_satellites",
    "latitude",
    "longitude",
    "altitude_m",
    "hdop",
    "speed_knots",
    "course_deg",
]


def parse_line(raw_line: str, state: dict):
    """
    Updates `state` in place with any new fields found in this NMEA line.
    Returns True if the line contained data worth logging (a GGA or RMC
    sentence), False otherwise (unhandled sentence type or parse failure).
    """
    raw_line = raw_line.strip()
    if not raw_line.startswith("$"):
        return False  # not a valid NMEA line (likely boot noise / partial read)

    try:
        msg = pynmea2.parse(raw_line)
    except pynmea2.ParseError:
        return False  # checksum failure or malformed sentence, skip it

    sentence_type = msg.sentence_type  # e.g. "GGA", "RMC"

    if sentence_type == "GGA":
        state["fix_quality"] = msg.gps_qual
        state["num_satellites"] = msg.num_sats
        state["latitude"] = msg.latitude
        state["longitude"] = msg.longitude
        state["altitude_m"] = msg.altitude
        state["hdop"] = msg.horizontal_dil
        return True

    if sentence_type == "RMC":
        # spd_over_grnd / true_course come back as None when the fix
        # doesn't support a reliable value (e.g. stationary, no course
        # can be computed). Normalize None -> "" so the CSV column has
        # a consistent type instead of mixing floats and the string
        # "None".
        speed = msg.spd_over_grnd
        course = msg.true_course
        state["speed_knots"] = speed if speed is not None else ""
        state["course_deg"] = course if course is not None else ""
        return True

    return False  # GSA/GSV/etc. -- not logged here


def main():
    parser = argparse.ArgumentParser(description="Parse and log GNSS NMEA data over serial.")
    parser.add_argument("--port", default="/dev/serial0", help="Serial port (default: /dev/serial0)")
    parser.add_argument("--baud", type=int, default=9600, help="Baud rate (default: 9600)")
    parser.add_argument("--duration", type=float, default=None, help="Seconds to run (default: run forever)")
    parser.add_argument("--out", default=None, help="Output CSV path (default: auto-timestamped filename)")
    args = parser.parse_args()

    out_path = args.out or f"gnss_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"

    ser = serial.Serial(args.port, args.baud, timeout=1)

    print(f"Reading from {args.port} at {args.baud} baud")
    print(f"Logging to {out_path}")
    print("Press Ctrl+C to stop.\n")

    # Running state -- GGA and RMC sentences arrive on separate lines,
    # so we accumulate the latest values from each and write a combined
    # row whenever either one updates. This means a row may have fields
    # from a slightly earlier sentence than the one that triggered the
    # write, which is fine at GNSS update rates (1-10Hz).
    state = {field: "" for field in FIELDNAMES if field != "timestamp"}

    start = time.monotonic()
    try:
        with open(out_path, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=FIELDNAMES)
            writer.writeheader()

            while args.duration is None or (time.monotonic() - start) < args.duration:
                try:
                    raw_line = ser.readline().decode("ascii", errors="replace")
                except UnicodeDecodeError:
                    continue

                if not raw_line:
                    continue

                updated = parse_line(raw_line, state)
                if not updated:
                    continue

                row = {"timestamp": datetime.now().isoformat(), **state}
                writer.writerow(row)
                f.flush()

                print(
                    f"fix={state['fix_quality']} sats={state['num_satellites']} "
                    f"lat={state['latitude']} lon={state['longitude']} "
                    f"alt={state['altitude_m']}m speed={state['speed_knots']}kt "
                    f"course={state['course_deg']}"
                )

    except KeyboardInterrupt:
        print("\nStopped by user.")
    finally:
        ser.close()
        print(f"Log saved to {out_path}")


if __name__ == "__main__":
    main()
