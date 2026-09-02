#!/usr/bin/env python3
"""
OBD-II PID Polling Logger
--------------------------
Polls a set of standard Mode 01 PIDs over SocketCAN (can0) using
request/response on the OBD-II functional broadcast ID (0x7DF),
decodes the responses, prints them live, and logs to a timestamped
CSV file.

Requires:
    pip3 install python-can --break-system-packages

Usage:
    python3 obd_logger.py [--rate 10] [--duration 60]
"""

import argparse
import csv
import time
from datetime import datetime

import can

# ---- OBD-II constants ----
OBD_REQUEST_ID = 0x7DF          # functional broadcast request
OBD_RESPONSE_IDS = {0x7E8, 0x7E9, 0x7EA, 0x7EB, 0x7EC, 0x7ED, 0x7EE, 0x7EF}
MODE_01 = 0x01

# PID -> (name, decode_fn, units)
PIDS = {
    0x0C: ("rpm", lambda d: ((d[0] * 256) + d[1]) / 4.0, "rpm"),
    0x0D: ("speed", lambda d: round(d[0] * 0.621371, 1), "mph"),       # raw value is km/h -> convert to mph
    0x05: ("coolant_temp", lambda d: round((d[0] - 40) * 9 / 5 + 32, 1), "F"),  # raw value is C -> convert to F
    0x04: ("engine_load", lambda d: round(d[0] / 2.55, 1), "%"),
    0x11: ("throttle_pos", lambda d: round(d[0] / 2.55, 1), "%"),
}


def build_request(pid: int) -> can.Message:
    """Builds a Mode 01 PID request frame."""
    data = [0x02, MODE_01, pid, 0x00, 0x00, 0x00, 0x00, 0x00]
    return can.Message(arbitration_id=OBD_REQUEST_ID, data=data, is_extended_id=False)


def decode_response(pid: int, data: bytes):
    """Decodes a Mode 01 response payload for a given PID."""
    if pid not in PIDS:
        return None
    name, decode_fn, units = PIDS[pid]
    # data[0] = length, data[1] = 0x41 (mode echo), data[2] = pid, data[3:] = payload
    payload = data[3:]
    try:
        value = decode_fn(payload)
    except (IndexError, ZeroDivisionError):
        return None
    return name, value, units


def poll_once(bus: can.Bus, pid: int, timeout: float = 0.3):
    """
    Sends a request for one PID and waits for a matching response.

    Returns None on a normal timeout/no-response (e.g. an unsupported
    PID) OR when the CAN bus itself is unavailable (e.g. the CANable
    adapter isn't currently connected to a vehicle -- SocketCAN raises
    can.CanOperationError with "Network is down" in that case). Either
    way the caller treats it the same: log a blank reading and move on,
    rather than crashing the whole session.
    """
    try:
        bus.send(build_request(pid))
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            msg = bus.recv(timeout=deadline - time.monotonic())
            if msg is None:
                break
            if msg.arbitration_id in OBD_RESPONSE_IDS and len(msg.data) >= 3:
                if msg.data[1] == 0x41 and msg.data[2] == pid:
                    return decode_response(pid, msg.data)
        return None
    except can.CanOperationError:
        # Bus is down (unplugged, no vehicle power on the OBD-II port,
        # etc.). Not fatal -- just means no reading this cycle.
        return None


def main():
    parser = argparse.ArgumentParser(description="Poll OBD-II PIDs over SocketCAN and log to CSV.")
    parser.add_argument("--interface", default="can0", help="SocketCAN interface name (default: can0)")
    parser.add_argument("--rate", type=float, default=5.0, help="Polling cycles per second (default: 5)")
    parser.add_argument("--duration", type=float, default=None, help="Seconds to run (default: run forever)")
    parser.add_argument("--out", default=None, help="Output CSV path (default: auto-timestamped filename)")
    args = parser.parse_args()

    out_path = args.out or f"obd_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
    period = 1.0 / args.rate

    # The CAN interface may not exist yet at startup (adapter not
    # plugged in, or can0 hasn't come up yet after a reboot). Rather
    # than crash immediately, retry for a while -- this matters most
    # when this script is launched automatically at boot/ignition
    # rather than by hand, where there's no one watching to restart it.
    bus = None
    while bus is None:
        try:
            bus = can.interface.Bus(channel=args.interface, interface="socketcan")
        except OSError as e:
            print(f"CAN interface '{args.interface}' not available ({e}); retrying in 5s...")
            time.sleep(5)

    print(f"Logging to {out_path}")
    print(f"Polling PIDs: {[hex(p) for p in PIDS]} at {args.rate} Hz")
    print("Press Ctrl+C to stop.\n")

    fieldnames = ["timestamp"] + [name for name, _, _ in PIDS.values()]

    start = time.monotonic()
    try:
        with open(out_path, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()

            consecutive_bus_failures = 0

            while args.duration is None or (time.monotonic() - start) < args.duration:
                cycle_start = time.monotonic()
                row = {"timestamp": datetime.now().isoformat()}
                display = []

                # If the bus has been down for a while, try recreating
                # the interface every ~20 cycles. This recovers
                # automatically if the OBD-II connector gets plugged
                # in partway through a run, instead of requiring a
                # manual restart of the script.
                if consecutive_bus_failures and consecutive_bus_failures % 20 == 0:
                    try:
                        bus.shutdown()
                    except Exception:
                        pass
                    try:
                        bus = can.interface.Bus(channel=args.interface, interface="socketcan")
                        print("(re-)connected to CAN interface")
                    except Exception:
                        pass  # still down, keep trying on the next round

                any_response = False
                for pid in PIDS:
                    result = poll_once(bus, pid)
                    if result:
                        any_response = True
                        name, value, units = result
                        row[name] = value
                        display.append(f"{name}={value}{units}")
                    else:
                        name = PIDS[pid][0]
                        row[name] = ""
                        display.append(f"{name}=NO_RESP")

                writer.writerow(row)
                f.flush()
                print(" | ".join(display))

                consecutive_bus_failures = 0 if any_response else consecutive_bus_failures + 1

                elapsed = time.monotonic() - cycle_start
                sleep_time = period - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)

    except KeyboardInterrupt:
        print("\nStopped by user.")
    finally:
        bus.shutdown()
        print(f"Log saved to {out_path}")


if __name__ == "__main__":
    main()
