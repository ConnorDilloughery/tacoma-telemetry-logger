#!/usr/bin/env python3
"""
Ignition Watcher
----------------
Runs continuously (started at boot via systemd). Periodically pings the
vehicle's ECU over CAN with a lightweight OBD-II request. If the ECU
responds, the vehicle is "on" and a recording session (run_session.sh)
is started automatically. If the ECU goes quiet for several consecutive
checks, the vehicle is considered "off" and the running session is
stopped gracefully.

Design notes:
- We reuse the exact same PID 0x00 request from earlier manual testing
  (the "which PIDs are supported" query) since it's the cheapest
  possible check -- any responding ECU answers it, and we don't care
  about the actual bitmask content, just whether *anything* answered.
- A single missed response isn't treated as "car off" -- CAN bus
  errors or a slow ECU on one particular check are normal. We require
  OFF_THRESHOLD consecutive silent checks before declaring the car off
  and stopping the session, to avoid starting/stopping repeatedly on
  a single dropped frame.
- The session is started as its own process group (start_new_session=
  True) so that stopping it means sending a signal to that whole group
  -- this ensures run_session.sh's own child processes (the three
  loggers) all receive the interrupt too, not just the shell script
  itself.

Requires:
    pip3 install python-can --break-system-packages
    (same as obd_logger.py)

Usage (normally run via systemd, but can be run manually for testing):
    python3 ignition_watcher.py
"""

import os
import signal
import subprocess
import time

import can

OBD_REQUEST_ID = 0x7DF
OBD_RESPONSE_IDS = {0x7E8, 0x7E9, 0x7EA, 0x7EB, 0x7EC, 0x7ED, 0x7EE, 0x7EF}

CHECK_INTERVAL_S = 2.0      # how often to ping the ECU
OFF_THRESHOLD = 5           # consecutive silent checks before declaring "car off" (~10s)
RUN_SESSION_SCRIPT = os.path.expanduser("~/run_session.sh")


def ecu_responds(bus: can.Bus, timeout: float = 0.5) -> bool:
    """Sends a lightweight PID 0x00 request and returns True if any ECU answers."""
    try:
        msg = can.Message(
            arbitration_id=OBD_REQUEST_ID,
            data=[0x02, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00],
            is_extended_id=False,
        )
        bus.send(msg)
    except can.CanOperationError:
        return False  # bus itself is down (adapter unplugged, etc.)

    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        response = bus.recv(timeout=deadline - time.monotonic())
        if response is None:
            break
        if response.arbitration_id in OBD_RESPONSE_IDS:
            return True
    return False


def start_session():
    print("Vehicle ON detected -- starting recording session.")
    # start_new_session=True makes this the leader of a new process
    # group, so we can later signal the whole group (the shell script
    # plus its three background loggers) with one os.killpg call.
    return subprocess.Popen(
        [RUN_SESSION_SCRIPT],
        start_new_session=True,
    )


def stop_session(proc: subprocess.Popen):
    print("Vehicle OFF detected -- stopping recording session.")
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        proc.wait(timeout=10)
    except (ProcessLookupError, subprocess.TimeoutExpired):
        pass


def main():
    print("Ignition watcher started. Waiting for vehicle...")

    session_proc = None
    silent_checks = 0

    while True:
        # Reconnecting to the bus each cycle is deliberate and cheap:
        # it means the watcher doesn't need its own reconnect logic
        # for a bus that comes and goes (adapter plugged in later,
        # vehicle battery disconnected, etc.) -- python-can's Bus
        # object handles a healthy can0 interface fine even if it
        # was briefly unavailable moments earlier.
        try:
            bus = can.interface.Bus(channel="can0", interface="socketcan")
            responded = ecu_responds(bus)
            bus.shutdown()
        except Exception as e:
            print(f"CAN check failed: {e}")
            responded = False

        if responded:
            silent_checks = 0
            if session_proc is None or session_proc.poll() is not None:
                session_proc = start_session()
        else:
            silent_checks += 1
            if session_proc is not None and silent_checks >= OFF_THRESHOLD:
                stop_session(session_proc)
                session_proc = None

        time.sleep(CHECK_INTERVAL_S)


if __name__ == "__main__":
    main()
