# Tacoma Telemetry & Sensor-Fusion Highway 17 Logger

A Raspberry Pi-based vehicle telemetry system built into a 2025 Toyota
Tacoma: it reads live CAN bus data over OBD-II, fuses it with GPS and
IMU readings using an Extended Kalman Filter, records dashcam video,
and automatically detects and clips hard-braking events -- all
triggered by the vehicle's own ignition, with no laptop required in
the field.

Built as a portfolio project targeting Integration & Test / Systems
Engineering roles in autonomous vehicles and robotics. The goal:
demonstrate the same skills those teams actually use day to day --
embedded Linux, multi-sensor integration, sensor fusion, and building
pipelines that survive real-world hardware faults -- on a real vehicle,
not a simulation.

## System Architecture

```
                    ┌─────────────────────────────────────┐
                    │         Raspberry Pi (4 / 3)         │
                    │                                       │
  OBD-II ──CAN──────┤  obd_logger.py     (CAN, ~5Hz)       │
                    │  gnss_logger.py    (UART, ~1-5Hz)    │
  GNSS ──UART───────┤  imu_logger.py     (I2C, 10Hz)       │
                    │  camera_logger.py  (CSI, 30fps)      │
  IMU ──I2C─────────┤                                       │
                    │  ignition_watcher.py (auto start/stop)│
  Camera ──CSI──────┤  run_session.sh      (launches all 4) │
                    └───────────────┬───────────────────────┘
                                    │  4 raw files per drive
                                    ▼
                     ┌──────────────────────────────┐
                     │      process_drive.sh         │
                     │                                │
                     │  align_logs.py    -- merge_asof│
                     │                      onto one  │
                     │                      timeline  │
                     │  extract_event_clips.py        │
                     │              -- hard-brake     │
                     │                 detection +    │
                     │                 video clips    │
                     │  ekf_fusion.py -- GNSS+OBD+IMU  │
                     │              sensor fusion      │
                     │  generate_drive_report.py       │
                     │              -- stats, plots,   │
                     │                 per-drive README│
                     └───────────────┬────────────────┘
                                     ▼
                          drives/<session_id>/
                          (pushed to this repo)
```

## Hardware

| Component | Part | Interface |
|---|---|---|
| Compute | Raspberry Pi 4 / 3 | -- |
| CAN adapter | DSD TECH SH-C31A (CANable 2.0 / candleLight firmware) | USB → SocketCAN |
| OBD-II tap | Pigtail cable, wired to CAN-H/CAN-L via a DB9 breakout | -- |
| GNSS | Beitian BN-880 (u-blox-class chipset + HMC5883L compass) | UART + I2C |
| IMU | BNO085 9-DOF (accelerometer + gyro + magnetometer, onboard sensor fusion) | I2C |
| Camera | Raspberry Pi Camera Module 3 (imx708), fixed infinity focus | CSI |

## Software Pipeline

**Acquisition (run continuously during a drive):**
- `obd_logger.py` -- polls standard Mode 01 PIDs (speed, RPM, coolant temp, load, throttle) over SocketCAN, with automatic reconnect if the CAN interface drops or isn't up yet at boot
- `gnss_logger.py` -- parses NMEA GGA/RMC sentences for position, altitude, fix quality, and speed/course
- `imu_logger.py` -- reads the BNO085's fused orientation quaternion, linear acceleration, and gyro, with outlier rejection for corrupted I2C reads
- `camera_logger.py` -- records continuous dashcam video (H.264/mp4) with a timestamped start marker for later event-clip extraction
- `ignition_watcher.py` -- pings the vehicle's ECU over CAN every 2 seconds; starts/stops the whole recording session automatically based on whether the engine is actually running, so nothing needs to be started by hand
- `run_session.sh` -- launches all four loggers together under one shared session ID

**Processing (run once after a drive, via `process_drive.sh`):**
- `align_logs.py` -- merges the three independently-clocked, independently-rated sensor logs onto one timeline using an as-of (nearest-timestamp) join
- `extract_event_clips.py` -- detects hard-braking events from the rate of change of OBD-reported speed (chosen over IMU acceleration since the IMU's mounting axes aren't yet calibrated to the vehicle's frame), and pulls a short video clip around each one
- `ekf_fusion.py` -- fuses GNSS position, OBD speed, and IMU motion into one continuous vehicle state estimate (see below)
- `generate_drive_report.py` -- computes trip stats and produces all plots and the per-drive README
- `update_index.py` -- rebuilds this table you're reading right now

## Sensor Fusion (EKF)

State vector: `[x, y, heading, speed]`, tracked with an Extended Kalman
Filter. The IMU's gyro/accelerometer drive the prediction step between
GPS updates; GNSS position and OBD speed correct the estimate whenever
a new reading arrives. A Zero-Velocity Update (ZUPT) pins the filter
when the vehicle is stopped, which prevents IMU gyro bias from slowly
spinning the heading estimate while parked -- a real failure mode this
project hit and fixed (see below).

**Documented limitation:** the IMU's mounting orientation isn't yet
calibrated against the vehicle's actual axes, so the filter assumes
the IMU's local X-axis is roughly forward and Z-axis is roughly
vertical. A logical next step is an explicit calibration pass (e.g.
comparing IMU heading change against GPS course change during turns).

## Engineering Log: Real Bugs Found & Fixed

This project was debugged against real hardware and real driving data,
not just synthetic tests. A few of the more interesting issues:

- **GNSS "null island":** the GPS module reports `(0, 0)` before
  acquiring a fix; left unfiltered, this turned a real 2-mile drive's
  computed distance into ~8,000 miles (measuring to the Gulf of
  Guinea). Fixed by filtering on both exact `(0,0)` and `fix_quality`.
- **EKF clock discontinuity:** the Pi has no battery-backed real-time
  clock, so its system clock can jump once it syncs with NTP mid-session.
  One 510-second timestamp gap in an otherwise ~0.1s-spaced log caused
  the EKF to project the vehicle position over a kilometer away in a
  single step. Fixed with a defensive `dt` clamp in the filter's
  predict step.
- **Buffered logging under systemd:** a background service's `print()`
  output doesn't reach the system log in real time by default, making
  it look like automatic ignition detection wasn't working when it
  actually was. Fixed by running Python unbuffered (`-u`).
- **CAN interface not persisting across reboots:** bringing `can0` up
  manually doesn't survive a reboot; fixed with a `systemd-networkd`
  config so the interface comes up automatically at boot.
- **Dashcam video never finalizing on a real drive:** the ignition
  watcher stops a session by sending `SIGINT` to the whole process
  group at once. `camera_logger.py`'s first version used
  `subprocess.run()` to launch `rpicam-vid`; when the parent Python
  process caught that same signal mid-recording, `subprocess.run()`'s
  own error handling immediately `SIGKILL`ed the still-recording child
  before it could close out the mp4 container, leaving every real
  drive's video file unplayable (`moov atom not found`) even though
  the file existed and had data in it. A first attempt fixed this by
  having the parent *ignore* the signal instead -- which stopped the
  premature kill, but now nothing told `rpicam-vid` to stop at all,
  and it just kept recording as an orphaned process indefinitely. The
  actual fix: catch the signal in the parent and explicitly forward it
  to the `rpicam-vid` child, then wait for it to exit on its own --
  guaranteeing a clean shutdown regardless of exactly how the parent
  itself gets signaled. Caught by testing the real stop path directly
  (`kill -INT` against a live session) rather than only a
  fixed-duration recording, which had been passing the whole time for
  the wrong reason.
- **Journal logs lost across an unplanned mid-drive reboot:** during
  one drive, the Pi rebooted on its own partway through (root cause
  undetermined -- power interruption is the leading suspect, given the
  dashboard mount had already come loose once before). The video never
  finalized, no surprise given a full system restart interrupts
  recording far more abruptly than any signal ever could -- but the
  bigger issue was that `journalctl` had **no record of the earlier
  boot at all**, since this Pi's `systemd-journald` wasn't configured
  to persist logs to disk by default; a reboot wipes them. That meant
  the first ~35 minutes of debugging context for that session were
  simply gone. Fixed by creating `/var/log/journal` and restarting
  `systemd-journald`, so future logs survive a reboot -- this doesn't
  prevent an unplanned restart, but it means the next one won't also
  erase the evidence needed to diagnose it.
- **IMU silently freezing on stale I2C readings mid-drive:** a 36-minute
  real drive came back with only 72 unique acceleration values across
  22,664 rows -- the BNO085 had locked onto a single stale reading
  within seconds of starting and never recovered, silently, with no
  exception raised. The library's own debug output showed the real
  cause: frequent `RuntimeError: ('Unprocessable Batch bytes', ...)`
  and `** UNKNOWN Report Type **` errors from `adafruit-circuitpython-
  bno08x`, a well-documented, unresolved issue in that library on
  Linux/I2C (confirmed against multiple independent reports of the
  same errors on otherwise-healthy, stationary setups, going back
  years) rather than anything specific to this wiring. Since the
  underlying library issue isn't something to fix directly, the
  practical workaround: detect staleness explicitly (the same
  acceleration reading repeating for ~2 seconds straight is itself a
  fault signal, since real sensor noise alone guarantees a stationary
  reading still changes slightly every sample) and automatically tear
  down and re-initialize the I2C connection when it's detected. Tested
  standalone, this successfully caught and recovered from the freeze
  multiple times within a two-minute run.
- **Root cause behind the IMU freezes, missing camera footage, and
  repeated mid-drive reboots: sustained undervoltage.** After the
  staleness fix above shipped, three more real drives still produced
  no video at all, with zero camera-related log lines -- meaning
  `camera_logger.py` never even started, not a signal-handling issue
  this time. Checking `vcgencmd get_throttled` returned `0x50005`:
  Raspberry Pi's own flag for "currently under-voltage," set
  continuously, not as an isolated blip. `journalctl` showed
  `Undervoltage detected!` firing constantly and continuously for
  hours, starting right at boot. This one finding retroactively
  explains several previously-separate mysteries: the recurring
  mid-drive reboots (a brownout, not a software crash or a loose
  wire), the exact-2.0-second clock discontinuities recurring
  throughout a drive (likely brief CPU/scheduler stalls under voltage
  stress), the camera never launching (it's the single heaviest power
  draw of the four sensors, the first thing marginal power can't
  sustain), and very plausibly a contributing factor in the I2C
  freezes attributed to the BNO08x library bug above -- a real
  upstream issue made more frequent by an underpowered bus, not
  necessarily occurring at this rate on its own. Confirmed the Pi is
  not *currently* under-voltage while idle, meaning the fix isn't
  necessarily "replace the whole power bank" -- next step is an
  active-load comparison (wall power vs. battery bank, all sensors
  running) to isolate whether this is the power bank's sustained
  current capacity, the cable, or a connector, before deciding on a
  hardware fix.

## Future Work

- Hardware-in-the-loop fault injection (simulated GPS dropouts,
  corrupted CAN frames, IMU glitches) as automated test cases
- CI pipeline that regression-tests the fusion pipeline against
  recorded sessions on every commit
- IMU-to-vehicle axis calibration
- Further EKF tuning (process/measurement noise, GPS course as a
  heading measurement)

## Drives

**9 drive(s) recorded.**

| Drive | Date | Duration | Distance | Max Speed | Hard Brakes |
|---|---|---|---|---|---|
| [20260904_063219](drives/20260904_063219/README.md) | 2026-09-04 | 0.9 min | 0.2 mi | 27.3 mph | 1 |
| [20260903_171327](drives/20260903_171327/README.md) | 2026-09-03 | 3.8 min | 0.1 mi | 28.6 mph | 3 |
| [20260903_170528](drives/20260903_170528/README.md) | 2026-09-03 | 5.1 min | 0.7 mi | 25.5 mph | 5 |
| [20260903_161819](drives/20260903_161819/README.md) | 2026-09-03 | 30.0 min | 19.6 mi | 67.7 mph | 3 |
| [20260902_191142](drives/20260902_191142/README.md) | 2026-09-02 | 37.8 min | 22.8 mi | 72.7 mph | 18 |
| [20260902_160823](drives/20260902_160823/README.md) | 2026-09-02 | 37.9 min | 22.1 mi | 72.1 mph | 13 |
| [20260902_131536](drives/20260902_131536/README.md) | 2026-09-02 | 3.6 min | 0.9 mi | 41.0 mph | 4 |
| [20260902_130608](drives/20260902_130608/README.md) | 2026-09-02 | 11.5 min | 1.3 mi | 36.0 mph | 4 |
| [20260902_074421](drives/20260902_074421/README.md) | 2026-09-02 | 0.9 min | 0.0 mi | 0.0 mph | 0 |
