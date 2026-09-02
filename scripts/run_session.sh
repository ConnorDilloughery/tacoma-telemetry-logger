#!/bin/bash
# run_session.sh
# ----------------
# Launches the CAN (OBD), GNSS, and IMU loggers together, tagged with a
# shared session ID so their output CSVs are easy to match up afterward
# for alignment.
#
# Design note: this doesn't try to make the three loggers poll in lockstep
# (GNSS updates at its own fix rate, IMU at 10-20Hz, OBD at whatever --rate
# you set) -- it just ensures they all START at the same wall-clock moment,
# using the same system clock for their timestamps. That's enough for the
# alignment step (align_logs.py) to accurately merge them afterward via
# nearest-timestamp matching.
#
# Usage:
#   ./run_session.sh [duration_seconds]
#
# Stops all three with Ctrl+C, or automatically after duration_seconds
# if provided.

SESSION_ID=$(date +%Y%m%d_%H%M%S)
DURATION=${1:-}

echo "Starting session: $SESSION_ID"

DURATION_FLAG=""
if [ -n "$DURATION" ]; then
    DURATION_FLAG="--duration $DURATION"
fi

python3 obd_logger.py --rate 5 $DURATION_FLAG --out "obd_${SESSION_ID}.csv" &
OBD_PID=$!

python3 gnss_logger.py $DURATION_FLAG --out "gnss_${SESSION_ID}.csv" &
GNSS_PID=$!

python3 imu_logger.py --rate 10 $DURATION_FLAG --out "imu_${SESSION_ID}.csv" &
IMU_PID=$!

python3 camera_logger.py $DURATION_FLAG --out "video_${SESSION_ID}.mp4" &
CAM_PID=$!

echo "Started:"
echo "  OBD  (pid $OBD_PID)  -> obd_${SESSION_ID}.csv"
echo "  GNSS (pid $GNSS_PID) -> gnss_${SESSION_ID}.csv"
echo "  IMU  (pid $IMU_PID)  -> imu_${SESSION_ID}.csv"
echo "  CAM  (pid $CAM_PID)  -> video_${SESSION_ID}.mp4"
echo ""
echo "Press Ctrl+C to stop all four."

# Forward Ctrl+C to all four child processes so they shut down cleanly
# (each script's own KeyboardInterrupt handler flushes/closes its file).
trap "echo ''; echo 'Stopping all loggers...'; kill $OBD_PID $GNSS_PID $IMU_PID $CAM_PID 2>/dev/null" INT

wait $OBD_PID $GNSS_PID $IMU_PID $CAM_PID

echo "Session $SESSION_ID complete."
echo "Files: obd_${SESSION_ID}.csv, gnss_${SESSION_ID}.csv, imu_${SESSION_ID}.csv, video_${SESSION_ID}.mp4"
