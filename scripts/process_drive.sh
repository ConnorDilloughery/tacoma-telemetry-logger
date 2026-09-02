#!/bin/bash
# process_drive.sh
# -----------------
# Takes one session's raw files (obd/gnss/imu CSVs + video, produced by
# run_session.sh) and turns them into a self-contained drives/<session>/
# folder: aligned data, hard-brake clips, stats, plots, and a README --
# ready to commit and push to GitHub as-is.
#
# Usage:
#   ./process_drive.sh <session_id>
#
# Example:
#   ./process_drive.sh 20260901_203201
#
# Expects the raw files to be in the current directory, named exactly as
# run_session.sh produces them: obd_<session>.csv, gnss_<session>.csv,
# imu_<session>.csv, video_<session>.mp4, video_<session>_meta.json

set -e

SESSION="$1"
if [ -z "$SESSION" ]; then
    echo "Usage: $0 <session_id>"
    exit 1
fi

DRIVE_DIR="drives/${SESSION}"
mkdir -p "${DRIVE_DIR}/raw"

OBD_CSV="obd_${SESSION}.csv"
GNSS_CSV="gnss_${SESSION}.csv"
IMU_CSV="imu_${SESSION}.csv"
VIDEO="video_${SESSION}.mp4"
VIDEO_META="video_${SESSION}_meta.json"

echo "Processing session ${SESSION}..."

# 1. Align the three sensor logs onto one timeline.
python3 align_logs.py --session "${SESSION}" --out "${DRIVE_DIR}/aligned.csv"

# 2. Detect hard-brake events and pull matching video clips, if a video
#    exists for this session (older sessions before the camera was
#    added won't have one -- that's fine, this step is skipped).
if [ -f "$VIDEO" ] && [ -f "$VIDEO_META" ]; then
    python3 extract_event_clips.py \
        --aligned "${DRIVE_DIR}/aligned.csv" \
        --video "$VIDEO" \
        --meta "$VIDEO_META" \
        --out-dir "${DRIVE_DIR}"
else
    echo "No video found for this session -- skipping clip extraction."
    python3 extract_event_clips.py \
        --aligned "${DRIVE_DIR}/aligned.csv" \
        --out-dir "${DRIVE_DIR}"
fi

# 3. Generate stats, plots, and the per-drive README.
python3 generate_drive_report.py \
    --aligned "${DRIVE_DIR}/aligned.csv" \
    --out-dir "${DRIVE_DIR}" \
    --session "${SESSION}" \
    --events "${DRIVE_DIR}/events.json"

# 4. Move the sensor CSVs into the drive folder so the whole session
#    is self-contained in one place. The full raw video is deliberately
#    NOT kept -- once clips are extracted around each event, the full
#    recording (often 50MB+ for a short drive) just adds bulk to the
#    repo without adding value beyond what the clips already show. Pass
#    --keep-video to preserve it if you want the full recording for
#    some other reason.
mv "$OBD_CSV" "$GNSS_CSV" "$IMU_CSV" "${DRIVE_DIR}/raw/" 2>/dev/null || true

KEEP_VIDEO=false
if [ "$2" == "--keep-video" ]; then
    KEEP_VIDEO=true
fi

if [ -f "$VIDEO" ]; then
    if [ "$KEEP_VIDEO" == true ]; then
        mv "$VIDEO" "${DRIVE_DIR}/raw/"
        [ -f "$VIDEO_META" ] && mv "$VIDEO_META" "${DRIVE_DIR}/raw/"
    else
        rm -f "$VIDEO" "$VIDEO_META"
        echo "Removed full raw video (kept only extracted clips). Use --keep-video to retain it."
    fi
fi

# 5. Rebuild the top-level gallery README across all drives.
python3 update_index.py

echo ""
echo "Done. Drive folder: ${DRIVE_DIR}"
echo "Review it, then: git add ${DRIVE_DIR} README.md && git commit -m \"Add drive ${SESSION}\" && git push"
