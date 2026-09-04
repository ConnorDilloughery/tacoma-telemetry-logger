"""
Regression tests for the telemetry processing pipeline.

These run the REAL pipeline scripts (align_logs.py, ekf_fusion.py,
extract_event_clips.py, generate_drive_report.py) against the REAL raw
sensor data already committed under drives/*/raw/ -- no mocked data,
no hardware required. That's what makes this CI-friendly: every drive
in the repo doubles as a regression fixture.

Each test targets a SPECIFIC real bug this project hit during
development (see the README's "Engineering Log"), so a future change
that reintroduces one of them fails the build instead of silently
shipping broken output:

- test_ekf_no_position_runaway   -- the clock-discontinuity bug
                                     (a 510s timestamp gap teleported
                                     the fused position ~2700m away)
- test_ekf_speed_is_physical     -- same bug, seen a different way
- test_distance_is_physical      -- the GNSS null-island bug
                                     (an unfiltered (0,0) fix inflated
                                     a 2-mile drive to ~8,000 miles)
- test_alignment_produces_rows   -- basic pipeline sanity: alignment
                                     shouldn't silently produce an
                                     empty or near-empty result

Run with:  pytest tests/ -v
"""

import json
import shutil
import subprocess
import sys
from pathlib import Path

import pandas as pd
import pytest

REPO_ROOT = Path(__file__).resolve().parent.parent
SCRIPTS_DIR = REPO_ROOT / "scripts"
DRIVES_DIR = REPO_ROOT / "drives"

# Physically-reasonable upper bounds for a passenger vehicle. These are
# deliberately generous -- the point is to catch nonsense (a runaway
# EKF reporting 4000 mph), not to be a precise speed limit checker.
MAX_PLAUSIBLE_SPEED_MPH = 130
MAX_PLAUSIBLE_STEP_METERS = 100   # max sane position change in one EKF step
MAX_PLAUSIBLE_DISTANCE_PER_MINUTE_MILES = 2.0  # ~120 mph sustained, generous


def discover_sessions():
    """
    Finds every drive with a COMPLETE set of raw sensor CSVs (OBD,
    GNSS, and IMU) committed under drives/*/raw/.

    Some drives in this repo are intentionally incomplete -- e.g. one
    preserved as evidence of the IMU failing to start at all during
    an undervoltage event (see the README's Engineering Log) -- and
    have only a subset of the usual three files. Those aren't testable
    sessions (align_logs.py requires all three), so we skip anything
    missing a sensor file rather than letting the pipeline crash on
    data that was never meant to be run through it.
    """
    sessions = []
    if not DRIVES_DIR.exists():
        return sessions
    for session_dir in sorted(DRIVES_DIR.iterdir()):
        raw_dir = session_dir / "raw"
        if not raw_dir.exists():
            continue
        obd_files = list(raw_dir.glob("obd_*.csv"))
        gnss_files = list(raw_dir.glob("gnss_*.csv"))
        imu_files = list(raw_dir.glob("imu_*.csv"))
        if obd_files and gnss_files and imu_files:
            sessions.append(session_dir.name)
    return sessions


SESSIONS = discover_sessions()

pytestmark = pytest.mark.skipif(
    not SESSIONS, reason="No drives with raw/ sensor data found under drives/ -- nothing to test against."
)


def run_script(script_name, args, cwd):
    """Runs one of the pipeline scripts as a subprocess, same as a real invocation."""
    result = subprocess.run(
        [sys.executable, str(SCRIPTS_DIR / script_name)] + args,
        cwd=cwd,
        capture_output=True,
        text=True,
    )
    return result


@pytest.fixture(params=SESSIONS)
def session_id(request):
    return request.param


@pytest.fixture
def aligned_csv(tmp_path, session_id):
    """
    Copies a real session's raw CSVs into a scratch directory and runs
    the real align_logs.py against them, exactly as process_drive.sh
    would. Returns the path to the resulting aligned CSV.
    """
    raw_dir = DRIVES_DIR / session_id / "raw"
    work_dir = tmp_path / session_id
    work_dir.mkdir()
    for f in raw_dir.glob(f"*_{session_id}.csv"):
        shutil.copy(f, work_dir / f.name)

    out_path = work_dir / "aligned.csv"
    result = run_script(
        "align_logs.py",
        ["--session", session_id, "--out", str(out_path)],
        cwd=work_dir,
    )
    assert result.returncode == 0, f"align_logs.py failed for {session_id}:\n{result.stderr}"
    assert out_path.exists(), f"align_logs.py did not produce {out_path}"
    return out_path


def test_alignment_produces_rows(aligned_csv):
    """Basic pipeline sanity: alignment shouldn't silently produce an empty result."""
    df = pd.read_csv(aligned_csv)
    assert len(df) > 0, "aligned.csv has zero rows"


def test_ekf_runs_without_error(aligned_csv, tmp_path):
    result = run_script(
        "ekf_fusion.py",
        ["--aligned", str(aligned_csv), "--out-dir", str(tmp_path)],
        cwd=tmp_path,
    )
    assert result.returncode == 0, f"ekf_fusion.py crashed:\n{result.stderr}"


def test_ekf_no_position_runaway(aligned_csv, tmp_path):
    """
    Regression test for the clock-discontinuity bug: a single bad
    timestamp gap once caused the EKF to jump the fused position
    ~2700m in one step. If dt clamping in ekf_fusion.py's predict()
    step is ever removed or broken, this test should catch it.
    """
    run_script("ekf_fusion.py", ["--aligned", str(aligned_csv), "--out-dir", str(tmp_path)], cwd=tmp_path)
    fused_path = tmp_path / "fused.csv"
    if not fused_path.exists():
        pytest.skip("No valid GPS fix in this session -- EKF fusion doesn't run (expected for a stationary-only drive).")

    df = pd.read_csv(fused_path)
    dx = df["fused_x_m"].diff()
    dy = df["fused_y_m"].diff()
    step = (dx**2 + dy**2) ** 0.5
    max_step = step.max()
    assert max_step < MAX_PLAUSIBLE_STEP_METERS, (
        f"EKF position jumped {max_step:.0f}m in a single step (max allowed: "
        f"{MAX_PLAUSIBLE_STEP_METERS}m) -- likely a clock discontinuity or "
        f"unclamped dt regression."
    )


def test_ekf_speed_is_physical(aligned_csv, tmp_path):
    run_script("ekf_fusion.py", ["--aligned", str(aligned_csv), "--out-dir", str(tmp_path)], cwd=tmp_path)
    fused_path = tmp_path / "fused.csv"
    if not fused_path.exists():
        pytest.skip("No valid GPS fix in this session -- EKF fusion doesn't run.")

    df = pd.read_csv(fused_path)
    max_speed = df["fused_speed_mph"].abs().max()
    assert max_speed < MAX_PLAUSIBLE_SPEED_MPH, (
        f"EKF fused speed reached {max_speed:.0f} mph (max allowed: "
        f"{MAX_PLAUSIBLE_SPEED_MPH}) -- likely diverging/unstable filter output."
    )


def test_distance_is_physical(aligned_csv, tmp_path):
    """
    Regression test for the GNSS null-island bug: an unfiltered (0,0)
    GPS fix once inflated a real ~2-mile, 3.6-minute drive's computed
    distance to ~8,000 miles. If the fix_quality/(0,0) filtering in
    generate_drive_report.py's valid_gnss_points() is ever weakened,
    this test should catch it.
    """
    events_path = tmp_path / "events.json"
    run_script(
        "extract_event_clips.py",
        ["--aligned", str(aligned_csv), "--out-dir", str(tmp_path)],
        cwd=tmp_path,
    )
    result = run_script(
        "generate_drive_report.py",
        [
            "--aligned", str(aligned_csv),
            "--out-dir", str(tmp_path),
            "--session", "test_session",
            "--events", str(events_path),
        ],
        cwd=tmp_path,
    )
    assert result.returncode == 0, f"generate_drive_report.py crashed:\n{result.stderr}"

    stats_path = tmp_path / "stats.json"
    assert stats_path.exists(), "generate_drive_report.py did not produce stats.json"
    stats = json.loads(stats_path.read_text())

    duration_min = stats["duration_s"] / 60
    max_plausible_distance = max(duration_min * MAX_PLAUSIBLE_DISTANCE_PER_MINUTE_MILES, 0.5)
    assert stats["distance_miles"] < max_plausible_distance, (
        f"Computed distance ({stats['distance_miles']:.1f} mi) is implausible for a "
        f"{duration_min:.1f}-minute drive (max plausible: {max_plausible_distance:.1f} mi) "
        f"-- likely an unfiltered bad GPS fix."
    )
    assert stats["distance_miles"] >= 0, "Computed distance is negative"
