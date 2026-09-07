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

# Reuse the actual clock-jump correction logic from ekf_fusion.py
# directly, rather than re-deriving similar logic in this test file:
# aligned.csv on disk always holds RAW, uncorrected timestamps (the
# correction only happens in-memory inside ekf_fusion.py), so a test
# that needs "real elapsed time since the last GPS fix" has to apply
# the identical correction first, or a drive with a genuine clock jump
# would silently get a wrong, misleading gap calculation.
sys.path.insert(0, str(SCRIPTS_DIR))
from ekf_fusion import fix_clock_jumps  # noqa: E402
DRIVES_DIR = REPO_ROOT / "drives"

# Physically-reasonable upper bounds for a passenger vehicle. These are
# deliberately generous -- the point is to catch nonsense (a runaway
# EKF reporting 4000 mph), not to be a precise speed limit checker.
MAX_PLAUSIBLE_SPEED_MPH = 130
MAX_PLAUSIBLE_SPEED_MPH_MS = 60.0  # ~134 mph in m/s -- a little headroom above
                                     # the EKF's own 45 m/s (~100 mph) innovation
                                     # gate threshold, so this test independently
                                     # confirms that gate is doing its job without
                                     # being so tight it flags legitimate rounding
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
    Regression test for the clock-discontinuity bug and for corrupted/
    implausible GPS readings: a single bad timestamp gap once caused
    the EKF to jump the fused position ~2700m in one step, and
    separately a corrupted-but-numerically-valid GPS coordinate once
    produced a multi-thousand-km jump.

    Checks PLAUSIBILITY, not a flat distance -- but critically, NOT by
    dividing a jump by the time between consecutive OUTPUT rows either.
    A Kalman correction is applied instantaneously, in a single ~0.1s
    output row, no matter how many real seconds the GPS gap it's
    correcting for actually spanned -- so "distance / row-to-row dt"
    makes a legitimate multi-hundred-meter catch-up after a real
    30-second blackout look identical to an impossible one after a
    single 0.1s step (confirmed: a real, evidence-backed 386m
    correction computed out to an "implied" 3841 m/s this way, despite
    being entirely legitimate). The correct comparison is the same one
    ekf_fusion.py's own innovation gate uses: distance versus the REAL
    elapsed time since the last genuinely new GPS reading, which has
    to come from aligned.csv's own GNSS timestamps, not fused.csv's
    row-to-row spacing.
    """
    run_script("ekf_fusion.py", ["--aligned", str(aligned_csv), "--out-dir", str(tmp_path)], cwd=tmp_path)
    fused_path = tmp_path / "fused.csv"
    if not fused_path.exists():
        pytest.skip("No valid GPS fix in this session -- EKF fusion doesn't run (expected for a stationary-only drive).")

    fused = pd.read_csv(fused_path)
    fused["timestamp"] = pd.to_datetime(fused["timestamp"])

    aligned = pd.read_csv(aligned_csv)
    aligned["timestamp"] = pd.to_datetime(aligned["timestamp"])
    aligned = fix_clock_jumps(aligned)

    # For each row, find how long it's been since gnss_latitude last
    # took on a genuinely new value (a proxy for "time since the last
    # real GPS fix", matching what the EKF's own gate tracks).
    if "gnss_latitude" not in aligned.columns:
        pytest.skip("No GNSS column in this session's aligned data.")

    is_new_fix = aligned["gnss_latitude"] != aligned["gnss_latitude"].shift(1)
    last_fix_time = aligned["timestamp"].where(is_new_fix).ffill()
    time_since_last_fix = (aligned["timestamp"] - last_fix_time).dt.total_seconds()

    dx = fused["fused_x_m"].diff()
    dy = fused["fused_y_m"].diff()
    step = (dx**2 + dy**2) ** 0.5

    # Align the two frames by row position (both are one row per IMU
    # sample from the same aligned.csv, so this holds as long as
    # ekf_fusion.py doesn't drop rows -- true today).
    n = min(len(step), len(time_since_last_fix))
    gap_s = time_since_last_fix.iloc[:n].reset_index(drop=True)
    step = step.iloc[:n].reset_index(drop=True)

    # A gap of 0 (or near-0) means this row wasn't a fresh GPS
    # correction at all, just an ordinary predict step; use a floor
    # so we're not dividing by a near-zero real gap either.
    MIN_MEANINGFUL_GAP_S = 0.5
    implied_speed = (step / gap_s).where(gap_s >= MIN_MEANINGFUL_GAP_S, 0)
    implied_speed = implied_speed.replace([float("inf"), -float("inf")], 0).fillna(0)
    max_implied_speed = implied_speed.max()

    assert max_implied_speed < MAX_PLAUSIBLE_SPEED_MPH_MS, (
        f"EKF position change implied a speed of {max_implied_speed:.0f} m/s "
        f"relative to actual elapsed time since the last real GPS fix "
        f"(max allowed: {MAX_PLAUSIBLE_SPEED_MPH_MS} m/s) -- likely a clock "
        f"discontinuity, unclamped dt regression, or the GPS innovation gate "
        f"failing to reject a corrupted/implausible fix."
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
