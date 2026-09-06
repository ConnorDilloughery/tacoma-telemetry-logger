# Drive: 20260904_061729 (partial -- IMU failed to start)

This drive is preserved as evidence for the ongoing undervoltage
investigation (see the top-level README's Engineering Log), not
processed through the normal pipeline.

- **OBD:** 2,977 rows recorded successfully. Real driving data --
  speeds ranged 0-33.6 mph (mean 13.1 mph), consistent with genuine
  stop-and-go driving, not a stationary/idle session.
- **GNSS:** 1,143 rows recorded successfully.
- **IMU:** never started. No `imu_20260904_061729.csv` was created at
  all -- not a mid-session freeze (which the staleness-detection fix
  now catches and recovers from), but a failure to initialize the
  I2C connection at the very start of the session.

No alignment, EKF fusion, or report was generated for this drive,
since the pipeline assumes all three sensors are present. The raw
OBD and GNSS CSVs are kept in `raw/` for reference.

This is the strongest single piece of evidence yet for the
undervoltage hypothesis: the IMU didn't degrade partway through like
in previous drives, it failed outright at startup, consistent with
insufficient power being available at the moment the session began.
