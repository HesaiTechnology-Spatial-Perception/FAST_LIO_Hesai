# Changelog — ROS 1 branch (main)

All notable changes to the ROS 1 branch. The ROS 2 branch keeps its own CHANGELOG.

## [Unreleased]

- Ignore exited zombie descendants during process-group cleanup in minimal
  containers; still terminate live descendants after their leader exits.

- Order ROS 1 mapping and sanitizer targets after message generation so clean
  parallel builds cannot compile before `fast_lio/Pose6D.h` exists.

- Pin a public driver/SDK reference and document per-model support, message
  contracts, validation scope and unavailable public JT32/MT60 combinations.
- Remove unsupported legacy model launches and obsolete Livox GDB examples;
  install only the supported launch catalog and check it in CI.
- Run installed bag/save/failure integration and actual-node regressions in CI;
  cover ROS1 cold/existing master recording with a synthetic driver.
- Require stable ROS2 DDS matching before resuming playback; preserve exact
  frame/IMU completeness checks, including the first and final scans.

- Install the ROS 1 mapping executable and its actual `rviz_cfg/` assets.

- Ship and install only the five standard model YAMLs; keep recording-specific
  overrides outside the checkout and select them with `--config PATH`.

- Require per-scan startup/registration accounting and final trajectory coverage;
  reject runtime IMU gaps above `common.imu_max_gap` (default 0.05 s).
- Start and stop an owned local ROS1 master for cold PCAP conversion; preserve an existing master.
- Validate all selected bag message header timestamps before replay or successful PCAP conversion;
  forward explicit PCAP `--timestamp-offset` through the unified entry.
- Fix feature-extraction scan-line endpoint bounds and initialize point metadata.
- Monitor live driver exits and bound process-group shutdown through INT/TERM/KILL.
- Add sanitizer and supervisor regressions, plus runtime gap/skipped-tail tests.

### Reliability and map export (local, unreleased)

- Default supplied model exports to sensor-time 1 Hz accumulation when saving is
  enabled; raw, 0.1 m voxel and combined exports remain explicit options.

- ROS 2 bag runs wait for matched publishers and use Reliable transport; control
  services stay responsive while mapping is backlogged.
- Per-run node exit status propagates ROS 1 child failures; runtime logs and
  optional trajectories are stored outside the source tree.
- Require explicit device-matched PCAP angle calibration; pass correction/firetime
  through the unified runner and pass a private config to the driver without editing its config.
- Use driver EOF exit and exact per-topic bag metadata instead of topic silence/text grep.
- Stop on sensor header rollback, validate Hesai PointCloud2 layouts, and resize
  matching buffers to the actual point count; fix cleared-vector writes and feature disB initialization.
- Restore Best Effort-compatible ROS 2 IMU QoS, and drain queued input independently of /clock.
- Add weighted voxel, streamed raw, sensor-time 1 Hz and combined map exports,
  atomic per-file snapshots, reusable checkpoints, JSON manifests and checked save failures.
- Stop duplicate ROS 2 preview accumulation while paused. Publish correctly ordered current odometry covariance.
- Add mapper status, exact input-coverage checks, run provenance and verified PCD payloads.
- Correct input-check exit codes, malformed-PCD handling and trajectory/local-geometry diagnostics;
  these checks do not certify global map accuracy.
- Remove dataset-specific MT60 timing from shipped configs; generic offset is zero.
- Add regression cases and fix shell syntax checks to inspect every script.


### Added
- `tools/run_fastlio.sh`, a single customer entry point for JT16/JT32/JT64P/JT128/MT60 live and rosbag operation, plus PCAP conversion for JT models supported by the selected driver.
- Unified PCD saving through `tools/run_fastlio.sh --save-map [PATH]`; all models keep background PCD buffering disabled unless saving is requested.
- `pcd_save/leaf_size`: export voxel size (0.1 m by default for voxel/all modes); weighted centroids preserve observation counts across snapshots. It does not filter the default 1 Hz export.
- `trajectory_save/tum_en` (default `false`): export the trajectory to `<common.log_dir>/traj_tum.txt` in TUM format (`timestamp x y z qx qy qz qw`) for external evaluation.
- `config_file` argument on model launch files; point to your own yaml without editing the packaged one.
- Startup config summary log line, plus plausibility warnings: implausible per-scan time span (wrong `preprocess/timestamp_unit`), implausibly large gyro bias after IMU init (wrong `common/imu_gyr_unit`), and a notice that `common/time_sync_en` only takes effect with Livox input.
- GitHub Actions build on `ros:noetic-ros-base-focal`; issue and PR templates.

### Fixed
- Invalid scan counts, point-filter values, timestamp units, and malformed
  extrinsics now fail at startup instead of risking array bounds errors,
  divide-by-zero, or invalid state initialization. Empty rings are skipped
  safely when feature extraction is enabled.
- JT64P now uses its actual JT128-compatible 128-ring PointCloud2 layout; the
  configuration no longer drops rings 64–127 when feature extraction is
  enabled, and input checks no longer infer JT64P from a 64-ring threshold.
- IMU angular-velocity handling supports `common/imu_gyr_unit: "auto"`, `"deg"`, or `"rad"` across the supplied model configurations. Auto mode handles known raw (`g`, `deg/s`) and SI (`m/s²`, `rad/s`) driver conventions.
- IMU samples used during automatic unit detection are retained and converted after detection instead of being dropped, preventing rosbag startup from stalling at LiDAR/IMU synchronization.
- Out-of-bounds write in runtime-log buffers after ~20 h of continuous operation (`s_plot11[scan_count]`, `MAXN` overflow).
- Debug record files (`Log/pos_log.txt`, `mat_pre.txt`, `mat_out.txt`, `dbg.txt`, `imu.txt`) were created — and `mat_pre.txt` appended every scan — even with `runtime_pos_log_enable: false`.
- Blind-zone skip loop in feature extraction could read out of bounds when an entire scan was within `blind`.
- `/map_save` reports the actually written point count when downsampling is enabled.

### Changed

- Bag and converted-PCAP playback accepts `--play-rate RATE`, pre-detects IMU units before launch, waits for queued data after playback, and keeps ROS 1 bag output quiet; map saving no longer rewrites an unchanged PCD during shutdown.
- Simplified PCAP-to-rosbag conversion to accept a positional PCAP path and automatically discover the model, driver workspace, calibration files, and output path; added `--dry-run` for inspecting the resolved inputs.
- Removed the unused ROS 1 `eigen_conversions` header and build dependency.
- The expected empty scan right after IMU initialization logs as INFO with an explanation instead of `No point, skip this scan!`.
- Runtime-log buffers (~66 MB) are allocated only when `runtime_pos_log_enable` is set.
- Removed the unused matplotlibcpp / Python build dependency.
