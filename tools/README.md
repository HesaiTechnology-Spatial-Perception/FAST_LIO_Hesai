# FAST_LIO_Hesai Tools

## Driver and installed-package support

Use the [driver compatibility matrix](../DRIVER_COMPATIBILITY.md) for the pinned public driver/SDK,
model/mode support, message units and evidence limits. JT32 requires a separately
obtained compatible driver for live/PCAP operation; MT60 needs a verified publisher
and is not registered for unified PCAP conversion. A model YAML is not a hardware
qualification. Do not apply a nonzero `--timestamp-offset` to the pinned public
driver: that driver does not implement the translation option.

Only the five Hesai model launches are distributed (ROS 2 also has the generic
`mapping.launch.py`). Unsupported legacy launches and the old Livox GDB example
have been removed. CI checks both source and installed catalogs, runs synthetic
bag replay and default 1 Hz saving through the installed entry, tests invalid
input and driver failure, and executes the actual-node regressions. ROS1 additionally
checks converter-owned and pre-existing masters with a synthetic driver. These
checks do not replace sensor/firmware qualification or real PCAP decoding tests.
ROS2 playback waits for stable DDS matching before resume; exact input counts and
registration coverage remain the acceptance criteria.

## Runtime coverage and process control

`common.imu_max_gap: 0.05` limits the gap between consecutive IMU sensor timestamps
throughout the run, independently of the opt-in stationary initialization guard.
Use a finite positive value appropriate to the verified sensor rate; pauses in bag
playback do not change sensor-time intervals. A larger gap stops mapping with exit 3.

`/mapping_status` reports `startup_frames` and `skipped_frames` in addition to
`registered_frames`. The runner requires startup + registered = synchronized,
zero skipped frames, and final registered time covering the last input scan.
A scan skipped after initialization makes the run incomplete even if all messages arrived.

Bag replay and PCAP conversion decode every selected LiDAR/IMU message and check
strictly increasing header timestamps. A zero origin is allowed when subsequent times
increase; all-zero streams, duplicates and rollback fail. Recorder receive times do
not substitute for invalid sensor header times. Use `bag_metadata.py --validate-timestamps`
to run the same audit; it reports sensor-time bounds and maximum IMU/LiDAR gaps.
PCAP `--timestamp-offset SEC` is an explicit driver clock translation, also accepted by
`run_fastlio.sh`; verify its time basis and driver support rather than guessing a value.
This audit does not certify per-point timing, lossless PCAP decoding or map accuracy.

ROS1 conversion reuses an available master. If none is reachable and ROS_MASTER_URI
names the local host, it starts an owned roscore, waits for readiness and stops only that
master at exit. An unavailable remote master fails with an explicit instruction.
The live runner monitors both mapper and driver; an unexpected driver exit is a failure.
Owned process groups get SIGINT (up to 30 s), SIGTERM (up to 5 s), then SIGKILL
(up to 1 s); forced or incomplete termination returns failure and cleanup continues.

Run `python3 tools/test_process_control.py` for supervisor regressions. Build with
`-DFAST_LIO_PREPROCESS_ASAN_TEST=ON` and run `fastlio_preprocess_test` for sparse/short
scan-line bounds under AddressSanitizer. `test_runtime_ros.py` includes `imu_gap` and
`skipped_tail` cases and must run in an isolated ROS domain/master.

## Guarded IMU initialization (opt-in)

For a recording with a verified stationary prefix, set these keys under
`mapping` (under `/**/ros__parameters` on ROS 2):

```yaml
imu_init_delay: 0.0
imu_init_duration: 0.5
imu_init_max_gyr_norm: 0.03
```

The limit is in **rad/s after gyro-unit conversion**, independent of the input
unit. It defaults to `0.0` (disabled), preserving legacy initialization.
When enabled, the whole fixed window is checked before setting gravity and
gyro bias: finite/nonzero acceleration, finite gyro below the limit, strictly
increasing IMU timestamps, no gaps over 50 ms, at least 11 samples, and coverage
within 50 ms of both window boundaries. Duration must be at least 0.1 s.
Failure stops mapping with `[imu_init_guard]` and exit code 2; it does not
silently search a later window. The accepted sensor timestamps and sample
count are logged. Delay is relative to the first initialization LiDAR frame,
not necessarily the start of the bag. A 0.03 rad/s limit is a screening setting,
not a universal sensor specification or a proof of stationarity (e.g. constant
translation cannot be detected). Confirm stationary recording and units;
do not raise the limit simply to bypass a failure.

For ROS 2 playback, start the player with `--start-paused`, wait for the
player publishers and mapper subscriptions to be discovered, then call
`/rosbag2_player/resume` (`rosbag2_interfaces/srv/Resume`). Waiting before
creating the player does not establish its DDS connections. Graph discovery
alone still does not guarantee every message was delivered.

Early IMU-processing returns now clear the output scan, so an empty IMU group
cannot reuse the previous scan in a new mapping update. This is a safety fix,
not evidence that this condition caused a particular failed map.

Standalone regression test (no ROS nodes or playback):

```bash
g++ -std=c++14 -Iinclude tools/test_imu_initialization_guard.cpp -o /tmp/fastlio_imu_guard_test
/tmp/fastlio_imu_guard_test
```

Helper tools for validating input data and preparing rosbag files before running FAST-LIO2.

---

## tools/run_fastlio.sh — One-command Customer Entry

Choose the LiDAR model and input mode; the script handles the remaining launch
and playback steps:

```bash
# Live sensor: start the Hesai driver and FAST-LIO2
./tools/run_fastlio.sh jt128 live

# Existing rosbag: start FAST-LIO2 and play the bag
./tools/run_fastlio.sh jt32 bag /data/jt32.bag

# MT60 rosbag: start the MT60 configuration and play the bag
./tools/run_fastlio.sh mt60 bag /data/mt60_bag

# Optional: replay a bag or converted PCAP at 2x speed
./tools/run_fastlio.sh jt16 bag /data/jt16_bag --play-rate 2.0

# PCAP: convert it, start FAST-LIO2, and play the generated bag
./tools/run_fastlio.sh jt128 pcap /data/JT128/input.pcap --correction /data/calibration/device.csv

# Add to any mode to save a timestamped PCD under ~/slam_ws/src/slam_maps/
./tools/run_fastlio.sh jt128 live --save-map
```

The same entry supports JT16, JT32, JT64P, JT128, and MT60 on both branches for live
and existing-bag input. ROS 1/ROS 2 is
detected from the branch. Use `--dry-run` to inspect the resolved workspaces and
actions, use `--play-rate RATE` for bag/PCAP playback, or use `--fastlio-ws` /
`--driver-ws` when a workspace is in a custom
location. JT32 live and PCAP modes still require a driver with UDP 1.12 support.
MT60 PCAP conversion is not registered; use a live sensor or an existing bag.
Use `--imu-gyr-unit rad|deg|auto` to override the model YAML. Otherwise the
YAML unit is preserved. Live, bag, and PCAP modes all delegate `auto` to the
node's multi-sample heuristic; the runner never guesses from the first IMU
sample or the driver folder/version. Acceleration scale cannot prove gyro
units. Prefer explicit units verified for the actual driver output; an old
bag keeps the units of the driver that recorded it, not today's driver.
For example, append `--imu-gyr-unit rad` only for confirmed rad/s input.
Runtime configuration is validated before conversion/launch, and its source,
effective gyro unit, and whether conversion is applied are printed at startup.
Run offline regression checks with `python3 tools/test_runtime_config.py`.
PCD buffering is off by default for every model and is enabled only when
`--save-map [PATH]` is requested.

---

## Map export and run validation

PCD export is opt-in (`--save-map`). Supplied model YAMLs default to a **sensor-time 1 Hz
accumulated map**. The estimator's `filter_size_surf` and `filter_size_map`
are independent of export resolution. The default 1 Hz export applies no
additional voxel filtering; `leaf_size` is used only by `voxel` and `all`.

```bash
# Default 1 Hz accumulated map in a new run directory
./tools/run_fastlio.sh jt128 bag /data/jt128_bag --save-map
# Three comparable exports from the same registered scans
./tools/run_fastlio.sh jt128 bag /data/jt128_bag --save-map --map-mode all
# Optional 0.1 m voxel map
./tools/run_fastlio.sh jt128 bag /data/jt128_bag --save-map --map-mode voxel
```

| `pcd_save.mode` / `--map-mode` | Output |
| --- | --- |
| `voxel` | Weighted centroids of all registered points; `leaf_size` / `--map-leaf-size` controls resolution |
| `raw` | All registered scans after preprocessing and IMU initialization; this is not the original sensor packet data |
| `1hz` | First registered scan in each one-second bucket, anchored at the first registered timestamp |
| `all` | Main voxel PCD plus sibling `_origin.pcd` and `_1hz.pcd` files |

Raw and 1 Hz records are spooled to private temporary files rather than retained
in RAM. Voxel accumulation preserves every observation's weight across frames
and repeated saves; its memory still grows with newly explored voxels. Raw
snapshots currently need disk space for both the spool and the published PCD.
All exports contain compact binary float32 XYZI records (16 bytes/point).

Each PCD is written to a temporary file beside its destination and atomically
renamed after a successful write. A failure leaves the previous snapshot intact
and returns an error. Atomicity is per file, not across the three-file set.
`pcd_save.interval: -1` saves on request/exit; a positive value checkpoints the
**whole accumulated map** every N registered scans without clearing it. A save
without new scans does not rewrite the files. `<map_file_path>.json` records
point counts, modes and sensor-time coverage. Saving/density changes do not
correct drift or add loop closure.

With no explicit path, output goes to a new
`~/slam_ws/src/slam_maps/fastlio_<model>_<run>/map.pcd` directory. Manual nodes
also allocate a unique default directory. Relative paths are resolved from the
process working directory; parent directories are created automatically.
Legacy YAMLs without `mode` retain `leaf_size=0` → raw and `leaf_size>0` → voxel.

The runner stores `run.json`, `effective.yaml`, process logs, `drain.json`,
`map_save.json`, `node_status.json` and `result.json` under `~/slam_ws/logs/fastlio_<model>_<run>/`
(or `--log-dir`). Metadata includes commit, branch and dirty-worktree state.
It waits for mapper readiness, uses a paused ROS 2 player until the mapper reports matched publishers,
checks exact LiDAR/IMU receipt counts and queue exhaustion, then checks the save
service response and PCD payload length. `--drain-timeout` and `--save-timeout`
default to 120 seconds. Missing messages or an IMU-uncovered tail are reported
as incomplete processing; a diagnostic partial map may still be saved.
**Processing completeness is not geometric accuracy.**
The node writes its own final exit result so ROS 1 launch wrappers cannot hide
a child failure. `common.log_dir` also holds enabled TUM trajectories and
diagnostic logs; the runner sets it to the run directory.

`/mapping_status` (`std_srvs/Trigger`) exposes counters and last timestamps.
ROS 2 `/Laser_map` is an optional 0.2 m voxel preview (`publish.map_leaf_size`),
updated from distinct sensor-time 1 Hz samples. Its publication timer does not
append the last scan again while input is paused. The preview, exported maps
and internal ikd-Tree map have different roles.

LiDAR/IMU header time rollback, repeated timestamps, or non-finite IMU
measurements stop the node with exit code 3. Start a new
mapping run after resetting a device clock or rewinding a bag; live estimator
state is not reused across timelines. ROS 2 live subscriptions default to Best Effort and accept both Best Effort
and Reliable publishers. The runner sets `common.reliable_input: true` and
a Reliable player QoS override for bag playback, then waits for actual matched
publishers before resuming. Manual Reliable subscriptions require Reliable
publishers; message-count verification remains necessary. Input-check FAIL results now return a
nonzero exit code; the short live checker is not a complete bag audit.

`config/` ships only the five standard model baselines: `jt16.yaml`, `jt32.yaml`,
`jt64p.yaml`, `jt128.yaml`, and `MT60.yaml`. Their LiDAR/IMU clock offsets are zero;
no recording-specific offset or startup delay is preset. These are model defaults,
not device calibration certificates. Verify IMU units and LiDAR/IMU extrinsics for
your device. Keep device- or recording-specific overrides outside the checkout and
select them explicitly with `--config /absolute/path/to/custom.yaml`. Saving remains
opt-in; the default export mode is sensor-time `1hz`.

---

## tools/check_input.py — Runtime Input Validator

Validates live LiDAR and IMU input (driver running or rosbag playing) before
starting FAST-LIO2.

**Checks:**

| # | Item | Failure means |
|---|------|---------------|
| 1 | `/lidar_points` topic exists | Driver not started |
| 2 | `/lidar_imu` topic exists | Driver not started |
| 3 | PointCloud2 fields and datatypes match `x/y/z/intensity=float32`, `ring=uint16`, `timestamp=float64` | FAST-LIO2 may reject or misread the cloud |
| 4 | Finite, varying per-point timestamps over the full inspected frame | Invalid timing prevents undistortion; packet ordering is reported separately |
| 4b | `timestamp_unit` inferred from data (and compared to config if given) | Wrong `timestamp_unit` → undistortion wrong |
| 4c | Frame interval stability / dropped frame detection | Frame loss degrades mapping |
| 5 | `ring` range matches model (JT16: 0–15, JT32: 0–31, JT64P/JT128: 0–127, MT60 MT_V7: 0–1) | Wrong `scan_line` config or driver layout |
| 6 | IMU frequency ≥ 100 Hz | IMU pipeline issue |
| 7 | Driver IMU units from acceleration norm (raw vs SI) | Wrong `imu_gyr_unit` config |
| 8 | `frame_id` of both sensors | TF / coordinate frame risk |
| 9 | LiDAR ↔ IMU time-base synchronization | Different clock sources → sync failure |

**Usage:**

```bash
# ROS 2
ros2 run fast_lio check_input.py --model jt128

# ROS 1
rosrun fast_lio check_input.py --model jt16

# Standalone (with ROS already running)
python3 tools/check_input.py --lidar_topic /lidar_points --imu_topic /lidar_imu --model jt128

# Compare against your configured timestamp_unit
python3 tools/check_input.py --model jt128 --timestamp-unit 0
```

| Option | Default | Description |
| ------ | ------- | ----------- |
| `--lidar_topic` | `/lidar_points` | Point cloud topic |
| `--imu_topic` | `/lidar_imu` | IMU topic |
| `--model` | `auto` | `jt16`, `jt32`, `jt64p`, `jt128`, `mt60`, or auto-detect |
| `--timeout` | `8.0` | Seconds to wait for messages |
| `--timestamp-unit` | (none) | Your `preprocess.timestamp_unit` (0–3); enables a mismatch check |

JT64P and JT128 share the same 128-ring PointCloud2 layout, so automatic ring
inspection reports a shared layout instead of guessing the product model. Pass
`--model jt64p` or `--model jt128` explicitly.

---

## tools/check_config.py — Static Config Validator

Validates a FAST-LIO2 yaml config **without** running ROS. Customers can run
it on a config file directly to catch the most common misconfigurations.

**Checks:**

| Item | Failure means |
|------|---------------|
| `preprocess.lidar_type` matches model + ROS version | Wrong LiDAR enum (ROS 1: 5/7/9/6/8, ROS 2: 1/3/5/2/4 for JT16/JT32/JT64P/JT128/MT60); JT64P's selector uses the same parser/schema as JT128 |
| `preprocess.scan_line` matches model | Wrong line count |
| `preprocess.timestamp_unit` is a valid enum (0–3) | Invalid unit |
| `point_filter_num` is a positive integer | Runtime divide-by-zero |
| `common.imu_gyr_unit` is `auto`, `deg`, or `rad` | Invalid unit |
| `preprocess.blind` positive and below `det_range` | All points filtered out |
| `mapping.extrinsic_R` is a valid rotation (orthonormal, det ≈ 1) | Bad extrinsic matrix |
| `mapping.extrinsic_T` has 3 elements | Malformed extrinsic |
| `pcd_save.pcd_save_en` vs `map_file_path` writability | PCD won't save |

Auto-detects ROS 1 (flat) vs ROS 2 (`/**: ros__parameters`) layout.
The mapping node repeats the bounds and extrinsic checks at startup and exits
with a fatal configuration error before subscribing when they fail.

**Usage:**

```bash
python3 tools/check_config.py --config config/jt128.yaml --model jt128 --ros 2
python3 tools/check_config.py --config config/jt16.yaml  --model jt16  --ros 1
```

| Option | Required | Description |
| ------ | -------- | ----------- |
| `--config` | ✓ | Path to the yaml config |
| `--model` | ✓ | `jt16`, `jt32`, `jt64p`, `jt128`, or `mt60` |
| `--ros` | ✓ | `1` or `2` (lidar_type enum differs) |

---

## tools/check_map.py — Map Quality Analyzer

Analyzes FAST-LIO2 **output map quality** and suggests likely causes — distinct
from `check_input.py`/`check_config.py`, which check inputs and config.

**Modes:**

```bash
# Offline: analyze a saved PCD
python3 tools/check_map.py --pcd ~/slam_ws/src/slam_maps/your_map.pcd

# Live: analyze the published map + trajectory
ros2 run fast_lio check_map.py --map-topic /Laser_map --odom-topic /Odometry   # ROS 2
rosrun fast_lio check_map.py --map-topic /Laser_map --odom-topic /Odometry       # ROS 1
```

**Metrics → meaning:**

| Metric | High value indicates |
|--------|----------------------|
| Local plane residual in 0.3 m cells | Local non-planarity; cannot detect separated duplicate walls or certify alignment |
| Point count / density | Too sparse → frame drops or `point_filter_num` too large |
| Trajectory Z change, range and maximum excursion | Coordinate-frame height variation; requires gravity alignment and actual motion/reference data to interpret as error |

Dependencies: `numpy` (required); `open3d` optional (faster PCD loading).
Thickness analysis is numpy-only, so it works without extra packages.

| Option | Default | Description |
| ------ | ------- | ----------- |
| `--pcd` | — | Offline: path to a `.pcd` map |
| `--map-topic` | `/Laser_map` | Live: accumulated map topic |
| `--odom-topic` | `/Odometry` | Live: odometry topic for trajectory |
| `--timeout` | `10` | Live: seconds to wait for the map |

> Live `/Laser_map` requires `publish.map_en` (ROS 2) or is unavailable on ROS 1
> by default — use `--pcd` for offline analysis in that case.

---

## tools/pcap_to_rosbag/ — PCAP → rosbag Converter

Converts a Hesai PCAP file to a FAST-LIO2-ready rosbag by driving the Hesai ROS Driver in PCAP playback mode and recording the output topics.

**Pipeline:**

```
input.pcap
  ↓  Hesai ROS Driver (source_type: 2)
/lidar_points + /lidar_imu
  ↓  rosbag record / ros2 bag record
output.bag  (ROS 1)  or  output/  (ROS 2)
  ↓  FAST-LIO2
/Odometry  /path  /cloud_registered  PCD map
```

**Prerequisites:**

- PCAP file parseable by Hesai ROS Driver
- An explicitly supplied device-matched `--correction` file and, if required, `--firetime`
- IMU data present in the PCAP (required for FAST-LIO2)
- Python 3 with PyYAML: `pip3 install pyyaml`

### Quick start

Set the driver workspace once when it is not in a standard location.

ROS 1:

```bash
export HESAI_DRIVER_WS=~/hesai_ros_ws
```

ROS 2:

```bash
export HESAI_DRIVER_WS=~/hesai_ros2_ws
```

Then pass the PCAP path and its device-matched `--correction` file. Put the model name in the file or parent directory
(`JT16`, `JT32`, `JT64P`, or `JT128`) so the script can infer it:

ROS 1 (writes `/data/JT128/input_ros1.bag`):

```bash
bash tools/pcap_to_rosbag/pcap_to_rosbag_ros1.sh /data/JT128/input.pcap --correction /data/calibration/device.csv
```

ROS 2 (writes `/data/JT128/input_ros2_bag/`):

```bash
bash tools/pcap_to_rosbag/pcap_to_rosbag_ros2.sh /data/JT128/input.pcap --correction /data/calibration/device.csv
```

Run with `--dry-run` first to see every resolved path without starting ROS or
changing files. If the current directory contains exactly one PCAP and its path
includes the model, the PCAP argument can also be omitted.

**All options:**

| Option | Default | Description |
| ------ | -------- | ----------- |
| positional path / `--pcap` | The only PCAP in the current directory | Input `.pcap` file |
| `--model` | Inferred from a JT16/JT32/JT64P/JT128 path | LiDAR model |
| `--correction` | Required | Device-matched angle correction file; never selected automatically |
| `--firetime` | Unset | Explicit device-matched firetime file, if required |
| `--output` | Beside the PCAP with `_ros1.bag` / `_ros2_bag` suffix | Output path |
| `--driver-ws` | `HESAI_DRIVER_WS` or a common workspace path | Hesai driver workspace |
| `--play-rate` | `1.0` | PCAP playback speed |
| `--timestamp-offset` | `0.0` | Seconds added to driver timestamps |
| `--lidar-topic` | `/lidar_points` | Point cloud topic |
| `--imu-topic` | `/lidar_imu` | IMU topic |
| `--dry-run` | Off | Resolve and print inputs without starting ROS |

**What the script does internally:**

1. Validates explicit calibration paths and generates a private driver config.
2. Starts the recorder, then starts the driver with `config_path` pointing to that config.
3. Waits for a normal driver EOF exit; topic silence is not accepted as EOF.
4. Closes the bag and checks exact per-topic types/counts through structured metadata.
5. Prints the result; the original driver config is never replaced.

The driver must support `config_path` and exit at PCAP EOF (`IsPlayEnded`).
Older builds that stay alive after EOF must be updated; Ctrl+C cancels conversion.
Nonempty topics do not by themselves prove lossless decoding or recording.

**Known limitations:**

1. PCAP must be parseable by the Hesai ROS Driver.
2. IMU data must be present in the PCAP for FAST-LIO2.
3. `/lidar_points` must contain `ring` and `timestamp` fields.
4. `timestamp_unit` must match the PCAP data. Prefer an explicit `imu_gyr_unit` verified from the publishing driver contract.
5. Field names in the driver config (e.g. `firetimes_path`) may vary across driver versions —
   verify against your driver's actual config.yaml structure.
6. JT32 requires a driver with UDP 1.12 support; the current public driver does not provide it.


## Regression tests

Offline: `python3 tools/test_runtime_config.py`, `python3 tools/test_reliability.py`,
and the standalone C++ tests listed in `.github/workflows/ci.yml`.
After building, `tools/test_runtime_ros.py --help` describes the integration test
arguments. Use an isolated ROS domain/master and a new output directory. It
publishes synthetic input, exercises a scan with more than 100,000 retained
points, verifies repeated saves and paused previews, and checks fatal input
errors. It is not a sensor accuracy benchmark.

For a newer rosbag2 metadata format rejected by Humble, a single SQLite bag can
be passed as its `.db3` file. Keep the original metadata; split bags and other
storage formats require compatible ROS tooling, not silently dropping files.
