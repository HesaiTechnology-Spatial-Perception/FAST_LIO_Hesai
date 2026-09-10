# FAST_LIO_Hesai Tools

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
./tools/run_fastlio.sh jt128 pcap /data/JT128/input.pcap

# Add to any mode to save a timestamped PCD under ~/slam_ws/src/slam_maps/
./tools/run_fastlio.sh jt128 live --save-map
```

The same entry supports JT16, JT32, JT128, and MT60 on both branches for live
and existing-bag input. ROS 1/ROS 2 is
detected from the branch. Use `--dry-run` to inspect the resolved workspaces and
actions, use `--play-rate RATE` for bag/PCAP playback, or use `--fastlio-ws` /
`--driver-ws` when a workspace is in a custom
location. JT32 live and PCAP modes still require a driver with UDP 1.12 support.
MT60 PCAP conversion is not registered; use a live sensor or an existing bag.
For bag/PCAP input, the script reads the first valid IMU sample before launch
and explicitly selects `deg` or `rad`, avoiding startup-order sensitivity.
PCD buffering is off by default for every model and is enabled only when
`--save-map [PATH]` is requested.

---

## tools/check_input.py — Runtime Input Validator

Validates live LiDAR and IMU input (driver running or rosbag playing) before
starting FAST-LIO2.

**Checks:**

| # | Item | Failure means |
|---|------|---------------|
| 1 | `/lidar_points` topic exists | Driver not started |
| 2 | `/lidar_imu` topic exists | Driver not started |
| 3 | PointCloud2 has required fields (`x y z intensity ring timestamp`) | FAST-LIO2 will fail to parse |
| 4 | `timestamp` is per-point and monotonically increasing | Motion undistortion broken |
| 4b | `timestamp_unit` inferred from data (and compared to config if given) | Wrong `timestamp_unit` → undistortion wrong |
| 4c | Frame interval stability / dropped frame detection | Frame loss degrades mapping |
| 5 | `ring` range matches model (JT16: 0–15, JT32: 0–31, JT128: 0–127, MT60 MT_V7: 0–1) | Wrong `scan_line` config |
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
| `--model` | `auto` | `jt16`, `jt32`, `jt128`, `mt60`, or auto-detect |
| `--timeout` | `8.0` | Seconds to wait for messages |
| `--timestamp-unit` | (none) | Your `preprocess.timestamp_unit` (0–3); enables a mismatch check |

---

## tools/check_config.py — Static Config Validator

Validates a FAST-LIO2 yaml config **without** running ROS. Customers can run
it on a config file directly to catch the most common misconfigurations.

**Checks:**

| Item | Failure means |
|------|---------------|
| `preprocess.lidar_type` matches model + ROS version | Wrong LiDAR enum (ROS 1: 5/7/6/8, ROS 2: 1/3/2/4 for JT16/JT32/JT128/MT60) |
| `preprocess.scan_line` matches model | Wrong line count |
| `preprocess.timestamp_unit` is a valid enum (0–3) | Invalid unit |
| `common.imu_gyr_unit` is `auto`, `deg`, or `rad` | Invalid unit |
| `preprocess.blind` positive and below `det_range` | All points filtered out |
| `mapping.extrinsic_R` is a valid rotation (orthonormal, det ≈ 1) | Bad extrinsic matrix |
| `mapping.extrinsic_T` has 3 elements | Malformed extrinsic |
| `pcd_save.pcd_save_en` vs `map_file_path` writability | PCD won't save |

Auto-detects ROS 1 (flat) vs ROS 2 (`/**: ros__parameters`) layout.

**Usage:**

```bash
python3 tools/check_config.py --config config/jt128.yaml --model jt128 --ros 2
python3 tools/check_config.py --config config/jt16.yaml  --model jt16  --ros 1
```

| Option | Required | Description |
| ------ | -------- | ----------- |
| `--config` | ✓ | Path to the yaml config |
| `--model` | ✓ | `jt16`, `jt32`, `jt128`, or `mt60` |
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
| Surface thickness (plane-fit residual) | Ghosting / double surfaces / misalignment → check extrinsics, time sync, `imu_gyr_unit` |
| Point count / density | Too sparse → frame drops or `point_filter_num` too large |
| Trajectory Z drift (live) | Insufficient init motion / degeneracy |

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
- A LiDAR-specific correction file; the script discovers it from the driver config or bundled SDK when possible
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

Then pass only the PCAP path. Put the model name in the file or parent directory
(`JT16`, `JT32`, or `JT128`) so the script can infer it:

ROS 1 (writes `/data/JT128/input_ros1.bag`):

```bash
bash tools/pcap_to_rosbag/pcap_to_rosbag_ros1.sh /data/JT128/input.pcap
```

ROS 2 (writes `/data/JT128/input_ros2_bag/`):

```bash
bash tools/pcap_to_rosbag/pcap_to_rosbag_ros2.sh /data/JT128/input.pcap
```

Run with `--dry-run` first to see every resolved path without starting ROS or
changing files. If the current directory contains exactly one PCAP and its path
includes the model, the PCAP argument can also be omitted.

**All options:**

| Option | Default | Description |
| ------ | -------- | ----------- |
| positional path / `--pcap` | The only PCAP in the current directory | Input `.pcap` file |
| `--model` | Inferred from a JT16/JT32/JT128 path | LiDAR model |
| `--correction` | Driver config or bundled SDK file | Angle correction file |
| `--firetime` | Bundled SDK file when available | Firetime correction file |
| `--output` | Beside the PCAP with `_ros1.bag` / `_ros2_bag` suffix | Output path |
| `--driver-ws` | `HESAI_DRIVER_WS` or a common workspace path | Hesai driver workspace |
| `--play-rate` | `1.0` | PCAP playback speed |
| `--timestamp-offset` | `0.0` | Seconds added to driver timestamps |
| `--lidar-topic` | `/lidar_points` | Point cloud topic |
| `--imu-topic` | `/lidar_imu` | IMU topic |
| `--dry-run` | Off | Resolve and print inputs without starting ROS |

**What the script does internally:**

1. Generates a temporary Hesai ROS Driver config with `source_type: 2` and PCAP paths
2. Backs up the original driver config, applies the temp config
3. Launches the driver in PCAP mode
4. Waits for `/lidar_points` and `/lidar_imu` to appear
5. Checks that point cloud contains `ring` and `timestamp` fields
6. Starts `rosbag record` / `ros2 bag record`
7. Detects PCAP end (topic silence ≥ 8 s) and stops recording
8. Restores the original driver config
9. Validates the output bag and prints FAST-LIO2 launch commands

**Known limitations:**

1. PCAP must be parseable by the Hesai ROS Driver.
2. IMU data must be present in the PCAP for FAST-LIO2.
3. `/lidar_points` must contain `ring` and `timestamp` fields.
4. `timestamp_unit` must match the PCAP data. Keep `imu_gyr_unit: "auto"` unless a manual override is required.
5. Field names in the driver config (e.g. `firetimes_path`) may vary across driver versions —
   verify against your driver's actual config.yaml structure.
6. JT32 requires a driver with UDP 1.12 support; the current public driver does not provide it.
