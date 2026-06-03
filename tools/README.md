# FAST_LIO_Hesai Tools

Helper tools for validating input data and preparing rosbag files before running FAST-LIO2.

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
| 5 | `ring` range matches model (JT16: 0–15, JT128: 0–127) | Wrong `scan_line` config |
| 6 | IMU frequency ≥ 100 Hz | IMU pipeline issue |
| 7 | Gyro magnitude sanity check (deg/s vs rad/s) | Wrong `imu_gyr_unit` config |
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
| `--model` | `jt128` | `jt16` or `jt128` |
| `--timeout` | `8.0` | Seconds to wait for messages |
| `--timestamp-unit` | (none) | Your `preprocess.timestamp_unit` (0–3); enables a mismatch check |

---

## tools/check_config.py — Static Config Validator

Validates a FAST-LIO2 yaml config **without** running ROS. Customers can run
it on a config file directly to catch the most common misconfigurations.

**Checks:**

| Item | Failure means |
|------|---------------|
| `preprocess.lidar_type` matches model + ROS version | Wrong LiDAR enum (ROS 1: 5/6, ROS 2: 1/2) |
| `preprocess.scan_line` matches model | Wrong line count |
| `preprocess.timestamp_unit` is a valid enum (0–3) | Invalid unit |
| `common.imu_gyr_unit` is `deg` or `rad` | Invalid unit |
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
| `--model` | ✓ | `jt16` or `jt128` |
| `--ros` | ✓ | `1` or `2` (lidar_type enum differs) |

---

## tools/check_map.py — Map Quality Analyzer

Analyzes FAST-LIO2 **output map quality** and suggests likely causes — distinct
from `check_input.py`/`check_config.py`, which check inputs and config.

**Modes:**

```bash
# Offline: analyze a saved PCD
python3 tools/check_map.py --pcd PCD/scans.pcd

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
- `correction.csv` and `firetime.csv` calibration files for the LiDAR
- IMU data present in the PCAP (required for FAST-LIO2)
- Python 3 with PyYAML: `pip3 install pyyaml`

### ROS 1

```bash
bash tools/pcap_to_rosbag/pcap_to_rosbag_ros1.sh \
  --model      jt128 \
  --pcap       /data/input.pcap \
  --correction /data/correction.csv \
  --firetime   /data/firetime.csv \
  --output     /data/output.bag \
  --driver-ws  ~/hesai_ros_ws
```

Output: a single `/data/output.bag` file.

### ROS 2

```bash
bash tools/pcap_to_rosbag/pcap_to_rosbag_ros2.sh \
  --model      jt128 \
  --pcap       /data/input.pcap \
  --correction /data/correction.csv \
  --firetime   /data/firetime.csv \
  --output     /data/output \
  --driver-ws  ~/hesai_ros2_ws
```

Output: a rosbag2 directory `/data/output/`.

**All options:**

| Option | Required | Description |
| ------ | -------- | ----------- |
| `--model` | ✓ | `jt16` or `jt128` |
| `--pcap` | ✓ | Path to `.pcap` file |
| `--correction` | ✓ | Path to correction `.csv` |
| `--firetime` | ✓ | Path to firetime `.csv` |
| `--output` | ✓ | Output bag path (file for ROS 1, directory for ROS 2) |
| `--driver-ws` | ✓ | Hesai ROS Driver workspace root |
| `--play-rate` | | PCAP playback speed (default: `1.0`) |
| `--lidar-topic` | | Override lidar topic (default: `/lidar_points`) |
| `--imu-topic` | | Override IMU topic (default: `/lidar_imu`) |

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

```
1. PCAP must be parseable by the Hesai ROS Driver.
2. IMU data must be present in the PCAP for FAST-LIO2.
3. /lidar_points must contain 'ring' and 'timestamp' fields.
4. timestamp_unit and imu_gyr_unit in the FAST-LIO2 config must match the PCAP data.
5. Field names in the driver config (e.g. firetime_file_path) may vary across driver versions —
   verify against your driver's actual config.yaml structure.
```
