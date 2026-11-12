---
name: run-fastlio2-hesai-ros2
description: >-
  Set up and run FAST-LIO2 on Hesai JT16 / JT32 / JT64P / JT128 / MT60 LiDARs on ROS 2 Humble
  (FAST_LIO_Hesai ROS2 branch). Detects and configures the local ROS 2
  environment, then runs FAST-LIO2 from a PCAP capture, an existing rosbag, or
  a live sensor. Use when building, configuring, validating, or running
  FAST-LIO2 mapping with a Hesai JT or MT60 LiDAR on ROS 2, converting a PCAP to a bag,
  or bringing up the lidar live.
---

# Run FAST-LIO2 on Hesai JT — ROS 2

Target: **ROS 2 Humble**. Build with `colcon`, launch with `ros2 launch`.
Package name: `fast_lio`. Run commands from the ROS 2 workspace root.

## Driver and installed-package support

Use the [driver compatibility matrix](../../DRIVER_COMPATIBILITY.md) for the pinned public driver/SDK,
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

## Standard configurations

`config/` ships only the five standard model baselines: `jt16.yaml`, `jt32.yaml`,
`jt64p.yaml`, `jt128.yaml`, and `MT60.yaml`. Their LiDAR/IMU clock offsets are zero;
no recording-specific offset or startup delay is preset. These are model defaults,
not device calibration certificates. Verify IMU units and LiDAR/IMU extrinsics for
your device. Keep device- or recording-specific overrides outside the checkout and
select them explicitly with `--config /absolute/path/to/custom.yaml`. Saving remains
opt-in; the default export mode is sensor-time `1hz`.

## Runtime acceptance

Use the unified entry's full header timestamp audit and frame accounting. A run with
`skipped_frames > 0`, missing registration coverage, or an IMU sensor-time gap above
`common.imu_max_gap` (default 0.05 s) is incomplete. Do not declare success from received
message counts alone. A PCAP clock offset must be explicitly verified and supplied
with `--timestamp-offset`; all-zero header times are a conversion failure. ROS1 cold
conversion manages its own local master and preserves an existing master. Live driver
exit or forced shutdown is a failure. Default map saving remains opt-in sensor-time 1 Hz.

## Recommended customer entry

```bash
./tools/run_fastlio.sh jt128 live
./tools/run_fastlio.sh jt32 bag /data/jt32_bag
./tools/run_fastlio.sh jt16 bag /data/jt16_bag --play-rate 2.0
./tools/run_fastlio.sh mt60 bag /data/mt60_bag
./tools/run_fastlio.sh jt128 pcap /data/JT128/input.pcap --correction /data/calibration/device.csv
```

This entry starts the required processes, handles playback or conversion, and
cleans up on exit. `--imu-gyr-unit rad|deg|auto` overrides the model YAML;
without it, the YAML setting is preserved. All input modes use the node's
multi-sample heuristic only when the effective unit is `auto`; no first-sample
bag inference is performed. Acceleration scale cannot prove gyro units:
prefer explicit units confirmed for the publishing driver (or the driver
that recorded an existing bag), not a version guessed from its folder name.
`--play-rate RATE` controls bag/PCAP replay speed.

## Workflow checklist

```
- [ ] Step 0: Confirm LiDAR model
- [ ] Step 1: Detect environment
- [ ] Step 2: Configure & build (if needed)
- [ ] Step 3: Pick data source (PCAP / rosbag / live sensor)
- [ ] Step 4: Validate config and input
- [ ] Step 5: Run FAST-LIO2 and view in RViz
```

## Step 0: Auto-detect LiDAR model

Detect JT models from the point cloud `ring` field when possible. MT60 MT_V7
publishes only rings 0 and 1, which is indistinguishable from a sparse JT frame.
JT64P and JT128 both publish rings 0–127 with the same PointCloud2 schema.
Pass `--model mt60`, `--model jt64p`, or `--model jt128` explicitly for these
ambiguous layouts.

With the sensor publishing or a rosbag2 playing (see Step 3), run:

```bash
ros2 run fast_lio check_input.py        # --model defaults to "auto"
```

It prints e.g. `auto-detected model: JT16 (ring max=15)`. Record the detected
model as `MODEL` (`jt16`, `jt32`, `jt64p`, `jt128`, or `mt60`) and substitute it in every command below.

| Model | Lines | Config | Launch file | `lidar_type` (ROS 2) |
| --- | --- | --- | --- | --- |
| JT16 | 16 | `config/jt16.yaml` | `mapping_jt16.launch.py` | 1 |
| JT32 | 32 | `config/jt32.yaml` | `mapping_jt32.launch.py` | 3 |
| JT64P | 128 | `config/jt64p.yaml` | `mapping_jt64p.launch.py` | 5 |
| JT128 | 128 | `config/jt128.yaml` | `mapping_jt128.launch.py` | 2 |
| MT60 MT_V7 | 2 | `config/MT60.yaml` | `mapping_mt60.launch.py` | 4 |

> JT32 support is pre-adapted in FAST-LIO2, but the current public Hesai ROS
> Driver does not parse JT32 UDP 1.12. JT32 currently requires a compatible
> driver or a prepared bag with valid point-cloud and IMU topics.

> If no data is flowing yet, first bring up the source (Step 3 Path C for a live
> sensor, or play a bag), then run the detection above.

## Step 1: Detect environment

```bash
echo "ROS_DISTRO=$ROS_DISTRO"                 # expect: humble
echo "RMW=$RMW_IMPLEMENTATION"
ros2 --version 2>/dev/null || echo "ros2 not on PATH — source /opt/ros/humble/setup.bash"
ls install/ 2>/dev/null && echo "workspace built" || echo "workspace NOT built"
python3 -c "import yaml" 2>/dev/null && echo "PyYAML ok" || echo "PyYAML missing"
```

Decision:
- `ros2` missing → `source /opt/ros/humble/setup.bash`.
- workspace not built → Step 2.
- workspace built → `source install/setup.bash`, then Step 3.

## Step 2: Configure & build

```bash
sudo apt update
sudo apt install -y ros-humble-pcl-ros ros-humble-pcl-conversions \
  ros-humble-tf2-ros libeigen3-dev libpcl-dev
pip3 install pyyaml

git submodule update --init --recursive

source /opt/ros/humble/setup.bash
colcon build --packages-select fast_lio --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 pkg prefix fast_lio
```

## Step 3: Pick data source

| Data the user has | Path |
| --- | --- |
| A `.pcap` capture (e.g. PandarView) | A |
| An existing rosbag2 directory | B |
| Nothing — live LiDAR | C |

### Path A: PCAP → rosbag2

```bash
# Set once only when the driver workspace cannot be discovered automatically.
export HESAI_DRIVER_WS=~/hesai_ros2_ws

# The JT16/JT32/JT128 directory name supplies the model; MT60 PCAP conversion
# is not registered.
bash tools/pcap_to_rosbag/pcap_to_rosbag_ros2.sh /path/to/JT128/input.pcap --correction /data/calibration/device.csv
```

Pass a device-matched `--correction` file explicitly; add `--firetime` when
required. Neither MODEL nor an old driver config proves device identity.
The driver must accept a private `config_path` and exit normally at PCAP EOF.
Continue with Path B.

### Path B: Run from a rosbag2

```bash
# Terminal 1
source install/setup.bash
ros2 launch fast_lio mapping_$MODEL.launch.py

# Terminal 2
ros2 bag play /path/to/output
```

### Path C: Live sensor

```bash
# Terminal 1: Hesai driver
cd ~/hesai_ros2_ws && source install/setup.bash
ros2 launch hesai_ros_driver start.launch.py

# Terminal 2: FAST-LIO2
cd <ws> && source install/setup.bash
ros2 launch fast_lio mapping_$MODEL.launch.py
```

## Step 4: Validate config and input

```bash
# Config (static)
CONFIG=config/$MODEL.yaml
[[ "$MODEL" != "mt60" ]] || CONFIG=config/MT60.yaml
python3 tools/check_config.py --config "$CONFIG" --model "$MODEL" --ros 2

# Input (runtime) — while bag is playing or driver is publishing
ros2 run fast_lio check_input.py --model $MODEL --timestamp-unit 0
```

Resolve any FAIL before running.

Keep `common.imu_gyr_unit: "auto"` for JT16, JT32, JT128, and MT60. FAST-LIO2
accepts either SI values (`rad/s`, `m/s²`) or SDK-scale values (`deg/s`, `g`),
detects the pair from startup acceleration, and converts angular velocity when
required. Use `"deg"` or `"rad"` only when the driver output is known and fixed.
For MT60, reject zero-stamped or all-zero IMU samples and load the matching
angle correction file in the Hesai driver before mapping.

## Step 5: View results in RViz

RViz opens with the launch file. Set **Fixed Frame** to `camera_init`,
subscribe to `/cloud_registered` and `/path`.

## Step 6: Save the PCD map (optional)

All models keep PCD buffering off by default. For the unified ROS 1/ROS 2
customer behavior, add `--save-map` to the one-command entry:

```bash
./tools/run_fastlio.sh jt128 live --save-map
./tools/run_fastlio.sh jt128 bag /data/input_bag \
  --save-map ~/slam_ws/src/slam_maps/customer_site.pcd
```

The script temporarily enables buffering, calls `/map_save`, and reports the
path. Without an explicit path it writes a timestamped file under
`~/slam_ws/src/slam_maps/`.

## Step 7: Analyze map quality (optional)

After mapping, assess the result and get problem-specific suggestions:

```bash
# Offline (recommended): analyze the saved PCD
python3 tools/check_map.py --pcd PCD/fast_lio2_jt_map.pcd

# Live: requires publish.map_en (JT16 default true; JT32/JT128 enable it)
ros2 run fast_lio check_map.py --map-topic /Laser_map --odom-topic /Odometry
```

It reports surface thickness (ghosting), density, and trajectory drift, and
maps issues to fixes (extrinsics / time sync / `imu_gyr_unit` /
`point_filter_num`). If it flags ghosting, re-run `check_input.py` and
`check_config.py` to find the root cause.

## Export and acceptance rules

Use `--save-map --map-mode all` to produce the voxel, original registered-scan,
and sensor-time 1 Hz variants from one run. Default export is the sensor-time 1 Hz accumulated map.
Select `--map-mode voxel` for a 0.1 m map.
Read `result.json` and `node_status.json`, `drain.json` and `map_save.json`; a process exit or a
nonempty PCD alone is not acceptance. Do not label `check_map.py` results as
geometric accuracy or use start/end Z alone as full-trajectory drift.
Clock rollback requires a fresh mapping run. See `tools/README.md` for the
checkpoint, failure and disk-space behavior.

For ROS 2 bags, the runner waits for matched publishers and configures Reliable
transport on both ends. Live sensor subscriptions retain Best Effort compatibility.
