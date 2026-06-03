---
name: run-fastlio2-hesai-ros2
description: >-
  Set up and run FAST-LIO2 on Hesai JT16 / JT128 LiDARs on ROS 2 Humble
  (FAST_LIO_Hesai ROS2 branch). Detects and configures the local ROS 2
  environment, then runs FAST-LIO2 from a PCAP capture, an existing rosbag, or
  a live sensor. Use when building, configuring, validating, or running
  FAST-LIO2 mapping with a Hesai JT LiDAR on ROS 2, converting a PCAP to a bag,
  or bringing up the lidar live.
---

# Run FAST-LIO2 on Hesai JT — ROS 2

Target: **ROS 2 Humble**. Build with `colcon`, launch with `ros2 launch`.
Package name: `fast_lio`. Run commands from the ROS 2 workspace root.

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

**Do not ask the user for the model — detect it automatically from the data.**
The model is determined by the point cloud `ring` field: max ring ≤ 15 → JT16,
otherwise → JT128.

With the sensor publishing or a rosbag2 playing (see Step 3), run:

```bash
ros2 run fast_lio check_input.py        # --model defaults to "auto"
```

It prints e.g. `auto-detected model: JT16 (ring max=15)`. Record the detected
model as `MODEL` (`jt16` or `jt128`) and substitute it in every command below.

| Model | Lines | Config | Launch file | `lidar_type` (ROS 2) |
| --- | --- | --- | --- | --- |
| JT16 | 16 | `config/jt16.yaml` | `mapping_jt16.launch.py` | 1 |
| JT128 | 128 | `config/jt128.yaml` | `mapping_jt128.launch.py` | 2 |

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
# Replace $MODEL with jt16 or jt128
bash tools/pcap_to_rosbag/pcap_to_rosbag_ros2.sh \
  --model      $MODEL \
  --pcap       /path/to/input.pcap \
  --correction /path/to/correction.csv \
  --firetime   /path/to/firetime.csv \
  --output     /path/to/output \
  --driver-ws  ~/hesai_ros2_ws
```

Output is a rosbag2 directory. Continue with Path B.

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
python3 tools/check_config.py --config config/$MODEL.yaml --model $MODEL --ros 2

# Input (runtime) — while bag is playing or driver is publishing
ros2 run fast_lio check_input.py --model $MODEL --timestamp-unit 0
```

Resolve any FAIL before running.

## Step 5: View results in RViz

RViz opens with the launch file. Set **Fixed Frame** to `camera_init`,
subscribe to `/cloud_registered` and `/path`.

Default behavior by model:
- **JT16**: `map_en`, `effect_map_en`, `pcd_save_en` all `true` by default (few points, lightweight).
- **JT128**: all three `false` by default (many points, reduces load). Enable in yaml when needed.

## Step 6: Save the PCD map (optional)

To persist the map as a `.pcd` file, enable saving in `config/$MODEL.yaml`
(JT16 has it on by default; JT128 is off):

```yaml
map_file_path: "PCD/fast_lio2_jt_map.pcd"   # optional; default PCD/scans.pcd
pcd_save:
  pcd_save_en: true
  interval: -1     # -1 = save once on exit; >0 = save every N frames
```

Two ways to trigger the save:
- **On exit**: stop the FAST-LIO2 node (`Ctrl+C`) — the map is written once.
- **On demand while running**: call the service:

```bash
ros2 service call /map_save std_srvs/srv/Trigger '{}'
```

`pcd_save_en: true` is required for `/map_save` to work. The file goes to
`map_file_path` (relative paths resolve under the package root).

## Step 7: Analyze map quality (optional)

After mapping, assess the result and get problem-specific suggestions:

```bash
# Offline (recommended): analyze the saved PCD
python3 tools/check_map.py --pcd PCD/fast_lio2_jt_map.pcd

# Live: requires publish.map_en (JT16 default true; JT128 enable it)
ros2 run fast_lio check_map.py --map-topic /Laser_map --odom-topic /Odometry
```

It reports surface thickness (ghosting), density, and trajectory drift, and
maps issues to fixes (extrinsics / time sync / `imu_gyr_unit` /
`point_filter_num`). If it flags ghosting, re-run `check_input.py` and
`check_config.py` to find the root cause.
