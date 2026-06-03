---
name: run-fastlio2-hesai-ros1
description: >-
  Set up and run FAST-LIO2 on Hesai JT16 / JT128 LiDARs on ROS 1 Melodic /
  Noetic (FAST_LIO_Hesai main branch). Detects and configures the local ROS 1
  environment, then runs FAST-LIO2 from a PCAP capture, an existing rosbag, or
  a live sensor. Use when building, configuring, validating, or running
  FAST-LIO2 mapping with a Hesai JT LiDAR on ROS 1, converting a PCAP to a bag,
  or bringing up the lidar live.
---

# Run FAST-LIO2 on Hesai JT — ROS 1

Target: **ROS 1 Melodic / Noetic**. Build with `catkin_make`, launch with
`roslaunch`. Package name: `fast_lio`.

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

With the sensor publishing or a rosbag playing (see Step 3), run:

```bash
rosrun fast_lio check_input.py          # --model defaults to "auto"
```

It prints e.g. `auto-detected model: JT16 (ring max=15)`. Record the detected
model as `MODEL` (`jt16` or `jt128`) and substitute it in every command below.

| Model | Lines | Config | Launch file | `lidar_type` (ROS 1) |
| --- | --- | --- | --- | --- |
| JT16 | 16 | `config/jt16.yaml` | `mapping_jt16.launch` | 5 |
| JT128 | 128 | `config/jt128.yaml` | `mapping_jt128.launch` | 6 |

> If no data is flowing yet, first bring up the source (Step 3 Path C for a live
> sensor, or play a bag), then run the detection above.

## Step 1: Detect environment

```bash
echo "ROS_DISTRO=$ROS_DISTRO"                 # expect: noetic or melodic
rosversion -d 2>/dev/null || echo "ROS 1 not on PATH — source /opt/ros/<distro>/setup.bash"
ls devel/ 2>/dev/null && echo "workspace built" || echo "workspace NOT built"
python3 -c "import yaml" 2>/dev/null && echo "PyYAML ok" || echo "PyYAML missing"
```

Decision:
- ROS not found → `source /opt/ros/noetic/setup.bash` (or `melodic`).
- workspace not built → Step 2.
- workspace built → `source devel/setup.bash`, then Step 3.

## Step 2: Configure & build

Replace `noetic` with `melodic` if needed:

```bash
sudo apt update
sudo apt install -y ros-noetic-pcl-ros ros-noetic-eigen-conversions \
  libeigen3-dev libpcl-dev
pip3 install pyyaml

git submodule update --init --recursive

cd <catkin_ws_root>
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

`livox_ros_driver` is **not required** for Hesai JT.

## Step 3: Pick data source

| Data the user has | Path |
| --- | --- |
| A `.pcap` capture (e.g. PandarView) | A |
| An existing `.bag` file | B |
| Nothing — live LiDAR | C |

### Path A: PCAP → rosbag

```bash
# Replace $MODEL with jt16 or jt128
bash tools/pcap_to_rosbag/pcap_to_rosbag_ros1.sh \
  --model      $MODEL \
  --pcap       /path/to/input.pcap \
  --correction /path/to/correction.csv \
  --firetime   /path/to/firetime.csv \
  --output     /path/to/output.bag \
  --driver-ws  ~/hesai_ros_ws
```

Output is a `.bag` file. Continue with Path B.

### Path B: Run from a rosbag

```bash
# Terminal 1
source devel/setup.bash
roslaunch fast_lio mapping_$MODEL.launch

# Terminal 2
rosbag play /path/to/output.bag
```

### Path C: Live sensor

```bash
# Terminal 1: Hesai driver
cd ~/hesai_ros_ws && source devel/setup.bash
roslaunch hesai_ros_driver start.launch

# Terminal 2: FAST-LIO2
cd <catkin_ws_root> && source devel/setup.bash
roslaunch fast_lio mapping_$MODEL.launch
```

## Step 4: Validate config and input

```bash
# Config (static)
python3 tools/check_config.py --config config/$MODEL.yaml --model $MODEL --ros 1

# Input (runtime) — while bag is playing or driver is publishing
rosrun fast_lio check_input.py --model $MODEL --timestamp-unit 0
```

Resolve any FAIL before running.

## Step 5: View results in RViz

RViz opens with the launch file. Set **Fixed Frame** to `camera_init`,
subscribe to `/cloud_registered` and `/path`.

On ROS 1, `/Laser_map` is not published by default (`publish_map` is commented
out in `src/laserMapping.cpp`). To view the full accumulated map, save a PCD
(Step 6) or uncomment `publish_map` and rebuild.

## Step 6: Save the PCD map (optional)

Both models default `pcd_save_en: false` on ROS 1. Enable saving in
`config/$MODEL.yaml`:

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
rosservice call /map_save "{}"
```

`pcd_save_en: true` is required for `/map_save` to work. The file goes to
`map_file_path` (relative paths resolve under the package root).

## Step 7: Analyze map quality (optional)

After mapping, assess the result and get problem-specific suggestions:

```bash
# Offline (recommended): analyze the saved PCD
python3 tools/check_map.py --pcd PCD/fast_lio2_jt_map.pcd

# Live: on ROS 1 /Laser_map is not published by default — prefer offline PCD
rosrun fast_lio check_map.py --map-topic /Laser_map --odom-topic /Odometry
```

It reports surface thickness (ghosting), density, and trajectory drift, and
maps issues to fixes (extrinsics / time sync / `imu_gyr_unit` /
`point_filter_num`). If it flags ghosting, re-run `check_input.py` and
`check_config.py` to find the root cause.
