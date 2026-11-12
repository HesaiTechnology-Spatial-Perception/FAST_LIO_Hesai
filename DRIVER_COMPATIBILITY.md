# Driver compatibility and validation scope

Reviewed on 2026-11-12. This is the support contract for both `main` (ROS 1)
and `ROS2` (ROS 2). A model YAML means a mapper input path exists; it does not
certify every firmware, driver build, or physical sensor of that model.

## Reproducible public driver baseline

- Repository: [HesaiLidar_ROS_2.0](https://github.com/HesaiTechnology/HesaiLidar_ROS_2.0)
- Driver package version: **1.5.0**
- Driver commit: **`3d54078a3d770004ef30ec2cacc26673f4172b22`**
- SDK submodule commit: **`357ff14c0b4486d21ac0de0b218e34546276cacb`**
- CPU path; GPU decoding is outside this project's validation scope.
- FAST_LIO_Hesai CI environments: Ubuntu 20.04 / ROS Noetic and Ubuntu 22.04 /
  ROS 2 Humble. Other ROS distributions are not part of the current CI contract.

Fetch the exact pair, rather than using a moving `master` as a release dependency:

```bash
mkdir -p ~/hesai_driver_ws/src
cd ~/hesai_driver_ws/src
git clone https://github.com/HesaiTechnology/HesaiLidar_ROS_2.0.git
cd HesaiLidar_ROS_2.0
git checkout --detach 3d54078a3d770004ef30ec2cacc26673f4172b22
git submodule update --init --recursive
git rev-parse HEAD
git -C src/driver/HesaiLidar_SDK_2.0 rev-parse HEAD
cd ../..
# Source /opt/ros/noetic/setup.bash OR /opt/ros/humble/setup.bash first.
rosdep install --from-paths src --ignore-src -r -y
# ROS 1:
catkin_make -DCMAKE_BUILD_TYPE=Release
# OR ROS 2:
colcon build --packages-select hesai_ros_driver --cmake-args -DCMAKE_BUILD_TYPE=Release
```

Use only the build command for your ROS generation. Keep calibration and network
settings in your own driver configuration. The mapper does not contain your
sensor's angle correction, firing-time correction, or measured extrinsics.

## Model and mode matrix

All five standard mapper configurations are installed on both ROS branches.
“Conditional” requires the message contract below and device-specific calibration.
“Prepared bag” means decoded PointCloud2 + Imu, not a bag of raw UDP packets.

| Model | Public baseline / protocol evidence | Live | Prepared bag | Unified PCAP conversion | Runtime evidence for the mapper |
| --- | --- | --- | --- | --- | --- |
| JT16 | SDK has UDP 1.8 decoder | Conditional on public baseline and valid IMU | Supported input path | Conditional; decoder present | JT16 recorded input passed Noetic and Humble; generated JT16-layout data runs in CI |
| JT32 | Pinned SDK has no UDP 1.12 decoder | Requires a separately obtained compatible driver | Supported input path | Requires a separately obtained compatible driver | A provided-driver 10-second PCAP conversion/replay passed Humble; this is not evidence for the public baseline |
| JT64P | Listed by upstream; 128-ring JT128-family input | Conditional; exact firmware/decoder combination must be checked | Supported input path | Conditional; device-matched 128-channel correction required | No hardware-qualified run in this release checklist |
| JT128 | SDK UDP 1.4 has JT128 dispatch | Conditional on public baseline and valid IMU | Supported input path | Conditional; decoder present | No hardware-qualified run in this release checklist |
| MT60 | No MT60-specific public driver baseline established | Requires a verified MT60-compatible publisher | Supported MT_V7 input path | Not registered in the unified converter | No hardware-qualified run in this release checklist |

No exact firmware versions have been qualified for release. Record firmware,
protocol and driver provenance with each hardware validation; do not infer a
firmware version from the model name. Previous recorded-data tests establish
mapper behavior, not the provenance of the driver that originally recorded a bag.
The provided JT32 driver snapshot has no independently reproducible public commit;
this project neither bundles it nor silently substitutes it for the public driver.
Users without that compatible driver should use a prepared, validated bag.

Sources for the public baseline: [driver README](https://github.com/HesaiTechnology/HesaiLidar_ROS_2.0/blob/3d54078a3d770004ef30ec2cacc26673f4172b22/README.md),
[SDK protocol dispatch](https://github.com/HesaiTechnology/HesaiLidar_SDK_2.0/blob/357ff14c0b4486d21ac0de0b218e34546276cacb/libhesai/UdpParser/udp_parser.h).

## Message and lifecycle contract

- Topics default to `/lidar_points` (`sensor_msgs/PointCloud2`) and `/lidar_imu`
  (`sensor_msgs/Imu`). Verify actual field types/rings with `tools/check_input.py`.
  JT64P and JT128 both use 128 rings; ring count cannot identify the model.
- JT inputs need `x/y/z/intensity` float32, `ring` uint16 and `timestamp` float64
  in seconds, plus valid header timestamps. Verify MT60 with `--model mt60`;
  it uses the two-ring MT_V7 model path with the same required field types.
- LiDAR/IMU headers must advance strictly, share a time basis and cover each scan.
  The whole bag is audited before mapping. The default permitted IMU gap is
  0.05 seconds. Driver point timestamps and headers must agree.
- IMU angular units must be established from the exact publisher/SDK contract.
  The pinned driver's ROS publishers copy SDK IMU numbers directly; publishing
  `sensor_msgs/Imu` alone does not prove rad/s or m/s². Verify stationary gravity
  and a known rotation, then use `--imu-gyr-unit rad` or `deg` explicitly.
  `auto` is a compatibility heuristic. Do not modify standard YAML to conceal
  a recording-specific time offset or unknown units.
- For PCAP conversion the driver must accept `config_path`, publish both topics,
  and exit successfully at EOF. The pinned public node implements `config_path`
  and the `IsPlayEnded()` loop. This is source-level interface evidence, not a
  complete physical-sensor PCAP qualification.
- **The pinned public driver does not implement `ros_timestamp_offset`.** Keep
  `--timestamp-offset` at zero with this baseline. A nonzero value requires an
  explicitly verified compatible driver that translates cloud/point/IMU times
  consistently. The CLI option does not add that capability to an arbitrary driver.
- Enable IMU output (`send_imu_ros`) and supply matching correction/firetime files
  as required by the sensor. Do not reuse calibration solely because the model matches.

## What CI proves

Both ROS jobs build and install the mapper, verify the standard config/launch
catalog, run the installed unified entry with a generated bag, validate complete
processing and default 1 Hz PCD saving, reject invalid timestamps, and propagate
a failed driver's exit code. Seven actual-node regressions additionally exercise
large frames, repeated save, rollback, duplicates, non-finite IMU, invalid schema,
IMU gaps and skipped-tail rejection (some share a single regression case).
ROS1 also tests cold master creation and preservation of an existing master during
conversion using a synthetic driver and the real ROS recorder. This fixture tests
process and recording behavior; it does **not** parse a real PCAP or qualify the
public driver, firmware, calibration or global map accuracy.

For a new driver/firmware release, record: exact driver and SDK commits (or source
hashes plus distribution location), dirty patches, OS/ROS/architecture, sensor
model/firmware/protocol, calibration hash, units/time source, input mode, complete
frame accounting, map-save result and the command used. Only then promote a
conditional row to a hardware-validated combination.
