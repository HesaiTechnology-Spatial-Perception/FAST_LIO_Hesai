# FAST_LIO_Hesai Agent Playbooks

Platform-neutral agent "skills" that teach an AI agent how to set up and run
FAST-LIO2 on Hesai JT16 / JT32 / JT64P / JT128 / MT60 LiDARs end to end.

These are **generic** Markdown playbooks — not tied to any single AI product.
Any agent (Claude, GPT, or a custom LLM pipeline) can use them: load
the relevant file as context / system prompt and follow the steps. Each file
has a small YAML header (`name`, `description`) for skill systems that read it,
and a plain-Markdown body that any model can follow directly.

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

## Files

| File | Target | Use |
| --- | --- | --- |
| `run_fastlio2_ros1.md` | ROS 1 Melodic / Noetic (`main` branch) | Build with `catkin_make`, launch with `roslaunch` |
| `run_fastlio2_ros2.md` | ROS 2 Humble (`ROS2` branch) | Build with `colcon`, launch with `ros2 launch` |

Pick the file matching your ROS version (or the checked-out branch).

## What the playbooks cover

A 5-step decision workflow:

1. Detect the local environment (ROS distro, build state, dependencies)
2. Configure and build the workspace if needed
3. Choose a data source:
   - a PCAP capture → convert to a rosbag
   - an existing rosbag → run directly
   - no data → bring up a live sensor
4. Validate config and input (`check_config.py`, `check_input.py`)
5. Run FAST-LIO2 and view the result in RViz

The playbooks reference the tools in this directory's parent (`tools/`):
`check_input.py`, `check_config.py`, and `pcap_to_rosbag/`.

## Using with a generic LLM

These files are plain instructions. To use outside an agent runtime, paste the
relevant playbook into the model's context as a system / developer prompt, then
ask the model to execute the steps in your shell.
