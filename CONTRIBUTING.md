# Contributing to FAST_LIO_Hesai

Thank you for helping improve FAST_LIO_Hesai.

## Choose the correct branch

- `main`: ROS 1 Melodic/Noetic
- `ROS2`: ROS 2 Humble

Open the pull request against the branch that you built and tested. If a
shared behavior changes, state whether the equivalent change is also required
on the other branch.

## Before opening a pull request

1. Keep the change focused and avoid generated maps, bags, logs, build output,
   or machine-specific paths.
2. Preserve upstream copyright and license notices.
3. Update the matching model YAML and launch file when adding parameters or
   changing defaults.
4. Update `README.md`, `tools/README.md`, and the English and Chinese developer
   guides when user-visible behavior changes.

Keep `config/` limited to standard model baselines. Store device- and
recording-specific overrides outside the checkout and use `--config PATH`.
Do not add capture-specific clock offsets, startup delays, or scene profiles
to the public model defaults. Update the explicit CMake install list when
adding a supported model.

Run the checks that apply to your change:

```bash
# Configuration and helper-tool checks
python3 tools/check_config.py --config config/jt128.yaml --model jt128 --ros 2
for script in tools/run_fastlio.sh tools/pcap_to_rosbag/*.sh; do bash -n "$script" || exit 1; done

# ROS 2
source /opt/ros/humble/setup.bash
colcon build --packages-select fast_lio \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

# ROS 1, from the catkin workspace root
source /opt/ros/noetic/setup.bash
catkin_make -DCMAKE_BUILD_TYPE=Release
```

For runtime changes, replay a representative bag or use a live sensor. Report
the LiDAR model, driver/SDK version, input rate, relevant YAML, and whether
`/Odometry`, `/cloud_registered`, RViz, and PCD saving were verified.

## Pull request description

Explain the problem, the behavior change, compatibility impact, and exact
verification performed. Attach text logs or a small screenshot only when they
clarify runtime behavior. Do not attach customer data or large bag/PCD files.

## Integration checks

Run `python3 tools/check_public_catalog.py --root . --ros 2` before building.
After building, copy the install tree to a separate workspace with no `devel/`,
source that install space, and run:

```bash
python3 tools/test_install_ros.py --ros 2 --workspace /path/to/test_ws --output /tmp/fastlio-install-test
```

Use a fresh output directory and isolated ROS2 domain. The ROS1 integration test
creates its own private local master and synthetic driver. CI then runs all cases
in `tools/test_runtime_ros.py` against the installed executable; see the workflow
for the environment and master setup. Driver changes must update
`DRIVER_COMPATIBILITY.md` with exact source revisions and the evidence scope.
