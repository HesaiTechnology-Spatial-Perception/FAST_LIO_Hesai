# Release process

FAST_LIO_Hesai keeps ROS 1 and ROS 2 on independent branches. ROS package
versions remain aligned, while Git tags include the ROS generation so that a
tag identifies exactly one commit:

- `v0.1.0-ros1` for the `main` branch;
- `v0.1.0-ros2` for the `ROS2` branch.

## Before tagging

1. Confirm `package.xml` and `CHANGELOG.md` describe the intended version.
2. Build the target branch in its supported ROS environment and require the
   installed end-to-end CI job to pass. Recheck the pinned driver/SDK and model
   qualification scope in `DRIVER_COMPATIBILITY.md`.
3. Run `tools/check_config.py` for JT16, JT32, JT64P, JT128, and MT60.
   Confirm a fresh install contains only the five standard model YAMLs; no
   device- or recording-specific configuration should be shipped.
4. Replay representative LiDAR and IMU data, check odometry and registered
   point-cloud output, and test map saving when it changed.
5. Confirm the English and Chinese developer guides match the released
   behavior.
6. Review the exact branch status, commits, and working-tree diff before any
   push.

## Publish

After the reviewed commit is on the remote branch, create a signed annotated
tag and push that tag explicitly. Example for ROS 1 version 0.1.0:

```bash
git switch main
git pull --ff-only origin main
git tag -s v0.1.0-ros1 -m "FAST_LIO_Hesai ROS 1 v0.1.0"
git push origin v0.1.0-ros1
```

Use the equivalent `ROS2` branch and `v0.1.0-ros2` tag for ROS 2. Create one
GitHub Release per tag, copy the matching changelog section into the release
notes, and link the other ROS generation's release. Do not attach generated
maps, bags, logs, or local build artifacts.

## Local development before a release

Keep pending work on separate ROS 1 and ROS 2 development branches and record
pre-existing uncommitted changes before editing. Keep the package version at
its current value until selecting a release; describe pending work under
`[Unreleased]`. A local review/test run is not a release, tag, commit or push.
Include the run's effective YAML, branch/commit/dirty state and validation
results in the review. Do not publish private captures or generated maps.
