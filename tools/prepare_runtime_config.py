#!/usr/bin/env python3
# Copyright 2026 Hesai Technology. All rights reserved.
# SPDX-License-Identifier: GPL-2.0

"""Prepare a temporary FAST-LIO config without guessing gyro units from data."""

import argparse
import copy
import math
import sys

import yaml


UNITS = ("auto", "deg", "rad")


def prepare_config(config, ros, imu_unit=None, map_path=None, map_mode=None, leaf_size=None, run_id=None, no_preview=False, log_dir=None, reliable_input=False):
    """CLI overrides YAML; absent units retain the node's auto default."""
    result = copy.deepcopy(config)
    try:
        params = result if str(ros) == "1" else result["/**"]["ros__parameters"]
        common = params.setdefault("common", {})
        unit = imu_unit if imu_unit is not None else common.get("imu_gyr_unit", "auto")
        if unit not in UNITS:
            raise ValueError("common.imu_gyr_unit must be auto, deg, or rad")
        common["imu_gyr_unit"] = unit
        if run_id:
            common["run_id"] = run_id
        if reliable_input and str(ros) == "2":
            common["reliable_input"] = True
        if log_dir:
            common["log_dir"] = log_dir
        if no_preview:
            params.setdefault("publish", {})["map_en"] = False
        if map_path:
            params["map_file_path"] = map_path
            pcd = params.setdefault("pcd_save", {})
            pcd["pcd_save_en"] = True
            pcd.setdefault("interval", -1)
            pcd.setdefault("leaf_size", 0.1)
            if map_mode is not None:
                pcd["mode"] = map_mode
            if leaf_size is not None:
                pcd["leaf_size"] = leaf_size
            mode = pcd.get("mode", "voxel" if pcd["leaf_size"] > 0 else "raw")
            if mode not in ("raw", "voxel", "1hz", "all"):
                raise ValueError("pcd_save.mode must be raw, voxel, 1hz, or all")
            leaf = pcd["leaf_size"]
            if isinstance(leaf, bool) or not isinstance(leaf, (int, float)) or not math.isfinite(leaf) or leaf < 0:
                raise ValueError("pcd_save.leaf_size must be finite and nonnegative")
            if mode in ("voxel", "all") and leaf <= 0:
                raise ValueError("Voxel export requires a positive leaf size")
            pcd["mode"] = mode
    except (KeyError, TypeError, AttributeError) as exc:
        raise ValueError("Expected a ROS 1 config or ROS 2 /**/ros__parameters mapping") from exc
    return result, unit


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source")
    parser.add_argument("target")
    parser.add_argument("--ros", choices=("1", "2"), required=True)
    parser.add_argument("--imu-gyr-unit", choices=UNITS)
    parser.add_argument("--save-map")
    parser.add_argument("--map-mode", choices=("raw", "voxel", "1hz", "all"))
    parser.add_argument("--map-leaf-size", type=float)
    parser.add_argument("--run-id")
    parser.add_argument("--log-dir")
    parser.add_argument("--reliable-input", action="store_true")
    parser.add_argument("--no-preview", action="store_true")
    args = parser.parse_args()
    try:
        with open(args.source, encoding="utf-8") as stream:
            config = yaml.safe_load(stream)
        config, unit = prepare_config(config, args.ros, args.imu_gyr_unit, args.save_map,
                                      args.map_mode, args.map_leaf_size, args.run_id, args.no_preview, args.log_dir, args.reliable_input)
        with open(args.target, "w", encoding="utf-8") as stream:
            yaml.safe_dump(config, stream, sort_keys=False, allow_unicode=True)
    except (OSError, ValueError, yaml.YAMLError) as exc:
        parser.exit(2, "[ERROR] Runtime configuration: {}\n".format(exc))
    origin = "CLI" if args.imu_gyr_unit is not None else "YAML/default"
    print("[INFO] config source: {}".format(args.source))
    print("[INFO] IMU gyro unit: {} ({})".format(unit, origin))
    if unit == "auto":
        print("[WARN] auto uses the node's multi-sample acceleration heuristic, not "
              "driver metadata. It cannot prove gyro units; confirm the driver's "
              "unit convention or pass --imu-gyr-unit rad|deg.", file=sys.stderr)
    else:
        print("[INFO] Expected input: {}; {}".format(
            unit + "/s", "convert once to rad/s" if unit == "deg" else "no gyro conversion"))


if __name__ == "__main__":
    main()
