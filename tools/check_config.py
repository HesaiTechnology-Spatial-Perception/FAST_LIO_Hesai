#!/usr/bin/env python3
# Copyright 2026 Hesai Technology. All rights reserved.
# SPDX-License-Identifier: GPL-2.0
#
# This file is part of FAST_LIO_Hesai, a fork of FAST_LIO
# (https://github.com/hku-mars/FAST_LIO) by the MARS Lab, HKU.
# This script is an original contribution by Hesai Technology.
"""
check_config.py  —  Statically validate a FAST-LIO2 (Hesai JT/MT60) yaml config

Does NOT require ROS to be running. Parses the yaml and checks parameter
consistency against the declared LiDAR model and ROS version, catching the
most common misconfigurations before launch.

Checks:
  - preprocess.lidar_type matches model + ROS version
  - preprocess.scan_line matches model
  - preprocess.timestamp_unit is a valid enum (0-3)
  - common.imu_gyr_unit is "auto", "deg", or "rad"
  - mapping.imu_init_delay and imu_init_duration are non-negative
  - mapping.imu_init_max_gyr_norm is non-negative when provided
  - mapping.imu_init_max_gap is positive when provided
  - preprocess.blind is positive and below mapping.det_range
  - mapping.extrinsic_R is a valid rotation matrix (orthonormal, det ~ 1)
  - mapping.extrinsic_T has 3 elements
  - pcd_save.pcd_save_en consistency with map_file_path writability

Usage:
  python3 tools/check_config.py --config config/MT60.yaml --model mt60 --ros 2
  python3 tools/check_config.py --config config/jt16.yaml  --model jt16  --ros 1
"""

import argparse
import math
import os
import sys

try:
    import yaml
except ImportError:
    sys.exit("[ERROR] PyYAML not found. Run: pip3 install pyyaml")

PASS = "\033[92m[PASS]\033[0m"
FAIL = "\033[91m[FAIL]\033[0m"
WARN = "\033[93m[WARN]\033[0m"
INFO = "\033[94m[INFO]\033[0m"

# expected per-model values
MODEL_SPECS = {
    "jt16":  {"scan_line": 16,  "lidar_type": {1: 5, 2: 1}},
    "jt32":  {"scan_line": 32,  "lidar_type": {1: 7, 2: 3}},
    "jt128": {"scan_line": 128, "lidar_type": {1: 6, 2: 2}},
    "mt60":  {"scan_line": 2,   "lidar_type": {1: 8, 2: 4}},
    # JT64P publishes the same 128-ring PointCloud2 layout as JT128. Keep a
    # distinct lidar_type only to preserve the model-specific launch/config path.
    "jt64p": {"scan_line": 128, "lidar_type": {1: 9, 2: 5}},
}


class ConfigChecker:
    def __init__(self, config_path, model, ros_ver):
        self.path     = config_path
        self.model    = model
        self.ros      = ros_ver
        self.spec     = MODEL_SPECS[model]
        self.failures = 0
        self.warnings = 0

    def _pass(self, msg): print(f"{PASS} {msg}")
    def _info(self, msg): print(f"{INFO} {msg}")
    def _fail(self, msg): print(f"{FAIL} {msg}"); self.failures += 1
    def _warn(self, msg): print(f"{WARN} {msg}"); self.warnings += 1

    # ── load + normalize ──────────────────────────────────────────────────
    def load(self):
        with open(self.path) as f:
            raw = yaml.safe_load(f)
        if raw is None:
            self._fail("Empty or invalid yaml file")
            return None
        # ROS 2 nests parameters under '/**': {'ros__parameters': {...}}
        if "/**" in raw and "ros__parameters" in raw["/**"]:
            self._info("Detected ROS 2 parameter layout (/**: ros__parameters)")
            return raw["/**"]["ros__parameters"]
        # ROS 1 uses a flat top-level layout
        self._info("Detected ROS 1 / flat parameter layout")
        return raw

    @staticmethod
    def _get(d, dotted, default=None):
        cur = d
        for k in dotted.split("."):
            if not isinstance(cur, dict) or k not in cur:
                return default
            cur = cur[k]
        return cur

    # ── checks ────────────────────────────────────────────────────────────
    def check(self, cfg):
        self._check_lidar_type(cfg)
        self._check_scan_line(cfg)
        self._check_timestamp_unit(cfg)
        self._check_point_filter(cfg)
        self._check_gyr_unit(cfg)
        self._check_imu_init_window(cfg)
        self._check_blind(cfg)
        self._check_extrinsic(cfg)
        self._check_pcd_save(cfg)

    def _check_lidar_type(self, cfg):
        val = self._get(cfg, "preprocess.lidar_type")
        expected = self.spec["lidar_type"][self.ros]
        if val is None:
            self._fail("preprocess.lidar_type is missing")
        elif not isinstance(val, int) or isinstance(val, bool) or val != expected:
            self._fail(f"preprocess.lidar_type={val}, expected {expected} "
                       f"for {self.model.upper()} on ROS {self.ros}")
        else:
            self._pass(f"preprocess.lidar_type={val} matches "
                       f"{self.model.upper()} (ROS {self.ros})")

    def _check_scan_line(self, cfg):
        val = self._get(cfg, "preprocess.scan_line")
        exp = self.spec["scan_line"]
        if val is None:
            self._fail("preprocess.scan_line is missing")
        elif not isinstance(val, int) or isinstance(val, bool) or val != exp:
            self._fail(f"preprocess.scan_line={val}, expected {exp} "
                       f"for {self.model.upper()}")
        else:
            self._pass(f"preprocess.scan_line={val} matches {self.model.upper()}")

    def _check_timestamp_unit(self, cfg):
        val = self._get(cfg, "preprocess.timestamp_unit")
        if val is None:
            self._warn("preprocess.timestamp_unit missing (defaults may apply)")
        elif not isinstance(val, int) or isinstance(val, bool) or val not in (0, 1, 2, 3):
            self._fail(f"preprocess.timestamp_unit={val} invalid "
                       f"(must be 0=s,1=ms,2=us,3=ns)")
        else:
            unit = {0: "s", 1: "ms", 2: "us", 3: "ns"}[val]
            self._pass(f"preprocess.timestamp_unit={val} ({unit}) valid")

    def _check_point_filter(self, cfg):
        val = self._get(cfg, "point_filter_num", 2)
        if not isinstance(val, int) or isinstance(val, bool) or val < 1:
            self._fail(f"point_filter_num={val} must be a positive integer")
        else:
            self._pass(f"point_filter_num={val} valid")

    def _check_gyr_unit(self, cfg):
        gap = self._get(cfg, "common.imu_max_gap", 0.05)
        if isinstance(gap, bool) or not isinstance(gap, (int, float)) or not math.isfinite(gap) or gap <= 0:
            self._fail("common.imu_max_gap must be finite and positive (seconds)")
        else:
            self._pass(f"Runtime IMU maximum gap: {gap} s")
        val = self._get(cfg, "common.imu_gyr_unit")
        if val is None:
            self._warn("common.imu_gyr_unit missing")
        elif val not in ("auto", "deg", "rad"):
            self._fail(f"common.imu_gyr_unit='{val}' invalid "
                       f"(must be 'auto', 'deg', or 'rad')")
        else:
            self._pass(f"common.imu_gyr_unit='{val}' valid")

    def _check_imu_init_window(self, cfg):
        delay = self._get(cfg, "mapping.imu_init_delay", 0.0)
        duration = self._get(cfg, "mapping.imu_init_duration", 0.0)
        limit = self._get(cfg, "mapping.imu_init_max_gyr_norm", 0.0)
        max_gap = self._get(cfg, "mapping.imu_init_max_gap", 0.05)
        if isinstance(delay, bool) or not isinstance(delay, (int, float)) or not math.isfinite(delay) or delay < 0:
            self._fail(f"mapping.imu_init_delay={delay} must be non-negative")
        elif (isinstance(duration, bool) or not isinstance(duration, (int, float)) or
              not math.isfinite(duration) or duration < 0):
            self._fail(f"mapping.imu_init_duration={duration} must be non-negative")
        elif isinstance(limit, bool) or not isinstance(limit, (int, float)) or not math.isfinite(limit) or limit < 0:
            self._fail("mapping.imu_init_max_gyr_norm must be finite and non-negative (rad/s)")
        elif (isinstance(max_gap, bool) or not isinstance(max_gap, (int, float)) or
              not math.isfinite(max_gap) or max_gap <= 0):
            self._fail(f"mapping.imu_init_max_gap={max_gap} must be positive")
        elif limit > 0 and duration < 0.1:
            self._fail("IMU initialization guard requires imu_init_duration >= 0.1 s")
        else:
            self._pass(
                f"IMU initialization window delay={delay} s, duration={duration} s, "
                f"max_gap={max_gap} s valid")
            if limit > 0:
                self._info(f"IMU initialization guard enabled: gyro limit={limit} rad/s")

    def _check_blind(self, cfg):
        blind = self._get(cfg, "preprocess.blind")
        det   = self._get(cfg, "mapping.det_range")
        if blind is None:
            self._warn("preprocess.blind missing")
            return
        if isinstance(blind, bool) or not isinstance(blind, (int, float)) or not math.isfinite(blind):
            self._fail(f"preprocess.blind={blind} must be a finite number")
            return
        if det is not None and (isinstance(det, bool) or not isinstance(det, (int, float)) or
                                not math.isfinite(det) or det <= 0):
            self._fail(f"mapping.det_range={det} must be a finite positive number")
            return
        if blind < 0:
            self._fail(f"preprocess.blind={blind} is negative")
        elif det is not None and blind >= det:
            self._fail(f"preprocess.blind={blind} ≥ mapping.det_range={det} "
                       f"— all points would be filtered out")
        else:
            self._pass(f"preprocess.blind={blind} m is reasonable")

    def _check_extrinsic(self, cfg):
        T = self._get(cfg, "mapping.extrinsic_T")
        R = self._get(cfg, "mapping.extrinsic_R")
        if T is None or R is None:
            self._fail("mapping.extrinsic_T / extrinsic_R missing")
            return
        if not isinstance(T, list) or len(T) != 3:
            self._fail(f"mapping.extrinsic_T must have 3 elements, got {T}")
            return
        if not isinstance(R, list) or len(R) != 9:
            self._fail(f"mapping.extrinsic_R must have 9 elements, got {R}")
            return
        values = T + R
        if any(isinstance(value, bool) or not isinstance(value, (int, float)) or
               not math.isfinite(value) for value in values):
            self._fail("mapping extrinsics must contain only finite numeric values")
            return
        rows = [R[0:3], R[3:6], R[6:9]]
        if any(abs(sum(value * value for value in row) - 1.0) > 1e-3
               for row in rows):
            self._fail("mapping.extrinsic_R rows must be unit length")
            return
        if any(abs(sum(rows[i][k] * rows[j][k] for k in range(3))) > 1e-3
               for i in range(3) for j in range(i + 1, 3)):
            self._fail("mapping.extrinsic_R rows must be orthogonal")
            return
        det = (R[0]*(R[4]*R[8] - R[5]*R[7])
               - R[1]*(R[3]*R[8] - R[5]*R[6])
               + R[2]*(R[3]*R[7] - R[4]*R[6]))
        if abs(det - 1.0) > 1e-3:
            self._fail(f"mapping.extrinsic_R determinant={det:.4f} (expected ~1.0) "
                       f"— not a valid rotation matrix")
        else:
            self._pass(f"mapping.extrinsic_R valid rotation (det={det:.4f})")
        est = self._get(cfg, "mapping.extrinsic_est_en")
        if est is True:
            self._info("mapping.extrinsic_est_en=true — online extrinsic estimation on; "
                       "set false if you already have accurate extrinsics")

    def _check_pcd_save(self, cfg):
        leaf = self._get(cfg, "pcd_save.leaf_size", 0.0)
        mode = self._get(cfg, "pcd_save.mode", "voxel" if isinstance(leaf, (int, float)) and leaf > 0 else "raw")
        if mode not in ("raw", "voxel", "1hz", "all"):
            self._fail("pcd_save.mode must be raw, voxel, 1hz, or all")
        if isinstance(leaf, bool) or not isinstance(leaf, (int, float)) or not math.isfinite(leaf) or leaf < 0:
            self._fail("pcd_save.leaf_size must be finite and nonnegative")
        elif mode in ("voxel", "all") and leaf <= 0:
            self._fail("Voxel export requires a positive pcd_save.leaf_size")
        interval = self._get(cfg, "pcd_save.interval", -1)
        if isinstance(interval, bool) or not isinstance(interval, int) or (interval != -1 and interval <= 0):
            self._fail("pcd_save.interval must be -1 or a positive number of registered frames")
        en = self._get(cfg, "pcd_save.pcd_save_en")
        if en is not None and not isinstance(en, bool):
            self._fail(f"pcd_save.pcd_save_en={en} must be true or false")
            return
        if not en:
            self._info("pcd_save.pcd_save_en=false — PCD will not be saved")
            return
        path = self._get(cfg, "map_file_path", "")
        if not isinstance(path, str):
            self._fail("map_file_path must be a string")
            return
        if not path:
            self._info("pcd_save enabled; map_file_path unset → defaults to a new ~/slam_ws/src/slam_maps/fastlio_<run>/map.pcd")
            return
        # relative paths are resolved against the process working directory;
        # we can only sanity-check absolute paths here.
        if os.path.isabs(path):
            parent = os.path.dirname(path)
            if parent and not os.path.isdir(parent):
                self._warn(f"map_file_path directory does not exist: {parent}")
            else:
                self._pass(f"pcd_save enabled → {path}")
        else:
            self._info(f"pcd_save enabled; map_file_path='{path}' "
                       f"(relative to the process working directory)")

    # ── summary ─────────────────────────────────────────────────────────
    def summary(self):
        print()
        if self.failures:
            print(f"{FAIL} {self.failures} error(s), {self.warnings} warning(s). "
                  f"Fix errors before launching FAST-LIO2.")
            return 1
        if self.warnings:
            print(f"{WARN} 0 errors, {self.warnings} warning(s). Config is usable.")
            return 0
        print(f"{PASS} Config looks good — no issues found.")
        return 0


def main():
    parser = argparse.ArgumentParser(
        description="Statically validate a FAST-LIO2 (Hesai JT/MT60) yaml config")
    parser.add_argument("--config", required=True, help="Path to the yaml config")
    parser.add_argument("--model",  required=True, choices=list(MODEL_SPECS.keys()),
                        help="LiDAR model: jt16, jt32, jt64p, jt128, or mt60")
    parser.add_argument("--ros",    type=int, required=True, choices=[1, 2],
                        help="ROS version: 1 or 2 (lidar_type enum differs)")
    args = parser.parse_args()

    if not os.path.isfile(args.config):
        sys.exit(f"[ERROR] Config file not found: {args.config}")

    print(f"\n{'='*62}")
    print(f"  FAST-LIO2 Config Checker  |  {args.model.upper()}  |  ROS {args.ros}")
    print(f"  config: {args.config}")
    print(f"{'='*62}\n")

    checker = ConfigChecker(args.config, args.model, args.ros)
    cfg = checker.load()
    if cfg is None:
        sys.exit(1)
    checker.check(cfg)
    rc = checker.summary()
    print(f"\n{'='*62}\n")
    sys.exit(rc)


if __name__ == "__main__":
    main()
