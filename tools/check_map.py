#!/usr/bin/env python3
# Copyright 2026 Hesai Technology. All rights reserved.
# SPDX-License-Identifier: GPL-2.0
#
# This file is part of FAST_LIO_Hesai, a fork of FAST_LIO
# (https://github.com/hku-mars/FAST_LIO) by the MARS Lab, HKU.
# This script is an original contribution by Hesai Technology.
"""
check_map.py  —  Analyze FAST-LIO2 map quality and suggest likely problems

Two modes:
  1. Offline:  --pcd <file>           analyze a saved .pcd map
  2. Live:     --map-topic /Laser_map  analyze the published map (and trajectory
               via --odom-topic /Odometry)

Metrics:
  - Point count, bounding box, extent
  - Voxel occupancy / density
  - Local surface thickness (plane-fit residual) — high values indicate
    ghosting / double surfaces / misalignment
  - Trajectory length and Z drift (live mode, from odometry)

Each metric maps to a PASS / WARN / FAIL verdict and actionable suggestions.

Usage:
  python3 tools/check_map.py --pcd PCD/scans.pcd --model jt128
  ros2 run fast_lio check_map.py --map-topic /Laser_map --odom-topic /Odometry
  rosrun fast_lio check_map.py --map-topic /Laser_map --odom-topic /Odometry

Dependencies: numpy (required). open3d (optional) speeds up PCD loading; a
builtin PCD parser is used otherwise. Thickness analysis is numpy-only.
"""

import argparse
import math
import struct
import sys
import time

try:
    import numpy as np
except ImportError:
    sys.exit("[ERROR] numpy is required. Run: pip3 install numpy")

# optional accelerator for PCD I/O only (broad except: some installs raise
# ABI errors, not ImportError)
try:
    import open3d as o3d
    HAVE_O3D = True
except Exception:
    HAVE_O3D = False

PASS = "\033[92m[PASS]\033[0m"
FAIL = "\033[91m[FAIL]\033[0m"
WARN = "\033[93m[WARN]\033[0m"
INFO = "\033[94m[INFO]\033[0m"

# Heuristic thresholds (meters). Local surface thickness ~ sensor noise for a
# well-aligned map; large values indicate ghosting / misalignment.
THICK_GOOD = 0.05
THICK_WARN = 0.15
DENSITY_MIN_PTS = 50_000     # below this the map is likely too sparse / not converged
Z_DRIFT_WARN = 0.5           # meters of vertical drift over the trajectory


class Verdict:
    def __init__(self):
        self.fails = 0
        self.warns = 0

    def p(self, msg): print(f"{PASS} {msg}")
    def i(self, msg): print(f"{INFO} {msg}")
    def w(self, msg): print(f"{WARN} {msg}"); self.warns += 1
    def f(self, msg): print(f"{FAIL} {msg}"); self.fails += 1


# ── point cloud loading ────────────────────────────────────────────────────

def load_pcd(path):
    """Return Nx3 float array of xyz from a .pcd file."""
    if HAVE_O3D:
        pc = o3d.io.read_point_cloud(path)
        pts = np.asarray(pc.points, dtype=np.float64)
        if pts.size == 0:
            raise ValueError("empty point cloud")
        return pts
    return _load_pcd_minimal(path)


def _load_pcd_minimal(path):
    """Minimal PCD reader (ascii or binary, xyz fields) without open3d."""
    with open(path, "rb") as f:
        fields, sizes, types, counts = [], [], [], []
        npts = 0
        data_fmt = "ascii"
        while True:
            line = f.readline().decode("ascii", "replace").strip()
            if not line:
                continue
            key = line.split()[0].upper() if line.split() else ""
            if key == "FIELDS":
                fields = line.split()[1:]
            elif key == "SIZE":
                sizes = [int(x) for x in line.split()[1:]]
            elif key == "TYPE":
                types = line.split()[1:]
            elif key == "COUNT":
                counts = [int(x) for x in line.split()[1:]]
            elif key == "POINTS":
                npts = int(line.split()[1])
            elif key == "DATA":
                data_fmt = line.split()[1]
                break
        idx = {n: i for i, n in enumerate(fields)}
        for r in ("x", "y", "z"):
            if r not in idx:
                raise ValueError(f"PCD missing field '{r}'")
        if data_fmt == "ascii":
            arr = np.loadtxt(f, dtype=np.float64)
            if arr.ndim == 1:
                arr = arr.reshape(1, -1)
            return arr[:, [idx["x"], idx["y"], idx["z"]]]
        # binary
        counts = counts or [1] * len(fields)
        np_types = {("F", 4): "f4", ("F", 8): "f8", ("U", 1): "u1",
                    ("U", 2): "u2", ("U", 4): "u4", ("I", 1): "i1",
                    ("I", 2): "i2", ("I", 4): "i4"}
        dtype = np.dtype([(fields[i], np_types[(types[i], sizes[i])])
                          for i in range(len(fields))])
        raw = np.frombuffer(f.read(npts * dtype.itemsize), dtype=dtype)
        return np.stack([raw["x"], raw["y"], raw["z"]], axis=1).astype(np.float64)


def pc2_to_xyz(msg, max_points=400_000):
    """Extract Nx3 xyz from a PointCloud2 message."""
    off = {f.name: f.offset for f in msg.fields}
    for r in ("x", "y", "z"):
        if r not in off:
            raise ValueError("PointCloud2 missing xyz fields")
    step = msg.point_step
    n = msg.width * msg.height
    stride = max(1, n // max_points)
    data = bytes(msg.data)
    xs, ys, zs = [], [], []
    for i in range(0, n, stride):
        base = i * step
        xs.append(struct.unpack_from("<f", data, base + off["x"])[0])
        ys.append(struct.unpack_from("<f", data, base + off["y"])[0])
        zs.append(struct.unpack_from("<f", data, base + off["z"])[0])
    pts = np.array([xs, ys, zs], dtype=np.float64).T
    return pts[np.isfinite(pts).all(axis=1)]


# ── metrics ─────────────────────────────────────────────────────────────────

def analyze_geometry(pts, v: Verdict):
    n = len(pts)
    bbox = pts.max(axis=0) - pts.min(axis=0)
    v.i(f"points: {n:,}  |  extent (m): "
        f"{bbox[0]:.1f} x {bbox[1]:.1f} x {bbox[2]:.1f}")

    if n < DENSITY_MIN_PTS:
        v.w(f"only {n:,} points — map may be too sparse or odometry did not "
            f"converge. Check for frame drops / large point_filter_num.")

    # local surface thickness via per-voxel plane fit (numpy-only)
    thick = _local_thickness(pts)
    if thick is None or len(thick) == 0:
        v.i("thickness analysis skipped (not enough dense surfaces sampled)")
        return
    med, p90 = float(np.median(thick)), float(np.percentile(thick, 90))
    if med > THICK_WARN:
        v.f(f"surface thickness: median {med*100:.1f} cm (p90 {p90*100:.1f} cm) "
            f"— strong ghosting / misalignment")
    elif med > THICK_GOOD:
        v.w(f"surface thickness: median {med*100:.1f} cm (p90 {p90*100:.1f} cm) "
            f"— some smearing; verify extrinsics / time sync / imu_gyr_unit")
    else:
        v.p(f"surface thickness: median {med*100:.1f} cm (p90 {p90*100:.1f} cm) "
            f"— surfaces are crisp")


def _local_thickness(pts, voxel=0.3, min_pts=10, max_points=1_500_000):
    """Per-voxel plane-fit residual (meters), numpy-only.

    Voxelize the cloud; within each voxel that has enough points, the residual
    along the smallest PCA axis approximates local surface thickness. Robust
    and dependency-free (no KDTree needed).
    """
    n = len(pts)
    if n < min_pts:
        return None
    if n > max_points:                       # subsample huge clouds for speed
        idx = np.random.default_rng(0).choice(n, size=max_points, replace=False)
        pts = pts[idx]

    keys = np.floor(pts / voxel).astype(np.int64)
    _, inv = np.unique(keys, axis=0, return_inverse=True)
    order = np.argsort(inv, kind="stable")
    inv_s, pts_s = inv[order], pts[order]
    splits = np.flatnonzero(np.diff(inv_s)) + 1
    res = [_plane_residual(g) for g in np.split(pts_s, splits) if len(g) >= min_pts]
    return np.array(res) if res else None


def _plane_residual(neighbors):
    """RMS distance of neighbors to their best-fit plane (smallest PCA axis)."""
    c = neighbors - neighbors.mean(axis=0)
    # smallest singular value / sqrt(N) ≈ RMS residual along the normal
    s = np.linalg.svd(c, compute_uv=False)
    return s[-1] / math.sqrt(len(neighbors))


def analyze_trajectory(positions, v: Verdict):
    if len(positions) < 2:
        v.i("trajectory: too few odometry samples to assess")
        return
    p = np.array(positions)
    seg = np.linalg.norm(np.diff(p, axis=0), axis=1)
    length = float(seg.sum())
    z_drift = float(abs(p[-1, 2] - p[0, 2]))
    v.i(f"trajectory: length {length:.1f} m, {len(p)} poses")
    if z_drift > Z_DRIFT_WARN:
        v.w(f"vertical drift {z_drift:.2f} m start→end — if the path was level, "
            f"this indicates Z drift (insufficient init motion / degeneracy)")
    else:
        v.p(f"vertical drift {z_drift:.2f} m — within expected range")


# ── runners ──────────────────────────────────────────────────────────────────

def run_offline(path, v: Verdict):
    print(f"{INFO} loading {path} ...")
    pts = load_pcd(path)
    analyze_geometry(pts, v)


def run_live(map_topic, odom_topic, timeout, v: Verdict):
    try:
        import rclpy
        from sensor_msgs.msg import PointCloud2
        from nav_msgs.msg import Odometry
        ros = 2
    except ImportError:
        try:
            import rospy
            from sensor_msgs.msg import PointCloud2
            from nav_msgs.msg import Odometry
            ros = 1
        except ImportError:
            sys.exit("[ERROR] live mode needs ROS. Source your workspace, or "
                     "use --pcd for offline analysis.")

    got = {"map": None, "odom": []}

    def map_cb(m): got["map"] = got["map"] or m
    def odom_cb(m):
        p = m.pose.pose.position
        got["odom"].append((p.x, p.y, p.z))

    if ros == 2:
        rclpy.init()
        node = rclpy.create_node("fast_lio_check_map")
        node.create_subscription(PointCloud2, map_topic, map_cb, 1)
        if odom_topic:
            node.create_subscription(Odometry, odom_topic, odom_cb, 50)
        deadline = time.time() + timeout
        while time.time() < deadline and got["map"] is None:
            rclpy.spin_once(node, timeout_sec=0.1)
        # collect a bit of odometry
        t2 = time.time() + min(3.0, timeout)
        while time.time() < t2:
            rclpy.spin_once(node, timeout_sec=0.1)
        node.destroy_node(); rclpy.shutdown()
    else:
        rospy.init_node("fast_lio_check_map", anonymous=True)
        rospy.Subscriber(map_topic, PointCloud2, map_cb)
        if odom_topic:
            rospy.Subscriber(odom_topic, Odometry, odom_cb)
        deadline = time.time() + timeout
        while time.time() < deadline and got["map"] is None and not rospy.is_shutdown():
            time.sleep(0.1)
        time.sleep(min(3.0, timeout))

    if got["map"] is None:
        v.f(f"no map received on '{map_topic}' within {timeout}s. On ROS 1 / "
            f"JT128 the map is not published by default — enable publish.map_en "
            f"(ROS 2) or analyze a saved PCD with --pcd.")
    else:
        analyze_geometry(pc2_to_xyz(got["map"]), v)
    if odom_topic:
        analyze_trajectory(got["odom"], v)


def main():
    ap = argparse.ArgumentParser(
        description="Analyze FAST-LIO2 map quality and suggest likely problems")
    ap.add_argument("--pcd", help="Offline: path to a saved .pcd map")
    ap.add_argument("--map-topic", default="/Laser_map",
                    help="Live: accumulated map topic (default: /Laser_map)")
    ap.add_argument("--odom-topic", default="/Odometry",
                    help="Live: odometry topic for trajectory (default: /Odometry)")
    ap.add_argument("--timeout", type=float, default=10.0,
                    help="Live: seconds to wait for the map (default: 10)")
    ap.add_argument("--model", default="auto", help="Informational only")
    args, _ = ap.parse_known_args()

    print(f"\n{'='*62}")
    print(f"  FAST-LIO2 Map Quality Checker"
          f"  |  {'offline' if args.pcd else 'live'}")
    print(f"  pcd I/O: {'open3d' if HAVE_O3D else 'builtin parser'}")
    print(f"{'='*62}\n")

    v = Verdict()
    if args.pcd:
        run_offline(args.pcd, v)
    else:
        run_live(args.map_topic, args.odom_topic, args.timeout, v)

    print()
    if v.fails:
        print(f"{FAIL} {v.fails} issue(s), {v.warns} warning(s). Likely fixes: "
              f"run check_input.py and check_config.py; verify LiDAR-IMU "
              f"extrinsics, time sync, and imu_gyr_unit.")
    elif v.warns:
        print(f"{WARN} 0 critical issues, {v.warns} warning(s). Map is usable; "
              f"review the notes above to improve quality.")
    else:
        print(f"{PASS} Map quality looks good.")
    print(f"\n{'='*62}\n")
    sys.exit(1 if v.fails else 0)


if __name__ == "__main__":
    main()
