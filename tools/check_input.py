#!/usr/bin/env python3
# Copyright 2026 Hesai Technology. All rights reserved.
# SPDX-License-Identifier: GPL-2.0
#
# This file is part of FAST_LIO_Hesai, a fork of FAST_LIO
# (https://github.com/hku-mars/FAST_LIO) by the MARS Lab, HKU.
# This script is an original contribution by Hesai Technology.
"""
check_input.py  —  Validate LiDAR and IMU inputs before running FAST-LIO2 (Hesai JT)

Checks:
  1. /lidar_points topic exists
  2. /lidar_imu topic exists
  3. PointCloud2 contains required fields: x y z intensity ring timestamp
  4. 'timestamp' is per-point and monotonically increasing within a frame
  5. 'ring' range matches the declared model (JT16: 0–15, JT128: 0–127)
  6. IMU publish frequency
  7. Gyro magnitude — coarse deg/s vs rad/s sanity check
  8. frame_id of both sensors (for coordinate frame awareness)

Usage (ROS 2):
    ros2 run fast_lio check_input.py
    ros2 run fast_lio check_input.py --lidar_topic /lidar_points --imu_topic /lidar_imu --model jt128

Usage (ROS 1):
    rosrun fast_lio check_input.py
    rosrun fast_lio check_input.py --lidar_topic /lidar_points --imu_topic /lidar_imu --model jt128
"""

import argparse
import math
import struct
import sys
import threading
import time

# ── ROS version detection ─────────────────────────────────────────────────────
try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Imu, PointCloud2
    ROS_VER = 2
except ImportError:
    try:
        import rospy
        from sensor_msgs.msg import Imu, PointCloud2
        ROS_VER = 1
    except ImportError:
        sys.exit(
            "[ERROR] No ROS Python library found. "
            "Source your ROS workspace first (e.g. source /opt/ros/humble/setup.bash)."
        )

# ── Constants ─────────────────────────────────────────────────────────────────
MODEL_SPECS = {
    "jt16":  {"scan_line": 16},
    "jt128": {"scan_line": 128},
}
REQUIRED_FIELDS = {"x", "y", "z", "intensity", "ring", "timestamp"}

PASS = "\033[92m[PASS]\033[0m"
FAIL = "\033[91m[FAIL]\033[0m"
WARN = "\033[93m[WARN]\033[0m"
INFO = "\033[94m[INFO]\033[0m"


# ── Helpers ───────────────────────────────────────────────────────────────────

def _field_names(pc_msg):
    return {f.name for f in pc_msg.fields}


def _stamp_sec(stamp):
    """Convert ROS1 or ROS2 stamp to float seconds."""
    try:
        return stamp.sec + stamp.nanosec * 1e-9        # ROS 2
    except AttributeError:
        return stamp.secs + stamp.nsecs * 1e-9         # ROS 1


def _read_field(pc_msg, field_name, fmt, size, max_points=500):
    """Extract scalar values from a named PointCloud2 field."""
    field = next((f for f in pc_msg.fields if f.name == field_name), None)
    if field is None:
        return None
    step = pc_msg.point_step
    off  = field.offset
    n    = min(pc_msg.width * pc_msg.height, max_points)
    data = bytes(pc_msg.data)
    return [struct.unpack_from(fmt, data, i * step + off)[0] for i in range(n)]


# ── Main checker ──────────────────────────────────────────────────────────────

class InputChecker:
    def __init__(self, lidar_topic, imu_topic, model, timeout, expected_unit=None):
        self.lidar_topic = lidar_topic
        self.imu_topic   = imu_topic
        self.auto_model  = (model == "auto")
        self.spec        = None if self.auto_model else MODEL_SPECS[model]
        self.model_label = "AUTO" if self.auto_model else model.upper()
        self.timeout     = timeout
        self.expected_unit = expected_unit   # from yaml, for comparison (or None)

        self.pc_msg    = None    # first full frame (for field/ring/timestamp checks)
        self.pc_stamps = []      # header stamps of all received frames (for drop/sync)
        self.imu_msgs  = []
        self._lock     = threading.Lock()
        self.done      = threading.Event()

    # ── callbacks ─────────────────────────────────────────────────────────

    def _pc_cb(self, msg):
        with self._lock:
            if self.pc_msg is None:
                self.pc_msg = msg
            if len(self.pc_stamps) < 100:
                self.pc_stamps.append(_stamp_sec(msg.header.stamp))
                self._maybe_done()

    def _imu_cb(self, msg):
        with self._lock:
            if len(self.imu_msgs) < 300:
                self.imu_msgs.append(msg)
                self._maybe_done()

    def _maybe_done(self):
        # collect enough frames for drop / sync analysis before stopping
        if len(self.pc_stamps) >= 15 and len(self.imu_msgs) >= 30:
            self.done.set()

    # ── ROS 2 ─────────────────────────────────────────────────────────────

    def run_ros2(self):
        rclpy.init()
        node = rclpy.create_node("fast_lio_check_input")

        # 1 & 2: topic existence
        deadline = time.time() + self.timeout
        pc_found = imu_found = False
        while time.time() < deadline:
            pc_found  = node.count_publishers(self.lidar_topic) > 0
            imu_found = node.count_publishers(self.imu_topic)   > 0
            if pc_found and imu_found:
                break
            time.sleep(0.2)

        self._report_topic(self.lidar_topic, pc_found)
        self._report_topic(self.imu_topic,   imu_found)

        if not pc_found or not imu_found:
            node.destroy_node()
            rclpy.shutdown()
            return

        node.create_subscription(PointCloud2, self.lidar_topic, self._pc_cb,  10)
        node.create_subscription(Imu,         self.imu_topic,   self._imu_cb, 100)

        deadline = time.time() + self.timeout
        while time.time() < deadline and not self.done.is_set():
            rclpy.spin_once(node, timeout_sec=0.1)

        node.destroy_node()
        rclpy.shutdown()
        self._analyze()

    # ── ROS 1 ─────────────────────────────────────────────────────────────

    def run_ros1(self):
        rospy.init_node("fast_lio_check_input", anonymous=True)

        all_topics = dict(rospy.get_published_topics())
        pc_found  = self.lidar_topic in all_topics
        imu_found = self.imu_topic   in all_topics
        self._report_topic(self.lidar_topic, pc_found)
        self._report_topic(self.imu_topic,   imu_found)

        if not pc_found or not imu_found:
            return

        rospy.Subscriber(self.lidar_topic, PointCloud2, self._pc_cb)
        rospy.Subscriber(self.imu_topic,   Imu,         self._imu_cb)

        self.done.wait(timeout=self.timeout)
        self._analyze()

    # ── analysis ──────────────────────────────────────────────────────────

    def _analyze(self):
        print()

        if self.pc_msg is None:
            print(f"{FAIL} No point cloud received on '{self.lidar_topic}' "
                  f"within {self.timeout}s — is the driver running?")
        else:
            if self.auto_model:
                self._detect_model()
            self._check_pc_fields()
            self._check_ring_range()
            self._check_timestamp()
            self._check_timestamp_unit()
            self._check_frame_drops()
            self._report_frame_id("LiDAR", self.pc_msg.header.frame_id)

        if not self.imu_msgs:
            print(f"{FAIL} No IMU data received on '{self.imu_topic}' "
                  f"within {self.timeout}s — is the driver running?")
        else:
            self._check_imu_freq()
            self._check_gyro_unit()
            self._report_frame_id("IMU",   self.imu_msgs[0].header.frame_id)

        # cross-sensor checks (need both)
        if self.pc_stamps and self.imu_msgs:
            self._check_lidar_imu_sync()

    # 3 ── required fields
    def _check_pc_fields(self):
        present = _field_names(self.pc_msg)
        missing = REQUIRED_FIELDS - present
        if missing:
            print(f"{FAIL} PointCloud2 missing fields: {sorted(missing)}")
            print(f"       present: {sorted(present)}")
            print(f"       → FAST-LIO2 will fail to parse point cloud")
        else:
            print(f"{PASS} PointCloud2 fields: "
                  f"{', '.join(sorted(REQUIRED_FIELDS))} — all present")

    # 4 ── timestamp monotonicity
    def _check_timestamp(self):
        vals = _read_field(self.pc_msg, "timestamp", "<d", 8)
        if vals is None:
            print(f"{FAIL} 'timestamp' field missing — motion undistortion impossible")
            return
        if len(vals) < 2:
            print(f"{WARN} Too few points to verify timestamp monotonicity")
            return
        reversals = sum(1 for a, b in zip(vals, vals[1:]) if b < a)
        span_ms   = (vals[-1] - vals[0]) * 1e3
        if reversals:
            print(f"{FAIL} timestamp not monotonic: {reversals} reversal(s) in "
                  f"{len(vals)} points — motion undistortion will be wrong")
        else:
            print(f"{PASS} timestamp: monotonic, frame span = {span_ms:.2f} ms "
                  f"(per-point timestamps OK)")

    # 4b ── timestamp_unit inference
    def _check_timestamp_unit(self):
        vals = _read_field(self.pc_msg, "timestamp", "<d", 8)
        if vals is None or len(vals) < 2:
            return
        span = vals[-1] - vals[0]   # per-point time span within one frame
        if span <= 0:
            return
        # A single JT frame spans ~0.1 s of real time. Infer the unit from the
        # numeric magnitude of that span.
        if   span < 1.0:        inferred, label = 0, "seconds"
        elif span < 1e3:        inferred, label = 1, "milliseconds"
        elif span < 1e6:        inferred, label = 2, "microseconds"
        else:                   inferred, label = 3, "nanoseconds"

        if self.expected_unit is None:
            print(f"{INFO} timestamp_unit inferred: {inferred} ({label}). "
                  f"Set preprocess.timestamp_unit: {inferred} in yaml")
        elif self.expected_unit != inferred:
            print(f"{FAIL} timestamp_unit mismatch: yaml says "
                  f"{self.expected_unit}, data looks like {inferred} ({label}) "
                  f"— motion undistortion will be wrong")
        else:
            print(f"{PASS} timestamp_unit: {inferred} ({label}) matches config")

    # 4c ── frame drop / interval stability
    def _check_frame_drops(self):
        stamps = self.pc_stamps
        if len(stamps) < 3:
            print(f"{WARN} Only {len(stamps)} frame(s) — cannot assess frame drops")
            return
        intervals = [b - a for a, b in zip(stamps, stamps[1:]) if b > a]
        if not intervals:
            print(f"{WARN} Frame timestamps not increasing — check driver clock")
            return
        median = sorted(intervals)[len(intervals) // 2]
        rate   = 1.0 / median if median > 0 else 0.0
        # a gap > 1.8x the median interval indicates a likely dropped frame
        gaps = [iv for iv in intervals if iv > 1.8 * median]
        if gaps:
            print(f"{WARN} {len(gaps)} possible frame drop(s) detected "
                  f"(rate ~{rate:.1f} Hz, max gap {max(gaps)*1e3:.0f} ms) "
                  f"— may degrade mapping")
        else:
            print(f"{PASS} frame rate: ~{rate:.1f} Hz, no drops "
                  f"in {len(stamps)} frames")

    # 0 ── auto-detect model from ring range
    def _detect_model(self):
        vals = _read_field(self.pc_msg, "ring", "<H", 2)
        if vals is None:
            print(f"{WARN} cannot auto-detect model: 'ring' field missing — "
                  f"defaulting to JT128. Pass --model jt16/jt128 explicitly.")
            self.spec = MODEL_SPECS["jt128"]
            self.model_label = "JT128"
            return
        max_ring = max(vals)
        model = "jt16" if max_ring <= 15 else "jt128"
        self.spec = MODEL_SPECS[model]
        self.model_label = model.upper()
        print(f"{INFO} auto-detected model: {self.model_label} "
              f"(ring max={max_ring})")

    # 5 ── ring range
    def _check_ring_range(self):
        vals = _read_field(self.pc_msg, "ring", "<H", 2)
        if vals is None:
            print(f"{FAIL} 'ring' field missing — cannot check scan line range")
            return
        max_ring  = max(vals)
        scan_line = self.spec["scan_line"]
        if max_ring >= scan_line:
            print(f"{FAIL} ring max={max_ring} ≥ scan_line={scan_line} "
                  f"({self.model_label}) — check preprocess.scan_line config")
        elif max_ring < scan_line - 1:
            print(f"{WARN} ring max={max_ring}, expected up to {scan_line - 1} "
                  f"({self.model_label}) — wrong --model? or sparse frame?")
        else:
            print(f"{PASS} ring range: 0–{max_ring} matches "
                  f"{self.model_label} (scan_line={scan_line})")

    # 6 ── IMU frequency
    def _check_imu_freq(self):
        msgs = self.imu_msgs
        if len(msgs) < 2:
            print(f"{WARN} Only {len(msgs)} IMU message(s) — cannot estimate frequency")
            return
        t0   = _stamp_sec(msgs[0].header.stamp)
        t1   = _stamp_sec(msgs[-1].header.stamp)
        if t1 <= t0:
            print(f"{WARN} IMU timestamps not increasing — check driver clock")
            return
        freq = (len(msgs) - 1) / (t1 - t0)
        if freq < 50:
            print(f"{FAIL} IMU frequency {freq:.1f} Hz — too low "
                  f"(FAST-LIO2 expects ≥ 100 Hz)")
        elif freq < 150:
            print(f"{WARN} IMU frequency {freq:.1f} Hz — lower than typical 200 Hz")
        else:
            print(f"{PASS} IMU frequency: {freq:.1f} Hz")

    # 7 ── gyro magnitude (deg/s vs rad/s detection)
    def _check_gyro_unit(self):
        mags = []
        for m in self.imu_msgs[:50]:
            g = m.angular_velocity
            mags.append(math.sqrt(g.x**2 + g.y**2 + g.z**2))
        median = sorted(mags)[len(mags) // 2]

        # Typical gyro at rest or mild motion: << 1 rad/s.
        # If median > ~8.7 rad/s (500 deg/s), almost certainly published in deg/s.
        threshold = math.radians(500)
        if median > threshold:
            print(f"{FAIL} gyro median ‖ω‖ = {median:.2f} — looks like deg/s "
                  f"(> {math.degrees(threshold):.0f} deg/s). "
                  f"Set imu_gyr_unit: \"deg\" in yaml")
        elif median > math.radians(50):
            print(f"{WARN} gyro median ‖ω‖ = {median:.4f} rad/s — unusually high. "
                  f"Verify imu_gyr_unit matches your driver output")
        else:
            print(f"{PASS} gyro magnitude: {median:.4f} rad/s — normal range")

    # 9 ── LiDAR / IMU time-base synchronization
    def _check_lidar_imu_sync(self):
        pc_t  = self.pc_stamps[len(self.pc_stamps) // 2]
        imu_t = [_stamp_sec(m.header.stamp) for m in self.imu_msgs]
        # nearest IMU stamp to the chosen LiDAR stamp
        nearest = min(imu_t, key=lambda t: abs(t - pc_t))
        offset_ms = abs(nearest - pc_t) * 1e3
        # also check overall ranges overlap (different clock base → huge offset)
        if offset_ms > 1000:
            print(f"{FAIL} LiDAR/IMU time bases differ by {offset_ms:.0f} ms "
                  f"— likely different clock sources; FAST-LIO2 will fail to sync")
        elif offset_ms > 100:
            print(f"{WARN} LiDAR/IMU stamp offset {offset_ms:.0f} ms — consider "
                  f"common.time_offset_lidar_to_imu or check time sync")
        else:
            print(f"{PASS} LiDAR/IMU time sync: nearest stamp offset "
                  f"{offset_ms:.1f} ms")

    # 8 ── frame_id
    def _report_frame_id(self, label, frame_id):
        if not frame_id:
            print(f"{WARN} {label} frame_id is empty — TF tree may be broken")
        else:
            print(f"{INFO} {label} frame_id: '{frame_id}'")

    def _report_topic(self, topic, found):
        if found:
            print(f"{PASS} topic '{topic}': publisher found")
        else:
            print(f"{FAIL} topic '{topic}': no publisher — "
                  f"is the driver started and publishing?")


# ── entry point ───────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="Validate LiDAR and IMU inputs for FAST-LIO2 (Hesai JT)")
    parser.add_argument("--lidar_topic", default="/lidar_points",
                        help="Point cloud topic  (default: /lidar_points)")
    parser.add_argument("--imu_topic",   default="/lidar_imu",
                        help="IMU topic           (default: /lidar_imu)")
    parser.add_argument("--model",       default="auto",
                        choices=list(MODEL_SPECS.keys()) + ["auto"],
                        help="LiDAR model: jt16, jt128, or auto "
                             "(auto-detect from ring range; default: auto)")
    parser.add_argument("--timeout",     type=float, default=8.0,
                        help="Seconds to wait for messages  (default: 8.0)")
    parser.add_argument("--timestamp-unit", type=int, default=None,
                        choices=[0, 1, 2, 3],
                        help="Your configured preprocess.timestamp_unit "
                             "(0=s,1=ms,2=us,3=ns); enables a mismatch check")
    # strip ROS remapping args that may be injected by rosrun
    args, _ = parser.parse_known_args()

    print(f"\n{'='*62}")
    print(f"  FAST-LIO2 Input Checker  |  model: {args.model.upper()}"
          f"  |  ROS {ROS_VER}")
    print(f"  lidar : {args.lidar_topic}")
    print(f"  imu   : {args.imu_topic}")
    print(f"{'='*62}\n")

    checker = InputChecker(
        lidar_topic=args.lidar_topic,
        imu_topic=args.imu_topic,
        model=args.model,
        timeout=args.timeout,
        expected_unit=args.timestamp_unit,
    )

    if ROS_VER == 2:
        checker.run_ros2()
    else:
        checker.run_ros1()

    print(f"\n{'='*62}\n")


if __name__ == "__main__":
    main()
