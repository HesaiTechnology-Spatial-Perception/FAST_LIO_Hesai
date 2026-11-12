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
  5. 'ring' range matches the declared model (JT16: 0–15, JT32: 0–31,
     JT128: 0–127, MT60 MT_V7: 0–1)
  6. IMU publish frequency
  7. Driver IMU units — raw g + deg/s vs SI m/s² + rad/s
  8. frame_id of both sensors (for coordinate frame awareness)

Usage (ROS 2):
    ros2 run fast_lio check_input.py
    ros2 run fast_lio check_input.py --lidar_topic /lidar_points --imu_topic /lidar_imu --model mt60

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
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import Imu, PointCloud2
    ROS_VER = 2
except ImportError:
    try:
        import rospy
        from sensor_msgs.msg import Imu, PointCloud2
        ROS_VER = 1
    except ImportError:
        ROS_VER = 0


# ── Constants ─────────────────────────────────────────────────────────────────
MODEL_SPECS = {
    "jt16":  {"scan_line": 16},
    "jt32":  {"scan_line": 32},
    "jt128": {"scan_line": 128},
    # JT64P and JT128 share the same 128-ring PointCloud2 layout.
    "jt64p": {"scan_line": 128},
    "mt60":  {"scan_line": 2},
}
REQUIRED_FIELDS = {"x", "y", "z", "intensity", "ring", "timestamp"}
EXPECTED_FIELD_TYPES = {
    "x": 7, "y": 7, "z": 7, "intensity": 7,  # PointField.FLOAT32
    "ring": 4,                                    # PointField.UINT16
    "timestamp": 8,                               # PointField.FLOAT64
}

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
    expected = {"<f": 7, "<H": 4, "<d": 8}.get(fmt)
    if field.datatype != expected or field.count != 1:
        return None
    step, off = pc_msg.point_step, field.offset
    point_count = pc_msg.width * pc_msg.height
    n = point_count if max_points is None else min(point_count, max_points)
    data = bytes(pc_msg.data)
    if n == 0 or step <= 0 or off < 0 or off + size > step or pc_msg.row_step < pc_msg.width * step:
        return []
    offsets = [(i // pc_msg.width) * pc_msg.row_step + (i % pc_msg.width) * step + off for i in range(n)]
    if offsets[-1] + size > len(data):
        return []
    if pc_msg.is_bigendian and fmt.startswith("<"):
        fmt = ">" + fmt[1:]
    return [struct.unpack_from(fmt, data, offset)[0] for offset in offsets]



# ── Main checker ──────────────────────────────────────────────────────────────

class InputChecker:
    def __init__(self, lidar_topic, imu_topic, model, timeout, expected_unit=None):
        self.lidar_topic = lidar_topic
        self.imu_topic   = imu_topic
        self.auto_model  = (model == "auto")
        self.spec        = None if self.auto_model else MODEL_SPECS[model]
        self.model_label = "AUTO" if self.auto_model else model.upper()
        self.timeout     = timeout
        self.failures = 0
        self.expected_unit = expected_unit   # from yaml, for comparison (or None)

        self.pc_msg    = None    # first full frame (for field/ring/timestamp checks)
        self.pc_stamps = []      # header stamps of all received frames (for drop/sync)
        self.imu_msgs  = []
        self._lock     = threading.Lock()
        self.done      = threading.Event()

    def _fail(self, message):
        self.failures += 1
        print(f"{FAIL} {message}")

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

        node.create_subscription(PointCloud2, self.lidar_topic, self._pc_cb, QoSProfile(depth=16, reliability=ReliabilityPolicy.BEST_EFFORT))
        node.create_subscription(Imu,         self.imu_topic,   self._imu_cb, QoSProfile(depth=1000, reliability=ReliabilityPolicy.BEST_EFFORT))

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
            self._fail(f"No point cloud received on '{self.lidar_topic}' "
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
            self._fail(f"No IMU data received on '{self.imu_topic}' "
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
            self._fail(f"PointCloud2 missing fields: {sorted(missing)}")
            print(f"       present: {sorted(present)}")
            print(f"       → FAST-LIO2 will fail to parse point cloud")
        else:
            print(f"{PASS} PointCloud2 fields: "
                  f"{', '.join(sorted(REQUIRED_FIELDS))} — all present")
        fields = {field.name: field for field in self.pc_msg.fields}
        mismatches = [
            f"{name}=datatype {fields[name].datatype}, count {fields[name].count}"
            for name, expected in EXPECTED_FIELD_TYPES.items()
            if name in fields and
            (fields[name].datatype != expected or fields[name].count != 1)
        ]
        if mismatches:
            self._fail(f"PointCloud2 field type mismatch: " + "; ".join(mismatches))
            print("       expected x/y/z/intensity=float32, ring=uint16, timestamp=float64")
        elif not missing:
            print(f"{PASS} PointCloud2 field datatypes match the Hesai parser")

    # 4 ── timestamp monotonicity
    def _check_timestamp(self):
        vals = _read_field(self.pc_msg, "timestamp", "<d", 8, max_points=None)
        if not vals:
            self._fail(f"'timestamp' field missing, empty, or truncated — "
                  "motion undistortion impossible")
            return
        if len(vals) < 2:
            print(f"{WARN} Too few points to verify timestamp monotonicity")
            return
        if not all(math.isfinite(t) for t in vals):
            self._fail("Non-finite per-point timestamps")
            return
        span = max(vals) - min(vals)
        if span <= 0:
            self._fail("All point timestamps are identical; motion compensation is unavailable")
            return
        reversals = sum(b < a for a, b in zip(vals, vals[1:]))
        print(f"{INFO} per-point timestamp span: {span:.9g} in field units ({len(vals)} points)")
        if reversals:
            print(f"{WARN} {reversals} point timestamp reversal(s); FAST-LIO sorts points by time. "
                  "Check packet ordering and the header/first-point time convention.")
        else:
            print(f"{PASS} Point timestamps are finite and ordered in the inspected frame")

    # 4b ── timestamp_unit inference
    def _check_timestamp_unit(self):
        vals = _read_field(self.pc_msg, "timestamp", "<d", 8, max_points=None)
        if vals is None or len(vals) < 2:
            return
        if not all(math.isfinite(t) for t in vals):
            return
        span = max(vals) - min(vals)   # per-point time span within one frame
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
            self._fail(f"timestamp_unit mismatch: yaml says "
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
        if any(b <= a for a, b in zip(stamps, stamps[1:])):
            self._fail("LiDAR header timestamps repeat or move backwards")
            return
        intervals = [b - a for a, b in zip(stamps, stamps[1:])]
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
                  f"observed in this {len(stamps)}-frame sample; this is not a full-bag audit")

    # 0 ── auto-detect model from ring range
    def _detect_model(self):
        vals = _read_field(self.pc_msg, "ring", "<H", 2, max_points=None)
        if not vals:
            print(f"{WARN} cannot auto-detect model: 'ring' field missing or empty — "
                  f"defaulting to JT128. Pass --model explicitly.")
            self.spec = MODEL_SPECS["jt128"]
            self.model_label = "JT128"
            return
        max_ring = max(vals)
        if max_ring <= 1:
            print(f"{WARN} ring max={max_ring} is ambiguous: MT60 MT_V7 uses "
                  "rings 0–1, but a sparse JT frame may look the same. "
                  "Defaulting to JT16; pass --model mt60 for MT60 data.")
            model = "jt16"
        elif max_ring <= 15:
            model = "jt16"
        elif max_ring <= 31:
            model = "jt32"
        else:
            model = "jt128"
        self.spec = MODEL_SPECS[model]
        if model == "jt128":
            self.model_label = "JT128/JT64P"
            print(f"{WARN} auto-detected shared 128-ring layout "
                  f"(ring max={max_ring}); JT128 and JT64P cannot be "
                  "distinguished from PointCloud2 fields. Pass --model "
                  "jt128 or --model jt64p explicitly.")
        else:
            self.model_label = model.upper()
            print(f"{INFO} auto-detected model: {self.model_label} "
                  f"(ring max={max_ring})")

    # 5 ── ring range
    def _check_ring_range(self):
        vals = _read_field(self.pc_msg, "ring", "<H", 2, max_points=None)
        if not vals:
            self._fail(f"'ring' field missing, empty, or truncated — cannot check scan line range")
            return
        max_ring  = max(vals)
        scan_line = self.spec["scan_line"]
        if max_ring >= scan_line:
            self._fail(f"ring max={max_ring} ≥ scan_line={scan_line} "
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
        if t0 == 0.0 and t1 == 0.0:
            self._fail(f"IMU timestamps are all zero — FAST-LIO2 cannot synchronize")
            return
        if t1 <= t0:
            self._fail(f"IMU timestamps not increasing — check driver clock")
            return
        stamps = [_stamp_sec(m.header.stamp) for m in msgs]
        intervals = [b - a for a, b in zip(stamps, stamps[1:])]
        if any(t <= 0 for t in intervals):
            self._fail("IMU timestamps repeat or move backwards inside the sample")
            return
        median = sorted(intervals)[len(intervals) // 2]
        if max(intervals) > 2.5 * median:
            print(f"{WARN} IMU gap: {max(intervals)*1000:.2f} ms (median {median*1000:.2f} ms)")
        freq = (len(msgs) - 1) / (t1 - t0)
        if freq < 50:
            self._fail(f"IMU frequency {freq:.1f} Hz — too low "
                  f"(FAST-LIO2 expects ≥ 100 Hz)")
        elif freq < 150:
            print(f"{WARN} IMU frequency {freq:.1f} Hz — lower than typical 200 Hz")
        else:
            print(f"{PASS} IMU frequency: {freq:.1f} Hz")

    # 7 ── driver IMU unit mode (raw g + deg/s vs SI m/s² + rad/s)
    def _check_gyro_unit(self):
        gyro_mags = []
        acc_mags = []
        for m in self.imu_msgs[:50]:
            g = m.angular_velocity
            a = m.linear_acceleration
            gyro_mags.append(math.sqrt(g.x**2 + g.y**2 + g.z**2))
            acc_mags.append(math.sqrt(a.x**2 + a.y**2 + a.z**2))
        if not all(math.isfinite(v) for v in gyro_mags + acc_mags):
            self._fail("Non-finite IMU acceleration or angular velocity")
            return
        median_gyro = sorted(gyro_mags)[len(gyro_mags) // 2]
        median_acc = sorted(acc_mags)[len(acc_mags) // 2]

        # Hesai ROS Driver versions switch acceleration and gyro units together.
        # Raw SDK output is near 1 g + deg/s; ROS-conformant output is near
        # 9.81 m/s² + rad/s. Acceleration is reliable even while stationary.
        if median_acc < 0.1:
            self._fail(f"IMU acceleration is zero/invalid: median ‖a‖={median_acc:.3f}; "
                  "FAST-LIO2 cannot initialize")
        elif median_acc < 4.0:
            print(f"{INFO} acceleration is consistent with raw g units: median ‖a‖={median_acc:.3f} g, "
                  f"gyro may be deg/s (median ‖ω‖={median_gyro:.3f}). "
                  f"Verify the driver contract before selecting common.imu_gyr_unit: \"deg\"")
        elif median_acc > 6.0:
            print(f"{PASS} acceleration is consistent with SI units: median ‖a‖={median_acc:.3f} m/s², "
                  f"gyro may be rad/s (median ‖ω‖={median_gyro:.4f}). "
                  f"Verify the driver contract before selecting common.imu_gyr_unit: \"rad\"")
        else:
            print(f"{WARN} cannot infer driver IMU units: median ‖a‖={median_acc:.3f}. "
                  f"Keep the sensor stationary at startup or set imu_gyr_unit manually")

    # 9 ── LiDAR / IMU time-base synchronization
    def _check_lidar_imu_sync(self):
        pc_t  = self.pc_stamps[len(self.pc_stamps) // 2]
        imu_t = [_stamp_sec(m.header.stamp) for m in self.imu_msgs]
        # nearest IMU stamp to the chosen LiDAR stamp
        nearest = min(imu_t, key=lambda t: abs(t - pc_t))
        offset_ms = abs(nearest - pc_t) * 1e3
        # also check overall ranges overlap (different clock base → huge offset)
        if offset_ms > 1000:
            self._fail(f"LiDAR/IMU time bases differ by {offset_ms:.0f} ms "
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
            self._fail(f"topic '{topic}': no publisher — "
                  f"is the driver started and publishing?")


# ── entry point ───────────────────────────────────────────────────────────────

def main():
    if ROS_VER == 0:
        print("[ERROR] Source a ROS workspace before checking live inputs.", file=sys.stderr)
        return 2
    parser = argparse.ArgumentParser(
        description="Validate LiDAR and IMU inputs for FAST-LIO2 (Hesai JT/MT60)")
    parser.add_argument("--lidar_topic", default="/lidar_points",
                        help="Point cloud topic  (default: /lidar_points)")
    parser.add_argument("--imu_topic",   default="/lidar_imu",
                        help="IMU topic           (default: /lidar_imu)")
    parser.add_argument("--model",       default="auto",
                        choices=list(MODEL_SPECS.keys()) + ["auto"],
                        help="LiDAR model: jt16, jt32, jt64p, jt128, mt60, or auto "
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


    return 1 if checker.failures else 0


if __name__ == "__main__":
    sys.exit(main())
