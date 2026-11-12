#!/usr/bin/env python3
# Copyright 2026 Hesai Technology. SPDX-License-Identifier: GPL-2.0
"""Read exact per-topic counts; never infer one topic's count from adjacent text."""
import argparse
import json
from pathlib import Path
import sqlite3
import subprocess
from urllib.parse import quote
import yaml




class HeaderTimeAudit:
    """Audit sensor header time, never the recorder's wall-clock receive time."""
    def __init__(self, topics):
        self.topics = {name: {"count": 0, "first_stamp_ns": None,
                             "last_stamp_ns": None, "max_gap_ns": 0} for name in topics}

    def observe(self, topic, msg):
        row = self.topics[topic]
        stamp = msg.header.stamp
        sec = int(stamp.sec if hasattr(stamp, "sec") else stamp.secs)
        nsec = int(stamp.nanosec if hasattr(stamp, "nanosec") else stamp.nsecs)
        if sec < 0 or not 0 <= nsec < 1000000000:
            raise ValueError(topic + " contains an invalid header timestamp")
        current = sec * 1000000000 + nsec
        previous = row["last_stamp_ns"]
        if previous is not None:
            if current <= previous:
                raise ValueError(f"{topic} header timestamp repeated or moved backwards at message "
                                 f"{row['count'] + 1}: {current} <= {previous}; "
                                 "verify the driver clock and any explicit --timestamp-offset")
            row["max_gap_ns"] = max(row["max_gap_ns"], current - previous)
        else:
            row["first_stamp_ns"] = current
        row["last_stamp_ns"] = current
        row["count"] += 1

    def finish(self, expected):
        for name, row in self.topics.items():
            if row["count"] != expected[name]["count"]:
                raise ValueError(name + " decoded count does not match bag metadata")
            if not row["count"] or row["last_stamp_ns"] == 0:
                raise ValueError(name + " has no usable header timestamps (all zero/empty)")
        return self.topics


def audit_header_times(path, ros, topics):
    audit = HeaderTimeAudit(topics)
    if int(ros) == 1:
        import rosbag
        with rosbag.Bag(str(path), "r") as bag:
            for topic, message, _ in bag.read_messages(topics=list(topics)):
                audit.observe(topic, message)
    else:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from sensor_msgs.msg import Imu, PointCloud2
        path = Path(path)
        metadata = {}
        if path.is_dir():
            metadata = yaml.safe_load((path / "metadata.yaml").read_text())["rosbag2_bagfile_information"]
        storage = metadata.get("storage_identifier", "sqlite3")
        reader_class = (rosbag2_py.SequentialCompressionReader if metadata.get("compression_format")
                        else rosbag2_py.SequentialReader)
        reader = reader_class()
        reader.open(rosbag2_py.StorageOptions(uri=str(path), storage_id=storage),
                    rosbag2_py.ConverterOptions("", ""))
        reader.set_filter(rosbag2_py.StorageFilter(topics=list(topics)))
        while reader.has_next():
            topic, data, _ = reader.read_next()
            kind = Imu if topics[topic]["type"].replace("/msg/", "/") == "sensor_msgs/Imu" else PointCloud2
            audit.observe(topic, deserialize_message(data, kind))
    return audit.finish(topics)


def inspect_bag(path, ros, lidar_topic="/lidar_points", imu_topic="/lidar_imu", validate_timestamps=False):
    path = Path(path)
    if int(ros) == 1:
        result = subprocess.run(["rosbag", "info", "--yaml", str(path)],
                                capture_output=True, text=True, check=True)
        meta = yaml.safe_load(result.stdout)
        topics = {t["topic"]: {"count": int(t["messages"]), "type": t["type"]}
                  for t in meta.get("topics", [])}
    elif path.is_file() and path.suffix == ".db3":
        with sqlite3.connect("file:" + quote(str(path.resolve())) + "?mode=ro", uri=True) as conn:
            rows = conn.execute("SELECT t.name, t.type, COUNT(m.id) FROM topics t "
                                "LEFT JOIN messages m ON t.id=m.topic_id GROUP BY t.id").fetchall()
        topics = {name: {"count": int(count), "type": kind} for name, kind, count in rows}
    else:
        with open(path / "metadata.yaml", encoding="utf-8") as stream:
            meta = yaml.safe_load(stream)["rosbag2_bagfile_information"]
        topics = {t["topic_metadata"]["name"]: {"count": int(t["message_count"]),
                   "type": t["topic_metadata"]["type"]} for t in meta.get("topics_with_message_count", [])}
    required = {lidar_topic: "sensor_msgs/PointCloud2", imu_topic: "sensor_msgs/Imu"}
    for name, kind in required.items():
        entry = topics.get(name)
        if not entry or entry["count"] <= 0:
            raise ValueError(name + " has no messages in bag")
        if entry["type"].replace("/msg/", "/") != kind:
            raise ValueError(name + " has unexpected type: " + entry["type"])
    result = {"path": str(path.resolve()), "ros": int(ros), "topics": topics,
            "lidar_topic": lidar_topic, "imu_topic": imu_topic,
            "lidar_count": topics[lidar_topic]["count"], "imu_count": topics[imu_topic]["count"]}
    if validate_timestamps:
        result["header_timestamps"] = audit_header_times(
            path, ros, {name: topics[name] for name in required})
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("path")
    parser.add_argument("--ros", required=True, choices=("1", "2"))
    parser.add_argument("--lidar-topic", default="/lidar_points")
    parser.add_argument("--imu-topic", default="/lidar_imu")
    parser.add_argument("--validate-timestamps", action="store_true",
                        help="Decode every selected message and require strictly increasing sensor header times")
    args = parser.parse_args()
    try:
        print(json.dumps(inspect_bag(args.path, args.ros, args.lidar_topic, args.imu_topic, args.validate_timestamps), indent=2))
    except (OSError, ValueError, KeyError, TypeError, ImportError, RuntimeError, sqlite3.Error, subprocess.CalledProcessError) as exc:
        parser.exit(2, "[ERROR] Bag validation: " + str(exc) + "\n")


if __name__ == "__main__":
    main()
