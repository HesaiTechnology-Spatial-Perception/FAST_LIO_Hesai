#!/usr/bin/env python3
# Copyright 2026 Hesai Technology. SPDX-License-Identifier: GPL-2.0
"""Wait for the mapper, verify bag coverage, and validate map-save responses."""
import argparse
import json
import os
from pathlib import Path
import time


def validate_drain(status, metadata):
    """None means still draining; an exception means a confirmed failed run."""
    if status.get("exit_code"):
        raise ValueError(status.get("error", "Mapper reported a failure"))
    for key, expected in (("received_lidar", metadata["lidar_count"]),
                          ("received_imu", metadata["imu_count"])):
        if status[key] > expected:
            raise ValueError(key + " exceeds bag count; another publisher may be active")
        if status[key] < expected:
            return None
    if status["pending_lidar"] or status.get("processing_frame", False):
        return None
    if status["synchronized_frames"] != metadata["lidar_count"]:
        raise ValueError("Synchronized frame count does not cover the bag")
    if status["registered_frames"] <= 0:
        raise ValueError("No registered frames were produced")
    if "startup_frames" not in status or "skipped_frames" not in status:
        raise ValueError("Mapper lacks frame accounting; rebuild FAST-LIO before replay")
    if status["skipped_frames"]:
        raise ValueError("Scans skipped after startup: " + str(status["skipped_frames"]))
    if status["startup_frames"] + status["registered_frames"] != status["synchronized_frames"]:
        raise ValueError("Synchronized scans are not all accounted for by startup/registration")
    if status["last_registered_stamp"] + 1e-6 < status["last_input_lidar"]:
        raise ValueError("Registered trajectory does not cover the final input scan")
    return True


def verify_saved_maps(report):
    files = report.get("files", [])
    if not files:
        raise ValueError("Map service did not report any output files")
    for entry in files:
        path = Path(entry["path"])
        with path.open("rb") as stream:
            fields = {}
            for _ in range(128):
                line = stream.readline()
                if not line:
                    raise ValueError("Truncated PCD header: " + str(path))
                parts = line.decode("ascii").strip().split()
                if not parts or parts[0].startswith("#"):
                    continue
                fields[parts[0]] = parts[1:]
                if parts[0] == "DATA":
                    break
            else:
                raise ValueError("PCD header exceeds 128 lines")
            count = int(fields["POINTS"][0])
            if fields["DATA"] != ["binary"] or count <= 0 or count != entry["points"]:
                raise ValueError("PCD format/count mismatch: " + str(path))
            sizes = list(map(int, fields["SIZE"]))
            counts = list(map(int, fields.get("COUNT", ["1"] * len(sizes))))
            if len(sizes) != len(counts):
                raise ValueError("Invalid PCD field layout")
            expected_bytes = count * sum(s * n for s, n in zip(sizes, counts))
            if path.stat().st_size - stream.tell() != expected_bytes:
                raise ValueError("PCD payload size mismatch: " + str(path))
    return files


class Client:
    def __init__(self, ros):
        self.ros = ros
        if ros == 2:
            import rclpy
            from std_srvs.srv import Trigger
            self.api, self.trigger = rclpy, Trigger
            rclpy.init()
            self.node = rclpy.create_node("fastlio_control_" + str(os.getpid()))
            self.clients = {}
        else:
            import rospy
            from std_srvs.srv import Trigger
            self.api, self.trigger = rospy, Trigger
            rospy.init_node("fastlio_control", anonymous=True, disable_signals=True)

    def call(self, name, timeout, service_type=None):
        kind = service_type or self.trigger
        if self.ros == 2:
            if name not in self.clients:
                self.clients[name] = self.node.create_client(kind, name)
            client = self.clients[name]
            start = time.monotonic()
            if not client.wait_for_service(timeout_sec=timeout):
                raise TimeoutError("Service unavailable: " + name)
            future = client.call_async(kind.Request())
            self.api.spin_until_future_complete(self.node, future, timeout_sec=max(0.0, timeout - (time.monotonic() - start)))
            if not future.done():
                future.cancel()
                raise TimeoutError("Service response timed out: " + name)
            return future.result()
        self.api.wait_for_service(name, timeout=timeout)
        return self.api.ServiceProxy(name, kind)()

    def status(self, timeout, run_id):
        response = self.call("/mapping_status", timeout)
        status = json.loads(response.message)
        if run_id and status.get("run_id") != run_id:
            raise ValueError("Mapper run ID mismatch; use an isolated ROS domain/master")
        if not response.success or status.get("exit_code"):
            raise ValueError(status.get("error", "Mapper failed"))
        return status

    def resume(self, args):
        if self.ros != 2:
            raise ValueError("Paused-player resume is used only for ROS 2")
        from rosbag2_interfaces.srv import Resume
        deadline = time.monotonic() + args.timeout
        stable_since = None
        while time.monotonic() < deadline:
            status = self.status(min(5, max(0.1, deadline-time.monotonic())), args.run_id)
            # Graph visibility alone can precede actual DDS matching by seconds.
            ready = status.get("matched_lidar_publishers", 0) > 0 and status.get("matched_imu_publishers", 0) > 0
            # Reader-side matching can precede the writer learning about the reader.
            # Require stable matching before releasing the first volatile sample.
            # This is a discovery grace period, not proof of delivery: drain still
            # checks every input count and final registration coverage.
            now = time.monotonic()
            if not ready:
                stable_since = None
            elif stable_since is None:
                stable_since = now
            if ready and now - stable_since >= 2.0:
                self.call("/" + args.player_name + "/resume", max(0.1, deadline - time.monotonic()), Resume)
                return {"player_resumed": args.player_name}
            self.api.spin_once(self.node, timeout_sec=0.1)
        raise TimeoutError("Player and mapper topic discovery did not finish")

    def close(self):
        if self.ros == 2:
            self.node.destroy_node()
            self.api.shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=("ready", "drain", "save", "resume"))
    parser.add_argument("--ros", type=int, choices=(1, 2), required=True)
    parser.add_argument("--timeout", type=float, default=120)
    parser.add_argument("--run-id", default="")
    parser.add_argument("--metadata")
    parser.add_argument("--output")
    parser.add_argument("--lidar-topic", default="/lidar_points")
    parser.add_argument("--imu-topic", default="/lidar_imu")
    parser.add_argument("--player-name", default="rosbag2_player")
    args = parser.parse_args()
    client = None
    result = {"action": args.action, "success": False}
    code = 0
    try:
        if args.timeout <= 0:
            raise ValueError("Timeout must be positive")
        client = Client(args.ros)
        status = client.status(min(args.timeout, 30), args.run_id)
        if args.action == "drain":
            if not args.metadata:
                raise ValueError("drain requires --metadata")
            metadata = json.loads(Path(args.metadata).read_text())
            deadline = time.monotonic() + args.timeout
            while validate_drain(status, metadata) is None:
                result["status"] = status
                if time.monotonic() >= deadline:
                    raise TimeoutError("Input not fully processed: received/expected LiDAR "
                        f"{status['received_lidar']}/{metadata['lidar_count']}, IMU "
                        f"{status['received_imu']}/{metadata['imu_count']}, pending scans "
                        f"{status['pending_lidar']}. The tail may lack IMU coverage; inspect status JSON.")
                time.sleep(0.2)
                status = client.status(min(5, max(0.1, deadline - time.monotonic())), args.run_id)
            result["expected"] = metadata
        elif args.action == "save":
            response = client.call("/map_save", args.timeout)
            if not response.success:
                raise ValueError("Map save failed: " + response.message)
            report = json.loads(response.message)
            verify_saved_maps(report)
            result["maps"] = report
        elif args.action == "resume":
            result.update(client.resume(args))
        result.update(success=True, status=status)
    except Exception as exc:
        result["error"] = str(exc)
        code = 1
    finally:
        if client:
            client.close()
    text = json.dumps(result, ensure_ascii=False, indent=2)
    if args.output:
        path = Path(args.output)
        path.parent.mkdir(parents=True, exist_ok=True)
        temp = path.with_name(path.name + ".tmp")
        temp.write_text(text + "\n", encoding="utf-8")
        temp.replace(path)
    print(text)
    return code


if __name__ == "__main__":
    raise SystemExit(main())
