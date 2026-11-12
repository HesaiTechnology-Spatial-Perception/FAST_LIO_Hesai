#!/usr/bin/env python3
# Copyright 2026 Hesai Technology. SPDX-License-Identifier: GPL-2.0
"""Regression cases for malformed input, calibration ambiguity and false success."""
import contextlib
import io
import json
import os
import sys
from pathlib import Path
import struct
import subprocess
import tempfile
from types import SimpleNamespace as NS
import unittest
from unittest import mock
import numpy as np
import yaml

from bag_metadata import inspect_bag, HeaderTimeAudit
from mapping_control import Client, validate_drain, verify_saved_maps
from prepare_runtime_config import prepare_config
from check_input import InputChecker, _read_field
from check_config import ConfigChecker
from check_map import Verdict, analyze_trajectory, _load_pcd_minimal


class ReliabilityTests(unittest.TestCase):
    def test_resume_requires_stable_matching_after_a_gap(self):
        clock = [0.0]
        client = Client.__new__(Client); client.ros = 2; client.node = object()
        def spin(node, timeout_sec):
            clock[0] += timeout_sec
        client.api = NS(spin_once=spin)
        # Matching briefly disappears after the initial discovery notification.
        client.status = lambda timeout, run_id: {
            'matched_lidar_publishers': int(not .5 <= clock[0] < 1.0),
            'matched_imu_publishers': 1}
        client.call = mock.Mock()
        args = NS(timeout=10, run_id='test', player_name='player')
        with mock.patch.dict(sys.modules, {'rosbag2_interfaces.srv': NS(Resume=object)}), \
             mock.patch('mapping_control.time.monotonic', side_effect=lambda: clock[0]):
            client.resume(args)
        self.assertGreaterEqual(clock[0], 3.0)
        client.call.assert_called_once()

    def test_resume_never_releases_an_unmatched_player(self):
        clock = [0.0]
        client = Client.__new__(Client); client.ros = 2; client.node = object()
        def spin(node, timeout_sec):
            clock[0] += timeout_sec
        client.api = NS(spin_once=spin)
        client.status = lambda timeout, run_id: {'matched_lidar_publishers': 1, 'matched_imu_publishers': 0}
        client.call = mock.Mock()
        args = NS(timeout=1, run_id='test', player_name='player')
        with mock.patch.dict(sys.modules, {'rosbag2_interfaces.srv': NS(Resume=object)}), \
             mock.patch('mapping_control.time.monotonic', side_effect=lambda: clock[0]):
            with self.assertRaises(TimeoutError):
                client.resume(args)
        client.call.assert_not_called()

    def test_empty_topic_is_not_masked_by_nonempty_neighbor(self):
        with tempfile.TemporaryDirectory() as folder:
            topics = [{"topic_metadata": {"name": name, "type": kind}, "message_count": count}
                      for name, kind, count in [("/lidar_points", "sensor_msgs/msg/PointCloud2", 0),
                                                ("/lidar_imu", "sensor_msgs/msg/Imu", 500)]]
            path = Path(folder, "metadata.yaml")
            path.write_text(yaml.safe_dump({"rosbag2_bagfile_information": {"topics_with_message_count": topics}}))
            with self.assertRaisesRegex(ValueError, "lidar_points"):
                inspect_bag(folder, 2)
            topics[0]["message_count"] = 10
            path.write_text(yaml.safe_dump({"rosbag2_bagfile_information": {"topics_with_message_count": topics}}))
            self.assertEqual(inspect_bag(folder, 2)["lidar_count"], 10)

    def test_calibration_never_silently_reuses_driver_file(self):
        common = Path(__file__).parent / "pcap_to_rosbag/common.sh"
        with tempfile.TemporaryDirectory() as folder:
            csv = Path(folder, "JT16.csv"); csv.write_text("wrong device")
            cfg = Path(folder, "config.yaml")
            cfg.write_text(yaml.safe_dump({"lidar": [{"driver": {"correction_file_path": str(csv)}}]}))
            result = subprocess.run(["bash", "-c", 'source "$1"; MODEL=jt128; DRIVER_CONFIG="$2"; '
                                     'warn() { echo "$*" >&2; }; pcap_find_calibration correction',
                                     "test", str(common), str(cfg)], capture_output=True, text=True)
            self.assertNotEqual(result.returncode, 0)
            self.assertEqual(result.stdout, "")

    def test_input_failures_have_a_failure_count(self):
        checker = InputChecker("/p", "/i", "jt16", 1)
        with contextlib.redirect_stdout(io.StringIO()):
            checker._report_topic("/p", False)
            checker._analyze()
        self.assertEqual(checker.failures, 3)

    def test_organized_cloud_row_padding(self):
        field = NS(name="timestamp", datatype=8, count=1, offset=0)
        msg = NS(fields=[field], point_step=8, row_step=24, width=2, height=2,
                 is_bigendian=False, data=struct.pack("<dd", 1, 2) + b"padding!" + struct.pack("<dd", 3, 4))
        self.assertEqual(_read_field(msg, "timestamp", "<d", 8, None), [1, 2, 3, 4])
        msg.data = msg.data[:-1]
        self.assertEqual(_read_field(msg, "timestamp", "<d", 8, None), [])

    def test_imu_rollback_inside_otherwise_increasing_sample(self):
        checker = InputChecker("/p", "/i", "jt16", 1)
        checker.imu_msgs = [NS(header=NS(stamp=NS(sec=1, nanosec=n))) for n in (0, 10000000, 5000000, 20000000)]
        with contextlib.redirect_stdout(io.StringIO()):
            checker._check_imu_freq()
        self.assertGreater(checker.failures, 0)

    def test_full_frame_timestamp_span(self):
        checker = InputChecker("/p", "/i", "jt16", 1, expected_unit=1)
        values = np.linspace(0, 100, 100000, dtype="<f8")
        checker.pc_msg = NS(fields=[NS(name="timestamp", datatype=8, count=1, offset=0)],
                            data=values.tobytes(), width=len(values), height=1, point_step=8,
                            row_step=8*len(values), is_bigendian=False)
        with contextlib.redirect_stdout(io.StringIO()):
            checker._check_timestamp_unit()
        self.assertEqual(checker.failures, 0)

    def test_trajectory_excursion_not_hidden_by_return_to_start(self):
        verdict = Verdict()
        with contextlib.redirect_stdout(io.StringIO()) as out:
            analyze_trajectory([(0, 0, 0), (1, 0, 5), (2, 0, 0)], verdict)
        self.assertGreater(verdict.warns, 0)
        self.assertIn("range 5.00", out.getvalue())
        self.assertNotIn("within expected range", out.getvalue())

    def test_truncated_pcd_header_and_payload(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder, "broken.pcd")
            path.write_text("FIELDS x y z\n")
            with self.assertRaisesRegex(ValueError, "Truncated"):
                _load_pcd_minimal(path)
            path.write_bytes(b"FIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nPOINTS 2\nDATA binary\n" + b"\0"*12)
            with self.assertRaisesRegex(ValueError, "payload"):
                _load_pcd_minimal(path)

    def test_pcd_count_fields_before_xyz(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder, "vector.pcd")
            path.write_bytes(b"FIELDS descriptor x y z\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 2 1 1 1\nPOINTS 1\nDATA binary\n" + struct.pack("<fffff", 8, 9, 1, 2, 3))
            np.testing.assert_array_equal(_load_pcd_minimal(path), [[1, 2, 3]])

    def test_drain_requires_all_messages_and_completed_scans(self):
        meta = {"lidar_count": 10, "imu_count": 200}
        status = dict(received_lidar=10, received_imu=200, pending_lidar=0,
                      synchronized_frames=10, registered_frames=7, startup_frames=3, skipped_frames=0,
                      last_input_lidar=10.0, last_registered_stamp=10.1, exit_code=0)
        self.assertTrue(validate_drain(status, meta))
        self.assertIsNone(validate_drain(dict(status, received_lidar=9), meta))
        self.assertIsNone(validate_drain(dict(status, pending_lidar=1), meta))
        self.assertIsNone(validate_drain(dict(status, processing_frame=True), meta))
        with self.assertRaises(ValueError): validate_drain(dict(status, received_imu=201), meta)
        with self.assertRaises(ValueError): validate_drain(dict(status, registered_frames=0), meta)

    def test_drain_rejects_missing_registration_and_truncated_tail(self):
        meta = {"lidar_count": 20, "imu_count": 321}
        status = dict(received_lidar=20, received_imu=321, pending_lidar=0,
                      synchronized_frames=20, registered_frames=6, startup_frames=3,
                      skipped_frames=11, last_input_lidar=1003.8,
                      last_registered_stamp=1001.78, exit_code=0)
        with self.assertRaisesRegex(ValueError, "skipped"):
            validate_drain(status, meta)
        with self.assertRaisesRegex(ValueError, "accounted"):
            validate_drain(dict(status, skipped_frames=0), meta)
        with self.assertRaisesRegex(ValueError, "final input"):
            validate_drain(dict(status, skipped_frames=0, registered_frames=17), meta)
        with self.assertRaisesRegex(ValueError, "rebuild"):
            validate_drain({k:v for k,v in status.items() if k != "startup_frames"}, meta)

    def test_runtime_imu_gap_limit_must_be_finite_positive(self):
        for value in (0, -1, True, float('nan'), float('inf'), '0.05'):
            checker = ConfigChecker('unused', 'jt16', 2)
            with contextlib.redirect_stdout(io.StringIO()):
                checker._check_gyr_unit({'common': {'imu_gyr_unit': 'rad', 'imu_max_gap': value}})
            self.assertEqual(checker.failures, 1, value)

    def test_header_audit_rejects_zero_repeat_and_rollback(self):
        for times in [(0, 0), (10, 10), (10, 9)]:
            audit = HeaderTimeAudit(['/p'])
            audit.observe('/p', NS(header=NS(stamp=NS(sec=times[0], nanosec=0))))
            with self.assertRaisesRegex(ValueError, 'repeated or moved backwards'):
                audit.observe('/p', NS(header=NS(stamp=NS(sec=times[1], nanosec=0))))
        audit = HeaderTimeAudit(['/p'])
        audit.observe('/p', NS(header=NS(stamp=NS(sec=0, nanosec=0))))
        with self.assertRaisesRegex(ValueError, 'all zero'):
            audit.finish({'/p': {'count': 1}})

    def test_header_audit_allows_zero_origin_and_checks_actual_counts(self):
        audit = HeaderTimeAudit(['/p'])
        for value in [0, 10000000, 20000000]:
            audit.observe('/p', NS(header=NS(stamp=NS(secs=0, nsecs=value))))
        result = audit.finish({'/p': {'count': 3}})
        self.assertEqual(result['/p']['max_gap_ns'], 10000000)
        with self.assertRaisesRegex(ValueError, 'decoded count'):
            audit.finish({'/p': {'count': 4}})

    def test_map_save_verifies_payload_and_count(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder, "map.pcd")
            header = b"FIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\nPOINTS 1\nDATA binary\n"
            report = {"files": [{"path": str(path), "points": 1}]}
            path.write_bytes(header + struct.pack("<ffff", 1, 2, 3, 4))
            self.assertEqual(len(verify_saved_maps(report)), 1)
            path.write_bytes(header + b"\0"*15)
            with self.assertRaises(ValueError): verify_saved_maps(report)

    def test_export_mode_validation(self):
        for ros in (1, 2):
            cfg = {} if ros == 1 else {"/**": {"ros__parameters": {}}}
            out, _ = prepare_config(cfg, ros, map_path="map.pcd", map_mode="all", leaf_size=0.1)
            params = out if ros == 1 else out["/**"]["ros__parameters"]
            self.assertEqual(params["pcd_save"]["mode"], "all")
            with self.assertRaises(ValueError):
                prepare_config(cfg, ros, map_path="map.pcd", map_mode="voxel", leaf_size=0)

    def test_launcher_restores_background_signals_and_owns_group(self):
        launcher=Path(__file__).with_name("launch_process.py")
        code="import os,signal; print(signal.getsignal(signal.SIGINT)==signal.default_int_handler, os.getpid()==os.getpgrp())"
        result=subprocess.run(["bash","-c",'trap "" INT; exec "$1" "$2" "$1" -c "$3"',
                               "test",sys.executable,str(launcher),code],capture_output=True,text=True)
        self.assertEqual(result.returncode,0,result.stderr)
        self.assertEqual(result.stdout.strip(),"True True")

    def test_converter_uses_private_config_with_literal_filenames(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder)
            source=root/"config.yaml"
            source.write_text(yaml.safe_dump({'lidar':[{'driver':{'source_type':1}}, {'driver':{'source_type':1}}]}))
            original=source.read_bytes()
            literal=str(root/"device'$(touch SHOULD_NOT_EXIST).csv")
            for ros in (1,2):
                script=(Path(__file__).parent/f"pcap_to_rosbag/pcap_to_rosbag_ros{ros}.sh").read_text()
                start=script.index('python3 - "$DRIVER_CONFIG"')
                end=script.index('\nPYEOF',start)+len('\nPYEOF')
                env={'PATH':os.environ.get('PATH','/usr/bin:/bin'),
                     'DRIVER_CONFIG':str(source),'TMP_CONFIG':str(root/f'private{ros}.yaml'),
                     'PCAP':literal+'.pcap','CORRECTION':literal,'FIRETIME':'',
                     'PLAY_RATE':'1','TIMESTAMP_OFFSET':'0','LIDAR_TOPIC':'/points','IMU_TOPIC':'/imu'}
                result=subprocess.run(['bash','-c',script[start:end]],cwd=folder,env=env,capture_output=True,text=True)
                self.assertEqual(result.returncode,0,result.stderr)
                cfg=yaml.safe_load(Path(env['TMP_CONFIG']).read_text())
                self.assertEqual(len(cfg['lidar']),1)
                self.assertEqual(cfg['lidar'][0]['driver']['pcap_type']['correction_file_path'],literal)
                self.assertTrue(cfg['lidar'][0]['ros']['send_imu_ros'])
                self.assertEqual(source.read_bytes(),original)
                self.assertFalse((root/'SHOULD_NOT_EXIST').exists())


if __name__ == "__main__":
    unittest.main()
